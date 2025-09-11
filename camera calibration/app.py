# app.py
from flask import Flask, render_template, Response, request, send_from_directory
from picamera2 import Picamera2
import cv2
import threading
import time
import numpy as np
import os
import datetime

# Optional: YOLO is loaded only if YOLO_MODEL_PATH env var is set and ultralytics is available.
class YoloDetection:
    def __init__(self, model_path=None):
        self.model = None
        if model_path:
            try:
                from ultralytics import YOLO
                self.model = YOLO(model_path, task="detect")
                print(f"Loaded YOLO model from: {model_path}")
            except Exception as e:
                print(f"Warning: couldn't load YOLO model ({e}). Processed mode will be disabled.")

    def process_frame(self, frame):
        if self.model is None:
            return frame
        results = self.model(frame)
        annotated = frame.copy()
        for r in results:
            if r.boxes is None or len(r.boxes) == 0:
                continue
            xyxys = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()
            clses = r.boxes.cls.cpu().numpy()
            for (x1, y1, x2, y2), conf, cls_id in zip(xyxys, confs, clses):
                x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
                label = f"{self.model.names[int(cls_id)]}: {conf:.2f}"
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(annotated, label, (x1, max(0, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        return annotated


# Camera streaming + optional stabilization + optional YOLO processing
class CameraStream:
    def __init__(self, width=640, height=640, fps_rate=0.033, yolo_model_path=None):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (width, height)})
        self.picam2.configure(config)
        self.picam2.start()

        self.yolo_model = YoloDetection(yolo_model_path)

        self.processing_mode = 'original'  # 'original' or 'processed'
        self.stabilization_enabled = False
        self.frame = None
        self.lock = threading.Lock()

        self.prev_gray = None
        self.transforms = []
        self.smoothed_transforms = []
        self.trajectory = []
        self.smoothing_radius = 10

        self.fps_rate = fps_rate
        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.capture_thread.daemon = True
        self.capture_thread.start()

    def stabilize_frame(self, frame):
        if not self.stabilization_enabled:
            return frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        if self.prev_gray is None:
            self.prev_gray = gray
            return frame
        prev_pts = cv2.goodFeaturesToTrack(self.prev_gray, maxCorners=200, qualityLevel=0.01,
                                           minDistance=30, blockSize=3)
        if prev_pts is None or len(prev_pts) < 10:
            self.prev_gray = gray
            return frame
        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, prev_pts, None)
        valid_prev = prev_pts[status == 1]
        valid_curr = curr_pts[status == 1]
        if len(valid_prev) < 10:
            self.prev_gray = gray
            return frame
        transform_matrix = cv2.estimateAffinePartial2D(valid_prev, valid_curr)[0]
        if transform_matrix is None:
            self.prev_gray = gray
            return frame
        dx = transform_matrix[0,2]
        dy = transform_matrix[1,2]
        da = np.arctan2(transform_matrix[1,0], transform_matrix[0,0])
        self.transforms.append([dx, dy, da])
        if len(self.trajectory) == 0:
            self.trajectory.append([dx, dy, da])
        else:
            prev_traj = self.trajectory[-1]
            self.trajectory.append([prev_traj[0] + dx, prev_traj[1] + dy, prev_traj[2] + da])
        if len(self.trajectory) >= self.smoothing_radius:
            start_idx = max(0, len(self.trajectory) - self.smoothing_radius)
            smoothed = np.mean(self.trajectory[start_idx:], axis=0)
            self.smoothed_transforms.append(smoothed)
        else:
            self.smoothed_transforms.append(self.trajectory[-1])
        if len(self.smoothed_transforms) > 0:
            smooth_dx = self.smoothed_transforms[-1][0] - self.trajectory[-1][0]
            smooth_dy = self.smoothed_transforms[-1][1] - self.trajectory[-1][1]
            smooth_da = self.smoothed_transforms[-1][2] - self.trajectory[-1][2]
            stabilize_matrix = np.array([
                [np.cos(-smooth_da), -np.sin(-smooth_da), smooth_dx],
                [np.sin(-smooth_da),  np.cos(-smooth_da), smooth_dy]
            ], dtype=np.float32)
            stabilized = cv2.warpAffine(frame, stabilize_matrix, (w, h))
            border = int(min(h, w) * 0.05)
            stabilized = stabilized[border:h - border, border:w - border]
            stabilized = cv2.resize(stabilized, (w, h))
            self.prev_gray = gray
            return stabilized
        self.prev_gray = gray
        return frame

    def capture_frames(self):
        while True:
            try:
                frame = self.picam2.capture_array()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                if self.stabilization_enabled:
                    frame_bgr = self.stabilize_frame(frame_bgr)
                if self.processing_mode == 'processed':
                    processed = self.yolo_model.process_frame(frame_bgr)
                else:
                    processed = frame_bgr
                with self.lock:
                    self.frame = processed
                time.sleep(self.fps_rate)
            except Exception as e:
                print(f"Capture error: {e}")
                time.sleep(0.5)

    def get_frame(self):
        with self.lock:
            if self.frame is not None:
                ret, buffer = cv2.imencode('.jpg', self.frame)
                if ret:
                    return buffer.tobytes()
        return None

    def set_mode(self, mode):
        self.processing_mode = mode

    def set_stabilization(self, enabled):
        self.stabilization_enabled = enabled
        if not enabled:
            self.prev_gray = None
            self.transforms = []
            self.smoothed_transforms = []
            self.trajectory = []

    def stop(self):
        try:
            self.picam2.stop()
        except Exception:
            pass


# -------------------- Flask app --------------------
app = Flask(__name__)

FPS_RATE = 0.033
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CAPTURE_DIR = os.path.join(BASE_DIR, "captures")
os.makedirs(CAPTURE_DIR, exist_ok=True)

# Allow enabling YOLO by setting YOLO_MODEL_PATH env var before running
yolo_model_path = os.environ.get("YOLO_MODEL_PATH", None)
camera = CameraStream(width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps_rate=FPS_RATE, yolo_model_path=yolo_model_path)


def generate_frames():
    while True:
        frame_bytes = camera.get_frame()
        if frame_bytes is None:
            continue
        # Zwracamy poprawny multipart boundary
        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n'
        )

@app.route('/video_feed')
def video_feed():
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )



@app.route('/')
def index():
    return render_template('index.html', width=CAMERA_WIDTH, height=CAMERA_HEIGHT,
                           stabilization='ON' if camera.stabilization_enabled else 'OFF',
                           mode=camera.processing_mode)



@app.route('/set_mode/<mode>')
def set_mode(mode):
    camera.set_mode(mode)
    return f"Mode set to: {mode}"


@app.route('/set_stabilization/<enabled>')
def set_stabilization(enabled):
    is_enabled = enabled.lower() == 'true'
    camera.set_stabilization(is_enabled)
    return f"Stabilization {'enabled' if is_enabled else 'disabled'}"


@app.route('/capture', methods=['POST'])
def capture():
    # Save current frame to CAPTURE_DIR with ISO timestamp
    with camera.lock:
        if camera.frame is None:
            return "No frame available", 500
        frame_to_save = camera.frame.copy()
    filename = datetime.datetime.now().strftime("%Y%m%d_%H%M%S.jpg")
    filepath = os.path.join(CAPTURE_DIR, filename)
    # frame is BGR, cv2.imwrite expects BGR
    success = cv2.imwrite(filepath, frame_to_save)
    if not success:
        return "Failed to save image", 500
    return f"Saved {filename}"


@app.route('/captures')
def captures_list():
    files = sorted([f for f in os.listdir(CAPTURE_DIR) if f.lower().endswith(('.jpg','.jpeg','.png'))],
                   key=lambda x: os.path.getmtime(os.path.join(CAPTURE_DIR, x)), reverse=True)
    return render_template('captures.html', files=files)


@app.route('/capture_file/<path:filename>')
def capture_file(filename):
    return send_from_directory(CAPTURE_DIR, filename)


if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        camera.stop()
