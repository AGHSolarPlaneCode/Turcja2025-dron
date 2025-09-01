from flask import Flask, render_template, Response, request
from picamera2 import Picamera2
import cv2
import threading
import time
import numpy as np
from simple_shapes_detection import shapes_detection


app = Flask(__name__)

FPS_RATE = 0.033
CAMERA_WIDTH = 480
CAMERA_HEIGHT = 320


class CameraStream:
    def __init__(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT)})
        self.picam2.configure(config)
        self.picam2.start()

        self.processing_mode = 'original'
        self.stabilization_enabled = False
        self.frame = None
        self.lock = threading.Lock()

        self.prev_gray = None
        self.transforms = []
        self.smoothed_transforms = []
        self.trajectory = []
        self.smoothing_radius = 10

        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.capture_thread.daemon = True
        self.capture_thread.start()

    def stabilize_frame(self, frame):
        if not self.stabilization_enabled:
            return frame

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        if self.prev_gray is None:
            self.prev_gray = gray
            return frame

        prev_pts = cv2.goodFeaturesToTrack(self.prev_gray,
                                           maxCorners=200,
                                           qualityLevel=0.01,
                                           minDistance=30,
                                           blockSize=3)

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

        dx = transform_matrix[0, 2]
        dy = transform_matrix[1, 2]
        da = np.arctan2(transform_matrix[1, 0], transform_matrix[0, 0])

        self.transforms.append([dx, dy, da])

        if len(self.trajectory) == 0:
            self.trajectory.append([dx, dy, da])
        else:
            prev_traj = self.trajectory[-1]
            self.trajectory.append([
                prev_traj[0] + dx,
                prev_traj[1] + dy,
                prev_traj[2] + da
            ])

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
                [np.sin(-smooth_da), np.cos(-smooth_da), smooth_dy]
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
                    processed = shapes_detection(frame_bgr)
                else:
                    processed = frame_bgr

                with self.lock:
                    self.frame = processed

                time.sleep(FPS_RATE)
            except Exception as e:
                print(f"Capture error: {e}")

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
        self.picam2.stop()


# Global camera instance
camera = CameraStream()


def generate_frames():
    while True:
        frame_bytes = camera.get_frame()
        if frame_bytes:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(FPS_RATE)


@app.route('/')
def index():
    return f'''
<!DOCTYPE html>
<html>
<head>
    <title>Raspberry Pi Camera Stream</title>
    <style>
        body {{ 
            font-family: Arial, sans-serif; 
            text-align: center; 
            margin: 50px; 
            background-color: #f0f0f0;
        }}
        .container {{
            background: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
            max-width: 800px;
            margin: 0 auto;
        }}
        .controls {{ 
            margin: 20px; 
        }}
        button {{ 
            margin: 5px; 
            padding: 10px 20px; 
            font-size: 16px; 
            border: none;
            border-radius: 5px;
            cursor: pointer;
            background-color: #007bff;
            color: white;
            transition: background-color 0.3s;
        }}
        button:hover {{
            background-color: #0056b3;
        }}
        button.active {{
            background-color: #28a745;
        }}
        img {{ 
            border: 2px solid #333; 
            border-radius: 10px;
        }}
        .stabilization-control {{
            margin: 20px 0;
            padding: 15px;
            background-color: #f8f9fa;
            border-radius: 5px;
            border: 1px solid #dee2e6;
        }}
        .checkbox-container {{
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 10px;
        }}
        input[type="checkbox"] {{
            width: 20px;
            height: 20px;
            cursor: pointer;
        }}
        label {{
            font-size: 18px;
            font-weight: bold;
            cursor: pointer;
        }}
        .status {{
            margin-top: 10px;
            font-size: 14px;
            color: #6c757d;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>Raspberry PI live video</h1>
        <img src="/video_feed" width="{CAMERA_WIDTH}" height="{CAMERA_HEIGHT}">

        <div class="stabilization-control">
            <div class="checkbox-container">
                <input type="checkbox" id="stabilization" onchange="toggleStabilization()">
                <label for="stabilization">Enable Video Stabilization</label>
            </div>
            <div class="status" id="stabilization-status">Stabilization: OFF</div>
        </div>

        <div class="controls">
            <h3>Modes:</h3>
            <button id="btn-original" onclick="setMode('original')" class="active">Original</button>
            <button id="btn-processed" onclick="setMode('processed')">Processed</button>
        </div>
    </div>

    <script>
        let currentMode = 'original';
        let stabilizationEnabled = false;

        function setMode(mode) {{
            currentMode = mode;
            fetch('/set_mode/' + mode);

            document.querySelectorAll('button[id^="btn-"]').forEach(btn => {{
                btn.classList.remove('active');
            }});
            document.getElementById('btn-' + mode).classList.add('active');
        }}

        function toggleStabilization() {{
            const checkbox = document.getElementById('stabilization');
            const status = document.getElementById('stabilization-status');

            stabilizationEnabled = checkbox.checked;

            fetch('/set_stabilization/' + (stabilizationEnabled ? 'true' : 'false'));

            if (stabilizationEnabled) {{
                status.textContent = 'Stabilization: ON';
                status.style.color = '#28a745';
            }} else {{
                status.textContent = 'Stabilization: OFF';
                status.style.color = '#dc3545';
            }}
        }}
    </script>
</body>
</html>
    '''


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/set_mode/<mode>')
def set_mode(mode):
    camera.set_mode(mode)
    return f"Mode set to: {mode}"


@app.route('/set_stabilization/<enabled>')
def set_stabilization(enabled):
    is_enabled = enabled.lower() == 'true'
    camera.set_stabilization(is_enabled)
    return f"Stabilization {'enabled' if is_enabled else 'disabled'}"



try:
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
except KeyboardInterrupt:
    print("Shutting down...")
finally:
    camera.stop()