import cv2
import numpy as np
from picamera2 import Picamera2
import datetime
import threading
import time
import base64
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
import io

class LiveDetection:
    def __init__(self):
        # Initialize PiCamera2
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (240, 135)}
        ))
        self.picam2.start()
        self.device = "Drone"  # "Drone" or "Plane"
        
        # Hardcoded values for this resolution and 40m AGL
        self.r_tri_treshold = 260    
        self.r_squ_treshold = 2300    
        self.b_squ_treshold = 9000    
        self.b_hex_treshold = 6000    
        self.deteced_square_blue = 0
        self.deteced_hexagon_blue = 0
        self.deteced_triangle_red = 0
        self.deteced_square_red = 0
        self.frames_in_row = 6
        
        # Streaming variables
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.streaming = True
        
        print("Camera initialized.")

    def detect_shape_streaming(self):
        """Modified detect_shape method for continuous streaming"""
        while self.streaming:
            frame = self.picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Blue mask
            lower_blue = np.array([90, 190, 70])
            upper_blue = np.array([120, 255, 255])
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
            
            # Red mask
            lower_red1 = np.array([0, 190, 70])
            upper_red1 = np.array([5, 255, 255])
            lower_red2 = np.array([170, 190, 70])
            upper_red2 = np.array([180, 255, 255])
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            
            contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            detection_made = False
            
            # Process blue contours
            for cnt in contours_blue:
                if cv2.contourArea(cnt) < 100:
                    continue
                approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
                
                if len(approx) == 4:
                    self.deteced_hexagon_blue = 0
                    if not (self.b_squ_treshold * 0.7 <= cv2.contourArea(cnt) <= self.b_squ_treshold * 1.3):
                        continue
                    srodek_kw_x = int((approx.ravel()[2] + approx.ravel()[6]) / 2)
                    srodek_kw_y = int((approx.ravel()[3] + approx.ravel()[7]) / 2)
                    cv2.drawContours(frame, [approx], -1, (255, 0, 0), 3)
                    cv2.circle(frame, (srodek_kw_x, srodek_kw_y), 3, (0, 255, 0), 3)
                    cv2.putText(frame, f"Blue Square ({self.deteced_square_blue})", 
                               (srodek_kw_x-50, srodek_kw_y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    print(f"[BLUE] Kwadrat -> środek: ({srodek_kw_x}, {srodek_kw_y}) count: {self.deteced_square_blue}")
                    self.deteced_square_blue += 1
                    
                    if self.deteced_square_blue == self.frames_in_row:
                        cv2.imwrite(f"./detected_shapes/square/blue/blue_square_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png", frame)
                        print(f"Saving blue square image.")
                        detection_made = True
                        self.deteced_square_blue = 0  # Reset after detection
                        
                elif len(approx) == 6:
                    self.deteced_square_blue = 0
                    if not (self.b_hex_treshold * 0.7 <= cv2.contourArea(cnt) <= self.b_hex_treshold * 1.3):
                        continue
                    srodek_sz_x = sum(approx.ravel()[i] for i in range(0, 12, 2)) // 6
                    srodek_sz_y = sum(approx.ravel()[i] for i in range(1, 12, 2)) // 6
                    cv2.drawContours(frame, [approx], -1, (255, 0, 0), 3)
                    cv2.circle(frame, (srodek_sz_x, srodek_sz_y), 3, (0, 255, 0), 3)
                    cv2.putText(frame, f"Blue Hexagon ({self.deteced_hexagon_blue})", 
                               (srodek_sz_x-50, srodek_sz_y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    print(f"[BLUE] Szesciokat -> środek: ({srodek_sz_x}, {srodek_sz_y}) count: {self.deteced_hexagon_blue}")
                    self.deteced_hexagon_blue += 1
                    
                    if self.deteced_hexagon_blue == self.frames_in_row:
                        cv2.imwrite(f"./detected_shapes/hexagon/blue_hexagon_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png", frame)
                        print(f"Saving blue hexagon image.")
                        detection_made = True
                        self.deteced_hexagon_blue = 0  # Reset after detection

            # Process red contours
            for cnt in contours_red:
                if cv2.contourArea(cnt) < 100:
                    continue
                approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
                
                if len(approx) == 3:
                    self.deteced_square_red = 0
                    if not (self.r_tri_treshold * 0.7 <= cv2.contourArea(cnt) <= self.r_tri_treshold * 1.3):
                        continue
                    srodek_tr_x = sum(approx.ravel()[i] for i in range(0, 6, 2)) // 3
                    srodek_tr_y = sum(approx.ravel()[i] for i in range(1, 6, 2)) // 3
                    cv2.drawContours(frame, [approx], -1, (0, 0, 255), 3)
                    cv2.circle(frame, (srodek_tr_x, srodek_tr_y), 3, (0, 255, 0), 3)
                    cv2.putText(frame, f"Red Triangle ({self.deteced_triangle_red})", 
                               (srodek_tr_x-50, srodek_tr_y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    print(f"[RED] Trojkat -> środek: ({srodek_tr_x}, {srodek_tr_y}) count: {self.deteced_triangle_red}")
                    self.deteced_triangle_red += 1
                    
                    if self.deteced_triangle_red == self.frames_in_row:
                        cv2.imwrite(f"./detected_shapes/triangle/red_triangle_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png", frame)
                        print(f"Saving red triangle image.")
                        detection_made = True
                        self.deteced_triangle_red = 0  # Reset after detection
                        
                elif len(approx) == 4:
                    self.deteced_triangle_red = 0
                    if not (self.r_squ_treshold * 0.7 <= cv2.contourArea(cnt) <= self.r_squ_treshold * 1.3):
                        continue
                    srodek_kw_x = int((approx.ravel()[2] + approx.ravel()[6]) / 2)
                    srodek_kw_y = int((approx.ravel()[3] + approx.ravel()[7]) / 2)
                    cv2.drawContours(frame, [approx], -1, (0, 0, 255), 3)
                    cv2.circle(frame, (srodek_kw_x, srodek_kw_y), 3, (0, 255, 0), 3)
                    cv2.putText(frame, f"Red Square ({self.deteced_square_red})", 
                               (srodek_kw_x-50, srodek_kw_y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    print(f"[RED] Kwadrat -> środek: ({srodek_kw_x}, {srodek_kw_y}) count: {self.deteced_square_red}")
                    self.deteced_square_red += 1
                    
                    if self.deteced_square_red == self.frames_in_row:
                        cv2.imwrite(f"./detected_shapes/square/red/red_square_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png", frame)
                        print(f"Saving red square image.")
                        detection_made = True
                        self.deteced_square_red = 0  # Reset after detection
            
            # Add timestamp to frame
            timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            cv2.putText(frame, timestamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Update current frame for streaming
            with self.frame_lock:
                self.current_frame = frame.copy()
            
            time.sleep(0.1)  # ~30 FPS

    def get_frame(self):
        """Get current frame for streaming"""
        with self.frame_lock:
            if self.current_frame is not None:
                _, buffer = cv2.imencode('.jpg', self.current_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                return buffer.tobytes()
        return None

    def stop_streaming(self):
        """Stop the streaming"""
        self.streaming = False
        self.picam2.stop()

# Flask application
app = Flask(__name__)
app.config['SECRET_KEY'] = 'your_secret_key_here'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global detection object
detection = None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        global detection
        while True:
            if detection:
                frame = detection.get_frame()
                if frame:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.03)
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    emit('status', {'msg': 'Connected to detection stream'})

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

if __name__ == "__main__":
    # Create detection instance
    detection = LiveDetection()
    
    # Start detection in separate thread
    detection_thread = threading.Thread(target=detection.detect_shape_streaming)
    detection_thread.daemon = True
    detection_thread.start()
    
    # Start Flask server
    print("Starting web server on http://0.0.0.0:5000")
    print("Access from other networks using your Raspberry Pi's IP address")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)