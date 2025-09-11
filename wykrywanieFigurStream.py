from flask import Flask, Response
import cv2
import numpy as np
from picamera2 import Picamera2

app = Flask(__name__)

# Initialize PiCamera2
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"format": 'XRGB8888', "size": (1280, 720)}
))
picam2.start()

device = "Plane"

#Hardcoded values for this resolution and 40m AGL
r_tri_treshold = 778
r_squ_treshold = 7180
b_squ_treshold = 28720
b_hex_treshold = 18655

def gen_frames():
    while True:
        frame = picam2.capture_array()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Blue mask
        lower_blue = np.array([90,190,70])
        upper_blue = np.array([120,255,255])
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

        # Process blue contours
        # Process blue contours
        for cnt in contours_blue:
            if cv2.contourArea(cnt) < 600:
                continue
            approx = cv2.approxPolyDP(cnt,0.10*cv2.arcLength(cnt,True),True)

            if len(approx) == 4 and device=="Plane":
                if cv2.contourArea(cnt) < b_squ_treshold * 0.7 or cv2.contourArea(cnt) > b_squ_treshold * 1.3:
                    continue
                # Draw contour
                cv2.drawContours(frame, [approx], -1, (255, 0, 0), 3)
                x, y = approx.ravel()[0], approx.ravel()[1]
                frame = cv2.putText(frame, 'Kwadrat ', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 4)
                srodek_kw_x = int((approx.ravel()[2]+approx.ravel()[6])/2)
                srodek_kw_y = int((approx.ravel()[3]+approx.ravel()[7])/2)
                cv2.circle(frame, (srodek_kw_x,srodek_kw_y), 3, (0, 255, 0), 3)

            elif len(approx) == 6 and device=="Drone":
                if cv2.contourArea(cnt) < b_hex_treshold * 0.7 or cv2.contourArea(cnt) > b_hex_treshold * 1.3:
                    continue
                # Draw contour
                cv2.drawContours(frame, [approx], -1, (255, 0, 0), 3)
                x, y = approx.ravel()[0], approx.ravel()[1]
                frame = cv2.putText(frame, 'Szesciokat ', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 4)
                srodek_sz_x = sum(approx.ravel()[i] for i in range(0, 12, 2)) // 6
                srodek_sz_y = sum(approx.ravel()[i] for i in range(1, 12, 2)) // 6
                cv2.circle(frame,(srodek_sz_x,srodek_sz_y), 3, (0, 255, 0), 3)

        # Process red contours
        for cnt in contours_red:
            if cv2.contourArea(cnt) < 600:
                continue
            approx = cv2.approxPolyDP(cnt,0.10 * cv2.arcLength(cnt,True),True)

            if len(approx) == 3 and device=="Drone":
                if cv2.contourArea(cnt) < r_tri_treshold * 0.7 or cv2.contourArea(cnt) > r_tri_treshold * 1.3:
                    continue
                # Draw contour
                cv2.drawContours(frame, [approx], -1, (0, 0, 255), 3)
                x, y = approx.ravel()[0], approx.ravel()[1]
                frame = cv2.putText(frame, 'Trojkat ', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4)
                srodek_tr_x = sum(approx.ravel()[i] for i in range(0,6,2)) // 3
                srodek_tr_y = sum(approx.ravel()[i] for i in range(1,6,2)) // 3
                cv2.circle(frame, (srodek_tr_x, srodek_tr_y), 3, (0, 255, 0), 3)

            if len(approx) == 4 and device=="Plane":
                if cv2.contourArea(cnt) < r_squ_treshold * 0.7 or cv2.contourArea(cnt) > r_squ_treshold * 1.3:
                    continue
                # Draw contour
                cv2.drawContours(frame, [approx], -1, (0, 0, 255), 3)
                x, y = approx.ravel()[0], approx.ravel()[1]
                frame = cv2.putText(frame, 'Kwadrat ', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 4)
                srodek_kw_x = int((approx.ravel()[2]+approx.ravel()[6])/2)
                srodek_kw_y = int((approx.ravel()[3]+approx.ravel()[7])/2)
                cv2.circle(frame, (srodek_kw_x,srodek_kw_y), 3, (0, 255, 0), 3)



        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        # Yield frame in HTTP multipart format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def video():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
