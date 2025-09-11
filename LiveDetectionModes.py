import cv2
import numpy as np
from picamera2 import Picamera2
import datetime
import requests


class LiveDetection:
    def __init__(self):

        self.SERVER_URL = "http://100.127.85.126:8000/upload"   # <- podaj IP Tailscale serwera iza
        self.AUTH_TOKEN = "iza123"                    # <- Twój token ustawiony na serwerze

        
        # Initialize PiCamera2
        self.picam2 = Picamera2()
        self.size = (1280, 720)
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": self.size}
        ))
        self.picam2.start()

        self.device = "Plane"  # "Drone" or "Plane"

        # Hardcoded values for this resolution and 40m AGL
        # 1280:720              2304:1296
        self.r_tri_treshold = 260    #r_tri_treshold = 778
        self.r_squ_treshold = 2300    #r_squ_treshold = 7180
        self.b_squ_treshold = 9000    #b_squ_treshold = 28720
        self.b_hex_treshold = 6000    #b_hex_treshold = 18655

        self.deteced_square_blue = 0
        self.deteced_hexagon_blue = 0
        self.deteced_triangle_red = 0
        self.deteced_square_red = 0

        self.frames_in_row = 6

        print("Camera initialized.")

    def upload_file(self,file_path, file_type):
        """
        file_type = 'image' lub 'log'
        """
        with open(file_path, "rb") as f:
            files = {"file": (file_path, f)}
            data = {"type": file_type}
            headers = {"X-Auth": self.AUTH_TOKEN}
            resp = requests.post(self.SERVER_URL, files=files, data=data, headers=headers, timeout=30)
        if resp.status_code == 200:
            print(f"[OK] {file_type} wysłany: {resp.json()}")
        else:
            print(f"[ERR] {resp.status_code} {resp.text}")
    
    def getResolution(self):
        return self.size
    def detect_shape(self):
        while True:
            frame = self.picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)  # bo XRGB8888 ma 4 kanały
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Blue mask
            lower_blue = np.array([90, 125, 70])
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

        

            # Process blue contours
            for cnt in contours_blue:
                if cv2.contourArea(cnt) < 100:
                    continue
                approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)

                if len(approx) == 4 and self.device == "Plane":
                    #if not (self.b_squ_treshold * 0.7 <= cv2.contourArea(cnt) <= self.b_squ_treshold * 1.3):
                    #    continue
                    srodek_kw_x = int((approx.ravel()[2] + approx.ravel()[6]) / 2)
                    srodek_kw_y = int((approx.ravel()[3] + approx.ravel()[7]) / 2)
                    cv2.drawContours(frame, [approx], -1, (255, 0, 0), 3)
                    cv2.circle(frame, (srodek_kw_x,srodek_kw_y), 3, (0, 255, 0), 3)
                    print(f"[BLUE] Kwadrat -> środek: ({srodek_kw_x}, {srodek_kw_y}) count: {self.deteced_square_blue}")
                    self.deteced_square_blue += 1
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)  # bo XRGB8888 ma 4 kanały
                    frame = cv2.resize(frame,(640,360))
                    filename = f"./detected_shapes/square/blue/blue_square_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                    cv2.imwrite(f"{filename}", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    self.upload_file(f"{filename}", "image")
                    #print(f"Saving blue square image.")
                    return len(approx), [srodek_kw_x, srodek_kw_y], False

                elif len(approx) == 6 and self.device == "Drone":
                    self.deteced_square_blue = 0  # reset counter for square when hexagon is detected
                    #if not (self.b_hex_treshold * 0.7 <= cv2.contourArea(cnt) <= self.b_hex_treshold * 1.3):
                    #    continue
                    srodek_sz_x = sum(approx.ravel()[i] for i in range(0, 12, 2)) // 6
                    srodek_sz_y = sum(approx.ravel()[i] for i in range(1, 12, 2)) // 6
                    cv2.drawContours(frame, [approx], -1, (255, 0, 0), 3)
                    cv2.circle(frame, (srodek_sz_x,srodek_sz_y), 3, (0, 255, 0), 3)
                    print(f"[BLUE] Szesciokat -> środek: ({srodek_sz_x}, {srodek_sz_y}) count: {self.deteced_hexagon_blue}")
                    self.deteced_hexagon_blue += 1
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)  # bo XRGB8888 ma 4 kanały
                    frame = cv2.resize(frame,(640,360))
                    filename = f"./detected_shapes/hexagon/blue_hexagon_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                    cv2.imwrite(f"{filename}", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    self.upload_file(f"{filename}", "image")
                    #cv2.imwrite(f"./detected_shapes/hexagon/blue_hexagon_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png", frame)
                    #print(f"Saving blue hexagon image.")
                    return len(approx), [srodek_sz_x, srodek_sz_y], False

            # Process red contours
            for cnt in contours_red:
                if cv2.contourArea(cnt) < 400:
                    continue
                approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)

                if len(approx) == 3 and self.device == "Drone":
                    self.deteced_square_red = 0  # reset counter for square when triangle is detected
                    #if not (self.r_tri_treshold * 0.7 <= cv2.contourArea(cnt) <= self.r_tri_treshold * 1.3):
                    #    continue
                    srodek_tr_x = sum(approx.ravel()[i] for i in range(0, 6, 2)) // 3
                    srodek_tr_y = sum(approx.ravel()[i] for i in range(1, 6, 2)) // 3
                    cv2.drawContours(frame, [approx], -1, (0, 0, 255), 3)
                    cv2.circle(frame, (srodek_tr_x, srodek_tr_y), 3, (0, 255, 0), 3)
                    print(f"[RED] Trojkat -> środek: ({srodek_tr_x}, {srodek_tr_y}) count: {self.deteced_triangle_red}")
                    self.deteced_triangle_red += 1                    
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)  # bo XRGB8888 ma 4 kanały
                    frame = cv2.resize(frame,(640,360))
                    filename = f"./detected_shapes/triangle/red_triangle_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                    cv2.imwrite(f"{filename}", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    self.upload_file(f"{filename}", "image")
                    #cv2.imwrite(f"./detected_shapes/triangle/red_triangle_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png", frame)
                    #print(f"Saving red triangle image.")
                    return len(approx), [srodek_tr_x, srodek_tr_y], True

                if len(approx) == 4 and self.device == "Plane":
                    self.deteced_triangle_red = 0
                    #if not (self.r_squ_treshold * 0.7 <= cv2.contourArea(cnt) <= self.r_squ_treshold * 1.3):
                    #    continue
                    srodek_kw_x = int((approx.ravel()[2] + approx.ravel()[6]) / 2)
                    srodek_kw_y = int((approx.ravel()[3] + approx.ravel()[7]) / 2)
                    cv2.drawContours(frame, [approx], -1, (0, 0, 255), 3)
                    cv2.circle(frame, (srodek_kw_x,srodek_kw_y), 3, (0, 255, 0), 3)
                    print(f"[RED] Kwadrat -> środek: ({srodek_kw_x}, {srodek_kw_y}) count: {self.deteced_square_red}")
                    self.deteced_square_red += 1
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)  # bo XRGB8888 ma 4 kanały
                    frame = cv2.resize(frame,(640,360))
                    filename = f"./detected_shapes/square/red/red_square_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                    cv2.imwrite(f"{filename}", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    self.upload_file(f"{filename}", "image")
                    #cv2.imwrite(f"./detected_shapes/square/red/red_square_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png", frame)
                    #print(f"Saving red square image.")
                    return len(approx), [srodek_kw_x, srodek_kw_y], True
            return 0, [0,0], False  # jeśli nic nie wykryto

if __name__ == "__main__":
    ld = LiveDetection()
    while True:
        f,c,is_red = ld.detect_shape()
        if f != 0:
            print(f"Detected shape with {f} sides at {c}")

                                 