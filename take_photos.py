from picamera2 import Picamera2
import cv2
import os
import time

# Directory to save images
save_dir = "chessboard_images"
os.makedirs(save_dir, exist_ok=True)

# Chessboard size (internal corners)
chessboard_size = (8, 5)

# Initialize camera (no preview needed)
picam2 = Picamera2()
time.sleep(2)  # warm-up

img_count = 0
max_images = 20  # total images to save
print("START")

while img_count < max_images:
    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect chessboard corners
    ret_corners, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    
    if ret_corners:
        img_path = os.path.join(save_dir, f"chessboard_{img_count}.png")
        cv2.imwrite(img_path, frame)
        print(f"Saved {img_path}")
        img_count += 1
    
    time.sleep(0.5)  # avoid flooding

print("Finished capturing chessboard images.")
picam2.close()
