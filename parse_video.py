import cv2
import os

# Path to your video
video_path = "/home/pi5/test.mp4"
output_folder = "./chessboard_images"

# Create output folder if it doesn't exist
os.makedirs(output_folder, exist_ok=True)

# Open the video
cap = cv2.VideoCapture(video_path)

frame_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break  # Exit loop if no frames left

    # Save frame as JPEG
    frame_filename = os.path.join(output_folder, f"frame_{frame_count:05d}.jpg")
    cv2.imwrite(frame_filename, frame)
    
    frame_count += 1

cap.release()
print(f"Saved {frame_count} frames to '{output_folder}'")
