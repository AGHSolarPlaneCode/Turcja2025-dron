import cv2
import numpy as np

cap = cv2.VideoCapture("/home/pi5/Turcja2025-dron/AI_Prototyping/YOLO_TEST1.MP4")

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = int(cap.get(cv2.CAP_PROP_FPS))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

out = cv2.VideoWriter("./output.mp4", fourcc, fps, (width, height))

current_frame = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    current_frame += 1

    # --- POSTĘP ---
    progress = (current_frame / frame_count) * 100
    print(f"Przetworzono klatkę {current_frame}/{frame_count} ({progress:.2f}%)", end="\r")

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([90, 100, 100])
    upper_blue = np.array([135, 255, 255])

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([5, 255, 255])
    lower_red2 = np.array([136, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                              cv2.inRange(hsv, lower_red2, upper_red2))

    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours_blue:
        if cv2.contourArea(cnt) < 300:
            continue
        approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
        cv2.drawContours(frame, [approx], 0, (255, 0, 0), 3)
        if len(approx) == 3:
            cv2.putText(frame, 'Trojkat', tuple(approx.ravel()[:2]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 4)
        elif len(approx) == 4:
            cv2.putText(frame, 'Kwadrat', tuple(approx.ravel()[:2]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 4)
        elif len(approx) == 6:
            cv2.putText(frame, 'Szesciokat', tuple(approx.ravel()[:2]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 4)

    for cnt in contours_red:
        if cv2.contourArea(cnt) < 300:
            continue
        approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
        cv2.drawContours(frame, [approx], 0, (0, 0, 255), 3)
        if len(approx) == 3:
            cv2.putText(frame, 'Trojkat', tuple(approx.ravel()[:2]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4)
        elif len(approx) == 4:
            cv2.putText(frame, 'Kwadrat', tuple(approx.ravel()[:2]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4)
        elif len(approx) == 6:
            cv2.putText(frame, 'Szesciokat', tuple(approx.ravel()[:2]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4)

    out.write(frame)

cap.release()
out.release()
print("\n✅ Przetwarzanie zakończone! Zapisano jako output.mp4")
