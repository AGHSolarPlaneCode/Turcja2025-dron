import cv2
import numpy as np
from ultralytics import YOLO
import argparse
import os

def load_yolo_model( model_path="YoloV8-finetune/DroneModel/yolov8n_V2_320/weights/best_ncnn_model"):
    """
    Load YOLO model. Downloads YOLOv8 nano model if not found locally.
    You can use: yolov8n.pt, yolov8s.pt, yolov8m.pt, yolov8l.pt, yolov8x.pt
    """
    try:
        model = YOLO(model_path)
        print(f"Loaded YOLO model: {model_path}")
        return model
    except Exception as e:
        print(f"Error loading model: {e}")
        return None

def process_video_with_yolo(video_path, output_path, model, confidence_threshold=0.5):
    """
    Process video with YOLO object detection and save annotated result
    """
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print(f"Error: Could not open video {video_path}")
        return False

    fps = int(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"Video properties: {width}x{height}, {fps} FPS, {total_frames} frames")

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1
        print(f"Processing frame {frame_count}/{total_frames}", end='\r')

        results = model(frame, conf=confidence_threshold)

        annotated_frame = results[0].plot()

        out.write(annotated_frame)

    cap.release()
    out.release()
    print(f"\nProcessing complete! Output saved to: {output_path}")
    return True

def process_single_frame(image_path, output_path, model, confidence_threshold=0.5):
    """
    Process a single image with YOLO object detection
    """

    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not load image {image_path}")
        return False

    results = model(image, conf=confidence_threshold)

    annotated_image = results[0].plot()

    cv2.imwrite(output_path, annotated_image)
    print(f"Detection complete! Output saved to: {output_path}")

    for r in results:
        boxes = r.boxes
        if boxes is not None:
            for box in boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = model.names[cls]
                print(f"Detected: {class_name} (confidence: {conf:.2f})")

    return True

def real_time_detection(model, confidence_threshold=0.5):
    """
    Real-time object detection using webcam
    """
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam")
        return

    print("Starting real-time detection. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame, conf=confidence_threshold)

        annotated_frame = results[0].plot()

        cv2.imshow('YOLO Detection', annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description='YOLO Object Detection')
    parser.add_argument('--input', type=str, help='Input video/image path')
    parser.add_argument('--output', type=str, help='Output path')
    parser.add_argument('--model', type=str, default='yolov8n.pt',
                       help='YOLO model path (default: yolov8n.pt)')
    parser.add_argument('--confidence', type=float, default=0.5,
                       help='Confidence threshold (default: 0.5)')
    parser.add_argument('--realtime', action='store_true',
                       help='Use webcam for real-time detection')

    args = parser.parse_args()

    model = load_yolo_model(args.model)
    if model is None:
        return

    if args.realtime:
        real_time_detection(model, args.confidence)
    elif args.input:
        if not os.path.exists(args.input):
            print(f"Error: Input file {args.input} not found")
            return

        video_extensions = ['.mp4', '.avi', '.mov', '.mkv', '.wmv', '.flv']
        image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff']

        file_ext = os.path.splitext(args.input)[1].lower()

        if not args.output:
            base_name = os.path.splitext(args.input)[0]
            args.output = f"{base_name}_detected{file_ext}"

        if file_ext in video_extensions:
            process_video_with_yolo(args.input, args.output, model, args.confidence)
        elif file_ext in image_extensions:
            process_single_frame(args.input, args.output, model, args.confidence)
        else:
            print(f"Error: Unsupported file format {file_ext}")
    else:
        print("Please provide --input path or use --realtime for webcam detection")
        print("\nExample usage:")
        print("  python yolo_detector.py --input video.mp4 --output detected_video.mp4")
        print("  python yolo_detector.py --input image.jpg")
        print("  python yolo_detector.py --realtime")

if __name__ == "__main__":
    main()