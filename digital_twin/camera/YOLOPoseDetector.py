from ultralytics import YOLO
import cv2
import torch
import os
import sys
import numpy as np

torch.cuda.empty_cache()

class YOLOObjectDetector:
    def __init__(self, model_path='yolov8n.pt', camera_id=0, conf_threshold=0.5, show_window=True, verbose=False):
        self.model_path = model_path
        self.camera_id = camera_id
        self.cap = None
        self.conf_threshold = conf_threshold
        self.show_window = show_window
        self.verbose = verbose
        self.model = None
        self.frame_width = None
        self.frame_height = None

    def initialize_camera(self):
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            raise ValueError(f"Unable to open camera with id {self.camera_id}")
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"[INFO] Camera initialized: {self.frame_width}x{self.frame_height}")

    def release_camera(self):
        if self.cap:
            self.cap.release()
            self.cap = None

    def load_model(self):
        if not self.verbose:
            sys.stdout = open(os.devnull, 'w')
        self.model = YOLO(self.model_path)
        if not self.verbose:
            sys.stdout = sys.__stdout__
        print(f"[INFO] YOLO model loaded: {self.model_path}")

    def get_detections(self, frame):
        results = self.model(frame)[0]
        detections = []

        if results.boxes is not None:
            for box in results.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if conf < self.conf_threshold:
                    continue

                xyxy = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
                x1, y1, x2, y2 = map(int, xyxy)
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                label = self.model.names[cls_id]

                detections.append({
                    'class_id': cls_id,
                    'label': label,
                    'confidence': conf,
                    'bbox': (x1, y1, x2, y2),
                    'center_px': (cx, cy),
                    'center_norm': (cx / self.frame_width, cy / self.frame_height)
                })

        return detections, results

    def camera_detection(self):
        self.initialize_camera()
        self.load_model()

        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    break

                detections, results = self.get_detections(frame)

                for det in detections:
                    x1, y1, x2, y2 = det['bbox']
                    cx, cy = det['center_px']
                    label = det['label']
                    conf = det['confidence']

                    # Draw bounding box and label
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                    cv2.putText(frame, f'{label} {conf:.2f}', (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                if self.show_window:
                    cv2.imshow('YOLO Object Detection', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                yield detections
        finally:
            self.release_camera()
            if self.show_window:
                cv2.destroyAllWindows()

    def predict_image(self, image_path):
        self.load_model()
        frame = cv2.imread(image_path)
        self.frame_height, self.frame_width = frame.shape[:2]

        detections, results = self.get_detections(frame)

        # Draw results
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            cx, cy = det['center_px']
            label = det['label']
            conf = det['confidence']

            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(frame, f'{label} {conf:.2f}', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        if self.show_window:
            cv2.imshow('YOLO Detection', frame)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return detections

    def close(self):
        print("[INFO] Closing camera...")
        self.release_camera()
        if self.show_window:
            cv2.destroyAllWindows()
        sys.exit(0)


if __name__ == '__main__':
    detector = YOLOObjectDetector(
        camera_id=0,
        model_path='yolo11n-seg.pt',
        conf_threshold=0.3,
        show_window=True
    )
    try:
        for detections in detector.camera_detection():
            print("Detections:", detections)  # list of dicts per frame
    except KeyboardInterrupt:
        detector.close()
