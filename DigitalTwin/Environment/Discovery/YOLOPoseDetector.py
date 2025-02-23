from ultralytics import YOLO
import cv2
import torch
import os
import sys

torch.cuda.empty_cache()
YOLO_VERBOSE = False

# Model Source https://docs.ultralytics.com/tasks/pose/
class YOLOPoseDetector(object):
    def __init__(self, model_path='yolo11m.pt', camera_id=0, conf_threshold=0.5, show_window=True, verbose=False):
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
        if self.cap is None:
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                raise ValueError(f"Unable to open camera with id {self.camera_id}")
            self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def release_camera(self):
        if self.cap:
            self.cap.release()
            self.cap = None

    def load_model(self):
        if not self.verbose:
            # Suppress terminal output
            sys.stdout = open(os.devnull, 'w')
        self.model = YOLO(self.model_path)
        if not self.verbose:
            # Restore terminal output
            sys.stdout = sys.__stdout__

    def detect_pose(self, image):
        if self.model is None:
            self.load_model()
        results = self.model(image)
        return results[0].plot()

    def camera_detection(self):
        self.initialize_camera()
        if self.model is None:
            self.load_model()
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    break
                
                # Perform object detection
                results = self.model(frame)

                # List to store all keypoints
                all_keypoints = []

                # Visualize the results on the frame
                for result in results:
                    boxes = result.boxes.cpu().numpy()
                    for box in boxes:
                        # Extract the center coordinates, width, and height
                        cx, cy, w, h = box.xywh[0].astype(float)
                        rect = ((cx, cy), (w, h), 0)  # Angle is set to 0 as it's not provided
                        box_points = cv2.boxPoints(rect).astype(int)

                        # Draw all boxes regardless of confidence score
                        cv2.drawContours(frame, [box_points], 0, (0, 255, 0), 2)
                        cv2.putText(frame, f'{result.names[int(box.cls[0])]} {box.conf[0]:.2f}',
                                    (int(cx), int(cy) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        # Normalize the coordinates and convert to tuple
                        normalized_cx = cx / self.frame_width
                        normalized_cy = cy / self.frame_height
                        all_keypoints.append((float(normalized_cx), float(normalized_cy)))

                # Display the frame if show_window is True
                if self.show_window:
                    cv2.imshow('YOLO Object Detection', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                yield all_keypoints
        finally:
            self.release_camera()
            if self.show_window:
                cv2.destroyAllWindows()

    def train(self, data_yaml, epochs=100, imgsz=640):
        if self.model is None:
            self.load_model()
        self.model.train(data=data_yaml, epochs=epochs, imgsz=imgsz)

    def validate(self):
        if self.model is None:
            self.load_model()
        return self.model.val()

    def predict_image(self, image_path):
        if self.model is None:
            self.load_model()
        return self.model(image_path)

    def close(self):
        print("Closing CV Process...")
        self.release_camera()
        if self.show_window:
            cv2.destroyAllWindows()
        sys.exit(0)

if __name__ == '__main__':
    detector = YOLOPoseDetector(camera_id=1, show_window=True, verbose=False, conf_threshold=0.25)
    try:
        for keypoints in detector.camera_detection():
            print("Detected keypoints:", keypoints)
    except KeyboardInterrupt:
        detector.close()