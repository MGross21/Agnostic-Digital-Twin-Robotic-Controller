from ultralytics import YOLO
import cv2
import torch

torch.cuda.empty_cache()

# Model Source https://docs.ultralytics.com/tasks/pose/
class YOLOPoseDetector:
    def __init__(self, model_path='yolo11m.pt', camera_id=0, conf_threshold=0.5, show_window=True):
        self.model = YOLO(model_path)
        self.camera_id = camera_id
        self.cap = None
        self.conf_threshold = conf_threshold
        self.show_window = show_window

    def initialize_camera(self):
        if self.cap is None:
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                raise ValueError(f"Unable to open camera with id {self.camera_id}")

    def release_camera(self):
        if self.cap:
            self.cap.release()
            self.cap = None

    def detect_pose(self, image):
        results = self.model(image)
        return results[0].plot()

    def camera_detection(self):
        self.initialize_camera()
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
                        x1, y1, x2, y2 = box.xyxy[0].astype(int)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, f'{result.names[int(box.cls[0])]} {box.conf[0]:.2f}',
                                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        
                        # Draw keypoints for pose estimation
                        if result.keypoints is not None:
                            keypoints = result.keypoints.cpu().numpy()
                            for keypoint in keypoints:
                                for x, y, conf in keypoint:
                                    if conf > self.conf_threshold:  # confidence threshold
                                        cv2.circle(frame, (int(x), int(y)), 3, (0, 0, 255), -1)
                                        all_keypoints.append((int(x), int(y)))

                # Display the frame if show_window is True
                if self.show_window:
                    cv2.imshow('YOLO Object Detection', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # Yield all keypoints for the current frame
                yield all_keypoints
        finally:
            self.release_camera()
            if self.show_window:
                cv2.destroyAllWindows()

    def train(self, data_yaml, epochs=100, imgsz=640):
        self.model.train(data=data_yaml, epochs=epochs, imgsz=imgsz)

    def validate(self):
        return self.model.val()

    def predict_image(self, image_path):
        return self.model(image_path)

if __name__ == '__main__':
    detector = YOLOPoseDetector(camera_id=0, show_window=True)
    for keypoints in detector.camera_detection():
        print(keypoints)