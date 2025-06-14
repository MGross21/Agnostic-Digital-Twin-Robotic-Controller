import cv2
import numpy as np
from ultralytics import YOLO

class YOLOPoseHomography:
    def __init__(self, yolo_model_path, aruco_dict_type=cv2.aruco.DICT_4X4_50, camera_matrix=None, dist_coeffs=None):
        self.model = YOLO(yolo_model_path)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.camera_matrix = camera_matrix if camera_matrix is not None else np.array([[1200, 0, 640], [0, 1200, 360], [0, 0, 1]])
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros(5)
        self.homography_matrix = None
        self.aruco_order = [0, 1, 2, 3]  # 0: TL, 1: TR, 2: BL, 3: BR
        self.field_size = (150, 150)  # mm

    def detect_aruco_tags(self, frame):
        undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        corners, ids, _ = self.aruco_detector.detectMarkers(undistorted)
        if ids is not None and len(ids) >= 4:
            # Sort corners by id order (0,1,2,3)
            id_list = ids.flatten().tolist()
            image_coords = [None]*4
            for idx, aruco_id in enumerate(id_list):
                if aruco_id in self.aruco_order:
                    image_coords[self.aruco_order.index(aruco_id)] = np.mean(corners[idx][0], axis=0)
            if all(c is not None for c in image_coords):
                image_coords = np.array(image_coords, dtype=np.float32)
                # Normalized field: (0,0), (1,0), (0,1), (1,1)
                world_coords = np.array([[0,0],[1,0],[0,1],[1,1]], dtype=np.float32)
                self.homography_matrix, _ = cv2.findHomography(image_coords, world_coords)
        return corners, ids

    def detect_objects(self, frame):
        results = self.model.predict(frame)
        objects = []
        if len(results) > 0:
            result = results[0]
            for i, box in enumerate(result.boxes.xyxy.cpu().numpy()):
                x_min, y_min, x_max, y_max = box[:4]
                x_center = (x_min + x_max) / 2
                y_center = (y_min + y_max) / 2
                norm_x, norm_y = None, None
                all_inside = True
                if self.homography_matrix is not None:
                    # Transform all four corners and center
                    pts = np.array([
                        [[x_min, y_min]],
                        [[x_max, y_min]],
                        [[x_max, y_max]],
                        [[x_min, y_max]],
                        [[x_center, y_center]]
                    ], dtype=np.float32)
                    norm_pts = cv2.perspectiveTransform(pts, self.homography_matrix)
                    # Check all corners and center are within [0,1] x [0,1]
                    for pt in norm_pts:
                        nx, ny = pt[0]
                        if not (0 <= nx <= 1 and 0 <= ny <= 1):
                            all_inside = False
                            break
                    norm_x, norm_y = norm_pts[-1][0]
                if not all_inside:
                    continue
                label = result.names[int(result.boxes.cls[i].cpu().numpy())]
                conf = float(result.boxes.conf[i].cpu().numpy())
                obj = {
                    'label': label,
                    'confidence': conf,
                    'bbox': [float(x_min), float(y_min), float(x_max), float(y_max)],
                    'center_px': [float(x_center), float(y_center)],
                    'center_norm': [float(norm_x), float(norm_y)] if norm_x is not None else None,
                    # 'z': None,  # Z estimation not implemented
                    # 'orientation': None  # Orientation estimation not implemented
                }
                objects.append(obj)
        return objects

    def process_frame(self, frame):
        aruco_corners, aruco_ids = self.detect_aruco_tags(frame)
        # Draw detected ArUCo markers on the frame
        if aruco_ids is not None:
            frame = cv2.aruco.drawDetectedMarkers(frame.copy(), aruco_corners, aruco_ids)
            # Draw workspace area by connecting the centers of the 4 tag corners (in order: 0,1,3,2)
            id_list = aruco_ids.flatten().tolist()
            centers = [None]*4
            for idx, aruco_id in enumerate(id_list):
                if aruco_id in self.aruco_order:
                    # Center of each tag
                    centers[self.aruco_order.index(aruco_id)] = np.mean(aruco_corners[idx][0], axis=0)
            if all(c is not None for c in centers):
                # Order: 0 (TL), 1 (TR), 3 (BR), 2 (BL), then back to 0
                polygon = np.array([centers[0], centers[1], centers[3], centers[2]], dtype=np.int32)
                cv2.polylines(frame, [polygon], isClosed=True, color=(255,0,255), thickness=2)
        objects = self.detect_objects(frame)
        # Draw bounding boxes and normalized field annotation
        for obj in objects:
            x_min, y_min, x_max, y_max = map(int, obj['bbox'])
            cx, cy = map(int, obj['center_px'])
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0,255,0), 2)
            cv2.circle(frame, (cx, cy), 4, (0,0,255), -1)
            label = obj['label']
            conf = obj['confidence']
            if obj['center_norm']:
                norm_x, norm_y = obj['center_norm']
                # Merge label, confidence, and normalized (x, y) into one line
                text = f'{label} {conf:.2f} ({norm_x:.2f},{norm_y:.2f})'
            else:
                text = f'{label} {conf:.2f}'
            # Draw single annotation, smaller font size
            cv2.putText(frame, text, (x_min, y_min-8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 1)
        return frame, objects

    def preview(self, camera_id=0):
        cap = cv2.VideoCapture(camera_id)
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            processed_frame, _ = self.process_frame(frame)
            cv2.imshow('YOLO Pose Homography', processed_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = YOLOPoseHomography(yolo_model_path="yolo11m.pt")
    detector.preview(camera_id=0)
