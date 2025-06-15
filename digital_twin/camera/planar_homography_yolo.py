import cv2
import numpy as np
from ultralytics import YOLO

IN_TO_MM = 25.4

class YOLOPoseHomography:
    def __init__(self, yolo_model_path, aruco_dict_type=cv2.aruco.DICT_4X4_50, camera_matrix=None, dist_coeffs=None):
        self.model = YOLO(yolo_model_path)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.camera_matrix = camera_matrix if camera_matrix is not None else np.array([[1200, 0, 640], [0, 1200, 360], [0, 0, 1]])
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros(5)
        self.homography_matrix = None
        self.aruco_order = [0, 1, 2, 3]  # 0: TL, 1: TR, 2: BR, 3: BL

        self.field_size = (41*IN_TO_MM, 26*IN_TO_MM)  # (width mm, height mm)

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
        # Get the current workspace polygon in image coordinates
        workspace_poly_img = getattr(self, 'last_workspace_img_poly', None)
        if len(results) > 0:
            result = results[0]
            for i, box in enumerate(result.boxes.xyxy.cpu().numpy()):
                x_min, y_min, x_max, y_max = box[:4]
                x_center = (x_min + x_max) / 2
                y_center = (y_min + y_max) / 2
                inside_workspace = True
                if workspace_poly_img is not None:
                    # Check if the center is inside the workspace polygon in image coordinates
                    if cv2.pointPolygonTest(workspace_poly_img, (x_center, y_center), False) < 0:
                        inside_workspace = False
                else:
                    inside_workspace = False
                if not inside_workspace:
                    continue
                norm_x, norm_y = None, None
                if self.homography_matrix is not None:
                    center_pt = np.array([[[x_center, y_center]]], dtype=np.float32)
                    norm_center = cv2.perspectiveTransform(center_pt, self.homography_matrix)[0][0]
                    norm_x, norm_y = norm_center
                label = result.names[int(result.boxes.cls[i].cpu().numpy())]
                conf = float(result.boxes.conf[i].cpu().numpy())
                obj = {
                    'label': label,
                    'confidence': conf,
                    'bbox': [float(x_min), float(y_min), float(x_max), float(y_max)],
                    'center_px': [float(x_center), float(y_center)],
                    'center_norm': [float(norm_x), float(norm_y)] if norm_x is not None else None,
                }
                objects.append(obj)
        return objects

    def process_frame(self, frame):
        aruco_corners, aruco_ids = self.detect_aruco_tags(frame)
        # Draw detected ArUCo markers on the frame
        warped = None
        M = None
        if aruco_ids is not None:
            frame = cv2.aruco.drawDetectedMarkers(frame.copy(), aruco_corners, aruco_ids)
            id_list = aruco_ids.flatten().tolist()
            centers = [None]*4
            for idx, aruco_id in enumerate(id_list):
                if aruco_id in self.aruco_order:
                    centers[self.aruco_order.index(aruco_id)] = np.mean(aruco_corners[idx][0], axis=0)
            if all(c is not None for c in centers):
                polygon = np.array([centers[0], centers[1], centers[2], centers[3]], dtype=np.int32)
                cv2.polylines(frame, [polygon], isClosed=True, color=(255,0,255), thickness=2)
                self.last_workspace_img_poly = polygon
                # Perspective transform: warp the workspace to a top-down view for visualization
                dst_pts = np.array([[0,0],[300,0],[300,300],[0,300]], dtype=np.float32)
                src_pts = np.array([centers[0], centers[1], centers[2], centers[3]], dtype=np.float32)
                if src_pts.shape == (4,2):
                    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
                    warped = cv2.warpPerspective(frame, M, (300,300))
        else:
            self.last_workspace_img_poly = None
        objects = self.detect_objects(frame)
        # Draw bounding boxes and normalized field annotation
        for obj in objects:
            x_min, y_min, x_max, y_max = map(int, obj['bbox'])
            cx, cy = map(int, obj['center_px'])
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0,255,0), 2)
            cv2.circle(frame, (cx, cy), 4, (0,0,255), -1)
            label = obj['label']
            conf = obj['confidence']
            real_x, real_y = None, None
            if obj['center_norm']:
                norm_x, norm_y = obj['center_norm']
                real_x = norm_x * self.field_size[0]
                real_y = norm_y * self.field_size[1]
                text1 = f'{label} {conf:.2f}'
                text2 = f'({real_x:.1f}, {real_y:.1f} mm)'
            else:
                text1 = f'{label} {conf:.2f}'
                text2 = ''
            cv2.putText(frame, text1, (x_min, y_min-25), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255,0,0), 1)
            cv2.putText(frame, text2, (x_min, y_min-5), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255,0,0), 1)
            # Draw object center on top-down view if available
            if 'M' in locals() and warped is not None:
                pt = np.array([[[cx, cy]]], dtype=np.float32)
                warped_pt = cv2.perspectiveTransform(pt, M)[0][0]
                wx, wy = int(warped_pt[0]), int(warped_pt[1])
                cv2.circle(warped, (wx, wy), 4, (0,0,255), -1)
        if warped is not None:
            cv2.imshow('Workspace Top-Down', warped)
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
