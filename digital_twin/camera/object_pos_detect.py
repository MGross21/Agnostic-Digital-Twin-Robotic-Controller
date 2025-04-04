import cv2
import numpy as np
from ultralytics import YOLO

class ObjectPositioningSystem:
    def __init__(self, yolo_model_path, aruco_dict_type=cv2.aruco.DICT_6X6_250, camera_matrix=None, dist_coeffs=None):
        # Initialize YOLO model
        self.model = YOLO(yolo_model_path)
        
        # Initialize ArUco dictionary and detector parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Camera calibration data
        self.camera_matrix = camera_matrix if camera_matrix is not None else np.array([[1200, 0, 640], [0, 1200, 360], [0, 0, 1]])
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros(5)
        
        # Homography matrix for real-world coordinate transformation
        self.homography_matrix = None

    def detect_aruco_tags(self, frame):
        """Detect ArUCo markers and calculate homography matrix."""
        undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        
        # Detect markers
        corners, ids, _ = self.aruco_detector.detectMarkers(undistorted_frame)
        
        if ids is not None and len(ids) >= 4:
            # Define real-world coordinates (12x7.5 inches converted to cm)
            world_coords = np.array([[0, 0], [12*2.54, 0], [12*2.54, 7.5*2.54], [0, 7.5*2.54]], dtype=np.float32)
            
            # Extract image coordinates from detected markers
            image_coords = []
            for i in range(len(ids)):
                image_coords.append(np.mean(corners[i][0], axis=0))
            image_coords = np.array(image_coords[:4], dtype=np.float32)  # Use first four markers
            
            # Compute homography matrix
            self.homography_matrix, _ = cv2.findHomography(image_coords, world_coords)
        
        return corners, ids

    def detect_objects(self, frame):
        """Detect objects using YOLO and draw bounding boxes."""
        results = self.model.predict(frame)  # Returns a list of results
        
        if len(results) > 0:
            result = results[0]  # Access the first (and only) result
            
            for box in result.boxes.xyxy.cpu().numpy():
                x_min, y_min, x_max, y_max = box[:4]
                x_center = (x_min + x_max) / 2
                y_center = (y_min + y_max) / 2
                
                # Draw bounding box on the frame
                cv2.rectangle(frame,
                              (int(x_min), int(y_min)),
                              (int(x_max), int(y_max)),
                              color=(0, 255, 0), thickness=2)
                
                # Annotate with pixel coordinates
                cv2.putText(frame,
                            f"Pixel: ({int(x_center)}, {int(y_center)})",
                            (int(x_min), int(y_min) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 255, 255),
                            thickness=1)
                
                # Transform to real-world coordinates if homography is available
                if self.homography_matrix is not None:
                    world_pos = cv2.perspectiveTransform(
                        np.array([[[x_center, y_center]]], dtype=np.float32),
                        self.homography_matrix
                    )
                    world_x, world_y = world_pos[0][0]
                    
                    # Annotate with real-world coordinates
                    cv2.putText(frame,
                                f"Real: ({world_x:.1f}cm, {world_y:.1f}cm)",
                                (int(x_min), int(y_min) - 30),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (255, 255, 255),
                                thickness=1)

    def process_frame(self, frame):
        """Process a single frame to detect ArUCo tags and objects."""
        aruco_corners, aruco_ids = self.detect_aruco_tags(frame)
        
        # Draw detected ArUCo markers on the frame
        if aruco_ids is not None:
            frame = cv2.aruco.drawDetectedMarkers(frame.copy(), aruco_corners, aruco_ids)
        
        # Detect objects and annotate the frame with bounding boxes and coordinates
        self.detect_objects(frame)
        
        return frame

    def preview(self):
        """Open a preview window to display live detection."""
        cap = cv2.VideoCapture(1)  # Use webcam
        
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            processed_frame = self.process_frame(frame)
            cv2.imshow("Object Positioning System", processed_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # Quit on 'q'
                break
        
        cap.release()
        cv2.destroyAllWindows()


# ======== USAGE EXAMPLE ========
if __name__ == "__main__":
    system = ObjectPositioningSystem(yolo_model_path="yolo11m.pt")
    system.preview()