from communication.localpubsub import LocalPubSub
from camera.planar_homography_yolo import YOLOPoseHomography
import cv2


def main():
    pub = LocalPubSub()
    detector = YOLOPoseHomography(yolo_model_path="yolo11m.pt")
    cap = cv2.VideoCapture(1)
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            _, objects = detector.process_frame(frame)
            pub.publish('camera/object_pose', objects)
            # Optional: show window for debug
            cv2.imshow('YOLO Pose Homography', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

