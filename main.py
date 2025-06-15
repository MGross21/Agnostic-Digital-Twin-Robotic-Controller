from armctl import *
import mujoco_toolbox as mjtb
from digital_twin.camera.planar_homography_yolo import YOLOPoseHomography
from pathlib import Path
import cv2
import numpy as np

### Object Z HEIGHTS ###
TABLE_Z = 150  # mm
CUP_Z = 165  # mm
HOVER_Z = 300  # mm

### Robot Offset ###
ROBOT_OFFSET_X = -165  # mm
ROBOT_OFFSET_Y = 520.7  # mm

# URDF and Mesh Paths
URDF_DIR = (Path(__file__).resolve().parent / "digital_twin" / "sim" / "model" / "static" / "ElephantRobotics" / "mycobot_pro600").resolve()
URDF = str((URDF_DIR / "mycobot_pro600.urdf").resolve())
URDF_MESH_DIR = str((URDF_DIR / "meshes").resolve())

def main():
    detector = YOLOPoseHomography()
    with (
        Pro600() as robot,
        mjtb.Simulation(URDF, meshdir=URDF_MESH_DIR) as digital_twin,
    ):
        digital_twin.gravity = [0, 0, 0]  # Disable gravity for digital twin
        digital_twin.qpos0 = np.array(robot.HOME_POSITION) # Initialize digital twin position
        digital_twin.launch(show_menu=False)
        
        cap = cv2.VideoCapture(0)
        last_target = None
        try:
            print("Starting digital twin. Press 'q' or Ctrl+C to stop.")
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    print("Camera frame not received. Exiting.")
                    break
                processed_frame, objects = detector.process_frame(frame)
                # Find the first cup detected
                cup_obj = next((obj for obj in objects if obj['label'].lower().startswith('cup') and obj['center_norm']), None)
                if cup_obj:
                    norm_x, norm_y = cup_obj['center_norm']
                    real_x = norm_x * detector.field_size[0] + ROBOT_OFFSET_X
                    real_y = norm_y * detector.field_size[1] + ROBOT_OFFSET_Y
                    target = (real_x, real_y)
                    # Only move if target changed
                    if last_target != target:
                        print(f"Cup detected at (x={real_x:.1f} mm, y={real_y:.1f} mm). Moving robot and digital twin.")

                        robot.move_cartesian(robot_pose=[real_x, real_y, HOVER_Z, 0, 0, 0], speed=500)
                        digital_twin.qpos = np.array(robot.get_joint_positions())

                        last_target = target
                cv2.imshow('Digital Twin', processed_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Quitting digital twin.")
                    break
        except KeyboardInterrupt:
            print("Digital twin interrupted by user.")
        finally:
            cap.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
