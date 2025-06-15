from armctl import *
import mujoco_toolbox as mjtb
from digital_twin.camera.planar_homography_yolo import YOLOPoseHomography
from pathlib import Path
import cv2
import numpy as np
import threading

### Object Z HEIGHTS ###
TABLE_Z = 150  # mm
CUP_Z = 165  # mm
HOVER_Z = 300  # mm

### Robot Offset ###
ROBOT_OFFSET_X = -165  # mm
ROBOT_OFFSET_Y = 520.7  # mm

ROBOT_TCP_Rxyz = [-180,0,0]

### MJCF MODEL PATH ###
MODEL_UR5_VENTION = (Path(__file__).resolve().parent / "digital_twin" / "sim" / "model" / "static" / "ur5e_vention.xml").resolve()

def main():
    detector = YOLOPoseHomography()
    with (
        UR5() as robot,
        mjtb.Simulation(MODEL_UR5_VENTION,controller=mjtb.real_time, initial_conditions={"qpos": robot.HOME_POSITION}) as sim,
    ):
        robot.home()
        sim.gravity = [0, 0, 0]  # Disable gravity for digital twin
        sim.launch(show_menu=False)
        
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
                    # Adjust coordinate mapping for camera-controlled coordinate system
                    real_x = norm_y * detector.field_size[1] + ROBOT_OFFSET_X  # Camera Y -> Robot X
                    real_y = -norm_x * detector.field_size[0] + ROBOT_OFFSET_Y  # Camera -X -> Robot Y
                    target = (real_x, real_y)
                    # Only move if target changed
                    if last_target != target:
                        print(f"Cup detected at (x={real_x:.1f} mm, y={real_y:.1f} mm). Moving robot and digital twin.")

                        try:
                            robot.move_cartesian([real_x, real_y, HOVER_Z, *ROBOT_TCP_Rxyz], speed=750)
                        except AssertionError as e:
                            print(f"Error moving robot: {e}")

                        # Update digital twin after starting robot movement
                        sim.controller(sim.model, sim.data, {"qpos": robot.get_joint_positions()})

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
