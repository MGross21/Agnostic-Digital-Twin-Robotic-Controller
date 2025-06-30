from armctl import *
import mujoco_toolbox as mjtb
from mujoco_toolbox import glovebox, WORLD_ASSETS, Builder
from digital_twin.camera.planar_homography_yolo import YOLOPoseHomography
from pathlib import Path
import cv2
import numpy as np
import mujoco
import math

### Object Z HEIGHTS ###
TABLE_Z = -230  # mm
CUP_Z = -200  # mm
HOVER_Z = -50  # mm

### Robot Offset ###
ROBOT_OFFSET_X = -165  # mm
ROBOT_OFFSET_Y = 520.7  # mm

ROBOT_TCP_Rxyz = [0,0,0] # No rotation for UR5 TCP

# Glovebox dimensions
GB_WIDTH = 75 # in
GB_DEPTH = 60 # in
GB_HEIGHT = 40 # in

IN_TO_M = 0.0254  # Inches to meters conversion

VENTION_STATIC_POSITION = -0.5

### MJCF MODEL PATH ###
MODEL_UR5_DIR = (Path(__file__).resolve().parent / "digital_twin" / "sim" / "model" / "static" ).resolve()
MODEL_UR5_VENTION = MODEL_UR5_DIR / "ur5e_vention.xml"
MODEL_MESHES = MODEL_UR5_DIR / "UniversalRobotics" / "UR5e" / "meshes"

gb = glovebox(width=GB_WIDTH*IN_TO_M, height=GB_HEIGHT*IN_TO_M, depth=GB_DEPTH*IN_TO_M, pos_x=0.5, pos_y=-0.4)

build = Builder((MODEL_UR5_VENTION.read_text()).format(meshdir=MODEL_MESHES.as_posix()), gb, WORLD_ASSETS)

def main():
    detector = YOLOPoseHomography()

    # Validate camera feed
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Unable to access the camera.")
        return

    # Initialize last_target to track movement
    last_target = None

    # Desired cup location
    motion_target = (900, 400)  # mm, relative to the robot's TCP (camera position)

    with (
        UR5() as ur5,
        mjtb.Simulation(build, controller=mjtb.real_time, initial_conditions={"qpos": [VENTION_STATIC_POSITION, *ur5.HOME_POSITION]}) as sim,
    ):
        # Set up the digital twin
        mujoco.mj_resetDataKeyframe(sim.model, sim.data, 0)
        sim._model.qpos0 = [0, 0, math.pi/2, -math.pi/2, math.pi/2, math.pi/2, 0]  # Initial joint positions
        sim.gravity = [0, 0, 0]
        sim.launch(show_menu=False)

        ur5.home()
        try:
            print("Starting digital twin. Press 'q' or Ctrl+C to stop.")
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    print("Error: Camera frame not received. Exiting.")
                    break

                # Process the frame and detect objects
                processed_frame, objects = detector.process_frame(frame)

                # Find the first cup detected
                cup_obj = next((obj for obj in objects if obj['label'].lower().startswith('cup') and obj['center_norm']), None)
                if cup_obj:
                    norm_x, norm_y = cup_obj['center_norm']
                    real_x = norm_y * detector.field_size[1] + ROBOT_OFFSET_X
                    real_y = -norm_x * detector.field_size[0] + ROBOT_OFFSET_Y
                    target = (real_x, real_y)

                    if last_target != target:
                        motion_target = target
                        print(f"Cup detected at (x={real_x:.1f} mm, y={real_y:.1f} mm). Moving robot and digital twin.")

                        try:
                            ur5.move_cartesian([real_x, real_y, HOVER_Z, *ROBOT_TCP_Rxyz], speed=0.1)
                        except AssertionError as e:
                            print(f"Error moving robot: {e}")

                        robot_joint_positions = ur5.get_joint_positions()
                        sim.controller(sim.model, sim.data, {"qpos": [VENTION_STATIC_POSITION, *robot_joint_positions]})

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
