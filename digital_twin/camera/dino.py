import pyrealsense2 as rs
import numpy as np
import cv2
from groundingdino.util.inference import load_model, predict
from ur5_ik_controller import compute_ik_and_send_command  # Your custom IK + socket controller

# ------------------------------
# Config
# ------------------------------
OBJECT_PROMPT = "a tool, a handle, a box, a black object"
DINO_WEIGHTS = "groundingdino_swint_ogc.pth"
DINO_CONFIG = "GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
DEPTH_SCALE = 0.001  # RealSense returns depth in mm; convert to meters

# ------------------------------
# Init Camera + DINO
# ------------------------------
model = load_model(DINO_WEIGHTS, config_path=DINO_CONFIG)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)
align = rs.align(rs.stream.color)

try:
    while True:
        # -------------------------
        # Get aligned RealSense frame
        # -------------------------
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color_img = np.asanyarray(color_frame.get_data())
        depth_img = np.asanyarray(depth_frame.get_data())

        # -------------------------
        # Run Grounding DINO
        # -------------------------
        boxes, logits, phrases = predict(model, color_img, OBJECT_PROMPT)

        if len(boxes) == 0:
            print("No objects found. Retrying...")
            continue

        # -------------------------
        # Choose closest object
        # -------------------------
        min_depth = float('inf')
        chosen_pose = None

        for box in boxes:
            x1, y1, x2, y2 = map(int, box)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            depth = depth_img[cy, cx] * DEPTH_SCALE

            if 0 < depth < min_depth:
                min_depth = depth
                chosen_pose = (cx, cy, depth)

        if chosen_pose is None:
            print("All objects too far or invalid depth.")
            continue

        # -------------------------
        # Convert (u, v, depth) to 3D world coordinate
        # -------------------------
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        u, v, z = chosen_pose
        x, y, z = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], z)

        # -------------------------
        # Send pose to IK solver and actuate
        # -------------------------
        print(f"Target pose: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        compute_ik_and_send_command(x, y, z)

        # Optionally: remove or ignore this object on next iteration

except KeyboardInterrupt:
    print("Shutting down.")
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
