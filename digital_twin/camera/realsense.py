import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

# ----------- Initialize RealSense -----------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

# Align depth to color stream
align = rs.align(rs.stream.color)

# ----------- Load YOLOv11 Model -----------
model = YOLO("yolo11m.pt")

# ----------- Real-Time Loop -----------
try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert frames to NumPy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # ----------- Run YOLOv11 Inference -----------
        results = model(color_image)[0]  # First item is result

        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = model.names[cls]

            # Get center of bounding box
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            # Get depth in meters
            depth = depth_image[cy, cx] * 0.001  # mm to meters

            # Draw bounding box and label
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(color_image,
                        f"{label} {conf:.2f} {depth:.2f}m",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2)

        # ----------- Show Frame -----------
        cv2.imshow("YOLOv11 + RealSense", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()