import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import cv2
from camera.planar_homography_yolo import YOLOPoseHomography
import imageio


def main():
    detector = YOLOPoseHomography()
    xs, ys = [], []
    cap = cv2.VideoCapture(0)
    frames = []
    fig, ax = plt.subplots()
    ax.set_xlim(0, detector.field_size[0])
    ax.set_ylim(0, detector.field_size[1])
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_title('Cup Positions')
    ax.legend(['Cup Path'])
    try:
        print("Test Started. Press Ctrl+C to stop.")
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            _, objects = detector.process_frame(frame)
            for obj in objects:
                if obj['label'].lower().startswith('cup') and obj['center_norm']:
                    norm_x, norm_y = obj['center_norm']
                    real_x = norm_x * detector.field_size[0]
                    real_y = norm_y * detector.field_size[1]
                    xs.append(real_x)
                    ys.append(real_y)
            if len(xs) > 1:
                ax.clear()
                ax.set_xlim(0, detector.field_size[0])
                ax.set_ylim(0, detector.field_size[1])
                ax.set_xlabel('X (mm)')
                ax.set_ylabel('Y (mm)')
                ax.set_title('Cup Positions')
                ax.plot(xs, ys, '-ob', markersize=3, linewidth=1)
                fig.tight_layout()
                fig.canvas.draw()
                img = np.array(fig.canvas.renderer.buffer_rgba())
                frames.append(img)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if frames:
            imageio.mimsave('cup_positions.gif', frames, fps=30)
            print('Saved cup_positions.gif')
        else:
            print('No frames to save.')

if __name__ == "__main__":
    main()
