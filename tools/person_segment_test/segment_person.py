import time

import torch
import cv2
from ultralytics import YOLO

MODEL_PATH = "yolo12l-person-seg-extended.pt"
CAMERA_INDEX = 1
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
CONFIDENCE = 0.25
WINDOW_NAME = "Live Person Segmentation"


def main() -> None:
    model = YOLO(MODEL_PATH)

    capture = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not capture.isOpened():
        raise RuntimeError(f"Could not open webcam at index {CAMERA_INDEX}.")
    
    if torch.cuda.is_available():
        print(f"Using CUDA device: {torch.cuda.get_device_name(0)}")
        model.to('cuda')
        device = 'cuda'
        use_half = True
    elif hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
        print("Using Apple Silicon MPS")
        model.to('mps')
        device = 'mps'
        use_half = False
    else:
        print("Using CPU")
        device = None
        use_half = False

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    previous_time = time.perf_counter()
    fps_ema = 0.0

    try:
        while True:
            success, frame = capture.read()
            if not success:
                continue

            if device == 'cuda':
                results = model(frame, classes=0, conf=CONFIDENCE, device=device, half=use_half)
            elif device == 'mps':
                results = model(frame, classes=0, conf=CONFIDENCE, device=device)
            else:
                results = model(frame, classes=0, conf=CONFIDENCE)
            results = model.predict(frame, conf=CONFIDENCE, verbose=False)
            annotated_frame = results[0].plot()

            current_time = time.perf_counter()
            instantaneous_fps = 1.0 / max(current_time - previous_time, 1e-6)
            previous_time = current_time
            fps_ema = instantaneous_fps if fps_ema == 0.0 else (fps_ema * 0.9 + instantaneous_fps * 0.1)

            cv2.putText(
                annotated_frame,
                f"FPS: {fps_ema:.1f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow(WINDOW_NAME, annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
