# fast_ball_person_yolo.py
import argparse
import time
import cv2
import torch
from ultralytics import YOLO


PERSON = 0
SPORTS_BALL = 32
CLASSES = [PERSON, SPORTS_BALL]


def select_device():
    if not torch.cuda.is_available():
        return "cpu", False

    try:
        test_boxes = torch.tensor([[0.0, 0.0, 1.0, 1.0]], device="cuda")
        test_scores = torch.tensor([0.9], device="cuda")
        torch.ops.torchvision.nms(test_boxes, test_scores, 0.5)
    except Exception:
        return "cpu", False

    return "cuda:0", True


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--source", default="0", help="Camera index, video path, or stream URL")
    p.add_argument("--model", default="yolo26n.pt", help="yolo26n.pt = fastest")
    p.add_argument("--imgsz", type=int, default=416)
    p.add_argument("--conf", type=float, default=0.35)
    p.add_argument("--iou", type=float, default=0.45)
    p.add_argument("--skip", type=int, default=0)
    p.add_argument("--no-display", action="store_true")
    return p.parse_args()


def open_source(source):
    if str(source).isdigit():
        cap = cv2.VideoCapture(int(source))
    else:
        cap = cv2.VideoCapture(source)

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def main():
    args = parse_args()

    # --- AUTO DEVICE SELECTION ---
    device, use_cuda = select_device()
    half = use_cuda  # FP16 only on GPU

    print(f"[INFO] Using device: {device} (FP16={'ON' if half else 'OFF'})")

    model = YOLO(args.model)

    cap = open_source(args.source)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open source: {args.source}")

    frame_count = 0
    last_boxes = []
    t0 = time.time()
    fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        do_infer = args.skip == 0 or frame_count % (args.skip + 1) == 0

        if do_infer:
            results = model.predict(
                frame,
                imgsz=args.imgsz,
                conf=args.conf,
                iou=args.iou,
                classes=CLASSES,
                device=device,
                half=half,
                verbose=False,
            )[0]

            last_boxes = []
            if results.boxes:
                for box in results.boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    label = "person" if cls == PERSON else "ball"
                    last_boxes.append((x1, y1, x2, y2, label, conf))

        for x1, y1, x2, y2, label, conf in last_boxes:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"{label} {conf:.2f}",
                (x1, max(20, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

        frame_count += 1
        now = time.time()
        if now - t0 >= 1.0:
            fps = frame_count / (now - t0)
            frame_count = 0
            t0 = now

        cv2.putText(
            frame,
            f"FPS: {fps:.2f}",
            (12, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )

        if not args.no_display:
            cv2.imshow("YOLO ball/person detector", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()