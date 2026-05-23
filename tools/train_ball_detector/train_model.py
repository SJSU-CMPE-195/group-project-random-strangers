from __future__ import annotations

import argparse
from pathlib import Path
from typing import List

from ultralytics import YOLO

from coco_utils import YOLO_DIR
from hardware_utils import select_best_device


def parse_yolo_batch_size(raw_batch_size: float, parser: argparse.ArgumentParser) -> int | float:
    """Validate and normalize the YOLO batch-size argument."""
    if raw_batch_size == -1:
        return int(raw_batch_size)
    if 0 < raw_batch_size < 1:
        return float(raw_batch_size)
    if raw_batch_size > 1:
        return int(raw_batch_size)

    parser.error("--batch-size must be -1, >0 and <1, or >1")
    raise AssertionError("parser.error should exit")


def find_best_or_last_weights(weights_dir: Path) -> Path:
    """Prefer best.pt, then fall back to last.pt."""
    best_weights = weights_dir / "best.pt"
    if best_weights.exists():
        return best_weights

    last_weights = weights_dir / "last.pt"
    if last_weights.exists():
        return last_weights

    raise FileNotFoundError(f"No best.pt or last.pt found under {weights_dir}")


def train_one_model(
    args: argparse.Namespace,
    *,
    data_yaml: Path,
    device: str,
    model: str,
) -> Path:
    """Train YOLO and return the best available weights path."""
    print(f"[INFO] Training {model} on {device}")
    yolo_model = YOLO(model)
    batch_size = parse_yolo_batch_size(args.batch_size, args.parser)

    results = yolo_model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=batch_size,
        device=device,
        amp=True,
        workers=args.cpu_workers,
        project=args.project,
        name=f"{args.run_prefix}_{Path(model).stem}",
        pretrained=True,
        patience=args.patience,
        cache=True if args.cache == "ram" else "disk" if args.cache == "disk" else False,
        exist_ok=args.exist_ok,
    )

    return find_best_or_last_weights(Path(results.save_dir) / "weights")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", nargs="+", default=["yolo26n.pt"])
    parser.add_argument("--data", default=str(YOLO_DIR / "coco_person_ball_subset.yaml"))
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch-size", type=float, default=24)
    parser.add_argument("--cpu-workers", type=int, default=8)
    parser.add_argument("--cache", default="disk", choices=["disk", "ram", "false"])
    parser.add_argument("--project", default="runs/person_ball")
    parser.add_argument("--run-prefix", default="subset_person_ball")
    parser.add_argument("--patience", type=int, default=20)
    parser.add_argument("--exist-ok", action="store_true")
    parser.add_argument("--device", default="auto", choices=["auto", "cuda", "mps", "cpu"])
    args = parser.parse_args()
    args.parser = parser
    return args


def main() -> None:
    args = parse_args()
    data_yaml = Path(args.data)
    if not data_yaml.exists():
        args.parser.error(f"Missing dataset YAML: {data_yaml}")

    if args.device == "auto":
        device, _use_half = select_best_device(args.parser)
    elif args.device == "cuda":
        device = "cuda:0"
    else:
        device = args.device

    weights: List[Path] = []
    for model in args.model:
        weights.append(train_one_model(args, data_yaml=data_yaml, device=device, model=model))

    for weight_path in weights:
        print(f"[WEIGHTS] {weight_path}")


if __name__ == "__main__":
    main()
