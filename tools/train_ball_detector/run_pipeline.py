from __future__ import annotations

import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent


def run_command(args: list[str]) -> None:
    print(f"[RUN] {' '.join(args)}")
    subprocess.run(args, check=True)


def main() -> None:
    python = sys.executable

    run_command(
        [
            python,
            str(SCRIPT_DIR / "download_dataset.py"),
            "--max-train",
            "4000",
            "--max-val",
            "278",
            "--download-workers",
            "24",
            "--min-ball-ratio",
            "0.5",
            "--min-det-area",
            "64",
            "--eval-count",
            "50",
        ]
    )

    run_command(
        [
            python,
            str(SCRIPT_DIR / "convert_labels.py"),
            "--write-yaml",
        ]
    )

    run_command(
        [
            python,
            str(SCRIPT_DIR / "train_model.py"),
            "--model",
            "yolo26s",
            "--epochs",
            "200",
            "--imgsz",
            "1280",
            "--batch-size",
            "0.9",
            "--cpu-workers",
            "4",
            "--cache",
            "disk",
        ]
    )

    run_command(
        [
            python,
            str(SCRIPT_DIR / "benchmark_export.py"),
            "--weights",
            "runs/person_ball/subset_person_ball_yolo26s/weights/best.pt",
            "--imgsz",
            "1280",
            "--eval-list",
            "eval_list.txt",
            "--export",
        ]
    )


if __name__ == "__main__":
    main()
