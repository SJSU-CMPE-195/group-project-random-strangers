from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, List

from coco_utils import YOLO_DIR, create_split_output_directories, read_split_manifest, write_yolo_dataset_yaml


def write_labels_for_split(split: str) -> int:
    data = read_split_manifest(split)
    _image_dir, label_dir = create_split_output_directories(split)

    count = 0
    for item in data["items"]:
        file_name = item["file_name"]
        label_lines = item.get("label_lines", [])
        destination_label_path = label_dir / file_name.replace(".jpg", ".txt")
        label_text = "\n".join(label_lines) + "\n" if label_lines else ""
        destination_label_path.write_text(label_text, encoding="utf-8")
        count += 1

    return count


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--splits", nargs="+", default=["train2017", "val2017"])
    parser.add_argument("--write-yaml", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    total = 0
    for split in args.splits:
        total += write_labels_for_split(split)
        print(f"[LABELS] wrote labels for {split}")

    if args.write_yaml:
        yaml_path = write_yolo_dataset_yaml()
        print(f"[YAML] wrote {yaml_path}")

    print(f"[LABELS] total={total}")


if __name__ == "__main__":
    main()
