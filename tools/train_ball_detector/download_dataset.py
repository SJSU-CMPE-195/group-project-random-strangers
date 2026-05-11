from __future__ import annotations

import argparse
import json
import random
from pathlib import Path
from typing import List, Optional

from coco_utils import (
    MANIFEST_DIR,
    YOLO_DIR,
    build_yolo_labels_from_coco_annotations,
    download_split_images,
    ensure_coco_annotations_exist,
    load_coco_instances_json,
    sample_images_with_class_balance,
    split_labeled_images_by_ball_presence,
    write_split_manifest,
)


def compute_max_images_for_ratios(
    *,
    background_count: int,
    ball_count: int,
    other_labeled_count: int,
    background_ratio: float,
    min_ball_ratio: float,
) -> int:
    other_ratio = 1.0 - background_ratio - min_ball_ratio
    if other_ratio < 0:
        raise ValueError("background_ratio + min_ball_ratio must be <= 1.0")

    limits = []
    if background_ratio > 0:
        limits.append(int(background_count / background_ratio))
    if min_ball_ratio > 0:
        limits.append(int(ball_count / min_ball_ratio))
    if other_ratio > 0:
        limits.append(int(other_labeled_count / other_ratio))

    if not limits:
        return 0

    return max(0, min(limits))


def build_split_manifest_and_download(
    split: str,
    *,
    max_images: Optional[int],
    download_workers: int,
    background_ratio: float,
    min_ball_ratio: float,
    min_det_area: int,
    seed: int,
) -> Path:
    coco_data = load_coco_instances_json(split)
    images_by_id, labels_by_image, rejected_small_detection_ids = build_yolo_labels_from_coco_annotations(
        coco_data,
        min_detection_area_pixels=min_det_area,
    )

    all_image_ids = set(images_by_id.keys())
    labeled_image_ids = set(labels_by_image.keys())
    background_candidate_ids = list(all_image_ids - labeled_image_ids - rejected_small_detection_ids)

    ball_items, other_labeled_items = split_labeled_images_by_ball_presence(labels_by_image)
    background_count = len(background_candidate_ids)
    ball_count = len(ball_items)
    other_labeled_count = len(other_labeled_items)

    if max_images is None:
        max_images = compute_max_images_for_ratios(
            background_count=background_count,
            ball_count=ball_count,
            other_labeled_count=other_labeled_count,
            background_ratio=background_ratio,
            min_ball_ratio=min_ball_ratio,
        )
        print(
            f"[VAL] computed max_images={max_images} from available "
            f"background={background_count}, ball={ball_count}, other_labeled={other_labeled_count}"
        )

    selected_items = sample_images_with_class_balance(
        labels_by_image,
        background_candidate_ids,
        max_images=max_images,
        seed=seed,
        background_ratio=background_ratio,
        min_ball_ratio=min_ball_ratio,
    )

    download_split_images(
        split,
        images_by_id=images_by_id,
        selected_items=selected_items,
        worker_count=download_workers,
    )

    return write_split_manifest(
        split,
        images_by_id=images_by_id,
        selected_items=selected_items,
        rejected_small_detection_ids=rejected_small_detection_ids,
    )


def load_existing_eval_list(output_path: Path) -> List[str]:
    """Return existing eval list entries if the file already exists."""
    if not output_path.exists():
        return []

    entries = []
    for raw_line in output_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        entries.append(line)
    return entries


def write_eval_list(
    *,
    split: str,
    count: int,
    seed: int,
    output_path: Path,
) -> int:
    existing = load_existing_eval_list(output_path)
    if existing:
        print(f"[EVAL] Reusing existing eval list: {output_path}")
        return len(existing)

    manifest_path = MANIFEST_DIR / f"{split}.json"
    if not manifest_path.exists():
        raise FileNotFoundError(f"Missing manifest: {manifest_path}")

    data = manifest_path.read_text(encoding="utf-8")
    items = [item["file_name"] for item in json.loads(data)["items"]]

    if count <= 0:
        output_path.write_text("", encoding="utf-8")
        return 0

    rng = random.Random(seed)
    if count < len(items):
        items = rng.sample(items, count)

    rel_paths = [str((YOLO_DIR / "images" / split / file_name).relative_to(YOLO_DIR)).replace("\\\\", "/") for file_name in items]
    output_path.write_text("\n".join(rel_paths) + "\n", encoding="utf-8")
    return len(rel_paths)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-train", type=int, default=10000)
    parser.add_argument("--download-workers", type=int, default=24)
    parser.add_argument("--background-ratio", type=float, default=0.05)
    parser.add_argument("--min-ball-ratio", type=float, default=0.35)
    parser.add_argument("--min-det-area", type=int, default=0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--eval-count", type=int, default=50)
    parser.add_argument("--eval-seed", type=int, default=42)
    parser.add_argument("--eval-split", default="val2017")
    parser.add_argument("--eval-list", default="eval_list.txt")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    ensure_coco_annotations_exist()

    train_manifest = build_split_manifest_and_download(
        "train2017",
        max_images=args.max_train,
        download_workers=args.download_workers,
        background_ratio=args.background_ratio,
        min_ball_ratio=args.min_ball_ratio,
        min_det_area=args.min_det_area,
        seed=args.seed,
    )
    val_manifest = build_split_manifest_and_download(
        "val2017",
        max_images=None,
        download_workers=args.download_workers,
        background_ratio=args.background_ratio,
        min_ball_ratio=args.min_ball_ratio,
        min_det_area=args.min_det_area,
        seed=args.seed,
    )

    eval_list_path = Path(args.eval_list)
    eval_count = write_eval_list(
        split=args.eval_split,
        count=args.eval_count,
        seed=args.eval_seed,
        output_path=eval_list_path,
    )

    print(f"[MANIFEST] wrote {train_manifest}")
    print(f"[MANIFEST] wrote {val_manifest}")
    print(f"[EVAL] wrote {eval_count} entries to {eval_list_path}")


if __name__ == "__main__":
    main()
