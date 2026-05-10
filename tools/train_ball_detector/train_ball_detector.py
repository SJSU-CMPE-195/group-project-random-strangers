# train_coco_person_ball_subset.py
from pathlib import Path
from zipfile import ZipFile
import concurrent.futures
import traceback
import os
import time
import argparse
import json
import shutil
import random
import requests
import torch
from tqdm import tqdm
from ultralytics import YOLO
from pathlib import Path


ROOT = Path("datasets/coco_person_ball_subset")
RAW = ROOT / "raw"
OUT = ROOT / "yolo"

ANN_URL = "http://images.cocodataset.org/annotations/annotations_trainval2017.zip"

COCO_PERSON = 1
COCO_SPORTS_BALL = 37

NEW_CLASS_MAP = {
    COCO_PERSON: 0,
    COCO_SPORTS_BALL: 1,
}


def download_file(url, dest, show_progress=True, max_retries=3, timeout=60):

    dest.parent.mkdir(parents=True, exist_ok=True)

    temp_path = dest.with_suffix(dest.suffix + ".part")

    for attempt in range(1, max_retries + 1):
        try:
            with requests.get(url, stream=True, timeout=timeout) as r:
                r.raise_for_status()
                total = int(r.headers.get("content-length", 0))

                with open(temp_path, "wb") as f:
                    if show_progress:
                        with tqdm(
                            total=total,
                            unit="B",
                            unit_scale=True,
                            desc=dest.name,
                        ) as bar:
                            for chunk in r.iter_content(1024 * 1024):
                                if chunk:
                                    f.write(chunk)
                                    bar.update(len(chunk))
                    else:
                        for chunk in r.iter_content(1024 * 1024):
                            if chunk:
                                f.write(chunk)

            # move temp file to final destination
            os.replace(str(temp_path), str(dest))
            return

        except Exception as e:
            if temp_path.exists():
                try:
                    temp_path.unlink()
                except Exception:
                    pass

            if attempt >= max_retries:
                print(f"[FAIL] {dest} after {attempt} attempts: {e}")
                raise
            else:
                backoff = 2 ** (attempt - 1)
                print(f"[RETRY] {dest} (attempt {attempt}/{max_retries}), retrying in {backoff}s")
                time.sleep(backoff)


def unzip(zip_path, dest):
    marker = dest / f".unzipped_{zip_path.stem}"
    if marker.exists():
        return

    with ZipFile(zip_path, "r") as z:
        z.extractall(dest)

    marker.touch()


def coco_to_yolo_bbox(bbox, width, height):
    x, y, w, h = bbox
    return (
        (x + w / 2) / width,
        (y + h / 2) / height,
        w / width,
        h / height,
    )
    
def stratified_sample_items(labels_by_image, max_images, seed=42, min_ball_ratio=0.35):
    rng = random.Random(seed)

    ball_items = []
    person_only_items = []

    for image_id, lines in labels_by_image.items():
        classes = {int(line.split()[0]) for line in lines}

        if 1 in classes:  # ball
            ball_items.append((image_id, lines))
        elif 0 in classes:  # person only
            person_only_items.append((image_id, lines))

    rng.shuffle(ball_items)
    rng.shuffle(person_only_items)

    if not max_images:
        return ball_items + person_only_items

    target_ball = int(max_images * min_ball_ratio)

    selected_ball = ball_items[:target_ball]
    remaining = max_images - len(selected_ball)

    selected_person = person_only_items[:remaining]

    # If not enough person-only images, fill with extra ball images.
    if len(selected_person) < remaining:
        extra_needed = remaining - len(selected_person)
        selected_person += ball_items[target_ball:target_ball + extra_needed]

    selected = selected_ball + selected_person
    rng.shuffle(selected)

    print(
        f"[SAMPLE] requested={max_images}, "
        f"selected={len(selected)}, "
        f"ball_images={len(selected_ball)}, "
        f"person_only_images={len(selected_person)}"
    )

    return selected


def convert_and_download_split(split, max_images=None, workers=8, background_ratio=0.05, seed=42, min_ball_ratio=0.35):
    ann_path = RAW / "annotations" / f"instances_{split}.json"

    out_img_dir = OUT / "images" / split
    out_lbl_dir = OUT / "labels" / split
    out_img_dir.mkdir(parents=True, exist_ok=True)
    out_lbl_dir.mkdir(parents=True, exist_ok=True)

    with open(ann_path, "r") as f:
        data = json.load(f)

    images = {img["id"]: img for img in data["images"]}
    labels_by_image = {}

    for ann in data["annotations"]:
        cat_id = ann["category_id"]

        if cat_id not in NEW_CLASS_MAP:
            continue

        if ann.get("iscrowd", 0):
            continue

        x, y, w, h = ann["bbox"]
        if w <= 1 or h <= 1:
            continue

        img = images[ann["image_id"]]
        cls = NEW_CLASS_MAP[cat_id]
        box = coco_to_yolo_bbox(ann["bbox"], img["width"], img["height"])

        line = f"{cls} " + " ".join(f"{v:.6f}" for v in box)
        labels_by_image.setdefault(ann["image_id"], []).append(line)

    positive_items = stratified_sample_items(
            labels_by_image,
            max_images=max_images,
            seed=seed,
            min_ball_ratio=min_ball_ratio,
        )

    # Add a small proportion of true background images (no person/ball labels).
    # Keep the final ratio close to `background_ratio` of the whole split export.
    all_image_ids = set(images.keys())
    positive_image_ids = set(labels_by_image.keys())
    background_candidates = list(all_image_ids - positive_image_ids)

    rng = random.Random(seed)
    rng.shuffle(background_candidates)

    if background_ratio <= 0:
        background_count = 0
    elif background_ratio >= 1:
        background_count = len(background_candidates)
    else:
        # Solve for backgrounds so that backgrounds/(positives+backgrounds) ~= ratio.
        background_count = int(round(len(positive_items) * background_ratio / (1 - background_ratio)))

    background_count = min(background_count, len(background_candidates))
    background_items = [(img_id, []) for img_id in background_candidates[:background_count]]

    items = positive_items + background_items

    labeled_count = len(positive_items)
    ball_count = sum(1 for _id, lines in positive_items if any(l.split()[0] == "1" for l in lines))
    person_count = sum(1 for _id, lines in positive_items if any(l.split()[0] == "0" for l in lines))

    print(
        f"[{split}] Downloading/converting {len(items)} images "
        f"({labeled_count} labeled: {ball_count} with ball, {person_count} with person; {len(background_items)} background, workers={workers})"
    )

    def _process_item(item):
        image_id, lines = item
        try:
            img = images[image_id]
            file_name = img["file_name"]

            dst_img = out_img_dir / file_name
            dst_lbl = out_lbl_dir / file_name.replace(".jpg", ".txt")

            if not dst_img.exists():
                url = img.get("coco_url")
                if not url:
                    url = f"http://images.cocodataset.org/{split}/{file_name}"
                # disable per-file tqdm when running in threads
                download_file(url, dst_img, show_progress=False)

            with open(dst_lbl, "w") as f:
                if lines:
                    f.write("\n".join(lines) + "\n")
                else:
                    # Empty label file => background (negative) sample for YOLO training.
                    f.write("")
            return True
        except Exception:
            print(f"[ERROR] processing image {image_id}:\n" + traceback.format_exc())
            return False

    # use a ThreadPoolExecutor for I/O-bound downloads
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as ex:
        futures = {ex.submit(_process_item, it): it for it in items}
        with tqdm(total=len(futures), desc=split) as pbar:
            for fut in concurrent.futures.as_completed(futures):
                _ = fut.result()
                pbar.update(1)


def write_yaml():
    yaml_path = OUT / "coco_person_ball_subset.yaml"
    yaml_path.write_text(f"""
path: {OUT.resolve()}
train: images/train2017
val: images/val2017

names:
  0: person
  1: ball
""".strip())
    return yaml_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="yolo26n.pt")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--max-train", type=int, default=10000)
    parser.add_argument("--max-val", type=int, default=1000)
    parser.add_argument("--download-workers", type=int, default=16)
    parser.add_argument("--cpu-workers", type=int, default=8)
    parser.add_argument("--cache", default="disk", choices=["disk", "ram", "false"])
    parser.add_argument("--background-ratio", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--min-ball-ratio", type=float, default=0.35)
    args = parser.parse_args()

    RAW.mkdir(parents=True, exist_ok=True)

    ann_zip = RAW / "annotations_trainval2017.zip"
    download_file(ANN_URL, ann_zip)
    unzip(ann_zip, RAW)

    # parallelize downloads/conversion using requested worker count
    convert_and_download_split(
        "train2017",
        max_images=args.max_train,
        workers=args.download_workers,
        background_ratio=args.background_ratio,
        seed=args.seed,
        min_ball_ratio=args.min_ball_ratio,
    )
    convert_and_download_split(
        "val2017",
        max_images=args.max_val,
        workers=args.download_workers,
        background_ratio=args.background_ratio,
        seed=args.seed,
        min_ball_ratio=args.min_ball_ratio,
    )

    data_yaml = write_yaml()

    use_cuda = torch.cuda.is_available()
    device = "cuda:0" if use_cuda else "cpu"

    print(f"[INFO] Training on {'CUDA' if use_cuda else 'CPU'}")

    model = YOLO(args.model)

    model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=0.8,
        device=device,
        amp=use_cuda,
        workers=args.cpu_workers,
        project="runs/person_ball",
        name=f"subset_person_ball_{Path(args.model).stem}",
        pretrained=True,
        patience=20,
        cache = True if args.cache == "ram" else 'disk' if args.cache == "disk" else False,
        exist_ok=False,
    )


if __name__ == "__main__":
    main()