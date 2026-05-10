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


ROOT = Path("datasets/coco_person_ball_subset")
RAW = ROOT / "raw"
OUT = ROOT / "yolo"
EVAL_LIST = Path("eval_images.txt")

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
    
def _target_counts(max_images, background_ratio, min_ball_ratio):
    """Return target counts for background, ball, and other labeled images.

    The counts always sum to at most max_images. `min_ball_ratio` is interpreted
    as the desired ball-image share of the whole exported split, not just the
    positive subset.
    """
    if max_images is None:
        return None, None, None

    max_images = max(0, int(max_images))
    background_ratio = min(max(background_ratio, 0.0), 1.0)
    min_ball_ratio = min(max(min_ball_ratio, 0.0), 1.0)

    target_background = int(round(max_images * background_ratio))
    target_background = min(target_background, max_images)

    remaining_after_background = max_images - target_background
    target_ball = int(round(max_images * min_ball_ratio))
    target_ball = min(target_ball, remaining_after_background)

    target_other_labeled = max_images - target_background - target_ball
    return target_background, target_ball, target_other_labeled


def _take(items, count):
    if count <= 0:
        return [], items
    return items[:count], items[count:]


def stratified_sample_items(
    labels_by_image,
    background_candidates,
    max_images,
    seed=42,
    background_ratio=0.05,
    min_ball_ratio=0.35,
):
    rng = random.Random(seed)

    ball_items = []
    other_labeled_items = []

    for image_id, lines in labels_by_image.items():
        classes = {int(line.split()[0]) for line in lines}

        if 1 in classes:  # contains a ball, with or without a person
            ball_items.append((image_id, lines))
        else:  # labeled, non-ball sample such as person-only
            other_labeled_items.append((image_id, lines))

    background_items = [(img_id, []) for img_id in background_candidates]

    rng.shuffle(ball_items)
    rng.shuffle(other_labeled_items)
    rng.shuffle(background_items)

    if max_images is None:
        selected = ball_items + other_labeled_items + background_items
        rng.shuffle(selected)
        return selected

    target_background, target_ball, target_other_labeled = _target_counts(
        max_images,
        background_ratio=background_ratio,
        min_ball_ratio=min_ball_ratio,
    )

    selected_background, background_left = _take(background_items, target_background)
    selected_ball, ball_left = _take(ball_items, target_ball)
    selected_other_labeled, other_labeled_left = _take(other_labeled_items, target_other_labeled)

    selected = selected_background + selected_ball + selected_other_labeled

    # Fill shortages from remaining candidates without exceeding max_images. This
    # keeps downloading replacement images when min-det-area or class shortages
    # remove candidates from one bucket.
    shortfall = max_images - len(selected)
    if shortfall > 0:
        filler_pool = other_labeled_left + ball_left + background_left
        rng.shuffle(filler_pool)
        selected.extend(filler_pool[:shortfall])

    selected = selected[:max_images]
    rng.shuffle(selected)

    selected_background_count = sum(1 for _id, lines in selected if not lines)
    selected_ball_count = sum(1 for _id, lines in selected if any(l.split()[0] == "1" for l in lines))
    selected_other_labeled_count = len(selected) - selected_background_count - selected_ball_count

    print(
        f"[SAMPLE] requested={max_images}, selected={len(selected)}, "
        f"background={selected_background_count}/{target_background}, "
        f"ball={selected_ball_count}/{target_ball}, "
        f"other_labeled={selected_other_labeled_count}/{target_other_labeled}"
    )

    return selected

def convert_and_download_split(
    split,
    max_images=None,
    workers=8,
    background_ratio=0.05,
    seed=42,
    min_ball_ratio=0.35,
    min_det_area=0,
):
    ann_path = RAW / "annotations" / f"instances_{split}.json"

    out_img_dir = OUT / "images" / split
    out_lbl_dir = OUT / "labels" / split
    out_img_dir.mkdir(parents=True, exist_ok=True)
    out_lbl_dir.mkdir(parents=True, exist_ok=True)

    with open(ann_path, "r") as f:
        data = json.load(f)

    images = {img["id"]: img for img in data["images"]}
    labels_by_image = {}
    rejected_small_det_image_ids = set()
    min_det_area = max(0, int(min_det_area or 0))

    for ann in data["annotations"]:
        cat_id = ann["category_id"]

        if cat_id not in NEW_CLASS_MAP:
            continue

        if ann.get("iscrowd", 0):
            continue

        x, y, w, h = ann["bbox"]
        if w <= 1 or h <= 1:
            continue

        image_id = ann["image_id"]
        if min_det_area and (w * h) < min_det_area:
            rejected_small_det_image_ids.add(image_id)
            continue

        img = images[image_id]
        cls = NEW_CLASS_MAP[cat_id]
        box = coco_to_yolo_bbox(ann["bbox"], img["width"], img["height"])

        line = f"{cls} " + " ".join(f"{v:.6f}" for v in box)
        labels_by_image.setdefault(image_id, []).append(line)

    if rejected_small_det_image_ids:
        # Drop the whole image if any person/ball detection in it is too small.
        for image_id in rejected_small_det_image_ids:
            labels_by_image.pop(image_id, None)

    all_image_ids = set(images.keys())
    labeled_image_ids = set(labels_by_image.keys())
    background_candidates = list(all_image_ids - labeled_image_ids - rejected_small_det_image_ids)

    items = stratified_sample_items(
        labels_by_image,
        background_candidates,
        max_images=max_images,
        seed=seed,
        background_ratio=background_ratio,
        min_ball_ratio=min_ball_ratio,
    )

    background_count = sum(1 for _id, lines in items if not lines)
    labeled_count = len(items) - background_count
    ball_count = sum(1 for _id, lines in items if any(l.split()[0] == "1" for l in lines))
    person_count = sum(1 for _id, lines in items if any(l.split()[0] == "0" for l in lines))

    print(
        f"[{split}] Downloading/converting {len(items)} images "
        f"({labeled_count} labeled: {ball_count} with ball, {person_count} with person; "
        f"{background_count} background, rejected_small_det={len(rejected_small_det_image_ids)}, "
        f"min_det_area={min_det_area}, workers={workers})"
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


def _benchmark_inference(model, device, imgsz, eval_images, runs=20, warmup=5):
    for _ in range(warmup):
        _ = model.predict(source=eval_images, imgsz=imgsz, device=device, verbose=False)

    if device.startswith("cuda"):
        torch.cuda.synchronize()

    start = time.perf_counter()
    for _ in range(runs):
        _ = model.predict(source=eval_images, imgsz=imgsz, device=device, verbose=False)

    if device.startswith("cuda"):
        torch.cuda.synchronize()

    elapsed = time.perf_counter() - start
    return (elapsed / runs) * 1000.0


def _select_export_sizes(model, device, target_latencies_ms, eval_images, max_size=(1080, 1920)):
    candidates = [
        (256, 256),
        (320, 320),
        (384, 384),
        (448, 448),
        (512, 512),
        (640, 640),
        (768, 768),
        (896, 896),
        (1024, 1024),
        (1280, 1280),
        (1536, 1536),
        max_size,
    ]

    timings = []
    for size in candidates:
        ms = _benchmark_inference(model, device, size, eval_images)
        print(f"[BENCH] imgsz={size[0]}x{size[1]} avg={ms:.2f}ms")
        timings.append((size, ms))

    selected = {}
    for target_ms in target_latencies_ms:
        eligible = [size for size, ms in timings if ms <= target_ms]
        if eligible:
            chosen = max(eligible, key=lambda s: s[0] * s[1])
        else:
            chosen = min(candidates, key=lambda s: s[0] * s[1])

        selected[target_ms] = chosen
        if chosen == max_size:
            break

    return selected


def _load_eval_images(list_path, base_dir):
    if not list_path.exists():
        return []

    images = []
    for line in list_path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        path = Path(line)
        if not path.is_absolute():
            path = base_dir / path
        images.append(str(path))

    return images


def _ensure_eval_images(list_path, base_dir, parser):
    if not list_path.exists():
        parser.error(f"Eval list missing: {list_path}")

    images = _load_eval_images(list_path, base_dir)
    if not images:
        parser.error(f"Eval list empty: {list_path}")

    for image_path in images:
        path = Path(image_path)
        if path.exists():
            continue

        try:
            rel_path = path.relative_to(base_dir)
        except ValueError:
            parser.error(f"Eval image missing and not under {base_dir}: {path}")

        parts = rel_path.parts
        if len(parts) < 3 or parts[0] != "images":
            parser.error(f"Eval image path must be under images/<split>: {path}")

        split = parts[1]
        filename = parts[-1]
        url = f"http://images.cocodataset.org/{split}/{filename}"

        try:
            download_file(url, path, show_progress=False)
        except Exception as exc:
            parser.error(f"Failed to download eval image {filename}: {exc}")

    return images


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
    parser.add_argument("--min-det-area", type=int, default=0, help="Minimum bbox area in pixels for person/ball detections in both training and validation sets. Images containing a smaller detection are skipped.")
    parser.add_argument("--batch-size", type=float, default=24, help="Training batch size passed to YOLO.")
    args = parser.parse_args()

    eval_images = _ensure_eval_images(EVAL_LIST, OUT, parser)

    RAW.mkdir(parents=True, exist_ok=True)

    ann_zip = RAW / "annotations_trainval2017.zip"
    if not ann_zip.exists():
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
        min_det_area=args.min_det_area,
    )
    convert_and_download_split(
        "val2017",
        max_images=args.max_val,
        workers=args.download_workers,
        background_ratio=args.background_ratio,
        seed=args.seed,
        min_ball_ratio=args.min_ball_ratio,
        min_det_area=args.min_det_area,
    )

    data_yaml = write_yaml()

    use_cuda = torch.cuda.is_available()
    device = "cuda:0" if use_cuda else "cpu"

    print(f"[INFO] Training on {'CUDA' if use_cuda else 'CPU'}")

    model = YOLO(args.model)

    if args.batch_size == -1:
        batch_size = int(args.batch_size)
    elif 0 < args.batch_size < 1:
        batch_size = float(args.batch_size)
    elif args.batch_size > 1:
        batch_size = int(args.batch_size)
    else:
        parser.error("--batch-size must be -1, >0 and <1, or >1")

    results = model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=batch_size,
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

    weights_dir = Path(results.save_dir) / "weights"
    best_weights = weights_dir / "best.pt"
    if not best_weights.exists():
        best_weights = weights_dir / "last.pt"

    export_model = YOLO(best_weights)
    model_stem = Path(args.model).stem
    export_root = Path("trained_models") / model_stem
    export_root.mkdir(parents=True, exist_ok=True)

    target_latencies_ms = [20, 10, 5]
    selected_sizes = _select_export_sizes(export_model, device, target_latencies_ms, eval_images)

    exported = set()
    for target_ms in target_latencies_ms:
        if target_ms not in selected_sizes:
            continue
        size = selected_sizes[target_ms]
        if size in exported:
            continue

        export_path = export_model.export(
            format="engine",
            imgsz=size,
            device=device,
            half=use_cuda,
            optimize=True,
            simplify=True,
            int8=False,
        )

        height, width = size
        dest_path = export_root / f"{model_stem}_{width}x{height}.engine"
        shutil.move(str(export_path), dest_path)
        exported.add(size)


if __name__ == "__main__":
    main()