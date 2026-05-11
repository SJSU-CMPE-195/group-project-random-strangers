from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
import random
import shutil
import time
import traceback
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple
from zipfile import ZipFile

import requests
import torch
from tqdm import tqdm
from ultralytics import YOLO


ROOT_DIR = Path("datasets/coco_person_ball_subset")
RAW_DIR = ROOT_DIR / "raw"
YOLO_DIR = ROOT_DIR / "yolo"
EVAL_LIST_PATH = Path("eval_images.txt")

COCO_ANNOTATIONS_URL = "http://images.cocodataset.org/annotations/annotations_trainval2017.zip"
COCO_IMAGE_URL_TEMPLATE = "http://images.cocodataset.org/{split}/{filename}"

COCO_PERSON_CLASS_ID = 1
COCO_SPORTS_BALL_CLASS_ID = 37
YOLO_PERSON_CLASS_ID = 0
YOLO_BALL_CLASS_ID = 1

COCO_TO_YOLO_CLASS_ID = {
    COCO_PERSON_CLASS_ID: YOLO_PERSON_CLASS_ID,
    COCO_SPORTS_BALL_CLASS_ID: YOLO_BALL_CLASS_ID,
}

DEFAULT_LATENCY_TARGETS_MS = (20.0, 10.0, 5.0)
DEFAULT_EXPORT_SIZE_CANDIDATES = (
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
)

ImageId = int
YoloLabelLines = List[str]
SampleItem = Tuple[ImageId, YoloLabelLines]
ImageSize = Tuple[int, int]  # (height, width)


# -----------------------------------------------------------------------------
# File download and extraction helpers
# -----------------------------------------------------------------------------


def download_file_with_retries(
    url: str,
    destination: Path,
    *,
    show_progress: bool = True,
    max_retries: int = 3,
    timeout_seconds: int = 60,
) -> None:
    """Download one file with retry and atomic replacement."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = destination.with_suffix(destination.suffix + ".part")

    for attempt in range(1, max_retries + 1):
        try:
            with requests.get(url, stream=True, timeout=timeout_seconds) as response:
                response.raise_for_status()
                total_bytes = int(response.headers.get("content-length", 0))

                with open(temporary_path, "wb") as file_handle:
                    if show_progress:
                        with tqdm(total=total_bytes, unit="B", unit_scale=True, desc=destination.name) as progress:
                            for chunk in response.iter_content(1024 * 1024):
                                if chunk:
                                    file_handle.write(chunk)
                                    progress.update(len(chunk))
                    else:
                        for chunk in response.iter_content(1024 * 1024):
                            if chunk:
                                file_handle.write(chunk)

            os.replace(str(temporary_path), str(destination))
            return

        except Exception as exc:
            delete_partial_download(temporary_path)
            if attempt >= max_retries:
                print(f"[FAIL] {destination} after {attempt} attempts: {exc}")
                raise

            backoff_seconds = 2 ** (attempt - 1)
            print(
                f"[RETRY] {destination} attempt {attempt}/{max_retries}; "
                f"retrying in {backoff_seconds}s"
            )
            time.sleep(backoff_seconds)



def delete_partial_download(path: Path) -> None:
    """Best-effort cleanup for failed partial downloads."""
    if not path.exists():
        return
    try:
        path.unlink()
    except Exception:
        pass



def extract_zip_once(zip_path: Path, destination_dir: Path) -> None:
    """Extract a zip file once and mark the extraction as complete."""
    marker_path = destination_dir / f".unzipped_{zip_path.stem}"
    if marker_path.exists():
        return

    with ZipFile(zip_path, "r") as zip_file:
        zip_file.extractall(destination_dir)

    marker_path.touch()



def ensure_coco_annotations_exist() -> None:
    """Download and extract the COCO annotation archive if needed."""
    RAW_DIR.mkdir(parents=True, exist_ok=True)
    annotation_zip_path = RAW_DIR / "annotations_trainval2017.zip"

    if not annotation_zip_path.exists():
        download_file_with_retries(COCO_ANNOTATIONS_URL, annotation_zip_path)

    extract_zip_once(annotation_zip_path, RAW_DIR)


# -----------------------------------------------------------------------------
# COCO to YOLO conversion and sampling
# -----------------------------------------------------------------------------


def convert_coco_bbox_to_yolo_format(
    coco_bbox: Sequence[float],
    image_width: int,
    image_height: int,
) -> Tuple[float, float, float, float]:
    """Convert COCO xywh pixel bbox to normalized YOLO x_center y_center w h."""
    x, y, width, height = coco_bbox
    return (
        (x + width / 2) / image_width,
        (y + height / 2) / image_height,
        width / image_width,
        height / image_height,
    )



def calculate_sample_targets(
    max_images: Optional[int],
    background_ratio: float,
    min_ball_ratio: float,
) -> Tuple[Optional[int], Optional[int], Optional[int]]:
    """Calculate requested counts for background, ball, and other labeled images."""
    if max_images is None:
        return None, None, None

    max_images = max(0, int(max_images))
    background_ratio = min(max(background_ratio, 0.0), 1.0)
    min_ball_ratio = min(max(min_ball_ratio, 0.0), 1.0)

    target_background_images = min(int(round(max_images * background_ratio)), max_images)
    remaining_after_background = max_images - target_background_images

    target_ball_images = min(int(round(max_images * min_ball_ratio)), remaining_after_background)
    target_other_labeled_images = max_images - target_background_images - target_ball_images

    return target_background_images, target_ball_images, target_other_labeled_images



def take_up_to_count(items: List[SampleItem], count: int) -> Tuple[List[SampleItem], List[SampleItem]]:
    """Split a list into selected and remaining items."""
    if count <= 0:
        return [], items
    return items[:count], items[count:]



def split_labeled_images_by_ball_presence(labels_by_image: Dict[ImageId, YoloLabelLines]) -> Tuple[List[SampleItem], List[SampleItem]]:
    """Separate labeled images into ball-containing and non-ball labeled samples."""
    ball_items: List[SampleItem] = []
    other_labeled_items: List[SampleItem] = []

    for image_id, label_lines in labels_by_image.items():
        class_ids = {int(line.split()[0]) for line in label_lines}
        if YOLO_BALL_CLASS_ID in class_ids:
            ball_items.append((image_id, label_lines))
        else:
            other_labeled_items.append((image_id, label_lines))

    return ball_items, other_labeled_items



def sample_images_with_class_balance(
    labels_by_image: Dict[ImageId, YoloLabelLines],
    background_candidate_ids: Sequence[ImageId],
    *,
    max_images: Optional[int],
    seed: int,
    background_ratio: float,
    min_ball_ratio: float,
) -> List[SampleItem]:
    """Sample a split while preserving requested background and ball proportions."""
    rng = random.Random(seed)
    ball_items, other_labeled_items = split_labeled_images_by_ball_presence(labels_by_image)
    background_items: List[SampleItem] = [(image_id, []) for image_id in background_candidate_ids]

    rng.shuffle(ball_items)
    rng.shuffle(other_labeled_items)
    rng.shuffle(background_items)

    if max_images is None:
        selected_items = ball_items + other_labeled_items + background_items
        rng.shuffle(selected_items)
        return selected_items

    target_background, target_ball, target_other_labeled = calculate_sample_targets(
        max_images,
        background_ratio,
        min_ball_ratio,
    )
    assert target_background is not None
    assert target_ball is not None
    assert target_other_labeled is not None

    selected_background, remaining_background = take_up_to_count(background_items, target_background)
    selected_ball, remaining_ball = take_up_to_count(ball_items, target_ball)
    selected_other_labeled, remaining_other_labeled = take_up_to_count(other_labeled_items, target_other_labeled)

    selected_items = selected_background + selected_ball + selected_other_labeled

    shortfall = max_images - len(selected_items)
    if shortfall > 0:
        filler_pool = remaining_other_labeled + remaining_ball + remaining_background
        rng.shuffle(filler_pool)
        selected_items.extend(filler_pool[:shortfall])

    selected_items = selected_items[:max_images]
    rng.shuffle(selected_items)

    print_sample_summary(selected_items, max_images, target_background, target_ball, target_other_labeled)
    return selected_items



def print_sample_summary(
    selected_items: Sequence[SampleItem],
    requested_count: int,
    target_background: int,
    target_ball: int,
    target_other_labeled: int,
) -> None:
    """Print the final sampled-image mix."""
    selected_background_count = sum(1 for _image_id, label_lines in selected_items if not label_lines)
    selected_ball_count = sum(
        1
        for _image_id, label_lines in selected_items
        if any(line.split()[0] == str(YOLO_BALL_CLASS_ID) for line in label_lines)
    )
    selected_other_labeled_count = len(selected_items) - selected_background_count - selected_ball_count

    print(
        f"[SAMPLE] requested={requested_count}, selected={len(selected_items)}, "
        f"background={selected_background_count}/{target_background}, "
        f"ball={selected_ball_count}/{target_ball}, "
        f"other_labeled={selected_other_labeled_count}/{target_other_labeled}"
    )



def load_coco_instances_json(split: str) -> dict:
    """Load one COCO instances JSON file."""
    annotation_path = RAW_DIR / "annotations" / f"instances_{split}.json"
    with open(annotation_path, "r", encoding="utf-8") as file_handle:
        return json.load(file_handle)



def build_yolo_labels_from_coco_annotations(
    coco_data: dict,
    *,
    min_detection_area_pixels: int,
) -> Tuple[Dict[ImageId, dict], Dict[ImageId, YoloLabelLines], set[ImageId]]:
    """Create YOLO label lines for supported COCO classes."""
    images_by_id: Dict[ImageId, dict] = {image["id"]: image for image in coco_data["images"]}
    labels_by_image: Dict[ImageId, YoloLabelLines] = {}
    rejected_small_detection_image_ids: set[ImageId] = set()
    min_detection_area_pixels = max(0, int(min_detection_area_pixels or 0))

    for annotation in coco_data["annotations"]:
        category_id = annotation["category_id"]
        if category_id not in COCO_TO_YOLO_CLASS_ID:
            continue
        if annotation.get("iscrowd", 0):
            continue

        x, y, width, height = annotation["bbox"]
        if width <= 1 or height <= 1:
            continue

        image_id = annotation["image_id"]
        if min_detection_area_pixels and (width * height) < min_detection_area_pixels:
            rejected_small_detection_image_ids.add(image_id)
            continue

        image = images_by_id[image_id]
        yolo_class_id = COCO_TO_YOLO_CLASS_ID[category_id]
        yolo_bbox = convert_coco_bbox_to_yolo_format(annotation["bbox"], image["width"], image["height"])
        label_line = f"{yolo_class_id} " + " ".join(f"{value:.6f}" for value in yolo_bbox)
        labels_by_image.setdefault(image_id, []).append(label_line)

    remove_images_with_small_detections(labels_by_image, rejected_small_detection_image_ids)
    return images_by_id, labels_by_image, rejected_small_detection_image_ids



def remove_images_with_small_detections(
    labels_by_image: Dict[ImageId, YoloLabelLines],
    rejected_image_ids: Iterable[ImageId],
) -> None:
    """Drop entire images that contain a too-small person or ball detection."""
    for image_id in rejected_image_ids:
        labels_by_image.pop(image_id, None)



def create_split_output_directories(split: str) -> Tuple[Path, Path]:
    """Create image and label output directories for one split."""
    image_dir = YOLO_DIR / "images" / split
    label_dir = YOLO_DIR / "labels" / split
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)
    return image_dir, label_dir



def build_coco_image_url(image_metadata: dict, split: str) -> str:
    """Return the COCO image URL for an image metadata record."""
    return image_metadata.get("coco_url") or COCO_IMAGE_URL_TEMPLATE.format(
        split=split,
        filename=image_metadata["file_name"],
    )



def download_image_and_write_yolo_label(
    item: SampleItem,
    *,
    images_by_id: Dict[ImageId, dict],
    split: str,
    image_dir: Path,
    label_dir: Path,
) -> bool:
    """Download one image and write its YOLO label file."""
    image_id, label_lines = item

    try:
        image_metadata = images_by_id[image_id]
        file_name = image_metadata["file_name"]
        destination_image_path = image_dir / file_name
        destination_label_path = label_dir / file_name.replace(".jpg", ".txt")

        if not destination_image_path.exists():
            download_file_with_retries(
                build_coco_image_url(image_metadata, split),
                destination_image_path,
                show_progress=False,
            )

        label_text = "\n".join(label_lines) + "\n" if label_lines else ""
        destination_label_path.write_text(label_text, encoding="utf-8")
        return True

    except Exception:
        print(f"[ERROR] processing image {image_id}:\n{traceback.format_exc()}")
        return False



def print_split_conversion_summary(
    split: str,
    selected_items: Sequence[SampleItem],
    rejected_small_detection_count: int,
    min_detection_area_pixels: int,
    worker_count: int,
) -> None:
    """Print one-line summary for split conversion."""
    background_count = sum(1 for _image_id, label_lines in selected_items if not label_lines)
    labeled_count = len(selected_items) - background_count
    ball_count = sum(
        1
        for _image_id, label_lines in selected_items
        if any(line.split()[0] == str(YOLO_BALL_CLASS_ID) for line in label_lines)
    )
    person_count = sum(
        1
        for _image_id, label_lines in selected_items
        if any(line.split()[0] == str(YOLO_PERSON_CLASS_ID) for line in label_lines)
    )

    print(
        f"[{split}] Downloading/converting {len(selected_items)} images "
        f"({labeled_count} labeled: {ball_count} with ball, {person_count} with person; "
        f"{background_count} background, rejected_small_det={rejected_small_detection_count}, "
        f"min_det_area={min_detection_area_pixels}, workers={worker_count})"
    )



def download_and_convert_split_to_yolo(
    split: str,
    *,
    max_images: Optional[int],
    worker_count: int,
    background_ratio: float,
    seed: int,
    min_ball_ratio: float,
    min_detection_area_pixels: int,
) -> None:
    """Convert one COCO split into YOLO images and labels."""
    image_dir, label_dir = create_split_output_directories(split)
    coco_data = load_coco_instances_json(split)
    images_by_id, labels_by_image, rejected_small_detection_image_ids = build_yolo_labels_from_coco_annotations(
        coco_data,
        min_detection_area_pixels=min_detection_area_pixels,
    )

    all_image_ids = set(images_by_id.keys())
    labeled_image_ids = set(labels_by_image.keys())
    background_candidate_ids = list(all_image_ids - labeled_image_ids - rejected_small_detection_image_ids)

    selected_items = sample_images_with_class_balance(
        labels_by_image,
        background_candidate_ids,
        max_images=max_images,
        seed=seed,
        background_ratio=background_ratio,
        min_ball_ratio=min_ball_ratio,
    )

    print_split_conversion_summary(
        split,
        selected_items,
        len(rejected_small_detection_image_ids),
        min_detection_area_pixels,
        worker_count,
    )

    with concurrent.futures.ThreadPoolExecutor(max_workers=worker_count) as executor:
        futures = {
            executor.submit(
                download_image_and_write_yolo_label,
                item,
                images_by_id=images_by_id,
                split=split,
                image_dir=image_dir,
                label_dir=label_dir,
            ): item
            for item in selected_items
        }

        with tqdm(total=len(futures), desc=split) as progress:
            for future in concurrent.futures.as_completed(futures):
                _ = future.result()
                progress.update(1)



def prepare_yolo_dataset(args: argparse.Namespace) -> Path:
    """Ensure the dataset exists and return the generated YOLO data YAML path."""
    ensure_coco_annotations_exist()

    download_and_convert_split_to_yolo(
        "train2017",
        max_images=args.max_train,
        worker_count=args.download_workers,
        background_ratio=args.background_ratio,
        seed=args.seed,
        min_ball_ratio=args.min_ball_ratio,
        min_detection_area_pixels=args.min_det_area,
    )
    download_and_convert_split_to_yolo(
        "val2017",
        max_images=args.max_val,
        worker_count=args.download_workers,
        background_ratio=args.background_ratio,
        seed=args.seed,
        min_ball_ratio=args.min_ball_ratio,
        min_detection_area_pixels=args.min_det_area,
    )

    return write_yolo_dataset_yaml()



def write_yolo_dataset_yaml() -> Path:
    """Write the Ultralytics dataset YAML file."""
    yaml_path = YOLO_DIR / "coco_person_ball_subset.yaml"
    yaml_path.write_text(
        f"""
path: {YOLO_DIR.resolve()}
train: images/train2017
val: images/val2017

names:
  0: person
  1: ball
""".strip(),
        encoding="utf-8",
    )
    return yaml_path


# -----------------------------------------------------------------------------
# Eval image handling
# -----------------------------------------------------------------------------


def load_eval_image_paths(list_path: Path, base_dir: Path) -> List[Path]:
    """Load eval image paths from a text file."""
    if not list_path.exists():
        return []

    eval_image_paths: List[Path] = []
    for raw_line in list_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue

        image_path = Path(line)
        if not image_path.is_absolute():
            image_path = base_dir / image_path
        eval_image_paths.append(image_path)

    return eval_image_paths



def download_missing_eval_image(image_path: Path, base_dir: Path, parser: argparse.ArgumentParser) -> None:
    """Download one missing eval image from COCO if its path is under images/<split>."""
    try:
        relative_path = image_path.relative_to(base_dir)
    except ValueError:
        parser.error(f"Eval image missing and not under {base_dir}: {image_path}")

    path_parts = relative_path.parts
    if len(path_parts) < 3 or path_parts[0] != "images":
        parser.error(f"Eval image path must be under images/<split>: {image_path}")

    split = path_parts[1]
    filename = path_parts[-1]
    image_url = COCO_IMAGE_URL_TEMPLATE.format(split=split, filename=filename)

    try:
        download_file_with_retries(image_url, image_path, show_progress=False)
    except Exception as exc:
        parser.error(f"Failed to download eval image {filename}: {exc}")



def ensure_eval_images_are_available(
    list_path: Path,
    base_dir: Path,
    parser: argparse.ArgumentParser,
) -> List[Path]:
    """Validate eval-image list and download any missing COCO eval images."""
    if not list_path.exists():
        parser.error(f"Eval list missing: {list_path}")

    eval_image_paths = load_eval_image_paths(list_path, base_dir)
    if not eval_image_paths:
        parser.error(f"Eval list empty: {list_path}")

    for image_path in eval_image_paths:
        if not image_path.exists():
            download_missing_eval_image(image_path, base_dir, parser)

    return eval_image_paths



def print_latency_eval_set_summary(eval_image_paths: Sequence[Path]) -> None:
    """Print the eval-image set used for latency measurement."""
    if not eval_image_paths:
        raise ValueError("At least one eval image is required for inference-time validation.")

    print(f"[BENCH] Using {len(eval_image_paths)} eval images for latency validation.")
    for index, eval_image_path in enumerate(eval_image_paths, start=1):
        print(f"[BENCH] eval_image[{index}]={eval_image_path}")


# -----------------------------------------------------------------------------
# Training
# -----------------------------------------------------------------------------


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



def require_cuda_device(parser: argparse.ArgumentParser) -> str:
    """Return a CUDA device string or exit with a clear error."""
    if not torch.cuda.is_available():
        parser.error(
            "CUDA is required because latency validation compiles the model to CUDA "
            "and TensorRT engine export also requires CUDA."
        )
    return "cuda:0"



def train_model(args: argparse.Namespace, data_yaml: Path, device: str, parser: argparse.ArgumentParser, model: str) -> Path:
    """Train YOLO and return the best available weights path."""
    print("[INFO] Training on CUDA")
    _model = YOLO(model)
    batch_size = parse_yolo_batch_size(args.batch_size, parser)

    results = _model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=batch_size,
        device=device,
        amp=True,
        workers=args.cpu_workers,
        project="runs/person_ball",
        name=f"subset_person_ball_{Path(model).stem}",
        pretrained=True,
        patience=20,
        cache=True if args.cache == "ram" else "disk" if args.cache == "disk" else False,
        exist_ok=False,
    )

    return find_best_or_last_weights(Path(results.save_dir) / "weights")



def find_best_or_last_weights(weights_dir: Path) -> Path:
    """Prefer best.pt, then fall back to last.pt."""
    best_weights = weights_dir / "best.pt"
    if best_weights.exists():
        return best_weights

    last_weights = weights_dir / "last.pt"
    if last_weights.exists():
        return last_weights

    raise FileNotFoundError(f"No best.pt or last.pt found under {weights_dir}")


# -----------------------------------------------------------------------------
# CUDA-optimized inference-time validation
# -----------------------------------------------------------------------------


def configure_cuda_for_low_latency_inference() -> None:
    """Enable CUDA settings that favor inference throughput/latency."""
    torch.backends.cudnn.benchmark = True

    if hasattr(torch, "set_float32_matmul_precision"):
        torch.set_float32_matmul_precision("high")

    if hasattr(torch.backends, "cuda") and hasattr(torch.backends.cuda, "matmul"):
        torch.backends.cuda.matmul.allow_tf32 = True

    if hasattr(torch.backends, "cudnn"):
        torch.backends.cudnn.allow_tf32 = True



def load_cuda_compiled_yolo_model(
    weights_path: Path,
    *,
    device: str,
    use_half_precision: bool,
) -> YOLO:
    """Load YOLO weights, move them to CUDA, fuse layers, and torch.compile the model."""
    if not device.startswith("cuda"):
        raise ValueError("Latency validation must run on a CUDA device.")
    if not hasattr(torch, "compile"):
        raise RuntimeError("torch.compile is required for CUDA-compiled latency validation.")

    configure_cuda_for_low_latency_inference()

    model = YOLO(weights_path)
    try:
        model.fuse()
    except Exception as exc:
        print(f"[WARN] Layer fusion skipped: {exc}")

    model.to(device)
    model.model.eval()

    if use_half_precision:
        model.model.half()

    try:
        model.model = torch.compile(model.model, mode="max-autotune", backend="inductor")
    except Exception as exc:
        raise RuntimeError(
            "Failed to compile YOLO model for CUDA latency validation. "
            "Set --latency-compile-mode reduce-overhead if max-autotune is not supported "
            "in your environment."
        ) from exc

    print(f"[BENCH] Model compiled on {device} with torch.compile(mode='max-autotune', backend='inductor')")
    return model



def normalize_image_size(image_size: int | Sequence[int]) -> ImageSize:
    """Normalize an int or length-2 sequence into (height, width)."""
    if isinstance(image_size, int):
        return image_size, image_size
    if len(image_size) != 2:
        raise ValueError(f"Expected one int or two ints for image size, got: {image_size}")
    return int(image_size[0]), int(image_size[1])



def yolo_imgsz_argument(image_size: ImageSize) -> List[int]:
    """Return Ultralytics imgsz argument in height-width order."""
    height, width = image_size
    return [height, width]



def synchronize_cuda_if_needed(device: str) -> None:
    """Synchronize CUDA work before/after timing."""
    if device.startswith("cuda"):
        torch.cuda.synchronize()



def benchmark_one_eval_image_latency_ms(
    model: YOLO,
    *,
    eval_image_path: Path,
    image_size: ImageSize,
    device: str,
    runs: int,
    warmup_runs: int,
    use_half_precision: bool,
) -> float:
    """Return average inference time in ms for one eval image."""
    predict_kwargs = {
        "source": str(eval_image_path),
        "imgsz": yolo_imgsz_argument(image_size),
        "device": device,
        "verbose": False,
        "half": use_half_precision,
    }

    with torch.inference_mode():
        for _ in range(warmup_runs):
            _ = model.predict(**predict_kwargs)

        synchronize_cuda_if_needed(device)
        start_time = time.perf_counter()

        for _ in range(runs):
            _ = model.predict(**predict_kwargs)

        synchronize_cuda_if_needed(device)

    elapsed_seconds = time.perf_counter() - start_time
    return (elapsed_seconds / runs) * 1000.0


def benchmark_all_eval_images_average_latency_ms(
    model: YOLO,
    *,
    eval_image_paths: Sequence[Path],
    image_size: ImageSize,
    device: str,
    runs: int,
    warmup_runs: int,
    use_half_precision: bool,
) -> float:
    """Benchmark every eval image for one size and return average per-image latency."""
    if not eval_image_paths:
        raise ValueError("At least one eval image is required for latency validation.")

    per_image_latencies_ms: List[float] = []
    height, width = image_size

    for eval_image_path in eval_image_paths:
        latency_ms = benchmark_one_eval_image_latency_ms(
            model,
            eval_image_path=eval_image_path,
            image_size=image_size,
            device=device,
            runs=runs,
            warmup_runs=warmup_runs,
            use_half_precision=use_half_precision,
        )
        per_image_latencies_ms.append(latency_ms)
        print(
            f"[BENCH] image={eval_image_path.name} "
            f"imgsz={width}x{height} latency={latency_ms:.2f}ms"
        )

    average_latency_ms = sum(per_image_latencies_ms) / len(per_image_latencies_ms)
    print(
        f"[BENCH] imgsz={width}x{height} "
        f"average_latency={average_latency_ms:.2f}ms "
        f"over {len(eval_image_paths)} eval images"
    )
    return average_latency_ms



def build_export_size_candidates(max_input_size: ImageSize) -> List[ImageSize]:
    """Build sorted export-size candidates, capped at the model input size."""
    max_height, max_width = max_input_size
    candidates = [
        (height, width)
        for height, width in DEFAULT_EXPORT_SIZE_CANDIDATES
        if height <= max_height and width <= max_width
    ]

    if max_input_size not in candidates:
        candidates.append(max_input_size)

    candidates = sorted(set(candidates), key=lambda size: (size[0] * size[1], size[0], size[1]))
    return candidates



def validate_export_size_within_input_size(export_size: ImageSize, max_input_size: ImageSize) -> None:
    """Prevent exporting an engine larger than the configured model input size."""
    export_height, export_width = export_size
    max_height, max_width = max_input_size

    if export_height > max_height or export_width > max_width:
        raise ValueError(
            f"Refusing to export {export_width}x{export_height}; "
            f"input size is {max_width}x{max_height}."
        )



def select_export_sizes_that_meet_latency_targets(
    model: YOLO,
    *,
    latency_targets_ms: Sequence[float],
    eval_image_paths: Sequence[Path],
    max_input_size: ImageSize,
    device: str,
    runs: int,
    warmup_runs: int,
    use_half_precision: bool,
) -> Dict[float, ImageSize]:
    """Benchmark each size across all eval images and select sizes by average latency."""
    candidates = build_export_size_candidates(max_input_size)
    active_targets = set(float(target) for target in latency_targets_ms)
    selected_sizes: Dict[float, ImageSize] = {}

    print(
        "[BENCH] Export candidates capped by input size "
        f"{max_input_size[1]}x{max_input_size[0]}: "
        + ", ".join(f"{width}x{height}" for height, width in candidates)
    )

    for candidate_size in candidates:
        if not active_targets:
            print("[STOP] All latency targets have failed; cancelling remaining inference-time tests.")
            break

        latency_ms = benchmark_all_eval_images_average_latency_ms(
            model,
            eval_image_paths=eval_image_paths,
            image_size=candidate_size,
            device=device,
            runs=runs,
            warmup_runs=warmup_runs,
            use_half_precision=use_half_precision,
        )

        height, width = candidate_size

        for target_ms in sorted(list(active_targets)):
            if latency_ms <= target_ms:
                selected_sizes[target_ms] = candidate_size
                continue

            print(
                f"[STOP] target={target_ms:.2f}ms missed at {width}x{height}: "
                f"average={latency_ms:.2f}ms over {len(eval_image_paths)} eval images. "
                "Cancelling larger-size tests for this target."
            )
            active_targets.remove(target_ms)

    return selected_sizes


# -----------------------------------------------------------------------------
# TensorRT export
# -----------------------------------------------------------------------------


def export_selected_tensor_rt_engines(
    weights_path: Path,
    *,
    selected_sizes_by_target_ms: Dict[float, ImageSize],
    model_stem: str,
    export_root: Path,
    max_input_size: ImageSize,
    device: str,
    use_half_precision: bool,
) -> List[Path]:
    """Export one TensorRT engine per unique selected size."""
    export_root.mkdir(parents=True, exist_ok=True)
    export_model = YOLO(weights_path)
    exported_sizes: set[ImageSize] = set()
    exported_paths: List[Path] = []

    for target_ms in sorted(selected_sizes_by_target_ms.keys(), reverse=True):
        export_size = selected_sizes_by_target_ms[target_ms]
        if export_size in exported_sizes:
            continue

        validate_export_size_within_input_size(export_size, max_input_size)
        height, width = export_size

        print(f"[EXPORT] target={target_ms:.2f}ms size={width}x{height}")
        exported_path = export_model.export(
            format="engine",
            imgsz=yolo_imgsz_argument(export_size),
            device=device,
            half=use_half_precision,
            optimize=True,
            simplify=True,
            int8=False,
        )

        destination_path = export_root / f"{model_stem}_{width}x{height}.engine"
        shutil.move(str(exported_path), destination_path)
        exported_sizes.add(export_size)
        exported_paths.append(destination_path)
        print(f"[EXPORT] wrote {destination_path}")

    if not exported_paths:
        print("[EXPORT] No engines exported because no candidate size met a latency target.")

    return exported_paths


# -----------------------------------------------------------------------------
# CLI and orchestration
# -----------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", nargs='*', type=str, default=["yolo26n.pt"], help="One or more YOLO weights paths or model names to train and export. Multiple models can be specified to run in sequence. Each model will be exported at the largest size that meets the latency targets. Default: %(default)s")
    parser.add_argument("--epochs", type=int, default=100, help="Maximum number of training epochs. Actual training may stop earlier if the model converges.")
    parser.add_argument("--imgsz", type=int, default=640, help="Maximum training image size (area). This is also the upper bound for exported engine input sizes. Models are trained at this size, but may be exported at smaller sizes that meet the latency targets.")
    parser.add_argument("--max-train", type=int, default=10000, help="Maximum number of training images to download and use. The actual number of images used may be slightly lower than this limit due to the class-balanced sampling strategy.")
    parser.add_argument("--max-val", type=int, default=1000, help="Maximum number of validation images to download and use for training. This is separate from the eval images used for latency validation, which are specified via the --eval-list argument.")
    parser.add_argument("--download-workers", type=int, default=24)
    parser.add_argument("--cpu-workers", type=int, default=8)
    parser.add_argument("--cache", default="disk", choices=["disk", "ram", "false"], help="Whether to cache the dataset for training. 'disk' caches to disk, 'ram' caches in RAM, and 'false' disables caching. Caching can speed up training but requires additional storage or memory. Default: %(default)s")
    parser.add_argument("--background-ratio", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--min-ball-ratio", type=float, default=0.35)
    parser.add_argument("--min-det-area", type=int, default=0, help="Minimum bbox area in pixels for person/ball detections in both training and validation sets. Images containing a smaller detection are skipped.",)
    parser.add_argument("--batch-size", type=float, default=24, help="Training batch size passed to YOLO.")
    parser.add_argument("--latency-runs", type=int, default=2, help="Timed runs per eval image for inference latency validation.",)
    parser.add_argument("--latency-warmup-runs", type=int, default=2, help="Warmup runs per eval image before timing inference latency.",)
    parser.add_argument("--latency-targets-ms", type=float, nargs="+", default=list(DEFAULT_LATENCY_TARGETS_MS), help="Latency targets in milliseconds. Models are exported in the largest size per inference time.",)

    args = parser.parse_args()
    args.parser = parser
    return args



def run_training_validation_and_export(args: argparse.Namespace) -> List[Path]:
    """Run dataset prep, training, all-eval-image latency validation, and export."""
    parser: argparse.ArgumentParser = args.parser
    device = require_cuda_device(parser)
    eval_image_paths = ensure_eval_images_are_available(EVAL_LIST_PATH, YOLO_DIR, parser)
    print_latency_eval_set_summary(eval_image_paths)

    for model in args.model:
        data_yaml = prepare_yolo_dataset(args)
        best_weights = train_model(args, data_yaml, device, parser, model)

        model_stem = Path(model).stem
        export_root = Path("trained_models") / model_stem
        max_input_size = normalize_image_size(args.imgsz)
        use_half_precision = True

        latency_model = load_cuda_compiled_yolo_model(
            best_weights,
            device=device,
            use_half_precision=use_half_precision,
        )

        selected_sizes_by_target_ms = select_export_sizes_that_meet_latency_targets(
            latency_model,
            latency_targets_ms=args.latency_targets_ms,
            eval_image_paths=eval_image_paths,
            max_input_size=max_input_size,
            device=device,
            runs=args.latency_runs,
            warmup_runs=args.latency_warmup_runs,
            use_half_precision=use_half_precision,
        )

        return export_selected_tensor_rt_engines(
            best_weights,
            selected_sizes_by_target_ms=selected_sizes_by_target_ms,
            model_stem=model_stem,
            export_root=export_root,
            max_input_size=max_input_size,
            device=device,
            use_half_precision=use_half_precision,
        )



def main() -> None:
    args = parse_args()
    run_training_validation_and_export(args)


if __name__ == "__main__":
    main()
