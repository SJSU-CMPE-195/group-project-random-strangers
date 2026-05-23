from __future__ import annotations

import concurrent.futures
import json
import os
import random
import time
import traceback
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple
from zipfile import ZipFile

import requests
from tqdm import tqdm


ROOT_DIR = Path("datasets/coco_person_ball_subset")
RAW_DIR = ROOT_DIR / "raw"
YOLO_DIR = ROOT_DIR / "yolo"
MANIFEST_DIR = ROOT_DIR / "manifests"

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


def load_coco_instances_json(split: str) -> dict:
    """Load one COCO instances JSON file."""
    annotation_path = RAW_DIR / "annotations" / f"instances_{split}.json"
    with open(annotation_path, "r", encoding="utf-8") as file_handle:
        return json.load(file_handle)


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


def split_labeled_images_by_ball_presence(
    labels_by_image: Dict[ImageId, YoloLabelLines]
) -> Tuple[List[SampleItem], List[SampleItem]]:
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


def download_image_file(
    image_metadata: dict,
    *,
    split: str,
    image_dir: Path,
) -> bool:
    """Download one image to the split directory."""
    try:
        file_name = image_metadata["file_name"]
        destination_image_path = image_dir / file_name

        if not destination_image_path.exists():
            download_file_with_retries(
                build_coco_image_url(image_metadata, split),
                destination_image_path,
                show_progress=False,
            )
        return True
    except Exception:
        print(f"[ERROR] download {image_metadata.get('id')}:\n{traceback.format_exc()}")
        return False


def download_split_images(
    split: str,
    *,
    images_by_id: Dict[ImageId, dict],
    selected_items: Sequence[SampleItem],
    worker_count: int,
) -> Path:
    """Download selected images for one split."""
    image_dir, _label_dir = create_split_output_directories(split)
    with concurrent.futures.ThreadPoolExecutor(max_workers=worker_count) as executor:
        futures = {
            executor.submit(
                download_image_file,
                images_by_id[image_id],
                split=split,
                image_dir=image_dir,
            ): image_id
            for image_id, _label_lines in selected_items
        }

        with tqdm(total=len(futures), desc=split) as progress:
            for future in concurrent.futures.as_completed(futures):
                _ = future.result()
                progress.update(1)

    return image_dir


def write_split_manifest(
    split: str,
    *,
    images_by_id: Dict[ImageId, dict],
    selected_items: Sequence[SampleItem],
    rejected_small_detection_ids: Iterable[ImageId],
) -> Path:
    """Write a manifest describing selected images and labels for a split."""
    MANIFEST_DIR.mkdir(parents=True, exist_ok=True)
    manifest_path = MANIFEST_DIR / f"{split}.json"
    payload = {
        "split": split,
        "rejected_small_detection_ids": sorted(int(image_id) for image_id in rejected_small_detection_ids),
        "items": [
            {
                "image_id": int(image_id),
                "file_name": images_by_id[image_id]["file_name"],
                "label_lines": list(label_lines),
            }
            for image_id, label_lines in selected_items
        ],
    }
    manifest_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return manifest_path


def read_split_manifest(split: str) -> dict:
    """Read a manifest describing selected images and labels for a split."""
    manifest_path = MANIFEST_DIR / f"{split}.json"
    with open(manifest_path, "r", encoding="utf-8") as file_handle:
        return json.load(file_handle)


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
