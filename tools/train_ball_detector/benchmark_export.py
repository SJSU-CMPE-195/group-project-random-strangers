from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import torch
from ultralytics import YOLO

from coco_utils import DEFAULT_EXPORT_SIZE_CANDIDATES, ImageSize, YOLO_DIR
from hardware_utils import require_cuda_device, select_best_device


DEFAULT_LATENCY_TARGETS_MS = (20.0, 10.0, 5.0)


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


def configure_cuda_for_low_latency_inference() -> None:
    """Enable CUDA settings that favor inference throughput/latency."""
    torch.backends.cudnn.benchmark = True

    if hasattr(torch, "set_float32_matmul_precision"):
        torch.set_float32_matmul_precision("high")

    if hasattr(torch.backends, "cuda") and hasattr(torch.backends.cuda, "matmul"):
        torch.backends.cuda.matmul.allow_tf32 = True

    if hasattr(torch.backends, "cudnn"):
        torch.backends.cudnn.allow_tf32 = True


def synchronize_cuda_if_needed(device: str) -> None:
    """Synchronize CUDA work before/after timing."""
    if device.startswith("cuda"):
        torch.cuda.synchronize()


def load_yolo_model_for_latency(
    weights_path: Path,
    *,
    device: str,
    use_half_precision: bool,
    compile_mode: str,
) -> YOLO:
    """Load YOLO weights and optionally torch.compile when CUDA is available."""
    model = YOLO(weights_path)

    if device.startswith("cuda"):
        configure_cuda_for_low_latency_inference()
        try:
            model.fuse()
        except Exception as exc:
            print(f"[WARN] Layer fusion skipped: {exc}")

        model.to(device)
        model.model.eval()

        if use_half_precision:
            model.model.half()

        if hasattr(torch, "compile"):
            try:
                model.model = torch.compile(model.model, mode=compile_mode, backend="inductor")
                print(f"[BENCH] Model compiled on {device} with torch.compile(mode='{compile_mode}')")
            except Exception as exc:
                print(f"[WARN] torch.compile failed: {exc}")
        else:
            print("[WARN] torch.compile not available; using eager mode.")
    else:
        print(f"[BENCH] Using {device} without torch.compile.")

    return model


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
        destination_path.parent.mkdir(parents=True, exist_ok=True)
        Path(exported_path).replace(destination_path)
        exported_sizes.add(export_size)
        exported_paths.append(destination_path)
        print(f"[EXPORT] wrote {destination_path}")

    if not exported_paths:
        print("[EXPORT] No engines exported because no candidate size met a latency target.")

    return exported_paths


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", required=True)
    parser.add_argument("--eval-list", default="eval_list.txt")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--latency-targets-ms", type=float, nargs="+", default=list(DEFAULT_LATENCY_TARGETS_MS))
    parser.add_argument("--latency-runs", type=int, default=2)
    parser.add_argument("--latency-warmup-runs", type=int, default=2)
    parser.add_argument("--device", default="auto", choices=["auto", "cuda", "mps", "cpu"])
    parser.add_argument("--compile-mode", default="max-autotune")
    parser.add_argument("--export", action="store_true")
    parser.add_argument("--export-root", default="trained_models")
    parser.add_argument("--model-stem", default="")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    weights_path = Path(args.weights)
    if not weights_path.exists():
        raise FileNotFoundError(weights_path)

    eval_list_path = Path(args.eval_list)
    eval_image_paths = load_eval_image_paths(eval_list_path, YOLO_DIR)
    if not eval_image_paths:
        raise ValueError(f"Eval list empty or missing: {eval_list_path}")

    if args.device == "auto":
        device, use_half_precision = select_best_device()
    elif args.device == "cuda":
        device, use_half_precision = require_cuda_device(argparse.ArgumentParser())
    else:
        device = args.device
        use_half_precision = False

    max_input_size = normalize_image_size(args.imgsz)

    latency_model = load_yolo_model_for_latency(
        weights_path,
        device=device,
        use_half_precision=use_half_precision,
        compile_mode=args.compile_mode,
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

    if args.export:
        if not device.startswith("cuda"):
            raise RuntimeError("TensorRT export requires CUDA.")

        model_stem = args.model_stem or weights_path.stem
        export_root = Path(args.export_root) / model_stem
        exported_paths = export_selected_tensor_rt_engines(
            weights_path,
            selected_sizes_by_target_ms=selected_sizes_by_target_ms,
            model_stem=model_stem,
            export_root=export_root,
            max_input_size=max_input_size,
            device=device,
            use_half_precision=use_half_precision,
        )

        for exported_path in exported_paths:
            print(f"[EXPORT] {exported_path}")


if __name__ == "__main__":
    main()
