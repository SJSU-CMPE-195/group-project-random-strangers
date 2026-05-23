from __future__ import annotations

import argparse
from typing import Tuple

import torch


def select_best_device(parser: argparse.ArgumentParser | None = None) -> Tuple[str, bool]:
    """Return the best available device and whether to prefer half precision."""
    if torch.cuda.is_available():
        return "cuda:0", True

    mps_backend = getattr(torch.backends, "mps", None)
    if mps_backend is not None and mps_backend.is_available():
        return "mps", False

    if parser is not None:
        print("[WARN] No GPU found; falling back to CPU.")
    return "cpu", False


def require_cuda_device(parser: argparse.ArgumentParser) -> Tuple[str, bool]:
    """Require CUDA for operations that need it (TensorRT export)."""
    if not torch.cuda.is_available():
        parser.error("CUDA is required for TensorRT export.")
    return "cuda:0", True
