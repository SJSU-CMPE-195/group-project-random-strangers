"""Helpers for preparing ONNX models for ZED object detection."""

from __future__ import annotations

from pathlib import Path


def optimize_onnx_model(
    onnx_path: str | Path,
    output_path: str | Path | None = None,
) -> Path:
    """Simplify an ONNX model once and reuse the generated model on later runs.

    The ``onnxsim`` dependency is imported only when optimization is requested,
    so normal detector runs do not require it.
    """

    source_path = Path(onnx_path)
    if not source_path.is_file():
        raise FileNotFoundError(f"ONNX model not found: {source_path}")

    optimized_path = (
        Path(output_path)
        if output_path is not None
        else source_path.with_name(f"{source_path.stem}.optimized{source_path.suffix}")
    )
    if optimized_path.is_file():
        return optimized_path

    try:
        from onnxsim import simplify
    except ImportError as error:
        raise RuntimeError(
            "ONNX optimization requires onnxsim. Install it with: "
            "pip install onnxsim"
        ) from error

    import onnx

    optimized_path.parent.mkdir(parents=True, exist_ok=True)
    model = onnx.load(str(source_path))
    optimized_model, is_valid = simplify(model)
    if not is_valid:
        raise RuntimeError(f"onnxsim could not validate optimized model: {source_path}")
    onnx.save(optimized_model, str(optimized_path))
    return optimized_path
