"""Estimate ball projectile trajectories."""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import NamedTuple

import cv2
import numpy as np


from ZedHandler import BALL_CLASS_ID, ZedHandler


GRAVITY = -9.80665
MAX_MEASUREMENTS = 30
MIN_TRAJECTORY_MEASUREMENTS = 3
MAX_FIT_ERROR_M = 0.15
GRAVITY_TOLERANCE = 5.0
PREDICTION_SECONDS = 1.0
TRAIL_DURATION_SECONDS = 2.0
TRAIL_SAMPLES_PER_SEGMENT = 8
SCREEN_SIZE = (640, 480)

class Measurement(NamedTuple):
    timestamp: float
    x: float
    y: float
    z: float

TrajectoryPredictor = Callable[[float | np.ndarray], np.ndarray]


def estimate_trajectory(measurements: Sequence[Measurement]) -> tuple[TrajectoryPredictor, np.ndarray, np.ndarray, np.ndarray]:
    """Fit projectile motion to timestamped 3D positions.

    Measurements are fitted against their absolute timestamps in seconds.
    The returned predictor also accepts absolute timestamps.
    """

    values = np.asarray(measurements, dtype=float)
    if len(values) < 3:
        raise ValueError("at least three measurements are required")
    if not np.isfinite(values).all():
        raise ValueError("measurements must contain only finite values")

    timestamps = values[:, 0]
    if np.any(np.diff(timestamps) <= 0):
        raise ValueError("measurement timestamps must be strictly increasing")

    positions = values[:, 1:4]

    x_coefficients = np.polyfit(timestamps, positions[:, 0], 1)
    y_coefficients = np.polyfit(timestamps, positions[:, 1], 1)
    z_coefficients = np.polyfit(timestamps, positions[:, 2], 2)

    def predict(time: float | np.ndarray) -> np.ndarray:
        """Predict one or more positions at absolute time in seconds."""

        time_array = np.asarray(time, dtype=float)
        x = np.polyval(x_coefficients, time_array)
        y = np.polyval(y_coefficients, time_array)
        z = np.polyval(z_coefficients, time_array)
        return np.stack((x, y, z), axis=-1)

    return predict, x_coefficients, y_coefficients, z_coefficients

def _select_recent_trajectory(measurements: Sequence[Measurement]) -> tuple[TrajectoryPredictor, int] | None:
    """Find the longest recent suffix that looks like a valid projectile."""

    if len(measurements) < MIN_TRAJECTORY_MEASUREMENTS:
        return None

    for start in range(len(measurements) - MIN_TRAJECTORY_MEASUREMENTS + 1):
        recent_measurements = measurements[start:]
        try:
            predict, _, _, z_coefficients = estimate_trajectory(recent_measurements)
        except (ValueError, np.linalg.LinAlgError):
            continue

        values = np.asarray(recent_measurements, dtype=float)
        predicted_positions = predict(values[:, 0])
        position_errors = predicted_positions - values[:, 1:4]
        fit_error = float(np.sqrt(np.mean(position_errors**2)))
        vertical_acceleration = float(2.0 * z_coefficients[0])

        if fit_error <= MAX_FIT_ERROR_M and abs(vertical_acceleration - GRAVITY) <= GRAVITY_TOLERANCE:
            return predict, len(recent_measurements)

    return None


def _smooth_trail(measurements: Sequence[Measurement], timestamp_seconds: float, to_screen: Callable[[np.ndarray], tuple[int, int]]) -> tuple[np.ndarray, np.ndarray]:
    """Return a smoothly interpolated screen trail and its point ages."""

    screen_points = np.asarray([to_screen(np.asarray(measurement[1:], dtype=float)) for measurement in measurements], dtype=float)
    ages = np.asarray([timestamp_seconds - measurement.timestamp for measurement in measurements], dtype=float)
    if len(screen_points) < 3:
        return screen_points.astype(np.int32), ages

    padded_points = np.concatenate((screen_points[:1], screen_points, screen_points[-1:]), axis=0)
    padded_ages = np.concatenate((ages[:1], ages, ages[-1:]))
    smooth_points: list[np.ndarray] = []
    smooth_ages: list[float] = []
    for index in range(1, len(padded_points) - 2):
        p0, p1, p2, p3 = padded_points[index - 1 : index + 3]
        age1, age2 = padded_ages[index], padded_ages[index + 1]
        for sample in range(TRAIL_SAMPLES_PER_SEGMENT):
            t = sample / TRAIL_SAMPLES_PER_SEGMENT
            point = 0.5 * (2 * p1 + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t**2 + (-p0 + 3 * p1 - 3 * p2 + p3) * t**3)
            smooth_points.append(point)
            smooth_ages.append(float(age1 + (age2 - age1) * t))

    smooth_points.append(screen_points[-1])
    smooth_ages.append(float(ages[-1]))
    return np.asarray(smooth_points, dtype=np.int32), np.asarray(smooth_ages)


def _trajectory_screen(measurements_by_id: dict[int, deque[Measurement]], timestamp_seconds: float) -> np.ndarray:
    """Render measured and predicted x/z paths onto a compact OpenCV canvas."""

    canvas = np.zeros((SCREEN_SIZE[1], SCREEN_SIZE[0], 3), dtype=np.uint8)
    canvas[:] = (24, 24, 24)

    active_measurements_by_id = {object_id: [measurement for measurement in measurements if timestamp_seconds - measurement.timestamp <= TRAIL_DURATION_SECONDS] for object_id, measurements in measurements_by_id.items()}
    all_positions = [np.asarray(measurement[1:], dtype=float) for measurements in active_measurements_by_id.values() for measurement in measurements]
    if not all_positions:
        cv2.putText(canvas, "Waiting for ball detections...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 220, 220), 2, cv2.LINE_AA)
        return canvas

    positions = np.asarray(all_positions)
    x_min, x_max = positions[:, 0].min(), positions[:, 0].max()
    z_min, z_max = positions[:, 2].min(), positions[:, 2].max()
    x_span = max(x_max - x_min, 1.0)
    z_span = max(z_max - z_min, 1.0)
    x_min -= 0.15 * x_span
    x_max += 0.15 * x_span
    z_min -= 0.15 * z_span
    z_max += 0.15 * z_span

    margin = 45

    def to_screen(position: np.ndarray) -> tuple[int, int]:
        x = margin + (position[0] - x_min) / (x_max - x_min) * (SCREEN_SIZE[0] - 2 * margin)
        y = SCREEN_SIZE[1] - margin - (position[2] - z_min) / (z_max - z_min) * (SCREEN_SIZE[1] - 2 * margin)
        return int(x), int(y)

    cv2.line(canvas, (margin, SCREEN_SIZE[1] - margin), (SCREEN_SIZE[0] - margin, SCREEN_SIZE[1] - margin), (100, 100, 100), 1)
    cv2.line(canvas, (margin, margin), (margin, SCREEN_SIZE[1] - margin), (100, 100, 100), 1)
    cv2.putText(canvas, "x (m)", (SCREEN_SIZE[0] - 80, SCREEN_SIZE[1] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    cv2.putText(canvas, "z (m)", (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

    colors = [(0, 220, 255), (0, 255, 120), (255, 160, 0), (255, 80, 180)]
    for color_index, (object_id, measurements) in enumerate(active_measurements_by_id.items()):
        if not measurements:
            continue
        color = colors[color_index % len(colors)]
        trail_points, trail_ages = _smooth_trail(measurements, timestamp_seconds, to_screen)
        for index in range(len(trail_points) - 1):
            age = max(trail_ages[index], trail_ages[index + 1])
            alpha = max(0.05, 1.0 - age / TRAIL_DURATION_SECONDS)
            overlay = canvas.copy()
            cv2.line(overlay, tuple(trail_points[index]), tuple(trail_points[index + 1]), color, 2, cv2.LINE_AA )
            cv2.addWeighted(overlay, alpha, canvas, 1.0 - alpha, 0.0, canvas)
        measured_point = tuple(trail_points[-1])
        cv2.circle(canvas, measured_point, 5, color, -1)

        if len(measurements) < 3:
            label = f"id={object_id} ({len(measurements)}/3)"
            cv2.putText(canvas, label, (measured_point[0] + 8, measured_point[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
            continue

        trajectory = _select_recent_trajectory(list(measurements))
        if trajectory is None:
            continue
        predict, fitted_measurement_count = trajectory

        future_times = np.linspace(timestamp_seconds, timestamp_seconds + PREDICTION_SECONDS, 30 )
        predicted_points = [to_screen(position) for position in predict(future_times)]
        for index in range(0, len(predicted_points) - 1, 2):
            cv2.line(canvas, predicted_points[index], predicted_points[index + 1], color, 1, cv2.LINE_AA )
        cv2.putText(canvas, f"id={object_id} fit={fitted_measurement_count}", (measured_point[0] + 8, measured_point[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    cv2.putText(canvas, f"frame time: {timestamp_seconds:.3f}s", (SCREEN_SIZE[0] - 190, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    return canvas


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Display ZED ball detections and projectile trajectories.")
    parser.add_argument("--onnx", type=Path, required=True, help="Path to the custom YOLO ONNX model.")
    parser.add_argument("--svo", help="Optional SVO file to use instead of the live ZED camera.",)
    return parser.parse_args()


def main() -> None:
    """Run live projectile tracking and display measured trajectories.

    Arguments:
        --onnx: Path to the custom YOLO ONNX model used by ``ZedHandler``.
            This argument is required.
        --svo: Optional path to an SVO recording. When omitted, the live ZED
            camera is used.

    Examples:
        Run with a live ZED camera:

            python projectileMotion.py --onnx best.onnx

        Run against an SVO recording:

            python projectileMotion.py --onnx best.onnx --svo recording.svo

        Press ``q`` or ``Esc`` in the trajectory window to stop the program.
    """
    args = parse_args()
    measurements_by_id: dict[int, deque[Measurement]] = defaultdict(lambda: deque(maxlen=MAX_MEASUREMENTS))

    with ZedHandler(args.onnx, svo_path=args.svo) as zed:
        while True:
            if not zed.grab():
                continue

            objects, timestamp_ns = zed.get_objects()
            timestamp_seconds = timestamp_ns / 1_000_000_000
            for obj in objects:
                if obj.raw_label != BALL_CLASS_ID:
                    continue
                measurements_by_id[obj.object_id].append(Measurement(timestamp_seconds, *obj.position))

            screen = _trajectory_screen(measurements_by_id, timestamp_seconds)
            cv2.imshow("Projectile trajectories", screen)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
