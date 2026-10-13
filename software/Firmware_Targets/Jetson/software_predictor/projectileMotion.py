"""Estimate ball projectile trajectories and display them in a fast 3D view."""

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
PREDICTION_SECONDS = 1.0
PREDICTION_SAMPLES = 18
TRAIL_DURATION_SECONDS = 2.0
SCREEN_SIZE = (640, 480)
DISPLAY_FPS = 30.0

# Fixed orthographic camera used to project XYZ into the 2D OpenCV window.
# Z remains the vertical axis, matching the projectile model below.
VIEW_AZIMUTH_DEG = 42.0
VIEW_ELEVATION_DEG = 24.0
VIEW_MARGIN_PX = 48

COLORS = (
    (0, 220, 255),
    (0, 255, 120),
    (255, 160, 0),
    (255, 80, 180),
)


class Measurement(NamedTuple):
    timestamp: float
    x: float
    y: float
    z: float


TrajectoryPredictor = Callable[[float | np.ndarray], np.ndarray]


class RenderTrack(NamedTuple):
    object_id: int
    positions: np.ndarray
    predicted_positions: np.ndarray | None
    fitted_measurement_count: int


def estimate_trajectory(
    measurements: Sequence[Measurement],
) -> tuple[TrajectoryPredictor, np.ndarray, np.ndarray, np.ndarray]:
    """Fit a gravity-constrained projectile trajectory.

    This is much cheaper and numerically better behaved than fitting three
    independent polynomials against large absolute timestamps.

    X and Y are fitted as constant-velocity motion. Z is fitted as
    constant-velocity motion plus the known gravitational acceleration.
    The returned coefficient arrays are still expressed in absolute time so
    callers that relied on the old function shape continue to work.
    """

    values = np.asarray(measurements, dtype=np.float64)
    if len(values) < MIN_TRAJECTORY_MEASUREMENTS:
        raise ValueError("at least three measurements are required")
    if not np.isfinite(values).all():
        raise ValueError("measurements must contain only finite values")

    timestamps = values[:, 0]
    if np.any(np.diff(timestamps) <= 0.0):
        raise ValueError("measurement timestamps must be strictly increasing")

    positions = values[:, 1:4]
    reference_time = float(timestamps[-1])
    dt = timestamps - reference_time

    # Remove the known gravity term from Z. All three corrected coordinates
    # can then be fitted with one very small linear regression.
    corrected = positions.copy()
    corrected[:, 2] -= 0.5 * GRAVITY * dt * dt

    mean_dt = float(dt.mean())
    centered_dt = dt - mean_dt
    denominator = float(np.dot(centered_dt, centered_dt))
    if denominator <= 1e-15:
        raise ValueError("measurement timestamps are too close together")

    mean_position = corrected.mean(axis=0)
    velocity = (centered_dt[:, None] * (corrected - mean_position)).sum(axis=0) / denominator
    position_at_reference = mean_position - velocity * mean_dt

    def predict(time: float | np.ndarray) -> np.ndarray:
        time_array = np.asarray(time, dtype=np.float64)
        delta = time_array - reference_time

        result = position_at_reference + delta[..., None] * velocity
        result = np.array(result, dtype=np.float64, copy=True)
        result[..., 2] += 0.5 * GRAVITY * delta * delta
        return result

    # Convert the relative-time model back into absolute-time polynomial
    # coefficients to preserve the original function's return contract.
    x_coefficients = np.asarray(
        [velocity[0], position_at_reference[0] - velocity[0] * reference_time],
        dtype=np.float64,
    )
    y_coefficients = np.asarray(
        [velocity[1], position_at_reference[1] - velocity[1] * reference_time],
        dtype=np.float64,
    )
    z_coefficients = np.asarray(
        [
            0.5 * GRAVITY,
            velocity[2] - GRAVITY * reference_time,
            position_at_reference[2]
            - velocity[2] * reference_time
            + 0.5 * GRAVITY * reference_time * reference_time,
        ],
        dtype=np.float64,
    )

    return predict, x_coefficients, y_coefficients, z_coefficients


def _candidate_starts(count: int) -> list[int]:
    """Return a small set of recent suffixes, longest first.

    The old implementation could perform up to 28 polynomial fits for one
    object on every rendered frame. This caps the search at a few useful
    windows while still recovering quickly after an outlier, bounce, or a new
    track segment.
    """

    starts: list[int] = []
    for window_size in (count, 24, 16, 12, 8, 5, 3):
        start = max(0, count - window_size)
        if count - start >= MIN_TRAJECTORY_MEASUREMENTS and start not in starts:
            starts.append(start)
    return starts


def _select_recent_trajectory(
    measurements: Sequence[Measurement],
) -> tuple[TrajectoryPredictor, int] | None:
    """Find a recent suffix that agrees with gravity-driven projectile motion."""

    count = len(measurements)
    if count < MIN_TRAJECTORY_MEASUREMENTS:
        return None

    for start in _candidate_starts(count):
        recent_measurements = measurements[start:]
        try:
            predict, _, _, _ = estimate_trajectory(recent_measurements)
        except (ValueError, np.linalg.LinAlgError):
            continue

        values = np.asarray(recent_measurements, dtype=np.float64)
        predicted_positions = predict(values[:, 0])
        errors = predicted_positions - values[:, 1:4]
        fit_error = float(np.sqrt(np.mean(errors * errors)))

        # Because gravity is built directly into the model, a low residual is
        # already the projectile-validity check. No second acceleration fit is
        # necessary.
        if fit_error <= MAX_FIT_ERROR_M:
            return predict, len(recent_measurements)

    return None


def _view_basis() -> np.ndarray:
    """Return a 2x3 orthographic basis for the fixed 3D camera."""

    azimuth = np.deg2rad(VIEW_AZIMUTH_DEG)
    elevation = np.deg2rad(VIEW_ELEVATION_DEG)

    right = np.asarray([np.cos(azimuth), -np.sin(azimuth), 0.0])
    up = np.asarray(
        [
            -np.sin(elevation) * np.sin(azimuth),
            -np.sin(elevation) * np.cos(azimuth),
            np.cos(elevation),
        ]
    )
    return np.vstack((right, up)).astype(np.float64)


VIEW_BASIS = _view_basis()


def _make_projector(scene_points: np.ndarray) -> Callable[[np.ndarray], np.ndarray]:
    """Create a vectorized XYZ -> screen-pixel projector for this frame."""

    view_points = np.asarray(scene_points, dtype=np.float64) @ VIEW_BASIS.T
    view_min = view_points.min(axis=0)
    view_max = view_points.max(axis=0)
    view_center = 0.5 * (view_min + view_max)

    span = np.maximum(view_max - view_min, 1e-6)
    usable_width = SCREEN_SIZE[0] - 2 * VIEW_MARGIN_PX
    usable_height = SCREEN_SIZE[1] - 2 * VIEW_MARGIN_PX
    scale = min(usable_width / span[0], usable_height / span[1])

    screen_center = np.asarray([SCREEN_SIZE[0] * 0.5, SCREEN_SIZE[1] * 0.5])

    def project(points: np.ndarray) -> np.ndarray:
        xyz = np.asarray(points, dtype=np.float64)
        view = xyz @ VIEW_BASIS.T
        screen = np.empty_like(view)
        screen[..., 0] = screen_center[0] + (view[..., 0] - view_center[0]) * scale
        screen[..., 1] = screen_center[1] - (view[..., 1] - view_center[1]) * scale
        return np.rint(screen).astype(np.int32)

    return project


def _draw_floor_grid(
    canvas: np.ndarray,
    project: Callable[[np.ndarray], np.ndarray],
    scene_points: np.ndarray,
) -> None:
    """Draw a small z=0 reference grid using only a handful of OpenCV lines."""

    x_min = min(float(scene_points[:, 0].min()), 0.0)
    x_max = max(float(scene_points[:, 0].max()), 0.0)
    y_min = min(float(scene_points[:, 1].min()), 0.0)
    y_max = max(float(scene_points[:, 1].max()), 0.0)

    x_span = max(x_max - x_min, 1.0)
    y_span = max(y_max - y_min, 1.0)
    x_pad = 0.08 * x_span
    y_pad = 0.08 * y_span
    x_min -= x_pad
    x_max += x_pad
    y_min -= y_pad
    y_max += y_pad

    grid_color = (48, 48, 48)
    for x in np.linspace(x_min, x_max, 6):
        line = project(np.asarray([[x, y_min, 0.0], [x, y_max, 0.0]]))
        cv2.line(canvas, tuple(line[0]), tuple(line[1]), grid_color, 1, cv2.LINE_8)

    for y in np.linspace(y_min, y_max, 6):
        line = project(np.asarray([[x_min, y, 0.0], [x_max, y, 0.0]]))
        cv2.line(canvas, tuple(line[0]), tuple(line[1]), grid_color, 1, cv2.LINE_8)


def _draw_axes(
    canvas: np.ndarray,
    project: Callable[[np.ndarray], np.ndarray],
    axis_length: float,
) -> None:
    """Draw XYZ axes from the ZED/camera coordinate origin."""

    axes = np.asarray(
        [
            [0.0, 0.0, 0.0],
            [axis_length, 0.0, 0.0],
            [0.0, axis_length, 0.0],
            [0.0, 0.0, axis_length],
        ],
        dtype=np.float64,
    )
    screen = project(axes)
    origin = tuple(screen[0])

    axis_info = (
        (1, (100, 100, 255), "X"),
        (2, (100, 255, 100), "Y"),
        (3, (255, 180, 100), "Z"),
    )
    for index, color, label in axis_info:
        end = tuple(screen[index])
        cv2.line(canvas, origin, end, color, 2, cv2.LINE_AA)
        cv2.putText(
            canvas,
            label,
            (end[0] + 4, end[1] - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )

    cv2.circle(canvas, origin, 3, (190, 190, 190), -1, cv2.LINE_AA)


def _collect_render_tracks(
    measurements_by_id: dict[int, deque[Measurement]],
    timestamp_seconds: float,
) -> tuple[list[RenderTrack], np.ndarray]:
    """Collect active tracks and calculate display-only predictions."""

    render_tracks: list[RenderTrack] = []
    scene_parts: list[np.ndarray] = [np.zeros((1, 3), dtype=np.float64)]

    future_times = np.linspace(
        timestamp_seconds,
        timestamp_seconds + PREDICTION_SECONDS,
        PREDICTION_SAMPLES,
        dtype=np.float64,
    )

    for object_id, measurement_deque in measurements_by_id.items():
        # The deque is tiny (<= MAX_MEASUREMENTS), so one list conversion is
        # cheaper than repeatedly converting slices later.
        measurements = [
            measurement
            for measurement in measurement_deque
            if timestamp_seconds - measurement.timestamp <= TRAIL_DURATION_SECONDS
        ]
        if not measurements:
            continue

        values = np.asarray(measurements, dtype=np.float64)
        positions = values[:, 1:4]
        scene_parts.append(positions)

        predicted_positions: np.ndarray | None = None
        fitted_measurement_count = 0

        if len(measurements) >= MIN_TRAJECTORY_MEASUREMENTS:
            trajectory = _select_recent_trajectory(measurements)
            if trajectory is not None:
                predict, fitted_measurement_count = trajectory
                predicted_positions = predict(future_times)
                scene_parts.append(predicted_positions)

        render_tracks.append(
            RenderTrack(
                object_id=object_id,
                positions=positions,
                predicted_positions=predicted_positions,
                fitted_measurement_count=fitted_measurement_count,
            )
        )

    return render_tracks, np.vstack(scene_parts)


def _trajectory_screen(
    measurements_by_id: dict[int, deque[Measurement]],
    timestamp_seconds: float,
) -> np.ndarray:
    """Render measured and predicted XYZ trajectories in a lightweight 3D view."""

    canvas = np.full(
        (SCREEN_SIZE[1], SCREEN_SIZE[0], 3),
        (24, 24, 24),
        dtype=np.uint8,
    )

    render_tracks, scene_points = _collect_render_tracks(
        measurements_by_id, timestamp_seconds
    )

    if not render_tracks:
        cv2.putText(
            canvas,
            "Waiting for ball detections...",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (220, 220, 220),
            2,
            cv2.LINE_AA,
        )
        return canvas

    # Include axis endpoints in the fitted viewport so the origin and axis
    # directions stay visible even when the ball is several meters away.
    world_extent = float(np.ptp(scene_points, axis=0).max())
    distance_extent = float(np.linalg.norm(scene_points, axis=1).max())
    axis_length = max(0.5, min(3.0, max(world_extent, 0.25 * distance_extent)))
    axis_points = np.asarray(
        [
            [axis_length, 0.0, 0.0],
            [0.0, axis_length, 0.0],
            [0.0, 0.0, axis_length],
        ],
        dtype=np.float64,
    )

    project = _make_projector(np.vstack((scene_points, axis_points)))
    _draw_floor_grid(canvas, project, scene_points)
    _draw_axes(canvas, project, axis_length)

    for color_index, track in enumerate(render_tracks):
        color = COLORS[color_index % len(COLORS)]
        measured_points = project(track.positions)

        # One polyline call replaces Catmull-Rom interpolation plus a full
        # canvas copy/addWeighted operation for every individual segment.
        if len(measured_points) >= 2:
            cv2.polylines(
                canvas,
                [measured_points.reshape(-1, 1, 2)],
                False,
                color,
                2,
                cv2.LINE_AA,
            )

        current_point = tuple(measured_points[-1])
        cv2.circle(canvas, current_point, 5, color, -1, cv2.LINE_AA)

        if track.predicted_positions is not None:
            predicted_points = project(track.predicted_positions)
            # Dashed predicted path, still only a handful of cheap line calls.
            for index in range(0, len(predicted_points) - 1, 2):
                cv2.line(
                    canvas,
                    tuple(predicted_points[index]),
                    tuple(predicted_points[index + 1]),
                    color,
                    1,
                    cv2.LINE_AA,
                )

        if track.fitted_measurement_count > 0:
            label = f"id={track.object_id} fit={track.fitted_measurement_count}"
        else:
            label = f"id={track.object_id} ({len(track.positions)}/3)"

        cv2.putText(
            canvas,
            label,
            (current_point[0] + 8, current_point[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )

    cv2.putText(
        canvas,
        "3D view: Z up",
        (12, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (190, 190, 190),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        f"frame time: {timestamp_seconds:.3f}s",
        (SCREEN_SIZE[0] - 190, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )
    return canvas


def _prune_stale_tracks(
    measurements_by_id: dict[int, deque[Measurement]],
    timestamp_seconds: float,
) -> None:
    """Remove IDs that have not produced a measurement within the trail window."""

    stale_ids = [
        object_id
        for object_id, measurements in measurements_by_id.items()
        if not measurements
        or timestamp_seconds - measurements[-1].timestamp > TRAIL_DURATION_SECONDS
    ]
    for object_id in stale_ids:
        del measurements_by_id[object_id]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Display ZED ball detections and projectile trajectories in 3D."
    )
    parser.add_argument(
        "--onnx",
        type=Path,
        required=True,
        help="Path to the custom YOLO ONNX model.",
    )
    parser.add_argument(
        "--svo",
        help="Optional SVO file to use instead of the live ZED camera.",
    )
    parser.add_argument(
        "--display-fps",
        type=float,
        default=DISPLAY_FPS,
        help=(
            "Maximum OpenCV display refresh rate. Tracking still consumes every "
            "camera frame. Default: 30."
        ),
    )
    return parser.parse_args()


def main() -> None:
    """Run live projectile tracking while rendering the UI at a lower rate.

    The camera/tracking loop is allowed to run at the full ZED rate. Expensive
    visualization and display-only trajectory calculations happen only at the
    requested display refresh rate, so a 100 Hz camera is not forced to render
    100 OpenCV windows per second.
    """

    args = parse_args()
    if args.display_fps <= 0.0:
        raise ValueError("--display-fps must be greater than 0")

    measurements_by_id: dict[int, deque[Measurement]] = defaultdict(
        lambda: deque(maxlen=MAX_MEASUREMENTS)
    )

    display_period = 1.0 / args.display_fps
    next_display_timestamp: float | None = None

    with ZedHandler(args.onnx, svo_path=args.svo) as zed:
        while True:
            if not zed.grab():
                continue

            objects, timestamp_ns = zed.get_objects()
            timestamp_seconds = timestamp_ns / 1_000_000_000.0

            for obj in objects:
                if obj.raw_label != BALL_CLASS_ID:
                    continue
                measurements_by_id[obj.object_id].append(
                    Measurement(timestamp_seconds, *obj.position)
                )

            if (
                next_display_timestamp is None
                or timestamp_seconds >= next_display_timestamp
            ):
                _prune_stale_tracks(measurements_by_id, timestamp_seconds)

                screen = _trajectory_screen(
                    measurements_by_id,
                    timestamp_seconds,
                )
                cv2.imshow("Projectile trajectories - 3D", screen)

                # Keep the next deadline tied to the current camera timestamp.
                # This prevents display work from trying to "catch up" by
                # rendering several frames back-to-back after a slow frame.
                next_display_timestamp = timestamp_seconds + display_period

                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
