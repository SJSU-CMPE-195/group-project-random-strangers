"""Fast fixed-gravity projectile fitting

The reusable trajectory-fitting interface is kept at the top of this file.

ZED coordinate system:
    RIGHT_HANDED_Y_UP
    +X = right
    +Y = up
    -Z = forward

Gravity is fixed at [0, -9.80665, 0] m/s^2 and is never estimated from measurements.
"""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

from ZedHandler import ZedCameraIntrinsics, ZedObject2D, ZedObject3D


# =============================================================================
# PUBLIC TRAJECTORY API
# =============================================================================

#standard gravitational acceleration in meters per second squared
GRAVITY_MPS2 = 9.80665

#gravity points straight down because the zed coordinate system uses +Y as up
GRAVITY_VECTOR = np.array([0.0, -GRAVITY_MPS2, 0.0], dtype=np.float64)


@dataclass(frozen=True, slots=True)
class TrajectoryFit:
    """Fixed-gravity ballistic state expressed at reference_time."""

    #absolute timestamp where position and velocity are defined
    reference_time: float

    #estimated xyz position at reference_time
    position: np.ndarray

    #estimated xyz velocity at reference_time
    velocity: np.ndarray

    #root-mean-square 3d fit error across the measurements
    rms_error_m: float

    #number of measurements used to create this fit
    measurement_count: int

    def predict(self, times: float | Sequence[float] | np.ndarray) -> np.ndarray:
        """Predict XYZ position at one or more absolute timestamps."""

        #convert scalars or arrays into a common numpy representation
        time_array = np.asarray(times, dtype=np.float64)

        #work relative to the fit reference time to avoid large absolute timestamp math
        dt = time_array - self.reference_time

        #p = p0 + v*dt + 1/2*g*dt^2
        return self.position + dt[..., None] * self.velocity + 0.5 * (dt * dt)[..., None] * GRAVITY_VECTOR


def fit_trajectory(timestamps: Sequence[float] | np.ndarray, positions_xyz: Sequence[Sequence[float]] | np.ndarray) -> TrajectoryFit:
    """Fit a 3D ballistic trajectory while treating gravity as perfectly known."""

    #timestamps are absolute seconds from any monotonic clock
    times = np.asarray(timestamps, dtype=np.float64).reshape(-1)
    #positions must be an N by 3 array of xyz camera coordinates
    positions = np.asarray(positions_xyz, dtype=np.float64)

    #reject malformed inputs before doing any linear algebra
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError("positions_xyz must have shape (N, 3)")
    if len(times) != len(positions):
        raise ValueError("timestamps and positions_xyz must have the same length")
    if len(times) < 2:
        raise ValueError("at least two measurements are required")
    if not np.isfinite(times).all() or not np.isfinite(positions).all():
        raise ValueError("measurements must contain only finite values")
    if np.any(np.diff(times) <= 0.0):
        raise ValueError("timestamps must be strictly increasing")

    #define the fitted state at the newest measurement time
    reference_time = float(times[-1])

    #relative time keeps the regression numerically stable even when camera timestamps are large
    dt = times - reference_time

    #subtract known gravitational displacement so the remaining motion is linear in time
    corrected_positions = positions - 0.5 * (dt * dt)[:, None] * GRAVITY_VECTOR

    #center time before solving the linear regression
    mean_dt = float(dt.mean())
    centered_dt = dt - mean_dt
    denominator = float(centered_dt @ centered_dt)

    if denominator <= 1e-15:
        raise ValueError("timestamps are too close together")

    #solve all three axes at once because each corrected axis is position + velocity*time
    mean_position = corrected_positions.mean(axis=0)
    velocity = (centered_dt[:, None] * (corrected_positions - mean_position)).sum(axis=0) / denominator
    position = mean_position - velocity * mean_dt

    #measure fit quality against the original unmodified measurements
    predicted_positions = position + dt[:, None] * velocity + 0.5 * (dt * dt)[:, None] * GRAVITY_VECTOR
    residuals = predicted_positions - positions
    rms_error_m = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))

    return TrajectoryFit(reference_time=reference_time, position=position, velocity=velocity, rms_error_m=rms_error_m, measurement_count=len(times))


# =============================================================================
# LIVE TRAJECTORY TRACKING
# =============================================================================

#maximum stored depth samples for one ball
MAX_MEASUREMENTS = 30

#minimum samples before attempting a trajectory fit
MIN_FIT_MEASUREMENTS = 4

#reject fits whose average 3d position error is too large
MAX_FIT_ERROR_M = 0.15

#fallback window sizes used if older samples make the full fit bad
FIT_WINDOWS = (30, 20, 12, 8, 4)

#remove trajectory state when a stable object id has not appeared for this long
DEFAULT_STALE_TRACK_SECONDS = 0.30

#debug display can run slower than the 100 hz detection path
DEFAULT_DISPLAY_FPS = 30.0

#only this much measured history is drawn on screen
TRAIL_DURATION_SECONDS = 1.25

#how far into the future the dashed prediction is drawn
PREDICTION_SECONDS = 1.0

#number of 3d future samples used to draw the prediction
PREDICTION_SAMPLES = 24

#positions closer than this to the camera plane are unsafe to project
MIN_CAMERA_DEPTH_M = 0.05

#reject obviously invalid depth positions beyond this distance
MAX_POSITION_DISTANCE_M = 100.0

#different stable ids cycle through these bgra colors
TRACK_COLORS = ((0, 220, 255, 255), (0, 255, 120, 255), (255, 160, 0, 255), (255, 80, 180, 255))


@dataclass(frozen=True, slots=True)
class BallMeasurement:
    """One valid 3D position sample used by the ballistic fitter."""

    #absolute capture time in seconds
    timestamp: float

    #xyz position in zed RIGHT_HANDED_Y_UP camera coordinates
    position: tuple[float, float, float]


@dataclass(slots=True)
class BallTrack:
    """Trajectory history and current fit for one stable ZedHandler object id."""

    #recent valid 3d samples, automatically capped at MAX_MEASUREMENTS
    measurements: deque[BallMeasurement] = field(default_factory=lambda: deque(maxlen=MAX_MEASUREMENTS))

    #updated whenever the id appears, even if depth is invalid on that frame
    last_seen_timestamp: float = float("-inf")

    #most recent accepted ballistic fit, or None until enough good data exists
    fit: TrajectoryFit | None = None


class BallTracker:
    """Maintain one ballistic trajectory history for each stable ZedHandler object id."""

    def __init__(self, stale_after_seconds: float = DEFAULT_STALE_TRACK_SECONDS) -> None:
        if stale_after_seconds <= 0.0:
            raise ValueError("stale_after_seconds must be positive")

        #time an id may disappear before its trajectory state is deleted
        self.stale_after_seconds = stale_after_seconds
        
        #stable object id to its measurement history and current fit
        self.tracks: dict[int, BallTrack] = {}

    def update(self, objects: Sequence[ZedObject3D], timestamp_seconds: float) -> None:
        """Update all ball histories from one ZED object result."""

        for obj in objects:
            #negative ids are not useful for persistent trajectory state
            if obj.object_id < 0:
                continue

            #create state the first time a stable object id appears
            track = self.tracks.get(obj.object_id)
            if track is None:
                track = BallTrack()
                self.tracks[obj.object_id] = track

            #keep track alive whenever the detector reports the id, even if depth is bad
            track.last_seen_timestamp = timestamp_seconds

            #only finite physically plausible xyz samples are allowed into the fit
            position = _validate_position(obj.position)
            if position is None:
                continue

            if track.measurements:
                previous_time = track.measurements[-1].timestamp

                #ignore an accidental duplicate sample from the same captured frame
                if timestamp_seconds == previous_time:
                    continue

                #camera time going backwards means the old history can no longer be fit safely
                if timestamp_seconds < previous_time:
                    track.measurements.clear()
                    track.fit = None

            #store this valid point and immediately refresh the trajectory fit
            track.measurements.append(BallMeasurement(timestamp=timestamp_seconds, position=position))
            track.fit = _fit_recent_measurements(track.measurements)

        #remove tracks that have genuinely stopped receiving detector updates
        self.prune(timestamp_seconds)

    def prune(self, timestamp_seconds: float) -> None:
        """Remove stable ids that have not been reported recently."""

        #collect first so the dictionary is not modified while it is being iterated
        stale_ids = [object_id for object_id, track in self.tracks.items() if timestamp_seconds - track.last_seen_timestamp > self.stale_after_seconds]

        for object_id in stale_ids:
            del self.tracks[object_id]


def _validate_position(position_xyz: Sequence[float]) -> tuple[float, float, float] | None:
    """Return a usable camera-space XYZ point or None if the depth measurement is bad."""

    if len(position_xyz) != 3:
        return None

    x = float(position_xyz[0])
    y = float(position_xyz[1])
    z = float(position_xyz[2])

    #nan or infinity usually means the zed depth result was invalid for this frame
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        return None

    #reject impossible outliers before they can ruin the trajectory fit
    if x * x + y * y + z * z > MAX_POSITION_DISTANCE_M**2:
        return None

    #RIGHT_HANDED_Y_UP uses negative z in front of the camera
    if -z <= MIN_CAMERA_DEPTH_M:
        return None

    return x, y, z


def _fit_recent_measurements(measurements: Sequence[BallMeasurement]) -> TrajectoryFit | None:
    """Fit the longest recent sample window that still looks ballistic."""

    #convert once so recent windows can use normal slicing
    samples = tuple(measurements)
    sample_count = len(samples)

    if sample_count < MIN_FIT_MEASUREMENTS:
        return None

    #try all available measurements first, then progressively shorter recent windows
    window_sizes = [sample_count]
    window_sizes.extend(size for size in FIT_WINDOWS if MIN_FIT_MEASUREMENTS <= size < sample_count)

    for window_size in window_sizes:
        window = samples[-window_size:]
        times = np.fromiter((sample.timestamp for sample in window), dtype=np.float64, count=window_size)
        positions = np.asarray([sample.position for sample in window], dtype=np.float64)

        try:
            fit = fit_trajectory(times, positions)
        except ValueError:
            continue

        #accept the longest recent window whose measurements follow the fixed-gravity model closely enough
        if fit.rms_error_m <= MAX_FIT_ERROR_M:
            return fit

    return None


# =============================================================================
# CAMERA OVERLAY
# =============================================================================


def _project_camera_points(points_xyz: np.ndarray, intrinsics: ZedCameraIntrinsics, frame_shape: tuple[int, ...]) -> tuple[np.ndarray, np.ndarray]:
    """Project RIGHT_HANDED_Y_UP 3D camera points onto the rectified left image."""

    #force an N by 3 array so scalar and vector callers use the same projection path
    points = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    frame_height, frame_width = frame_shape[:2]

    #scale calibration if the returned debug image size differs from the calibration resolution
    scale_x = frame_width / intrinsics.width
    scale_y = frame_height / intrinsics.height
    fx = intrinsics.fx * scale_x
    fy = intrinsics.fy * scale_y
    cx = intrinsics.cx * scale_x
    cy = intrinsics.cy * scale_y

    #zed +X points right, +Y points up, and -Z points forward
    x = points[:, 0]
    image_y = -points[:, 1]
    depth = -points[:, 2]

    #only points in front of the camera can be projected safely
    valid = np.isfinite(points).all(axis=1) & (depth > MIN_CAMERA_DEPTH_M)
    pixels = np.full((len(points), 2), np.nan, dtype=np.float64)
    pixels[valid, 0] = fx * x[valid] / depth[valid] + cx
    pixels[valid, 1] = fy * image_y[valid] / depth[valid] + cy
    return pixels, valid


def _draw_segment(frame: np.ndarray, start: np.ndarray, end: np.ndarray, color: tuple[int, int, int, int], thickness: int) -> None:
    """Draw one projected line segment after clipping it to the image bounds."""

    import cv2

    if not np.isfinite(start).all() or not np.isfinite(end).all():
        return

    height, width = frame.shape[:2]
    start_point = (int(round(float(start[0]))), int(round(float(start[1]))))
    end_point = (int(round(float(end[0]))), int(round(float(end[1]))))
    visible, clipped_start, clipped_end = cv2.clipLine((0, 0, width, height), start_point, end_point)

    if visible:
        cv2.line(frame, clipped_start, clipped_end, color, thickness, cv2.LINE_AA)


def _draw_detection_box(frame: np.ndarray, box: ZedObject2D, ball: ZedObject3D | None, color: tuple[int, int, int, int]) -> None:
    """Draw one debug detector box and its current confidence."""

    import cv2

    #boxes come directly from the zed sdk only on debug display frames
    points = np.asarray(box.bounding_box_2d, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 2 or points.shape[1] < 2:
        return
    if not np.isfinite(points[:, :2]).all():
        return

    polygon = np.rint(points[:, :2]).astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(frame, [polygon], True, color, 1, cv2.LINE_AA)

    #place the label immediately above the box when possible
    x = int(points[:, 0].min())
    y = int(points[:, 1].min())
    label = f"ball {box.object_id}" if ball is None else f"ball {ball.object_id}  {ball.confidence:.2f}"
    cv2.putText(frame, label, (max(4, x), max(18, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)


def _draw_measured_trail(frame: np.ndarray, measurements: Sequence[BallMeasurement], intrinsics: ZedCameraIntrinsics, color: tuple[int, int, int, int]) -> tuple[int, int] | None:
    """Project and draw the measured 3D ball history onto the camera image."""

    import cv2

    if not measurements:
        return None

    #project every recent valid 3d measurement into image pixels in one vectorized operation
    positions = np.asarray([measurement.position for measurement in measurements], dtype=np.float64)
    pixels, valid = _project_camera_points(positions, intrinsics, frame.shape)
    valid_pixels = pixels[valid]

    if len(valid_pixels) == 0:
        return None

    #connect only neighboring samples that both project in front of the camera
    for index in range(len(pixels) - 1):
        if valid[index] and valid[index + 1]:
            _draw_segment(frame, pixels[index], pixels[index + 1], color, 2)

    #mark the newest visible measured position with a filled circle
    latest = valid_pixels[-1]
    latest_point = (int(round(float(latest[0]))), int(round(float(latest[1]))))
    if 0 <= latest_point[0] < frame.shape[1] and 0 <= latest_point[1] < frame.shape[0]:
        cv2.circle(frame, latest_point, 4, color, -1, cv2.LINE_AA)

    return latest_point


def _draw_prediction(frame: np.ndarray, fit: TrajectoryFit, timestamp_seconds: float, intrinsics: ZedCameraIntrinsics, color: tuple[int, int, int, int]) -> None:
    """Draw the fitted future 3D trajectory as dashed image-space segments."""

    #prediction begins at now, but never before the fit's reference state
    start_time = max(timestamp_seconds, fit.reference_time)
    future_times = np.linspace(start_time, start_time + PREDICTION_SECONDS, PREDICTION_SAMPLES, dtype=np.float64)
    pixels, valid = _project_camera_points(fit.predict(future_times), intrinsics, frame.shape)

    #skip every other segment to create a cheap dashed line without alpha blending
    for index in range(0, len(pixels) - 1, 2):
        if valid[index] and valid[index + 1]:
            _draw_segment(frame, pixels[index], pixels[index + 1], color, 2)


def draw_tracking_overlay(frame: np.ndarray, tracker: BallTracker, current_objects: Sequence[ZedObject3D], boxes_2d: Sequence[ZedObject2D], timestamp_seconds: float, intrinsics: ZedCameraIntrinsics) -> None:
    """Overlay current detector boxes, measured paths, and fitted future paths."""

    import cv2

    #lookup lets each debug box get confidence without searching the object list repeatedly
    objects_by_id = {obj.object_id: obj for obj in current_objects}

    #draw current detector boxes first so trajectory lines remain visible over them
    for box in boxes_2d:
        color = TRACK_COLORS[box.object_id % len(TRACK_COLORS)]
        _draw_detection_box(frame, box, objects_by_id.get(box.object_id), color)

    for object_id, track in tracker.tracks.items():
        #only draw recent history even though the fitter may retain a longer window
        recent_measurements = [measurement for measurement in track.measurements if timestamp_seconds - measurement.timestamp <= TRAIL_DURATION_SECONDS]
        if not recent_measurements:
            continue

        color = TRACK_COLORS[object_id % len(TRACK_COLORS)]
        latest_pixel = _draw_measured_trail(frame, recent_measurements, intrinsics, color)

        #prediction appears only after enough measurements form an acceptable ballistic fit
        if track.fit is not None:
            _draw_prediction(frame, track.fit, timestamp_seconds, intrinsics, color)

        if latest_pixel is None:
            continue

        x, y = latest_pixel
        if not (0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]):
            continue

        #show sample count while fitting, then show fit quality and estimated speed
        if track.fit is None:
            text = f"{len(track.measurements)}/{MIN_FIT_MEASUREMENTS} samples"
        else:
            speed = float(np.linalg.norm(track.fit.velocity))
            text = f"fit={track.fit.measurement_count}  err={track.fit.rms_error_m:.3f}m  v={speed:.1f}m/s"

        text_position = (max(4, min(frame.shape[1] - 260, x + 7)), min(frame.shape[0] - 6, y + 16))
        cv2.putText(frame, text, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.40, color, 1, cv2.LINE_AA)

    #small status line confirms how many stable tracks exist and which gravity model is active
    cv2.putText(frame, f"tracks={len(tracker.tracks)}  g=-Y {GRAVITY_MPS2:.4f}m/s^2", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (240, 240, 240, 255), 1, cv2.LINE_AA)


# =============================================================================
# MAIN PROGRAM
# =============================================================================


def parse_args():
    """Parse command-line settings used only by the executable camera program."""

    import argparse
    from pathlib import Path

    parser = argparse.ArgumentParser(description="Track balls and overlay ballistic trajectories on ZED video.")
    parser.add_argument("--onnx", type=Path, required=True, help="Path to the exported YOLO ONNX model.")
    parser.add_argument("--svo", help="Optional SVO recording instead of a live camera.")
    parser.add_argument("--confidence", type=float, default=25.0, help="Ball detector confidence threshold as a percentage.")
    parser.add_argument("--dynamic-input-size", type=int, help="Optional dynamic ONNX input size, for example 512 or 640.")
    parser.add_argument("--static-camera", action="store_true", help="Tell ZED positional tracking that the camera does not move.")
    parser.add_argument("--display-fps", type=float, default=DEFAULT_DISPLAY_FPS, help="Debug display rate only; object detection still runs on every camera frame.")
    parser.add_argument("--stale-track-seconds", type=float, default=DEFAULT_STALE_TRACK_SECONDS, help="Remove trajectory state after an id has not been updated for this long.")
    parser.add_argument("--measure-processing-time", action="store_true", help="Print SDK object processing and total get_objects latency.")
    parser.add_argument("--no-display", action="store_true", help="Disable image retrieval and OpenCV drawing completely.")
    return parser.parse_args()


def main() -> None:
    """Run the 100 Hz object path and optionally overlay trajectories on the real camera image."""

    from ZedHandler import ZedHandler

    args = parse_args()

    #validate command-line values before camera initialization
    if not args.onnx.is_file():
        raise FileNotFoundError(f"ONNX model not found: {args.onnx}")
    if not 0.0 <= args.confidence <= 100.0:
        raise ValueError("--confidence must be between 0 and 100")
    if args.dynamic_input_size is not None and (args.dynamic_input_size <= 0 or args.dynamic_input_size % 32 != 0):
        raise ValueError("--dynamic-input-size must be a positive multiple of 32")
    if args.display_fps <= 0.0:
        raise ValueError("--display-fps must be positive")
    if args.stale_track_seconds <= 0.0:
        raise ValueError("--stale-track-seconds must be positive")

    #avoid importing opencv at all when the display is disabled
    cv2 = None
    if not args.no_display:
        import cv2 as _cv2
        cv2 = _cv2

    #trajectory state is independent from ZedHandler's private sdk-id reassociation state
    tracker = BallTracker(args.stale_track_seconds)
    #display period lets the camera/object path run faster than the debug visualization
    display_period = 1.0 / args.display_fps
    last_display_time = float("-inf")

    try:
        with ZedHandler(onnx_path=args.onnx, confidence_percent=args.confidence, dynamic_input_size=args.dynamic_input_size, svo_path=args.svo, static_camera=args.static_camera, measure_processing_time=args.measure_processing_time) as zed:
            #intrinsics are only needed when 3d predictions will be drawn on the image
            intrinsics = None if args.no_display else zed.get_camera_intrinsics()

            while True:
                #hot path: grab, native yolo/tracking, minimal object conversion
                result = zed.grab_and_get_objects()
                if result is None:
                    continue

                #convert the image timestamp to seconds for trajectory math
                objects, timestamp_ns = result
                timestamp_seconds = timestamp_ns * 1e-9

                #update fitting on every available camera frame
                tracker.update(objects, timestamp_seconds)

                #optional profiling shows how much time is sdk work versus python-side handling
                if args.measure_processing_time:
                    sdk_time = zed.last_processing_time_seconds
                    total_time = zed.last_get_objects_time_seconds
                    if sdk_time is not None and total_time is not None:
                        print(f"SDK={sdk_time * 1000.0:.3f} ms  get_objects={total_time * 1000.0:.3f} ms")

                #stop here so image retrieval and drawing cost nothing in headless mode
                if args.no_display:
                    continue

                #skip debug frames until the requested display period has elapsed
                if timestamp_seconds - last_display_time < display_period:
                    continue

                assert cv2 is not None
                assert intrinsics is not None

                #debug path: only now retrieve pixels and convert current 2d boxes
                frame, boxes_2d = zed.get_frame()
                assert isinstance(frame, np.ndarray)
                draw_tracking_overlay(frame, tracker, objects, boxes_2d, timestamp_seconds, intrinsics)
                cv2.imshow("Ball trajectory tracking", frame)
                last_display_time = timestamp_seconds

                #q or escape exits the live preview
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    finally:
        if cv2 is not None:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
