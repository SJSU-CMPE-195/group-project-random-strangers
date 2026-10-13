"""Fast fixed-gravity projectile fitting

The reusable trajectory-fitting interface is kept at the top of this file.

Trajectory fitting uses ZED's gravity-aligned RIGHT_HANDED_Y_UP world frame.
The world +Y axis points up, so gravity is fixed at [0, -9.80665, 0] m/s^2.
World-space predictions are transformed into the current camera frame only for visualization.
"""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

from ZedHandler import ZedCameraIntrinsics, ZedCameraPose, ZedObject2D, ZedObject3D


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
    #positions must be an N by 3 array of gravity-aligned world coordinates
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

#minimum number of valid 3d samples before a track can possibly be considered projectile motion
MIN_FIT_MEASUREMENTS = 8

#ignore early world-space samples while ZED finishes settling its IMU gravity alignment
GRAVITY_ALIGNMENT_WARMUP_SECONDS = 1.5

#minimum observed time needed before gravity curvature is large enough to distinguish from straight motion
MIN_PROJECTILE_DURATION_SECONDS = 0.18

#full 3d sanity limit; projectile classification itself uses vertical-axis curvature below
MAX_FIT_ERROR_M = 0.25

#fixed-gravity vertical motion must fit at least this much better than constant vertical velocity
MIN_BALLISTIC_ERROR_IMPROVEMENT_M = 0.001

#allow modest depth noise while still requiring gravity to improve the vertical fit
MAX_BALLISTIC_TO_LINEAR_ERROR_RATIO = 0.99

#reject a single-frame position jump that would imply an impossible dodgeball speed
MAX_PLAUSIBLE_BALL_SPEED_MPS = 50.0

#fallback window sizes used if older samples no longer describe the current throw
FIT_WINDOWS = (30, 24, 20, 16, 12, 8)

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

#camera-space points closer than this to the camera plane are unsafe to project
MIN_CAMERA_DEPTH_M = 0.05

#different stable ids cycle through these bgra colors
TRACK_COLORS = ((0, 220, 255, 255), (0, 255, 120, 255), (255, 160, 0, 255), (255, 80, 180, 255))


@dataclass(frozen=True, slots=True)
class BallMeasurement:
    """One valid 3D position sample used by the ballistic fitter."""

    #absolute capture time in seconds
    timestamp: float

    #xyz position in the gravity-aligned zed world frame
    position: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class ScreenMeasurement:
    """One debug-only measured ball position in actual left-image pixels."""

    #absolute capture time in seconds
    timestamp: float

    #center of the zed 2d detection box in left-image pixels
    position: tuple[float, float]


@dataclass(slots=True)
class BallTrack:
    """Trajectory history and current fit for one stable ZedHandler object id."""

    #recent valid 3d samples, automatically capped at MAX_MEASUREMENTS
    measurements: deque[BallMeasurement] = field(default_factory=lambda: deque(maxlen=MAX_MEASUREMENTS))

    #debug-only screen positions sampled only when get_frame() is called
    screen_measurements: deque[ScreenMeasurement] = field(default_factory=lambda: deque(maxlen=MAX_MEASUREMENTS))

    #updated whenever the id appears, even if depth is invalid on that frame
    last_seen_timestamp: float = float("-inf")

    #most recent accepted ballistic fit, or None when the motion does not currently look ballistic
    fit: TrajectoryFit | None = None

    #short debug explanation for why a prediction is or is not currently available
    fit_status: str = "waiting for gravity alignment"


class BallTracker:
    """Maintain one ballistic trajectory history for each stable ZedHandler object id."""

    def __init__(self, stale_after_seconds: float = DEFAULT_STALE_TRACK_SECONDS) -> None:
        if stale_after_seconds <= 0.0:
            raise ValueError("stale_after_seconds must be positive")

        #time an id may disappear before its trajectory state is deleted
        self.stale_after_seconds = stale_after_seconds
        
        #stable object id to its measurement history and current fit
        self.tracks: dict[int, BallTrack] = {}

        #first ZED image timestamp seen by this tracker; used only for gravity-alignment warmup
        self.start_timestamp: float | None = None

        #last global timestamp lets an SVO restart or clock reset restart the warmup cleanly
        self.last_timestamp: float | None = None

        #becomes true once startup samples have been discarded and world-space fitting may begin
        self.gravity_alignment_ready = False

    def update(self, objects: Sequence[ZedObject3D], timestamp_seconds: float) -> None:
        """Update all ball histories from one ZED object result."""

        #restart trajectory state if the input clock moves backwards, such as an SVO restart
        if self.last_timestamp is not None and timestamp_seconds < self.last_timestamp:
            self.tracks.clear()
            self.start_timestamp = timestamp_seconds
            self.gravity_alignment_ready = False
        self.last_timestamp = timestamp_seconds

        #start the gravity-alignment warmup from the first successfully retrieved ZED frame
        if self.start_timestamp is None:
            self.start_timestamp = timestamp_seconds

        warmup_remaining = GRAVITY_ALIGNMENT_WARMUP_SECONDS - (timestamp_seconds - self.start_timestamp)

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

            #do not mix provisional startup world coordinates with the final gravity-aligned world frame
            if warmup_remaining > 0.0:
                track.measurements.clear()
                track.fit = None
                track.fit_status = f"gravity warmup {warmup_remaining:.1f}s"
                continue

            #clear every pre-alignment history exactly once when the warmup ends
            if not self.gravity_alignment_ready:
                track.measurements.clear()
                track.fit = None
                track.fit_status = "collecting post-alignment samples"

            #only finite world-space xyz samples are allowed into the fit
            position = _validate_position(obj.position)
            if position is None:
                track.fit = None
                track.fit_status = "invalid 3d position"
                continue

            if track.measurements:
                previous = track.measurements[-1]
                dt = timestamp_seconds - previous.timestamp

                #ignore an accidental duplicate sample from the same captured frame
                if dt == 0.0:
                    continue

                #clock reversal means this track history is no longer usable
                if dt < 0.0:
                    track.measurements.clear()
                    track.fit = None
                    track.fit_status = "timestamp reset"

                #a sudden world-frame reorientation or bad depth sample appears as an impossible instantaneous speed
                if dt > 0.0:
                    dx = position[0] - previous.position[0]
                    dy = position[1] - previous.position[1]
                    dz = position[2] - previous.position[2]
                    measured_speed = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
                    if measured_speed > MAX_PLAUSIBLE_BALL_SPEED_MPS:
                        track.measurements.clear()
                        track.fit = None
                        track.fit_status = f"world/depth jump {measured_speed:.0f}m/s"

            #store this valid point and immediately refresh the trajectory fit
            track.measurements.append(BallMeasurement(timestamp=timestamp_seconds, position=position))
            track.fit, track.fit_status = _fit_recent_measurements(track.measurements)

        #the global world frame is considered usable only after the warmup has fully elapsed
        if warmup_remaining <= 0.0:
            self.gravity_alignment_ready = True

        #remove tracks that have genuinely stopped receiving detector updates
        self.prune(timestamp_seconds)

    def update_screen_positions(self, boxes_2d: Sequence[ZedObject2D], timestamp_seconds: float) -> None:
        """Store true image-space ball centers only on debug display frames."""

        for box in boxes_2d:
            track = self.tracks.get(box.object_id)
            if track is None:
                continue

            center = _box_center(box.bounding_box_2d)
            if center is None:
                continue

            #do not add the same camera frame twice if the overlay function is called more than once
            if track.screen_measurements and timestamp_seconds <= track.screen_measurements[-1].timestamp:
                continue

            track.screen_measurements.append(ScreenMeasurement(timestamp=timestamp_seconds, position=center))

    def prune(self, timestamp_seconds: float) -> None:
        """Remove stable ids that have not been reported recently."""

        #collect first so the dictionary is not modified while it is being iterated
        stale_ids = [object_id for object_id, track in self.tracks.items() if timestamp_seconds - track.last_seen_timestamp > self.stale_after_seconds]

        for object_id in stale_ids:
            del self.tracks[object_id]


def _validate_position(position_xyz: Sequence[float]) -> tuple[float, float, float] | None:
    """Return a usable gravity-aligned world-space XYZ point or None when ZED depth is invalid."""

    if len(position_xyz) != 3:
        return None

    x = float(position_xyz[0])
    y = float(position_xyz[1])
    z = float(position_xyz[2])

    #world coordinates can grow as the camera moves, so only reject non-finite depth results here
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        return None

    return x, y, z


def _constant_velocity_axis_error(times: np.ndarray, values: np.ndarray) -> float:
    """Return RMS error of the best constant-velocity line for one coordinate axis."""

    reference_time = float(times[-1])
    dt = times - reference_time
    mean_dt = float(dt.mean())
    centered_dt = dt - mean_dt
    denominator = float(centered_dt @ centered_dt)

    if denominator <= 1e-15:
        return float("inf")

    mean_value = float(values.mean())
    velocity = float((centered_dt * (values - mean_value)).sum() / denominator)
    value_at_reference = mean_value - velocity * mean_dt
    residuals = value_at_reference + dt * velocity - values
    return float(np.sqrt(np.mean(residuals * residuals)))


def _projectile_motion_check(times: np.ndarray, positions: np.ndarray, fit: TrajectoryFit) -> tuple[bool, str]:
    """Check whether the measured vertical curvature is actually consistent with gravity."""

    duration = float(times[-1] - times[0])
    if len(times) < MIN_FIT_MEASUREMENTS:
        return False, f"need {MIN_FIT_MEASUREMENTS} samples"
    if duration < MIN_PROJECTILE_DURATION_SECONDS:
        return False, f"need {MIN_PROJECTILE_DURATION_SECONDS - duration:.2f}s more"

    #keep a generous full-3d sanity bound so a wildly incorrect depth track is never predicted
    if fit.rms_error_m > MAX_FIT_ERROR_M:
        return False, f"3d fit err {fit.rms_error_m:.2f}m"

    #gravity changes only world Y, so compare models on Y instead of letting unrelated X/Z depth noise hide the curvature
    predicted_y = fit.predict(times)[:, 1]
    ballistic_y_error = float(np.sqrt(np.mean((predicted_y - positions[:, 1]) ** 2)))
    linear_y_error = _constant_velocity_axis_error(times, positions[:, 1])

    #a stationary or constant-velocity object has essentially no evidence of gravitational curvature
    if not math.isfinite(linear_y_error) or linear_y_error <= 1e-9:
        return False, "no vertical curvature"

    improvement = linear_y_error - ballistic_y_error
    ratio = ballistic_y_error / linear_y_error

    #require gravity to provide a real, but intentionally modest, improvement over straight vertical motion
    if improvement < MIN_BALLISTIC_ERROR_IMPROVEMENT_M:
        return False, f"linear-like y {ballistic_y_error:.3f}/{linear_y_error:.3f}m"
    if ratio > MAX_BALLISTIC_TO_LINEAR_ERROR_RATIO:
        return False, f"weak gravity y {ballistic_y_error:.3f}/{linear_y_error:.3f}m"

    return True, f"projectile y {ballistic_y_error:.3f}/{linear_y_error:.3f}m"


def _fit_recent_measurements(measurements: Sequence[BallMeasurement]) -> tuple[TrajectoryFit | None, str]:
    """Fit the longest recent sample window that passes the projectile-motion checks."""

    #convert once so recent windows can use normal slicing
    samples = tuple(measurements)
    sample_count = len(samples)

    if sample_count < MIN_FIT_MEASUREMENTS:
        return None, f"{sample_count}/{MIN_FIT_MEASUREMENTS} samples"

    #try all available measurements first, then progressively shorter recent windows
    window_sizes = [sample_count]
    window_sizes.extend(size for size in FIT_WINDOWS if MIN_FIT_MEASUREMENTS <= size < sample_count)
    first_rejection = "no valid fit"

    for window_index, window_size in enumerate(window_sizes):
        window = samples[-window_size:]
        times = np.fromiter((sample.timestamp for sample in window), dtype=np.float64, count=window_size)

        #skip windows too short in real time before allocating the positions array or running the regression
        duration = float(times[-1] - times[0])
        if duration < MIN_PROJECTILE_DURATION_SECONDS:
            if window_index == 0:
                first_rejection = f"need {MIN_PROJECTILE_DURATION_SECONDS - duration:.2f}s more"
            continue

        positions = np.asarray([sample.position for sample in window], dtype=np.float64)

        try:
            fit = fit_trajectory(times, positions)
        except ValueError:
            if window_index == 0:
                first_rejection = "trajectory fit failed"
            continue

        accepted, reason = _projectile_motion_check(times, positions, fit)
        if accepted:
            return fit, reason
        if window_index == 0:
            first_rejection = reason

    return None, first_rejection


# =============================================================================
# CAMERA OVERLAY
# =============================================================================


def _camera_to_world_rotation(camera_pose: ZedCameraPose) -> np.ndarray:
    """Convert ZED's camera-to-world quaternion into a 3x3 rotation matrix."""

    qx, qy, qz, qw = camera_pose.orientation
    quaternion_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if quaternion_norm <= 1e-12:
        raise ValueError("camera orientation quaternion is invalid")

    #normalize because small numerical drift should not change the projection scale
    qx /= quaternion_norm
    qy /= quaternion_norm
    qz /= quaternion_norm
    qw /= quaternion_norm

    #this rotation maps a point from the current camera frame into the ZED world frame
    return np.array([[1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)], [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)], [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)]], dtype=np.float64)


def _world_to_camera_points(points_world: np.ndarray, camera_pose: ZedCameraPose) -> np.ndarray:
    """Transform gravity-aligned world points into the current left-camera coordinate frame."""

    points = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    camera_position_world = np.asarray(camera_pose.position, dtype=np.float64)
    camera_to_world = _camera_to_world_rotation(camera_pose)

    #for row vectors, multiplying by camera_to_world is equivalent to applying its transpose to column vectors
    return (points - camera_position_world) @ camera_to_world


def _project_camera_points(points_camera: np.ndarray, intrinsics: ZedCameraIntrinsics, frame_shape: tuple[int, ...]) -> tuple[np.ndarray, np.ndarray]:
    """Project RIGHT_HANDED_Y_UP camera-frame points onto the rectified left image."""

    points = np.asarray(points_camera, dtype=np.float64).reshape(-1, 3)
    frame_height, frame_width = frame_shape[:2]

    #scale calibration if the returned debug image size differs from the calibration resolution
    scale_x = frame_width / intrinsics.width
    scale_y = frame_height / intrinsics.height
    fx = intrinsics.fx * scale_x
    fy = intrinsics.fy * scale_y
    cx = intrinsics.cx * scale_x
    cy = intrinsics.cy * scale_y

    #RIGHT_HANDED_Y_UP camera coordinates use +X right, +Y up, and -Z forward
    x = points[:, 0]
    image_y = -points[:, 1]
    depth = -points[:, 2]

    #front-of-camera checks belong here because the fitter now operates entirely in world coordinates
    valid = np.isfinite(points).all(axis=1) & (depth > MIN_CAMERA_DEPTH_M)
    pixels = np.full((len(points), 2), np.nan, dtype=np.float64)
    pixels[valid, 0] = fx * x[valid] / depth[valid] + cx
    pixels[valid, 1] = fy * image_y[valid] / depth[valid] + cy
    return pixels, valid


def _project_world_points(points_world: np.ndarray, camera_pose: ZedCameraPose, intrinsics: ZedCameraIntrinsics, frame_shape: tuple[int, ...]) -> tuple[np.ndarray, np.ndarray]:
    """Transform world points into the current camera frame and project them into image pixels."""

    camera_points = _world_to_camera_points(points_world, camera_pose)
    return _project_camera_points(camera_points, intrinsics, frame_shape)


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


def _box_center(bounding_box_2d: Sequence[Sequence[float]]) -> tuple[float, float] | None:
    """Return the center of one ZED 2D box in actual left-image pixels."""

    points = np.asarray(bounding_box_2d, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 2 or points.shape[1] < 2 or not np.isfinite(points[:, :2]).all():
        return None

    #average all sdk box corners instead of assuming a particular corner ordering
    center = points[:, :2].mean(axis=0)
    return float(center[0]), float(center[1])


def _draw_measured_trail(frame: np.ndarray, measurements: Sequence[ScreenMeasurement], color: tuple[int, int, int, int]) -> tuple[int, int] | None:
    """Draw measured history from the real ZED 2D detections instead of reprojected 3D centroids."""

    import cv2

    if not measurements:
        return None

    #using the real detector box centers avoids camera/world-axis mistakes and depth-centroid projection offsets in the debug trail
    pixels = np.asarray([measurement.position for measurement in measurements], dtype=np.float64)

    for index in range(len(pixels) - 1):
        _draw_segment(frame, pixels[index], pixels[index + 1], color, 2)

    latest = pixels[-1]
    latest_point = (int(round(float(latest[0]))), int(round(float(latest[1]))))
    if 0 <= latest_point[0] < frame.shape[1] and 0 <= latest_point[1] < frame.shape[0]:
        cv2.circle(frame, latest_point, 4, color, -1, cv2.LINE_AA)

    return latest_point


def _draw_prediction(frame: np.ndarray, fit: TrajectoryFit, timestamp_seconds: float, camera_pose: ZedCameraPose, intrinsics: ZedCameraIntrinsics, color: tuple[int, int, int, int], anchor_pixel: tuple[int, int] | None) -> None:
    """Transform the world-space prediction into the current camera frame and draw it."""

    #trajectory math stays entirely in the gravity-aligned world frame
    start_time = max(timestamp_seconds, fit.reference_time)
    future_times = np.linspace(start_time, start_time + PREDICTION_SECONDS, PREDICTION_SAMPLES, dtype=np.float64)
    future_world_positions = fit.predict(future_times)
    pixels, valid = _project_world_points(future_world_positions, camera_pose, intrinsics, frame.shape)

    #the 3d object center and 2d detector center can differ slightly, so visually anchor the projected curve to the current detection
    if anchor_pixel is not None and np.any(valid):
        first_valid_index = int(np.flatnonzero(valid)[0])
        pixel_offset = np.asarray(anchor_pixel, dtype=np.float64) - pixels[first_valid_index]
        pixels[valid] += pixel_offset

    #skip every other segment to create a cheap dashed line without alpha blending
    for index in range(0, len(pixels) - 1, 2):
        if valid[index] and valid[index + 1]:
            _draw_segment(frame, pixels[index], pixels[index + 1], color, 2)


def draw_tracking_overlay(frame: np.ndarray, tracker: BallTracker, current_objects: Sequence[ZedObject3D], boxes_2d: Sequence[ZedObject2D], timestamp_seconds: float, camera_pose: ZedCameraPose | None, intrinsics: ZedCameraIntrinsics) -> None:
    """Overlay current detector boxes, measured paths, and fitted future paths."""

    import cv2

    #store the current real image-space centers only on display frames so the hot 100 hz path is unchanged
    tracker.update_screen_positions(boxes_2d, timestamp_seconds)

    #lookup lets each debug box get confidence without searching the object list repeatedly
    objects_by_id = {obj.object_id: obj for obj in current_objects}

    #draw current detector boxes first so trajectory lines remain visible over them
    for box in boxes_2d:
        color = TRACK_COLORS[box.object_id % len(TRACK_COLORS)]
        _draw_detection_box(frame, box, objects_by_id.get(box.object_id), color)

    for object_id, track in tracker.tracks.items():
        #draw the measured path from true 2d detections rather than projected depth points
        recent_screen_measurements = [measurement for measurement in track.screen_measurements if timestamp_seconds - measurement.timestamp <= TRAIL_DURATION_SECONDS]
        if not recent_screen_measurements:
            continue

        color = TRACK_COLORS[object_id % len(TRACK_COLORS)]
        latest_pixel = _draw_measured_trail(frame, recent_screen_measurements, color)

        #prediction appears only for projectile motion and only when the current world-to-camera pose is valid
        if track.fit is not None and camera_pose is not None:
            _draw_prediction(frame, track.fit, timestamp_seconds, camera_pose, intrinsics, color, latest_pixel)

        if latest_pixel is None:
            continue

        x, y = latest_pixel
        if not (0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]):
            continue

        #show exactly why a prediction is absent so projectile gating can be tuned from real camera data
        if track.fit is None:
            text = track.fit_status
        else:
            speed = float(np.linalg.norm(track.fit.velocity))
            text = f"fit={track.fit.measurement_count}  err={track.fit.rms_error_m:.3f}m  v={speed:.1f}m/s"

        text_position = (max(4, min(frame.shape[1] - 260, x + 7)), min(frame.shape[0] - 6, y + 16))
        cv2.putText(frame, text, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.40, color, 1, cv2.LINE_AA)

    #small status line confirms that gravity comes from the ZED-aligned world frame rather than camera orientation
    pose_status = "world pose ok" if camera_pose is not None else "world pose unavailable"
    gravity_status = "gravity ready" if tracker.gravity_alignment_ready else "gravity settling"
    cv2.putText(frame, f"tracks={len(tracker.tracks)}  {gravity_status}  {pose_status}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (240, 240, 240, 255), 1, cv2.LINE_AA)


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

                #debug path: retrieve image data plus the current camera pose only when visualization is actually requested
                frame, boxes_2d = zed.get_frame()
                camera_pose = zed.get_camera_pose()
                assert isinstance(frame, np.ndarray)
                draw_tracking_overlay(frame, tracker, objects, boxes_2d, timestamp_seconds, camera_pose, intrinsics)
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
