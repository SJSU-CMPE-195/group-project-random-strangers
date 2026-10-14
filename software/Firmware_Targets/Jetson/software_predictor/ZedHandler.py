"""Fast ZED native-YOLO ball detection and tracking API.

Model classes:
    0 = person
    1 = ball
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import pyzed.sl as sl

BALL_CLASS_ID = 1
DEFAULT_CONFIDENCE_PERCENT = 25.0
TIMING_WINDOW_FRAMES = 120
DEFAULT_DISPLAY_FPS = 30.0
DEFAULT_PRINT_FPS = 10.0
DEFAULT_DEPTH_STABILIZATION = 0
DEFAULT_TRACKING_COMPUTE = "cpu"
DEFAULT_DEPTH_MODE = "neural-light"


@dataclass(frozen=True, slots=True)
class ZedObject3D:
    """Minimal object returned by the real-time tracking path."""

    #id assigned by the zed sdk tracker
    object_id: int

    #original class id from the detector
    raw_label: int

    #object center in the gravity-aligned frame, in meters
    position: tuple[float, float, float]

    #detector confidence (0-1)
    confidence: float

    #tracking state reported by the zed sdk
    tracking_state: object


@dataclass(frozen=True, slots=True)
class ZedObject2D:
    """Where object is in image"""

    #zed sdk id
    object_id: int

    #original class id from the detector
    raw_label: int

    #2d corners in left-camera pixel coordinates
    bounding_box_2d: tuple[tuple[float, float], ...]


@dataclass(frozen=True, slots=True)
class ZedCameraIntrinsics:
    """Rectified left-camera intrinsics used to project 3D points into the image."""

    #focal length in pixels
    fx: float
    fy: float

    #optical center in pixels
    cx: float
    cy: float

    #calibration image size
    width: int
    height: int


@dataclass(frozen=True, slots=True)
class ZedCameraPose:
    """Where the center of the left camera is in respect to the gravity aligned frame"""

    #left-camera origin expressed in world coordinates, in meters
    position: tuple[float, float, float]

    #camera-to-world orientation quaternion stored as x, y, z, w
    orientation: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class ZedTimingStats:
    """Rolling statistics sampled from ZED Camera.get_current_fps()."""

    #most recent fps value reported by the zed sdk
    latest_fps: float

    #mean of recent zed fps samples
    mean_fps: float

    #median of recent zed fps samples
    median_fps: float

    #5th percentile fps, useful as a slow-frame throughput metric
    p05_fps: float

    #number of zed fps samples currently in the rolling window
    sample_count: int


def _check_success(status: sl.ERROR_CODE, operation: str) -> None:
    """Convert a ZED SDK error code into a normal Python exception."""

    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"{operation} failed: {status}")


def _make_runtime_filter(confidence_percent: float) -> sl.CustomObjectDetectionRuntimeParameters:
    """Configure object recognition thresholds"""

    runtime = sl.CustomObjectDetectionRuntimeParameters()
    runtime.object_detection_properties.detection_confidence_threshold = confidence_percent

    #disable person detections by setting their required confidence to 100 percent
    person = sl.CustomObjectDetectionProperties()
    person.detection_confidence_threshold = 100.0
    if hasattr(person, "enabled"):
        person.enabled = False

    #allow ball detections at the requested confidence threshold
    ball = sl.CustomObjectDetectionProperties()
    ball.detection_confidence_threshold = confidence_percent
    if hasattr(ball, "enabled"):
        ball.enabled = True

    #a thrown ball moves freely in 3d, so never constrain the zed tracker to the ground plane
    if hasattr(ball, "is_grounded"):
        ball.is_grounded = False
    if hasattr(ball, "is_static"):
        ball.is_static = False

    #disable sdk smoothing
    if hasattr(ball, "velocity_smoothing_factor"):
        ball.velocity_smoothing_factor = 0.0
    if hasattr(ball, "min_velocity_threshold"):
        ball.min_velocity_threshold = 0.0
    if hasattr(ball, "prediction_timeout_s"):
        ball.prediction_timeout_s = 0.0
    if hasattr(ball, "min_confirmation_time_s"):
        ball.min_confirmation_time_s = 0.0

    runtime.object_class_detection_properties = {0: person, BALL_CLASS_ID: ball}
    return runtime


class ZedHandler:
    """This class manages the ZED SDK and its connection to the camera

        During initialization, it uploads the YOLO model to the ZED SDK and enables object tracking
        call grab_and_get_objects() to return detection positions
    """

    def __init__(self, onnx_path: str | Path, confidence_percent: float = DEFAULT_CONFIDENCE_PERCENT, dynamic_input_size: int | None = None, svo_path: str | None = None, static_camera: bool = False, measure_framerate: bool = False, allow_reduced_precision_inference: bool = True, depth_stabilization: int = DEFAULT_DEPTH_STABILIZATION, tracking_compute: str = DEFAULT_TRACKING_COMPUTE, depth_mode: str = DEFAULT_DEPTH_MODE) -> None:
        #validate arguments before opening the camera
        model_path = Path(onnx_path)
        if not model_path.is_file():
            raise FileNotFoundError(f"ONNX model not found: {model_path}")
        if not 0.0 <= confidence_percent <= 100.0:
            raise ValueError("confidence_percent must be between 0 and 100")
        if dynamic_input_size is not None and (dynamic_input_size <= 0 or dynamic_input_size % 32 != 0):
            raise ValueError("dynamic_input_size must be a positive multiple of 32")
        if not 0 <= depth_stabilization <= 100:
            raise ValueError("depth_stabilization must be between 0 and 100")
        if tracking_compute not in ("auto", "cpu", "gpu"):
            raise ValueError("tracking_compute must be auto, cpu, or gpu")
        if depth_mode not in ("neural-light", "performance"):
            raise ValueError("depth_mode must be neural-light or performance")

        #framerate statistics
        self._measure_framerate = measure_framerate
        self._current_fps: float | None = None
        self._timing_samples_fps: deque[float] = deque(maxlen=TIMING_WINDOW_FRAMES)

        #container for ZED object detections
        self._objects = sl.Objects()

        #camera runtime parameters
        self._runtime = sl.RuntimeParameters()
        self._runtime.measure3D_reference_frame = sl.REFERENCE_FRAME.WORLD #tell ZED sdk to use gravity aligned coordinates
        self._object_runtime = _make_runtime_filter(confidence_percent) #put the filter on model outputs

        #state variables
        self._frame_ready = False
        self._objects_ready = False
        self._frame_timestamp_ns: int | None = None

        #camera image buffer (mostly for debug)
        self._image_buffer: sl.Mat | None = None
        self._intrinsics: ZedCameraIntrinsics | None = None #camera intrinsics allow for visualization using zed image frames
        self._camera_pose: sl.Pose | None = None

        #create the ZED camera
        self._zed = sl.Camera()
        input_type = sl.InputType()
        if svo_path:
            input_type.set_from_svo_file(svo_path)

        #configure the camera for the 100 hz pipeline
        init = sl.InitParameters(input_t=input_type)
        init.camera_resolution = sl.RESOLUTION.VGA
        init.camera_fps = 100
        init.depth_mode = sl.DEPTH_MODE.NEURAL_LIGHT if depth_mode == "neural-light" else sl.DEPTH_MODE.NEURAL if depth_mode == "neural" else sl.DEPTH_MODE.PERFORMANCE
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init.depth_maximum_distance = 20
        init.depth_stabilization = depth_stabilization

        #use the sdk's cuda graph path when this sdk version exposes it
        if hasattr(init, "allow_depth_cuda_graph"):
            init.allow_depth_cuda_graph = True

        _check_success(self._zed.open(init), "Opening ZED camera")

        try:
            #make ZED use IMU to find gravity
            positional_tracking = sl.PositionalTrackingParameters()
            positional_tracking.enable_imu_fusion = True
            positional_tracking.set_gravity_as_origin = True

            #GEN_3 is the fastest positional tracking mode
            if hasattr(positional_tracking, "mode") and hasattr(sl, "POSITIONAL_TRACKING_MODE") and hasattr(sl.POSITIONAL_TRACKING_MODE, "GEN_3"):
                positional_tracking.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3

            #don't need to save a map of the area
            if hasattr(positional_tracking, "enable_area_memory"):
                positional_tracking.enable_area_memory = False
            if hasattr(positional_tracking, "enable_pose_smoothing"):
                positional_tracking.enable_pose_smoothing = False

            #SDK 5.5 can keep some work on CPU so the GPU is freer for YOLO and depth
            if tracking_compute != "auto" and hasattr(positional_tracking, "compute_preference"):
                preference_type = getattr(sl, "POSITIONAL_TRACKING_COMPUTE_PREFERENCE", None)
                if preference_type is not None:
                    preference_name = "PREFER_CPU" if tracking_compute == "cpu" else "PREFER_GPU"
                    if hasattr(preference_type, preference_name):
                        positional_tracking.compute_preference = getattr(preference_type, preference_name)

            if static_camera:
                positional_tracking.set_as_static = True
            _check_success(self._zed.enable_positional_tracking(positional_tracking), "Enabling positional tracking")

            #run the exported onnx model through the sdk's native yolo path
            detection = sl.ObjectDetectionParameters()
            detection.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_YOLOLIKE_BOX_OBJECTS
            detection.custom_onnx_file = str(model_path.resolve())
            detection.enable_tracking = True
            detection.enable_segmentation = False

            #skip the SDK's 3d NMS pass
            if hasattr(detection, "filtering_mode") and hasattr(sl, "OBJECT_FILTERING_MODE") and hasattr(sl.OBJECT_FILTERING_MODE, "NONE"):
                detection.filtering_mode = sl.OBJECT_FILTERING_MODE.NONE
            if hasattr(detection, "prediction_timeout_s"):
                detection.prediction_timeout_s = 0.0

            #allow fp16 or other reduced precision inference when supported
            if hasattr(detection, "allow_reduced_precision_inference"):
                detection.allow_reduced_precision_inference = allow_reduced_precision_inference

            #only override the model input resolution when explicitly requested
            if dynamic_input_size is not None:
                detection.custom_onnx_dynamic_input_shape = sl.Resolution(dynamic_input_size, dynamic_input_size)

            _check_success(self._zed.enable_object_detection(detection), "Enabling native YOLO object detection")
        except Exception:
            self.close()
            raise

    def grab(self) -> bool:
        """Acquire the next frame without retrieving image pixels or object data."""

        if self._zed is None:
            raise RuntimeError("ZED camera is closed")

        #tell ZED to process the latest frame
        status = self._zed.grab(self._runtime)

        self._frame_ready = status == sl.ERROR_CODE.SUCCESS
        self._objects_ready = False

        if self._frame_ready:
            #get the exact measurement time
            self._frame_timestamp_ns = int(self._zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())

            #get FPS if requested
            if self._measure_framerate:
                current_fps = float(self._zed.get_current_fps())
                if math.isfinite(current_fps) and current_fps > 0.0:
                    self._current_fps = current_fps
                    self._timing_samples_fps.append(current_fps)

        return self._frame_ready

    def get_objects(self) -> tuple[list[ZedObject3D], int]:
        """Return tracked balls from the latest grabbed frame"""

        if self._zed is None:
            raise RuntimeError("ZED camera is closed")
        if not self._frame_ready:
            raise RuntimeError("Call grab() successfully before get_objects()")

        #get the objects
        status = self._zed.retrieve_custom_objects(self._objects, self._object_runtime)
        _check_success(status, "Retrieving ZED objects")

        results: list[ZedObject3D] = []

        for obj in self._objects.object_list:
            #make sure that no people detections found their way in
            raw_label = int(obj.raw_label)
            if raw_label != BALL_CLASS_ID:
                continue

            object_id = int(obj.id)

            #copy the three position values needed by the trajectory code
            position = obj.position
            x = float(position[0])
            y = float(position[1])
            z = float(position[2])

            results.append(ZedObject3D(object_id=object_id, raw_label=raw_label, position=(x, y, z), confidence=float(obj.confidence) * 0.01, tracking_state=obj.tracking_state))

        #get_frame() is now allowed to use the sdk objects from this exact frame
        self._objects_ready = True

        if self._frame_timestamp_ns is None:
            raise RuntimeError("Frame timestamp is unavailable")

        return results, self._frame_timestamp_ns

    def grab_and_get_objects(self) -> tuple[list[ZedObject3D], int] | None:
        """grabs one frame and immediately gets objects."""

        if not self.grab():
            return None
        return self.get_objects()

    def get_frame(self) -> tuple[object, list[ZedObject2D]]:
        """Return the current left image and current ball boxes for debugging/visualization."""

        if self._zed is None:
            raise RuntimeError("ZED camera is closed")
        if not self._frame_ready:
            raise RuntimeError("Call grab() successfully before get_frame()")
        if not self._objects_ready:
            raise RuntimeError("Call get_objects() for the current frame before get_frame()")

        #allocate the image buffer (if needed)
        if self._image_buffer is None:
            self._image_buffer = sl.Mat()

        #retrieve pixels
        _check_success(self._zed.retrieve_image(self._image_buffer, sl.VIEW.LEFT), "Retrieving left camera image")

        boxes: list[ZedObject2D] = []

        #list the objects inside the frame
        for obj in self._objects.object_list:
            raw_label = int(obj.raw_label)
            if raw_label != BALL_CLASS_ID:
                continue

            object_id = int(obj.id)
            bounding_box_2d = tuple((float(point[0]), float(point[1])) for point in obj.bounding_box_2d)
            boxes.append(ZedObject2D(object_id=object_id, raw_label=raw_label, bounding_box_2d=bounding_box_2d))

        #get_data returns the sdk-backed BGRA numpy view without a full-frame color conversion
        return self._image_buffer.get_data(), boxes

    def get_camera_intrinsics(self) -> ZedCameraIntrinsics:
        """Return cached rectified left-camera calibration used for 3D-to-2D projection."""

        if self._intrinsics is not None:
            return self._intrinsics
        if self._zed is None:
            raise RuntimeError("ZED camera is closed")

        #read calibration once because it does not change while this camera is open
        info = self._zed.get_camera_information()
        configuration = info.camera_configuration
        camera = configuration.calibration_parameters.left_cam
        resolution = configuration.resolution
        self._intrinsics = ZedCameraIntrinsics(fx=float(camera.fx), fy=float(camera.fy), cx=float(camera.cx), cy=float(camera.cy), width=int(resolution.width), height=int(resolution.height))
        return self._intrinsics

    def get_camera_pose(self) -> ZedCameraPose | None:
        """Return the current left-camera pose in the gravity-aligned world frame for visualization."""

        if self._zed is None:
            raise RuntimeError("ZED camera is closed")
        if not self._frame_ready:
            raise RuntimeError("Call grab() successfully before get_camera_pose()")

        if self._camera_pose is None:
            self._camera_pose = sl.Pose()

        #get the transform from the  current left-camera frame into the gravity-aligned world frame
        tracking_state = self._zed.get_position(self._camera_pose, sl.REFERENCE_FRAME.WORLD)
        if tracking_state != sl.POSITIONAL_TRACKING_STATE.OK:
            return None

        translation = self._camera_pose.get_translation().get()
        orientation = self._camera_pose.get_orientation().get()
        return ZedCameraPose(position=(float(translation[0]), float(translation[1]), float(translation[2])), orientation=(float(orientation[0]), float(orientation[1]), float(orientation[2]), float(orientation[3])))

    @property
    def application_fps(self) -> float | None:
        """Latest application FPS reported directly by ZED Camera.get_current_fps()."""

        return self._current_fps

    @property
    def timing_stats(self) -> ZedTimingStats | None:
        """Rolling statistics of values reported by ZED Camera.get_current_fps()."""

        samples = self._timing_samples_fps
        count = len(samples)
        if count == 0 or self._current_fps is None:
            return None

        #mean fps
        ordered = sorted(samples)
        mean_fps = sum(ordered) / count

        middle = count // 2
        if count % 2:
            median_fps = ordered[middle]
        else:
            median_fps = 0.5 * (ordered[middle - 1] + ordered[middle])

        #5th percentile lowest fps
        p05_index = min(count - 1, max(0, math.ceil(0.05 * count) - 1))
        p05_fps = ordered[p05_index]

        return ZedTimingStats(latest_fps=self._current_fps, mean_fps=mean_fps, median_fps=median_fps, p05_fps=p05_fps, sample_count=count)

    def close(self) -> None:
        """Release all ZED resources. Calling close() more than once is safe."""

        if self._zed is None:
            return

        self._zed.disable_object_detection()
        self._zed.disable_positional_tracking()
        self._zed.close()
        self._zed = None

        #invalidate data that referred to the closed camera
        self._frame_ready = False
        self._objects_ready = False
        self._image_buffer = None
        self._intrinsics = None
        self._camera_pose = None

    def __enter__(self) -> "ZedHandler":
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.close()


# =============================================================================
# STANDALONE DEBUG PROGRAM
# =============================================================================


def parse_args():
    """Parse command-line options for running ZedHandler by itself."""

    import argparse

    parser = argparse.ArgumentParser(description="Run ZED native YOLO ball detection and print tracked detections.")
    parser.add_argument("--onnx", type=Path, required=True, help="Path to the exported YOLO ONNX model.")
    parser.add_argument("--confidence", type=float, default=DEFAULT_CONFIDENCE_PERCENT, help="Ball confidence threshold as a percentage.")
    parser.add_argument("--dynamic-input-size", type=int, help="Dynamic ONNX inference size. Try 320 or 384 for much higher FPS; ZED defaults dynamic models to 512 when unset.")
    parser.add_argument("--svo", help="Optional SVO recording instead of a live ZED camera.")
    parser.add_argument("--static-camera", action="store_true", help="Tell ZED positional tracking that the camera does not move.")
    parser.add_argument("--measure-framerate", action="store_true", help="Measure actual application frame rate through the ZED pipeline and print rolling statistics once per second.")
    parser.add_argument("--depth-stabilization", type=int, default=DEFAULT_DEPTH_STABILIZATION, help="ZED depth stabilization from 0-100. Default 0 for maximum throughput.")
    parser.add_argument("--depth-mode", choices=("neural-light", "performance"), default=DEFAULT_DEPTH_MODE, help="Depth engine. neural-light is the supported fast neural mode; performance is the deprecated legacy mode but can be faster on some GPUs.")
    parser.add_argument("--tracking-compute", choices=("auto", "cpu", "gpu"), default=DEFAULT_TRACKING_COMPUTE, help="GEN_3 positional-tracking compute preference. CPU usually leaves more GPU for YOLO/depth.")
    parser.add_argument("--display-fps", type=float, default=DEFAULT_DISPLAY_FPS, help="Debug display rate only; detection still runs on every available camera frame.")
    parser.add_argument("--print-fps", type=float, default=DEFAULT_PRINT_FPS, help="Maximum rate for printing detections; detection still runs on every available camera frame. Use 0 to disable detection printing.")
    parser.add_argument("--no-display", action="store_true", help="Do not retrieve or show the camera image.")
    return parser.parse_args()


def main() -> None:
    """Run object processing continuously while throttling terminal and display work."""

    args = parse_args()

    if args.display_fps <= 0.0:
        raise ValueError("--display-fps must be positive")
    if args.print_fps < 0.0:
        raise ValueError("--print-fps must not be negative")

    #opencv and numpy are only needed by the optional debug display
    cv2 = None
    np = None
    if not args.no_display:
        import cv2 as _cv2
        import numpy as _np
        cv2 = _cv2
        np = _np

    display_period = 1.0 / args.display_fps
    print_period = float("inf") if args.print_fps == 0.0 else 1.0 / args.print_fps
    last_display_time = float("-inf")
    last_print_time = float("-inf")
    last_timing_report_time = float("-inf")

    try:
        with ZedHandler(onnx_path=args.onnx, confidence_percent=args.confidence, dynamic_input_size=args.dynamic_input_size, svo_path=args.svo, static_camera=args.static_camera, measure_framerate=args.measure_framerate, depth_stabilization=args.depth_stabilization, tracking_compute=args.tracking_compute, depth_mode=args.depth_mode) as zed:
            print("ZED ball detection and tracking running")
            print(f"Performance config: depth={args.depth_mode} GEN_3 area-memory=off tracking-compute={args.tracking_compute} depth-stabilization={args.depth_stabilization} sdk-object-tracking=True input={args.dynamic_input_size or 'model/default'}")
            print("Press q in the display window to quit" if not args.no_display else "Press Ctrl+C to quit")

            while True:
                #grabs and processes the next available camera frame first
                result = zed.grab_and_get_objects()
                if result is None:
                    continue

                objects, timestamp_ns = result
                timestamp_seconds = timestamp_ns * 1e-9

                #only print to terminal every once in a while
                if args.print_fps > 0.0 and timestamp_seconds - last_print_time >= print_period:
                    for obj in objects:
                        x, y, z = obj.position
                        print(f"id={obj.object_id} state={obj.tracking_state} conf={obj.confidence:.2f} world=({x:.3f}, {y:.3f}, {z:.3f}) m")
                    last_print_time = timestamp_seconds

                #report the fps calculated internally by zed about once per second
                if args.measure_framerate and timestamp_seconds - last_timing_report_time >= 1.0:
                    stats = zed.timing_stats
                    if stats is not None:
                        print(f"zed fps current={stats.latest_fps:.1f}  avg={stats.mean_fps:.1f}  median={stats.median_fps:.1f}  p05={stats.p05_fps:.1f}  n={stats.sample_count}  balls={len(objects)}")
                    last_timing_report_time = timestamp_seconds

                #if no display is requested, skip image and opencv work
                if args.no_display:
                    continue

                #image retrieval and drawing run only at the requested debug display rate
                if timestamp_seconds - last_display_time < display_period:
                    continue

                assert cv2 is not None
                assert np is not None

                frame, boxes = zed.get_frame()
                objects_by_id = {obj.object_id: obj for obj in objects}

                for box in boxes:
                    points = np.asarray(box.bounding_box_2d, dtype=np.int32)
                    if points.ndim != 2 or points.shape[0] < 2:
                        continue

                    cv2.polylines(frame, [points.reshape(-1, 1, 2)], True, (0, 255, 0, 255), 2, cv2.LINE_AA)

                    ball = objects_by_id.get(box.object_id)
                    confidence = "" if ball is None else f" conf={ball.confidence:.2f}"
                    label = f"ball id={box.object_id}{confidence}"
                    x = int(points[:, 0].min())
                    y = max(20, int(points[:, 1].min()) - 8)
                    cv2.putText(frame, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0, 255), 2, cv2.LINE_AA)

                cv2.imshow("ZED tracked balls", frame)
                last_display_time = timestamp_seconds

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    finally:
        if cv2 is not None:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
