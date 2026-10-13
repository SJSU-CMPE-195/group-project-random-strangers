""" Configures the ZED SDK to use our custom YOLO detection model to grab and track dodgeballs

Model classes:
    0 = person
    1 = ball (only ball is left on for object tracking)
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import cv2
import numpy as np
import pyzed.sl as sl

from software.Tools.onnx_optimizer import optimize_onnx_model


BALL_CLASS_ID = 1
DEFAULT_CONFIDENCE_PERCENT = 25.0


def check_success(status: sl.ERROR_CODE, operation: str) -> None:
    """Raise a useful error when a ZED SDK operation fails."""

    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"{operation} failed: {status}")


def draw_balls(frame: sl.Mat | np.ndarray, objects: sl.Objects) -> None:
    """Draw only class-1 objects returned by the ZED tracker."""

    if isinstance(frame, sl.Mat):
        frame_data = frame.get_data()
    else:
        frame_data = frame
    for obj in objects.object_list:
        if int(obj.raw_label) != BALL_CLASS_ID:
            continue

        corners = np.asarray(obj.bounding_box_2d)
        if corners.shape[0] < 4:
            continue

        points = corners[:, :2].astype(np.int32)
        cv2.polylines(frame_data,[points], isClosed=True, color=(0, 255, 0), thickness=2)

        x = int(points[:, 0].min())
        y = max(20, int(points[:, 1].min()) - 8)
        text = (
            f"ball id={int(obj.id)} "
            f"conf={float(obj.confidence) / 100.0:.2f}"
        )

        cv2.putText(frame_data, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)


def print_balls(objects: sl.Objects) -> None:
    """Print the tracked 3D state of each ball."""

    for obj in objects.object_list:
        if int(obj.raw_label) != BALL_CLASS_ID:
            continue

        position = np.asarray(obj.position, dtype=np.float32)
        velocity = np.asarray(obj.velocity, dtype=np.float32)

        print(
            f"id={int(obj.id)} "
            f"state={obj.tracking_state} "
            f"conf={float(obj.confidence) / 100.0:.2f} "
            f"position=({position[0]:.3f}, {position[1]:.3f}, "
            f"{position[2]:.3f}) m "
            f"velocity=({velocity[0]:.3f}, {velocity[1]:.3f}, "
            f"{velocity[2]:.3f}) m/s"
        )


def configure_runtime_filter(confidence_percent: float) -> sl.CustomObjectDetectionRuntimeParameters:
    """Configure the SDK's custom-detector runtime thresholds."""

    runtime = sl.CustomObjectDetectionRuntimeParameters()
    runtime.object_detection_properties.detection_confidence_threshold = (confidence_percent)

    person_properties = sl.CustomObjectDetectionProperties()
    person_properties.detection_confidence_threshold = 100.0

    ball_properties = sl.CustomObjectDetectionProperties()
    ball_properties.detection_confidence_threshold = confidence_percent

    runtime.object_class_detection_properties = {
        0: person_properties,
        1: ball_properties,
    }

    return runtime


def initialize_camera(svo_path: str | None = None) -> sl.Camera:
    """Configure and open the ZED camera or an optional SVO input."""

    zed = sl.Camera()
    input_type = sl.InputType()

    if svo_path:
        input_type.set_from_svo_file(svo_path)

    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.VGA
    init.camera_fps = 100
    init.depth_mode = sl.DEPTH_MODE.NEURAL_LIGHT
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init.depth_maximum_distance = 20

    check_success(zed.open(init), "Opening ZED camera")
    return zed


def deinitialize_camera(zed: sl.Camera) -> None:
    """Release ZED resources."""

    zed.disable_object_detection()
    zed.disable_positional_tracking()
    zed.close()


@dataclass(frozen=True)
class ZedObject3D:
    """A tracked object with a stable ID and its 3D state in meters."""

    object_id: int
    sdk_id: int
    raw_label: int
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    confidence: float
    tracking_state: object
    identity_changed: bool
    bounding_box_2d: tuple[tuple[float, float], ...]


@dataclass
class _TrackedIdentity:
    stable_id: int
    raw_label: int
    position: np.ndarray
    last_seen_frame: int
    sdk_id: int
    identity_changed: bool = False


class ZedHandler:
    """Open a ZED camera and produce tracked objects with stable IDs.

    ``object_id`` is assigned by this handler and remains stable when the ZED
    SDK changes an object's native ID during a short detection gap. If a
    native ID is reused for a different object, the handler assigns a new
    ``object_id`` and sets ``identity_changed`` to ``True`` for that object.

    Example:
        handler = ZedHandler("best.onnx")
        try:
            while True:
                if not handler.grab():
                    continue
                objects, frame_timestamp_ns = handler.get_objects()
                for obj in objects:
                    print(obj.object_id, obj.position, obj.identity_changed)
                print(f"frame timestamp: {frame_timestamp_ns} ns")
        finally:
            handler.close()
    """

    def __init__(
        self,
        onnx_path: str | Path,
        confidence_percent: float = DEFAULT_CONFIDENCE_PERCENT,
        dynamic_input_size: int | None = None,
        svo_path: str | None = None,
        static_camera: bool = False,
        max_missing_frames: int = 5,
        max_reassociation_distance_m: float = 2.0,
        measure_processing_time: bool = False,
    ) -> None:
        model_path = Path(onnx_path)
        if not model_path.is_file():
            raise FileNotFoundError(f"ONNX model not found: {model_path}")
        if not 0.0 <= confidence_percent <= 100.0:
            raise ValueError("confidence_percent must be between 0 and 100")
        if dynamic_input_size is not None and (
            dynamic_input_size <= 0 or dynamic_input_size % 32 != 0
        ):
            raise ValueError("dynamic_input_size must be a positive multiple of 32")
        if max_missing_frames < 0:
            raise ValueError("max_missing_frames must not be negative")
        if max_reassociation_distance_m <= 0:
            raise ValueError("max_reassociation_distance_m must be positive")

        self._confidence_percent = confidence_percent
        self._max_missing_frames = max_missing_frames
        self._max_reassociation_distance_m = max_reassociation_distance_m
        self._measure_processing_time = measure_processing_time
        self._last_processing_time_ns: int | None = None
        self._identities: dict[int, _TrackedIdentity] = {}
        self._next_stable_id = 0
        self._frame_number = 0
        self._objects = sl.Objects()
        self._runtime = sl.RuntimeParameters()
        self._custom_runtime = configure_runtime_filter(confidence_percent)
        self._frame_ready = False
        self._frame_timestamp_ns: int | None = None
        self._zed = initialize_camera(svo_path)

        try:
            positional_tracking = sl.PositionalTrackingParameters()
            if static_camera:
                positional_tracking.set_as_static = True
            check_success(
                self._zed.enable_positional_tracking(positional_tracking),
                "Enabling positional tracking",
            )

            object_parameters = sl.ObjectDetectionParameters()
            object_parameters.detection_model = (
                sl.OBJECT_DETECTION_MODEL.CUSTOM_YOLOLIKE_BOX_OBJECTS
            )
            object_parameters.custom_onnx_file = str(model_path.resolve())
            object_parameters.enable_tracking = True
            object_parameters.enable_segmentation = False
            if dynamic_input_size is not None:
                object_parameters.custom_onnx_dynamic_input_shape = sl.Resolution(
                    dynamic_input_size, dynamic_input_size
                )

            check_success(
                self._zed.enable_object_detection(object_parameters),
                "Enabling native YOLO object detection",
            )
        except Exception:
            deinitialize_camera(self._zed)
            raise

    def grab(self) -> bool:
        """Acquire the next camera frame without retrieving image pixels.

        Call :method:`get_objects` after a successful grab. Image data is not
        copied or converted by this method.
        """
        if self._zed is None:
            raise RuntimeError("ZED camera is closed")

        status = self._zed.grab(self._runtime)
        self._frame_ready = status == sl.ERROR_CODE.SUCCESS
        if self._frame_ready:
            self._frame_timestamp_ns = int(
                self._zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
            )
        return self._frame_ready

    def get_objects(self) -> tuple[list[ZedObject3D], int]:
        """Return objects and the image timestamp from the latest grabbed frame.

        The timestamp is returned as nanoseconds from the ZED SDK image clock.
        It identifies when the frame was captured, rather than when object
        retrieval completed.
        """

        if self._zed is None:
            raise RuntimeError("ZED camera is closed")
        if not self._frame_ready:
            raise RuntimeError("Call grab() successfully before get_objects()")
        if self._measure_processing_time:
            start_time_ns = time.perf_counter_ns()
            status = self._zed.retrieve_custom_objects(
                self._objects, self._custom_runtime
            )
            self._last_processing_time_ns = time.perf_counter_ns() - start_time_ns
        else:
            status = self._zed.retrieve_custom_objects(
                self._objects, self._custom_runtime
            )
            self._last_processing_time_ns = None
        check_success(status, "Retrieving ZED objects")
        self._frame_number += 1

        results: list[ZedObject3D] = []
        used_stable_ids: set[int] = set()
        for obj in self._objects.object_list:
            sdk_id = int(obj.id)
            raw_label = int(obj.raw_label)
            position = np.asarray(obj.position, dtype=np.float32)
            velocity = np.asarray(obj.velocity, dtype=np.float32)

            identity = self._get_identity(
                sdk_id, raw_label, position, used_stable_ids
            )
            used_stable_ids.add(identity.stable_id)
            identity.position = position.copy()
            identity.last_seen_frame = self._frame_number
            identity.sdk_id = sdk_id

            position_3d = (
                float(position[0]),
                float(position[1]),
                float(position[2]),
            )
            velocity_3d = (
                float(velocity[0]),
                float(velocity[1]),
                float(velocity[2]),
            )
            corners = np.asarray(obj.bounding_box_2d, dtype=np.float32)
            bounding_box_2d = tuple(
                (float(point[0]), float(point[1])) for point in corners[:, :2]
            )

            results.append(
                ZedObject3D(
                    object_id=identity.stable_id,
                    sdk_id=sdk_id,
                    raw_label=raw_label,
                    position=position_3d,
                    velocity=velocity_3d,
                    confidence=float(obj.confidence) / 100.0,
                    tracking_state=obj.tracking_state,
                    identity_changed=identity.identity_changed,
                    bounding_box_2d=bounding_box_2d,
                )
            )
            identity.identity_changed = False

        if self._frame_timestamp_ns is None:
            raise RuntimeError("Frame timestamp is unavailable")
        return results, self._frame_timestamp_ns

    @property
    def last_processing_time_seconds(self) -> float | None:
        """Return the latest object-detection processing time in seconds."""

        if self._last_processing_time_ns is None:
            return None
        
        return self._last_processing_time_ns / 1_000_000_000

    def grab_and_get_objects(
        self,
    ) -> tuple[list[ZedObject3D], int] | None:
        """Acquire a frame and return its objects and timestamp, or ``None``."""

        if not self.grab():
            return None
        return self.get_objects()

    def grab_and_get_objects_blocking(self) -> tuple[list[ZedObject3D], int]:
        """Wait for a frame, then return its objects and image timestamp."""

        while not self.grab():
            if self._zed is None:
                raise RuntimeError("ZED camera is closed")
        return self.get_objects()

    def get_frame(
        self,
        image_buffer: sl.Mat | None = None,
        camera_side: Literal["left", "right"] = "left",
    ) -> np.ndarray:
        """Retrieve the latest camera frame as a BGR NumPy array.

        This performs image retrieval and BGRA-to-BGR conversion only when
        called. It does not acquire another camera frame. If ``image_buffer``
        is supplied, the ZED SDK fills that persistent ``sl.Mat`` instead of
        allocating a temporary image buffer.
        """

        if self._zed is None:
            raise RuntimeError("ZED camera is closed")
        if not self._frame_ready:
            raise RuntimeError("Call grab() successfully before get_frame()")

        zed_image = image_buffer if image_buffer is not None else sl.Mat()
        check_success(
            self._zed.retrieve_image(
                zed_image,
                sl.VIEW.LEFT if camera_side == "left" else sl.VIEW.RIGHT,
            ),
            f"Retrieving {camera_side} camera image",
        )
        frame_bgra = zed_image.get_data()
        frame_bgr = frame_bgra[..., :3]
        return frame_bgr

    def _get_identity(
        self,
        sdk_id: int,
        raw_label: int,
        position: np.ndarray,
        used_stable_ids: set[int],
    ) -> _TrackedIdentity:
        identity = self._identities.get(sdk_id)
        previous_identity = identity
        native_id_was_reused = identity is not None
        if identity is not None:
            gap = self._frame_number - identity.last_seen_frame
            distance = float(np.linalg.norm(position - identity.position))
            if (
                gap <= self._max_missing_frames
                and distance <= self._max_reassociation_distance_m
                and raw_label == identity.raw_label
                and identity.stable_id not in used_stable_ids
            ):
                return identity

            identity = None

        candidates = [
            candidate
            for candidate in self._identities.values()
            if candidate is not previous_identity
            if candidate.stable_id not in used_stable_ids
            and candidate.raw_label == raw_label
            and self._frame_number - candidate.last_seen_frame
            <= self._max_missing_frames
            and float(np.linalg.norm(position - candidate.position))
            <= self._max_reassociation_distance_m
        ]
        if candidates:
            identity = min(
                candidates,
                key=lambda candidate: float(
                    np.linalg.norm(position - candidate.position)
                ),
            )
            identity.identity_changed = (
                native_id_was_reused or identity.sdk_id == sdk_id
            )
            self._identities[sdk_id] = identity
            return identity

        identity = _TrackedIdentity(
            stable_id=self._next_stable_id,
            raw_label=raw_label,
            position=position.copy(),
            last_seen_frame=self._frame_number,
            sdk_id=sdk_id,
            identity_changed=native_id_was_reused,
        )
        self._next_stable_id += 1
        self._identities[sdk_id] = identity
        return identity

    def close(self) -> None:
        """Disable ZED tracking and release the camera."""

        if self._zed is not None:
            deinitialize_camera(self._zed)
            self._zed = None
            self._frame_ready = False

    def __enter__(self) -> ZedHandler:
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.close()


    '''
    Running the screen capture as main:
    run:
    python screencapGrabber.py --onnx best.onnx
'''

def draw_objects(frame: sl.Mat | np.ndarray, objects: list[ZedObject3D]) -> None:
    """Draw ball detections directly onto an ``sl.Mat`` or NumPy frame."""

    frame_data = frame.get_data() if isinstance(frame, sl.Mat) else frame
    for obj in objects:
        if obj.raw_label != BALL_CLASS_ID:
            continue

        corners = np.asarray(obj.bounding_box_2d, dtype=np.int32)
        if corners.shape[0] < 4:
            continue
        cv2.polylines(
            frame_data,
            [corners],
            isClosed=True,
            color=(0, 255, 0),
            thickness=2,
        )
        x = int(corners[:, 0].min())
        y = int(corners[:, 1].min())
        cv2.putText(
            frame_data,
            f"ball id={obj.object_id} conf={obj.confidence:.2f}",
            (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )


def print_objects(objects: list[ZedObject3D]) -> None:
    """Print the tracked 3D state of each object."""

    for obj in objects:
        print(
            f"id={obj.object_id} "
            f"sdk_id={obj.sdk_id} "
            f"identity_changed={obj.identity_changed} "
            f"state={obj.tracking_state} "
            f"conf={obj.confidence:.2f} "
            f"position=({obj.position[0]:.3f}, {obj.position[1]:.3f}, "
            f"{obj.position[2]:.3f}) m "
            f"velocity=({obj.velocity[0]:.3f}, {obj.velocity[1]:.3f}, "
            f"{obj.velocity[2]:.3f}) m/s"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a custom YOLO ONNX model natively through ZED SDK 5.5+.")

    parser.add_argument("--onnx", required=True, help="Path to the exported YOLO ONNX model, for example best.onnx")

    parser.add_argument(
        "--optimize-onnx",
        action="store_true",
        help="Simplify the ONNX model once and reuse <name>.optimized.onnx on later runs",
    )

    parser.add_argument("--confidence", type=float, default=DEFAULT_CONFIDENCE_PERCENT, help="Ball confidence threshold as a percentage (default: 25)")
    
    parser.add_argument("--dynamic-input-size", type=int, metavar="SIZE", help=(
            "Input size for an ONNX model exported with dynamic=True. "
            "Must be a multiple of 32, such as 512 or 640."
        ))
    
    parser.add_argument("--no-display", action="store_true", help="Do not open an OpenCV preview window")
    
    parser.add_argument("--static-camera", action="store_true", help="Tell ZED the camera is stationary for faster/stabler tracking")

    parser.add_argument("--svo", help="Optional SVO file instead of a live ZED camera")

    parser.add_argument(
        "--measure-processing-time",
        action="store_true",
        help="Measure and print custom object-detection processing time",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()
    onnx_path = Path(args.onnx)

    if not onnx_path.is_file():
        raise FileNotFoundError(f"ONNX model not found: {onnx_path}")

    if args.optimize_onnx:
        onnx_path = optimize_onnx_model(onnx_path)
        print(f"Using optimized ONNX model: {onnx_path}")

    if not 0.0 <= args.confidence <= 100.0:
        raise ValueError("--confidence must be between 0 and 100")

    if args.dynamic_input_size is not None:
        if args.dynamic_input_size <= 0 or args.dynamic_input_size % 32 != 0:
            raise ValueError("--dynamic-input-size must be a positive multiple of 32")

    try:
        with ZedHandler(
            onnx_path=onnx_path,
            confidence_percent=args.confidence,
            dynamic_input_size=args.dynamic_input_size,
            svo_path=args.svo,
            static_camera=args.static_camera,
            measure_processing_time=args.measure_processing_time,
        ) as zed:
            print("ZED native YOLO detection and tracking are running")
            print("Class 0 (person): ignored")
            print("Class 1 (ball): displayed and printed")

            image_buffer = sl.Mat()

            while True:
                if not zed.grab():
                    continue

                objects, frame_timestamp_ns = zed.get_objects()
                print_objects(objects)
                print(f"frame timestamp: {frame_timestamp_ns} ns")
                if args.measure_processing_time:
                    processing_time_seconds = zed.last_processing_time_seconds
                    if processing_time_seconds is not None:
                        print(
                            "object detection processing time: "
                            f"{processing_time_seconds * 1000:.3f} ms"
                        )

                if not args.no_display:
                    zed.get_frame(image_buffer=image_buffer)
                    draw_objects(image_buffer, objects)
                    cv2.imshow("ZED tracked balls", image_buffer.get_data())

                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
    finally:
        if not args.no_display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
