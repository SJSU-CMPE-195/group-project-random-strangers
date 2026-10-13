""" Configures the ZED SDK to use our custom YOLO detection model to grab and track dodgeballs

Model classes:
    0 = person
    1 = ball (only ball is left on for object tracking)
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl


BALL_CLASS_ID = 1
DEFAULT_CONFIDENCE_PERCENT = 25.0


def check_success(status: sl.ERROR_CODE, operation: str) -> None:
    """Raise a useful error when a ZED SDK operation fails."""

    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"{operation} failed: {status}")


def draw_balls(frame_bgr: np.ndarray, objects: sl.Objects) -> None:
    """Draw only class-1 objects returned by the ZED tracker."""

    for obj in objects.object_list:
        if int(obj.raw_label) != BALL_CLASS_ID:
            continue

        corners = np.asarray(obj.bounding_box_2d)
        if corners.shape[0] < 4:
            continue

        points = corners[:, :2].astype(np.int32)
        cv2.polylines(frame_bgr,[points], isClosed=True, color=(0, 255, 0), thickness=2)

        x = int(points[:, 0].min())
        y = max(20, int(points[:, 1].min()) - 8)
        text = (
            f"ball id={int(obj.id)} "
            f"conf={float(obj.confidence) / 100.0:.2f}"
        )

        cv2.putText(frame_bgr, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)


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
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init.depth_maximum_distance = 20

    check_success(zed.open(init), "Opening ZED camera")
    return zed


def deinitialize_camera(zed: sl.Camera) -> None:
    """Release ZED resources and close any OpenCV preview windows."""

    cv2.destroyAllWindows()
    zed.disable_object_detection()
    zed.disable_positional_tracking()
    zed.close()


'''
Running the screen capture as main:
    run:
    python screencapGrabber.py --onnx best.onnx
'''

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a custom YOLO ONNX model natively through ZED SDK 5.5+.")

    parser.add_argument("--onnx", required=True, help="Path to the exported YOLO ONNX model, for example best.onnx")

    parser.add_argument("--confidence", type=float, default=DEFAULT_CONFIDENCE_PERCENT, help="Ball confidence threshold as a percentage (default: 25)")
    
    parser.add_argument("--dynamic-input-size", type=int, metavar="SIZE", help=(
            "Input size for an ONNX model exported with dynamic=True. "
            "Must be a multiple of 32, such as 512 or 640."
        ))
    
    parser.add_argument("--no-display", action="store_true", help="Do not open an OpenCV preview window")
    
    parser.add_argument("--static-camera", action="store_true", help="Tell ZED the camera is stationary for faster/stabler tracking")

    parser.add_argument("--svo", help="Optional SVO file instead of a live ZED camera")

    return parser.parse_args()


def main() -> None:
    args = parse_args()
    onnx_path = Path(args.onnx)

    if not onnx_path.is_file():
        raise FileNotFoundError(f"ONNX model not found: {onnx_path}")

    if not 0.0 <= args.confidence <= 100.0:
        raise ValueError("--confidence must be between 0 and 100")

    if args.dynamic_input_size is not None:
        if args.dynamic_input_size <= 0 or args.dynamic_input_size % 32 != 0:
            raise ValueError("--dynamic-input-size must be a positive multiple of 32")

    zed = initialize_camera(args.svo)

    try:
        #enable object tracking
        positional_tracking = sl.PositionalTrackingParameters()
        if args.static_camera:
            positional_tracking.set_as_static = True

        check_success(zed.enable_positional_tracking(positional_tracking), "Enabling positional tracking")

        #add the model
        object_parameters = sl.ObjectDetectionParameters()
        object_parameters.detection_model = (sl.OBJECT_DETECTION_MODEL.CUSTOM_YOLOLIKE_BOX_OBJECTS)
        object_parameters.custom_onnx_file = str(onnx_path.resolve())
        object_parameters.enable_tracking = True
        object_parameters.enable_segmentation = False

        if args.dynamic_input_size is not None:
            object_parameters.custom_onnx_dynamic_input_shape = sl.Resolution(args.dynamic_input_size, args.dynamic_input_size)

        #load the model
        print("Loading and optimizing the model with ZED SDK...")
        check_success(zed.enable_object_detection(object_parameters), "Enabling native YOLO object detection")

        custom_runtime = configure_runtime_filter(args.confidence)
        objects = sl.Objects()
        runtime = sl.RuntimeParameters()
        zed_image = sl.Mat()

        print("ZED native YOLO detection and tracking are running")
        print("Class 0 (person): ignored")
        print("Class 1 (ball): displayed and printed")

        while True:
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue

            #so much easier, sorry Eric
            status = zed.retrieve_custom_objects(objects, custom_runtime)
            if status != sl.ERROR_CODE.SUCCESS:
                continue

            print_balls(objects)

            if not args.no_display:
                zed.retrieve_image(zed_image, sl.VIEW.LEFT)
                frame_bgra = zed_image.get_data()
                frame_bgr = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)
                draw_balls(frame_bgr, objects)
                cv2.imshow("ZED tracked balls", frame_bgr)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

    finally:
        deinitialize_camera(zed)


if __name__ == "__main__":
    main()
