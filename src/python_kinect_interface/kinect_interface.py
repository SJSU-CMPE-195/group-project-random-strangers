from __future__ import annotations

import argparse
import ctypes
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


KINECT_JOINT_COUNT = 25
KINECT_MAX_BODIES = 6
KINECT_COLOR_WIDTH = 1920
KINECT_COLOR_HEIGHT = 1080
KINECT_COLOR_BYTES_PER_PIXEL = 4

S_OK = 0
WAIT_OBJECT_0 = 0
WAIT_TIMEOUT = 0x00000102
E_PENDING = -2147483638  # 0x8000000A as a signed 32-bit HRESULT.

DEFAULT_DLL_PATH = Path(__file__).resolve().parent / "cpp_wrapper" / "kinect_wrapper.dll"
DLL_PATH = Path(os.environ.get("KINECT_WRAPPER_DLL", DEFAULT_DLL_PATH))


def _signed32(value: int) -> int:
    value &= 0xFFFFFFFF
    if value & 0x80000000:
        return value - 0x100000000
    return value


def _hr_hex(value: int) -> str:
    return f"0x{int(value) & 0xFFFFFFFF:08X}"


def _raise_hresult(name: str, result: int) -> None:
    raise RuntimeError(f"{name} failed with HRESULT {_hr_hex(result)} ({result})")


class Coordinate2DC(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
    ]


class Coordinate3DC(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
    ]


class QuaternionC(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("w", ctypes.c_float),
    ]


class JointC(ctypes.Structure):
    _fields_ = [
        ("joint_type", ctypes.c_int32),
        ("tracking_state", ctypes.c_int32),
        ("position", Coordinate3DC),
        ("orientation", QuaternionC),
        ("color_position", Coordinate2DC),
    ]


class BodyDataC(ctypes.Structure):
    _fields_ = [
        ("body_id", ctypes.c_uint64),
        ("joints", JointC * KINECT_JOINT_COUNT),
        ("orientations", QuaternionC * KINECT_JOINT_COUNT),
        ("timestamp", ctypes.c_int64),
    ]


@dataclass(slots=True)
class ColorFrame:
    width: int
    height: int
    bytes_per_pixel: int
    timestamp: int
    data: bytes  # BGRA bytes. Use Qt QImage Format_ARGB32 on little-endian Windows.


@dataclass(slots=True)
class BodyFrame:
    raw_bodies: list[BodyDataC]
    normalized_bodies: list[BodyDataC]


def _optional_function(library: ctypes.CDLL, name: str):
    try:
        return getattr(library, name)
    except AttributeError:
        return None


def _load_kinect_library() -> ctypes.CDLL:
    if not DLL_PATH.exists():
        raise FileNotFoundError(f"Could not find kinect_wrapper.dll at {DLL_PATH}")

    library = ctypes.CDLL(str(DLL_PATH))
    _configure_library(library)
    return library


def _configure_library(library: ctypes.CDLL) -> None:
    library.kinect_create.restype = ctypes.c_void_p
    library.kinect_create.argtypes = []

    library.kinect_initialize.restype = ctypes.c_long
    library.kinect_initialize.argtypes = [ctypes.c_void_p]

    library.kinect_close.restype = ctypes.c_long
    library.kinect_close.argtypes = [ctypes.c_void_p]

    library.kinect_wait_for_frame.restype = ctypes.c_long
    library.kinect_wait_for_frame.argtypes = [ctypes.c_void_p, ctypes.c_uint32]

    library.kinect_get_latest_bodies.restype = ctypes.c_long
    library.kinect_get_latest_bodies.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(BodyDataC),
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_int),
    ]

    normalized_fn = _optional_function(library, "kinect_get_latest_bodies_normalized")
    library._has_normalized_bodies = normalized_fn is not None
    if normalized_fn is not None:
        normalized_fn.restype = ctypes.c_long
        normalized_fn.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(BodyDataC),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
        ]

    body_frame_fn = _optional_function(library, "kinect_get_latest_body_frame")
    library._has_body_frame = body_frame_fn is not None
    if body_frame_fn is not None:
        body_frame_fn.restype = ctypes.c_long
        body_frame_fn.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(BodyDataC),
            ctypes.POINTER(BodyDataC),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
        ]

    size_fn = _optional_function(library, "kinect_get_color_frame_size")
    library._has_color_frame_size = size_fn is not None
    if size_fn is not None:
        size_fn.restype = ctypes.c_long
        size_fn.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
        ]

    color_fn = _optional_function(library, "kinect_get_latest_color_frame")
    library._has_color_frame = color_fn is not None
    if color_fn is not None:
        color_fn.restype = ctypes.c_long
        color_fn.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int64),
        ]

    library.kinect_destroy.restype = ctypes.c_long
    library.kinect_destroy.argtypes = [ctypes.c_void_p]


class KinectSensor:
    def __init__(self) -> None:
        self._lib = _load_kinect_library()
        self._handle = self._lib.kinect_create()
        self._color_buffer = None
        self._color_buffer_size = 0
        self._color_width = KINECT_COLOR_WIDTH
        self._color_height = KINECT_COLOR_HEIGHT
        self._color_bytes_per_pixel = KINECT_COLOR_BYTES_PER_PIXEL

        if not self._handle:
            raise RuntimeError("kinect_create failed")

    def initialize(self) -> None:
        result = _signed32(self._lib.kinect_initialize(self._handle))
        if result != S_OK:
            _raise_hresult("kinect_initialize", result)

        # Refresh real dimensions after initialization. Kinect v2 should be 1920x1080x4.
        try:
            self._color_width, self._color_height, self._color_bytes_per_pixel = self.get_color_frame_size()
        except Exception:
            self._color_width = KINECT_COLOR_WIDTH
            self._color_height = KINECT_COLOR_HEIGHT
            self._color_bytes_per_pixel = KINECT_COLOR_BYTES_PER_PIXEL

    def wait_for_frame(self, timeout_ms: int = 1000) -> bool:
        """Wait for a body frame event.

        Returns True when a body frame is available and False on timeout.
        Raises RuntimeError for actual Kinect/API errors.
        """
        result = _signed32(self._lib.kinect_wait_for_frame(self._handle, ctypes.c_uint32(timeout_ms)))
        if result == WAIT_OBJECT_0:
            return True
        if result == WAIT_TIMEOUT:
            return False
        _raise_hresult("kinect_wait_for_frame", result)
        return False

    def get_latest_bodies(self) -> list[BodyDataC]:
        bodies = (BodyDataC * KINECT_MAX_BODIES)()
        count = ctypes.c_int()

        result = _signed32(
            self._lib.kinect_get_latest_bodies(
                self._handle,
                bodies,
                KINECT_MAX_BODIES,
                ctypes.byref(count),
            )
        )
        if result != S_OK:
            _raise_hresult("kinect_get_latest_bodies", result)

        return list(bodies[: count.value])

    def get_latest_bodies_normalized(self) -> list[BodyDataC]:
        if not getattr(self._lib, "_has_normalized_bodies", False):
            raise RuntimeError("The loaded DLL does not export kinect_get_latest_bodies_normalized. Rebuild the wrapper.")

        bodies = (BodyDataC * KINECT_MAX_BODIES)()
        count = ctypes.c_int()

        result = _signed32(
            self._lib.kinect_get_latest_bodies_normalized(
                self._handle,
                bodies,
                KINECT_MAX_BODIES,
                ctypes.byref(count),
            )
        )
        if result != S_OK:
            _raise_hresult("kinect_get_latest_bodies_normalized", result)

        return list(bodies[: count.value])

    def get_latest_body_frame(self) -> BodyFrame:
        """Get raw and normalized bodies from the same Kinect body frame."""
        if getattr(self._lib, "_has_body_frame", False):
            raw_bodies = (BodyDataC * KINECT_MAX_BODIES)()
            normalized_bodies = (BodyDataC * KINECT_MAX_BODIES)()
            count = ctypes.c_int()

            result = _signed32(
                self._lib.kinect_get_latest_body_frame(
                    self._handle,
                    raw_bodies,
                    normalized_bodies,
                    KINECT_MAX_BODIES,
                    ctypes.byref(count),
                )
            )
            if result != S_OK:
                _raise_hresult("kinect_get_latest_body_frame", result)

            return BodyFrame(
                raw_bodies=list(raw_bodies[: count.value]),
                normalized_bodies=list(normalized_bodies[: count.value]),
            )

        # Fallback for an older DLL. This can acquire two different frames, so rebuild the
        # wrapper for exact synchronization between raw and normalized bodies.
        return BodyFrame(
            raw_bodies=self.get_latest_bodies(),
            normalized_bodies=self.get_latest_bodies_normalized(),
        )

    def get_color_frame_size(self) -> tuple[int, int, int]:
        if not getattr(self._lib, "_has_color_frame_size", False):
            return (KINECT_COLOR_WIDTH, KINECT_COLOR_HEIGHT, KINECT_COLOR_BYTES_PER_PIXEL)

        width = ctypes.c_int()
        height = ctypes.c_int()
        bytes_per_pixel = ctypes.c_int()
        result = _signed32(
            self._lib.kinect_get_color_frame_size(
                self._handle,
                ctypes.byref(width),
                ctypes.byref(height),
                ctypes.byref(bytes_per_pixel),
            )
        )
        if result != S_OK:
            _raise_hresult("kinect_get_color_frame_size", result)

        return (width.value, height.value, bytes_per_pixel.value)

    def get_latest_color_frame(self) -> Optional[ColorFrame]:
        if not getattr(self._lib, "_has_color_frame", False):
            raise RuntimeError("The loaded DLL does not export kinect_get_latest_color_frame. Rebuild the wrapper.")

        width = ctypes.c_int(self._color_width)
        height = ctypes.c_int(self._color_height)
        timestamp = ctypes.c_int64()
        required_size = self._color_width * self._color_height * self._color_bytes_per_pixel

        if self._color_buffer is None or self._color_buffer_size != required_size:
            self._color_buffer = (ctypes.c_uint8 * required_size)()
            self._color_buffer_size = required_size

        result = _signed32(
            self._lib.kinect_get_latest_color_frame(
                self._handle,
                self._color_buffer,
                required_size,
                ctypes.byref(width),
                ctypes.byref(height),
                ctypes.byref(timestamp),
            )
        )

        if result == E_PENDING:
            return None
        if result != S_OK:
            _raise_hresult("kinect_get_latest_color_frame", result)

        frame_size = width.value * height.value * self._color_bytes_per_pixel
        data = ctypes.string_at(self._color_buffer, frame_size)
        return ColorFrame(
            width=width.value,
            height=height.value,
            bytes_per_pixel=self._color_bytes_per_pixel,
            timestamp=timestamp.value,
            data=data,
        )

    def close(self) -> None:
        if getattr(self, "_handle", None):
            self._lib.kinect_close(self._handle)
            self._lib.kinect_destroy(self._handle)
            self._handle = None

    def __enter__(self) -> "KinectSensor":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


JOINT_NAMES = [
    "SpineBase",
    "SpineMid",
    "Neck",
    "Head",
    "ShoulderLeft",
    "ElbowLeft",
    "WristLeft",
    "HandLeft",
    "ShoulderRight",
    "ElbowRight",
    "WristRight",
    "HandRight",
    "HipLeft",
    "KneeLeft",
    "AnkleLeft",
    "FootLeft",
    "HipRight",
    "KneeRight",
    "AnkleRight",
    "FootRight",
    "SpineShoulder",
    "HandTipLeft",
    "ThumbLeft",
    "HandTipRight",
    "ThumbRight",
]


def print_frame(bodies: list[BodyDataC], *, normalized: bool = False) -> None:
    if not bodies:
        print("No bodies detected")
        return

    label = "normalized" if normalized else "raw"
    for body in bodies:
        print(f"Body {body.body_id} ({label}) timestamp={body.timestamp}")
        for index, joint in enumerate(body.joints):
            print(
                f"  {index:02d} {JOINT_NAMES[index]:>14}: "
                f"state={joint.tracking_state} "
                f"pos=({joint.position.x:.3f}, {joint.position.y:.3f}, {joint.position.z:.3f}) "
                f"color=({joint.color_position.x:.1f}, {joint.color_position.y:.1f})"
            )


def _run_cli() -> None:
    parser = argparse.ArgumentParser(description="Read Kinect body/color frames through the C++ wrapper")
    parser.add_argument("--timeout-ms", type=int, default=1000, help="Frame wait timeout in milliseconds")
    parser.add_argument("--normalized", action="store_true", help="Print normalized body coordinates")
    parser.add_argument("--color", action="store_true", help="Also acquire and print color frame metadata")
    args = parser.parse_args()

    with KinectSensor() as sensor:
        sensor.initialize()
        print("Kinect initialized. Press Ctrl+C to stop.")

        try:
            while True:
                if not sensor.wait_for_frame(args.timeout_ms):
                    print("Timed out waiting for body frame")
                    continue

                body_frame = sensor.get_latest_body_frame()
                bodies = body_frame.normalized_bodies if args.normalized else body_frame.raw_bodies
                print_frame(bodies, normalized=args.normalized)

                if args.color:
                    color = sensor.get_latest_color_frame()
                    if color is None:
                        print("Color frame: pending")
                    else:
                        print(f"Color frame: {color.width}x{color.height} timestamp={color.timestamp} bytes={len(color.data)}")

                print("-" * 60)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    _run_cli()
