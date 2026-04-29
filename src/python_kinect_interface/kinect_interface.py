from __future__ import annotations

import argparse
import ctypes
from pathlib import Path


KINECT_JOINT_COUNT = 25
KINECT_MAX_BODIES = 6

S_OK = 0

DLL_PATH = Path(__file__).resolve().parent / "cpp_wrapper" / "kinect_wrapper.dll"

class Coordinate3DC(ctypes.Structure):
	_fields_ = [
        ("x", ctypes.c_float), 
        ("y", ctypes.c_float), 
        ("z", ctypes.c_float)]


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
	]


class BodyDataC(ctypes.Structure):
	_fields_ = [
		("body_id", ctypes.c_uint64),
		("joints", JointC * KINECT_JOINT_COUNT),
		("orientations", QuaternionC * KINECT_JOINT_COUNT),
		("timestamp", ctypes.c_int64),
	]


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

	library.kinect_destroy.restype = ctypes.c_long
	library.kinect_destroy.argtypes = [ctypes.c_void_p]


class KinectSensor:
	def __init__(self) -> None:
		self._lib = _load_kinect_library()
		self._handle = self._lib.kinect_create()
		if not self._handle:
			raise RuntimeError("kinect_create failed")

	def initialize(self) -> None:
		result = self._lib.kinect_initialize(self._handle)
		if result != S_OK:
			raise RuntimeError(f"kinect_initialize failed with HRESULT {result}")

	def wait_for_frame(self, timeout_ms: int = 1000) -> None:
		result = self._lib.kinect_wait_for_frame(self._handle, ctypes.c_uint32(timeout_ms))
		if result != S_OK:
			raise RuntimeError(f"kinect_wait_for_frame failed with HRESULT {result}")

	def get_latest_bodies(self) -> list[BodyDataC]:
		bodies = (BodyDataC * KINECT_MAX_BODIES)()
		count = ctypes.c_int()

		result = self._lib.kinect_get_latest_bodies(
			self._handle,
			bodies,
			KINECT_MAX_BODIES,
			ctypes.byref(count),
		)
		if result != S_OK:
			raise RuntimeError(f"kinect_get_latest_bodies failed with HRESULT {result}")

		return list(bodies[: count.value])

	def close(self) -> None:
		if getattr(self, "_handle", None):
			self._lib.kinect_close(self._handle)
			self._lib.kinect_destroy(self._handle)
			self._handle = None

	def __enter__(self) -> "KinectSensor":
		return self

	def __exit__(self, exc_type, exc, tb) -> None:
		self.close()


def print_frame(bodies: list[BodyDataC]) -> None:
	if not bodies:
		print("No bodies detected")
		return

	for body in bodies:
		print(f"Body {body.body_id} timestamp={body.timestamp}")
		for index, joint in enumerate(body.joints):
			print(
				f"  Joint {index:02d}: "
				f"state={joint.tracking_state} "
				f"pos=({joint.position.x:.3f}, {joint.position.y:.3f}, {joint.position.z:.3f})"
			)


if __name__ == "__main__":
	parser = argparse.ArgumentParser(description="Read Kinect body frames through the C++ wrapper")
	parser.add_argument("--timeout-ms", type=int, default=1000, help="Frame wait timeout in milliseconds")
	args = parser.parse_args()

	with KinectSensor() as sensor:
		sensor.initialize()
		print("Kinect initialized. Press Ctrl+C to stop.")

		try:
			while True:
				sensor.wait_for_frame(args.timeout_ms)
				bodies = sensor.get_latest_bodies()
				print_frame(bodies)
				print("-" * 60)
		except KeyboardInterrupt:
			pass
