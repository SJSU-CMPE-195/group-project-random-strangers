from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# Add src directory to path so we can import python_kinect_interface
sys.path.insert(0, str(Path(__file__).parent.parent))

from PySide6 import QtCore, QtGui, QtWidgets
Signal = QtCore.Signal
Slot = QtCore.Slot
QT_API = "PySide6"

from python_kinect_interface.kinect_interface import BodyDataC, ColorFrame, JOINT_NAMES, KinectSensor

TRACKING_NOT_TRACKED = 0
TRACKING_INFERRED = 1
TRACKING_TRACKED = 2

# Kinect v2 joint connectivity.
BONES = [
    (0, 1), (1, 20), (20, 2), (2, 3),
    (20, 4), (4, 5), (5, 6), (6, 7), (7, 21), (6, 22),
    (20, 8), (8, 9), (9, 10), (10, 11), (11, 23), (10, 24),
    (0, 12), (12, 13), (13, 14), (14, 15),
    (0, 16), (16, 17), (17, 18), (18, 19),
]

BODY_COLORS = [
    QtGui.QColor(0, 220, 255),
    QtGui.QColor(255, 210, 0),
    QtGui.QColor(80, 255, 120),
    QtGui.QColor(255, 120, 220),
    QtGui.QColor(255, 120, 80),
    QtGui.QColor(180, 150, 255),
]


def _qt_enum(owner, group_name: str, value_name: str):
    group = getattr(owner, group_name, None)
    if group is not None and hasattr(group, value_name):
        return getattr(group, value_name)
    return getattr(owner, value_name)


ALIGN_CENTER = _qt_enum(QtCore.Qt, "AlignmentFlag", "AlignCenter")
ALIGN_LEFT = _qt_enum(QtCore.Qt, "AlignmentFlag", "AlignLeft")
ALIGN_VCENTER = _qt_enum(QtCore.Qt, "AlignmentFlag", "AlignVCenter")
HORIZONTAL = _qt_enum(QtCore.Qt, "Orientation", "Horizontal")
DASH_LINE = _qt_enum(QtCore.Qt, "PenStyle", "DashLine")
DOT_LINE = _qt_enum(QtCore.Qt, "PenStyle", "DotLine")
SOLID_LINE = _qt_enum(QtCore.Qt, "PenStyle", "SolidLine")
ANTIALIASING = _qt_enum(QtGui.QPainter, "RenderHint", "Antialiasing")
QIMAGE_FORMAT_ARGB32 = _qt_enum(QtGui.QImage, "Format", "Format_ARGB32")


@dataclass(slots=True)
class KinectUiFrame:
    color: Optional[ColorFrame]
    raw_bodies: list[BodyDataC]
    normalized_bodies: list[BodyDataC]


def _body_color(index: int) -> QtGui.QColor:
    return BODY_COLORS[index % len(BODY_COLORS)]


def _finite(value: float) -> bool:
    return math.isfinite(float(value))


def _joint_has_position(joint) -> bool:
    return joint.tracking_state != TRACKING_NOT_TRACKED and all(
        _finite(v) for v in (joint.position.x, joint.position.y, joint.position.z)
    )


def _joint_has_color_position(joint, width: int, height: int) -> bool:
    if joint.tracking_state == TRACKING_NOT_TRACKED:
        return False
    x = float(joint.color_position.x)
    y = float(joint.color_position.y)
    return _finite(x) and _finite(y) and 0.0 <= x < width and 0.0 <= y < height


def _aspect_fit_rect(source_w: int, source_h: int, target: QtCore.QRectF) -> QtCore.QRectF:
    if source_w <= 0 or source_h <= 0 or target.width() <= 0 or target.height() <= 0:
        return QtCore.QRectF(target)

    scale = min(target.width() / source_w, target.height() / source_h)
    width = source_w * scale
    height = source_h * scale
    left = target.x() + (target.width() - width) * 0.5
    top = target.y() + (target.height() - height) * 0.5
    return QtCore.QRectF(left, top, width, height)


def _body_tooltip(body: BodyDataC) -> str:
    lines = [f"Body {body.body_id} normalized joint positions (meters)"]
    for index, joint in enumerate(body.joints):
        if joint.tracking_state == TRACKING_NOT_TRACKED:
            continue
        lines.append(
            f"{index:02d} {JOINT_NAMES[index]}: "
            f"x={joint.position.x:+.3f}, y={joint.position.y:+.3f}, z={joint.position.z:+.3f}"
        )
    return "\n".join(lines)


class KinectWorker(QtCore.QThread):
    frame_ready = Signal(object)
    status = Signal(str)
    error = Signal(str)

    def __init__(self, timeout_ms: int = 33, parent=None) -> None:
        super().__init__(parent)
        self.timeout_ms = timeout_ms
        self._running = True

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        sensor: Optional[KinectSensor] = None
        last_color: Optional[ColorFrame] = None
        color_supported = True

        try:
            sensor = KinectSensor()
            sensor.initialize()
            self.status.emit("Kinect initialized")

            while self._running:
                try:
                    if not sensor.wait_for_frame(self.timeout_ms):
                        continue

                    body_frame = sensor.get_latest_body_frame()

                    if color_supported:
                        try:
                            color = sensor.get_latest_color_frame()
                            if color is not None:
                                last_color = color
                        except Exception as exc:
                            color_supported = False
                            self.status.emit(f"Color frames disabled: {exc}")

                    self.frame_ready.emit(
                        KinectUiFrame(
                            color=last_color,
                            raw_bodies=body_frame.raw_bodies,
                            normalized_bodies=body_frame.normalized_bodies,
                        )
                    )
                except Exception as exc:
                    # Keep the UI alive through transient AcquireLatestFrame races.
                    self.error.emit(str(exc))
                    self.msleep(50)

        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            if sensor is not None:
                sensor.close()
            self.status.emit("Kinect stopped")


class SceneView(QtWidgets.QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._color: Optional[ColorFrame] = None
        self._bodies: list[BodyDataC] = []
        self.setMinimumSize(640, 360)

    def set_frame(self, color: Optional[ColorFrame], bodies: list[BodyDataC]) -> None:
        self._color = color
        self._bodies = bodies
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt override name.
        painter = QtGui.QPainter(self)
        painter.setRenderHint(ANTIALIASING, True)
        painter.fillRect(self.rect(), QtGui.QColor(18, 18, 20))

        if self._color is None:
            painter.setPen(QtGui.QColor(230, 230, 230))
            painter.drawText(QtCore.QRectF(self.rect()), ALIGN_CENTER, "Waiting for Kinect RGB frame...")
            return

        image = QtGui.QImage(
            self._color.data,
            self._color.width,
            self._color.height,
            self._color.width * self._color.bytes_per_pixel,
            QIMAGE_FORMAT_ARGB32,
        )
        target = _aspect_fit_rect(self._color.width, self._color.height, QtCore.QRectF(self.rect()))
        painter.drawImage(target, image)

        painter.setPen(QtGui.QPen(QtGui.QColor(230, 230, 230), 1))
        painter.drawRect(target)

        for index, body in enumerate(self._bodies):
            self._draw_body(painter, body, index, target, self._color.width, self._color.height)

        painter.setPen(QtGui.QColor(245, 245, 245))
        painter.drawText(
            QtCore.QRectF(10, 8, max(0, self.width() - 20), 24),
            ALIGN_LEFT | ALIGN_VCENTER,
            f"Full RGB scene with tracked skeleton overlays ({len(self._bodies)} bodies)",
        )

    def _draw_body(
        self,
        painter: QtGui.QPainter,
        body: BodyDataC,
        color_index: int,
        image_rect: QtCore.QRectF,
        image_w: int,
        image_h: int,
    ) -> None:
        color = _body_color(color_index)
        points: dict[int, QtCore.QPointF] = {}

        for joint_index, joint in enumerate(body.joints):
            if not _joint_has_color_position(joint, image_w, image_h):
                continue
            x = image_rect.x() + (float(joint.color_position.x) / image_w) * image_rect.width()
            y = image_rect.y() + (float(joint.color_position.y) / image_h) * image_rect.height()
            points[joint_index] = QtCore.QPointF(x, y)

        if not points:
            return

        base_width = max(2, int(image_rect.width() / 450))
        tracked_pen = QtGui.QPen(color, base_width, SOLID_LINE)
        inferred_pen = QtGui.QPen(color.lighter(150), max(1, base_width - 1), DASH_LINE)

        for a, b in BONES:
            if a not in points or b not in points:
                continue
            a_state = body.joints[a].tracking_state
            b_state = body.joints[b].tracking_state
            painter.setPen(tracked_pen if a_state == TRACKING_TRACKED and b_state == TRACKING_TRACKED else inferred_pen)
            painter.drawLine(points[a], points[b])

        painter.setBrush(QtGui.QBrush(color))
        painter.setPen(QtGui.QPen(QtGui.QColor(20, 20, 20), 1))
        radius = max(3, int(image_rect.width() / 350))
        for joint_index, point in points.items():
            if body.joints[joint_index].tracking_state == TRACKING_INFERRED:
                painter.setBrush(QtGui.QBrush(color.lighter(165)))
            else:
                painter.setBrush(QtGui.QBrush(color))
            painter.drawEllipse(point, radius, radius)

        label_joint = 3 if 3 in points else 20 if 20 in points else next(iter(points))
        label_pos = points[label_joint] + QtCore.QPointF(8.0, -8.0)
        painter.setPen(QtGui.QPen(color, 1))
        painter.drawText(label_pos, f"Body {body.body_id}")


class SkeletonCard(QtWidgets.QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._body: Optional[BodyDataC] = None
        self._color_index = 0
        self.setMinimumHeight(245)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding if hasattr(QtWidgets.QSizePolicy, "Policy") else QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Policy.Fixed if hasattr(QtWidgets.QSizePolicy, "Policy") else QtWidgets.QSizePolicy.Fixed)

    def set_body(self, body: BodyDataC, color_index: int) -> None:
        self._body = body
        self._color_index = color_index
        self.setToolTip(_body_tooltip(body))
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt override name.
        painter = QtGui.QPainter(self)
        painter.setRenderHint(ANTIALIASING, True)
        painter.fillRect(self.rect(), QtGui.QColor(35, 35, 38))

        card = QtCore.QRectF(self.rect()).adjusted(5, 5, -5, -5)
        painter.setBrush(QtGui.QBrush(QtGui.QColor(48, 48, 54)))
        painter.setPen(QtGui.QPen(QtGui.QColor(95, 95, 105), 1))
        painter.drawRoundedRect(card, 10, 10)

        if self._body is None:
            painter.setPen(QtGui.QColor(220, 220, 220))
            painter.drawText(card, ALIGN_CENTER, "No body")
            return

        color = _body_color(self._color_index)
        title_rect = QtCore.QRectF(card.x() + 12, card.y() + 8, card.width() - 24, 24)
        painter.setPen(color)
        painter.drawText(title_rect, ALIGN_LEFT | ALIGN_VCENTER, f"Body {self._body.body_id} - normalized skeleton")

        draw_area = QtCore.QRectF(card.x() + 18, card.y() + 40, card.width() - 36, card.height() - 92)
        self._draw_normalized_skeleton(painter, self._body, draw_area, color)

        footer = self._footer_text(self._body)
        footer_rect = QtCore.QRectF(card.x() + 12, card.bottom() - 44, card.width() - 24, 36)
        painter.setPen(QtGui.QColor(220, 220, 225))
        painter.drawText(footer_rect, ALIGN_LEFT | ALIGN_VCENTER, footer)

    def _draw_normalized_skeleton(
        self,
        painter: QtGui.QPainter,
        body: BodyDataC,
        area: QtCore.QRectF,
        color: QtGui.QColor,
    ) -> None:
        painter.setPen(QtGui.QPen(QtGui.QColor(95, 95, 105), 1, DOT_LINE))
        origin = QtCore.QPointF(area.center().x(), area.top() + area.height() * 0.58)
        scale = min(area.width() / 2.6, area.height() / 2.5)
        painter.drawLine(QtCore.QPointF(area.left(), origin.y()), QtCore.QPointF(area.right(), origin.y()))
        painter.drawLine(QtCore.QPointF(origin.x(), area.top()), QtCore.QPointF(origin.x(), area.bottom()))

        points: dict[int, QtCore.QPointF] = {}
        for joint_index, joint in enumerate(body.joints):
            if not _joint_has_position(joint):
                continue
            x = origin.x() + float(joint.position.x) * scale
            y = origin.y() - float(joint.position.y) * scale
            points[joint_index] = QtCore.QPointF(x, y)

        tracked_pen = QtGui.QPen(color, 3, SOLID_LINE)
        inferred_pen = QtGui.QPen(color.lighter(150), 2, DASH_LINE)
        for a, b in BONES:
            if a not in points or b not in points:
                continue
            a_state = body.joints[a].tracking_state
            b_state = body.joints[b].tracking_state
            painter.setPen(tracked_pen if a_state == TRACKING_TRACKED and b_state == TRACKING_TRACKED else inferred_pen)
            painter.drawLine(points[a], points[b])

        painter.setPen(QtGui.QPen(QtGui.QColor(20, 20, 20), 1))
        for joint_index, point in points.items():
            painter.setBrush(QtGui.QBrush(color.lighter(165) if body.joints[joint_index].tracking_state == TRACKING_INFERRED else color))
            painter.drawEllipse(point, 4, 4)

        painter.setPen(QtGui.QColor(190, 190, 195))
        painter.drawText(QtCore.QRectF(area.left(), area.bottom() - 18, area.width(), 18), ALIGN_CENTER, "X/Y meters; SpineBase at origin")

    @staticmethod
    def _footer_text(body: BodyDataC) -> str:
        tracked = sum(1 for joint in body.joints if joint.tracking_state == TRACKING_TRACKED)
        inferred = sum(1 for joint in body.joints if joint.tracking_state == TRACKING_INFERRED)
        root = body.joints[0].position
        head = body.joints[3].position
        return (
            f"Tracked {tracked}, inferred {inferred}\n"
            f"Root=({root.x:+.2f},{root.y:+.2f},{root.z:+.2f}) m  "
            f"Head=({head.x:+.2f},{head.y:+.2f},{head.z:+.2f}) m"
        )


class BodyPanel(QtWidgets.QScrollArea):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setWidgetResizable(True)
        self._cards: dict[int, SkeletonCard] = {}

        self._container = QtWidgets.QWidget()
        self._layout = QtWidgets.QVBoxLayout(self._container)
        self._layout.setContentsMargins(8, 8, 8, 8)
        self._layout.setSpacing(8)

        self._placeholder = QtWidgets.QLabel("No skeletons detected")
        self._placeholder.setAlignment(ALIGN_CENTER)
        self._placeholder.setMinimumHeight(120)
        self._layout.addWidget(self._placeholder)
        self._layout.addStretch(1)
        self.setWidget(self._container)

    def update_bodies(self, bodies: list[BodyDataC]) -> None:
        active_ids: set[int] = set()
        for index, body in enumerate(bodies):
            body_id = int(body.body_id)
            active_ids.add(body_id)
            card = self._cards.get(body_id)
            if card is None:
                card = SkeletonCard()
                self._cards[body_id] = card
                self._layout.insertWidget(max(0, self._layout.count() - 1), card)
            card.set_body(body, index)

        for body_id in list(self._cards.keys()):
            if body_id in active_ids:
                continue
            card = self._cards.pop(body_id)
            self._layout.removeWidget(card)
            card.setParent(None)
            card.deleteLater()

        self._placeholder.setVisible(not bodies)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, timeout_ms: int = 33) -> None:
        super().__init__()
        self.setWindowTitle(f"Kinect Skeleton UI ({QT_API})")
        self.resize(1500, 850)

        self.scene = SceneView()
        self.body_panel = BodyPanel()

        splitter = QtWidgets.QSplitter(HORIZONTAL)
        splitter.addWidget(self.scene)
        splitter.addWidget(self.body_panel)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([750, 750])
        self.setCentralWidget(splitter)

        self._frames = 0
        self._last_fps_time = time.monotonic()
        self._last_error = ""

        self.worker = KinectWorker(timeout_ms=timeout_ms, parent=self)
        self.worker.frame_ready.connect(self.on_frame)
        self.worker.status.connect(self.on_status)
        self.worker.error.connect(self.on_error)
        self.worker.start()

        self.statusBar().showMessage("Starting Kinect...")

    @Slot(object)
    def on_frame(self, frame: KinectUiFrame) -> None:
        self.scene.set_frame(frame.color, frame.raw_bodies)
        self.body_panel.update_bodies(frame.normalized_bodies)

        self._frames += 1
        now = time.monotonic()
        elapsed = now - self._last_fps_time
        if elapsed >= 1.0:
            fps = self._frames / elapsed
            self._frames = 0
            self._last_fps_time = now
            self.statusBar().showMessage(
                f"Bodies: {len(frame.raw_bodies)} | RGB: {'yes' if frame.color else 'pending'} | UI FPS: {fps:.1f}"
            )

    @Slot(str)
    def on_status(self, message: str) -> None:
        self.statusBar().showMessage(message)

    @Slot(str)
    def on_error(self, message: str) -> None:
        if message != self._last_error:
            self._last_error = message
            self.statusBar().showMessage(f"Kinect error: {message}")

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override name.
        self.worker.stop()
        self.worker.wait(2000)
        event.accept()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Qt Kinect skeleton viewer")
    parser.add_argument("--timeout-ms", type=int, default=33, help="Kinect body-frame wait timeout in milliseconds")
    args = parser.parse_args(argv)

    app = QtWidgets.QApplication(sys.argv if argv is None else [sys.argv[0], *argv])
    window = MainWindow(timeout_ms=args.timeout_ms)
    window.show()

    exec_fn = getattr(app, "exec", None) or getattr(app, "exec_")
    return int(exec_fn())


if __name__ == "__main__":
    raise SystemExit(main())
