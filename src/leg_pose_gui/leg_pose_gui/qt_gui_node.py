import sys
import threading
from collections import deque

import pyqtgraph as pg
import rclpy
from cv_bridge import CvBridge
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from leg_pose_msgs.msg import LegJointAngles, LegTrackingStatus


ANGLE_FIELDS = [
    ("Hip", "hip_flexion_extension_deg", "hip_valid", "hip_confidence"),
    ("Knee", "knee_flexion_deg", "knee_valid", "knee_confidence"),
    ("Ankle DP", "ankle_dorsi_plantar_deg", "ankle_dorsi_valid", "ankle_dorsi_confidence"),
    (
        "Ankle IE",
        "ankle_inversion_eversion_deg",
        "ankle_inversion_valid",
        "ankle_inversion_confidence",
    ),
]


class LegPoseQtNode(Node):
    def __init__(self, window) -> None:
        super().__init__("leg_pose_qt_gui_node")
        self._window = window
        self._bridge = CvBridge()
        self._neutral_client = self.create_client(Trigger, "/leg_pose/capture_neutral_pose")
        self.create_subscription(Image, "/leg_pose/front/skeleton_overlay", self._on_front, 10)
        self.create_subscription(Image, "/leg_pose/side/skeleton_overlay", self._on_side, 10)
        self.create_subscription(LegJointAngles, "/leg_pose/joint_angles", self._on_angles, 10)
        self.create_subscription(
            LegTrackingStatus,
            "/leg_pose/tracking_status",
            self._on_status,
            10,
        )

    def _on_front(self, msg: Image) -> None:
        self._window.set_image("front", self._image_to_pixmap(msg))

    def _on_side(self, msg: Image) -> None:
        self._window.set_image("side", self._image_to_pixmap(msg))

    def _on_angles(self, msg: LegJointAngles) -> None:
        self._window.set_angles(msg)

    def _on_status(self, msg: LegTrackingStatus) -> None:
        self._window.set_status(msg)

    def capture_neutral(self) -> None:
        if not self._neutral_client.service_is_ready():
            self._window.set_calibration_message("neutral service unavailable")
            return
        future = self._neutral_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_neutral_response)

    def _on_neutral_response(self, future) -> None:
        try:
            response = future.result()
            self._window.set_calibration_message(response.message)
        except Exception as exc:
            self._window.set_calibration_message(str(exc))

    def _image_to_pixmap(self, msg: Image) -> QPixmap:
        cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        height, width, channels = cv_image.shape
        bytes_per_line = channels * width
        image = QImage(
            cv_image.data,
            width,
            height,
            bytes_per_line,
            QImage.Format_RGB888,
        ).copy()
        return QPixmap.fromImage(image)


class LegPoseWindow(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Leg Pose Tracking")
        self.resize(1320, 820)
        self._history = {name: deque(maxlen=300) for name, *_ in ANGLE_FIELDS}
        self._sample = 0
        self._node = None
        self._build_ui()

    def bind_node(self, node: LegPoseQtNode) -> None:
        self._node = node

    def _build_ui(self) -> None:
        root = QHBoxLayout(self)
        camera_column = QVBoxLayout()
        camera_column.addWidget(self._camera_panel("Front", "front"))
        camera_column.addWidget(self._camera_panel("Side", "side"))
        root.addLayout(camera_column, 3)

        side_column = QVBoxLayout()
        angle_grid = QGridLayout()
        self._angle_labels = {}
        for index, (title, *_rest) in enumerate(ANGLE_FIELDS):
            card, value = self._angle_card(title)
            self._angle_labels[title] = value
            angle_grid.addWidget(card, index // 2, index % 2)
        side_column.addLayout(angle_grid)

        self._plot = pg.PlotWidget()
        self._plot.addLegend()
        self._plot.showGrid(x=True, y=True)
        self._curves = {
            name: self._plot.plot(name=name, pen=pg.intColor(index))
            for index, (name, *_rest) in enumerate(ANGLE_FIELDS)
        }
        side_column.addWidget(self._plot, 2)

        self._status = QLabel("Waiting for tracking status")
        self._status.setWordWrap(True)
        side_column.addWidget(self._status)

        self._calibration = QLabel("Neutral pose: not captured")
        side_column.addWidget(self._calibration)

        capture = QPushButton("Capture Neutral Pose")
        capture.clicked.connect(self._capture_neutral)
        side_column.addWidget(capture)
        root.addLayout(side_column, 2)

    def _camera_panel(self, title: str, key: str) -> QFrame:
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        layout = QVBoxLayout(frame)
        label = QLabel(title)
        image = QLabel("Waiting for %s overlay" % title.lower())
        image.setAlignment(Qt.AlignCenter)
        image.setMinimumSize(640, 300)
        image.setStyleSheet("background: #111; color: #ddd;")
        setattr(self, "_%s_image" % key, image)
        layout.addWidget(label)
        layout.addWidget(image, 1)
        return frame

    def _angle_card(self, title: str):
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        layout = QVBoxLayout(frame)
        heading = QLabel(title)
        value = QLabel("--.- deg")
        value.setStyleSheet("font-size: 28px; font-weight: 600;")
        layout.addWidget(heading)
        layout.addWidget(value)
        return frame, value

    def _capture_neutral(self) -> None:
        if self._node is None:
            return
        self._node.capture_neutral()

    def set_image(self, key: str, pixmap: QPixmap) -> None:
        label = getattr(self, "_%s_image" % key)
        label.setPixmap(pixmap.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def set_angles(self, msg: LegJointAngles) -> None:
        self._sample += 1
        x_values = list(range(max(0, self._sample - 300), self._sample))
        for title, field, valid_field, confidence_field in ANGLE_FIELDS:
            value = float(getattr(msg, field))
            valid = bool(getattr(msg, valid_field))
            confidence = float(getattr(msg, confidence_field))
            self._history[title].append(value)
            self._angle_labels[title].setText(
                "%.1f deg | %s | %.2f" % (value, "valid" if valid else "invalid", confidence)
            )
            y_values = list(self._history[title])
            self._curves[title].setData(x_values[-len(y_values):], y_values)

    def set_status(self, msg: LegTrackingStatus) -> None:
        warnings = ", ".join(msg.warnings) if msg.warnings else "none"
        self._status.setText(
            "angle_hz=%.1f | front=%s | side=%s | dt=%.1f ms | warnings=%s"
            % (
                msg.angle_publish_hz,
                msg.front_camera_connected,
                msg.side_camera_connected,
                msg.camera_timestamp_delta_ms,
                warnings,
            )
        )

    def set_calibration_message(self, message: str) -> None:
        self._calibration.setText("Neutral pose: %s" % message)


def main(args=None) -> None:
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    window = LegPoseWindow()
    node = LegPoseQtNode(window)
    window.bind_node(node)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    window.show()
    try:
        app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()
