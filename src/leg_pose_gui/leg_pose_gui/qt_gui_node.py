import sys
import threading
from collections import deque

import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, Trigger

from leg_pose_msgs.msg import LegJointAngles, LegKeypoints2D, LegTrackingStatus

try:
    from PySide6.QtCore import QObject, Qt, Signal, Slot
    from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
    from PySide6.QtWidgets import (
        QApplication,
        QFrame,
        QFormLayout,
        QGridLayout,
        QHBoxLayout,
        QLabel,
        QPushButton,
        QDoubleSpinBox,
        QVBoxLayout,
        QWidget,
    )
except ImportError:
    from PyQt5.QtCore import QObject, Qt, pyqtSignal as Signal, pyqtSlot as Slot
    from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPixmap
    from PyQt5.QtWidgets import (
        QApplication,
        QFrame,
        QFormLayout,
        QGridLayout,
        QHBoxLayout,
        QLabel,
        QPushButton,
        QDoubleSpinBox,
        QVBoxLayout,
        QWidget,
    )


ANGLE_FIELDS = [
    ("Hip", "hip_flexion_extension_deg", "hip_valid", "hip_confidence"),
    ("Knee", "knee_flexion_deg", "knee_valid", "knee_confidence"),
    ("Ankle DP", "ankle_dorsi_plantar_deg", "ankle_dorsi_valid", "ankle_dorsi_confidence"),
]

PLOT_COLORS = {
    "Hip": QColor(230, 85, 85),
    "Knee": QColor(80, 150, 240),
    "Ankle DP": QColor(80, 190, 120),
}

SAFE_ANGLE_PARAMS = [
    ("Hip safe", "safe_hip_flexion_extension_deg"),
    ("Knee safe", "safe_knee_flexion_deg"),
    ("Ankle DP safe", "safe_ankle_dorsi_plantar_deg"),
]


class LegPoseQtNode(Node):
    def __init__(self, bridge) -> None:
        super().__init__("leg_pose_qt_gui_node")
        self._bridge_to_ui = bridge
        self._bridge = CvBridge()
        self._neutral_client = self.create_client(Trigger, "/leg_pose/capture_neutral_pose")
        self._safety_gate_client = self.create_client(
            SetBool,
            "/leg_pose/set_safety_gate_enabled",
        )
        self._safe_get_client = self.create_client(
            GetParameters,
            "/angle_safety_gate_node/get_parameters",
        )
        self._safe_set_client = self.create_client(
            SetParameters,
            "/angle_safety_gate_node/set_parameters",
        )
        self.create_subscription(Image, "/leg_pose/side/skeleton_overlay", self._on_side, 10)
        self.create_subscription(
            LegKeypoints2D,
            "/leg_pose/side/keypoints_2d",
            self._on_side_keypoints,
            10,
        )
        self.create_subscription(LegJointAngles, "/leg_pose/joint_angles", self._on_angles, 10)
        self.create_subscription(
            LegJointAngles,
            "/leg_pose/joint_angles_raw",
            self._on_raw_angles,
            10,
        )
        self.create_timer(1.0, self._refresh_safe_angles)
        self.create_subscription(
            LegTrackingStatus,
            "/leg_pose/tracking_status",
            self._on_status,
            10,
        )

    def _on_side(self, msg: Image) -> None:
        self._bridge_to_ui.image.emit("side", self._image_to_pixmap(msg))

    def _on_side_keypoints(self, msg: LegKeypoints2D) -> None:
        self._bridge_to_ui.keypoints.emit(msg)

    def _on_angles(self, msg: LegJointAngles) -> None:
        self._bridge_to_ui.output_angles.emit(msg)

    def _on_raw_angles(self, msg: LegJointAngles) -> None:
        self._bridge_to_ui.raw_angles.emit(msg)

    def _on_status(self, msg: LegTrackingStatus) -> None:
        self._bridge_to_ui.status.emit(msg)

    def capture_neutral(self) -> None:
        if not self._neutral_client.service_is_ready():
            self._bridge_to_ui.calibration_message.emit("neutral service unavailable")
            return
        future = self._neutral_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_neutral_response)

    def set_safety_gate_enabled(self, enabled: bool) -> None:
        if not self._safety_gate_client.service_is_ready():
            self._bridge_to_ui.publish_message.emit("safety gate service unavailable")
            return
        request = SetBool.Request()
        request.data = enabled
        future = self._safety_gate_client.call_async(request)
        future.add_done_callback(self._on_publish_response)

    def set_safe_angles(self, values) -> None:
        if not self._safe_set_client.service_is_ready():
            self._bridge_to_ui.safe_message.emit("safe angle parameter service unavailable")
            return
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name, Parameter.Type.DOUBLE, float(value)).to_parameter_msg()
            for name, value in values.items()
        ]
        future = self._safe_set_client.call_async(request)
        future.add_done_callback(self._on_set_safe_angles_response)

    def _refresh_safe_angles(self) -> None:
        if not self._safe_get_client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = [name for _title, name in SAFE_ANGLE_PARAMS]
        future = self._safe_get_client.call_async(request)
        future.add_done_callback(self._on_safe_angles_response)

    def _on_neutral_response(self, future) -> None:
        try:
            response = future.result()
            self._bridge_to_ui.calibration_message.emit(response.message)
        except Exception as exc:
            self._bridge_to_ui.calibration_message.emit(str(exc))

    def _on_publish_response(self, future) -> None:
        try:
            response = future.result()
            self._bridge_to_ui.publish_message.emit(response.message)
        except Exception as exc:
            self._bridge_to_ui.publish_message.emit(str(exc))

    def _on_safe_angles_response(self, future) -> None:
        try:
            response = future.result()
            values = {}
            for (_title, name), value in zip(SAFE_ANGLE_PARAMS, response.values):
                values[name] = float(value.double_value)
            self._bridge_to_ui.safe_angles.emit(values)
        except Exception as exc:
            self._bridge_to_ui.safe_message.emit(str(exc))

    def _on_set_safe_angles_response(self, future) -> None:
        try:
            response = future.result()
            if all(result.successful for result in response.results):
                self._bridge_to_ui.safe_message.emit("Safe angles updated")
            else:
                reasons = [
                    result.reason
                    for result in response.results
                    if not result.successful and result.reason
                ]
                message = "; ".join(reasons) or "safe angle update failed"
                self._bridge_to_ui.safe_message.emit(message)
        except Exception as exc:
            self._bridge_to_ui.safe_message.emit(str(exc))

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


class UiBridge(QObject):
    image = Signal(str, object)
    keypoints = Signal(object)
    output_angles = Signal(object)
    raw_angles = Signal(object)
    status = Signal(object)
    calibration_message = Signal(str)
    publish_message = Signal(str)
    safe_angles = Signal(object)
    safe_message = Signal(str)


class AnglePlot(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(240)
        self._series = {}
        self._raw_series = {}

    def set_series(self, series, raw_series) -> None:
        self._series = {name: list(values) for name, values in series.items()}
        self._raw_series = {name: list(values) for name, values in raw_series.items()}
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(18, 18, 18))
        painter.setRenderHint(QPainter.Antialiasing)
        self._draw_grid(painter)
        all_values = [
            value
            for values in list(self._series.values()) + list(self._raw_series.values())
            for value in values
        ]
        if not all_values:
            painter.setPen(QColor(220, 220, 220))
            painter.drawText(self.rect(), Qt.AlignCenter, "Waiting for angle data")
            return
        min_value = min(min(all_values), -5.0)
        max_value = max(max(all_values), 5.0)
        if max_value - min_value < 1.0:
            max_value += 0.5
            min_value -= 0.5
        for name, values in self._raw_series.items():
            color = QColor(PLOT_COLORS[name])
            color.setAlpha(95)
            self._draw_line(painter, values, min_value, max_value, color, 1)
        for name, values in self._series.items():
            self._draw_line(painter, values, min_value, max_value, PLOT_COLORS[name], 2)
        self._draw_legend(painter)

    def _draw_grid(self, painter: QPainter) -> None:
        painter.setPen(QPen(QColor(55, 55, 55), 1))
        for index in range(1, 4):
            y = int(self.height() * index / 4)
            painter.drawLine(0, y, self.width(), y)
        painter.setPen(QPen(QColor(35, 35, 35), 1))
        for index in range(1, 6):
            x = int(self.width() * index / 6)
            painter.drawLine(x, 0, x, self.height())

    def _draw_line(self, painter, values, min_value, max_value, color, width) -> None:
        if len(values) < 2:
            return
        painter.setPen(QPen(color, width))
        plot_width = max(1, self.width() - 20)
        plot_height = max(1, self.height() - 30)
        left = 10
        top = 10
        max_points = max(2, len(values) - 1)
        previous = None
        for index, value in enumerate(values):
            x = left + int(plot_width * index / max_points)
            normalized = (value - min_value) / (max_value - min_value)
            y = top + int(plot_height * (1.0 - normalized))
            if previous is not None:
                painter.drawLine(previous[0], previous[1], x, y)
            previous = (x, y)

    def _draw_legend(self, painter: QPainter) -> None:
        x = 12
        y = self.height() - 12
        for name in PLOT_COLORS:
            painter.setPen(QPen(PLOT_COLORS[name], 3))
            painter.drawLine(x, y, x + 18, y)
            painter.setPen(QColor(225, 225, 225))
            painter.drawText(x + 24, y + 4, name)
            x += 110


class LegPoseWindow(QWidget):
    def __init__(self, bridge: UiBridge) -> None:
        super().__init__()
        self.setWindowTitle("Leg Pose Tracking")
        self.setFixedSize(1320, 860)
        self._history = {name: deque(maxlen=300) for name, *_ in ANGLE_FIELDS}
        self._raw_history = {name: deque(maxlen=300) for name, *_ in ANGLE_FIELDS}
        self._sample = 0
        self._raw_sample = 0
        self._node = None
        self._bridge = bridge
        self._safe_angle_inputs = {}
        self._updating_safe_angles = False
        self._last_side_keypoints = {}
        self._last_raw_angles_msg = None
        self._build_ui()
        self._connect_bridge()

    def bind_node(self, node: LegPoseQtNode) -> None:
        self._node = node

    def _build_ui(self) -> None:
        root = QHBoxLayout(self)
        camera_panel = QWidget()
        camera_panel.setFixedWidth(760)
        camera_column = QVBoxLayout(camera_panel)
        camera_column.addWidget(self._camera_panel("Side", "side"))
        root.addWidget(camera_panel)

        side_panel = QWidget()
        side_panel.setFixedWidth(520)
        side_column = QVBoxLayout(side_panel)
        angle_grid = QGridLayout()
        self._angle_labels = {}
        for index, (title, *_rest) in enumerate(ANGLE_FIELDS):
            card, value = self._angle_card(title)
            self._angle_labels[title] = value
            angle_grid.addWidget(card, index // 2, index % 2)
        side_column.addLayout(angle_grid)

        self._plot = AnglePlot()
        self._plot.setFixedHeight(210)
        side_column.addWidget(self._plot, 2)

        self._raw_preview = QLabel("Detected preview: waiting for /leg_pose/joint_angles_raw")
        self._raw_preview.setWordWrap(True)
        self._raw_preview.setFixedHeight(54)
        side_column.addWidget(self._raw_preview)

        self._status = QLabel("Waiting for tracking status")
        self._status.setWordWrap(True)
        self._status.setFixedHeight(54)
        side_column.addWidget(self._status)

        self._calibration = QLabel("Neutral pose: not captured")
        self._calibration.setFixedHeight(28)
        side_column.addWidget(self._calibration)

        self._publish_state = QLabel("Safety gate: OFF (detected passthrough)")
        self._publish_state.setFixedHeight(28)
        side_column.addWidget(self._publish_state)

        side_column.addWidget(self._safe_angle_panel())

        publish_controls = QHBoxLayout()
        gate_off = QPushButton("Safety Gate OFF")
        gate_off.clicked.connect(self._disable_safety_gate)
        gate_on = QPushButton("Safety Gate ON")
        gate_on.clicked.connect(self._enable_safety_gate)
        publish_controls.addWidget(gate_off)
        publish_controls.addWidget(gate_on)
        side_column.addLayout(publish_controls)

        capture = QPushButton("Capture Neutral Pose")
        capture.clicked.connect(self._capture_neutral)
        side_column.addWidget(capture)
        root.addWidget(side_panel)

    def _connect_bridge(self) -> None:
        self._bridge.image.connect(self.set_image)
        self._bridge.keypoints.connect(self.set_keypoints)
        self._bridge.output_angles.connect(self.set_angles)
        self._bridge.raw_angles.connect(self.set_raw_angles)
        self._bridge.status.connect(self.set_status)
        self._bridge.calibration_message.connect(self.set_calibration_message)
        self._bridge.publish_message.connect(self.set_publish_message)
        self._bridge.safe_angles.connect(self.set_safe_angles)
        self._bridge.safe_message.connect(self.set_safe_message)

    def _camera_panel(self, title: str, key: str) -> QFrame:
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        layout = QVBoxLayout(frame)
        label = QLabel(title)
        image = QLabel("Waiting for %s overlay" % title.lower())
        image.setAlignment(Qt.AlignCenter)
        image.setFixedSize(730, 700)
        image.setStyleSheet("background: #111; color: #ddd;")
        setattr(self, "_%s_image" % key, image)
        layout.addWidget(label)
        layout.addWidget(image, 1)
        return frame

    def _angle_card(self, title: str):
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        frame.setFixedSize(250, 92)
        layout = QVBoxLayout(frame)
        heading = QLabel(title)
        value = QLabel("--.- deg")
        value.setFixedWidth(220)
        value.setStyleSheet("font-size: 28px; font-weight: 600;")
        layout.addWidget(heading)
        layout.addWidget(value)
        return frame, value

    def _safe_angle_panel(self) -> QFrame:
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        frame.setFixedHeight(150)
        layout = QVBoxLayout(frame)
        title = QLabel("Safety Gate Angles")
        layout.addWidget(title)
        form = QFormLayout()
        for label, name in SAFE_ANGLE_PARAMS:
            spin = QDoubleSpinBox()
            spin.setRange(-180.0, 180.0)
            spin.setDecimals(1)
            spin.setSingleStep(1.0)
            spin.setSuffix(" deg")
            spin.setFixedWidth(140)
            spin.valueChanged.connect(self._safe_angle_value_changed)
            self._safe_angle_inputs[name] = spin
            form.addRow(label, spin)
        layout.addLayout(form)
        self._safe_angle_state = QLabel("Safe angles: waiting for parameters")
        self._safe_angle_state.setFixedHeight(24)
        layout.addWidget(self._safe_angle_state)
        return frame

    def _capture_neutral(self) -> None:
        if self._node is None:
            return
        self._node.capture_neutral()

    def _disable_safety_gate(self) -> None:
        if self._node is None:
            return
        self._node.set_safety_gate_enabled(False)

    def _enable_safety_gate(self) -> None:
        if self._node is None:
            return
        self._node.set_safety_gate_enabled(True)

    def _safe_angle_value_changed(self, _value=None) -> None:
        if self._node is None or self._updating_safe_angles:
            return
        values = {
            name: spin.value()
            for name, spin in self._safe_angle_inputs.items()
        }
        self._node.set_safe_angles(values)

    @Slot(str, object)
    def set_image(self, key: str, pixmap: QPixmap) -> None:
        label = getattr(self, "_%s_image" % key)
        annotated = pixmap
        if key == "side":
            annotated = self._annotate_side_image(pixmap)
        scaled = annotated.scaled(
            label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        label.setPixmap(scaled)

    @Slot(object)
    def set_keypoints(self, msg: LegKeypoints2D) -> None:
        self._last_side_keypoints = {
            keypoint.name: keypoint
            for keypoint in msg.keypoints
        }

    @Slot(object)
    def set_angles(self, msg: LegJointAngles) -> None:
        self._sample += 1
        for title, field, valid_field, confidence_field in ANGLE_FIELDS:
            value = float(getattr(msg, field))
            valid = bool(getattr(msg, valid_field))
            confidence = float(getattr(msg, confidence_field))
            self._history[title].append(value)
            self._angle_labels[title].setText(
                "%.1f deg | %s | %.2f" % (value, "valid" if valid else "invalid", confidence)
            )
        if msg.header.frame_id == "safety_gate":
            self._publish_state.setText("Safety gate: ON (safe angles)")
        elif msg.header.frame_id == "hold_last_valid":
            self._publish_state.setText("Safety gate: OFF (holding last valid)")
        else:
            self._publish_state.setText("Safety gate: OFF (detected passthrough)")
        self._plot.set_series(self._history, self._raw_history)

    @Slot(object)
    def set_raw_angles(self, msg: LegJointAngles) -> None:
        self._raw_sample += 1
        self._last_raw_angles_msg = msg
        preview = []
        for title, field, valid_field, confidence_field in ANGLE_FIELDS:
            value = float(getattr(msg, field))
            valid = bool(getattr(msg, valid_field))
            confidence = float(getattr(msg, confidence_field))
            self._raw_history[title].append(value)
            preview.append(
                "%s %.1f deg %s %.2f"
                % (title, value, "valid" if valid else "invalid", confidence)
            )
        self._raw_preview.setText("Detected preview: " + " | ".join(preview))
        self._plot.set_series(self._history, self._raw_history)

    @Slot(object)
    def set_status(self, msg: LegTrackingStatus) -> None:
        warnings = ", ".join(msg.warnings) if msg.warnings else "none"
        self._status.setText(
            "angle_hz=%.1f | side_camera=%s | warnings=%s"
            % (
                msg.angle_publish_hz,
                msg.side_camera_connected,
                warnings,
            )
        )

    @Slot(str)
    def set_calibration_message(self, message: str) -> None:
        self._calibration.setText("Neutral pose: %s" % message)

    @Slot(str)
    def set_publish_message(self, message: str) -> None:
        self._publish_state.setText(message)

    @Slot(object)
    def set_safe_angles(self, values) -> None:
        self._updating_safe_angles = True
        try:
            for name, value in values.items():
                if name in self._safe_angle_inputs:
                    self._safe_angle_inputs[name].setValue(float(value))
        finally:
            self._updating_safe_angles = False
        self._safe_angle_state.setText(
            "Safe: hip %.1f | knee %.1f | ankle %.1f"
            % (
                values.get("safe_hip_flexion_extension_deg", 0.0),
                values.get("safe_knee_flexion_deg", 0.0),
                values.get("safe_ankle_dorsi_plantar_deg", 0.0),
            )
        )

    @Slot(str)
    def set_safe_message(self, message: str) -> None:
        self._safe_angle_state.setText(message)

    def _annotate_side_image(self, pixmap: QPixmap) -> QPixmap:
        if self._last_raw_angles_msg is None or not self._last_side_keypoints:
            return pixmap
        annotated = QPixmap(pixmap)
        painter = QPainter(annotated)
        painter.setRenderHint(QPainter.Antialiasing)
        self._draw_angle_label(
            painter,
            "hip",
            "Hip",
            self._last_raw_angles_msg.hip_flexion_extension_deg,
            self._last_raw_angles_msg.hip_valid,
            self._last_raw_angles_msg.hip_confidence,
            QColor(230, 85, 85),
            -20,
            -30,
        )
        self._draw_angle_label(
            painter,
            "knee",
            "Knee",
            self._last_raw_angles_msg.knee_flexion_deg,
            self._last_raw_angles_msg.knee_valid,
            self._last_raw_angles_msg.knee_confidence,
            QColor(80, 150, 240),
            18,
            -20,
        )
        self._draw_angle_label(
            painter,
            "ankle",
            "Ankle",
            self._last_raw_angles_msg.ankle_dorsi_plantar_deg,
            self._last_raw_angles_msg.ankle_dorsi_valid,
            self._last_raw_angles_msg.ankle_dorsi_confidence,
            QColor(80, 190, 120),
            18,
            22,
        )
        painter.end()
        return annotated

    def _draw_angle_label(
        self,
        painter: QPainter,
        keypoint_name: str,
        title: str,
        value: float,
        valid: bool,
        confidence: float,
        color: QColor,
        offset_x: int,
        offset_y: int,
    ) -> None:
        keypoint = self._last_side_keypoints.get(keypoint_name)
        if keypoint is None:
            return
        text = "%s %.1f deg %s %.2f" % (
            title,
            value,
            "valid" if valid else "invalid",
            confidence,
        )
        x = int(keypoint.x) + offset_x
        y = int(keypoint.y) + offset_y
        metrics = painter.fontMetrics()
        width = metrics.horizontalAdvance(text) + 12
        height = metrics.height() + 8
        x = max(4, min(x, painter.device().width() - width - 4))
        y = max(height + 4, min(y, painter.device().height() - 4))
        painter.setPen(QPen(color, 2))
        painter.drawLine(int(keypoint.x), int(keypoint.y), x, y - height // 2)
        fill = QColor(0, 0, 0, 185)
        painter.fillRect(x, y - height, width, height, fill)
        painter.setPen(QPen(color, 2))
        painter.drawRect(x, y - height, width, height)
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(x + 6, y - 6, text)


def main(args=None) -> None:
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    bridge = UiBridge()
    window = LegPoseWindow(bridge)
    node = LegPoseQtNode(bridge)
    window.bind_node(node)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    window.show()
    try:
        app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()
