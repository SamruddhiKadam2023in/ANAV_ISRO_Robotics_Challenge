# isro_gcs_ardupilot.py
# A Modern, High-Performance Ground Control Station for ArduPilot
# Built for the ISRO Robotics Challenge 2025
#
# Author: Gemini
# Dependencies: pyqt6, dronekit, opencv-python, pyqtgraph, numpy

import sys
import math
import time
import cv2
import numpy as np
import collections
from pymavlink import mavutil

# Compatibility patch for dronekit on Python 3.10+
try:
    from collections.abc import MutableMapping
except ImportError:
    from collections import MutableMapping
collections.MutableMapping = MutableMapping

from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QGridLayout, QFrame, QLineEdit, QPlainTextEdit,
    QStackedWidget, QSizePolicy
)
from PyQt6.QtGui import (
    QFont, QImage, QPixmap, QPainter, QColor, QPen,
    QBrush, QPainterPath
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QPointF, QRectF

from dronekit import connect, VehicleMode, APIException
import pyqtgraph as pg

STYLESHEET = """
/* Base */
QWidget {
    background-color: #0b1220; /* deep navy */
    color: #e6f0ff;
    font-family: 'Segoe UI','Inter','Tahoma','Consolas', monospace;
    font-size: 14px;
}

/* Sidebar */
QFrame#Sidebar {
    background-color: #0a1526;
    border-right: 1px solid #22324a;
    padding: 12px;
}

/* Title and headers */
QLabel#HeaderLabel {
    font-size: 14px;
    font-weight: 800;
    text-transform: uppercase;
    color: #86b9ff;
    padding: 8px 0 6px 0;
    border-bottom: 1px solid #22324a;
    margin-top: 8px;
    letter-spacing: 0.4px;
}

/* Cards */
QFrame#Card {
    background-color: #0f1b2e;
    border: 1px solid #22324a;
    border-radius: 12px;
    padding: 12px;
}

/* Data Badges */
QLabel#DataLabel {
    font-size: 20px;
    font-weight: 900;
    color: #00d4ff;      /* cyan accent */
    background-color: #0e223a;
    border: 1px solid #22405f;
    padding: 6px 10px;
    border-radius: 10px;
}

/* NEW: Style for the Edge Count Label */
QLabel#EdgeCountLabel {
    font-weight: 700;
    padding: 10px;
    background-color: #0e223a;
    border: 1px solid #22405f;
    border-radius: 10px;
}


/* Inputs & Text Areas */
QLineEdit, QPlainTextEdit {
    background-color: #0b1220;
    border: 1px solid #22324a;
    border-radius: 8px;
    padding: 7px 9px;
    selection-background-color: #00d4ff;
    selection-color: #06101c;
}
QPlainTextEdit { line-height: 1.4; }

/* Buttons */
QPushButton {
    background-color: #1a2b44;
    border: 1px solid #22324a;
    border-radius: 10px;
    padding: 10px 12px;
    font-weight: 700;
    letter-spacing: 0.2px;
}
QPushButton:hover { background-color: #20334f; border-color: #2b3c58; }
QPushButton:pressed { background-color: #12223b; }
QPushButton:disabled { background-color: #0f1c33; color: #7d90b2; border-color: #18263d; }
QPushButton:checked { background-color: #0f2842; border-color: #2f4d73; color: #ffffff; }

/* Nav Buttons (sidebar) */
QPushButton#NavButton {
    text-align: left;
    padding: 10px 12px;
    border-left: 3px solid transparent;
}
QPushButton#NavButton:hover {
    background-color: #15263f;
    border-left: 3px solid #2b3c58;
}
QPushButton#NavButton:checked {
    background-color: #112943;
    border-left: 3px solid #00d4ff; /* active cyan accent */
    color: #ffffff;
}

/* Contextual actions */
QPushButton#ArmButton {
    background-color: #28cf73;      /* success green */
    border-color: #40e08a;
    color: #04151f;
}
QPushButton#ArmButton:hover { background-color: #34e182; }
QPushButton#DisarmButton {
    background-color: #e05757;      /* danger red */
    border-color: #f07070;
    color: #04151f;
}
QPushButton#DisarmButton:hover { background-color: #f07070; }

/* Tooltips */
QToolTip {
    background-color: #0e1a2d;
    color: #e6f0ff;
    border: 1px solid #22324a;
    padding: 6px 8px;
    border-radius: 8px;
}
"""

# --- WORKER THREADS ---

class DroneWorker(QThread):
    """Handles DroneKit communication to prevent freezing the GUI"""
    connection_status = pyqtSignal(str, str)
    vehicle_data_updated = pyqtSignal(dict)

    def __init__(self, connection_string):
        super().__init__()
        self.connection_string = connection_string
        self.vehicle = None
        self._is_running = True
        # support both sequence and all-motor tests
        self._test_sequence_requested = False
        self._test_all_requested = False
        self._test_params = {}

    def run(self):
        try:
            self.connection_status.emit("Connecting...", "orange")
            self.vehicle = connect(self.connection_string, wait_ready=True, heartbeat_timeout=30)

            if not self.vehicle:
                self.connection_status.emit("Connection Failed", "#da3633")
                return

            self.connection_status.emit(f"Connected: APM {self.vehicle.version}", "#238636")

            self.vehicle.add_attribute_listener('attitude', self.attribute_callback)
            self.vehicle.add_attribute_listener('location.local_frame', self.attribute_callback)
            self.vehicle.add_attribute_listener('armed', self.attribute_callback)
            self.vehicle.add_attribute_listener('mode', self.attribute_callback)
            self.vehicle.add_attribute_listener('battery', self.attribute_callback)

            while self._is_running and self.vehicle:
                if self._test_sequence_requested:
                    self._execute_motor_test_sequence()
                    self._test_sequence_requested = False

                if self._test_all_requested:
                    self._execute_motor_test_all()
                    self._test_all_requested = False

                self.msleep(200)

        except APIException as e:
            self.connection_status.emit(f"API Error: {e}", "#da3633")
        except Exception as e:
            self.connection_status.emit(f"Error: {e}", "#da3633")
        finally:
            if self.vehicle:
                self.vehicle.close()
            self.connection_status.emit("Disconnected", "#da3633")

    def attribute_callback(self, vehicle, attr_name, value):
        data = {
            'roll': self.vehicle.attitude.roll,
            'pitch': self.vehicle.attitude.pitch,
            'yaw': self.vehicle.attitude.yaw,
            'north': self.vehicle.location.local_frame.north,
            'east': self.vehicle.location.local_frame.east,
            'down': self.vehicle.location.local_frame.down,
            'voltage': self.vehicle.battery.voltage,
            'level': self.vehicle.battery.level,
            'mode': self.vehicle.mode.name,
            'is_armed': self.vehicle.armed,
        }
        self.vehicle_data_updated.emit(data)

    def arm_vehicle(self):
        if self.vehicle and not self.vehicle.armed:
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True
            self.log_message("ARM command sent.")

    def disarm_vehicle(self):
        if self.vehicle and self.vehicle.armed:
            self.vehicle.channels.overrides = {}
            self.vehicle.armed = False
            self.log_message("DISARM command sent. RC overrides cleared.")

    def start_motor_test_sequence(self, throttle, duration):
        if self.vehicle:
            self.log_message("Sequential motor test initiated.")
            self._test_params = {'throttle': throttle, 'duration': duration}
            self._test_sequence_requested = True
        else:
            self.log_message("Cannot start test: Not connected.")

    def start_motor_test_all(self, throttle, duration):
        if self.vehicle:
            self.log_message("Simultaneous motor test initiated.")
            self._test_params = {'throttle': throttle, 'duration': duration}
            self._test_all_requested = True
        else:
            self.log_message("Cannot start test: Not connected.")

    def _execute_motor_test_sequence(self):
        params = self._test_params
        throttle_percent = params.get('throttle', 0)
        duration = params.get('duration', 0)
        motor_count = 6  # Hexacopter

        self.log_message(f"Testing {motor_count} motors in sequence at {throttle_percent}%...")

        for motor_index in range(1, motor_count + 1):
            if not self._is_running:
                break
            self.log_message(f"Testing Motor {motor_index}...")
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0, mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
                motor_index, 0, throttle_percent, duration, 0, 0, 0
            )
            self.vehicle.send_mavlink(msg)
            self.msleep(int(duration * 1000) + 500)

        self.log_message("Sequential motor test finished.")

    def _execute_motor_test_all(self):
        params = self._test_params
        throttle_percent = params.get('throttle', 0)
        duration = params.get('duration', 0)

        self.log_message("WARNING: Arming vehicle for simultaneous test.")
        if not self.vehicle.armed:
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True
            arm_wait_count = 0
            while not self.vehicle.armed and arm_wait_count < 50:
                self.msleep(100)
                arm_wait_count += 1
            if not self.vehicle.armed:
                self.log_message("Error: Failed to arm vehicle. Aborting test.")
                return

        self.log_message(f"Vehicle armed. Spinning all motors at {throttle_percent}%...")
        min_pwm, max_pwm = 1100, 1900
        pwm_value = int(min_pwm + (throttle_percent / 100.0) * (max_pwm - min_pwm))
        self.vehicle.channels.overrides['3'] = pwm_value
        self.msleep(int(duration * 1000))
        self.log_message("Stopping motors and disarming.")
        self.vehicle.channels.overrides = {}
        self.vehicle.armed = False
        self.log_message("Simultaneous motor test finished.")

    def log_message(self, message):
        self.connection_status.emit(message, 'white')

    def stop(self):
        self._is_running = False

class VideoWorker(QThread):
    frame_updated = pyqtSignal(np.ndarray, int)

    def __init__(self, video_source, backend_preferences=None, fallback_size=(640, 480)):
        super().__init__()
        self.video_source = video_source
        self._is_running = True
        self._fallback_size = fallback_size
        self.backend_preferences = backend_preferences or [
            getattr(cv2, name, None) for name in ['CAP_DSHOW', 'CAP_MSMF', 'CAP_ANY']
        ]
        self.vio_enabled = True

    def set_vio_enabled(self, enabled: bool):
        self.vio_enabled = enabled

    def process_frame_vio(self, frame: np.ndarray):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        kernel = np.ones((3, 3), np.uint8)
        thick_edges = cv2.dilate(edges, kernel, iterations=1)
        contours, _ = cv2.findContours(thick_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        output_frame = frame.copy()
        cv2.drawContours(output_frame, contours, -1, (255, 0, 255), 2)
        edges_colored = cv2.cvtColor(thick_edges, cv2.COLOR_GRAY2BGR)
        final_output = cv2.addWeighted(output_frame, 0.7, edges_colored, 0.3, 0)
        return final_output, len(contours)

    def run(self):
        cap = None
        for backend in self.backend_preferences:
            if backend is None:
                continue
            try:
                cap = cv2.VideoCapture(self.video_source, backend)
                if cap and cap.isOpened():
                    break
            except Exception:
                pass
        
        if not (cap and cap.isOpened()):
            black_frame = np.zeros((self._fallback_size[0], self._fallback_size[1], 3), dtype=np.uint8)
            while self._is_running:
                self.frame_updated.emit(black_frame, -1)
                self.msleep(500)
            return

        while self._is_running and cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                if self.vio_enabled:
                    processed_frame, edge_count = self.process_frame_vio(frame)
                    self.frame_updated.emit(processed_frame, edge_count)
                else:
                    self.frame_updated.emit(frame, -1)
            else:
                self.msleep(100)
        
        if cap:
            cap.release()

    def stop(self):
        self._is_running = False

# --- CUSTOM WIDGETS (UI INDICATORS) ---

class AttitudeIndicator(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(200, 200)
        self.pitch, self.roll = 0.0, 0.0
        self.sky_color, self.ground_color = QColor("#0d1117"), QColor("#010409")
        self.grid_color, self.instrument_pen = QColor(0, 255, 255, 40), QPen(QColor(0, 255, 255), 2)
    def set_attitude(self, pitch, roll):
        self.pitch, self.roll = pitch, roll
        self.update()
    def paintEvent(self, event):
        painter = QPainter(self); painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        width, height = self.width(), self.height(); center = QPointF(width/2, height/2); radius = min(width,height)/2-10
        painter.setPen(self.instrument_pen); painter.drawEllipse(center, radius, radius); painter.save()
        path = QPainterPath(); path.addEllipse(center, radius, radius); painter.setClipPath(path)
        painter.translate(center); painter.rotate(math.degrees(self.roll)); painter.translate(-center)
        pitch_offset = radius * math.sin(self.pitch)
        painter.setBrush(self.sky_color); painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRect(int(center.x()-radius*2), int(center.y()-radius*2), int(radius*4), int(radius*2+pitch_offset))
        painter.setBrush(self.ground_color)
        painter.drawRect(int(center.x()-radius*2), int(center.y()+pitch_offset), int(radius*4), int(radius*2))
        painter.setPen(self.grid_color)
        for i in range(-10,11): painter.drawLine(int(center.x()-radius*2), int(center.y()+pitch_offset+i*20), int(center.x()+radius*2), int(center.y()+pitch_offset+i*20))
        painter.restore()
        painter.setPen(QPen(QColor("#ffff00"), 3))
        painter.drawLine(QPointF(center.x()-40, center.y()), QPointF(center.x()-15, center.y())); painter.drawLine(QPointF(center.x()+15, center.y()), QPointF(center.x()+40, center.y()))
        painter.drawLine(QPointF(center.x(), center.y()-5), QPointF(center.x()-15, center.y())); painter.drawLine(QPointF(center.x(), center.y()-5), QPointF(center.x()+15, center.y()))

class HeadingIndicator(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(200, 200); self.yaw = 0.0
        self.rose_pen, self.rose_brush = QPen(QColor(0,255,255),2), QColor("#0d1117"); self.marker_pen, self.pointer_pen = QPen(QColor("#e6edf3")), QPen(QColor("#ffff00"),3)
        self.font, self.value_font = QFont("Consolas",10,QFont.Weight.Bold), QFont("Consolas",16,QFont.Weight.Bold)
    def set_heading(self, yaw): self.yaw = yaw; self.update()
    def paintEvent(self, event):
        painter = QPainter(self); painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        width, height = self.width(), self.height(); center = QPointF(width/2, height/2); radius = min(width,height)/2-10
        painter.setPen(self.rose_pen); painter.setBrush(self.rose_brush); painter.drawEllipse(center, radius, radius); painter.save()
        painter.translate(center); painter.rotate(-math.degrees(self.yaw)); painter.setFont(self.font)
        for angle in range(0,360,15):
            painter.save(); painter.rotate(angle); painter.setPen(self.marker_pen)
            if angle%90==0: painter.drawLine(0,int(-radius),0,int(-radius+12)); char={0:'N',90:'E',180:'S',270:'W'}[angle]; painter.drawText(QRectF(-10,-radius+15,20,15),Qt.AlignmentFlag.AlignCenter,char)
            elif angle%30==0: painter.drawLine(0,int(-radius),0,int(-radius+8))
            else: painter.drawLine(0,int(-radius),0,int(-radius+4))
            painter.restore()
        painter.restore()
        painter.setPen(self.pointer_pen); path=QPainterPath(); path.moveTo(center.x(),center.y()-radius-5); path.lineTo(center.x()-8,center.y()-radius+10); path.lineTo(center.x()+8,center.y()-radius+10); path.closeSubpath(); painter.drawPath(path)
        painter.setFont(self.value_font); painter.setPen(self.marker_pen); heading_deg=(math.degrees(self.yaw)+360)%360; painter.drawText(self.rect(),Qt.AlignmentFlag.AlignCenter|Qt.AlignmentFlag.AlignBottom,f"{heading_deg:.1f}°")

# --- MAIN GUI WINDOW ---

class GCSApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ISRO Drone Mission Control")
        self.setGeometry(100, 100, 1600, 900)
        self.setStyleSheet(STYLESHEET)
        self.drone_worker = None
        self.video_worker = None
        self.vehicle_data = {}
        self.initUI()
        self.show()

    def initUI(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        sidebar = QFrame(self); sidebar.setObjectName("Sidebar"); sidebar.setFixedWidth(280)
        sidebar_layout = QVBoxLayout(sidebar)
        title_label = QLabel("MISSION CONTROL"); title_label.setAlignment(Qt.AlignmentFlag.AlignCenter); title_label.setStyleSheet("font-size: 22px; font-weight: 800; color: #00d4ff; padding: 12px; letter-spacing: 0.5px;")
        self.connection_string_input = QLineEdit("COM8"); self.connection_string_input.setToolTip("Enter connection string (e.g., COM8 or udp:127.0.0.1:14550)")
        self.connect_button = QPushButton("Connect"); self.connect_button.setMinimumHeight(36); self.connect_button.setToolTip("Connect or disconnect from the vehicle"); self.connect_button.clicked.connect(self.toggle_connection)
        self.connection_status_label = QLabel("Status: Disconnected"); self.connection_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter); self.connection_status_label.setStyleSheet("padding: 6px; background-color: #e05757; border-radius: 8px; color: #04151f;")
        nav_header = QLabel("VIEWS"); nav_header.setObjectName("HeaderLabel")
        self.dashboard_button = QPushButton("Dashboard"); self.flight_data_button = QPushButton("Flight Data"); self.map_view_button = QPushButton("VIO Map View")
        for b in (self.dashboard_button, self.flight_data_button, self.map_view_button): b.setCheckable(True); b.setMinimumHeight(36); b.setObjectName("NavButton")
        self.dashboard_button.setChecked(True)
        self.dashboard_button.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(0))
        self.flight_data_button.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(1))
        self.map_view_button.clicked.connect(lambda: self.stacked_widget.setCurrentIndex(2))
        log_header = QLabel("MESSAGE LOG"); log_header.setObjectName("HeaderLabel")
        self.log_console = QPlainTextEdit(); self.log_console.setReadOnly(True); self.log_console.setToolTip("System and vehicle messages will appear here")
        sidebar_layout.addWidget(title_label); sidebar_layout.addWidget(self.connection_string_input); sidebar_layout.addWidget(self.connect_button); sidebar_layout.addWidget(self.connection_status_label); sidebar_layout.addSpacing(20); sidebar_layout.addWidget(nav_header); sidebar_layout.addWidget(self.dashboard_button); sidebar_layout.addWidget(self.flight_data_button); sidebar_layout.addWidget(self.map_view_button); sidebar_layout.addStretch(); sidebar_layout.addWidget(log_header); sidebar_layout.addWidget(self.log_console)

        self.stacked_widget = QStackedWidget()
        self.create_dashboard_page()
        self.create_flight_data_page()
        self.create_map_view_page()
        main_layout.addWidget(sidebar)
        main_layout.addWidget(self.stacked_widget)
        self.log_message("System Initialized. Ready to connect.")

    def create_dashboard_page(self):
        page = QWidget(); layout = QGridLayout(page)
        video_frame = QFrame(); video_frame.setObjectName("Card"); video_layout = QVBoxLayout(video_frame)
        video_header = QLabel("Live Video Feed"); video_header.setObjectName("HeaderLabel")
        self.video_label = QLabel("Not Connected"); self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter); self.video_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding); self.video_label.setMinimumSize(360, 240); self.video_label.setStyleSheet("background-color: #050a14; border: 1px solid #22324a; border-radius: 10px;"); self.video_label.setToolTip("Camera preview")
        
        vio_controls_layout = QHBoxLayout()
        self.vio_toggle_button = QPushButton("Disable VIO Edge Overlay"); self.vio_toggle_button.setCheckable(True); self.vio_toggle_button.setChecked(True)
        self.vio_toggle_button.toggled.connect(self.toggle_vio_overlay)
        
        self.edge_count_label = QLabel("Edges Detected: N/A")
        # CHANGE: Set object name for specific styling
        self.edge_count_label.setObjectName("EdgeCountLabel")
        self.edge_count_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        vio_controls_layout.addWidget(self.vio_toggle_button)
        vio_controls_layout.addSpacing(10)
        vio_controls_layout.addWidget(self.edge_count_label, 1) # Add stretch factor

        video_layout.addWidget(video_header); video_layout.addWidget(self.video_label, 1); video_layout.addLayout(vio_controls_layout)

        instruments_frame = QFrame(); instruments_frame.setObjectName("Card"); instruments_layout = QHBoxLayout(instruments_frame); self.attitude_indicator = AttitudeIndicator(); self.heading_indicator = HeadingIndicator(); instruments_layout.addWidget(self.attitude_indicator); instruments_layout.addWidget(self.heading_indicator)
        status_frame = QFrame(); status_frame.setObjectName("Card"); status_layout = QVBoxLayout(status_frame); status_header = QLabel("Vehicle Status & Controls"); status_header.setObjectName("HeaderLabel"); self.status_grid = QGridLayout(); self.mode_label = self.create_data_display("Mode", "N/A"); self.battery_label = self.create_data_display("Battery", "N/A"); self.altitude_label = self.create_data_display("Altitude", "N/A"); self.armed_label = self.create_data_display("State", "DISARMED"); self.status_grid.addWidget(self.mode_label[0], 0, 0); self.status_grid.addWidget(self.battery_label[0], 1, 0); self.status_grid.addWidget(self.altitude_label[0], 0, 1); self.status_grid.addWidget(self.armed_label[0], 1, 1); motor_control_header = QLabel("Motor Test System"); motor_control_header.setObjectName("HeaderLabel"); test_input_layout = QHBoxLayout(); throttle_label = QLabel("Throttle %:"); self.throttle_input = QLineEdit("5.0"); self.throttle_input.setFixedWidth(72); self.throttle_input.setToolTip("Motor throttle percentage (0–100)"); duration_label = QLabel("Duration (s):"); self.duration_input = QLineEdit("2.0"); self.duration_input.setFixedWidth(72); self.duration_input.setToolTip("How long to spin (0.1–10.0 seconds)"); test_input_layout.addWidget(throttle_label); test_input_layout.addWidget(self.throttle_input); test_input_layout.addSpacing(10); test_input_layout.addWidget(duration_label); test_input_layout.addWidget(self.duration_input); test_input_layout.addStretch(); test_buttons_layout = QHBoxLayout(); self.test_all_button = QPushButton("Spin All Motors"); self.test_all_button.setMinimumHeight(36); self.test_all_button.setToolTip("Spin all motors together"); self.test_all_button.clicked.connect(self.send_motor_command_all); self.test_sequence_button = QPushButton("Spin in Sequence"); self.test_sequence_button.setMinimumHeight(36); self.test_sequence_button.setToolTip("Spin each motor one-by-one"); self.test_sequence_button.clicked.connect(self.send_motor_command_sequence); test_buttons_layout.addWidget(self.test_all_button); test_buttons_layout.addWidget(self.test_sequence_button); self.arm_button = QPushButton("ARM"); self.arm_button.setObjectName("ArmButton"); self.arm_button.setMinimumHeight(36); self.arm_button.setToolTip("Arm the vehicle"); self.arm_button.clicked.connect(self.arm_vehicle); self.disarm_button = QPushButton("DISARM"); self.disarm_button.setObjectName("DisarmButton"); self.disarm_button.setMinimumHeight(36); self.disarm_button.setToolTip("Disarm the vehicle"); self.disarm_button.clicked.connect(self.disarm_vehicle)
        status_layout.addWidget(status_header); status_layout.addLayout(self.status_grid); status_layout.addWidget(motor_control_header); status_layout.addLayout(test_input_layout); status_layout.addLayout(test_buttons_layout); status_layout.addStretch(); status_layout.addWidget(self.arm_button); status_layout.addWidget(self.disarm_button)
        layout.addWidget(video_frame, 0, 0, 2, 1); layout.addWidget(instruments_frame, 0, 1); layout.addWidget(status_frame, 1, 1); layout.setColumnStretch(0, 3); layout.setColumnStretch(1, 2)
        self.stacked_widget.addWidget(page)

    def create_flight_data_page(self):
        page = QWidget(); layout = QGridLayout(page); self.telemetry_labels = {}
        data_points = {"Position (Local Frame)": ["North (m)", "East (m)", "Down (m)"], "Attitude": ["Roll (deg)", "Pitch (deg)", "Yaw (deg)"], "Power": ["Voltage (V)", "Level (%)"],}
        for col, (header, labels) in enumerate(data_points.items()):
            header_label = QLabel(header); header_label.setObjectName("HeaderLabel"); layout.addWidget(header_label, 0, col)
            for row, label_text in enumerate(labels, 1):
                key = label_text.split(' ')[0].lower()
                data_display = self.create_data_display(label_text, "0.00"); layout.addWidget(data_display[0], row, col); self.telemetry_labels[key] = data_display[1]
        layout.setRowStretch(len(data_points) + 1, 1); layout.setColumnStretch(len(data_points), 1)
        self.stacked_widget.addWidget(page)

    def create_map_view_page(self):
        page = QWidget(); layout = QVBoxLayout(page); header = QLabel("VIO / SLAM Position (2D View)"); header.setObjectName("HeaderLabel")
        self.plot_widget = pg.PlotWidget(); self.plot_widget.setBackground('#0b1220'); self.plot_widget.showGrid(x=True, y=True, alpha=0.25); self.plot_widget.setLabel('left','North (m)'); self.plot_widget.setLabel('bottom','East (m)'); self.plot_widget.setAspectLocked(True)
        try: axis_pen=pg.mkPen(color=(120,150,180),width=1); self.plot_widget.getAxis('left').setTextPen(axis_pen); self.plot_widget.getAxis('bottom').setTextPen(axis_pen); self.plot_widget.getAxis('left').setPen(axis_pen); self.plot_widget.getAxis('bottom').setPen(axis_pen)
        except Exception: pass
        self.drone_path = self.plot_widget.plot(pen=pg.mkPen(color=(255,194,75),width=2)); self.drone_symbol=pg.ScatterPlotItem(size=15,pen=pg.mkPen(None),brush=pg.mkBrush(0,212,255)); self.plot_widget.addItem(self.drone_symbol); self.path_data={'x':[],'y':[]}
        layout.addWidget(header); layout.addWidget(self.plot_widget); self.stacked_widget.addWidget(page)

    def create_data_display(self, name, initial_value):
        container = QFrame(); container.setObjectName("Card"); layout = QHBoxLayout(container); name_label = QLabel(name); value_label = QLabel(initial_value); value_label.setObjectName("DataLabel"); value_label.setAlignment(Qt.AlignmentFlag.AlignRight); layout.addWidget(name_label); layout.addWidget(value_label)
        return container, value_label

    def toggle_connection(self):
        if not (self.drone_worker and self.drone_worker.isRunning()):
            connection_str = self.connection_string_input.text().strip()
            self.log_message(f"Attempting to connect to {connection_str}...")
            self.drone_worker = DroneWorker(connection_str); self.drone_worker.connection_status.connect(self.update_connection_status); self.drone_worker.vehicle_data_updated.connect(self.update_vehicle_data); self.drone_worker.start()
            self.connect_button.setText("Disconnect")
            if not (self.video_worker and self.video_worker.isRunning()):
                self.video_worker = VideoWorker(0); self.video_worker.frame_updated.connect(self.update_video_frame); self.video_worker.start()
        else:
            self.log_message("Disconnecting...")
            if self.drone_worker: self.drone_worker.stop()
            if self.video_worker: self.video_worker.stop()
            self.connect_button.setText("Connect")

    def toggle_vio_overlay(self, checked):
        if self.video_worker:
            self.video_worker.set_vio_enabled(checked); status = "Enabled" if checked else "Disabled"; self.log_message(f"VIO Edge Overlay {status}.")
            self.vio_toggle_button.setText(f"{'Disable' if checked else 'Enable'} VIO Edge Overlay")
        else: self.log_message("Cannot toggle VIO: Video feed not active."); self.vio_toggle_button.setChecked(False)

    def update_connection_status(self, message, color):
        self.connection_status_label.setText(f"Status: {message}"); self.connection_status_label.setStyleSheet(f"padding: 6px; background-color: {color}; border-radius: 8px;")
        if color != 'orange': self.log_message(message)

    def update_vehicle_data(self, data):
        self.vehicle_data.update(data); pitch,roll,yaw = data.get('pitch',0),data.get('roll',0),data.get('yaw',0)
        self.attitude_indicator.set_attitude(pitch, roll); self.heading_indicator.set_heading(yaw)
        self.mode_label[1].setText(data.get('mode','N/A')); self.battery_label[1].setText(f"{data.get('level',0) or 0}%"); self.altitude_label[1].setText(f"{-data.get('down',0) if data.get('down') is not None else 0:.2f} m")
        is_armed = data.get('is_armed',False); armed_state = "ARMED" if is_armed else "DISARMED"; armed_color = "#3fb950" if is_armed else "#f85149"
        self.armed_label[1].setText(armed_state); self.armed_label[1].setStyleSheet(f"color: {armed_color}; font-weight: bold;")
        self.telemetry_labels['north'].setText(f"{data.get('north',0) or 0:.2f}"); self.telemetry_labels['east'].setText(f"{data.get('east',0) or 0:.2f}"); self.telemetry_labels['down'].setText(f"{data.get('down',0) or 0:.2f}"); self.telemetry_labels['roll'].setText(f"{math.degrees(roll):.2f}"); self.telemetry_labels['pitch'].setText(f"{math.degrees(pitch):.2f}"); self.telemetry_labels['yaw'].setText(f"{(math.degrees(yaw)+360)%360:.2f}"); self.telemetry_labels['voltage'].setText(f"{data.get('voltage',0) or 0:.2f}"); self.telemetry_labels['level'].setText(f"{data.get('level',0) or 0}")
        north, east = data.get('north'), data.get('east')
        if north is not None and east is not None:
            self.path_data['x'].append(east); self.path_data['y'].append(north)
            if len(self.path_data['x'])>500: self.path_data['x'].pop(0); self.path_data['y'].pop(0)
            self.drone_path.setData(self.path_data['x'], self.path_data['y']); self.drone_symbol.setData([east],[north])

    def update_video_frame(self, frame, edge_count):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        qt_image = QImage(rgb_image.data, w, h, ch * w, QImage.Format.Format_RGB888)
        self.video_label.setPixmap(QPixmap.fromImage(qt_image).scaled(self.video_label.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))

        if edge_count >= 0:
            self.edge_count_label.setText(f"Edges Detected: {edge_count}")
        else:
            self.edge_count_label.setText("VIO Disabled")

    def arm_vehicle(self):
        if self.drone_worker and self.drone_worker.isRunning(): self.drone_worker.arm_vehicle()
    def disarm_vehicle(self):
        if self.drone_worker and self.drone_worker.isRunning(): self.drone_worker.disarm_vehicle()
    def _get_test_params(self):
        if not (self.drone_worker and self.drone_worker.isRunning()): self.log_message("Error: Not connected to a vehicle."); return None, None
        try:
            throttle = float(self.throttle_input.text()); duration = float(self.duration_input.text())
            if not (0 <= throttle <= 100): self.log_message("Error: Throttle must be between 0 and 100."); return None, None
            if not (0.1 <= duration <= 10): self.log_message("Error: Duration must be between 0.1 and 10s."); return None, None
            return throttle, duration
        except ValueError: self.log_message("Error: Invalid throttle/duration. Enter numbers."); return None, None

    def send_motor_command_all(self):
        throttle, duration = self._get_test_params()
        if throttle is not None: self.drone_worker.start_motor_test_all(throttle, duration)
    def send_motor_command_sequence(self):
        throttle, duration = self._get_test_params()
        if throttle is not None: self.drone_worker.start_motor_test_sequence(throttle, duration)

    def log_message(self, message):
        self.log_console.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {message}")
    def closeEvent(self, event):
        if self.drone_worker: self.drone_worker.stop()
        if self.video_worker: self.video_worker.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gcs = GCSApp()
    sys.exit(app.exec())