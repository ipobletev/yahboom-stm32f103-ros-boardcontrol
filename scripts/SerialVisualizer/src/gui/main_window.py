import sys
import time
import csv
import os
from datetime import datetime
from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                              QLabel, QComboBox, QPushButton, QGroupBox, 
                              QTabWidget, QFileDialog)
from PySide6.QtCore import QTimer, Qt, Slot
import serial.tools.list_ports
import numpy as np

from communication.serial_manager import SerialManager
from core.protocol import parse_machine_info, parse_imu, parse_encoder, parse_pid_debug, pack_cmd_vel, pack_enum, pack_config
from gui.tabs.raw_data_tab import RawDataTab
from gui.tabs.graphs_tab import GraphsTab
from gui.tabs.status_tab import StatusTab
from gui.tabs.view_3d_tab import View3DTab
from gui.tabs.connection_tab import ConnectionTab
from gui.tabs.pid_tuning_tab import PIDTuningTab

class SerialVisualizerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 Control Board Serial Visualizer")
        self.resize(1200, 900)

        self.serial_manager = SerialManager()
        
        # UI State
        self.board_connected = False
        self.last_machine_info_time = 0
        self.topic_stats = {} # {id: {"count": X, "freq": Y}}
        self.last_stats_time = time.time()
        
        # Data Cache
        self.current_data_cache = {}
        
        # Recording/Playback
        self.is_recording = False
        self.recorded_data = []
        self.recording_start_time = 0
        self.playback_data = []
        self.playback_index = 0
        self.playback_timer = QTimer()
        self.playback_timer.timeout.connect(self.update_playback)
        self.playback_interval = 50 # ms

        # Heartbeat timer (2s)
        self.heartbeat_timer = QTimer()
        self.heartbeat_timer.timeout.connect(self.send_heartbeat)

        # Error mapping
        self.sys_errors = {
            (1 << 0): "IMU Init Error",
            (1 << 1): "Motor Init Error",
            (1 << 2): "Encoder Init Error",
            (1 << 3): "Serial ROS Init Error",
            (1 << 8): "IMU Timeout",
            (1 << 9): "Serial Timeout",
            (1 << 10): "Command Timeout",
            (1 << 16): "Queue Full",
            (1 << 17): "Timer Fail",
            (1 << 18): "Malloc Fail",
            (1 << 24): "Watchdog Reset",
            (1 << 25): "Brownout Detected",
            (1 << 26): "Overtemperature",
        }

        self.setup_ui()
        self.setup_connections()
        
        # Timers
        self.port_timer = QTimer(); self.port_timer.timeout.connect(self.update_ports); self.port_timer.start(2000)
        self.stats_timer = QTimer(); self.stats_timer.timeout.connect(self.refresh_stats); self.stats_timer.start(1000)
        
        self.update_ports()

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Connection Group
        conn_group = QGroupBox("Connection")
        conn_layout = QHBoxLayout(conn_group)
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "115200", "230400", "460800", "921600"])
        self.baud_combo.setCurrentText("115200")
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connect)
        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.port_combo)
        conn_layout.addWidget(QLabel("Baud:"))
        conn_layout.addWidget(self.baud_combo)
        conn_layout.addWidget(self.connect_btn)
        
        self.lbl_conn_status = QLabel("○ Disconnected")
        self.lbl_conn_status.setStyleSheet("color: gray; font-weight: bold; margin-left: 20px;")
        conn_layout.addWidget(self.lbl_conn_status)
        conn_layout.addStretch()
        main_layout.addWidget(conn_group)

        # Tabs
        self.tabs = QTabWidget()
        self.raw_tab = RawDataTab()
        self.graphs_tab = GraphsTab()
        self.status_tab = StatusTab(self.sys_errors)
        self.view_3d_tab = View3DTab()
        self.conn_tab = ConnectionTab()
        self.pid_tuning_tab = PIDTuningTab()
        
        self.tabs.addTab(self.raw_tab, "Raw Data")
        self.tabs.addTab(self.graphs_tab, "Graphs")
        self.tabs.addTab(self.pid_tuning_tab, "PID Tuning")
        self.tabs.addTab(self.status_tab, "System Status")
        self.tabs.addTab(self.view_3d_tab, "3D Orientation")
        self.tabs.addTab(self.conn_tab, "Connection Freq")
        
        main_layout.addWidget(self.tabs)

    def setup_connections(self):
        self.serial_manager.data_received.connect(self.handle_serial_data, Qt.QueuedConnection)
        
        # Raw Tab Signals
        self.raw_tab.velocity_sent.connect(self.send_velocity)
        self.raw_tab.mode_manual_requested.connect(self.set_mode_manual)
        self.raw_tab.mode_auto_requested.connect(self.set_mode_auto)
        self.raw_tab.run_requested.connect(self.send_run)
        self.raw_tab.estop_requested.connect(self.send_estop)
        self.raw_tab.reset_stop_requested.connect(self.send_reset_stop)
        self.raw_tab.recording_toggled.connect(self.toggle_recording)
        self.raw_tab.csv_load_requested.connect(self.load_csv)
        self.raw_tab.playback_toggled.connect(self.toggle_playback)
        
        # PID Tuning Tab Signals
        self.pid_tuning_tab.pid_update_requested.connect(self.send_pid_config)
        self.pid_tuning_tab.wheel_diam_update_requested.connect(self.send_wheel_config)

    @Slot(int, bytes)
    def handle_serial_data(self, topic_id, payload):
        if self.playback_timer.isActive(): return

        # Stats
        if topic_id not in self.topic_stats: self.topic_stats[topic_id] = {"count": 0, "freq": 0}
        self.topic_stats[topic_id]["count"] += 1
        self.conn_tab.update_frame(topic_id)

        if topic_id == self.serial_manager.protocol.TOPIC_PUB_MACHINE_INFO:
            self.last_machine_info_time = time.time()
            if not self.board_connected:
                self.board_connected = True
                self.lbl_conn_status.setText("● Connected")
                self.lbl_conn_status.setStyleSheet("color: #2ecc71; font-weight: bold; margin-left: 20px;")
            
            info = parse_machine_info(payload)
            if info:
                self.current_data_cache.update({
                    "state": info["state"], "mode": info["mode"], "wheels": int(info["moving_wheels"]),
                    "spatial": int(info["moving_spatial"]), "error": info["error_code"], "roll": info["roll"],
                    "pitch": info["pitch"], "velocity": info["velocity"], "battery": info["battery"],
                    "temperature": info["temperature"], "angular_velocity": info.get("angular_velocity", 0.0)
                })
                self.raw_tab.update_machine_info(info)
                self.status_tab.update_errors(info["error_code"])

        elif topic_id == self.serial_manager.protocol.TOPIC_PUB_IMU:
            imu = parse_imu(payload)
            if imu:
                self.current_data_cache.update({
                    "acc_x": imu["acc"][0], "acc_y": imu["acc"][1], "acc_z": imu["acc"][2],
                    "gyro_x": imu["gyro"][0], "gyro_y": imu["gyro"][1], "gyro_z": imu["gyro"][2],
                    "mag_x": imu["mag"][0], "mag_y": imu["mag"][1], "mag_z": imu["mag"][2]
                })
                self.raw_tab.update_imu_labels(imu)
                self.graphs_tab.update_imu(imu)
                self.view_3d_tab.update_orientation(self.current_data_cache)

        elif topic_id == self.serial_manager.protocol.TOPIC_PUB_ENCODER:
            encoders = parse_encoder(payload)
            if encoders:
                self.current_data_cache.update({
                    "enc_fl": encoders[0], "enc_fr": encoders[1], "enc_bl": encoders[2], "enc_br": encoders[3]
                })
                self.raw_tab.update_encoder_labels(encoders)
                self.graphs_tab.update_encoders(encoders)

        elif topic_id == self.serial_manager.protocol.TOPIC_PUB_PID_DEBUG:
            pid_debug = parse_pid_debug(payload)
            if pid_debug:
                self.pid_tuning_tab.update_pid_debug(pid_debug)

        if self.is_recording:
            row = {"timestamp": time.time() - self.recording_start_time}
            row.update(self.current_data_cache)
            self.recorded_data.append(row)
            self.raw_tab.lbl_rec_count.setText(f"Samples: {len(self.recorded_data)}")

    def toggle_connect(self):
        if not self.serial_manager.is_connected():
            port = self.port_combo.currentText()
            baud = int(self.baud_combo.currentText())
            if self.serial_manager.connect(port, baud):
                self.connect_btn.setText("Disconnect")
                self.heartbeat_timer.start(2000)
        else:
            self.serial_manager.disconnect()
            self.heartbeat_timer.stop()
            self.connect_btn.setText("Connect")
            self.board_connected = False
            self.lbl_conn_status.setText("○ Disconnected")
            self.lbl_conn_status.setStyleSheet("color: gray; font-weight: bold; margin-left: 20px;")
            self.reset_ui()

    def update_ports(self):
        current = self.port_combo.currentText()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current in ports: self.port_combo.setCurrentText(current)

    def refresh_stats(self):
        now = time.time()
        dt = now - self.last_stats_time
        if dt > 0:
            freqs = {}
            for tid, info in self.topic_stats.items():
                freq = info["count"] / dt
                info["freq"] = freq
                info["count"] = 0
                freqs[tid] = freq
            self.raw_tab.set_freqs(freqs)
            if self.board_connected:
                self.conn_tab.update_freqs(freqs)
        self.last_stats_time = now
        
        # Watchdog
        if self.board_connected and (now - self.last_machine_info_time > 2.5):
            self.board_connected = False
            self.lbl_conn_status.setText("○ Disconnected")
            self.lbl_conn_status.setStyleSheet("color: red; font-weight: bold; margin-left: 20px;")
            self.reset_ui()

    def reset_ui(self):
        self.raw_tab.reset_placeholders()
        self.status_tab.reset_placeholders()
        self.conn_tab.reset_placeholders()
        self.pid_tuning_tab.reset_placeholders()

    # Commands
    def send_pid_config(self, motor_id, values):
        # item_id for motor PID is same as motor_id (0-3)
        self.serial_manager.write(self.serial_manager.protocol.TOPIC_SUB_CONFIG, pack_config(motor_id, values))

    def send_wheel_config(self, diameter):
        # item_id for wheel diameter is 4
        self.serial_manager.write(self.serial_manager.protocol.TOPIC_SUB_CONFIG, pack_config(4, diameter))
    def send_velocity(self, lx, az):
        self.serial_manager.write(self.serial_manager.protocol.TOPIC_SUB_CMD_VEL, pack_cmd_vel(lx, 0, 0, az))

    def set_mode_manual(self):
        self.serial_manager.write(self.serial_manager.protocol.TOPIC_SUB_OPERATION_MODE, pack_enum(0)) # MODE_MANUAL

    def set_mode_auto(self):
        self.serial_manager.write(self.serial_manager.protocol.TOPIC_SUB_OPERATION_MODE, pack_enum(1)) # MODE_AUTONOMOUS

    def send_run(self):
        self.serial_manager.write(self.serial_manager.protocol.TOPIC_SUB_OPERATION_RUN, pack_enum(0))

    def send_estop(self):
        self.serial_manager.write(self.serial_manager.protocol.TOPIC_SUB_ESTOP_CMD, pack_enum(3))

    def send_reset_stop(self):
        self.serial_manager.write(self.serial_manager.protocol.TOPIC_SUB_RESET_STOP_CMD, pack_enum(2))

    def send_heartbeat(self):
        if self.serial_manager.is_connected():
            # Send an empty byte to confirm connection
            self.serial_manager.write(self.serial_manager.protocol.TOPIC_SUB_CONFIRM_CONN, pack_enum(0))

    # Recording
    def toggle_recording(self, active):
        if active:
            self.is_recording = True
            self.recorded_data = []
            self.recording_start_time = time.time()
            self.raw_tab.btn_toggle_rec.setText("Stop & Save")
            self.raw_tab.lbl_rec_status.setText("Status: RECORDING...")
            self.raw_tab.lbl_rec_status.setStyleSheet("color: red; font-weight: bold;")
        else:
            self.is_recording = False
            self.raw_tab.btn_toggle_rec.setText("Start Recording")
            self.raw_tab.lbl_rec_status.setText("Status: Idle")
            self.raw_tab.lbl_rec_status.setStyleSheet("")
            if self.recorded_data: self.save_csv()

    def save_csv(self):
        name = f"session_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path, _ = QFileDialog.getSaveFileName(self, "Save Recording", name, "CSV (*.csv)")
        if path:
            with open(path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.recorded_data[0].keys())
                writer.writeheader()
                writer.writerows(self.recorded_data)

    # Playback
    def load_csv(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load Recording", "", "CSV (*.csv)")
        if path:
            self.playback_data = []
            with open(path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    self.playback_data.append({k: (float(v) if k != "timestamp" else v) for k, v in row.items()})
            self.raw_tab.lbl_loaded_file.setText(os.path.basename(path))
            self.raw_tab.btn_play_pause.setEnabled(True)

    def toggle_playback(self, active):
        if active:
            if self.serial_manager.is_connected(): self.toggle_connect()
            self.playback_index = 0
            self.playback_timer.start(self.playback_interval)
            self.raw_tab.btn_play_pause.setText("Pause")
        else:
            self.playback_timer.stop()
            self.raw_tab.btn_play_pause.setText("Play")

    def update_playback(self):
        if self.playback_index >= len(self.playback_data):
            if self.raw_tab.chk_loop.isChecked(): self.playback_index = 0
            else:
                self.raw_tab.btn_play_pause.setChecked(False)
                return
        
        row = self.playback_data[self.playback_index]
        self.playback_index += 1
        self.raw_tab.play_progress.setValue(int(100 * self.playback_index / len(self.playback_data)))
        
        self.current_data_cache.update(row)
        # Update components from cache
        self.raw_tab.update_machine_info({
            "state": int(row["state"]), "mode": int(row["mode"]), "moving_wheels": bool(row["wheels"]),
            "moving_spatial": bool(row["spatial"]), "error_code": int(row["error"]), "roll": row["roll"],
            "pitch": row["pitch"], "velocity": row["velocity"], "battery": row["battery"],
            "temperature": row["temperature"], "angular_velocity": row.get("angular_velocity", 0.0)
        })
        self.raw_tab.update_encoder_labels([row["enc_fl"], row["enc_fr"], row["enc_bl"], row["enc_br"]])
        self.graphs_tab.update_from_cache(row)
        self.view_3d_tab.update_orientation(row)
