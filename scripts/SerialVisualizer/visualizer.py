import sys
import serial
import serial.tools.list_ports
import struct
import threading
import time
from datetime import datetime
import csv
import os
import math
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QPushButton, 
                             QGridLayout, QGroupBox, QDoubleSpinBox, QTabWidget,
                             QTableWidget, QTableWidgetItem, QHeaderView, QFileDialog,
                             QLineEdit, QScrollArea, QCheckBox, QProgressBar, QSizePolicy) # Added QLineEdit, QScrollArea, QCheckBox, QSizePolicy
from PySide6.QtCore import QTimer, Qt, Signal, Slot
import pyqtgraph as pg
import pyqtgraph.opengl as gl # Added pyqtgraph.opengl
import numpy as np

from serial_ros import (SerialProtocol, parse_machine_info, parse_imu, 
                        parse_encoder, pack_cmd_vel, pack_enum)


class KalmanAngle:
    def __init__(self):
        self.Q_angle = 0.001
        self.Q_bias = 0.003
        self.R_measure = 0.03
        self.angle = 0.0
        self.bias = 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]

    def compute(self, newAngle, newRate, dt):
        # Predict
        rate = newRate - self.bias
        self.angle += dt * rate

        # Update P
        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Innovation
        y = newAngle - self.angle
        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]

        # Update state
        self.angle += K[0] * y
        self.bias += K[1] * y

        # Update covariance P
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle

class SerialVisualizer(QMainWindow):
    data_received = Signal(int, bytes)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 Control Board Serial Visualizer")
        self.resize(1200, 900)

        self.protocol = SerialProtocol()
        self.serial_port = None
        self.running = False

        # Data storage for plots
        self.max_points = 200
        self.imu_data = {
            "acc": [np.zeros(self.max_points) for _ in range(3)],
            "gyro": [np.zeros(self.max_points) for _ in range(3)],
            "mag": [np.zeros(self.max_points) for _ in range(3)]
        }
        self.encoder_data = [np.zeros(self.max_points) for _ in range(4)]

        # Topic statistics
        self.topic_stats = {
            self.protocol.TOPIC_PUB_MACHINE_INFO: {"name": "MACHINE_INFO", "count": 0, "freq": 0.0},
            self.protocol.TOPIC_PUB_IMU: {"name": "IMU", "count": 0, "freq": 0.0},
            self.protocol.TOPIC_PUB_ENCODER: {"name": "ENCODER", "count": 0, "freq": 0.0}
        }
        self.last_stats_time = time.time()
        self.last_machine_info_time = 0
        self.board_connected = False
        self.lbl_topic_freqs = {}

        # Error mapping (matching app_errors.h)
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

        # Recording state
        self.is_recording = False
        self.recorded_data = [] # List of dicts
        self.recording_start_time = 0
        
        # Cache for 3D view data
        self.current_data_cache = {}
        
        # Playback
        self.playback_data = [] # List of dicts
        self.playback_index = 0
        self.is_playing = False
        self.playback_timer = QTimer()
        self.playback_timer.timeout.connect(self.update_playback)
        self.playback_interval = 50 # ms

        self.setup_ui()
        
        self.data_received.connect(self.handle_data)
        
        # Timer to update port list
        self.port_timer = QTimer()
        self.port_timer.timeout.connect(self.update_ports)
        self.port_timer.start(2000)
        
        # Timer to update topic stats frequency
        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self.refresh_topic_stats)
        self.stats_timer.start(1000)
        
        # Kalman Filters
        self.kalman_roll = KalmanAngle()
        self.kalman_pitch = KalmanAngle()
        self.kalman_yaw = KalmanAngle()
        self.last_imu_time = 0
        
        self.update_ports()

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # --- Connection Row ---
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

        # --- Tab Widget ---
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        self.setup_raw_data_tab()
        self.setup_graphs_tab()
        self.setup_status_tab()
        
        # 3D View Tab
        self.view_3d_tab = self.create_3d_view_tab()
        self.tabs.addTab(self.view_3d_tab, "3D Orientation")

    def setup_status_tab(self):
        status_tab = QWidget()
        self.tabs.addTab(status_tab, "System Status")
        layout = QVBoxLayout(status_tab)

        # Errors Table
        err_detailed_group = QGroupBox("Health Monitoring & Error Codes")
        err_detailed_layout = QVBoxLayout(err_detailed_group)
        self.error_table = QTableWidget(len(self.sys_errors), 3)
        self.error_table.setHorizontalHeaderLabels(["Component / System", "Error Code", "Status"])
        self.error_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.error_table.verticalHeader().setVisible(False)
        self.error_table.setEditTriggers(QTableWidget.NoEditTriggers)
        
        # Pre-fill table with "-" (No data yet)
        for row, (bit, name) in enumerate(self.sys_errors.items()):
            self.error_table.setItem(row, 0, QTableWidgetItem(name))
            
            # Error Code Column (Bitmask)
            item_code = QTableWidgetItem(f"0x{bit:08X}")
            item_code.setTextAlignment(Qt.AlignCenter)
            item_code.setForeground(Qt.gray)
            self.error_table.setItem(row, 1, item_code)

            # Status Column
            item_status = QTableWidgetItem("-")
            item_status.setTextAlignment(Qt.AlignCenter)
            item_status.setForeground(Qt.gray)
            self.error_table.setItem(row, 2, item_status)
            
        err_detailed_layout.addWidget(self.error_table)
        layout.addWidget(err_detailed_group)
        layout.addStretch()

    def setup_raw_data_tab(self):
        raw_tab = QWidget()
        self.tabs.addTab(raw_tab, "Raw Data")
        layout = QVBoxLayout(raw_tab)

        # Dashboard Area
        dash_layout = QHBoxLayout()
        
        # Machine Info
        info_group = QGroupBox("Machine Info")
        info_grid = QGridLayout(info_group)
        self.lbl_state = QLabel("-")
        self.lbl_mode = QLabel("-")
        self.lbl_moving_wheels = QLabel("-")
        self.lbl_moving_spatial = QLabel("-")
        self.lbl_roll = QLabel("-")
        self.lbl_pitch = QLabel("-")
        self.lbl_velocity = QLabel("-")
        self.lbl_battery = QLabel("-")
        self.lbl_temperature = QLabel("-")
        self.lbl_angular_velocity = QLabel("-")
        self.lbl_error_code = QLabel("-")
        info_grid.addWidget(QLabel("State:"), 0, 0)
        info_grid.addWidget(self.lbl_state, 0, 1)
        info_grid.addWidget(QLabel("Mode:"), 1, 0)
        info_grid.addWidget(self.lbl_mode, 1, 1)
        info_grid.addWidget(QLabel("Moving (Wheels):"), 2, 0)
        info_grid.addWidget(self.lbl_moving_wheels, 2, 1)
        info_grid.addWidget(QLabel("Moving (Spatial):"), 3, 0)
        info_grid.addWidget(self.lbl_moving_spatial, 3, 1)
        info_grid.addWidget(QLabel("Roll:"), 4, 0)
        info_grid.addWidget(self.lbl_roll, 4, 1)
        info_grid.addWidget(QLabel("Pitch:"), 5, 0)
        info_grid.addWidget(self.lbl_pitch, 5, 1)
        info_grid.addWidget(QLabel("Velocity:"), 6, 0)
        info_grid.addWidget(self.lbl_velocity, 6, 1)
        info_grid.addWidget(QLabel("Ang. Vel:"), 7, 0)
        info_grid.addWidget(self.lbl_angular_velocity, 7, 1)
        info_grid.addWidget(QLabel("Battery:"), 8, 0)
        info_grid.addWidget(self.lbl_battery, 8, 1)
        info_grid.addWidget(QLabel("Temp:"), 9, 0)
        info_grid.addWidget(self.lbl_temperature, 9, 1)
        info_grid.addWidget(QLabel("<b>Error Code:</b>"), 10, 0)
        info_grid.addWidget(self.lbl_error_code, 10, 1)
        
        self.lbl_topic_freqs[self.protocol.TOPIC_PUB_MACHINE_INFO] = QLabel("0.0 Hz")
        info_grid.addWidget(QLabel("Freq:"), 11, 0)
        info_grid.addWidget(self.lbl_topic_freqs[self.protocol.TOPIC_PUB_MACHINE_INFO], 11, 1)
        
        dash_layout.addWidget(info_group)

        # Encoders
        enc_group = QGroupBox("Encoders")
        enc_grid = QGridLayout(enc_group)
        self.lbl_encs = [QLabel("-") for _ in range(4)]
        labels = ["FL:", "FR:", "BL:", "BR:"]
        for i in range(4):
            enc_grid.addWidget(QLabel(labels[i]), i // 2, (i % 2) * 2)
            enc_grid.addWidget(self.lbl_encs[i], i // 2, (i % 2) * 2 + 1)
        
        self.lbl_topic_freqs[self.protocol.TOPIC_PUB_ENCODER] = QLabel("0.0 Hz")
        enc_grid.addWidget(QLabel("<b>Freq:</b>"), 2, 0)
        enc_grid.addWidget(self.lbl_topic_freqs[self.protocol.TOPIC_PUB_ENCODER], 2, 1)
        
        dash_layout.addWidget(enc_group)
        
        layout.addLayout(dash_layout)

        # IMU Raw values
        imu_group = QGroupBox("IMU Raw Data")
        imu_grid = QGridLayout(imu_group)
        self.lbl_acc = [QLabel("-") for _ in range(3)]
        self.lbl_gyro = [QLabel("-") for _ in range(3)]
        self.lbl_mag = [QLabel("-") for _ in range(3)]
        
        imu_grid.addWidget(QLabel("Accelerometer (X,Y,Z):"), 0, 0)
        for i in range(3): imu_grid.addWidget(self.lbl_acc[i], 0, i+1)
        
        imu_grid.addWidget(QLabel("Gyroscope (X,Y,Z):"), 1, 0)
        for i in range(3): imu_grid.addWidget(self.lbl_gyro[i], 1, i+1)
        
        imu_grid.addWidget(QLabel("Magnetometer (X,Y,Z):"), 2, 0)
        for i in range(3): imu_grid.addWidget(self.lbl_mag[i], 2, i+1)
        
        self.lbl_topic_freqs[self.protocol.TOPIC_PUB_IMU] = QLabel("0.0 Hz")
        imu_grid.addWidget(QLabel("<b>Freq:</b>"), 3, 0)
        imu_grid.addWidget(self.lbl_topic_freqs[self.protocol.TOPIC_PUB_IMU], 3, 1)
        
        layout.addWidget(imu_group)

        # Control Row
        # Control Row
        self.ctrl_group = QGroupBox("Control")
        ctrl_main_layout = QHBoxLayout(self.ctrl_group)
        
        # Velocity Control
        vel_layout = QGridLayout()
        self.spin_lx = QDoubleSpinBox(); self.spin_lx.setRange(-1.0, 1.0); self.spin_lx.setSingleStep(0.1)
        self.spin_az = QDoubleSpinBox(); self.spin_az.setRange(-2.0, 2.0); self.spin_az.setSingleStep(0.1)
        vel_layout.addWidget(QLabel("Linear X:"), 0, 0); vel_layout.addWidget(self.spin_lx, 0, 1)
        vel_layout.addWidget(QLabel("Angular Z:"), 1, 0); vel_layout.addWidget(self.spin_az, 1, 1)
        btn_send_vel = QPushButton("Send Velocity")
        btn_send_vel.clicked.connect(self.send_velocity)
        vel_layout.addWidget(btn_send_vel, 2, 0, 1, 2)
        ctrl_main_layout.addLayout(vel_layout)
        
        # Mode/State Toggle
        mode_layout = QVBoxLayout()
        btn_run = QPushButton("START / RUN")
        btn_run.setStyleSheet("background-color: green; color: white; font-weight: bold;")
        btn_run.clicked.connect(self.send_run_cmd)
        btn_toggle_mode = QPushButton("Toggle Mode (Manual/Auto)")
        btn_toggle_mode.clicked.connect(self.toggle_mode_cmd)
        self.btn_reset_stop = QPushButton("RESET STOP")
        self.btn_reset_stop.setStyleSheet("background-color: #f39c12; color: white; font-weight: bold;")
        self.btn_reset_stop.clicked.connect(self.send_reset_stop)
        btn_estop = QPushButton("EMERGENCY STOP")
        btn_estop.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        btn_estop.clicked.connect(self.send_estop)
        mode_layout.addWidget(btn_run)
        mode_layout.addWidget(btn_toggle_mode)
        mode_layout.addWidget(self.btn_reset_stop)
        mode_layout.addWidget(btn_estop)
        ctrl_main_layout.addLayout(mode_layout)
        layout.addWidget(self.ctrl_group)

        # Recording Control
        self.rec_group = QGroupBox("Data Recording")
        rec_layout = QHBoxLayout(self.rec_group) # horizontal for record button + info
        self.btn_toggle_rec = QPushButton("Start Recording")
        self.btn_toggle_rec.setStyleSheet("background-color: #3498db; color: white; font-weight: bold;")
        self.btn_toggle_rec.clicked.connect(self.toggle_recording)
        
        info_rec_layout = QVBoxLayout()
        self.lbl_rec_status = QLabel("Status: Idle")
        self.lbl_rec_count = QLabel("Samples: 0")
        info_rec_layout.addWidget(self.lbl_rec_status)
        info_rec_layout.addWidget(self.lbl_rec_count)
        
        rec_layout.addWidget(self.btn_toggle_rec)
        rec_layout.addLayout(info_rec_layout)
        
        layout.addWidget(self.rec_group)

        # Playback Control
        play_group = QGroupBox("Data Playback")
        play_layout = QGridLayout(play_group)
        
        self.btn_load_csv = QPushButton("Load CSV")
        self.btn_load_csv.clicked.connect(self.load_csv_playback)
        self.lbl_loaded_file = QLabel("No file loaded")
        self.lbl_loaded_file.setStyleSheet("color: gray; font-style: italic;")
        
        self.btn_play_pause = QPushButton("Play")
        self.btn_play_pause.clicked.connect(self.toggle_playback)
        self.btn_play_pause.setEnabled(False)
        
        self.chk_loop = QCheckBox("Loop Playback")
        
        self.play_progress = QProgressBar()
        self.play_progress.setValue(0)
        self.play_progress.setFormat("%p%")

        play_layout.addWidget(self.btn_load_csv, 0, 0)
        play_layout.addWidget(self.lbl_loaded_file, 0, 1)
        play_layout.addWidget(self.btn_play_pause, 1, 0)
        play_layout.addWidget(self.chk_loop, 1, 1)
        play_layout.addWidget(self.play_progress, 2, 0, 1, 2)
        
        layout.addWidget(play_group)
        layout.addStretch()

    def setup_graphs_tab(self):
        graph_tab = QWidget()
        self.tabs.addTab(graph_tab, "Graphs")
        layout = QGridLayout(graph_tab)

        # Acc Plot
        self.acc_plot = pg.PlotWidget(title="Accelerometer (m/s²)")
        self.acc_plot.addLegend()
        self.acc_curves = [self.acc_plot.plot(pen='r', name='X'),
                           self.acc_plot.plot(pen='g', name='Y'),
                           self.acc_plot.plot(pen='b', name='Z')]
        
        # Gyro Plot
        self.gyro_plot = pg.PlotWidget(title="Gyroscope (deg/s)")
        self.gyro_plot.addLegend()
        self.gyro_curves = [self.gyro_plot.plot(pen='r', name='X'),
                            self.gyro_plot.plot(pen='g', name='Y'),
                            self.gyro_plot.plot(pen='b', name='Z')]
        
        # Mag Plot
        self.mag_plot = pg.PlotWidget(title="Magnetometer (uT)")
        self.mag_plot.addLegend()
        self.mag_curves = [self.mag_plot.plot(pen='r', name='X'),
                           self.mag_plot.plot(pen='g', name='Y'),
                           self.mag_plot.plot(pen='b', name='Z')]
        
        # Encoder Plot
        self.enc_plot = pg.PlotWidget(title="Encoders (Ticks)")
        self.enc_plot.addLegend()
        self.enc_curves = [self.enc_plot.plot(pen='r', name='FL'),
                           self.enc_plot.plot(pen='g', name='FR'),
                           self.enc_plot.plot(pen='b', name='BL'),
                           self.enc_plot.plot(pen='y', name='BR')]

        layout.addWidget(self.acc_plot, 0, 0)
        layout.addWidget(self.gyro_plot, 0, 1)
        layout.addWidget(self.mag_plot, 1, 0)
        layout.addWidget(self.enc_plot, 1, 1)

    def update_ports(self):
        current_port = self.port_combo.currentText()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current_port in ports:
            self.port_combo.setCurrentText(current_port)

    def toggle_connect(self):
        if not self.running:
            port = self.port_combo.currentText()
            baud = int(self.baud_combo.currentText())
            try:
                self.serial_port = serial.Serial(port, baud, timeout=0.1)
                self.running = True
                self.connect_btn.setText("Disconnect")
                self.read_thread = threading.Thread(target=self.serial_reader, daemon=True)
                self.read_thread.start()
            except Exception as e:
                print(f"Error connecting: {e}")
        else:
            self.running = False
            if self.serial_port:
                self.serial_port.close()
            self.connect_btn.setText("Connect")

    def serial_reader(self):
        while self.running:
            try:
                if self.serial_port.in_waiting:
                    b = self.serial_port.read(1)[0]
                    res = self.protocol.unpack_byte(b)
                    if res:
                        topic_id, payload = res
                        self.data_received.emit(topic_id, payload)
                else:
                    time.sleep(0.001)
            except Exception as e:
                print(f"Read error: {e}")
                break

    @Slot(int, bytes)
    def handle_data(self, topic_id, payload):
        # Prevent update if playback is active
        if hasattr(self, 'is_playing') and self.is_playing:
            return

        # Update statistics
        if topic_id in self.topic_stats:
            self.topic_stats[topic_id]["count"] += 1
        else:
            # Handle unknown topics
            self.topic_stats[topic_id] = {"name": f"UNKNOWN_0X{topic_id:02X}", "count": 1, "freq": 0.0}
            # Note: The UI might not show this new topic until manual refresh or dynamic UI update implementation

        if not hasattr(self, 'current_data_cache'):
            self.current_data_cache = {
                "state": 0, "mode": 0, "wheels": 0, "spatial": 0, "error": 0,
                "roll": 0, "pitch": 0, "velocity": 0, "battery": 0, "temperature": 0,
                "acc_x": 0, "acc_y": 0, "acc_z": 0,
                "gyro_x": 0, "gyro_y": 0, "gyro_z": 0,
                "mag_x": 0, "mag_y": 0, "mag_z": 0,
                "enc_fl": 0, "enc_fr": 0, "enc_bl": 0, "enc_br": 0,
                "angular_velocity": 0
            }

        if topic_id == self.protocol.TOPIC_PUB_MACHINE_INFO:
            self.last_machine_info_time = time.time()
            if not self.board_connected:
                self.board_connected = True
                self.lbl_conn_status.setText("● Connected")
                self.lbl_conn_status.setStyleSheet("color: #2ecc71; font-weight: bold; margin-left: 20px;")

            info = parse_machine_info(payload)
            if info:
                self.current_data_cache.update({
                    "state": info["state"], "mode": info["mode"], 
                    "wheels": int(info["moving_wheels"]), 
                    "spatial": int(info["moving_spatial"]),
                    "error": info["error_code"],
                    "roll": info["roll"],
                    "pitch": info["pitch"],
                    "velocity": info["velocity"],
                    "battery": info["battery"],
                    "temperature": info["temperature"],
                    "angular_velocity": info.get("angular_velocity", 0.0)
                })
                states = ["IDLE", "MOVING", "TEMP_STOP", "E_STOP"]
                modes = ["MANUAL", "AUTONOMOUS"]
                self.lbl_state.setText(states[info["state"]] if info["state"] < len(states) else str(info["state"]))
                self.lbl_mode.setText(modes[info["mode"]] if info["mode"] < len(modes) else str(info["mode"]))
                self.lbl_moving_wheels.setText("YES" if info["moving_wheels"] else "NO")
                self.lbl_moving_spatial.setText("YES" if info["moving_spatial"] else "NO")
                self.lbl_roll.setText(f"{info['roll']:.1f}°")
                self.lbl_pitch.setText(f"{info['pitch']:.1f}°")
                self.lbl_velocity.setText(f"{info['velocity']:.2f} m/s")
                self.lbl_angular_velocity.setText(f"{info.get('angular_velocity', 0.0):.2f} °/s")
                self.lbl_battery.setText(f"{info['battery']:.2f} V")
                self.lbl_temperature.setText(f"{info['temperature']:.1f} °C")
                self.lbl_error_code.setText(f"0x{info['error_code']:08X}")
                
                # Style colors for easier visualization
                self.lbl_moving_wheels.setStyleSheet("color: lime; font-weight: bold;" if info["moving_wheels"] else "color: gray;")
                self.lbl_moving_spatial.setStyleSheet("color: orange; font-weight: bold;" if info["moving_spatial"] else "color: gray;")
                
                if info["error_code"] != 0:
                    self.lbl_error_code.setStyleSheet("color: red; font-weight: bold;")
                else:
                    self.lbl_error_code.setStyleSheet("color: gray;")

                # Update Error Table
                for row, (bit, name) in enumerate(self.sys_errors.items()):
                    is_active = (info["error_code"] & bit) != 0
                    code_item = self.error_table.item(row, 1)
                    status_item = self.error_table.item(row, 2)
                    
                    if is_active:
                        status_item.setText("FAULT")
                        status_item.setBackground(Qt.red)
                        status_item.setForeground(Qt.white)
                        code_item.setForeground(Qt.white)
                        code_item.setBackground(Qt.darkRed)
                    else:
                        status_item.setText("OK")
                        status_item.setBackground(Qt.transparent)
                        status_item.setForeground(Qt.green)
                        code_item.setForeground(Qt.white)
                        code_item.setBackground(Qt.transparent)

                if info["state"] == 3: # E_STOP
                    self.lbl_state.setStyleSheet("color: red; font-weight: bold;")
                else:
                    self.lbl_state.setStyleSheet("")

        elif topic_id == self.protocol.TOPIC_PUB_IMU:
            imu = parse_imu(payload)
            if imu:
                for i in range(3):
                    # Update Acc
                    self.imu_data["acc"][i] = np.roll(self.imu_data["acc"][i], -1)
                    self.imu_data["acc"][i][-1] = imu["acc"][i]
                    self.acc_curves[i].setData(self.imu_data["acc"][i])
                    self.lbl_acc[i].setText(f"{imu['acc'][i]:.2f}")
                    
                    # Update Gyro
                    self.imu_data["gyro"][i] = np.roll(self.imu_data["gyro"][i], -1)
                    self.imu_data["gyro"][i][-1] = imu["gyro"][i]
                    self.gyro_curves[i].setData(self.imu_data["gyro"][i])
                    self.lbl_gyro[i].setText(f"{imu['gyro'][i]:.2f}")

                    # Update Mag
                    self.imu_data["mag"][i] = np.roll(self.imu_data["mag"][i], -1)
                    self.imu_data["mag"][i][-1] = imu["mag"][i]
                    self.mag_curves[i].setData(self.imu_data["mag"][i])
                    self.lbl_mag[i].setText(f"{imu['mag'][i]:.2f}")
                
                self.current_data_cache.update({
                    "acc_x": imu["acc"][0], "acc_y": imu["acc"][1], "acc_z": imu["acc"][2],
                    "gyro_x": imu["gyro"][0], "gyro_y": imu["gyro"][1], "gyro_z": imu["gyro"][2],
                    "mag_x": imu["mag"][0], "mag_y": imu["mag"][1], "mag_z": imu["mag"][2]
                })
                # Update 3D view
                if hasattr(self, 'update_3d_view'):
                    self.update_3d_view()

        elif topic_id == self.protocol.TOPIC_PUB_ENCODER:
            encoders = parse_encoder(payload)
            if encoders:
                for i in range(min(len(encoders), 4)):
                    self.lbl_encs[i].setText(str(encoders[i]))
                    # Update Encoder Plots
                    self.encoder_data[i] = np.roll(self.encoder_data[i], -1)
                    self.encoder_data[i][-1] = encoders[i]
                    self.enc_curves[i].setData(self.encoder_data[i])
                
                self.current_data_cache.update({
                    "enc_fl": encoders[0], "enc_fr": encoders[1], 
                    "enc_bl": encoders[2], "enc_br": encoders[3]
                })

        # Record if active (record a sample every time we get a message, resulting in high-rate log)
        if self.is_recording:
            row = {"timestamp": time.time() - self.recording_start_time}
            row.update(self.current_data_cache)
            self.recorded_data.append(row)
            self.lbl_rec_count.setText(f"Samples: {len(self.recorded_data)}")

    def send_velocity(self):
        if not self.serial_port or not self.serial_port.is_open:
            return
        lx = self.spin_lx.value()
        az = self.spin_az.value()
        data = pack_cmd_vel(lx, 0.0, 0.0, az)
        frame = self.protocol.pack(self.protocol.TOPIC_SUB_CMD_VEL, data)
        self.serial_port.write(frame)

    def toggle_mode_cmd(self):
        if not self.serial_port or not self.serial_port.is_open:
            return
        new_mode = 1 if self.lbl_mode.text() == "MANUAL" else 0
        data = pack_enum(new_mode)
        frame = self.protocol.pack(self.protocol.TOPIC_SUB_OPERATION_MODE, data)
        self.serial_port.write(frame)

    def send_estop(self):
        if not self.serial_port or not self.serial_port.is_open:
            return
        data = pack_enum(3) # STATE_EMERGENCY_STOP
        frame = self.protocol.pack(self.protocol.TOPIC_SUB_ESTOP_CMD, data)
        self.serial_port.write(frame)

    def send_run_cmd(self):
        if (not self.serial_port or not self.serial_port.is_open):
            return
        # Values matching system_state_t in global.h: STATE_IDLE = 0
        data = pack_enum(0)
        frame = self.protocol.pack(self.protocol.TOPIC_SUB_OPERATION_RUN, data)
        self.serial_port.write(frame)

    def send_reset_stop(self):
        if (not self.serial_port or not self.serial_port.is_open):
            return
        data = pack_enum(2) # STATE_TEMPORAL_STOP
        frame = self.protocol.pack(self.protocol.TOPIC_SUB_RESET_STOP_CMD, data)
        self.serial_port.write(frame)

    def toggle_recording(self):
        if not self.is_recording:
            self.is_recording = True
            self.recorded_data = []
            self.recording_start_time = time.time()
            self.btn_toggle_rec.setText("Stop & Save Recording")
            self.btn_toggle_rec.setStyleSheet("background-color: #e67e22; color: white; font-weight: bold;")
            self.lbl_rec_status.setText("Status: RECORDING...")
            self.lbl_rec_status.setStyleSheet("color: red; font-weight: bold;")
        else:
            self.is_recording = False
            self.btn_toggle_rec.setText("Start Recording")
            self.btn_toggle_rec.setStyleSheet("background-color: #3498db; color: white; font-weight: bold;")
            self.lbl_rec_status.setText("Status: Idle")
            self.lbl_rec_status.setStyleSheet("")
            if len(self.recorded_data) > 0:
                self.save_recording_to_csv()

    def save_recording_to_csv(self):
        default_name = f"session_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        filename, _ = QFileDialog.getSaveFileName(self, "Save Data Recording", default_name, "CSV Files (*.csv)")
        if filename:
            try:
                with open(filename, 'w', newline='') as f:
                    if not self.recorded_data:
                        return
                    writer = csv.DictWriter(f, fieldnames=self.recorded_data[0].keys())
                    writer.writeheader()
                    writer.writerows(self.recorded_data)
                print(f"Recorded {len(self.recorded_data)} samples to {filename}")
            except Exception as e:
                print(f"Error saving CSV: {e}")
        self.recorded_data = []
        self.lbl_rec_count.setText("Samples: 0")

    def update_graphs(self):
        # Update Acc/Gyro/Mag from current_data_cache
        ax = self.current_data_cache.get("acc_x", 0)
        ay = self.current_data_cache.get("acc_y", 0)
        az = self.current_data_cache.get("acc_z", 0)
        gx = self.current_data_cache.get("gyro_x", 0)
        gy = self.current_data_cache.get("gyro_y", 0)
        gz = self.current_data_cache.get("gyro_z", 0)
        mx = self.current_data_cache.get("mag_x", 0)
        my = self.current_data_cache.get("mag_y", 0)
        mz = self.current_data_cache.get("mag_z", 0)
        
        # Helper to update arrays
        for arr, val in [(self.imu_data["acc"], [ax, ay, az]),
                         (self.imu_data["gyro"], [gx, gy, gz]),
                         (self.imu_data["mag"], [mx, my, mz])]:
            for i in range(3):
                arr[i] = np.roll(arr[i], -1)
                arr[i][-1] = val[i]
                
        # Helper to update curves
        for i in range(3):
            self.acc_curves[i].setData(self.imu_data["acc"][i])
            self.gyro_curves[i].setData(self.imu_data["gyro"][i])
            self.mag_curves[i].setData(self.imu_data["mag"][i])
            
            self.lbl_acc[i].setText(f"{self.current_data_cache.get('acc_x' if i==0 else 'acc_y' if i==1 else 'acc_z', 0):.2f}")
            self.lbl_gyro[i].setText(f"{self.current_data_cache.get('gyro_x' if i==0 else 'gyro_y' if i==1 else 'gyro_z', 0):.2f}")
            self.lbl_mag[i].setText(f"{self.current_data_cache.get('mag_x' if i==0 else 'mag_y' if i==1 else 'mag_z', 0):.2f}")

    def update_graphs(self):
        # Update Acc/Gyro/Mag from current_data_cache
        ax = self.current_data_cache.get("acc_x", 0)
        ay = self.current_data_cache.get("acc_y", 0)
        az = self.current_data_cache.get("acc_z", 0)
        gx = self.current_data_cache.get("gyro_x", 0)
        gy = self.current_data_cache.get("gyro_y", 0)
        gz = self.current_data_cache.get("gyro_z", 0)
        mx = self.current_data_cache.get("mag_x", 0)
        my = self.current_data_cache.get("mag_y", 0)
        mz = self.current_data_cache.get("mag_z", 0)
        
        # Helper to update arrays
        for arr, val in [(self.imu_data["acc"], [ax, ay, az]),
                         (self.imu_data["gyro"], [gx, gy, gz]),
                         (self.imu_data["mag"], [mx, my, mz])]:
            for i in range(3):
                arr[i] = np.roll(arr[i], -1)
                arr[i][-1] = val[i]
                
        # Helper to update curves
        for i in range(3):
            self.acc_curves[i].setData(self.imu_data["acc"][i])
            self.gyro_curves[i].setData(self.imu_data["gyro"][i])
            self.mag_curves[i].setData(self.imu_data["mag"][i])
            
            self.lbl_acc[i].setText(f"{self.current_data_cache.get('acc_x' if i==0 else 'acc_y' if i==1 else 'acc_z', 0):.2f}")
            self.lbl_gyro[i].setText(f"{self.current_data_cache.get('gyro_x' if i==0 else 'gyro_y' if i==1 else 'gyro_z', 0):.2f}")
            self.lbl_mag[i].setText(f"{self.current_data_cache.get('mag_x' if i==0 else 'mag_y' if i==1 else 'mag_z', 0):.2f}")

    def load_csv_playback(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Open CSV Recording", "", "CSV Files (*.csv)")
        if filename:
            try:
                with open(filename, 'r') as f:
                    reader = csv.DictReader(f)
                    self.playback_data = []
                    for row in reader:
                        # Convert values to numbers
                        new_row = {}
                        for k, v in row.items():
                            try:
                                new_row[k] = float(v)
                            except:
                                new_row[k] = v
                        self.playback_data.append(new_row)
                        
                if self.playback_data:
                    self.lbl_loaded_file.setText(os.path.basename(filename))
                    self.btn_play_pause.setEnabled(True)
                    self.playback_index = 0
                    print(f"Loaded {len(self.playback_data)} samples.")
            except Exception as e:
                print(f"Error loading CSV: {e}")

    def toggle_playback(self):
        if not self.is_playing:
            if not self.playback_data:
                return
            
            # Auto-disconnect if connected
            if self.running:
                print("Auto-disconnecting serial for playback...")
                self.toggle_connect()

            self.is_playing = True
            self.btn_play_pause.setText("Pause")
            self.playback_timer.start(self.playback_interval)
            
            # Disable other controls
            self.ctrl_group.setEnabled(False)
            self.rec_group.setEnabled(False)
            self.connect_btn.setEnabled(False)
            
            # If start from beginning
            if self.playback_index >= len(self.playback_data):
                self.playback_index = 0
        else:
            self.is_playing = False
            self.btn_play_pause.setText("Play")
            self.playback_timer.stop()
            
            # Enable controls
            self.ctrl_group.setEnabled(True)
            self.rec_group.setEnabled(True)
            self.connect_btn.setEnabled(True)

    def update_playback(self):
        if not self.playback_data:
            return
            
        if self.playback_index >= len(self.playback_data):
            if self.chk_loop.isChecked():
                self.playback_index = 0
            else:
                self.toggle_playback() # Stop
                return
        
        row = self.playback_data[self.playback_index]
        self.playback_index += 1
        
        # Update Progress Bar
        progress = int((self.playback_index / len(self.playback_data)) * 100)
        self.play_progress.setValue(progress)
        
        # Update cache
        self.current_data_cache.update(row)
        
        # Update UI Elements manually
        # Machine Info
        states = ["IDLE", "MOVING", "TEMP_STOP", "E_STOP"]
        modes = ["MANUAL", "AUTONOMOUS"]
        
        state = int(row.get("state", 0))
        mode = int(row.get("mode", 0))
        
        self.lbl_state.setText(states[state] if state < len(states) else str(state))
        self.lbl_mode.setText(modes[mode] if mode < len(modes) else str(mode))
        
        wheels = int(row.get("wheels", 0))
        spatial = int(row.get("spatial", 0))
        self.lbl_moving_wheels.setText("YES" if wheels else "NO")
        self.lbl_moving_spatial.setText("YES" if spatial else "NO")
        
        self.lbl_moving_wheels.setStyleSheet("color: lime; font-weight: bold;" if wheels else "color: gray;")
        self.lbl_moving_spatial.setStyleSheet("color: orange; font-weight: bold;" if spatial else "color: gray;")
        
        self.lbl_roll.setText(f"{row.get('roll', 0):.1f}°")
        self.lbl_pitch.setText(f"{row.get('pitch', 0):.1f}°")
        self.lbl_velocity.setText(f"{row.get('velocity', 0):.2f} m/s")
        self.lbl_battery.setText(f"{row.get('battery', 0):.2f} V")
        self.lbl_temperature.setText(f"{row.get('temperature', 0):.1f} °C")
        
        err = int(row.get("error", 0))
        self.lbl_error_code.setText(f"0x{err:08X}")
        self.lbl_error_code.setStyleSheet("color: red; font-weight: bold;" if err != 0 else "color: gray;")
        
        # Update Encoders Labels
        encs = [int(float(row.get(k, 0))) for k in ["enc_fl", "enc_fr", "enc_bl", "enc_br"]]
        for i, val in enumerate(encs):
            self.lbl_encs[i].setText(str(val))
            # Plots update
            self.encoder_data[i] = np.roll(self.encoder_data[i], -1)
            self.encoder_data[i][-1] = val
            self.enc_curves[i].setData(self.encoder_data[i])

        # Update Graphs logic (Acc/Gyro/Mag) - Reuse update_graphs?
        # update_graphs reads from current_data_cache, so just call it
        self.update_graphs()
        
        # Update 3D View
        if hasattr(self, 'update_3d_view'):
            self.update_3d_view()

    def refresh_topic_stats(self):
        now = time.time()
        dt = now - self.last_stats_time
        if dt <= 0: return

        for tid, info in self.topic_stats.items():
            freq = info["count"] / dt
            info["freq"] = freq
            info["count"] = 0
            if tid in self.lbl_topic_freqs:
                self.lbl_topic_freqs[tid].setText(f"{freq:.2f} Hz")
        
        # Connection Watchdog
        if self.board_connected and (now - self.last_machine_info_time > 2.5):
            self.board_connected = False
            self.lbl_conn_status.setText("○ Disconnected")
            self.lbl_conn_status.setStyleSheet("color: #e74c3c; font-weight: bold; margin-left: 20px;")
            self.reset_ui_placeholders()

        self.last_stats_time = now

    def reset_ui_placeholders(self):
        """Reset all data labels to '-' when board is disconnected"""
        self.lbl_state.setText("-")
        self.lbl_mode.setText("-")
        self.lbl_moving_wheels.setText("-")
        self.lbl_moving_spatial.setText("-")
        self.lbl_roll.setText("-")
        self.lbl_pitch.setText("-")
        self.lbl_velocity.setText("-")
        self.lbl_battery.setText("-")
        self.lbl_temperature.setText("-")
        self.lbl_error_code.setText("-")
        self.lbl_state.setStyleSheet("")
        self.lbl_error_code.setStyleSheet("")
        self.lbl_moving_wheels.setStyleSheet("")
        self.lbl_moving_spatial.setStyleSheet("")
        
        for lbl in self.lbl_encs: lbl.setText("-")
        for lbl in self.lbl_acc: lbl.setText("-")
        for lbl in self.lbl_gyro: lbl.setText("-")
        for lbl in self.lbl_mag: lbl.setText("-")
        
        for tid in self.lbl_topic_freqs:
            self.lbl_topic_freqs[tid].setText("0.0 Hz")
        
        # Reset error table
        for row in range(self.error_table.rowCount()):
            status_item = self.error_table.item(row, 2)
            code_item = self.error_table.item(row, 1)
            status_item.setText("-")
            status_item.setBackground(Qt.transparent)
            status_item.setForeground(Qt.gray)
            code_item.setBackground(Qt.transparent)
            code_item.setForeground(Qt.gray)

    def create_3d_view_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0) # Maximize space
        
        self.gl_view = gl.GLViewWidget()
        self.gl_view.opts['distance'] = 40
        self.gl_view.setWindowTitle('3D Orientation')
        self.gl_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Grid
        g = gl.GLGridItem()
        g.scale(2,2,1)
        self.gl_view.addItem(g)
        
        # Axis
        axis = gl.GLAxisItem()
        axis.setSize(10,10,10)
        self.gl_view.addItem(axis)
        
        # Board representation (Box)
        # Dimensions approx: 10cm x 6cm x 1cm -> 10, 6, 1 units
        self.board_box = gl.GLBoxItem(size=pg.Vector(10, 6, 1), color=(200, 200, 200, 255))
        self.board_box.translate(-5, -3, 0) # Center the box
        self.gl_view.addItem(self.board_box)
        
        layout.addWidget(self.gl_view, 1) # Stretch factor 1
        
        # Info label
        self.lbl_3d_info = QLabel("R: 0.0 P: 0.0 Y: 0.0")
        self.lbl_3d_info.setStyleSheet("padding: 5px; font-weight: bold;")
        layout.addWidget(self.lbl_3d_info, 0) # Stretch factory 0
        
        widget.setLayout(layout)
        return widget

    def update_3d_view(self):
        # Calculate RPY from IMU data
        ax = self.current_data_cache.get("acc_x", 0)
        ay = self.current_data_cache.get("acc_y", 0)
        az = self.current_data_cache.get("acc_z", 0)
        
        # Simple Roll/Pitch from Acc
        # Roll = atan2(ay, az)
        # Pitch = atan2(-ax, sqrt(ay*ay + az*az))
        roll = math.atan2(ay, az) * 57.2958
        pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 57.2958
        
        # Yaw from Mag (Tilt compensated)
        mx = self.current_data_cache.get("mag_x", 0)
        my = self.current_data_cache.get("mag_y", 0)
        mz = self.current_data_cache.get("mag_z", 0)
        
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        
        # Simple tilt compensation
        by = my * math.cos(roll_rad) - mz * math.sin(roll_rad)
        bx = mx * math.cos(pitch_rad) + my * math.sin(pitch_rad) * math.sin(roll_rad) + mz * math.sin(pitch_rad) * math.cos(roll_rad)
        
        yaw = math.atan2(-by, bx) * 57.2958
        
        # Apply Kalman Filter
        # -------------------
        gx = self.current_data_cache.get("gyro_x", 0)
        gy = self.current_data_cache.get("gyro_y", 0)
        gz = self.current_data_cache.get("gyro_z", 0)

        dt = 0.01 # Default 10ms (100Hz)
        now = time.time()
        if self.last_imu_time > 0:
            dt = now - self.last_imu_time
        
        # Sanity check for dt
        if dt > 0.5: dt = 0.01 
        self.last_imu_time = now

        # Compute smoothed angles
        # Note: Gyro axes mapping depends on sensor mounting. Assuming standard matching for now.
        # Gyro rate is in dps (degrees per second)
        roll_k = self.kalman_roll.compute(roll, gx, dt)
        pitch_k = self.kalman_pitch.compute(pitch, gy, dt)
        
        # For yaw, we use magnetometer offset as 'newAngle' and gyro z as rate
        # Warning: Yaw has no 'gravity' vector to correct drift perfectly without mag, 
        # but here we use the mag-derived yaw as the measurement.
        # Handle wrapping for yaw (0-360)? Kalman expects continuous linear. 
        # A simple approach is passing the difference or unwrapping, but for small steps it's fine 
        # unless it crosses +/-180.
        
        # Simple wrap fix before filter:
        if (yaw - self.kalman_yaw.angle) > 180: self.kalman_yaw.angle += 360
        if (yaw - self.kalman_yaw.angle) < -180: self.kalman_yaw.angle -= 360
        
        yaw_k = self.kalman_yaw.compute(yaw, gz, dt)

        self.lbl_3d_info.setText(f"R: {roll_k:.1f} P: {pitch_k:.1f} Y: {yaw_k:.1f} (Raw Y: {yaw:.1f})")
        
        # Update transform
        tr = pg.Transform3D()
        tr.rotate(yaw_k, 0, 0, 1)
        tr.rotate(pitch_k, 0, 1, 0)
        tr.rotate(roll_k, 1, 0, 0)
        
        # Reset and apply
        self.board_box.setTransform(tr)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SerialVisualizer()
    window.show()
    sys.exit(app.exec())
