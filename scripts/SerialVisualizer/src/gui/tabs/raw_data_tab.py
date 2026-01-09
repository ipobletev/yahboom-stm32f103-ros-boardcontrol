from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
                              QGroupBox, QLabel, QDoubleSpinBox, QPushButton, 
                              QProgressBar, QCheckBox, QFileDialog)
from PySide6.QtCore import Qt, Signal

class RawDataTab(QWidget):
    velocity_sent = Signal(float, float) # lx, az
    mode_manual_requested = Signal()
    mode_auto_requested = Signal()
    estop_requested = Signal()
    run_requested = Signal()
    reset_stop_requested = Signal()
    recording_toggled = Signal(bool)
    csv_load_requested = Signal()
    playback_toggled = Signal(bool)
    loop_toggled = Signal(bool)

    def __init__(self):
        super().__init__()
        self.lbl_topic_freqs = {}
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)

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
        
        self.lbl_freq_machine = QLabel("0.0 Hz")
        info_grid.addWidget(QLabel("Freq:"), 11, 0)
        info_grid.addWidget(self.lbl_freq_machine, 11, 1)
        
        dash_layout.addWidget(info_group)

        # Encoders
        enc_group = QGroupBox("Encoders")
        enc_grid = QGridLayout(enc_group)
        self.lbl_encs = [QLabel("-") for _ in range(4)]
        labels = ["FL:", "FR:", "BL:", "BR:"]
        for i in range(4):
            enc_grid.addWidget(QLabel(labels[i]), i // 2, (i % 2) * 2)
            enc_grid.addWidget(self.lbl_encs[i], i // 2, (i % 2) * 2 + 1)
        
        self.lbl_freq_encoder = QLabel("0.0 Hz")
        enc_grid.addWidget(QLabel("<b>Freq:</b>"), 2, 0)
        enc_grid.addWidget(self.lbl_freq_encoder, 2, 1)
        
        dash_layout.addWidget(enc_group)
        
        layout.addLayout(dash_layout)

        # Data Area (IMU & PID)
        data_layout = QHBoxLayout()

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
        
        self.lbl_freq_imu = QLabel("0.0 Hz")
        imu_grid.addWidget(QLabel("<b>Freq:</b>"), 3, 0)
        imu_grid.addWidget(self.lbl_freq_imu, 3, 1)
        
        data_layout.addWidget(imu_group)

        # PID Raw Data
        pid_group = QGroupBox("PID Raw Data")
        pid_grid = QGridLayout(pid_group)
        self.lbl_pid_target = [QLabel("-") for _ in range(4)]
        self.lbl_pid_current = [QLabel("-") for _ in range(4)]
        self.lbl_pid_error = [QLabel("-") for _ in range(4)]
        self.lbl_pid_kp = [QLabel("-") for _ in range(4)]
        self.lbl_pid_ki = [QLabel("-") for _ in range(4)]
        self.lbl_pid_kd = [QLabel("-") for _ in range(4)]
        
        wheels = ["FL", "FR", "BL", "BR"]
        pid_grid.addWidget(QLabel("<b>Wheel</b>"), 0, 0)
        pid_grid.addWidget(QLabel("<b>Target</b>"), 0, 1)
        pid_grid.addWidget(QLabel("<b>Cur</b>"), 0, 2)
        pid_grid.addWidget(QLabel("<b>Err</b>"), 0, 3)
        pid_grid.addWidget(QLabel("<b>Kp</b>"), 0, 4)
        pid_grid.addWidget(QLabel("<b>Ki</b>"), 0, 5)
        pid_grid.addWidget(QLabel("<b>Kd</b>"), 0, 6)
        
        for i in range(4):
            pid_grid.addWidget(QLabel(wheels[i]), i+1, 0)
            pid_grid.addWidget(self.lbl_pid_target[i], i+1, 1)
            pid_grid.addWidget(self.lbl_pid_current[i], i+1, 2)
            pid_grid.addWidget(self.lbl_pid_error[i], i+1, 3)
            pid_grid.addWidget(self.lbl_pid_kp[i], i+1, 4)
            pid_grid.addWidget(self.lbl_pid_ki[i], i+1, 5)
            pid_grid.addWidget(self.lbl_pid_kd[i], i+1, 6)
            
        data_layout.addWidget(pid_group)
        layout.addLayout(data_layout)

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
        btn_send_vel.clicked.connect(lambda: self.velocity_sent.emit(self.spin_lx.value(), self.spin_az.value()))
        vel_layout.addWidget(btn_send_vel, 2, 0, 1, 2)
        ctrl_main_layout.addLayout(vel_layout)
        
        # Mode/State Toggle
        mode_layout = QVBoxLayout()
        btn_run = QPushButton("START / RUN")
        btn_run.setStyleSheet("background-color: green; color: white; font-weight: bold;")
        btn_run.clicked.connect(self.run_requested.emit)
        btn_manual_mode = QPushButton("MANUAL MODE")
        btn_manual_mode.setStyleSheet("background-color: #2980b9; color: white; font-weight: bold;")
        btn_manual_mode.clicked.connect(self.mode_manual_requested.emit)
        btn_auto_mode = QPushButton("AUTO MODE")
        btn_auto_mode.setStyleSheet("background-color: #8e44ad; color: white; font-weight: bold;")
        btn_auto_mode.clicked.connect(self.mode_auto_requested.emit)
        self.btn_reset_stop = QPushButton("RESET STOP")
        self.btn_reset_stop.setStyleSheet("background-color: #f39c12; color: white; font-weight: bold;")
        self.btn_reset_stop.clicked.connect(self.reset_stop_requested.emit)
        btn_estop = QPushButton("EMERGENCY STOP")
        btn_estop.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        btn_estop.clicked.connect(self.estop_requested.emit)
        mode_layout.addWidget(btn_run)
        mode_layout.addWidget(btn_manual_mode)
        mode_layout.addWidget(btn_auto_mode)
        mode_layout.addWidget(self.btn_reset_stop)
        mode_layout.addWidget(btn_estop)
        ctrl_main_layout.addLayout(mode_layout)
        layout.addWidget(self.ctrl_group)

        # Recording Control
        self.rec_group = QGroupBox("Data Recording")
        rec_layout = QHBoxLayout(self.rec_group)
        self.btn_toggle_rec = QPushButton("Start Recording")
        self.btn_toggle_rec.setStyleSheet("background-color: #3498db; color: white; font-weight: bold;")
        self.btn_toggle_rec.setCheckable(True)
        self.btn_toggle_rec.toggled.connect(self.recording_toggled.emit)
        
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
        self.btn_load_csv.clicked.connect(self.csv_load_requested.emit)
        self.lbl_loaded_file = QLabel("No file loaded")
        self.lbl_loaded_file.setStyleSheet("color: gray; font-style: italic;")
        
        self.btn_play_pause = QPushButton("Play")
        self.btn_play_pause.setCheckable(True)
        self.btn_play_pause.toggled.connect(self.playback_toggled.emit)
        self.btn_play_pause.setEnabled(False)
        
        self.chk_loop = QCheckBox("Loop Playback")
        self.chk_loop.toggled.connect(self.loop_toggled.emit)
        
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

    def update_machine_info(self, info):
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
        
        self.lbl_moving_wheels.setStyleSheet("color: lime; font-weight: bold;" if info["moving_wheels"] else "color: gray;")
        self.lbl_moving_spatial.setStyleSheet("color: orange; font-weight: bold;" if info["moving_spatial"] else "color: gray;")
        self.lbl_error_code.setStyleSheet("color: red; font-weight: bold;" if info["error_code"] != 0 else "color: gray;")
        if info["state"] == 3: self.lbl_state.setStyleSheet("color: red; font-weight: bold;")
        else: self.lbl_state.setStyleSheet("")

    def update_imu_labels(self, imu):
        for i in range(3):
            self.lbl_acc[i].setText(f"{imu['acc'][i]:.2f}")
            self.lbl_gyro[i].setText(f"{imu['gyro'][i]:.2f}")
            self.lbl_mag[i].setText(f"{imu['mag'][i]:.2f}")

    def update_encoder_labels(self, encoders):
        for i in range(min(len(encoders), 4)):
            self.lbl_encs[i].setText(str(encoders[i]))

    def update_pid_labels(self, pid_debug):
        for i in range(4):
            self.lbl_pid_target[i].setText(f"{pid_debug['target'][i]:.2f}")
            self.lbl_pid_current[i].setText(f"{pid_debug['current'][i]:.2f}")
            self.lbl_pid_error[i].setText(f"{pid_debug['error'][i]:.2f}")
            if "kp" in pid_debug:
                self.lbl_pid_kp[i].setText(f"{pid_debug['kp'][i]:.2f}")
                self.lbl_pid_ki[i].setText(f"{pid_debug['ki'][i]:.2f}")
                self.lbl_pid_kd[i].setText(f"{pid_debug['kd'][i]:.2f}")

    def set_freqs(self, freqs):
        # freqs is a dict {topic_id: freq}
        # Mapping hardcoded for now or use protocol constants if possible
        if 0x01 in freqs: self.lbl_freq_machine.setText(f"{freqs[0x01]:.2f} Hz")
        if 0x02 in freqs: self.lbl_freq_imu.setText(f"{freqs[0x02]:.2f} Hz")
        if 0x03 in freqs: self.lbl_freq_encoder.setText(f"{freqs[0x03]:.2f} Hz")

    def reset_placeholders(self):
        for lbl in [self.lbl_state, self.lbl_mode, self.lbl_moving_wheels, self.lbl_moving_spatial,
                    self.lbl_roll, self.lbl_pitch, self.lbl_velocity, self.lbl_battery, 
                    self.lbl_temperature, self.lbl_error_code, self.lbl_angular_velocity]:
            lbl.setText("-")
            lbl.setStyleSheet("")
        for lbl in self.lbl_encs + self.lbl_acc + self.lbl_gyro + self.lbl_mag + \
                   self.lbl_pid_target + self.lbl_pid_current + self.lbl_pid_error + \
                   self.lbl_pid_kp + self.lbl_pid_ki + self.lbl_pid_kd:
            lbl.setText("-")
        self.lbl_freq_machine.setText("0.0 Hz")
        self.lbl_freq_imu.setText("0.0 Hz")
        self.lbl_freq_encoder.setText("0.0 Hz")
