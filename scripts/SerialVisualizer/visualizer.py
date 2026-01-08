import sys
import serial
import serial.tools.list_ports
import struct
import threading
import time
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QPushButton, 
                             QGridLayout, QGroupBox, QDoubleSpinBox, QTabWidget)
from PySide6.QtCore import QTimer, Qt, Signal, Slot
import pyqtgraph as pg
import numpy as np

from serial_ros import (SerialRosProtocol, parse_machine_info, parse_imu, 
                        parse_encoder, pack_cmd_vel, pack_enum)

class SerialVisualizer(QMainWindow):
    data_received = Signal(int, bytes)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 Control Board Serial Visualizer")
        self.resize(1200, 900)

        self.protocol = SerialRosProtocol()
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

        self.setup_ui()
        
        self.data_received.connect(self.handle_data)
        
        # Timer to update port list
        self.port_timer = QTimer()
        self.port_timer.timeout.connect(self.update_ports)
        self.port_timer.start(2000)
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
        main_layout.addWidget(conn_group)

        # --- Tab Widget ---
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        self.setup_raw_data_tab()
        self.setup_graphs_tab()

    def setup_raw_data_tab(self):
        raw_tab = QWidget()
        self.tabs.addTab(raw_tab, "Raw Data")
        layout = QVBoxLayout(raw_tab)

        # Dashboard Area
        dash_layout = QHBoxLayout()
        
        # Machine Info
        info_group = QGroupBox("Machine Info")
        info_grid = QGridLayout(info_group)
        self.lbl_state = QLabel("IDLE")
        self.lbl_mode = QLabel("MANUAL")
        self.lbl_moving = QLabel("NO")
        info_grid.addWidget(QLabel("State:"), 0, 0)
        info_grid.addWidget(self.lbl_state, 0, 1)
        info_grid.addWidget(QLabel("Mode:"), 1, 0)
        info_grid.addWidget(self.lbl_mode, 1, 1)
        info_grid.addWidget(QLabel("Moving:"), 2, 0)
        info_grid.addWidget(self.lbl_moving, 2, 1)
        dash_layout.addWidget(info_group)

        # Encoders
        enc_group = QGroupBox("Encoders")
        enc_grid = QGridLayout(enc_group)
        self.lbl_encs = [QLabel("0") for _ in range(4)]
        labels = ["FL:", "FR:", "BL:", "BR:"]
        for i in range(4):
            enc_grid.addWidget(QLabel(labels[i]), i // 2, (i % 2) * 2)
            enc_grid.addWidget(self.lbl_encs[i], i // 2, (i % 2) * 2 + 1)
        dash_layout.addWidget(enc_group)
        
        layout.addLayout(dash_layout)

        # IMU Raw values
        imu_group = QGroupBox("IMU Raw Data")
        imu_grid = QGridLayout(imu_group)
        self.lbl_acc = [QLabel("0.00") for _ in range(3)]
        self.lbl_gyro = [QLabel("0.00") for _ in range(3)]
        self.lbl_mag = [QLabel("0.00") for _ in range(3)]
        
        imu_grid.addWidget(QLabel("Accelerometer (X,Y,Z):"), 0, 0)
        for i in range(3): imu_grid.addWidget(self.lbl_acc[i], 0, i+1)
        
        imu_grid.addWidget(QLabel("Gyroscope (X,Y,Z):"), 1, 0)
        for i in range(3): imu_grid.addWidget(self.lbl_gyro[i], 1, i+1)
        
        imu_grid.addWidget(QLabel("Magnetometer (X,Y,Z):"), 2, 0)
        for i in range(3): imu_grid.addWidget(self.lbl_mag[i], 2, i+1)
        
        layout.addWidget(imu_group)

        # Control Row
        ctrl_group = QGroupBox("Control")
        ctrl_main_layout = QHBoxLayout(ctrl_group)
        
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
        btn_toggle_mode = QPushButton("Toggle Mode (Manual/Auto)")
        btn_toggle_mode.clicked.connect(self.toggle_mode_cmd)
        btn_estop = QPushButton("EMERGENCY STOP")
        btn_estop.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        btn_estop.clicked.connect(self.send_estop)
        mode_layout.addWidget(btn_toggle_mode)
        mode_layout.addWidget(btn_estop)
        ctrl_main_layout.addLayout(mode_layout)

        layout.addWidget(ctrl_group)
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
        if topic_id == self.protocol.TOPIC_PUB_MACHINE_INFO:
            info = parse_machine_info(payload)
            if info:
                states = ["IDLE", "MOVING", "TEMP_STOP", "E_STOP"]
                modes = ["MANUAL", "AUTONOMOUS"]
                self.lbl_state.setText(states[info["state"]] if info["state"] < len(states) else str(info["state"]))
                self.lbl_mode.setText(modes[info["mode"]] if info["mode"] < len(modes) else str(info["mode"]))
                self.lbl_moving.setText("YES" if info["moving"] else "NO")
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

        elif topic_id == self.protocol.TOPIC_PUB_ENCODER:
            encoders = parse_encoder(payload)
            if encoders:
                for i in range(min(len(encoders), 4)):
                    self.lbl_encs[i].setText(str(encoders[i]))
                    # Update Encoder Plots
                    self.encoder_data[i] = np.roll(self.encoder_data[i], -1)
                    self.encoder_data[i][-1] = encoders[i]
                    self.enc_curves[i].setData(self.encoder_data[i])

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
        data = pack_enum(3)
        frame = self.protocol.pack(self.protocol.TOPIC_SUB_OPERATION_RUN, data)
        self.serial_port.write(frame)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SerialVisualizer()
    window.show()
    sys.exit(app.exec())
