from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
                              QGroupBox, QLabel, QDoubleSpinBox, QPushButton, 
                              QScrollArea)
from PySide6.QtCore import Qt, Signal, Slot
import pyqtgraph as pg
import numpy as np

class PIDTuningTab(QWidget):
    pid_update_requested = Signal(int, list) # motor_id, [kp, ki, kd]
    wheel_diam_update_requested = Signal(float)

    def __init__(self, max_points=200):
        super().__init__()
        self.max_points = max_points
        self.pid_data = {
            "target": [np.zeros(self.max_points) for _ in range(4)],
            "current": [np.zeros(self.max_points) for _ in range(4)],
            "error": [np.zeros(self.max_points) for _ in range(4)]
        }
        self.setup_ui()

    def setup_ui(self):
        main_layout = QVBoxLayout(self)
        
        # --- Control Section ---
        ctrl_layout = QHBoxLayout()
        
        # PID Controls for each motor
        self.pid_controls = []
        for i in range(4):
            group = QGroupBox(f"Motor {i+1} PID")
            grid = QGridLayout(group)
            
            kp_spin = QDoubleSpinBox(); kp_spin.setRange(0, 100); kp_spin.setSingleStep(0.1); kp_spin.setValue(1.5)
            ki_spin = QDoubleSpinBox(); ki_spin.setRange(0, 100); ki_spin.setSingleStep(0.1); ki_spin.setValue(0.5)
            kd_spin = QDoubleSpinBox(); kd_spin.setRange(0, 100); kd_spin.setSingleStep(0.01); kd_spin.setValue(0.0)
            
            grid.addWidget(QLabel("Kp:"), 0, 0); grid.addWidget(kp_spin, 0, 1)
            grid.addWidget(QLabel("Ki:"), 1, 0); grid.addWidget(ki_spin, 1, 1)
            grid.addWidget(QLabel("Kd:"), 2, 0); grid.addWidget(kd_spin, 2, 1)
            
            self.lbl_kp_actual = QLabel("Actual: -"); grid.addWidget(self.lbl_kp_actual, 0, 2)
            self.lbl_ki_actual = QLabel("Actual: -"); grid.addWidget(self.lbl_ki_actual, 1, 2)
            self.lbl_kd_actual = QLabel("Actual: -"); grid.addWidget(self.lbl_kd_actual, 2, 2)

            btn = QPushButton("Update PID")
            btn.clicked.connect(lambda checked=False, idx=i: self.emit_pid_update(idx))
            grid.addWidget(btn, 3, 0, 1, 3)
            
            self.pid_controls.append({
                "kp": kp_spin, "ki": ki_spin, "kd": kd_spin,
                "kp_act": self.lbl_kp_actual, "ki_act": self.lbl_ki_actual, "kd_act": self.lbl_kd_actual
            })
            ctrl_layout.addWidget(group)
            
        main_layout.addLayout(ctrl_layout)
        
        # Wheel Diameter
        wheel_group = QGroupBox("Physical Params")
        wheel_layout = QHBoxLayout(wheel_group)
        self.wheel_spin = QDoubleSpinBox(); self.wheel_spin.setRange(0.01, 1.0); self.wheel_spin.setSingleStep(0.001); self.wheel_spin.setDecimals(4); self.wheel_spin.setValue(0.080)
        wheel_layout.addWidget(QLabel("Wheel Diameter (m):"))
        wheel_layout.addWidget(self.wheel_spin)
        btn_wheel = QPushButton("Update Wheel")
        btn_wheel.clicked.connect(lambda: self.wheel_diam_update_requested.emit(self.wheel_spin.value()))
        wheel_layout.addWidget(btn_wheel)
        main_layout.addWidget(wheel_group)

        # --- Graphs Section ---
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        graph_widget = QWidget()
        graph_layout = QGridLayout(graph_widget)
        
        self.speed_plots = []
        self.speed_curves = []
        self.error_plots = []
        self.error_curves = []
        
        titles = ["FL (M1)", "FR (M2)", "BL (M3)", "BR (M4)"]
        for i in range(4):
            # Speed Plot (Target vs Current)
            p_speed = pg.PlotWidget(title=f"{titles[i]} Speed (Target vs Current)")
            p_speed.addLegend()
            c_target = p_speed.plot(pen='y', name='Target')
            c_current = p_speed.plot(pen='g', name='Current')
            self.speed_plots.append(p_speed)
            self.speed_curves.append((c_target, c_current))
            graph_layout.addWidget(p_speed, i, 0)
            
            # Error Plot
            p_error = pg.PlotWidget(title=f"{titles[i]} PID Error")
            c_error = p_error.plot(pen='r')
            self.error_plots.append(p_error)
            self.error_curves.append(c_error)
            graph_layout.addWidget(p_error, i, 1)
            
        scroll.setWidget(graph_widget)
        main_layout.addWidget(scroll)

    def emit_pid_update(self, idx):
        ctrls = self.pid_controls[idx]
        self.pid_update_requested.emit(idx, [ctrls["kp"].value(), ctrls["ki"].value(), ctrls["kd"].value()])

    def update_pid_debug(self, data):
        """
        data: {'target': [4], 'current': [4], 'error': [4]}
        """
        for i in range(4):
            # Update data arrays
            self.pid_data["target"][i] = np.roll(self.pid_data["target"][i], -1)
            self.pid_data["target"][i][-1] = data["target"][i]
            
            self.pid_data["current"][i] = np.roll(self.pid_data["current"][i], -1)
            self.pid_data["current"][i][-1] = data["current"][i]
            
            self.pid_data["error"][i] = np.roll(self.pid_data["error"][i], -1)
            self.pid_data["error"][i][-1] = data["error"][i]
            
            # Update curves
            self.speed_curves[i][0].setData(self.pid_data["target"][i])
            self.speed_curves[i][1].setData(self.pid_data["current"][i])
            self.error_curves[i].setData(self.pid_data["error"][i])

            # Update actual gain labels
            if "kp" in data:
                self.pid_controls[i]["kp_act"].setText(f"Actual: {data['kp'][i]:.2f}")
                self.pid_controls[i]["ki_act"].setText(f"Actual: {data['ki'][i]:.2f}")
                self.pid_controls[i]["kd_act"].setText(f"Actual: {data['kd'][i]:.2f}")

    def reset_placeholders(self):
        for i in range(4):
            self.pid_data["target"][i].fill(0)
            self.pid_data["current"][i].fill(0)
            self.pid_data["error"][i].fill(0)
            self.speed_curves[i][0].setData(self.pid_data["target"][i])
            self.speed_curves[i][1].setData(self.pid_data["current"][i])
            self.error_curves[i].setData(self.pid_data["error"][i])
