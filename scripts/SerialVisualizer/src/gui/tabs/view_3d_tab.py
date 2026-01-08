from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QSizePolicy
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import math
import time
from core.filters import KalmanAngle

class View3DTab(QWidget):
    def __init__(self):
        super().__init__()
        # Kalman Filters
        self.kalman_roll = KalmanAngle()
        self.kalman_pitch = KalmanAngle()
        self.kalman_yaw = KalmanAngle()
        self.last_imu_time = 0
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.gl_view = gl.GLViewWidget()
        self.gl_view.opts['distance'] = 40
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
        self.board_box = gl.GLBoxItem(size=pg.Vector(10, 6, 1), color=(200, 200, 200, 255))
        self.board_box.translate(-5, -3, 0)
        self.gl_view.addItem(self.board_box)
        
        layout.addWidget(self.gl_view, 1)
        
        # Info label
        self.lbl_3d_info = QLabel("R: 0.0 P: 0.0 Y: 0.0")
        self.lbl_3d_info.setStyleSheet("padding: 5px; font-weight: bold;")
        layout.addWidget(self.lbl_3d_info, 0)

    def update_orientation(self, data_cache):
        ax = data_cache.get("acc_x", 0)
        ay = data_cache.get("acc_y", 0)
        az = data_cache.get("acc_z", 0)
        
        # Simple Roll/Pitch from Acc
        roll = math.atan2(ay, az) * 57.2958
        pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 57.2958
        
        # Yaw from Mag (Tilt compensated)
        mx = data_cache.get("mag_x", 0)
        my = data_cache.get("mag_y", 0)
        mz = data_cache.get("mag_z", 0)
        
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        
        by = my * math.cos(roll_rad) - mz * math.sin(roll_rad)
        bx = mx * math.cos(pitch_rad) + my * math.sin(pitch_rad) * math.sin(roll_rad) + mz * math.sin(pitch_rad) * math.cos(roll_rad)
        
        yaw = math.atan2(-by, bx) * 57.2958
        
        # Kalman update
        gx = data_cache.get("gyro_x", 0)
        gy = data_cache.get("gyro_y", 0)
        gz = data_cache.get("gyro_z", 0)

        dt = 0.01
        now = time.time()
        if self.last_imu_time > 0:
            dt = now - self.last_imu_time
        if dt > 0.5: dt = 0.01 
        self.last_imu_time = now

        roll_k = self.kalman_roll.compute(roll, gx, dt)
        pitch_k = self.kalman_pitch.compute(pitch, gy, dt)
        
        # Wrap fix for yaw
        if (yaw - self.kalman_yaw.angle) > 180: self.kalman_yaw.angle += 360
        if (yaw - self.kalman_yaw.angle) < -180: self.kalman_yaw.angle -= 360
        yaw_k = self.kalman_yaw.compute(yaw, gz, dt)

        self.lbl_3d_info.setText(f"R: {roll_k:.1f} P: {pitch_k:.1f} Y: {yaw_k:.1f} (Raw Y: {yaw:.1f})")
        
        # Update transform
        tr = pg.Transform3D()
        tr.rotate(yaw_k, 0, 0, 1)
        tr.rotate(pitch_k, 0, 1, 0)
        tr.rotate(roll_k, 1, 0, 0)
        self.board_box.setTransform(tr)
