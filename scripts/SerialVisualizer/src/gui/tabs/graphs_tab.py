from PySide6.QtWidgets import QWidget, QGridLayout
import pyqtgraph as pg
import numpy as np

class GraphsTab(QWidget):
    def __init__(self, max_points=200):
        super().__init__()
        self.max_points = max_points
        self.imu_data = {
            "acc": [np.zeros(self.max_points) for _ in range(3)],
            "gyro": [np.zeros(self.max_points) for _ in range(3)],
            "mag": [np.zeros(self.max_points) for _ in range(3)]
        }
        self.encoder_data = [np.zeros(self.max_points) for _ in range(4)]
        self.setup_ui()

    def setup_ui(self):
        layout = QGridLayout(self)

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

    def update_imu(self, imu):
        for i in range(3):
            # Update Acc
            self.imu_data["acc"][i] = np.roll(self.imu_data["acc"][i], -1)
            self.imu_data["acc"][i][-1] = imu["acc"][i]
            self.acc_curves[i].setData(self.imu_data["acc"][i])
            
            # Update Gyro
            self.imu_data["gyro"][i] = np.roll(self.imu_data["gyro"][i], -1)
            self.imu_data["gyro"][i][-1] = imu["gyro"][i]
            self.gyro_curves[i].setData(self.imu_data["gyro"][i])

            # Update Mag
            self.imu_data["mag"][i] = np.roll(self.imu_data["mag"][i], -1)
            self.imu_data["mag"][i][-1] = imu["mag"][i]
            self.mag_curves[i].setData(self.imu_data["mag"][i])

    def update_encoders(self, encoders):
        for i in range(min(len(encoders), 4)):
            self.encoder_data[i] = np.roll(self.encoder_data[i], -1)
            self.encoder_data[i][-1] = encoders[i]
            self.enc_curves[i].setData(self.encoder_data[i])

    def update_from_cache(self, cache):
        # Update Acc/Gyro/Mag from cache
        ax = cache.get("acc_x", 0)
        ay = cache.get("acc_y", 0)
        az = cache.get("acc_z", 0)
        gx = cache.get("gyro_x", 0)
        gy = cache.get("gyro_y", 0)
        gz = cache.get("gyro_z", 0)
        mx = cache.get("mag_x", 0)
        my = cache.get("mag_y", 0)
        mz = cache.get("mag_z", 0)
        
        self.update_imu({"acc": [ax, ay, az], "gyro": [gx, gy, gz], "mag": [mx, my, mz]})

        # Encoders
        encs = [cache.get(k, 0) for k in ["enc_fl", "enc_fr", "enc_bl", "enc_br"]]
        self.update_encoders(encs)
