from PySide6.QtWidgets import QWidget, QVBoxLayout, QGridLayout, QGroupBox, QLabel, QPushButton
import pyqtgraph as pg
import numpy as np
import time

class ConnectionTab(QWidget):
    def __init__(self, max_points=86400):
        super().__init__()
        self.max_points = max_points
        # Store frequencies for Machine Info (0x01), IMU (0x02), Encoder (0x03)
        self.freq_data = {
            0x01: np.zeros(self.max_points),
            0x02: np.zeros(self.max_points),
            0x03: np.zeros(self.max_points)
        }
        self.max_points_inst = 500 # Recent burst history
        self.inst_freq_data = {
            0x01: np.zeros(self.max_points_inst),
            0x02: np.zeros(self.max_points_inst),
            0x03: np.zeros(self.max_points_inst)
        }
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)

        # Main Plot for frequencies
        self.plot_freq = pg.PlotWidget(title="Topic Frequencies (Hz)")
        self.plot_freq.addLegend()
        self.plot_freq.setLabel('left', 'Frequency', units='Hz')
        self.plot_freq.setLabel('bottom', 'Time', units='s')
        self.plot_freq.showGrid(x=True, y=True)
        
        self.curves = {
            0x01: self.plot_freq.plot(pen='r', name='Machine Info'),
            0x02: self.plot_freq.plot(pen='g', name='IMU'),
            0x03: self.plot_freq.plot(pen='b', name='Encoder')
        }
        
        # Instantaneous Frequency Plot (Burst)
        self.plot_inst = pg.PlotWidget(title="Instantaneous Frequency (Hz) - Burst View")
        self.plot_inst.addLegend()
        self.plot_inst.setLabel('left', 'Freq', units='Hz')
        self.plot_inst.setLabel('bottom', 'Samples')
        self.plot_inst.showGrid(x=True, y=True)
        self.plot_inst.setYRange(0, 150) # Assuming most topics are below 100Hz
        
        # Data already initialized in __init__
        self.last_frame_time = {
            0x01: 0,
            0x02: 0,
            0x03: 0
        }
        
        self.inst_curves = {
            0x01: self.plot_inst.plot(pen='r', name='Machine Info'),
            0x02: self.plot_inst.plot(pen='g', name='IMU'),
            0x03: self.plot_inst.plot(pen='b', name='Encoder')
        }
        
        layout.addWidget(self.plot_inst)

        # Statistics Group
        stats_group = QGroupBox("Transmission Statistics & Health")
        stats_layout = QGridLayout(stats_group)
        
        self.lbl_stats = {}
        topics = [
            (0x01, "Machine Info"),
            (0x02, "IMU"),
            (0x03, "Encoder")
        ]
        
        for i, (tid, name) in enumerate(topics):
            stats_layout.addWidget(QLabel(f"<b>{name}:</b>"), i, 0)
            self.lbl_stats[tid] = {
                "avg": QLabel("0.00 Hz"),
                "jit": QLabel("0 ms"),
                "status": QLabel("Offline")
            }
            stats_layout.addWidget(QLabel("Avg Freq:"), i, 1)
            stats_layout.addWidget(self.lbl_stats[tid]["avg"], i, 2)
            stats_layout.addWidget(QLabel("Avg Jitter:"), i, 3)
            stats_layout.addWidget(self.lbl_stats[tid]["jit"], i, 4)
            stats_layout.addWidget(QLabel("Status:"), i, 5)
            stats_layout.addWidget(self.lbl_stats[tid]["status"], i, 6)
            
        layout.addWidget(stats_group)
        
        # Clean Button
        self.btn_clear = QPushButton("Clean Data")
        self.btn_clear.clicked.connect(self.clear_data)
        self.btn_clear.setFixedHeight(40)
        self.btn_clear.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #d32f2f;
            }
        """)
        layout.addWidget(self.btn_clear)
        
        layout.addStretch()

    def update_frame(self, topic_id):
        """Called whenever a frame is received to update instantaneous stats."""
        # Only track topics we are interested in for frequency analysis
        if topic_id not in self.inst_freq_data:
            return
            
        now = time.time() * 1000 # ms
        if self.last_frame_time.get(topic_id, 0) > 0:
            dt = now - self.last_frame_time[topic_id]
            if dt > 0:
                inst_freq = 1000.0 / dt
                self.inst_freq_data[topic_id] = np.roll(self.inst_freq_data[topic_id], -1)
                self.inst_freq_data[topic_id][-1] = inst_freq
                if topic_id in self.inst_curves:
                    self.inst_curves[topic_id].setData(self.inst_freq_data[topic_id])
        self.last_frame_time[topic_id] = now

    def update_freqs(self, freqs):
        """
        freqs: dict {topic_id: freq_hz}
        """
        for tid in [0x01, 0x02, 0x03]:
            val = freqs.get(tid, 0.0)
            self.freq_data[tid] = np.roll(self.freq_data[tid], -1)
            self.freq_data[tid][-1] = val
            self.curves[tid].setData(self.freq_data[tid])
            
            # Update individual stats
            if val > 0:
                self.lbl_stats[tid]["avg"].setText(f"{val:.2f} Hz")
                
                # Calculate avg jitter/freq from the buffer
                i_data = self.inst_freq_data[tid][self.inst_freq_data[tid] > 0]
                if len(i_data) > 0:
                    avg_f = np.mean(i_data)
                    std_f = np.std(i_data)
                    self.lbl_stats[tid]["jit"].setText(f"{avg_f:.1f} ± {std_f:.1f} Hz")
                    
                    if std_f < 5: # Stable freq
                        self.lbl_stats[tid]["status"].setText("Stable")
                        self.lbl_stats[tid]["status"].setStyleSheet("color: green; font-weight: bold;")
                    elif std_f < 50:
                        self.lbl_stats[tid]["status"].setText("Intermittent")
                        self.lbl_stats[tid]["status"].setStyleSheet("color: orange; font-weight: bold;")
                    else:
                        self.lbl_stats[tid]["status"].setText("High Jumper")
                        self.lbl_stats[tid]["status"].setStyleSheet("color: red; font-weight: bold;")
            else:
                self.lbl_stats[tid]["avg"].setText("0.00 Hz")
                self.lbl_stats[tid]["status"].setText("Offline")
                self.lbl_stats[tid]["status"].setStyleSheet("color: gray;")

    def reset_placeholders(self):
        for tid in [0x01, 0x02, 0x03]:
            # self.freq_data[tid].fill(0)  # Removed to keep historical data
            # self.inst_freq_data[tid].fill(0) # Removed to keep historical data
            self.last_frame_time[tid] = 0
            # self.curves[tid].setData(self.freq_data[tid])
            # self.inst_curves[tid].setData(self.inst_freq_data[tid])
            self.lbl_stats[tid]["avg"].setText("0.00 Hz")
            self.lbl_stats[tid]["jit"].setText("0 Hz")
            self.lbl_stats[tid]["status"].setText("Offline")
            self.lbl_stats[tid]["status"].setStyleSheet("color: gray;")

    def clear_data(self):
        """Resets all frequency buffers and clears the plots."""
        for tid in [0x01, 0x02, 0x03]:
            self.freq_data[tid].fill(0)
            self.inst_freq_data[tid].fill(0)
            self.last_frame_time[tid] = 0
            self.curves[tid].setData(self.freq_data[tid])
            self.inst_curves[tid].setData(self.inst_freq_data[tid])
            self.lbl_stats[tid]["avg"].setText("0.00 Hz")
            self.lbl_stats[tid]["jit"].setText("0 Hz")
            self.lbl_stats[tid]["status"].setText("Offline")
            self.lbl_stats[tid]["status"].setStyleSheet("color: gray;")
