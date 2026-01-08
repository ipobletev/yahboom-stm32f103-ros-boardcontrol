from PySide6.QtWidgets import QWidget, QVBoxLayout, QGridLayout, QGroupBox, QLabel
import pyqtgraph as pg
import numpy as np
import time

class ConnectionTab(QWidget):
    def __init__(self, max_points=100):
        super().__init__()
        self.max_points = max_points
        # Store frequencies for Machine Info (0x01), IMU (0x02), Encoder (0x03)
        self.freq_data = {
            0x01: np.zeros(self.max_points),
            0x02: np.zeros(self.max_points),
            0x03: np.zeros(self.max_points)
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
        
        # Jitter Plot (Time between frames)
        self.plot_jitter = pg.PlotWidget(title="Inter-frame Latency (ms)")
        self.plot_jitter.addLegend()
        self.plot_jitter.setLabel('left', 'Delta Time', units='ms')
        self.plot_jitter.setLabel('bottom', 'Samples')
        self.plot_jitter.showGrid(x=True, y=True)
        self.plot_jitter.setYRange(0, 500) # Should be low for stable connection
        
        self.jitter_data = {
            0x01: np.zeros(self.max_points),
            0x02: np.zeros(self.max_points),
            0x03: np.zeros(self.max_points)
        }
        self.last_frame_time = {
            0x01: 0,
            0x02: 0,
            0x03: 0
        }
        
        self.jitter_curves = {
            0x01: self.plot_jitter.plot(pen='r', name='Machine Info'),
            0x02: self.plot_jitter.plot(pen='g', name='IMU'),
            0x03: self.plot_jitter.plot(pen='b', name='Encoder')
        }
        
        layout.addWidget(self.plot_jitter)

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
        layout.addStretch()

    def update_frame(self, topic_id):
        """Called whenever a frame is received to update jitter data."""
        now = time.time() * 1000 # ms
        if self.last_frame_time.get(topic_id, 0) > 0:
            dt = now - self.last_frame_time[topic_id]
            self.jitter_data[topic_id] = np.roll(self.jitter_data[topic_id], -1)
            self.jitter_data[topic_id][-1] = dt
            if topic_id in self.jitter_curves:
                self.jitter_curves[topic_id].setData(self.jitter_data[topic_id])
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
                
                # Calculate avg jitter from the buffer
                j_data = self.jitter_data[tid][self.jitter_data[tid] > 0]
                if len(j_data) > 0:
                    avg_j = np.mean(j_data)
                    std_j = np.std(j_data)
                    self.lbl_stats[tid]["jit"].setText(f"{avg_j:.1f} ± {std_j:.1f} ms")
                    
                    if std_j < 10: # Low jitter
                        self.lbl_stats[tid]["status"].setText("Stable")
                        self.lbl_stats[tid]["status"].setStyleSheet("color: green; font-weight: bold;")
                    elif std_j < 50:
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
            self.freq_data[tid].fill(0)
            self.jitter_data[tid].fill(0)
            self.last_frame_time[tid] = 0
            self.curves[tid].setData(self.freq_data[tid])
            self.jitter_curves[tid].setData(self.jitter_data[tid])
            self.lbl_stats[tid]["avg"].setText("0.00 Hz")
            self.lbl_stats[tid]["jit"].setText("0 ms")
            self.lbl_stats[tid]["status"].setText("Offline")
            self.lbl_stats[tid]["status"].setStyleSheet("color: gray;")
