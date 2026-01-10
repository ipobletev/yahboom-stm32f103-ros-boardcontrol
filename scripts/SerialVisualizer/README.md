# Yahboom STM32 Serial Visualizer

![Main Dashboard](../../docs/media/gui1.png)

This is a real-time visualizer based on Python and Qt for monitoring and controlling the Yahboom STM32F103 board via the `SerialROS` protocol.

## 🖼️ Gallery

````carousel
![Raw Data Tab](../../docs/media/gui1.png)
<!-- slide -->
![Graphs Tab](../../docs/media/gui2.png)
<!-- slide -->
![System Status Tab](../../docs/media/gui3.png)
<!-- slide -->
![3D Orientation Tab](../../docs/media/gui4.png)
<!-- slide -->
![PID Tuning Tab](../../docs/media/gui5.png)
<!-- slide -->
![Connection Frequency Tab](../../docs/media/gui6.png)
````

## 🚀 Quick Start

### Windows
Simply run the batch file:
```cmd
.\launch.bat
```

### Linux / macOS
Grant execution permissions and launch the script:
```bash
chmod +x launch.sh
./launch.sh
```

The launch scripts will automatically create a virtual environment (`.venv`), install the necessary dependencies, and open the application.

## 📊 Features

- **Status Dashboard**: Real-time visualization of the system status (IDLE, MOVING, E_STOP), operation mode, and movement detection.
- **Real-time Graphs**: Acceleration (m/s²) and angular velocity (deg/s) monitoring from the IMU.
- **Encoder Readings**: Visualization of the accumulated values for the 4 motors (FL, FR, BL, BR).
- **Remote Control**:
  - Sending velocity commands (`cmd_vel`).
  - Toggling between Manual and Autonomous modes.
  - Emergency Stop (E-Stop) button.

## 🛠️ Project Structure

- `src/main.py`: Main application with the Qt interface (PySide6).
- `src/communication/serial_manager.py`: Serial port management and thread handling.
- `src/core/protocol.py`: Implementation of the communication protocol (packing/unpacking/checksum).
- `requirements.txt`: Python dependencies.

## 🧪 Requirements

To run this visualizer, you need:
- Python 3.8+
- A serial connection to the STM32 board (or a pair of virtual serial ports for testing).

## 📦 Main Dependencies

- **PySide6**: Framework for the graphical interface.
- **pyqtgraph**: High-performance library for 2D graphics.
- **pyserial**: Communication over the serial port.
- **numpy**: Numerical data processing.
