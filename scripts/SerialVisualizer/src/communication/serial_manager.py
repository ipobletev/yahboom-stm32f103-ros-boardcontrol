import serial
import threading
import time
from PySide6.QtCore import QObject, Signal, Slot
from core.protocol import SerialProtocol

class SerialManager(QObject):
    data_received = Signal(int, bytes)
    error_occurred = Signal(str)

    def __init__(self):
        super().__init__()
        self.protocol = SerialProtocol()
        self.serial_port = None
        self.running = False
        self.read_thread = None

    def connect(self, port, baud):
        if self.running:
            self.disconnect()
        
        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
            self.running = True
            self.read_thread = threading.Thread(target=self.serial_reader, daemon=True)
            self.read_thread.start()
            return True
        except Exception as e:
            self.error_occurred.emit(str(e))
            return False

    def disconnect(self):
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None

    def serial_reader(self):
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    b = self.serial_port.read(1)[0]
                    res = self.protocol.unpack_byte(b)
                    if res and self.running:
                        topic_id, payload = res
                        try:
                            self.data_received.emit(topic_id, payload)
                        except RuntimeError: # Signal source has been deleted
                            break
                else:
                    time.sleep(0.001)
            except Exception as e:
                if self.running:
                    try:
                        self.error_occurred.emit(f"Read error: {e}")
                    except RuntimeError:
                        pass
                break

    def write(self, topic_id, data):
        if self.serial_port and self.serial_port.is_open:
            frame = self.protocol.pack(topic_id, data)
            self.serial_port.write(frame)
            return True
        return False

    def is_connected(self):
        return self.serial_port is not None and self.serial_port.is_open
