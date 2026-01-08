import serial
import time
import random
import struct
from serial_ros import SerialRosProtocol, pack_cmd_vel

def run_simulator(port="COM10"):
    print(f"Starting simulator on {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
    except Exception as e:
        print(f"Error: Could not open port {port}. Make sure you have a virtual serial pair (e.g., COM10 <-> COM11).")
        return

    protocol = SerialRosProtocol()

    t = 0
    while True:
        # 1. Send Machine Info (1Hz)
        if int(t * 10) % 10 == 0:
            # state, mode, moving_w, moving_s (4 bytes)
            payload = struct.pack("<BBBB", random.randint(0, 3), random.randint(0, 1), random.randint(0, 1), random.randint(0, 1))
            frame = protocol.pack(protocol.TOPIC_PUB_MACHINE_INFO, payload)
            ser.write(frame)

        # 2. Send IMU (50Hz)
        acc = [random.uniform(-1, 1), random.uniform(-1, 1), 9.8 + random.uniform(-0.1, 0.1)]
        gyro = [random.uniform(-5, 5) for _ in range(3)]
        mag = [random.uniform(-50, 50) for _ in range(3)]
        payload = struct.pack("<9f", *acc, *gyro, *mag)
        frame = protocol.pack(protocol.TOPIC_PUB_IMU, payload)
        ser.write(frame)

        # 3. Send Encoders (20Hz)
        if int(t * 100) % 5 == 0:
            encs = [int(1000 * time.time() % 10000) + i*100 for i in range(4)]
            payload = struct.pack("<4i", *encs)
            frame = protocol.pack(protocol.TOPIC_PUB_ENCODER, payload)
            ser.write(frame)

        # Check for incoming commands
        if ser.in_waiting:
            b = ser.read(1)[0]
            res = protocol.unpack_byte(b)
            if res:
                topic_id, payload = res
                print(f"Received command: Topic={topic_id}, Len={len(payload)}")

        time.sleep(0.02)
        t += 0.02

if __name__ == "__main__":
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
    run_simulator(port)
