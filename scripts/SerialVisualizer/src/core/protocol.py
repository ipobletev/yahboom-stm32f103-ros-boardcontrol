import struct

class SerialProtocol:
    HEADER1 = 0xAA
    HEADER2 = 0x55
    MAX_PAYLOAD = 128

    # Topic IDs
    TOPIC_PUB_MACHINE_INFO = 0x01
    TOPIC_PUB_IMU = 0x02
    TOPIC_PUB_ENCODER = 0x03
    TOPIC_SUB_CMD_VEL = 0x04
    TOPIC_SUB_OPERATION_MODE = 0x05
    TOPIC_SUB_OPERATION_RUN = 0x06
    TOPIC_SUB_RESET_STOP_CMD = 0x07
    TOPIC_SUB_ESTOP_CMD = 0x08
    TOPIC_PUB_PID_DEBUG = 0x09
    TOPIC_SUB_CONFIRM_CONN = 0x0A
    TOPIC_SUB_CONFIG = 0x0B

    # States for unpacking
    STATE_WAIT_H1 = 0
    STATE_WAIT_H2 = 1
    STATE_WAIT_TOPIC = 2
    STATE_WAIT_LEN = 3
    STATE_WAIT_PAYLOAD = 4
    STATE_WAIT_CHECKSUM = 5

    def __init__(self):
        self.state = self.STATE_WAIT_H1
        self.payload = bytearray()
        self.topic_id = 0
        self.length = 0
        self.checksum = 0

    def calculate_checksum(self, topic_id, length, payload):
        checksum = topic_id ^ length
        for b in payload:
            checksum ^= b
        return checksum & 0xFF

    def pack(self, topic_id, data):
        """
        Pack data into a frame. 'data' should be bytes.
        """
        length = len(data)
        if length > self.MAX_PAYLOAD:
            raise ValueError("Payload too long")
        
        checksum = self.calculate_checksum(topic_id, length, data)
        header = struct.pack("BBBB", self.HEADER1, self.HEADER2, topic_id, length)
        return header + data + struct.pack("B", checksum)

    def unpack_byte(self, b):
        """
        State machine to unpack a single byte.
        Returns (topic_id, payload) if a frame is complete, else None.
        """
        if self.state == self.STATE_WAIT_H1:
            if b == self.HEADER1:
                self.state = self.STATE_WAIT_H2
        
        elif self.state == self.STATE_WAIT_H2:
            if b == self.HEADER2:
                self.state = self.STATE_WAIT_TOPIC
            else:
                self.state = self.STATE_WAIT_H1
        
        elif self.state == self.STATE_WAIT_TOPIC:
            self.topic_id = b
            self.state = self.STATE_WAIT_LEN
        
        elif self.state == self.STATE_WAIT_LEN:
            if b <= self.MAX_PAYLOAD:
                self.length = b
                self.payload = bytearray()
                if self.length == 0:
                    self.state = self.STATE_WAIT_CHECKSUM
                else:
                    self.state = self.STATE_WAIT_PAYLOAD
            else:
                self.state = self.STATE_WAIT_H1
        
        elif self.state == self.STATE_WAIT_PAYLOAD:
            self.payload.append(b)
            if len(self.payload) >= self.length:
                self.state = self.STATE_WAIT_CHECKSUM
        
        elif self.state == self.STATE_WAIT_CHECKSUM:
            received_checksum = b
            calculated_checksum = self.calculate_checksum(self.topic_id, self.length, self.payload)
            self.state = self.STATE_WAIT_H1
            if received_checksum == calculated_checksum:
                return self.topic_id, bytes(self.payload)
        
        return None

# Data Structure Parsers
def parse_machine_info(data):
    try:
        # struct machine_info { uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, pad[3], uint32_t, float, float, float, float, float, float } -> 36 bytes
        if len(data) >= 36:
            state, mode, wheels, spatial, estop, error, roll, pitch, velocity, battery, temperature, angular_velocity = struct.unpack("<BBBBB 3x I ffffff", data[:36])
            return {
                "state": state, 
                "mode": mode, 
                "moving_wheels": bool(wheels),
                "moving_spatial": bool(spatial),
                "estop": bool(estop),
                "error_code": error,
                "roll": roll,
                "pitch": pitch,
                "velocity": velocity,
                "battery": battery,
                "temperature": temperature,
                "angular_velocity": angular_velocity
            }
        # Intermediate version (32 bytes)
        elif len(data) >= 32:
            state, mode, wheels, spatial, error, roll, pitch, velocity, battery, temperature, angular_velocity = struct.unpack("<BBBB I ffffff", data[:32])
            return {
                "state": state, 
                "mode": mode, 
                "moving_wheels": bool(wheels),
                "moving_spatial": bool(spatial),
                "estop": (state == 3), # Heuristic: STATE_STOP_EMERGENCY = 3
                "error_code": error,
                "roll": roll,
                "pitch": pitch,
                "velocity": velocity,
                "battery": battery,
                "temperature": temperature,
                "angular_velocity": angular_velocity
            }
    except:
        pass
    return None

def parse_imu(data):
    # float acc[3], gyro[3], mag[3] -> 9 floats
    try:
        if len(data) == 36:
            vals = struct.unpack("<9f", data)
            return {
                "acc": vals[0:3],
                "gyro": vals[3:6],
                "mag": vals[6:9]
            }
    except:
        pass
    return None

def parse_encoder(data):
    # int32_t enc[4]
    try:
        if len(data) == 16:
            vals = struct.unpack("<4i", data)
            return vals
    except:
        pass
    return None

def parse_pid_debug(data):
    # struct { float target[4], current[4], error[4], kp[4], ki[4], kd[4] } -> 24 floats = 96 bytes
    try:
        if len(data) == 96:
            vals = struct.unpack("<24f", data)
            return {
                "target": vals[0:4],
                "current": vals[4:8],
                "error": vals[8:12],
                "kp": vals[12:16],
                "ki": vals[16:20],
                "kd": vals[20:24]
            }
        # Fallback for old firmware (48 bytes)
        elif len(data) == 48:
            vals = struct.unpack("<12f", data)
            return {
                "target": vals[0:4],
                "current": vals[4:8],
                "error": vals[8:12],
                "kp": [0.0]*4, "ki": [0.0]*4, "kd": [0.0]*4
            }
    except:
        pass
    return None

def pack_cmd_vel(lx, ly, lz, az):
    return struct.pack("<4f", lx, ly, lz, az)

def pack_enum(val):
    return struct.pack("B", val)

def pack_config(item_id, values):
    """
    Pack configuration update.
    'values' should be a list of floats (e.g. [kp, ki, kd]) or a single float (diameter).
    """
    if hasattr(values, '__iter__'):
        return struct.pack("<B" + "f"*len(values), item_id, *values)
    else:
        return struct.pack("<Bf", item_id, values)
