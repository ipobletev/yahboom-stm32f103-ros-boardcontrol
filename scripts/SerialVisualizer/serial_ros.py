import struct

class SerialRosProtocol:
    HEADER1 = 0xAA
    HEADER2 = 0x55
    MAX_PAYLOAD = 64

    # Topic IDs
    TOPIC_PUB_MACHINE_INFO = 0x01
    TOPIC_PUB_IMU = 0x02
    TOPIC_PUB_ENCODER = 0x03
    TOPIC_SUB_CMD_VEL = 0x04
    TOPIC_SUB_OPERATION_MODE = 0x05
    TOPIC_SUB_OPERATION_RUN = 0x06
    TOPIC_SUB_RESET_STOP_CMD = 0x07
    TOPIC_SUB_ESTOP_CMD = 0x08

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
        # struct machine_info { uint8_t, uint8_t, uint8_t, uint8_t, uint32_t, float, float, float, float, float } -> 28 bytes
        if len(data) >= 28:
            state, mode, wheels, spatial, error, roll, pitch, velocity, battery, temperature = struct.unpack("<BBBB I fffff", data[:28])
            return {
                "state": state, 
                "mode": mode, 
                "moving_wheels": bool(wheels),
                "moving_spatial": bool(spatial),
                "error_code": error,
                "roll": roll,
                "pitch": pitch,
                "velocity": velocity,
                "battery": battery,
                "temperature": temperature
            }
        # Fallback for old firmware (24 bytes)
        elif len(data) >= 24:
            state, mode, wheels, spatial, error, roll, pitch, velocity, battery = struct.unpack("<BBBB I ffff", data[:24])
            return {
                "state": state, 
                "mode": mode, 
                "moving_wheels": bool(wheels),
                "moving_spatial": bool(spatial),
                "error_code": error,
                "roll": roll,
                "pitch": pitch,
                "velocity": velocity,
                "battery": battery,
                "temperature": 0.0 # Default value
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

def pack_cmd_vel(lx, ly, lz, az):
    return struct.pack("<4f", lx, ly, lz, az)

def pack_enum(val):
    return struct.pack("B", val)
