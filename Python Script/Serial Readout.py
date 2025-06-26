import serial
import struct

PORT = 'COM13'
BAUDRATE = 230400
TIMEOUT = 1

def read_rpm_frame(ser):
    while True:
        byte = ser.read(1)
        if not byte:
            return None
        if byte[0] == 0xAA:  # Look for start byte
            payload = ser.read(2)
            if len(payload) == 2:
                rpm = struct.unpack('<H', payload)[0]
                return rpm

with serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT) as ser:
    print(f"Opened {PORT} at {BAUDRATE} baud.")
    while True:
        rpm = read_rpm_frame(ser)
        if rpm is not None:
            print(f"RPM: {rpm}")
