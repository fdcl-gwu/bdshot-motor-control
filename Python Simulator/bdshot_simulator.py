import matplotlib.pyplot as plt
import numpy as np

def bdshot_crc(value_12bit: int) -> int:
    """
    Compute the BDShot 4-bit checksum based on bitwise XOR and inversion logic.

    :param value_12bit: 12-bit integer (throttle + telemetry)
    :return: 4-bit CRC
    """
    assert 0 <= value_12bit < (1 << 12), "Inumpyut must be a 12-bit value"

    crc = ~(value_12bit ^ (value_12bit >> 4) ^ (value_12bit >> 8)) & 0x0F
    return crc

def extract_throttle(packet_12bit: int) -> int:
    """
    Extracts the throttle value (base-10) from a 12-bit DShot/BDShot packet.

    :param packet_12bit: A 12-bit integer (throttle + telemetry)
    :return: Integer throttle value (0 to 2047)
    """
    assert 0 <= packet_12bit < (1 << 12), "Input must be a 12-bit value"
    return packet_12bit >> 1


def make_bdshot_packet(throttle: int, telemetry: int) -> int:
    """
    Construct the 12-bit BDShot packet from throttle and telemetry flag.

    :param throttle: Throttle value (0–2047)
    :param telemetry: Telemetry flag (0 or 1)
    :return: 12-bit integer representing the packet (11 throttle bits + 1 telemetry bit)
    """
    if not (0 <= throttle <= 2047):
        raise ValueError("Throttle must be between 0 and 2047")
    if telemetry not in (0, 1):
        raise ValueError("Telemetry must be 0 or 1")

    return (throttle << 1) | telemetry


def format_nibbles(value):
    binary = format(value, '012b')  # Convert to 12-bit binary string
    return f"{binary[:4]} {binary[4:8]} {binary[8:]}"  # Add spaces for nibbles

def plot_bdshot_packet_waveform(packet_12bit: int):
    """
    Plot BDShot waveform using PWM-style digital signaling:
    - Each bit slot starts HIGH
    - Drops LOW for a short (0) or long (1) duration
    - Returns to HIGH for the rest of the bit slot
    """
    if not (0 <= packet_12bit < (1 << 12)):
        raise ValueError("Input must be a 12-bit value (0–4095)")
    crc = bdshot_crc(packet_12bit)
    packet_16 = (packet_12bit << 4) | crc

    bits = [(packet_16 >> i) & 1 for i in reversed(range(16))]

    # Timing settings (normalized units)
    short_pulse = 0.3  # LOW time for bit 0
    long_pulse = 0.7   # LOW time for bit 1

    # Build waveform
    t = [0.0, 0.5]
    v = [1, 1]  # start HIGH

    for i, bit in enumerate(bits):
        start = t[-1]
        pulse = long_pulse if bit == 1 else short_pulse
        end_low = start + pulse
        end_bit = start + 1.0

        # Drop to LOW
        t += [start, end_low]
        v += [0, 0]

        # Return to HIGH
        t += [end_low, end_bit]
        v += [1, 1]

    # Plot
    plt.figure(figsize=(14, 3))
    plt.step(t, v, where='post', linewidth=2, color='black')
    plt.ylim(-0.5, 1.5)
    plt.yticks([0, 1], ["LOW", "HIGH"])
    bit_positions = np.arange(0.5, len(bits), 1.0)
    plt.xticks(bit_positions, [f"{i}" for i in range(1,17)])
    plt.grid(True, axis='x', linestyle=':', color='gray')

    packet_16_formatted = format(packet_16, '016b')
    packet_16_formatted = f"{packet_16_formatted[:4]} {packet_16_formatted[4:8]} {packet_16_formatted[8:12]} {packet_16_formatted[12:]}"
    plt.title(f"BDShot PWM Waveform for Packet={packet_16_formatted}")
    plt.xlabel("Bit Position (MSB → LSB)")
    plt.tight_layout()
    plt.show()

def decode_gcr(value: int) -> int:
    """
    Decodes a 20 bit GCR encoded value that has been mapped to 21 bits.
    Start with a 21 bit value and returns a 20 bit GCR
    """
    gcr = (value ^ (value >> 1))
    return gcr

def gcr_decode_20_to_16(gcr: int) -> int:
    # GCR decoding table: 5-bit encoded to 4-bit data
    gcr_table = {
        '11001': '0000',
        '11011': '0001',
        '10010': '0010',
        '10011': '0011',
        '11101': '0100',
        '10101': '0101',
        '10110': '0110',
        '10111': '0111',
        '11010': '1000',
        '01001': '1001',
        '01010': '1010',
        '01011': '1011',
        '11110': '1100',
        '01101': '1101',
        '01110': '1110',
        '01111': '1111',
    }

    # Convert 20-bit integer to binary string, zero-padded
    gcr_bin = f"{gcr:20b}"

    decoded_bits = ""
    for i in range(0, 20, 5):
        codeword = gcr_bin[i:i+5]
        if codeword not in gcr_table:
            raise ValueError(f"Invalid GCR code: {codeword}")
        decoded_bits += gcr_table[codeword]

    # Convert the full 16-bit binary string to an integer
    return int(decoded_bits, 2)

def extract_erpm(telemetry_value: int) -> float:
    if telemetry_value < 0 or telemetry_value >= (1 << 16):
        raise ValueError("Telemetry value must be a 16-bit integer (0 to 65535)")

    # Extract fields
    crc = telemetry_value & 0xF
    base_period = (telemetry_value >> 4) & 0x1FF  # 9 bits
    exponent = (telemetry_value >> 13) & 0x7       # 3 bits

    # Compute period in microseconds
    period_us = base_period << exponent

    if period_us == 0:
        raise ValueError("Invalid period (0 us), cannot compute eRPM")

    # Compute eRPM
    erpm = 1_000_000 * 60 / (period_us * 7)

    return erpm

def parse_edt_frame(frame: int):
    if frame < 0 or frame > 0xFFFF:
        raise ValueError("Input must be a 16-bit integer (0 to 65535)")

    # Strip CRC (lowest 4 bits)
    data = frame >> 4

    # Extract exponent and base_period
    exponent = (data >> 9) & 0b111
    base_period = data & 0x1FF

    # Check for EDT frame
    is_edt = (exponent & 1) == 0 and (base_period & 0x100) == 0

    if is_edt:
        telemetry_type = (data >> 8) & 0xF
        telemetry_value = data & 0xFF

        # Optional: human-readable type names
        type_names = {
            0x02: "Temperature (°C)",
            0x04: "Voltage (V)",
            0x06: "Current (A)",
            0x08: "Debug 1",
            0x0A: "Debug 2",
            0x0C: "Debug 3",
            0x0E: "State/Event"
        }

        type_label = type_names.get(telemetry_type, f"Unknown (0x{telemetry_type:X})")

        # Optional: scaling
        if telemetry_type == 0x04:  # Voltage
            telemetry_value = telemetry_value * 0.25
        elif telemetry_type == 0x06:  # Current
            telemetry_value = telemetry_value * 1.0  # amps
        elif telemetry_type == 0x02:  # Temperature
            telemetry_value = telemetry_value  # °C

        return ("EDT", type_label, telemetry_value)

    else:
        # Compute period
        period_us = base_period << exponent
        if period_us == 0:
            return ("eRPM", 0)
        erpm = 1_000_000 * 60 / (period_us * 7)
        return ("eRPM", erpm)