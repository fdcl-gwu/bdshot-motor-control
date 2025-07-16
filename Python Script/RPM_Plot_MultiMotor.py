import sys
import serial
import struct
import threading
import time
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLineEdit, QPushButton, QLabel
)
from PyQt5.QtCore import QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# --- Config ---
PORT = 'COM15'
BAUDRATE = 230400
BUFFER_SIZE = 300
NUM_MOTORS = 4
HEADERS = [0xA1, 0xA2, 0xA3, 0xA4]
CMD_HEADERS = [0xB1, 0xB2, 0xB3, 0xB4]

# --- Kalman Filter ---
class KalmanFilter1D:
    def __init__(self, Q=1e-2, R=50, initial_estimate=0):
        self.x = initial_estimate
        self.P = 1.0
        self.Q = Q
        self.R = R

    def update(self, measurement):
        self.P += self.Q
        K = self.P / (self.P + self.R)
        self.x += K * (measurement - self.x)
        self.P *= (1 - K)
        return self.x

# --- Serial Reader Thread ---
class SerialReader(threading.Thread):
    def __init__(self, ser, raw_buffers, filtered_buffers, lock):
        super().__init__(daemon=True)
        self.ser = ser
        self.raw_buffers = raw_buffers
        self.filtered_buffers = filtered_buffers
        self.lock = lock
        self.kalman_filters = [KalmanFilter1D() for _ in range(NUM_MOTORS)]
        self.running = True

    def run(self):
        try:
            while self.running:
                b = self.ser.read(1)
                if b and b[0] in HEADERS:
                    motor_index = HEADERS.index(b[0])
                    payload = self.ser.read(2)
                    if len(payload) == 2:
                        raw_rpm = struct.unpack('<H', payload)[0]
                        filtered_rpm = self.kalman_filters[motor_index].update(raw_rpm)
                        with self.lock:
                            self.raw_buffers[motor_index].append(raw_rpm)
                            self.filtered_buffers[motor_index].append(filtered_rpm)
                    else:
                        time.sleep(0.001)
        except serial.SerialException as e:
            print(f"Serial error: {e}")

# --- GUI Widget ---
class RPMPlotter(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RPM Plot: Raw (dotted), Filtered (solid)")

        self.raw_buffers = [deque([0]*BUFFER_SIZE, maxlen=BUFFER_SIZE) for _ in range(NUM_MOTORS)]
        self.filtered_buffers = [deque([0]*BUFFER_SIZE, maxlen=BUFFER_SIZE) for _ in range(NUM_MOTORS)]
        self.lock = threading.Lock()
        
        self.ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        self.serial_thread = SerialReader(
            self.ser,
            self.raw_buffers,
            self.filtered_buffers,
            self.lock
        )
        self.serial_thread.start()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)

        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        control_layout = QHBoxLayout()
        self.input_fields = []
        self.send_buttons = []

        for i in range(NUM_MOTORS):
            vbox = QVBoxLayout()
            label = QLabel(f"Motor {i+1}")
            input_field = QLineEdit()
            input_field.setPlaceholderText("Throttle")
            send_button = QPushButton("Send")
            send_button.clicked.connect(lambda _, idx=i: self.send_throttle(idx))
            vbox.addWidget(label)
            vbox.addWidget(input_field)
            vbox.addWidget(send_button)
            self.input_fields.append(input_field)
            self.send_buttons.append(send_button)
            control_layout.addLayout(vbox)

        layout.addLayout(control_layout)
        self.setLayout(layout)

        self.ax = self.figure.add_subplot(111)
        self.lines_raw = []
        self.lines_filtered = []
        colors = ['r', 'g', 'b', 'm']
        for i in range(NUM_MOTORS):
            line_raw, = self.ax.plot([], [], linestyle=':', color=colors[i], label=f"Motor {i+1} Raw")
            line_filt, = self.ax.plot([], [], linestyle='-', color=colors[i], label=f"Motor {i+1} Filtered")
            self.lines_raw.append(line_raw)
            self.lines_filtered.append(line_filt)

        self.ax.set_xlim(0, BUFFER_SIZE)
        self.ax.set_ylim(0, 10000)
        self.ax.set_xlabel("Time (frames)")
        self.ax.set_ylabel("RPM")
        self.ax.legend(loc='upper right')
        self.ax.grid(True)

    def send_throttle(self, motor_index):
        try:
            throttle_str = self.input_fields[motor_index].text()
            throttle = int(throttle_str)
            if not (0 <= throttle <= 2047):
                print(f"Invalid throttle {throttle} for motor {motor_index+1}")
                return
            packet = bytes([CMD_HEADERS[motor_index], throttle & 0xFF, (throttle >> 8) & 0x07])
            self.ser.write(packet)
            print(f"Sent to Motor {motor_index+1}: {throttle}")
        except ValueError:
            print("Invalid input")
        except serial.SerialException as e:
            print(f"Serial error: {e}")

    def update_plot(self):
        with self.lock:
            for i in range(NUM_MOTORS):
                raw = list(self.raw_buffers[i])
                filt = list(self.filtered_buffers[i])
                x = range(len(raw))
                self.lines_raw[i].set_data(x, raw)
                self.lines_filtered[i].set_data(x, filt)

        max_rpm = max(max(buf) for buf in self.filtered_buffers if buf)
        self.ax.set_ylim(0, max(1000, max_rpm + 500))
        self.canvas.draw()

    def closeEvent(self, event):
        self.serial_thread.running = False
        self.serial_thread.join()
        self.ser.close()
        event.accept()

# --- Entry ---
if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = RPMPlotter()
    win.resize(1000, 700)
    win.show()
    sys.exit(app.exec_())
