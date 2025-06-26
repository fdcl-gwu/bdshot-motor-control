import sys
import serial
import struct
import threading
import time
import math
from collections import deque

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# --- Config ---
PORT = 'COM13'
BAUDRATE = 230400
BUFFER_SIZE = 300

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

    def get_sigma_bounds(self):
        sigma = 3 * math.sqrt(self.P)
        return self.x - sigma, self.x + sigma

# --- Serial Reader Thread ---
class SerialReader(threading.Thread):
    def __init__(self, raw_buffer, filtered_buffer, lower_bound_buffer, upper_bound_buffer, lock):
        super().__init__(daemon=True)
        self.raw_buffer = raw_buffer
        self.filtered_buffer = filtered_buffer
        self.lower_bound_buffer = lower_bound_buffer
        self.upper_bound_buffer = upper_bound_buffer
        self.lock = lock
        self.kalman = KalmanFilter1D()
        self.running = True

    def run(self):
        try:
            with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
                while self.running:
                    b = ser.read(1)
                    if b and b[0] == 0xAA:
                        payload = ser.read(2)
                        if len(payload) == 2:
                            raw_rpm = struct.unpack('<H', payload)[0]
                            filtered_rpm = self.kalman.update(raw_rpm)
                            lower, upper = self.kalman.get_sigma_bounds()
                            with self.lock:
                                self.raw_buffer.append(raw_rpm)
                                self.filtered_buffer.append(filtered_rpm)
                                self.lower_bound_buffer.append(lower)
                                self.upper_bound_buffer.append(upper)
                    else:
                        time.sleep(0.001)
        except serial.SerialException as e:
            print(f"Serial error: {e}")

# --- GUI Widget ---
class RPMPlotter(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RPM Plot: Raw (Red), Filtered (Blue), 3σ Bounds (Gray)")

        self.raw_buffer = deque([0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        self.filtered_buffer = deque([0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        self.lower_bound_buffer = deque([0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        self.upper_bound_buffer = deque([0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        self.lock = threading.Lock()

        self.init_ui()

        self.serial_thread = SerialReader(
            self.raw_buffer,
            self.filtered_buffer,
            self.lower_bound_buffer,
            self.upper_bound_buffer,
            self.lock
        )
        self.serial_thread.start()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)

    def init_ui(self):
        layout = QVBoxLayout()
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        self.ax = self.figure.add_subplot(111)
        self.line_raw, = self.ax.plot([], [], 'r-', lw=1.5, label="Raw RPM")
        self.line_filtered, = self.ax.plot([], [], 'b-', lw=2, label="Filtered RPM")
        self.line_upper, = self.ax.plot([], [], 'k--', lw=1, label="3σ Upper Bound")
        self.line_lower, = self.ax.plot([], [], 'k--', lw=1, label="3σ Lower Bound")

        self.label_raw = self.ax.text(0.01, 0.95, "", transform=self.ax.transAxes, color='red')
        self.label_filtered = self.ax.text(0.01, 0.90, "", transform=self.ax.transAxes, color='blue')

        self.ax.set_xlim(0, BUFFER_SIZE)
        self.ax.set_ylim(0, 10000)
        self.ax.set_xlabel("Time (frames)")
        self.ax.set_ylabel("RPM")
        self.ax.legend(loc='upper right')
        self.ax.grid(True)

    def update_plot(self):
        with self.lock:
            raw = list(self.raw_buffer)
            filt = list(self.filtered_buffer)
            lower = list(self.lower_bound_buffer)
            upper = list(self.upper_bound_buffer)

        x = range(len(raw))
        self.line_raw.set_data(x, raw)
        self.line_filtered.set_data(x, filt)
        self.line_lower.set_data(x, lower)
        self.line_upper.set_data(x, upper)

        max_y = max(max(raw), max(filt), max(upper), 1000)
        self.ax.set_ylim(0, max_y + 500)

        if raw:
            self.label_raw.set_text(f"Raw: {raw[-1]:.0f} RPM")
        if filt:
            self.label_filtered.set_text(f"Filtered: {filt[-1]:.0f} RPM")

        self.canvas.draw()

    def closeEvent(self, event):
        self.serial_thread.running = False
        event.accept()

# --- Entry ---
if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = RPMPlotter()
    win.resize(1000, 600)
    win.show()
    sys.exit(app.exec_())
