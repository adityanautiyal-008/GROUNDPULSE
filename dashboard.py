"""
╔══════════════════════════════════════════════════════════════╗
║   GroundPulse — Live Dashboard                              ║
║   LoRa Base Station Visualization                           ║
╚══════════════════════════════════════════════════════════════╝

Reads serial data from ESP32 LoRa receiver and displays
live confidence tracking of human detection.
"""

import serial
import time
import matplotlib.pyplot as plt
from collections import deque

# ─────────────────────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────────────────────
SERIAL_PORT = "COM5"        # Change to your port (Linux: /dev/ttyUSB0)
BAUD_RATE   = 115200
MAX_POINTS  = 100           # Points on graph

# ─────────────────────────────────────────────────────────────
# SERIAL INIT
# ─────────────────────────────────────────────────────────────
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"[OK] Connected to {SERIAL_PORT}")
except:
    print("[ERROR] Could not open serial port")
    exit(1)

time.sleep(2)

# ─────────────────────────────────────────────────────────────
# DATA BUFFERS
# ─────────────────────────────────────────────────────────────
confidence_data = deque(maxlen=MAX_POINTS)
time_data = deque(maxlen=MAX_POINTS)
start_time = time.time()

# ─────────────────────────────────────────────────────────────
# MATPLOTLIB SETUP
# ─────────────────────────────────────────────────────────────
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], linewidth=2)

ax.set_title("GroundPulse — Human Detection Confidence")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Confidence (%)")
ax.set_ylim(0, 100)
ax.grid(True)

# ─────────────────────────────────────────────────────────────
# PARSER
# ─────────────────────────────────────────────────────────────
def parse_packet(line):
    """
    Expected format:
    GP,<score>,<freq>,<co2>,<accel>,<human>,<battery>,<packetID>
    """
    try:
        parts = line.strip().split(",")

        if parts[0] != "GP":
            return None

        return {
            "score":   float(parts[1]),
            "freq":    float(parts[2]),
            "co2":     int(parts[3]),
            "accel":   float(parts[4]),
            "human":   int(parts[5]),
            "battery": int(parts[6]),
            "packet":  int(parts[7])
        }
    except:
        return None

# ─────────────────────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────────────────────
print("GroundPulse Dashboard Running...\n")

while True:
    try:
        raw = ser.readline().decode(errors="ignore").strip()

        if not raw or not raw.startswith("GP"):
            continue

        data = parse_packet(raw)
        if not data:
            continue

        t = time.time() - start_time

        confidence_data.append(data["score"])
        time_data.append(t)

        # Terminal output
        print(
            f"[{data['packet']:04d}] "
            f"Score:{data['score']:3.0f}% | "
            f"Hz:{data['freq']:.2f} | "
            f"CO2:+{data['co2']}ppm | "
            f"Bat:{data['battery']}% | "
            f"{'HUMAN' if data['human'] else '----'}"
        )

        if data["human"]:
            print(">>> HUMAN DETECTED <<<")

        # Update plot
        line.set_data(time_data, confidence_data)
        ax.set_xlim(max(0, t - 30), t + 1)
        plt.pause(0.01)

    except KeyboardInterrupt:
        print("\nDashboard stopped.")
        break
