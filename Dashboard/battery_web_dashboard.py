# battery_web_dashboard.py

from flask import Flask, jsonify
import threading
import serial
import time
import serial.tools.list_ports
import argparse
import sys
import re
import numpy as np

app = Flask(__name__)

battery_data = "<b>Waiting for data...</b>"
log_buffer = []
frame_count = 0
blink = False
lock = threading.Lock()

# --- NEW: initialize the three grids as globals ---
voltage_grid = []
temperature_grid = []
balance_grid = []

START_FRAME = b'\xFF\xA3'
STOP_FRAME = b'\xFF\xB3'
MIN_FRAME_LENGTH = 100
READ_CHUNK_SIZE = 512
BAUDRATE = 115200
MAX_LOG_LINES = 100
CLIENTS = 2  # Number of clients in the battery system

coeff = [6.87565181e-40, -9.27411410e-35, 5.48516319e-30, -1.87118391e-25,
         4.07565006e-21, -5.93033515e-17, 5.86752736e-13, -3.95278974e-09,
         1.80075051e-05, -5.76649020e-02, 1.65728946e+02]

def calc_temp(volt):
    if volt > 23000:
        return 0
    elif volt < 2000:
        return 100
    return np.polyval(coeff, volt)

def parse_frame(data):
    global frame_count, blink
    try:
        frame_count += 1
        blink = not blink

        totalVoltage = (data[0] + data[1]*256) * 0.1
        highestCellVoltage = (data[2] + data[3]*256) * 0.0001
        lowestCellVoltage = (data[4] + data[5]*256) * 0.0001
        meanCellVoltage = (data[6] + data[7]*256) * 0.0001
        highestCellTemp = calc_temp(data[8] + data[9]*256)
        lowestCellTemp = calc_temp(data[10] + data[11]*256)
        meanCellTemp = calc_temp(data[12] + data[13]*256)
        status = data[14]
        error = data[15]
        current = int.from_bytes(data[16:20], byteorder='little', signed=True) * 0.001
        voltage = int.from_bytes(data[20:24], byteorder='little', signed=True) * 0.001
        counter = int.from_bytes(data[24:28], byteorder='little', signed=True) / 3600
        time_per_cycle = data[28] + data[29]*256
        adbms_temp = (data[30] + data[31]*256) * 0.01

        relays = f"AIR pos: {'1' if status & 32 else '0'}<br>"
        relays += f"AIR neg: {'1' if status & 8 else '0'}<br>"
        relays += f"Precharge: {'1' if status & 16 else '0'}<br>"

        dot = '⚫︎' if blink else ' '

        html = f"<b>Battery Monitor</b> {dot}<br>"
        html += f"Frame Count: {frame_count}<br>"
        html += f"Total Voltage: {totalVoltage:.2f} V<br>"
        html += f"Highest Cell Voltage: {highestCellVoltage:.2f} V<br>"
        html += f"Lowest Cell Voltage: {lowestCellVoltage:.2f} V<br>"
        html += f"Mean Cell Voltage: {meanCellVoltage:.2f} V<br>"
        html += f"Highest Cell Temperature: {highestCellTemp:.1f} °C<br>"
        html += f"Lowest Cell Temperature: {lowestCellTemp:.1f} °C<br>"
        html += f"Mean Cell Temperature: {meanCellTemp:.1f} °C<br>"
        html += f"Highest ADBMS Temp: {adbms_temp:.1f} °C<br>"
        html += f"Actual Current: {current:.2f} A<br>"
        html += f"Actual Voltage: {voltage:.2f} V<br>"
        html += f"Current Counter: {counter:.3f} Ah<br>"
        html += f"Status Code: 0x{status:02X}<br>"
        html += f"Error Code: 0x{error:02X}<br>"
        html += f"Cycle Time: {time_per_cycle} ms<br>"
        html += relays
        html += f"<small>Raw data: {data[:32].hex()}</small><br>"

        return html
    except Exception as e:
        return f"<b>Parse error:</b> {e}"

# --- NEW HELPER FUNCTIONS to build each grid --- 
def parse_voltage_grid(data):
    """
    After parsing CLIENTS*4 bytes of balancing info, the next CLIENTS*18*2 bytes
    correspond to per-cell voltages. Each 2-byte little-endian value is scaled
    by 0.0001 to get volts. Returns a flat list of length (CLIENTS * 18).
    """
    # first skip balance bytes:
    offset = 32 + CLIENTS * 4
    volt_buffer = []
    for i in range(CLIENTS * 18):
        lo = data[offset + i*2]
        hi = data[offset + i*2 + 1]
        raw_val = lo + (hi << 8)
        cell_voltage = raw_val * 0.0001
        volt_buffer.append(cell_voltage)
    return volt_buffer


def parse_temperature_grid(data):
    """
    After balance (CLIENTS*4 bytes) and voltage (CLIENTS*18*2 bytes),
    the next CLIENTS*8*2 bytes correspond to per-cell temperature ADC values.
    We read each 2-byte little-endian word, then apply calc_temp(raw_adc) to get °C.
    Returns a list of length (CLIENTS * 8).
    """
    # compute offset: 32 (header) + balance + voltage
    offset = 32 + CLIENTS*4 + CLIENTS*18*2
    temp_buffer = []
    for i in range(CLIENTS * 8):
        lo = data[offset + i*2]
        hi = data[offset + i*2 + 1]
        raw_adc = lo + (hi << 8)
        temp_c = calc_temp(raw_adc)
        temp_buffer.append(temp_c)
    return temp_buffer

def parse_balance_grid(data):
    """
    From byte offset 32 onward, every 4 bytes correspond to one client's balance
    word (little-endian). Returns a list of integers of length CLIENTS.
    """
    offset = 32
    balance_cells = []
    for i in range(CLIENTS):
        # read 4 bytes for client i, little-endian:
        b0 = data[offset + i*4]
        b1 = data[offset + i*4 + 1]
        b2 = data[offset + i*4 + 2]
        b3 = data[offset + i*4 + 3]
        word = (b0
                + (b1 << 8)
                + (b2 << 16)
                + (b3 << 24))
        balance_cells.append(word)
    return balance_cells

def uart_reader(port="/dev/ttyACM0"):
    global battery_data, voltage_grid, temperature_grid, balance_grid, log_buffer
    ser = serial.Serial(port, BAUDRATE, timeout=0.1)
    buffer = bytearray()
    line_buf = ''
    timestamp_pattern = re.compile(r"^\[\d{2}:\d{2}:\d{2}\.\d{3},\d{3}\]")
    START_SEQ = bytes.fromhex('FFA3')
    STOP_SEQ = bytes.fromhex('FFB3')
    print("[UART] Reader started.")

    while True:
        byte = ser.read(1)
        if not byte:
            continue

        # Append to buffer for frame extraction
        buffer += byte

        # Decode byte for log line buffering
        try:
            char = byte.decode('utf-8', errors='ignore')
        except:
            char = ''

        if char:
            line_buf += char
            if char == '\n':
                # Check if line starts with timestamp
                if timestamp_pattern.match(line_buf):
                    log_buffer.append(line_buf)
                    if len(log_buffer) > MAX_LOG_LINES:
                        log_buffer = log_buffer[-MAX_LOG_LINES:]
                # Reset line buffer regardless
                line_buf = ''

        # Check for start marker in buffer
        start_idx = buffer.find(START_SEQ)
        if start_idx != -1:
            stop_idx = buffer.find(STOP_SEQ, start_idx + len(START_SEQ))
            if stop_idx != -1:
                # Extract full frame including markers
                frame = buffer[start_idx: stop_idx + len(STOP_SEQ)]
                hex_str = frame.hex().upper()
                print(f"\n[UART FRAME] {hex_str}")
                if len(frame) >= MIN_FRAME_LENGTH:
                    with lock:
                        raw = bytearray(frame[2:-2])  # Remove start and stop markers
                        battery_data = parse_frame(raw)
                        
                        # --- NEW: fill each grid based on 'raw' ---
                        voltage_grid = parse_voltage_grid(raw)
                        temperature_grid = parse_temperature_grid(raw)
                        balance_grid = parse_balance_grid(raw)

                # Remove processed bytes                
                buffer = buffer[stop_idx + len(STOP_SEQ):]

@app.route("/")
def index():
    return open("web_dashboard_final.html").read()

@app.route("/battery")
def battery():
    with lock:
        return battery_data

@app.route("/battery-json")
def battery_json():
    with lock:
        return jsonify({
            "voltage_grid": voltage_grid,
            "temperature_grid": temperature_grid,
            "balance_grid": balance_grid
        })

@app.route("/logs")
def logs():
    with lock:
        return jsonify(log_buffer)

def select_serial_port():
    ports = list(serial.tools.list_ports.comports())
    for i, port in enumerate(ports):
        print(f"{i}: {port.device} - {port.description}")
    idx = int(input("Select shared UART port: "))
    return ports[idx].device

if __name__ == "__main__":
    port = select_serial_port()
    threading.Thread(target=uart_reader, args=(port,), daemon=True).start()
    app.run(debug=False, port=5000)
