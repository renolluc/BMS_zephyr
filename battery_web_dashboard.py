
from flask import Flask, Response, stream_with_context
from flask import jsonify
import threading
import time
import serial
import serial.tools.list_ports
import numpy as np
import re

app = Flask(__name__)
console_lines = []
battery_data = "<b>Waiting for data...</b>"
lock = threading.Lock()

BAUDRATE = 115200
START_FRAME = b'\xFF\xA3'
STOP_FRAME = b'\xFF\xB3'

ansi_escape = re.compile(r'\x1B\[[0-?]*[ -/]*[@-~]')

coeff = [6.87565181e-40,-9.27411410e-35,5.48516319e-30,-1.87118391e-25,4.07565006e-21,-5.93033515e-17,
         5.86752736e-13,-3.95278974e-09,1.80075051e-05,-5.76649020e-02,1.65728946e+02]

def parse_frame(data):
    global voltage_grid
    voltage_grid = [[0.0 for _ in range(18)] for _ in range(8)]
    try:
        totalVoltage = (data[0] + data[1]*256) * 0.1
        highestCellVoltage = (data[2] + data[3]*256) * 0.0001
        lowestCellVoltage = (data[4] + data[5]*256) * 0.0001
        meanCellVoltage = (data[6] + data[7]*256) * 0.0001
    except Exception as e:
        return f"<b>Parse error:</b> {e}"
    try:
        for i in range(8):
            for j in range(18):
                idx = 28 + (i * 18 + j) * 2
                if idx + 1 < len(data):
                    voltage = (data[idx] + data[idx+1] * 256) * 0.0001
                    voltage_grid[i][j] = round(voltage, 3)
        html = f"<b>Battery Monitor</b><br>"
        html += f"Total Voltage: {totalVoltage:.2f} V<br>"
        html += f"Highest Cell Voltage: {highestCellVoltage:.2f} V<br>"
        html += f"Lowest Cell Voltage: {lowestCellVoltage:.2f} V<br>"
        html += f"Mean Cell Voltage: {meanCellVoltage:.2f} V<br>"
        html += f"<small>Raw data: {data[:28].hex()}</small><br>"

        return html
    except Exception as e:
        return f"<b>Parse error:</b> {e}"

def uart_reader(port):
    global battery_data, console_lines
    try:
        ser = serial.Serial(port, BAUDRATE, timeout=0.5)
        buffer = bytearray()
        line_buffer = bytearray()
        print("[UART] Reader started.")

        while True:
            chunk = ser.read(128)
            if not chunk:
                continue
            buffer += chunk

            for b in chunk:
                line_buffer.append(b)
                if b == 10:
                    try:
                        line = line_buffer.decode(errors='ignore').strip()
                        if line:
                            clean = ansi_escape.sub('', line)
                            console_lines.append(clean)
                            if len(console_lines) > 200:
                                console_lines = console_lines[-200:]
                    except Exception as e:
                        print("[UART] Decode error:", e)
                    line_buffer.clear()

            while True:
                start = buffer.find(START_FRAME)
                if start == -1:
                    break
                end = buffer.find(STOP_FRAME, start)
                if end == -1:
                    break
                frame = buffer[start+2:end]
                buffer = buffer[end+2:]
                if len(frame) >= 28:
                    html = parse_frame(frame)
                    with lock:
                        battery_data = html
    except Exception as e:
        print("[UART] Error:", e)

@app.route("/")
def index():
    return open("web_dashboard_final.html").read()

@app.route("/battery")
def battery():
    with lock:
        return battery_data

@app.route("/console-log")
def stream_logs():
    print("[SSE] Client connected to /console-log")
    def event_stream():
        yield "retry: 1000\n"
        yield "data: [connected]\n\n"
        last_idx = 0
        while True:
            if len(console_lines) > last_idx:
                for line in console_lines[last_idx:]:
                    yield f"data: {line}\n\n"
                last_idx = len(console_lines)
            else:
                yield "data: \n\n"
            time.sleep(0.5)
    return Response(stream_with_context(event_stream()), mimetype="text/event-stream")


from flask import jsonify

@app.route("/battery-json")
def battery_json():
    with lock:
        return jsonify({
            "voltage_grid": voltage_grid
        })
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
