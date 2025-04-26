
from flask import Flask, Response, stream_with_context, jsonify
import threading
import time
import serial
import serial.tools.list_ports
import numpy as np
import re

app = Flask(__name__)
console_lines = []
battery_data = "<b>Waiting for data...</b>"
voltage_grid = [[0.0 for _ in range(18)] for _ in range(8)]  # default 8x18 grid
temperature_grid = [[0.0 for _ in range(8)] for _ in range(8)]  # default 8x8 grid
lock = threading.Lock()

BAUDRATE = 115200
START_FRAME = b'\xFF\xA3'
STOP_FRAME = b'\xFF\xB3'

ansi_escape = re.compile(r'\x1B\[[0-?]*[ -/]*[@-~]')

def parse_frame(data):
    try:
        totalVoltage = (data[0] + data[1]*256) * 0.1
        highestCellVoltage = (data[2] + data[3]*256) * 0.0001
        lowestCellVoltage = (data[4] + data[5]*256) * 0.0001
        meanCellVoltage = (data[6] + data[7]*256) * 0.0001
        highestCellTemp = (data[8] + data[9]*256) * 0.01
        lowestCellTemp = (data[10] + data[11]*256) * 0.01
        meanCellTemp = (data[12] + data[13]*256) * 0.01
        status = data[14]
        error = data[15]
        current = (data[16] + data[17]*256 + data[18]*65536 + data[19]*16777216) * 0.001
        counter = (data[20] + data[21]*256 + data[22]*65536 + data[23]*16777216) * 0.001
        time_per_cycle = data[24] + data[25]*256
        adbms_temp = (data[26] + data[27]*256) * 0.01

        html = f""
        html += f"Total Voltage: {totalVoltage:.2f} V<br>"
        html += f"Highest Cell Voltage: {highestCellVoltage:.2f} V<br>"
        html += f"Lowest Cell Voltage: {lowestCellVoltage:.2f} V<br>"
        html += f"Mean Cell Voltage: {meanCellVoltage:.2f} V<br>"
        html += f"Highest Cell Temperature: {highestCellTemp:.1f} °C<br>"
        html += f"Lowest Cell Temperature: {lowestCellTemp:.1f} °C<br>"
        html += f"Mean Cell Temperature: {meanCellTemp:.1f} °C<br>"
        html += f"Highest ADBMS Temp: {adbms_temp:.1f} °C<br>"
        html += f"Actual Current: {current:.2f} A<br>"
        html += f"Current Counter: {counter:.3f} Ah<br>"
        html += f"Status Code: 0x{status:04X}<br>"
        html += f"Error Code: {error:04X}<br>"
        html += f"Cycle Time: {time_per_cycle} ms<br>"
        html += f"<small>Raw data: {data[:28].hex()}</small><br>"

        return html
    except Exception as e:
        return f"<b>Parse error:</b> {e}"

def uart_reader(port):
    global battery_data, console_lines, voltage_grid
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

            line_buffer = bytearray()
            for b in chunk:
                line_buffer.append(b)
                if b == 10:  # Newline indicates end of line
                    try:
                        line = line_buffer.decode(errors='ignore').strip()

                        # Allow only fully printable ASCII lines (32–126) or those with Zephyr tags
                        if line and (
                            all(32 <= ord(c) <= 126 for c in line) or
                            any(tag in line for tag in ["<inf>", "<err>", "<dbg>", "<wrn>"])
                        ):
                            console_lines.append(line)
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
                
                if len(frame) >= 28 + (8 * 18 + 8 * 3) * 2:
                    battery_data = parse_frame(frame)

                    voltage_offset = 28
                    temperature_offset = voltage_offset + 8 * 18 * 2

                    new_voltage_grid = []
                    new_temperature_grid = []

                    for row in range(8):
                        voltage_row = []
                        for col in range(18):
                            idx = voltage_offset + 2 * (row * 18 + col)
                            raw = frame[idx] + (frame[idx + 1] << 8)
                            voltage = round(raw / 10000, 3)
                            voltage_row.append(voltage)
                        new_voltage_grid.append(voltage_row)

                        temp_row = []
                        for col in range(8):
                            idx = temperature_offset + 2 * (row * 3 + col)
                            raw = frame[idx] + (frame[idx + 1] << 8)
                            temp = round(raw / 100, 1)
                            temp_row.append(temp)
                        new_temperature_grid.append(temp_row)

                    with lock:
                        voltage_grid[:] = new_voltage_grid
                        temperature_grid[:] = new_temperature_grid

    except Exception as e:
        print("[UART] Error:", e)

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
            "temperature_grid": temperature_grid
        })

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

            time.sleep(0.5)
    return Response(stream_with_context(event_stream()), mimetype="text/event-stream")

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
