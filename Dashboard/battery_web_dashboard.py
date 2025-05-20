from flask import Flask, jsonify
import threading
import serial
import time
import serial.tools.list_ports

app = Flask(__name__)

battery_data = "<b>Waiting for data...</b>"
voltage_grid = [[0.0 for _ in range(18)] for _ in range(8)]
temperature_grid = [[0.0 for _ in range(8)] for _ in range(8)]
lock = threading.Lock()

START_FRAME = b'\xFF\xA3'
STOP_FRAME = b'\xFF\xB3'
MIN_FRAME_LENGTH = 476
READ_CHUNK_SIZE = 512
BAUDRATE = 115200

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
        current = int.from_bytes(data[16:20], byteorder='little', signed=True) * 0.001
        voltage = int.from_bytes(data[20:24], byteorder='little') * 0.001
        counter = int.from_bytes(data[24:28], byteorder='little') * 0.001
        time_per_cycle = data[28] + data[29]*256
        adbms_temp = (data[30] + data[31]*256) * 0.01

        # Python has multi-line strings ;)
        html = f"Total Voltage: {totalVoltage:.2f} V<br>"
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
        html += f"<small>Raw data: {data[:32].hex()}</small><br>"

        return html
    except Exception as e:
        return f"<b>Parse error:</b> {e}"

def uart_reader(port="/dev/ttyACM0"):
    global battery_data, voltage_grid, temperature_grid
    ser = serial.Serial(port, BAUDRATE, timeout=0.5)
    buffer = bytearray()

    print("[UART] Reader started.")

    while True:
        buffer += ser.read(READ_CHUNK_SIZE)
        start = buffer.find(START_FRAME)
        end = buffer.find(STOP_FRAME, start + 250)

        if start != -1 and end != -1 and end > start:
            frame = buffer[start + 2:end]
            buffer = buffer[end + 2:]

            if len(frame) >= MIN_FRAME_LENGTH:
                with lock:
                    battery_data = parse_frame(frame)

                    volt_offset = 28
                    temp_offset = volt_offset + 8*18*2

                    for row in range(8):
                        for col in range(18):
                            idx = volt_offset + 2 * (row * 18 + col)
                            raw = frame[idx] + (frame[idx+1] << 8)
                            voltage_grid[row][col] = round(raw / 10000, 3)

                        for col in range(8):
                            idx = temp_offset + 2 * (row * 3 + col)
                            raw = frame[idx] + (frame[idx+1] << 8)
                            temperature_grid[row][col] = round(raw / 100, 1)

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
