#!/usr/bin/env python3

import serial
import argparse
import sys
import re


def extract_and_print(port, baud, timeout):
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        print(f"Opened serial port {port} at {baud} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    buffer = bytearray()
    line_buf = ''
    timestamp_pattern = re.compile(r"^\[\d{2}:\d{2}:\d{2}\.\d{3},\d{3}\]")
    START_SEQ = bytes.fromhex('FFA3')
    STOP_SEQ = bytes.fromhex('FFB3')

    try:
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
                        sys.stdout.write(line_buf)
                        sys.stdout.flush()
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
                    # Remove processed bytes
                    buffer = buffer[stop_idx + len(STOP_SEQ):]
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        ser.close()


def main():
    parser = argparse.ArgumentParser(description="Read from serial port, filter logs by timestamp and extract UART frames (FFA3..FFB3)")
    parser.add_argument('--port', required=True, help="Serial port (e.g., /dev/ttyUSB0 or COM3)")
    parser.add_argument('--baud', type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument('--timeout', type=float, default=1.0, help="Read timeout in seconds")

    args = parser.parse_args()
    extract_and_print(args.port, args.baud, args.timeout)


if __name__ == "__main__":
    main()