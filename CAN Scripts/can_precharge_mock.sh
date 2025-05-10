#!/bin/bash
# ==============================================================================
# CAN Precharge Mock Script
#
# This script simulates a precharge by sending voltage values that rise gradually
# from 0V to a defined max value (default: 600V), with low current values.
#
# === Configuration ===
CAN_INTERFACE="can0"        # Name of your CAN interface
BITRATE="500000"            # CAN bitrate
INTERVAL_MS="100"           # Default interval is 100 milliseconds
MAX_VOLTAGE=600000          # Default max voltage in mV (600V)
VOLTAGE_STEP=5000           # Voltage increase per step (5V)
CURRENT_RANGE=500           # Low current range ±500 mA

# === Initialization ===
current_voltage=0

# === Check if CAN interface is up ===
echo "[INFO] Checking if $CAN_INTERFACE is up..."
if ! ip link show "$CAN_INTERFACE" | grep -q "state UP"; then
    echo "[WARN] $CAN_INTERFACE is down. Attempting to bring it up..."
    sudo modprobe peak_usb
    sudo ip link set "$CAN_INTERFACE" up type can bitrate $BITRATE
    if [ $? -ne 0 ]; then
        echo "[ERROR] Failed to bring up $CAN_INTERFACE. Exiting."
        exit 1
    fi
    echo "[INFO] $CAN_INTERFACE is now up with bitrate $BITRATE"
else
    echo "[INFO] $CAN_INTERFACE is already up."
fi

# === Function to send CAN messages ===
send_can_messages() {
    # Send Voltage (ID 0x522)
    voltage_hex=$(printf "%08X" $current_voltage)
    echo "[SEND] Sending CAN message (Voltage): 0x522 01 00 $voltage_hex"
    cansend $CAN_INTERFACE "522#0100${voltage_hex}"

    # Send Low Current (ID 0x521)
    current_value=$(shuf -i -${CURRENT_RANGE}-${CURRENT_RANGE} -n 1)
    current_hex=$(printf "%08X" $current_value)
    echo "[SEND] Sending CAN message (Current): 0x521 00 00 $current_hex"
    cansend $CAN_INTERFACE "521#0000${current_hex}"
}

# === Main Loop ===
echo "[INFO] Starting CAN Precharge Mock (600V Max, 100ms Interval)..."
while true; do
    send_can_messages

    # Increase voltage until it reaches the max value
    if (( current_voltage < MAX_VOLTAGE )); then
        (( current_voltage += VOLTAGE_STEP ))
        if (( current_voltage > MAX_VOLTAGE )); then
            current_voltage=$MAX_VOLTAGE
        fi
    fi

    sleep $(echo "$INTERVAL_MS / 1000" | bc -l)
done
