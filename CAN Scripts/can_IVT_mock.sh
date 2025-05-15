#!/bin/bash

# ==============================================================================
# CAN Send Interval Script with Random Valid Data
#
# This script sends two predefined CAN messages (IDs 0x521 and 0x522) at a user-
# defined adjustable interval. The messages contain random but valid current and
# voltage data, formatted according to the IVT-S datasheet specifications.
#
# === Requirements ===
# - Linux system with SocketCAN support
# - `can-utils` installed (for cansend)
#     sudo apt install can-utils
#
# - The CAN interface (e.g., can0) must be available (connected via USB-CAN or similar)
# - User must have permission to bring up CAN interfaces (may require sudo)
#
# === Usage ===
#     ./can_send_interval.sh [interval_milliseconds]
#     (e.g., ./can_send_interval.sh 500)
#
# === Change Interval during Runtime ===
#     Send the signal USR1 (kill -USR1 <PID>) and input new interval in ms
# ==============================================================================

# === Configuration ===
CAN_INTERFACE="can0"        # Name of your CAN interface
BITRATE="500000"            # CAN bitrate
INTERVAL_MS="${1:-1000}"    # Default interval is 1000 milliseconds if not specified

# Function to update interval dynamically
update_interval() {
    echo "[INFO] Enter new interval in milliseconds: "
    read new_interval
    INTERVAL_MS="$new_interval"
    echo "[INFO] Interval updated to $INTERVAL_MS milliseconds."
}

# Trap USR1 signal for interval change
trap 'update_interval' USR1

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

# === Generate Random CAN Messages ===
generate_random_current() {
    # Generate a random 4-byte signed current value (-2,147,483,648 to +2,147,483,647)
    local current_value=$((RANDOM * RANDOM % 2147483648))
    # Randomly make it negative
    if (( RANDOM % 2 == 0 )); then
        current_value=$(( -1 * current_value ))
    fi
    printf "%08X" $((current_value & 0xFFFFFFFF))
}

generate_random_voltage() {
    # Generate a random 4-byte signed voltage value (-2,147,483,648 to +2,147,483,647)
    local voltage_value=$((RANDOM * RANDOM % 2147483648))
    # Randomly make it negative
    if (( RANDOM % 2 == 0 )); then
        voltage_value=$(( -1 * voltage_value ))
    fi
    printf "%08X" $((voltage_value & 0xFFFFFFFF))
}


# === Sending CAN Messages at Interval ===
echo "[INFO] Sending CAN messages every $INTERVAL_MS milliseconds..."
while true; do
    # Generate random current message (ID 0x521)
    current_hex=$(generate_random_current)
    echo "[SEND] Sending CAN message (Current): 0x521 00 00 $current_hex"
    cansend $CAN_INTERFACE "521#0000${current_hex}"

    # Generate random voltage message (ID 0x522)
    voltage_hex=$(generate_random_voltage)
    echo "[SEND] Sending CAN message (Voltage): 0x522 01 00 $voltage_hex"
    cansend $CAN_INTERFACE "522#0100${voltage_hex}"

    sleep $(echo "$INTERVAL_MS / 1000" | bc -l)
done

# 
