#!/bin/bash

# ==============================================================================
# CAN Auto-Reply Script
#
# This script listens on a CAN interface (e.g., can0) for a specific CAN ID
# and optionally matching data payload. When a match is detected, it sends
# a predefined CAN message in response.
#
# === Requirements ===
# - Linux system with SocketCAN support
# - `can-utils` installed (for candump and cansend)
#     sudo apt install can-utils
#
# - The CAN interface (e.g., can0) must be available (connected via USB-CAN or similar)
# - User must have permission to bring up CAN interfaces (may require sudo)
#
# === Optional sudo config (to avoid password prompt) ===
# Use visudo to allow running `ip` without password:
#     yourusername ALL=(ALL) NOPASSWD: /sbin/ip
#
# === Usage ===
#     ./can_autoreply.sh
#
# ==============================================================================

# === Configuration ===
CAN_INTERFACE="can0"                          # Name of your CAN interface
BITRATE="500000"                              # CAN bitrate
TRIGGER_ID="123"                              # CAN ID to look for (in hex, no 0x prefix)
TRIGGER_DATA="01 02 03 04 05 06 07 08"        # Data pattern to match (optional)
REPLY_ID="456"                                # CAN ID for the response
REPLY_DATA="11 22 33 44"                      # Data payload for the response

# Format reply for cansend (e.g., 456#11223344)
REPLY_CAN_STRING="${REPLY_ID}#$(echo ${REPLY_DATA} | sed 's/ //g')"

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

# === Start Listening and Responding ===
echo "[INFO] Listening on $CAN_INTERFACE for CAN ID $TRIGGER_ID with data '$TRIGGER_DATA'"
echo "[INFO] Will reply with: $REPLY_CAN_STRING"

candump $CAN_INTERFACE | while read -r line; do
    # Show incoming CAN frame
    echo "[RECV] $line"

    # Check for matching ID
    if echo "$line" | grep -q -i " $TRIGGER_ID "; then
        echo "[MATCH] ID $TRIGGER_ID found in message"

        # Check for data match if specified
        if [ -z "$TRIGGER_DATA" ] || echo "$line" | grep -qi "$TRIGGER_DATA"; then
            echo "[MATCH] Data matched (or not specified)"
            echo "[SEND] Sending response: $REPLY_CAN_STRING"
            cansend $CAN_INTERFACE $REPLY_CAN_STRING
        else
            echo "[SKIP] Data does not match"
        fi
    fi
done

# === End of Script ===
