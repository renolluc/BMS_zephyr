#!/bin/bash

# === Configuration ===
CAN_INTERFACE="can0"              # Name of your CAN interface
TRIGGER_ID="123"                  # CAN ID to look for (in hex, no 0x prefix)
TRIGGER_DATA="01 02 03 04 05 06 07 08"       # Data pattern to match (optional)
REPLY_ID="456"                    # CAN ID for the response
REPLY_DATA="11 22 33 44"         # Data payload for the response

# Format reply for cansend (e.g., 456#11223344)
REPLY_CAN_STRING="${REPLY_ID}#$(echo ${REPLY_DATA} | sed 's/ //g')"

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