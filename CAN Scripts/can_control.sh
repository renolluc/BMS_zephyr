#!/bin/bash

# ==============================================================================
# CAN Control Script
#
# This script allows you to choose between different CAN functionalities:
# - CAN Auto-Reply (can_autoreply.sh)
# - CAN IVT Mock (can_IVT_mock.sh)
# - CAN Precharge Mock (can_precharge_mock.sh)
#
# === Usage ===
#     ./can_control.sh
#
# === Requirements ===
# - Linux system with SocketCAN support
# - `can-utils` installed (for candump, cansend)
# - Make sure can_autoreply.sh, can_IVT_mock.sh, can_precharge_mock.sh are in the same directory
# ==============================================================================

# === Function Definitions ===
function show_menu() {
    echo "==================================="
    echo "         CAN Control Script        "
    echo "==================================="
    echo "1. Start CAN Auto-Reply (can_autoreply.sh)"
    echo "2. Start CAN IVT Mock (can_IVT_mock.sh)"
    echo "3. Start CAN Precharge Mock (can_precharge_mock.sh)"
    echo "4. Add New Function (Template)"
    echo "5. Exit"
    echo "==================================="
}

function start_autoreply() {
    if [[ -x "./can_autoreply.sh" ]]; then
        echo "[INFO] Starting CAN Auto-Reply..."
        ./can_autoreply.sh
    else
        echo "[ERROR] can_autoreply.sh is not executable or not found."
        exit 1
    fi
}

function start_ivt_mock() {
    if [[ -x "./can_IVT_mock.sh" ]]; then
        echo "[INFO] Starting CAN IVT Mock..."
        ./can_IVT_mock.sh
    else
        echo "[ERROR] can_IVT_mock.sh is not executable or not found."
        exit 1
    fi
}

function start_precharge_mock() {
    if [[ -x "./can_precharge_mock.sh" ]]; then
        echo "[INFO] Starting CAN Precharge Mock..."
        ./can_precharge_mock.sh
    else
        echo "[ERROR] can_precharge_mock.sh is not executable or not found."
        exit 1
    fi
}

function add_new_function() {
    echo "==================================="
    echo "Enter the name of the new function (without .sh):"
    read new_function
    new_function_script="${new_function}.sh"

    echo "Creating new function: ${new_function_script}..."
    cat << EOF > $new_function_script
#!/bin/bash
# ==============================================================
# ${new_function}.sh
#
# Description: Describe the purpose of this new function here.
#
# === Requirements ===
# - Linux system with SocketCAN support
# - 'can-utils' installed
#
# === Usage ===
#     ./${new_function_script}
# ==============================================================

echo "[INFO] This is your new CAN function: ${new_function}."
echo "[INFO] Customize this script to perform your desired task."
EOF

    chmod +x $new_function_script
    echo "[INFO] New function ${new_function_script} created and made executable."
}

# === Main Control Loop ===
while true; do
    show_menu
    echo "Select an option (1-5):"
    read choice

    case $choice in
        1) start_autoreply ;;
        2) start_ivt_mock ;;
        3) start_precharge_mock ;;
        4) add_new_function ;;
        5) echo "Exiting..."; exit 0 ;;
        *) echo "[ERROR] Invalid choice. Please try again." ;;
    esac
done
