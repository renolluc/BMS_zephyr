/*
 * SPI_MB.c
 * 
 * Description: This module handles SPI communication for the BMS system.
 * 
 * Author: renolluc / grossfa2
 * Date: 22.03.2025
 * 
 */

#include <SPI_MB.h>

/**
 * @brief Sets the name and logging levels for this module.
 *
 * Possible log levels:
 * - LOG_LEVEL_NONE
 * - LOG_LEVEL_ERR
 * - LOG_LEVEL_WRN
 * - LOG_LEVEL_INF
 * - LOG_LEVEL_DBG
 */
LOG_MODULE_REGISTER(spi, LOG_LEVEL_DBG);

// define spi1 device
const struct device *spi1_dev = DEVICE_DT_GET(SPI_DEVICE);
// define GPIOB device
#define CS_PIN_PB1 1
#define GPIOB_DEVICE DT_NODELABEL(gpiob)
const struct device *gpio_dev = DEVICE_DT_GET(GPIOB_DEVICE);



struct spi_config spi_cfg= {
	// 8-bit word size, MSB first, Master mode  
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	// Frequency in Hz
    .frequency = SPI_FREQ,
    .cs = {.gpio = spi_cs_pb1_spec, .delay = 0},
};

struct spi_config spi_cfg_test= {
	// 8-bit word size, MSB first, Master mode  
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	// Frequency in Hz
    .frequency = SPI_FREQ,
};



//define Test Variables
#define SPI_LOOPBACK_TEST false

#define ISO_SPI_CS1_Pin GPIO_PIN_1



// Default Configuration Register
// data for CRFA, ADCOPT = 1, REFON = 1, GPIOx = 1
static const uint8_t CFGAR[] = {0xF9, 0x00, 0xF0, 0xFF, 0x00, 0x00};
// data for CRFB, MUTE = 1, GPIOx = 1		
static const uint8_t CFGBR[] = {0x0F, 0x80, 0x00, 0x00, 0x00, 0x00};	
// Read Voltages Register	
static const uint8_t RDCV[] = {RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF};
// Read Temp Register	
static const uint8_t RDAUX[] = {RDAUXA, RDAUXB, RDAUXC, RDAUXD};			


/**NEUE FUNKTIONEN
 * 
 * 
 * 
 * **/

/**
 * @brief Wakes up the ADBMS1818 daisy chain.
 *
 * This function sends a series of chip-select (CS) pulses to wake up a daisy chain of ADBMS1818 devices.
 * The sequence consists of (NUM_OF_CLIENTS + 2) pulses. For each pulse:
 *   - The CS pin is set low.
 *   - A 1 µs delay is applied using k_busy_wait().
 *   - The CS pin is set high.
 *   - A 400 µs delay is applied using k_busy_wait() to allow the device to register the pulse.
 * Finally, a short delay of about 10 µs is applied to ensure the device is ready for subsequent SPI communication.
 *
 * @note Adjust the delays if your ADBMS1818 datasheet requires different timings.
 */
void spi_wake_up(void){
    
        /* (NUM_OF_CLIENTS + 2) Wake‑up CS‑Pulse senden */
        for (uint8_t i = 0; i < NUM_OF_CLIENTS + 2; i++) {
            /* CS low */
            gpio_pin_set(gpio_dev, CS_PIN_PB1, 1);
            k_busy_wait(1);      /* 1 µs */

            /* CS high */
            gpio_pin_set(gpio_dev, CS_PIN_PB1, 0);
            k_busy_wait(400);    /* 400 µs */
        }
    
        /* Abschließende Delay von 10 µs, damit die Geräte bereit sind */
        k_busy_wait(10);         /* 10 µs */
}

/**
 * @brief Send a wake-up sequence to the ADBMS1818 using the Daisy Chain method.
 *
 * This function initiates the wake-up process for the ADBMS1818 devices in a daisy chain.
 * It sends a wake-up message (two bytes of 0xFF) over the SPI bus. The function checks if the
 * SPI device is ready before performing the transmission.
 *
 * When the SPI_WAKEUP_LOOPBACK_TEST flag is enabled, the function performs a loopback test:
 * it receives the sent wake-up message, compares the received data with the transmitted data,
 * and prints a success or failure message based on the comparison. This test helps verify proper
 * wiring (i.e., MOSI to MISO connection).
 *
 * In the normal operation mode (when SPI_WAKEUP_LOOPBACK_TEST is disabled), the wake-up message is
 * simply transmitted without expecting a response.
 *
 * A short delay (10 microseconds) is inserted after the message transmission to allow the devices to
 * settle after waking up.
 *
 * @return 0 on success, or a negative errno error code on failure.
 */
int spi_loopback() {
    // Check if the SPI device is ready. If not, log an error message and return -ENODEV.
    if (!device_is_ready(spi1_dev)) {
        LOG_ERR("SPI device is not ready\n");
        return -ENODEV;
    }
    
    // Define the wake-up message (2 bytes with value 0xFF each).
    uint8_t msg_data[2] = {0xAB, 0xCD};

    // Initialize the SPI transmit buffer with the wake-up message.
    struct spi_buf tx_buf = {
        .buf = msg_data,
        .len = sizeof(msg_data)
    };
    // Wrap the transmit buffer in a SPI buffer set.
    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };

    #if SPI_LOOPBACK_TEST
        // If loopback test is enabled, prepare a receive buffer to capture the echoed data.
        struct spi_buf_set rx_test_buf;
        uint8_t rx_data[sizeof(msg_data)] = { 0 };
        struct spi_buf rx_bufs[] = {
            { .buf = rx_data, .len = sizeof(rx_data) }
        };
        rx_test_buf.buffers = rx_bufs;
        rx_test_buf.count = 1;
        
        // Perform the SPI transceive using the test configuration.
        spi_transceive(spi1_dev, &spi_cfg_test, &tx, &rx_test_buf);
        
        // Compare the transmitted wake-up message with the received message.
        bool match = (memcmp(msg_data, rx_data, sizeof(msg_data)) == 0);
        
        // Log the results of the loopback test.
        LOG_DBG("\nSPI Wakeup Loopback Test");
        LOG_DBG("\nSent:    %02X %02X", msg_data[0], msg_data[1]);
        LOG_DBG("\nReceived:%02X %02X", rx_data[0], rx_data[1]);
        if (match) {
            LOG_DBG("\nSPI Loopback SUCCESS!\n");
        } else {
            LOG_ERR("\nSPI Loopback FAILED! Check MOSI-MISO wiring.\n");
        }
    #else
        // In normal operation, perform the SPI transceive without a receive buffer.
        int ret = spi_transceive(spi1_dev, &spi_cfg, &tx, NULL);
        if (ret < 0) {
            LOG_ERR("SPI transceive failed: %d\n", ret);
            return ret;
        }else{
        LOG_INF("\nSPI Message Sent: %02X %02X\n", msg_data[0], msg_data[1]);
    }
    #endif

    // Wait a short period (10 microseconds) after sending the wake-up message.
    k_sleep(K_USEC(10));
    
    return 0;
}

/**
 * @brief Generate a 15-bit Packet Error Code (PEC) for a data buffer.
 *
 * This function calculates a 15-bit PEC using the polynomial:
 *   x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1.
 *
 * The PEC is initialized to 0x0010 and then updated bit-by-bit for each bit in the
 * data array. For each byte, the function iterates from the most-significant bit (bit 7)
 * to the least-significant bit (bit 0), computing intermediate values and updating
 * specific bits of the PEC according to the polynomial. Finally, the result is shifted
 * one last time to finalize the PEC value.
 *
 * @param data   Pointer to the input data bytes over which the PEC is computed.
 * @param length Number of bytes in the input data array.
 *
 * @return The computed 15-bit PEC, stored in a 16-bit unsigned integer.
 */
uint16_t spi_generate_pec(const uint8_t data[], size_t length) {
    // Initial value of PEC (Packet Error Code)
    uint16_t pec = 0x0010;

    // Characteristic polynomial: x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
    for (size_t i = 0; i < length; ++i) {
        for (int bit = 7; bit >= 0; --bit) {
            // XOR the current data bit with the MSB (bit 14) of the current PEC value.
            uint16_t in0 = ((data[i] >> bit) & 0x01) ^ ((pec >> 14) & 0x01);
			
            // Compute intermediate values to update the PEC based on the polynomial.
            uint16_t in3 = in0 ^ ((pec >> 2) & 0x01);
            uint16_t in4 = in0 ^ ((pec >> 3) & 0x01);
            uint16_t in7 = in0 ^ ((pec >> 6) & 0x01);
            uint16_t in8 = in0 ^ ((pec >> 7) & 0x01);
            uint16_t in10 = in0 ^ ((pec >> 9) & 0x01);
            uint16_t in14 = in0 ^ ((pec >> 13) & 0x01);

            // Shift the current PEC value left by 1 bit.
            pec <<= 1;

            // Update specific bits of the PEC based on the intermediate XOR results.
            pec = (pec & 0x3FFF) | (in14 << 14); // Update bit 14
            pec = (pec & 0xFBFF) | (in10 << 10);   // Update bit 10
            pec = (pec & 0xFEFF) | (in8 << 8);     // Update bit 8
            pec = (pec & 0xFF7F) | (in7 << 7);     // Update bit 7
            pec = (pec & 0xFFEF) | (in4 << 4);     // Update bit 4
            pec = (pec & 0xFFF7) | (in3 << 3);     // Update bit 3
            pec = (pec & 0xFFFE) | (in0 << 0);     // Update bit 0
        }
    }

    // Finalize the PEC calculation by shifting left by 1 bit.
    pec <<= 1;

    return pec;
}

/**
 * @brief Creates a 4-byte SPI command for the ADBMS1818.
 *
 * This function takes a 16-bit input command and constructs a 4-byte command packet.
 * The packet consists of a 2-byte command (transmitted most significant byte first)
 * followed by a 2-byte CRC/PEC, which is calculated over the command bytes using the
 * generatePEC function.
 *
 * The packet format is:
 * @code
 * [CMD_MSB][CMD_LSB][PEC_MSB][PEC_LSB]
 * @endcode
 *
 * @param cmd_in The 16-bit command to be sent.
 * @param cmd_out Pointer to an output buffer that must be at least 4 bytes long.
 */
void spi_create_command(uint16_t cmd_in, uint8_t *cmd_out) {
    /* Prepare an array to hold the two command bytes for PEC calculation */
    uint8_t crc_data[2];
    uint16_t crc;

    /* Split the 16-bit command into two 8-bit values */
    crc_data[0] = cmd_in >> 8;        // Most significant byte (MSB)
    crc_data[1] = cmd_in & 0xFF;        // Least significant byte (LSB)

    /* Compute the CRC/PEC for the 2-byte command */
    crc = spi_generate_pec(crc_data, 2);

    /* Build the command packet:
     * - Bytes 0-1: the command (MSB first)
     * - Bytes 2-3: the computed CRC/PEC (MSB first)
     */
    cmd_out[0] = cmd_in >> 8;
    cmd_out[1] = cmd_in & 0xFF;
    cmd_out[2] = crc >> 8;   // PEC MSB
    cmd_out[3] = crc & 0xFF;   // PEC LSB
}

/**
 * @brief Sends a 16-bit command over SPI.
 *
 * This helper function builds a 4-byte command packet using spi_create_command,
 * then transmits it over SPI using Zephyr’s spi_transceive API. The response is
 * discarded (or could be used for validation if needed).
 *
 * @param command The 16-bit command to send.
 * @return 0 on success, or a negative errno error code on failure.
 */
int spi_send_command(uint16_t command) {
    uint8_t cmd[4];
    uint8_t rx[4];
    
    // Build the command packet (command + PEC)
    spi_create_command(command, cmd);
    
    // Wrap the command packet into SPI buffer structures
    struct spi_buf tx_buf = { .buf = cmd, .len = sizeof(cmd) };
    struct spi_buf_set tx_buf_set = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf rx_buf = { .buf = rx, .len = sizeof(rx) };
    struct spi_buf_set rx_buf_set = { .buffers = &rx_buf, .count = 1 };
    
    // Transmit the command over SPI
    return spi_transceive(spi1_dev, &spi_cfg, &tx_buf_set, &rx_buf_set);
}

/**
 * @brief Write a group of registers via SPI.
 *
 * This function performs a SPI transaction to write register data to multiple clients.
 * The transmit buffer is organized as follows:
 * - 4 bytes for the SPI command (which includes a 2-byte command and 2-byte PEC).
 * - For each client, 8 bytes are allocated, where the first 6 bytes are the data to be written
 *   and the last 2 bytes are the computed PEC for the data.
 *
 * The function uses the Zephyr SPI API by encapsulating the transmit and receive buffers
 * in spi_buf and spi_buf_set structures.
 *
 * @param command A 16-bit command to be sent.
 * @param data Pointer to the data buffer containing 6 bytes for each client.
 *
 * @return 0 on success, or a negative errno error code on failure.
 */
int spi_write_registergroup(uint16_t command, uint8_t *data) {
    /* Create a transmit buffer that includes:
     * - 4 bytes for the command (with PEC)
     * - 8 bytes per client (6 data bytes + 2 PEC bytes)
     */
    uint8_t tx_data[4 + NUM_OF_CLIENTS * 8];

    /* Insert the command and its PEC into the beginning of the transmit buffer */
    spi_create_command(command, tx_data);

    /* For each client, copy 6 data bytes into the transmit buffer and compute the PEC for these bytes */
    for (uint16_t i = 0; i < NUM_OF_CLIENTS; i++) {
        for (uint16_t j = 0; j < 6; j++) {
            tx_data[4 + i * 8 + j] = data[i * 6 + j];
        }
        /* Compute the PEC for the data block and store it in the next 2 bytes */
        uint16_t crc = spi_generate_pec(&data[i * 6], 6);
        tx_data[4 + i * 8 + 6] = crc >> 8;
        tx_data[4 + i * 8 + 7] = crc & 0xFF;
    }

    /* Create a receive buffer of the same size as the transmit buffer */
    uint8_t rx_data[sizeof(tx_data)];

    /* Wrap the transmit buffer in an spi_buf structure */
    struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = sizeof(tx_data)
    };
    /* Create a spi_buf_set for the transmit buffer */
    struct spi_buf_set tx_buf_set = {
        .buffers = &tx_buf,
        .count = 1
    };

    /* Wrap the receive buffer in an spi_buf structure */
    struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = sizeof(rx_data)
    };
    /* Create a spi_buf_set for the receive buffer */
    struct spi_buf_set rx_buf_set = {
        .buffers = &rx_buf,
        .count = 1
    };

    /* Execute the SPI transceive operation using the Zephyr API */
    int ret = spi_transceive(spi1_dev, &spi_cfg, &tx_buf_set, &rx_buf_set);
    if (ret < 0) {
        LOG_ERR("SPI transceive failed: %d\n", ret);
        return ret;  /* Return the negative errno code */
    }

    return 0;  /* Success */
}

/**
 * @brief Read a group of registers via SPI.
 *
 * This function performs a SPI transaction to read register data from
 * multiple clients. The transmit data consists of a 4-byte command (command
 * plus PEC) followed by a dummy-filled area (8 bytes per client, representing
 * 6 bytes of expected data and 2 bytes for the PEC).
 *
 * The SPI API is utilized by wrapping the transmit and receive arrays in
 * spi_buf and spi_buf_set structures. After the SPI transceive call, the function
 * validates the PEC for each client by computing the PEC over the received data and
 * comparing it with the received PEC bytes. If the CRC check passes for each client,
 * the 6 data bytes per client are copied into the user-provided data buffer.
 *
 * @param command The 16-bit command to be sent.
 * @param data Pointer to the buffer where the read register data for all clients will be stored.
 *             The buffer should be large enough to accommodate 6 bytes per client.
 *
 * @return 0 on success, or a negative errno error code on failure.
 */
int spi_read_registergroup(uint16_t command, uint8_t *data) {
    /* Allocate a transmit buffer containing:
     *  - 4 bytes for the command (command + PEC)
     *  - 8 bytes per client (6 bytes expected data + 2 bytes PEC)
     */
    uint8_t tx_data[4 + NUM_OF_CLIENTS * 8];

    /* Initialize the data area (after the first 4 bytes for the command)
     * with dummy values.
     */
    for (uint16_t i = 0; i < (sizeof(tx_data) - 4); i++) {
        tx_data[i + 4] = DUMMY;
    }

    /* Fill the first 4 bytes with the SPI command and its calculated PEC */
    spi_create_command(command, tx_data);

    /* Allocate a corresponding receive buffer */
    uint8_t rx_data[sizeof(tx_data)];

    /* Wrap the transmit buffer into an SPI buffer structure */
    struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = sizeof(tx_data)
    };
    /* Group the transmit buffer in a buffer set */
    struct spi_buf_set tx_buf_set = {
        .buffers = &tx_buf,
        .count = 1
    };

    /* Wrap the receive buffer in an SPI buffer structure */
    struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = sizeof(rx_data)
    };
    /* Group the receive buffer into a buffer set */
    struct spi_buf_set rx_buf_set = {
        .buffers = &rx_buf,
        .count = 1
    };

    /* Execute the SPI transaction */
    int ret = spi_transceive(spi1_dev, &spi_cfg, &tx_buf_set, &rx_buf_set);
    if (ret < 0) {
        LOG_ERR("SPI transceive failed: %d\n", ret);
        return ret;
    }

    /* Process each client's received data.
     * For each client, compute the PEC over 6 bytes of data and compare it with the received PEC.
     * If the PEC matches, copy the 6 data bytes to the output buffer.
     */
    for (uint16_t i = 0; i < NUM_OF_CLIENTS; i++) {
        uint16_t computed_crc = spi_generate_pec(&rx_data[4 + i * 8], 6);
        uint16_t received_crc = (rx_data[4 + i * 8 + 6] << 8) | rx_data[4 + i * 8 + 7];

        if (computed_crc != received_crc) {
            LOG_ERR("CRC check failed for client %d: computed 0x%04x, received 0x%04x\n",
                   i, computed_crc, received_crc);
            return -EIO;
        }

        for (uint16_t j = 0; j < 6; j++) {
            data[i * 6 + j] = rx_data[4 + i * 8 + j];
        }
    }
    return 0;
}

/**
 * @brief Reads voltage measurements from multiple clients.
 *
 * This function performs the following sequence:
 * 1. Wakes up the devices using spi_wake_up().
 * 2. Sends the ADC voltage conversion command using spi_send_command(ADCV), which internally
 *    builds and sends the command using spi_create_command.
 * 3. Waits 3 milliseconds for the ADC conversion to complete (using Zephyr's k_msleep).
 * 4. Reads voltage data from 6 different register groups (one per voltage channel) via Read_Registergroup.
 *    Each register group read returns a 6-byte block per client.
 * 5. If the read is successful for a channel, the 6-byte block for each client is placed into the
 *    output buffer in a byte-wise manner. Each client has 36 bytes allocated in the output (6 channels * 6 bytes each).
 * 6. On any read error, the entire output buffer is cleared and the function returns the error status.
 *
 * The input parameter, data_buffer, is declared as a pointer to uint16_t; however, the underlying
 * byte-wise storage is used. This is why a cast to a uint8_t pointer is performed.
 *
 * @param data_buffer Pointer to a 16-bit data buffer that will receive the voltage measurements.
 *                    The buffer must be large enough to hold NUM_OF_CLIENTS * 36 bytes.
 *
 * @return 0 on success, or a negative errno error code on failure.
 */
int spi_read_voltages(uint16_t *data_buffer) {
    /* Cast the 16-bit pointer to an 8-bit pointer to allow for byte-wise access. */
    uint8_t *buffer = (uint8_t*)(data_buffer);
    int status = ETIME;
    /* Temporary buffer for a single transmission (6 bytes per client). */
    uint8_t short_buffer[NUM_OF_CLIENTS * 6];

    /* Wake up the devices. */
    spi_wake_up();
    
    /* Send the ADC conversion command using the new command function which uses spi_create_command internally. */
    spi_send_command(ADCV);

    /* Use Zephyr's delay function instead of HAL_Delay to integrate properly with the system scheduler. */
    k_msleep(3);

    /* Loop over the 6 voltage channels (or register groups). */
    for (uint16_t i = 0; i < 6; i++) {
        status = spi_read_registergroup(RDCV[i], short_buffer);
		if(status < 0) {
            /* On error, clear the entire output buffer and return the error status. */
            memset(buffer, 0, NUM_OF_CLIENTS * 36);
            return status;
        }
        else{
            /* For each client, copy the 6-byte data block from the temporary buffer to the correct 
             * position in the output buffer.
             * Each client receives 36 bytes in total, with each of the 6 channels contributing 6 bytes.
             */
            for (uint16_t j = 0; j < NUM_OF_CLIENTS; j++) {
                for (uint16_t k = 0; k < 6; k++) {
                    buffer[j * 36 + i * 6 + k] = short_buffer[j * 6 + k];
                }
            }
        } 
    }
    return 0;
}

/**
 * @brief Reads auxiliary temperature registers via SPI.
 *
 * This function reads temperature (and auxiliary) data from the device by performing a series
 * of SPI transactions. It uses the following workflow:
 *  - Wake up the device chain using spi_wake_up().
 *  - Send the ADC auxiliary conversion command (ADAX) using spi_send_command().
 *  - Wait 3 ms for the conversion to complete using k_msleep().
 *  - For each of the 4 auxiliary register groups (AUXA to AUXD):
 *      - For AUXA to AUXC (i.e. when i < 3), read 6 bytes per client.
 *      - For AUXD (i.e. when i == 3), read 2 bytes per client.
 *  - The data from each register group is stored in a temporary buffer (sbuffer) and then
 *    copied into the output buffer in a format that allocates 20 bytes per client.
 *  - Finally, the function rearranges the output buffer to remove unwanted data items
 *    (such as reference voltage and certain GPIO values).
 *
 * @note The data_buffer parameter is of type uint16_t* but is accessed as a raw byte array
 *       by casting it to uint8_t*. Ensure that the provided buffer is large enough to hold the
 *       expected data (NUM_OF_CLIENTS * 20 bytes).
 *
 * @param data_buffer Pointer to the output data buffer.
 *
 * @return 0 on success, or a negative error code on failure.
 */
int spi_read_temp(uint16_t *data_buffer) {
    int status = 0;
    
    /* Cast the 16-bit data buffer to a byte pointer to enable byte-wise access. */
    uint8_t *buffer = (uint8_t*)(data_buffer);

    /* Temporary buffer for a single SPI register group read (6 bytes per client). */
    uint8_t short_buffer[NUM_OF_CLIENTS * 6];

    /* Wake up the device chain. */
    spi_wake_up();
    
    /* Send the ADC auxiliary conversion command using spi_send_command, which utilizes spi_create_command. */
    spi_send_command(ADAX);
    
    /* Delay 3 ms for the ADC conversion to complete. */
    k_msleep(3);
    
    /* Loop through the 4 auxiliary register groups (AUXA, AUXB, AUXC, AUXD). */
    for (uint16_t i = 0; i < 4; i++) {
        /* Read the auxiliary register group specified by RDAUX[i] into the temporary buffer. */
        status = spi_read_registergroup(RDAUX[i], short_buffer);
        
        /* On a read error, clear the entire output buffer (NUM_OF_CLIENTS * 20 bytes) and return the error code. */
        if (status < 0) {
            for (uint16_t j = 0; j < (NUM_OF_CLIENTS * 20); j++) {
                buffer[j] = 0;
            }
            return status;
        } else {
            if (i < 3) {  /* For AUXA to AUXC: copy 6 bytes per client. */
                for (uint16_t j = 0; j < NUM_OF_CLIENTS; j++) {
                    for (uint16_t k = 0; k < 6; k++) {
                        buffer[j * 20 + i * 6 + k] = short_buffer[j * 6 + k];
                    }
                }
            } else {      /* For AUXD: copy only 2 bytes per client. */
                for (uint16_t j = 0; j < NUM_OF_CLIENTS; j++) {
                    for (uint16_t k = 0; k < 2; k++) {
                        buffer[j * 20 + i * 6 + k] = short_buffer[j * 6 + k];
                    }
                }
            }
        }
    }

    /* Rearrange the buffer to remove reference voltage and specific GPIO values.
     * This loop shifts the data within the output buffer starting at index 10 up to
     * 16 * NUM_OF_CLIENTS bytes. The variable 'j' is initially set to 12 and is incremented
     * differently based on its modulo relative to 20 to skip unwanted data.
     */
    uint16_t j = 12;
    for (uint16_t i = 10; i < 16 * NUM_OF_CLIENTS; i++) {
        buffer[i] = buffer[j];
        if ((j % 20 == 9) || (j % 20 == 17)) {
            j += 3;
        } else {
            j++;
        }
    }
    
    return status;
}

/**
 * @brief Reads and returns the maximum temperature from ADBMS status registers.
 *
 * This function performs the following steps:
 *  1. Wakes up the device chain by calling wake_up().
 *  2. Sends the ADC status command (ADSTAT) using spi_send_command() to initiate 
 *     temperature conversion.
 *  3. Waits for 3 milliseconds using k_msleep() to allow the ADC conversion to complete.
 *  4. Reads the status register group using spi_read_registergroup() with the command RDSTATA.
 *  5. For each client, extracts the 16-bit raw temperature value from the received
 *     data (located at byte indices 2 and 3 of a 6-byte data block per client).
 *  6. Determines the highest (maximum) raw temperature among all clients.
 *  7. Converts the maximum raw temperature to a calibrated value using the formula:
 *
 *         calibrated_temperature = (raw_temperature * 0.0001 / 0.0076) - 276
 *
 * If an SPI error occurs during the register read, the function returns a constant error
 * value of 100.
 *
 * @return The calibrated maximum temperature among all clients, or 100 if an SPI read error occurs.
 */
uint16_t spi_read_adbms_temp() {
    int status = 0;
    // Temporary buffer for a single SPI transmission: 6 bytes per client.
    uint8_t short_buffer[NUM_OF_CLIENTS * 6];
    uint16_t maxtemp = 0;
    uint16_t tempx = 0;
    
    // Wake up the device chain.
    spi_wake_up();
    // Send the ADC status command to initiate temperature conversion.
    spi_send_command(ADSTAT);
    // Wait 3 milliseconds for conversion to complete.
    k_msleep(3);
    
    // Read the status registers using the command RDSTATA.
    status = spi_read_registergroup(RDSTATA, short_buffer);
    if (status < 0) {
        // Return error value 100 if the SPI read fails.
        return 100;
    }
    
    // Loop through each client.
    for (uint8_t i = 0; i < NUM_OF_CLIENTS; i++) {
        // Extract the raw temperature value from the current client's data.
        // Temperature bytes are assumed to be at positions 2 (LSB) and 3 (MSB).
        tempx = short_buffer[i * 6 + 2] | (short_buffer[i * 6 + 3] << 8);
        if (tempx > maxtemp) {
            maxtemp = tempx;
        }
    }
    
    // Convert the maximum raw temperature to a calibrated temperature.
    // The conversion formula is: (raw * 0.0001 / 0.0076) - 276.
    maxtemp = maxtemp * 0.0001 / 0.0076 - 276;
    return maxtemp;
}

/**
 * @brief Configures cell discharge (balancing) settings via SPI.
 *
 * This function configures the discharge settings (cell balancing) for each battery cell by writing
 * configuration data to the device registers over SPI. Two configuration arrays are prepared (config_data_A
 * and config_data_B) based on base arrays (CFGAR and CFGBR). Specific bits in these arrays are modified using
 * data from the cells_to_balance array. The modified configuration data is then sent to the device using
 * spi_write_registergroup(). After a short delay, a final command is sent via spi_send_command() to either 
 * enable (UNMUTE) or disable (MUTE) cell discharge based on whether any cell requires balancing.
 *
 * @param cells_to_balance Pointer to an array of uint32_t values, where each element specifies the balancing
 *                         configuration for the corresponding cell. The array length is assumed to be at least
 *                         NUM_OF_CLIENTS.
 *
 * @return 0 on success, or a negative error code if any SPI operation fails.
 */
int spi_set_discharge_cell_x(uint32_t* cells_to_balance) {
    int status = 0;                      /* Initialize status to 0 (no errors) */
    uint8_t enable_discharge = 0;          /* Flag to indicate if discharge balancing should be enabled */

    spi_wake_up();                /* Wake up the ADBMS devices */

    /* Prepare configuration data arrays for each client (6 bytes per client) */
    uint8_t config_data_A[NUM_OF_CLIENTS * 6];
    uint8_t config_data_B[NUM_OF_CLIENTS * 6];

    for (uint16_t i = 0; i < NUM_OF_CLIENTS; i++) {  /* For each client, generate configuration data */
        for (uint16_t j = 0; j < 6; j++) {
            config_data_A[i * 6 + j] = CFGAR[j];     /* Initialize config_data_A using the CFGAR base array */
            config_data_B[i * 6 + j] = CFGBR[j];     /* Initialize config_data_B using the CFGBR base array */
            if (j == 0) {
                /* For the first byte, modify config_data_B with bits 8-11 from cells_to_balance */
                config_data_B[i * 6 + j] |= (cells_to_balance[NUM_OF_CLIENTS - 1 - i] >> 8) & 0xF0;
            } else if (j == 1) {
                /* For the second byte, modify config_data_B with bits 16-17 from cells_to_balance */
                config_data_B[i * 6 + j] |= (cells_to_balance[NUM_OF_CLIENTS - 1 - i] >> 16) & 0x03;
            } else if (j == 4) {
                /* For the fifth byte, modify config_data_A with the lower 8 bits from cells_to_balance */
                config_data_A[i * 6 + j] |= cells_to_balance[NUM_OF_CLIENTS - 1 - i] & 0xFF;
            } else if (j == 5) {
                /* For the sixth byte, modify config_data_A with bits 8-11 from cells_to_balance */
                config_data_A[i * 6 + j] |= (cells_to_balance[NUM_OF_CLIENTS - 1 - i] >> 8) & 0x0F;
            }
        }
        /* If any cell in the input array is non-zero, enable balancing */
        if (cells_to_balance[i]) {
            enable_discharge = 1;
        }
    }

    /* Write configuration data arrays to device registers via SPI */
    status |= spi_write_registergroup(WRCFGA, config_data_A);
    status |= spi_write_registergroup(WRCFGB, config_data_B);

    k_msleep(1);  /* Small delay to ensure the configuration is properly applied */

    /* Enable or disable cell discharge balancing based on the configuration */
    if (enable_discharge) {
        status |= spi_send_command(UNMUTE);  /* Enable cell discharge (balancing) */
    } else {
        status |= spi_send_command(MUTE);    /* Disable cell discharge */
    }

    return status;  /* Return the aggregated status of all SPI operations */
}

/**
 * @brief Initializes the ADBMS hardware.
 *
 * This function initializes the ADBMS hardware by sending configuration data to the device registers over SPI.
 * The initialization process follows these steps:
 *  - Build configuration data arrays for register groups A and B using base data arrays (CFGAR and CFGBR).
 *  - Wake up the ADBMS devices using spi_wakeup_adbms1818().
 *  - Wait 1 ms to allow hardware stabilization.
 *  - Send the MUTE command to ensure that discharge is disabled during configuration.
 *  - Write the configuration data to registers WRCFGA and WRCFGB using spi_write_registergroup().
 *  - Read back the configuration from registers RDCFGA and RDCFGB using Read_Registergroup().
 *  - Compare the written configuration with the read-back data. If any discrepancies are found, the function
 *    returns an error code (-ENODEV); otherwise, it returns 0 indicating success.
 *
 * @return 0 on success, or a negative error code if hardware initialization fails.
 */
int spi_adbms1818_hw_init() {

    /********************************************************************************/
    //AUSKOMMENTIERT SOLANGE HARDWARE NICHT VERFÜGBAR IST
    /********************************************************************************/


    int status = 0;                           /* Initialize status to 0 (no errors) */
    uint8_t not_valid = 0;                     /* Accumulator for any discrepancies in configuration verification */

    uint8_t config_data_A[NUM_OF_CLIENTS * 6];  /* Array to hold configuration data for register group A */
    uint8_t config_data_B[NUM_OF_CLIENTS * 6];  /* Array to hold configuration data for register group B */
    uint8_t read_data_A[NUM_OF_CLIENTS * 6];    /* Array to store read-back data for register group A */
    uint8_t read_data_B[NUM_OF_CLIENTS * 6];    /* Array to store read-back data for register group B */

    /* Build configuration data for each client using the predefined base arrays CFGAR and CFGBR */
    for (uint16_t i = 0; i < NUM_OF_CLIENTS; i++) {
        for (uint16_t j = 0; j < 6; j++) {
            config_data_A[i * 6 + j] = CFGAR[j];  /* Copy CFGAR into config_data_A for client i */
            config_data_B[i * 6 + j] = CFGBR[j];    /* Copy CFGBR into config_data_B for client i */
        }
    }

    spi_wake_up();   /* Wake up the ADBMS devices */
    k_msleep(1);              /* Wait for 1 ms to ensure device stabilization */

    status |= spi_send_command(MUTE);   /* Send the MUTE command to disable discharge while configuring */

    /* Write the configuration data to the device registers */
    status |= spi_write_registergroup(WRCFGA, config_data_A);
    status |= spi_write_registergroup(WRCFGB, config_data_B);

    /* Read back the configuration from the device registers to verify correct configuration */
    status |= spi_read_registergroup(RDCFGA, read_data_A);
    status |= spi_read_registergroup(RDCFGB, read_data_B);

    if (status < 0) {
        LOG_ERR("SPI transaction failed: %d\n", status);
        return status;      /* Return early if there was an error during SPI transactions */
    }

    /* Compare the written configuration with the read-back data for each client (skip first byte) */
    for (uint16_t i = 0; i < NUM_OF_CLIENTS; i++) {
        for (uint16_t j = 1; j < 6; j++) {
            /* Accumulate differences from register group A and B */
            not_valid += (config_data_A[i * 6 + j] - read_data_A[i * 6 + j]);
            not_valid += (config_data_B[i * 6 + j] - read_data_B[i * 6 + j]);
        }
    }

    /* If any differences were detected, return -ENODEV to indicate a hardware initialization failure */
    if (not_valid) {
        LOG_ERR("Configuration verification failed: %d\n", not_valid);
        return -ENODEV;
    }
    
    return 0;           /* Return 0 on successful initialization */

}

