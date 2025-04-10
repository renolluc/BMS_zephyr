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
#include <zephyr/logging/log.h>


// define spi1 device
const struct device *spi1_dev = DEVICE_DT_GET(SPI_DEVICE);


struct spi_config spi_cfg= {
	// 8-bit word size, MSB first, Master mode  
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	// Frequency in Hz
    .frequency = SPI_FREQ
};

struct spi_config spi_cfg_test= {
	// 8-bit word size, MSB first, Master mode  
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
	// Frequency in Hz
    .frequency = SPI_FREQ,
	// Chip select control  
    .cs = NULL,
};



//define Test Variables
#define SPI_WAKEUP_LOOPBACK_TEST false

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

void delay_1us(){	// delay 960ns + pin delay 45ns = 1050ns
	for(volatile uint32_t i=0; i<5; i++);	// 100ns per cycle
}

void wake_up(){
	for(uint8_t i=0; i<NUM_OF_CLIENTS+2; i++){
		GPIOB->BSRR = ISO_SPI_CS1_Pin<<16;	// CS low
		delay_1us();
		GPIOB->BSRR = ISO_SPI_CS1_Pin;	// CS high
		for(uint16_t i=0; i<400;i++){
			delay_1us();
		}
	}
	for(volatile uint32_t i=0; i<100; i++);	// 100ns per cycle, 10us, communication ready time
}

//OLD TRANSCEIVE FUNCTION
HAL_StatusTypeDef SPI_Transceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
	GPIOB->BSRR = ISO_SPI_CS1_Pin<<16;	// CS low
	delay_1us();
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, size, 200);
	GPIOB->BSRR = ISO_SPI_CS1_Pin;	// CS high
	return status;
}

// Function to generate 15-bit packet error code
uint16_t generatePEC(uint8_t data[], size_t length) {
    // Initial value of PEC
    uint16_t pec = 0x0010;
    // Characteristic polynomial: x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
    for (size_t i = 0; i < length; ++i) {
            for (int bit = 7; bit >= 0; --bit) {
                uint16_t in0 = ((data[i] >> bit) & 0x01) ^ ((pec >> 14) & 0x01);
                uint16_t in3 = in0 ^ ((pec >> 2) & 0x01);
                uint16_t in4 = in0 ^ ((pec >> 3) & 0x01);
                uint16_t in7 = in0 ^ ((pec >> 6) & 0x01);
                uint16_t in8 = in0 ^ ((pec >> 7) & 0x01);
                uint16_t in10 = in0 ^ ((pec >> 9) & 0x01);
                uint16_t in14 = in0 ^ ((pec >> 13) & 0x01);
                pec <<= 1;
                pec = (pec & 0x3FFF) | (in14 << 14);
                pec = (pec & 0xFBFF) | (in10 << 10);
                pec = (pec & 0xFEFF) | (in8 << 8);
                pec = (pec & 0xFF7F) | (in7 << 7);
                pec = (pec & 0xFFEF) | (in4 << 4);
                pec = (pec & 0xFFF7) | (in3 << 3);
                pec = (pec & 0xFFFE) | (in0 << 0);
            }
        }
    pec <<=1;
    return pec;
}

HAL_StatusTypeDef Command(uint16_t command){	// checked
	uint8_t tx_data[4];
	uint8_t crc_data[2];
	crc_data[0] = command>>8;
	crc_data[1] = command&0xFF;
	uint16_t crc = generatePEC(crc_data, 2);
	tx_data[0] = command>>8;
	tx_data[1] = command&0xFF;
	tx_data[2] = crc>>8;
	tx_data[3] = crc&0xFF;
	uint8_t rx_data[4];
	return SPI_Transceive(tx_data, rx_data, 4);
}

HAL_StatusTypeDef Write_Registergroup(uint16_t command, uint8_t *data){		// write data to every single client, data length: 6*NUM_OF_CLIENTS
	uint8_t tx_data[4+NUM_OF_CLIENTS*8];
	uint8_t crc_data[2];
	crc_data[0] = command>>8;
	crc_data[1] = command&0xFF;
	uint16_t crc = generatePEC(crc_data, 2);
	tx_data[0] = command>>8;
	tx_data[1] = command&0xFF;
	tx_data[2] = crc>>8;
	tx_data[3] = crc&0xFF;
	for(uint16_t i=0; i<NUM_OF_CLIENTS; i++){
		for(uint16_t j=0; j<6; j++){
			tx_data[4+i*8+j] = data[i*6+j];
		}
		uint16_t crc = generatePEC(&data[i*6], 6);
		tx_data[4+i*8+6] = crc>>8;
		tx_data[4+i*8+7] = crc&0xFF;
	}
	uint8_t rx_data[sizeof(tx_data)];
	return SPI_Transceive(tx_data, rx_data, sizeof(tx_data));
}

HAL_StatusTypeDef Read_Registergroup(uint16_t command, uint8_t *buffer){		// checked, read data for every single client, data length: 6*NUM_OF_CLIENTS
	uint8_t tx_data[4+NUM_OF_CLIENTS*8];
	for(uint16_t i=0; i<sizeof(tx_data)-4; i++){
		tx_data[i+4] = DUMMY;
	}
	uint8_t crc_data[2];
	crc_data[0] = command>>8;
	crc_data[1] = command&0xFF;
	uint16_t crc = generatePEC(crc_data, 2);
	tx_data[0] = command>>8;
	tx_data[1] = command&0xFF;
	tx_data[2] = crc>>8;
	tx_data[3] = crc&0xFF;
	uint8_t rx_data[sizeof(tx_data)];
	HAL_StatusTypeDef status = SPI_Transceive(tx_data, rx_data, sizeof(tx_data));		// read data
	uint16_t not_valid;
	for(uint16_t i=0; i<NUM_OF_CLIENTS; i++){
		crc = generatePEC(&rx_data[4+i*8], 6);
		not_valid = (rx_data[10+i*8]<<8 | rx_data[11+i*8])-crc;		// check crc
		for(uint16_t j=0; j<6; j++){			// write to buffer
			buffer[i*6+j] = rx_data[4+i*8+j];
		}
		if(not_valid){
			return HAL_ERROR;
		}
	}
	return status;
}

HAL_StatusTypeDef Read_Voltages(uint16_t *data_buffer){		// checked, NUM_OF_CLIENTS * 18
	uint8_t *buffer = (uint8_t*)(data_buffer);
	HAL_StatusTypeDef status = HAL_TIMEOUT;
	uint8_t sbuffer[NUM_OF_CLIENTS*6];		// short buffer for a single transmission
	wake_up();
	Command(ADCV);
	HAL_Delay(3);
	for(uint16_t i=0; i<6; i++){
		//wake_up();		// used for debug
		status = Read_Registergroup(RDCV[i], sbuffer);
		if(status==HAL_OK){
			for(uint16_t j=0; j<NUM_OF_CLIENTS; j++){
				for(uint16_t k=0;k<6; k++){
					buffer[j*36+i*6+k] = sbuffer[j*6+k];
				}
			}
		}else{
			for(uint16_t j=0; j<(NUM_OF_CLIENTS*36); j++){
				buffer[j] = 0;
			}
			return status;
		}
	}
	return status;
}

HAL_StatusTypeDef Read_Temp(uint16_t *data_buffer){		// buffer NUM_OF_CLIENTS * 10
	uint8_t *buffer = (uint8_t*)(data_buffer);
	HAL_StatusTypeDef status;
	uint8_t sbuffer[NUM_OF_CLIENTS*6];		// short buffer for a single transmission
	wake_up();
	Command(ADAX);
	HAL_Delay(3);
	for(uint16_t i=0; i<4; i++){
		//wake_up();		// used for debug
		status = Read_Registergroup(RDAUX[i], sbuffer);
		if(status==HAL_OK){
			if(i<3){		// Register AUXA - AUXC
				for(uint16_t j=0; j<NUM_OF_CLIENTS; j++){		// read 6 Bytes
					for(uint16_t k=0;k<6; k++){
						buffer[j*20+i*6+k] = sbuffer[j*6+k];
					}
				}
			}else{			// Register AUXD
				for(uint16_t j=0; j<NUM_OF_CLIENTS; j++){		// read 2 Bytes
					for(uint16_t k=0;k<2; k++){
						buffer[j*20+i*6+k] = sbuffer[j*6+k];
					}
				}
			}
		}else{
			for(uint16_t j=0; j<(NUM_OF_CLIENTS*20); j++){
				buffer[j] = 0;
			}
			return status;
		}
	}
	// delete reference voltage and gpio9 => values 5 and 9
	uint16_t j=12;
	for(uint16_t i=10; i<16*NUM_OF_CLIENTS; i++){
		buffer[i] = buffer[j];
		if((j%20==9) || (j%20==17)){
			j+=3;
		}else{
			j++;
		}
	}
	return status;
}

uint16_t read_ADBMS_Temp(){
	HAL_StatusTypeDef status;
	uint8_t sbuffer[NUM_OF_CLIENTS*6];		// short buffer for a single transmission
	uint16_t maxtemp = 0;
	uint16_t tempx = 0;
	wake_up();
	Command(ADSTAT);
	HAL_Delay(3);
	status = Read_Registergroup(RDSTATA, sbuffer);
	if (status){
		return 100;
	}
	for(uint8_t i=0; i<NUM_OF_CLIENTS; i++){
		tempx = sbuffer[i*6 +2] | (sbuffer[i*6 +3]<<8);
		if (tempx > maxtemp) {
			maxtemp = tempx;
		}
	}
	maxtemp = maxtemp * 0.0001 / 0.0076 - 276;
	return maxtemp;
}

HAL_StatusTypeDef set_DCCx(uint32_t* cells_to_balance){		// set discharge per cell to true/false
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t enable_discharge = 0;

	wake_up();
	// set dccx per client
	uint8_t config_data_A[NUM_OF_CLIENTS*6];
	uint8_t config_data_B[NUM_OF_CLIENTS*6];
	for(uint16_t i=0; i<NUM_OF_CLIENTS; i++){		// create data
		for(uint16_t j=0; j<6; j++){
			config_data_A[i*6+j] = CFGAR[j];
			config_data_B[i*6+j] = CFGBR[j];
			if(j==0){
				config_data_B[i*6+j] |= (cells_to_balance[NUM_OF_CLIENTS-1-i]>>8) & 0xF0;
			}else if(j==1){
				config_data_B[i*6+j] |= (cells_to_balance[NUM_OF_CLIENTS-1-i]>>16) & 0x03;
			}else if(j==4){
				config_data_A[i*6+j] |= cells_to_balance[NUM_OF_CLIENTS-1-i] & 0xFF;
			}else if(j==5){
				config_data_A[i*6+j] |= (cells_to_balance[NUM_OF_CLIENTS-1-i]>>8) & 0x0F;
			}
		}
		if(cells_to_balance[i]){
			enable_discharge = 1;
		}
	}
	status |= Write_Registergroup(WRCFGA, config_data_A);		// send Data
	status |= Write_Registergroup(WRCFGB, config_data_B);

	HAL_Delay(1);

	if(enable_discharge){		// enable/disable balancing
		status|= Command(UNMUTE);
	}else{
		status|= Command(MUTE);
	}

	return status;
}

HAL_StatusTypeDef ADBMS_HW_Init(){
	uint8_t config_data_A[NUM_OF_CLIENTS*6];
	uint8_t config_data_B[NUM_OF_CLIENTS*6];
	for(uint16_t i=0; i<NUM_OF_CLIENTS; i++){
		for(uint16_t j=0; j<6; j++){
			config_data_A[i*6+j] = CFGAR[j];
			config_data_B[i*6+j] = CFGBR[j];
		}
	}
	wake_up();
	HAL_Delay(1);		// timeout for stability
	HAL_StatusTypeDef status = HAL_OK;
	status |= Command(MUTE);
	status |= Write_Registergroup(WRCFGA, config_data_A);
	status |= Write_Registergroup(WRCFGB, config_data_B);
	uint8_t read_data_A[NUM_OF_CLIENTS*6];
	uint8_t read_data_B[NUM_OF_CLIENTS*6];
	status |= Read_Registergroup(RDCFGA, read_data_A);
	status |= Read_Registergroup(RDCFGB, read_data_B);

	if(status != HAL_OK){
		return status;
	}
	uint8_t not_valid = 0;
	for(uint16_t i=0; i<NUM_OF_CLIENTS; i++){
		for(uint16_t j=1; j<6; j++){
			not_valid += (config_data_A[i*6+j] - read_data_A[i*6+j]);
			not_valid += (config_data_B[i*6+j] - read_data_B[i*6+j]);
		}
	}
	if(not_valid){
		return HAL_ERROR;
	}else{
		return HAL_OK;
	}
}



// Wake-up sequence Daisy Chain Method
int spi_wakeup_adbms1818() {
	
	if (!device_is_ready(spi1_dev)) {
        printk("SPI device is not ready\n");
        return -ENODEV;
    }
	
	// Wake-up message
    uint8_t wakeup_msg_data[2] = {0xFF, 0xFF};
	// SPI buffer
    struct spi_buf tx_buf = {.buf = wakeup_msg_data, .len = sizeof(wakeup_msg_data)};
	// SPI buffer set
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

	// Send wake-up message
	#if SPI_WAKEUP_LOOPBACK_TEST
		struct spi_buf_set rx_test_buf;
		uint8_t rx_wakeup_data[sizeof(wakeup_msg_data)] = { 0 };
		struct spi_buf rx_bufs[] = { { .buf = rx_wakeup_data, .len = sizeof(rx_wakeup_data) } };
		rx_test_buf.buffers = rx_bufs;
		rx_test_buf.count = 1;
		spi_transceive(spi1_dev, &spi_cfg_test, &tx, &rx_test_buf);
	    // Check if the received data matches the sent wake-up message
		bool wakeup_match = (memcmp(wakeup_msg_data, rx_wakeup_data, sizeof(wakeup_msg_data)) == 0);
		
		// Print results
        printk("\nSPI Wakeup Loopback Test");
        printk("\nSent:    %02X %02X", wakeup_msg_data[0], wakeup_msg_data[1]);
        printk("\nReceived:%02X %02X", rx_wakeup_data[0], rx_wakeup_data[1]);

        if (wakeup_match) {
            printk("\nSPI Loopback SUCCESS!\n");
        } else {
            printk("\nSPI Loopback FAILED! Check MOSI-MISO wiring.\n");
        }
		
	#else
		spi_transceive(spi1_dev, &spi_cfg, &tx, NULL);
		printk("\nSPI Wakeup Message Sent: %02X %02X\n", wakeup_msg_data[0], wakeup_msg_data[1]);

	#endif
    
    k_sleep(K_USEC(10));  // Small delay after wakeup
	return 0;
}


/* Kann gelöscht werden ist nun in spi_wakeup_adbms1818 drin mit Test Variabel*/
// SPI Physical Loopback Test (MOSI -> MISO)
/*int spi_test_physical_loopback(void)
{
	
    if (!device_is_ready(spi1_dev)) {
        printk("SPI device is not ready\n");
        return -ENODEV;
    }
	
	// Create test pattern
    uint8_t tx_data[] = { 0xA5, 0x5A, 0xC3, 0x3C };
	// Create receive Array with size of test pattern
    uint8_t rx_data[sizeof(tx_data)] = {0};

	// Create SPI TX buffers
    struct spi_buf tx_bufs[] = {
        { .buf = tx_data, .len = sizeof(tx_data) }
    };

	// Create SPI RX buffers
    struct spi_buf rx_bufs[] = {
        { .buf = rx_data, .len = sizeof(rx_data) }
    };

	// Create SPI buffer sets
    struct spi_buf_set tx = { .buffers = tx_bufs, .count = 1 };
    struct spi_buf_set rx = { .buffers = rx_bufs, .count = 1 };

        // Send and receive SPI data
        int ret = spi_transceive(spi1_dev, &spi_cfg, &tx, &rx);
        if (ret < 0) {
            printk("SPI Transceive failed\n");
            return ret;
        }

        // Check if received data matches sent data
        bool match = (memcmp(tx_data, rx_data, sizeof(tx_data)) == 0);

		// Print results
        printk("\nSPI Loopback Test");
        printk("\nSent:    %02X %02X %02X %02X", tx_data[0], tx_data[1], tx_data[2], tx_data[3]);
        printk("\nReceived:%02X %02X %02X %02X", rx_data[0], rx_data[1], rx_data[2], rx_data[3]);

        if (match) {
            printk("\nSPI Loopback SUCCESS!\n");
        } else {
            printk("\nSPI Loopback FAILED! Check MOSI-MISO wiring.\n");
        }

  
    return 0;
}
*/

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
    crc = generatePEC(crc_data, 2);

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
 * @return 0 on success, or a negative errno value if an error occurs during the SPI transaction.
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
        printk("SPI transceive failed: %d\n", ret);
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
 * @return 0 on success, or a negative errno value (e.g. -EIO) if an error occurs.
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
        printk("SPI transceive failed: %d\n", ret);
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
            printk("CRC check failed for client %d: computed 0x%04x, received 0x%04x\n",
                   i, computed_crc, received_crc);
            return -EIO;
        }

        for (uint16_t j = 0; j < 6; j++) {
            data[i * 6 + j] = rx_data[4 + i * 8 + j];
        }
    }
    return 0;
}


