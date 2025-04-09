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
    .frequency = SPI_FREQ,
	// Chip select control
	// DEFAULT PIN A3/PA4
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

// Function to generate 15-bit packet error code (PEC)
uint16_t spi_generate_pec(const uint8_t data[], size_t length) {
	// Initial value of PEC (Packet Error Code)
	uint16_t pec = 0x0010;

	// Characteristic polynomial: x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
	for (size_t i = 0; i < length; ++i) {
		for (int bit = 7; bit >= 0; --bit) {
			// XOR the input bit with the MSB of the current PEC
			uint16_t in0 = ((data[i] >> bit) & 0x01) ^ ((pec >> 14) & 0x01);
			

			// Update specific bits of the PEC based on the polynomial
			uint16_t in3 = in0 ^ ((pec >> 2) & 0x01);
			uint16_t in4 = in0 ^ ((pec >> 3) & 0x01);
			uint16_t in7 = in0 ^ ((pec >> 6) & 0x01);
			uint16_t in8 = in0 ^ ((pec >> 7) & 0x01);
			uint16_t in10 = in0 ^ ((pec >> 9) & 0x01);
			uint16_t in14 = in0 ^ ((pec >> 13) & 0x01);

			// Shift the PEC left by 1 bit
			pec <<= 1;

			// Apply the polynomial updates to the PEC
			pec = (pec & 0x3FFF) | (in14 << 14); // Update bit 14
			pec = (pec & 0xFBFF) | (in10 << 10); // Update bit 10
			pec = (pec & 0xFEFF) | (in8 << 8);   // Update bit 8
			pec = (pec & 0xFF7F) | (in7 << 7);   // Update bit 7
			pec = (pec & 0xFFEF) | (in4 << 4);   // Update bit 4
			pec = (pec & 0xFFF7) | (in3 << 3);   // Update bit 3
			pec = (pec & 0xFFFE) | (in0 << 0);   // Update bit 0
		}
	}

	// Shift the PEC left by 1 bit to finalize the calculation
	pec <<= 1;

	return pec;
}

// Function to send command
HAL_StatusTypeDef spi_send_command(uint8_t cmd_high, uint8_t cmd_low) {
	// Command buffer
    uint8_t cmd[4] = {cmd_high, cmd_low, 0, 0};
	// Compute CRC-15 for command
    uint16_t crc = spi_generate_pec(cmd, 2);
	// Add CRC to command buffer
    cmd[2] = (crc >> 8) & 0xFF;
    cmd[3] = crc & 0xFF;

    struct spi_buf tx_buf = {.buf = cmd, .len = sizeof(cmd)};
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    int ret = spi_transceive(spi1_dev, &spi_cfg, &tx, NULL);
    
	if (ret < 0) {
        printk("SPI Transceive failed: %d\n", ret);
        return HAL_ERROR;
    }

    return HAL_OK;
}

// Function to read response
int spi_read_response(uint8_t *data, size_t len) {
    uint8_t response[len + 2];  // Data + 2 bytes PEC
    struct spi_buf rx_buf = {.buf = response, .len = sizeof(response)};
    struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    if (spi_read(spi1_dev, &spi_cfg, &rx) != 0) {
        return -1;  // SPI error
    }

    // Validate PEC
    uint16_t received_pec = (response[len] << 8) | response[len + 1];
    if (spi_generate_pec(response, len) != received_pec) {
        return -2;  // PEC error
    }

    // Copy valid data
    memcpy(data, response, len);
    return 0;
}
