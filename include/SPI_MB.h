/**
 * @file   SPI_MB.h
 * @brief  API-definition for the SPI Module in the BMS-System.
 * @author renolluc / grossfa2
 * @date   20.04.2025
 */

#ifndef INC_SPI_MB_H_
#define INC_SPI_MB_H_

/* Zephyr-Framework includes */
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
/* Project includes */

// ADBMS Register Addresses
// DATASHEET ADBMS1818 by Analog Devices p. 61
// Write Configuration Register Group A
#define WRCFGA 0x0001
// Write Configuration Register Group B
#define WRCFGB 0x0024
// Read Configuration Register Group A
#define RDCFGA 0x0002
// Read Configuration Register Group B
#define RDCFGB 0x0026


// Read Cell Voltage Register Group A
#define RDCVA 0x0004
// Read Cell Voltage Register Group B
#define RDCVB 0x0006
// Read Cell Voltage Register Group C
#define RDCVC 0x0008
// Read Cell Voltage Register Group D
#define RDCVD 0x000A
// Read Cell Voltage Register Group E
#define RDCVE 0x0009
// Read Cell Voltage Register Group F
#define RDCVF 0x000B

// Read Auxiliary Register Group A
#define RDAUXA 0x000C
// Read Auxiliary Register Group B
#define RDAUXB 0x000E
// Read Auxiliary Register Group C
#define RDAUXC 0x000D
// Read Auxiliary Register Group D
#define RDAUXD 0x000F

// Read Status Register Group A
#define RDSTATA 0x0010
// Read Status Register Group B
#define RDSTATB 0x0012

// Write S Control Register Group
#define WRSCTRL 0x0014
// Write PWM Register Group
#define WRPWM 0x0020
// Write PWM/S Control Register Group B 
#define WRPSB 0x001C
// Read S Control Register Group
#define RDSCTRL 0x0016
// Read PWM Register Group
#define RDPWM 0x0022
// Read PWM/S Register Group
#define RDPSB 0x001E
// Start S Control Pulsing and Poll Status
#define STSCTRL 0x0019
// Clear S Control Register Group
#define CLRSCTRL 0x0018

// Clear Cell Voltage Register Groups
#define CLRCELL 0x0711
// Clear Auxiliary Registers Groups
#define CLRAUX 0x0712
// Clear Status Registers Groups
#define CLRSTAT 0x0713
// Poll ADC Conversion Status
#define PLADC 0x0714
// Diagnose MUX and Poll Status
#define DIAGN 0x0715
// Write Communication Register Group
#define WRCOMM 0x0721
// Read Communication Register Group
#define RDCOMM 0x0722
// Start I2C/SPI Communication
#define STCOMM 0x0723
// Mute Discharge
#define MUTE 0x0028
// Unmute Discharge
#define UNMUTE 0x0029

// Start Cell Voltage ADC Conversion and Poll Status
// 3kHz  Cell, all Cells
#define ADCV 0x0360   
// Start GPIOs ADC Conversion and Poll Status
// 14kHz GPIO
#define ADAX 0x04E0   
// Start Status Group ADC Conversion and Poll Status
// 14kHz internal Temp
#define ADSTAT 0x04EA 


/* Settings */
#define NUM_OF_CLIENTS 8
#define DUMMY 0xAA
/* Frequency in Hz */
#define SPI_FREQ 1000000


// Zephyr SPI device binding
#define SPI_DEVICE DT_NODELABEL(spi1)
extern const struct device *spi1_dev;

static const struct gpio_dt_spec spi_cs_pb1_spec = {
    .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(spi_cs_pb1))),
    .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(spi_cs_pb1), 0),
    .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(spi_cs_pb1), 0),
  };

extern uint16_t spi_generate_pec(const uint8_t data[], size_t len);
extern int spi_read_voltages(uint16_t *data_buffer);
extern int spi_read_temp(uint16_t *data_buffer);
extern uint16_t spi_read_adbms_temp();
extern int spi_set_discharge_cell_x(uint32_t *data_buffer);
void spi_wake_up();
int spi_adbms1818_hw_init();
extern int spi_loopback();

#endif /* INC_SPI_MB_H_ */