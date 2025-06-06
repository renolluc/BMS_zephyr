/**
 * @file   spi_mb.h
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

/** ADBMS Register Addresses 
* DATASHEET ADBMS1818 by Analog Devices p. 61
*/

/** @brief Write Configuration Register Group A */
#define WRCFGA 0x0001
/** @brief Write Configuration Register Group B */
#define WRCFGB 0x0024
/** @brief Read Configuration Register Group A */
#define RDCFGA 0x0002
/** @brief Read Configuration Register Group B */
#define RDCFGB 0x0026


/** @brief Read Cell Voltage Register Group A */
#define RDCVA 0x0004
/** @brief Read Cell Voltage Register Group B */
#define RDCVB 0x0006
/** @brief Read Cell Voltage Register Group C */
#define RDCVC 0x0008
/** @brief Read Cell Voltage Register Group D */
#define RDCVD 0x000A
/** @brief Read Cell Voltage Register Group E */
#define RDCVE 0x0009
/** @brief Read Cell Voltage Register Group F */
#define RDCVF 0x000B

/** @brief Read Auxiliary Register Group A */
#define RDAUXA 0x000C
/** @brief Read Auxiliary Register Group B */
#define RDAUXB 0x000E
/** @brief Read Auxiliary Register Group C */
#define RDAUXC 0x000D
/** @brief Read Auxiliary Register Group D */
#define RDAUXD 0x000F

/** @brief Read Status Register Group A */
#define RDSTATA 0x0010
/** @brief Read Status Register Group B */
#define RDSTATB 0x0012

/** @brief Write S Control Register Group */
#define WRSCTRL 0x0014
/** @brief Write PWM Register Group */
#define WRPWM 0x0020
/** @brief Write PWM/S Control Register Group B */ 
#define WRPSB 0x001C
/** @brief Read S Control Register Group */
#define RDSCTRL 0x0016
/** @brief Read PWM Register Group */
#define RDPWM 0x0022
/** @brief Read PWM/S Register Group */
#define RDPSB 0x001E
/** @brief Start S Control Pulsing and Poll Status */
#define STSCTRL 0x0019
/** @brief Clear S Control Register Group */
#define CLRSCTRL 0x0018

/** @brief Clear Cell Voltage Register Groups */
#define CLRCELL 0x0711
/** @brief Clear Auxiliary Registers Groups */
#define CLRAUX 0x0712
/** @brief Clear Status Registers Groups */
#define CLRSTAT 0x0713
/** @brief Poll ADC Conversion Status */
#define PLADC 0x0714
/** @brief Diagnose MUX and Poll Status */
#define DIAGN 0x0715
/** @brief Write Communication Register Group */
#define WRCOMM 0x0721
/** @brief Read Communication Register Group */
#define RDCOMM 0x0722
/** @brief Start I2C/SPI Communication */
#define STCOMM 0x0723
/** @brief Mute Discharge */
#define MUTE 0x0028
/** @brief Unmute Discharge */
#define UNMUTE 0x0029

/** @brief Start Cell Voltage ADC Conversion and Poll Status
* 3kHz  Cell, all Cells */
#define ADCV 0x0360   
/** @brief Start GPIOs ADC Conversion and Poll Status
* 14kHz GPIO */
#define ADAX 0x04E0   
/** @brief Start Status Group ADC Conversion and Poll Status
* 14kHz internal Temp */
#define ADSTAT 0x04EA 


/** @brief Settings */
#define NUM_OF_CLIENTS 8
#define DUMMY 0xAA
/** @brief Frequency for SPI-Communication in Hz */
#define SPI_FREQ 500000


/** @brief Zephyr SPI device binding */
#define SPI_DEVICE DT_NODELABEL(spi1)

/** @brief SPI chip select pin specification */
static const struct gpio_dt_spec spi_cs_pin_spec = {
    .port = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(spi_cs_pin))),
    .pin = DT_GPIO_HOG_PIN_BY_IDX(DT_NODELABEL(spi_cs_pin), 0),
    .dt_flags = DT_GPIO_HOG_FLAGS_BY_IDX(DT_NODELABEL(spi_cs_pin), 0),
};

/** @brief read voltages of the battery cells and store them in the data_buffer
 *  @param data_buffer Pointer to a buffer where the cell voltages are stored
 *  @retval 0 on success, negative errno code otherwise.
 */
int spi_read_voltages(uint16_t *data_buffer);

/** @brief read temperatures of the battery cells and store them in the data_buffer
 *  @param data_buffer Pointer to a buffer where the cell temperatures are stored
 *  @retval 0 on success, negative errno code otherwise.
 */
int spi_read_temp(uint16_t *data_buffer);

/** @brief read the adbms1818 chip temperature
 *  @return  returns the highest temperature of the adbms1818 chips in degree Celsius 
 */
uint16_t spi_read_adbms_temp();

/** @brief sets the cells to be discharged
 *  @param data_buffer Pointer to a buffer where the cell discharge mask is stored
 *  @retval 0 on success, negative errno code otherwise.
 */
int spi_set_discharge_cell_x(uint32_t *data_buffer);

/** @brief sends a wake-up signal to the adbms1818 chips
 */
void spi_wake_up();

/** @brief initializes the SPI interface and the GPIOs for the SPI
 *  @retval 0 on success, negative errno code otherwise.
 */
int spi_adbms1818_hw_init();

/** @brief performs a loopback test on the SPI interface
 *  @retval 0 on success, negative errno code otherwise.
 */
int spi_loopback();

#endif /* INC_SPI_MB_H_ */