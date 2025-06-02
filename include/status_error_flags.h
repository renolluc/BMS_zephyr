/**
* @file   status_error_flags.h
* @brief  Flags for the Battery in the BMS-System.
* @author renolluc / grossfa2
* @date   20.04.2025
*/

#ifndef INC_STATUS_ERROR_FLAGS_H_
#define INC_STATUS_ERROR_FLAGS_H_
 
/** Status Flags to ECU, 1 Byte */

/** @brief set to 1 when accumulator system is ok, otherwise 0 */
#define STATUS_BATTERY_OK	    (1<<0)
/** @brief set to 1 when system is in charging mode, otherwise 0 */
#define STATUS_CHARGING		    (1<<1)		
/** @brief set to 1 when MB temperature is ok, set to 0 if one or more MB has overtemperature due to balancing */
#define STATUS_MB_TEMP_OK	    (1<<2)		
/** @brief set to 1 when AIR negative is closed */
#define STATUS_AIR_NEGATIVE     (1<<3)		
/** @brief set to 1 when AIR precharge is closed */
#define STATUS_PRECHARGE 	    (1<<4)		
/** @brief set to 1 when AIR positive is closed */
#define STATUS_AIR_POSITIVE     (1<<5)		
/** @brief unused, set to 0 */
#define STATUS_RESERVE2		    (1<<6)		
/** @brief unused, set to 0 */
#define STATUS_RESERVE1		    (1<<7)		
 
/** >> Error Flags to ECU, 1 Byte */

/** @brief set to 1 if any error with the battery cells occur */
#define ERROR_BATTERY		(1<<0)		
/** @brief set to 1 if any cell has overtemperature */
#define ERROR_TEMP			(1<<1)		
/** @brief set to 1 if any cell has over-/undervoltage */
#define ERROR_VOLT			(1<<2)		
/** @brief set to 1 if SDC is in error state */
#define ERROR_SDC			(1<<3)		
/** @brief set to 1 if an error related to IsoSPI occurs */
#define ERROR_SPI			(1<<4)		
/** @brief set to 1 if CAN buffer overflow occurs */
#define ERROR_CAN			(1<<5)		
/** @brief set to 1 if IVT-S doesn't send data */
#define ERROR_IVT			(1<<6)		
/** @brief unused, set to 0 */
#define ERROR_RESERVE1		(1<<7)		

/** >> Status Flags from ECU, 1 Byte */

#define BATTERY_ON 		    (1<<0)
/** @brief unused, set to 0 */
#define ECU_STATUS_RESERVE1		(1<<1)
/** @brief unused, set to 0 */
#define ECU_STATUS_RESERVE2		(1<<2)
/** @brief unused, set to 0 */
#define ECU_STATUS_RESERVE3		(1<<3)
/** @brief unused, set to 0 */
#define ECU_STATUS_RESERVE4		(1<<4)
/** @brief unused, set to 0 */
#define ECU_STATUS_RESERVE5		(1<<5)
/** @brief unused, set to 0 */
#define ECU_STATUS_RESERVE6		(1<<6)
/** @brief unused, set to 0 */
#define ECU_STATUS_RESERVE7		(1<<7)

/** >> Event Flags */
/** @brief Event bitmask for reporting battery errors */
#define EVT_ERROR_BIT (1 << 0)

#endif /* INC_STATUS_ERROR_FLAGS_H_ */