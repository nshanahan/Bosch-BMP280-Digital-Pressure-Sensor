/** 
 * @file bmp280.h
 * @brief Driver for Bosch BMP280 Digital Pressure Sensor.
 */
 
#ifndef _BMP280_H
#define _BMP280_H

#ifdef __cplusplus
extern "C" {
#endif

 /***********************************************************
                         Includes
 ***********************************************************/
#include <stdint.h>
#include <stdbool.h>

 /***********************************************************
                          Macros
 ***********************************************************/
//BMP280 I2C slave address
//Only relevant if BMP280_TWI = 1
#define BMP280_I2C_ADDR 0x77

//BMP280 Slave Select Pin
//Only relevant if BMP280_TWI = 0
#define BMP280_SS 2

//BMP280 Chip ID
#define BMP280_CHIP_ID 0x58

//BM280 features I2C and SPI interfaces
//Define as 1 to select I2C or 0 to select SPI
#define BMP280_TWI 1

//100KHz Standard Mode
//Assumes 8MHz clock frequency
#define DEFAULT_BIT_RATE 0x48

 /***********************************************************
                        Public Functions
 ***********************************************************/
/*!
 * @brief Initializes BMP280 device. Verifies chip ID is correct,
 * reads compensation data, defines sampling settings, and sets 
 * power mode.
 * @return True if initialization succeeds. False, otherwise.
 */
bool bmp280_init(void);

/*!
 * @brief Reads and compensates BMP280 temperature data.
 * @return Compensated temperature in degrees Celsius.
 */
float bmp280_getTemperature(void);

/*!
 * @brief Reads and compensates BM280 pressure data.
 * @return Compensated pressure in Pascals.
 */
float bmp280_getPressure(void);
 
#ifdef __cplusplus
}
#endif

#endif /* _BMP280_H */ 
