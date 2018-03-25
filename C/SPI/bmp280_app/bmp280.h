/** @file bmp280.h
 *
 * @brief Driver for Bosch BMP280 Digital Pressure Sensor.
 *
 * @par
 * Nicholas Shanahan (2018)
 *
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
//BMP280 Slave Select Pin
#define BMP280_SS 2

//BMP280 Chip ID
#define BMP280_CHIP_ID 0x58

/***********************************************************
                        Public Functions
***********************************************************/
/*!
 * @brief Initializes BMP280 device. Verifies chip ID is correct,
 * reads compensation data, defines sampling settings, and sets 
 * power mode.
 * @return bool
 */
bool bmp280_init(void);

/*!
 * @brief Reads and compensates BMP280 temperature data.
 * @return float
 */
float bmp280_getTemperature(void);

/*!
 * @brief Reads and compensates BM280 pressure data.
 * @return float`
 */
float bmp280_getPressure(void);

#ifdef __cplusplus
}
#endif

#endif /* _BMP280_H */ 
