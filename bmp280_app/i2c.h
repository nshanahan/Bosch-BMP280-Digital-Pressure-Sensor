/*! 
 * \file i2c.c
 *
 * \brief Driver for AVR I2C interface. Only supports
 *  Master Transmitter and Master Receiver modes.
 *
 */
 
#ifndef _I2C_H
#define _I2C_H

#ifdef __cplusplus
extern "C" {
#endif

 /***********************************************************
                         Includes
 ***********************************************************/
#include <stdint.h>
#include <stdbool.h>

 /***********************************************************
                        Public Functions
 ***********************************************************/
/*!
 * @brief Initializes TWI hardware and sets bit rate.
 * @param[in] bit_rate Divisor to determine I2C bit rate.
 * @param[out] None.
 * @return None.
 */
void i2c_init(uint8_t bit_rate);

/*!
 * @brief Writes arbitrary number of bytes to slave device.
 * @param[in] sla Salve device address.
 * @param[in] data Pointer to data buffer.
 * @param[in] size Number of bytes to write.
 * @param[in] repeat Flag to indicate whether or not the MCU should
 * maintain control of the I2C bus.
 * @param[out] None.
 * @return bool
 */
bool i2c_write(uint8_t sla, uint8_t *data, uint8_t size, bool repeat);

/*!
 * @brief Reads an arbitrary number of bytes from slave device.
 * @param[in] sla Slave device address.
 * @param[in] size Number of bytes to read. 
 * @param[in] repeat Flag to indicate whether or not the MCU should
 * maintain control of the I2C bus.
 * @param[out] buffer Buffer that data is read into.
 * @return bool
 */
bool i2c_read(uint8_t sla, uint8_t *buffer, uint8_t size, bool repeat);

#ifdef __cplusplus
}
#endif

#endif /* _I2C_H */
