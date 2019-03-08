/*! 
 * \file i2c.c
 *
 * \brief Driver for AVR I2C interface. Only supports
 *  Master Transmitter and Master Receiver modes.
 *
 */

/***********************************************************
                        Includes
***********************************************************/
#include "i2c.h"
#include <avr/io.h>
#include <util/twi.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/***********************************************************
                        Macros
***********************************************************/

/***********************************************************
                 Private Function Prototypes
***********************************************************/
static void start(void);
static void stop(void);
static void send_byte(void);
static void send_ack(void);
static void send_nack(void);
static void busy_wait(void);

/*!
 * @brief Sends the I2C start condition.
 * @param[in] None.
 * @param[out] None.
 * @return None.
*/
static void start(void)
{
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
}

/*!
 * @brief Sends the I2C stop condition.
 * @param[in] None.
 * @param[out] None.
 * @return None.
*/
static void stop(void)
{
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
  
  while (!(TWCR & _BV(TWSTO)));
}

/*!
 * @brief Sends a byte data over I2C bus.
 * @param[in] None.
 * @param[out] None.
 * @return None.
*/
static void send_byte(void)
{
  TWCR = _BV(TWINT) | _BV(TWEN);
}

/*!
 * @brief Sends a acknowledgment over I2C bus.
 * @param[in] None.
 * @param[out] None.
 * @return None.
*/
static void send_ack(void)
{
  TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN);
}

/*!
 * @brief Sends non-acknowledgment over I2C bus.
 * @param[in] None.
 * @param[out] None.
 * @return None.
*/
static void send_nack(void)
{
  TWCR = _BV(TWINT) | _BV(TWEN);
}

/*!
 * @brief Polls control register to determine if I2C hardware is busy.
 * @param[in] None.
 * @param[out] None.
 * @return None.
*/
static void busy_wait(void)
{
  while (!(TWCR & _BV(TWINT)));
}

/*!
 * @brief Set MCU as I2C master device.
 * @param[in] sla Slave address of MCU.
 * @param[out] None.
 * @return bool
*/
static bool set_master(uint8_t sla)
{
  bool err = false;
  start();
  busy_wait();

  //Verify TWI is in start state
  if ((TW_STATUS != TW_START) &&
      (TW_STATUS != TW_REP_START))
  {
    err = true;
  }

  if (!err)
  {
    //write SLA+R/W to bus
    TWDR = sla;
    send_byte();
    busy_wait();

    //verify SLA+R/W was acknowledged
    if ((TW_STATUS != TW_MT_SLA_ACK) &&
        (TW_STATUS != TW_MR_SLA_ACK))
    {
      err = true;
    }
  }

  return err;
}

/***********************************************************
                       Public Functions
***********************************************************/
//see i2c.h
void i2c_init(uint8_t bit_rate)
{
  TWBR = bit_rate;
  //Disable TWI bit rate prescaler
  TWSR &= ~(_BV(TWPS0) | _BV(TWPS1));
  //Activate TWI
  TWCR |= _BV(TWEN);
}

//see i2c.h
bool i2c_write(uint8_t sla, uint8_t *data, uint8_t size, bool repeat)
{
  uint8_t idx = 0;
  bool err = false;

  if ((sla == 0) || (data == NULL) || (size == 0))
  {
    err = true;
  }

  if (!err)
  {
    sla <<= 1;
    //set MCU to Master Transmitter mode
    err = set_master(sla);

    if (!err)
    {
      for (idx = 0; idx < size; idx++)
      {
        TWDR = data[idx];
        send_byte();
        busy_wait();

        //verify byte was acknowledged by slave
        if (TW_STATUS != TW_MT_DATA_ACK)
        {
          err = true;
          break;
        }
      }
    }
  }

  //release control of bus
  if (!repeat)
  {
    stop();
  }

  return err;
}

//see i2c.h
bool i2c_read(uint8_t sla, uint8_t *buffer, uint8_t size, bool repeat)
{
  uint8_t idx = 0;
  bool err = false;

  if ((sla == 0) || (buffer == NULL) || (size == 0))
  {
    err = true;
  }

  if (!err)
  {
    sla = (sla << 1) | TW_READ;
    //set MCU to Master Receiver mode
    err = set_master(sla);

    if (!err)
    {
      for (idx = 0; idx < size; idx++)
      {
        //wait for new byte to read
        busy_wait();

        //tell slave to stop transmitting
        if (idx == (size - 1))
        {
          send_nack();
        }
        else
        {
          send_ack();
        }

        busy_wait();
        buffer[idx] = TWDR;

        if ((TW_STATUS != TW_MR_DATA_ACK) &&
            (TW_STATUS != TW_MR_DATA_NACK))
        {
          err = true;
          break;
        }
      }
    }
  }

  //release control of bus
  if (!repeat)
  {
    stop();
  }

  return err;
}
