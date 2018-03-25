/***********************************************************

   Nicholas Shanahan (2018)

   Module Name: i2c

   Description:
    - Driver for AVR I2C interface. Also known as the
      Two-Wire Interface (TWI).

 ***********************************************************/

/***********************************************************
						Includes
***********************************************************/
#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>
#include "i2c.h"

/***********************************************************
							Includes
***********************************************************/
#define TWI_STATUS (TWSR & 0xF8)

/***********************************************************
					Function Prototypes
***********************************************************/
static void start(void);
static void stop(void);
static void send_byte(void);
static void send_ack(void);
static void send_nack(void);
static void busy_wait(void);

/***********************************************************

   Name: start

   Description:
    - Sends I2C start condition.

 ***********************************************************/
static void start(void)
{
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
}

/***********************************************************

  Name: stop

  Description:
   - Sends I2C stop condition and polls control register to
     confirm that stop was received.

***********************************************************/
static void stop(void)
{
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);

  do {
    //nothing
  } while (!(TWCR & _BV(TWSTO)));
}

/***********************************************************

  Name: send_byte

  Description:
   - Sends a byte of data of I2C bus.

***********************************************************/
static void send_byte(void)
{
  TWCR = _BV(TWINT) | _BV(TWEN);
}

/***********************************************************

  Name: send_ack

  Description:
   - Sends acknowledgement over I2C bus.

***********************************************************/
static void send_ack(void)
{
  TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN);
}

/***********************************************************

  Name: send_nack

  Description:
   - Sends non-acknowledgement over I2C bus.

***********************************************************/
static void send_nack(void)
{
  TWCR = _BV(TWINT) | _BV(TWEN);
}

/***********************************************************

  Name: busy_wait

  Description:
   - Polls control register to determine if I2C interface
     is in use.


***********************************************************/
static void busy_wait(void)
{
  do {
    //nothing
  } while (!(TWCR & _BV(TWINT)));
}

/***********************************************************

  Name: set_master

  Description:
   - Sets the MCU as the master I2C device.

***********************************************************/
static bool set_master(uint8_t sla)
{
  bool err = false;

  start();
  busy_wait();

  if ((TWI_STATUS != START) &&
      (TWI_STATUS != REPEATED_START))
  {

    err = true;
  }

  if (!err)
  {
    TWDR = sla;
    send_byte();
    busy_wait();

    if ((TWI_STATUS != ADDR_WRITE_ACK) &&
        (TWI_STATUS != ADDR_READ_ACK))
    {
      err = true;
    }
  }

  return err;
}

//default constructor
i2c::i2c()
  : _sla(0)
{
  //do nothing
}

//see i2c.h
i2c::i2c(uint8_t sla, uint8_t bit_rate)
  : _sla(sla)
{
  TWSR &= ~(_BV(TWPS0) | _BV(TWPS1));
  TWBR = bit_rate;
  //PRR0 &= ~_BV(PRTWI);
  TWCR |= _BV(TWEN);
}

//see i2c.h
bool i2c::write(uint8_t *data, uint8_t size, bool repeat)
{
  uint8_t sla = _sla << 1;
  bool err = set_master(sla);

  if (!err)
  {
    for (int i = 0; i < size; i++)
    {
      TWDR = data[i];
      send_byte();
      busy_wait();

      if (TWI_STATUS != DATA_WRITE_ACK)
      {
        err = true;
        break;
      }
    }
  }

  if (!repeat)
  {
    stop();
  }

  return err;
}

//see i2c.h
bool i2c::read(uint8_t *buffer, uint8_t size, bool repeat)
{
  bool err = false;

  if (!buffer || size == 0)
  {
    err = true;
  }

  if (!err)
  {
    uint8_t sla = (_sla << 1) | 0x1;
    err = set_master(sla);
  }

  if (!err)
  {
    while (size > 0)
    {
      //new TWI data available
      if (TWCR & _BV(TWINT))
      {
        if (size == 1)
        {
          send_nack();
        }

        else
        {
          send_ack();
        }

        busy_wait();
        *buffer++ = TWDR;
        size--;

        if ((TWI_STATUS != DATA_READ_ACK) &&
            (TWI_STATUS != DATA_READ_NACK))
        {
          err = true;
          break;
        }
      }
    }
  }

  if (!repeat)
  {
    stop();
  }

  return err;
}

//deconstructor
i2c::~i2c()
{
  //do nothing
}
//end i2c.pp
