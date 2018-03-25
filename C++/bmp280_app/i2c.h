/***********************************************************

   Nicholas Shanahan (2018)

   Module Name: i2c

   Description:
    - Driver for AVR I2C interface. Also known as the
      Two-Wire Interface (TWI).

 ***********************************************************/

#ifndef I2C_H_
#define I2C_H_

/***********************************************************
						 Includes
***********************************************************/
#include <stdint.h>
#include <stdbool.h>

/***********************************************************
						 Enums
***********************************************************/

//TWI status codes
enum
{
  START = 0x08,
  REPEATED_START = 0x10,
  ADDR_WRITE_ACK = 0x18,
  ADDR_WRITE_NACK = 0x20,
  DATA_WRITE_ACK = 0x28,
  DATA_WRITE_NACK = 0x30,
  ARBITRATION_LOST = 0x38,
  ADDR_READ_ACK = 0x40,
  ADDR_READ_NACK = 0x48,
  DATA_READ_ACK = 0x50,
  DATA_READ_NACK = 0x58
};

/***********************************************************
							Classes
***********************************************************/
class i2c
{
  public:

    //default constructor
    i2c();

    //constructor
    i2c(uint8_t sla, uint8_t bit_rate);

    //deconstructor
    ~i2c();

    /***********************************************************

       Name: write

       Description:
        - Writes n bytes of data to slave device.

       Inputs:
        - data - Pointer to data to write.
        - size - Number of bytes to write.
        - repeat - Repeated start flag.

       Outputs:
        - None.

       Return:
        - Boolean true indicates write was successful. Boolean
     		false indicates write failed.

     ***********************************************************/
    bool write(uint8_t *data, uint8_t size, bool repeat);

    /***********************************************************

       Name: read

       Description:
        - Reads n bytes of data from slave device in buffer.

       Inputs:
        - size - Number of bytes to read.
        - repeat - Repeated start flag.

       Outputs:
        - buffer - Buffer to store read data.

       Return:
        - Boolean true indicates write was successful. Boolean
     		false indicates write failed.

     ***********************************************************/
    bool read(uint8_t *buffer, uint8_t size, bool repeat);

  private:

    //I2C slave address
    uint8_t _sla;
};

#endif





