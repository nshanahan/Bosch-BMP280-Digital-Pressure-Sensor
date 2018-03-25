/***********************************************************

  Nicholas Shanahan (2018)

  Module Name: bmp280

  Description:
   - Driver for Bosch BMP280 Digital Pressure Sensor.

***********************************************************/

#ifndef BMP280_H_
#define BMP280_H_

/***********************************************************
						Includes
***********************************************************/
#include "i2c.h"

/***********************************************************
							Defines
***********************************************************/
//BMP280 I2C slave address
#define BMP280_I2C_ADDR 0x77

//BMP280 Chip ID
#define BMP280_CHIP_ID 0x58

//100KHz Standard Mode
//Assumes 8MHz clock frequency
#define DEFAULT_BIT_RATE 0x48

//Pressure oversampling settings
#define PRESSURE_OVERSAMPLE_SKIPPED 0x00
#define PRESSURE_OVERSAMPLE_X1			0x04
#define PRESSURE_OVERSAMPLE_X2			0x08
#define PRESSURE_OVERSAMPLE_X4			0x0C
#define PRESSURE_OVERSAMPLE_X8			0x10
#define PRESSURE_OVERSAMPLE_X16			0x14

//Temperature oversampling settings
#define TEMPERATURE_OVERSAMPLE_SKIPPED 0x00
#define TEMPERATURE_OVERSAMPLE_X1			 0x20
#define TEMPERATURE_OVERSAMPLE_X2			 0x40
#define TEMPERATURE_OVERSAMPLE_X4			 0x60
#define TEMPERATURE_OVERSAMPLE_X8			 0x80
#define TEMPERATURE_OVERSAMPLE_X16		 0xA0

//Power modes
#define SLEEP_MODE  0x00
#define FORCED_MODE	0x01
#define NORMAL_MODE	0x03

/***********************************************************
							Enums
***********************************************************/
//BMP280 register map
enum
{
  DIG_T1_REG = 0x88,
  DIG_T2_REG = 0x8A,
  DIG_T3_REG = 0x8C,
  DIG_P1_REG = 0x8E,
  DIG_P2_REG = 0x90,
  DIG_P3_REG = 0x92,
  DIG_P4_REG = 0x94,
  DIG_P5_REG = 0x96,
  DIG_P6_REG = 0x98,
  DIG_P7_REG = 0x9A,
  DIG_P8_REG = 0x9C,
  DIG_P9_REG = 0x9E,
  CHIP_ID_REG = 0xD0,
  RESET_REG = 0xE0,
  STATUS_REG = 0xF3,
  CTRL_MEAS_REG = 0xF4,
  CONFIG_REG = 0xF5,
  PRESS_MSB_REG = 0xF7,
  PRESS_LSB_REG = 0xF8,
  PRESS_XLSB_REG = 0xF9,
  TEMP_MSB_REG = 0xFA,
  TEMP_LSB_REG = 0xFB,
  TEMP_XLSB_REG = 0xFC
};

/***********************************************************
						Typedefs
***********************************************************/
typedef struct
{
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
} calib_params;

/***********************************************************
						Classes
***********************************************************/
class bmp280
{
  public:

    //default constructor
    bmp280();

    //constructor
    bmp280(uint8_t bit_rate);

    //deconstructor
    ~bmp280();

    /***********************************************************

      Name: init

      Description:
       - Initializes BMP280 device. Verifies chip ID is correct,
         reads compensation data, and sets control register.

      Inputs:
       - None.

      Outputs:
       - None.

      Return:
       - Returns Boolean true if initialization is successful,
    		 otherwise false.

    ***********************************************************/
    bool init(void);

    /***********************************************************

      Name: getTemperature

      Description:
       - Reads and compensates BMP280 temperature data.

      Inputs:
       - None.

      Outputs:
       - None.

      Return:
       - Returns compensated temperature as a single-precision
         floating point value.

    ***********************************************************/
    float getTemperature(void);

    /***********************************************************

      Name: getPressure

      Description:
       - Reads and compensates BMP280 pressure data.

      Inputs:
       - None.

      Outputs:
       - None.

      Return:
       - Returns compensated presure as a single-precision
         floating point value.

    ***********************************************************/
    float getPressure(void);

  private:

    //I2C bit rate
    uint8_t bit_rate;

    //BMP280 calibration data
    calib_params calibration;

    //BMP280 fine temperature
    int32_t t_fine;

    //I2C interface object
    i2c *I2C;

    /***********************************************************

      Name: bmp280_read

      Description:
       - Reads an arbitrary number of bytes over the I2C bus from
    		 the BMP280 device. Data is read into user provided buffer.

    ***********************************************************/
    bool bmp280_read(uint8_t addr, uint8_t *buffer, int size);

    /***********************************************************

      Name: bmp280_write

      Description:
       - Writes one byte of data to the specified BMP280 register
    		 over the I2C bus (never need to write more than one byte).

    ***********************************************************/
    bool bmp280_write(uint8_t addr, uint8_t data);

};

#endif
