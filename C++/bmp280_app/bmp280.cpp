/***********************************************************

  Nicholas Shanahan (2018)

  Module Name: bmp280

  Description:
   - Driver for Bosch BMP280 Digital Pressure Sensor.

***********************************************************/

/***********************************************************
								 Includes
***********************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "bmp280.h"
//CPU frequency (required for delay library)
#define F_CPU 16000000UL
#include <util/delay.h>

/***********************************************************
										Defines
***********************************************************/
//register address boundaries
#define MIN_REGISTER_ADDRESS 0x88 //dig_T1 register
#define MAX_REGISTER_ADDRESS 0xFC //temp_xlsb register

//length of pressure data in bytes
#define PRESSURE_DATA_LEN 3

//length of temperature data in bytes
#define TEMPERATURE_DATA_LEN 3

//pressure data byte indices
#define PRESS_MSB  0
#define PRESS_LSB  1
#define PRESS_XLSB 2

//temperature data byte indices
#define TEMP_MSB  0
#define TEMP_LSB	1
#define TEMP_XLSB 2

/***********************************************************
						    Private Method Prototypes
***********************************************************/
//See bmp280.h
bool bmp280::bmp280_read(uint8_t addr, uint8_t *buffer, int size)
{
  bool err = false;

  if ((addr < MIN_REGISTER_ADDRESS) ||
      (addr > MAX_REGISTER_ADDRESS) ||
      (!buffer) || (size == 0))
  {
    err = true;
  }

  if (!err)
  {
    //write register address
    err = I2C->write((uint8_t*)&addr, sizeof(uint8_t), true);
  }

  if (!err)
  {
    //read register value(s)
    err = I2C->read(buffer, size, false);
  }

  return err;
}

//See bmp280.h
bool bmp280::bmp280_write(uint8_t addr, uint8_t data)
{
  bool err = false;

  if ((addr < MIN_REGISTER_ADDRESS) || (addr > MAX_REGISTER_ADDRESS))
  {
    err = true;
  }

  if (!err)
  {
    uint8_t buffer[2] = { addr, data };
    //write data to BMP280
    err = I2C->write((uint8_t*)&buffer, sizeof(buffer), false);
  }

  return err;
}

/***********************************************************
							Public Method Prototypes
***********************************************************/
//See bmp280.h
bmp280::bmp280()
  : t_fine(0), bit_rate(DEFAULT_BIT_RATE)
{

}

//See bmp280.h
bmp280::bmp280(uint8_t bit_rate)
  : t_fine(0)
{
  this->bit_rate = bit_rate;
}

//See bmp280.h
bool bmp280::init(void)
{
  bool err = false;
  uint8_t chipid = 0;

  //instantiate I2C object
  I2C = new i2c(BMP280_I2C_ADDR, this->bit_rate);

  if (!I2C)
  {
    err = true;
  }

  if (!err)
  {
    //read chip ID register
    err = bmp280_read(CHIP_ID_REG, &chipid, sizeof(uint8_t));

    if (chipid != BMP280_CHIP_ID)
    {
      err = true;
    }
  }

  if (!err)
  {
    uint8_t data[sizeof(calib_params)];
    _delay_ms(1);
    //read calibration parameters
    err = bmp280_read(DIG_T1_REG, (uint8_t*)&data, sizeof(calib_params));

    if (!err)
    {
      calibration.dig_T1 = (data[1] << 8) | data[0];
      calibration.dig_T2 = (data[3] << 8) | data[2];
      calibration.dig_T3 = (data[5] << 8) | data[4];
      calibration.dig_P1 = (data[7] << 8) | data[6];
      calibration.dig_P2 = (data[9] << 8) | data[8];
      calibration.dig_P3 = (data[11] << 8) | data[10];
      calibration.dig_P4 = (data[13] << 8) | data[12];
      calibration.dig_P5 = (data[15] << 8) | data[14];
      calibration.dig_P6 = (data[17] << 8) | data[16];
      calibration.dig_P7 = (data[19] << 8) | data[18];
      calibration.dig_P8 = (data[21] << 8) | data[20];
      calibration.dig_P9 = (data[23] << 8) | data[22];
    }
  }

  if (!err)
  {
    /*
       control register settings recommended for ultra high resolution, low-power, handheld devices
       https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
       Section 3.8.2 Table 15
    */
    err = bmp280_write(CTRL_MEAS_REG, PRESSURE_OVERSAMPLE_X16 | TEMPERATURE_OVERSAMPLE_X2 | NORMAL_MODE);
  }

  return err;
}

//See bmp280.h
float bmp280::getTemperature(void)
{
  bool err = false;
  uint32_t uncomp_temp = 0;
  float temp = 0;
  //temperature data buffer
  uint8_t data[TEMPERATURE_DATA_LEN];

  //read 20 bit uncompensated temperature
  err = bmp280_read(TEMP_MSB_REG, (uint8_t*)&data, TEMPERATURE_DATA_LEN);

  if (!err)
  {
    //format temperature data (** Needs to be done in steps like this! **)
    uncomp_temp = data[TEMP_MSB];
    uncomp_temp <<= 8;
    uncomp_temp |= data[TEMP_LSB];
    uncomp_temp <<= 8;
    uncomp_temp |= data[TEMP_XLSB];
    /*
       compensate temperature
       https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
       Section 3.11.3
    */
    int32_t adc_T = uncomp_temp;
    adc_T >>= 4;
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calibration.dig_T1 << 1))) * ((uint32_t)calibration.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calibration.dig_T1)) * ((adc_T >> 4) - ((int32_t)calibration.dig_T1))) >> 12) *
                    ((int32_t)calibration.dig_T3)) >> 14;
    //fine temperature used to calculate pressure
    t_fine = var1 + var2;
    temp = (t_fine * 5 + 128) >> 8;
    temp /= 100;
  }

  return temp;
}

//See bmp280.h
float bmp280::getPressure(void)
{
  bool err = false;
  uint32_t uncomp_press = 0;
  float press = 0;
  //pressure data buffer
  uint8_t data[PRESSURE_DATA_LEN];

  //must read temperature first
  getTemperature();
  //read 20 bit uncompensated pressure
  err = bmp280_read(PRESS_MSB_REG, (uint8_t*)&data, PRESSURE_DATA_LEN);

  if (!err)
  {
    uncomp_press = data[PRESS_MSB];
    uncomp_press <<= 8;
    uncomp_press |= data[PRESS_LSB];
    uncomp_press <<= 8;
    uncomp_press |= data[PRESS_XLSB];
    /*
      compensate pressure
      https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
      Section 3.11.3
    */
    int64_t p;
    int32_t adc_P = uncomp_press;
    adc_P >>= 4;
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)calibration.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calibration.dig_P5) << 17);
    var2 = var2 + (((int64_t)calibration.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calibration.dig_P3) >> 8) + ((var1 * (int64_t)calibration.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calibration.dig_P1) >> 33;

    if (var1 != 0)
    {
      p = 1048576 - adc_P;
      p = (((p << 31) - var2) * 3125) / var1;
      var1 = (((int64_t)calibration.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
      var2 = (((int64_t)calibration.dig_P8) * p) >> 19;
      p = ((p + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7) << 4);
      press = (float)p / 256;
    }
  }

  return press;
}

//See bmp280.h
bmp280::~bmp280()
{
  delete I2C;
}
