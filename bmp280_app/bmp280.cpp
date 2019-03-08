/** 
 * @file bmp280.c
 * @brief Driver for Bosch BMP280 Digital Pressure Sensor.
 */

/***********************************************************
                        Includes
***********************************************************/
#include "bmp280.h"
#include "i2c.h"
#include "spi.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
//CPU frequency needed for util library
#define F_CPU 16000000UL
#include <util/delay.h>

#include "Arduino.h"

/***********************************************************
                        Macros
***********************************************************/
#define PRESS_DATA_LEN 3
#define TEMP_DATA_LEN  3

//pressure register indices
#define PRESS_MSB  0
#define PRESS_LSB  1
#define PRESS_XLSB 2

//temperature register indices
#define TEMP_MSB  0
#define TEMP_LSB  1
#define TEMP_XLSB 2

//BMP280 register map
#define DIG_T1_REG     0x88
#define DIG_T2_REG     0x8A
#define DIG_T3_REG     0x8C
#define DIG_P1_REG     0x8E
#define DIG_P2_REG     0x90
#define DIG_P3_REG     0x92
#define DIG_P4_REG     0x94
#define DIG_P5_REG     0x96
#define DIG_P6_REG     0x98
#define DIG_P7_REG     0x9A
#define DIG_P8_REG     0x9C
#define DIG_P9_REG     0x9E
#define CHIP_ID_REG    0xD0
#define RESET_REG      0xE0
#define STATUS_REG     0xF3
#define CTRL_MEAS_REG  0xF4
#define CONFIG_REG     0xF5
#define PRESS_MSB_REG  0xF7
#define PRESS_LSB_REG  0xF8
#define PRESS_XLSB_REG 0xF9
#define TEMP_MSB_REG   0xFA
#define TEMP_LSB_REG   0xFB
#define TEMP_XLSB_REG  0xFC

//register address boundaries
#define MIN_REG_ADDR (DIG_T1_REG)
#define MAX_REG_ADDR (TEMP_XLSB_REG)

//pressure oversampling settings
#define PRESSURE_OVERSAMPLE_SKIPPED 0x00
#define PRESSURE_OVERSAMPLE_X1      0x04
#define PRESSURE_OVERSAMPLE_X2      0x08
#define PRESSURE_OVERSAMPLE_X4      0x0C
#define PRESSURE_OVERSAMPLE_X8      0x10
#define PRESSURE_OVERSAMPLE_X16     0x14

//temperature oversampling settings
#define TEMPERATURE_OVERSAMPLE_SKIPPED 0x00
#define TEMPERATURE_OVERSAMPLE_X1      0x20
#define TEMPERATURE_OVERSAMPLE_X2      0x40
#define TEMPERATURE_OVERSAMPLE_X4      0x60
#define TEMPERATURE_OVERSAMPLE_X8      0x80
#define TEMPERATURE_OVERSAMPLE_X16     0xA0

//power modes
#define SLEEP_MODE  0x00
#define FORCED_MODE 0x01
#define NORMAL_MODE 0x03

/***********************************************************
                      Typedefs
***********************************************************/
//BMP280 calibration parameters
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
} calibration_param_t;

/***********************************************************
                      Variables
***********************************************************/
static calibration_param_t calibration;
static int32_t t_fine;

/***********************************************************
                  Private Function Prototypes
***********************************************************/
static bool bmp280_read(uint8_t addr, uint8_t *buffer, int size);
static bool bmp280_write(uint8_t addr, uint8_t data);
static inline int16_t byte_swap(int16_t *data) __attribute__ ((optimize("-O2")));

/*!
 * @brief Helper method to byte swap 16-bit values
 * @param[in] data 16-bit value to be byte swapped.
 * @return Return byte-swapped value.
 */
static inline int16_t byte_swap(int16_t data)
{
  int16_t tmp = (data >> 8) & 0x00FF;
  tmp |= ((data << 8) & 0xFF00);
  return tmp;
}

/*!
 * @brief Reads an arbitrary number of bytes from the BMP280 device
 * beginning at the specified address.
 * @param[in] addr Register address to begin reading from.
 * @param[in] size Number of bytes to read.
 * @param[out] buffer Buffer data is read into.
 * @return Boolean true if read was successful. False, otherwise.
*/
static bool bmp280_read(uint8_t addr, uint8_t *buffer, int size)
{
  bool err = false;

  if ((addr < MIN_REG_ADDR) || (addr > MAX_REG_ADDR) ||
      (buffer == NULL) || (size == 0))
  {
    err = true;
  }

  if (!err)
  {
#if BMP280_TWI
    //tell slave which register will be read
    err = i2c_write(BMP280_I2C_ADDR, (uint8_t*)&addr, sizeof(addr), true);
    //read data
    err |= i2c_read(BMP280_I2C_ADDR, buffer, size, false);
#else
    addr |= SPI_READ_MASK;
    //initiate SPI transfer
    spi_master_begin(BMP280_SS);
    //tell BMP280 which address to start reading from
    err = spi_master_write((uint8_t*)&addr, sizeof(addr));
    //read consecutive registers
    err |= spi_master_read(buffer, size);
    //end SPI transfer
    spi_master_end(BMP280_SS);
#endif
  }

  return err;
}

/*!
 * @brief Writes a byte of data to the BMP280 device.
 * @param[in] addr Register address to write to.
 * @param[in] data Data value to write.
 * @return Boolean true if write was successful. False, otherwise.
*/
static bool bmp280_write(uint8_t addr, uint8_t data)
{
  bool err = false;

  if ((addr < MIN_REG_ADDR) || (addr > MAX_REG_ADDR))
  {
    err = true;
  }

  if (!err)
  {
#if BMP280_TWI
    //write register address
    err = i2c_write(BMP280_I2C_ADDR, (uint8_t*)&addr, sizeof(addr), true);
    //write data value
    err |= i2c_write(BMP280_I2C_ADDR, (uint8_t*)&data, sizeof(data), false);
#else
    addr &= SPI_WRITE_MASK;
    //initiate SPI transfer
    spi_master_begin(BMP280_SS);
    //tell BMP280 which register to write
    err = spi_master_write((uint8_t*)&addr, sizeof(addr));
    //write data to BM280 register
    err |= spi_master_write((uint8_t*)&data, sizeof(data));
    //end SPI transfer
    spi_master_end(BMP280_SS);
#endif
  }

  return err;
}

/***********************************************************
                    Public Functions
***********************************************************/
//see bmp280.h
bool bmp280_init(void)
{
  bool err = false;
  uint8_t chipid = 0;
  uint8_t ctrl = 0;
  int16_t data;

#if BMP280_TWI
  //initialize TWI
  i2c_init(DEFAULT_BIT_RATE);
#else
  //slave device pin mask
  uint8_t ss_mask = _BV(BMP280_SS);
  //initialize SPI
  spi_master_init(ss_mask, SPI_DORD_MSB_FIRST, SPI_MODE_0);
#endif
  
  //verify device has expected chip ID
  err = bmp280_read(CHIP_ID_REG, &chipid, sizeof(chipid));

  if (chipid != BMP280_CHIP_ID)
  {
    Serial.println("Failed to locate BM280 sensor.");
    Serial.print("Sensor ID: ");
    Serial.println(chipid, HEX);
    err = true;
  }

  if (!err)
  {
    _delay_ms(1);
	  //read calibration data one register a time
	  //want to avoid making two copies of calibration data in memory
    //sensor returns LSB first so everything has to be byte-swapped
    err = bmp280_read(DIG_T1_REG, (uint8_t*)&data, sizeof(data));
	  calibration.dig_T1 = (uint16_t)byte_swap(data);

	  err |= bmp280_read(DIG_T2_REG, (uint8_t*)&data, sizeof(data));
	  calibration.dig_T2 = byte_swap(data);

	  err |= bmp280_read(DIG_T3_REG, (uint8_t*)&data, sizeof(data));
	  calibration.dig_T3 = byte_swap(data);
	
	  err |= bmp280_read(DIG_P1_REG, (uint8_t*)&data, sizeof(data));
	  calibration.dig_P1 = (uint16_t)byte_swap(data);
	
	  err |= bmp280_read(DIG_P2_REG, (uint8_t*)&data, sizeof(data));
	  calibration.dig_P2 = byte_swap(data);
	
	  err |= bmp280_read(DIG_P3_REG, (uint8_t*)&data, sizeof(data));
	  calibration.dig_P3 = byte_swap(data);
	
	  err |= bmp280_read(DIG_P4_REG, (uint8_t*)&data, sizeof(data));
	  calibration.dig_P4 = byte_swap(data);
	
	  err |= bmp280_read(DIG_P5_REG, (uint8_t*)&data, sizeof(data));
	  calibration.dig_P5 = byte_swap(data);
	
	  err |= bmp280_read(DIG_P6_REG, (uint8_t*)&data, sizeof(data));
	  calibration.dig_P6 = byte_swap(data);
	
	  err |= bmp280_read(DIG_P7_REG, (uint8_t*)&data, sizeof(data));
	  calibration.dig_P7 = byte_swap(data);
	
	  err |= bmp280_read(DIG_P8_REG, (uint8_t*)&data, sizeof(data));
	  calibration.dig_P8 = byte_swap(data);
	
	  err |= bmp280_read(DIG_P9_REG, (uint8_t*)&data, sizeof(data));
	  calibration.dig_P9 = byte_swap(data);
  }
 /*
  * Recommended settings for ultra high resolution, low-power, hand-held devices.
  * https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
  * Section 3.8.2 Table 15
  */
  if (!err)
  {
	  ctrl = PRESSURE_OVERSAMPLE_X16 | TEMPERATURE_OVERSAMPLE_X2 | NORMAL_MODE;
    err = bmp280_write(CTRL_MEAS_REG, ctrl);
  }

  return err;
}

//see bmp280.h
float bmp280_getTemperature(void)
{
  bool err = false;
  uint32_t uncomp_temp = 0;
  float temp = 0;
  uint8_t data[TEMP_DATA_LEN];

  //read 20 bit uncompensated temperature
  err = bmp280_read(TEMP_MSB_REG, data, sizeof(data));

  if (!err)
  {
    //format temperature data
    uncomp_temp = data[TEMP_MSB];
    uncomp_temp <<= 8;
    uncomp_temp |= data[TEMP_LSB];
    uncomp_temp <<= 8;
    uncomp_temp |= data[TEMP_XLSB];
    /*
     * compensate temperature
     * https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
     * Section 3.11.3
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

//see bmp280.h
float bmp280_getPressure(void)
{
  bool err = false;
  uint32_t uncomp_press = 0;
  float press = 0;
  uint8_t data[PRESS_DATA_LEN];
  int64_t p;

  //must read temperature first
  bmp280_getTemperature();
  //read 20 bit uncompensated pressure
  err = bmp280_read(PRESS_MSB_REG, data, sizeof(data));

  if (!err)
  {
    uncomp_press = data[PRESS_MSB];
    uncomp_press <<= 8;
    uncomp_press |= data[PRESS_LSB];
    uncomp_press <<= 8;
    uncomp_press |= data[PRESS_XLSB];
    /*
     * compensate pressure
     * https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
     * Section 3.11.3
    */ 
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