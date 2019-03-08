/** @file bmp280_app
 *  
 *  @brief Test application for BMP280 Digital Pressure Sensor.
 *  
 *  @par
 *  Nicholas Shanahan (2018)
 */

 /***********************************************************
                 Includes
***********************************************************/
#include "bmp280.h"

/***********************************************************
                Macros
***********************************************************/
//meters to feet conversion
#define METERS_TO_FEET 3.28084F

/***********************************************************
                 Variables
***********************************************************/
//error flag
bool err = false;

//Local sea-level atmospheric pressure
const float sea_level_hPa = 1038.8;

void setup()
{
  Serial.begin(9600);
  err = bmp280_init();

  if (err)
  {
    Serial.println("BMP280 initialization failed.");
    delay(1000);
    exit(0);
  }

  Serial.println("BMP280 initialization successful!\n");
}

void loop()
{
  float t_celsius = bmp280_getTemperature();
  float pressure = bmp280_getPressure();

   //convert temperature to Fahrenheit
  float t_fahrenheit = ((t_celsius * 9) / 5) + 32;

  Serial.print("Temperature (ºF): ");
  Serial.println(t_fahrenheit, 4);
  Serial.print("Pressure (Pa): ");
  Serial.println(pressure, 4);

  //compute altitude
  pressure /= 100;
  float altitude = (44330 * (1.0 - pow(pressure / sea_level_hPa, 0.1903))) * METERS_TO_FEET;

  Serial.print("Altitude (Feet): ");
  Serial.println(altitude, 4);
  Serial.println();

  delay(2000);
}

