/***********************************************************

  Nicholas Shanahan (2018)

  Module Name: bmp280_app

  Description:
   - Application for Bosch BMP280 Digital Pressure Sensor.

***********************************************************/

/***********************************************************
                 Includes
***********************************************************/
#include "bmp280.h"

/***********************************************************
                Defines
***********************************************************/
//meters to feet conversion
#define METERS_TO_FEET 3.28084F

/***********************************************************
                 Variables
***********************************************************/
//BMP280 Digital Pressure Sensor
bmp280 *BMP280;
//error flag
bool err = false;
//Local sea-level atmospheric pressure
const float sea_level_hPa = 1038.8;

/***********************************************************
                Function Prototypes
***********************************************************/
void setup()
{
  Serial.begin(9600);

  BMP280 = new bmp280();
  //initialize sensor
  err = BMP280->init();

  if (err)
  {
    Serial.println("BMP280 Initialization failed.");
    delay(500);
    exit(0);
  }

  Serial.println("BM280 initialization successful!\n");
}

void loop()
{
  float t_celcius = BMP280->getTemperature();
  float pressure = BMP280->getPressure();

  //convert temperature to Fahrenheit
  float t_fahrenheit = ((t_celcius * 9) / 5) + 32;

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

  delay(10000);
}
