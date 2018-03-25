/*************************************************************
 *
 * @file spi.c
 * 
 * @par Nicholas Shanahan (2018)
 *
 * @brief Driver for AVR serial peripheral interface (SPI).
 * Utilizes existing AVR SPI hardware. Driver allows for
 * multiple slave devices (5) when the AVR is configured
 * as the  master (must be on same port). SPI Double Speed mode
 * is not supported by this driver because it does not guarantee
 * reliable operation.
 * 
 *************************************************************/
 
/**************************************************************
                            Includes
***************************************************************/
#include "spi.h"
#include <avr/io.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
//CPU frequency required for util library
#define F_CPU 16000000UL
#include <util/delay.h>

/**************************************************************
                            Macros
***************************************************************/
#define CONCAT(A,B)  (A##B)
#define PORT(letter) (CONCAT(PORT,letter))
#define DDR(letter)  (CONCAT(DDR,letter))

//Set clock polarity
#define SPI_CPOL_IDLE_HIGH 1
#define SPI_CPOL_IDLE_LOW  0
//Set clock phase (sampling edge)
#define SPI_CPHA_FALLING_EDGE 1
#define SPI_CPHA_RISING_EDGE  0

//SPI dummy byte value
#define SPI_DUMMY 0x00

//clock period in microseconds
#define CPU_CLK_PERIOD ((1 / F_CPU) * 1000000UL)

/**************************************************************
                            Variables
***************************************************************/
bool spi_master_mode;

/**************************************************************
                     Private Function Prototypes
***************************************************************/
static inline void pull_ss_low(uint8_t pin) __attribute__ ((always_inline));
static inline void pull_ss_high(uint8_t pin) __attribute__ ((always_inline));
static inline bool spi_is_busy(void) __attribute__ ((always_inline));
static uint8_t spi_read_write_byte(uint8_t data);

/*!
 * @brief Pulls the Slave Select pin low.
 * @param[in] pin Pin number slave is connected to.
 * @return None.
 */
static inline void pull_ss_low(uint8_t pin)
{
  PORT(SPI_PORT) &= ~_BV(pin);
  _delay_us(CPU_CLK_PERIOD * 2);
}

/*!
 * @brief Pulls the Slave Select pin high.
 * @param[in] pin Pin number slave is connected to.
 * @return None.
 */
static inline void pull_ss_high(uint8_t pin)
{
  PORT(SPI_PORT) |= _BV(pin);
  _delay_us(CPU_CLK_PERIOD * 2);
}

/*!
 * @brief Polls the SPI interrupt flag to determine if
 * hardware is busy.
 * @return bool
 */
static inline bool spi_is_busy(void)
{
  return (!(SPSR & (1 << SPIF)));
}
 
/*!
 * @brief Writes a byte of data to the circular buffer and
 * reads the new value pushed into the buffer.
 * @param[in] data Byte to be written.
 * @return uint8_t
 */
static uint8_t spi_read_write_byte(uint8_t data)
{
  SPDR = data;
  
  while (spi_is_busy());
  
  return SPDR;
}

/*!
 * @brief Writes some number of bytes to slave SPI device.
 * @param[in] data Data to be written to bus.
 * @param[in] size Number of bytes to write.
 * @return bool
 */
static bool spi_write(uint8_t *data, uint8_t size)
{
  uint8_t idx = 0;
  bool err = false;
  
  if ((data == NULL) || (size == 0))
  {
    err = true;
  }
  
  else
  { 
    for (idx = 0; idx < size; idx++)
    {
      spi_read_write_byte(data[idx]);
    }
  }
  
  return err;
}

/**************************************************************
                         Public Functions
***************************************************************/  
//See spi.h
void spi_master_init(uint8_t ss_mask, bool dord, spi_mode_t mode)
{
  spi_master_mode = false;
  //configure SPI input pins
  DDR(SPI_PORT) &= ~_BV(DD_MISO);
  //configure SPI output pins
  DDR(SPI_PORT) |= ss_mask | _BV(DD_MOSI) | _BV(DD_SCK);
  //idle slave select (SS) pins high
  PORT(SPI_PORT) |= ss_mask;
  //enable SPI interface and configure as master
  SPCR = _BV(SPE) | _BV(MSTR);
  
  switch (mode)
  {
    //clock polarity - idle high
    //clock phase - falling edge
    case SPI_MODE_3:
      SPCR |= _BV(CPOL) | _BV(CPHA);
      break;
      
    //clock polarity - idle high
    //clock phase - rising edge
    case SPI_MODE_2:
      SPCR |= _BV(CPOL);
      SPCR &= ~_BV(CPHA);
      break;
      
    //clock polarity - idle low
    //clock phase - falling edge
    case SPI_MODE_1:
      SPCR &= ~_BV(CPOL);
      SPCR |= _BV(CPHA);
      break;
      
    //clock polarity - idle low
    //clock phase - rising edge
    case SPI_MODE_0:
      SPCR &= ~(_BV(CPOL) | _BV(CPHA));
      break;
      
    default:
      SPCR &= ~(_BV(CPOL) | _BV(CPHA));
  }
  
  //set data order
  if (dord == SPI_DORD_LSB_FIRST)
  {
    SPCR |= _BV(DORD);
  }
}

//See spi.h
void spi_slave_init(void)
{
  DDR(SPI_PORT) |= _BV(DD_MISO);
  //configure SPI input pins
  DDR(SPI_PORT) &= ~(_BV(DD_SS) | _BV(DD_MOSI) | _BV(DD_SCK));
  //enable SPI interface and configure as slave
  SPCR = _BV(SPE);
}

//See spi.h
void spi_set_sck_prescaler(spi_sck_t prescale)
{
  switch (prescale)
  {
    case SPI_SCK_DIV_4:
      SPCR &= ~(_BV(SPR1) | _BV(SPR0));
      break;
    
    case SPI_SCK_DIV_16:
      SPCR &= ~_BV(SPR1) ;
      SPCR |= _BV(SPR0);
      break;
      
    case SPI_SCK_DIV_64:
      SPCR &= ~_BV(SPR0);
      SPCR |= _BV(SPR1);
      break;
    
    case SPI_SCK_DIV_128:
      SPCR |= _BV(SPR1) | _BV(SPR0);
      break;
    
    default:
       SPCR &= ~(_BV(SPR1) | _BV(SPR0));
  }
}

//See spi.h
void spi_set_dord_lsb(void)
{
  SPCR |= _BV(DORD);
}

//See spi.h
void spi_set_dord_msb(void)
{
  SPCR &= ~_BV(DORD);
}

//See spi.h
void spi_master_begin(uint8_t pin)
{
  spi_master_mode = true;
  //initiate transfer
  pull_ss_low(pin);
}

//See spi.h
void spi_master_end(uint8_t pin)
{
  spi_master_mode = false;
  //end transfer
  pull_ss_high(pin);
}

//See spi.h
bool spi_master_write(uint8_t *data, uint8_t size)
{
  bool err = false;
  
  if (!spi_master_mode)
  {
    err = true;
  }
  
  else
  {
    err = spi_write(data, size);
  }
  
  return err;
}

//See spi.h
bool spi_master_read(uint8_t *buffer, uint8_t size)
{
  uint8_t idx = 0;
  bool err = false;
  
  if ((buffer == NULL) || (size == 0))
  {
    err = true;
  }
  
  else
  {
    for (idx = 0; idx < size; idx++)
    {
      buffer[idx] = spi_read_write_byte(SPI_DUMMY);
    }
  }
  
  return err;
}

//See spi.h
bool spi_slave_write(uint8_t *data, uint8_t size)
{
  return spi_write(data, size);
}

//See spi.h
bool spi_slave_read(uint8_t *buffer, uint8_t size)
{
  uint8_t idx = 0;
  bool err = false;
  
  if ((buffer == NULL) || (size == 0))
  {
    err = true;
  }
  
  else
  {
    for (idx = 0; idx < size; idx++)
    {
      while (spi_is_busy());
      
      buffer[idx] = SPDR; 
    }
  }
  
  return err;
}


