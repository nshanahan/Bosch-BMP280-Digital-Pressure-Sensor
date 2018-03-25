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
 
#ifndef _SPI_H
#define _SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************
                            Includes
***************************************************************/
#include <stdint.h>
#include <stdbool.h>

/**************************************************************
                            Macros
***************************************************************/
/* 
 * SPI Interface Pin Mappings
 * Architecture-dependent and should be defined by the user.
 */
#define SPI_PORT B //SPI Interface port letter
#define DD_SS    2 //Default Slave Select pin
#define DD_MOSI  3 //Master-Out/Slave-In pin
#define DD_MISO  4 //Master-In/Slave-Out pin
#define DD_SCK	 5 //Synchronizing Clock pin

/* SPI Control Register - see section 18.5.1 of Atmel-8272G */
#define SPIE 7 //SPI Interrupt Enable bit
#define SPE  6 //SPI Enable bit
#define DORD 5 //Data Order bit
#define MSTR 4 //Master/Slave Select bit
#define CPOL 3 //Clock Polarity bit
#define CPHA 2 //Clock Phase bit
#define SPR1 1 //SPI Clock Rate Select bit 1
#define SPR0 0 //SPI Clock Rate Select bit 0

/* SPI Status Register - see section 18.5.2 of Atmel-8272G */
#define SPIF  7 //SPI Interrupt Flag
#define WCOL  6 //Write Collision Flag
//Bits 5:1 reserved
#define SPI2X 0 //Double SPI Speed bit

//Data transfer order
#define SPI_DORD_LSB_FIRST 1
#define SPI_DORD_MSB_FIRST 0

/**************************************************************
                            Typedefs
***************************************************************/
//SPI Modes
typedef enum {
  SPI_MODE_3 = 3, //CPOL=1, CPHA=1
  SPI_MODE_2 = 2, //CPOL=1, CPHA=0
  SPI_MODE_1 = 1, //CPOL=0, CPHA=1
  SPI_MODE_0 = 0  //CPOL=0, CPHA=0
} spi_mode_t;

//SPI Serial Clock (SCK) Prescalers
typedef enum {
	SPI_SCK_DIV_4,
	SPI_SCK_DIV_16,
	SPI_SCK_DIV_64,
	SPI_SCK_DIV_128
} spi_sck_t;

/**************************************************************
                         Public Functions
***************************************************************/  

/*!
 * @brief Initializes the AVR SPI hardware and establishes the MCU
 * as a master device. Default data order is MSB first.
 * @param[in] ss_mask Mask that defines the locations of slave device
 * SS (slave-select) pins on the default SPI port. Ex/ If there are
 * 3 slave SPI devices whose SS lines are connected to pins PB0, 
 * PB1, and PB4 the ss_pins mask would have the value 0x13. If only 
 * one device is present, the default SS pin is defined by the SS
 * macro above.
 * @param[in] dord User must specify the desired data order for SPI 
 * writes. Set as either "SPI_DORD_LSB_FIRST" or "SPI_DORD_MSB_FIRST".
 * @param[in] mode SPI mode to define clock polarity and clock phase.
 * @return None.
 */
void spi_master_init(uint8_t ss_mask, bool dord, spi_mode_t mode);

/*!
 * @brief Initializes the AVR SPI hardware and establishes the
 * MCU as a slave device.
 * @return None.
 */
void spi_slave_init(void);

/*!
 * @brief Sets the prescale value for the SPI Serial Clock (SCK).
 * Default value is f_osc/4 (SPI_SCK_DIV_4).
 * @param[in] prescale Prescale value. Four possible prescale values
 * may be specified (see above).
 * @return None.
 */
void spi_set_sck_prescaler(spi_sck_t prescale);

/*!
 * @brief Sets the order in which a byte is transmitted to least
 * significant bit (LSB) first.
 * @return None.
 */
void spi_set_dord_lsb(void);

/*!
 * @brief Sets the order in which a byte is transmitted to most
 * significant bit (MSB) first.
 * @return None.
 */
void spi_set_dord_msb(void);

/*!
 * @brief Pulls the slave select (SS) AKA chip select (CS) pin
 * low to initiate a transfer as the master SPI device.
 * @return None.
 */
void spi_master_begin(uint8_t pin);

/*!
 * @brief Ends a transfer by asserting the SS line.
 * @return None.
 */
void spi_master_end(uint8_t pin);

/*!
 * @brief Writes n bytes of data (in master mode) to a slave device.
 * @param[in] data Data to be written.
 * @param[in] size Number of bytes to write.
 * @return bool
 */
bool spi_master_write(uint8_t *data, uint8_t size);

/*!
 * @brief Reads n bytes of data (in master mode) from a slave device.
 * @param[in] size Size of data buffer.
 * @param[out] buffer Pointer to buffer to read data into.
 * @return bool
 */
bool spi_master_read(uint8_t *buffer, uint8_t size);

/*!
 * @brief Writes a byte of data to a SPI master.
 * @param[in] data Data to be written to master.
 * @param[in] size Number of bytes to write.
 * @return bool
 */
bool spi_slave_write(uint8_t *data, uint8_t size);

/*!
 * @brief Reads a byte of data (in slave mode) from a SPI master.
 * Shifting is initiated when the master pulls SS low.
 * @return None.
 */
bool spi_slave_read(uint8_t *buffer, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif /* _SPI_H */
