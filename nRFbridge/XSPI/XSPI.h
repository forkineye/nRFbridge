/*
 * XSPI.h
 *
 * Project: XSPI
 * Copyright (c) 2014 Shelby Merrick
 * http://www.forkineye.com
 *
 *  This program is provided free for you to use in any way that you wish,
 *  subject to the laws and regulations where you are using it.  Due diligence
 *  is strongly suggested before using this code.  Please give credit where due.
 *
 *  The Author makes no warranty of any kind, express or implied, with regard
 *  to this program or the documentation contained in this document.  The
 *  Author shall not be liable in any event for incidental or consequential
 *  damages in connection with, or arising out of, the furnishing, performance
 *  or use of these programs.
 *
 */ 

#ifndef XSPI_H_
#define XSPI_H_

#include "../config.h"
#include <avr/io.h>
#include <stdbool.h>

#if defined(__AVR_ATXmega16A4__) || \
defined (__AVR_ATxmega16A4U__) || \
defined (__AVR_ATxmega32A4__) || \
defined (__AVR_ATxmega32A4U__) || \
defined (__AVR_ATxmega64A4U__) || \
defined (__AVR_ATxmega128A4U__)
#   define XSPI_MOSI    PIN5_bm
#   define XSPI_MISO    PIN6_bm
#   define XSPI_SCK     PIN7_bm
#   define XSPI_SS      PIN4_bm
#   define XSPI_XCK0    PIN1_bm
#   define XSPI_RXD0    PIN2_bm
#   define XSPI_TXD0    PIN3_bm
#   define XSPI_XCK1    PIN5_bm
#   define XSPI_RXD1    PIN6_bm
#   define XSPI_TXD1    PIN7_bm
#elif defined (__AVR_ATxmega8E5__) || \
defined (__AVR_ATxmega16E5__) || \
defined (__AVR_ATxmega32E5__)
#   define XSPI_MOSI    PIN7_bm
#   define XSPI_MISO    PIN6_bm
#   define XSPI_SCK     PIN5_bm
#   define XSPI_SS      PIN4_bm
#   define XSPI_XCK0    PIN1_bm
#   define XSPI_RXD0    PIN2_bm
#   define XSPI_TXD0    PIN3_bm
/* if remapped? 
#   define XSPI_XCK1	PIN5_bm
#   define XSPI_RXD1	PIN6_bm
#   define XSPI_TXD1	PIN7_bm
*/
#else
#   error ** Device not supported by XSPI **
#endif

/************************************************************************/
/* Normal hardware SPI stuff                                            */
/************************************************************************/

/*! \brief SPI Master initialization function.
 *  \param port         Pointer to the port on which this SPI module resides.
 *  \param spi          Pointer to SPI_t module structure.
 *  \param mode         Clock and polarity mode for SPI.
 *  \param lsb          Set to true for LSB data, false for MSB.
 *  \param prescaler    SPI clock prescaler value.
 *  \param clk2x        Enables SPI clock double-speed.
 */
static inline void xspi_master_init(PORT_t *port, SPI_t *spi, SPI_MODE_t mode, bool lsb, SPI_PRESCALER_t prescaler, bool clk2x) {
    port->DIRSET = XSPI_MOSI | XSPI_SCK | XSPI_SS;
    spi->CTRL = SPI_ENABLE_bm |	SPI_MASTER_bm | mode | prescaler |
            (clk2x ? SPI_CLK2X_bm : 0) | (lsb ? SPI_DORD_bm : 0);
}

/*! \brief SPI Slave initialization function.
 *  \param port Pointer to the port on which this SPI module resides.
 *  \param spi  Pointer to XSPI_t configuration structure.
 *  \param mode Clock and polarity mode for SPI.
 *  \param lsb  Set to true for LSB data, false for MSB.
 */
static inline void xspi_slave_init(PORT_t *port, SPI_t *spi, SPI_MODE_t mode, bool lsb) {
    port->DIRSET = XSPI_MISO;
    port->DIRCLR = XSPI_SCK | XSPI_SS;
    spi->CTRL = SPI_ENABLE_bm | mode | (lsb ? SPI_DORD_bm : 0);
}

/*! \brief Blocking call that sends and returns a single byte.
 *  \param spi  Pointer to SPI_t module structure.
 *  \return     Single byte read from SPI.
 */
static inline uint8_t xspi_transfer_byte(SPI_t *spi, uint8_t val) {
    spi->DATA = val;
    while(!(spi->STATUS & SPI_IF_bm));
    return spi->DATA;
}

/*! \brief Sends a packet, ignoring any returned SPI data.
 *  \param spi  Pointer to SPI_t module structure.
 *  \param data Pointer to the data being sent.
 *  \param len  Length in bytes of the data being sent.
 */
void xspi_send_packet(SPI_t *spi, uint8_t *data, uint8_t len);

/*! \brief Retrieves a packet of data via SPI
 *  \param spi  Pointer to SPI_t module structure.
 *  \param data Pointer to a buffer to store the retrieved data.
 *  \param len  Size of the buffer in bytes.
 */
void xspi_get_packet(SPI_t *spi, uint8_t *data, uint8_t len);


/************************************************************************/
/* USART specific SPI stuff                                             */
/************************************************************************/
#define USART_UDORD_bm USART_CHSIZE2_bm
#define USART_UCPHA_bm USART_CHSIZE1_bm
#define SERIAL_SPI_UBBRVAL(Baud)    ((Baud < (F_CPU / 2)) ? ((F_CPU / (2 * Baud)) - 1) : 0)

/*! \brief SPI Master USART initialization function.
 *  \param port         Pointer to the port on which this SPI module resides.
 *  \param usart        Pointer to USART_t module structure.
 *  \param mode         Clock and polarity mode for SPI.
 *  \param lsb          Set to true for LSB data, false for MSB.
 *  \param prescaler    SPI clock prescaler value.
 *  \param clk2x        Enables SPI clock double-speed.
 */
static inline void xspi_usart_master_init(PORT_t *port, USART_t *usart, SPI_MODE_t mode, uint32_t baudrate) {
    uint16_t baudval = SERIAL_SPI_UBBRVAL(baudrate);
	
    usart->BAUDCTRLB = (baudval >> 8);
    usart->BAUDCTRLA = (baudval & 0xFF);
    // For now, only Mode 0 supported.
    usart->CTRLC = USART_CMODE_MSPI_gc;
    usart->CTRLB = (USART_RXEN_bm | USART_TXEN_bm); 	
}

/*! \brief Blocking call that sends and returns a single byte.
 *  \param usart    Pointer to USART_t module structure.
 *  \param val      Byte to send.
 *  \return         Single byte read from SPI.
 */
static inline uint8_t xspi_usart_transfer_byte(USART_t *usart, uint8_t val) {
    usart->DATA = val;
    while(!(usart->STATUS & USART_TXCIF_bm));
    usart->STATUS = USART_TXCIF_bm;
    return usart->DATA;
}

/*! \brief Blocking call that sends a single buffered byte.
 *  \param usart    Pointer to USART_t module structure.
 *  \param val      Byte to send.
 */
static inline void xspi_usart_send_byte(USART_t *usart, uint8_t val) {
    while (!(usart->STATUS & USART_DREIF_bm));
    usart->DATA = val;
}

/*! \brief Blocking call that retrieves a single byte.
 *  \param usart    Pointer to USART_t module structure.
 *  \return         Next byte from the UART
 */
static inline uint8_t xspi_usart_get_byte(USART_t *usart) {
    return xspi_usart_transfer_byte(usart, 0xFF);
}

/*! \brief Sends a packet of data via UART in Master SPI mode.
 *  \param usart    Pointer to USART_t module structure.
 *  \param data     Pointer to a buffer to store the retrieved data.
 *  \param len      Size of the buffer in bytes.
 */
void xspi_usart_send_packet(USART_t *usart, uint8_t *data, uint8_t len);

/*! \brief Retrieves a packet of data via UART in Master SPI mode.
 *  \param usart    Pointer to USART_t module structure.
 *  \param data     Pointer to a buffer to store the retrieved data.
 *  \param len      Size of the buffer in bytes.
 */
void xspi_usart_get_packet(USART_t *usart, uint8_t *data, uint8_t len);

#endif /* XSPI_H_ */