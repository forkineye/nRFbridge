/*
 * XUSART.h
 *
 * Project: XUSART
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

#ifndef XUSART_H_
#define XUSART_H_

#include "../config.h"
#include "../RingBuffer.h"
#include <avr/io.h>
#include <stdbool.h>

/*
 * \brief Set the baudrate value in the USART module
 *
 * ** This is borrowed from ASF - Thanks Atmel :) **
 *
 * This function sets the baudrate register with scaling regarding the CPU
 * frequency and makes sure the baud rate is supported by the hardware.
 * The function can be used if you don't want to calculate the settings
 * yourself or changes to baudrate at runtime is required.
 *
 * \param usart The USART module.
 * \param baud The baudrate.
 * \param cpu_hz The CPU frequency.
 *
 * \retval true if the hardware supports the baud rate
 * \retval false if the hardware does not support the baud rate (i.e. it's
 *               either too high or too low.)
 */
bool xusart_set_baudrate(USART_t *usart, uint32_t baud, uint32_t cpu_hz);

/*! \brief Sends a packet of data.
 *  \param usart    Pointer to USART_t module structure.
 *  \param data     Pointer to the data packet to send.
 *  \param len      Size of the buffer in bytes.
 */
void xusart_send_packet(USART_t *usart, uint8_t *data, uint8_t len);

/*! \brief Blocking call that retrieves a packet of data.
 *  \param usart    Pointer to USART_t module structure.
 *  \param data     Pointer to a buffer to hold our data packet.
 *  \param len      Size of the buffer in bytes.
 */
void xusart_get_packet(USART_t *usart, uint8_t *data, uint8_t len);

/*! \brief Sends all the contents of a RingBuffer object.
 *  \param usart    Pointer to USART_t module structure.
 *  \param buffer   Pointer to a RingBuffer structure that contains our data.
  */
void xusart_send_buffer(USART_t *usart, RingBuff_t *buffer);

/*! \brief Blocking call that retrieves a packet to a RingBuffer object.
 *  \param usart    Pointer to USART_t module structure.
 *  \param buffer   Pointer to a RingBuffer structure to hold our data.
 *  \param len      Size of the buffer in bytes.
 */
void xusart_get_buffer(USART_t *usart, RingBuff_t *buffer, uint8_t len);

/*! \brief Function that sets the USART frame format.
 *  \param usart        Pointer to the USART module.
 *  \param chsize       Character size.
 *  \param parity       Parity mode.
 *  \param twoStopBits  Flag to enable 2 stop bits.  Set to 0 for 1 stop bit.
 */
static inline void xusart_set_format(USART_t *usart, USART_CHSIZE_t chsize, USART_PMODE_t parity, bool twoStopBits) {
	usart->CTRLC = (uint8_t) chsize | parity | (twoStopBits ? USART_SBMODE_bm : 0);
}                      

/*! \brief Enable USART receiver.
 *  \param usart Pointer to the USART module
 */
static inline void xusart_enable_rx(USART_t *usart) {
     usart->CTRLB |= USART_RXEN_bm;
}     


/*! \brief Disable USART receiver.
 *  \param usart Pointer to the USART module.
 */
static inline void xusart_disable_rx(USART_t *usart) {
     usart->CTRLB &= ~USART_RXEN_bm;
}     


/*! \brief Enable USART transmitter.
 *  \param usart Pointer to the USART module.
 */
static inline void xusart_enable_tx(USART_t *usart) {
    usart->CTRLB |= USART_TXEN_bm;
}    


/*! \brief Disable USART transmitter.
 *  \param _usart Pointer to the USART module.
 */
static inline void xusart_disable_tx(USART_t *usart) {
     usart->CTRLB &= ~USART_TXEN_bm;
}     

/*! \brief Set the mode the USART run in.
 *
 * Set the mode the USART run in. The default mode is asynchronous mode.
 *
 *  \param  usart   Pointer to the USART module register section.
 *  \param  mode    Selects the USART mode. Use  USART_CMODE_t type.
 *
 *  USART modes:
 *  - 0x0        : Asynchronous mode.
 *  - 0x1        : Synchronous mode.
 *  - 0x2        : IrDA mode.
 *  - 0x3        : Master SPI mode.
 */
static inline void xusart_set_mode(USART_t *usart, USART_CMODE_t mode) {                                      \
	usart->CTRLC = (usart->CTRLC & ~USART_CMODE_gm) | mode;
}

/*! \brief Blocking call that sends one byte.
 *  \param usart    Pointer to the USART module.
 *  \param data     The character to send.
 */
static inline void xusart_putchar(USART_t *usart, uint8_t data) {
    while(!(usart->STATUS & USART_DREIF_bm));
    usart->DATA = data;
}    

 /*! \brief Blocking call that returns one byte read from the USART.
 *   \param usart    Pointer to USART_t module structure.
 *   \return         Single byte read from usart->DATA.
 */
static inline uint8_t xusart_getchar(USART_t *usart) {
    while (!(usart->STATUS & USART_RXCIF_bm));
    return usart->DATA;
}    

#endif /* XUSART_H_ */
