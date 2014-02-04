/*
 * XSPI.c
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

#include "XSPI.h"

void xspi_send_packet(SPI_t *spi, uint8_t *data, uint8_t len) {
    while (len--) {
        spi->DATA = *data++;
        while(!(spi->STATUS & SPI_IF_bm));
    }
}

void xspi_get_packet(SPI_t *spi, uint8_t *data, uint8_t len) {
    while (len--) {
        spi->DATA = 0xFF;
        while(!(spi->STATUS & SPI_IF_bm));
        *data++ = spi->DATA;
    }
}

void xspi_usart_send_packet(USART_t *usart, uint8_t *data, uint8_t len) {
    while (len--)
        xspi_usart_send_byte(usart, *data++);
}	

void xspi_usart_get_packet(USART_t *usart, uint8_t *data, uint8_t len) {
    while (len--)
        *data++ = xspi_usart_get_byte(usart);
}