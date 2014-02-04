/*
 * config.h
 *
 * Project: nRFbridge
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

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 32000000UL        /* Set our clock define so delay functions are happy */

#define NRF_CHANNEL 100             /* default nRF channel */
#define NRF_RATE XNRF_250KBPS       /* default nRF data rate */

#define USART_BAUDRATE 115200

#endif /* CONFIG_H_ */