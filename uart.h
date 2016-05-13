/***********************************************************************
* Title: UART Driver
* Purpose: Header for UART routines for serial IO
* Author  : Vincent Coffey (vhc1003@rit.edu)
* Revised : February 02, 2016
* Date: 2/2/2016
* Some of the contents are obtained by the courtesy of the material
*		provided by Freescale Semiconductor, Inc.
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
***********************************************************************/

#ifndef UART_H
#define UART_H

#include <stdint.h>

/***********************************************************************
* PURPOSE: Put an array of characters for transmission
*
* INPUTS:
*		char *ptr_str - Pointer to the array of characters
* RETURNS:
***********************************************************************/
void uart_put(char *ptr_str);

/***********************************************************************
* PURPOSE: Initialize UART0 for use
*
* INPUTS:
* RETURNS:
***********************************************************************/
void uart_init(void);

/***********************************************************************
* PURPOSE: Read a character from UART0
*
* INPUTS:
* RETURNS:
*		uint8_t retVal - The byte read from UART0
***********************************************************************/
uint8_t uart_getchar(void);

/***********************************************************************
* PURPOSE: Put a character for transmission
*
* INPUTS:
*		char ch - The char to put for transmission
* RETURNS:
***********************************************************************/
void uart_putchar(char ch);


/***********************************************************************
* PURPOSE: Put an integer for transmission
*
* INPUTS:
*		int i - The int to put for transmission
* RETURNS:
***********************************************************************/
void uart_putnumU(int i);

#endif
