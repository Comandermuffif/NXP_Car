/***********************************************************************
* Title: Lab 2 - UART Driver
* Purpose: Header for UART routines for serial IO
* Author  : Vincent Coffey (vhc1003@rit.edu)
* Revised : February 02, 2016
* Date: 2/2/2016
* Some of the contents are obtained by the courtesy of the material
*		provided by Freescale Semiconductor, Inc.
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
***********************************************************************/

#include <stdio.h>
#include "MK64F12.h"
#include "uart.h"

//default baud rate 
#define BAUD_RATE 9600
//default system clock (see DEFAULT_SYSTEM_CLOCK  in system_MK64F12.c)
#define SYS_CLOCK 20485760

/*
UART Baud Rate Register High - UART0_BDH
UART Baud Rate Register Low - UART0_BDL
UART Data Register - UART0_D

ALT3
PTB16 - UART0_RX
PTB17 - UART0_TX

*/

/***********************************************************************
* PURPOSE: Initialize UART0 for use
*
* INPUTS:
* RETURNS:
***********************************************************************/
void uart_init()
{
	//define variables for baud rate and baud rate fine adjust
	uint16_t ubd, brfa;

	//Enable clock for UART
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
	
	//Configure the port control register to alternative 3 (which is UART mode for K64)
	PORTB_PCR16 = PORT_PCR_MUX(3);
	PORTB_PCR17 = PORT_PCR_MUX(3);

	/*Configure the UART for establishing serial communication*/

	//Disable transmitter and receiver until proper settings are chosen for the UART module
	UART0_C2 &= ~UART_C2_TE_MASK;
	UART0_C2 &= ~UART_C2_RE_MASK;

	//Select default transmission/reception settings for serial communication of UART by clearing the control register 1
	UART0_C1 &= 0x00;

	//UART Baud rate is calculated by: baud rate = UART module clock / (16 × (SBR[12:0] + BRFD))
	//13 bits of SBR are shared by the 8 bits of UART0_BDL and the lower 5 bits of UART0_BDH 
	//BRFD is dependent on BRFA, refer Table 52-234 in K64 reference manual
	//BRFA is defined by the lower 4 bits of control register, UART0_C4 

	//calculate baud rate settings: ubd = UART module clock/16* baud rate
	ubd = (uint16_t)((SYS_CLOCK)/(BAUD_RATE * 16));  

	//clear SBR bits of BDH
	UART0_BDH &= ~UART_BDH_SBR_MASK;
	
	//distribute this ubd in BDH and BDL
	UART0_BDH |= (ubd >> 8) & UART_BDH_SBR_MASK;
	UART0_BDL = ubd;

	//BRFD = (1/32)*BRFA 
	//make the baud rate closer to the desired value by using BRFA
	brfa = (((SYS_CLOCK*32)/(BAUD_RATE * 16)) - (ubd * 32));

	//write the value of brfa in UART0_C4
	UART0_C4 |= brfa & UART_C4_BRFA_MASK;
		
	//Enable transmitter and receiver of UART
	UART0_C2 |= UART_C2_TE_MASK;
	UART0_C2 |= UART_C2_RE_MASK;

}

/***********************************************************************
* PURPOSE: Read a character from UART0
*
* INPUTS:
* RETURNS:
*		uint8_t retVal - The byte read from UART0
***********************************************************************/
uint8_t uart_getchar()
{
	/* Wait until there is space for more data in the receiver buffer*/
	while((UART0_S1 & UART_S1_RDRF_MASK) != UART_S1_RDRF_MASK)
	{}

	/* Return the 8-bit data from the receiver */
	return UART0_D;
}

/***********************************************************************
* PURPOSE: Put a character for transmission
*
* INPUTS:
*		char ch - The char to put for transmission
* RETURNS:
***********************************************************************/
void uart_putchar(char ch)
{
	/* Wait until transmission of previous bit is complete */
	while((UART0_S1 & UART_S1_TDRE_MASK) != UART_S1_TDRE_MASK)
	{}
	/* Send the character */
	UART0_D = ch;
}

/***********************************************************************
* PURPOSE: Put an array of characters for transmission
*
* INPUTS:
*		char *ptr_str - Pointer to the array of characters
* RETURNS:
***********************************************************************/
void uart_put(char *ptr_str){
	/*use putchar to print string*/
	int i = 0;
	while(ptr_str[i] != 0x0)
	{
		uart_putchar(ptr_str[i++]);
	}
}

/***********************************************************************
* PURPOSE: Put an integer for transmission
*
* INPUTS:
*		int i - The int to put for transmission
* RETURNS:
***********************************************************************/
void uart_putnumU(int i)
{
	char buffer[256];
	sprintf(buffer, "%d", i);
	uart_put(buffer);
}
