/*
 * Main Method for testing the PWM Code for the K64F
 * PWM signal can be connected to output pins are PC3 and PC4
 * 
 * Author:  
 * Created:  
 * Modified:  
 */

#include <stdlib.h>

#include "MK64F12.h"
#include "uart.h"
#include "DC_Driver.h"
#include "Servo_Driver.h"
#include "Camera_Driver.h"
#include "main.h"

/*
0 = wait
1 = go
2 = dead
*/

/***********************************************************************
* PURPOSE: Main entry point for the program
*
* INPUTS:
* TURNS:
*   int retVal - The exit code of the program
***********************************************************************/
int main(void)
{	
	int lastState = -1;
	initialize();
	uart_put("Running... \n\r");
	
	for(;;)
	{
		if(state == 0 && ((GPIOC_PDIR & (1 << 6)) == 0))
		{
			state = 1;
		}
		
		findLineLocation();
		
		if(state != lastState)
		{
			switch(state)
			{
				case 0:
					setDCMotor(0, 0);
					setDCMotor(0, 1);
					setServoMotor(50);
					//Yellow
					disable_all();
					enable_red();
					enable_green();
					break;
				case 1:
					//Green
					disable_all();
					enable_green();
					break;
				case 2:
					setDCMotor(0, 0);
					setDCMotor(0, 1);
					setServoMotor(50);
					//Red
					disable_all();
					enable_red();
					break;
			}
			lastState = state;
		}
	}
}
/***********************************************************************
* PURPOSE: Delay the program
*
* INPUTS:
*   int del - The number of milliseconds to wait
* RETURNS:
***********************************************************************/
void delay(int del)
{
	int i;
	for (i=0; i<del*50000; i++){
		// Do nothing
	}
}
/***********************************************************************
* PURPOSE: Run all needed program initialization
*
* INPUTS:
* RETURNS:
***********************************************************************/
void initialize(void)
{
	// Initialize UART
	uart_init();

	//Initialize the DC motors
	InitDCMotors();

	//Initialize Servo
	InitServoMotor();
	
	//Initialize camera
	InitCamera();
	
	//Init Pushbutton
	Button_Init();
	
	//Init Status LEDs
	LED_Init();
}

/*********************************************************************** 
* PURPOSE: Initialize push button pin for input
*
* INPUTS:
* RETURNS:
***********************************************************************/
void Button_Init(void)
{
	// Enable clock for Port C PTC6 button
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	// Configure the Mux for the button
	PORTC_PCR6 = PORT_PCR_MUX(1);

	// Set the push button as an input (0)
	GPIOC_PDDR &= ~(1 << 6);
}

/*********************************************************************** 
* PURPOSE: Turn red portion of RGB LED on
*
* INPUTS:
* RETURNS:
***********************************************************************/
void enable_red(void)
{
	GPIOB_PCOR = (1 << 22);
}

/*********************************************************************** 
* PURPOSE: Turn blue portion of RGB LED on
*
* INPUTS:
* RETURNS:
***********************************************************************/
void enable_blue(void)
{
	GPIOB_PCOR = (1 << 21);
}

/*********************************************************************** 
* PURPOSE: Turn green portion of RGB LED on
*
* INPUTS:
* RETURNS:
***********************************************************************/
void enable_green(void)
{
	GPIOE_PCOR = (1 << 26);
}

/*********************************************************************** 
* PURPOSE: Turn off RGB LED
*
* INPUTS:
* RETURNS:
***********************************************************************/
void disable_all(void)
{
	GPIOB_PSOR = (1UL << 21) | (1UL << 22);
	GPIOE_PSOR = 1UL << 26;
}

/***********************************************************************
* PURPOSE: Initialize LED pins and multiplexers for output
*
* INPUTS:
* RETURNS:
***********************************************************************/
void LED_Init(void)
{
	// Enable clocks on Ports B, C, and E for LED timing
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// Configure the Signal Multiplexer for GPIO
	PORTB_PCR21 = PORT_PCR_MUX(1);
	PORTB_PCR22 = PORT_PCR_MUX(1);
	PORTE_PCR26 = PORT_PCR_MUX(1);
	
	// Switch the GPIO pins to output mode (1)
	GPIOB_PDDR |= (1 << 21) | (1 << 22);
	GPIOE_PDDR |= (1 << 26);

	// Turn off the LEDs (Active  low)
	GPIOB_PSOR |= (1 << 21) | (1 << 22);
	GPIOE_PSOR |= (1 << 26);
}
