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

void initialize(void);
void delay(int del);

/***********************************************************************
* PURPOSE: Main entry point for the program
*
* INPUTS:
* TURNS:
*   int retVal - The exit code of the program
***********************************************************************/
int main(void)
{	
	int i;
	initialize();
	setDCMotor(38, 1);
	setDCMotor(38, 0);
	setServoMotor(50);
	// Print welcome over serial
	uart_put("Running... \n\r");
	
	for(;;)
	{
	}
}
/***********************************************************************
* PURPOSE: Delay the program
*
* INPUTS:
*   int del - The number of milliseconds to wait
* RETURNS:
***********************************************************************/
void delay(int del){
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
void initialize()
{
	// Initialize UART
	uart_init();

	//Initialize the DC motors
	InitDCMotors();

	//Initialize Servo
	InitServoMotor();
	
	//Initialize camera
	InitCamera();
}
