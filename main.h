/*
 * Main Method for testing the PWM Code for the K64F
 * PWM signal can be connected to output pins are PC3 and PC4
 * 
 * Author:  
 * Created:  
 * Modified:  
 */

int state;

/***********************************************************************
* PURPOSE: Main entry point for the program
*
* INPUTS:
* TURNS:
*   int retVal - The exit code of the program
***********************************************************************/
int main(void);

/***********************************************************************
* PURPOSE: Delay the program
*
* INPUTS:
*   int del - The number of milliseconds to wait
* RETURNS:
***********************************************************************/
void delay(int del);

/***********************************************************************
* PURPOSE: Run all needed program initialization
*
* INPUTS:
* RETURNS:
***********************************************************************/
void initialize(void);

/*********************************************************************** 
* PURPOSE: Initialize push button pin for input
*
* INPUTS:
* RETURNS:
***********************************************************************/
void Button_Init(void);

/*********************************************************************** 
* PURPOSE: Turn red portion of RGB LED on
*
* INPUTS:
* RETURNS:
***********************************************************************/
void enable_red(void);

/*********************************************************************** 
* PURPOSE: Turn blue portion of RGB LED on
*
* INPUTS:
* RETURNS:
***********************************************************************/
void enable_blue(void);

/*********************************************************************** 
* PURPOSE: Turn green portion of RGB LED on
*
* INPUTS:
* RETURNS:
***********************************************************************/
void enable_green(void);

/*********************************************************************** 
* PURPOSE: Turn off RGB LED
*
* INPUTS:
* RETURNS:
***********************************************************************/
void disable_all(void);

/***********************************************************************
* PURPOSE: Initialize LED pins and multiplexers for output
*
* INPUTS:
* RETURNS:
***********************************************************************/
void LED_Init(void);
