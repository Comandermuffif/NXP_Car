/***********************************************************************
* Title: 	NXP Cup Servo Driver
* Purpose: 	Drive Servo Motors
* Author: 	Vincent Coffey (vhc1003@rit.edu)
* Revised: 	March 30, 2016
* Date:		03/30/2016
***********************************************************************/

/***********************************************************************
* PURPOSE: Set the PWM of the servo
*
* INPUTS:		unsigned in dutyCycle - The duty of the servo (0-100)
* RETURNS:
***********************************************************************/
void setServoMotor(unsigned int dutyCycle);
/***********************************************************************
* PURPOSE: Initialize the timer for the servo
*
* INPUTS:
* RETURNS:
***********************************************************************/
void InitServoMotor(void);
/***********************************************************************
* PURPOSE: Interupt handler for the servo timer
*
* INPUTS:
* RETURNS:
***********************************************************************/
void FTM2_IRQHandler(void);
