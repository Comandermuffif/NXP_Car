/***********************************************************************
* Title: 	Lab06 - Motors
* Purpose: 	Drive DC Motors
* Author: 	Vincent Coffey (vhc1003@rit.edu)
* Revised: 	March 30, 2016
* Date:		03/30/2016
***********************************************************************/

/***********************************************************************
* PURPOSE: Set the PWM of the drive motors
*
* INPUTS:		unsigned in DutyCycle - The duty cycle of the motors (0-100)
*						int motorSelect - 0 = Motor0, Others = Motor1
* RETURNS:
***********************************************************************/
void setDCMotor(unsigned int DutyCycle, int motorSelect);

/***********************************************************************
* PURPOSE: Initialize the FlexTimer for DC Motors
*
* INPUTS:
* RETURNS:
***********************************************************************/
void InitDCMotors(void);

/***********************************************************************
* PURPOSE: Interupt handler for the servo timer
*
* INPUTS:
* RETURNS:
***********************************************************************/
void FTM0_IRQHandler(void);
