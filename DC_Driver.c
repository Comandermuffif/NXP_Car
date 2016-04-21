/***********************************************************************
* Title: 	Lab06 - Motors
* Purpose: 	Drive DC Motors
* Author: 	Vincent Coffey (vhc1003@rit.edu)
* Revised: 	March 30, 2016
* Date:		03/30/2016
***********************************************************************/

#include "MK64F12.h"

/*From clock setup 0 in system_MK64f12.c*/
#define SYSTEM_CLOCK			20485760u
#define MOTOR_OFFSET_0			0.0
#define MOTOR_OFFSET_1			0.0
#define MOTOR_FREQUNECY			10000

/*
	PortC3 - Motor0 Drive 
	PortC4 - Motor1 Drive
*/

/***********************************************************************
* PURPOSE: Set the PWM of the drive motors
*
* INPUTS:		unsigned in dutyCycle - The duty cycle of the motors (0-100)
*						int motorSelect - 0 = Motor0, Others = Motor1
* RETURNS:
***********************************************************************/
void setDCMotor(unsigned int dutyCycle, int motorSelect)
{
	if(motorSelect == 0)
	{
		FTM0_C3V = (uint16_t) (((SYSTEM_CLOCK/MOTOR_FREQUNECY) * dutyCycle) / 100);
	}
	else
	{
		FTM0_C2V = (uint16_t) (((SYSTEM_CLOCK/MOTOR_FREQUNECY) * dutyCycle) / 100);
	}
	FTM0_MOD = (SYSTEM_CLOCK/MOTOR_FREQUNECY);
}

/***********************************************************************
* PURPOSE: Initialize the FlexTimer for DC Motors
*
* INPUTS:
* RETURNS:
***********************************************************************/
void InitDCMotors(void)
{
	// 12.2.13 Enable the Clock to the FTM0 Module
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;

	// Enable clock on PORT A so it can output
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;

	// 11.4.1 Route the output of TPM channel 0 to the pins
	// Use drive strength enable flag to high drive strength
	//These port/pins may need to be updated for the K64 <Yes, they do. Here are two that work.>
	PORTC_PCR3  = PORT_PCR_MUX(4)  | PORT_PCR_DSE_MASK; //Ch2
	PORTC_PCR4  = PORT_PCR_MUX(4)  | PORT_PCR_DSE_MASK;//Ch3

	// 39.3.10 Disable Write Protection
	FTM0_MODE |= FTM_MODE_WPDIS_MASK;

	// 39.3.4 FTM Counter Value
	// Initialize the CNT to 0 before writing to MOD
	FTM0_CNT = 0;

	// 39.3.8 Set the Counter Initial Value to 0
	FTM0_CNTIN = 0;

	// 39.3.5 Set the Modulo resister
	FTM0_MOD = (SYSTEM_CLOCK/MOTOR_FREQUNECY);

	// 39.3.6 Set the Status and Control of both channels
	// Used to configure mode, edge and level selection
	// See Table 39-67,  Edge-aligned PWM, High-true pulses (clear out on match)
	FTM0_C3SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C3SC &= ~FTM_CnSC_ELSA_MASK;

	// See Table 39-67,  Edge-aligned PWM, Low-true pulses (clear out on match)
	FTM0_C2SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM0_C2SC &= ~FTM_CnSC_ELSA_MASK;

	// 39.3.3 FTM Setup
	// Set prescale value to 1 
	// Chose system clock source
	// Timer Overflow Interrupt Enable
	FTM0_SC = FTM_SC_PS(0) | FTM_SC_CLKS(1) | FTM_SC_TOIE_MASK;
	
}
