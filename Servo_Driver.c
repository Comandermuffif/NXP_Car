/***********************************************************************
* Title: 	Lab06 - Motors
* Purpose: 	Drive Servo Motors
* Author: 	Vincent Coffey (vhc1003@rit.edu)
* Revised: 	March 30, 2016
* Date:		03/30/2016
***********************************************************************/

#include "MK64F12.h"
#include "Servo_Driver.h"

#define CLOCK					20485760u
// Update frequency of Servos
#define Frequency				50
// Minimum and Max turn to avoid stripping servo gears
#define MIN_TURN				13
#define MAX_TURN				87
// Servo center value to account for offset (unused)
#define SERVO_CENTER			50

/***********************************************************************
* PURPOSE: Set the PWM of the servo
*
* INPUTS:		unsigned in dutyCycle - The duty of the servo (0-100)
* RETURNS:
***********************************************************************/
void setServoMotor(unsigned int dutyCycle)
{
	uint16_t mod = (uint16_t) (((CLOCK * (dutyCycle + 100)) /(Frequency * 8)) / 2000);
	
	if(dutyCycle < MIN_TURN)
	{
		mod = (uint16_t) (((CLOCK * (MIN_TURN + 100)) /(Frequency * 8)) / 2000);
	}
	else if(dutyCycle > MAX_TURN)
	{
		mod = (uint16_t) (((CLOCK * (MAX_TURN + 100)) /(Frequency * 8)) / 2000);
	}

	// Set outputs 
	FTM3_C4V = mod;

	// Update the clock to the new frequency
	FTM3_MOD = (CLOCK/Frequency/8);
}
/***********************************************************************
* PURPOSE: Initialize the timer for the servo
*
* INPUTS:
* RETURNS:
***********************************************************************/
void InitServoMotor(void)
{
	//PTC8 FTM3_CH4
	
	// 12.2.13 Enable the Clock to the FTM0 Module
	SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;

	// Enable clock on PORT A so it can output
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;

	// 11.4.1 Route the output of TPM channel 0 to the pins
	// Use drive strength enable flag to high drive strength
	//These port/pins may need to be updated for the K64 <Yes, they do. Here are two that work.>
	PORTC_PCR8  = PORT_PCR_MUX(3)  | PORT_PCR_DSE_MASK; //Ch4

	// 39.3.10 Disable Write Protection
	FTM3_MODE |= FTM_MODE_WPDIS_MASK;

	// 39.3.4 FTM Counter Value
	// Initialize the CNT to 0 before writing to MOD
	FTM3_CNT = 0;

	// 39.3.8 Set the Counter Initial Value to 0
	FTM3_CNTIN = 0;

	// 39.3.5 Set the Modulo resister
	FTM3_MOD = (CLOCK/Frequency/8);

	// 39.3.6 Set the Status and Control of both channels
	// Used to configure mode, edge and level selection
	// See Table 39-67,  Edge-aligned PWM, High-true pulses (clear out on match)
	FTM3_C4SC |= FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM3_C4SC &= ~FTM_CnSC_ELSA_MASK;

	// 39.3.3 FTM Setup
	// Set prescale value to 1 
	// Chose system clock source
	// Timer Overflow Interrupt Enable
	FTM3_SC = FTM_SC_PS(3) | FTM_SC_CLKS(1) | FTM_SC_TOIE_MASK;
}
