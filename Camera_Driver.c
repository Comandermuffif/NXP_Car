/***********************************************************************
* Title: 		Car - Camera
* Purpose: 	Drive Camera
* Author: 	Vincent Coffey (vhc1003@rit.edu)
						Alex Avery
* Revised: 	March 30, 2016
* Date:			03/30/2016
***********************************************************************/

#include <stdlib.h>

#include "MK64F12.h"
#include "uart.h"
#include "Camera_Driver.h"
#include "DC_Driver.h"
#include "Servo_Driver.h"

#define DEFAULT_SYSTEM_CLOCK 20485760u
#define MOD_AMOUNT 50

/* The size of the camera line scan*/
#define ARRAY_SIZE			128

/* The number of history points to keep */
#define ERROR_HISTORY_SIZE	4

/* The left index of the usable camera data */
#define LEFT_BOOKEND		2
/* The right index of the usable camera data */
#define RIGHT_BOOKEND		125

// Threshold = avg * scalar / divisor
#define THRESHOLD_SCALAR	1
#define THRESHOLD_DIVISOR	1
#define MIN_THRESHOLD 4000
#define TURN_SCALAR 3

#define KP	0.9
#define KI	0.1
#define KD	0

/***********************************************************************
* PURPOSE: Pixel counter for storing camera data
*	-2 is set for the SI pulse
***********************************************************************/
int pixcnt = -2;

/***********************************************************************
* PURPOSE: The clock value that's toggled by the FTM
***********************************************************************/
int clkval = 0;

/***********************************************************************
* PURPOSE: The last value sent to the servo
***********************************************************************/
int lastTurn;

/***********************************************************************
* PURPOSE: The latest value being read by the camera
*	buffer - Buffer used to store line data
***********************************************************************/
uint16_t buffer[ARRAY_SIZE];

/***********************************************************************
* PURPOSE: The last differences of the error
*		The error is calculated as the difference between
*		the camera center and the calculated center
*		Positive = Too far to right, Negative = Too far to left
*	errorHistory - Array of camera errors
***********************************************************************/
int errorHistory[ERROR_HISTORY_SIZE];

/***********************************************************************
* PURPOSE: The current ADC value
***********************************************************************/
uint16_t ADC0VAL;

/***********************************************************************
* PURPOSE: ADC0 Conversion Complete ISR
*
* INPUTS:
* RETURNS:
***********************************************************************/
void ADC0_IRQHandler(void)
{
	ADC0VAL = ADC0_RA;		
}

/***********************************************************************
* PURPOSE: FTM2 ISR, called once every integration period by PIT0
*		Triggers the SI pulse and toggles clock for 128 cycles
*
* INPUTS:
* RETURNS:
***********************************************************************/
void FTM2_IRQHandler(void)
{ //For FTM timer
	// Clear interrupt
	FTM2_SC &= ~FTM_SC_TOF_MASK;
	
	// Toggle clk
	GPIOB_PTOR |= (1 << 9); // CLK = !CLK
	
	// Line capture logic
	if ((pixcnt >= 2) && (pixcnt < 2 * ARRAY_SIZE)) {
		if (!clkval) {	// check for falling edge
			// ADC read (note that integer division is 
			//  occurring here for indexing the array)
			buffer[pixcnt/2] = ADC0VAL;
		}
		pixcnt += 1;
	} else if (pixcnt < 2) {
		if (pixcnt == -1) {
			GPIOB_PSOR |= (1 << 23); // SI = 1
		} else if (pixcnt == 1) {
			GPIOB_PCOR |= (1 << 23); // SI = 0
			// ADC read
			buffer[0] = ADC0VAL;
		} 
		pixcnt += 1;
	} else {
		GPIOB_PCOR |= (1 << 9); // CLK = 0
		clkval = 0; // make sure clock variable = 0
		pixcnt = -2; // reset counter
		
		findLineLocation(buffer); //Process the line and set the servo
		
		// Disable FTM2 interrupts (until PIT0 overflows
		//   again and triggers another line capture)
		FTM2_SC &= ~FTM_SC_TOIE_MASK;
	}
	return;
}

/***********************************************************************
* PURPOSE: Determines the integration period
*		Triggers the clock logic from FTM2 when overlows. 
*
* INPUTS:
* RETURNS:
***********************************************************************/
void PIT0_IRQHandler(void)
{
	// Clear interrupt
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	
	// Setting mod resets the FTM counter
	//FTM2_MOD = FTM_MOD_MOD_MASK & (DEFAULT_SYSTEM_CLOCK /100000 << FTM_MOD_MOD_SHIFT);
	FTM2_MOD = MOD_AMOUNT;
	
	// Enable FTM2 interrupts (camera)
	FTM2_SC |= FTM_SC_TOIE_MASK;
	
	return;
}

/***********************************************************************
* PURPOSE: Initialize all required modules for camera measurement
*
* INPUTS:
* RETURNS:
***********************************************************************/
void InitCamera(void)
{
	init_GPIO(); // For CLK and SI output on GPIO
	init_FTM2(); // To generate CLK, SI, and trigger ADC
	init_ADC0();
	init_PIT();	// To trigger camera read based on integration time
}

/***********************************************************************
* PURPOSE: Initialize FTM2
*
* INPUTS:
* RETURNS:
***********************************************************************/
void init_FTM2()
{
	// Enable clock
	SIM_SCGC6 |= SIM_SCGC6_FTM2_MASK;

	// Disable Write Protection
	FTM2_MODE |= FTM_MODE_WPDIS_MASK;
	
	// Set output to '1' on init
	FTM2_OUTINIT |= FTM_OUTINIT_CH0OI_MASK;
	
	// Initialize the CNT to 0 before writing to MOD
	FTM2_CNT = 0x00;
	
	// Set the Counter Initial Value to 0
	FTM2_CNTIN = 0x00;
	
	// Set the period (~10us)
	//FTM2_MOD = FTM_MOD_MOD_MASK & ((DEFAULT_SYSTEM_CLOCK /100000) << FTM_MOD_MOD_SHIFT);
	
	FTM2_MOD = MOD_AMOUNT;
	
	// 50% duty
	//FTM2_C0V = FTM_CnV_VAL_MASK & (((DEFAULT_SYSTEM_CLOCK /100000) << 1) << FTM_CnV_VAL_SHIFT);
	FTM2_C0V = 50;
	
	// Set edge-aligned mode
	FTM2_C0SC |= FTM_CnSC_MSB_MASK;
	
	// Enable High-true pulses
	// ELSB = 1, ELSA = 0
	FTM2_C0SC |= FTM_CnSC_ELSB_MASK;
	FTM2_C0SC &= ~FTM_CnSC_ELSA_MASK;
	
	// Enable hardware trigger from FTM2
	FTM2_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;
	
	// Don't enable interrupts yet (disable)
	FTM2_SC &= ~FTM_SC_TOIE_MASK;
	
	// No prescalar, system clock
	FTM2_SC = FTM_SC_CLKS(0x01) | FTM_SC_PS(0);
	
	// Set up interrupt
	NVIC_EnableIRQ(FTM2_IRQn);
	return;
}

/***********************************************************************
* PURPOSE: Initialize PIT
*
* INPUTS:
* RETURNS:
***********************************************************************/
void init_PIT(void)
{
	// Setup periodic interrupt timer (PIT)
	
	// Enable clock for timers
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	
	// Enable timers to continue in debug mode
	PIT_MCR |= PIT_MCR_FRZ_MASK; // In case you need to debug
	PIT_MCR &= ~PIT_MCR_MDIS_MASK;
	
	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	// TODO: Look into this value
	PIT_LDVAL0 |= PIT_LDVAL_TSV_MASK & ( DEFAULT_SYSTEM_CLOCK / 100 << PIT_LDVAL_TSV_SHIFT);
	
	// Enable timer interrupts
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK;
	
	// Enable the timer
	PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;

	// Clear interrupt flag
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

	// Enable PIT interrupt in the interrupt controller
	NVIC_EnableIRQ(PIT0_IRQn);
	return;
}

/***********************************************************************
* PURPOSE: Initialize GPIO
*	PTB9 - Camera CLK
*	PTB23 - Camera SI
*	PTB22 - Red LED
*
* INPUTS:
* RETURNS:
***********************************************************************/
void init_GPIO(void)
{
	// Enable LED and GPIO so we can see results
	SIM_SCGC5 |= (SIM_SCGC5_PORTB_MASK);
	
	PORTB_PCR9 = PORT_PCR_MUX(1);
	PORTB_PCR22 = PORT_PCR_MUX(1);
	PORTB_PCR23 = PORT_PCR_MUX(1);
	
	GPIOB_PDDR |= (1 << 23) | (1 << 22) | (1 << 9);	
	return;
}

/***********************************************************************
* PURPOSE: Initialize ADC0
*
* INPUTS:
* RETURNS:
***********************************************************************/
void init_ADC0(void)
{
	unsigned int calib;
	// Turn on ADC0
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;

	// Single ended 16 bit conversion, no clock divider
	ADC0_CFG1 |= ADC_CFG1_MODE_MASK;
	//ADC0_CFG1 |= ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE_MASK;

	// Do ADC Calibration for Singled Ended ADC. Do not touch.
	ADC0_SC3 = ADC_SC3_CAL_MASK;
	while ( (ADC0_SC3 & ADC_SC3_CAL_MASK) != 0 );
	calib = ADC0_CLP0; calib += ADC0_CLP1; calib += ADC0_CLP2;
	calib += ADC0_CLP3; calib += ADC0_CLP4; calib += ADC0_CLPS;
	calib = calib >> 1; calib |= 0x8000;
	ADC0_PG = calib;

	// Select hardware trigger.
	ADC0_SC2 |= ADC_SC2_ADTRG_MASK; 

	// Set to single ended mode	
	//ADC0_SC1A = 0x0000;
	//ADC0_SC1A |= ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(0x1);
	ADC0_SC1A = 0x41;

	// Set up FTM2 trigger on ADC0
	// FTM2 select
	SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(0xA);
	// Alternative trigger en.
	SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK;
	// Pretrigger A
	SIM_SOPT7 &= ~SIM_SOPT7_ADC0PRETRGSEL_MASK;


	// Enable NVIC interrupt
	NVIC_EnableIRQ(ADC0_IRQn);
}

/***********************************************************************
* PURPOSE: Find the line location and set servo and motor based on that
*
* INPUTS:
*		uint16_t *curr_line - The array of values from the camera
* RETURNS:
***********************************************************************/
void findLineLocation(uint16_t *curr_line)
{
	int center, error, newTurn, i;
	
	crushLine(curr_line);
	center = findCenter(curr_line);
	//Positive = Too far to right, Negative = Too far to left
	error = (ARRAY_SIZE >> 1) - center;
	newTurn = lastTurn + KP * (error - errorHistory[0])
		+ KI * (error + errorHistory[0])/2
		+ KD * (error - 2 * errorHistory[0] + errorHistory[1]);
	
	setServoMotor(error + 64);
	
	//setServoMotor(newTurn);
	lastTurn = newTurn;
	for(i = ERROR_HISTORY_SIZE - 1; i > 0; i--)
	{
		errorHistory[i] = errorHistory[i - 1];
	}
	errorHistory[0] = error;
}

/***********************************************************************
* PURPOSE: Convert analog line to binary
*		1 = black, 0 = white
*
* INPUTS:
*		uint16_t *line - The array of values to convert (in position)
* RETURNS:
***********************************************************************/
void crushLine(uint16_t *line)
{
	uint16_t average = 0;
	uint16_t threshold = 0;
	int i;
	
	for(i = LEFT_BOOKEND; i <= RIGHT_BOOKEND; i++)
	{
		average += line[i];
	}
	average = average / ((RIGHT_BOOKEND + 1) - LEFT_BOOKEND);
	threshold = (average * THRESHOLD_SCALAR)/THRESHOLD_DIVISOR;
	
	if(threshold < MIN_THRESHOLD)
	{
		threshold = MIN_THRESHOLD;
	}
	
	for(i = 0; i < ARRAY_SIZE; i++)
	{
		// 1 = black, 0 = white
		line[i] = (line[i] < threshold);
	}
}

/***********************************************************************
* PURPOSE: Find the index of the largest white section
*
* INPUTS:
*		uint16_t *line - The array of values to convert (in position)
* RETURNS:
*		int retVal - The index of the largest white area
***********************************************************************/
int findCenter(uint16_t *line)
{
	int retVal = ARRAY_SIZE/2;
	int retValArea = 0;
	
	int tempLeftIndex = -1;
	int tempRightIndex = -1;
	int tempArea = 0;
	int i;
	
	for(i = LEFT_BOOKEND; i <= RIGHT_BOOKEND; i++)
	{
		if(line[i] == 0)
		{
			if(tempLeftIndex == -1)
			{
				tempLeftIndex = i;
			}
			tempArea++;
			tempRightIndex = i;
		}
		else
		{
			if(tempArea > retValArea)
			{
				retVal = (tempLeftIndex + tempRightIndex)/2;
				retValArea = tempArea;
			}
			tempArea = 0;
			tempLeftIndex = -1;
			tempRightIndex = -1;
		}
	}
	if(tempArea > retValArea)
	{
		retVal = (tempLeftIndex + tempRightIndex)/2;
		retValArea = tempArea;
	}
	
	return retVal;
}
