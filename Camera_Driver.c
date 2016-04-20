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

#define LEFT_BOOKEND		2
#define RIGHT_BOOKEND		125
// Threshold = avg * scalar / divisor
#define THRESHOLD_SCALAR	2
#define THRESHOLD_DIVISOR	3
#define MIN_THRESHOLD 12000
#define TURN_SCALAR 1.2
#define HIST_SIZE

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#define DEFAULT_SYSTEM_CLOCK 20485760u 
// Integration time (seconds)
// Determines how high the camera values are
// Don't exceed 100ms or the caps will saturate
// Must be above 1.25 ms based on camera clk 
//	(camera clk is the mod value set in FTM2)
#define INTEGRATION_TIME .0075f

// Pixel counter for camera logic
// Starts at -2 so that the SI pulse occurs
//		ADC reads start
int pixcnt = -2;

/***********************************************************************
* PURPOSE: The clock value that's toggled by the FTM
***********************************************************************/
int clkval = 0;

/***********************************************************************
* PURPOSE: The latest value being read by the camera
*	line - The buffer currently being used to store new data
*	buffer0, buffer1 - Buffers used to store line data
***********************************************************************/
uint16_t *line;
uint16_t buffer0[128];
uint16_t buffer1[128];

uint16_t rolling_buffer[8][128];
uint16_t blurred_buffer[128];


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
void FTM2_IRQHandler(void){ //For FTM timer
	// Clear interrupt
	FTM2_SC &= ~FTM_SC_TOF_MASK;
	
	// Toggle clk
	GPIOB_PTOR |= (1 << 9); // CLK = !CLK
	
	// Line capture logic
	if ((pixcnt >= 2) && (pixcnt < 256)) {
		if (!clkval) {	// check for falling edge
			// ADC read (note that integer division is 
			//  occurring here for indexing the array)
			line[pixcnt/2] = ADC0VAL;
		}
		pixcnt += 1;
	} else if (pixcnt < 2) {
		if (pixcnt == -1) {
			GPIOB_PSOR |= (1 << 23); // SI = 1
		} else if (pixcnt == 1) {
			GPIOB_PCOR |= (1 << 23); // SI = 0
			// ADC read
			line[0] = ADC0VAL;
		} 
		pixcnt += 1;
	} else {
		GPIOB_PCOR |= (1 << 9); // CLK = 0
		clkval = 0; // make sure clock variable = 0
		pixcnt = -2; // reset counter
		
		findLineLocation(line);
		
		if(line == &buffer0[0])
		{
			line = &buffer1[0];
		}
		else
		{
			line = &buffer0[0];
		}
		
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
void PIT0_IRQHandler(void){
	// Clear interrupt
	PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	
	// Setting mod resets the FTM counter
	//FTM2_MOD = FTM_MOD_MOD_MASK & (DEFAULT_SYSTEM_CLOCK /100000 << FTM_MOD_MOD_SHIFT);
	FTM2_MOD = 100;
	
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
	line = &buffer0[0];
	
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
void init_FTM2(){
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
	FTM2_MOD = 100;
	
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
void init_PIT(void){
	// Setup periodic interrupt timer (PIT)
	
	// Enable clock for timers
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
	
	// Enable timers to continue in debug mode
	PIT_MCR |= PIT_MCR_FRZ_MASK; // In case you need to debug
	PIT_MCR &= ~PIT_MCR_MDIS_MASK;
	
	// PIT clock frequency is the system clock
	// Load the value that the timer will count down from
	PIT_LDVAL0 |= PIT_LDVAL_TSV_MASK & ( DEFAULT_SYSTEM_CLOCK / 100 << PIT_LDVAL_TSV_SHIFT);
	//PIT_LDVAL0 |= PIT_LDVAL_TSV_MASK & ( DEFAULT_SYSTEM_CLOCK / 250 << PIT_LDVAL_TSV_SHIFT);
	
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
void init_GPIO(void){
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
void init_ADC0(void) {
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
*		curr_line - The array of values from the camera
* RETURNS:
***********************************************************************/
void findLineLocation(uint16_t *curr_line)
{
	int i, left_index, right_index, largest_dark_area, curr_area, largest_area_index;
	unsigned int average, threshold;
	
	uint16_t processed_line[128];
	
	average = 0;
	for(i = LEFT_BOOKEND; i<RIGHT_BOOKEND; i++)
	{
		average = average + curr_line[i];
	}
	average = average / (RIGHT_BOOKEND - LEFT_BOOKEND);
	threshold = (average * THRESHOLD_SCALAR) / THRESHOLD_DIVISOR;
	if(threshold < MIN_THRESHOLD){
		return;
	}
	// insert filtering here?
	for(i = LEFT_BOOKEND; i < RIGHT_BOOKEND; i++)
	{
		processed_line[i] = (curr_line[i] > threshold);
	}
	largest_area_index = 0;
	largest_dark_area = 0;
	curr_area = 0;
	i = (LEFT_BOOKEND + RIGHT_BOOKEND) / 2;
	left_index = i;
	right_index = i;
	
	// look for dark spot at center
	if(processed_line[i] == 0)
	{
		//cheat for now, optimize later? Can compute area by indexes wo iterating
		while(processed_line[--i] == 0)
		{
			largest_dark_area++;
			if(i <= LEFT_BOOKEND)
			{
				break;
			}
		}
		left_index = i;
		
		i = right_index;
		
		while(processed_line[++i])
		{
			largest_dark_area++;
			if(i >= RIGHT_BOOKEND)
			{
				break;
			}
		}
		right_index = i;
		
		largest_area_index = (left_index + right_index) / 2;
	}
	
	while(left_index >= LEFT_BOOKEND)
	{
		//can optimize as above
		i = left_index;
		curr_area = 0;
		while(processed_line[--left_index]){
			if(left_index <= LEFT_BOOKEND)
			{
				break;
			}
		}
		while(processed_line[left_index--] == 0){
			curr_area++;
			if(left_index <= LEFT_BOOKEND - 1)
			{
				if(curr_area > largest_dark_area)
				{
					largest_dark_area = curr_area;
					largest_area_index = i - (curr_area / 2);
				}
				break;
			}
		}
	}
	while(right_index <= RIGHT_BOOKEND)
	{
		//can optimize as above
		i = right_index;
		curr_area = 0;
		while(processed_line[++right_index]){
			if(right_index == RIGHT_BOOKEND)
			{
				break;
			}
		}
		while(processed_line[right_index++] == 0){
			curr_area++;
			if(right_index >= RIGHT_BOOKEND + 1)
			{
				if(curr_area > largest_dark_area)
				{
					largest_dark_area = curr_area;
					largest_area_index = i + (curr_area / 2);
				}
				break;
			}
		}
	}
	//set servo
	setServoMotor(TURN_SCALAR*((largest_area_index * 100) / 128));
	uart_putnumU(largest_area_index);
	uart_put(" ");
	uart_putnumU((largest_area_index * 100) / 128);
	
	uart_put("\r\n");
}
