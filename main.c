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

#define LEFT_BOOKEND		0
#define RIGHT_BOOKEND		127
// Threshold = avg * scalar / divisor
#define THRESHOLD_SCALAR	1
#define THRESHOLD_DIVISOR	2

uint16_t *curr_line;

void initialize(void);

/***********************************************************************
* PURPOSE: Main entry point for the program
*
* INPUTS:
* RETURNS:
*   int retVal - The exit code of the program
***********************************************************************/
int main(void)
{
	int i, left_index, right_index, largest_dark_area, curr_area, largest_area_index;
	unsigned int average, threshold;
	
	uint16_t processed_line[128];
	
	initialize();
	
	// Print welcome over serial
	uart_put("Running... \n\r");
	  
	for(;;){
		curr_line = GetLineData();
		if(curr_line == NULL)
		{
			continue;
		}
		average = 0;
		for(i = LEFT_BOOKEND; i<RIGHT_BOOKEND; i++)
		{
			average = average + curr_line[i];
		}
		average = average / (RIGHT_BOOKEND - LEFT_BOOKEND);
		threshold = (average * THRESHOLD_SCALAR) / THRESHOLD_DIVISOR;
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
		setServoMotor((largest_area_index * 100) / 128);
	}
	return 0;
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

	InitCamera();
}
