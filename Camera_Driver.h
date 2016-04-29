/***********************************************************************
* Title: 		Car - Camera
* Purpose: 	Drive Camera
* Author: 	Vincent Coffey (vhc1003@rit.edu)
						Alex Avery
* Revised: 	March 30, 2016
* Date:			03/30/2016
***********************************************************************/

/***********************************************************************
* Title: 		Car - Camera
* Purpose: 	Drive Camera
* Author: 	Vincent Coffey (vhc1003@rit.edu)
						Alex Avery
* Revised: 	March 30, 2016
* Date:			03/30/2016
***********************************************************************/

/***********************************************************************
* PURPOSE: ADC0 Conversion Complete ISR
*
* INPUTS:
* RETURNS:
***********************************************************************/
void ADC0_IRQHandler(void);

/***********************************************************************
* PURPOSE: FTM2 ISR, called once every integration period by PIT0
*		Triggers the SI pulse and toggles clock for 128 cycles
*
* INPUTS:
* RETURNS:
***********************************************************************/
void FTM2_IRQHandler(void);

/***********************************************************************
* PURPOSE: Determines the integration period
*		Triggers the clock logic from FTM2 when overlows. 
*
* INPUTS:
* RETURNS:
***********************************************************************/
void PIT0_IRQHandler(void);

/***********************************************************************
* PURPOSE: Initialize all required modules for camera measurement
*
* INPUTS:
* RETURNS:
***********************************************************************/
void InitCamera(void);

/***********************************************************************
* PURPOSE: Initialize FTM2
*
* INPUTS:
* RETURNS:
***********************************************************************/
void init_FTM2();

/***********************************************************************
* PURPOSE: Initialize PIT
*
* INPUTS:
* RETURNS:
***********************************************************************/
void init_PIT(void);

/***********************************************************************
* PURPOSE: Initialize GPIO
*	PTB9 - Camera CLK
*	PTB23 - Camera SI
*	PTB22 - Red LED
*
* INPUTS:
* RETURNS:
***********************************************************************/
void init_GPIO(void);

/***********************************************************************
* PURPOSE: Initialize ADC0
*
* INPUTS:
* RETURNS:
***********************************************************************/
void init_ADC0(void);

/***********************************************************************
* PURPOSE: Find the line location and set servo and motor based on that
*
* INPUTS:
*		uint16_t *curr_line - The array of values from the camera
* RETURNS:
***********************************************************************/
void findLineLocation(uint16_t *curr_line);

/***********************************************************************
* PURPOSE: Convert analog line to binary
*		1 = black, 0 = white
*
* INPUTS:
*		uint16_t *line - The array of values to convert (in position)
* RETURNS:
***********************************************************************/
void crushLine(uint16_t *line);

/***********************************************************************
* PURPOSE: Find the index of the largest white section
*
* INPUTS:
*		uint16_t *line - The array of values to convert (in position)
* RETURNS:
*		int retVal - The index of the largest white area
***********************************************************************/
int findCenter(uint16_t *line);
