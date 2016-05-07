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
* PURPOSE: Get the latest line data
*
* INPUTS:
* RETURNS:
*	uint16_t *retVal - The latest line data, NULL if no new data
***********************************************************************/
uint16_t *GetLineData(void);

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
void init_FTM2(void);

/***********************************************************************
* PURPOSE: Initialize PIT
*
* INPUTS:
* RETURNS:
***********************************************************************/
void init_PIT(void);

/***********************************************************************
* PURPOSE: Initialize GPIO
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

/*****
Buffer new line and blur
*/
void bufferAndBlur(uint16_t *curr_line);

/***********************************************************************
* PURPOSE: Find the line location and set servo and motor based on that
*
* INPUTS:
*		curr_line - The array of values from the camera
* RETURNS:
***********************************************************************/
void findLineLocation(void);
