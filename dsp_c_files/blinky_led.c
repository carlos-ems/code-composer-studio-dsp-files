/*
 * blinky_led.c
 *
 *  Created on: Feb. 10, 2023
 *      Author: Carlos
 */

#include "F28x_Project.h"

#define BLINKY_LED_GPIO1 31

void main(void)
{
	InitSysCtrl();
	InitGpio();
	GPIO_SetupPinMux(BLINKY_LED_GPIO1, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(BLINKY_LED_GPIO1, GPIO_OUTPUT, GPIO_PUSHPULL);

	while(1){

	    // BLINK LED 31

	    // Turn on LED
	    GPIO_WritePin(BLINKY_LED_GPIO1, 0);
	    DELAY_US(1000*500);

	    // Turn off LED
	    GPIO_WritePin(BLINKY_LED_GPIO1, 1);
	    DELAY_US(1000*500);
	}
}
