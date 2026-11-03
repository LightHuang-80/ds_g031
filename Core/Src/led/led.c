/*
 * led.c
 *
 *  Created on: 2026年9月18日
 *      Author: Huang
 */
#include <stdio.h>
#include "main.h"
#include "led.h"
#include "dynamixel.h"
#include "config.h"

uint8_t  led_flag = 0;

void LED_R_on()
{
	HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_RESET);
}

void LED_R_off()
{
	HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
}

void LED_G_on()
{
	HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_RESET);
}

void LED_G_off()
{
	HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_SET);
}

void LED_B_on()
{
	HAL_GPIO_WritePin(B_LED_GPIO_Port, B_LED_Pin, GPIO_PIN_RESET);
}

void LED_B_off()
{
	HAL_GPIO_WritePin(B_LED_GPIO_Port, B_LED_Pin, GPIO_PIN_SET);
}

void LED_off()
{
	LED_R_off();
	LED_G_off();
	LED_B_off();
}

void LED_process(uint8_t led)
{
	static uint32_t led_cnt = 0;
	uint8_t flag = 0;

	if (DXL_RAMTable[LED_ADDR]){
		uint32_t tick = HAL_GetTick();

	    if (tick - led_cnt >= 20){
	    	led_cnt = tick;
			LED_off();

	    	if (++flag >= 3){
	    		led_flag = 0;
	    	}else{
	    		if (led == 1){
	    			LED_B_on();
	    		}else{
	    			LED_G_on();
	    		}
	    	}
	    }
	}else{
	   LED_off();
	}
}
