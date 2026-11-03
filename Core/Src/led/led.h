/*
 * led.h
 *
 *  Created on: 2026年9月18日
 *      Author: Huang
 */

#ifndef SRC_LED_LED_H_
#define SRC_LED_LED_H_

void LED_R_on();
void LED_R_off();
void LED_G_on();
void LED_G_off();
void LED_B_on();
void LED_B_off();

void LED_off();

void LED_process(uint8_t led);

#endif /* SRC_LED_LED_H_ */
