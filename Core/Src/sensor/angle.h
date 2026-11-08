/*
 * angle.h
 *
 *  Created on: 2026年10月7日
 *      Author: Huang
 */

#ifndef SRC_SENSOR_ANGLE_H_
#define SRC_SENSOR_ANGLE_H_

#include "as5600.h"

void ANGLE_Init();
as5600_error_t ANGLE_getRawAngle(uint16_t *raw_angle);
void ANGLE_Process();
void ANGLE_StartRead(void);

#endif /* SRC_SENSOR_ANGLE_H_ */
