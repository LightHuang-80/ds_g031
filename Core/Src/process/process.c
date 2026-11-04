/*
 * process.c
 *
 *  Created on: 2026年9月21日
 *      Author: Huang
 */
#include "stm32g0xx_it.h"
#include "config.h"
#include "motor_control.h"
#include "dynamixel.h"
#include "motion.h"
#include "eeprom.h"
#include "comm.h"
#include "servo.h"
#include "process.h"

void Proc_action(uint16_t action_reg)
{
#ifndef DEBUG
	switch(action_reg){
	case GOAL_POSITION_ADDR:{
		// go position

	}break;
	case GOAL_VELOCITY_ADDR:{
		// go velocity

	}break;
	case GOAL_CURRENT_ADDR:{
		// go current torque

	}break;
	default:
		break;
	}
#endif
}

void Proc_addr_data(uint16_t addr, uint8_t* data_ptr, uint16_t data_len)
{

	switch(addr){

	case 9:{
        port_state.return_delay_time = DXL_RAMTable[9];
	}break;

	case 10:{ // drive_mode
		servo_rt.drive_mode = DXL_RAMTable[10];
		/*update the gpio dir*/
	    if (servo_rt.drive_mode){
		  HAL_GPIO_WritePin(ANGLE_DIR_GPIO_Port, ANGLE_DIR_Pin, GPIO_PIN_RESET);
	    }else{
		  HAL_GPIO_WritePin(ANGLE_DIR_GPIO_Port, ANGLE_DIR_Pin, GPIO_PIN_SET);
	    }
	}break;

	case 20:{ // Home offset
		servo_rt.home_offset = ControlTable_ReadInt32(20);
		if (servo_rt.home_offset >= 2048 || servo_rt.home_offset <= -2048)
			servo_rt.home_offset = 0;

	}break;

	case 48:{
		Servo_Position_limit_Update();
	}break;
	case 52:{
		Servo_Position_limit_Update();
	}break;

	case 68:{
		// 1Byte Status Return Level
		port_state.resp_level = DXL_RAMTable[68];
	}break;

	case 76: // 76	2	Velocity I Gain
	case 78: // 78	2	Velocity P Gain
	{
		Motion_Vel_PI_Update();
	}break;
	case 80:
	case 82:
	case 84:
	{
		Motion_Pos_PID_Update();
		// Ram update only
		return;
	}
	case 88:
	case 90:
	case 108:
	case 112:{
		Motion_Profile_Update();
		return;
	}break;

	case 116: {
		// 116	4	Goal Position
		Motion_goPosition();

		// RAM update only
		return;
	}break;

	case TORQUE_ENABLE_ADDR:{
		// Torque
		uint8_t torque = (uint8_t)DXL_RAMTable[TORQUE_ENABLE_ADDR];
		if (!torque){
			Motor_Output_Disable();
		}
	}break;

	default:
		break;
	}

#ifndef DEBUG
	//48	4	Max Position Limit
	//52	4	Min Position Limit

	// 76	2	Velocity I Gain
	// 78	2	Velocity P Gain

	// 80	2	Position D Gain
	// 82	2	Position I Gain
	// 84	2	Position P Gain
	// 88	2	Feedforward 2nd Gain
	// 90	2	Feedforward 1st Gain

	// 64	1	Torque Enable
	uint8_t  update_rom = 0;
	uint16_t rom_addr = 0;
	uint64_t rom_data = 0;

	for (uint16_t ridx = 0; ridx < ROM_PARAMS_NUM; ridx++){
		if (DXL_EEPROMTable_Idxs[ridx].ram_addr == addr){
			update_rom = 1;
			rom_addr = DXL_EEPROMTable_Idxs[ridx].idx;
			if (data_len == 1)      {rom_data = data_ptr[0];}
			else if (data_len == 2) {
				rom_data = data_ptr[1];
				rom_data <<= 8; rom_data &= 0xFF00;
				rom_data += data_ptr[0];
			}
			else if (data_len == 4) {
				rom_data = data_ptr[3];
				rom_data <<= 8; rom_data &= 0xFF00;
				rom_data += data_ptr[2];
				rom_data <<= 8; rom_data &= 0xFFFF00;
				rom_data += data_ptr[1];
				rom_data <<= 8; rom_data &= 0xFFFFFF00;
				rom_data += data_ptr[0];
			}else{
				update_rom = 0;
			}
			break;
		}
	}

	/* Update the ROM with rom_data*/
	if (update_rom == 1){
		EE_WriteVariable(rom_addr, rom_data);
	}

	if (addr == ID_ADDR || addr == BAUDRATE_ADDR) {
		HAL_Delay(2);
		system_reboot();
	}

#endif
}

void Proc_control_table_Update(uint8_t* packet, uint16_t pkt_len)
{

	uint16_t addr = packet[9]; addr <<= 8; addr &= 0xFF00;
	addr +=  packet[8] & 0x00FF;

	uint16_t param_len = packet[6]; param_len <<= 8; param_len &= 0xFF00;
	param_len += packet[5] & 0x00FF;

	/* param length = 减去3 ， 再减去2 bytes的param address*/
	param_len -= 5;

	Proc_addr_data(addr, (uint8_t*)(&packet[10]), param_len);
#ifndef DEBUG
#endif
}
