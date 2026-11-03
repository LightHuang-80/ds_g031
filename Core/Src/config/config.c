/*
 * config.c
 *
 *  Created on: 2026年9月20日
 *      Author: Huang
 */
#include <string.h>

#include "eeprom.h"
#include "dynamixel.h"
#include "servo.h"
#include "comm.h"
#include "config.h"

/*
 *
 [control table]
Address	Size	Data Name
0	2	Model Number
2	4	Model Information
6	1	Firmware Version
7	1	ID
8	1	Baud Rate
9	1	Return Delay Time
10	1	Drive Mode
11	1	Operating Mode
12	1	Secondary(Shadow) ID
13	1	Protocol Type
20	4	Homing Offset
24	4	Moving Threshold
31	1	Temperature Limit
32	2	Max Voltage Limit
34	2	Min Voltage Limit
36	2	PWM Limit
38	2	Current Limit
44	4	Velocity Limit
48	4	Max Position Limit
52	4	Min Position Limit
62	1	PWM Slope
63	1	Shutdown
64	1	Torque Enable
65	1	LED
68	1	Status Return Level
69	1	Registered Instruction
70	1	Hardware Error Status
76	2	Velocity I Gain
78	2	Velocity P Gain
80	2	Position D Gain
82	2	Position I Gain
84	2	Position P Gain
88	2	Feedforward 2nd Gain
90	2	Feedforward 1st Gain
98	1	Bus Watchdog
100	2	Goal PWM
102	2	Goal Current
104	4	Goal Velocity
108	4	Profile Acceleration
112	4	Profile Velocity
116	4	Goal Position
120	2	Realtime Tick
122	1	Moving
123	1	Moving Status
124	2	Present PWM
126	2	Present Current
128	4	Present Velocity
132	4	Present Position
136	4	Velocity Trajectory
140	4	Position Trajectory
144	2	Present Input Voltage
146	1	Present Temperature
168	2	Indirect Address 1
224	1	Indirect Data 1
168	2	Indirect Address Write
224	1	Indirect Data Write
180	2	Indirect Address Read
230	1	Indirect Data Read
 */

volatile uint8_t DXL_RAMTable[256] = {
		0xC8, 0x05,      // 0	2	Model Number      #12.0
		0, 1, 0, 1, // 2	4	Model Information #custom
		2, 			// 6	1	Firmware Version  #custom
		1, 			// 7	1	ID
		3, 			// 8	1	Baud Rate
		2, 		// 9	1	Return Delay Time
		0,  // 10	1	Drive Mode
		3,  // 11	1	Operating Mode
		0,  // 12	1	Secondary(Shadow) ID
		2,  // 13	1	Protocol Type
		0, 0, 0, 0, 0, 0, // 14
		0, 0, 0, 0,  // 20	4	Homing Offset
		10, 0, 0, 0,  // 24	4	Moving Threshold
		0, 0, 0,     // 28
		70, // 31	1	Temperature Limit
		60,  0, // 32	2	Max Voltage Limit
		140, 0, // 34	2	Min Voltage Limit
		0x3F, 6, // 36	2	PWM Limit

		0, 0x04,       // 38	2	Current Limit
		0, 0, 0, 0,    // 40

		0, 0x04, 0, 0, // 44	4	Velocity Limit

		0, 0x10, 0, 0, // 48	4	Max Position Limit  # 4096
		0, 0, 0, 0,    // 52	4	Min Position Limit  # 0
		0, 0, 0, 0,
		1, // 60	1	Startup Configuration
		0, // 61
		255, // 62	1	PWM Slope
		0, // 63	1	Shutdown
		1, // 64	1	Torque Enable
		1, // 65	1	LED
		0, 0, // 66
		2, // 68	1	Status Return Level
		0, // 69	1	Registered Instruction
		0, // 70	1	Hardware Error Status
    0,0,0,0,0, // 71
		20, 0, // 76	2	Velocity I Gain
		120, 0, // 78	2	Velocity P Gain

		25,  0, // 80	2	Position D Gain
		0,   0, // 82	2	Position I Gain
		0xB0, 0x04, // 84	2	Position P Gain
	0,0,
		0,  0,  // 88	2	Feedforward 2nd Gain
		5,  0,  // 90	2	Feedforward 1st Gain
	0,0,0,0,0,0, // 91
		1,              // 98	1	Bus Watchdog
	0,
		0x20, 0x03,     // 100	2	Goal PWM
		0, 0, 		// 102	2	Goal Current
		254, 0, 0, 0,   // 104	4	Goal Velocity
		0x00, 0x02, 0, 0, // 108	4	Profile Acceleration
		0x4c, 1, 0, 0, // 112	4	Profile Velocity
		0, 0, 0, 0, // 116	4	Goal Position
		2, 0,       // 120	2	Realtime Tick
		1, // 122	1	Moving
		1, // 123	1	Moving Status
		0, 0, // 124	2	Present PWM
		0, 0, // 126	2	Present Current
		0, 0, 0, 0, // 128	4	Present Velocity
		3, 1, 0, 0, // 132	4	Present Position
		0, 0, 0, 0, // 136	4	Velocity Trajectory
		0, 0, 0, 0, // 140	4	Position Trajectory
		0, 0, // 144	2	Present Input Voltage
		30, // 146	1	Present Temperature
	0,0,0,0,0, // 147
	0,0,0,0,0, // 152
	0,0,0,0,0, // 157
	0,0,0,0,0, // 162
	0,         // 167
		0, 0, // 168	2	Indirect Address 1 / 2	Indirect Address Write
	0,0,0,0,0, // 170
	0,0,0,0,0, // 175
		0, 0, // 180	2	Indirect Address Read
	0,0,0,     // 182
	0,0,0,0,0, // 185
	0,0,0,0,0, // 190
	0,0,0,0,0, // 195
	0,0,0,0,0, // 200
	0,0,0,0,0, // 205
	0,0,0,0,0, // 210
	0,0,0,0,0, // 215
	0,0,0,0,   // 220

		0, // 224	1	Indirect Data 1 / Indirect Data Write

	0,0,0,0,0, // 225

		0// 230	1	Indirect Data Read
};

/*EEPROM storage index*/
/*
 * // EEPROM AREA
#define ID_ADDR                  3
#define BAUD_RATE_ADDR           4
#define RETURN_DELAY_TIME_ADDR   5
#define CW_ANGLE_LIMIT_ADDR      6
#define CCW_ANGLE_LIMIT_ADDR     8
#define TEMP_LIMIT_ADDR         11
#define MIN_VOLTAGE_LIMIT_ADDR  12
#define MAX_VOLTAGE_LIMIT_ADDR  13
#define MAX_TORQUE_ADDR         14
#define STATUS_RET_LEVEL_ADDR   16
#define ALARM_LED_ADDR          17
#define SHUTDOWN_ADDR           18
*/

EEPROMTable_Idx_t DXL_EEPROMTable_Idxs[ROM_PARAMS_NUM] = {
		{0, 7, 1}, // 7	1	ID
		{1, 8, 1}, // 8	1	Baud Rate
		{2, 9, 1}, // 9	1	Return Delay Time
		{3, 52, 4}, // 52	4	Min Position Limit
		{4, 48, 4}, // 48	4	Max Position Limit
		{5, 31, 1}, // 31	1	Temperature Limit
		{6, 34, 2}, // 34	2	Min Voltage Limit
		{7, 32, 2}, // 32	2	Max Voltage Limit
		{8, 38, 2}, // 38	2	Current Limit
		{9, 68, 2}, // 68	1	Status Return Level
		{10, 65, 1}, // 65	1	LED
		{11, 63, 1}, // 63	1	Shutdown
		{12, 76, 2}, // 76	2	Velocity I Gain
		{13, 78, 2}, // 78	2	Velocity P Gain
		//{14, 80, 2}, // 80	2	Position D Gain
		//{15, 82, 2}, // 82	2	Position I Gain
		//{16, 84, 2}, // 84	2	Position P Gain
		{17, 88, 2}, // 88	2	Feedforward 2nd Gain
		{18, 90, 2}, // 90	2	Feedforward 1st Gain
		{19, 108, 4}, // 108	4	Profile Acceleration
		{20, 112, 4},  // 112	4	Profile Velocity
		{21, 20, 4},  // 20	4	Homing Offset
		{22, 10, 1},  // 10 1   Drive mode
		{23, 11, 1}   // 11 1   Operation mode
};

/*
RAMTable_Idx_t RAMTable_Idxs[RAM_PARAMS_NUM] = {
		{0, 0xFF, 2},  // 0	2	Model Number
		{2, 0xFF, 4},  // 2	4	Model Information
		{6, 0xFF, 1},  // 6	1	Firmware Version
		{7, 0, 1},  // 7	1	ID
		{8, 1, 1},  // 8	1	Baud Rate
		{9, 2, 1},  // 9	1	Return Delay Time
		{10, 0xFF, 1},  // 10	1	Drive Mode
		{11, 0xFF, 1},  // 11	1	Operating Mode
		{12, 0xFF, 1},  // 12	1	Secondary(Shadow) ID
		{13, 0xFF, 1},  // 13	1	Protocol Type
		{20, 0xFF, 4},  // 20	4	Homing Offset
		{24, 0xFF, 4},  // 24	4	Moving Threshold
		{31, 5, 1},  // 31	1	Temperature Limit
		{32, 7, 2},  // 32	2	Max Voltage Limit
		{34, 6, 2},  // 34	2	Min Voltage Limit
		{36, 0xFF, 2},  // 36	2	PWM Limit
		{38, 8, 2},  // 38	2	Current Limit
		{44, 0xFF, 4},  // 44	4	Velocity Limit
		{48, 4, 4},  // 48	4	Max Position Limit
		{52, 3, 4},  // 52	4	Min Position Limit
		{62, 0xFF, 1},  // 62	1	PWM Slope
		{63, 11, 1},  // 63	1	Shutdown
		{64, 0xFF, 1},  // 64	1	Torque Enable
		{65, 10, 1},  // 65	1	LED
		{68, 9, 1},  // 68	1	Status Return Level
		{69, 0xFF, 1},  // 69	1	Registered Instruction
		{70, 0xFF, 1},  // 70	1	Hardware Error Status
		{76, 12, 2},  // 76	2	Velocity I Gain
		{78, 13, 2},  // 78	2	Velocity P Gain
		{80, 14, 2},  // 80	2	Position D Gain
		{82, 15, 2},  // 82	2	Position I Gain
		{84, 16, 2},  // 84	2	Position P Gain
		{88, 17, 2},  // 88	2	Feedforward 2nd Gain
		{90, 18, 2},  // 90	2	Feedforward 1st Gain
		{98, 0xFF, 1},  // 98	1	Bus Watchdog
		{100, 0xFF, 2},  // 100	2	Goal PWM
		{102, 0xFF, 2},  // 102	2	Goal Current
		{104, 0xFF, 4},  // 104	4	Goal Velocity
		{108, 19, 4},  // 108	4	Profile Acceleration
		{112, 20, 4},  // 112	4	Profile Velocity
		{116, 0xFF, 4},  // 116	4	Goal Position
		{120, 0xFF, 2},  // 120	2	Realtime Tick
		{122, 0xFF, 1},  // 122	1	Moving
		{123, 0xFF, 1},  // 123	1	Moving Status
		{124, 0xFF, 2},  // 124	2	Present PWM
		{126, 0xFF, 2},  // 126	2	Present Current
		{128, 0xFF, 4},  // 128	4	Present Velocity
		{132, 0xFF, 4},  // 132	4	Present Position
		{136, 0xFF, 4},  // 136	4	Velocity Trajectory
		{140, 0xFF, 4},  // 140	4	Position Trajectory
		{144, 0xFF, 2},  // 144	2	Present Input Voltage
		{146, 0xFF, 1},  // 146	1	Present Temperature
		{168, 0xFF, 2},  // 168	2	Indirect Address 1
		{224, 0xFF, 1},  // 224	1	Indirect Data 1
		{168, 0xFF, 2},  // 168	2	Indirect Address Write
		{224, 0xFF, 1},  // 224	1	Indirect Data Write
		{180, 0xFF, 2},  // 180	2	Indirect Address Read
		{230, 0xFF, 1}   // 230	1	Indirect Data Read
};
*/
uint16_t  VirtAddVarTab[ROM_PARAMS_NUM];
uint16_t  NumbOfVar;

/*
uint16_t Config_UpdateRomTable(uint8_t romidx, uint8_t* data, uint8_t len)
{
	uint64_t romdata = 0;
	uint16_t status = HAL_OK;

	if (romidx >= 0 && romidx < ROM_PARAMS_NUM){
		memcpy((void *)&romdata, (const void *)data, len);
		status = EE_WriteVariable(romidx, romdata);
	}

	return status;
}

void Config_UpdateRamTable(uint8_t addr, uint8_t* data, uint8_t len)
{
	uint8_t idx = 0;

	while (idx < RAM_PARAMS_NUM){
		if (addr == RAMTable_Idxs[idx].ram_addr && len <= RAMTable_Idxs[idx].len){
			// ram ane rom table update

			uint8_t romidx = 0xFF;
			if (RAMTable_Idxs[idx].romidx != 0xFF){
				romidx = RAMTable_Idxs[idx].romidx;
				uint16_t status = Config_UpdateRomTable(romidx, data, len);
				if (status != HAL_OK){
					// dont update the ramtable
					break;
				}
			}

			// update ram table
			memcpy((void*)&(DXL_RAMTable[addr]), (const void *)data, len);
		}

		idx ++;
	}
}
*/

void Config_BuildRamTable()
{

	// Load data from rom and update ram table
	uint8_t idx = 0;

	while (idx < ROM_PARAMS_NUM){
		uint16_t addr = (uint16_t)DXL_EEPROMTable_Idxs[idx].idx;
		uint64_t data;
		uint16_t status = EE_ReadVariable(addr, &data);

		if (status == HAL_OK){
			// update ram table
			uint8_t offset = DXL_EEPROMTable_Idxs[idx].ram_addr;
			uint8_t* data_ptr = (uint8_t*)&data;
			for (uint8_t i = 0; i < DXL_EEPROMTable_Idxs[idx].len; i++){
				DXL_RAMTable[offset + i] = data_ptr[i];
			}
		}
		idx ++;
	}

}

void Config_Init()
{
	NumbOfVar = ROM_PARAMS_NUM;
	for(uint8_t idx = 0; idx < NumbOfVar; idx++){
		VirtAddVarTab[idx] = idx;
	}

	HAL_FLASH_Unlock();
	EE_Init();
	HAL_FLASH_Lock();

	Config_BuildRamTable();
}

void Config_UpdateRuntime_Params()
{
	port_state.servo_id = DXL_RAMTable[ID_ADDR];

	port_state.bus_state = BUS_INIT;
	port_state.rx_packet_state = PKT_NONE;

	// Status Return Level
	port_state.resp_level = DXL_RAMTable[68];
	port_state.return_delay_time = DXL_RAMTable[9];

	servo_rt.drive_mode = DXL_RAMTable[DRIVE_MODE_ADDR];

	// Update the home offset
	servo_rt.home_offset = ControlTable_ReadInt32(20);
	if (servo_rt.home_offset >= 2048 || servo_rt.home_offset <= -2048){
		servo_rt.home_offset = 0;
	}

	// check the startup option
	uint8_t startup = DXL_RAMTable[STARTUP_ADDR];
	DXL_RAMTable[TORQUE_ENABLE_ADDR] = startup & 0x01;

	/* Servo min & max position */
	servo_rt.min_pos = ControlTable_ReadInt32(MIN_POSITION_LIMIT_ADDR);
	servo_rt.max_pos = ControlTable_ReadInt32(MAX_POSITION_LIMIT_ADDR);
}

uint32_t ControlTable_ReadUInt32(uint16_t address)
{
    return ((uint32_t)(DXL_RAMTable[address] & 0x000000FF))       |
           ((uint32_t)(DXL_RAMTable[address + 1] & 0x000000FF) << 8)  |
           ((uint32_t)(DXL_RAMTable[address + 2] & 0x000000FF) << 16) |
           ((uint32_t)(DXL_RAMTable[address + 3] & 0x000000FF) << 24);
}

int32_t ControlTable_ReadInt32(uint16_t address)
{
    return ((int32_t)(DXL_RAMTable[address] & 0x000000FF))       |
           ((int32_t)(DXL_RAMTable[address + 1] & 0x000000FF) << 8)  |
           ((int32_t)(DXL_RAMTable[address + 2] & 0x000000FF) << 16) |
           ((int32_t)(DXL_RAMTable[address + 3] & 0x000000FF) << 24);
}

void ControlTable_WriteInt32(int32_t value, uint16_t address)
{
	int16_t lv = (int16_t)(value & 0x0000FFFF);
	int16_t hv = (int16_t)((value >> 16) & 0x0000FFFF);

	// Update the DXL_RAMTable
	DXL_RAMTable[address] =  (uint8_t)(((uint16_t)(lv) >> 0) & 0xFF);
	DXL_RAMTable[address+1] = (uint8_t)(((uint16_t)(lv) >> 8) & 0xFF);

	DXL_RAMTable[address+2] = (uint8_t)(((uint16_t)(hv) >> 0) & 0xFF);
	DXL_RAMTable[address+3] = (uint8_t)(((uint16_t)(hv) >> 8) & 0xFF);
}

void ControlTable_WriteUInt32(uint32_t value, uint16_t address)
{
	uint16_t lv = (uint16_t)(value & 0x0000FFFF);
	uint16_t hv = (uint16_t)((value >> 16) & 0x0000FFFF);

	// Update the DXL_RAMTable
	DXL_RAMTable[address] = (uint8_t)(((uint16_t)(lv) >> 0) & 0xFF);
	DXL_RAMTable[address+1] =(uint8_t)(((uint16_t)(lv) >> 8) & 0xFF);

	DXL_RAMTable[address+2] = (uint8_t)(((uint16_t)(hv) >> 0) & 0xFF);
	DXL_RAMTable[address+3] = (uint8_t)(((uint16_t)(hv) >> 8) & 0xFF);
}

int16_t  ControlTable_ReadInt16(uint16_t address)
{
	return ((int16_t)(DXL_RAMTable[address] & 0x00FF ))       |
	       ((int16_t)(DXL_RAMTable[address + 1] & 0x00FF) << 8);

}
void ControlTable_WriteInt16(int16_t value, uint16_t address)
{
	// Update the DXL_RAMTable
	DXL_RAMTable[address] = (uint8_t)(((int16_t)(value) >> 0) & 0xFF);
	DXL_RAMTable[address+1] =(uint8_t)(((int16_t)(value) >> 8) & 0xFF);
}


uint16_t  ControlTable_ReadUInt16(uint16_t address)
{
	return ((uint16_t)(DXL_RAMTable[address] & 0x00FF))   |
	       ((uint16_t)(DXL_RAMTable[address + 1] & 0x00FF) << 8);

}
void ControlTable_WriteUInt16(uint16_t value, uint16_t address)
{
	// Update the DXL_RAMTable
	DXL_RAMTable[address] = (uint8_t)(((uint16_t)(value) >> 0) & 0xFF);
	DXL_RAMTable[address+1] =(uint8_t)(((uint16_t)(value) >> 8) & 0xFF);
}
