/*
 * config.h
 *
 *  Created on: 2026年9月20日
 *      Author: Huang
 */

#ifndef SRC_CONFIG_CONFIG_H_
#define SRC_CONFIG_CONFIG_H_

#include <stdint.h>

typedef struct EEPROMTable_Idx_ {
	uint8_t idx;
	uint8_t ram_addr;
	uint8_t len;
}EEPROMTable_Idx_t;

typedef struct RAMTable_Idx_ {
	uint8_t ram_addr;
	uint8_t romidx;
	uint8_t len;
}RAMTable_Idx_t;

extern volatile uint8_t DXL_RAMTable[256];

#define RAM_PARAMS_NUM  58
#define ROM_PARAMS_NUM	21

extern EEPROMTable_Idx_t DXL_EEPROMTable_Idxs[ROM_PARAMS_NUM];
extern RAMTable_Idx_t DXL_RAMTable_Idxs[RAM_PARAMS_NUM];

void Config_Init();
void Config_UpdateRuntime_Params();

uint32_t ControlTable_ReadUInt32(uint16_t address);
int32_t  ControlTable_ReadInt32(uint16_t address);

void ControlTable_WriteInt32(int32_t value, uint16_t address);
void ControlTable_WriteUInt32(uint32_t value, uint16_t address);

int16_t  ControlTable_ReadInt16(uint16_t address);
void ControlTable_WriteInt16(int16_t value, uint16_t address);

uint16_t  ControlTable_ReadUInt16(uint16_t address);
void ControlTable_WriteUInt16(uint16_t value, uint16_t address);

#endif /* SRC_CONFIG_CONFIG_H_ */
