/*
 * process.h
 *
 *  Created on: 2026年9月21日
 *      Author: Huang
 */

#ifndef SRC_PROCESS_PROCESS_H_
#define SRC_PROCESS_PROCESS_H_

void Proc_action(uint16_t action_reg);
void Proc_control_table_Update(uint8_t* packet, uint16_t pkt_len);
void Proc_addr_data(uint16_t addr, uint8_t* data_ptr, uint16_t data_len);
#endif /* SRC_PROCESS_PROCESS_H_ */
