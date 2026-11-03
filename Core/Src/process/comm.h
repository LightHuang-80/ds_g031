/*
 * comm.h
 *
 *  Created on: 2026年9月19日
 *      Author: Huang
 */

#ifndef SRC_PROCESS_COMM_H_
#define SRC_PROCESS_COMM_H_


void COMM_sendDebug();

typedef enum
{
    BUS_INIT,
	BUS_RX,
    BUS_PROCESS,
    BUS_WAIT_RESPONSE,
    BUS_TX_END,
    BUS_WAIT_TC,
	BUS_ERROR_RECOVER
} BusState;

typedef enum {
	PKT_NONE,
	PKT_READY,
	PKT_DATA_SHORT,
	PKT_DATA_INVALID,
	PKT_CRC_FAILED
}Packet_State;

typedef struct Port_State_ {
	BusState      bus_state;
	Packet_State  rx_packet_state;

	uint8_t servo_id;
	uint8_t resp_status;
	uint8_t resp_level;
	uint8_t resp_index;

	uint8_t baudrate;

	uint8_t return_delay_time;
}Port_State_t;

extern volatile Port_State_t port_state;

void COMM_Init();
void COMM_send(uint8_t* data, uint16_t len);
uint16_t COMM_Process();

void LPUSART_idle_isr();

#endif /* SRC_PROCESS_COMM_H_ */
