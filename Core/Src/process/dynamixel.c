#include <string.h>

#include "config.h"
#include <dynamixel.h>
#include "comm.h"
#include "process.h"

sync_pkt_data_t   sync_pkt;
reg_pkt_data_t    reg_pkt;

// PULL-UP RESISTOR: 1.8K
// BAUDRATE: 117 647 (WHEN BAD COMMUNICATION WITH 115 200)

void send_packet_v2(UART_HandleTypeDef *huart, uint8_t id, uint8_t inst, uint8_t *params, uint16_t params_len)
{
	uint16_t packet_length = 10 + params_len;

	uint8_t packet[48];

	packet[0] = 0xFF; // Header 1
	packet[1] = 0xFF; // Header 2
	packet[2] = 0xFD; // Header 3
	packet[3] = 0x00; // Reserved
	packet[4] = id;   // Packet ID

	// Length = Parameter length + 3
	packet[5] = GET_INT16_LSB(params_len + 3); // Length 1 (LSB)
	packet[6] = GET_INT16_MSB(params_len + 3); // Length 2 (MSB)

	packet[7] = (uint8_t)inst; // Instrucion

	// Parameter 1 to X
	for (uint16_t i = 0; i < params_len; i++)
		packet[8 + i] = params[i];

	// CRC
	uint16_t crc = update_crc(0, packet, packet_length - 2); // Calculating CRC
	packet[packet_length - 2] = GET_INT16_LSB(crc);          // CRC 1 (LSB)
	packet[packet_length - 1] = GET_INT16_MSB(crc);          // CRC 2 (MSB)

	COMM_send(packet, packet_length);

}

uint8_t parse_status_packet_v2(uint8_t *packet, uint32_t packet_len, uint8_t *id, uint8_t *params, uint16_t *params_len, uint8_t *error, uint8_t *crc_check)
{
	// Check instruction
	if (packet[7] != 0x55)
		return 0;

	*id = packet[4];
	*error = packet[8];

	uint16_t length = packet[5] + ((uint16_t)(packet[6] << 8) & 0xFF00);
	*params_len = length - 3 - 1;

	for (uint16_t i = 0; i < *params_len; i++)
	  params[i] = packet[9 + i];

	// CRC check
	uint16_t crc = update_crc(0, packet, packet_len - 2);          // Calculating CRC
	uint8_t crc_l = packet[packet_len - 2] == GET_INT16_LSB(crc);  // CRC 1 (LSB)
	uint8_t crc_h = packet[packet_len - 1] == GET_INT16_MSB(crc);  // CRC 2 (MSB)
	*crc_check = (crc_l && crc_h);

	return 1;
}

// ============================================================================================================== //
// SPECIFIC FUNCTIONS
// ============================================================================================================== //
void dynamixel_ping(UART_HandleTypeDef *huart, uint8_t id)
{
	uint8_t params[4];
	params[0] = DXL_RAMTable[70];  // Error status
	params[1] = DXL_RAMTable[0];   // Model number LSM
	params[2] = DXL_RAMTable[1];   // Model number HSM
	params[3] = DXL_RAMTable[6];   // Hardware firmware

	// Response allways with PING code
	send_packet_v2(huart, id, STATUS_RETURN, params, 4);
}

uint8_t dynamixel_check_address_rw(uint16_t addr)
{
	uint8_t writable = 1;

	/* unwritable address*/
	if ((addr >= 0 && addr <= 6) ||
		(addr >= 120 && addr <= 147) ||
		addr == 69 || addr == 70 ||
		addr >= 252){
		writable = 0;
	}

	return writable;
}

void dynamixel_action(UART_HandleTypeDef *huart, uint8_t* packet, uint16_t packet_len)
{
	uint8_t error = DXL_RAMTable[HARDWARE_ERR_ADDR];

	if (DXL_RAMTable[REG_INST_ADDR]){
		DXL_RAMTable[REG_INST_ADDR] = 0;

		// do action
		memcpy((void*)(DXL_RAMTable + reg_pkt.reg_addr), (const void*)reg_pkt.reg_data, reg_pkt.data_len);
		Proc_addr_data(reg_pkt.reg_addr, reg_pkt.reg_data, reg_pkt.data_len);

		// send status report
		if (port_state.resp_level >= 2){
			if (port_state.return_delay_time > 0){
				delay_us(port_state.return_delay_time * 2);
			}

			send_packet_v2(huart, DXL_RAMTable[ID_ADDR], 0x55, &error, 1);
		}
	}
}

uint8_t dynamixel_write(UART_HandleTypeDef *huart, uint8_t* packet, uint16_t packet_len)
{
	uint16_t addr = packet[9]; addr <<= 8; addr &= 0xFF00;
	addr +=  packet[8] & 0x00FF;

	uint16_t params_len  = packet[6];  params_len <<= 8; params_len &= 0xFF00;
	params_len += packet[5] & 0x00FF;

	uint8_t error = 0;

	/* unwritable address*/
	if (!dynamixel_check_address_rw(addr) || params_len > 12){
		error = ACCESS_ERROR;
	}

	if (!error){
		uint8_t* params = (uint8_t*)&(packet[10]);
		memcpy((void*)(DXL_RAMTable + addr), (const void*)params, params_len - 5);
		error = 0;
	}

	if (port_state.resp_level >= 2){
		if (port_state.return_delay_time > 0){
			delay_us(port_state.return_delay_time * 2);
		}

		send_packet_v2(huart, DXL_RAMTable[ID_ADDR], 0x55, &error, 1);
	}

	return error;
}

void dynamixel_regwrite(UART_HandleTypeDef *huart, uint8_t* packet, uint16_t packet_len)
{
	uint16_t addr = packet[9]; addr <<= 8; addr &= 0xFF00;
	addr +=  packet[8] & 0x00FF;

	uint16_t params_len  = packet[6];  params_len <<= 8; params_len &= 0xFF00;
	params_len += packet[5] & 0x00FF;

	uint8_t error = 0;

	/* unwritable address*/
	if (!dynamixel_check_address_rw(addr) || params_len > 12){
		error = ACCESS_ERROR;
	}

	if (!error){
		uint8_t* params = (uint8_t*)&(packet[10]);

		//memcpy((void*)(DXL_RAMTable + addr), (const void*)params, params_len - 5);

		reg_pkt.reg_addr = addr;
		reg_pkt.data_len = params_len - 5;
		memcpy((void*)reg_pkt.reg_data, (const void*)params, params_len - 5);

		DXL_RAMTable[REG_INST_ADDR] = 1;
		error = 0;
	}

	if (port_state.resp_level >= 2){
		if (port_state.return_delay_time > 0){
			delay_us(port_state.return_delay_time * 2);
		}

		send_packet_v2(huart, DXL_RAMTable[ID_ADDR], 0x55, &error, 1);
	}
}

void dynamixel_read(UART_HandleTypeDef *huart, uint8_t* packet, uint16_t packet_len)
{
	uint8_t id = DXL_RAMTable[ID_ADDR];

	uint16_t addr = packet[9]; addr <<= 8;
	addr += packet[8];

	uint16_t len  = packet[11]; len <<= 8;
	len +=  packet[10];

	uint8_t error;
	uint8_t params[12];

	if (addr >= 231 || len > 12){
		error = ACCESS_ERROR;

		if (port_state.resp_level >= 1)
			send_packet_v2(huart, id, 0x55, &error, 1);
	}else{
		/* write the hardware error byte */
		params[0] = DXL_RAMTable[HARDWARE_ERR_ADDR];
		memcpy((void*)(&params[1]), (const void*)(DXL_RAMTable + addr), len);

		if (port_state.resp_level >= 1)
			send_packet_v2(huart, id, 0x55, params, len + 1);
	}
}
void dynamixel_fast_status_resp(UART_HandleTypeDef *huart, uint16_t crc_sum)
{
	sync_pkt.status_data[0] = DXL_RAMTable[HARDWARE_ERR_ADDR];
	sync_pkt.status_data[1] = DXL_RAMTable[ID_ADDR];

	memcpy((void*)(&sync_pkt.status_data[2]),
						(const void*)(&DXL_RAMTable[sync_pkt.reg_addr]),
						sync_pkt.data_len);

	uint8_t cc[2];
	cc[0] = GET_INT16_LSB(crc_sum);
	cc[1] = GET_INT16_MSB(crc_sum);

	// crc update include the lastest crc_sum
	uint16_t crc = update_crc(crc_sum, cc, 2);

	crc = update_crc(crc, sync_pkt.status_data, sync_pkt.data_len + 2); // Calculating CRC
	sync_pkt.status_data[sync_pkt.data_len + 2] = GET_INT16_LSB(crc);          // CRC 1 (LSB)
	sync_pkt.status_data[sync_pkt.data_len + 3] = GET_INT16_MSB(crc);          // CRC 2 (MSB)

	COMM_send(sync_pkt.status_data, sync_pkt.data_len + 4);
}

void dynamixel_status_resp(UART_HandleTypeDef *huart, uint8_t resp_id)
{
	// copy the data
	sync_pkt.status_data[0] = DXL_RAMTable[HARDWARE_ERR_ADDR];

	memcpy((void*)(&sync_pkt.status_data[1]),
			(const void*)(&DXL_RAMTable[sync_pkt.reg_addr]),
			sync_pkt.data_len);

	send_packet_v2(huart, resp_id, 0x55, sync_pkt.status_data, sync_pkt.data_len + 1);
}

void dynamixel_first_fast_status_resp(UART_HandleTypeDef *huart, uint8_t resp_id)
{
	uint8_t packet[48];
	uint8_t params_len = sync_pkt.data_len + 2;

	sync_pkt.status_data[0] = DXL_RAMTable[HARDWARE_ERR_ADDR];
	sync_pkt.status_data[1] = resp_id;

	memcpy((void*)(&sync_pkt.status_data[2]),
			(const void*)(&DXL_RAMTable[sync_pkt.reg_addr]),
			sync_pkt.data_len);

	packet[0] = 0xFF; // Header 1
	packet[1] = 0xFF; // Header 2
	packet[2] = 0xFD; // Header 3
	packet[3] = 0x00; // Reserved
	packet[4] = BROADCAST_ID;   // Packet ID

	// Length = Parameter length + 3
	/* Err + devid0 + data + 2crc + Err + devid1 + data + 2crc*/
	uint16_t total_params_len = sync_pkt.devices_num * (4 + sync_pkt.data_len);
	total_params_len -= 2;  // the last 2crc

	packet[5] = GET_INT16_LSB(total_params_len + 3); // Length 1 (LSB)
	packet[6] = GET_INT16_MSB(total_params_len + 3); // Length 2 (MSB)

	packet[7] = (uint8_t)STATUS_RETURN; // Instrucion

	// Parameter 1 to X
	for (uint16_t i = 0; i < params_len; i++)
		packet[8 + i] = sync_pkt.status_data[i];

	// CRC
	uint16_t packet_length = params_len + 8;
	uint16_t crc = update_crc(0, packet, packet_length); // Calculating CRC

	packet[packet_length    ] = GET_INT16_LSB(crc);          // CRC 1 (LSB)
	packet[packet_length + 1] = GET_INT16_MSB(crc);          // CRC 2 (MSB)
	packet_length += 2;

	COMM_send(packet, packet_length);
}

// ============================================================================================================== //
// UTILITY FUNCTIONS
// ============================================================================================================== //
uint16_t verify_dxl_packet(uint8_t* pkt, uint16_t data_len, uint16_t* packet_len)
{
	uint16_t content_len = 0;

	*packet_len = 0;

	if (data_len < 10){
		// header      4
		// packet id   1
		// length      2
		// instruction 1
		// parameters  1
		// crc         2
		// too less data
		return DATA_LENGTH_ERROR;
	}

	if (pkt[0] == 0xFF && pkt[1] == 0xFF && pkt[2] == 0xFD && pkt[3] == 0){
		content_len = (pkt[5] & 0x00FF) + ((uint16_t)(pkt[6] << 8) & 0xFF00);
	}else{
		// invalid header
		return INSTRUCTION_ERROR_V2;
	}

	if (content_len >= 384){
		/* illegal length */
		return DATA_RANGE_ERROR;
	}

	uint16_t crc_th = content_len + 5;

	/* check the special instruction 0x55
     * the status packet for fast sync read
     */
	if (pkt[4] == BROADCAST_ID && pkt[7] == STATUS_RETURN){
		if (sync_pkt.sync_type == FAST_SYNC_READ_TYPE && sync_pkt.sync_state == FAST_SYNC_READ_WAIT){
			if (sync_pkt.resp_index < 1){
				/*illegal data*/
				return INSTRUCTION_ERROR_V2;
			}
			/* sum the status length */
			/* 第一个的status 长度, 包括 error + device id + paramlen + 2crc  */
			uint16_t status_pkt_len = sync_pkt.data_len + 12;

			/* （本设备的回复顺序-1） * (err+id+data_len+2CRC) */
			status_pkt_len += (sync_pkt.resp_index - 1) * (sync_pkt.data_len + 4);

			if (data_len < status_pkt_len){
				return DATA_LENGTH_ERROR;
			}

			crc_th = status_pkt_len - 2;
		}
	}

	if (data_len < (crc_th + 2)){
		// too less data
		return DATA_LENGTH_ERROR;
	}

	uint16_t crc = pkt[crc_th] + (pkt[crc_th+1] << 8);
	uint16_t crc_sum = update_crc(0, pkt, crc_th);
	if (crc == crc_sum){
		*packet_len = crc_th + 2;
		// return the packet full length
		return 0;
	}else{
		// crc error
		*packet_len = crc_th + 2;
		return CRC_ERROR;
	}

	return 0;
}

uint8_t baudrate_to_value(uint32_t baudrate)
{
	return (uint8_t)((2000000 / baudrate) - 1);
}

uint32_t value_to_baudrate(uint8_t value)
{
	return (uint32_t)(2000000 / (value + 1));
}

uint16_t pos_to_value(float pos_deg)
{
	return (uint16_t)(pos_deg * 3.41);
}

uint16_t vel_to_value(float speed_deg_s)
{
	return (uint16_t)(speed_deg_s * 1.50);
}

uint16_t vel_to_value_wheel_mode(float speed_pct)
{
	return (uint16_t)(speed_pct * 10.24);
}

float value_to_load(uint16_t value)
{
	float load = (value & 0x3FF) * 0.097;

	if (((value >> 10) & 1) == 0)
		load = -load;

	return load;
}

// Used only by protocol v2.0
// Source: https://emanual.robotis.com/docs/en/dxl/crc/
uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
	uint16_t i, j;
	uint16_t crc_table[256] = {
		0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
		0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
		0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
		0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
		0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
		0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
		0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
		0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
		0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
		0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
		0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
		0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
		0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
		0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
		0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
		0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
		0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
		0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
		0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
		0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
		0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
		0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
		0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
		0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
		0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
		0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
		0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
		0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
		0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
		0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
	};

	for (j = 0; j < data_blk_size; j++)
	{
		i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}
