/*
 * comm.c
 *
 *  Created on: 2026年9月19日
 *      Author: Huang
 */

#include <string.h>

#include "usart.h"
#include "dma.h"
#include "comm.h"
#include "dma.h"
#include "usart.h"
#include "config.h"
#include "dynamixel.h"
#include "process.h"
#include "CO_fifo.h"
#include "stm32g0xx_it.h"
#include "servo.h"

#include "main.h"

#define RTX_DMA_BUF_SIZE  384
uint8_t  uart_dma_rtx[RTX_DMA_BUF_SIZE];

#define RX_BUF_SIZE 512
uint8_t  dxl_ring_buf[RX_BUF_SIZE];

#define DXL_PKT_MAX_LEN 382
uint8_t  dxl_packet_buf[DXL_PKT_MAX_LEN];

volatile Port_State_t port_state;
CO_fifo_t    port_fifo;

uint32_t     tx_start_tick;
volatile uint8_t  comm_recover_req = 0;   // ISR 置位,主循环清除
volatile uint32_t comm_err_ore, comm_err_fe, comm_err_ne, comm_err_pe, comm_err_dma;

HAL_StatusTypeDef COMM_rx();
static inline uint8_t rx_is_armed(void)
{
    return (hlpuart1.RxState == HAL_UART_STATE_BUSY_RX) &&
           (hlpuart1.Instance->CR1 & USART_CR1_RE) &&
           (hlpuart1.Instance->CR3 & USART_CR3_DMAR);
}
static inline uint8_t rx_dma_ready(void)
{
    return (hlpuart1.RxState == HAL_UART_STATE_BUSY_RX) &&
           (hlpuart1.Instance->CR3 & USART_CR3_DMAR);
}
/* ================================================================== *
 * =========================Port ISR ================================ *
 * ================================================================== */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != LPUART1) return;
    if (port_state.bus_state != BUS_WAIT_TC) return;

    if (rx_dma_ready()) {
        /* 先清残留,再切方向 */
        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF | UART_CLEAR_OREF |
                                     UART_CLEAR_FEF  | UART_CLEAR_NEF  |
                                     UART_CLEAR_PEF);

        /* 一次原子操作:关 TE、开 RE */
        ATOMIC_MODIFY_REG(huart->Instance->CR1, USART_CR1_TE, USART_CR1_RE);

        port_state.bus_state = BUS_RX;
    } else {
        /* DMA 没挂着,交给主循环 COMM_Process -> COMM_rx 完整恢复 */
        port_state.bus_state = BUS_TX_END;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance != LPUART1)
	        return;

	uint8_t rcvReady = 0;

	uint8_t retry = 0;
	while (retry++ <= 3){
		if (HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, uart_dma_rtx, RTX_DMA_BUF_SIZE) == HAL_OK){
			__HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);
			rcvReady = 1;
		}
	}

	if (Size > 0 && Size <= RTX_DMA_BUF_SIZE){
		CO_fifo_write(
			&port_fifo,
			(const char *)uart_dma_rtx,
			Size,
			NULL
		);
	}else{
		port_state.bus_state = BUS_ERROR_RECOVER;
		return;
	}

	port_state.rx_packet_state = PKT_READY;

	/*
	 * 重新启动下一次接收
	 */
#ifdef UART_SEND
	HAL_UARTEx_ReceiveToIdle_IT(&hlpuart1,
	            uart_dma_rtx,
	            RTX_DMA_BUF_SIZE);
#else

	if (!rcvReady)
		port_state.bus_state = BUS_ERROR_RECOVER;
#endif
}

/*
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != LPUART1)
	    return;

    // 1. 停止所有 DMA 和当前传输
     HAL_UART_DMAStop(huart);

     // 2. 清除所有可能的错误标志
     __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);

     // 3. 【核心】如果外设依然锁死，通过 RCC 强制硬复位 LPUART 硬件
     __HAL_RCC_LPUART1_FORCE_RESET();
     for(volatile int i = 0; i < 200; i++); // 短暂延时确保复位完成
     __HAL_RCC_LPUART1_RELEASE_RESET();

     // 4. 重新初始化 LPUART 的硬件寄存器
     // 这里的 MX_LPUART1_UART_Init 是 CubeMX 自动生成的初始化函数
     extern void MX_LPUART1_UART_Init(void);
     MX_LPUART1_UART_Init();

	port_state.bus_state = BUS_ERROR_RECOVER;
}
*/

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != LPUART1)
        return;

    uint32_t err = huart->ErrorCode;

    // 统计,方便判断到底是 ORE 还是 FE/NE 占主导
    if (err & HAL_UART_ERROR_ORE) comm_err_ore++;
    if (err & HAL_UART_ERROR_FE)  comm_err_fe++;
    if (err & HAL_UART_ERROR_NE)  comm_err_ne++;
    if (err & HAL_UART_ERROR_PE)  comm_err_pe++;
    if (err & HAL_UART_ERROR_DMA) comm_err_dma++;

    // 清错误标志(HAL 一般已清,这里再保险一次)
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                 UART_CLEAR_NEF  | UART_CLEAR_PEF);

    // 正在发送(WAIT_TC)时,只有 DMA 错误才需要中止发送;
    // RX 侧的噪声错误不应该打断自己的 TX
    if (port_state.bus_state == BUS_WAIT_TC && !(err & HAL_UART_ERROR_DMA))
        return;

    // 只置请求标志,不直接改 bus_state,避免和 COMM_rx() 里的赋值互相覆盖
    comm_recover_req = 1;
}
/* ================================================================== *
 * ================================================================== */
HAL_StatusTypeDef COMM_rx(void)
{
    HAL_StatusTypeDef status;
    uint8_t retry;

    // 1. 彻底关掉发送
    CLEAR_BIT(hlpuart1.Instance->CR1, USART_CR1_TE);

    /*
     * 1. 停止接收
     */
    status = HAL_UART_AbortReceive(&hlpuart1);
    if (status != HAL_OK)
    	return status;

    /*
     * 2. 清 UART 错误标志
     */
    __HAL_UART_CLEAR_FLAG(
        &hlpuart1,
		UART_CLEAR_IDLEF |
        UART_CLEAR_OREF |
        UART_CLEAR_FEF  |
        UART_CLEAR_NEF  |
        UART_CLEAR_PEF
    );

    // 3. 读空 RDR（防止残留字节）
    while (__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_RXNE)) {
        volatile uint8_t dummy = (uint8_t)(hlpuart1.Instance->RDR);
        (void)dummy;
    }

    /*
     * 3. 半双工切换到接收
     */
    status = HAL_HalfDuplex_EnableReceiver(&hlpuart1);
    if (status != HAL_OK)
    	return status;

    /*
     * 5. 启动 ReceiveToIdle DMA
     */
    for (retry = 0; retry < 3; retry++){
        status = HAL_UARTEx_ReceiveToIdle_DMA(
            &hlpuart1,
            uart_dma_rtx,
            RTX_DMA_BUF_SIZE
        );

        if (status == HAL_OK)
        {
            __HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);
            if (!comm_recover_req)               // 启动期间没有新错误才进入 BUS_RX
                port_state.bus_state = BUS_RX;

            return HAL_OK;
        }

        /*
         * 启动失败，先停止再重试
         */
        HAL_UART_AbortReceive(&hlpuart1);
        HAL_Delay(1);
    }

    /*
     * 6. 3次都失败
     */
    port_state.bus_state = BUS_ERROR_RECOVER;
    return HAL_ERROR;
}

void COMM_send(uint8_t* data, uint16_t len)
{
	if (len == 0 || port_state.bus_state == BUS_WAIT_TC){
		// don't start comm tx mode
		// no data to send or uart is tx busy
		return;
	}

	HAL_HalfDuplex_EnableTransmitter(&hlpuart1);

	/*
	 * 发送前：
	 * 关闭 TC 中断
	 * 清掉可能残留的 TC
	 */
	__HAL_UART_DISABLE_IT(&hlpuart1, UART_IT_TC);
	__HAL_UART_CLEAR_FLAG(&hlpuart1, UART_FLAG_TC);

	port_state.bus_state = BUS_WAIT_TC;
    tx_start_tick = HAL_GetTick();

	if (data != uart_dma_rtx){
		memcpy(uart_dma_rtx, data, len);
	}


#ifdef UART_SEND
	// 1*. 直接发送
	HAL_UART_Transmit(&hlpuart1, data, len, 0xFF);


	// 2. 确保最后一个 bit 已经发送 //
	uint32_t timeout = HAL_GetTick();

	while (__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_TC) == RESET)
	{
	    if ((HAL_GetTick() - timeout) > 5)
	    {
	        port_state.bus_state = BUS_ERROR_RECOVER;
	        return;
	    }
	}

	// 3. 清 TC //
	__HAL_UART_CLEAR_FLAG(
	        &hlpuart1,
	        UART_FLAG_TC
	    );

	port_state.bus_state = BUS_TX_END;

#else
	// 1. 启动 DMA 发送
	HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&hlpuart1, uart_dma_rtx, len);

	if (status != HAL_OK){
	    port_state.bus_state = BUS_ERROR_RECOVER;
	    return;
	}

	__HAL_DMA_DISABLE_IT(
		hlpuart1.hdmatx,
		DMA_IT_HT
	);

#endif

}

void COMM_Init()
{

	CO_fifo_init(&port_fifo, (char*)dxl_ring_buf, RX_BUF_SIZE);

	/*
	 * 确保错误中断开启
	 */
	__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_ERR);

	COMM_rx();
}

void COMM_parseFastSyncRead(uint8_t* packet, uint16_t pkt_len)
{

}

void COMM_parseSyncWrite(uint8_t* packet, uint16_t pkt_len)
{
#ifndef DEBUG
	uint8_t dev_id = DXL_RAMTable[ID_ADDR];

	// check the dev list
	uint16_t data_len;
	MAKE_UINT16(data_len, packet[5], packet[6]);

	uint16_t param_addr;
	MAKE_UINT16(param_addr, packet[8], packet[9]);

	uint16_t param_len;
	MAKE_UINT16(param_len, packet[10], packet[11]);

	// 5-6   data length
	// 7     instruction
	// 8-9   address
	// 10-11 parameter length
	// 12    sync id
	// 13 ~ 13 + parameter length  data
	// ...   sync id
	// ...+...   data
	uint16_t devs = (data_len - 7) / (1 + param_len);

	for (uint16_t i = 0; i < devs; i++){
		if (packet[12 + i*(1 + param_len)] == dev_id){
			// update the ram table
			uint8_t* dataptr = &(packet[12+i*(1 + param_len)+1]);
			memcpy((void*)(&DXL_RAMTable[param_addr]), (const void*)dataptr, param_len);

			//process the data
			Proc_addr_data(param_addr, dataptr, param_len);
			break;
		}
	}
#endif
}

void COMM_parseBulkWrite(uint8_t* packet, uint16_t pkt_len)
{
#ifndef DEBUG
	uint8_t dev_id = DXL_RAMTable[ID_ADDR];

	// check the dev list
	uint16_t data_len;
	MAKE_UINT16(data_len, packet[5], packet[6]);

	data_len -= 3;

	uint16_t i = 0;

	while (i < data_len){
		uint8_t pkt_id = packet[8+i];
		i += 1;

		uint16_t param_addr;
		MAKE_UINT16(param_addr, packet[8+i], packet[8+i+1]);
		i += 2;

		uint16_t param_len;
		MAKE_UINT16(param_len, packet[8+i], packet[8+i+1]);
		i += 2;

		if (dev_id == pkt_id){
			memcpy((void*)(&DXL_RAMTable[param_addr]), (const void*)&packet[8+i], param_len);
		}

		i += param_len;
	}
#endif
}

void COMM_processFastSyncRespond(uint16_t crc_sum)
{
#ifndef DEBUG
	// No delay return times
	// make fast response
	dynamixel_fast_status_resp(&hlpuart1, crc_sum);

	sync_pkt.sync_state = FAST_SYNC_READ_RESPOND;
#endif
}

void COMM_processSyncRespond(uint8_t inst)
{
#ifndef DEBUG
	// delay return times
	if (port_state.return_delay_time > 0){
		delay_us(port_state.return_delay_time * 2);
	}

	// make response
	dynamixel_status_resp(&hlpuart1, inst);

	// update the sync state
	if(sync_pkt.sync_state == SYNC_READ_WAIT){
		sync_pkt.sync_state = SYNC_READ_RESPOND;
	}else if (sync_pkt.sync_state == BULK_READ_WAIT){
		sync_pkt.sync_state = BULK_READ_RESPOND;
	}else if (sync_pkt.sync_state == FAST_SYNC_READ_WAIT){
		sync_pkt.sync_state = FAST_SYNC_READ_RESPOND;
	}

#endif
}

uint8_t COMM_checkIsResp(uint8_t* packet, uint16_t pkt_len)
{
#ifndef DEBUG
	uint8_t pkt_id = packet[4];

	if (sync_pkt.devices_num < 1)
		return 0;

	for (uint16_t i = 0; i < sync_pkt.devices_num - 1; i++){
		if (sync_pkt.devices_chain[i] == pkt_id && sync_pkt.devices_chain[i+1] == port_state.servo_id){
			return 1;
		}
	}
#endif
	return 0;
}

uint8_t COMM_parseBulkRead(uint8_t* packet, uint16_t pkt_len)
{
	uint8_t inlist = 0;

#ifndef DEBUG
	uint8_t dev_id = DXL_RAMTable[ID_ADDR];

	// take the devs id list
	uint16_t params_len;
	MAKE_UINT16(params_len, packet[5], packet[6]);

	uint16_t idx = 0;
	for (uint16_t i = 0; i < 255 ;i++){

		uint16_t addr;
		uint16_t len;

		uint8_t pkt_id = packet[8 + idx];
		idx += 1;

		MAKE_UINT16(addr, packet[8+idx], packet[8+idx+1]);
		idx += 2;

		MAKE_UINT16(len, packet[8+idx], packet[8+idx+1]);
		idx += 2;

		if (dev_id == pkt_id){
			sync_pkt.data_len = len;
			sync_pkt.reg_addr = addr;
			sync_pkt.sync_type = BULK_READ_TYPE;
			sync_pkt.sync_state = BULK_READ_WAIT;
			sync_pkt.resp_index = i;
			inlist = 1;
		}

		sync_pkt.devices_chain[i] = pkt_id;
		sync_pkt.devices_num += 1;

		if (idx >= params_len - 3){
			break;
		}
	}
#endif
	return inlist;
}

uint8_t COMM_parseSyncRead(uint8_t* packet, uint16_t pkt_len, uint8_t type, uint8_t state)
{
	uint8_t inlist = 0;
	// take the devs id list
	uint16_t params_len;
	MAKE_UINT16(params_len, packet[5], packet[6]);

	// 保存sync read 的信息
	MAKE_UINT16(sync_pkt.reg_addr, packet[8], packet[9]);
	MAKE_UINT16(sync_pkt.data_len, packet[10], packet[11]);

	sync_pkt.devices_num = params_len - 7;
	if (sync_pkt.devices_num < 1 || sync_pkt.devices_num > 30){
		// exceed max devices num
		return 0;
	}

	for (uint8_t i = 0; i < sync_pkt.devices_num; i++){
		sync_pkt.devices_chain[i] = packet[12+i];

		if (sync_pkt.devices_chain[i] == port_state.servo_id){
			// Save the response type and state
			sync_pkt.sync_type  = type;
			sync_pkt.sync_state = state;
			sync_pkt.resp_index = i;
			inlist = 1;
		}
	}

	if (!inlist){
		// not in sync list
		sync_pkt.sync_state = 0;
		sync_pkt.sync_type  = SYNC_NONE;
	}

#ifndef DEBUG
#endif
	return inlist;
}

void COMM_ParsePacket(uint8_t* packet, uint16_t pkt_len)
{
	// skip the header check && crc check
	uint8_t id   = packet[4];
	uint8_t inst = packet[7];

	// port state 的 return delay time update
	port_state.return_delay_time = DXL_RAMTable[9];

	switch(inst){
	case PING:{
		// PING
		if (id == BROADCAST_ID || id == port_state.servo_id){
			if (id == BROADCAST_ID){
				// 需要做delay return time
				delay_us(port_state.return_delay_time * 2);
				delay_us(port_state.servo_id * 100);
			}

			dynamixel_ping(&hlpuart1, port_state.servo_id);
		}
	}break;
	case READ:{
		if (id == port_state.servo_id){
			dynamixel_read(&hlpuart1, packet, pkt_len);
		}
	}break;
	case WRITE:{
		if (id == port_state.servo_id){
			if (!dynamixel_write(&hlpuart1, packet, pkt_len)){

				// memory pointers update
				Proc_control_table_Update(packet, pkt_len);
			}
		}
	}break;
	case REG_WRITE:{
		if (id == port_state.servo_id){
			dynamixel_regwrite(&hlpuart1, packet, pkt_len);
		}
	}break;
	case ACTION:{
		if (id == port_state.servo_id || id == BROADCAST_ID){
			dynamixel_action(&hlpuart1, packet, pkt_len);
		}
	}break;
	case REBOOT:{
		if (id == port_state.servo_id){
			uint8_t err = DXL_RAMTable[HARDWARE_ERR_ADDR];
			send_packet_v2(&hlpuart1, id, 0x55, &err, 1);

			HAL_Delay(2);

			// do action
			system_reboot();
		}
	}break;
	case SYNC_READ:{
		if (id == BROADCAST_ID){
			// reset sync
			sync_pkt.sync_type  = SYNC_NONE;
			sync_pkt.sync_state = 0;
			sync_pkt.devices_num = 0;

			// 解析sync read，并且存储设备列表
			uint8_t inlist = COMM_parseSyncRead(packet, pkt_len,
												SYNC_READ_TYPE,
												SYNC_READ_WAIT);

			if (inlist){

				if (port_state.servo_id == sync_pkt.devices_chain[0]){
					// the first device in list
					COMM_processSyncRespond(port_state.servo_id);
				}
			}
		}
	}break;

	case FAST_SYNC_READ:{
		if (id == BROADCAST_ID){
			// reset sync
			sync_pkt.sync_type  = SYNC_NONE;
			sync_pkt.sync_state = 0;
			sync_pkt.devices_num = 0;

			// 解析sync read，并且存储设备列表
			uint8_t inlist = COMM_parseSyncRead(packet, pkt_len,
												FAST_SYNC_READ_TYPE,
												FAST_SYNC_READ_WAIT);
			if (inlist){

				if (port_state.servo_id == sync_pkt.devices_chain[0]){
					// first delay return times
					if (port_state.return_delay_time > 0){
						delay_us(port_state.return_delay_time * 2);
					}

					// the first device in list
					dynamixel_first_fast_status_resp(&hlpuart1, port_state.servo_id);
				}
			}
		}
	}break;

	case STATUS_RETURN:{
		if (id == BROADCAST_ID){
			if (sync_pkt.sync_type == FAST_SYNC_READ_TYPE &&
				sync_pkt.sync_state == FAST_SYNC_READ_WAIT){
				// get the last crc sum
				uint16_t crc_sum = packet[pkt_len - 1];
				crc_sum <<= 8; crc_sum &= 0xFF00;
				crc_sum += (packet[pkt_len - 2] & 0x00FF);

				// send back fast sync read response
				COMM_processFastSyncRespond(crc_sum);
			}
		}else{
			if (sync_pkt.sync_state == SYNC_READ_WAIT ||
					sync_pkt.sync_state == BULK_READ_WAIT){
				if (COMM_checkIsResp(packet, pkt_len)){
					COMM_processSyncRespond(port_state.servo_id);
				}
			}
		}
	}break;

	case SYNC_WRITE:{
		if (id == BROADCAST_ID){
			COMM_parseSyncWrite(packet, pkt_len);
		}
	}break;

	case BULK_READ:{
		if (id == BROADCAST_ID){
			// 解析sync read，并且存储设备列表
			sync_pkt.sync_type   = SYNC_NONE;
			sync_pkt.sync_state  = 0;
			sync_pkt.devices_num = 0;

			uint8_t inlist = COMM_parseBulkRead(packet, pkt_len);
			if (inlist && port_state.servo_id == sync_pkt.devices_chain[0]){
				COMM_processSyncRespond(port_state.servo_id);
			}
		}
	}break;

	case BULK_WRITE: {
		if (id == BROADCAST_ID){
			COMM_parseBulkWrite(packet, pkt_len);
		}
	}break;

	default:
		break;
	}
}

void COMM_PacketHandler()
{
	uint16_t rx_sz;
	while(1){
		rx_sz = CO_fifo_getOccupied(&port_fifo);
		if (rx_sz < 10 || rx_sz > RX_BUF_SIZE ){
			/* 最小长度*/

			if (rx_sz > RX_BUF_SIZE){
				CO_fifo_reset(&port_fifo);
			}
			break;
		}

		// reset to max packet length DXL_PKT_MAX_LEN
		rx_sz = (rx_sz > DXL_PKT_MAX_LEN) ? DXL_PKT_MAX_LEN : rx_sz;

		CO_fifo_altBegin(&port_fifo, 0);
		uint16_t rcv_len = CO_fifo_altRead(&port_fifo, (char *)dxl_packet_buf, rx_sz);

		/*verify the recv data*/
		uint16_t pkt_len = 0;
		uint16_t err = verify_dxl_packet(dxl_packet_buf, rcv_len, &pkt_len);

		if (!err){
			/* Correct packet*/
			if (pkt_len < rcv_len){
				CO_fifo_offset(&port_fifo, pkt_len);
			}else if (pkt_len == rcv_len){
				CO_fifo_altFinish(&port_fifo, NULL);
			}else{
				/*invalid packet length*/
				CO_fifo_altFinish(&port_fifo, NULL);
				break;
			}

			/* parse the packet*/
			COMM_ParsePacket(dxl_packet_buf, pkt_len);
		}else {
			if (err == CRC_ERROR){
				// skip this packet //
				if (rcv_len < pkt_len){
					CO_fifo_offset(&port_fifo, rcv_len);
				}else{
					CO_fifo_offset(&port_fifo, pkt_len);
				}
			}else if (err == DATA_RANGE_ERROR){

				CO_fifo_offset(&port_fifo, rcv_len);

			}else if (err == INSTRUCTION_ERROR_V2){
				// dxl protocol header error //
				CO_fifo_offset(&port_fifo, 1);

			}else if (err == DATA_LENGTH_ERROR){
				// wait the next input //
				break;
			}else{
				// Unknown issue
				break;
			}
		}
	}
}

uint16_t COMM_Process()
{
	// 先清标志,再恢复:恢复期间再来的错误会重新置位,不会丢
    if (comm_recover_req) {
        comm_recover_req = 0;
        port_state.bus_state = BUS_ERROR_RECOVER;
    }

	if (port_state.bus_state == BUS_TX_END || port_state.bus_state == BUS_ERROR_RECOVER){

        if (port_state.bus_state == BUS_ERROR_RECOVER) {
            HAL_UART_Abort(&hlpuart1);      // 让 HAL 的 gState/RxState 回到 READY,不要延时
            sync_pkt.sync_type  = SYNC_NONE; // 顺便清掉 sync 状态,避免误回复
            sync_pkt.sync_state = 0;
        }

		COMM_rx();
	}

	if (port_state.bus_state == BUS_RX && !rx_is_armed())
	    port_state.bus_state = BUS_ERROR_RECOVER;

	if (port_state.bus_state == BUS_WAIT_TC &&
	    (HAL_GetTick() - tx_start_tick) > 5 /* ms,按最大包长算 */)
	    port_state.bus_state = BUS_ERROR_RECOVER;

	if (port_state.rx_packet_state == PKT_READY){
		port_state.rx_packet_state = PKT_NONE;
		COMM_PacketHandler();
	}

	return port_state.bus_state;
}
