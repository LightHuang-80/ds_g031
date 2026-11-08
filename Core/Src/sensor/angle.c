/*
 * angle.c
 *
 *  Created on: 2026年10月7日
 *      Author: Huang
 */

#include "i2c.h"
#include "as5600.h"
#include "motion.h"
#include "angle.h"
#include "main.h"

#define AS5600_I2C_ADDR        (0x36 << 1)
#define AS5600_REG_RAWANGLE_H  0x0C
#define ANGLE_FAIL_LIMIT       5U        /* 连续失败超过这么多个 TIM2 周期, 置故障 */

static I2C_HandleTypeDef *as5600_hi2c = &hi2c1;
static uint16_t angle_fail_cnt;
static uint8_t  angle_fault;
static uint8_t  as5600_fresh;

// 全局变量
uint8_t as5600_buf[2];
volatile uint16_t as5600_angle = 0;

static inline void angle_fail(void)
{
    if (angle_fail_cnt < 255U) {
        angle_fail_cnt++;
    }
}

void i2c1_bus_recover(void)
{
    HAL_I2C_DeInit(&hi2c1);

    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);       // SDA 释放
    for (int i = 0; i < 9; i++) {                             // 给从机补 9 个时钟
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); delay_us(5);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   delay_us(5);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)) break;       // SDA 被放开就停
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); delay_us(5);   // 补一个 STOP
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   delay_us(5);

    MX_I2C1_Init();                                           // MspInit 会把引脚配回 AF_OD
}

uint32_t AS5600_I2C_Xfer(
    uint8_t slave_address,
    const uint8_t *p_tx_buffer,
    size_t tx_buffer_size,
    uint8_t *p_rx_buffer,
    size_t rx_buffer_size)
{
    HAL_StatusTypeDef result = HAL_OK;

    if (p_tx_buffer == NULL || tx_buffer_size == 0)
        return HAL_ERROR;

    result = HAL_I2C_Master_Transmit(
        as5600_hi2c,
		 (uint16_t)(slave_address << 1),
        (uint8_t *)p_tx_buffer,
        (uint16_t)tx_buffer_size,
        2
    );

    if (result != HAL_OK)
        return result;

    if (p_rx_buffer != NULL && rx_buffer_size > 0)
    {
        result = HAL_I2C_Master_Receive(
            as5600_hi2c,
			 (uint16_t)(slave_address << 1),
            p_rx_buffer,
            (uint16_t)rx_buffer_size,
            2
        );
    }

    return result;
}

void ANGLE_Init()
{
	as5600_init(AS5600_I2C_Xfer);

	as5600_angle = 0;

	/* first angle read */
	uint8_t ret = 0;
	while (ret < 5){
		as5600_error_t e = as5600_get_raw_angle((uint16_t*)&as5600_angle);
		if (e == AS5600_ERROR_SUCCESS) break;
		ret ++;
	}
}

as5600_error_t ANGLE_getRawAngle(uint16_t *raw_angle)
{
    if (as5600_fresh) {
        as5600_fresh = 0;
        *raw_angle   = as5600_angle;
        return AS5600_ERROR_SUCCESS;
    }
    return AS5600_ERROR_I2C_ERROR;
}


/* ---- I2C 读取完成 (中断里) ---- */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c1) return;

    as5600_angle   = (uint16_t)((((uint16_t)as5600_buf[0] << 8) | as5600_buf[1]) & 0x0FFF);

    as5600_fresh   = 1;
    angle_fail_cnt = 0;                  /* 成功完成才清零, 不是"发起成功"就清零 */
}

/* ---- I2C 传输出错 (中断里), HAL 在处理错误时已把状态恢复为 READY ---- */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c1) return;

    // 1. 先把状态清干净（非常重要）
    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    angle_fail();
}

void ANGLE_StartRead(void)
{
    if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY) {
        if (HAL_I2C_Mem_Read_IT(&hi2c1, AS5600_I2C_ADDR, AS5600_REG_RAWANGLE_H,
                                I2C_MEMADD_SIZE_8BIT, as5600_buf, 2) != HAL_OK) {
            angle_fail();                /* 总线忙 / 器件无应答 / 地址阶段出错 */
        }
    } else {
        angle_fail();                    /* 上一次读取还没完成 (正常情况下不会发生) */
    }

    if (angle_fail_cnt > ANGLE_FAIL_LIMIT) {
        angle_fault = 1;
    }
}

void ANGLE_Process()
{
	if (angle_fault){
		angle_fault = 0;

		i2c1_bus_recover();
	}
}
