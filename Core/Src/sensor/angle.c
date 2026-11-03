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

static I2C_HandleTypeDef *as5600_hi2c = &hi2c1;
static uint16_t angle_fail_cnt;
static uint8_t  angle_fault;

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
    angle_fault = 0;
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
}

as5600_error_t ANGLE_getRawAngle(uint16_t *raw_angle)
{
    as5600_error_t e = as5600_get_raw_angle(raw_angle);
    if (e == AS5600_ERROR_SUCCESS) {
        angle_fail_cnt = 0;
    } else if (++angle_fail_cnt > 5) {      // 连续失败 5 次
        angle_fault = 1;                    // 需要做I2C总线恢复
    }
    return e;
}

void ANGLE_Process()
{
	if (angle_fault){
		i2c1_bus_recover();
	}
}
