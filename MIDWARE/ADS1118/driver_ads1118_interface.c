/**
 ******************************************************************************
 * @file      driver_ads1118_interface.c
 * @brief     ADS1118 platform interface implementation for STM32F103 (HAL)
 *
 * CS_ADC is on PB0 (active-low).
 * SPI1 must already be initialised by MX_SPI1_Init() before ads1118_init().
 ******************************************************************************
 */

#include "driver_ads1118_interface.h"
#include "spi.h"
#include "usart.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include <stdarg.h>
#include <stdio.h>

/* SPI timeout in ms --------------------------------------------------------*/
#define ADS1118_SPI_TIMEOUT_MS  100U

/**
 * @brief  SPI bus init — SPI1 is already started by MX_SPI1_Init() in main.
 *         Nothing extra to do here.
 */
uint8_t ads1118_interface_spi_init(void)
{
    /* Enable SPI isolator (active-high enable), then set CS idle-high */
    HAL_GPIO_WritePin(EN_SPI_GPIO_Port, EN_SPI_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, GPIO_PIN_SET);
    return 0;
}

/**
 * @brief  SPI bus deinit — leave SPI1 running for any future use.
 */
uint8_t ads1118_interface_spi_deinit(void)
{
    HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, GPIO_PIN_SET);
    return 0;
}

/**
 * @brief  Full-duplex SPI transfer with CS_ADC chip-select.
 */
uint8_t ads1118_interface_spi_transmit(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, ADS1118_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, GPIO_PIN_SET);

    return (status == HAL_OK) ? 0 : 1;
}

/**
 * @brief  Millisecond delay.
 */
void ads1118_interface_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
 * @brief  Debug print via USART1.
 */
void ads1118_interface_debug_print(const char *const fmt, ...)
{
    char    buf[128];
    va_list args;
    int     len;

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100U);
    }
}
