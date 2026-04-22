/**
 ******************************************************************************
 * @file      driver_ads1118_interface.h
 * @brief     ADS1118 platform interface for STM32F103 (HAL)
 *            Implements the callbacks required by the libdriver ADS1118 driver.
 *
 * Hardware connections (from schematic):
 *   SPI1 SCK  -> PA5 (22Ω series)
 *   SPI1 MISO -> PA6 (22Ω series)
 *   SPI1 MOSI -> PA7 (22Ω series)
 *   CS_ADC    -> PB0 (active-low chip select)
 ******************************************************************************
 */

#ifndef DRIVER_ADS1118_INTERFACE_H
#define DRIVER_ADS1118_INTERFACE_H

#include "driver_ads1118.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t ads1118_interface_spi_init(void);
uint8_t ads1118_interface_spi_deinit(void);
uint8_t ads1118_interface_spi_transmit(uint8_t *tx, uint8_t *rx, uint16_t len);
void    ads1118_interface_delay_ms(uint32_t ms);
void    ads1118_interface_debug_print(const char *const fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_ADS1118_INTERFACE_H */
