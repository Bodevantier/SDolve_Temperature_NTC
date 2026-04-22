/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      driver_ads1118.h
 * @brief     driver ads1118 header file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2024-02-28
 *
 * Source: https://github.com/libdriver/ads1118
 */

#ifndef DRIVER_ADS1118_H
#define DRIVER_ADS1118_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ads1118 bool enumeration definition
 */
typedef enum
{
    ADS1118_BOOL_FALSE = 0x00,        /**< disable function */
    ADS1118_BOOL_TRUE  = 0x01,        /**< enable function */
} ads1118_bool_t;

/**
 * @brief ads1118 range enumeration definition
 */
typedef enum
{
    ADS1118_RANGE_6P144V = 0x00,        /**< 6.144V range */
    ADS1118_RANGE_4P096V = 0x01,        /**< 4.096V range */
    ADS1118_RANGE_2P048V = 0x02,        /**< 2.048V range */
    ADS1118_RANGE_1P024V = 0x03,        /**< 1.024V range */
    ADS1118_RANGE_0P512V = 0x04,        /**< 0.512V range */
    ADS1118_RANGE_0P256V = 0x05,        /**< 0.256V range */
} ads1118_range_t;

/**
 * @brief ads1118 sample rate enumeration definition
 */
typedef enum
{
    ADS1118_RATE_8SPS   = 0x00,        /**< 8 samples per second */
    ADS1118_RATE_16SPS  = 0x01,        /**< 16 samples per second */
    ADS1118_RATE_32SPS  = 0x02,        /**< 32 samples per second */
    ADS1118_RATE_64SPS  = 0x03,        /**< 64 samples per second */
    ADS1118_RATE_128SPS = 0x04,        /**< 128 samples per second */
    ADS1118_RATE_250SPS = 0x05,        /**< 250 samples per second */
    ADS1118_RATE_475SPS = 0x06,        /**< 475 samples per second */
    ADS1118_RATE_860SPS = 0x07,        /**< 860 samples per second */
} ads1118_rate_t;

/**
 * @brief ads1118 channel enumeration definition
 */
typedef enum
{
    ADS1118_CHANNEL_AIN0_AIN1 = 0x00,        /**< AIN0 and AIN1 pins */
    ADS1118_CHANNEL_AIN0_AIN3 = 0x01,        /**< AIN0 and AIN3 pins */
    ADS1118_CHANNEL_AIN1_AIN3 = 0x02,        /**< AIN1 and AIN3 pins */
    ADS1118_CHANNEL_AIN2_AIN3 = 0x03,        /**< AIN2 and AIN3 pins */
    ADS1118_CHANNEL_AIN0_GND  = 0x04,        /**< AIN0 and GND (single-ended) */
    ADS1118_CHANNEL_AIN1_GND  = 0x05,        /**< AIN1 and GND (single-ended) */
    ADS1118_CHANNEL_AIN2_GND  = 0x06,        /**< AIN2 and GND (single-ended) */
    ADS1118_CHANNEL_AIN3_GND  = 0x07,        /**< AIN3 and GND (single-ended) */
} ads1118_channel_t;

/**
 * @brief ads1118 mode enumeration definition
 */
typedef enum
{
    ADS1118_MODE_ADC         = 0x00,        /**< ADC conversion mode */
    ADS1118_MODE_TEMPERATURE = 0x01,        /**< internal temperature sensor mode */
} ads1118_mode_t;

/**
 * @brief ads1118 handle structure definition
 */
typedef struct ads1118_handle_s
{
    uint8_t (*spi_init)(void);                                            /**< spi_init function pointer */
    uint8_t (*spi_deinit)(void);                                          /**< spi_deinit function pointer */
    uint8_t (*spi_transmit)(uint8_t *tx, uint8_t *rx, uint16_t len);     /**< spi_transmit function pointer */
    void    (*delay_ms)(uint32_t ms);                                     /**< delay_ms function pointer */
    void    (*debug_print)(const char *const fmt, ...);                   /**< debug_print function pointer */
    uint8_t inited;                                                       /**< inited flag */
} ads1118_handle_t;

/**
 * @brief ads1118 information structure definition
 */
typedef struct ads1118_info_s
{
    char     chip_name[32];                /**< chip name */
    char     manufacturer_name[32];        /**< manufacturer name */
    char     interface[8];                 /**< chip interface name */
    float    supply_voltage_min_v;         /**< chip min supply voltage */
    float    supply_voltage_max_v;         /**< chip max supply voltage */
    float    max_current_ma;               /**< chip max current */
    float    temperature_min;              /**< chip min operating temperature */
    float    temperature_max;              /**< chip max operating temperature */
    uint32_t driver_version;               /**< driver version */
} ads1118_info_t;

/* Link macros ---------------------------------------------------------------*/
#define DRIVER_ADS1118_LINK_INIT(HANDLE, STRUCTURE)           memset(HANDLE, 0, sizeof(STRUCTURE))
#define DRIVER_ADS1118_LINK_SPI_INIT(HANDLE, FUC)            (HANDLE)->spi_init = FUC
#define DRIVER_ADS1118_LINK_SPI_DEINIT(HANDLE, FUC)          (HANDLE)->spi_deinit = FUC
#define DRIVER_ADS1118_LINK_SPI_TRANSMIT(HANDLE, FUC)        (HANDLE)->spi_transmit = FUC
#define DRIVER_ADS1118_LINK_DELAY_MS(HANDLE, FUC)            (HANDLE)->delay_ms = FUC
#define DRIVER_ADS1118_LINK_DEBUG_PRINT(HANDLE, FUC)         (HANDLE)->debug_print = FUC

/* Function prototypes -------------------------------------------------------*/
uint8_t ads1118_info(ads1118_info_t *info);
uint8_t ads1118_init(ads1118_handle_t *handle);
uint8_t ads1118_deinit(ads1118_handle_t *handle);
uint8_t ads1118_single_read(ads1118_handle_t *handle, int16_t *raw, float *v);
uint8_t ads1118_continuous_read(ads1118_handle_t *handle, int16_t *raw, float *v);
uint8_t ads1118_start_continuous_read(ads1118_handle_t *handle);
uint8_t ads1118_stop_continuous_read(ads1118_handle_t *handle);
uint8_t ads1118_temperature_convert(ads1118_handle_t *handle, int16_t raw, float *deg);
uint8_t ads1118_set_channel(ads1118_handle_t *handle, ads1118_channel_t channel);
uint8_t ads1118_get_channel(ads1118_handle_t *handle, ads1118_channel_t *channel);
uint8_t ads1118_set_range(ads1118_handle_t *handle, ads1118_range_t range);
uint8_t ads1118_get_range(ads1118_handle_t *handle, ads1118_range_t *range);
uint8_t ads1118_set_rate(ads1118_handle_t *handle, ads1118_rate_t rate);
uint8_t ads1118_get_rate(ads1118_handle_t *handle, ads1118_rate_t *rate);
uint8_t ads1118_set_mode(ads1118_handle_t *handle, ads1118_mode_t mode);
uint8_t ads1118_get_mode(ads1118_handle_t *handle, ads1118_mode_t *mode);
uint8_t ads1118_set_dout_pull_up(ads1118_handle_t *handle, ads1118_bool_t enable);
uint8_t ads1118_get_dout_pull_up(ads1118_handle_t *handle, ads1118_bool_t *enable);
uint8_t ads1118_transmit(ads1118_handle_t *handle, uint8_t *tx, uint8_t *rx, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_ADS1118_H */
