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
 * @file      driver_ads1118.c
 * @brief     driver ads1118 source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2024-02-28
 *
 * Source: https://github.com/libdriver/ads1118
 */

#include "driver_ads1118.h"

/* Chip information ----------------------------------------------------------*/
#define CHIP_NAME          "Texas Instruments ADS1118"
#define MANUFACTURER_NAME  "Texas Instruments"
#define SUPPLY_VOLTAGE_MIN 2.0f
#define SUPPLY_VOLTAGE_MAX 5.5f
#define MAX_CURRENT        0.3f
#define TEMPERATURE_MIN    -40.0f
#define TEMPERATURE_MAX    125.0f
#define DRIVER_VERSION     1000

/* Command definition --------------------------------------------------------*/
#define COMMAND_VALID  (1 << 1)

/**
 * @brief  Read 4 bytes from SPI and return the config word (bytes 2-3)
 */
static uint8_t a_ads1118_spi_read(ads1118_handle_t *handle, uint16_t *data)
{
    uint8_t tx_buf[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rx_buf[4] = {0};

    if (handle->spi_transmit(tx_buf, rx_buf, 4) != 0)
    {
        return 1;
    }
    *data = (uint16_t)((uint16_t)(rx_buf[2]) << 8) | rx_buf[3];

    return 0;
}

/**
 * @brief  Read 2 bytes from SPI and return the conversion result
 */
static uint8_t a_ads1118_spi_read_data(ads1118_handle_t *handle, int16_t *data)
{
    uint8_t tx_buf[2] = {0xFF, 0xFF};
    uint8_t rx_buf[2] = {0};

    if (handle->spi_transmit(tx_buf, rx_buf, 2) != 0)
    {
        return 1;
    }
    *data = (int16_t)((uint16_t)(rx_buf[0]) << 8) | rx_buf[1];

    return 0;
}

/**
 * @brief  Write a 16-bit config word over SPI
 */
static uint8_t a_ads1118_spi_write(ads1118_handle_t *handle, uint16_t data)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    tx_buf[0] = (data >> 8) & 0xFF;
    tx_buf[1] = (data >> 0) & 0xFF;

    if (handle->spi_transmit(tx_buf, rx_buf, 2) != 0)
    {
        return 1;
    }

    return 0;
}

/* Public functions ----------------------------------------------------------*/

uint8_t ads1118_init(ads1118_handle_t *handle)
{
    if (handle == NULL)                              { return 2; }
    if (handle->debug_print == NULL)                 { return 3; }
    if (handle->spi_init == NULL)                    { handle->debug_print("ads1118: spi_init is null.\n");     return 3; }
    if (handle->spi_deinit == NULL)                  { handle->debug_print("ads1118: spi_deinit is null.\n");   return 3; }
    if (handle->spi_transmit == NULL)                { handle->debug_print("ads1118: spi_transmit is null.\n"); return 3; }
    if (handle->delay_ms == NULL)                    { handle->debug_print("ads1118: delay_ms is null.\n");     return 3; }

    if (handle->spi_init() != 0)
    {
        handle->debug_print("ads1118: spi init failed.\n");
        return 1;
    }

    handle->inited = 1;
    return 0;
}

uint8_t ads1118_deinit(ads1118_handle_t *handle)
{
    uint8_t  res;
    uint16_t conf;

    if (handle == NULL)          { return 2; }
    if (handle->inited != 1)     { return 3; }

    res = a_ads1118_spi_read(handle, &conf);
    if (res != 0)
    {
        handle->debug_print("ads1118: read config failed.\n");
        return 4;
    }
    conf &= ~(0x01 << 8);
    conf |=  (0x01 << 8);   /* power down (single-shot mode) */
    res = a_ads1118_spi_write(handle, conf);
    if (res != 0)
    {
        handle->debug_print("ads1118: write config failed.\n");
        return 4;
    }

    res = handle->spi_deinit();
    if (res != 0)
    {
        handle->debug_print("ads1118: spi deinit failed.\n");
        return 1;
    }

    handle->inited = 0;
    return 0;
}

uint8_t ads1118_set_channel(ads1118_handle_t *handle, ads1118_channel_t channel)
{
    uint8_t  res;
    uint16_t conf;

    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    res = a_ads1118_spi_read(handle, &conf);
    if (res != 0) { handle->debug_print("ads1118: read config failed.\n"); return 1; }

    conf &= ~(0x07U << 12);
    conf |=  (uint16_t)(channel & 0x07U) << 12;
    conf &= ~(3U << 1);
    conf |=  COMMAND_VALID;

    res = a_ads1118_spi_write(handle, conf);
    if (res != 0) { handle->debug_print("ads1118: write config failed.\n"); return 1; }

    return 0;
}

uint8_t ads1118_get_channel(ads1118_handle_t *handle, ads1118_channel_t *channel)
{
    uint8_t  res;
    uint16_t conf;

    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    res = a_ads1118_spi_read(handle, &conf);
    if (res != 0) { handle->debug_print("ads1118: read config failed.\n"); return 1; }

    *channel = (ads1118_channel_t)((conf >> 12) & 0x07);
    return 0;
}

uint8_t ads1118_set_range(ads1118_handle_t *handle, ads1118_range_t range)
{
    uint8_t  res;
    uint16_t conf;

    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    res = a_ads1118_spi_read(handle, &conf);
    if (res != 0) { handle->debug_print("ads1118: read config failed.\n"); return 1; }

    conf &= ~(0x07U << 9);
    conf |=  (uint16_t)(range & 0x07U) << 9;
    conf &= ~(3U << 1);
    conf |=  COMMAND_VALID;

    res = a_ads1118_spi_write(handle, conf);
    if (res != 0) { handle->debug_print("ads1118: write config failed.\n"); return 1; }

    return 0;
}

uint8_t ads1118_get_range(ads1118_handle_t *handle, ads1118_range_t *range)
{
    uint8_t  res;
    uint16_t conf;

    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    res = a_ads1118_spi_read(handle, &conf);
    if (res != 0) { handle->debug_print("ads1118: read config failed.\n"); return 1; }

    *range = (ads1118_range_t)((conf >> 9) & 0x07);
    return 0;
}

uint8_t ads1118_set_rate(ads1118_handle_t *handle, ads1118_rate_t rate)
{
    uint8_t  res;
    uint16_t conf;

    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    res = a_ads1118_spi_read(handle, &conf);
    if (res != 0) { handle->debug_print("ads1118: read config failed.\n"); return 1; }

    conf &= ~(0x07U << 5);
    conf |=  (uint16_t)(rate & 0x07U) << 5;
    conf &= ~(3U << 1);
    conf |=  COMMAND_VALID;

    res = a_ads1118_spi_write(handle, conf);
    if (res != 0) { handle->debug_print("ads1118: write config failed.\n"); return 1; }

    return 0;
}

uint8_t ads1118_get_rate(ads1118_handle_t *handle, ads1118_rate_t *rate)
{
    uint8_t  res;
    uint16_t conf;

    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    res = a_ads1118_spi_read(handle, &conf);
    if (res != 0) { handle->debug_print("ads1118: read config failed.\n"); return 1; }

    *rate = (ads1118_rate_t)((conf >> 5) & 0x07);
    return 0;
}

uint8_t ads1118_set_mode(ads1118_handle_t *handle, ads1118_mode_t mode)
{
    uint8_t  res;
    uint16_t conf;

    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    res = a_ads1118_spi_read(handle, &conf);
    if (res != 0) { handle->debug_print("ads1118: read config failed.\n"); return 1; }

    conf &= ~(1U << 4);
    conf |=  (uint16_t)(mode & 0x01U) << 4;
    conf &= ~(3U << 1);
    conf |=  COMMAND_VALID;

    res = a_ads1118_spi_write(handle, conf);
    if (res != 0) { handle->debug_print("ads1118: write config failed.\n"); return 1; }

    return 0;
}

uint8_t ads1118_get_mode(ads1118_handle_t *handle, ads1118_mode_t *mode)
{
    uint8_t  res;
    uint16_t conf;

    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    res = a_ads1118_spi_read(handle, &conf);
    if (res != 0) { handle->debug_print("ads1118: read config failed.\n"); return 1; }

    *mode = (ads1118_mode_t)((conf >> 4) & 0x01);
    return 0;
}

uint8_t ads1118_set_dout_pull_up(ads1118_handle_t *handle, ads1118_bool_t enable)
{
    uint8_t  res;
    uint16_t conf;

    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    res = a_ads1118_spi_read(handle, &conf);
    if (res != 0) { handle->debug_print("ads1118: read config failed.\n"); return 1; }

    conf &= ~(1U << 3);
    conf |=  (uint16_t)(enable & 0x01U) << 3;
    conf &= ~(3U << 1);
    conf |=  COMMAND_VALID;

    res = a_ads1118_spi_write(handle, conf);
    if (res != 0) { handle->debug_print("ads1118: write config failed.\n"); return 1; }

    return 0;
}

uint8_t ads1118_get_dout_pull_up(ads1118_handle_t *handle, ads1118_bool_t *enable)
{
    uint8_t  res;
    uint16_t conf;

    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    res = a_ads1118_spi_read(handle, &conf);
    if (res != 0) { handle->debug_print("ads1118: read config failed.\n"); return 1; }

    *enable = (ads1118_bool_t)((conf >> 3) & 0x01);
    return 0;
}

uint8_t ads1118_single_read(ads1118_handle_t *handle, int16_t *raw, float *v)
{
    uint8_t  res;
    uint8_t  range;
    uint8_t  rate;
    uint16_t conf;

    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    res = a_ads1118_spi_read(handle, &conf);
    if (res != 0) { handle->debug_print("ads1118: read config failed.\n"); return 1; }

    range = (uint8_t)((conf >> 9) & 0x07);
    conf &= ~(1U << 8);
    conf |=  (1U << 8);    /* single-shot mode */
    conf |=  (1U << 15);   /* start conversion */

    res = a_ads1118_spi_write(handle, conf);
    if (res != 0) { handle->debug_print("ads1118: write config failed.\n"); return 1; }

    /* Wait for conversion to complete based on sample rate */
    rate = (uint8_t)((conf >> 5) & 0x07);
    switch (rate)
    {
        case 0:  handle->delay_ms(138); break;   /* 8 SPS   */
        case 1:  handle->delay_ms(69);  break;   /* 16 SPS  */
        case 2:  handle->delay_ms(35);  break;   /* 32 SPS  */
        case 3:  handle->delay_ms(18);  break;   /* 64 SPS  */
        case 4:  handle->delay_ms(9);   break;   /* 128 SPS */
        case 5:  handle->delay_ms(5);   break;   /* 250 SPS */
        case 6:  handle->delay_ms(3);   break;   /* 475 SPS */
        case 7:  handle->delay_ms(2);   break;   /* 860 SPS */
        default: handle->delay_ms(200); break;
    }

    res = a_ads1118_spi_read_data(handle, raw);
    if (res != 0) { handle->debug_print("ads1118: single read failed.\n"); return 1; }

    /* Convert raw value to voltage */
    if      (range == ADS1118_RANGE_6P144V) { *v = (float)(*raw) * 6.144f  / 32768.0f; }
    else if (range == ADS1118_RANGE_4P096V) { *v = (float)(*raw) * 4.096f  / 32768.0f; }
    else if (range == ADS1118_RANGE_2P048V) { *v = (float)(*raw) * 2.048f  / 32768.0f; }
    else if (range == ADS1118_RANGE_1P024V) { *v = (float)(*raw) * 1.024f  / 32768.0f; }
    else if (range == ADS1118_RANGE_0P512V) { *v = (float)(*raw) * 0.512f  / 32768.0f; }
    else if (range == ADS1118_RANGE_0P256V) { *v = (float)(*raw) * 0.256f  / 32768.0f; }
    else
    {
        handle->debug_print("ads1118: range is invalid (conf=0x%04X range=%u).\n",
                            (unsigned)conf, (unsigned)range);
        return 1;
    }

    return 0;
}

uint8_t ads1118_start_continuous_read(ads1118_handle_t *handle)
{
    uint8_t  res;
    uint16_t conf;

    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    res = a_ads1118_spi_read(handle, &conf);
    if (res != 0) { handle->debug_print("ads1118: read config failed.\n"); return 1; }

    conf &= ~(0x01U << 8);   /* clear single-shot bit -> continuous mode */

    res = a_ads1118_spi_write(handle, conf);
    if (res != 0) { handle->debug_print("ads1118: write config failed.\n"); return 1; }

    return 0;
}

uint8_t ads1118_stop_continuous_read(ads1118_handle_t *handle)
{
    uint8_t  res;
    uint16_t conf;

    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    res = a_ads1118_spi_read(handle, &conf);
    if (res != 0) { handle->debug_print("ads1118: read config failed.\n"); return 1; }

    conf &= ~(0x01U << 8);
    conf |=  (0x01U << 8);   /* single-shot / power-down */

    res = a_ads1118_spi_write(handle, conf);
    if (res != 0) { handle->debug_print("ads1118: write config failed.\n"); return 1; }

    return 0;
}

uint8_t ads1118_continuous_read(ads1118_handle_t *handle, int16_t *raw, float *v)
{
    uint8_t  res;
    uint8_t  range;
    uint16_t conf;

    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    res = a_ads1118_spi_read(handle, &conf);
    if (res != 0) { handle->debug_print("ads1118: read config failed.\n"); return 1; }

    range = (uint8_t)((conf >> 9) & 0x07);

    res = a_ads1118_spi_read_data(handle, raw);
    if (res != 0) { handle->debug_print("ads1118: continuous read failed.\n"); return 1; }

    if      (range == ADS1118_RANGE_6P144V) { *v = (float)(*raw) * 6.144f  / 32768.0f; }
    else if (range == ADS1118_RANGE_4P096V) { *v = (float)(*raw) * 4.096f  / 32768.0f; }
    else if (range == ADS1118_RANGE_2P048V) { *v = (float)(*raw) * 2.048f  / 32768.0f; }
    else if (range == ADS1118_RANGE_1P024V) { *v = (float)(*raw) * 1.024f  / 32768.0f; }
    else if (range == ADS1118_RANGE_0P512V) { *v = (float)(*raw) * 0.512f  / 32768.0f; }
    else if (range == ADS1118_RANGE_0P256V) { *v = (float)(*raw) * 0.256f  / 32768.0f; }
    else
    {
        handle->debug_print("ads1118: range is invalid.\n");
        return 1;
    }

    return 0;
}

uint8_t ads1118_temperature_convert(ads1118_handle_t *handle, int16_t raw, float *deg)
{
    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    *deg = (float)(raw) * 0.03125f / 4.0f;
    return 0;
}

uint8_t ads1118_transmit(ads1118_handle_t *handle, uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (handle == NULL)      { return 2; }
    if (handle->inited != 1) { return 3; }

    if (handle->spi_transmit(tx, rx, len) != 0)
    {
        return 1;
    }
    return 0;
}

uint8_t ads1118_info(ads1118_info_t *info)
{
    if (info == NULL) { return 2; }

    memset(info, 0, sizeof(ads1118_info_t));
    strncpy(info->chip_name,         CHIP_NAME,         31);
    strncpy(info->manufacturer_name, MANUFACTURER_NAME, 31);
    strncpy(info->interface,         "SPI",              7);
    info->supply_voltage_min_v = SUPPLY_VOLTAGE_MIN;
    info->supply_voltage_max_v = SUPPLY_VOLTAGE_MAX;
    info->max_current_ma       = MAX_CURRENT;
    info->temperature_min      = TEMPERATURE_MIN;
    info->temperature_max      = TEMPERATURE_MAX;
    info->driver_version       = DRIVER_VERSION;

    return 0;
}
