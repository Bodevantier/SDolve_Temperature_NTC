/**
 ******************************************************************************
 * @file      ntc_temperature.c
 * @brief     NTC 10K thermistor temperature calculation
 ******************************************************************************
 */

#include "ntc_temperature.h"
#include <math.h>

/**
 * @brief  Convert ADS1118 voltage reading to NTC temperature.
 *
 * Uses the B-parameter (two-point) Steinhart-Hart approximation:
 *   R_ntc = R_series * V / (VCC - V)
 *   T [K] = 1 / ( 1/T0 + (1/B) * ln(R_ntc / R0) )
 *   T [°C] = T [K] - 273.15
 *
 * @param  voltage_v  Voltage at AIN0 in volts.
 * @param  temp_c     Output temperature in °C.
 * @return 0 on success, 1 if voltage is out of valid range.
 */
int NTC_VoltageToTemperature(float voltage_v, float *temp_c)
{
    float r_ntc;
    float t_kelvin;

    /* Guard against division by zero at the supply rails */
    if (voltage_v <= 0.0f || voltage_v >= NTC_VCC_V)
    {
        *temp_c = -273.15f;
        return 1;
    }

    /* Calculate NTC resistance from voltage divider */
    r_ntc = NTC_R_SERIES_OHM * voltage_v / (NTC_VCC_V - voltage_v);

    /* Steinhart-Hart B-parameter equation */
    t_kelvin = 1.0f / (1.0f / NTC_T0_KELVIN + (1.0f / NTC_B_CONSTANT) * logf(r_ntc / NTC_R0_OHM));

    *temp_c = t_kelvin - 273.15f + NTC_TEMP_OFFSET_C;
    return 0;
}
