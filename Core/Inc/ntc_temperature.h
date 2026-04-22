/**
 ******************************************************************************
 * @file      ntc_temperature.h
 * @brief     NTC 10K thermistor temperature calculation
 *
 * Circuit (from schematic):
 *   3.3V_SENSOR_safe
 *        |
 *      R34 (10kΩ)
 *        |
 *        +-----> AIN0 (ADS1118)
 *        |
 *      NTC (10kΩ @ 25°C)
 *        |
 *       GND
 *
 * The voltage at AIN0:  V = VCC * R_ntc / (R_series + R_ntc)
 * NTC resistance:       R_ntc = R_series * V / (VCC - V)
 * Temperature (B-param): T = 1 / (1/T0 + (1/B) * ln(R_ntc/R0))
 ******************************************************************************
 */

#ifndef NTC_TEMPERATURE_H
#define NTC_TEMPERATURE_H

#ifdef __cplusplus
extern "C" {
#endif

/* NTC parameters — adjust B to match your specific NTC datasheet -----------*/
#define NTC_B_CONSTANT     3950.0f    /**< B-constant in Kelvin              */
#define NTC_R0_OHM         10000.0f   /**< Nominal resistance at T0 (10 kΩ) */
#define NTC_T0_KELVIN      298.15f    /**< T0 = 25 °C in Kelvin             */
#define NTC_R_SERIES_OHM   10000.0f   /**< Series resistor R34 (10 kΩ)      */
#define NTC_VCC_V          3.3f       /**< Supply voltage (3.3V_SENSOR_safe) */

/* Single-point calibration offset — measure actual temp and adjust ----------
 * Example: if code reads 55.0 C but actual is 25.0 C, set offset to -30.0f  */
#define NTC_TEMP_OFFSET_C  0.0f       /**< Calibration offset in °C          */

/**
 * @brief  Convert ADS1118 voltage reading to NTC temperature.
 * @param  voltage_v  Voltage measured at AIN0 in volts.
 * @param  temp_c     Output: calculated temperature in degrees Celsius.
 * @return 0 on success, 1 if voltage is out of valid range.
 */
int NTC_VoltageToTemperature(float voltage_v, float *temp_c);

#ifdef __cplusplus
}
#endif

#endif /* NTC_TEMPERATURE_H */
