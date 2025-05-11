/**
 ******************************************************************************
 * @file           : TMP117_STM32_HAL_lib.c
 * @brief          : Library for TMP117 sensor using STM32 HAL.
 ******************************************************************************
 *
 * ## Overview
 * This library provides functions to interface with the TMP117 temperature
 * sensor using STM32 HAL. It includes functions to read raw temperature data
 * and convert it into degrees Celsius with integer and fractional components.
 *
 * ## Key Components
 * - **I²C Communication**:
 *   - `HAL_I2C_Mem_Read` is used to read sensor data over I²C.
 * - **Temperature Conversion**:
 *   - Raw data from the TMP117 is converted into Celsius using the scaling
 *     factor defined in the TMP117 datasheet (\(2^{-7}\)).
 *
 * ## Notes
 * - Ensure the TMP117 is connected to the correct I²C bus with the appropriate
 *   I²C address.
 * - This library assumes the HAL I²C driver is properly configured.
 * - Refer to the TMP117 datasheet for detailed information about the device.
 *
 ******************************************************************************
 */

#include "TMP117_STM32_HAL_lib.h"

/**
 * @brief Reads raw temperature data from the TMP117 sensor.
 * @details Uses the STM32 HAL function `HAL_I2C_Mem_Read` to perform an I²C
 *          memory read operation. The TMP117 sensor address and register are
 *          specified, and the raw temperature data is read into a buffer.
 * @param STM32_I2C_handle: Handle to the I²C peripheral used for communication.
 * @param TMP117_address: I²C address of the TMP117 sensor (7-bit format).
 * @param TMP117_raw_temp: Pointer to a 2-byte buffer where the raw temperature data will be stored.
 * @retval HAL_StatusTypeDef: HAL_OK if the operation was successful, or an error status if it failed.
 */
HAL_StatusTypeDef TMP117_read_raw_temp_value(I2C_HandleTypeDef STM32_I2C_handle, uint8_t TMP117_address, uint8_t *TMP117_raw_temp)
{
	HAL_StatusTypeDef I2C_transfer_status; // Variable to store the status of the I²C read operation

	// Perform an I²C memory read to get 2 bytes of raw temperature data from the TMP117
	I2C_transfer_status = HAL_I2C_Mem_Read(&STM32_I2C_handle, (TMP117_address << 1), TMP117_REG_ADD, 1, TMP117_raw_temp, 2, 1000);

	return I2C_transfer_status; // Return the status of the I²C transfer
}

/**
 * @brief Converts raw temperature data from the TMP117 sensor into Celsius.
 * @details The raw data from the TMP117 is a signed 16-bit value scaled by \(2^{-7}\),
 *          representing the temperature in degrees Celsius. This function separates
 *          the integer and fractional parts of the temperature for easy display.
 * @param TMP117_raw_temp: Pointer to a 2-byte buffer containing the raw temperature data.
 * @param temp_integer: Pointer to an `int16_t` where the integer part of the temperature will be stored.
 * @param temp_fraction: Pointer to an `int16_t` where the fractional part (scaled to 2 decimal places) will be stored.
 * @retval None
 */
void TMP117_conv_raw_temp_celsius(uint8_t *TMP117_raw_temp, int16_t *temp_integer, int16_t *temp_fraction)
{
	// Combine the two bytes into a signed 16-bit raw temperature value and scale to Celsius
	float temp_calc = ((int16_t)((TMP117_raw_temp[0] << 8) | TMP117_raw_temp[1])) / 128.0f;

	// Extract the integer and fractional parts of the temperature
	*temp_integer = (int16_t)temp_calc; // Integer part
	*temp_fraction = (int16_t)((temp_calc - *temp_integer) * 100); // Fractional part scaled to 2 decimal places

	// Ensure the fractional part is positive
	if ((*temp_fraction) < 0) {
		*temp_fraction = -(*temp_fraction);
	}
}
