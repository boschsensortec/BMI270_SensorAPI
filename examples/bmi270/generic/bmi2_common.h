/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#ifndef __BMI2_COMMON_H__
#define __BMI2_COMMON_H__

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "bmi2.h"

/*!
 *  @brief Function for initialization of I2C bus
 *
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t bmi2_user_i2c_init(void);

/*!
 *  @brief Function for initialization of SPI bus
 *
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t bmi2_user_spi_init(void);

/*!
 *  @brief This function provides the delay for required time (Microseconds) as per the input provided in some of the
 *  APIs.
 *  @param[in] period_us  : The required wait time in microseconds.
 *  @param[in] intf_ptr   : Pointer to interface.
 *  @return void.
 *
 */
void user_delay_us(uint32_t period_us, void *intf_ptr);

/*!
 *  @brief This function provides the delay for required time (Milliseconds) as per the input provided in some of the
 *  APIs.
 *  @param[in] period_ms  : The required wait time in milliseconds.
 *  @return void.
 *
 */
void user_delay_ms(uint32_t period_ms);

/*!
 *  @brief This function is for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data: Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *  @param[in] intf_ptr : Pointer to interface.
 *
 *  @return Status of execution
 *
 *  @retval =  BMI2_INTF_RET_SUCCESS -> Success
 *  @retval != BMI2_INTF_RET_SUCCESS -> Failure
 *
 */
int8_t user_i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief This function is for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *  @param[in] intf_ptr : Pointer to interface.
 *
 *  @return Status of execution
 *
 *  @retval =  BMI2_INTF_RET_SUCCESS -> Success
 *  @retval != BMI2_INTF_RET_SUCCESS -> Failure
 *
 */
int8_t user_i2c_reg_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief This function is for reading the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *  @param[in] intf_ptr : Pointer to interface.
 *
 *  @return Status of execution
 *
 *  @retval =  BMI2_INTF_RET_SUCCESS -> Success
 *  @retval != BMI2_INTF_RET_SUCCESS -> Failure
 *
 */
int8_t user_spi_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief This function is for writing the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose data has to be written.
 *  @param[in] length       : No of bytes to write.
 *  @param[in] intf_ptr     : Pointer to interface.
 *
 *  @return Status of execution
 *
 *  @retval =  BMI2_INTF_RET_SUCCESS -> Success
 *  @retval != BMI2_INTF_RET_SUCCESS -> Failure
 *
 */
int8_t user_spi_reg_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief This function is to select the interface between SPI and I2C.
 *
 *  @param[in] dev : Structure instance of bmi2_dev
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t bmi2_interface_selection(struct bmi2_dev *dev);

/*!
 *  @brief This API is used to print the execution status.
 *
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bmi2_error_codes_print_result(int8_t rslt);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* __BMI2_COMMON_H__ */
