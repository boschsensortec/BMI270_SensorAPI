/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
#include <stdio.h>
#include "bmi2_ois_common.h"

/******************************************************************************/
/*!                     Global declaration                                    */

/*! Device address for primary and secondary interface */
static uint8_t ois_dev_addr;

/******************************************************************************/
/*!                        Functions                                          */

/*!
 * @brief Function for initialization of SPI bus.
 */
int8_t bmi2_user_spi_init(void)
{

    /* Implement SPI bus initialization according to the target machine. */
    return 0;
}

/*!
 * @brief This function provides the delay for required time (Microseconds) as per the input provided in some of the
 * APIs.
 */
void user_delay_us(uint32_t period_us, void *intf_ptr)
{
    /* Wait for a period amount of microseconds. */
}

/*!
 * @brief This function is for writing the sensor's registers through SPI bus.
 */
int8_t user_spi_reg_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Write to registers using SPI. Return 0 for a successful execution. */
    return 0;
}

/*!
 * @brief This function is for reading the sensor's registers through SPI bus.
 */
int8_t user_spi_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{

    /* Read from registers using SPI. Return 0 for a successful execution. */
    return 0;
}

/******************************************************************************/
int8_t bmi2_ois_dev_init(struct bmi2_ois_dev *ois_dev)
{
    int8_t rslt;

    if (ois_dev != NULL)
    {
        /* To initialize the user SPI function */
        bmi2_user_spi_init();

        /* SPI interface for OIS */
        /* Chip select IO pin */
        ois_dev_addr = 0;
        ois_dev->ois_read = user_spi_reg_read;
        ois_dev->ois_write = user_spi_reg_write;
        ois_dev->ois_delay_us = user_delay_us;
        ois_dev->intf_ptr = &ois_dev_addr;
    }
    else
    {
        rslt = BMI2_OIS_E_NULL_PTR;
    }

    return rslt;
}

/******************************************************************************/

/*!
 * @brief This internal API prints the execution status
 */
void bmi2_error_codes_print_result(int8_t rslt)
{
    switch (rslt)
    {
        case BMI2_OK:

            /*! Do nothing */
            break;
        case BMI2_OIS_E_NULL_PTR:
            printf("Error [%d] : Null pointer error.\r\n", rslt);
            printf(
                "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
            break;
        case BMI2_OIS_E_COM_FAIL:
            printf("Error [%d] : Communication failure error.\r\n", rslt);
            printf(
                "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
            break;

        case BMI2_OIS_E_INVALID_SENSOR:
            printf(
                "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the available one\r\n",
                rslt);
            break;

        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}
