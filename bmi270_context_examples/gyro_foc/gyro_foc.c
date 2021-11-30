/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bmi270_context.h"
#include "common.h"
#include "coines.h"

/******************************************************************************/
/*!                   Macro Definitions                                       */

#define GYRO_SAMPLE_COUNT  UINT8_C(100)

/******************************************************************************/
/*!         Global Variable Declaration                                       */

/* Structure to store temporary axes data values */
struct temp_axes_val
{
    /* X data */
    int32_t x;

    /* Y data */
    int32_t y;

    /* Z data */
    int32_t z;
};

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is to perform gyro FOC
 *
 *  @param[in,out] dev                 : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t perform_gyro_foc_test(struct bmi2_dev *bmi2_dev);

/*!
 *  @brief This internal API is to determind if gyro FOC data is within defined limits
 *
 *  @param[in,out] dev                 : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t verify_gyro_foc_data(struct bmi2_dev *bmi2_dev);

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Sensor initialization configuration. */
    struct bmi2_dev dev;

    uint8_t try = 0, j = 0;
    int8_t rslt;
    uint8_t sens_list = BMI2_GYRO;

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&dev, BMI2_I2C_INTF);
    bmi2_error_codes_print_result(rslt);

    printf("Functional test for gyro foc start..\n");

    /* Perform FOC for different ranges */
    for (j = 0; j < 2; j++)
    {
        rslt = 0;
        try = 0;

        if (j == 0)
        {
            printf("Shake the sensor and press 5\n");
        }
        else if (j > 0)
        {
            printf("Keep sensor stable in right position and press 5\n");
        }

        while (1)
        {
            scanf("%hu", (short unsigned int *)&try);
            if (try == 5)
            {
                break;
            }
        }

        /* Initialize the sensor */
        rslt = bmi270_context_init(&dev);
        bmi2_error_codes_print_result(rslt);

        /* Enable gyroscope */
        rslt = bmi270_context_sensor_enable(&sens_list, 1, &dev);
        bmi2_error_codes_print_result(rslt);

        rslt = perform_gyro_foc_test(&dev);

        if ((j == 0) && (rslt == BMI2_E_OUT_OF_RANGE))
        {
            printf("\n#########   Valid input - sensor is shaking   #########\n\n");
            if (rslt != BMI2_E_OUT_OF_RANGE)
            {
                printf("rslt != BMI2_E_OUT_OF_RANGE\n");
                break;
            }
        }
        else if ((j > 0) && (rslt == BMI2_OK))
        {
            printf("\n#########   Valid input - Sensor is not shaking   #########\n\n");
            bmi2_error_codes_print_result(rslt);
        }
        else if ((j == 0) && (rslt == BMI2_OK))
        {
            printf("\n#########   Invalid input - sensor is not shaking   #########\n\n");
            if (rslt == BMI2_OK)
            {
                printf("rslt == BMI2_OK\n");
                break;
            }
        }
        else if ((j > 0) && (rslt == BMI2_E_OUT_OF_RANGE))
        {
            printf("\n#########   Invalid input - Sensor is shaking   #########\n\n");
            if (rslt == BMI2_E_OUT_OF_RANGE)
            {
                printf("rslt == BMI2_E_OUT_OF_RANGE\n");
                break;
            }
        }
        else if (rslt == BMI2_E_OUT_OF_RANGE)
        {
            bmi2_error_codes_print_result(rslt);
        }
    }

    bmi2_coines_deinit();

    return rslt;
}

static int8_t verify_gyro_foc_data(struct bmi2_dev *bmi2_dev)
{
    int8_t rslt;
    struct bmi2_sens_axes_data gyr_foc_data[GYRO_SAMPLE_COUNT] = { { 0 } };
    struct temp_axes_val temp_foc_data = { 0 };
    struct bmi2_sens_axes_data avg_foc_data = { 0 };
    struct bmi2_sens_data sensor_data = { { 0 } };
    uint16_t drdy_status = 0;
    uint8_t i;

    /* Read gyroscope values before/after FOC */
    for (i = 0; i < GYRO_SAMPLE_COUNT; i++)
    {
        while (1)
        {
            /* To get the data ready interrupt status of gyro. */
            rslt = bmi2_get_int_status(&drdy_status, bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            /* Read gyroscope data based on data ready interrupt */
            if ((rslt == BMI2_OK) && (drdy_status & BMI2_GYR_DRDY_INT_MASK))
            {
                rslt = bmi2_get_sensor_data(&sensor_data, bmi2_dev);
                bmi2_error_codes_print_result(rslt);

                memcpy(&gyr_foc_data[i], &sensor_data.gyr, sizeof(struct bmi2_sens_axes_data));

                printf("X[%d] = %5d,  Y[%d] = %5d,  Z[%d] = %5d\n",
                       i,
                       gyr_foc_data[i].x,
                       i,
                       gyr_foc_data[i].y,
                       i,
                       gyr_foc_data[i].z);

                temp_foc_data.x += gyr_foc_data[i].x;
                temp_foc_data.y += gyr_foc_data[i].y;
                temp_foc_data.z += gyr_foc_data[i].z;
                break;
            }
        }
    }

    /* Taking average values to calculate percentage deviation */
    avg_foc_data.x = (int16_t)(temp_foc_data.x / GYRO_SAMPLE_COUNT);
    avg_foc_data.y = (int16_t)(temp_foc_data.y / GYRO_SAMPLE_COUNT);
    avg_foc_data.z = (int16_t)(temp_foc_data.z / GYRO_SAMPLE_COUNT);

    printf("# Average GYRO Axes FOC\n");

    printf("\n# Avg_X = %5d \t Avg_Y = %5d \t Avg_Z = %5d\n", avg_foc_data.x, avg_foc_data.y, avg_foc_data.z);

    /* The measurement result should be 0 +/- 1dps */
    if (((avg_foc_data.x >= -(BMI2_GYRO_FOC_2000_DPS_REF)) && (avg_foc_data.x <= BMI2_GYRO_FOC_2000_DPS_REF)) &&
        ((avg_foc_data.y >= -(BMI2_GYRO_FOC_2000_DPS_REF)) && (avg_foc_data.y <= (BMI2_GYRO_FOC_2000_DPS_REF))) &&
        ((avg_foc_data.z >= -(BMI2_GYRO_FOC_2000_DPS_REF)) && (avg_foc_data.z <= BMI2_GYRO_FOC_2000_DPS_REF)))
    {
        printf("\n### Gyro Axes data within range ###\n");
        rslt = BMI2_OK;
    }
    else
    {
        printf("\n### Gyro Axes data out of range ###\n");
        rslt = BMI2_E_OUT_OF_RANGE;
    }

    return rslt;
}

/* Perform FOC for different ranges*/
static int8_t perform_gyro_foc_test(struct bmi2_dev *bmi2_dev)
{
    int8_t rslt;
    struct bmi2_sens_config config = { 0 };

    uint8_t sens_list = BMI2_GYRO;

    /* Initialize the sensor */
    rslt = bmi270_context_init(bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Configure the type of feature. */
    config.type = BMI2_GYRO;

    rslt = bmi270_context_get_sensor_config(&config, 1, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT2, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Set Output Data Rate to 25Hz */
    config.cfg.gyr.odr = BMI2_GYR_ODR_25HZ;

    /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
    config.cfg.gyr.range = BMI2_GYR_RANGE_2000;

    /* Set Gyroscope bandwidth to CIC mode */
    config.cfg.gyr.bwp = BMI2_GYR_CIC_MODE;

    /* Enable/Disable the noise performance mode for precision yaw rate sensing
     * There are two modes
     *  0 -> Ultra low power mode(Default)
     *  1 -> High performance mode
     */
    config.cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

    /* Enable/Disable the filter performance mode where averaging of samples
     * will be done based on above set bandwidth and ODR.
     * There are two modes
     *  0 -> Ultra low power mode
     *  1 -> High performance mode(Default)
     */
    config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    /* Set the gyro configurations. */
    rslt = bmi270_context_set_sensor_config(&config, 1, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* NOTE:
     * Gyro enable must be done after setting configurations
     */
    rslt = bmi270_context_sensor_enable(&sens_list, 1, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    printf("# BWP : %d   ODR : %d   Range : %d\n", config.cfg.gyr.bwp, config.cfg.gyr.odr, config.cfg.gyr.range);

    printf("\n# Before GYRO FOC\n");

    rslt = verify_gyro_foc_data(bmi2_dev);

    printf("\n\n########## Perform GYRO FOC ##########\n\n");

    /* Perform gyroscope FOC */
    rslt = bmi2_perform_gyro_foc(bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Provide delay after performing FOC */
    bmi2_dev->delay_us(40000, bmi2_dev->intf_ptr);

    if (rslt == BMI2_OK)
    {
        printf("\n# After GYRO FOC\n");

        rslt = verify_gyro_foc_data(bmi2_dev);
    }

    return rslt;
}
