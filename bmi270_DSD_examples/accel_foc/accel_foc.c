/**\
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "bmi270_dsd.h"
#include "common.h"
#include "coines.h"

/******************************************************************************/
/*!                   Macro Definitions                                       */

#define ACCEL_SAMPLE_COUNT  UINT8_C(100)

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH       (9.80665f)

/******************************************************************************/
/*!         Global Variable Declaration                                       */

/*! Structure to store temporary axes data values */
struct temp_axes_val
{
    /* X data */
    float x;

    /* Y data */
    float y;

    /* Z data */
    float z;
};

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used perform accel foc and determine limits based on range
 *
 *  @param[in] range              : Range of Accel
 *  @param[in] g_value_foc        : Structure instance of bmi2_accel_foc_g_value.
 *  @param[in,out] bmi2_dev       : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t perform_foc_range_test(uint8_t range, struct bmi2_accel_foc_g_value g_value_foc,
                                     struct bmi2_dev *bmi2_dev);

/*!
 *  @brief This internal API is to determine if average accel FOC data is within limits
 *
 *  @param[in] avg_accel_foc_data      : Average Accel FOC value
 *  @param[in] reference               : Reference LSB based on Accel Range
 *  @param[in] foc_sign                : Input sign of performed Accel FOC
 *  @param[in] min_val                 : Minimum acceptable LSB limit
 *  @param[in] max_val                 : Maximum acceptable LSB limit
 *
 *  @return Status of execution.
 */
static int8_t accel_foc_report(float avg_accel_foc_data, float reference, uint8_t foc_sign, float min_val,
                               float max_val);

/*!
 *  @brief This internal API is to collect and verify accel sensor data
 *
 *  @param[in] range                   : Value of Accel range
 *  @param[in] reference               : Reference LSB based on Accel Range
 *  @param[in] matched_axis            : Input Axis to perform Accel FOC
 *  @param[in] foc_sign                : Input sign to perform Accel FOC
 *  @param[in,out] bmi2_dev            : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t verify_accel_foc_data(uint8_t range,
                                    float reference,
                                    int8_t matched_axis,
                                    uint8_t foc_sign,
                                    struct bmi2_dev *bmi2_dev);

/*!
 *  @brief This internal API is to calculate noise level for Accel FOC data
 *
 *  @param[in] matched_axis            : Input Axis to perform accel FOC
 *  @param[in] accel_foc_data          : Array of Accel FOC data
 *  @param[in] avg_accel_foc_data      : Average Accel FOC data
 *
 *  @return Status of execution.
 */
static void calculate_noise(int8_t matched_axis,
                            float g_range,
                            const struct bmi2_sens_axes_data *accel_foc_data,
                            struct temp_axes_val avg_accel_foc,
                            const struct bmi2_dev *dev);

/*!
 *  @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Accel values in meter per second squared.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Sensor initialization configuration. */
    struct bmi2_dev dev;

    uint8_t try = 0, j;
    int8_t rslt;
    uint8_t sens_list = BMI2_ACCEL;
    struct bmi2_sens_config sens_cfg = { 0 };
    uint8_t data = 0, range, input_axis = 0;

    /* Set accel foc axis and it's sign (x, y, z, sign) */
    struct bmi2_accel_foc_g_value g_value_foc = { 0, 0, 0, 0 };

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&dev, BMI2_I2C_INTF);
    bmi2_error_codes_print_result(rslt);

    printf("Functional test for accel foc start..\n\n");

    printf("Choose the axis for accel FOC to be done\n");
    printf("Press '1' to choose X axis\n");
    printf("Press '2' to choose Y axis\n");
    printf("Press '3' to choose Z axis\n");

    printf("Press '4' to choose -X axis\n");
    printf("Press '5' to choose -Y axis\n");
    printf("Press '6' to choose -Z axis\n");

    for (;;)
    {
        scanf("%hu", (short unsigned int *)&input_axis);
        if (input_axis > 0 && input_axis < 7)
        {
            break;
        }
    }

    if (input_axis == 1)
    {
        printf("The chosen input axis for FOC is : X\n");

        g_value_foc.x = 1;
        g_value_foc.y = 0;
        g_value_foc.z = 0;
        g_value_foc.sign = 0;
    }
    else if (input_axis == 2)
    {
        printf("The chosen input axis for FOC is : Y\n");

        g_value_foc.x = 0;
        g_value_foc.y = 1;
        g_value_foc.z = 0;
        g_value_foc.sign = 0;
    }
    else if (input_axis == 3)
    {
        printf("The chosen input axis for FOC is : Z\n");

        g_value_foc.x = 0;
        g_value_foc.y = 0;
        g_value_foc.z = 1;
        g_value_foc.sign = 0;
    }
    else if (input_axis == 4)
    {
        printf("The chosen input axis for FOC is : -X\n");

        g_value_foc.x = 1;
        g_value_foc.y = 0;
        g_value_foc.z = 0;
        g_value_foc.sign = 1;
    }
    else if (input_axis == 5)
    {
        printf("The chosen input axis for FOC is : -Y\n");

        g_value_foc.x = 0;
        g_value_foc.y = 1;
        g_value_foc.z = 0;
        g_value_foc.sign = 1;
    }
    else if (input_axis == 6)
    {
        printf("The chosen input axis for FOC is : -Z\n");

        g_value_foc.x = 0;
        g_value_foc.y = 0;
        g_value_foc.z = 1;
        g_value_foc.sign = 1;
    }

    printf("Confirm your chosen axis and the sensor keeping position are same before doing FOC\n");

    for (j = 0; j < 2; j++)
    {
        try = 0;

        if (j == 1)
        {
            printf("Keep sensor in wrong position and press 5\n");
        }
        else if (j == 0)
        {
            printf("Keep sensor in right position and press 5\n");
        }

        for (;;)
        {
            scanf("%hu", (short unsigned int *)&try);
            if (try == 5)
            {
                break;
            }
        }

        for (range = BMI2_ACC_RANGE_2G; range <= BMI2_ACC_RANGE_16G; range++)
        {
            /****************************************************************/
            /* Initialize by enabling configuration load */
            printf("#########################################################\n\n");

            printf("Uploading configuration file\n");
            rslt = bmi270_dsd_init(&dev);
            bmi2_error_codes_print_result(rslt);

            printf("Configuration file uploaded\n");
            printf("Chip ID :0x%x\n", dev.chip_id);

            sens_cfg.type = BMI2_ACCEL;

            /* Testing with different settings other than the default configurations
             *  Default is : 50Hz ODR and 2g RANGE
             *  Note - Change accel_conf.range for testing in different range values
             */
            sens_cfg.cfg.acc.odr = BMI2_ACC_ODR_50HZ;
            sens_cfg.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

            /****************************************************************/
            sens_cfg.cfg.acc.range = range;

            /* Set the configuration details */
            rslt = bmi2_set_sensor_config(&sens_cfg, 1, &dev);
            bmi2_error_codes_print_result(rslt);

            /* NOTE:
             * Accel enable must be done after setting configurations
             */
            rslt = bmi2_sensor_enable(&sens_list, 1, &dev);
            bmi2_error_codes_print_result(rslt);

            /* Get the configuration details to verify whether the configured values are set */
            rslt = bmi2_get_sensor_config(&sens_cfg, 1, &dev);
            bmi2_error_codes_print_result(rslt);

            printf("\nAccel Configurations:\n");
            printf("ODR:\t %s\n", enum_to_string(BMI2_ACC_ODR_50HZ));
            printf("Bandwidth:\t %s\n", enum_to_string(BMI2_ACC_NORMAL_AVG4));

            /* Perform FOC for different ranges */
            rslt = perform_foc_range_test(range, g_value_foc, &dev);

            if ((j == 1) && (rslt == BMI2_E_OUT_OF_RANGE))
            {
                printf("\n#########   Valid input - Wrong position   #########\n\n");
                bmi2_error_codes_print_result(rslt);
            }
            else if ((j == 0) && (rslt == BMI2_OK))
            {
                printf("\n#########   Valid input - Right position   #########\n\n");
                bmi2_error_codes_print_result(rslt);
            }
            else if ((j == 1) && (rslt == BMI2_OK))
            {
                printf("\n#########   Invalid input - Right position   #########\n\n");
                bmi2_error_codes_print_result(rslt);
            }
            else if ((j == 0) && (rslt == BMI2_E_OUT_OF_RANGE))
            {
                printf("\n#########   Invalid input - Wrong position   #########\n\n");
                bmi2_error_codes_print_result(rslt);
            }
            else if ((j == 0) && (rslt == BMI2_E_OUT_OF_RANGE))
            {
                printf("\n#########   Valid input - Right position   #########\n\n");
                printf("\n#########   Before FOC is better than after FOC   #########\n\n");
                bmi2_error_codes_print_result(rslt);
            }
            else if ((j == 1) && (rslt == BMI2_E_OUT_OF_RANGE))
            {
                printf("\n#########   Invalid input - Right position   #########\n\n");
                printf("\n#########   Before FOC is better than after FOC   #########\n\n");
                bmi2_error_codes_print_result(rslt);
            }
        }

        /* Disable offset compensation */
        rslt = bmi2_get_regs(BMI2_NV_CONF_ADDR, &data, 1, &dev);
        bmi2_error_codes_print_result(rslt);

        data = BMI2_SET_BIT_VAL0(data, BMI2_NV_ACC_OFFSET);

        rslt = bmi2_set_regs(BMI2_NV_CONF_ADDR, &data, 1, &dev);
        bmi2_error_codes_print_result(rslt);

        /* Get the accelerometer configuration details to verify whether it is reverted back to the configured ones */
        rslt = bmi2_get_sensor_config(&sens_cfg, 1, &dev);
        bmi2_error_codes_print_result(rslt);
    }

    bmi2_coines_deinit();

    return rslt;
}

static int8_t accel_foc_report(float avg_accel_foc_data, float reference, uint8_t foc_sign, float min_val,
                               float max_val)
{
    int8_t rslt = BMI2_OK;
    int16_t diff_after = 0;

    if (foc_sign == 0)
    {
        if ((avg_accel_foc_data >= (min_val)) && (avg_accel_foc_data <= (max_val)))
        {
            if (avg_accel_foc_data >= reference)
            {
                diff_after = (int16_t)(avg_accel_foc_data - reference);
            }
            else
            {
                diff_after = (int16_t)(reference - avg_accel_foc_data);
            }

            printf("\n# ********** PASS | Difference = %d **********\n", diff_after);
            printf("\n# Avg_FOC %f in range\n", avg_accel_foc_data);
            rslt = BMI2_OK;
        }
        else
        {
            if (avg_accel_foc_data >= reference)
            {
                diff_after = (int16_t)(avg_accel_foc_data - reference);
            }
            else
            {
                diff_after = (int16_t)(reference - avg_accel_foc_data);
            }

            printf("\n# ********** FAIL | Difference = %d **********\n", diff_after);
            printf("\n# Avg_FOC %f not in range\n", avg_accel_foc_data);
            rslt = BMI2_E_OUT_OF_RANGE;
        }
    }

    if (foc_sign == 1)
    {
        if ((avg_accel_foc_data <= (min_val)) && (avg_accel_foc_data >= (max_val)))
        {
            if (avg_accel_foc_data <= reference)
            {
                diff_after = (int16_t)(avg_accel_foc_data - reference);
            }
            else
            {
                diff_after = (int16_t)(reference - avg_accel_foc_data);
            }

            printf("\n# ********** PASS | Difference = %d **********\n", diff_after);
            printf("\n# Avg_FOC %f in range\n", avg_accel_foc_data);
            rslt = BMI2_OK;
        }
        else
        {
            if (avg_accel_foc_data <= reference)
            {
                diff_after = (int16_t)(avg_accel_foc_data - reference);
            }
            else
            {
                diff_after = (int16_t)(reference - avg_accel_foc_data);
            }

            printf("\n# ********** FAIL | Difference = %d **********\n", diff_after);
            printf("\n# Avg_FOC %f not in range\n", avg_accel_foc_data);
            rslt = BMI2_E_OUT_OF_RANGE;
        }
    }

    return rslt;
}

static void calculate_noise(int8_t matched_axis,
                            float g_range,
                            const struct bmi2_sens_axes_data *accel_foc_data,
                            struct temp_axes_val avg_accel_foc,
                            const struct bmi2_dev *bmi2_dev)
{
    double variance = 0;
    double noise_level;
    uint16_t indx = 0;
    float axis_ms2_val;

    if (matched_axis == 'X')
    {
        for (indx = 0; indx < ACCEL_SAMPLE_COUNT; indx++)
        {
            axis_ms2_val = lsb_to_mps2(accel_foc_data[indx].x, g_range, bmi2_dev->resolution);
            variance += (double)((axis_ms2_val - avg_accel_foc.x) * (axis_ms2_val - avg_accel_foc.x));
        }
    }
    else if (matched_axis == 'Y')
    {
        for (indx = 0; indx < ACCEL_SAMPLE_COUNT; indx++)
        {
            axis_ms2_val = lsb_to_mps2(accel_foc_data[indx].y, g_range, bmi2_dev->resolution);
            variance += (double)((axis_ms2_val - avg_accel_foc.y) * (axis_ms2_val - avg_accel_foc.y));
        }
    }
    else if (matched_axis == 'Z')
    {
        for (indx = 0; indx < ACCEL_SAMPLE_COUNT; indx++)
        {
            axis_ms2_val = lsb_to_mps2(accel_foc_data[indx].z, g_range, bmi2_dev->resolution);
            variance += (double)((axis_ms2_val - avg_accel_foc.z) * (axis_ms2_val - avg_accel_foc.z));
        }
    }

    noise_level = sqrt((double)variance);

    printf("\n# ********** NOISE LEVEL in MS2 = %lf **********\n", noise_level);
}

static int8_t verify_accel_foc_data(uint8_t range,
                                    float reference,
                                    int8_t matched_axis,
                                    uint8_t foc_sign,
                                    struct bmi2_dev *bmi2_dev)
{
    int8_t rslt;
    uint8_t i;
    float xl, yl, zl;
    float xh, yh, zh;
    float min_val = 0;
    float max_val = 0;
    struct bmi2_sens_axes_data accel_foc_data[ACCEL_SAMPLE_COUNT] = { { 0 } };
    struct temp_axes_val temp_foc_data = { 0 };
    struct bmi2_sens_data sensor_data = { { 0 } };
    float acc_x = 0, acc_y = 0, acc_z = 0;
    float g_range;

    /* Setting initial values */
    xl = yl = zl = 9.80665f;
    xh = yh = zh = -9.80665f;

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    switch (range)
    {
        case 0:
            g_range = BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G);
            break;
        case 1:
            g_range = BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_4G);
            break;
        case 2:
            g_range = BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_8G);
            break;
        case 3:
            g_range = BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_16G);
            break;
        default:
            g_range = BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G);
            break;

    }

    printf("Acquisition iteration Count:\t %d\n\n", ACCEL_SAMPLE_COUNT);

    printf("SENS_TIME\t ACC_MS2_X\t ACC_MS2_Y\t ACC_MS2_Z\n\n");

    /* Read accelerometer values before/after FOC */
    for (i = 0; i < ACCEL_SAMPLE_COUNT; i++)
    {
        for (;;)
        {
            rslt = bmi2_get_sensor_data(&sensor_data, bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            if ((rslt == BMI2_OK) && (sensor_data.status & BMI2_DRDY_ACC))
            {
                memcpy(&accel_foc_data[i], &sensor_data.acc, sizeof(struct bmi2_sens_axes_data));

                /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
                acc_x = lsb_to_mps2(accel_foc_data[i].x, g_range, bmi2_dev->resolution);
                acc_y = lsb_to_mps2(accel_foc_data[i].y, g_range, bmi2_dev->resolution);
                acc_z = lsb_to_mps2(accel_foc_data[i].z, g_range, bmi2_dev->resolution);
                #ifndef PC
                printf("%8lu\t %+9.3f\t %+9.3f\t %+9.3f\n", sensor_data.sens_time, acc_x, acc_y, acc_z);
                #else
                printf("%8u\t %+9.3f\t %+9.3f\t %+9.3f\n", sensor_data.sens_time, acc_x, acc_y, acc_z);
                #endif

                if (xl > acc_x)
                {
                    xl = acc_x;
                }

                if (xh < acc_x)
                {
                    xh = acc_x;
                }

                if (yl > acc_y)
                {
                    yl = acc_y;
                }

                if (yh < acc_y)
                {
                    yh = acc_y;
                }

                if (zl > acc_z)
                {
                    zl = acc_z;
                }

                if (zh < acc_z)
                {
                    zh = acc_z;
                }

                temp_foc_data.x += acc_x;
                temp_foc_data.y += acc_y;
                temp_foc_data.z += acc_z;
                break;

            }
        }
    }

    /* Taking average values to calculate percentage deviation */
    temp_foc_data.x = (float)(temp_foc_data.x / ACCEL_SAMPLE_COUNT);
    temp_foc_data.y = (float)(temp_foc_data.y / ACCEL_SAMPLE_COUNT);
    temp_foc_data.z = (float)(temp_foc_data.z / ACCEL_SAMPLE_COUNT);

    printf("********* MIN & MAX VALUES ********\n");

    printf("XL = %5f  YL = %5f  ZL = %5f\n", xl, yl, zl);
    printf("XH = %5f  YH = %5f  ZH = %5f\n", xh, yh, zh);

    printf("***** AVERAGE AFTER FOC *****\n\n");
    printf("AVG_ACC_MS2_X\t AVG_ACC_MS2_Y\t AVG_ACC_MS2_Z\n\n");
    printf("%+9.3f\t %+9.3f\t %+9.3f\n", acc_x, acc_y, acc_z);

    /* Calculate noise level */
    calculate_noise(matched_axis, g_range, accel_foc_data, temp_foc_data, bmi2_dev);

    /* "zero-g offset" of accel is +/- 20 mg for all ranges as per datasheet */
    if (range == 0)
    {
        /* Min and Max limits for Range 2G */
        min_val = lsb_to_mps2((int16_t) BMI2_ACC_2G_MIN_NOISE_LIMIT,
                              BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G),
                              bmi2_dev->resolution);
        max_val = lsb_to_mps2((int16_t) BMI2_ACC_2G_MAX_NOISE_LIMIT,
                              BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G),
                              bmi2_dev->resolution);
    }
    else if (range == 1)
    {
        /* Min and Max limits for Range 4G */
        min_val = lsb_to_mps2((int16_t) BMI2_ACC_4G_MIN_NOISE_LIMIT,
                              BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_4G),
                              bmi2_dev->resolution);
        max_val = lsb_to_mps2((int16_t) BMI2_ACC_4G_MAX_NOISE_LIMIT,
                              BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_4G),
                              bmi2_dev->resolution);
    }
    else if (range == 2)
    {
        /* Min and Max limits for Range 8G */
        min_val = lsb_to_mps2((int16_t) BMI2_ACC_8G_MIN_NOISE_LIMIT,
                              BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_8G),
                              bmi2_dev->resolution);
        max_val = lsb_to_mps2((int16_t) BMI2_ACC_8G_MAX_NOISE_LIMIT,
                              BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_8G),
                              bmi2_dev->resolution);
    }
    else if (range == 3)
    {
        /* Min and Max limits for Range 16G */
        min_val = lsb_to_mps2((int16_t) BMI2_ACC_16G_MIN_NOISE_LIMIT,
                              BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_16G),
                              bmi2_dev->resolution);
        max_val = lsb_to_mps2((int16_t) BMI2_ACC_16G_MAX_NOISE_LIMIT,
                              BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_16G),
                              bmi2_dev->resolution);
    }

    if ((matched_axis == 'X') && (foc_sign == 0))
    {
        rslt = accel_foc_report(temp_foc_data.x, reference, foc_sign, min_val, max_val);

        printf("Range : %d  Avg_FOC-X : %f   Reference : %f   Min_Value : %f  Max_Value : %f\n",
               range,
               temp_foc_data.x,
               reference,
               min_val,
               max_val);
    }
    else if ((matched_axis == 'Y') && (foc_sign == 0))
    {
        rslt = accel_foc_report(temp_foc_data.y, reference, foc_sign, min_val, max_val);

        printf("Range : %d  Avg_FOC-Y : %f   Reference : %f   Min_Value : %f  Max_Value : %f\n",
               range,
               temp_foc_data.y,
               reference,
               min_val,
               max_val);
    }
    else if ((matched_axis == 'Z') && (foc_sign == 0))
    {
        rslt = accel_foc_report(temp_foc_data.z, reference, foc_sign, min_val, max_val);

        printf("Range : %d  Avg_FOC-Z : %f   Reference : %f   Min_Value : %f  Max_Value : %f\n",
               range,
               temp_foc_data.z,
               reference,
               min_val,
               max_val);
    }
    else if ((matched_axis == 'X') && (foc_sign == 1))
    {
        rslt = accel_foc_report(temp_foc_data.x, reference * -1.0f, foc_sign, min_val * -1.0f, max_val * -1.0f);

        printf("Range : %d  Avg_FOC-X : %f   Reference : %f   Min_Value : %f  Max_Value : %f\n",
               range,
               temp_foc_data.x,
               reference * -1.0f,
               min_val * -1.0f,
               max_val * -1.0f);
    }
    else if ((matched_axis == 'Y') && (foc_sign == 1))
    {
        rslt = accel_foc_report(temp_foc_data.y, reference * -1.0f, foc_sign, min_val * -1.0f, max_val * -1.0f);

        printf("Range : %d  Avg_FOC-Y : %f   Reference : %f   Min_Value : %f  Max_Value : %f\n",
               range,
               temp_foc_data.y,
               reference * -1.0f,
               min_val * -1.0f,
               max_val * -1.0f);
    }
    else if ((matched_axis == 'Z') && (foc_sign == 1))
    {
        rslt = accel_foc_report(temp_foc_data.z, reference * -1.0f, foc_sign, min_val * -1.0f, max_val * -1.0f);

        printf("Range : %d  Avg_FOC-Z : %f   Reference : %f  Min_Value : %f  Max_Value : %f\n",
               range,
               temp_foc_data.z,
               reference * -1.0f,
               min_val * -1.0f,
               max_val * -1.0f);
    }

    return rslt;

}

/* Perform FOC for different range and resolutions */
static int8_t perform_foc_range_test(uint8_t range, struct bmi2_accel_foc_g_value g_value_foc,
                                     struct bmi2_dev *bmi2_dev)
{
    int8_t rslt;
    int8_t matched_axis = 0;
    float reference = 0;

    switch (range)
    {
        /* Reference LSB value of 2G */
        case 0:
            reference = lsb_to_mps2((int16_t) BMI2_ACC_FOC_2G_REF,
                                    BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G),
                                    bmi2_dev->resolution);
            break;

        /* Reference LSB value of 4G */
        case 1:
            reference = lsb_to_mps2((int16_t) BMI2_ACC_FOC_4G_REF,
                                    BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_4G),
                                    bmi2_dev->resolution);
            break;

        /* Reference LSB value of 8G */
        case 2:
            reference = lsb_to_mps2((int16_t) BMI2_ACC_FOC_8G_REF,
                                    BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_8G),
                                    bmi2_dev->resolution);
            break;

        /* Reference LSB value of 16G */
        case 3:
            reference = lsb_to_mps2((int16_t) BMI2_ACC_FOC_16G_REF,
                                    BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_16G),
                                    bmi2_dev->resolution);
            break;

        default:
            break;
    }

    if (g_value_foc.x == 1)
    {
        matched_axis = 'X';
    }
    else if (g_value_foc.y == 1)
    {
        matched_axis = 'Y';
    }
    else if (g_value_foc.z == 1)
    {
        matched_axis = 'Z';
    }

    if (g_value_foc.sign == 1)
    {
        printf("MATCHED AXIS : -%c\n", matched_axis);
    }
    else
    {
        printf("MATCHED AXIS :  %c\n", matched_axis);
    }

    printf("\n\n# Before FOC\n");
    rslt = verify_accel_foc_data(range, reference, matched_axis, g_value_foc.sign, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    printf("\n\n######### Perform Accel FOC #########\n\n");

    /* Perform accelerometer FOC */
    rslt = bmi2_perform_accel_foc(&g_value_foc, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Provide delay after performing FOC */
    bmi2_dev->delay_us(40000, bmi2_dev->intf_ptr);

    printf("\n\n# After FOC\n");
    rslt = verify_accel_foc_data(range, reference, matched_axis, g_value_foc.sign, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}
