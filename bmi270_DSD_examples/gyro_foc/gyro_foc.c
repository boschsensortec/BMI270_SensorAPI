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
#include <string.h>
#include <math.h>

#include "bmi270_dsd.h"
#include "common.h"
#include "coines.h"

/******************************************************************************/
/*!                   Macro Definitions                                       */

/* Pi value used in converting gyro degrees per second to radian per second */
#define PI                 (3.14f)

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
 *  @param[in,out] bmi                 : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t perform_gyro_foc_test(struct bmi2_dev *bmi);

/*!
 *  @brief This internal API is to determind if gyro FOC data is within defined limits
 *
 *  @param[in,out] bmi                 : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t verify_gyro_foc_data(struct bmi2_dev *bmi);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] gyro_lsb  : LSB from each axis.
 *  @param[in] dps_range : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *  @param[in] gyro_data : Structure instance to store converted gyro data.
 *
 *  @return void.
 */
static void lsb_to_dps(int16_t gyro_lsb, float dps_range, uint8_t bit_width,
                       struct bmi2_gyro_processed_data *gyro_data);

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Sensor initialization configuration. */
    struct bmi2_dev dev;

    uint8_t try = 0, j;
    int8_t rslt;

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&dev, BMI2_SPI_INTF);
    bmi2_error_codes_print_result(rslt);

    printf("Functional test for gyro foc start..\n");

    /* Perform FOC for different ranges */
    for (j = 0; j < 2; j++)
    {
        try = 0;

        if (j == 0)
        {
            printf("Shake the sensor and press 5\n");
        }
        else if (j > 0)
        {
            printf("Keep sensor stable in right position and press 5\n");
        }

        for (;;)
        {
            scanf("%hu", (short unsigned int *)&try);
            if (try == 5)
            {
                break;
            }
        }

        if (rslt == BMI2_OK)
        {
            rslt = perform_gyro_foc_test(&dev);
            bmi2_error_codes_print_result(rslt);

            if (j == 0)
            {
                if (rslt == BMI2_E_OUT_OF_RANGE)
                {
                    printf("\n#########   Valid input - sensor is shaking   #########\n\n");
                    printf("rslt is BMI2_E_OUT_OF_RANGE\n");
                }
                else if (rslt == BMI2_OK)
                {
                    printf("\n#########   Invalid input - sensor is not shaking   #########\n\n");
                    printf("rslt is BMI2_OK\n");

                    break;
                }
            }
            else if (j > 0)
            {
                if (rslt == BMI2_E_OUT_OF_RANGE)
                {
                    printf("\n#########   Invalid input - Sensor is shaking   #########\n\n");
                    printf("rslt is BMI2_E_OUT_OF_RANGE\n");

                    break;
                }
                else if (rslt == BMI2_OK)
                {
                    printf("\n#########   Valid input - Sensor is not shaking   #########\n\n");
                    printf("rslt is BMI2_OK\n");
                }
            }
        }

        rslt = 0;
    }

    bmi2_coines_deinit();

    return rslt;
}

static int8_t verify_gyro_foc_data(struct bmi2_dev *bmi)
{
    int8_t rslt;
    struct bmi2_sens_axes_data gyr_foc_data[GYRO_SAMPLE_COUNT] = { { 0 } };
    struct temp_axes_val temp_foc_data = { 0 };
    struct bmi2_sens_axes_data avg_foc_data = { 0 };
    struct bmi2_sens_data sensor_data = { { 0 } };

    /* Variable that stores converted gyro data*/
    struct bmi2_gyro_processed_data gyro_data_x;
    struct bmi2_gyro_processed_data gyro_data_y;
    struct bmi2_gyro_processed_data gyro_data_z;
    uint8_t i;

    printf("\n%10s %12s %12s %12s %12s %12s %12s %12s %12s %12s %12s %12s %12s\n",
           "SENS_TIME",
           "GYRO_LSB_X",
           "GYRO_LSB_Y",
           "GYRO_LSB_Z",
           "GYRO_X",
           "GYRO_Y",
           "GYRO_Z",
           "GYRO_DPS_X",
           "GYRO_DPS_Y",
           "GYRO_DPS_Z",
           "GYRO_RPS_X",
           "GYRO_RPS_Y",
           "GYRO_RPS_Z");

    /* Read gyroscope values before/after FOC */
    for (i = 0; i < GYRO_SAMPLE_COUNT; i++)
    {
        for (;;)
        {
            rslt = bmi2_get_sensor_data(&sensor_data, bmi);
            bmi2_error_codes_print_result(rslt);

            if ((rslt == BMI2_OK) && (sensor_data.status & BMI2_DRDY_GYR))
            {
                memcpy(&gyr_foc_data[i], &sensor_data.gyr, sizeof(struct bmi2_sens_axes_data));

                /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                lsb_to_dps(gyr_foc_data[i].x, BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000), bmi->resolution, &gyro_data_x);
                lsb_to_dps(gyr_foc_data[i].y, BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000), bmi->resolution, &gyro_data_y);
                lsb_to_dps(gyr_foc_data[i].z, BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000), bmi->resolution, &gyro_data_z);
                printf(
                    #ifndef PC
                    "%10lu %12d %12d %12d %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f\n",
                    #else
                    "%10u %12d %12d %12d %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f\n",
                    #endif
                    sensor_data.sens_time,
                    gyr_foc_data[i].x,
                    gyr_foc_data[i].y,
                    gyr_foc_data[i].z,
                    gyro_data_x.gyro_raw,
                    gyro_data_y.gyro_raw,
                    gyro_data_z.gyro_raw,
                    gyro_data_x.gyro_dps,
                    gyro_data_y.gyro_dps,
                    gyro_data_z.gyro_dps,
                    gyro_data_x.gyro_rps,
                    gyro_data_y.gyro_rps,
                    gyro_data_z.gyro_rps);

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

    lsb_to_dps(avg_foc_data.x, BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000), bmi->resolution, &gyro_data_x);
    lsb_to_dps(avg_foc_data.y, BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000), bmi->resolution, &gyro_data_y);
    lsb_to_dps(avg_foc_data.z, BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000), bmi->resolution, &gyro_data_z);

    printf("# Average GYRO Axes FOC\n");
    printf("\nAVG_GYRO_X_DPS\t AVG_GYRO_Y_DPS\t AVG_GYRO_Z_DPS\n\n");
    printf("%+9.3f\t %+9.3f\t %+9.3f\n", gyro_data_x.gyro_dps, gyro_data_y.gyro_dps, gyro_data_z.gyro_dps);

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
static int8_t perform_gyro_foc_test(struct bmi2_dev *bmi)
{
    int8_t rslt;
    struct bmi2_sens_config config = { 0 };

    uint8_t sens_list = BMI2_GYRO;

    /* Initialize the sensor */
    printf("Uploading configuration file\n");
    rslt = bmi270_dsd_init(bmi);
    bmi2_error_codes_print_result(rslt);

    printf("Configuration file uploaded\n");
    printf("Chip ID:0x%x\n", bmi->chip_id);

    /* Configure the type of feature. */
    config.type = BMI2_GYRO;

    rslt = bmi2_get_sensor_config(&config, BMI2_N_SENSE_COUNT_1, bmi);
    bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT2, bmi);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        printf("\nInterrupt Pin Configurations:\n");
        printf("Interrupt Signal:\t %s\n", enum_to_string(BMI2_DRDY_INT));
        printf("Interrupt GPIO:\t %s\n", enum_to_string(BMI2_INT2));
    }

    /* Set Output Data Rate to 25Hz */
    config.cfg.gyr.odr = BMI2_GYR_ODR_25HZ;

    /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
    config.cfg.gyr.range = BMI2_GYR_RANGE_2000;

    /* Set Gyroscope bandwidth to Normal mode */
    config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

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
    rslt = bmi2_set_sensor_config(&config, BMI2_N_SENSE_COUNT_1, bmi);
    bmi2_error_codes_print_result(rslt);

    printf("\nGyro Configurations\n");
    printf("ODR:\t %s\n", enum_to_string(BMI2_GYR_ODR_25HZ));
    printf("Range:\t %s\n", enum_to_string(BMI2_GYR_RANGE_2000));
    printf("Bandwidth:\t %s\n", enum_to_string(BMI2_GYR_NORMAL_MODE));
    printf("Filter performance:\t %s\n", enum_to_string(BMI2_PERF_OPT_MODE));
    printf("Noise performance:\t %s\n", enum_to_string(BMI2_PERF_OPT_MODE));
    printf("Sensor Resolution:%u\n", bmi->resolution);

    /* NOTE:
     * Gyro enable must be done after setting configurations
     */
    rslt = bmi2_sensor_enable(&sens_list, BMI2_N_SENSE_COUNT_1, bmi);
    bmi2_error_codes_print_result(rslt);

    printf("\n# Before GYRO FOC\n");

    rslt = verify_gyro_foc_data(bmi);
    bmi2_error_codes_print_result(rslt);

    printf("\n\n########## Perform GYRO FOC ##########\n\n");

    /* Perform gyroscope FOC */
    rslt = bmi2_perform_gyro_foc(bmi);
    bmi2_error_codes_print_result(rslt);

    /* Provide delay after performing FOC */
    bmi->delay_us(40000, bmi->intf_ptr);

    if (rslt == BMI2_OK)
    {
        printf("\n# After GYRO FOC\n");

        rslt = verify_gyro_foc_data(bmi);
    }

    return rslt;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static void lsb_to_dps(int16_t gyro_lsb, float dps_range, uint8_t bit_width, struct bmi2_gyro_processed_data *gyro_data)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / power));

    gyro_data->gyro_raw = (float)gyro_lsb / half_scale;
    gyro_data->gyro_dps = gyro_data->gyro_raw * dps_range;
    gyro_data->gyro_rps = (gyro_data->gyro_dps * PI) / 180;
}
