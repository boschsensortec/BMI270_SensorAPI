/**\
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include <math.h>
#include "bmi270_dsd.h"
#include "common.h"

/******************************************************************************/
/*!                Macro definition                                           */

/* Pi value used in converting gyro degrees per second to radian per second */
#define PI  (3.14f)

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations for gyro.
 *
 *  @param[in] dev       : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_gyro_config(struct bmi2_dev *dev);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Degree per second.
 */
static void lsb_to_dps(int16_t gyro_lsb, float dps_range, uint8_t bit_width,
                       struct bmi2_gyro_processed_data *gyro_data);

/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program. */
int main(void)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to define limit to print gyro data. */
    uint8_t limit = 100;

    uint8_t indx = 1;

    /* Sensor initialization configuration. */
    struct bmi2_dev bmi;

    /* Structure to define type of sensor and their respective data. */
    struct bmi2_sens_data sensor_data = { { 0 } };

    /* Variable that stores converted gyro data*/
    struct bmi2_gyro_processed_data gyro_data_x;
    struct bmi2_gyro_processed_data gyro_data_y;
    struct bmi2_gyro_processed_data gyro_data_z;

    /* Assign gyro sensor to variable. */
    uint8_t sens_list = BMI2_GYRO;

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&bmi, BMI2_I2C_INTF);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmi270_dsd. */
    printf("Uploading configuration file\n");
    rslt = bmi270_dsd_init(&bmi);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        printf("Configuration file uploaded\n");
        printf("Chip ID :0x%x\n", bmi.chip_id);

        /* Gyro configuration settings. */
        rslt = set_gyro_config(&bmi);
        bmi2_error_codes_print_result(rslt);

        if (rslt == BMI2_OK)
        {
            /* NOTE:
             * Gyro enable must be done after setting configurations
             */

            /* Enable the selected sensors. */
            rslt = bmi2_sensor_enable(&sens_list, 1, &bmi);
            bmi2_error_codes_print_result(rslt);

            printf("\nAquisition Iteration Count:\t %d\n\n", limit);
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

            while (indx <= limit)
            {
                rslt = bmi2_get_sensor_data(&sensor_data, &bmi);
                bmi2_error_codes_print_result(rslt);

                if ((rslt == BMI2_OK) && (sensor_data.status & BMI2_DRDY_GYR))
                {
                    /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                    lsb_to_dps(sensor_data.gyr.x, BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000), bmi.resolution, &gyro_data_x);
                    lsb_to_dps(sensor_data.gyr.y, BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000), bmi.resolution, &gyro_data_y);
                    lsb_to_dps(sensor_data.gyr.z, BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000), bmi.resolution, &gyro_data_z);

#ifndef PC

                    printf("%10lu %12d %12d %12d %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f\n",
                           sensor_data.sens_time,
                           sensor_data.gyr.x,
                           sensor_data.gyr.y,
                           sensor_data.gyr.z,
                           gyro_data_x.gyro_raw,
                           gyro_data_y.gyro_raw,
                           gyro_data_z.gyro_raw,
                           gyro_data_x.gyro_dps,
                           gyro_data_y.gyro_dps,
                           gyro_data_z.gyro_dps,
                           gyro_data_x.gyro_rps,
                           gyro_data_y.gyro_rps,
                           gyro_data_z.gyro_rps);
#else
                    printf("%10u %12d %12d %12d %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f\n",
                           sensor_data.sens_time,
                           sensor_data.gyr.x,
                           sensor_data.gyr.y,
                           sensor_data.gyr.z,
                           gyro_data_x.gyro_raw,
                           gyro_data_y.gyro_raw,
                           gyro_data_z.gyro_raw,
                           gyro_data_x.gyro_dps,
                           gyro_data_y.gyro_dps,
                           gyro_data_z.gyro_dps,
                           gyro_data_x.gyro_rps,
                           gyro_data_y.gyro_rps,
                           gyro_data_z.gyro_rps);
#endif

                    indx++;
                }
            }
        }
    }

    bmi2_coines_deinit();

    return rslt;
}

/*!
 *  @brief This internal API is used to set configurations for gyro.
 */
static int8_t set_gyro_config(struct bmi2_dev *dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi2_sens_config config;

    /* Configure the type of feature. */
    config.type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(&config, 1, dev);
    bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT2, dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config.cfg.gyr.odr = BMI2_GYR_ODR_100HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config.cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
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
        rslt = bmi2_set_sensor_config(&config, 1, dev);
        bmi2_error_codes_print_result(rslt);

        printf("*************************************\n");
        printf("Gyro Configurations\n");
        printf("ODR:\t %s\n", enum_to_string(BMI2_GYR_ODR_100HZ));
        printf("Range:\t %s\n", enum_to_string(BMI2_GYR_RANGE_2000));
        printf("Bandwidth:\t %s\n", enum_to_string(BMI2_GYR_NORMAL_MODE));
        printf("Noise performance:\t %s\n", enum_to_string(BMI2_POWER_OPT_MODE));
        printf("Filter performance:\t %s\n", enum_to_string(BMI2_PERF_OPT_MODE));
        printf("Resolution:%u\n", dev->resolution);
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
