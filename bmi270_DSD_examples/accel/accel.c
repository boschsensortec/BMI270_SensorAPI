/**\
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmi270_dsd.h"
#include "common.h"
#include <math.h>

/******************************************************************************/
/*!                Macro definition                                           */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal API is used to set configurations for accel.
 *
 *  @param[in] bmi       : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_accel_config(struct bmi2_dev *bmi);

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
static void lsb_to_mps2(int16_t accel_lsb,
                        float g_range,
                        uint8_t bit_width,
                        struct bmi2_accel_processed_data *accel_data);

/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program. */
int main(void)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable that store converted Sensor Data from LSB to  (Raw, G-value and m/s2 )  */
    struct bmi2_accel_processed_data accel_data_x;
    struct bmi2_accel_processed_data accel_data_y;
    struct bmi2_accel_processed_data accel_data_z;

    /* Variable to define limit to print accel data. */
    uint8_t limit = 100;

    /* Assign accel sensor to variable. */
    uint8_t sensor_list = BMI2_ACCEL;

    /* Sensor initialization configuration. */
    struct bmi2_dev bmi;

    /* Structure to define type of sensor and their respective data. */
    struct bmi2_sens_data sens_data = { { 0 } };

    uint8_t indx = 1;
    struct bmi2_sens_config config;

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

    printf("Configuration file uploaded\n");
    printf("Chip ID :0x%x\n", bmi.chip_id);

    if (rslt == BMI2_OK)
    {
        /* Accel configuration settings. */
        rslt = set_accel_config(&bmi);
        bmi2_error_codes_print_result(rslt);

        if (rslt == BMI2_OK)
        {
            printf("Acquisition Iteration Count :%d\n", limit);
            printf(
                "SENS_TIME_LSB\t ACC_LSB_X\t ACC_LSB_Y\t ACC_LSB_Z\t ACC_RAW_X\t ACC_RAW_Y\t ACC_RAW_Z\t ACC_G_X\t ACC_G_Y\t ACC_G_Z\t ACC_MS2_X\t ACC_MS2_Y\t ACC_MS2_Z\n");

            /* NOTE:
             * Accel enable must be done after setting configurations
             */
            rslt = bmi2_sensor_enable(&sensor_list, 1, &bmi);
            bmi2_error_codes_print_result(rslt);

            if (rslt == BMI2_OK)
            {
                config.type = BMI2_ACCEL;

                /* Get the accel configurations. */
                rslt = bmi2_get_sensor_config(&config, 1, &bmi);
                bmi2_error_codes_print_result(rslt);

                while (indx <= limit)
                {
                    rslt = bmi2_get_sensor_data(&sens_data, &bmi);
                    bmi2_error_codes_print_result(rslt);

                    if ((rslt == BMI2_OK) && (sens_data.status & BMI2_DRDY_ACC))
                    {
                        /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
                        lsb_to_mps2(sens_data.acc.x,
                                    BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G),
                                    bmi.resolution,
                                    &accel_data_x);
                        lsb_to_mps2(sens_data.acc.y,
                                    BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G),
                                    bmi.resolution,
                                    &accel_data_y);
                        lsb_to_mps2(sens_data.acc.z,
                                    BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G),
                                    bmi.resolution,
                                    &accel_data_z);

                        /* Print the Sensor data */
                        printf(
#ifndef PC
                            "%8lu\t %8d\t %8d\t %8d\t %+9.3f\t %+9.3f\t %+9.3f\t %+9.3f\t %9.3f\t %+9.3f\t %+9.3f\t %+9.3f\t %+9.3f\n",
#else
                            "%8u\t %8d\t %8d\t %8d\t %+9.3f\t %+9.3f\t %+9.3f\t %+8.3f\t %7.3f\t %+7.3f\t %+9.3f\t %+9.3f\t %+9.3f\n",
#endif
                            sens_data.sens_time,
                            sens_data.acc.x,
                            sens_data.acc.y,
                            sens_data.acc.z,
                            accel_data_x.accel_raw,
                            accel_data_y.accel_raw,
                            accel_data_z.accel_raw,
                            accel_data_x.accel_g,
                            accel_data_y.accel_g,
                            accel_data_z.accel_g,
                            accel_data_x.accel_ms2,
                            accel_data_y.accel_ms2,
                            accel_data_z.accel_ms2);

                        indx++;
                    }
                }
            }
        }
    }

    bmi2_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for accel.
 */
static int8_t set_accel_config(struct bmi2_dev *bmi)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer configuration. */
    struct bmi2_sens_config config;

    /* Configure the type of feature. */
    config.type = BMI2_ACCEL;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(&config, 1, bmi);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config.cfg.acc.odr = BMI2_ACC_ODR_200HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config.cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the accel configurations. */
        rslt = bmi2_set_sensor_config(&config, 1, bmi);
        bmi2_error_codes_print_result(rslt);

        /* Map data ready interrupt to interrupt pin. */
        rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi);
        bmi2_error_codes_print_result(rslt);
        printf("*************************************\n");
        printf("Accel Configurations:\n");
        printf("ODR:\t %s\n", enum_to_string(BMI2_ACC_ODR_200HZ));
        printf("Range:\t %s\n", enum_to_string(BMI2_ACC_RANGE_2G));
        printf("Bandwidth:\t %s\n", enum_to_string(BMI2_ACC_NORMAL_AVG4));
        printf("Filter performance:\t %s\n", enum_to_string(BMI2_PERF_OPT_MODE));
        printf("Resolution:%u\n", bmi->resolution);
        printf("\n");
    }

    return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static void lsb_to_mps2(int16_t accel_lsb,
                        float g_range,
                        uint8_t bit_width,
                        struct bmi2_accel_processed_data *accel_data)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    accel_data->accel_raw = (float)accel_lsb / half_scale;
    accel_data->accel_g = accel_data->accel_raw * g_range;
    accel_data->accel_ms2 = accel_data->accel_g * GRAVITY_EARTH;
}
