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

/* Sensor initialization configuration. */
static struct bmi2_dev bmi2_dev;

/******************************************************************************/
/*!         Static Function Declarations                     */

/******************************************************************************/
/*!            Functions                                        */
/* This function starts the execution of program. */
int main(void)
{
    /* Structure to define the type of sensor and their respective data. */
    struct bmi2_feat_sensor_data sensor_data = { 0 };

    /* Structure to define the type of sensor and its configurations. */
    struct bmi2_sens_config config;

    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Accel sensor and door state detector feature are listed in array. */
    uint8_t sensor_sel[3] = { BMI2_ACCEL, BMI2_GYRO, BMI2_DOOR_STATE_DETECTOR };

    /* Type of sensor to get door state detector data. */
    sensor_data.type = BMI2_DOOR_STATE_DETECTOR;

    uint16_t loop = 50;

    uint8_t gpio_0 = 0;
    uint8_t calibration_flag = 0;
    float dsd_heading_output = 0.0;

    /* Select features and their pins to be mapped to. */
    struct bmi2_sens_int_config sens_int = { .type = BMI2_DOOR_STATE_DETECTOR, .hw_int_pin = BMI2_INT1 };

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&bmi2_dev, BMI2_SPI_INTF);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmi270_dsd. */
    printf("Uploading configuration file\n");
    rslt = bmi270_dsd_init(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    printf("Configuration file uploaded\n");
    printf("Chip ID :0x%x\n", bmi2_dev.chip_id);

    /* Enable the selected sensor. */
    rslt = bmi270_dsd_sensor_enable(sensor_sel, BMI2_N_SENSE_COUNT_3, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Configure the type of sensor. */
    config.type = BMI2_DOOR_STATE_DETECTOR;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_dsd_get_sensor_config(&config, BMI2_N_SENSE_COUNT_1, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    printf("\nInterrupt configuration\n");

    /* Map the feature interrupt for door state detector. */
    rslt = bmi270_dsd_map_feat_int(&sens_int, BMI2_N_SENSE_COUNT_1, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        printf("Interrupt Enabled: \t %s\n", enum_to_string(BMI2_DOOR_STATE_DETECTOR));
        printf("Interrupt Mapped to: \t %s\n\n", enum_to_string(BMI2_INT1));

        printf("\nPlease open the door greater than 20deg in 30 sec.\n");
        printf(
            "\nCheck whether calibration is completed! Please close the door and keep it static.\nCalibration should be completed in 5 sec.\n");
        while (!calibration_flag)
        {
            rslt = bmi2_get_regs(BMI2_DSD_OUT_ADDR, &gpio_0, 1, &bmi2_dev); /* 0x1E */
            calibration_flag = BMI2_GET_BITS(gpio_0, BMI270_DSD_CALIB_FLAG);
        }

        printf("calibration_flag = %d\n", calibration_flag);

        if (calibration_flag == 1)
        {
            printf("\nRotate the board to set heading value not equal to 0:\n");
            printf("\n%10s\t %10s\n", "Data_set", "Heading");
            for (int i = 0; i < loop; i++)
            {
                rslt = bmi270_dsd_get_feature_data(&sensor_data, BMI2_N_SENSE_COUNT_1, &bmi2_dev);
                bmi2_error_codes_print_result(rslt);
                dsd_heading_output = (float)(sensor_data.sens_data.door_state_detector_output.heading_output / 100.0);
                printf("%10d\t %10.2f\n", i, dsd_heading_output);
                coines_delay_msec(50);
            }

            printf("\nmanual heading reset simulation\n");

            rslt = bmi270_dsd_get_sensor_config(&config, BMI2_N_SENSE_COUNT_1, &bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            config.cfg.door_state_detector.reset_enable_flag = 1;
            rslt = bmi270_dsd_set_sensor_config(&config, BMI2_N_SENSE_COUNT_1, &bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            coines_delay_msec(2);
            printf("\nmanual heading reset done\n");
            printf("\n%10s\t %10s\n", "Data_set", "Heading");
            for (int i = 0; i < loop; i++)
            {
                /* / * Get heading output. * / */
                rslt = bmi270_dsd_get_feature_data(&sensor_data, BMI2_N_SENSE_COUNT_1, &bmi2_dev);
                bmi2_error_codes_print_result(rslt);
                dsd_heading_output = (float)(sensor_data.sens_data.door_state_detector_output.heading_output / 100.0);
                printf("%10d\t %10.2f\n", i, dsd_heading_output);
            }
        }
    }

    bmi2_coines_deinit();

    return rslt;
}
