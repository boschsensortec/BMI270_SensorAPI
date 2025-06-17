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

/******************************************************************************/
/*!            Functions                                        */
/* This function starts the execution of program. */
int main(void)
{
    /* Sensor initialization configuration. */
    struct bmi2_dev bmi2_dev;

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

    /* Variable to get door state detector interrupt status. */
    uint16_t int_status = 0;

    uint16_t loop = 4;

    uint8_t gpio_0 = 0;
    uint8_t remap_flag = 0;
    uint8_t calibration_flag = 0;
    uint8_t dsd_event_out = 0;
    float dsd_heading_output = 0.0;

    /* Select features and their pins to be mapped to. */
    struct bmi2_sens_int_config sens_int = { .type = BMI2_DOOR_STATE_DETECTOR, .hw_int_pin = BMI2_INT1 };

    enum REMAP_FLAG {
        REMAP_DONE = 1,
        TIMEOUT = 2
    };

    /* The door event are listed in array. */
    const char *door_event[3] = { "NONE", "DOOR_CLOSE", "DOOR_OPEN" };

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&bmi2_dev, BMI2_SPI_INTF);
    bmi2_error_codes_print_result(rslt);

    printf("Uploading configuration file\n");

    /* Initialize bmi270_dsd. */
    rslt = bmi270_dsd_init(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    printf("Configuration file uploaded\n");
    printf("Chip ID :0x%x\n", bmi2_dev.chip_id);

    /* Disable advance power save mode. */
    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Configure the type of sensor. */
    config.type = BMI2_DOOR_STATE_DETECTOR;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_dsd_get_sensor_config(&config, 1, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Note: The values below are from door_state_detector example*/

    /* load and apply axis remap information */
    config.cfg.door_state_detector.remap_flag = 1;
    config.cfg.door_state_detector.z_sign = 1;
    config.cfg.door_state_detector.z_axis = 0;

    /* load and apply gyro bias information */
    config.cfg.door_state_detector.bias_x_low_word = 0x1653;
    config.cfg.door_state_detector.bias_x_high_word = 0;
    config.cfg.door_state_detector.bias_y_low_word = 0xf78b;
    config.cfg.door_state_detector.bias_y_high_word = 0xffff;
    config.cfg.door_state_detector.bias_z_low_word = 0x401;
    config.cfg.door_state_detector.bias_z_high_word = 0;
    config.cfg.door_state_detector.gyro_calib_apply = 1;

    rslt = bmi270_dsd_set_sensor_config(&config, 1, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Enable the selected sensor. */
    rslt = bmi270_dsd_sensor_enable(sensor_sel, 3, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    printf("\nInterrupt configuration\n");

    /* Map the feature interrupt for door state detector. */
    rslt = bmi270_dsd_map_feat_int(&sens_int, 1, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        printf("Interrupt Enabled: \t %s\n", enum_to_string(BMI2_DOOR_STATE_DETECTOR));
        printf("Interrupt Mapped to: \t %s\n\n", enum_to_string(BMI2_INT1));

        printf("\nNo need to perform axis remap\n");
        while (!remap_flag)
        {
            rslt = bmi270_dsd_get_sensor_config(&config, 1, &bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            remap_flag = config.cfg.door_state_detector.remap_flag;
        }

        if (remap_flag == REMAP_DONE)
        {
            printf("\nAxis remap is done\n");
        }
        else if (remap_flag == TIMEOUT)
        {
            printf("\nTimeout!\n");
        }

        if (remap_flag == REMAP_DONE)
        {
            printf("\nNo need to perform gyro calibration.\n");
            while (!calibration_flag)
            {
                rslt = bmi2_get_regs(BMI2_DSD_OUT_ADDR, &gpio_0, 1, &bmi2_dev); /* 0x1E */
                bmi2_error_codes_print_result(rslt);

                calibration_flag = BMI2_GET_BITS(gpio_0, BMI270_DSD_CALIB_FLAG);
            }

            printf("\nCalibration_flag = %d\n", calibration_flag);

            printf("\nRotate the board to perform door event:\n");

            /* Loop to get door event. */
            while (loop)
            {
                /* To get the interrupt status of step detector. */
                rslt = bmi2_get_int_status(&int_status, &bmi2_dev);
                bmi2_error_codes_print_result(rslt);

                /* To check the interrupt status of step detector. */
                if (int_status & BMI270_DSD_STATUS_MASK)
                {
                    printf("\nDoor event detected\n");

                    /* Get door event output. */
                    rslt = bmi270_dsd_get_feature_data(&sensor_data, 1, &bmi2_dev);
                    dsd_event_out = sensor_data.sens_data.door_state_detector_output.door_event_output;
                    printf("Door event = %s\n", door_event[dsd_event_out]);

                    /* Get heading output. */
                    dsd_heading_output =
                        (float)(sensor_data.sens_data.door_state_detector_output.heading_output / 100.0);
                    printf("Heading values = %4.2f\n", dsd_heading_output);

                    loop--;
                }
            }

            printf("\nCheck heading output, please rotate sensor more than 90 deg\n");
            while (dsd_heading_output < 90.0)
            {
                /* / * Get heading output. * / */
                rslt = bmi270_dsd_get_feature_data(&sensor_data, 1, &bmi2_dev);
                bmi2_error_codes_print_result(rslt);
                dsd_heading_output = (float)(sensor_data.sens_data.door_state_detector_output.heading_output / 100.0);
            }

            printf("Heading values = %4.2f > 90 deg\n", dsd_heading_output);

            /* Disable the selected sensor. */
            printf("\nDisable the door lock\n");
            rslt = bmi270_dsd_sensor_disable(sensor_sel, 3, &bmi2_dev);
            bmi2_error_codes_print_result(rslt);
        }
    }

    bmi2_coines_deinit();

    return rslt;
}
