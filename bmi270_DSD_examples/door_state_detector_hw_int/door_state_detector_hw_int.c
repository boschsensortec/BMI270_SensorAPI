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
#include "coines.h"

/******************************************************************************/
/*!           Global variable Declaration                                     */

volatile uint8_t drdy_int_status = 0;

/*!
 * @brief This internal API is used to set the interrupt status
 */
static void interrupt_callback(uint32_t param1, uint32_t param2)
{
    (void)param1;
    (void)param2;
    drdy_int_status = 1;
}

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

    struct bmi2_int_pin_config pin_config = { 0 };

    /* Variable to get door state detector interrupt status. */
    uint16_t int_status = 0;

    uint16_t loop = 4;

    uint8_t gpio_0 = 0;
    uint8_t remap_flag = 0;
    uint8_t calibration_flag = 0;
    uint8_t dsd_event_out = 0;
    float dsd_heading_output = 0.0;

    uint8_t board = 0;

    /* Remap Output*/
    uint8_t z_sign = 0;
    uint8_t z_axis = 0;

    /* Gyro bias output*/
    int32_t bias_x = 0;
    int32_t bias_y = 0;
    int32_t bias_z = 0;

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

    /* Initialize bmi270_dsd. */
    printf("Uploading configuration file\n");
    rslt = bmi270_dsd_init(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    printf("Configuration file uploaded\n");
    printf("Chip ID :0x%x\n", bmi2_dev.chip_id);

    /* Disable advance power save mode. */
    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

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

    printf("Interrupt Enabled: \t %s\n", enum_to_string(BMI2_DOOR_STATE_DETECTOR));
    printf("Interrupt Mapped to: \t %s\n\n", enum_to_string(BMI2_INT1));

    pin_config.pin_type = BMI2_INT1;
    pin_config.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    pin_config.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    pin_config.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    pin_config.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

    rslt = bmi2_set_int_pin_config(&pin_config, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    struct bmi2_int_pin_config pin_config_1 = { 0 };
    pin_config_1.pin_type = BMI2_INT1;
    rslt = bmi2_get_int_pin_config(&pin_config_1, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Get board information */
    get_board_info(&board);

    /*
     * Attach interrupt based on board
     */
    if (board == BOARD_MCU_APP20)
    {
        if (pin_config.pin_type == BMI2_INT1)
        {
            coines_attach_interrupt(COINES_SHUTTLE_PIN_20, interrupt_callback, COINES_PIN_INTERRUPT_RISING_EDGE);
        }
        else if (pin_config.pin_type == BMI2_INT2)
        {
            coines_attach_interrupt(COINES_SHUTTLE_PIN_21, interrupt_callback, COINES_PIN_INTERRUPT_RISING_EDGE);
        }
    }

#if !defined(MCU_APP20)
    else
    {
        if (pin_config.pin_type == BMI2_INT1)
        {
            coines_attach_interrupt(COINES_MINI_SHUTTLE_PIN_1_6, interrupt_callback, COINES_PIN_INTERRUPT_RISING_EDGE);
        }
        else if (pin_config.pin_type == BMI2_INT2)
        {
            coines_attach_interrupt(COINES_MINI_SHUTTLE_PIN_1_7, interrupt_callback, COINES_PIN_INTERRUPT_RISING_EDGE);
        }
    }
#endif

    if (rslt == BMI2_OK)
    {
        printf("\nPlease open the door greater than 20deg within 30 sec.\n");
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
            printf("\nPlease close the door and keep it static.\nCalibration should be completed in 5 sec.\n");
            while (!calibration_flag)
            {
                rslt = bmi2_get_regs(BMI2_DSD_OUT_ADDR, &gpio_0, 1, &bmi2_dev); /* 0x1E */
                bmi2_error_codes_print_result(rslt);

                calibration_flag = BMI2_GET_BITS(gpio_0, BMI270_DSD_CALIB_FLAG);
            }

            printf("\nCalibration_flag = %d\n", calibration_flag);

            if (calibration_flag == 1)
            {
                printf("\nRotate the board to perform door event:\n");

                /* Loop to get door event. */
                while (loop)
                {
                    if (drdy_int_status == 1)
                    {
                        drdy_int_status = 0;

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
                }

                printf("\nCheck heading output, please rotate sensor more than 90 deg\n");
                while (dsd_heading_output < 90.0)
                {
                    /* / * Get heading output. * / */
                    rslt = bmi270_dsd_get_feature_data(&sensor_data, 1, &bmi2_dev);
                    bmi2_error_codes_print_result(rslt);
                    dsd_heading_output =
                        (float)(sensor_data.sens_data.door_state_detector_output.heading_output / 100.0);
                }

                printf("Heading values = %4.2f > 90 deg\n", dsd_heading_output);

                /* Disable the selected sensor. */
                printf("\nDisable the door lock\n");
                rslt = bmi270_dsd_sensor_disable(sensor_sel, 3, &bmi2_dev);
                bmi2_error_codes_print_result(rslt);

                /* Store remap info and gyro bias*/
                printf("\nStore remap info and gyro bias.\n");
                printf(
                    "\nAfter rebooting the device, set them before enabling the sensor to skip the procedure of remap and calibration.\n");
                rslt = bmi270_dsd_get_sensor_config(&config, 1, &bmi2_dev);
                bmi2_error_codes_print_result(rslt);

                /* Get remap outputs*/
                z_axis = config.cfg.door_state_detector.z_axis;
                z_sign = config.cfg.door_state_detector.z_sign;
                printf("\nz axis = %d , z sign = %d\n", z_axis, z_sign);

                /* Get gyro bias outputs, with resolution 0.001 */
                bias_x =
                    (int32_t)(((uint32_t)config.cfg.door_state_detector.bias_x_high_word << 16) |
                              (uint32_t)config.cfg.door_state_detector.bias_x_low_word);
                bias_y =
                    (int32_t)(((uint32_t)config.cfg.door_state_detector.bias_y_high_word << 16) |
                              (uint32_t)config.cfg.door_state_detector.bias_y_low_word);
                bias_z =
                    (int32_t)(((uint32_t)config.cfg.door_state_detector.bias_z_high_word << 16) |
                              (uint32_t)config.cfg.door_state_detector.bias_z_low_word);

                printf("bias x_high_word= 0x%x , x_low_word= 0x%x\n",
                       config.cfg.door_state_detector.bias_x_high_word,
                       config.cfg.door_state_detector.bias_x_low_word);
                printf("bias y_high_word= 0x%x , y_low_word= 0x%x\n",
                       config.cfg.door_state_detector.bias_y_high_word,
                       config.cfg.door_state_detector.bias_y_low_word);
                printf("bias z_high_word= 0x%x , z_low_word= 0x%x\n",
                       config.cfg.door_state_detector.bias_z_high_word,
                       config.cfg.door_state_detector.bias_z_low_word);

                printf("\nx axis bias(0.001 lsb) = %ld , y axis bias(0.001 lsb) = %ld , z axis bias(0.001 lsb) = %ld\n",
                       (long signed int)bias_x,
                       (long signed int)bias_y,
                       (long signed int)bias_z);
            }
        }
    }

    bmi2_coines_deinit();

    return rslt;
}
