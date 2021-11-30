/**\
 * Copyright (c) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmi270_legacy.h"
#include "bmm150.h"
#include "coines.h"
#include "common.h"

/******************************************************************************/
/*!                Macro definition                                           */

/*! Macros to select the sensors                   */
#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)
#define AUX            UINT8_C(0x02)

/*! Macro define limit to print data */
#define SAMPLE_COUNT   UINT8_C(20)

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*****************************************************************************/
/*!         Structure declaration                                            */

/* Sensor initialization configuration. */
struct bmi2_dev aux_bmi2_dev;

/******************************************************************************/
/*!                 Functions                                                 */

/**
 * aux_i2c_read - Reads data from auxiliary sensor in manual mode.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Aux data pointer to store the read data.
 *  @param[in] length       : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
static int8_t aux_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/**
 * aux_i2c_write - Writes data to the auxiliary sensor in manual mode.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Aux data pointer to store the data being written.
 *  @param[in] length       : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
static int8_t aux_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 * APIs.
 *
 *  @param[in] period       : The required wait time in microsecond.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return void.
 */
static void aux_delay_us(uint32_t period, void *intf_ptr);

/*!
 *  @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Value in meter per second squared.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Value in degree per second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/*!
 *  @brief This API is used to print the execution status.
 *
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
static void bmm150_error_codes_print_result(int8_t rslt);

/* This function starts the execution of program. */
int main(void)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to select the pull-up resistor which is set to trim register */
    uint8_t regdata;

    /* Variable to define limit to print aux data. */
    uint8_t limit = SAMPLE_COUNT;

    /* Variable to print data set count of aux data. */
    uint8_t count = 1;

    /* Accel, Gyro and Aux sensors are listed in array. */
    uint8_t sensor_list[3] = { BMI2_ACCEL, BMI2_GYRO, BMI2_AUX };

    /* Initialize the interrupt status of accel, gyro and aux. */
    uint16_t int_status = 0;

    /* Sensor initialization configuration. */
    struct bmm150_dev aux_bmm150_dev;

    /* bmm150 settings configuration */
    struct bmm150_settings settings;

    /* bmm150 magnetometer data */
    struct bmm150_mag_data mag_data;

    /* Structure to define the type of the sensor and its configurations. */
    struct bmi2_sens_config config[3];

    /* Structure to define type of sensor and their respective data. */
    struct bmi2_sens_data sensor_data = { { 0 } };

    /* Variables to define read the accel and gyro data in float */
    float x = 0, y = 0, z = 0;

    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;
    config[AUX].type = BMI2_AUX;

    /* Array of eight bytes to store x, y, z and r axis aux data. */
    uint8_t aux_data[8] = { 0 };

    /* To enable the i2c interface settings for bmm150. */
    uint8_t aux_bmm150_dev_addr = BMM150_DEFAULT_I2C_ADDRESS;
    aux_bmm150_dev.intf_ptr = &aux_bmm150_dev_addr;
    aux_bmm150_dev.read = aux_i2c_read;
    aux_bmm150_dev.write = aux_i2c_write;
    aux_bmm150_dev.delay_us = aux_delay_us;

    /* As per datasheet, aux interface with bmi270_legacy will support only for I2C */
    aux_bmm150_dev.intf = BMM150_I2C_INTF;

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&aux_bmi2_dev, BMI2_I2C_INTF);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmi270_legacy. */
    rslt = bmi270_legacy_init(&aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Pull-up resistor 2k is set to the trim register */
    regdata = BMI2_ASDA_PUPSEL_2K;
    rslt = bmi2_set_regs(BMI2_AUX_IF_TRIM, &regdata, 1, &aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_legacy_get_sensor_config(config, 3, &aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Configurations for accel. */
    config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    config[ACCEL].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

    /* Configurations for gyro. */
    config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    config[GYRO].cfg.gyr.noise_perf = BMI2_GYR_RANGE_2000;
    config[GYRO].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config[GYRO].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    /* Configurations for aux. */
    config[AUX].cfg.aux.odr = BMI2_AUX_ODR_100HZ;
    config[AUX].cfg.aux.aux_en = BMI2_ENABLE;
    config[AUX].cfg.aux.i2c_device_addr = BMM150_DEFAULT_I2C_ADDRESS;
    config[AUX].cfg.aux.manual_en = BMI2_ENABLE;
    config[AUX].cfg.aux.fcu_write_en = BMI2_ENABLE;
    config[AUX].cfg.aux.man_rd_burst = BMI2_AUX_READ_LEN_3;
    config[AUX].cfg.aux.read_addr = BMM150_REG_DATA_X_LSB;

    /* Set new configurations for accel, gyro and aux. */
    rslt = bmi270_legacy_set_sensor_config(config, 3, &aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* NOTE:
     * Accel and gyro enable must be done after setting configurations
     */
    rslt = bmi270_legacy_sensor_enable(sensor_list, 3, &aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmm150. */
    rslt = bmm150_init(&aux_bmm150_dev);
    bmm150_error_codes_print_result(rslt);

    /* Set the power mode to normal mode. */
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, &aux_bmm150_dev);
    bmm150_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    printf("MAGNETOMETER, ACCEL AND GYRO DATA IN MANUAL MODE\n");

#ifdef BMM150_USE_FIXED_POINT
    printf("Magnetometer data contains fraction part (last 4 bits) and decimal part\n\n");
#endif

    if (aux_bmm150_dev.chip_id == BMM150_CHIP_ID)
    {
        printf("\nValid BMM150 (Aux) sensor - Chip ID : 0x%x\n", aux_bmm150_dev.chip_id);

        while (1)
        {
            /* Delay has been added to get aux, accel and gyro data at the interval of every 0.05 second. */
            aux_bmi2_dev.delay_us(50000, aux_bmi2_dev.intf_ptr);

            /* To get the data ready interrupt status of accel, gyro and aux */
            rslt = bmi2_get_int_status(&int_status, &aux_bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            /* To check the data ready interrupt status and print the status for 20 samples. */
            if ((int_status & BMI2_ACC_DRDY_INT_MASK) && (int_status & BMI2_GYR_DRDY_INT_MASK))
            {
                rslt = bmi2_get_sensor_data(&sensor_data, &aux_bmi2_dev);
                bmi2_error_codes_print_result(rslt);

                if (rslt == BMI2_OK)
                {
                    printf("\nData set : %d", count);

                    printf("\nAcc_Raw_X = %d\t", sensor_data.acc.x);
                    printf("Acc_Raw_Y = %d\t", sensor_data.acc.y);
                    printf("Acc_Raw_Z = %d", sensor_data.acc.z);

                    /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
                    x = lsb_to_mps2(sensor_data.acc.x, 2, aux_bmi2_dev.resolution);
                    y = lsb_to_mps2(sensor_data.acc.y, 2, aux_bmi2_dev.resolution);
                    z = lsb_to_mps2(sensor_data.acc.z, 2, aux_bmi2_dev.resolution);

                    /* Print the data in m/s2. */
                    printf("\nAcc_ms2_X = %4.2f\t Acc_ms2_Y = %4.2f\t Acc_ms2_Z = %4.2f\n", x, y, z);

                    printf("\nGyr_Raw_X = %d\t", sensor_data.gyr.x);
                    printf("Gyr_Raw_Y = %d\t", sensor_data.gyr.y);
                    printf("Gyr_Raw_Z = %d", sensor_data.gyr.z);

                    /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                    x = lsb_to_dps(sensor_data.gyr.x, 2000, aux_bmi2_dev.resolution);
                    y = lsb_to_dps(sensor_data.gyr.y, 2000, aux_bmi2_dev.resolution);
                    z = lsb_to_dps(sensor_data.gyr.z, 2000, aux_bmi2_dev.resolution);

                    /* Print the data in dps. */
                    printf("\nGyro_DPS_X = %4.2f\t Gyro_DPS_Y = %4.2f\t Gyro_DPS_Z = %4.2f\n", x, y, z);
                }

                /* Read aux data from the bmm150 data registers. */
                rslt = bmi2_read_aux_man_mode(BMM150_REG_DATA_X_LSB, aux_data, 8, &aux_bmi2_dev);
                bmi2_error_codes_print_result(rslt);
                if (rslt == BMI2_OK)
                {
                    /* Compensating the raw auxiliary data available from the BMM150 API. */
                    rslt = bmm150_aux_mag_data(aux_data, &mag_data, &aux_bmm150_dev);
                    bmm150_error_codes_print_result(rslt);
                    printf("\nMag_uT_X = %ld\t  Mag_uT_Y = %ld\t  Mag_uT_Z = %ld\n", (long unsigned int)mag_data.x,
                           (long unsigned int)mag_data.y, (long unsigned int)mag_data.z);
                }

                count++;
                limit--;

                if (limit == 0)
                {
                    break;
                }
            }
        }
    }
    else
    {
        printf("\nInvalid BMM150 (Aux) sensor - Chip ID : 0x%x\n", aux_bmm150_dev.chip_id);
    }

    bmi2_coines_deinit();

    return rslt;
}

/*!
 * @brief This function reads the data from auxiliary sensor in manual mode.
 */
static int8_t aux_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int8_t rslt;

    rslt = bmi2_read_aux_man_mode(reg_addr, reg_data, length, &aux_bmi2_dev);

    return rslt;
}

/*!
 * @brief This function writes the data to auxiliary sensor in manual mode.
 */
static int8_t aux_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int8_t rslt;

    rslt = bmi2_write_aux_man_mode(reg_addr, reg_data, length, &aux_bmi2_dev);

    return rslt;
}

/*!
 * Delay function map to COINES platform
 */
static void aux_delay_us(uint32_t period, void *intf_ptr)
{
    coines_delay_usec(period);
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale))) * (val);
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
static void bmm150_error_codes_print_result(int8_t rslt)
{
    if (rslt != BMM150_OK)
    {
        switch (rslt)
        {
            case BMM150_E_NULL_PTR:
                printf("Error [%d] : Null pointer error.", rslt);
                printf(
                    "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
                break;

            case BMM150_E_COM_FAIL:
                printf("Error [%d] : Communication failure error.", rslt);
                printf(
                    "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
                break;

            case BMM150_E_DEV_NOT_FOUND:
                printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                       rslt);
                break;

            case BMM150_E_INVALID_CONFIG:
                printf("Error [%d] : Invalid sensor configuration.", rslt);
                printf(" It occurs when there is a mismatch in the requested feature with the available one\r\n");
                break;

            default:
                printf("Error [%d] : Unknown error code\r\n", rslt);
                break;
        }
    }
}
