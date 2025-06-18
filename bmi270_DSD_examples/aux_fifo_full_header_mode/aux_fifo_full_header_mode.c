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
#include "bmm150.h"
#include "common.h"

/******************************************************************************/
/*!                Macro definition                                           */

/*! Macros to select the sensors                   */
#define ACCEL                           UINT8_C(0x00)
#define GYRO                            UINT8_C(0x01)
#define AUX                             UINT8_C(0x02)

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH                   (9.80665f)

/* Pi value used in converting gyro degrees per second to radian per second */
#define PI                              (3.14f)

/*! Buffer size allocated to store raw FIFO data. */
#define BMI2_FIFO_RAW_DATA_BUFFER_SIZE  UINT16_C(2048)

/*! Length of data to be read from FIFO. */
#define BMI2_FIFO_RAW_DATA_USER_LENGTH  UINT16_C(2048)

/*! Number of accel frames to be extracted from FIFO. */

/*! Calculation for frame count: Total frame count = Fifo buffer size(2048)/ Total frames(6 Accel, 6 Gyro, 8 Aux and 1 header,
 * totaling to 21) which equals to 97.
 *
 * Extra frames to parse sensortime data
 */
#define BMI2_FIFO_ACCEL_FRAME_COUNT     UINT8_C(120)

/*! Number of gyro frames to be extracted from FIFO. */
#define BMI2_FIFO_GYRO_FRAME_COUNT      UINT8_C(120)

/*! Number of aux frames to be extracted from FIFO. */
#define BMI2_FIFO_AUX_FRAME_COUNT       UINT8_C(120)

/*! Macro to read sensortime byte in FIFO. */
#define SENSORTIME_OVERHEAD_BYTE        UINT8_C(220)

/*****************************************************************************/
/*!         Structure declaration                                            */

/* Sensor initialization configuration. */
struct bmi2_dev aux_bmi2_dev;

/******************************************************************************/
/*!                        Global Variables                                   */

/* To read sensortime, extra 3 bytes are added to fifo buffer. */
uint16_t fifo_buffer_size = BMI2_FIFO_RAW_DATA_BUFFER_SIZE + SENSORTIME_OVERHEAD_BYTE;

/* Number of bytes of FIFO data
 * NOTE : Dummy byte (for SPI Interface) required for FIFO data read must be given as part of array size
 * Array size same as fifo_buffer_size
 */
uint8_t fifo_data[BMI2_FIFO_RAW_DATA_BUFFER_SIZE + SENSORTIME_OVERHEAD_BYTE];

/* Array of accelerometer frames */
struct bmi2_sens_axes_data fifo_accel_data[BMI2_FIFO_ACCEL_FRAME_COUNT] = { { 0 } };

/* Array of gyro frames */
struct bmi2_sens_axes_data fifo_gyro_data[BMI2_FIFO_GYRO_FRAME_COUNT] = { { 0 } };

/* Array of aux frames */
struct bmi2_aux_fifo_data fifo_aux_data[BMI2_FIFO_AUX_FRAME_COUNT] = { { { 0 } } };

/*******************************************************************************/
/*!                 Functions                                                  */

/**
 * aux_i2c_read - Reads data from auxiliary sensor.
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
 * aux_i2c_write - Writes data to the auxiliary sensor.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data    : Aux data pointer to store the data being written.
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
 *  @brief This API is used to print the execution status.
 *
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
static void bmm150_error_codes_print_result(int8_t rslt);

/*!
 *  @brief This internal API converts raw sensor values(LSB) to meters per seconds square.
 *
 *  @param[in] accel_lsb  : lsb sensor value.
 *  @param[in] g_range    : Accel Range selected (2G, 4G, 8G, 16G).
 *  @param[in] resolution : Resolution of the sensor.
 *  @param[in] accel_data : Structure instance of accel_Data to store the converted data .
 *
 *  @return void.
 *
 */
static void lsb_to_ms2(int16_t accel_lsb,
                       float g_range,
                       uint8_t resolution,
                       struct bmi2_accel_processed_data *accel_data);

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

int main(void)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to select the pull-up resistor which is set to trim register */
    uint8_t regdata;

    /* Variable that store converted Sensor Data from LSB to  (Raw, G-value and m/s2 )  */
    struct bmi2_accel_processed_data accel_data_x;
    struct bmi2_accel_processed_data accel_data_y;
    struct bmi2_accel_processed_data accel_data_z;

    /* Variable that stores converted gyro data*/
    struct bmi2_gyro_processed_data gyro_data_x;
    struct bmi2_gyro_processed_data gyro_data_y;
    struct bmi2_gyro_processed_data gyro_data_z;

    /* Initialize the interrupt status of accel, gyro and aux. */
    uint16_t int_status = 0;

    /* Accel, Gyro and Aux sensors are listed in array. */
    uint8_t sensor_list[3] = { BMI2_ACCEL, BMI2_GYRO, BMI2_AUX };

    /* Structure to define the type of the sensor and its configurations. */
    struct bmi2_sens_config config[3];

    /* Sensor initialization configuration. */
    struct bmm150_dev aux_bmm150_dev;

    /* bmm150 settings configuration */
    struct bmm150_settings settings;

    /* bmm150 magnetometer data */
    struct bmm150_mag_data mag_data, mag_raw;

    uint16_t index = 0;

    uint16_t accel_frame_length = BMI2_FIFO_ACCEL_FRAME_COUNT;

    uint16_t gyro_frame_length = BMI2_FIFO_GYRO_FRAME_COUNT;

    uint16_t aux_frame_length = BMI2_FIFO_AUX_FRAME_COUNT;

    uint16_t fifo_length = 0;

    int8_t try = 1;

    /* Initialize FIFO frame structure. */
    struct bmi2_fifo_frame fifoframe = { 0 };

    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;
    config[AUX].type = BMI2_AUX;

    /* To enable the i2c interface settings for bmm150. */
    uint8_t aux_bmm150_dev_addr = BMM150_DEFAULT_I2C_ADDRESS;
    aux_bmm150_dev.intf_ptr = &aux_bmm150_dev_addr;
    aux_bmm150_dev.read = aux_i2c_read;
    aux_bmm150_dev.write = aux_i2c_write;
    aux_bmm150_dev.delay_us = aux_delay_us;

    /* As per datasheet, aux interface with bmi270_dsd will support only for I2C */
    aux_bmm150_dev.intf = BMM150_I2C_INTF;

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&aux_bmi2_dev, BMI2_I2C_INTF);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmi270_dsd. */
    printf("Uploading configuration file\n");
    rslt = bmi270_dsd_init(&aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    printf("Configuration file uploaded\n");
    printf("Chip ID :0x%x\n", aux_bmi2_dev.chip_id);

    /* Pull-up resistor 2k is set to the trim regiter */
    regdata = BMI2_ASDA_PUPSEL_2K;
    rslt = bmi2_set_regs(BMI2_AUX_IF_TRIM, &regdata, 1, &aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_dsd_get_sensor_config(config, 3, &aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Configurations for accel. */
    config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    config[ACCEL].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_50HZ;
    config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

    /* Configurations for gyro. */
    config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
    config[GYRO].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_50HZ;
    config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config[GYRO].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    /* Configurations for aux. */
    config[AUX].cfg.aux.odr = BMI2_AUX_ODR_50HZ;
    config[AUX].cfg.aux.aux_en = BMI2_ENABLE;
    config[AUX].cfg.aux.i2c_device_addr = BMM150_DEFAULT_I2C_ADDRESS;
    config[AUX].cfg.aux.fcu_write_en = BMI2_ENABLE;
    config[AUX].cfg.aux.man_rd_burst = BMI2_AUX_READ_LEN_3;
    config[AUX].cfg.aux.read_addr = BMM150_REG_DATA_X_LSB;

    /* Set new configurations for accel, gyro and aux. */
    rslt = bmi270_dsd_set_sensor_config(config, 3, &aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    printf("*************************************\n");
    printf("\nAccel Configurations\n");
    printf("ODR : %s\t\n", enum_to_string(BMI2_ACC_ODR_50HZ));
    printf("Range : %s\t\n", enum_to_string(BMI2_ACC_RANGE_2G));
    printf("Bandwidth : %s\t\n", enum_to_string(BMI2_ACC_OSR2_AVG2));
    printf("Filter performance : %s\t\n", enum_to_string(BMI2_PERF_OPT_MODE));

    printf("*************************************\n");
    printf("\nGyro Configurations\n");
    printf("ODR : %s\t\n", enum_to_string(BMI2_GYR_ODR_50HZ));
    printf("Range : %s\t\n", enum_to_string(BMI2_GYR_RANGE_2000));
    printf("Bandwidth : %s\t\n", enum_to_string(BMI2_GYR_OSR2_MODE));
    printf("Noise performance : %s\t\n", enum_to_string(BMI2_POWER_OPT_MODE));
    printf("Filter performance : %s\t\n", enum_to_string(BMI2_PERF_OPT_MODE));
    printf("Ois Range : %s\t\n", enum_to_string(BMI2_GYR_OIS_2000));

    printf("*************************************\n");
    printf("\nAUX Configurations\n");
    printf("ODR : %s\t\n", enum_to_string(BMI2_AUX_ODR_50HZ));
    printf("AUX Enable : %s\t\n", enum_to_string(BMI2_ENABLE));
    printf("I2C Device Address : %s\t\n", enum_to_string(BMM150_DEFAULT_I2C_ADDRESS));
    printf("FCU Write Enable : %s\t\n", enum_to_string(BMI2_ENABLE));
    printf("Manual Read Burst : %s\t\n", enum_to_string(BMI2_AUX_READ_LEN_3));
    printf("Read Address : %s\t\n", enum_to_string(BMM150_REG_DATA_X_LSB));

    /* NOTE:
     * Accel and gyro enable must be done after setting configurations
     */
    rslt = bmi270_dsd_sensor_enable(sensor_list, 3, &aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmm150. */
    rslt = bmm150_init(&aux_bmm150_dev);
    bmm150_error_codes_print_result(rslt);

    /* Set the power mode to normal mode. */
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, &aux_bmm150_dev);
    bmm150_error_codes_print_result(rslt);

    rslt = bmi270_dsd_get_sensor_config(config, 3, &aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Disable manual mode so that the data mode is enabled. */
    config[AUX].cfg.aux.manual_en = BMI2_DISABLE;

    /* Set the aux configurations. */
    rslt = bmi270_dsd_set_sensor_config(config, 3, &aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Before setting FIFO, disable the advance power save mode. */
    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Initially disable all configurations in fifo. */
    rslt = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, &aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Update FIFO structure. */
    /* Mapping the buffer to store the fifo data. */
    fifoframe.data = fifo_data;

    /* Length of FIFO frame. */
    /* To read sensortime, extra 3 bytes are added to fifo user length. */
    fifoframe.length = BMI2_FIFO_RAW_DATA_USER_LENGTH + SENSORTIME_OVERHEAD_BYTE;

    /* Set FIFO configuration by enabling accel, gyro and timestamp.
     * NOTE 1: The header mode is enabled by default.
     * NOTE 2: By default the FIFO operating mode is in FIFO mode.
     * NOTE 3: Sensortime is enabled by default */
    printf("\nFIFO is configured in header mode\n");
    rslt = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_ENABLE, &aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Map FIFO full interrupt. */
    fifoframe.data_int_map = BMI2_FFULL_INT;
    rslt = bmi2_map_data_int(fifoframe.data_int_map, BMI2_INT1, &aux_bmi2_dev);
    bmi2_error_codes_print_result(rslt);

#ifdef BMM150_USE_FIXED_POINT
    printf("Magnetometer data contains fraction part (last 4 bits) and decimal part\n\n");
#endif

    if (aux_bmm150_dev.chip_id == BMM150_CHIP_ID)
    {
        printf("\nValid BMM150 (Aux) sensor - Chip ID : 0x%x\n", aux_bmm150_dev.chip_id);

        while (try <= 3)
        {
            /* Read FIFO data on interrupt. */
            rslt = bmi2_get_int_status(&int_status, &aux_bmi2_dev);
            bmi2_error_codes_print_result(rslt);

            if ((rslt == BMI2_OK) && (int_status & BMI2_FFULL_INT_STATUS_MASK))
            {
                printf("\nIteration : %d\n", try);

                accel_frame_length = BMI2_FIFO_ACCEL_FRAME_COUNT;

                gyro_frame_length = BMI2_FIFO_GYRO_FRAME_COUNT;

                aux_frame_length = BMI2_FIFO_AUX_FRAME_COUNT;

                rslt = bmi2_get_fifo_length(&fifo_length, &aux_bmi2_dev);
                bmi2_error_codes_print_result(rslt);

                /* Updating FIFO length to be read based on available length and dummy byte updation */
                fifoframe.length = fifo_length + SENSORTIME_OVERHEAD_BYTE + aux_bmi2_dev.dummy_byte;

                printf("\nFIFO data bytes available : %d \n", fifo_length);
                printf("\nFIFO data bytes requested : %d \n", fifoframe.length);

                /* Read FIFO data. */
                rslt = bmi2_read_fifo_data(&fifoframe, &aux_bmi2_dev);
                bmi2_error_codes_print_result(rslt);

                /* Read FIFO data on interrupt. */
                rslt = bmi2_get_int_status(&int_status, &aux_bmi2_dev);
                bmi2_error_codes_print_result(rslt);

                if (rslt == BMI2_OK)
                {
                    printf("\nRequested accelerometer data frames before parsing: %d\n", accel_frame_length);

                    /* Parse the FIFO data to extract accelerometer data from the FIFO buffer. */
                    rslt = bmi2_extract_accel(fifo_accel_data, &accel_frame_length, &fifoframe, &aux_bmi2_dev);
                    printf("Parsed accelerometer data frames: %d\n", accel_frame_length);

                    printf("\nAccel data in LSB units and in Gravity\n");

                    printf(
                        "\n%8s\t %12s\t %12s\t %12s\t %12s\t %12s\t %12s\t %12s\t %12s\t %12s\t %12s\t %12s\t %12s\n",
                        "INDEX",
                        "ACC_LSB_X",
                        "ACC_LSB_Y",
                        "ACC_LSB_Z",
                        "ACC_RAW_X",
                        "ACC_RAW_Y",
                        "ACC_RAW_Z",
                        "ACC_G_X",
                        "ACC_G_Y",
                        "ACC_G_Z",
                        "ACC_MS2_X",
                        "ACC_MS2_Y",
                        "ACC_MS2_Z");

                    /* Print the parsed accelerometer data from the FIFO buffer. */
                    for (index = 0; index < accel_frame_length; index++)
                    {
                        /* Converting lsb to G force for 16-bit accelerometer at 2G range. */
                        lsb_to_ms2(fifo_accel_data[index].x,
                                   BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G),
                                   aux_bmi2_dev.resolution,
                                   &accel_data_x);
                        lsb_to_ms2(fifo_accel_data[index].y,
                                   BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G),
                                   aux_bmi2_dev.resolution,
                                   &accel_data_y);
                        lsb_to_ms2(fifo_accel_data[index].z,
                                   BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G),
                                   aux_bmi2_dev.resolution,
                                   &accel_data_z);

                        /* Print the data in Gravity. */
                        printf(
                            "%8u\t %12d\t %12d\t %12d\t %+12.3f\t %+12.3f\t %+12.3f\t %+12.3f\t %12.3f\t %+12.3f\t %+12.3f\t %+12.3f\t %+12.3f\n",
                            index,
                            fifo_accel_data[index].x,
                            fifo_accel_data[index].y,
                            fifo_accel_data[index].z,
                            accel_data_x.accel_raw,
                            accel_data_y.accel_raw,
                            accel_data_z.accel_raw,
                            accel_data_x.accel_g,
                            accel_data_y.accel_g,
                            accel_data_z.accel_g,
                            accel_data_x.accel_ms2,
                            accel_data_y.accel_ms2,
                            accel_data_z.accel_ms2);
                    }

                    printf("\nRequested gyro data frames before parsing: %d \n", gyro_frame_length);

                    /* Parse the FIFO data to extract gyro data from the FIFO buffer. */
                    (void)bmi2_extract_gyro(fifo_gyro_data, &gyro_frame_length, &fifoframe, &aux_bmi2_dev);
                    printf("Parsed gyroscope data frames: %d\n", gyro_frame_length);

                    printf("\nGyro data in LSB units and degrees per second\n");
                    printf("\n%10s %12s %12s %12s %12s %12s %12s %12s %12s %12s %12s %12s %12s\n",
                           "INDEX",
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

                    /* Print the parsed gyro data from the FIFO buffer. */
                    for (index = 0; index < gyro_frame_length; index++)
                    {

                        /* Converting lsb to degree per second for 16 bit gyro at 2000dps range */
                        lsb_to_dps(fifo_gyro_data[index].x,
                                   BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000),
                                   aux_bmi2_dev.resolution,
                                   &gyro_data_x);
                        lsb_to_dps(fifo_gyro_data[index].y,
                                   BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000),
                                   aux_bmi2_dev.resolution,
                                   &gyro_data_y);
                        lsb_to_dps(fifo_gyro_data[index].z,
                                   BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000),
                                   aux_bmi2_dev.resolution,
                                   &gyro_data_z);

                        printf("%10u %12d %12d %12d %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f\n",
                               index,
                               fifo_gyro_data[index].x,
                               fifo_gyro_data[index].y,
                               fifo_gyro_data[index].z,
                               gyro_data_x.gyro_raw,
                               gyro_data_y.gyro_raw,
                               gyro_data_z.gyro_raw,
                               gyro_data_x.gyro_dps,
                               gyro_data_y.gyro_dps,
                               gyro_data_z.gyro_dps,
                               gyro_data_x.gyro_rps,
                               gyro_data_y.gyro_rps,
                               gyro_data_z.gyro_rps);
                    }

                    printf("\nRequested aux data frames before parsing: %d \n", aux_frame_length);

                    /* Parse the FIFO data to extract aux data from the FIFO buffer. */
                    (void)bmi2_extract_aux(fifo_aux_data, &aux_frame_length, &fifoframe, &aux_bmi2_dev);
                    printf("Parsed aux data frames: %d \n", aux_frame_length);

                    printf("\n%10s\t %12s\t %12s\t %12s\t %12s\t %12s\t %12s\n",
                           "AUX_DATA",
                           "Mag_Raw_X",
                           "Mag_Raw_Y",
                           "Mag_Raw_Z",
                           "Mag_uT_X",
                           "Mag_uT_Y",
                           "Mag_uT_Z");

                    /* Print the parsed aux data from the FIFO buffer. */
                    for (index = 0; index < aux_frame_length; index++)
                    {
                        /* Compensating the raw auxiliary data available from the BMM150 API. */
                        rslt = bmm150_aux_mag_data(fifo_aux_data[index].data, &mag_data, &aux_bmm150_dev);
                        bmm150_error_codes_print_result(rslt);

                        mag_raw.x = ((uint16_t)fifo_aux_data[index].data[0] >> 3 & 0x001F) |
                                    ((uint16_t)fifo_aux_data[index].data[1] << 5 & 0x1FE0);

                        mag_raw.y = ((uint16_t)fifo_aux_data[index].data[2] >> 3 & 0x001F) |
                                    ((uint16_t)fifo_aux_data[index].data[3] << 5 & 0x1FE0);

                        mag_raw.z = ((uint16_t)fifo_aux_data[index].data[4] >> 1 & 0x007F) |
                                    ((uint16_t)fifo_aux_data[index].data[5] << 7 & 0x1F80);

                        printf("%10d\t %12ld\t %12ld\t %12ld\t %12ld\t %12ld\t %12ld\n",
                               index,
                               (long int)mag_raw.x,
                               (long int)mag_raw.y,
                               (long int)mag_raw.z,
                               (long int)mag_data.x,
                               (long int)mag_data.y,
                               (long int)mag_data.z);
                    }

                    /* Print control frames like sensor time and skipped frame count. */
                    printf("\nSkipped frame count = %d\n", fifoframe.skipped_frame_count);

                    printf("Sensor time(in seconds) = %.4lf  s\r\n",
                           (fifoframe.sensor_time * BMI2_SENSORTIME_RESOLUTION));

                    try++;
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
 * @brief This function reads the data from auxiliary sensor
 */
static int8_t aux_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int8_t rslt;

    (void)intf_ptr;

    rslt = bmi2_read_aux_man_mode(reg_addr, reg_data, (uint16_t)length, &aux_bmi2_dev);

    return rslt;
}

/*!
 * @brief This function writes the data to auxiliary sensor
 */
static int8_t aux_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int8_t rslt;

    (void)intf_ptr;

    rslt = bmi2_write_aux_man_mode(reg_addr, reg_data, (uint16_t)length, &aux_bmi2_dev);

    return rslt;
}

/*!
 * Delay function map to COINES platform
 */
static void aux_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    coines_delay_usec(period);
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

/*!
 * @brief This internal API converts raw sensor register content (LSB) to corresponding accel raw value, g value and m/s2 value
 */
static void lsb_to_ms2(int16_t accel_lsb,
                       float g_range,
                       uint8_t resolution,
                       struct bmi2_accel_processed_data *accel_data)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)resolution) / 2.0f));

    accel_data->accel_raw = (float)accel_lsb / half_scale;
    accel_data->accel_g = accel_data->accel_raw * g_range;
    accel_data->accel_ms2 = accel_data->accel_g * GRAVITY_EARTH;
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
