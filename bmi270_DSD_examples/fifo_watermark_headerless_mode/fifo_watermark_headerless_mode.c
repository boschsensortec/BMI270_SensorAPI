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
/*!                  Macros                                                   */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH                         (9.80665f)

/* Pi value used in converting gyro degrees per second to radian per second */
#define PI                                    (3.14f)

/*! Buffer size allocated to store raw FIFO data */
#define BMI270_DSD_FIFO_RAW_DATA_BUFFER_SIZE  UINT16_C(800)

/*! Length of data to be read from FIFO */
#define BMI270_DSD_FIFO_RAW_DATA_USER_LENGTH  UINT16_C(800)

/*! Number of accel frames to be extracted from FIFO */

/*!
 * Calculation:
 * fifo_watermark_level = 650, accel_frame_len = 6, gyro_frame_len = 6.
 * fifo_accel_frame_count = (650 / (6 + 6 )) = 54 frames
 */
#define BMI270_DSD_FIFO_ACCEL_FRAME_COUNT     UINT8_C(55)

/*! Number of gyro frames to be extracted from FIFO */
#define BMI270_DSD_FIFO_GYRO_FRAME_COUNT      UINT8_C(55)

/*! Setting a watermark level in FIFO */
#define BMI270_DSD_FIFO_WATERMARK_LEVEL       UINT16_C(650)

/******************************************************************************/
/*!                        Global Variables                                   */

/* Number of bytes of FIFO data
 * NOTE : Dummy byte (for SPI Interface) required for FIFO data read must be given as part of array size
 */
uint8_t fifo_data[BMI270_DSD_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

/* Array of accelerometer frames -> Total bytes =
 * 50 * (6 axes bytes) = 300 bytes */
struct bmi2_sens_axes_data fifo_accel_data[BMI270_DSD_FIFO_ACCEL_FRAME_COUNT] = { { 0 } };

/* Array of gyro frames -> Total bytes =
 * 50 * (6 axes bytes) = 300 bytes */
struct bmi2_sens_axes_data fifo_gyro_data[BMI270_DSD_FIFO_GYRO_FRAME_COUNT] = { { 0 } };

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal API is used to set configurations for accel and gyro.
 *  @param[in] dev       : Structure instance of bmi2_dev.
 *  @return Status of execution.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *dev);

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

/* This function starts the execution of program. */
int main(void)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to index bytes. */
    uint16_t index = 0;

    uint16_t fifo_length = 0;

    uint16_t accel_frame_length = BMI270_DSD_FIFO_ACCEL_FRAME_COUNT;

    uint16_t gyro_frame_length = BMI270_DSD_FIFO_GYRO_FRAME_COUNT;

    uint8_t try = 1;

    /* Variable that store converted Sensor Data from LSB to  (Raw, G-value and m/s2 )  */
    struct bmi2_accel_processed_data accel_data_x;
    struct bmi2_accel_processed_data accel_data_y;
    struct bmi2_accel_processed_data accel_data_z;

    /* Variable that stores converted gyro data*/
    struct bmi2_gyro_processed_data gyro_data_x;
    struct bmi2_gyro_processed_data gyro_data_y;
    struct bmi2_gyro_processed_data gyro_data_z;

    /* Sensor initialization configuration. */
    struct bmi2_dev dev;

    /* Initialize FIFO frame structure. */
    struct bmi2_fifo_frame fifoframe = { 0 };

    /* Accel and gyro sensor are listed in array. */
    uint8_t sensor_sel[2] = { BMI2_ACCEL, BMI2_GYRO };

    /* Variable to get fifo water-mark interrupt status. */
    uint16_t int_status = 0;

    uint16_t watermark = 0;

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&dev, BMI2_I2C_INTF);
    bmi2_error_codes_print_result(rslt);

    printf("Uploading configuration file\n");

    /* Initialize bmi270_dsd. */
    rslt = bmi270_dsd_init(&dev);
    bmi2_error_codes_print_result(rslt);

    printf("Configuration file uploaded\n");
    printf("Chip ID :0x%x\n", dev.chip_id);

    /* Configuration settings for accel and gyro. */
    rslt = set_accel_gyro_config(&dev);
    bmi2_error_codes_print_result(rslt);

    /* NOTE: Accel and Gyro enable must be done after setting configurations */
    rslt = bmi270_dsd_sensor_enable(sensor_sel, 2, &dev);
    bmi2_error_codes_print_result(rslt);

    /* Before setting FIFO, disable the advance power save mode. */
    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &dev);
    bmi2_error_codes_print_result(rslt);

    /* Initially disable all configurations in fifo. */
    rslt = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, &dev);
    bmi2_error_codes_print_result(rslt);

    /* Update FIFO structure. */
    /* Mapping the buffer to store the fifo data. */
    fifoframe.data = fifo_data;

    /* Length of FIFO frame. */
    fifoframe.length = BMI270_DSD_FIFO_RAW_DATA_USER_LENGTH;

    /* Set FIFO configuration by enabling accel, gyro.
     * NOTE 1: The header mode is enabled by default.
     * NOTE 2: By default the FIFO operating mode is FIFO mode. */
    rslt = bmi2_set_fifo_config(BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN, BMI2_ENABLE, &dev);
    bmi2_error_codes_print_result(rslt);

    printf("FIFO is configured in headerless mode\n");

    /* To enable headerless mode, disable the header. */
    if (rslt == BMI2_OK)
    {
        rslt = bmi2_set_fifo_config(BMI2_FIFO_HEADER_EN, BMI2_DISABLE, &dev);
        bmi2_error_codes_print_result(rslt);
    }

    /* FIFO water-mark interrupt is enabled. */
    fifoframe.data_int_map = BMI2_FWM_INT;

    /* Map water-mark interrupt to the required interrupt pin. */
    rslt = bmi2_map_data_int(fifoframe.data_int_map, BMI2_INT1, &dev);
    bmi2_error_codes_print_result(rslt);

    /* Set water-mark level. */
    fifoframe.wm_lvl = BMI270_DSD_FIFO_WATERMARK_LEVEL;

    fifoframe.length = BMI270_DSD_FIFO_RAW_DATA_USER_LENGTH;

    /* Set the water-mark level if water-mark interrupt is mapped. */
    rslt = bmi2_set_fifo_wm(fifoframe.wm_lvl, &dev);
    bmi2_error_codes_print_result(rslt);

    while (try <= 3)
    {
        /* Read FIFO data on interrupt. */
        rslt = bmi2_get_int_status(&int_status, &dev);
        bmi2_error_codes_print_result(rslt);

        /* To check the status of FIFO watermark interrupt. */
        if ((rslt == BMI2_OK) && (int_status & BMI2_FWM_INT_STATUS_MASK))
        {
            printf("\nIteration : %d\n", try);

            rslt = bmi2_get_fifo_wm(&watermark, &dev);
            bmi2_error_codes_print_result(rslt);
            printf("\nFIFO watermark level : %d\n", watermark);

            rslt = bmi2_get_fifo_length(&fifo_length, &dev);
            bmi2_error_codes_print_result(rslt);

            accel_frame_length = BMI270_DSD_FIFO_ACCEL_FRAME_COUNT;
            gyro_frame_length = BMI270_DSD_FIFO_GYRO_FRAME_COUNT;

            /* Updating FIFO length to be read based on available length and dummy byte updation */
            fifoframe.length = fifo_length + dev.dummy_byte;

            printf("\nFIFO data bytes available : %d \n", fifo_length);
            printf("\nFIFO data bytes requested : %d \n", fifoframe.length);

            /* Read FIFO data. */
            rslt = bmi2_read_fifo_data(&fifoframe, &dev);
            bmi2_error_codes_print_result(rslt);

            /* Read FIFO data on interrupt. */
            rslt = bmi2_get_int_status(&int_status, &dev);
            bmi2_error_codes_print_result(rslt);

            if (rslt == BMI2_OK)
            {
                printf("\nRequested accelerometer data frames before parsing: %d\n", accel_frame_length);

                /* Parse the FIFO data to extract accelerometer data from the FIFO buffer. */
                (void)bmi2_extract_accel(fifo_accel_data, &accel_frame_length, &fifoframe, &dev);
                printf("Parsed accelerometer data frames: %d\n", accel_frame_length);

                printf("\nAccel data in LSB units and in Gravity\n");

                printf("\n%8s\t %12s\t %12s\t %12s\t %12s\t %12s\t %12s\t %12s\t %12s\t %12s\t %12s\t %12s\t %12s\n",
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
                               dev.resolution,
                               &accel_data_x);
                    lsb_to_ms2(fifo_accel_data[index].y,
                               BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G),
                               dev.resolution,
                               &accel_data_y);
                    lsb_to_ms2(fifo_accel_data[index].z,
                               BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G),
                               dev.resolution,
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
                (void)bmi2_extract_gyro(fifo_gyro_data, &gyro_frame_length, &fifoframe, &dev);
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
                               dev.resolution,
                               &gyro_data_x);
                    lsb_to_dps(fifo_gyro_data[index].y,
                               BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000),
                               dev.resolution,
                               &gyro_data_y);
                    lsb_to_dps(fifo_gyro_data[index].z,
                               BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000),
                               dev.resolution,
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
            }

            try++;
        }
    }

    bmi2_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for accel.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accel and gyro configurations. */
    struct bmi2_sens_config config[2];

    /* Configure the type of feature. */
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_dsd_get_sensor_config(config, 2, dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameter according to their requirement. */
        /* Accel configuration settings. */
        /* Output Data Rate. By default ODR is set as 100Hz for accel. */
        config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* Gyro configuration settings. */
        /* Output Data Rate. Default ODR is 200Hz, setting to 100Hz. */
        config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope Bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set new configurations. */
        rslt = bmi270_dsd_set_sensor_config(config, 2, dev);
        bmi2_error_codes_print_result(rslt);

        printf("*************************************\n");
        printf("\nAccel Configurations\n");
        printf("ODR : %s\t\n", enum_to_string(BMI2_ACC_ODR_100HZ));
        printf("Range : %s\t\n", enum_to_string(BMI2_ACC_RANGE_2G));
        printf("Bandwidth : %s\t\n", enum_to_string(BMI2_ACC_NORMAL_AVG4));
        printf("Filter performance : %s\t\n", enum_to_string(BMI2_PERF_OPT_MODE));

        printf("*************************************\n");
        printf("\nGyro Configurations\n");
        printf("ODR : %s\t\n", enum_to_string(BMI2_GYR_ODR_100HZ));
        printf("Range : %s\t\n", enum_to_string(BMI2_GYR_RANGE_2000));
        printf("Bandwidth : %s\t\n", enum_to_string(BMI2_GYR_NORMAL_MODE));
        printf("Noise performance : %s\t\n", enum_to_string(BMI2_POWER_OPT_MODE));
        printf("Filter performance : %s\t\n", enum_to_string(BMI2_PERF_OPT_MODE));
    }

    return rslt;
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
