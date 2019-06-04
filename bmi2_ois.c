/**
 * Copyright (C) 2019 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file	bmi2_ois.c
 * @date	2019-5-28
 * @version	v2.19.0
 *
 */

/******************************************************************************/

/*!  @name          Header Files                                  */
/******************************************************************************/
#include "bmi2_ois.h"

/******************************************************************************/

/*!         Local Function Prototypes
 ******************************************************************************/

/*!
 * @brief This internal API gets the OIS accelerometer and the gyroscope data.
 *
 * @param[out] ois_data                     : Structure instance of bmi2_sens_axes_data.
 * @param[in] reg_addr                      : Register address where data is stored.
 * @param[in] ois_dev                       : Structure instance of bmi2_ois_dev.
 * @param[in] ois_gyr_cross_sens_zx         :"gyroscope cross sensitivity value which was calculated during
 * bmi2xy_init(), refer the example ois_accel_gyro.c for more info"
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t get_ois_acc_gyr_data(struct bmi2_ois_sens_axes_data *ois_data,
                                   uint8_t reg_addr,
                                   const struct bmi2_ois_dev *ois_dev,
                                   int16_t ois_gyr_cross_sens_zx);

/*!
 * @brief This internal API is used to validate the OIS device pointer for null
 * conditions.
 *
 * @param[in] ois_dev : Structure instance of bmi2_ois_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t null_ptr_check(const struct bmi2_ois_dev *ois_dev);

/*!
 * @brief This internal API corrects the gyroscope cross-axis sensitivity
 * between the z and the x axis.
 *
 * @param[in] ois_dev               : Structure instance of bmi2_ois_dev.
 * @param[in] ois_gyr_cross_sens_zx : "gyroscope cross sensitivity value which was calculated during bmi2xy_init(),
 * refer the example ois_accel_gyro.c for more info"
 *
 */
static void bmi2_ois_comp_gyro_cross_axis_sensitivity(struct bmi2_ois_sens_axes_data *ois_data,
                                                      int16_t ois_gyr_cross_sens_zx);

/******************************************************************************/
/*!  @name      User Interface Definitions                            */
/******************************************************************************/

/*!
 * @brief This API reads the data from the given OIS register address of bmi2
 * sensor.
 */
int8_t bmi2_get_ois_regs(uint8_t ois_reg_addr,
                         uint8_t *ois_reg_data,
                         uint16_t data_len,
                         const struct bmi2_ois_dev *ois_dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define dummy byte for SPI read */
    uint8_t dummy_byte = 1;

    /* Variable to define temporary length */
    uint16_t temp_len = data_len + dummy_byte;

    /* Variable to define temporary buffer */
    uint8_t temp_buf[temp_len];

    /* Variable to index bytes read */
    uint16_t index = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(ois_dev);
    if ((rslt == BMI2_OIS_OK) && (ois_reg_data != NULL))
    {
        /* Configuring reg_addr for SPI Interface */
        ois_reg_addr = (ois_reg_addr | BMI2_OIS_SPI_RD_MASK);

        /* Read from OIS register through OIS interface */
        rslt = ois_dev->ois_read(ois_dev->dev_id, ois_reg_addr, temp_buf, temp_len);
        if (rslt == BMI2_OIS_OK)
        {
            /* Read the data from the position next to dummy byte */
            while (index < data_len)
            {
                ois_reg_data[index] = temp_buf[index + dummy_byte];
                index++;
            }
        }
        else
        {
            rslt = BMI2_OIS_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMI2_OIS_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API writes data to the given OIS register address of bmi2 sensor.
 */
int8_t bmi2_set_ois_regs(uint8_t ois_reg_addr,
                         uint8_t *ois_reg_data,
                         uint16_t data_len,
                         const struct bmi2_ois_dev *ois_dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Null-pointer check */
    rslt = null_ptr_check(ois_dev);
    if ((rslt == BMI2_OIS_OK) && (ois_reg_data != NULL))
    {
        /* Configuring reg_addr for SPI Interface */
        ois_reg_addr = (ois_reg_addr & BMI2_OIS_SPI_WR_MASK);

        /* Burst write to OIS register through OIS interface */
        rslt = ois_dev->ois_write(ois_dev->dev_id, ois_reg_addr, ois_reg_data, data_len);
        if (rslt != BMI2_OIS_OK)
        {
            rslt = BMI2_OIS_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMI2_OIS_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API enables/disables accelerometer/gyroscope data read through
 * OIS interface.
 */
int8_t bmi2_set_ois_config(const struct bmi2_ois_dev *ois_dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(ois_dev);
    if (rslt == BMI2_OIS_OK)
    {
        rslt = bmi2_get_ois_regs(BMI2_OIS_CONFIG_ADDR, &reg_data, 1, ois_dev);
        if (rslt == BMI2_OIS_OK)
        {
            /* Enable/Disable accelerometer */
            reg_data = BMI2_OIS_SET_BITS(reg_data, BMI2_OIS_ACC_EN, ois_dev->acc_en);

            /* Enable/Disable gyroscope */
            reg_data = BMI2_OIS_SET_BITS(reg_data, BMI2_OIS_GYR_EN, ois_dev->gyr_en);

            /* Set the OIS configurations */
            rslt = bmi2_set_ois_regs(BMI2_OIS_CONFIG_ADDR, &reg_data, 1, ois_dev);
        }
    }

    return rslt;
}

/*!
 * @brief This API gets the status of accelerometer/gyroscope enable for data
 * read through OIS interface.
 */
int8_t bmi2_get_ois_config(struct bmi2_ois_dev *ois_dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t reg_data = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(ois_dev);
    if (rslt == BMI2_OIS_OK)
    {
        rslt = bmi2_get_ois_regs(BMI2_OIS_CONFIG_ADDR, &reg_data, 1, ois_dev);
        if (rslt == BMI2_OIS_OK)
        {
            /* Get the status of accelerometer enable */
            ois_dev->acc_en = BMI2_OIS_GET_BITS(reg_data, BMI2_OIS_ACC_EN);

            /* Get the status of gyroscope enable */
            ois_dev->gyr_en = BMI2_OIS_GET_BITS(reg_data, BMI2_OIS_GYR_EN);
        }
    }

    return rslt;
}

/*!
 * @brief This API reads accelerometer/gyroscope data through OIS interface.
 */
int8_t bmi2_read_ois_data(const uint8_t *sens_sel,
                          uint8_t n_sens,
                          struct bmi2_ois_dev *ois_dev,
                          int16_t gyr_cross_sens_zx)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define loop */
    uint8_t loop = 0;

    /* Variable to update register address */
    uint8_t reg_addr = 0;

    /* Null-pointer check */
    rslt = null_ptr_check(ois_dev);
    if ((rslt == BMI2_OIS_OK) && (sens_sel != NULL))
    {

        for (loop = 0; loop < n_sens; loop++)
        {
            switch (sens_sel[loop])
            {
                case BMI2_OIS_ACCEL:

                    /* Update OIS accelerometer address */
                    reg_addr = BMI2_OIS_ACC_X_LSB_ADDR;

                    /* Get OIS accelerometer data */
                    rslt = get_ois_acc_gyr_data(&ois_dev->acc_data, reg_addr, ois_dev, 0);
                    break;
                case BMI2_OIS_GYRO:

                    /* Update OIS gyroscope address */
                    reg_addr = BMI2_OIS_GYR_X_LSB_ADDR;

                    /* Get OIS gyroscope data */
                    rslt = get_ois_acc_gyr_data(&ois_dev->gyr_data, reg_addr, ois_dev, gyr_cross_sens_zx);
                    break;
                default:
                    rslt = BMI2_OIS_E_INVALID_SENSOR;
                    break;
            }

            /* Return error if any of the get sensor data fails */
            if (rslt != BMI2_OIS_OK)
            {
                break;
            }
        }
    }
    else
    {
        rslt = BMI2_OIS_E_NULL_PTR;
    }

    return rslt;
}

/***************************************************************************/

/*!         Local Function Definitions
 ****************************************************************************/

/*!
 * @brief This internal API gets the accelerometer and the gyroscope data.
 */
static int8_t get_ois_acc_gyr_data(struct bmi2_ois_sens_axes_data *ois_data,
                                   uint8_t reg_addr,
                                   const struct bmi2_ois_dev *ois_dev,
                                   int16_t ois_gyr_cross_sens_zx)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variables to store MSB value */
    uint8_t msb;

    /* Variables to store LSB value */
    uint8_t lsb;

    /* Variables to store both MSB and LSB value */
    uint16_t msb_lsb;

    /* Variables to define index */
    uint8_t index = 0;

    /* Array to define data stored in register */
    uint8_t reg_data[BMI2_OIS_ACC_GYR_NUM_BYTES] = { 0 };

    /* Read the sensor data */
    rslt = bmi2_get_ois_regs(reg_addr, reg_data, BMI2_OIS_ACC_GYR_NUM_BYTES, ois_dev);
    if (rslt == BMI2_OIS_OK)
    {
        /* Read x-axis data */
        lsb = reg_data[index++];
        msb = reg_data[index++];
        msb_lsb = ((uint16_t)msb << 8) | (uint16_t)lsb;
        ois_data->x = (int16_t)msb_lsb;

        /* Read y-axis data */
        lsb = reg_data[index++];
        msb = reg_data[index++];
        msb_lsb = ((uint16_t)msb << 8) | (uint16_t)lsb;
        ois_data->y = (int16_t)msb_lsb;

        /* Read z-axis data */
        lsb = reg_data[index++];
        msb = reg_data[index++];
        msb_lsb = ((uint16_t)msb << 8) | (uint16_t)lsb;
        ois_data->z = (int16_t)msb_lsb;

        bmi2_ois_comp_gyro_cross_axis_sensitivity(ois_data, ois_gyr_cross_sens_zx);
    }

    return rslt;
}

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct bmi2_ois_dev *ois_dev)
{
    /* Variable to define error */
    int8_t rslt = BMI2_OIS_OK;

    if ((ois_dev == NULL) || (ois_dev->ois_read == NULL) || (ois_dev->ois_write == NULL) ||
        (ois_dev->ois_delay_ms == NULL))
    {
        /* Device structure pointer is NULL */
        rslt = BMI2_OIS_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API corrects the gyroscope cross-axis sensitivity
 * between the z and the x axis.
 */
static void bmi2_ois_comp_gyro_cross_axis_sensitivity(struct bmi2_ois_sens_axes_data *ois_data,
                                                      int16_t ois_gyr_cross_sens_zx)
{

    /* Get the compensated gyroscope x-axis */
    ois_data->x = ois_data->x - (int16_t)(((int32_t)ois_gyr_cross_sens_zx * (int32_t)ois_data->z) / 512);
}
