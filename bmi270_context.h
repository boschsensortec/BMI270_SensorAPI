/**
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file       bmi270_context.h
 * @date       2020-06-05
 * @version    v2.53.2
 *
 */

/**
 * \ingroup bmi2xy
 * \defgroup bmi270_context BMI270_CONTEXT
 * @brief Sensor driver for BMI270_CONTEXT sensor
 */

#ifndef BMI270_CONTEXT_H_
#define BMI270_CONTEXT_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************/

/*!             Header files
 ****************************************************************************/
#include "bmi2.h"

/***************************************************************************/

/*!               Macro definitions
 ****************************************************************************/

/*! @name BMI270_CONTEXT Chip identifier */
#define BMI270_CONTEXT_CHIP_ID                       UINT8_C(0x24)

/*! @name BMI270_CONTEXT feature input start addresses */
#define BMI270_CONTEXT_STEP_CNT_1_STRT_ADDR          UINT8_C(0x00)
#define BMI270_CONTEXT_STEP_CNT_4_STRT_ADDR          UINT8_C(0x02)
#define BMI270_CONTEXT_MAX_BURST_LEN_STRT_ADDR       UINT8_C(0x08)
#define BMI270_CONTEXT_CRT_GYRO_SELF_TEST_STRT_ADDR  UINT8_C(0x09)
#define BMI270_CONTEXT_ABORT_STRT_ADDR               UINT8_C(0x09)
#define BMI270_CONTEXT_NVM_PROG_PREP_STRT_ADDR       UINT8_C(0x0A)
#define BMI270_CONTEXT_ACT_RGN_SETT_STRT_ADDR        UINT8_C(0x00)
#define BMI270_CONTEXT_ACT_RGN_STRT_ADDR             UINT8_C(0x0A)

/*! @name BMI270_CONTEXT feature output start addresses */
#define BMI270_CONTEXT_STEP_CNT_OUT_STRT_ADDR        UINT8_C(0x00)
#define BMI270_CONTEXT_GYR_USER_GAIN_OUT_STRT_ADDR   UINT8_C(0x04)
#define BMI270_CONTEXT_GYRO_CROSS_SENSE_STRT_ADDR    UINT8_C(0x0C)
#define BMI270_CONTEXT_NVM_VFRM_OUT_STRT_ADDR        UINT8_C(0x0E)

/*! @name Defines maximum number of pages */
#define BMI270_CONTEXT_MAX_PAGE_NUM                  UINT8_C(8)

/*! @name Defines maximum number of feature input configurations */
#define BMI270_CONTEXT_MAX_FEAT_IN                   UINT8_C(9)

/*! @name Defines maximum number of feature outputs */
#define BMI270_CONTEXT_MAX_FEAT_OUT                  UINT8_C(5)

/*! @name Mask definitions for feature interrupt status bits */
#define BMI270_CONTEXT_STEP_CNT_STATUS_MASK          UINT8_C(0x01)

/***************************************************************************/

/*!     BMI270_CONTEXT User Interface function prototypes
 ****************************************************************************/

/**
 * \ingroup bmi270_context
 * \defgroup bmi270_contextApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bmi270_contextApiInit
 * \page bmi270_context_api_bmi270_context_init bmi270_context_init
 * \code
 * int8_t bmi270_context_init(struct bmi2_dev *dev);
 * \endcode
 * @details This API:
 *  1) updates the device structure with address of the configuration file.
 *  2) Initializes BMI270_CONTEXT sensor.
 *  3) Writes the configuration file.
 *  4) Updates the feature offset parameters in the device structure.
 *  5) Updates the maximum number of pages, in the device structure.
 *
 * @param[in, out] dev      : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 *
 * @retval BMI2_OK - Success
 * @retval BMI2_E_NULL_PTR - Error: Null pointer error
 * @retval BMI2_E_COM_FAIL - Error: Communication fail
 * @retval BMI2_E_DEV_NOT_FOUND - Invalid device
 */
int8_t bmi270_context_init(struct bmi2_dev *dev);

/******************************************************************************/
/*! @name       C++ Guard Macros                                      */
/******************************************************************************/
#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* BMI270_CONTEXT_H_ */
