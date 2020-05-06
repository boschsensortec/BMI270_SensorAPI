/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmi2.h"
#include "bmi270.h"
#include "bmi2_common.h"

/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program. */
int main(void)
{

    /*! Sensor initialization configuration. */
    struct bmi2_dev bmi2_dev;

    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Initialize the dev structure */
    rslt = bmi2_interface_selection(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmi270. */
    rslt = bmi270_init(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* This API runs the CRT process. */
        rslt = bmi2_do_crt(&bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        /* Do not move the board while doing CRT. If so, it will throw an abort error. */
        if (rslt == BMI2_OK)
        {
            printf("CRT successfully completed.");
        }
    }

    return rslt;
}
