/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *           Copyright 2014 Big Switch Networks, Inc.
 *           Copyright 2014 Accton Technology Corporation.
 *
 * Licensed under the Eclipse Public License, Version 1.0 (the
 * "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 *
 *        http://www.eclipse.org/legal/epl-v10.html
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the
 * License.
 *
 * </bsn.cl>
 ************************************************************
 *
 * Thermal Sensor Platform Implementation.
 *
 ***********************************************************/
#include <onlplib/file.h>
#include <onlp/platformi/thermali.h>
#include "platform_lib.h"

#define VALIDATE(_id)                           \
    do {                                        \
        if(!ONLP_OID_IS_THERMAL(_id)) {         \
            return ONLP_STATUS_E_INVALID;       \
        }                                       \
    } while(0)

struct threshold_t {
    char *warning;
    char *error;
    char *shutdown;
};

struct threshold_t threshold[TOTAL_THERMAL_COUNT+1] =
{
    [0].warning = "reserved",
    [0].error = "reserved",
    [0].shutdown = "reserved",
    [THERMAL_1_ON_MAINBOARD].warning  = "/sys/bus/platform/devices/8730_thermal/temp1_warning",
    [THERMAL_1_ON_MAINBOARD].error    = "/sys/bus/platform/devices/8730_thermal/temp1_error",
    [THERMAL_1_ON_MAINBOARD].shutdown = "/sys/bus/platform/devices/8730_thermal/temp1_shutdown",
    [THERMAL_2_ON_MAINBOARD].warning  = "/sys/bus/platform/devices/8730_thermal/temp2_warning",
    [THERMAL_2_ON_MAINBOARD].error    = "/sys/bus/platform/devices/8730_thermal/temp2_error",
    [THERMAL_2_ON_MAINBOARD].shutdown = "/sys/bus/platform/devices/8730_thermal/temp2_shutdown",
    [THERMAL_3_ON_MAINBOARD].warning  = "/sys/bus/platform/devices/8730_thermal/temp3_warning",
    [THERMAL_3_ON_MAINBOARD].error    = "/sys/bus/platform/devices/8730_thermal/temp3_error",
    [THERMAL_3_ON_MAINBOARD].shutdown = "/sys/bus/platform/devices/8730_thermal/temp3_shutdown",
    [THERMAL_4_ON_MAINBOARD].warning  = "/sys/bus/platform/devices/8730_thermal/temp4_warning",
    [THERMAL_4_ON_MAINBOARD].error    = "/sys/bus/platform/devices/8730_thermal/temp4_error",
    [THERMAL_4_ON_MAINBOARD].shutdown = "/sys/bus/platform/devices/8730_thermal/temp4_shutdown",
    [THERMAL_1_ON_PSU1].warning  = "/sys/bus/platform/devices/8730_psu/psu1_temp1_warning",
    [THERMAL_1_ON_PSU1].error    = "/sys/bus/platform/devices/8730_psu/psu1_temp1_error",
    [THERMAL_1_ON_PSU1].shutdown = "/sys/bus/platform/devices/8730_psu/psu1_temp1_shutdown",
    [THERMAL_2_ON_PSU1].warning  = "/sys/bus/platform/devices/8730_psu/psu1_temp2_warning",
    [THERMAL_2_ON_PSU1].error    = "/sys/bus/platform/devices/8730_psu/psu1_temp2_error",
    [THERMAL_2_ON_PSU1].shutdown = "/sys/bus/platform/devices/8730_psu/psu1_temp2_shutdown",
    [THERMAL_3_ON_PSU1].warning  = "/sys/bus/platform/devices/8730_psu/psu1_temp3_warning",
    [THERMAL_3_ON_PSU1].error    = "/sys/bus/platform/devices/8730_psu/psu1_temp3_error",
    [THERMAL_3_ON_PSU1].shutdown = "/sys/bus/platform/devices/8730_psu/psu1_temp3_shutdown",
    [THERMAL_1_ON_PSU2].warning  = "/sys/bus/platform/devices/8730_psu/psu2_temp1_warning",
    [THERMAL_1_ON_PSU2].error    = "/sys/bus/platform/devices/8730_psu/psu2_temp1_error",
    [THERMAL_1_ON_PSU2].shutdown = "/sys/bus/platform/devices/8730_psu/psu2_temp1_shutdown",
    [THERMAL_2_ON_PSU2].warning  = "/sys/bus/platform/devices/8730_psu/psu2_temp2_warning",
    [THERMAL_2_ON_PSU2].error    = "/sys/bus/platform/devices/8730_psu/psu2_temp2_error",
    [THERMAL_2_ON_PSU2].shutdown = "/sys/bus/platform/devices/8730_psu/psu2_temp2_shutdown",
    [THERMAL_3_ON_PSU2].warning  = "/sys/bus/platform/devices/8730_psu/psu2_temp3_warning",
    [THERMAL_3_ON_PSU2].error    = "/sys/bus/platform/devices/8730_psu/psu2_temp3_error",
    [THERMAL_3_ON_PSU2].shutdown = "/sys/bus/platform/devices/8730_psu/psu2_temp3_shutdown",
};

static char* devfiles__[] =  /* must map with onlp_thermal_id (platform_lib.h) */
{
    "reserved",
	"/sys/bus/platform/devices/8730_thermal/temp1_input",
	"/sys/bus/platform/devices/8730_thermal/temp2_input",
	"/sys/bus/platform/devices/8730_thermal/temp3_input",
	"/sys/bus/platform/devices/8730_thermal/temp4_input",
	"/sys/bus/platform/devices/8730_psu/psu1_temp1_input",
	"/sys/bus/platform/devices/8730_psu/psu1_temp2_input",
	"/sys/bus/platform/devices/8730_psu/psu1_temp3_input",
	"/sys/bus/platform/devices/8730_psu/psu2_temp1_input",
	"/sys/bus/platform/devices/8730_psu/psu2_temp2_input",
	"/sys/bus/platform/devices/8730_psu/psu2_temp3_input",
};

/* Static values */
static onlp_thermal_info_t linfo[] = {
    { }, /* Not used */
    {   { ONLP_THERMAL_ID_CREATE(THERMAL_1_ON_MAINBOARD), "CPU Core", 0},
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, {76000, 80000, 84000}
    },
    {   { ONLP_THERMAL_ID_CREATE(THERMAL_2_ON_MAINBOARD), "Chassis Thermal Sensor 1 (AFO)", 0},
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, {54000, 60000, 66000}
    },
    {   { ONLP_THERMAL_ID_CREATE(THERMAL_3_ON_MAINBOARD), "Chassis Thermal Sensor 2 (Hot Spot)", 0},
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, {76000, 80000, 84000}
    },
    {   { ONLP_THERMAL_ID_CREATE(THERMAL_4_ON_MAINBOARD), "Chassis Thermal Sensor 3 (AFI)", 0},
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, {54000, 60000, 66000}
    },
    {   { ONLP_THERMAL_ID_CREATE(THERMAL_1_ON_PSU1), "PSU-1 Thermal Sensor 1", ONLP_PSU_ID_CREATE(PSU1_ID)},
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, {76000, 80000, 84000}
    },
    {   { ONLP_THERMAL_ID_CREATE(THERMAL_2_ON_PSU1), "PSU-1 Thermal Sensor 2", ONLP_PSU_ID_CREATE(PSU1_ID)},
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, {76000, 80000, 84000}
    },
    {   { ONLP_THERMAL_ID_CREATE(THERMAL_3_ON_PSU1), "PSU-1 Thermal Sensor 3", ONLP_PSU_ID_CREATE(PSU1_ID)},
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, {76000, 80000, 84000}
    },
    {   { ONLP_THERMAL_ID_CREATE(THERMAL_1_ON_PSU2), "PSU-2 Thermal Sensor 1", ONLP_PSU_ID_CREATE(PSU2_ID)},
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, {76000, 80000, 84000}
    },
    {   { ONLP_THERMAL_ID_CREATE(THERMAL_2_ON_PSU2), "PSU-2 Thermal Sensor 2", ONLP_PSU_ID_CREATE(PSU2_ID)},
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, {76000, 80000, 84000}
    },
    {   { ONLP_THERMAL_ID_CREATE(THERMAL_3_ON_PSU2), "PSU-2 Thermal Sensor 3", ONLP_PSU_ID_CREATE(PSU2_ID)},
        ONLP_THERMAL_STATUS_PRESENT,
        ONLP_THERMAL_CAPS_ALL, 0, {76000, 80000, 84000}
    },
};

/*
 * This will be called to intiialize the thermali subsystem.
 */
int
onlp_thermali_init(void)
{
    DIAG_PRINT("%s", __FUNCTION__);
    return ONLP_STATUS_OK;
}

/*
 * Retrieve the information structure for the given thermal OID.
 *
 * If the OID is invalid, return ONLP_E_STATUS_INVALID.
 * If an unexpected error occurs, return ONLP_E_STATUS_INTERNAL.
 * Otherwise, return ONLP_STATUS_OK with the OID's information.
 *
 * Note -- it is expected that you fill out the information
 * structure even if the sensor described by the OID is not present.
 */
int
onlp_thermali_info_get(onlp_oid_t id, onlp_thermal_info_t* info)
{
    int   tid;
    VALIDATE(id);

    tid = ONLP_OID_ID_GET(id);

    /* Set the onlp_oid_hdr_t and capabilities */
    *info = linfo[tid];

    onlp_file_read_int(&info->thresholds.warning, threshold[tid].warning);
    onlp_file_read_int(&info->thresholds.error, threshold[tid].error);
    onlp_file_read_int(&info->thresholds.shutdown, threshold[tid].shutdown);

    return onlp_file_read_int(&info->mcelsius, devfiles__[tid]);
}

