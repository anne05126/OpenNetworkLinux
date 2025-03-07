/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *           Copyright 2014 Big Switch Networks, Inc.
 *           Copyright 2018 Alpha Networks Incorporation.
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
 *
 *
 ***********************************************************/
#ifndef __PLATFORM_LIB_H__
#define __PLATFORM_LIB_H__

#include <onlplib/file.h>
#include "x86_64_extremenetworks_8730_32d_log.h"


#define PSU1_ID 1
#define PSU2_ID 2
#define PSU_FAN1_ID 1
#define PSU_FAN2_ID 2

#define CHASSIS_LED_COUNT       5
#define CHASSIS_PSU_COUNT       2
#define PER_PSU_FAN_COUNT   	2
#define PER_PSU_THERMAL_COUNT   3

#define CHASSIS_FAN_COUNT (ONLP_FAN_ID_MAX - (CHASSIS_PSU_COUNT * PER_PSU_FAN_COUNT) - 1)
#define CHASSIS_THERMAL_COUNT (ONLP_THERMAL_ID_MAX - (CHASSIS_PSU_COUNT * PER_PSU_THERMAL_COUNT) - 1)
#define TOTAL_THERMAL_COUNT (ONLP_THERMAL_ID_MAX - 1)


#define PSU_HWMON_PREFIX            	"/sys/bus/platform/devices/8730_psu/"
#define PSU_EEPROM_PREFIX            	"/sys/bus/platform/devices/8730_psu/"

#define FAN_BOARD_PATH	                "/sys/bus/platform/devices/8730_fan/"

#define ONIE_EEPROM_PATH                "/sys/bus/i2c/devices/41-0055/eeprom"

#define NUM_OF_SFP_PORT 	                32      /* 32 * 400G */
#define NUM_OF_IOBM_PORT 	       		 	3       /* 1 * 100G, 2 * 10G */
#define NUM_OF_IOBM_QSFP28_PORT 	        1       /* 1 * 100G */
#define NUM_OF_IOBM_SFP_PORT 	            2       /* 2 * 10G */
#define NUM_OF_QSFP_PORT_CPLD 	            2
#define NUM_OF_QSFP_PER_PORT_CPLD           2

#define SFP_START_INDEX                     0       /* Both QSFP and SFP */
#define SFP_PLUS_EEPROM_I2C_ADDR            0x50    /* SFP+ EEPROM Physical Address in the I2C */  
#define SFP_DOM_EEPROM_I2C_ADDR             0x51


#define QSFP_PORT_INDEX_START               0
#define QSFP_PORT_INDEX_END                 31
#define IOBM_QSFP28_PORT_INDEX              32
#define IOBM_SFP_PORT_INDEX_START           33
#define IOBM_SFP_PORT_INDEX_END             34



#define IS_QSFP_PORT(_port) (_port >= QSFP_PORT_INDEX_START && _port <= QSFP_PORT_INDEX_END)
#define IS_IOBM_PORT(_port) (_port >= IOBM_QSFP28_PORT_INDEX && _port <= IOBM_SFP_PORT_INDEX_END)
#define IS_IOBM_QSFP28_PORT(_port) (_port == IOBM_QSFP28_PORT_INDEX)
#define IS_IOBM_SFP_PORT(_port) (_port >= IOBM_SFP_PORT_INDEX_START && _port <= IOBM_SFP_PORT_INDEX_END)

typedef enum psu_type {
    PSU_TYPE_UNKNOWN,
    PSU_TYPE_AC_F2B,
    PSU_TYPE_AC_B2F,
    PSU_TYPE_DC_48V_F2B,
    PSU_TYPE_DC_48V_B2F
} psu_type_t;


enum onlp_thermal_id
{
    THERMAL_RESERVED = 0,
    THERMAL_1_ON_MAINBOARD,  /* Main Board Bottom CPU Temp */
    THERMAL_2_ON_MAINBOARD,  /* Main Board Bottom TMP75_0 Temp (AFO)*/
    THERMAL_3_ON_MAINBOARD,  /* Main Board Bottom TMP75_1 Temp (HOT Spot) */
    THERMAL_4_ON_MAINBOARD,  /* Main Board Bottom TMP75_2 Temp (AFI) */
    THERMAL_1_ON_PSU1,
    THERMAL_2_ON_PSU1,
    THERMAL_3_ON_PSU1,
    THERMAL_1_ON_PSU2,
    THERMAL_2_ON_PSU2,
    THERMAL_3_ON_PSU2,
    ONLP_THERMAL_ID_MAX,
};


enum fan_id {
    FAN_1_ON_FAN_BOARD = 1,
    FAN_2_ON_FAN_BOARD,
    FAN_3_ON_FAN_BOARD,
    FAN_4_ON_FAN_BOARD,
    FAN_5_ON_FAN_BOARD,
    FAN_6_ON_FAN_BOARD,
    FAN_7_ON_FAN_BOARD,
    FAN_1_ON_PSU_1,
    FAN_2_ON_PSU_1,
    FAN_1_ON_PSU_2,
    FAN_2_ON_PSU_2,
    ONLP_FAN_ID_MAX,
};

/* FAN related data
 */
enum onlp_fan_id
{
    FAN_RESERVED = 0,
    FAN_1,
    FAN_2,
    FAN_3,
    FAN_4,
    FAN_1_ON_PSU1,
    FAN_1_ON_PSU2,
};


/* 
 * LED ID (need to sync with "enum onlp_led_id" defined in ledi.c)
 */

enum onlp_led_id
{
    LED_RESERVED = 0,
	LED_PWR,
    LED_STAT,
    LED_FAN,
    LED_PSU,
    LED_SEC  
};


#define DIAG_FLAG_ON 1
#define DIAG_FLAG_OFF 0
char diag_flag_set(char d);
char diag_flag_get(void);

char diag_debug_trace_on(void);
char diag_debug_trace_off(void);
char diag_debug_trace_check(void);

char diag_debug_pause_platform_manage_on(void);
char diag_debug_pause_platform_manage_off(void);
char diag_debug_pause_platform_manage_check(void);

/*
* TLV parsering for specific type code
*/
int eeprom_tlv_read(uint8_t *rdata, int type, char *data);

#define DIAG_TRACE(fmt,args...) if(diag_debug_trace_check()) printf("\n[TRACE]"fmt"\n", args)
#define DIAG_PRINT(fmt,args...) DIAG_TRACE(fmt,args);else if(diag_flag_get()) printf("[DIAG]"fmt"\n", args) 

char* sfp_control_to_str(int value);
char *rtrim(char *str);
char *ltrim(char *str);
char *trim(char *str);
psu_type_t psu_type_get(int id, char* modelname, int modelname_len);

#endif  /* __PLATFORM_LIB_H__ */
