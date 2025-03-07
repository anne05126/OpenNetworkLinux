/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *           Copyright 2014 Big Switch Networks, Inc.
 *           Copyright 2017 Accton Technology Corporation.
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
#include <onlplib/file.h>
#include <onlp/platformi/ledi.h>
#include "platform_lib.h"

#define LED_FORMAT      "/sys/bus/platform/devices/7830_bmc_led/%s"
#define LED_FORMAT_I2C  "/sys/bus/i2c/devices/0-005f/%s"

/* PWR_LED(POWER_LED) */
#define PWR_LED_MODE_OFF               		0x0
#define PWR_LED_MODE_GREEN                	0x1

/* STAT_LED(STATUS_LED) */
#define STAT_LED_MODE_OFF                 	0x0
#define STAT_LED_MODE_GREEN               	0x1
#define STAT_LED_MODE_AMBER               	0x2
#define STAT_LED_MODE_GREEN_AMBER_BLINKING 	0x3

/* FAN_LED */
#define FAN_LED_MODE_OFF                    0x0
#define FAN_LED_MODE_GREEN                  0x1
#define FAN_LED_MODE_AMBER                  0x2
#define FAN_LED_MODE_GREEN_AMBER_BLINKING   0x3

/* PSU_LED */
#define PSU_LED_MODE_OFF                   	0x0
#define PSU_LED_MODE_GREEN_BLINKING        	0x1
#define PSU_LED_MODE_GREEN			        0x2
#define PSU_LED_MODE_AMBER_BLINKING        	0x3
#define PSU_LED_MODE_AMBER        			0x4
#define PSU_LED_MODE_GREEN_AMBER_BLINKING  	0x5

/* SEC_LED(Security_LED) */
#define SEC_LED_MODE_OFF               		0x0
#define SEC_LED_MODE_BLUE              		0x1


#define VALIDATE(_id)                           \
    do {                                        \
        if(!ONLP_OID_IS_LED(_id)) {             \
            return ONLP_STATUS_E_INVALID;       \
        }                                       \
    } while(0)


typedef struct led_light_mode_map {
    enum onlp_led_id id;
    int driver_led_mode;
    enum onlp_led_mode_e onlp_led_mode;
} led_light_mode_map_t;

led_light_mode_map_t led_map[] = {
	{LED_PWR,    	PWR_LED_MODE_OFF,            		ONLP_LED_MODE_OFF},
    {LED_PWR,    	PWR_LED_MODE_GREEN,          		ONLP_LED_MODE_GREEN},
    
    {LED_STAT,  	STAT_LED_MODE_OFF,             		ONLP_LED_MODE_OFF},
    {LED_STAT,  	STAT_LED_MODE_GREEN,           		ONLP_LED_MODE_GREEN},
    {LED_STAT,  	STAT_LED_MODE_AMBER,           		ONLP_LED_MODE_ORANGE},
    {LED_STAT,     	STAT_LED_MODE_GREEN_AMBER_BLINKING, ONLP_LED_MODE_BLINKING},   

    {LED_FAN,     	FAN_LED_MODE_OFF,             		ONLP_LED_MODE_OFF},
    {LED_FAN,     	FAN_LED_MODE_GREEN,           		ONLP_LED_MODE_GREEN},
    {LED_FAN,     	FAN_LED_MODE_AMBER,           		ONLP_LED_MODE_ORANGE},
    {LED_FAN,     	FAN_LED_MODE_GREEN_AMBER_BLINKING,  ONLP_LED_MODE_BLINKING},

    {LED_PSU,     	PSU_LED_MODE_OFF,             		ONLP_LED_MODE_OFF},
    {LED_PSU,  		PSU_LED_MODE_GREEN,           		ONLP_LED_MODE_GREEN},
    {LED_PSU,  		PSU_LED_MODE_GREEN_BLINKING,  		ONLP_LED_MODE_GREEN_BLINKING},
    {LED_PSU,     	PSU_LED_MODE_AMBER,           		ONLP_LED_MODE_ORANGE},
    {LED_PSU,		PSU_LED_MODE_AMBER_BLINKING,  		ONLP_LED_MODE_ORANGE_BLINKING},
    {LED_PSU,     	PSU_LED_MODE_GREEN_AMBER_BLINKING,  ONLP_LED_MODE_BLINKING},

	{LED_SEC,		SEC_LED_MODE_OFF,            		ONLP_LED_MODE_OFF},
    {LED_SEC,  		SEC_LED_MODE_BLUE,           		ONLP_LED_MODE_BLUE},   

};

static char *leds[] =  /* must map with onlp_led_id (platform_lib.h) */
{
    "reserved",
	"led_pwr",
	"led_status",
	"led_fan",    
    "led_psu",
    "led_security",
};

/*
 * Get the information for the given LED OID.
 */
static onlp_led_info_t linfo[] =
{
    { }, /* Not used */
    {
        { ONLP_LED_ID_CREATE(LED_PWR), "Chassis LED 1 (POWER LED)", 0 },
        ONLP_LED_STATUS_PRESENT,
        ONLP_LED_CAPS_ON_OFF | ONLP_LED_CAPS_GREEN,
    },
    {
        { ONLP_LED_ID_CREATE(LED_STAT), "Chassis LED 2 (STATUS LED)", 0 },
        ONLP_LED_STATUS_PRESENT,
        ONLP_LED_CAPS_ON_OFF | ONLP_LED_CAPS_GREEN | ONLP_LED_CAPS_ORANGE | ONLP_LED_CAPS_GREEN_BLINKING | ONLP_LED_CAPS_ORANGE_BLINKING,
    },
    {
        { ONLP_LED_ID_CREATE(LED_FAN), "Chassis LED 3 (FAN LED)", 0 },
        ONLP_LED_STATUS_PRESENT,
        ONLP_LED_CAPS_ON_OFF | ONLP_LED_CAPS_GREEN | ONLP_LED_CAPS_ORANGE | ONLP_LED_CAPS_GREEN_BLINKING | ONLP_LED_CAPS_ORANGE_BLINKING,
    },
    {
        { ONLP_LED_ID_CREATE(LED_PSU), "Chassis LED 4 (PSU LED)", 0 },
        ONLP_LED_STATUS_PRESENT,
        ONLP_LED_CAPS_ON_OFF | ONLP_LED_CAPS_GREEN | ONLP_LED_CAPS_ORANGE | ONLP_LED_CAPS_GREEN_BLINKING | ONLP_LED_CAPS_ORANGE_BLINKING,
    },
    {
        { ONLP_LED_ID_CREATE(LED_SEC), "Chassis LED 5 (SECURITY LED)", 0 },
        ONLP_LED_STATUS_PRESENT,
        ONLP_LED_CAPS_ON_OFF | ONLP_LED_CAPS_BLUE,
    }
};

static int driver_to_onlp_led_mode(enum onlp_led_id id, int driver_led_mode)
{
    int i, nsize = sizeof(led_map)/sizeof(led_map[0]);

    for (i = 0; i < nsize; i++)
    {
        if (id == led_map[i].id && driver_led_mode == led_map[i].driver_led_mode)
        {
            DIAG_PRINT("%s, id:%d, driver_led_mode:%d to onlp_led_mode:%d", 
                __FUNCTION__, id, driver_led_mode, led_map[i].onlp_led_mode);
            return led_map[i].onlp_led_mode;
        }
    }

    return 0;
}

static int onlp_to_driver_led_mode(enum onlp_led_id id, onlp_led_mode_t onlp_led_mode)
{
    int i, nsize = sizeof(led_map)/sizeof(led_map[0]);

    for(i = 0; i < nsize; i++)
    {
        if (id == led_map[i].id && onlp_led_mode == led_map[i].onlp_led_mode)
        {
            DIAG_PRINT("%s, id:%d, onlp_led_mode:%d to driver_led_mode:%d", 
                __FUNCTION__, id, onlp_led_mode, led_map[i].driver_led_mode);
            return led_map[i].driver_led_mode;
        }
    }

    return 0;
}

/*
 * This function will be called prior to any other onlp_ledi_* functions.
 */
int
onlp_ledi_init(void)
{
    DIAG_PRINT("%s", __FUNCTION__);

    return ONLP_STATUS_OK;
}

int
onlp_ledi_info_get(onlp_oid_t id, onlp_led_info_t* info)
{
    int  lid, value;

    VALIDATE(id);

    lid = ONLP_OID_ID_GET(id);

    /* Set the onlp_oid_hdr_t and capabilities */
    *info = linfo[ONLP_OID_ID_GET(id)];

    
    switch (lid)
    {
        case LED_PWR:
        case LED_SEC:
            /* Get LED mode from CPU board */
            if (onlp_file_read_int(&value, LED_FORMAT_I2C, leds[lid]) < 0) 
            {
                AIM_LOG_ERROR("[CPU] Unable to read status from file "LED_FORMAT, leds[lid]);
                return ONLP_STATUS_E_INTERNAL;
            }
            break;
        case LED_STAT:
        case LED_FAN:
        case LED_PSU:
            /* Get LED mode from BMC */
            if (onlp_file_read_int(&value, LED_FORMAT, leds[lid]) < 0) 
            {
                AIM_LOG_ERROR("[BMC] Unable to read status from file "LED_FORMAT, leds[lid]);
                return ONLP_STATUS_E_INTERNAL;
            }
            break;
        default:
            break;
    }
    

    info->mode = driver_to_onlp_led_mode(lid, value);

    /* Set the on/off status */
    if (info->mode != ONLP_LED_MODE_OFF) 
    {
        info->status |= ONLP_LED_STATUS_ON;
    }

    return ONLP_STATUS_OK;
}

/*
 * Turn an LED on or off.
 *
 * This function will only be called if the LED OID supports the ONOFF
 * capability.
 *
 * What 'on' means in terms of colors or modes for multimode LEDs is
 * up to the platform to decide. This is intended as baseline toggle mechanism.
 */
int
onlp_ledi_set(onlp_oid_t id, int on_or_off)
{
    VALIDATE(id);

    if (!on_or_off) 
    {
        return onlp_ledi_mode_set(id, ONLP_LED_MODE_OFF);
    }
    else
    {
        /*Just pick a color to light*/
        int rv, i;
        onlp_led_info_t info;
        uint32_t caps;

        rv = onlp_ledi_info_get(id, &info);
        if (rv < 0)
            return rv;

        caps = info.caps;
        /*Bit scan*/
        for (i = 1; i < sizeof(caps)*8; i++) 
        {
            if( caps & (1<<i)) {
                return onlp_ledi_mode_set(id, i);
            }
        }
    }
	
    return ONLP_STATUS_E_UNSUPPORTED;
}

/*
 * This function puts the LED into the given mode. It is a more functional
 * interface for multimode LEDs.
 *
 * Only modes reported in the LED's capabilities will be attempted.
 */
int
onlp_ledi_mode_set(onlp_oid_t id, onlp_led_mode_t mode)
{
    int  lid;
    VALIDATE(id);

    lid = ONLP_OID_ID_GET(id);
    switch (lid)
    {
        case LED_PWR:
        case LED_SEC:
            /* Set from CPU to cpld */
            if (onlp_file_write_int(onlp_to_driver_led_mode(lid , mode), LED_FORMAT_I2C, leds[lid]) < 0) 
            {
                return ONLP_STATUS_E_INTERNAL;
            }
            break;
        case LED_FAN:
        case LED_PSU:
            /* Set from BMC to cpld */
            if (onlp_file_write_int(onlp_to_driver_led_mode(lid , mode), LED_FORMAT, leds[lid]) < 0) 
            {
                return ONLP_STATUS_E_INTERNAL;
            }
            break;
        case LED_STAT:
            return ONLP_STATUS_E_UNSUPPORTED;
            break;
        
        default:
            break;
    }
    

    return ONLP_STATUS_OK;
}

