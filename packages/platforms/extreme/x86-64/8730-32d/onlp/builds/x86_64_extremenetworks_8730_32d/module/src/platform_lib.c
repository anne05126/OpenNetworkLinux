/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *           Copyright 2014 Big Switch Networks, Inc.
 *           Copyright 2018 Alpha Networks Incorporation
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
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <onlplib/file.h>
#include <AIM/aim.h>
#include <onlp/platformi/sfpi.h>
#include "platform_lib.h"
#include <ctype.h>

#define DEBUG_FLAG 0
#define PSU_MODEL_NAME_LEN          10


static char diag_flag=0;

char diag_flag_set(char d)
{
    diag_flag = d;
    return 0;
}

char diag_flag_get(void)
{
    return diag_flag;
}

char diag_debug_trace_on(void)
{
    system("echo 1 > /tmp/onlpi_dbg_trace");
    return 0;
}

char diag_debug_trace_off(void)
{
    system("echo 0 > /tmp/onlpi_dbg_trace");
    return 0;
}

char diag_debug_trace_check(void)
{
    char flag = 0;
    FILE* file = fopen ("/tmp/onlpi_dbg_trace", "r");
    if (file == NULL)
    {
        return 0;
    }
    flag = fgetc (file);
    fclose (file);

    return (flag == '1')?1:0;
}

char* sfp_control_to_str(int value)
{
    switch (value)
    {
        case ONLP_SFP_CONTROL_RESET:
            return "RESET";
        case ONLP_SFP_CONTROL_RESET_STATE:
            return "RESET_STATE";
        case ONLP_SFP_CONTROL_RX_LOS:
            return "RX_LOS";
        case ONLP_SFP_CONTROL_TX_FAULT:
            return "TX_FAULT";
        case ONLP_SFP_CONTROL_TX_DISABLE:
            return "TX_DISABLE";
        case ONLP_SFP_CONTROL_TX_DISABLE_CHANNEL:
            return "TX_DISABLE_CHANNEL";
        case ONLP_SFP_CONTROL_LP_MODE:
            return "LP_MODE";
        case ONLP_SFP_CONTROL_POWER_OVERRIDE:
            return "POWER_OVERRIDE";

        default:
            return "UNKNOW";
    }
    return "";
}

char diag_debug_pause_platform_manage_on(void)
{
    system("echo 1 > /tmp/onlpi_dbg_pause_pm");
    return 0;
}

char diag_debug_pause_platform_manage_off(void)
{
    system("echo 0 > /tmp/onlpi_dbg_pause_pm");
    return 0;
}

char diag_debug_pause_platform_manage_check(void)
{
    char flag = 0;
    FILE* file = fopen ("/tmp/onlpi_dbg_pause_pm", "r");
    if (file == NULL)
    {
        return 0;
    }
    flag = fgetc (file);
    fclose (file);

    return (flag == '1')?1:0;
}

/* trim tail(right) space */
char *rtrim(char *str) 
{ 
    if (str == NULL || *str == '\0') 
    { 
        return str; 
    } 
    int len = strlen(str); 
    char *p = str + len - 1; 
    while (p >= str && (isspace(*p) || *p < 33 || *p > 122)) /* only print ascii 33~122 */
    { 
        *p = '\0'; --p; 
    } 
    return str; 
}

/* trim head(left) space */
char *ltrim(char *str) 
{ 
    if (str == NULL || *str == '\0') 
    { 
        return str; 
    } 
    int len = 0; 
    char *p = str;
    while (*p != '\0' && (isspace(*p) || *p < 33 || *p > 122)) /* only print ascii 33~122 */
    { 
        ++p; ++len; 
    } 
    memmove(str, p, strlen(str) - len + 1); 
    return str; 
}  

char *trim(char *str) 
{ 
    str = rtrim(str); 
    str = ltrim(str); 
    return str; 
} 

psu_type_t psu_type_get(int id, char* modelname, int modelname_len)
{
    char  model_name[PSU_MODEL_NAME_LEN + 1] = {0};

	/* Get PSU model name */
	char *string = NULL;
    int len = onlp_file_read_str(&string, "%s""psu%d_model", PSU_EEPROM_PREFIX, id);
    if (string && len) {
        aim_strlcpy(model_name, string, len+1);
    }

    if (string) {
        aim_free(string);
        string = NULL;
    }
	

	if(modelname)
	{
    	memcpy(modelname, model_name, sizeof(model_name));
	}

	/* Check AC model name */
	strcpy(model_name, trim(model_name));
	if(strcmp(model_name, "FSJ004EAAG") == 0)
	{
        return PSU_TYPE_AC_B2F;
	}
	if(strcmp(model_name, "FSJ001EAAG") == 0)
	{
        return PSU_TYPE_AC_F2B;
	}

	/* Check DC model name */
	if(strcmp(model_name, "FSJ036EAAG") == 0)
	{
        return PSU_TYPE_DC_48V_B2F;
	}
	if(strcmp(model_name, "FSJ035EAAG") == 0)
	{
        return PSU_TYPE_DC_48V_F2B;
	}

    return PSU_TYPE_UNKNOWN;
}

#define EEPROM_SIZE 256
int eeprom_tlv_read(uint8_t *rdata, int type, char *data)
{
    uint16_t i, j, TLV_length;
    int total_data_length = EEPROM_SIZE;
    int TLV_type;

    for (i = 0; i < total_data_length;)
    {
        if (i + 4 > total_data_length)
            break;

        TLV_type = ((*(rdata + i)) << 8) + (*(rdata + i + 1));
        TLV_length = ((*(rdata + i + 2)) << 8) + (*(rdata + i + 3));

        //printf("Type:%d TLV_type:%d Len:%d\n", type, TLV_type, TLV_length);

        if (TLV_length > EEPROM_SIZE - 1)
            return -1;
        
        if (TLV_type == type)
        {
            for (j = 0; j < TLV_length; j++)
            {
                data[j] = *(rdata + i + j + 4);
                //printf("TLV match\n");
            }
            data[j] = '\0';
            return 0;
        }
        i += (TLV_length + 4);
        //printf("i:%d\n",i);
    }
    //printf("TLV NOT match\n");
    return -1;
}
