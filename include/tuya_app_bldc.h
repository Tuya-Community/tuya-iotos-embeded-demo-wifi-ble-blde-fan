/**
 * @File: tuya_app_bldc.h 
 * @Author: shiliu.yang@tuya.com 
 * @Last Modified time: 2021-01-05 
 * @Description: flame detection demo 
 */
#ifndef __TUYA_APP_BLDC_H__
#define __TUYA_APP_BLDC_H__

#include "tuya_cloud_types.h"
#include "tuya_gpio.h"

#define ROTARY_N_PIN    TY_GPIOA_6
#define ROTARY_A_PIN    TY_GPIOA_26
#define ROTARY_B_PIN    TY_GPIOA_9

VOID read_flash_fan_state(VOID);
VOID write_flash_fan_state(VOID);
VOID erase_flash_fan_state(VOID);

VOID change_fan_state(VOID);

VOID fan_pwm_init(VOID);
VOID rotary_key_init(VOID);

#endif /* end tuya_app_bldc.h */

