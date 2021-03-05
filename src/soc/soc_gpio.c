/*
 * @Author: wls
 * @email: wuls@tuya.com
 * @LastEditors: wls
 * @file name:  soc_gpio.c
 * @Description: BK7231 general gpio proc
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Date: 2019-04-16 16:46:43
 * @LastEditTime: 2019-04-25 17:21:28
 */

#include "soc_gpio.h"
#include "tuya_gpio.h"
#include "light_printf.h"


VOID vSocPinInit(UCHAR_T pin, IN CONST BOOL_T in, IN CONST BOOL_T high)
{
    OPERATE_RET opRet = -1;
    opRet = tuya_gpio_inout_set_select(pin, in, high);
    if(opRet != OPRT_OK) {
        PR_ERR("gpio %d init error!");
    }
}


/**
 * @brief: SOC general pin Reset proc
 * @param {none}
 * @retval: none
 */
VOID vSocPinReset(UCHAR_T pin)
{
    OPERATE_LIGHT opRet = -1;
	
    opRet = tuya_gpio_inout_set(pin,FALSE);
	if(opRet != LIGHT_OK) {
        PR_ERR("gpio %d set output error!");
        return ;
    }
	
    opRet = tuya_gpio_write(pin,FALSE);
    if(opRet != LIGHT_OK) {
        PR_ERR("gpio %d output low error!");
        return ;
    }
}

/**
 * @brief: SOC i2c ctrl set proc
 * @param {none}
 * @retval: none
 */
VOID vSocPinSet(UCHAR_T pin)
{
    OPERATE_LIGHT opRet = -1;
    
    opRet = tuya_gpio_inout_set(pin,FALSE);
    if(opRet != LIGHT_OK) {
        PR_ERR("gpio %d set output error!");
        return ;
    }

    opRet = tuya_gpio_write(pin,TRUE);
    if(opRet != LIGHT_OK) {
        PR_ERR("gpio %d output high error!");
        return ;
    }
}


VOID vSocPinOutputSet(UCHAR_T ucPin, BOOL_T bHigh)
{
    OPERATE_RET opRet = -1;
    opRet = tuya_gpio_write(ucPin,bHigh);
    if(opRet != OPRT_OK) {
        PR_ERR("gpio %d output high error!");
        return ;
    }
}

