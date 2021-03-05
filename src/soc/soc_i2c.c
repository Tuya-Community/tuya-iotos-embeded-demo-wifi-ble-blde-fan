/*
 * @Author: wls
 * @email: wuls@tuya.com
 * @LastEditors: wls
 * @file name:  soc_i2c.c
 * @Description: Realtek 8710bn IIC gpio proc
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Date: 2019-04-16 16:46:43
 * @LastEditTime: 2019-04-25 17:21:28
 */

#include "soc_i2c.h"
#include "tuya_gpio.h"
#include "light_printf.h"


STATIC UCHAR_T gucI2cInitFlag = FALSE;
STATIC I2C_PIN_T gtI2cGpio;
STATIC UCHAR_T ucSDAStatus = FALSE;

/**
 * @brief: SOC i2c init
 * @param {I2C_PIN_T *pI2CPinCfg --> i2c init gpio struct}
 * @retval: none
 */
OPERATE_LIGHT opSocI2CInit(IN I2C_PIN_T *pI2CPinCfg)
{
    OPERATE_LIGHT opRet = 0;

    if(gucI2cInitFlag != FALSE) {
        PR_NOTICE("SOC I2C already init!");
        return LIGHT_OK;
    }
    
    opRet = tuya_gpio_inout_set(pI2CPinCfg ->ucSdaPin, TRUE);      /* input */
    if(opRet != LIGHT_OK){
        PR_ERR("soc i2c sda gpio init error!");
        return LIGHT_INVALID_PARM;
    }
    
    opRet = tuya_gpio_inout_set(pI2CPinCfg ->ucSclPin, TRUE);      /* input */
    if(opRet != LIGHT_OK){
        PR_ERR("soc i2c scl gpio init error!");
        return LIGHT_INVALID_PARM;
    }
     
    memset(&gtI2cGpio, 0, SIZEOF(I2C_PIN_T));
    memcpy(&gtI2cGpio, pI2CPinCfg, SIZEOF(I2C_PIN_T));

    gucI2cInitFlag = TRUE;
    
    return LIGHT_OK;
}

/**
 * @brief: SOC i2c SDA reset proc
 * @param {none}
 * @retval: none
 */
VOID vSocI2CSdaReset(VOID)
{
    OPERATE_LIGHT opRet = 0;
    
    if(gucI2cInitFlag != TRUE) {
        PR_ERR("I2C is not't init!");
        return ;
    }

    opRet = tuya_gpio_inout_set(gtI2cGpio.ucSdaPin, FALSE);      /* output */
    if(opRet != LIGHT_OK) {
        PR_ERR("soc i2c sda gpio set output error!");
        return ;
    }
    
    opRet = tuya_gpio_write(gtI2cGpio.ucSdaPin, FALSE);
    if(opRet != LIGHT_OK) {
        PR_ERR("I2C SDA gpio set error!");
        return ;
    }
    ucSDAStatus = FALSE;
}

/**
 * @brief: SOC i2c SDA set proc
 * @param {none}
 * @retval: none
 */
VOID vSocI2CSdaSet(VOID)
{
    OPERATE_LIGHT opRet = 0;
    
    if(gucI2cInitFlag != TRUE) {
        PR_ERR("I2C is not't init!");
        return ;
    }

    opRet = tuya_gpio_inout_set(gtI2cGpio.ucSdaPin, TRUE);      /* input */
    if(opRet != LIGHT_OK) {
        PR_ERR("soc i2c sda gpio set input error!");
        return ;
    }
    ucSDAStatus = TRUE;
}


/**
 * @brief: SOC i2c SCL reset proc
 * @param {none}
 * @retval: none
 */
VOID vSocI2CSclReset(VOID)
{
    OPERATE_LIGHT opRet = 0;

    if(gucI2cInitFlag != TRUE){
        PR_ERR("I2C is not't init!");
        return ;
    }

    opRet = tuya_gpio_inout_set(gtI2cGpio.ucSclPin, FALSE);      /* output */
    if(opRet != LIGHT_OK){
        PR_ERR("soc i2c scl gpio set output error!");
        return ;
    }
    
    opRet = tuya_gpio_write(gtI2cGpio.ucSclPin, FALSE);
    if(opRet != LIGHT_OK) {
        PR_ERR("I2C scl gpio set error!");
        return ;
    }
}

/**
 * @brief: SOC i2c SCL set proc
 * @param {none}
 * @retval: none
 */
VOID vSocI2CSclSet(VOID)
{
    OPERATE_LIGHT opRet = 0;

    if(gucI2cInitFlag != TRUE){
        PR_ERR("I2C is not't init!");
        return ;
    }

    opRet = tuya_gpio_inout_set(gtI2cGpio.ucSclPin, TRUE);      /* input */
    if(opRet != LIGHT_OK){
        PR_ERR("soc i2c scl gpio set input error!");
        return ;
    }
    
}

/**
 * @brief: SOC i2c SDA(output) read
 * @param {none}
 * @retval: none
 */
UCHAR_T ucSocI2CSdaOutputRead(VOID)
{
    return ucSDAStatus;
}

/**
 * @brief: SOC i2c SDA(input) read
 * @param {none}
 * @retval: none
 */
UCHAR_T ucSocI2CSdaInputRead(VOID)
{

    return tuya_gpio_read(gtI2cGpio.ucSdaPin);
    //return ucSDAStatus;
}

/**
 * @brief: SOC i2c delay
 * @param {none}
 * @attention: soc i2c software delay, this maybe need adjust
 * @retval: none
 */
VOID vSocI2CDelay(USHORT_T usTime)
{
    /* usTime = usTime * gain, gain need set by test! */
    while(usTime --);
}

