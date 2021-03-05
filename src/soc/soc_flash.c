/*
 * @Author: wls
 * @email: wuls@tuya.com
 * @LastEditors:   
 * @file name: soc_flash.c
 * @Description: soc flash proc
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Date: 2019-05-06 10:00:26
 * @LastEditTime: 2019-10-21 14:15:59
 */

#include "soc_flash.h"
#include "uf_file.h"
#include "light_types.h"
#include "light_printf.h"
#include "light_tools.h"
#include "soc_timer.h"


/**
 * @brief: wifi uf write(a+ mode)
 * @param {IN CHAR_T *pFilename -> file name}
 * @param {IN UCHAR_T *pData -> save data}
 * @param {IN USHORT_T usLen -> save data len}
 * @retval: OPERATE_LIGHT
 */
STATIC OPERATE_LIGHT opSocFlashFileWrite(IN CHAR_T *pFilename, IN UCHAR_T *pData, IN USHORT_T usLen)
{
    OPERATE_LIGHT opRet = -1;
    uFILE * fp = NULL;
    UINT_T uiOffSet = 0;
    UINT_T uiWriteCnt = 0;
    
    fp = ufopen(pFilename, "a+");
    if(NULL == fp) {
        PR_ERR("uf file %s can't open and write data!", pFilename);
        return LIGHT_COM_ERROR;
    }
    
    uiOffSet = ufseek(fp, 0, UF_SEEK_SET);
    if(uiOffSet != 0) {
        PR_ERR("uf file %s Set file offset to 0 error!", pFilename);
        return LIGHT_COM_ERROR;
    }

    uiWriteCnt = ufwrite(fp, pData, usLen);
    if(uiWriteCnt != usLen) {
        PR_ERR("uf file %s write data error!", pFilename);
        return LIGHT_COM_ERROR;
    }

    opRet = ufclose(fp);
    if(opRet != LIGHT_OK) {
        PR_ERR("uf file %s close error!", pFilename);
        return opRet;
    }

    return LIGHT_OK;
}

/**
 * @brief: wifi uf read
 * @param {IN CHAR_T *pFilename -> read file name}
 * @param {IN USHORT_T usLen -> read data len}
 * @param {OUT UCHAR_T *pData -> read data}
 * @retval: read data cnt
 */
STATIC INT_T uiSocFlashFileRead(IN CHAR_T *pFilename, IN USHORT_T usLen, OUT UCHAR_T *pData)
{
    OPERATE_LIGHT opRet = -1;
    uFILE * fp = NULL;
    INT_T uiReadCnt = 0;

    fp = ufopen(pFilename, "r+");
    if(NULL == fp) {
        PR_ERR("uf file %s can't open and read data!", pFilename);
        return LIGHT_COM_ERROR;
    }

    PR_DEBUG("uf open OK");
    uiReadCnt = ufread(fp, pData, usLen);
    PR_DEBUG("uf file %s read data %d!", pFilename, uiReadCnt);

    opRet = ufclose(fp);
    if(opRet != LIGHT_OK) {
        PR_ERR("uf file %s close error!", pFilename);
        return opRet;
    }
    
    return uiReadCnt;
}

/**
 * @brief: soc data save
 * @param {IN SOC_FLASH_SAVE_TYPE_E eDataType -> save type(meaning data kind)}
 * @param {IN UINT_T uiAddr -> this type data address offset}
 * @param {IN UCHAR_T *pData -> save data}
 * @param {IN USHORT_T usLen -> save data len}
 * @retval: OPERATE_LIGHT
 */
OPERATE_LIGHT opSocFlashWrite(IN SOC_FLASH_SAVE_TYPE_E eDataType, IN UINT_T uiAddr, IN UCHAR_T *pData, IN USHORT_T usLen)
{
    OPERATE_LIGHT opRet = -1;
    CHAR_T cTemp[4] = {0};

    if(eDataType >= SAVE_TYP_MAX) {
        PR_ERR("Write soc flash type error!");
        return LIGHT_INVALID_PARM;
    }

    vNum2Str(0, eDataType, 4, cTemp);
    opRet = opSocFlashFileWrite(cTemp, pData, usLen);
    if(opRet != LIGHT_OK) {
        return opRet;
    }
    
    return LIGHT_OK;
}

/**
 * @brief: soc flash save data read
 * @param {IN SOC_FLASH_SAVE_TYPE_E eDataType -> read data type(meaning data kind)}
 * @param {IN UINT_T uiAddr -> this type data address offset}
 * @param {IN USHORT_T ucLen -> read data len}
 * @param {OUT UCHAR_T *pData -> read data}
 * @retval: read data cnt
 */
INT_T uiSocFlashRead(IN SOC_FLASH_SAVE_TYPE_E eDataType, IN UINT_T uiAddr, IN USHORT_T usLen, OUT UCHAR_T *pData)
{
    OPERATE_LIGHT opRet = -1;
    INT_T uiReadCnt = 0;
    CHAR_T cTemp[4] = {0};

    if(eDataType >= SAVE_TYP_MAX) {
        PR_ERR("Read soc flash type error!");
        return LIGHT_INVALID_PARM;
    }
    
    vNum2Str(0, eDataType, 4, cTemp);
    PR_DEBUG("file name %s",cTemp);
    uiReadCnt = uiSocFlashFileRead(cTemp, usLen, pData);

    return uiReadCnt;
}

/**
 * @brief: soc flash oem cfg data read
 * @param {IN USHORT_T usLen -> write data len}
 * @param {IN UCHAR_T *pData -> write data}
 * @return: OPERATE_LIGHT
 * @retval: none
 */
OPERATE_LIGHT opSocOemCfgWrite(IN USHORT_T usLen, IN UCHAR_T *pData)
{
    OPERATE_LIGHT opRet = -1;
    uFILE * fp = NULL;
    UINT_T uiWriteCnt = 0;
    
    fp = ufopen("oem_cfg", "w+");
    if(NULL == fp) {
        PR_ERR("oem cfg uf file can't open and write data!");
        return LIGHT_COM_ERROR;
    }

    uiWriteCnt = ufwrite(fp, pData, usLen);
    if(uiWriteCnt != usLen) {
        PR_ERR("oem cfg uf file write data error!");
        return LIGHT_COM_ERROR;
    }

    opRet = ufclose(fp);
    if(opRet != LIGHT_OK) {
        PR_ERR("oem cfg uf file close error!");
        return opRet;
    }
    PR_NOTICE("save oem cfg into uf ok");
    return LIGHT_OK;
}

/**
 * @brief: soc flash oem cfg data read
 * @param {OUT USHORT_T *pLen -> read data len}
 * @param {OUT UCHAR_T *pData -> read data}
 * @retval: OPERATE_LIGHT
 */
OPERATE_LIGHT opSocOemCfgRead(OUT USHORT_T *pLen, OUT UCHAR_T *pData)
{
    OPERATE_LIGHT opRet = -1;
    uFILE * fp = NULL;
    UINT_T uiReadCnt = 0;
    

    fp = ufopen("oem_cfg", "r+");
    if(NULL == fp) {
        PR_ERR("uf file can't open and read data!");
        return LIGHT_COM_ERROR;
    }

    uiReadCnt = ufread(fp, pData, 1024);
    PR_DEBUG("oem cfg uf file read data %d!", uiReadCnt);
    PR_DEBUG("oem cfg uf file %s", pData);
    *pLen = uiReadCnt;
    
    opRet = ufclose(fp);
    if(opRet != LIGHT_OK) {
        PR_ERR("oem cfg uf file close error!");
        return opRet;
    }

    PR_DEBUG("oem read OK");
    return LIGHT_OK;
}

/**
 * @berief: soc flash gamma data read
 * @param {IN USHORT_T len -> write data len}
 * @param {IN UCHAR_T *data -> write data}
 * @return: OPERATE_RET
 * @retval: none
 */
OPERATE_RET opSocGammaCfgWrite(IN USHORT_T len, IN UCHAR_T *data)
{
    OPERATE_RET opRet = -1;
    uFILE * fp = NULL;
    UINT_T uiWriteCnt = 0;
    
    fp = ufopen("gamma_cfg", "w+");
    if(NULL == fp) {
        PR_ERR("gamma cfg uf file can't open and write data!");
        return OPRT_COM_ERROR;
    }

    uiWriteCnt = ufwrite(fp, data, len);
    if(uiWriteCnt != len) {
        PR_ERR("gamma cfg uf file write data error!");
        return OPRT_COM_ERROR;
    }

    opRet = ufclose(fp);
    if(opRet != OPRT_OK) {
        PR_ERR("gamma cfg uf file close error!");
        return opRet;
    }
    PR_NOTICE("save gamma cfg into uf ok len[%d]", len);
    return OPRT_OK;
}

/**
 * @berief: soc flash gamma cfg data read
 * @param {OUT USHORT_T *len -> read data len}
 * @param {OUT UCHAR_T *data -> read data}
 * @return: OPERATE_RET
 * @retval: none
 */
OPERATE_RET opSocGammaCfgRead(OUT USHORT_T *len, OUT UCHAR_T *data)
{
    OPERATE_RET opRet = -1;
    uFILE * fp = NULL;
    UINT_T uiReadCnt = 0;

    fp = ufopen("gamma_cfg", "r+");
    if(NULL == fp) {
        PR_ERR("uf file can't open and read data!");
        return OPRT_COM_ERROR;
    }

    uiReadCnt = ufread(fp, data, 4096);
    PR_DEBUG("gamma cfg uf file read data %d!", uiReadCnt);
    PR_DEBUG("gamma cfg uf file %s", data);
    *len = uiReadCnt;
    
    opRet = ufclose(fp);
    if(opRet != OPRT_OK) {
        PR_ERR("gamma cfg uf file close error!");
        return opRet;
    }

    PR_DEBUG("gamma read OK");
    return OPRT_OK;
}


/**
 * @berief: soc flash special block delete
 * @param {none}
 * @retval: OPERATE_LIGHT
 */
OPERATE_LIGHT opSocFlashEraseSpecial(IN SOC_FLASH_SAVE_TYPE_E DataType, IN UINT_T addr)
{
    OPERATE_LIGHT opRet = 0;
    CHAR_T cTemp[4] = {0};

    vNum2Str(0, DataType, 4, cTemp);
    
    if(!ufexist(cTemp)) {   /* file don't exist */
        return LIGHT_OK;
    }
    
    opRet = ufdelete(cTemp);
    if(opRet != LIGHT_OK) {
        PR_ERR("Delete %s file error!", cTemp);
        return opRet;
    }

    return LIGHT_OK;
}

/**
 * @brief: soc flash erase all
 * @param {none}
 * @retval: OPERATE_LIGHT
 */
OPERATE_LIGHT opSocFlashErase(VOID)
{
    OPERATE_LIGHT opRet = -1;
    UCHAR_T i = 0;
    UCHAR_T ucEarseCnt = 0;


    for(i = 0; i < SAVE_TYP_MAX; i++) {
        opRet = opSocFlashEraseSpecial(i, 0);
        if(LIGHT_OK == opRet) {
            ucEarseCnt++;
        }
    }
    
    if(ufexist("oem_cfg")) {
        opRet = ufdelete("oem_cfg");
        if(opRet != LIGHT_OK) {
            PR_ERR("Delete oem_cfg file error!");
            return LIGHT_COM_ERROR;
        }
    }

    if(ufexist("gamma_cfg")) {
        opRet = ufdelete("gamma_cfg");
        if(opRet != OPRT_OK) {
            PR_ERR("Delete gamma_cfg file error!");
            return OPRT_COM_ERROR;
        }
    }

    if(ucEarseCnt < SAVE_TYP_MAX) {
        return LIGHT_COM_ERROR;
    }
    
    return LIGHT_OK;
   
}


