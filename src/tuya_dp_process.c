/**
 * @File: tuya__dp_process.c 
 * @Author: shiliu.yang@tuya.com 
 * @Last Modified time: 2021-01-20 
 * @Description: sandswich BLDC demo 
 */

#include "tuya_dp_process.h"
#include "tuya_iot_com_api.h"
#include "uni_log.h"
#include "tuya_app_bldc.h"

FAN_STATE_T  fan_state = {
    .dp_id_switch = DPID_SWITCH,
    .dp_id_mode   = DPID_MODE,
    .dp_id_fan_speed = DPID_FAN_SPEED,

    .on_off = FALSE,
    .mode   = NORMAL_MODE,
    .speed  = 1,
};

 /***********************************************************
 *   Function:  dp_process
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    处理 dp 数据 
 ***********************************************************/
VOID dp_process(IN CONST TY_OBJ_DP_S *root)
{
    UCHAR_T dpid;

    dpid = root->dpid;
    PR_DEBUG("dpid:%d",dpid);

    switch(dpid) {
        case DPID_SWITCH:
            fan_state.on_off = root->value.dp_bool;
        break;
            
        case DPID_MODE:
            fan_state.mode = root->value.dp_enum;            
        break;
        
        case DPID_FAN_SPEED:
            fan_state.speed = root->value.dp_value;
        break;

        default:
        break;
    }

    change_fan_state();
    return;
}

 /***********************************************************
 *   Function:  updata_dp_all
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    上报所有 dp 
 ***********************************************************/
VOID updata_dp_all(VOID)
{
    OPERATE_RET op_ret = OPRT_OK;

    INT_T dp_cnt = 0;
    dp_cnt = 3;

    TY_OBJ_DP_S *dp_arr = (TY_OBJ_DP_S *)Malloc(dp_cnt*SIZEOF(TY_OBJ_DP_S));
    if(NULL == dp_arr) {
        PR_ERR("malloc failed");
        return;
    }

    memset(dp_arr, 0, dp_cnt*SIZEOF(TY_OBJ_DP_S));

    dp_arr[0].dpid = fan_state.dp_id_switch;
    dp_arr[0].type = PROP_BOOL;
    dp_arr[0].time_stamp = 0;
    dp_arr[0].value.dp_bool = fan_state.on_off;

    dp_arr[1].dpid = fan_state.dp_id_mode;
    dp_arr[1].type = PROP_ENUM;
    dp_arr[1].time_stamp = 0;
    dp_arr[1].value.dp_enum = fan_state.mode;

    dp_arr[2].dpid = fan_state.dp_id_fan_speed;
    dp_arr[2].type = PROP_VALUE;
    dp_arr[2].time_stamp = 0;
    dp_arr[2].value.dp_value = fan_state.speed;

    op_ret = dev_report_dp_json_async(NULL ,dp_arr,dp_cnt);
    Free(dp_arr);
    if(OPRT_OK != op_ret) {
        PR_ERR("dev_report_dp_json_async relay_config data error,err_num",op_ret);
    }

    PR_DEBUG("dp_query report_all_dp_data");
    
    return;
}



