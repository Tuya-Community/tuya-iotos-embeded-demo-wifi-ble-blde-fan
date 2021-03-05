/**
 * @File: tuya_dp_process.h 
 * @Author: shiliu.yang@tuya.com 
 * @Last Modified time: 2021-01-20 
 * @Description: bldc fan demo 
 */
#ifndef __TUYA_DP_PROCESS_H_
#define __TUYA_DP_PROCESS_H_

#include "tuya_cloud_com_defs.h"
#include "tuya_cloud_types.h"

//dp 点枚举类型
typedef UCHAR_T         BLDC_FAN_DPID_T;
#define DPID_SWITCH     1
#define DPID_MODE       2
#define DPID_FAN_SPEED  3

// 工作模式 枚举
typedef UCHAR_T         FAN_WORK_MODE_T; 
#define NORMAL_MODE     0 
#define NATURAL_MODE    1 
#define SLEEP_MODE      2


//设备状态结构体
typedef struct {
    /* BLDC 风扇 dp点  */
    BLDC_FAN_DPID_T dp_id_switch;
    BLDC_FAN_DPID_T dp_id_mode;
    BLDC_FAN_DPID_T dp_id_fan_speed;

    /* 风扇状态定义 */
    BOOL_T          on_off;
    FAN_WORK_MODE_T mode;
    UINT_T          speed;

}FAN_STATE_T;


extern FAN_STATE_T  fan_state;

VOID updata_dp_all(VOID);
VOID dp_process(IN CONST TY_OBJ_DP_S *root);


#endif

