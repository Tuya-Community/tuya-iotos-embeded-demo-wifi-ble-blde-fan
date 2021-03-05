/**
 * @File: tuya_app_flame_sensor_drive.c 
 * @Author: shiliu.yang@tuya.com 
 * @Last Modified time: 2021-01-20 
 * @Description: sandswich BLDC demo 
 */

#include "tuya_dp_process.h"
#include "tuya_app_bldc.h"
#include "tuya_key.h"
#include "tuya_iot_com_api.h"
#include "BkDriverGpio.h"
#include "uni_log.h"
#include "soc_pwm.h"
#include "soc_flash.h"
#include "crc_8.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uni_thread.h"

//风扇状态存储偏移量
#define FAN_STATE_OFFSET        0x00

//风扇状态存储长度：数据头—风扇开关—风扇模式—风扇转速—CRC_8校验
#define FAN_STATE_STORAGE_LEN   5   

//数据帧头
#define FAN_DATA_HEAD           0xFF

//电机状态在 flash 中的存放数据
#define FLASH_FAN_STATE_ON_OFF  1
#define FLASH_FAN_STATE_MODE    2
#define FLASH_FAN_STATE_SPEED   3

//pwm 设置相关
#define PWM_FREQUENCY   1000

//输出 PWM 占空比
#define PWM_FAN_OFF (5 * 10.0)
#define PWM_FAN_MIN (30 * 10.0)
#define PWM_FAN_MAX (99 * 10.0)

//自然风模式，定时器，标志
#define NATURAL_MODE_TIMER          1
#define NATURAL_SPEED_CHANGE_TIME   10 //单位：秒
static SHORT_T speed_low_flag = 0x00;

//睡眠风模式
#define SLEEP_MODE_TIMER        2
#define SLEEP_SPEED_CHANGE_TIME (60 * 60) //单位：秒

//风扇在按键上的档位
UINT_T g_fan_speed_gear[] = {1, 20, 40, 60, 80, 100};
CHAR_T g_current_gear = 0;

//按键相关
INT_T KEY_TIMER_MS;
KEY_USER_DEF_S key_user_def;

 /***********************************************************
 *   Function:  fan_pwm_init
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    芯片上电后初始话pwm，并将 pwm 输出关闭电机的占空比 
 ***********************************************************/
VOID fan_pwm_init(VOID)
{
    UCHAR_T pwm_gpio_list[] = {8};
    UCHAR_T pwm_num = (SIZEOF(pwm_gpio_list) / SIZEOF(pwm_gpio_list[0]));

    opSocPwmInit(PWM_FREQUENCY, (PWM_FAN_OFF), pwm_num, pwm_gpio_list, TRUE, FALSE);
}

 /***********************************************************
 *   Function:  rotary_key_task
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    旋钮动作后回调任务 
 ***********************************************************/
VOID rotary_key_ab_task(VOID)
{
    INT_T bk_ret, i;

    //去使能 旋钮开关外部中断
    bk_ret = BkGpioFinalize(ROTARY_A_PIN);
    if (kGeneralErr == bk_ret) {
        PR_ERR("BkGpioFinalize error!");
        return;
    }

    //如果关机，不执行任何操作
    if (fan_state.on_off == FALSE) {
        //使能旋钮开关外部中断
        BkGpioEnableIRQ(ROTARY_A_PIN, IRQ_TRIGGER_FALLING_EDGE, rotary_key_ab_task, NULL);
        return;
    }

    //判断当前档位
    for (i=0; i<(SIZEOF(g_fan_speed_gear)/SIZEOF(g_fan_speed_gear[0])); i++) {
        if (fan_state.speed <= g_fan_speed_gear[i]) {
            g_current_gear = i;
            break;
        }
    }
    
    if(tuya_gpio_read(ROTARY_A_PIN) != tuya_gpio_read(ROTARY_B_PIN)) {
        //PR_NOTICE("A != B +++ "); //顺时针方向
        if ((g_current_gear + 1) > ((SIZEOF(g_fan_speed_gear)/SIZEOF(g_fan_speed_gear[0]))-1)) {
            //PR_NOTICE("current gear is %d.", g_current_gear);
            //使能旋钮开关外部中断
            BkGpioEnableIRQ(ROTARY_A_PIN, IRQ_TRIGGER_FALLING_EDGE, rotary_key_ab_task, NULL);
            return;
        }

        fan_state.speed = g_fan_speed_gear[++g_current_gear];
    } else {
        //PR_NOTICE("A == B --- "); //逆时针方向
        if (g_current_gear - 1 < 0) {
            //PR_NOTICE("current gear is %d.", g_current_gear);
            //使能旋钮开关外部中断
            BkGpioEnableIRQ(ROTARY_A_PIN, IRQ_TRIGGER_FALLING_EDGE, rotary_key_ab_task, NULL);
            return;
        }

        fan_state.speed = g_fan_speed_gear[--g_current_gear];
    } 

    change_fan_state();

    //使能旋钮开关外部中断
    BkGpioEnableIRQ(ROTARY_A_PIN, IRQ_TRIGGER_FALLING_EDGE, rotary_key_ab_task, NULL);

    return;
}

 /***********************************************************
 *   Function:  rotary_key_n_task
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    旋钮开关 按键回调任务
 ***********************************************************/
VOID rotary_key_n_task(TY_GPIO_PORT_E port,PUSH_KEY_TYPE_E type,INT_T cnt)
{
    if (port == ROTARY_N_PIN) {
        switch (type) {
            case NORMAL_KEY:
                if (fan_state.on_off == FALSE) {
                    fan_state.on_off = TRUE;
                    PR_NOTICE("fan on");
                } else {
                    fan_state.on_off = FALSE;
                    PR_NOTICE("fan off");
                }
                break;
            case LONG_KEY:
                PR_NOTICE("ROTARY_N_PORT long press.");
                break;
            case SEQ_KEY:
                PR_NOTICE("ROTARY_N_PORT SEQ press, the count is %d.", cnt);
                break;
            default:
                break;
        }

        change_fan_state();
    }
}

 /***********************************************************
 *   Function:  rotary_key_init
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    旋钮开关初始化 
 ***********************************************************/
VOID rotary_key_init(VOID)
{
    OPERATE_RET opRet;

    //设置旋钮开关的引脚为输入模式
    tuya_gpio_inout_set(ROTARY_N_PIN, TRUE);
    tuya_gpio_inout_set(ROTARY_A_PIN, TRUE);
    tuya_gpio_inout_set(ROTARY_B_PIN, TRUE);

    /* 旋钮开关按下检测初始化 */
    opRet = key_init(NULL, 0, KEY_TIMER_MS);
    if (opRet != OPRT_OK) {
        PR_ERR("key_init err:%d", opRet);
        return opRet;
    }

    memset(&key_user_def, 0, SIZEOF(key_user_def));
    key_user_def.port = ROTARY_N_PIN;
    key_user_def.long_key_time = 3000;
    key_user_def.low_level_detect = TRUE;
    key_user_def.lp_tp = LP_ONCE_TRIG;
    key_user_def.call_back = rotary_key_n_task;
    key_user_def.seq_key_detect_time = 400;

    //注册按钮按键
    opRet = reg_proc_key(&key_user_def);
    if (opRet != OPRT_OK) {
        PR_ERR("reg_proc_key err:%d", opRet);
        return opRet;
    }

    /* 旋钮正反转检测初始化 */
    BkGpioEnableIRQ(ROTARY_A_PIN, IRQ_TRIGGER_FALLING_EDGE, rotary_key_ab_task, NULL);
}

 /***********************************************************
 *   Function:  get_flash_fan_state
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    从 flash 中读取电扇断电前状态，存到电扇状态结构体内 
 ***********************************************************/
VOID read_flash_fan_state(VOID)
{
    INT_T opRet, i;
    UCHAR_T fan_state_data_crc;
    UCHAR_T before_fan_power_off_state[FAN_STATE_STORAGE_LEN]; //断电前风扇状态

    opRet = uiSocFlashRead(SAVE_TYP1, FAN_STATE_OFFSET, FAN_STATE_STORAGE_LEN*SIZEOF(UCHAR_T), before_fan_power_off_state);
    if (opRet != FAN_STATE_STORAGE_LEN) {
        PR_ERR("read data error for flash");
        return;
    }

    //判断头部数据是否正确 
    if (before_fan_power_off_state[0] != FAN_DATA_HEAD) {
        PR_ERR("data head error");
        return;
    }

    fan_state_data_crc = get_crc_8(before_fan_power_off_state, (FAN_STATE_STORAGE_LEN - 1)*SIZEOF(UCHAR_T));
    //校验数据是否正确
    if (fan_state_data_crc != before_fan_power_off_state[FAN_STATE_STORAGE_LEN - 1]) { 
        PR_ERR("crc error, before_fan_power_off_state[%d] = %02x, crc data = %02x.", FAN_STATE_STORAGE_LEN - 1, before_fan_power_off_state[FAN_STATE_STORAGE_LEN - 1], fan_state_data_crc);
        return;
    }

    //将从 flash 读取到的数据，存放到结构体中
    fan_state.on_off    = before_fan_power_off_state[FLASH_FAN_STATE_ON_OFF];
    fan_state.mode      = before_fan_power_off_state[FLASH_FAN_STATE_MODE];
    fan_state.speed     = before_fan_power_off_state[FLASH_FAN_STATE_SPEED];

    return;
}

 /***********************************************************
 *   Function:  write_flash_fan_state
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    写电机状态到 flash 中  
 ***********************************************************/
VOID write_flash_fan_state(VOID) 
{
    INT_T opRet, i;
    UCHAR_T fan_state_buffer[FAN_STATE_STORAGE_LEN];

    fan_state_buffer[0] = FAN_DATA_HEAD;
    fan_state_buffer[1] = fan_state.on_off;
    fan_state_buffer[2] = fan_state.mode;
    fan_state_buffer[3] = fan_state.speed;
    fan_state_buffer[4] = get_crc_8(fan_state_buffer, (FAN_STATE_STORAGE_LEN - 1)*SIZEOF(UCHAR_T));
    
    // for (i=0; i<FAN_STATE_STORAGE_LEN; i++) {
    //     PR_NOTICE(" +++ fan_state_buffer is [%d] : %02x", i, fan_state_buffer[i]);
    // }

    opRet = opSocFlashWrite(SAVE_TYP1, FAN_STATE_OFFSET, fan_state_buffer, FAN_STATE_STORAGE_LEN * SIZEOF(UCHAR_T)); 
    if (opRet != LIGHT_OK) {
        PR_ERR("write flash error");
    }
    
    return;
}

 /***********************************************************
 *   Function:  erase_flash_fan_state
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    恢复出厂设置  
 ***********************************************************/
VOID erase_flash_fan_state(VOID) 
{
    INT_T opRet, i;
    UCHAR_T fan_state_buffer[FAN_STATE_STORAGE_LEN];

    fan_state.on_off = FALSE;
    fan_state.mode   = NORMAL_MODE;
    fan_state.speed  = 1;

    fan_state_buffer[0] = FAN_DATA_HEAD;
    fan_state_buffer[1] = fan_state.on_off;
    fan_state_buffer[2] = fan_state.mode;
    fan_state_buffer[3] = fan_state.speed;
    fan_state_buffer[4] = get_crc_8(fan_state_buffer, (FAN_STATE_STORAGE_LEN - 1)*SIZEOF(UCHAR_T));

    // for (i=0; i<FAN_STATE_STORAGE_LEN; i++) {
    //     PR_NOTICE(" +++ fan_state_buffer is [%d] : %02x", i, fan_state_buffer[i]);
    // }

    opRet = opSocFlashWrite(SAVE_TYP1, FAN_STATE_OFFSET, fan_state_buffer, FAN_STATE_STORAGE_LEN * SIZEOF(UCHAR_T)); 
    if (opRet != LIGHT_OK) {
        PR_ERR("write flash error");
    }

    return;
}

 /***********************************************************
 *   Function:  fan_speed_output_pwm
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    将设置的速度值输出为 PWM，对电机进行控制   
 ***********************************************************/
static VOID fan_speed_output_pwm(UINT_T speed)
{
    UINT_T  fan_speed_pwm_duty_cycle = 0;

    if (speed <= 0) {
        vSocPwmSetDuty(0, (PWM_FAN_OFF));
        return;
    }

    //由于电机在30%以下工作时间过长会出现异常，这里对 PWM 输出进行一些处理，使输出的 PWM 在 30%-99% 之间
    fan_speed_pwm_duty_cycle = (UINT_T)(PWM_FAN_MIN + ((PWM_FAN_MAX - PWM_FAN_MIN) * (speed / 100.0)));

    //输出的PWM占空比极性为高，电机需要的占空比极性为低。这里取一下反
    vSocPwmSetDuty(0, (fan_speed_pwm_duty_cycle));

    return;
}

 /***********************************************************
 *   Function:  fan_mode_normal
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    普通模式 
 ***********************************************************/
static VOID fan_mode_normal(VOID)
{
    INT_T opRet = LIGHT_OK;

    //关闭睡眠模式的定时器，防止干扰普通模式的运行
    opRet = opSocSWTimerStop(SLEEP_MODE_TIMER);
    if (opRet != LIGHT_OK) {
        PR_ERR("stop sleep timer error");
    }

    //关闭自然模式的定时器，防止干扰普通模式的运行
    opRet = opSocSWTimerStop(NATURAL_MODE_TIMER);
    if (opRet != LIGHT_OK) {
        PR_ERR("stop natural timer error");
    }

    fan_speed_output_pwm(fan_state.speed);
}

 /***********************************************************
 *   Function:  fan_mode_natural_task
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    自然风模式 回调函数
 ***********************************************************/
static VOID fan_mode_natural_task(VOID)
{
    //如果关机，不执行任何操作
    if (fan_state.on_off == FALSE) {
        opSocSWTimerStop(NATURAL_MODE_TIMER);
        return;
    }

    if (speed_low_flag) {
        fan_speed_output_pwm(1);
        PR_NOTICE("natural mode low speed");
    } else {
        PR_NOTICE("natural mode high speed");
        fan_speed_output_pwm(fan_state.speed);
    }
    speed_low_flag = ~(speed_low_flag);
    opSocSWTimerStart(NATURAL_MODE_TIMER, NATURAL_SPEED_CHANGE_TIME * 1000, fan_mode_natural_task);
}

 /***********************************************************
 *   Function:  fan_mode_natural
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    自然风模式 
 ***********************************************************/
static VOID fan_mode_natural(VOID)
{
    INT_T opRet = LIGHT_OK;

    //关闭睡眠模式的定时器，防止干扰自然模式的运行 
    opRet = opSocSWTimerStop(SLEEP_MODE_TIMER);
    if (opRet != LIGHT_OK) {
        PR_ERR("stop sleep timer error");
    }

    speed_low_flag = ~(0x00);
    fan_speed_output_pwm(fan_state.speed);

    opSocSWTimerStart(NATURAL_MODE_TIMER, NATURAL_SPEED_CHANGE_TIME * 1000, fan_mode_natural_task);
}

 /***********************************************************
 *   Function:  fan_sleep_mode_task
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    睡眠风模式 定时器回调任务
 ***********************************************************/
static VOID fan_sleep_mode_task(VOID)
{
    //判断当前是不是最低档。若为最低档，停止软件定时器，关闭风扇 
    if (fan_state.speed <= g_fan_speed_gear[0]) {
        opSocSWTimerStop(SLEEP_MODE_TIMER);
        fan_speed_output_pwm(0);
        fan_state.on_off = FALSE;
        change_fan_state();
        return;
    }

    PR_NOTICE("current gear is %d.", g_current_gear);
    fan_state.speed = g_fan_speed_gear[g_current_gear--];

    //改变档位转速
    fan_speed_output_pwm(fan_state.speed);
    //写入风扇状态到falsh中
    write_flash_fan_state();
    //上传dp点到涂鸦云平台
    updata_dp_all();    

    //启动睡眠模式，1h 减一档
    opSocSWTimerStart(SLEEP_MODE_TIMER, SLEEP_SPEED_CHANGE_TIME * 1000, fan_sleep_mode_task);
}

 /***********************************************************
 *   Function:  fan_mode_sleep
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    睡眠风模式 
 ***********************************************************/
static VOID fan_mode_sleep(VOID)
{
    INT_T opRet = LIGHT_OK;
    SHORT_T i;

    //关闭自然模式的定时器，防止干扰睡眠模式模式的运行 
    opRet = opSocSWTimerStop(NATURAL_MODE_TIMER);
    if (opRet != LIGHT_OK) {
        PR_ERR("stop sleep timer error");
    }

    //判断当前档位
    for (i=0; i<(SIZEOF(g_fan_speed_gear)/SIZEOF(g_fan_speed_gear[0])); i++) {
        if (fan_state.speed <= g_fan_speed_gear[i]) {
            g_current_gear = i;
            break;
        }
    }

    PR_NOTICE("current gear is %d.", g_current_gear);
    fan_state.speed = g_fan_speed_gear[g_current_gear--];
    //改变档位转速
    fan_speed_output_pwm(fan_state.speed);
    //写入风扇状态到falsh中
    write_flash_fan_state();
    //上传dp点到涂鸦云平台
    updata_dp_all();   
    

    opSocSWTimerStart(SLEEP_MODE_TIMER, SLEEP_SPEED_CHANGE_TIME * 1000, fan_sleep_mode_task);
}

 /***********************************************************
 *   Function:  change_fan_state
 *   Input:     none
 *   Output:    none
 *   Return:    none
 *   Notice:    改变电机状态 
 ***********************************************************/
VOID change_fan_state(VOID)
{
    if (fan_state.on_off == FALSE) {
        fan_speed_output_pwm(0);

        opSocSWTimerStop(SLEEP_MODE_TIMER);
        opSocSWTimerStop(NATURAL_MODE_TIMER);

        write_flash_fan_state();
        updata_dp_all();
        return;
    }

    if (fan_state.mode == SLEEP_MODE) {
        fan_mode_sleep();
    } else if (fan_state.mode == NATURAL_MODE) {
        fan_mode_natural();
    } else {
        fan_mode_normal();
    }

//    write_flash_fan_state();
//    updata_dp_all();
    
    return;
}

