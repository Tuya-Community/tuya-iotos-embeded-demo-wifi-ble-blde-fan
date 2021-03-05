# Tuya IoTOS Embeded Demo WiFi & BLE Smart BLDC Fan

[English](./README.md) | [中文](./README_zh.md) 

## Introduction 

This Demo implements a smart fan through Tuya Smart Cloud Platform, Tuya Smart APP, IoTOS Embeded WiFi &Ble SDK, using Tuya WiFi/WiFi+BLE series modules to achieve remote data monitoring.

Implemented features.

+ BLDC Motor speed increases with decreasing PWM duty cycle** 
+ Rotary switch (encoder) Press the switch, rotate clockwise to increase the wind speed by one gear, rotate counterclockwise to decrease the wind speed by one gear
+ app control, timer on/off, 3 modes (normal wind, natural wind: big and small intervals of 5 seconds, sleep wind: automatic downshifting every hour), power-off memory




## Quick start 

### Compile and burn
+ Download [Tuya IoTOS Embeded WiFi & BLE sdk](https://github.com/tuya/tuya-iotos-embeded-sdk-wifi-ble-bk7231t) 

+ Download the demo to the apps directory of the SDK 

  ```bash
  $ cd apps
  $ git clone https://github.com/Tuya-Community/tuya-iotos-embeded-demo-wifi-ble-blde-fan
  ```
  
+ Execute the following command in the SDK root directory to start compilation.

  ```bash
  sh build_app.sh apps/tuya-iotos-embeded-demo-wifi-ble-blde-fan tuya-iotos-embeded-demo-wifi-ble-blde-fan 1.0.0 
  ```

+ Firmware burn-in license information please refer to: [Wi-Fi + BLE series module burn-in license](https://developer.tuya.com/cn/docs/iot/device-development/burn-and-authorization/burn-and-authorize-wifi-ble-modules/burn-and-authorize-wb-series-modules?id=Ka78f4pttsytd) 

 

 ### File introduction 

```bash
├── include
│ ├── common
│ │ ├── light_printf.h
│ │ ├── light_tools.h
│ │ ├── light_types.h
│ ├── soc
│ │ ├─ soc_flash.h
│ │ ├── soc_gpio.h
│ │ ├── soc_i2c.h
│ │ ├── soc_pwm.h
│ │ ├── soc_timer.h
│ ├── tuya_app_bldc.h
│ ├── tuya_device.h
│ └── tuya_dp_process.h
└── src
    ├── common
    │ └── light_tools.c
    ├─ soc
    │ ├── soc_flash.c
    │ ├── soc_gpio.c
    │ ├── soc_i2c.c
    │ ├── soc_pwm.c
    │ └── soc_timer.c
    ├── tuya_app_bldc.c
    ├── tuya_device.c
    └── tuya_dp_process.c
```




 ### Demo information 

| Product ID | d250w0jkqj1xrux6 |
| :------: | :--------------------------------------------------------: |
| Chip Platform | BK7231T |
| Compiler Environment | ty_iot_wf_bt_sdk_bk_bk7231t_1.0.2 (Tuya IOT Platform SDK 1.0.2) |

  

### Demo entry

Entry file: tuya_device.c

Important functions: device_init()

+ Call tuya_iot_wf_soc_dev_init_param() interface to initialize the SDK, configure the operating mode, the mating mode, and register various callback functions and store the firmware key and PID.

+ Calling the tuya_iot_reg_get_wf_nw_stat_cb() interface to register the device network status callback functions.

+ Call the application layer initialization function air_box_device_init()

 

### DP point related

+ Report dp point interface: dev_report_dp_json_async()

| function name | OPERATE_RET dev_report_dp_json_async(IN CONST CHAR_T *dev_id,IN CONST TY_OBJ_DP_S *dp_data,IN CONST UINT_T cnt) |
| ------- | ------------------------------------------------------------ |
| devid | device id (if gateway, MCU, SOC class device then devid = NULL; if sub-device, then devid = sub-device_id) |
| dp_data | dp structure array name |
| cnt | Number of elements of the dp structure array |
| return | OPRT_OK: Success Other: Failure |

### I/O List 

| BLDC sandwich board pinout | corresponding connected WB3S pinout |
| :-------------------: | :----------------: |
| VCC | 3.3V |
| GND | GND |
| PWM1 | PWM2/GPIOA_8 |

| Encoder | WB3S |
| :----: | :---------: |
| N | TY_GPIOA_6 |
| A | TY_GPIOA_26 |
| B | TY_GPIOA_9 |

## Related Documents

Tuya Demo Center: https://developer.tuya.com/demo/



## Technical Support

You can get support for Tuya by using the following methods:

- Developer Center: https://developer.tuya.com
- Help Center: https://support.tuya.com/help
- Technical Support Work Order Center: [https://service.console.tuya.com](https://service.console.tuya.com/) 
