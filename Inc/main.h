#ifndef __main_H
#define __main_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
//#include "stm32f4xx_hal.h"
//#include "can.h"
//#include "dma.h"
//#include "iwdg.h"
#include "spi.h"
//#include "tim.h"
//#include "usart.h"
//#include "wwdg.h"
//#include "gpio.h"	 
	 
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "wwdg.h"
#include "adc.h"
#include "delay.h"
#include "sys.h"
#include "gpio.h"
#include "test_imu.h"
#include "bkd_usart.h"
#include "bkd_can.h"
#include "bkd_pid.h"
#include "bkd_control.h"
#include "bkd_remote.h"
#include "sys.h"
#include "delay.h"
#include "RM_Cilent_UI.h"
#include "ui.h"
#include "string.h"
#include "bkd_Small_gyroscope.h"
/* USER CODE zEND Includes */



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */



/* USER CODE BEGIN Prototypes */
/********************调试校准标志位******************************/
#define Calibration_open 1
#define Calibration_close 0


//校准开启按钮--开启即关闭所有输出，保证车辆静止，保证安全
//#define Calibration_switch Calibration_close

//校准云台--输出云台机械角度值
#define CloudParam_Calibration 1

#define YAW_Mechanics_Angle 2720
#define PITCH_Mechanics_Angle 5800

//陀螺仪校准--输出陀螺仪各项数据--误差消除参数调节
#define IMU_Cclibration 2

#define	IMU  9//6//9//18   //反向      		//偏差值
#define	IMU_ 2//60.4//56//57.5// 57			//零漂

//校准模式选择
#define Calibration_Mode Calibration_close

/**************************************************************/

//////////////////-------------------------华丽的分割线-------------------------//////////////////

/********************各项参数调整标志位******************************/
//摩擦轮速度
#define highfeedmotorspeed 2
#define lowfeedmotorspeed  1
#define rcfeedmotorspeed   3
#define feedmotor_off      0
//射速调节
#define shoot_speed 80   //调值范围：0-80

//射频调节
#define shoot_frequency 

//键盘控制底盘速度调节
#define keyboard_chassis_speed 

//鼠标灵敏度调节
#define mouse_sensitivity_speed

//遥控器速度比调节
#define romote_rate_speed 5

/**************************************************************/
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ main_H */


