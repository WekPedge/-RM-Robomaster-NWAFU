#ifndef __bkd_remote_H
#define __bkd_remote_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */



/* USER CODE BEGIN Private defines */
#define CLOSE 0
#define OPEN  1
#define RC 0
#define PC 1
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\


typedef struct
{
	uint8_t chassis;//底盘数 QWEASD
	uint8_t mode;//模式数 F V 鼠标右键 自动瞄准 大符 防御
	uint8_t reset;//复位数 Z X C 激光 摩擦轮 看门狗复位
	uint8_t stall;//档位数 shift ctrl 加速 减速
}Keyboard_t;
void RC_change(void);
void PC_yidong(void);
void motor_start(void);
void motor_close(void);
void keyboard_data_set(void);
void Remote_Task(void);
void RC_control(void);
void PC_control(void);
void Parking(void);
void mouse_PC_control(void);
void mode_PC_control(void);
extern Keyboard_t Keyboard;
extern double vx;
extern double vy;
extern double w0;
extern double vz;
extern uint8_t control_mode;
extern uint8_t friction_flag;
extern double K_Speed;
extern uint8_t shooting_flag;
extern int16_t shooting_position;
/* USER CODE END Private defines */



/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

