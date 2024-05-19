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
/********************����У׼��־λ******************************/
#define Calibration_open 1
#define Calibration_close 0


//У׼������ť--�������ر������������֤������ֹ����֤��ȫ
//#define Calibration_switch Calibration_close

//У׼��̨--�����̨��е�Ƕ�ֵ
#define CloudParam_Calibration 1

#define YAW_Mechanics_Angle 2720
#define PITCH_Mechanics_Angle 5800

//������У׼--��������Ǹ�������--���������������
#define IMU_Cclibration 2

#define	IMU  9//6//9//18   //����      		//ƫ��ֵ
#define	IMU_ 2//60.4//56//57.5// 57			//��Ư

//У׼ģʽѡ��
#define Calibration_Mode Calibration_close

/**************************************************************/

//////////////////-------------------------�����ķָ���-------------------------//////////////////

/********************�������������־λ******************************/
//Ħ�����ٶ�
#define highfeedmotorspeed 2
#define lowfeedmotorspeed  1
#define rcfeedmotorspeed   3
#define feedmotor_off      0
//���ٵ���
#define shoot_speed 80   //��ֵ��Χ��0-80

//��Ƶ����
#define shoot_frequency 

//���̿��Ƶ����ٶȵ���
#define keyboard_chassis_speed 

//��������ȵ���
#define mouse_sensitivity_speed

//ң�����ٶȱȵ���
#define romote_rate_speed 5

/**************************************************************/
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ main_H */


