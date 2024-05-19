#ifndef __bkd_pid_H
#define __bkd_pid_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */



/* USER CODE BEGIN Private defines */
#define Incremental 0
#define Positional  1
typedef struct
{
    //PID 三参数
    double Kp;
    double Ki;
    double Kd;
	double Ka;
	//最大输出 死区
	double max_out;  //最大输出
	double dead_band;//PID偏差死区
	double intergral_band;//积分区
	double max_input;//最大输入
    //PID输出值
    double output;
	double output_compensation;
    //误差
	double e_max;
    double e[3];//2最新 1上一次 0上上次
	double d_last;
	double intergral;
	double sum_e;
	double d_band;	
	double integral_limit;
	double p_limit;	
		
} PidTypeDef;
void PID_Init(PidTypeDef * pid,double kp,double ki,double kd,double ka,double max_out,double dead_band,double i_deadband,double max_input);
void PID_Calc(PidTypeDef * pid, double rel_val, double set_val,uint8_t  PID_mode);
void PID2_Calc(PidTypeDef * pid, double rel_val, double set_val,uint8_t  PID_mode);
void PID_Calc_Speed(PidTypeDef * pid, double ret_val, double set_val);//单环式皮带
void PID_Calc_Position(PidTypeDef * pid, double ret_val, double set_val);//单环位置式皮带
void PID_Calc1(PidTypeDef * pid, double rel_val, double set_val,uint8_t PID_mode);
void abs_limit(float a, float ABS_MAX);

double pid_myself(double fb,double input);
void PID_SET_Init(void);
float MyAbs(float num);
extern PidTypeDef CM1_speed_pid;
extern PidTypeDef CM2_speed_pid;
extern PidTypeDef CM3_speed_pid;
extern PidTypeDef CM4_speed_pid;
extern PidTypeDef CM5_speed_pid;
extern PidTypeDef CM6_speed_pid;

extern PidTypeDef CM1_position_pid;
extern PidTypeDef CM2_position_pid;
extern PidTypeDef CM3_position_pid;
extern PidTypeDef CM4_position_pid;

extern PidTypeDef Gyro_normal_pid;
extern PidTypeDef Yaw_speed_pid;
extern PidTypeDef Yaw_position_pid;
extern PidTypeDef Yaw_Auto_Aim_pid;

extern PidTypeDef Pitch_speed_pid;
extern PidTypeDef Pitch_position_pid;
extern PidTypeDef Pitch_Auto_Aim_pid;

extern PidTypeDef Shoot_speed_pid;
extern PidTypeDef Shoot_position_pid;

extern PidTypeDef Yaw_backspeed_pid;
extern PidTypeDef Yaw_backsposition_pid;
/* USER CODE END Private defines */



/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

