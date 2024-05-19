#include "main.h"
#include "bkd_Autoaiming.h"
#include "bkd_remote.h"
#include "math.h"
#include "bkd_feedmotor.h"
#include "bkd_Small_gyroscope.h"
double yaw_degree;
int Cloud_Init_Flag=0;
double zzz;
extern int aiming;
extern uint8_t Auto_Aim_Flag_P,Auto_Aim_Flag_Y;
extern float ax,ay,az,gx,gy,gz;
extern double pitchposition;
extern double yawposition;
extern double yawposition1;
extern double pitchposition1;
extern int32_t bullet_motor_flag;
int Aim_Mechanics_Angle;
extern float Auto_Aim_Vx,Auto_Aim_Vy;
extern int16_t speed_shoot;
extern int16_t Shoot_out;
int buchang=60;
int buchangxxx=-26;
int xiuzhengzhi=0;
void Autoaiming(void)
{
		if((dx<0||dx>640)||(aiming==0))
		{
			if(Auto_Aim_Flag_Y==0)
			{
				vz=0;
				Gimbal.angle_yaw2=0;
				Auto_Aim_Flag_Y=1;
			}
			Gimbal.angle_yaw2+=((double)(gz)/32.8f)*0.002f;
			yawposition1=Gimbal.angle_yaw2;
			PID_Calc_Position(&Yaw_position_pid,yawposition1*1.8,-vz);//vz
			PID_Calc_Position(&Yaw_speed_pid,(double)gz/32.8f*1.8,Yaw_position_pid.output);
			if(Auto_Aim_Flag_P==0)
			{
				w0=1.8*pitchposition;
				Auto_Aim_Flag_P=1;
			}
			if(Pitch_position_Feedback.real[0]<(4081))
			pitchposition=Pitch_position_Feedback.real[0]-PITCH_Mechanics_Angle+8192;
			else
			pitchposition=Pitch_position_Feedback.real[0]-PITCH_Mechanics_Angle;
			PID_Calc_Position(&Pitch_position_pid, pitchposition*1.8,-w0);
			PID_Calc_Position(&Pitch_speed_pid,(double)gx/32.8f*1.8,Pitch_position_pid.output);
			CAN_Send_Message(&hcan1, 0X1ff,Yaw_speed_pid.output,Pitch_speed_pid.output,Shoot_out,0);			
		}
		else
		{
//			HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
			if(Pitch_position_Feedback.real[0]<(4081))
			pitchposition=Pitch_position_Feedback.real[0]-PITCH_Mechanics_Angle+8192;
			else
			pitchposition=Pitch_position_Feedback.real[0]-PITCH_Mechanics_Angle;
			w0=1.8*pitchposition;
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);
			Auto_Aim_Flag_P=0;
			Auto_Aim_Flag_Y=0;
			Auto_Aim_Vx=dx-320-buchangxxx;
			Auto_Aim_Vy=dy-240-buchang;
			PID_Calc_Position(&Yaw_Auto_Aim_pid,Auto_Aim_Vx,0);
			PID_Calc_Position(&Yaw_speed_pid,(double)gz/32.8f*1.8,Yaw_Auto_Aim_pid.output);	
			PID_Calc_Position(&Pitch_Auto_Aim_pid,-Auto_Aim_Vy,0);
			PID_Calc_Position(&Pitch_speed_pid,(double)gx/32.8f*1.8,Pitch_Auto_Aim_pid.output);
			CAN_Send_Message(&hcan1, 0X1ff,Yaw_speed_pid.output,Pitch_speed_pid.output,Shoot_out,0);		
		}
}

void Cloud(void)
{
			Auto_Aim_Flag_P=0;
			Auto_Aim_Flag_Y=0;

		pitchposition=Pitch_position_Feedback.real[0]-PITCH_Mechanics_Angle;
	if(Yaw_position_Feedback.real[0]>(YAW_Mechanics_Angle+4081))
		yaw_degree=Yaw_position_Feedback.real[0]-YAW_Mechanics_Angle-8192;
	else
		yaw_degree=Yaw_position_Feedback.real[0]-YAW_Mechanics_Angle;
		yawposition=Gimbal.angle_yaw1;
	
	if(Cloud_Init_Flag==0)
	{
	PID_Calc_Position(&Yaw_position_pid,yaw_degree*0.01,0);//vz
	PID_Calc_Position(&Yaw_speed_pid,(double)gz/32.8f*1.8,Yaw_position_pid.output);	//Yaw_position_pid.output
	PID_Calc_Position(&Pitch_position_pid,pitchposition,0);
	PID_Calc_Position(&Pitch_speed_pid,(double)gx/32.8f*1.8,Pitch_position_pid.output);//Pitch_position_pid.output
	}
	if(((Yaw_position_Feedback.real[0]-YAW_Mechanics_Angle)<40)&&((Yaw_position_Feedback.real[0]-YAW_Mechanics_Angle)>-40))	
	{		
		Cloud_Init_Flag=1;
	}
	if(Cloud_Init_Flag==1)
	{
		PID_Calc_Position(&Yaw_position_pid,yawposition*1.8,-vz);//vz
		PID_Calc_Position(&Yaw_speed_pid,(double)gz/32.8f*1.8,Yaw_position_pid.output);	//Yaw_position_pid.output

		PID_Calc_Position(&Pitch_position_pid, pitchposition*1.8,-w0);
		PID_Calc_Position(&Pitch_speed_pid,(double)gx/32.8f*1.8,Pitch_position_pid.output);//Pitch_position_pid.output
	}
	CAN_Send_Message(&hcan1, 0X1ff,Yaw_speed_pid.output,Pitch_speed_pid.output,Shoot_out,0);
}
