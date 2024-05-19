#include "main.h"
#include "bkd_remote.h"
#include "math.h"
#include "bkd_feedmotor.h"
#include "bkd_Small_gyroscope.h"
#include "bkd_Autoaiming.h"
#include "string.h"
#include   <stdlib.h>  
#include "Heat.h"
static uint32_t time_tick_1ms = 0;
uint32_t ui_10hzflag = 0;//��ʱ����ui�ã�
static uint32_t time_tick = 0;//���ڼ�delay��
int shijian=1;
int YAW;
float cur;
int siqu=1;
float tem_p;
uint16_t adcx;
int yawdegree_now;
double yawposition;
extern int shootout;
extern int shootout;
extern uint8_t key_w;
extern uint8_t key_s;
extern uint8_t key_a;
extern uint8_t key_d;
extern uint8_t key_shift;
extern uint8_t key_ctrl;
extern uint8_t key_q;
extern uint8_t key_e;
extern uint8_t key_r;
extern uint8_t key_f;
extern uint8_t key_g;
extern uint8_t key_z;
extern uint8_t key_x;
extern uint8_t key_c;
extern uint8_t key_v;
extern uint8_t key_b;
extern int Cloud_Init_Flag;

int aiming=0;
double pitchposition;
double yawposition1=0;
double pitchposition1=0;
extern int Aim_Mechanics_Angle;
extern u8 IMU_count_flag;
extern int16_t speed_shoot;
extern u8 Auto_Aim_flag;
extern int16_t Shoot_out;
extern float ax,ay,az,gx,gy,gz;
extern int32_t bullet_motor_flag;
extern PidTypeDef CM1_current_pid;
extern PidTypeDef CM2_current_pid;
extern PidTypeDef CM3_current_pid;
extern PidTypeDef CM4_current_pid;
float Auto_Aim_Vx=0,Auto_Aim_Vy=0;
extern int8_t Small_gyroscope_flag;
uint8_t Auto_Aim_Flag_P=1,Auto_Aim_Flag_Y=1;


extern uint8_t power_limit;
extern float power;
extern int rank;
float speed_t=0;
float speed_t_start=0;
float speed_range=0;
int speed_t_flag=0;//С���ݼ�ʱ��־

void Control_Task(void)
{
	
	time_tick_1ms++;
  speed_t=0.1*time_tick_1ms/100.0;
	if((time_tick_1ms-time_tick)>=shijian) aiming=0;
	
	if(Calibration_Mode==CloudParam_Calibration)
	{
		Parking();
	}
	else if(Calibration_Mode==IMU_Cclibration)
	{
		Parking();
	}
	else if(Calibration_Mode==Calibration_close)
	{
		if(time_tick_1ms <3000);
		else if(time_tick_1ms <5000);
		else if(time_tick_1ms >5000)
		{
			if(time_tick_1ms%2 == 0)   //500hz  
			{	
				Remote_Task();
			}
			if((time_tick_1ms+1)%2 == 0&&control_mode==RC)   //750hz
			{	
				CloudParam_ControlLoop();
			}
			if(time_tick_1ms%4 == 0&&control_mode==RC)  //250hz
			{
				CMControlLoop();
			}
			if(time_tick_1ms%100 == 0)  //10hz
			{
				ui_10hzflag=1;
				CAN_Send_Message(&hcan1,0X210,(power_limit-2)*100,0,0,0);
			}
		}
	}
	else;
}

void Fake_Delay(int dzsj)//����ʱ��
{
	time_tick=time_tick_1ms;//��¼��ʼʱ��
	shijian=dzsj;
}



/**************���̵����������************/
//���̵Ŀ������񣬰�������⻷�������ڻ�����
//�⻷�����ʻ��ջ�����
//�ڻ����ٶȻ��ջ�����
//���ô���PID���ƣ���ң�����Ĳ���ֵ�ǹ��ʻ��������������ʻ�����������ٶȻ���������
//��ʵ���������һ�����������ƣ�����ǵ���ڲ���ɵģ�����������Ǵ���
	int16_t CM1Speed,CM2Speed,CM3Speed,CM4Speed=0;//�����ĸ�������ٶ�ֵ������1�ţ�����2�ţ�����3�ţ�����4��
	double vx=0;//����x����ٶ�ֵ��x����1��3���ƽ��
	double vy=0;//����y����ٶ�ֵ��y����2��1���ƽ��
	double w0;
	double vz;
//	double K_Speed=8;//���岦�˸����ٶȷŴ����ֵ

	int16_t gyro_speed=0;//�����Ƿ��ص��ٶȲ���ֵ
	uint8_t  denfense_time_flag;//����Ť��������־λ
	int16_t denfense_time_count;//���̿�ʼŤ���ͼ���
	int16_t denfense_time_period=80;//����Ť��������=80*4*4=1080ms��ͨ�����������������ٶȡ�
	int16_t yaw_bias_position=0;//PC���Ƶ�ʱ���ó������ƶ���̨
void CMControlLoop(void)
	/****����̨yaw�����е0���λ�ò�*******/
{	
  adcx=Get_Adc_Average(ADC_CHANNEL_8,20);//��ȡͨ��5��ת��ֵ��20��ȡƽ��
	tem_p=(float)adcx*(3.3/4096);          //��ȡ�����Ĵ�С����ʵ�ʵ�ѹֵ������3.1111
	cur=-(10*tem_p-25.2f);
 //  yawdegree_now=Yaw_position_Feedback.ecd_value; //���������صĽǶ�ֵ
	
	if(YAW_Mechanics_Angle<1000)
	{
	   if(Yaw_position_Feedback.real[0]>7000)//��ת��һȦ
	 {yawdegree_now =Yaw_position_Feedback.real[0]-8191;}    //pitchdegree_middle_default=4050
	   else 
	 {yawdegree_now =Yaw_position_Feedback.real[0]; }
	}
	else if(YAW_Mechanics_Angle>7000)
	{
	   if(Yaw_position_Feedback.real[0]<1000)//��ת��һȦ
	 {yawdegree_now =Yaw_position_Feedback.real[0]+8191;}
	   else 
		 yawdegree_now =Yaw_position_Feedback.real[0]; 
	}
	else
	yawdegree_now =Yaw_position_Feedback.real[0]; 
	/***********/
	if(tRC_Data.switch_right==3&&(Small_gyroscope_flag==-1))	//Ť���ٶ�pid���㣬�ɱ�������е�Ǿ���	
	{
		//========
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);//��
		//========
		if(yawdegree_now<(YAW_Mechanics_Angle-siqu))
		{	YAW=YAW_Mechanics_Angle-siqu;
		PID_Calc1(&Gyro_normal_pid,yawdegree_now,YAW,Incremental);}
		else if(yawdegree_now>(YAW_Mechanics_Angle+siqu))
		{
			YAW=YAW_Mechanics_Angle+siqu;
		  PID_Calc1(&Gyro_normal_pid,yawdegree_now,YAW,Incremental);}
		else
		if(yawdegree_now<(YAW_Mechanics_Angle-siqu))
		{	YAW=YAW_Mechanics_Angle-siqu;
		PID_Calc1(&Gyro_normal_pid,yawdegree_now,YAW,Incremental);}
		else if(yawdegree_now>(YAW_Mechanics_Angle+siqu))
		{
			YAW=YAW_Mechanics_Angle+siqu;
		  PID_Calc1(&Gyro_normal_pid,yawdegree_now,YAW,Incremental);}
		else
		Gyro_normal_pid.output=0;
		CM1Speed=(vx+vy+Gyro_normal_pid.output*0.7)*romote_rate_speed;//��С����ģʽ�µ��̽���
		CM2Speed=(-vx+vy+Gyro_normal_pid.output*0.7)*romote_rate_speed;
		CM3Speed=(-vx-vy+Gyro_normal_pid.output*0.7)*romote_rate_speed;
		CM4Speed=(vx-vy+Gyro_normal_pid.output*0.7)*romote_rate_speed;
			
		PID_Calc1(&CM1_speed_pid,CM1_Feedback.real[0],CM1Speed,Positional);
		PID_Calc1(&CM2_speed_pid,CM2_Feedback.real[0],CM2Speed,Positional);
		PID_Calc1(&CM3_speed_pid,CM3_Feedback.real[0],CM3Speed,Positional);
		PID_Calc1(&CM4_speed_pid,CM4_Feedback.real[0],CM4Speed,Positional);
		
		if(Cloud_Init_Flag==1)
		CAN_Send_Message(&hcan1,0X200,CM1_speed_pid.output,CM2_speed_pid.output,CM3_speed_pid.output,CM4_speed_pid.output);
    speed_t_flag=0;//����С���ݼ�ʱ��־
	}
	else if(tRC_Data.switch_right==1||Small_gyroscope_flag==1)
	{ 	  
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);//��
		Small_gyroscope();




	}
}

void CloudParam_ControlLoop(void)
{
	
	if(((tRC_Data.switch_right==3)||(tRC_Data.switch_right==1)))
	{
		if(Auto_Aim_flag==1)
		{
			vz=0;
			Gimbal.angle_yaw1=0;
			w0=1.8*pitchposition;
		}
		if(Cloud_Init_Flag==1)
		IMU_count_flag=1;
		Auto_Aim_flag=0;
//		Shooting_rotate();
		feedmotor(speed_shoot,bullet_motor_flag);
		Cloud();
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);	
	}
//	else if((tRC_Data.mouse.press_right))
//	{
//		Auto_Aim_flag=1;
//		IMU_count_flag=0;
//		feedmotor(speed_shoot,bullet_motor_flag);
//		Autoaiming();
//	}
}

void friction_Init(void)
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	TIM_SetTIM2Compare1(2500);
	 
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	TIM_SetTIM2Compare2(2500);
}
