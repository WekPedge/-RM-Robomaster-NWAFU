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
uint32_t ui_10hzflag = 0;//计时数（ui用）
static uint32_t time_tick = 0;//用在假delay中
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
int speed_t_flag=0;//小陀螺计时标志

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

void Fake_Delay(int dzsj)//动作时间
{
	time_tick=time_tick_1ms;//记录初始时间
	shijian=dzsj;
}



/**************底盘电机控制任务************/
//底盘的控制任务，包括电机外环控制与内环控制
//外环：功率环闭环控制
//内环：速度环闭环控制
//采用串行PID控制，即遥控器的拨杆值是功率环的输入量，功率环的输出量是速度环的输入量
//其实电调本身还有一个电流环控制，这个是电调内部完成的，这个不用我们处理
	int16_t CM1Speed,CM2Speed,CM3Speed,CM4Speed=0;//定义四个电机的速度值，左上1号，右上2号，左下3号，右下4号
	double vx=0;//定义x向的速度值，x向与1、3电机平行
	double vy=0;//定义y向的速度值，y向与2、1电机平行
	double w0;
	double vz;
//	double K_Speed=8;//定义拨杆给定速度放大比例值

	int16_t gyro_speed=0;//陀螺仪返回的速度补偿值
	uint8_t  denfense_time_flag;//底盘扭腰计数标志位
	int16_t denfense_time_count;//底盘开始扭腰就计数
	int16_t denfense_time_period=80;//底盘扭腰的周期=80*4*4=1080ms，通过控制周期来控制速度。
	int16_t yaw_bias_position=0;//PC控制的时候，让车身不动移动云台
void CMControlLoop(void)
	/****求云台yaw轴与机械0点的位置差*******/
{	
  adcx=Get_Adc_Average(ADC_CHANNEL_8,20);//获取通道5的转换值，20次取平均
	tem_p=(float)adcx*(3.3/4096);          //获取计算后的带小数的实际电压值，比如3.1111
	cur=-(10*tem_p-25.2f);
 //  yawdegree_now=Yaw_position_Feedback.ecd_value; //编码器返回的角度值
	
	if(YAW_Mechanics_Angle<1000)
	{
	   if(Yaw_position_Feedback.real[0]>7000)//旋转了一圈
	 {yawdegree_now =Yaw_position_Feedback.real[0]-8191;}    //pitchdegree_middle_default=4050
	   else 
	 {yawdegree_now =Yaw_position_Feedback.real[0]; }
	}
	else if(YAW_Mechanics_Angle>7000)
	{
	   if(Yaw_position_Feedback.real[0]<1000)//旋转了一圈
	 {yawdegree_now =Yaw_position_Feedback.real[0]+8191;}
	   else 
		 yawdegree_now =Yaw_position_Feedback.real[0]; 
	}
	else
	yawdegree_now =Yaw_position_Feedback.real[0]; 
	/***********/
	if(tRC_Data.switch_right==3&&(Small_gyroscope_flag==-1))	//扭腰速度pid计算，由编码器机械角决定	
	{
		//========
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);//左
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
		CM1Speed=(vx+vy+Gyro_normal_pid.output*0.7)*romote_rate_speed;//非小陀螺模式下底盘解算
		CM2Speed=(-vx+vy+Gyro_normal_pid.output*0.7)*romote_rate_speed;
		CM3Speed=(-vx-vy+Gyro_normal_pid.output*0.7)*romote_rate_speed;
		CM4Speed=(vx-vy+Gyro_normal_pid.output*0.7)*romote_rate_speed;
			
		PID_Calc1(&CM1_speed_pid,CM1_Feedback.real[0],CM1Speed,Positional);
		PID_Calc1(&CM2_speed_pid,CM2_Feedback.real[0],CM2Speed,Positional);
		PID_Calc1(&CM3_speed_pid,CM3_Feedback.real[0],CM3Speed,Positional);
		PID_Calc1(&CM4_speed_pid,CM4_Feedback.real[0],CM4Speed,Positional);
		
		if(Cloud_Init_Flag==1)
		CAN_Send_Message(&hcan1,0X200,CM1_speed_pid.output,CM2_speed_pid.output,CM3_speed_pid.output,CM4_speed_pid.output);
    speed_t_flag=0;//变速小陀螺计时标志
	}
	else if(tRC_Data.switch_right==1||Small_gyroscope_flag==1)
	{ 	  
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);//左
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
