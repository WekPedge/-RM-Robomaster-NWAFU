#include "main.h"
#include "heat.h"
#include "math.h"

#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<7)

#define KEY_PRESSED_OFFSET_R ((uint16_t)0x01<<8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)0x01<<9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)0x01<<10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)0x01<<11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)0x01<<12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)0x01<<13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)0x01<<14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)0x01<<15)

uint8_t key_w;
uint8_t key_s;
uint8_t key_a;
uint8_t key_d;
uint8_t key_shift;
uint8_t key_ctrl;
uint8_t key_q;
uint8_t key_e;
uint8_t key_r;
uint8_t key_f;
uint8_t key_g;
uint8_t key_z;
uint8_t key_x;
uint8_t key_c;
uint8_t key_v;
uint8_t key_b;

float v_z;
float dang;//键盘车速比率
float dangYP;//yaw pitch 遥控比率
float MAX=1;
float speed_x;
float speed_y;
float MIN=0.33;

int duojits=1000;
int shootingspeed_tc;

Keyboard_t Keyboard;
uint8_t control_mode;
uint8_t key_x_flag=0;
uint8_t feedspeed_flag=0;
uint8_t autoaim_flag=0;
float pwm_speed=1000;
float Shooting_flag=0;
int16_t speed_shoot=0;
float mousex=0,mousey=0;
uint8_t feedmotor_flag=0;	//摩擦轮开关标志位，1为开，0为关。
extern int rank;
extern float maxenergy;
extern float energy;
extern int Cloud_Init_Flag;
extern float lastenergy;
extern u8 IMU_count_flag;
extern uint8_t energy_flag;
extern uint8_t shoot_state;
int32_t bullet_motor_flag=0;
int16_t shooting_position=0;
int8_t Small_gyroscope_flag=-1;
int shootflag=0,shootcnt=-1,shootout;
int shootflag2=0;
extern int buchang;
int feedmotor_id=0;//判断摩擦轮状态

void motor_start()
{
	if(pwm_speed<1700)  
	{pwm_speed+=1;}
	TIM_SetTIM2Compare1(pwm_speed);
	TIM_SetTIM2Compare2(pwm_speed);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);//左
}

void motor_close()
{
	TIM_SetTIM2Compare1(1000);
	TIM_SetTIM2Compare2(1000);
	
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);//左
}

/********遥控任务处理，由右侧拨杆执行（取各方向的速度）**********/

void Remote_Task(void)
{
	if(tRC_Data.switch_right==3||tRC_Data.switch_right==1)
	{
		RC_control();
		PC_control();
		control_mode=RC;
	}
	else
	{
		Parking();
		control_mode=PC;
	}
}

void	keyboard_data_set()
{
	key_w = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_W))>>0;
	key_s = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_S))>>1;
	key_a = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_A))>>2;
	key_d = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_D))>>3;
	key_shift = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_SHIFT))>>4;
	key_ctrl = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_CTRL))>>5;
	key_q = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_Q))>>6;
	key_e = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_E))>>7;
	key_r = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_R))>>8;
	key_f = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_F))>>9;
	key_g = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_G))>>10;
	key_z = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_Z))>>11;
	key_x = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_X))>>12;
	key_c = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_C))>>13;
	key_v = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_V))>>14;
	key_b = ((tRC_Data.key.v)&(KEY_PRESSED_OFFSET_B))>>15;
}

/**************遥控器控制任务**********/
uint8_t duoji_flag=1;
uint8_t shooting_flag=0;
 void RC_control(void)//各方向速度设置
{ 
		if(mousex>0&&mousex<5)
		mousex=0.0001*(pow(2.718,mousex*0.002)-1);
	else if(mousex>-5&&mousex<0)
		mousex=-0.0001*(pow(2.718,-mousex*0.002)-1);
	
	mousex=tRC_Data.mouse.x;
	mousey=tRC_Data.mouse.y;
	if((tRC_Data.ch1<10)&&(tRC_Data.ch1>0))tRC_Data.ch1=0;
	if((tRC_Data.ch1>-10)&&(tRC_Data.ch1<0))tRC_Data.ch1=0;
	vz+=tRC_Data.ch1*0.0015f+mousex*dangYP/80+v_z;
	if((tRC_Data.ch2<10)&&(tRC_Data.ch2>0))tRC_Data.ch2=0;
	if((tRC_Data.ch2>-10)&&(tRC_Data.ch2<0))tRC_Data.ch2=0;
	w0+=mousey*dangYP/15*2-tRC_Data.ch2*0.01f; 
	if(w0>900)w0=900;
	if(w0<-1100)w0=-1100;//
	vx=tRC_Data.ch4+speed_x;
	vy=tRC_Data.ch3+speed_y;
	
	switch(tRC_Data.switch_left)
	{
		case 1:	
	 {		
		TIM_SetTIM2Compare1(2500);//kai
		feedmotor_flag=0;
		feedspeed_flag=0;
		 bullet_motor_flag = 0;
		break;
	 }
		case 3:	
	 {	
		TIM_SetTIM2Compare1(1000);//闭下
//		 feedmotor_flag=0;
//		feedspeed_flag=0;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
		break;
	 }
		
		case 2:	
	 {
		 //
		TIM_SetTIM2Compare1(1000);//闭
		feedspeed_flag=1;
		feedmotor_flag=1;
    speed_shoot=-4800;	     //低射速  4200=11.2，5000=17，5200=18.7，4800=17+
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	  bullet_motor_flag = 1000;
		feedmotor_id=rcfeedmotorspeed;
		 
		 
		 		 //		
//		          feedmotor_flag=1;
//		 					shootflag=1;
//							Shooting_flag=18.0; 
//						if(energy<(maxenergy-20))bullet_motor_flag+=495;
//					feedspeed_flag=1;
		break;		
	 }
		default:break;
	}
}


/*******驻停********/
void Parking(void)
{
	IMU_count_flag=0;
	CAN_Send_Message(&hcan1, 0X200,0,0,0,0);
	CAN_Send_Message(&hcan1, 0X1ff,0,0,0,0);
	CAN_Send_Message(&hcan2, 0X200,0,0,0,0);
	pwm_speed=1000;
	Cloud_Init_Flag=0;
	w0=0;
	Small_gyroscope_flag=-1;
}
/*******电脑控制任务********/
double K_stall=1;
void PC_control(void)
{
	keyboard_data_set();
//	Shooting_rotate();
	if(key_ctrl)
	{
		dangYP=0.3;
	}
	else	dangYP=1;

// if(key_q)	
//	{
//		feedmotor_flag=1;
//		feedspeed_flag=1;
//    speed_shoot=-4000;     //低射速4500=13+,4200=14+，4100=14
//		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);	
//		buchang=62;
//		feedmotor_id=lowfeedmotorspeed;
////		if(shootflag==0)
////	   {
////			if(feedspeed_flag==1)
////			{
//////				shootflag=1;
//////	      Shooting_flag=18.0;
//////		    speed_shoot=-0x1770;			 
//////		    bullet_motor_flag+=12000;
//////			 	feedspeed_flag=1;
////				shootflag=1;
////				speed_shoot=-4200;
////				bullet_motor_flag+=10;
////				feedspeed_flag=1;
////			}
////	   } 
//	}
/*******摩擦轮控制任务********/
 if(key_f)		//开启摩擦轮
  {
		feedmotor_flag=1;
		feedspeed_flag=1;
    speed_shoot=-6000;  //16.8-17.2     低射速4500=13+,4200=14+，4100=14
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);	
		buchang=62;
		feedmotor_id=lowfeedmotorspeed;
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);//左
	}
	
	else if(key_g)		//开启摩擦轮
  {
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,0);
		feedmotor_flag=1;
		feedspeed_flag=1;
    	speed_shoot=-4800;     //  23.0-23.5    高射速4800=16,4700=17
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
		buchang=58;
		feedmotor_id=highfeedmotorspeed;
		
	}
	else if(key_c)     //关闭摩擦轮
	{
		feedmotor_flag=0;
		CAN_Send_Message(&hcan2, 0X200,0,0,0,0);
		feedspeed_flag=0;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
		TIM_SetTIM2Compare2(2500);
		feedmotor_id=feedmotor_off;
	}	

	else if(key_x)
	{
		if(key_x_flag==0)
		{
			 key_x_flag=1;
			 Small_gyroscope_flag*=(-1);
			 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);//左
		}
	}
	else if(key_b)
	{
		TIM_SetTIM2Compare2(300);
		TIM_SetTIM2Compare1(2500);
	}
//	else if(tRC_Data.mouse.press_left||tRC_Data.switch_left==2) //鼠标左键单发
//  {
//		if(shoot_state==1)
//		{

//		{
//			if(shootflag==0)
//			{
//				if(feedspeed_flag==1)
//				{
//					shootflag=1;
//					Shooting_flag=18.0; 
//					bullet_motor_flag+=495;
//					feedspeed_flag=1;
//				}
//			}
//		}
//   }
//}
	else if(tRC_Data.mouse.press_right)
	{
		bullet_motor_flag = 5000;
}
	else if(!tRC_Data.mouse.press_right&&tRC_Data.mouse.press_left)
	{
		bullet_motor_flag = 3000;
}

    else
 {shootflag=0;shootout=0;v_z=0;key_x_flag=0;shootflag2=0;
		if(tRC_Data.switch_left==2){bullet_motor_flag=1000;}
		else {bullet_motor_flag=0;}
 }//改


 if(key_w||key_s||key_a||key_d)//改3
  {
	   if(key_shift)
		{dang = 1.5;}
		else if(key_ctrl) dang=0.5;
	    else dang=0.5;
		speed_x+=(float)(key_w-key_s)*2;//键盘操作XY方向速度
		speed_y+=(float)(key_d-key_a)*2;
		dang=dang*1.8;//地盘速度
			if(speed_x>660*dang)//最大速度限幅
				 speed_x = 660*dang;
			else if(speed_x<-660*dang)
				 speed_x = -660*dang;
			else
				 speed_x=speed_x;
			if(key_w == key_s)
		  {
		    if(speed_x>10)
		       speed_x -= 5;
		    else if(speed_x<-10)
		       speed_x += 5;
		    else
		       speed_x = 0;
	    }
	
	    if(speed_y>660*dang)
		     speed_y = 660*dang;
	    else if(speed_y<-660*dang)
		     speed_y=-660*dang;
	    else
		     speed_y=speed_y;
		if(key_d == key_a)
	 {
		  if(speed_y>10)
		     speed_y -= 10;
		  else if(speed_y<-10)
		     speed_y += 10;
		  else
		     speed_y = 0;
	 }
	}
	
  else
 {speed_x=0;speed_y=0;}//改
}


/**********复位任务************/

void mode_PC_control(void)
{
	Keyboard.mode=tRC_Data.key_code&0xc7;
	switch(Keyboard.mode)
	{
		case 1://R
		{
			break;
		}

		default:
			break;
	}
}
