#include "judgement.h"
#include "proticol.h"
#include "string.h"
#include "main.h"
#include "Heat.h"
float energy=0;
int rank=1;
extern float Shooting_flag;
uint8_t Rotational_number=0;
extern uint8_t feedmotor_flag;
extern receive_judge_t judge_rece_mesg;
extern u8 bloodstage;
extern int32_t speed;
extern int32_t bullet_motor_flag;
extern uint8_t key_v;
extern uint8_t key_r;
extern uint8_t key_shift;
uint8_t Bunker_Flag=0;
uint8_t rote_flag=0;
float maxenergy;//爆发优先{150,280,400} 冷却优先{50,100,150} 弹速优先{75,150,200}
//float power[3]={60,80,100};//功率优先[60,80,100],血量优先[45,50,55]
float power=115;//功率优先[60,80,100],血量优先[45,50,55]
extern uint8_t feedspeed_flag;
void Shooting_rotate()
{
	rank=judge_rece_mesg.robot_state_data.robot_level;
	maxenergy=judge_rece_mesg.robot_state_data.shooter_id1_17mm_cooling_limit;;
	if(rank==0)
	  rank=1;	
	
	if(key_r||(tRC_Data.switch_left==2))
	{
		if(rote_flag==0)
		{
			if(feedspeed_flag==1)
			{
//			Rotational_number=(int)(((maxenergy-energy)/10)*0.5);
		 Rotational_number=(int)(((maxenergy-energy)/10)*0.5);
		  bullet_motor_flag+=495*Rotational_number; 
			rote_flag=1;
			}
    }
	}
	else 
	{ 
		rote_flag=0;
	}
}
