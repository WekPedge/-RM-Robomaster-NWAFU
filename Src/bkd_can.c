#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"


float yaw1,pitch1,shootspeed,shootposition1,shootposition2;
float scap_power=0,scap_voltage=0,scap_current=0,cap_voltage=0,power_aim=6000;//输入功率、输入电压、输入电流、电容电压、目标功率
ReceiveTypeDef CM1_Feedback={0};
ReceiveTypeDef CM2_Feedback={0};
ReceiveTypeDef CM3_Feedback={0};
ReceiveTypeDef CM4_Feedback={0};
ReceiveTypeDef CM5_Feedback={0};
ReceiveTypeDef CM6_Feedback={0};
ReceiveTypeDef Yaw_speed_Feedback;
ReceiveTypeDef Yaw_position_Feedback;
ReceiveTypeDef Pitch_speed_Feedback;
ReceiveTypeDef Pitch_position_Feedback;
ReceiveTypeDef Shoot_speed_Feedback;
ReceiveTypeDef Shoot_position_Feedback;
ReceiveTypeDef power_Feedback;//电容控制接受
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
	if(_hcan==&hcan1)
	{
		switch(_hcan->pRxMsg->StdId)
    {
		case 0x201:
		{
			Speed_Data_deal(&CM1_Feedback,&Rx1Message);
			break;	
		}
		case 0x202:
		{
			Speed_Data_deal(&CM2_Feedback,&Rx1Message);
			break;
		}
		case 0x203:
		{
			Speed_Data_deal(&CM3_Feedback,&Rx1Message);			
			break;
		}
		case 0x204:
		{
			Speed_Data_deal(&CM4_Feedback,&Rx1Message);
		break;
		}
		case 0x205:
		{		
			Position_Data_deal(&Yaw_position_Feedback,&Rx1Message);		
			if(Calibration_Mode==CloudParam_Calibration)
				printf("   YAW_Mechanics_Angle:%d  ",Yaw_position_Feedback.real[0]);//3789
			else;
			break;			
		}
		case 0x206:
		{
			Position_Data_deal(&Pitch_position_Feedback,&Rx1Message);
		yaw1=Yaw_position_Feedback.real[0];
	  pitch1=Pitch_position_Feedback.real[0];	
			if(Calibration_Mode==CloudParam_Calibration)
				printf("   PITCH_Mechanics_Angle:%d \n ",Pitch_position_Feedback.real[0]);//974
			else;
			break;
		}
		case 0x207:
		{
			Position_Round_Data_deal(&Shoot_position_Feedback,&Rx1Message);
			Shooting_Speed_Data_deal(&Shoot_speed_Feedback,&Rx1Message);
			shootspeed=Shoot_speed_Feedback.real[0];
			shootposition1=Shoot_position_Feedback.ecd_value;
			shootposition2=Shoot_position_Feedback.real[0];
			break;
		}
				case 0x211:
		{
			powerdata_receive(&power_Feedback,&Rx1Message);
			scap_voltage=power_Feedback.powerreal[0];//输入电压
			scap_current=power_Feedback.powerreal[2];//输入电流
			cap_voltage=power_Feedback.powerreal[1];//电容电压
			if (cap_voltage<=17){HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);}
			else{HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);}
			scap_power=scap_voltage*scap_current;//输入功率(实际）
			power_aim=power_Feedback.powerreal[3];//目标功率（功率上限）
			break;
		}

		//printf("can  ");
		//__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_FMP0);//重新开启FIF00消息挂号中断

	}
	//printf("can  ");
	__HAL_CAN_ENABLE_IT(_hcan,CAN_IT_FMP0);//重新开启FIF00消息挂号中断
	}
	
	else if(_hcan==&hcan2)
	{
		switch(_hcan->pRxMsg->StdId)
    {
		case 0x201:
		{
			Speed_Data_deal(&CM5_Feedback,&Rx2Message);
			break;	
		}
		case 0x202:
		{
			Speed_Data_deal(&CM6_Feedback,&Rx2Message);
			break;
		}
	}
	__HAL_CAN_ENABLE_IT(_hcan,CAN_IT_FMP0);
  }
	}

HAL_StatusTypeDef CAN_Send_Message(CAN_HandleTypeDef* _hcan, uint16_t _id, int16_t message1,int16_t message2,int16_t message3,int16_t message4)//, uint8_t* _pBuff)
{

  _hcan->pTxMsg->RTR = CAN_RTR_DATA;
  _hcan->pTxMsg->IDE = CAN_ID_STD;
  _hcan->pTxMsg->StdId = _id;

			_hcan->pTxMsg->DLC = 8; 									//3510 = EC60四个电机
			//_hcan->pTxMsg->StdId = 0x1ff;
			_hcan->pTxMsg->Data[0] = (uint8_t)(message1>>8);
			_hcan->pTxMsg->Data[1] = (uint8_t)(message1);
			_hcan->pTxMsg->Data[2] = (uint8_t)(message2>>8);
			_hcan->pTxMsg->Data[3] = (uint8_t)(message2);
			_hcan->pTxMsg->Data[4] = (uint8_t)(message3>>8);
			_hcan->pTxMsg->Data[5] = (uint8_t)(message3);
			_hcan->pTxMsg->Data[6] = (uint8_t)(message4>>8);
			_hcan->pTxMsg->Data[7] = (uint8_t)(message4);
	HAL_CAN_Transmit(_hcan,100);
}

/***********************底盘电机的速度反馈处理**************************/
void Speed_Data_deal(ReceiveTypeDef *Receive,CanRxMsgTypeDef * msg)
{
	Receive->real[Receive->count]=(msg->Data[2]<<8)|msg->Data[3];
	Receive->count++;
	if(	Receive->count>3)
	{
		//Receive->calc=(Receive->real[0]+Receive->real[1]+Receive->real[2]+Receive->real[3])/4;
		Receive->count=0;
	}
}
/*********************云台电机的位置反馈处理****************************/
void Position_Data_deal(ReceiveTypeDef *Receive,CanRxMsgTypeDef * msg)
{
	Receive->real[Receive->count]=((msg->Data[0]<<8)|msg->Data[1]);//接收到的真实数据值
	Receive->count++;
	if(	Receive->count>1)//同上边速度反馈一样，只不过云台PID控制频率为500HZ
	{
		//Receive->calc=(Receive->real[0]+Receive->real[1])/2;//处理过后的平均数据值
		Receive->count=0;
	}
}
void Shooting_Speed_Data_deal(ReceiveTypeDef *Receive,CanRxMsgTypeDef * msg)
{
	Receive->real[Receive->count]=(msg->Data[2]<<8)|msg->Data[3];//接收到的真实数据值
	Receive->count++;
	if(	Receive->count>3)//四次处理一次平均值，接收反馈频率是1KHZ，数据平均值处理后频率为500HZ，PID处理频率为500HZ
	{
		//Receive->calc=(Receive->real[0]+Receive->real[1])/2;//处理过后的平均数据值
		Receive->count=0;
	}
}
void Position_Round_Data_deal(ReceiveTypeDef *Receive,CanRxMsgTypeDef * msg)
{
//	Receive->real[0]=((msg->Data[0]<<8)|msg->Data[1])/100;//接收到的真实数据值
	Receive->count++;
	if(	Receive->count>1)
	{
		Receive->real[0]=((msg->Data[0]<<8)|msg->Data[1])/100;//接收到的真实数据值
		if((Receive->real[0]-Receive->real[1])>50)
		{
			Receive->round_cnt--;
		}
		else if((Receive->real[1]-Receive->real[0])>50)
		{
			Receive->round_cnt++;
		}

		Receive->ecd_value=-Receive->real[0]+81.91*Receive->round_cnt+Receive->bias;
		Receive->real[1]=Receive->real[0];
		Receive->count=0;
	}
	
}
void powerdata_receive(ReceiveTypeDef *Receive,CanRxMsgTypeDef * msg)
{
	Receive->powerreal[0]=((msg->Data[0])|msg->Data[1]<<8)/100.f;//输入电压
	Receive->powerreal[1]=((msg->Data[2])|msg->Data[3]<<8)/100.f;//电容电压
	Receive->powerreal[2]=((msg->Data[4])|msg->Data[5]<<8)/100.f;//输入电流
	Receive->powerreal[3]=((msg->Data[6])|msg->Data[7]<<8)/100.f;//目标功率

	Receive->count++;
	if(	Receive->count>1) 
	{
		Receive->count=0;
	}

}