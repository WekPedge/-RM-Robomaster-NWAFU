#ifndef __bkd_can_H
#define __bkd_can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */



/* USER CODE BEGIN Private defines */
typedef struct
{
  int16_t  real[8];
	uint8_t   count;
	int16_t round_cnt;
	int32_t ecd_value;
	int16_t sum;
	int16_t bias;
	int16_t calc;
	float powerreal[8];
} ReceiveTypeDef;
					    						   										 							 				  
void Speed_Data_deal(ReceiveTypeDef *Receive,CanRxMsgTypeDef * msg);
void Position_Data_deal(ReceiveTypeDef *Receive,CanRxMsgTypeDef * msg);
void Position_Round_Data_deal(ReceiveTypeDef *Receive,CanRxMsgTypeDef * msg);
void Shooting_Speed_Data_deal(ReceiveTypeDef *Receive,CanRxMsgTypeDef * msg);
void powerdata_receive(ReceiveTypeDef *Receive,CanRxMsgTypeDef * msg);
/* USER CODE END Private defines */



/* USER CODE BEGIN Prototypes */
HAL_StatusTypeDef CAN_Send_Message(CAN_HandleTypeDef* _hcan, uint16_t _id, int16_t message1,int16_t message2,int16_t message3,int16_t message4);//, uint8_t* _pBuff)
extern ReceiveTypeDef CM1_Feedback;
extern ReceiveTypeDef CM2_Feedback;
extern ReceiveTypeDef CM3_Feedback;
extern ReceiveTypeDef CM4_Feedback;
extern ReceiveTypeDef CM5_Feedback;
extern ReceiveTypeDef CM6_Feedback;

extern ReceiveTypeDef Yaw_speed_Feedback;
extern ReceiveTypeDef Yaw_position_Feedback;

extern ReceiveTypeDef Pitch_speed_Feedback;
extern ReceiveTypeDef Pitch_position_Feedback;

extern ReceiveTypeDef Shoot_speed_Feedback;
extern ReceiveTypeDef Shoot_position_Feedback;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */
/**
  * @}
  */

/**
  * @}
  */

