#ifndef __bkd_usart_H
#define __bkd_usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#define RC_BUF_LEN    100
#define BT_RXBUF_LEN  20
#define BT_TXBUF_LEN  20
/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */

/** 
  * @brief  遥控数据结构体
  */
typedef struct{
	//遥控器通道
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	uint8_t switch_left;
	uint8_t switch_right;
	//鼠标
	struct{
		int16_t x;
		int16_t y;
		int16_t z;
		
		uint16_t press_left;
		uint16_t press_right;
	}mouse;
	//键盘
	uint16_t key_code;
	struct
	{
		uint16_t v;
	}key;
	uint8_t keyBoardFlag[16][2];
 // SelfCheckTypeDef  tSelfCheck;
/*************************************************************************************
   * 键盘通道:15   14   13   12   11   10    9   8    7    6     5    4    3    2    1
   *          V    C    X	   Z    G    F    R    E   Q   CTRL  SHIFT  D    A    S    W
**************************************************************************************/
}RC_TypeDef;

/* USER CODE END Private defines */

extern uint8_t arrRC_Buf[100];
extern uint8_t arrRC1_Buf[8];
extern uint8_t vision[16];
extern uint16_t dx,dy;
extern uint8_t rdata[8];
extern RC_TypeDef tRC_Data;

/* USER CODE BEGIN Prototypes */
HAL_StatusTypeDef Junru_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void Junru_UsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size);
void Junru1_UsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size);
void  Remote_CHECKING(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

