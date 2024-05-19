#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "dma.h"
#include "bkd_usart.h"
extern int aiming;
extern void Fake_Delay();
RC_TypeDef tRC_Data;
uint8_t arrRC_Buf[100];
uint8_t arrRC1_Buf[8];
uint8_t arrBT_RxBuf[BT_RXBUF_LEN];
uint8_t rdata[8];
uint8_t vision[16]={0};
uint16_t dx=0,dy=0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	u8 i;
	u8 Fir_Head;
	aiming=1;
	Fake_Delay(200);
	if(huart->Instance==USART6)
	{
		for(i=0;i<8;i++)
		{
		 vision[i]=rdata[i];
		 vision[i+8]=rdata[i];
		}
		for(i=0;i<16;i++)
		{
			if(vision[i]=='#')
			{
				Fir_Head=i;
				break;
			}
		}
		dx=100*(vision[Fir_Head+1]-48)+10*(vision[Fir_Head+2]-48)+(vision[Fir_Head+3]-48);
		dy=100*(vision[Fir_Head+4]-48)+10*(vision[Fir_Head+5]-48)+vision[Fir_Head+6]-48;
	}
}

HAL_StatusTypeDef Junru_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
	Junru_UART_Receive_DMA(huart,pData,Size);

	return HAL_OK;
}

void Dma_Callback_RC_Handle(RC_TypeDef* rc, uint8_t* buff)
{
	rc->ch1 = (buff[0] | buff[1]<<8) & 0x07FF;
	rc->ch1 -= 1024;
	rc->ch2 = (buff[1]>>3 | buff[2]<<5 ) & 0x07FF;
	rc->ch2 -= 1024;
	rc->ch3 = (buff[2]>>6 | buff[3]<<2 | buff[4]<<10) & 0x07FF;
	rc->ch3 -= 1024;
	rc->ch4 = (buff[4]>>1 | buff[5]<<7) & 0x07FF;		
	rc->ch4 -= 1024;
	
	rc->switch_left = ( (buff[5] >> 4)& 0x000C ) >> 2;
	rc->switch_right =  (buff[5] >> 4)& 0x0003 ;
	
	rc->mouse.x = buff[6] | (buff[7] << 8);
	rc->mouse.y = buff[8] | (buff[9] << 8);
	rc->mouse.z = buff[10]| (buff[11] << 8);
	
	rc->mouse.press_left 	= buff[12];
	rc->mouse.press_right = buff[13];
	
	rc->key.v = buff[14] | buff[15] << 8; 
//	rc->key_code= buff[14] | buff[15] << 8;
}

void UART_IdleRxCallback(UART_HandleTypeDef *huart)
{
		Dma_Callback_RC_Handle(&tRC_Data, arrRC_Buf);
}

void Junru_UsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size)
{
	uint32_t DMA_FLAGS;
	UART_IdleRxCallback(huart);
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
      __HAL_UART_CLEAR_IDLEFLAG(huart); 
	  DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx);
      __HAL_DMA_DISABLE(huart->hdmarx);
	  __HAL_DMA_CLEAR_FLAG(huart->hdmarx,DMA_FLAGS);
	  __HAL_DMA_SET_COUNTER(huart->hdmarx,Size);
	  __HAL_DMA_ENABLE(huart->hdmarx);
 	}
}



