#ifndef __bkd_control_H
#define __bkd_control_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* USER CODE BEGIN Includes */
/* USER CODE BEGIN Private defines */
void Control_Task(void);
void CMControlLoop(void);
void CloudParam_ControlLoop(void);
void DUOJI_close(void);
void DUOJI_open(void);
void friction_Init(void);

uint16_t Get_Adc_Average(uint32_t ch,uint8_t times);
#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

