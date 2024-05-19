#ifndef _MYIIC_H
#define _MYIIC_H
#include "stdint.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F429������
//IIC��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/1/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
//IO��������
#define SDA_IN()  {GPIOF->MODER&=~(3<<(0*2));GPIOF->MODER|=0<<0*2;}	//PH5����ģʽ      PF0=����ģʽ
#define SDA_OUT() {GPIOF->MODER&=~(3<<(0*2));GPIOF->MODER|=1<<0*2;}//PH5���ģʽ       PF0=���ģʽ   ������һ��IOռ2λ
//IO����
#define IIC_SCL   PFout(1) //SCL
#define IIC_SDA   PFout(0) //SDA
#define READ_SDA  PFin(0)  //����SDA

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
#endif
