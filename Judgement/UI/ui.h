#ifndef __UI_H__
#define __UI_H__
#include "stm32f4xx_hal.h"
//��Ļ�ֱ���1920x1080
#define SCREEN_WIDTH 1080
#define SCREEN_LENGTH 1920


typedef __packed struct
{
uint8_t graphic_name[3];
uint32_t operate_tpye:3;
uint32_t graphic_tpye:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t radius:10;
uint32_t end_x:11;
uint32_t end_y:11;
} graphic_data_struct_t;

typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[7];//����ͼ���������ͼ����������ĳ��ȣ���Ҫ���������ϵͳ��������ͼ��������Ӧ������ID
} ext_client_custom_graphic_t;


//����������Ϣ
typedef __packed struct
{
uint16_t data_cmd_id;	//���ݶ�����ID
uint16_t sender_ID;	//������ID
uint16_t receiver_ID;	//������ID
ext_client_custom_graphic_t graphic_custom;//�Զ���ͼ������
}ext_student_interactive_header_data_t;	

void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len);
void draw_a_line();
extern ext_student_interactive_header_data_t custom_grapic_draw;			//�Զ���ͼ�����
extern ext_client_custom_graphic_t custom_graphic;	//�Զ���ͼ��



#endif


