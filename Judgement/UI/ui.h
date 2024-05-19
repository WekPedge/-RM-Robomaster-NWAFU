#ifndef __UI_H__
#define __UI_H__
#include "stm32f4xx_hal.h"
//屏幕分辨率1920x1080
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
graphic_data_struct_t grapic_data_struct[7];//绘制图像的数量即图像数据数组的长度，但要看清楚裁判系统给的增加图形数量对应的内容ID
} ext_client_custom_graphic_t;


//交互数据信息
typedef __packed struct
{
uint16_t data_cmd_id;	//数据段内容ID
uint16_t sender_ID;	//发送者ID
uint16_t receiver_ID;	//接受者ID
ext_client_custom_graphic_t graphic_custom;//自定义图形数据
}ext_student_interactive_header_data_t;	

void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len);
void draw_a_line();
extern ext_student_interactive_header_data_t custom_grapic_draw;			//自定义图像绘制
extern ext_client_custom_graphic_t custom_graphic;	//自定义图像



#endif


