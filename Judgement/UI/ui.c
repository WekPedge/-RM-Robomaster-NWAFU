#include "ui.h"
#include "proticol.h"
#include "string.h"
#include "main.h"
#include "usart.h"
/**
  * @brief     pack data to bottom device
  * @param[in] sof：framehearder
  * @param[in] cmd_id:  command id of data
  * @param[in] *p_data: pointer to the data to be sent
  * @param[in] len:     the data length
  */
#define MAX_SIZE          128    //上传数据最大的长度
#define frameheader_len  5       //帧头长度
#define cmd_len          2       //命令码长度
#define crc_len          2       //CRC16校验
uint8_t seq=0;

void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
	unsigned char i=i;
	
	uint8_t tx_buff[MAX_SIZE];

	uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //数据帧长度	

	memset(tx_buff,0,frame_length);  //存储数据的数组清零
	
	/*****帧头打包*****/
	tx_buff[0] = sof;//数据帧起始字节
	memcpy(&tx_buff[1],(uint8_t*)&len, sizeof(len));//数据帧中data的长度
	tx_buff[3] = seq;//包序号
	append_crc8_check_sum(tx_buff,frameheader_len);  //帧头校验CRC8

	/*****命令码打包*****/
	memcpy(&tx_buff[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****数据打包*****/
	memcpy(&tx_buff[frameheader_len+cmd_len], p_data, len);
	append_crc16_check_sum(tx_buff,frame_length);  //一帧数据校验CRC16

	if (seq == 0xff) seq=0;
  else seq++;
	
	/*****数据上传*****/
	  HAL_UART_Transmit_DMA(&huart7,tx_buff,frame_length);
}



ext_student_interactive_header_data_t custom_grapic_draw;			//自定义图像绘制
ext_client_custom_graphic_t custom_graphic;	//自定义图像

//初始化图形数据变量
	//自定义图形绘制
	void draw_a_line()
	{
		custom_grapic_draw.data_cmd_id=0x0104;//绘制七个图形（内容ID，查询裁判系统手册）
		

			custom_grapic_draw.sender_ID=103;//发送者ID，机器人对应ID，此处为蓝方英雄
			custom_grapic_draw.receiver_ID=0x0167;//接收者ID，操作手客户端ID，此处为蓝方英雄操作手客户端
		//自定义图像数据
		{
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 97;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 97;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;//图形名
		//上面三个字节代表的是图形名，用于图形索引，可自行定义
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=1;//图形操作，0：空操作；1：增加；2：修改；3：删除；
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=0;//图形类型，0为直线，其他的查看用户手册
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=1;//图层数
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=1;//颜色
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_angle=0;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=1;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=SCREEN_LENGTH/2;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=SCREEN_WIDTH/2;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_x=SCREEN_LENGTH/2;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_y=SCREEN_WIDTH/2-300;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].radius=0;
		}
	}
	//此处只绘制了图形1，其他图形参考上述自行给图形数据数组赋值即可






		
		
		
