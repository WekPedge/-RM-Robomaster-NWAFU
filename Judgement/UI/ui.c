#include "ui.h"
#include "proticol.h"
#include "string.h"
#include "main.h"
#include "usart.h"
/**
  * @brief     pack data to bottom device
  * @param[in] sof��framehearder
  * @param[in] cmd_id:  command id of data
  * @param[in] *p_data: pointer to the data to be sent
  * @param[in] len:     the data length
  */
#define MAX_SIZE          128    //�ϴ��������ĳ���
#define frameheader_len  5       //֡ͷ����
#define cmd_len          2       //�����볤��
#define crc_len          2       //CRC16У��
uint8_t seq=0;

void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
	unsigned char i=i;
	
	uint8_t tx_buff[MAX_SIZE];

	uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //����֡����	

	memset(tx_buff,0,frame_length);  //�洢���ݵ���������
	
	/*****֡ͷ���*****/
	tx_buff[0] = sof;//����֡��ʼ�ֽ�
	memcpy(&tx_buff[1],(uint8_t*)&len, sizeof(len));//����֡��data�ĳ���
	tx_buff[3] = seq;//�����
	append_crc8_check_sum(tx_buff,frameheader_len);  //֡ͷУ��CRC8

	/*****��������*****/
	memcpy(&tx_buff[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****���ݴ��*****/
	memcpy(&tx_buff[frameheader_len+cmd_len], p_data, len);
	append_crc16_check_sum(tx_buff,frame_length);  //һ֡����У��CRC16

	if (seq == 0xff) seq=0;
  else seq++;
	
	/*****�����ϴ�*****/
	  HAL_UART_Transmit_DMA(&huart7,tx_buff,frame_length);
}



ext_student_interactive_header_data_t custom_grapic_draw;			//�Զ���ͼ�����
ext_client_custom_graphic_t custom_graphic;	//�Զ���ͼ��

//��ʼ��ͼ�����ݱ���
	//�Զ���ͼ�λ���
	void draw_a_line()
	{
		custom_grapic_draw.data_cmd_id=0x0104;//�����߸�ͼ�Σ�����ID����ѯ����ϵͳ�ֲᣩ
		

			custom_grapic_draw.sender_ID=103;//������ID�������˶�ӦID���˴�Ϊ����Ӣ��
			custom_grapic_draw.receiver_ID=0x0167;//������ID�������ֿͻ���ID���˴�Ϊ����Ӣ�۲����ֿͻ���
		//�Զ���ͼ������
		{
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 97;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 97;
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;//ͼ����
		//���������ֽڴ������ͼ����������ͼ�������������ж���
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=1;//ͼ�β�����0���ղ�����1�����ӣ�2���޸ģ�3��ɾ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=0;//ͼ�����ͣ�0Ϊֱ�ߣ������Ĳ鿴�û��ֲ�
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=1;//ͼ����
		custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=1;//��ɫ
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
	//�˴�ֻ������ͼ��1������ͼ�βο��������и�ͼ���������鸳ֵ����






		
		
		
