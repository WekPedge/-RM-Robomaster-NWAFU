/*************************************************************

RM�Զ���UIЭ��       ����RM2020ѧ������ͨ��Э��V1.1

ɽ������ѧ ����ս�� ����@Rjgawuie

**************************************************************/


#include "RM_Cilent_UI.h"
#include "main.h"
#include "string.h"
#include "Heat.h"
#include "judgement.h"
#include "proticol.h"
unsigned char UI_Seq;                      //�����


/****************************************��������ӳ��************************************/

/********************************************ɾ������*************************************
**������Del_Operate  ��Ӧͷ�ļ�ɾ������
        Del_Layer    Ҫɾ���Ĳ� ȡֵ0-9
*****************************************************************************************/

void UI_Send( uint8_t *pData, uint16_t Size)
{
	 HAL_UART_Transmit_DMA(&huart7,pData,Size);
   while(__HAL_UART_GET_FLAG(&huart7, UART_FLAG_TC)==RESET);
	 pData++;
}
void UI_Delete(uint8_t Del_Operate,uint8_t Del_Layer)
{

   unsigned char *framepoint;                      //��дָ��
   uint16_t frametail=0xFFFF;                        //CRC16У��ֵ
   int loop_control;                       //For����ѭ������
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   UI_Data_Delete del;
   
   framepoint=(unsigned char *)&framehead;
   
   framehead.SOF=UI_SOF;
   framehead.Data_Length=8;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //����ͷ����
   
   datahead.Data_ID=UI_Data_ID_Del;
   datahead.Sender_ID=Robot_ID;
   datahead.Receiver_ID=Cilent_ID;                          //����������
   
   del.Delete_Operate=Del_Operate;
   del.Layer=Del_Layer;                                     //������Ϣ
   
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&del;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(del),frametail);  //CRC16У��ֵ����
   
   framepoint=(unsigned char *)&framehead;
	 UI_Send(framepoint,sizeof(framehead));
//   HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(framehead)); 
   framepoint=(unsigned char *)&datahead;
	 UI_Send(framepoint,sizeof(datahead));
//   HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(datahead));
   framepoint=(unsigned char *)&del;
	 UI_Send(framepoint,sizeof(del));
//   HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(del));                                                                //��������֡
   framepoint=(unsigned char *)&frametail;
	 UI_Send(framepoint,sizeof(frametail));
//   HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(frametail));
   
   UI_Seq++;                                                         //�����+1
}
/************************************************����ֱ��*************************************************
**������*image Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
        imagename[3]   ͼƬ���ƣ����ڱ�ʶ����
        Graph_Operate   ͼƬ��������ͷ�ļ�
        Graph_Layer    ͼ��0-9
        Graph_Color    ͼ����ɫ
        Graph_Width    ͼ���߿�
        Start_x��Start_x    ��ʼ����
        End_x��End_y   ��������
**********************************************************************************************************/
        
void Line_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y)
{
   int i;
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************���ƾ���*************************************************
**������*image Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
        imagename[3]   ͼƬ���ƣ����ڱ�ʶ����
        Graph_Operate   ͼƬ��������ͷ�ļ�
        Graph_Layer    ͼ��0-9
        Graph_Color    ͼ����ɫ
        Graph_Width    ͼ���߿�
        Start_x��Start_x    ��ʼ����
        End_x��End_y   �������꣨�Զ������꣩
**********************************************************************************************************/
        
void Rectangle_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y)
{
   int i;
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Rectangle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************������Բ*************************************************
**������*image Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
        imagename[3]   ͼƬ���ƣ����ڱ�ʶ����
        Graph_Operate   ͼƬ��������ͷ�ļ�
        Graph_Layer    ͼ��0-9
        Graph_Color    ͼ����ɫ
        Graph_Width    ͼ���߿�
        Start_x��Start_x    Բ������
        Graph_Radius  ͼ�ΰ뾶
**********************************************************************************************************/
        
void Circle_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t Graph_Radius)
{
   int i;
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Circle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->radius = Graph_Radius;
}

/************************************************����Բ��*************************************************
**������*image Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
        imagename[3]   ͼƬ���ƣ����ڱ�ʶ����
        Graph_Operate   ͼƬ��������ͷ�ļ�
        Graph_Layer    ͼ��0-9
        Graph_Color    ͼ����ɫ
        Graph_Width    ͼ���߿�
        Graph_StartAngle,Graph_EndAngle    ��ʼ����ֹ�Ƕ�
        Start_y,Start_y    Բ������
        x_Length,y_Length   x,y�������᳤���ο���Բ
**********************************************************************************************************/
        
void Arc_Draw(Graph_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_StartAngle,uint32_t Graph_EndAngle,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t x_Length,uint32_t y_Length)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Arc;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_StartAngle;
   image->end_angle = Graph_EndAngle;
   image->end_x = x_Length;
   image->end_y = y_Length;
}



/************************************************���Ƹ���������*************************************************
**������*image Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
        imagename[3]   ͼƬ���ƣ����ڱ�ʶ����
        Graph_Operate   ͼƬ��������ͷ�ļ�
        Graph_Layer    ͼ��0-9
        Graph_Color    ͼ����ɫ
        Graph_Width    ͼ���߿�
        Graph_Size     �ֺ�
        Graph_Digit    С��λ��
        Start_x��Start_x    ��ʼ����
        Graph_Float   Ҫ��ʾ�ı���
**********************************************************************************************************/
        
void Float_Draw(Float_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,float Graph_Float)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Float;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_Size;
   image->end_angle = Graph_Digit;
   image->graph_Float = Graph_Float;
}



/************************************************�����ַ�������*************************************************
**������*image Graph_Data���ͱ���ָ�룬���ڴ��ͼ������
        imagename[3]   ͼƬ���ƣ����ڱ�ʶ����
        Graph_Operate   ͼƬ��������ͷ�ļ�
        Graph_Layer    ͼ��0-9
        Graph_Color    ͼ����ɫ
        Graph_Width    ͼ���߿�
        Graph_Size     �ֺ�
        Graph_Digit    �ַ�����
        Start_x��Start_x    ��ʼ����
        *Char_Data          �������ַ�����ʼ��ַ
**********************************************************************************************************/
        
void Char_Draw(String_Data *image,char imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,char *Char_Data)
{
   int i;
      for(i=0;i<Graph_Digit;i++)
   {
      image->show_Data[i]=*Char_Data;
      Char_Data++;
   }
   for(i=0;i<3&&imagename[i]!=0;i++)
    image->Graph_Control.graphic_name[2-i]=imagename[i];
   image->Graph_Control.graphic_tpye = UI_Graph_Char;
   image->Graph_Control.operate_tpye = Graph_Operate;
   image->Graph_Control.layer = Graph_Layer;
   image->Graph_Control.color = Graph_Color;
   image->Graph_Control.width = Graph_Width;
   image->Graph_Control.start_x = Start_x;
   image->Graph_Control.start_y = Start_y;
   image->Graph_Control.start_angle = Graph_Size;
   image->Graph_Control.end_angle = Graph_Digit;
   
//   for(i=0;i<Graph_Digit;i++)
//   {
//      image->show_Data[i]=*Char_Data;
//      Char_Data++;
//   }
}

/************************************************UI���ͺ�����ʹ������Ч��*********************************
**������ cnt   ͼ�θ���
         ...   ͼ�α�������


Tips�����ú���ֻ������1��2��5��7��ͼ�Σ�������ĿЭ��δ�漰
**********************************************************************************************************/
int UI_ReFresh(int cnt,...)
{
   int i;
   Graph_Data imageData;
   unsigned char *framepoint;                      //��дָ��
   uint16_t frametail=0xFFFF;                        //CRC16У��ֵ
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   
   va_list ap;
   va_start(ap,cnt);
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+cnt*15;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //����ͷ����
   
   switch(cnt)
   {
      case 1:
         datahead.Data_ID=UI_Data_ID_Draw1;
         break;
      case 2:
         datahead.Data_ID=UI_Data_ID_Draw2;
         break;
      case 5:
         datahead.Data_ID=UI_Data_ID_Draw5;
         break;
      case 7:
         datahead.Data_ID=UI_Data_ID_Draw7;
         break;
      default:
         return (-1);
   }
   datahead.Sender_ID=Robot_ID;
   datahead.Receiver_ID=Cilent_ID;                          //����������
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);          //CRC16У��ֵ���㣨���֣�
   framepoint=(unsigned char *)&framehead;
	 UI_Send(framepoint,sizeof(framehead));
//   HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(framehead));

   framepoint=(unsigned char *)&datahead;
	 UI_Send(framepoint,sizeof(datahead));
//   HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(datahead));
   for(i=0;i<cnt;i++)
   {
      imageData=va_arg(ap,Graph_Data);
      
      framepoint=(unsigned char *)&imageData;
      frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail); 		 //CRC16У��
			UI_Send(framepoint,sizeof(imageData));
//      HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(imageData));                                              //����ͼƬ֡
   }
   framepoint=(unsigned char *)&frametail;
	 UI_Send(framepoint,sizeof(frametail));
//   HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(frametail));
   va_end(ap);
   UI_Seq++;                                                         //�����+1
   return 0;
}

/************************************************UI�����ַ���ʹ������Ч��*********************************
**������ cnt   ͼ�θ���
         ...   ͼ�α�������


Tips�����ú���ֻ������1��2��5��7��ͼ�Σ�������ĿЭ��δ�漰
**********************************************************************************************************/
int Char_ReFresh(String_Data string_Data)
{
   int i;
   String_Data imageData;
   unsigned char *framepoint;                      //��дָ��
   uint16_t frametail=0xFFFF;                        //CRC16У��ֵ
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   imageData=string_Data;
   

   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+45;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //����ͷ����
   

   datahead.Data_ID=UI_Data_ID_DrawChar;

   datahead.Sender_ID=Robot_ID;
   datahead.Receiver_ID=Cilent_ID;                          //����������
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&imageData;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16У��   //CRC16У��ֵ���㣨���֣�
   
   framepoint=(unsigned char *)&framehead;
	 UI_Send(framepoint,sizeof(framehead));
//	 HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(framehead));
	
	 
   framepoint=(unsigned char *)&datahead;
	 UI_Send(framepoint,sizeof(datahead));
//   HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(datahead));                                                  //���Ͳ�������  
   framepoint=(unsigned char *)&imageData;
	 UI_Send(framepoint,sizeof(imageData));
//	 HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(imageData)); 
                                              //����ͼƬ֡
   
   framepoint=(unsigned char *)&frametail;
	 UI_Send(framepoint,sizeof(frametail));
//   HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(frametail));
   UI_Seq++;                                                         //�����+1
   return 0;
}
/**********************************************************************************************************/

int Float_ReFresh(Float_Data float_Data)
{
   int i;
   Float_Data imageData;
   unsigned char *framepoint;                      //��дָ��
   uint16_t frametail=0xFFFF;                        //CRC16У��ֵ
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   imageData=float_Data;
   

   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+45;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //����ͷ����
   

   datahead.Data_ID=UI_Data_ID_Draw1;

   datahead.Sender_ID=Robot_ID;
   datahead.Receiver_ID=Cilent_ID;                          //����������
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&imageData;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16У��   //CRC16У��ֵ���㣨���֣�
   
   framepoint=(unsigned char *)&framehead;
	 UI_Send(framepoint,sizeof(framehead));
//	 HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(framehead));
	
	 
   framepoint=(unsigned char *)&datahead;
	 UI_Send(framepoint,sizeof(datahead));
//   HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(datahead));                                                  //���Ͳ�������  
   framepoint=(unsigned char *)&imageData;
	 UI_Send(framepoint,sizeof(imageData));
//	 HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(imageData)); 
                                              //����ͼƬ֡
   
   framepoint=(unsigned char *)&frametail;
	 UI_Send(framepoint,sizeof(frametail));
//   HAL_UART_Transmit_DMA(&huart7,framepoint,sizeof(frametail));
   UI_Seq++;                                                         //�����+1
   return 0;
}

/*****************************************************CRC8У��ֵ����**********************************************/
const unsigned char CRC8_INIT_UI = 0xff; 
const unsigned char CRC8_TAB_UI[256] = 
{ 
0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 
0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 
0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 
0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 
0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 
0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 
0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 
0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 
0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 
0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 
0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 
0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35, 
};
unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8) 
{ 
unsigned char ucIndex; 
while (dwLength--) 
{ 
ucIndex = ucCRC8^(*pchMessage++); 
ucCRC8 = CRC8_TAB_UI[ucIndex]; 
} 
return(ucCRC8); 
}

uint16_t CRC_INIT_UI = 0xffff; 
const uint16_t wCRC_Table_UI[256] = 
{ 
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/* 
** Descriptions: CRC16 checksum function 
** Input: Data to check,Stream length, initialized checksum 
** Output: CRC checksum 
*/ 
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC) 
{ 
uint8_t chData; 
if (pchMessage == NULL) 
{ 
return 0xFFFF; 
} 
while(dwLength--) 
{ 
chData = *pchMessage++;
(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table_UI[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 
0x00ff]; 
} 
return wCRC; 
}



/**************************��̬UI��UI��ʼ������**************/
	String_Data CH_SHOOT;
	String_Data CH_SGY;
	String_Data CH_CAP_V;
	String_Data DATA_CAP_VOLTAGE;
String_Data DATA_SGY_VOLTAGE;
	String_Data POWERLINE;
	char v_ch[]="V";//��λ
	char w_ch[]="W";//��λ
	char powerline_arr[30];
	char cap_voltage_arr[30];
	char sgyd_arr[30];
	char shoot_arr[30]="shoot";
	char cap_v_arr[30]="cap_v";
	char sgy_arr[30]="sgy";
	char flrb_arr[4]="FRBL";
	uint8_t Senbuff[] = "Zxiaoxuan"; 
	uint8_t Senbuf[] = "jjjj";
//		HAL_UART_Transmit_DMA(&huart7, (uint8_t *)Senbuff, sizeof(Senbuff));
//		while((HAL_UART_Transmit_DMA(&huart7,Senbuff,sizeof(Senbuff))==HAL_BUSY));
//		HAL_UART_Transmit_DMA(&huart7, (uint8_t *)Senbuf, sizeof(Senbuf));
		
Graph_Data G1,G2,G3,G4,G5,G6,G7,G8,G9,G15,G17,G16,G10,G11,G12,G13,G14,G15,G16,G17,G18;	
extern float scap_power,scap_voltage,scap_current,cap_voltage,power_aim;//���빦�ʣ�ʵ�ʣ��������ѹ��������������ݵ�ѹ��Ŀ�깦�ʣ����ޣ�
extern float chassis_power_limit;
extern float chassis_power_real;
void UI_Init()//UI��ʼ��
{
		memset(&G1,0,sizeof(G1));//���Ĵ���
		memset(&G2,0,sizeof(G2));//�ϻ�����
		memset(&G3,0,sizeof(G3));//����ˮƽ��
		memset(&G4,0,sizeof(G4));//ǹ��������
		memset(&G5,0,sizeof(G5));//�»�����
		memset(&G6,0,sizeof(G6));//Զ���������
		memset(&G7,0,sizeof(G7));
		memset(&CH_SHOOT,0,sizeof(CH_SHOOT));//Ħ���ֱ�ʶ
	
		memset(&G15,0,sizeof(G15));//Ħ����
		memset(&G17,0,sizeof(G17));//����
		memset(&G16,0,sizeof(G16));//С����
		memset(&G10,0,sizeof(G10));//�󳵵�
		memset(&G11,0,sizeof(G11));//�ҳ���
		memset(&G12,0,sizeof(G12));//ǰ����
		memset(&G13,0,sizeof(G13));//����������
		memset(&G14,0,sizeof(G14));//��ǰ������
		memset(&G18,0,sizeof(G18));//�������޵�
}
void UI_set()//��̬UI���ã���̬UI��ʼ�趨
{ 
//����UI
    Line_Draw(&G1,"091",UI_Graph_ADD,9,UI_Color_Green,1,960,330,960,620);//���Ĵ���
		Line_Draw(&G2,"092",UI_Graph_ADD,9,UI_Color_Green,1,902,426,1018,426);//1M��h=426,l=90(�������ߣ�
		Circle_Draw(&G3,"093",UI_Graph_ADD,9,UI_Color_Main,1,960,429,35);//1.5MȦh=429,r=35        1.5m��h=420,l=68
		Circle_Draw(&G4,"094",UI_Graph_ADD,9,UI_Color_Main,1,960,429,30);//2-3mȦh=429,r=30              2-3m��h=410,l=45
		Circle_Draw(&G5,"095",UI_Graph_ADD,9,UI_Color_Main,1,960,429,59);//װ�װ巶ΧԲȦ,r=59,h=429 ����h=467,l=90 ����h=390,l=90
	  Line_Draw(&G6,"096",UI_Graph_ADD,9,UI_Color_Green,1,937,410,983,410);//2-3m��h=410,l=45
		Line_Draw(&G7,"097",UI_Graph_ADD,9,UI_Color_Purplish_red,1,800,540,1120,540);//ˮƽ��
		Line_Draw(&G8,"087",UI_Graph_ADD,9,UI_Color_Cyan,1,960-12,347,960+12,347);//5MԶ������ h=347,l=25
//����UI
		Line_Draw(&G10,"081",UI_Graph_ADD,8,UI_Color_Orange,5,960-400,0,960-375,70);//�󣬳���h=70,��l=751,��l=800
		Line_Draw(&G11,"082",UI_Graph_ADD,8,UI_Color_Orange,5,960+400,0,960+375,70);//��
	  Line_Draw(&G12,"083",UI_Graph_ADD,8,UI_Color_Orange,5,960-375,70,960+375,70);//ǰ����
//���̹�����
		Line_Draw(&G13,"084",UI_Graph_ADD,8,UI_Color_Main,20,960-310,10,960+310,10);
		Line_Draw(&G14,"085",UI_Graph_ADD,8,UI_Color_Green,15,960-300,10,960+300,10);
		sprintf(powerline_arr,"%.2f %s",chassis_power_real,w_ch);
		Char_Draw(&POWERLINE,"086",UI_Graph_ADD,8 ,UI_Color_Green,18,5,4,960+305,20,powerline_arr);
//Ħ���ֱ�ʶ
	  Circle_Draw(&G15,"071",UI_Graph_ADD,7,UI_Color_Main,13,170,640,15);
	  Char_Draw(&CH_SHOOT,"072",UI_Graph_ADD,7 ,UI_Color_Yellow,18,5,4,60,640,shoot_arr);			
//С���ݱ�ʶ	
	  sprintf(sgyd_arr,"%d",0);
		Char_Draw(&DATA_SGY_VOLTAGE,"073",UI_Graph_ADD,7 ,UI_Color_Green,18,10,4,170,590,sgyd_arr);
	  Char_Draw(&CH_SGY,"074",UI_Graph_ADD,7 ,UI_Color_Cyan,18,15,4,60,590,sgy_arr);
//���ݵ�ѹ��ʶ
		Circle_Draw(&G17,"075",UI_Graph_ADD,7,UI_Color_Main,13,170,540,15);
	  Char_Draw(&CH_CAP_V,"076",UI_Graph_ADD,7 ,UI_Color_Green,18,5,4,60,540,cap_v_arr);
//���ݵ�ѹ����
		float2char(scap_current*cap_voltage,cap_voltage_arr);
		Char_Draw(&DATA_CAP_VOLTAGE,"077",UI_Graph_ADD,7 ,UI_Color_Green,18,5,4,190,540,cap_voltage_arr);
//���̹������������޵�򻺴���
		Line_Draw(&G9,"061",UI_Graph_ADD,6,UI_Color_Yellow,20,960,10,960+5,10);
//UI����

		UI_ReFresh(7,G1,G2,G3,G4,G5,G6,G7);//����ͼ��	
		UI_ReFresh(7,G8,G9,G10,G11,G12,G13,G14);
		UI_ReFresh(5,G15,G16,G17,G9,G1);
		Char_ReFresh(CH_CAP_V);
		Char_ReFresh(CH_SGY);
		Char_ReFresh(CH_SHOOT);
		Char_ReFresh(DATA_CAP_VOLTAGE);
		Char_ReFresh(DATA_SGY_VOLTAGE);
		Char_ReFresh(POWERLINE);
}

extern int feedmotor_id;
extern int8_t Small_gyroscope_flag;
extern float power;
extern int rank;
extern receive_judge_t judge_rece_mesg;


uint32_t cap_voltage_color;
uint32_t powerline_color;
uint32_t sgy_color;
extern int rank;
void UI_change()
{
		//���ݵ�ѹ�ж�
	
			cap_voltage_color=(chassis_power_limit<=15)*UI_Color_Purplish_red+(chassis_power_limit>15)*UI_Color_Green;	//����15����,����15���̱� ���ݵ�ѹ
			sprintf(cap_voltage_arr,"%.2f%s",chassis_power_limit,v_ch);
			Circle_Draw(&G17,"075",UI_Graph_Change,7,cap_voltage_color,13,170,540,15);
			Char_Draw(&DATA_CAP_VOLTAGE,"077",UI_Graph_Change,7 ,cap_voltage_color,18,10,4,190,540,cap_voltage_arr);
		
		//���̹���
			sprintf(powerline_arr,"%.2f %s",scap_current*cap_voltage,w_ch);
			powerline_color=(chassis_power_real<=chassis_power_limit)*UI_Color_Green+(chassis_power_real>chassis_power_limit)*UI_Color_Orange;//û��Ϊ�̣�����Ϊ��	���̹���
			Line_Draw(&G14,"085",UI_Graph_Change,8,powerline_color,15,960-300,10,960-300+(chassis_power_real/chassis_power_limit)*300,10);
			Char_Draw(&POWERLINE,"086",UI_Graph_Change,8 ,powerline_color,18,10,4,960+305,20,powerline_arr);
				
		//С�����ж�
			if(tRC_Data.switch_right==1||Small_gyroscope_flag==1)		
			{
				if(rank==1)										//1					С����
				{
					sprintf(sgyd_arr,"%d",1);
	
					Char_Draw(&DATA_SGY_VOLTAGE,"073",UI_Graph_Change,7 ,UI_Color_Green,18,10,4,170,590,sgyd_arr);
//					Circle_Draw(&G16,"073",UI_Graph_Change,7,UI_Color_Green,13,170,590,15);
				}
				else if(rank==2)										//2					С����
				{
				sprintf(sgyd_arr,"%d",2);
	
					Char_Draw(&DATA_SGY_VOLTAGE,"073",UI_Graph_Change,7 ,UI_Color_Yellow,18,10,4,170,590,sgyd_arr);
				}
				else if(rank==3)										//3					С����
				{
			sprintf(sgyd_arr,"%d",3);
	
					Char_Draw(&DATA_SGY_VOLTAGE,"073",UI_Graph_Change,7 ,UI_Color_Orange,18,10,4,170,590,sgyd_arr);
				}
			}
				else																																		//�ر�Ϊ��						С����
				{
					sprintf(sgyd_arr,"%d",0);
					Char_Draw(&DATA_SGY_VOLTAGE,"073",UI_Graph_Change,7 ,UI_Color_Orange,18,10,4,170,590,sgyd_arr);
				}
		//Ħ�����ж�
		//#define highfeedmotorspeed 2 ,#define lowfeedmotorspeed  1 ,#define rcfeedmotorspeed   3 ,#define feedmotor_off      0
				switch(feedmotor_id)
				{
					case lowfeedmotorspeed:																							//����Ϊ��						Ħ����
					{
						Circle_Draw(&G15,"071",UI_Graph_Change,7,UI_Color_Green,13,170,640,15);
						break;
					}
					case highfeedmotorspeed:																						//����Ϊ�� 				Ħ����
					{
						Circle_Draw(&G15,"071",UI_Graph_Change,7,UI_Color_Orange,13,170,640,15);
						break;
					}
					case rcfeedmotorspeed: 																							//ң��������Ϊ�� 		Ħ����
					{
						Circle_Draw(&G15,"071",UI_Graph_Change,7,UI_Color_Cyan,13,170,640,15);
						break;
					}
					case feedmotor_off:																									//�ر�Ϊ��						Ħ����
					{
						Circle_Draw(&G15,"071",UI_Graph_Change,7,UI_Color_Black,13,170,640,15);
						break;
					}
					
				}
				Line_Draw(&G13,"084",UI_Graph_Change,8,UI_Color_Main,20,960-305,10,960+305,10);
				UI_ReFresh(5,G15,G17,G16,G13,G14);
				Char_ReFresh(DATA_CAP_VOLTAGE);
				Char_ReFresh(CH_CAP_V);
				Char_ReFresh(CH_SGY);
				Char_ReFresh(CH_SHOOT);
				Char_ReFresh(DATA_SGY_VOLTAGE);
				Char_ReFresh(POWERLINE);

}
void float2char(float f,char str[30])
{
	sprintf(str,"%.2f",f);
}
