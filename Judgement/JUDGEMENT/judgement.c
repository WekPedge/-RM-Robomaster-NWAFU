#include "judgement.h"
#include "proticol.h"
#include "string.h"
#include "main.h"
extern uint8_t RxBuf0[1024],RxBuf1[1024];
extern uint32_t Uart7DMABufferSize;
unpack_data_t judge_unpack_obj;
receive_judge_t judge_rece_mesg;
u8 bloodstage;
uint8_t energy_flag=0;
extern u8 IMU_count_flag;
extern float energy;
extern float maxenergy;
extern int shootflag;

//extern uint8_t client_id;


float lastenergy=0;
float chassis_power_real;
float chassis_power_limit;
int hurt_id;
uint8_t robot_id;
uint8_t shoot_state;
uint8_t power_limit;
uint16_t bullet_remaining_num;
extern int rank;
uint8_t dma_current_memory_target(DMA_Stream_TypeDef *dma_stream)
{
  uint8_t tmp = 0;

  /* Get the current memory target */
  if ((dma_stream->CR & DMA_SxCR_CT) != 0)
  {
    /* Current memory buffer used is Memory 1 */
    tmp = 1;
  }
  else
  {
    /* Current memory buffer used is Memory 0 */
    tmp = 0;
  }
  return tmp;
}
uint16_t remain_data_counter_temp=0;
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  /* Return the number of remaining data units for DMAy Streamx */
  return ((uint16_t)(dma_stream->NDTR));
}
void get_dma_memory_msg(DMA_Stream_TypeDef *dma_stream, uint8_t *mem_id, uint16_t *remain_cnt)
{
  *mem_id     = dma_current_memory_target(dma_stream);
  *remain_cnt =  dma_current_data_counter(dma_stream);
}
#define JUDGE_FIFO_BUFLEN 500
//for debug
int dma_write_len = 0;
int fifo_overflow = 0;



uart_dma_rxdata_t judge_rx_obj;   

fifo_s_t  judge_rxdata_fifo;    //裁判系统接收队列
uint8_t   judge_rxdata_fifo_buffer[JUDGE_FIFO_BUFLEN];  //裁判系统接受数据存放点
void Judge_uart_init(void)
{
	
  fifo_s_init(&judge_rxdata_fifo, judge_rxdata_fifo_buffer, JUDGE_FIFO_BUFLEN);
  judge_rx_obj.data_fifo = &judge_rxdata_fifo;
  judge_rx_obj.buff_size = Uart7DMABufferSize;
  judge_rx_obj.buff[0] = RxBuf0;  //DMA第一个缓冲区首地址
  judge_rx_obj.buff[1] = RxBuf1;
	
  /* initial judge data unpack object */
  judge_unpack_obj.data_fifo = &judge_rxdata_fifo;
  judge_unpack_obj.p_header = (frame_header_t *)judge_unpack_obj.protocol_packet;
  judge_unpack_obj.index = 0;
  judge_unpack_obj.data_len = 0;
  judge_unpack_obj.unpack_step = STEP_HEADER_SOF;
}
  int16_t  tmp_len;
	uint8_t  current_memory_id;
void data_process1(void)
{
//  int16_t  tmp_len;
//	uint8_t  current_memory_id;
  uint16_t remain_data_counter;
	uint8_t  *pdata = RxBuf0;
  get_dma_memory_msg(DMA1_Stream3, &current_memory_id, &remain_data_counter);
	    if (current_memory_id)
    {
      judge_rx_obj.write_index = Uart7DMABufferSize*2 - remain_data_counter;    //缓冲区2
    }
    else
    {
      judge_rx_obj.write_index =Uart7DMABufferSize - remain_data_counter;     //缓冲区1
    }
		
		  if (judge_rx_obj.write_index < judge_rx_obj.read_index)     //读指针在写指针前面 分两次入队
  {
    dma_write_len = Uart7DMABufferSize*2 - judge_rx_obj.read_index + judge_rx_obj.write_index;   //此次DMA接收数据大小
    
    tmp_len = Uart7DMABufferSize*2 - judge_rx_obj.read_index;
    if (tmp_len != fifo_s_puts(&judge_rxdata_fifo, &pdata[judge_rx_obj.read_index], tmp_len))   //入队
      fifo_overflow = 1;
    else
      fifo_overflow = 0;
    judge_rx_obj.read_index = 0;
    
    tmp_len = judge_rx_obj.write_index;
    if (tmp_len != fifo_s_puts(&judge_rxdata_fifo, &pdata[judge_rx_obj.read_index], tmp_len))
      fifo_overflow = 1;
    else
      fifo_overflow = 0;
    judge_rx_obj.read_index = judge_rx_obj.write_index;
  }
  else                                           
  {
    dma_write_len = judge_rx_obj.write_index - judge_rx_obj.read_index;
    
    tmp_len = judge_rx_obj.write_index - judge_rx_obj.read_index;
    if (tmp_len != fifo_s_puts(&judge_rxdata_fifo, &pdata[judge_rx_obj.read_index], tmp_len))
      fifo_overflow = 1;
    else
      fifo_overflow = 0;
    judge_rx_obj.read_index = (judge_rx_obj.write_index) % (Uart7DMABufferSize*2);
  }
}
#define PROTOCAL_FRAME_MAX_SIZE  200
//uint8_t        protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
uint16_t       index1;
uint8_t 			byte1;
void data_process2(unpack_data_t *p_obj, uint8_t sof)
{
	uint8_t byte = 0;
	index1=0;
   while ( fifo_used_count(p_obj->data_fifo) )  //返回队列中的数据
	 {
	  byte = fifo_s_get(p_obj->data_fifo);   //出队
		 byte1=byte;
//		protocol_packet[index1++] = byte;
//		     byte = fifo_s_get(p_obj->data_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
    
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == HEADER_LEN)
        {
          if ( verify_crc8_check_sum(p_obj->protocol_packet, HEADER_LEN) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  

      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_crc16_check_sum(p_obj->protocol_packet, HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN) )
          {
            if (sof == UP_REG_ID)
            {
              //pc_data_handler(p_obj->protocol_packet);
            }
            else  //DN_REG_ID
            {
              judgement_data_handler(p_obj->protocol_packet);
            }
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
	 }

void judgement_data_handler(uint8_t *p_frame)//裁判系统接收数据处理
{
  frame_header_t *p_header = (frame_header_t*)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;    //????????????
  uint8_t  invalid_cmd = 0;
  
  switch (cmd_id)
  {
    case GAME_RESULT_ID:
      memcpy(&judge_rece_mesg.game_result_data, data_addr, data_length);
    break;
		
    case GAME_STATE_ID:
      memcpy(&judge_rece_mesg.robot_state_data, data_addr, data_length);
		  bloodstage=judge_rece_mesg.robot_state_data.remain_HP;
			robot_id=judge_rece_mesg.robot_state_data.robot_id;
			rank=judge_rece_mesg.robot_state_data.robot_level;
			maxenergy=judge_rece_mesg.robot_state_data.shooter_id1_17mm_cooling_limit;;
			shoot_state=judge_rece_mesg.robot_state_data.mains_power_shooter_output;
			power_limit=judge_rece_mesg.robot_state_data.chassis_power_limit;
		  if(judge_rece_mesg.robot_state_data.remain_HP==0)
			{
				IMU_count_flag=0;
			}
    break;

    case POWER_HEAT_DATA_ID:
      memcpy(&judge_rece_mesg.power_heat_data, data_addr, data_length);
			energy=judge_rece_mesg.power_heat_data.shooter_heat0;
			chassis_power_real=judge_rece_mesg.power_heat_data.chassis_power;
			chassis_power_limit=judge_rece_mesg.power_heat_data.chassis_power_buffer;
		
    break;
		
    case GAME_POS_ID:
      memcpy(&judge_rece_mesg.robot_pos_data, data_addr, data_length);
    break;

    case GAME_BUFF_ID:
      memcpy(&judge_rece_mesg.buff_musk_data, data_addr, data_length);
    break; 
    		
    case GAME_HURT_ID:
      memcpy(&judge_rece_mesg.robot_hurt_data, data_addr, data_length);
	hurt_id=judge_rece_mesg.robot_hurt_data.armor_id;
    break;
		
		case GAME_SHOOT_DATA_ID:
      memcpy(&judge_rece_mesg.real_shoot_data, data_addr, data_length);
    break;
		case GAME_REMAIN_DATA_ID:
      memcpy(&judge_rece_mesg.bullet_remain_data, data_addr, data_length);
			bullet_remaining_num=judge_rece_mesg.bullet_remain_data.bullet_remaining_num_17mm;
    break;
    default:
      invalid_cmd = 1;
    break;
  }
		
  /* valid forward data */

}
