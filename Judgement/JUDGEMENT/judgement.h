#ifndef __JUDGEMENT_H
#define __JUDGEMENT_H

#include "data_fifo.h"
#define UP_REG_ID    0xA0  //上位机通信
#define DN_REG_ID    0xA5  //裁判系统通信
#define HEADER_LEN   sizeof(frame_header_t)
#define CMD_LEN      2    //命令帧
#define CRC_LEN      2    //CRC16校验

#define PROTOCAL_FRAME_MAX_SIZE  200
/** 
  * @brief  帧头定义
  */
typedef __packed struct
{
  uint8_t  sof;							//0xA0为与上位机通信，0xA5为与下位机通信
  uint16_t data_length;			//数据段长度
  uint8_t  seq;							//包序号
  uint8_t  crc8;						//帧头校验
} frame_header_t;

/** 
  * @brief  解包步骤定义
  */
typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  DMA_Stream_TypeDef *dma_stream;
  fifo_s_t           *data_fifo;
  uint16_t           buff_size;
  uint8_t            *buff[2];
  uint16_t           read_index;
  uint16_t           write_index;
} uart_dma_rxdata_t;

typedef struct
{
  fifo_s_t       *data_fifo;
  frame_header_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

typedef enum
{
//从裁判系统接收
  GAME_INFO_ID       = 0x0001,		//比赛信息，10Hz频率
  GAME_RESULT_ID     = 0x0002,		//比赛结果数据，结束时发送一次
	GAME_SURVIVOR_ID   = 0x0003,    //比赛机器人存活数据，1Hz发送一次
	EVENT_DATA_ID      = 0x0101,    //场地事件数据，事件改变后发送
	SUPPLY_ACTION_ID   = 0x0102,    //场地补给站动作标识数据，动作改变后发送
	// SUPPLY_BOOKING_ID  = 0x0103,    //对抗赛未开发，请求补给站补单数据，由参赛队发送
	GAME_STATE_ID      = 0x0201,    //机器人状态数据
	POWER_HEAT_DATA_ID = 0x0202,		//实时功率热量数据，50hz发送
	GAME_POS_ID        = 0x0203,    //机器人位置数据，10hz发送
	GAME_BUFF_ID       = 0x0204,    //机器人增益数据，增益状态改变后发送
	GAME_ENERGY_ID     = 0x0205,    //空中机器人能量状态数据，10hz发送，只有无人机主控发送
	GAME_HURT_ID       = 0x0206,    //伤害状态数据，伤害发生后发送
	GAME_SHOOT_DATA_ID = 0x0207,    //实时射击数据，子弹发射后发送
	GAME_REMAIN_DATA_ID = 0x0208,   //子弹剩余发射数
} judge_data_id_e;

typedef __packed struct   
{
  uint8_t game_type : 4;
  uint8_t game_progress : 4;
  uint16_t stage_remain_time;
}game_state_t;

typedef __packed struct   
{
  uint8_t winner;
}game_result_t;

typedef __packed struct   
{
  uint16_t robot_legion;
}game_robot_survivors_t;

typedef __packed struct   
{
  uint32_t event_type;
}event_data_t;

typedef __packed struct   
{
 uint8_t supply_projectile_id; 
 uint8_t supply_robot_id; 
 uint8_t supply_projectile_step; 
 uint8_t supply_projectile_num;
}supply_projectile_action_t;

typedef __packed struct  
{
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id; 
uint8_t supply_num;
}supply_projectile_booking_t;

typedef __packed struct   //数据丢失处
{
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t remain_HP;
  uint16_t max_HP;
	
	uint16_t shooter_id1_17mm_cooling_rate;
	uint16_t shooter_id1_17mm_cooling_limit;

	uint16_t chassis_power_limit;

  uint8_t mains_power_gimbal_output : 1;
  uint8_t mains_power_chassis_output : 1;
  uint8_t mains_power_shooter_output : 1;
}game_robot_state_t;

typedef __packed struct    
{
  uint16_t chassis_volt; 
  uint16_t chassis_current; 
  float chassis_power; 
  uint16_t chassis_power_buffer; 
  uint16_t shooter_heat0; 
  uint16_t shooter_heat1; 
}power_heat_data_t;

typedef __packed struct  
{
 float x;
  float y;
  float z;
  float yaw;
}game_robot_pos_t;

typedef __packed struct  
{
  uint8_t power_rune_buff;
}buff_musk_t;

typedef __packed struct   
{
  uint8_t energy_point;
  uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct   
{
  uint8_t armor_id : 4;
  uint8_t hurt_type : 4;
} robot_hurt_t;

typedef __packed struct  
{
  uint8_t bullet_type;
  uint8_t bullet_freq;
  float bullet_speed;
} shoot_data_t;
/** 
  * @brief  student custom data
  */
typedef __packed struct
{
  float data1;
  float data2;
  float data3;
	uint8_t data4;
} client_show_data_t;

typedef __packed struct
{
  uint8_t  data[64];
} user_to_server_t;

typedef __packed struct
{
  uint8_t  data[32];
} server_to_user_t;

typedef __packed struct
{
 uint16_t bullet_remaining_num_17mm;
uint16_t bullet_remaining_num_42mm;
uint16_t coin_remaining_num;
} bullet_remaining_t;

typedef struct
{
	game_state_t                game_stage_data;                //0x0001
	game_result_t               game_result_data;               //0x0002
	game_robot_survivors_t      game_robot_survivors_data;      //0x0003
	event_data_t                gain_event_data;                //0x0101
	supply_projectile_action_t  supply_projectile_action_data;  //0x0102
	supply_projectile_booking_t supply_projectile_booking_data; //0x0103
	game_robot_state_t          robot_state_data;								//0x0201
	power_heat_data_t           power_heat_data;								//0x0202
	game_robot_pos_t            robot_pos_data;									//0x0203
	buff_musk_t                 buff_musk_data;									//0x0204
	aerial_robot_energy_t       aerial_robot_energy_data;				//0x0205
	robot_hurt_t                robot_hurt_data;								//0x0206
	shoot_data_t                real_shoot_data;								//0x0207
	bullet_remaining_t					bullet_remain_data;							//0x0208

} receive_judge_t;



void data_process1(void);
void data_process2(unpack_data_t *p_obj,uint8_t sof);
void judgement_data_handler(uint8_t *p_frame);
void dma_buffer_to_unpack_buffer(uart_dma_rxdata_t *dma_obj);
void data_process1(void);
void Judge_uart_init(void);
#endif

