/**
  *@file test_imu.h
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */
  
#ifndef _TEST__IMU_H
#define _TEST__IMU_H

#include "stm32f4xx_hal.h"
#include "main.h"

#define MPU6500_NSS_Low() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU6500_NSS_High() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  
  int16_t temp;
  
  int16_t gx;
  int16_t gy;
  int16_t gz;
  
  int16_t mx;
  int16_t my;
  int16_t mz;
}IMUDataTypedef;

typedef struct
{
  float ax;
  float ay;
  float az;
  
  int16_t temp;
  
  float gx;
  float gy;
  float gz;
  
  int16_t mx;
  int16_t my;
  int16_t mz;
}IMUDatareal;

typedef struct
{
  double angle_yaw1;
	double angle_yaw2;
 // float ay;
  //float az;
  
 // int16_t temp;
  
 // float gx;
 // float gy;
 // float gz;
  
 // int16_t mx;
 // int16_t my;
 // int16_t mz;
}IMUData;

typedef __packed struct
{
	int16_t MaxMagX;
	int16_t MaxMagY;
	int16_t MaxMagZ;
	int16_t MinMagX;
	int16_t MinMagY;
	int16_t MinMagZ;
}MagMaxMinData_t;
typedef struct{
	float X;
	float Y;
	float Z;
}S_float_XYZ;

extern MagMaxMinData_t MagMaxMinData;
extern uint32_t arrx,arry,arrz;
extern uint8_t MPU_id;
extern IMUDataTypedef imu_data;
extern IMUData Gimbal;
extern IMUDatareal FLOAT_date;
uint8_t MPU6500_Init(void);
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data);
uint8_t MPU6500_Read_Reg(uint8_t const reg);
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len);
void IMU_Get_Data(void);
void IST8310_Get_Data(void);
uint8_t IST8310_Init(void);
void update(int16_t x,int16_t y,int16_t z);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
extern S_float_XYZ	Euler_Angle;		//四元数计算出的欧拉角
float Get_Yaw_Angle(int16_t GyroscopeRawDate,int16_t nn);
#endif

