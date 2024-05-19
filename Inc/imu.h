#ifndef __IMU_H
#define __IMU_H

#include "sys.h"

#define M_PI  (float)3.1415926535

typedef __packed struct
{
    int16_t     MagXOffset;
    int16_t     MagYOffset;
    int16_t     MagZOffset;
    float       MagXScale;
    float       MagYScale;
    float       MagZScale;    
    uint8_t     MagCaliFlag;
}MagCaliStruct_t;

extern float arrmx,arrmy,arrmz;
extern MagCaliStruct_t MagSavedCaliData;	

void IST_Eliminate_Deviation(uint32_t mx,uint32_t my,uint32_t mz);
void GET_Last_Value(void);
uint32_t Get_Time_Micros(void);
void Init_Quaternion(void);//根据测量数据，初始化q0,q1,q2.q3，从而加快收敛速度
void IMU_getYawPitchRoll(volatile float * angles);
void GetPitchYawGxGyGz(float * angles);


#endif

