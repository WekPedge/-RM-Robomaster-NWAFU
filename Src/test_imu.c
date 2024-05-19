/**
  *@file test_imu.c
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */
  

#include "test_imu.h"
#include "mpu6500_reg.h"
#include "IST8310_reg.h"
#include "spi.h"
#include "math.h"


float ax,ay,az,gx,gy,gz;
uint8_t MPU_id = 0;
uint16_t timenow,timelast;
IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};
IMUDatareal FLOAT_date={0};
IMUData Gimbal={0.0};
IMUDataTypedef imu_data_offest = {0,0,0,0,0,0,0,0,0,0};
uint32_t IST[3][11];//[0]-[9]为最近10次数据 [10]为10次数据的平均值 注：磁传感器的采样频率慢，所以单独列出
int16_t MPU6050_FIFO[6][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值
//int16_t HMC5883_FIFO[3][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值 注：磁传感器的采样频率慢，所以单独列出
uint32_t arrx,arry,arrz;
extern uint8_t feedmotor_flag;
MagMaxMinData_t MagMaxMinData;
/*************寄存器读写************/
//Write a register to MPU6500
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg&0x7f;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  MPU_Tx = data;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();
  return 0;
}

//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg|0x80;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();
  return MPU_Rx;
}

//Read registers from MPU6500,address begin with regAddr
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
  static uint8_t MPU_Rx, MPU_Tx, MPU_Tx_buff[14] = {0xff};
  MPU6500_NSS_Low();
  
  MPU_Tx = regAddr|0x80;
  MPU_Tx_buff[0] = MPU_Tx;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi5, MPU_Tx_buff, pData, len, 55);
  
  MPU6500_NSS_High();
  return 0;
}

//Write IST8310 register through MPU6500
static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x080 | 0x01);
  HAL_Delay(10);
}

//Write IST8310 register through MPU6500
static uint8_t IST_Reg_Read_By_MPU(uint8_t addr)
{
  uint8_t data;
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, addr);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  HAL_Delay(10);
  data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  HAL_Delay(10);
  return data;
}

//Initialize the MPU6500 I2C Slave0 for I2C reading
static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x03);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  HAL_Delay(6);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  HAL_Delay(7);
}
/*************寄存器读写************/
//Initialize the IST8310
uint8_t IST8310_Init(void)
{
  MPU6500_Write_Reg(MPU6500_USER_CTRL, 0x30);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d);
  HAL_Delay(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01);
  if(IST8310_DEVICE_ID_A != IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
    return 1; //error
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) != 0x00)
    return 2;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) != 0x00)
    return 3;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
  if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) != 0x24)
    return 4;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
  if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) != 0xc0)
    return 5;
  HAL_Delay(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  HAL_Delay(10);
  
  MPU_Auto_Read_IST_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  HAL_Delay(100);
  return 0;
}

//Set the accelerated velocity resolution
uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr<<3);
}

//Set the angular velocity resolution
uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr)        //fsr=0：250 1：500 。。。
{
  return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr<<3);   // 0x10=10000 再右移3位 =00010=2
}
//Get 6 axis data from MPU6500
void IST8310_Get_Data()//获取地磁计的数据
{
  uint8_t mpu_buff[20];
 // MPU6500_Read_Regs(IST8310_AVGCNTL, &mpu_buff[14], 6);
  //MPU6500_Read_Regs(IST8310_ADDRESS, mpu_buff, 6);
	
//	mpu_buff[14]=MPU6500_Read_Regs(IST8310_R_XL,mpu_buff,8);
//  mpu_buff[15]=MPU6500_Read_Regs(IST8310_R_XM ,mpu_buff,8); 	
//  imu_data.mx = mpu_buff[14]<<8 |mpu_buff[15];
//	
//	mpu_buff[16]=MPU6500_Read_Regs(IST8310_R_YL,mpu_buff,8);
//  mpu_buff[17]=MPU6500_Read_Regs(IST8310_R_YM ,mpu_buff,8); 	
//  imu_data.my = mpu_buff[16]<<8 |mpu_buff[17];
//	
//	mpu_buff[18]=MPU6500_Read_Regs(IST8310_R_ZL,mpu_buff,8);
//  mpu_buff[19]=MPU6500_Read_Regs(IST8310_R_ZM ,mpu_buff,8); 	
//  imu_data.mz = mpu_buff[18]<<8 |mpu_buff[19];
	
	MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H|0x80, mpu_buff, 20);
	
		mpu_buff[14]=MPU6500_Read_Reg(IST8310_R_XL);
  mpu_buff[15]=MPU6500_Read_Reg(IST8310_R_XM ); 	
  imu_data.mx = mpu_buff[14]<<8 |mpu_buff[15];
	
	mpu_buff[16]=MPU6500_Read_Reg(IST8310_R_YL);
  mpu_buff[17]=MPU6500_Read_Reg(IST8310_R_YM ); 	
  imu_data.my = mpu_buff[16]<<8 |mpu_buff[17];
  
	mpu_buff[18]=MPU6500_Read_Reg(IST8310_R_ZL);
  mpu_buff[19]=MPU6500_Read_Reg(IST8310_R_ZM ); 	
  imu_data.mz = mpu_buff[18]<<8 |mpu_buff[19];
	
	
	update(imu_data.mx,imu_data.my,imu_data.mz);
//  imu_data.mx = mpu_buff[14]<<8 |mpu_buff[15];
//  imu_data.my = mpu_buff[16]<<8 |mpu_buff[17];
//  imu_data.mz = mpu_buff[18]<<8 |mpu_buff[19];
//  update(imu_data.mx,imu_data.my,imu_data.mz);

 /* imu_data.temp = mpu_buff[6]<<8 |mpu_buff[7];
  
  imu_data.gx = mpu_buff[8]<<8 |mpu_buff[9] - imu_data_offest.gx;
  imu_data.gy = mpu_buff[10]<<8 |mpu_buff[11] - imu_data_offest.gy;
  imu_data.gz = mpu_buff[12]<<8 |mpu_buff[13] - imu_data_offest.gz;*/
  //printf ("22");
  
}
//Get 6 axis data from MPU6500

void update(int16_t x,int16_t y,int16_t z)//地磁计的中值滤波
{
		uint8_t i = 0;
	int32_t sum=0;

	for(i=1;i<10;i++)
	{
		IST[0][i-1]=IST[0][i];
		IST[1][i-1]=IST[1][i];
		IST[2][i-1]=IST[2][i];
	}
	IST[0][9]= x;//将新的数据放置到 数据的最后面
	IST[1][9]= y;
	IST[2][9]= z;
	
	for(i=0;i<10;i++)//求当前数组的合，再取平均值
	{	
		 sum+=IST[0][i];
	}
	IST[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=IST[1][i];
	}
	IST[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=IST[2][i];
	}
	IST[2][10]=sum/10;
	arrx=IST[0][10];
	arry=IST[1][10];
	arrz=IST[2][10];
	//printf (" %d  %d  %d ",arrx,arry,arrz);
	//以上全部为未校准数据
	/************用于偏差校正，见官方开源，暂时无用的函数*******************/
	if(MagMaxMinData.MinMagX>IST[0][10])
	{
		MagMaxMinData.MinMagX=(int16_t)IST[0][10];
	}
	if(MagMaxMinData.MinMagY>IST[1][10])
	{
		MagMaxMinData.MinMagY=(int16_t)IST[1][10];
	}
	if(MagMaxMinData.MinMagZ>IST[2][10])
	{
		MagMaxMinData.MinMagZ=(int16_t)IST[2][10];
	}

	if(MagMaxMinData.MaxMagX<IST[0][10])
	{
		MagMaxMinData.MaxMagX=(int16_t)IST[0][10];		
	}
	if(MagMaxMinData.MaxMagY<IST[1][10])
	{
		MagMaxMinData.MaxMagY = IST[1][10];
	}
	if(MagMaxMinData.MaxMagZ<IST[2][10])
	{
		MagMaxMinData.MaxMagZ=(int16_t)IST[2][10];
	}
	/************无用的函数*******************/
 //printf (" %d  %d  %d      %d  %d  %d\n",MagMaxMinData.MinMagX,MagMaxMinData.MinMagY,MagMaxMinData.MinMagZ   ,MagMaxMinData.MaxMagX,MagMaxMinData.MaxMagY,MagMaxMinData.MaxMagZ);	
}

void MPU6050_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz) //mpu均值滤波[0]-[9]为最近10次数据 [10]为10次数据的平均值
{
	uint8_t i = 0;
	int32_t sum=0;
	
	for(i=1;i<10;i++)
	{
		MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
		MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
		MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
		MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
		MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
		MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
	}
	
	MPU6050_FIFO[0][9]=ax;//将新的数据放置到 数据的最后面
	MPU6050_FIFO[1][9]=ay;
	MPU6050_FIFO[2][9]=az;
	MPU6050_FIFO[3][9]=gx;
	MPU6050_FIFO[4][9]=gy;
	MPU6050_FIFO[5][9]=gz;
	
	for(i=0;i<10;i++)//求当前数组的合，再取平均值
	{	
		 sum+=MPU6050_FIFO[0][i];
	}
	MPU6050_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[1][i];
	}
	MPU6050_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[2][i];
	}
	MPU6050_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[3][i];
	}
	MPU6050_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[4][i];
	}
	MPU6050_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[5][10]=sum/10;
	
}
//int16_t MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz
//				,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz;
//int32_t MPU6050_FIFO[2][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值	
//#define		IMU_ 57.5//60.4//56//57.5// 57
int gz_cor=-50;//-95
int cor_flag=0;
void IMU_Get_Data()
{
  uint8_t mpu_buff[14];
	//
	if(cor_flag==0)
	{
	if(feedmotor_flag==1) gz_cor=gz_cor-1;
		cor_flag=1;
	}
//if(timenow-timelast )
  MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
  
  imu_data.ax = mpu_buff[0]<<8 |mpu_buff[1];
  imu_data.ay = mpu_buff[2]<<8 |mpu_buff[3];
  imu_data.az = mpu_buff[4]<<8 |mpu_buff[5];
  
  imu_data.temp = mpu_buff[6]<<8 |mpu_buff[7];
  
  imu_data.gx = mpu_buff[8]<<8 |mpu_buff[9] - imu_data_offest.gx;     //减去偏离值？ 
  imu_data.gy = mpu_buff[10]<<8 |mpu_buff[11] - imu_data_offest.gy;
  imu_data.gz = mpu_buff[12]<<8 |mpu_buff[13] - imu_data_offest.gz;
  //printf ("22");
	//printf ("%d %d %d \n",imu_data.ax,imu_data.ay,imu_data.az);
	imu_data.ax=((float) imu_data.ax) ;    //为什么这里进行IMU数据转换？
	imu_data.ay=((float) imu_data.ay) ;
	imu_data.az=((float) imu_data.az) ;
	ax=(imu_data.ax&0xffd0);    //这是什么意思？
	ay=(imu_data.ay&0xffd0);
	az=(imu_data.az&0xffd0);
	gx=Get_Yaw_Angle((imu_data.gx-40),10);
	gy=Get_Yaw_Angle((imu_data.gy-35),4);
	gz=Get_Yaw_Angle((imu_data.gz+gz_cor),20);


	MPU6050_DataSave(ax,ay,az,gx,gy,gz);


	FLOAT_date.ax=MPU6050_FIFO[0][10];
	FLOAT_date.ay=MPU6050_FIFO[1][10];
	FLOAT_date.az=MPU6050_FIFO[2][10];
	FLOAT_date.gy = MPU6050_FIFO[4][10]/32.8f; //转成度每秒
//	if(Calibration_Mode==IMU_Cclibration)
//		printf (" yaw_raw_date(AFTER Mean filter):%f  ",FLOAT_date.gy);
//	else;
	FLOAT_date.gx = MPU6050_FIFO[3][10]/32.8f; //转成度每秒
	//printf ("%f \n",FLOAT_date.gx);
	FLOAT_date.gz = MPU6050_FIFO[5][10]/32.8f; //转成度每秒
	//printf ("%f \n",FLOAT_date.gz);
}
int16_t Error_Date[80] = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
										10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
										20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
										30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 
										40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
										50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
										60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
										70, 71, 72, 73, 74, 75, 76, 77, 78, 79}; 
int16_t Error_Date1[80]={-1,-2,-3,-4,-5,-6,-7,-8,-9,-10,-11,-12
-13,-14,-15,-16,-17,-18,-19,-20,-21,-22,-23,-24,-25,-26,-27,-28
-29,-30,-31,-32,-33,-34,-35,-36,-37,-38,-39,-40,-41,-42,-43,-44,-45,-46,-47,-48,-49,-50,-51,-52,-53,-54,-55,-56,-57,-58,-59,-60,-61,-62,-63,-64,
-65,-66,-67,-68,-69,-70,-71,-72,-73,-74,-75,-76,-77,-78,-79};
/*---------------------------------------------------------
输入数据:经过均值滤波的陀螺仪角速度值
函数功能:除去陀螺仪角速度静止状态的偏差值，使其积分不会偏移	
				 常用于校准yaw角速度值
输出数据:经过矫正的角速度值	
										
很简单效果也很明显~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~非常的nice！
妈妈再也不用担心积分后yaw角度会漂移了  (*^__^*) 						
----------------------------------------------------------*/		
//#define		IMU  18//6//9//18   //反向						
float Get_Yaw_Angle(int16_t GyroscopeRawDate,int16_t nn)
{
		uint8_t i;	
		int16_t error_date=0;		//偏差值
		int16_t Correct_Date=0;	//修正值
		float Correct_Date_Output=0;//修正输出值
	//printf ("%d ",GyroscopeRawDate);
		for(i = 0; i < nn; i++)
		{
				if(GyroscopeRawDate ==  Error_Date[i])
				{
						error_date = Error_Date[i];
					//printf ("%d \n",error_date);
						break;
				}
		}
			for(i = 0; i < nn; i++)
		{
				if(GyroscopeRawDate ==  Error_Date1[i])
				{
						error_date = Error_Date1[i];
					//printf ("%d \n",error_date);
						break;
				}
		}
//printf (" 11error_date%d  ",error_date);		
		Correct_Date = GyroscopeRawDate - error_date;
		//printf ("%d \n",GyroscopeRawDate);
		//printf (" error_date%d  ",error_date);
		//printf (" Correct_Date%d  ",Correct_Date);
		if(Correct_Date == 0)		Correct_Date_Output = Correct_Date;
		if(Correct_Date < 0)		Correct_Date_Output=(float) Correct_Date;
		if(Correct_Date > 0)		Correct_Date_Output=(float) Correct_Date ;//Correct_Date_Output =(float) Correct_Date / 32.8f; //转成度每秒
	if(Calibration_Mode==IMU_Cclibration)
		printf (" Error elimination YAW:%f  ",Correct_Date_Output);
	else;
		return Correct_Date_Output;
}


//Initialize the MPU6500
uint8_t MPU6500_Init(void)
{
  uint8_t index = 0;
  uint8_t MPU6500_Init_Data[10][2] =   {
//    {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
//    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
//    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro
//    {MPU6500_CONFIG,        0x03},      // LPF 98Hz
//    {MPU6500_GYRO_CONFIG,   0x10},      // +-2000dps
//    {MPU6500_ACCEL_CONFIG,  0x00},      // +-8G
//    {MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
//    {MPU6500_USER_CTRL,     0x00},      // Enable AUX
	  
//    {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
//    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
//    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro
//    {MPU6500_CONFIG,        0x02},      // LPF 98Hz
//    {MPU6500_GYRO_CONFIG,   0x18},      // +-2000dps
//    {MPU6500_ACCEL_CONFIG,  0x10},      // +-8G
//    {MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
//    {MPU6500_USER_CTRL,     0x20},      // Enable AUX
	  
	  {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device复位MPU
    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z 以陀螺仪的Z轴为参考方向
    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro 唤醒MPU
    {MPU6500_CONFIG,        0x02},      // LPF 98Hz 低通滤波器为98HZ
    {MPU6500_GYRO_CONFIG,   0x10},      // +-1000dps
    {MPU6500_ACCEL_CONFIG,  0x10},      // +-8G  
    {MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
    {MPU6500_USER_CTRL,     0x20},      // Enable AUX 使能IIC主模式
  };
  
  HAL_Delay(100);
  MPU_id = MPU6500_Read_Reg(MPU6500_WHO_AM_I);  //read id of device,check if MPU6500 or not
  
  for(index = 0; index < 10; index++)    //把这些配置写进寄存器里面
  {
    MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
    HAL_Delay(1);
  }

  return 0;
}


//#define kp 10.0f		//比例增益控制加速度计的收敛速度
//#define ki	0.008f		//积分增益控制陀螺仪偏差的收敛速度
//#define halfT 0.001f	//周期的一半
//#define RTA		57.295780	//弧度到角度需要乘的系数(180/3.14)
//#define	ATR		0.0174533	//角度到弧度需要乘的系数(3.14/180)
//S_float_XYZ	Euler_Angle;		//四元数计算出的欧拉角
//float q0 = 1,q1 = 0,q2 = 0,q3 = 0;		//四元数元素代表估计取向，四元素初始值
//float exInt = 0, eyInt = 0,ezInt = 0;	//比例积分误差，初始值误差
///*------四元数解析-----*/
///*
//*	ax,ay,az为在XYZ轴上的加速度
//*	gx,gy,gz为在XYZ轴上的陀螺仪
//*/
//void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
//{
//	float norm;				//归一化变量
//	
//	float vx,vy,vz;
//	float ex,ey,ez;
//	
///*---计算平方项，节省运算时间- 其实鄙人觉得这一步并没有什么卵用，但由于参考别人的四元数算法，鄙人也没怎么搞太懂---*/
//	float q0q0 = q0 * q0;
//	float q0q1 = q0 * q1;
//	float q0q2 = q0 * q2;
//	
//	float q1q1 = q1 * q1;
//	float q1q3 = q1 * q3;
//	
//	float q2q2 = q2 * q2;
//	float q2q3 = q2 * q3;
//	
//	float q3q3 = q3 * q3;
//	
//	if(ax*ay*az == 0)
//			return;
//	
//	norm = sqrt((ax * ax) + (ay * ay) + (az *az));		//加速度数据归一化
///*----计算 规范后的加速度-----*/
//	ax = ax / norm;			 
//	ay = ay / norm;
//	az = az / norm;
//	
///*-----估计重力方向和流量/变迁----*/
//	vx = 2*(q1q3 - q0q2);		////四元素中xyz的表示			
//	vy = 2*(q0q1 + q2q3);
//	vz = q0q0 - q1q1 - q2q2 + q3q3 ;

//	ex = (ay*vz - az*vy) ;            //向量外积在相减得到差分就是误差
//	ey = (az*vx - ax*vz) ;
//	ez = (ax*vy - ay*vx) ;	

///*----对误差进行积分-----*/
//	exInt = exInt + ex * ki;								
//	eyInt = eyInt + ey * ki;
//	ezInt = ezInt + ez * ki;
//	
///*-----调整陀螺仪测量-------*/
//	gx = gx + kp*ex + exInt;		   		//将误差PI后补偿到陀螺仪，即补偿零点漂移
//	gy = gy + kp*ey + eyInt;
//	gz = gz + kp*ez + ezInt;				//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减
//		//printf("%f %f %f\n",gx,gy,gz);
///*------四元数的微分方程------*/
//	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
//	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
//	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
//	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
//printf ("%f %f %f %f \n",q0,q1,q2,q3);

///*----四元数归一化----*/	
//	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
///*----规范后的四元数------*/
//	q0 = q0 / norm;
//	q1 = q1 / norm;
//	q2 = q2 / norm;
//	q3 = q3 / norm;
//	
///*-------得出欧拉角-------*/
//	//Euler_Angle.Z = GYRO_I.Z;
//	Euler_Angle.X = asin( (-2 * q1 * q3) + (2 * q0 * q2)) * RTA;
//	Euler_Angle.Y = atan2( ((2 * q2 * q3) + (2 * q0 * q1)) , ((-2 * q1 * q1) - (2 * q2 * q2) + 1)) * RTA;

//}

