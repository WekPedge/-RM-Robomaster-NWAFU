#include "main.h"
#include "math.h"

float arrmx,arrmy,arrmz;
MagCaliStruct_t MagSavedCaliData;	

uint32_t Get_Time_Micros(void)
{
	return TIM2->CNT;
}

void IST_Eliminate_Deviation(uint32_t mx,uint32_t my,uint32_t mz)
{

	MagMaxMinData.MaxMagX = -4096;	//��ԭ���ı궨ֵ���
	MagMaxMinData.MaxMagY = -4096;
	MagMaxMinData.MaxMagZ = -4096;
	MagMaxMinData.MinMagX = 4096;
	MagMaxMinData.MinMagY = 4096;
	MagMaxMinData.MinMagZ = 4096;
	
	MagSavedCaliData.MagXOffset = (MagMaxMinData.MaxMagX + MagMaxMinData.MinMagX)/2;
	MagSavedCaliData.MagYOffset = (MagMaxMinData.MaxMagY + MagMaxMinData.MinMagY)/2;
	MagSavedCaliData.MagZOffset = (MagMaxMinData.MaxMagZ + MagMaxMinData.MinMagZ)/2;
	MagSavedCaliData.MagXScale = 1.0;
	MagSavedCaliData.MagYScale = 1.0;
	MagSavedCaliData.MagZScale = 1.0;	
	
	arrmx=((float)(mx - MagSavedCaliData.MagXOffset)) * MagSavedCaliData.MagXScale;
	arrmy=((float)(my - MagSavedCaliData.MagYOffset)) * MagSavedCaliData.MagYScale;
	arrmz=((float)(mz - MagSavedCaliData.MagZOffset)) * MagSavedCaliData.MagZScale;

}

void GET_Last_Value(void)
{	
	IMU_Get_Data();
//	IST8310_Get_Data();
	//IST_Eliminate_Deviation(arrx,arry,arrz);

}




volatile float exInt, eyInt, ezInt;  // ������
volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;

volatile float mygetqval[9];	//���ڴ�Ŵ�����ת�����������
//static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;   //��������ڴ��ļ���
float gx, gy, gz, ax, ay, az, mx, my, mz;   
static volatile float q[4]; //����Ԫ��
volatile uint32_t lastUpdate, now; // �������ڼ��� ��λ us
volatile float angle[3] = {0};
volatile float yaw_temp,pitch_temp,roll_temp;
volatile float last_yaw_temp,last_pitch_temp,last_roll_temp;
volatile float yaw_angle,pitch_angle,roll_angle; //ʹ�õ��ĽǶ�ֵ

// Fast inverse square-root
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float invSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x) 	
��������� Ҫ�����ֵ
��������� ���
*******************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void Init_Quaternion
*��������:	 ��ʼ����Ԫ��
��������� ��ǰ�Ĳ���ֵ��
���������û��
*******************************************************************************/
//��ʼ��IMU����
#define BOARD_DOWN 1   //�������泯�°ڷ�

void Init_Quaternion(void)//���ݲ������ݣ���ʼ��q0,q1,q2.q3���Ӷ��ӿ������ٶ�
{
//	IST_Eliminate_Deviation(arrx,arry,arrz);
//	int16_t hx,hy,hz;
	int16_t hx,hy;
	GET_Last_Value();
	hx=arrx;
	hy=arry;
//m
	#ifdef BOARD_DOWN
	if(hx<0 && hy <0)   //OK
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}
		
	}
	else if (hx<0 && hy > 0) //OK
	{
		if(fabs(hx/hy)>=1)   
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}
		
	}
	else if (hx > 0 && hy > 0)   //OK
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}
		
	}
	else if (hx > 0 && hy < 0)     //OK
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;			
		}
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}		
	}
	#else
		if(hx<0 && hy <0)
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = 0.195;
			q1 = -0.015;
			q2 = 0.0043;
			q3 = 0.979;
		}
		else
		{
			q0 = 0.555;
			q1 = -0.015;
			q2 = 0.006;
			q3 = 0.829;
		}
		
	}
	else if (hx<0 && hy > 0)
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = -0.193;
			q1 = -0.009;
			q2 = -0.006;
			q3 = 0.979;
		}
		else
		{
			q0 = -0.552;
			q1 = -0.0048;
			q2 = -0.0115;
			q3 = 0.8313;
		}
		
	}
	else if (hx>0 && hy > 0)
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = -0.9785;
			q1 = 0.008;
			q2 = -0.02;
			q3 = 0.195;
		}
		else
		{
			q0 = -0.9828;
			q1 = 0.002;
			q2 = -0.0167;
			q3 = 0.5557;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if(fabs(hx/hy)>=1)
		{
			q0 = -0.979;
			q1 = 0.0116;
			q2 = -0.0167;
			q3 = -0.195;			
		}
		else
		{
			q0 = -0.83;
			q1 = 0.014;
			q2 = -0.012;
			q3 = -0.556;
		}		
	}
	#endif
	
	//����hx hy hz���ж�q��ֵ��ȡ�ĸ������ֵ���ƽ�����,��ʼֵ������ŷ����ת������Ԫ������õ�
	 
}


	




/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_AHRSupdate
*��������:	 ����AHRS ������Ԫ�� 
��������� ��ǰ�Ĳ���ֵ��
���������û��
*******************************************************************************/
#define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.01f   // integral gain governs rate of convergence of gyroscope biases
void IMU_AHRSupdate(void) {
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez,halfT;
    float tempq0,tempq1,tempq2,tempq3;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;   

    gx =(FLOAT_date.gx)* M_PI/180;
    gz =( FLOAT_date.gz)* M_PI/180;
//	gy = FLOAT_date.gy/32.8f* M_PI/180;
    gy =Get_Yaw_Angle(FLOAT_date.gy)* M_PI/180;
    ax = FLOAT_date.ax;
    ay =FLOAT_date.ay;
    az = FLOAT_date.az;
	//az=az-33;
//    mx =arrx;
//    my =arry;
//    mz =arrz;		
	mx =0;
    my =0;
    mz =0;		

    now = Get_Time_Micros();  //��ȡʱ�� ��λ��us   
    if(now<lastUpdate)
    {
		halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);   //  uint 0.5s
    }
    else	
    {
        halfT =  ((float)(now - lastUpdate) / 2000000.0f);
    }
    lastUpdate = now;	//����ʱ��
    //������ƽ�����㷨
    norm = invSqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //�ѼӼƵ���ά����ת�ɵ�λ������
    norm = invSqrt(mx*mx + my*my + mz*mz);          
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm; 
    // compute reference direction of flux
    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz; 
    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;	
        ezInt = ezInt + ez * Ki * halfT;
        // �ò���������PI����������ƫ
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }
    // ��Ԫ��΢�ַ���
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

    // ��Ԫ���淶��
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;

}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getQ(float * q)
*��������:	 ������Ԫ�� ���ص�ǰ����Ԫ����ֵ
��������� ��Ҫ�����Ԫ���������׵�ַ
���������û��
*******************************************************************************/

void IMU_getQ(volatile float * q) {

   GET_Last_Value() ;	 //��ȡԭʼ����,���ٶȼƺʹ�������ԭʼֵ��������ת������deg/s
//	Init_Quaternion();

    IMU_AHRSupdate();
    q[0] = q0; //���ص�ǰֵ
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getYawPitchRoll(float * angles)
*��������:	 ������Ԫ�� ���ص�ǰ��������̬����
��������� ��Ҫ�����̬�ǵ������׵�ַ
���������û��
*******************************************************************************/
void IMU_getYawPitchRoll(volatile float * angles) 
{  
    // volatile float gx=0.0, gy=0.0, gz=0.0; //������������
    IMU_getQ(q); //����ȫ����Ԫ��
    //��Ԫ��ת����ŷ���ǣ��������Ǻ������㼴��
	// atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
    angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw        -pi----pi
    angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch    -pi/2    --- pi/2 
    angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll       -pi-----pi  
}

static int yaw_count = 0;
void GetPitchYawGxGyGz(float * angles)
{

	angle[0]= angles[0] ;
	last_yaw_temp = yaw_temp;
	yaw_temp = angle[0]; 
	if(yaw_temp-last_yaw_temp>=330)  //yaw��ǶȾ����������������
	{
		yaw_count--;
	}
	else if (yaw_temp-last_yaw_temp<=-330)
	{
		yaw_count++;
	}
	yaw_angle = yaw_temp + yaw_count*360;  //yaw��Ƕ�
	angles[0] =yaw_angle;
}

