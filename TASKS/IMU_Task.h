#ifndef _IMU_H_
#define _IMU_H_

#include <math.h>
#include "stm32f4xx.h"
#include "mpu6500_driver.h"
#include "mpu6500_int.h"

#define M_PI  (float)3.1415926535
	
typedef struct 
{
    float yaw_angle;
    float pitch_angle;
    float roll_angle;
}Euler_angles_t;
extern volatile Euler_angles_t euler_angle;

void Init_Quaternion(void);
void IMU_getYawPitchRoll(volatile float * ypr); //������̬
void GetPitchYawGxGyGz(void);
void IMU_getValues(volatile float * values);
    
extern int16_t MPU6500_FIFO[7][11];//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ
extern int16_t HMC5883_FIFO[3][11];//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ ע���Ŵ������Ĳ���Ƶ���������Ե����г�


extern void IMU_Task(void *param);

#endif

