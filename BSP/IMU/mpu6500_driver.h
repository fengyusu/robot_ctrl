#ifndef __MPU6500_DRIVER_H__
#define __MPU6500_DRIVER_H__

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include <stdint.h>
#include "stm32f4xx.h"
#include "mpu6500_spi.h"
#include "mpu6500_int.h"

/*
*********************************************************************************************************
*                                            DEFINES
*********************************************************************************************************
*/
// 定义MPU6500内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define ACCEL_CONFIG2   0x1D

#define USER_CTRL       0x6A
#define I2C_MST_CTRL    0x24
#define I2C_MST_DELAY_CTRL 0x67

#define I2C_SLV0_ADDR   0x25
#define I2C_SLV0_REG    0x26
#define I2C_SLV0_CTRL   0x27
#define I2C_SLV1_ADDR   0x28
#define I2C_SLV1_REG    0x29
#define I2C_SLV1_CTRL   0x2A
#define I2C_SLV2_ADDR   0x2B
#define I2C_SLV2_REG    0x2C
#define I2C_SLV2_CTRL   0x2D
#define I2C_SLV3_ADDR   0x2E
#define I2C_SLV3_REG    0x2F
#define I2C_SLV3_CTRL   0x30
#define I2C_SLV4_ADDR   0x31
#define I2C_SLV4_REG    0x32
#define I2C_SLV4_CTRL   0x34
#define I2C_SLV4_DI     0x35

#define I2C_SLV0_DO     0x63
#define I2C_SLV1_DO     0x64
#define I2C_SLV2_DO     0x65
#define I2C_SLV3_DO     0x66
#define I2C_SLV4_DO     0x33

#define	INT_PIN_CFG	    0x37
#define	INT_ENABLE	    0x38
#define	INT_STATUS	    0x3A

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48


#define EXT_SENS_DATA_00    0x49 
#define EXT_SENS_DATA_01    0x4A 
#define EXT_SENS_DATA_02    0x4B 
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D 
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F 
#define EXT_SENS_DATA_07    0x50 
#define EXT_SENS_DATA_08    0x51 
#define EXT_SENS_DATA_09    0x52 


#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x70，只读)

#define MPU6500_DATA_START      ACCEL_XOUT_H
#define READ    0x80
#define WRITE   0x00

//定义IST8310的内部寄存器地址

#define IST8310_ADDRESS          0x0E
#define IST8310_DEVICE_ID_A      0x10

// IST8310 register map. For details see IST8310 datasheet
#define IST8310_WHO_AM_I    0x00
#define IST8310_R_CONFA     0x0A
#define IST8310_R_CONFB     0x0B
#define IST8310_R_MODE      0x02

#define IST8310_R_XL        0x03
#define IST8310_R_XM        0x04
#define IST8310_R_YL        0x05  
#define IST8310_R_YM        0x06  
#define IST8310_R_ZL        0x07  
#define IST8310_R_ZM        0x08  

#define IST8310_AVGCNTL     0x41
#define IST8310_PDCNTL      0x42

#define IST8310_ODR_MODE            0x01    //sigle measure mode

#define DELAY_CNT   4           //陀螺仪初始化命令延时


/*
 *  定义结构体
 */
typedef struct __MPU6500_RAW_Data__
{
    short Accel_X;  //寄存器原生X轴加速度表示值
    short Accel_Y;  //寄存器原生Y轴加速度表示值
    short Accel_Z;  //寄存器原生Z轴加速度表示值
    short Temp;     //寄存器原生温度表示值
    short Gyro_X;   //寄存器原生X轴陀螺仪表示值
    short Gyro_Y;   //寄存器原生Y轴陀螺仪表示值
    short Gyro_Z;   //寄存器原生Z轴陀螺仪表示值
	short Mag_X;    //寄存器原生X轴陀螺仪表示值
    short Mag_Y;    //寄存器原生Y轴陀螺仪表示值
    short Mag_Z;    //寄存器原生Z轴陀螺仪表示值
	
}MPU6500_RAW_DATA;
extern volatile MPU6500_RAW_DATA    MPU6500_Raw_Data; 

typedef struct __MPU6500_REAL_Data__
{
    float Accel_X;  //转换成实际的X轴加速度，
    float Accel_Y;  //转换成实际的Y轴加速度，
    float Accel_Z;  //转换成实际的Z轴加速度，
    float Temp;     //转换成实际的温度，单位为摄氏度
    float Gyro_X;   //转换成实际的X轴角加速度，
    float Gyro_Y;   //转换成实际的Y轴角加速度，
    float Gyro_Z;   //转换成实际的Z轴角加速度
	float Mag_X;    //转换成实际的X轴角加速度，
    float Mag_Y;    //转换成实际的Y轴角加速度，
    float Mag_Z;    //转换成实际的Z轴角加速度
	
}MPU6500_REAL_DATA;
extern volatile MPU6500_REAL_DATA   MPU6500_Real_Data;

//the max and min data of the mag
typedef __packed struct
{
	int16_t MaxMagX;
	int16_t MaxMagY;
	int16_t MaxMagZ;
	int16_t MinMagX;
	int16_t MinMagY;
	int16_t MinMagZ;
}MagMaxMinData_t;

uint8_t MPU6500_Init(void);
uint8_t MPU6500_Enable(void);
void MPU6500_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* tem);
void MPU6500_getlastMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz,int16_t* tem);
uint8_t BSP_IST8310_Init(void);
void IST8310_mgetValues(volatile float *arry);
void IST8310_getlastValues(int16_t *x,int16_t *y,int16_t *z);

#endif
/******************************** end of bsp_mpu6500_driver.h ***********************************/



