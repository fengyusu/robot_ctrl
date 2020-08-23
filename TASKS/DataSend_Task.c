#include "includes.h"
#include "mpu6500_driver.h"
#include <math.h>
 
void param_display(void)
{
	
	uint8_t head[2] = {0};
	uint16_t send_data[3][8] = { { 0 }, { 0 }, { 0 } };

			head[0] = 'S';
			head[1] = 'T';
			
			send_data[0][0] = (int16_t)(MPU6500_Real_Data.Accel_X);
			send_data[0][1] = (int16_t)(MPU6500_Real_Data.Accel_Y);
			send_data[0][2] = (int16_t)(MPU6500_Real_Data.Accel_Z);
			send_data[0][3] = (int16_t)(0);
			send_data[0][4] = (int16_t)(0);
			send_data[0][5] = (int16_t)(0);
			send_data[0][6] = (int16_t)(0);
			send_data[0][7] = (int16_t)(0);

			send_data[1][0] = (int16_t)(euler_angle.yaw_angle);
			send_data[1][1] = (int16_t)(euler_angle.pitch_angle);
			send_data[1][2] = (int16_t)(euler_angle.roll_angle);
			send_data[1][3] = (int16_t)(0);
			send_data[1][4] = (int16_t)(0);
			send_data[1][5] = (int16_t)(0);
			send_data[1][6] = (int16_t)(0);
			send_data[1][7] = (int16_t)(0);

			send_data[2][0] = (int16_t)(MPU6500_Real_Data.Gyro_X);
			send_data[2][1] = (int16_t)(MPU6500_Real_Data.Gyro_Y);
			send_data[2][2] = (int16_t)(MPU6500_Real_Data.Gyro_Z);
			send_data[2][3] = (int16_t)(0);
			send_data[2][4] = (int16_t)(0);
			send_data[2][5] = (int16_t)(0);
			send_data[2][6] = (int16_t)(0);
			send_data[2][7] = (int16_t)(0);

	UART3_PrintBlock(head, 2);
	UART3_PrintBlock((uint8_t *)send_data,24*2);

}


void DataSend_Task(void *param)
{
    while(1)
	{
//	    param_display();
		vTaskDelay(10);
		printf("%f,%f,%f,%f,%f,%f\r\n",MPU6500_Real_Data.Accel_X,MPU6500_Real_Data.Accel_Y,MPU6500_Real_Data.Accel_Z,
		                               MPU6500_Real_Data.Gyro_X,MPU6500_Real_Data.Gyro_Y,MPU6500_Real_Data.Gyro_Z);
	}
}
