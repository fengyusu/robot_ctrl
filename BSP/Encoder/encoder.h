#ifndef __ENCODER__H_
#define __ENCODER__H_

#include "stm32f4xx.h"

#define RATE_BUF_SIZE 6

typedef __packed struct{
	int32_t raw_value;   									//����������������ԭʼֵ����ǰֵ
	int32_t last_raw_value;								    //��һ�εı�����ԭʼֵ
	int32_t ecd_value;                                      //���������������ı�����ֵ
	int32_t diff;											//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                                     //������
	uint8_t buf_count;								        //�˲�����buf��
	int32_t ecd_bias;										//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	                    //buf��for filter
	int32_t round_cnt;										//Ȧ��
	int32_t filter_rate;									//�ٶ�
	int16_t speed_rpm;
	float ecd_angle;										//�Ƕ�
}Encoder;

extern void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg);
extern void EncoderProcess(volatile Encoder *v, CanRxMsg * msg);

#endif
