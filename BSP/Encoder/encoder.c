#include "encoder.h"

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg)
{
    v->raw_value = (msg->Data[0]<<8)|msg->Data[1];  //保存初始编码器值作为偏差  
    v->ecd_value = v->raw_value;
    v->last_raw_value = v->raw_value;
    v->temp_count++;
}


void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->speed_rpm = (msg->Data[2]<<8)|msg->Data[3];
	v->diff = v->raw_value - v->last_raw_value;
    //得到编码器速度，单位：线-每-周期
	if(v->diff < -6500)    //两次编码器的反馈值差别太大，表示圈数发生了改变,7500是1个单位时间（1ms）的最大角度。
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>6500)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到连续角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);					
 }

 