#include "includes.h"
#include "Protocol_Task.h"
#include "bsp_can1.h"
#include "encoder.h"
#include <stdlib.h>
#include <math.h>

WorkState_e workState = PREPARE_STATE;
SubWorkState_e subWorkState = PREPARE_STATE_STEP1;

PID_Regulator_t GMPPositionPID = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;     
PID_Regulator_t GMPSpeedPID    = GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t GMYPositionPID = GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT;			
PID_Regulator_t GMYSpeedPID    = GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT;


volatile Encoder GMYawEncoder   = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMPitchEncoder = {0,0,0,0,0,0,0,0,0};

float ZGyroModuleAngle = 0.0f;

RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp   = RAMP_GEN_DAFAULT;

static QueueHandle_t gimbalCtrlQueue = NULL;

static int16_t GimbalYawOffset;
static int16_t GimbalPitchOffset;

Gimbal_Ctrl_t GimbalCtrl;

uint32_t gimbal_task_tick = 0;



QueueHandle_t gimbalCtrlQueue_Get(void)
{
    return gimbalCtrlQueue;
}

void GimbalParamInit(void)
{
    gimbal_task_tick = 0;
	
//	AppParamInit();
	GMPitchEncoder.ecd_bias = 8000;
	GMYawEncoder.ecd_bias = 6310;
//	GMPitchEncoder.ecd_bias = 5520;
//	GMYawEncoder.ecd_bias = 7886;
    workState = PREPARE_STATE;
	
	GMPitchRamp.SetScale(&GMPitchRamp, PREPARE_TIME_TICK_MS);
	GMYawRamp.SetScale(&GMYawRamp, PREPARE_TIME_TICK_MS);
	GMPitchRamp.ResetCounter(&GMPitchRamp);
	GMYawRamp.ResetCounter(&GMYawRamp);
	
	GimbalCtrl.ctrl_mode = REMOTE_CTRL;
	GimbalCtrl.pitch_angle_dynamic_ref = 0.0f;
	GimbalCtrl.yaw_angle_dynamic_ref = 0.0f;
	

	
	GMPPositionPID.Reset(&GMPPositionPID);
	GMPSpeedPID.Reset(&GMPSpeedPID);
	GMYPositionPID.Reset(&GMYPositionPID);
	GMYSpeedPID.Reset(&GMYSpeedPID);
	
	CAN1_Receive_Enable();
	Set_Gimbal_Current(CAN1, 0, 0, 0);
}

void CalibGimbalOffset(void)
{
	if((RC_CtrlData.rc.ch0 == 1684) && (RC_CtrlData.rc.ch1 == 1684))
	{
		GMPitchEncoder.ecd_bias = GMPitchEncoder.raw_value;
		GMYawEncoder.ecd_bias = GMYawEncoder.raw_value;
		gAppParam.GimbalCaliData.GimbalCaliFlag = PARAM_CALI_DONE;
		gAppParam.GimbalCaliData.GimbalPitchOffset = GMPitchEncoder.ecd_bias;
		gAppParam.GimbalCaliData.GimbalYawOffset = GMYawEncoder.ecd_bias;	
		AppParamSave();
	}
}


void SetWorkState(WorkState_e state)
{
    workState = state;
}

WorkState_e GetWorkState(void)
{
	return workState;
}

void SetSubWorkState(SubWorkState_e state)
{
    subWorkState = state;
}

SubWorkState_e GetSubWorkState()
{
	return subWorkState;
}


void WorkStateFSM(void)
{
	WorkState_e lastWorkState= workState;

	switch(workState)
	{
		case PREPARE_STATE:
		{
			if(GimbalCtrl.ctrl_mode == STOP)
			{
				workState = STOP_STATE;
			}
//			else if(GetCaliCmdFlagGrp())   
//			{
//				workState = CALI_STATE;
//			}
            else if(gimbal_task_tick < PREPARE_STEP1_TIME_TICK_MS)
            {
                subWorkState = PREPARE_STATE_STEP1;
            }
			 else if(gimbal_task_tick <= PREPARE_STEP2_TIME_TICK_MS)
            {
                subWorkState = PREPARE_STATE_STEP2;
            }
            else if(gimbal_task_tick <= PREPARE_STEP3_TIME_TICK_MS)
            {
                subWorkState = PREPARE_STATE_STEP3;
            }
			else if(gimbal_task_tick <= PREPARE_STEP4_TIME_TICK_MS)
            {
                subWorkState = PREPARE_STATE_STEP4;
            }
            else if( gimbal_task_tick > PREPARE_TIME_TICK_MS )
              //  PREPARE_TIME_TICK_MS 
			{
				workState = NORMAL_STATE;
			}			
		}break;
		case NORMAL_STATE:     
		{
			if(GimbalCtrl.ctrl_mode == STOP )
			{
				workState = STOP_STATE;
			}
//			else if(GetCaliCmdFlagGrp())   
//			{
//				workState = CALI_STATE;
//			}
			else if(0)
			{
				//workState = STANDBY_STATE;      
			}			
		}break;
		case STOP_STATE:   
		{
			if(GimbalCtrl.ctrl_mode != STOP )
			{
				workState = PREPARE_STATE;   
			}
		}break;
		case CALI_STATE:      
		{
			if(GimbalCtrl.ctrl_mode == STOP )
			{
				workState = STOP_STATE;
			}
		}break;	    
		default:
		{
			
		}
	}

	if((lastWorkState != workState) && (workState == PREPARE_STATE))  
	{
		GimbalParamInit();
	}	
}

static uint32_t can1_count = 0;
void Can1_ReceiveGimbalMsgProcess(CanRxMsg * msg)
{      

	switch(msg->StdId)
	{
		case CAN_BUS1_YAW_MOTOR_FEEDBACK_MSG_ID:
		{
			can1_count++;
 
				 
            if(can1_count <= 10)	
			{
				GetEncoderBias(&GMYawEncoder ,msg);
			}	
            else
			{
				EncoderProcess(&GMYawEncoder ,msg);	
			}				
				// 比较保存编码器的值和偏差值，如果编码器的值和初始偏差之间差距超过阈值，将偏差值做处理，防止出现云台反方向运动
			if(GetWorkState() == PREPARE_STATE)   //准备阶段要求二者之间的差值一定不能大于阈值，否则肯定是出现了临界切换
			{
				if((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) <-4000)
				{
					GMYawEncoder.ecd_bias = GimbalYawOffset + 8192;
				}
				else if((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) > 4000)
				{
					GMYawEncoder.ecd_bias = GimbalYawOffset - 8192;
				}
			}
		}break;
				
		case CAN_BUS1_PITCH_MOTOR_FEEDBACK_MSG_ID:
		{
                    //GMPitchEncoder.ecd_bias = pitch_ecd_bias;
			if(can1_count <= 10)	
			{
				GetEncoderBias(&GMPitchEncoder ,msg);
			}	
            else
			{
				EncoderProcess(&GMPitchEncoder ,msg);	
			}	
                    //码盘中间值设定也需要修改
            if(GetWorkState() == PREPARE_STATE)   //准备阶段要求二者之间的差值一定不能大于阈值，否则肯定是出现了临界切换
            {
				if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) <-4000)
                {
					GMPitchEncoder.ecd_bias = GimbalPitchOffset + 8192;
                }
                else if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) > 4000)
                {
					GMPitchEncoder.ecd_bias = GimbalPitchOffset - 8192;
                }
			}
		}break;		
				
			
		case CAN_ID_CAR_CHASSIS:
		{
			FIFO_S_Puts(GetCan1RxFifo(),msg->Data,msg->DLC);
			BaseType_t xHighPriorityTaskWoken = pdFALSE;
			BaseType_t xResult = xEventGroupSetBitsFromISR(xGlobalEventGroup, EVENT_CAN1_RECEIVE, &xHighPriorityTaskWoken);
			if(xResult != pdFALSE)
			{
				portYIELD_FROM_ISR(xHighPriorityTaskWoken);
			}
		}break;
		
		default:
			break;

	}
	 
}

static uint32_t can2_count = 0;
uint16_t GyoInitFlag = 0;
void Can2_ReceiveGyroMsgProcess(CanRxMsg * msg)
{      
    can2_count++;
    switch(msg->StdId)
    {
        case CAN_BUS2_ZGYRO_FEEDBACK_MSG_ID:
        {

            ZGyroModuleAngle = -0.01f*((int32_t)(msg->Data[0]<<24)|(int32_t)(msg->Data[1]<<16) | (int32_t)(msg->Data[2]<<8) | (int32_t)(msg->Data[3])); 

			GyoInitFlag = 0;
        }break;                
        default:
        {
        }
    }	 
}

void GYRO_RST(void)
{
    CanTxMsg tx_message;
    
    tx_message.StdId = 0x404;//send to gyro controll board
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x01;
    tx_message.Data[2] = 0x02;
    tx_message.Data[3] = 0x03;
    tx_message.Data[4] = 0x04;
    tx_message.Data[5] = 0x05;
    tx_message.Data[6] = 0x06;
    tx_message.Data[7] = 0x07;
    
    CAN_Transmit(CAN2,&tx_message);
}

/********************************************************************************
   给云台电调板发送指令，ID号为0x1FF，三个个电调板，数据回传ID为0x205、0206和0x207
*********************************************************************************/
void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq,int16_t shoot_iq)
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(gimbal_yaw_iq >> 8);
    tx_message.Data[1] = (unsigned char)gimbal_yaw_iq;
    tx_message.Data[2] = (unsigned char)(gimbal_pitch_iq >> 8);
    tx_message.Data[3] = (unsigned char)gimbal_pitch_iq;
    tx_message.Data[4] = (unsigned char)(shoot_iq >> 8);
    tx_message.Data[5] = (unsigned char)shoot_iq;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
	
	
}

void SetGimbalMotorOutput(void)
{
	//云台控制输出								
	if(GetWorkState() == STOP_STATE)
	{
	    Set_Gimbal_Current(CAN1, 0, 0, 0);     //yaw + pitch + bullet	
	}
	else
	{
		Set_Gimbal_Current(CAN1, (int16_t)GMYSpeedPID.output, (int16_t)GMPSpeedPID.output, 0);
//		 Set_Gimbal_Current(CAN1, 0, 0, 0);
	}
}






//云台pitch轴控制程序
void GMPitchControlLoop(void)
{
//    GMPPositionPID.kp = 10;   //设置pid的参数
//	GMPPositionPID.ki = 0;
//	GMPPositionPID.kd = 30;
//		
//	GMPSpeedPID.kp = 5;
//	GMPSpeedPID.ki = 0;
//	GMPSpeedPID.kd = 16;
	
    if(GetWorkState() == PREPARE_STATE)
    {             
		if(GetSubWorkState() == PREPARE_STATE_STEP1 || GetSubWorkState() == PREPARE_STATE_STEP2)
		{
            GMPitchRamp.SetScale(&GMPitchRamp ,PREPARE_STEP2_TIME_TICK_MS); 
            GMPPositionPID.ref = 0;
            GMPPositionPID.fdb = (-GMPitchEncoder.ecd_angle)* GMPitchRamp.Calc(&GMPitchRamp);    //加入斜坡函数       
		}	
        else if(GetSubWorkState() == PREPARE_STATE_STEP3 || GetSubWorkState() == PREPARE_STATE_STEP4)	
		{
		    GMPPositionPID.ref = GimbalCtrl.pitch_angle_dynamic_ref;             //由遥控器获得
            GMPPositionPID.fdb = -GMPitchEncoder.ecd_angle; 
		}			
    }
    else
    {
        GMPPositionPID.ref = GimbalCtrl.pitch_angle_dynamic_ref;             //由遥控器获得
        GMPPositionPID.fdb = -GMPitchEncoder.ecd_angle;                     //由编码器获得        
    }
	GMPPositionPID.Calc(&GMPPositionPID);   //得到pitch轴位置环输出控制量
    //pitch speed control
	GMPSpeedPID.ref = GMPPositionPID.output;
	GMPSpeedPID.fdb = -MPU6500_Real_Data.Gyro_X;
	GMPSpeedPID.Calc(&GMPSpeedPID);         //得到pitch轴速度环的输出控制量    
		
		//GMPSpeedPID.output = GimbalCtrl.pitch_speed_ref *2;
}

float GyroYawAngleBias = 0.0f;
float ImuYawAngleBias = 0.0f;
float EncoderYawAngleBias = 0.0f;
uint16_t chassis_rotate_lock = 0;
//云台yaw轴控制程序

void GMYawControlLoop(void)
{
	if(GetWorkState() == PREPARE_STATE)
	{
		if(GetSubWorkState() == PREPARE_STATE_STEP1 || GetSubWorkState() == PREPARE_STATE_STEP2)
		{
		    GMYPositionPID.ref = 0;//yaw_angle_init;
            GMYPositionPID.fdb = 0;//-GMYawEncoder.ecd_angle; 
		}
		else if(GetSubWorkState() == PREPARE_STATE_STEP3 || GetSubWorkState() == PREPARE_STATE_STEP4)
		{
			GMYawRamp.SetScale(&GMYawRamp ,PREPARE_STEP4_TIME_TICK_MS-PREPARE_STEP2_TIME_TICK_MS);
            GMYPositionPID.ref = 0.0f;
            GMYPositionPID.fdb = (-GMYawEncoder. ecd_angle)*GMYawRamp.Calc(&GMYawRamp); 
			
			GyroYawAngleBias = (-ZGyroModuleAngle);
			ImuYawAngleBias = euler_angle.yaw_angle;
			EncoderYawAngleBias = (-GMYawEncoder. ecd_angle);
		}
	}
	else
	{  
		GMYPositionPID.ref = GimbalCtrl.yaw_angle_dynamic_ref;
		
//		GMYPositionPID.fdb = (-ZGyroModuleAngle - GyroYawAngleBias); 
		
		if(fabs(GimbalCtrl.yaw_speed_ref) >= 0.01)
		{
		    GMYPositionPID.fdb = (-ZGyroModuleAngle) - (GyroYawAngleBias);
			EncoderYawAngleBias = (-GMYawEncoder. ecd_angle) - (GMYPositionPID.fdb);
			chassis_rotate_lock = 260;
		}
		else if(fabs(GimbalCtrl.yaw_speed_ref) < 0.01)
		{
			if(chassis_rotate_lock>0)
			{
				chassis_rotate_lock--;
				GMYPositionPID.fdb = (-ZGyroModuleAngle) - (GyroYawAngleBias);
				EncoderYawAngleBias = (-GMYawEncoder. ecd_angle) - (GMYPositionPID.fdb);
			}
			else 
			{
				GMYPositionPID.fdb = (-GMYawEncoder. ecd_angle) - (EncoderYawAngleBias);
				GyroYawAngleBias = (-ZGyroModuleAngle) - (GMYPositionPID.fdb);
			}
		}
		
//		if(fabs(GimbalCtrl.yaw_speed_ref) >= 0.01)
//		{
//			GMYPositionPID.fdb = (euler_angle.yaw_angle - ImuYawAngleBias);
//		    EncoderYawAngleBias = (-GMYawEncoder.ecd_angle) - (GMYPositionPID.fdb);
//			chassis_rotate_lock = 80;
//		}
//		else if(fabs(GimbalCtrl.yaw_speed_ref) < 0.01)
//		{
//			if(chassis_rotate_lock>0)
//			{
//				chassis_rotate_lock--;
//				GMYPositionPID.fdb = (euler_angle.yaw_angle - ImuYawAngleBias);
//			    EncoderYawAngleBias = (-GMYawEncoder.ecd_angle) - GMYPositionPID.fdb;
//			}
//			else
//			{
//		        GMYPositionPID.fdb = (-GMYawEncoder.ecd_angle) - (EncoderYawAngleBias);
//			    ImuYawAngleBias = euler_angle.yaw_angle - GMYPositionPID.fdb;
//			}
//		}
		
	}
    //yaw轴位置环的目标值和反馈值
	GMYPositionPID.Calc(&GMYPositionPID);        
	//yaw speed control
	GMYSpeedPID.ref = GMYPositionPID.output;
	GMYSpeedPID.fdb =  -MPU6500_Real_Data.Gyro_Z;
	GMYSpeedPID.Calc(&GMYSpeedPID);	 
}

/*
void GMYawControlLoop(void)
{
	if(GetWorkState() == PREPARE_STATE)
	{
		if(GetSubWorkState() == PREPARE_STATE_STEP1 || GetSubWorkState() == PREPARE_STATE_STEP2)
		{
		    GMYPositionPID.ref = 0;//yaw_angle_init;
            GMYPositionPID.fdb = 0;//-GMYawEncoder.ecd_angle; 
		}
		else if(GetSubWorkState() == PREPARE_STATE_STEP3 || GetSubWorkState() == PREPARE_STATE_STEP4)
		{
			GMYawRamp.SetScale(&GMYawRamp ,PREPARE_STEP4_TIME_TICK_MS-PREPARE_STEP2_TIME_TICK_MS);
            GMYPositionPID.ref = 0.0f;
            GMYPositionPID.fdb = (-GMYawEncoder. ecd_angle)*GMYawRamp.Calc(&GMYawRamp); 
			
			GyroYawAngleBias = ZGyroModuleAngle;
		}
	}
	else
	{  
		GMYPositionPID.ref = GimbalCtrl.yaw_angle_dynamic_ref;
		GMYPositionPID.fdb = -(ZGyroModuleAngle-GyroYawAngleBias);  
		
	}
    //yaw轴位置环的目标值和反馈值
	GMYPositionPID.Calc(&GMYPositionPID);        
	//yaw speed control
	GMYSpeedPID.ref = GMYPositionPID.output;
	GMYSpeedPID.fdb =  -MPU6500_Real_Data.Gyro_Z;
	GMYSpeedPID.Calc(&GMYSpeedPID);	 
}
*/

void GimabalCtrl_Task(void *param)
{
	BaseType_t xResult; 
    TickType_t xLasWakeTime;
	const TickType_t xFrequency = 1;
	const TickType_t xTicksToWait = 2 / portTICK_PERIOD_MS;
	
	uint16_t diff_time = 1;
	Gimbal_Ctrl_t gimbal_ctrl_tmp;
	float pitch_last_ref,yaw_last_ref;
	
	
	gimbalCtrlQueue = xQueueCreate(10, sizeof(Gimbal_Ctrl_t));
	if(gimbalCtrlQueue == 0){}
	
	GimbalParamInit();
	vTaskDelay(1000);
	
	while((xEventGroupWaitBits(xGlobalEventGroup,EVENT_REMOTE_REV,pdTRUE,pdTRUE,xTicksToWait) & EVENT_REMOTE_REV) != EVENT_REMOTE_REV);
	
//	while(gAppParam.GimbalCaliData.GimbalCaliFlag != PARAM_CALI_DONE)
//	{
//		CalibGimbalOffset();
//		vTaskDelay(10);
//	}
	
	xLasWakeTime = xTaskGetTickCount();
    while(1)
	{

		xResult = xQueueReceive(gimbalCtrlQueue_Get(), &gimbal_ctrl_tmp, (TickType_t)xFrequency);

		if(xResult == pdPASS)
		{
			diff_time = (gimbal_ctrl_tmp.ctrl_cycle) / xFrequency;
			if(diff_time<1)diff_time = 1;
				
			pitch_last_ref = GimbalCtrl.pitch_angle_dynamic_ref;
			yaw_last_ref = GimbalCtrl.yaw_angle_dynamic_ref; 
				
			GimbalCtrl.ctrl_mode  = gimbal_ctrl_tmp.ctrl_mode;
			GimbalCtrl.ctrl_cycle = gimbal_ctrl_tmp.ctrl_cycle;
			GimbalCtrl.pitch_speed_ref = gimbal_ctrl_tmp.pitch_speed_ref;
			GimbalCtrl.yaw_speed_ref = gimbal_ctrl_tmp.yaw_speed_ref;
			
			vTaskDelay(xFrequency);
		}
		
		if(GetWorkState() != PREPARE_STATE)
		{
			GimbalCtrl.pitch_angle_dynamic_ref += (gimbal_ctrl_tmp.pitch_angle_dynamic_ref - pitch_last_ref) / (float)(diff_time);
			GimbalCtrl.yaw_angle_dynamic_ref += (gimbal_ctrl_tmp.yaw_angle_dynamic_ref - yaw_last_ref) / (float)(diff_time);	
		}
		
		gimbal_task_tick ++;
			
	    WorkStateFSM();
		
		if(gimbal_task_tick == PREPARE_STEP2_TIME_TICK_MS)
		{
			GYRO_RST();
		}
			
		GMPitchControlLoop();
		GMYawControlLoop();
		
		
		SetGimbalMotorOutput();
	    
		
//		vTaskDelayUntil(&xLasWakeTime, xFrequency);
	}
}
