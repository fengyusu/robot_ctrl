#include "param.h"
#include "bsp_flash.h"
#include "mpu6500_driver.h"
#include "GimbalCtrl_Task.h"
#include <string.h>
#include <stdint.h>
//#include "imu.h"
//工作流程介绍：
/*
1、上位机发送过来校准参数
2、调用XXXCaliProcess,将接收到的校准数据保存到GyroCaliData、GimbalCaliData、MagCaliData等中
3、校准完成后，调用SetCaliData(*v)，将校准数据保存到gAppParamFalsh中,并写入Flash中
*/
AppParam_t gAppParam;	//配置信息,这里保存着最新的校准值，并且与Flash中的内容同步
static GyroCaliStruct_t GyroCaliData;        //保存校准陀螺仪偏差值
static GimbalCaliStruct_t  GimbalCaliData;   //保存云台编码器偏差值
static MagCaliStruct_t  MagCaliData;         //保存磁力计校准值
PIDParamStruct_t PIDCaliData;  //保存pitch轴position校准值
//这几个变量用于在实际程序中应用
GimbalCaliStruct_t GimbalSavedCaliData;    	    //gimbal pitch yaw encoder offset
GyroCaliStruct_t GyroSavedCaliData;     	    //gyro offset data
AccCaliStruct_t AccSavedCaliData;    	    	//ACC offset data
MagCaliStruct_t MagSavedCaliData;			    //Mag offset data
PIDParamStruct_t PitchPositionSavedPID;        	//PID offset data
PIDParamStruct_t PitchSpeedSavedPID;        	//PID offset data
PIDParamStruct_t YawPositionSavedPID;        	//PID offset data
PIDParamStruct_t YawSpeedSavedPID;        	    //PID offset data

uint8_t app_param_calied_flag = 0;


extern MagMaxMinData_t MagMaxMinData;


//用于在SuperviseTask中设置错误标志位
uint8_t Is_AppParam_Calied(void)
{
	return app_param_calied_flag;    //param未初始化
}

//用于保存数据到flash中
uint8_t AppParamSave(void)
{
    uint8_t retval = 1;   
    retval = BSP_FLASH_Write(PARAM_SAVED_START_ADDRESS, (uint8_t *)&gAppParam, sizeof(AppParam_t));    
    if(retval == 0)
    {
			
    }
    return retval;   
}


void SetGimbalCaliData(GimbalCaliStruct_t *cali_data)
{
	if(cali_data != NULL)
    {
		memcpy(&gAppParam.GimbalCaliData, cali_data, sizeof(*cali_data));
		AppParamSave();
	}
}


void SetGyroCaliData(GyroCaliStruct_t *cali_data)
{
	if(cali_data != NULL)
    {
		memcpy(&gAppParam.GyroCaliData, cali_data, sizeof(*cali_data));
		AppParamSave();
	}
}  

void SetAccCaliData(AccCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(&gAppParam.AccCaliData, cali_data, sizeof(*cali_data));
		AppParamSave();
    }
}

void SetMagCaliData(MagCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
		memcpy(&gAppParam.MagCaliData, cali_data, sizeof(*cali_data));   //step1: copy data to struct
		AppParamSave();	
    }
																														 //step2:write data to the flash
}

//PID offset data saved in the memory 
void SetPIDCaliData(PIDParamStruct_t *cali_data)
{
	if(cali_data != NULL)
    {
//		if(cali_data->pid_type == PID_TYPE_POSITION && cali_data->motor_type == MOTOR_TYPE_PITCH)
//		{
//			cali_data->kp_offset += gAppParam.PitchPositionPID.kp_offset;
//			cali_data->ki_offset += gAppParam.PitchPositionPID.ki_offset;
//			cali_data->kd_offset += gAppParam.PitchPositionPID.kd_offset;			
//			memcpy(&gAppParam.PitchPositionPID, cali_data, sizeof(*cali_data));
//		}
//		else if(cali_data->pid_type == PID_TYPE_SPEED && cali_data->motor_type == MOTOR_TYPE_PITCH)
//		{
//			cali_data->kp_offset += gAppParam.PitchSpeedPID.kp_offset;
//			cali_data->ki_offset += gAppParam.PitchSpeedPID.ki_offset;
//			cali_data->kd_offset += gAppParam.PitchSpeedPID.kd_offset;	
//			memcpy(&gAppParam.PitchSpeedPID, cali_data, sizeof(*cali_data));
//		}
//		else if(cali_data->pid_type == PID_TYPE_POSITION && cali_data->motor_type == MOTOR_TYPE_YAW)
//		{
//			cali_data->kp_offset += gAppParam.YawPositionPID.kp_offset;
//			cali_data->ki_offset += gAppParam.YawPositionPID.ki_offset;
//			cali_data->kd_offset += gAppParam.YawPositionPID.kd_offset;	
//			memcpy(&gAppParam.YawPositionPID, cali_data, sizeof(*cali_data));
//		}
//		else if(cali_data->pid_type == PID_TYPE_SPEED && cali_data->motor_type == MOTOR_TYPE_YAW)
//		{
//			cali_data->kp_offset += gAppParam.YawSpeedPID.kp_offset;
//			cali_data->ki_offset += gAppParam.YawSpeedPID.ki_offset;
//			cali_data->kd_offset += gAppParam.YawSpeedPID.kd_offset;	
//			memcpy(&gAppParam.YawSpeedPID, cali_data, sizeof(*cali_data));
//		}
		AppParamSave();	
	}
}

void GetGimbalCaliData(GimbalCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(cali_data, &gAppParam.GimbalCaliData, sizeof(GimbalCaliStruct_t));
    }
}

void GetGyroCaliData(GyroCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(cali_data, &gAppParam.GyroCaliData, sizeof(GyroCaliStruct_t));
    }
}

void GetAccCaliData(AccCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(cali_data, &gAppParam.AccCaliData, sizeof(AccCaliStruct_t));
    }
}

void GetMagCaliData(MagCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(cali_data, &gAppParam.MagCaliData, sizeof(MagCaliStruct_t));
    }
}

uint8_t IsGimbalCalied(void)
{
    return (gAppParam.GimbalCaliData.GimbalCaliFlag == PARAM_CALI_DONE);
}

uint8_t IsGyroCalied(void)
{
    return (gAppParam.GyroCaliData.GyroCaliFlag == PARAM_CALI_DONE);
}

uint8_t IsAccCalied(void)
{
    return (gAppParam.AccCaliData.AccCaliFlag == PARAM_CALI_DONE);
}

uint8_t IsMagCalied(void)
{
    return (gAppParam.MagCaliData.MagCaliFlag == PARAM_CALI_DONE);
}

/*********************************************************************
 * @fn      CalibrateLoop
 *
 * @brief   do the calibration according to the corresponding cali flag
 *
 * @param   *flag_grp - the pointer to the cali flag group
 *
 * @return  none
 *********************************************************************
 */


static uint32_t CaliCmdFlagGrp = 0;     //cali cmd flag group every bit represents a cali cmd received from the PC

void SetCaliCmdFlag(uint32_t flag)  //设置校准标志位
{
	CaliCmdFlagGrp |= flag;
}

void ResetCaliCmdFlag(uint32_t flag)
{
	CaliCmdFlagGrp &= ~flag;
}

uint32_t GetCaliCmdFlagGrp()
{
	return CaliCmdFlagGrp;
}

//to check whether a specfic flag if set,1 is set,0 is reset
uint8_t IsCaliCmdFlagSet(uint32_t flag)
{
	if(flag & CaliCmdFlagGrp)
	{
		return 1;
	}else
	{
		return 0;	
	}
}

void CalibrateLoop(void)
{
    CALI_STATE_e cali_result;    
    //gyro cali 
    if(IsCaliCmdFlagSet(CALI_START_FLAG_GYRO))   //
	{
		ResetCaliCmdFlag(CALI_START_FLAG_GYRO);
	}
	else if(IsCaliCmdFlagSet(CALI_END_FLAG_GYRO))   //calibrate the 
    {
        cali_result = GyroCaliProcess();  
		if(cali_result == CALI_STATE_ERR)
		{
			
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{
			SetGyroCaliData(&GyroCaliData);   //set the apparamStruct using the GyroCaliData, and save apparamStruct to the flash 
			ResetCaliCmdFlag(CALI_END_FLAG_GYRO);		//reset the cali cmd
		}
    }
	else if(IsCaliCmdFlagSet(CALI_START_FLAG_GIMBAL))   //calibrate the 
	{
		ResetCaliCmdFlag(CALI_START_FLAG_GIMBAL);
	}
	else if(IsCaliCmdFlagSet(CALI_END_FLAG_GIMBAL))
	{
	    cali_result = GimbalCaliProcess();  
		if(cali_result == CALI_STATE_ERR)
		{
			
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{
			SetGimbalCaliData(&GimbalCaliData);           //set the apparamStruct using the GyroCaliData, and save apparamStruct to the flash 
  //update the parameter
			ResetCaliCmdFlag(CALI_END_FLAG_GIMBAL);
		}	
	}
	else if(IsCaliCmdFlagSet(CALI_START_FLAG_MAG))
	{
		cali_result = MagStartCaliProcess();   //reset the max min data of the magenemter
		if(cali_result == CALI_STATE_ERR)
		{
			
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{			
			ResetCaliCmdFlag(CALI_START_FLAG_MAG);
		}	
	}
	else if(IsCaliCmdFlagSet(CALI_END_FLAG_MAG))
	{
		cali_result = MagEndCaliProcess();  
		if(cali_result == CALI_STATE_ERR)
		{
			
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{
			SetMagCaliData(&MagCaliData);                 //set the apparamStruct using the GyroCaliData, and save apparamStruct to the flash 
  //update the parameter
			ResetCaliCmdFlag(CALI_END_FLAG_MAG);
		}		
	}
	else if(IsCaliCmdFlagSet(CALI_FLAG_PID))
	{
		SetPIDCaliData(&PIDCaliData);                 //将接收到的PIDCaliData数据保存到apparamStruct中
  //update the parameter
		ResetCaliCmdFlag(CALI_FLAG_PID);
	}
}



CALI_STATE_e  GimbalCaliProcess()     //返回校准状态   ERROR DONE
{
	static uint32_t loopCount = 0;
	static uint32_t loopTime = 10;
	static int32_t pitchSum = 0;
	static int32_t yawSum = 0;
	
	if(loopCount++<loopTime)   //in cali state
	{
//		pitchSum += GMPitchEncoder.raw_value;
//		yawSum += GMYawEncoder.raw_value;
		return CALI_STATE_IN;
	}
	else
	{		
		GimbalCaliData.GimbalPitchOffset = pitchSum/loopTime;   //读取pitch轴陀螺仪作为偏差
	    GimbalCaliData.GimbalYawOffset = yawSum/loopTime;		//读取yaw轴陀螺仪作为偏差
		GimbalCaliData.GimbalCaliFlag = PARAM_CALI_DONE;
		pitchSum = 0;
		yawSum = 0;
		loopCount = 0;
		return CALI_STATE_DONE;
	}	
}
int16_t acc_offset[3];
CALI_STATE_e  GyroCaliProcess()     
{
	int16_t temp[7] = {0};
	static uint16_t loopCount = 0;
	static uint16_t loopTime = 20;
	static int32_t gyroXSum = 0;
	static int32_t gyroYSum = 0;
	static int32_t gyroZSum = 0;
    //test cail acc
    static int32_t accXSum = 0;
    static int32_t accYSum = 0;
    static int32_t accZSum = 0;
    
	//将gyro值清零,如此得到的才是原始值
	GyroSavedCaliData.GyroXOffset = 0;
	GyroSavedCaliData.GyroYOffset = 0;
	GyroSavedCaliData.GyroZOffset = 0;	
	//process of the cali if error return error, elseif in processing return in , and if done return done
	if(loopCount++<loopTime)   //in cali state
	{
		MPU6500_getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5],&temp[6]);
		gyroXSum += temp[3];
		gyroYSum += temp[4];
		gyroZSum += temp[5];
        
        //test cail acc
        accXSum += temp[0];
        accYSum += temp[1];
        accZSum += temp[2];
        
		return CALI_STATE_IN;
	}
	else
	{					
		GyroCaliData.GyroXOffset = gyroXSum/loopTime;       //读取pitch轴陀螺仪作为偏差
	    GyroCaliData.GyroYOffset = gyroYSum/loopTime;		//读取yaw轴陀螺仪作为偏差
		GyroCaliData.GyroZOffset = gyroZSum/loopTime;		//读取yaw轴陀螺仪作为偏差
        
        //test cail acc
        acc_offset[0] = accXSum/loopTime;
        acc_offset[1] = accYSum/loopTime;
        acc_offset[2] = accZSum/loopTime;
        
		GyroCaliData.GyroCaliFlag = PARAM_CALI_DONE;
		gyroXSum = 0;
		gyroYSum = 0;
		gyroZSum = 0;
        
        //test cail acc
        accXSum = 0;
        accYSum = 0;
        accZSum = 0;
        
		loopCount = 0;
		return CALI_STATE_DONE;
	}
}

CALI_STATE_e  MagStartCaliProcess()
{	
	MagMaxMinData.MaxMagX = -4096;	//将原来的标定值清除
	MagMaxMinData.MaxMagY = -4096;
	MagMaxMinData.MaxMagZ = -4096;
	MagMaxMinData.MinMagX = 4096;
	MagMaxMinData.MinMagY = 4096;
	MagMaxMinData.MinMagZ = 4096;
	return CALI_STATE_DONE;	
}
CALI_STATE_e  MagEndCaliProcess()
{
//	if(Is_Lost_Error_Set(LOST_ERROR_IMU))    
//	{
//		return CALI_STATE_ERR;
//	}
//	else
//	{
		MagCaliData.MagXOffset = (MagMaxMinData.MaxMagX + MagMaxMinData.MinMagX)/2;
		MagCaliData.MagYOffset = (MagMaxMinData.MaxMagY + MagMaxMinData.MinMagY)/2;
		MagCaliData.MagZOffset = (MagMaxMinData.MaxMagZ + MagMaxMinData.MinMagZ)/2;
		MagCaliData.MagXScale = 1.0;
		MagCaliData.MagYScale = 1.0;
		MagCaliData.MagZScale = 1.0;	
		MagCaliData.MagCaliFlag = PARAM_CALI_DONE;
		return CALI_STATE_DONE;		
//	}	
}

//copy src pid offset data received from the PC to the static PitchPostionCaliData/PitchSpeedCaliData
CALI_STATE_e PIDCaliProcess(PIDParamStruct_t *cali_data)
{
	if(cali_data!=NULL)
	{
		memcpy(&PIDCaliData, cali_data, sizeof(*cali_data));
		return CALI_STATE_DONE;
	}	
    return CALI_STATE_DONE;
}

void Gimbal_Offset_Param_Init(void)
{	
//	GMPitchEncoder.ecd_bias = gAppParam.GimbalCaliData.GimbalPitchOffset;
//	GMYawEncoder.ecd_bias = gAppParam.GimbalCaliData.GimbalYawOffset;	
}


//用于从flash中读取校准数据
void AppParamInit(void)
{
    AppParam_t tmp_param;
    
	gAppParam.ParamSavedFlag = 0;
    memcpy(&tmp_param, (void *)PARAM_SAVED_START_ADDRESS, sizeof(AppParam_t));
    
	if((tmp_param.FirmwareVersion) != VERSION)
	{
        gAppParam.FirmwareVersion = VERSION;
		gAppParam.ParamSavedFlag = 1;
	}
	
	if(tmp_param.GimbalCaliData.GimbalCaliFlag == PARAM_CALI_DONE)
	{
	    memcpy(&(gAppParam.GimbalCaliData), &(tmp_param.GimbalCaliData), sizeof(GimbalCaliStruct_t));
		Gimbal_Offset_Param_Init();
	}
	else
	{
	    gAppParam.GimbalCaliData.GimbalCaliFlag = PARAM_CALI_NONE;
	}
	
	
	if(gAppParam.ParamSavedFlag)
	{
		AppParamSave();
	}
   		
}

void UploadParameter(void)
{				
  

}
