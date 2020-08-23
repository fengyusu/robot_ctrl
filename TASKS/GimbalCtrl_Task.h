#ifndef __GIMBALCTRL_H_
#define __GIMBALCTRL_H_

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "encoder.h"

#define EVENT_REMOTE_REV   (0x1 << 0)

#define YAW_POSITION_KP_DEFAULTS  30
#define YAW_POSITION_KI_DEFAULTS  0.02f
#define YAW_POSITION_KD_DEFAULTS  36

#define YAW_SPEED_KP_DEFAULTS  16
#define YAW_SPEED_KI_DEFAULTS  0
#define YAW_SPEED_KD_DEFAULTS  25


#define PITCH_POSITION_KP_DEFAULTS   35     //38  35
#define PITCH_POSITION_KI_DEFAULTS   0.05     //0.05 0.05
#define PITCH_POSITION_KD_DEFAULTS   30      //50  30

#define PITCH_SPEED_KP_DEFAULTS  12
#define PITCH_SPEED_KI_DEFAULTS  0
#define PITCH_SPEED_KD_DEFAULTS  25

#define GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	PITCH_POSITION_KP_DEFAULTS,\
	PITCH_POSITION_KI_DEFAULTS,\
	PITCH_POSITION_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

//gimbal speed pid control
#define GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	PITCH_SPEED_KP_DEFAULTS,\
	PITCH_SPEED_KI_DEFAULTS,\
	PITCH_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

//gimbal yaw position pid control
#define GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	YAW_POSITION_KP_DEFAULTS,\
	YAW_POSITION_KI_DEFAULTS,\
	YAW_POSITION_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	600,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

//gimbal yaw speed pid control
#define GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	YAW_SPEED_KP_DEFAULTS,\
	YAW_SPEED_KI_DEFAULTS,\
	YAW_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	20.0f,\
	0.0f,\
	16.0f,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\



#define CAN_BUS1_YAW_MOTOR_FEEDBACK_MSG_ID             0x205
#define CAN_BUS1_PITCH_MOTOR_FEEDBACK_MSG_ID           0x206

#define CAN_BUS2_ZGYRO_FEEDBACK_MSG_ID                 0x401

#define PREPARE_TIME_TICK_MS                4000      //prapare time in ms
#define PREPARE_STEP4_TIME_TICK_MS          3000    //校准陀螺仪
#define PREPARE_STEP3_TIME_TICK_MS          2500    //校准陀螺仪
#define PREPARE_STEP2_TIME_TICK_MS          1500       //yaw移动至前端
#define PREPARE_STEP1_TIME_TICK_MS          500         //pitch抬至水平



typedef enum
{
	MOUDLE_GIMBAL = 1,
	MOUDLE_SHOOTER = 2,
}CtrlMoudleType_e;

typedef enum
{
	AUTOMOTIVE = 1,
	STOP = 2,
	REMOTE_CTRL = 3,
}InputMode_e;


typedef enum
{
    PREPARE_STATE,     		//上电后初始化状态 4s钟左右
    STANDBY_STATE,			//云台停止不转状态，无遥控控制
    NORMAL_STATE,			//正常状态，遥控器控制
    STOP_STATE,        	    //停止运动状态
    CALI_STATE,    			//校准状态
}WorkState_e;

typedef enum 
{
    PREPARE_STATE_STEP1 = 0,        //pitch抬头至水平
    PREPARE_STATE_STEP2 = 1,        //yaw移动至正中
    PREPARE_STATE_STEP3 = 2,        //校准陀螺仪
    PREPARE_STATE_STEP4 = 3         //陀螺仪校准成功
}SubWorkState_e;



typedef struct
{
	InputMode_e ctrl_mode;
//	CtrlMoudleType_e moudle_type;
	uint16_t    ctrl_cycle;    //控制信号发送周期
    float pitch_angle_dynamic_ref;
    float yaw_angle_dynamic_ref;
    float pitch_angle_static_ref;
    float yaw_angle_static_ref;
    float pitch_speed_ref;
    float yaw_speed_ref;
}Gimbal_Ctrl_t;

extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;
extern uint16_t chassis_rotate_lock;

extern void GimabalCtrl_Task(void *param);
extern WorkState_e GetWorkState(void);
extern QueueHandle_t gimbalCtrlQueue_Get(void);
extern void Can1_ReceiveGimbalMsgProcess(CanRxMsg * msg);
extern void Can2_ReceiveGyroMsgProcess(CanRxMsg * msg);
void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq,int16_t shoot_iq);

#endif

