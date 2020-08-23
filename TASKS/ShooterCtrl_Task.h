#ifndef __SHOOTERCTRL_H_
#define __SHOOTERCTRL_H_

#include "stm32f4xx.h"
#include "includes.h"





#define SHOOT_MOTOR_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	4.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	4900,\
	3500,\
	1500,\
	0,\
	800,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define SHOOT_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	12.0f,\
	0.0f,\
	8.0f,\
	0,\
	0,\
	0,\
	6500,\
	5000,\
	1000,\
	0,\
	30000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define CAN_BUS1_SHOOT_MOTOR_FEEDBACK_MSG_ID           (0x201)
#define SHOOT_MOTOR_REDUCTION_RATIO                    (36)

#define FRIC_WHEEL_SPEED_STOP               (1000)
#define FRIC_WHEEL_SPEED_ON                 (1250)

#define SHOOT_SPEED_HIGH                    (-1000)
#define SHOOT_SPEED_MID                     (-660)
#define SHOOT_SPEED_LOW                     (-350)

#define BULLET_OVERTIME_TIME                (1500)


typedef enum
{
	ZERO = 0,
	SINGLE = 1,
	DOUBLE = 2,
	TRIBLE = 3,
	QUATARY = 4,
	PENTA = 5,
	CONTINUATION = 10,
}ShootMode_e;

typedef enum
{
	BULLET_LOADING = 0,
	BULLET_LOADED = 1,
	BULLET_SHOOTINE = 2,
	BULLET_SHOOTED = 3,
	CONTINUATION_SHOOTINE = 4,
    OVERTIME = 5,
}ShootState_e;

typedef enum
{
	KEY_DOWN = 0,
	KEY_UP = 1,
}ShootKeyState_e;

typedef enum
{
	FRIC_WHEEL_OFF = 0,
	FRIC_WHEEL_ON = 1,
	FRIC_WHEEL_INVALID = 2,
}FricWheelMode_e;



typedef struct
{
	InputMode_e ctrl_mode;
	ShootMode_e shoot_mode;
	FricWheelMode_e frci_wheel_mode;

}Shooter_Ctrl_t;


extern void ShooterCtrl_Task(void *param);
void Set_Shooter_Current(CAN_TypeDef *CANx, int16_t shoot_iq);
extern QueueHandle_t shooterCtrlQueue_Get(void);
extern void Can1_ReceiveShooterMsgProcess(CanRxMsg * msg);

#endif

