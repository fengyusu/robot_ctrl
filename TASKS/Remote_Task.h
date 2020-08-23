#ifndef __REMOTE_H_
#define __REMOTE_H_

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

#define PITCH_MAX		(20.0f)
#define YAW_MAX 		(720.0f)   

#define REMOTE_CONTROLLER_STICK_OFFSET      (1024u)
#define STICK_TO_PITCH_ANGLE_INC_FACT       (0.0048f)
#define STICK_TO_YAW_ANGLE_INC_FACT         (0.0048f)
#define STICK_TO_YAW_ANGLE_MOTION_FACT      (0.0032f)

#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		(0.029f)
#define MOUSE_TO_YAW_ANGLE_INC_FACT 		(0.045f)

#define CHASSIS_SPEED_LOW                   (400)
#define CHASSIS_SPEED_HIGH                  (800)

enum
{
	KB_ACTION = 0,
	KB_NO_ACTION = 1,
};

enum
{
	ROTATE_WITH_GIMBAL = 0,
	ROTATE_WITH_REMOTE = 1,
};

enum
{
	MODE_LOW_SPEED = 0,
	MODE_HIGH_SPEED = 1,
};

typedef __packed struct
{
	uint8_t kb_state; //0:有动作 1：无动作
	uint8_t chassis_mode;//0:随动 1:不随动
	uint8_t chassis_speed_mode;//0:低速  1:高速
	int16_t chassis_vx;
	int16_t chassis_vy;
	uint8_t shoot_mode;//0:没有动作 1:发一发子弹 2:发4发子弹  3:连续发射
}KB_Ctl_t;

typedef __packed struct
{
	uint8_t mouse_l;
	uint8_t mouse_r;
	uint8_t W;
	uint8_t S;
	uint8_t A;
	uint8_t D;
	uint8_t SHIFT;
	uint8_t CTRL;
	uint8_t Q;
	uint8_t E;
	uint8_t R;
	uint8_t F;
	uint8_t G;
	uint8_t Z;
	uint8_t X;
	uint8_t C;
	uint8_t V;
	uint8_t B;
}Key_Mouse_State_t;

typedef enum
{
	IDLE = 0,
	DOWN_FIRST,
	UP_FIRST,
	DOWN_SECOND,
	CLICK,
	DOUBLE_CLICK,
	LONG_PRESS,
	LONG_PRESS_UP,
}KB_Fsm_e;

enum
{
	RC_UP = 1,
	RC_MI = 3,
	RC_DN = 2,
};

typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote;

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;

typedef	__packed union 
{
	uint16_t key_code;
/**********************************************************************************
 * keyboard :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 *            V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 ************************************************************************************/
	__packed struct {
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;
	}key_bit;
}Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;

extern RC_Ctl_t RC_CtrlData; 

void RemoteDataPrcess(uint8_t *pData);
extern void Remote_Task(void *param);
QueueHandle_t remoteQueue_Get(void);

#endif

