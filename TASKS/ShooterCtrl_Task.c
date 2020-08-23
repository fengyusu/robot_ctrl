#include "includes.h"
#include "Protocol_Task.h"
#include "bsp_can1.h"
#include "encoder.h"
#include <stdlib.h>
#include <math.h>


PID_Regulator_t ShootMotorPositionPID = SHOOT_MOTOR_POSITION_PID_DEFAULT;      //shoot motor
PID_Regulator_t ShootMotorSpeedPID    = SHOOT_MOTOR_SPEED_PID_DEFAULT;

volatile Encoder GMShootEncoder = {0,0,0,0,0,0,0,0,0};

static QueueHandle_t shooterCtrlQueue = NULL;


Shooter_Ctrl_t ShooterCtrl;

uint32_t shooter_task_tick = 0;



QueueHandle_t shooterCtrlQueue_Get(void)
{
    return shooterCtrlQueue;
}

void ShooterParamInit(void)
{
    shooter_task_tick = 0;
	

	ShooterCtrl.ctrl_mode = REMOTE_CTRL;
	ShooterCtrl.shoot_mode = ZERO;
	ShooterCtrl.frci_wheel_mode = FRIC_WHEEL_OFF;

	
	ShootMotorSpeedPID.Reset(&ShootMotorSpeedPID);
	ShootMotorSpeedPID.Reset(&ShootMotorPositionPID);
	
	Set_Shooter_Current(CAN1, 0);
}



void Can1_ReceiveShooterMsgProcess(CanRxMsg * msg)
{      
    static uint32_t can1_shooter_count = 0;
	
	switch(msg->StdId)
	{
		case CAN_BUS1_SHOOT_MOTOR_FEEDBACK_MSG_ID:         //²¦µ¯µç»ú
		{
			can1_shooter_count++;
		    (can1_shooter_count<=50) ? GetEncoderBias(&GMShootEncoder ,msg):EncoderProcess(&GMShootEncoder ,msg);
		}break;
			
		
		default:
			break;

	}
	 
}


void Set_Shooter_Current(CAN_TypeDef *CANx, int16_t shoot_iq)
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(shoot_iq >> 8);
    tx_message.Data[1] = (unsigned char)shoot_iq;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
	
	
}

void SetShooterMotorOutput(void)
{
							
	if(GetWorkState() == STOP_STATE)
	{
	    Set_Shooter_Current(CAN1, 0);     // bullet	
	}
	else
	{
		Set_Shooter_Current(CAN1, (int16_t)ShootMotorSpeedPID.output); 
	}
}

uint32_t mouse_press_time;
uint32_t bullet_shoot_time;
uint32_t mouse_shoot_time_diff;

uint8_t fric_wheel_state;
uint8_t bullet_ready = 0;
uint32_t bullet_block_time = 0;
uint8_t bullet_blocked;
float shoot_motor_angle;
ShootState_e shoot_state = BULLET_LOADING;
ShootKeyState_e key_state,key_last_state;
void ShooterControlLoop(void)
{

	
	switch(ShooterCtrl.shoot_mode)
	{
		case ZERO:	
		{
		    if(shoot_state == CONTINUATION_SHOOTINE)
			{
			    shoot_state = BULLET_LOADING;
			}
		}break;
		
		case SINGLE:
		case DOUBLE:
		case TRIBLE:
		case QUATARY:
		case PENTA:
		{
		    if(shoot_state == CONTINUATION_SHOOTINE)
			{
			    shoot_state = BULLET_LOADING;
			}
			bullet_ready = ShooterCtrl.shoot_mode;
			ShooterCtrl.shoot_mode = ZERO;
		}break;
		
		case CONTINUATION:
		{
			shoot_state = CONTINUATION_SHOOTINE;
		}break;
	}
	
	key_last_state = key_state;
	key_state = GET_SHOOT_KEY();
	
	if(ShooterCtrl.frci_wheel_mode == FRIC_WHEEL_OFF)// && (shoot_state == BULLET_LOADING || shoot_state == BULLET_LOADED))
	{
		SetFrictionWheelSpeed(FRIC_WHEEL_SPEED_STOP);
		
		shoot_state = BULLET_LOADING;
		bullet_ready = 0;
		bullet_block_time = 0;
		ShootMotorSpeedPID.ref = 0;
	}
	else if(ShooterCtrl.frci_wheel_mode == FRIC_WHEEL_ON)
	{
		SetFrictionWheelSpeed(FRIC_WHEEL_SPEED_ON);
		
		if(bullet_ready && shoot_state != CONTINUATION_SHOOTINE)
		{
		    if(++bullet_block_time > BULLET_OVERTIME_TIME)
			{
			    bullet_ready = 0;
				bullet_block_time = 0;
				ShootMotorSpeedPID.ref = 0;
				shoot_state = OVERTIME;
				shoot_motor_angle = GMShootEncoder.ecd_angle;
			}
		}
		
		switch(shoot_state)
		{
		    case BULLET_LOADING:
			{
				if(key_state == KEY_UP)
				{
					ShootMotorSpeedPID.ref = SHOOT_SPEED_LOW;
					bullet_ready = 0;
				}
				else if(key_state == KEY_DOWN)
				{
				    ShootMotorSpeedPID.ref = 0;
					shoot_state = BULLET_LOADED;
				}
			}break;
		
			case BULLET_LOADED:
			{
				if(key_state == KEY_DOWN)
				{
					ShootMotorSpeedPID.ref = 0;
				    if(bullet_ready)
					{
						shoot_state = BULLET_SHOOTINE;
					}
				}
				else //error
				{
				    shoot_state = BULLET_LOADING;
				}
			}break;
			
		    case BULLET_SHOOTINE:
			{
				if(key_state == KEY_DOWN)
				{
					if(bullet_ready > 1)
					{
				        ShootMotorSpeedPID.ref = SHOOT_SPEED_HIGH;
					}
					else if(bullet_ready == 1)
					{
						ShootMotorSpeedPID.ref = SHOOT_SPEED_MID;
					}
					else
					{
						ShootMotorSpeedPID.ref = 0;
						shoot_state = BULLET_LOADING;
					}
				}
				else if((key_last_state == KEY_DOWN) && (key_state == KEY_UP))
				{
				    shoot_state = BULLET_SHOOTED;
					if(bullet_ready)
					{
						bullet_ready--;
						bullet_block_time = 0;
					}
				}
				else //error
				{
				    shoot_state = BULLET_LOADING;
				}
			}break;
		
			case BULLET_SHOOTED:
			{
				bullet_shoot_time = ulHighFrequencyTimerTicks;
				if(bullet_shoot_time > mouse_press_time)
				{
				    mouse_shoot_time_diff = bullet_shoot_time - mouse_press_time;
				}
			    if(key_state == KEY_UP)
				{
					if(bullet_ready)
					{
				        ShootMotorSpeedPID.ref = SHOOT_SPEED_MID;
					}
					else
					{
						ShootMotorSpeedPID.ref = SHOOT_SPEED_LOW;
					}
				}
				else if((key_last_state == KEY_UP) && (key_state == KEY_DOWN))
				{
				    ShootMotorSpeedPID.ref = 0;
					shoot_state = BULLET_LOADED;
				}
				else //error
				{
				    shoot_state = BULLET_LOADING;
				}
			}break;
			
			case CONTINUATION_SHOOTINE:
			{
				bullet_ready = 0;
				
				if(bullet_blocked)
				{
					ShootMotorPositionPID.ref = shoot_motor_angle + 20 * SHOOT_MOTOR_REDUCTION_RATIO;
					ShootMotorPositionPID.fdb = GMShootEncoder.ecd_angle;
					ShootMotorPositionPID.Calc(&ShootMotorPositionPID);
					ShootMotorSpeedPID.ref = ShootMotorPositionPID.output;
					if(fabs(ShootMotorPositionPID.ref - ShootMotorPositionPID.fdb) < 5)
					{
						ShootMotorSpeedPID.ref = 0;
					    bullet_blocked = 0;
					}
				}
				else
				{
					ShootMotorSpeedPID.ref = SHOOT_SPEED_HIGH;
					
					shoot_motor_angle = GMShootEncoder.ecd_angle;
					if(fabs(ShootMotorSpeedPID.fdb) < 5)
					{
					    if(++bullet_block_time > 600)
						{
							ShootMotorSpeedPID.ref = 0;
							bullet_block_time = 0;
							bullet_blocked = 1;
						}
					}
				}
			}break;
			
			case OVERTIME:
			{
				ShootMotorPositionPID.ref = shoot_motor_angle + 20 * SHOOT_MOTOR_REDUCTION_RATIO;
				ShootMotorPositionPID.fdb = GMShootEncoder.ecd_angle;
				ShootMotorPositionPID.Calc(&ShootMotorPositionPID);
				ShootMotorSpeedPID.ref = ShootMotorPositionPID.output;
				if(fabs(ShootMotorPositionPID.ref - ShootMotorPositionPID.fdb) < 5)
				{
					ShootMotorSpeedPID.ref = 0;
					shoot_state = BULLET_LOADING;
				}
			}break;
		}
				
	}
	
	
//	ShootMotorSpeedPID.ref = GimbalCtrl.pitch_speed_ref;
	
	
	ShootMotorSpeedPID.fdb = GMShootEncoder.filter_rate;  
	ShootMotorSpeedPID.Calc(&ShootMotorSpeedPID);

//	ShootMotorSpeedPID.output = -800;
	SetShooterMotorOutput();
}

ShootKeyState_e mouse_state,mouse_last_state;

void ShooterCtrl_Task(void *param)
{
	BaseType_t xResult; 
	const TickType_t xFrequency = 1;
	const TickType_t xTicksToWait = 2 / portTICK_PERIOD_MS;
	
	
	shooterCtrlQueue = xQueueCreate(10, sizeof(Shooter_Ctrl_t));
	if(shooterCtrlQueue == 0){}
	
	ShooterParamInit();
	vTaskDelay(1000);
	
    while(1)
	{
		mouse_last_state = mouse_state;
		mouse_state = GET_MOUSE_KEY();
	 	if(mouse_state == KEY_UP && mouse_last_state == KEY_DOWN)
		{
		    mouse_press_time = ulHighFrequencyTimerTicks;
		}
		
		xResult = xQueueReceive(shooterCtrlQueue_Get(), &ShooterCtrl, (TickType_t)xFrequency);
		if(xResult == pdPASS)
		{
			vTaskDelay(xFrequency);
		}
		
		shooter_task_tick ++;
			
		ShooterControlLoop();
	    
	}
}
