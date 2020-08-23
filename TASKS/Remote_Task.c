#include "includes.h"

static QueueHandle_t remoteQueue = NULL;

RC_Ctl_t RC_CtrlData; 
RC_Ctl_t RC_LastCtrlData; 
KB_Ctl_t KB_CtlData;
Key_Mouse_State_t  key_mouse_state;

Gimbal_Ctrl_t GimbalCtrl_By_Remote;
Shooter_Ctrl_t ShooterCtrl_By_Remote;
Chassis_Ctrl_t ChassisCtrl_By_Gimbal;

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 

void RemoteDataPrcess(uint8_t *pData)
{
    if(pData == NULL)
    {
        return;
    }
	
	memcpy(&RC_LastCtrlData, &RC_CtrlData, sizeof(RC_Ctl_t));
    
    RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
    RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                         ((int16_t)pData[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
    
    RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

    RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

    RC_CtrlData.mouse.press_l = pData[12];
    RC_CtrlData.mouse.press_r = pData[13];
 
    RC_CtrlData.key.key_code = ((uint16_t)(pData[14] | pData[15] << 8));
}

uint8_t getKBState(void)
{
    if(RC_CtrlData.mouse.x == 0 && RC_CtrlData.mouse.y == 0 && RC_CtrlData.mouse.z == 0 && \
	   RC_CtrlData.mouse.press_l == 0 && RC_CtrlData.mouse.press_r == 0 && \
	   RC_CtrlData.key.key_code == 0)
	{
	    return KB_NO_ACTION;
	}
	else
	{
	    return KB_ACTION;
	}

}

void KeyMouseFsm(uint8_t *p_state, uint8_t input)
{
	uint8_t confirm_counter = ((*p_state)&0xf0)>>4;
	switch ((*p_state)&0x0f)
	{
		case IDLE:
		{
			if(input)
			{
				*p_state &= 0xf0;
				*p_state |= DOWN_FIRST;
			}
		}break;
		
		case DOWN_FIRST:
		{
			if(confirm_counter > 12)
			{
				*p_state &= 0xf0;
				*p_state |= LONG_PRESS;
				confirm_counter = 0;
			}
			else
			{
				if(input)
				{
					confirm_counter++;
				}
				else 
				{
					*p_state &= 0xf0;
					*p_state |= UP_FIRST;
					confirm_counter = 0;
				}
		  }
		}break;
		
		case UP_FIRST:
		{
			if(confirm_counter > 8)
			{
				*p_state &= 0xf0;
				*p_state |= CLICK;
				confirm_counter = 0;
			}
			else
			{
				if(!input)
				{
					confirm_counter++;
				}
				else 
				{
					*p_state &= 0xf0;
					*p_state |= DOWN_SECOND;
					confirm_counter = 0;
				}
			}
		}break;

		case DOWN_SECOND:
		{
			if(confirm_counter > 12)
			{
				*p_state &= 0xf0;
				*p_state |= LONG_PRESS;
				confirm_counter = 0;
			}
			else
			{
				if(input)
				{
					confirm_counter++;
				}
				else 
				{
					*p_state &= 0xf0;
					*p_state |= DOUBLE_CLICK;
					confirm_counter = 0;
				}
		  }
		}break;
		
		case CLICK:
		{
			*p_state &= 0xf0;
			*p_state |= IDLE;
			confirm_counter = 0;
		}break;
		
		case DOUBLE_CLICK:
		{
			*p_state &= 0xf0;
			*p_state |= IDLE;
			confirm_counter = 0;
		}break;
		
		case LONG_PRESS:
		{
            if(!input)
			{
				*p_state &= 0xf0;
				*p_state |= LONG_PRESS_UP;
			}		
		}break;
		
		case LONG_PRESS_UP:
		{
            *p_state &= 0xf0;
			*p_state |= IDLE;
			confirm_counter = 0;		
		}break ;

		default:
			break;
	}
	
	*p_state &= 0x0f;
	*p_state |= (confirm_counter<<4);
}

void updateKBFsm(void)
{
    KeyMouseFsm(&key_mouse_state.mouse_l,RC_CtrlData.mouse.press_l);
    KeyMouseFsm(&key_mouse_state.mouse_r,RC_CtrlData.mouse.press_r);
	KeyMouseFsm(&key_mouse_state.W,RC_CtrlData.key.key_bit.W);
	KeyMouseFsm(&key_mouse_state.S,RC_CtrlData.key.key_bit.S);
	KeyMouseFsm(&key_mouse_state.A,RC_CtrlData.key.key_bit.A);
	KeyMouseFsm(&key_mouse_state.D,RC_CtrlData.key.key_bit.D);
	KeyMouseFsm(&key_mouse_state.Q,RC_CtrlData.key.key_bit.Q);
	KeyMouseFsm(&key_mouse_state.E,RC_CtrlData.key.key_bit.E);
	KeyMouseFsm(&key_mouse_state.V,RC_CtrlData.key.key_bit.V);
	KeyMouseFsm(&key_mouse_state.SHIFT,RC_CtrlData.key.key_bit.SHIFT);
	KeyMouseFsm(&key_mouse_state.CTRL,RC_CtrlData.key.key_bit.CTRL);
}

uint32_t shoot_ctrl_mode_counter = 0;
void getRemoteRef(void)
{

	
    GimbalCtrl_By_Remote.pitch_speed_ref = (RC_CtrlData.rc.ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) + (-RC_CtrlData.mouse.y);
	GimbalCtrl_By_Remote.yaw_speed_ref = (RC_CtrlData.rc.ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) + (RC_CtrlData.mouse.x);
    
    ////云台控制数据
	GimbalCtrl_By_Remote.pitch_angle_dynamic_ref -= ((RC_CtrlData.rc.ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT) + \
	                                                (-RC_CtrlData.mouse.y * MOUSE_TO_PITCH_ANGLE_INC_FACT);
	GimbalCtrl_By_Remote.yaw_angle_dynamic_ref   += ((RC_CtrlData.rc.ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_MOTION_FACT) + \
													(RC_CtrlData.mouse.x * MOUSE_TO_YAW_ANGLE_INC_FACT);
//	VAL_LIMIT(GimbalCtrl_By_Remote.pitch_angle_dynamic_ref, -PITCH_MAX-10, PITCH_MAX+16);
	VAL_LIMIT(GimbalCtrl_By_Remote.pitch_angle_dynamic_ref, -PITCH_MAX-5, PITCH_MAX+8);
	
	////发射机构控制数据
	if(RC_CtrlData.rc.s1 == RC_DN && RC_LastCtrlData.rc.s1 == RC_MI)
	{
	    ShooterCtrl_By_Remote.shoot_mode = SINGLE;
		shoot_ctrl_mode_counter = 0;
	}
	else if(RC_CtrlData.rc.s1 == RC_DN)
	{
	    if(++shoot_ctrl_mode_counter >= 100)
		{
			ShooterCtrl_By_Remote.shoot_mode = CONTINUATION;
			shoot_ctrl_mode_counter = 100;
		}
		else
		{
		    ShooterCtrl_By_Remote.shoot_mode = ZERO;
		}
	}
	else
	{
	    ShooterCtrl_By_Remote.shoot_mode = ZERO;
		shoot_ctrl_mode_counter = 0;
	}
	
	if(RC_CtrlData.rc.s1 == RC_UP && RC_LastCtrlData.rc.s1 == RC_MI)
	{
		if(ShooterCtrl_By_Remote.frci_wheel_mode == FRIC_WHEEL_OFF)
		{
	        ShooterCtrl_By_Remote.frci_wheel_mode = FRIC_WHEEL_ON;
            LASER_ON();
		}
		else if(ShooterCtrl_By_Remote.frci_wheel_mode == FRIC_WHEEL_ON)
		{
		    ShooterCtrl_By_Remote.frci_wheel_mode = FRIC_WHEEL_OFF;
            LASER_OFF();
		}
	}
	
	if(key_mouse_state.mouse_l != IDLE)
	{
		if(((key_mouse_state.mouse_l & 0x0f) == DOWN_FIRST) && (((key_mouse_state.mouse_l & 0xf0) >> 4) == 1))
		{
			ShooterCtrl_By_Remote.shoot_mode = SINGLE;
		}
		else if(((key_mouse_state.mouse_l & 0x0f) == DOWN_SECOND) && (((key_mouse_state.mouse_l & 0xf0) >> 4) == 1))
		{
			ShooterCtrl_By_Remote.shoot_mode = QUATARY;
		}
		else if(key_mouse_state.mouse_l == LONG_PRESS)
		{
			ShooterCtrl_By_Remote.shoot_mode = CONTINUATION;
		}
		else 
		{
			ShooterCtrl_By_Remote.shoot_mode = ZERO;
		}
    }
	
	if(key_mouse_state.mouse_r == LONG_PRESS)
	{
	    if(ShooterCtrl_By_Remote.frci_wheel_mode == FRIC_WHEEL_ON)
		{
		    ShooterCtrl_By_Remote.frci_wheel_mode = FRIC_WHEEL_OFF;
            LASER_OFF();
		}
	}
	else if(key_mouse_state.mouse_r == CLICK)
	{
	    ShooterCtrl_By_Remote.frci_wheel_mode = FRIC_WHEEL_ON;
        LASER_ON();
	}
	
	
	
	////底盘控制数据
	if(RC_CtrlData.key.key_bit.W)
	{
	    KB_CtlData.chassis_vy += 10;
	}
	else if(RC_CtrlData.key.key_bit.S)
	{
	     KB_CtlData.chassis_vy -= 10;
	}
	else
	{
	     KB_CtlData.chassis_vy = 0;
	}
	
	if(RC_CtrlData.key.key_bit.A)
	{
	    KB_CtlData.chassis_vx -= 10;
	}
	else if(RC_CtrlData.key.key_bit.D)
	{
		KB_CtlData.chassis_vx += 10;
	}
	else
	{
	    KB_CtlData.chassis_vx = 0;
	}
	
	if(key_mouse_state.SHIFT == LONG_PRESS)
	{
		KB_CtlData.chassis_speed_mode = MODE_HIGH_SPEED;
	}
	else
	{
		KB_CtlData.chassis_speed_mode = MODE_LOW_SPEED;
	}
	
	if(KB_CtlData.chassis_speed_mode == MODE_HIGH_SPEED)
	{
		VAL_LIMIT(KB_CtlData.chassis_vx, -CHASSIS_SPEED_HIGH, CHASSIS_SPEED_HIGH);
	    VAL_LIMIT(KB_CtlData.chassis_vy, -CHASSIS_SPEED_HIGH, CHASSIS_SPEED_HIGH);
	}
	else if(KB_CtlData.chassis_speed_mode == MODE_LOW_SPEED)
	{
		VAL_LIMIT(KB_CtlData.chassis_vx, -CHASSIS_SPEED_LOW, CHASSIS_SPEED_LOW);
	    VAL_LIMIT(KB_CtlData.chassis_vy, -CHASSIS_SPEED_LOW, CHASSIS_SPEED_LOW);
	}
	
	CMRotatePID.ref = 0;
	CMRotatePID.fdb = GMYawEncoder.ecd_angle;
    CMRotatePID.Calc(&CMRotatePID);   
	if(chassis_rotate_lock)
	{
		ChassisCtrl_By_Gimbal.rotate_ref = CMRotatePID.output;
	}
	else
	{
		ChassisCtrl_By_Gimbal.rotate_ref = 0;
	}
//	ChassisCtrl_By_Gimbal.rotate_ref = RC_CtrlData.rc.ch2 - REMOTE_CONTROLLER_STICK_OFFSET;
	ChassisCtrl_By_Gimbal.forward_back_ref = (RC_CtrlData.rc.ch1 - REMOTE_CONTROLLER_STICK_OFFSET) + KB_CtlData.chassis_vy;
	ChassisCtrl_By_Gimbal.left_right_ref   = (RC_CtrlData.rc.ch0 - REMOTE_CONTROLLER_STICK_OFFSET) + KB_CtlData.chassis_vx;
}

void clearRemoteRef(void)
{
    GimbalCtrl_By_Remote.pitch_speed_ref = 0;
	GimbalCtrl_By_Remote.yaw_speed_ref   = 0;	
	GimbalCtrl_By_Remote.pitch_angle_dynamic_ref = 0;
	GimbalCtrl_By_Remote.yaw_angle_dynamic_ref   = 0;
	GimbalCtrl_By_Remote.pitch_angle_static_ref = 0;
	GimbalCtrl_By_Remote.yaw_angle_static_ref   = 0;
	
	ShooterCtrl_By_Remote.frci_wheel_mode = FRIC_WHEEL_OFF;
	ShooterCtrl_By_Remote.shoot_mode = ZERO;
	
	ChassisCtrl_By_Gimbal.forward_back_ref = 0;
    ChassisCtrl_By_Gimbal.left_right_ref = 0;
    ChassisCtrl_By_Gimbal.rotate_ref = 0;
}

uint8_t RemoteCmdProcess(void)
{
	if(abs(RC_CtrlData.rc.ch0 - 1024) > 660)
	{
	    return 1;
	}
	
	switch(RC_CtrlData.rc.s2)
	{
	    case 1:
		{
		    GimbalCtrl_By_Remote.ctrl_mode = AUTOMOTIVE;
			
			ShooterCtrl_By_Remote.ctrl_mode = AUTOMOTIVE;
			
			ChassisCtrl_By_Gimbal.input_mode = AUTOMOTIVE;
			
		}break;
		
		case 2:
		{
		    GimbalCtrl_By_Remote.ctrl_mode = STOP;
			
			ShooterCtrl_By_Remote.ctrl_mode = STOP;
			
			ChassisCtrl_By_Gimbal.input_mode = STOP;
			
			clearRemoteRef();			
		}break;

        case 3:
		{
		    GimbalCtrl_By_Remote.ctrl_mode = REMOTE_CTRL;
			GimbalCtrl_By_Remote.ctrl_cycle = 10;
			
			ShooterCtrl_By_Remote.ctrl_mode = REMOTE_CTRL;
			
			ChassisCtrl_By_Gimbal.input_mode = REMOTE_CTRL;
			ChassisCtrl_By_Gimbal.ctrl_mode = SPEED;
			
			if(GetWorkState() != PREPARE_STATE)
			{
				getRemoteRef();
			}
			else 
			{
				clearRemoteRef();
			}
		}break;		
		
		default:
			return 1;
	}
	
	
	return 0;
}

void Remote_Task(void *param)
{
	BaseType_t xResult;
	EventBits_t uxBits;
	static uint8_t remoteBuff[RC_FRAME_LENGTH];

	
    remoteQueue = xQueueCreate(10, RC_FRAME_LENGTH);
	if(remoteQueue == 0){}
	
    while(1)
	{
		xResult = xQueueReceive(remoteQueue, remoteBuff, portMAX_DELAY);
		if(xResult == pdPASS)
		{	
			uxBits = xEventGroupSetBits(xGlobalEventGroup, EVENT_REMOTE_REV);
			if((uxBits & EVENT_REMOTE_REV) != 0){}	
				
			RemoteDataPrcess(remoteBuff);
			
            KB_CtlData.kb_state = getKBState();
			updateKBFsm();
				
			if(!RemoteCmdProcess())
			{
			    xQueueSend(gimbalCtrlQueue_Get(), (void *)&GimbalCtrl_By_Remote, (TickType_t)5);
				
				xQueueSend(shooterCtrlQueue_Get(), (void *)&ShooterCtrl_By_Remote, (TickType_t)5);
				
				Potocol_Transmit(MODULE_TYPE, 0, MODULE_TYPE_CAR_CHASSIS, 0, CMD_CHASSIS_CTRL, (uint8_t *)&ChassisCtrl_By_Gimbal, sizeof(Chassis_Ctrl_t));
			}
			



		}
	}
}

QueueHandle_t remoteQueue_Get(void)
{
    return remoteQueue;
}
