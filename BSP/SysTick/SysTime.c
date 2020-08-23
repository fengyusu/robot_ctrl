//******************************************************************************************
//!
//! \file   SoftTimer.c
//! \brief  SoftTimer Implement File
//!         This module is used to replace 'delay' function in state machine
//! Copyright (c) 2014 Cedar MIT License
//!
//! Permission is hereby granted, free of charge, to any person obtaining a copy
//! of this software and associated documentation files (the "Software"), to deal
//! in the Software without restriction, including without limitation the rights to
//! use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
//! the Software, and to permit persons to whom the Software is furnished to do so,
//! subject to the following conditions:
//!
//! The above copyright notice and this permission notice shall be included in all
//! copies or substantial portions of the Software.
//!
//! THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//! IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//! FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//! AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//! LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//! OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//! IN THE SOFTWARE.
///
//******************************************************************************************

#include "SysTime.h"

uint32_t systemCounter;


//******************************************************************************************
//!                     Typedef
//******************************************************************************************
typedef struct
{
    uint16_t handle;                                       // Timer Resource Handle
    uint32_t delay;                                        // Delay Value    
}TimerElem_t;


//******************************************************************************************
//!                     Private Variable
//******************************************************************************************

//static volatile uint32_t TimerCnt    = 0;
//static volatile uint16_t HandleIndex = 0;
//static TimerElem_t       SoftTimer[TIMER_ELEMENT_NUM_MAX];

TimerElem_t       SoftTimer[TIMER_ELEMENT_NUM_MAX] = {0};



//******************************************************************************************
//!                     Function Implement
//******************************************************************************************



//if other timer is used, the default Init and Destroy function is not defined 
#ifndef __OTHER_TIMER_USED 
// 1ms interval
void SysTick_Init(void)
{
    /* SystemFrequency / 1000    1ms中断一次
	 * SystemFrequency / 100000	 10us中断一次
	 * SystemFrequency / 1000000 1us中断一次
	 */
	if (SysTick_Config(SystemCoreClock / 1000000))
	{ 
		/* Capture error */ 
		while (1);
	}
}


static void HardTimer_Destroy(void)
{    
    
}
#endif 

//******************************************************************************************
//
//! \brief  Initialize Timer resource
//!
//! \param  None.
//! \retval 
//!         - SOFT_TIMER_SUCCESS
//!
//! \note
//!         - This function must be called first !.
//!         
//
//******************************************************************************************
uint16_t SoftTimer_Init(void)
{
    uint8_t i = 0;

    // Clear All Elements
    for(i = 1; i < TIMER_ELEMENT_NUM_MAX; i++)
    {
        SoftTimer[i].handle    = 0;
        SoftTimer[i].delay     = 0;        
    }

#ifndef __OTHER_TIMER_USED
    SysTick_Init();
#endif 
		
    return (SOFT_TIMER_SUCCESS);
}

uint16_t SoftTimer_Destroy(void)
{
    uint8_t i = 0;

    // Clear All Elements
    for(i = 1; i < TIMER_ELEMENT_NUM_MAX; i++)
    {
        SoftTimer[i].handle    = 0;
        SoftTimer[i].delay     = 0;        
    }

//if other timer is used, this function is not called 		
#ifndef __OTHER_TIMER_USED 
    HardTimer_Destroy();
#endif 
		
    return (SOFT_TIMER_SUCCESS);
}


//******************************************************************************************
//
//! \brief  Request Timer resource
//!
//! \param  [in] Tick is time eclipse value.
//! \retval 
//!         - Zero                     Operate Failure, No Timer Available
//!         - Non-Zero                 Valid Timer Handle
//!
//! \note
//!         - Timer handle only can be used once.
//!         
//
//******************************************************************************************
uint16_t SoftTimer_Req(uint32_t Tick)
{
    uint8_t i = 0;

    for (i = 1; i < TIMER_ELEMENT_NUM_MAX; i++)
    {
        if (SoftTimer[i].handle == 0)
        {
            CRITICAL_SETCION_ENTER();

            SoftTimer[i].handle    = i;
            SoftTimer[i].delay     = Tick;            

            CRITICAL_SETCION_EXIT();

            return (i);
        }
    }

    return (0);
}

//******************************************************************************************
//
//! \brief  Update Timer
//!
//! \param  [in] id is timer handler
//! \retval 
//!         - Zero                     success
//!         - Non-Zero                 failure
//!         
//
//******************************************************************************************
uint16_t SoftTimer_Update(uint32_t Id, uint32_t Tick)
{
    for (uint8_t i = 1; i < TIMER_ELEMENT_NUM_MAX; i++)
    {
        if (SoftTimer[i].handle == Id)
        {
            CRITICAL_SETCION_ENTER();

            SoftTimer[i].delay = Tick;

            CRITICAL_SETCION_EXIT();

            // Success
            return (0);
        }
    }

    // Failure
    return (1);
}

//******************************************************************************************
//
//! \brief  Destory Timer
//!
//! \param  [in] id is timer handler
//! \retval 
//!         - Zero                     success
//!         - Non-Zero                 failure
//!         
//
//******************************************************************************************
uint16_t SoftTimer_UnReg(uint32_t Id)
{
    for (uint8_t i = 1; i < TIMER_ELEMENT_NUM_MAX; i++)
    {
        if (SoftTimer[i].handle == Id)
        {
            CRITICAL_SETCION_ENTER();

            SoftTimer[i].handle    = 0;
            SoftTimer[i].delay     = 0;            

            CRITICAL_SETCION_EXIT();

            // Success
            return (0);
        }
    }

    // Failure
    return (1);
}


//******************************************************************************************
//
//! \brief  Check Timer status
//!         You can check register timer status at any time.
//!
//! \param  [in] Handle is Timer Handle, which you can get it from \ref SoftTimer_Req.
//! \retval 
//!         - \ref SOFT_TIMER_ING      Timer Counting
//!         - \ref SOFT_TIMER_TIMEOUT  Timer TimeOut
//!         - \ref SOFT_TIMER_ERR      Invalid Timer Handle
//!
//! \note
//!         - You must call \ref SoftTimer_Req to request an valid timer handle.
//!         - Timer handle only can be used once.
//!         
//
//******************************************************************************************
uint16_t SoftTimer_Check(uint16_t Handle)
{    
    uint16_t retval = SOFT_TIMER_ERR;

    CRITICAL_SETCION_ENTER();    
    if(SoftTimer[Handle].handle ==  Handle)
    {
        if(SoftTimer[Handle].delay)
        {
            retval = SoftTimer[Handle].delay;
        }
        else
        {
            retval = SOFT_TIMER_TIMEOUT;
        }
    }
    CRITICAL_SETCION_EXIT();

    return (retval);
}

//******************************************************************************************
//
//! \brief  SoftTimer Hook Function
//!         This callback function must be called interval
//!
//! \note   Typical 1ms interval
//
//******************************************************************************************
void TimerISR_Hook(void)
{    
    for(uint8_t i = 1; i < TIMER_ELEMENT_NUM_MAX; i++)
    {
        if(SoftTimer[i].handle != 0)
        {
            if(SoftTimer[i].delay)
            {
                SoftTimer[i].delay--;
            }
        }
    }
}

//if other timer is used, this isr handler is not defined  
#ifndef __OTHER_TIMER_USED

/**
  * @brief  获取节拍程序
  * @param  无
  * @retval 无
  * @attention  在 SysTick 中断函数 SysTick_Handler()调用
  */
 

#endif 
