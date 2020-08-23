//******************************************************************************************
//!
//! \file   SoftTimer.h
//! \brief  SoftTimer Interface File
//!         This module is used to replace 'delay' function in state machine
//! \author cedar
//! \date   2014-9-17
//! \email  xuesong5825718@gmail.com
//! \qq     819280802
//!
//! \license
//!
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

#ifndef __SOFT_TIMER_H__
#define __SOFT_TIMER_H__

#ifdef __cplusplus
extern "C"
{
#endif

//******************************************************************************************
//!                           Portable Configure
//******************************************************************************************

//! If your compiler support stdint.h file, then try to use it, otherwise using folling section
#if 0
typedef unsigned char      uint8_t ;
typedef unsigned short int uint16_t;
typedef unsigned int       uint32_t;
#else
#include <stdint.h>
#endif

#include "stm32f4xx.h"

	
	
	//******************************************************************************************
//!                     Macro Function
//******************************************************************************************
#define INT_STATE uint32_t
#define MASTER_INT_STATE_GET()         __get_PRIMASK()                    // ¡¤¦Ì??PRIMASK¦Ì??¦Ì
#define MASTER_INT_ENABLE()            do{__enable_irq();}while(0)        // CPSIE I  
#define MASTER_INT_DISABLE()           do{__disable_irq();}while(0)       // CPSID I
#define MASTER_INT_RESTORE(x)          do{__set_PRIMASK(x);}while(0)      // ??PRIMASK¨¦¨¨???ax


//******************************************************************************************
//!                               CRITICAL SECTION FUNCTIONS    
//Description: Disable/Enable interrupts  by preventing the state of interrupts. Generally 
//speaking, the state of the interrupt disable flag is stored in the local variable 'cpu_state'
//and interrupts are then disabled ('cpu_state' is allocated in all functions that need to 
//disable interrupts).The previous interrupt state is restored by copying 'cpu_state' into the 
//CPU's Exception mask register.
//
//
//; Note(s)     : 
//  These functions are used in general like this :
//
//                       void  Task (void  *p_arg)
//                       {
//                           CPU_CRITICAL_ENTER();              
//                               :
//                               :
//                           CPU_CRITICAL_EXIT();                                            
//                      }
//
//******************************************************************************************


#define CRITICAL_SETCION_ENTER()                        \
do                                                      \
{                                                       \
    INT_STATE cpu_state = MASTER_INT_STATE_GET();       \
    MASTER_INT_DISABLE();


#define CRITICAL_SETCION_EXIT()                         \
    MASTER_INT_RESTORE(cpu_state);                      \
}while(0)

//******************************************************************************************
//!                           Function Parameters
//******************************************************************************************
//#define __OTHER_TIMER_USED 

#define TIMER_ELEMENT_NUM_MAX      20                      //!< Maxium SoftTimer

#define SOFT_TIMER_SUCCESS         0                       //!< Operate success
#define SOFT_TIMER_ING             1                       //!< Timer Counting
#define SOFT_TIMER_TIMEOUT         0                       //!< Timer Timeout
#define SOFT_TIMER_ERR             3                       //!< Operate failure, invaild handle

//******************************************************************************************
//!                           PUBLIC API
//******************************************************************************************

extern uint16_t SoftTimer_Init(void);
extern uint16_t SoftTimer_Destroy(void);
extern uint16_t SoftTimer_Req(uint32_t Delay);
extern uint16_t SoftTimer_Update(uint32_t Id, uint32_t Tick);
extern uint16_t SoftTimer_UnReg(uint32_t Id);
extern uint16_t SoftTimer_Check(uint16_t Handle);
extern void TimerISR_Hook(void);

extern uint32_t systemCounter;

#ifdef __cplusplus
}
#endif

#endif // __SOFT_TIMER_H__
