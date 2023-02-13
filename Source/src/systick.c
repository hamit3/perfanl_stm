/******************************************************************************/
/** @file       systick.c
 *******************************************************************************
 *
 *  @brief      General systick module for stm32l0
 *              <p>
 *              Allows to generate delays in busy loop
 *
 *  @author     wht4
 *
 *  @remark     Last modifications
 *                 \li V1.0, February 2016, wht4, initial release
 *
 ******************************************************************************/
/*
 *  functions  global:
 *              systick_init
 *              systick_delayMs
 *              systick_getTick
 *              systick_irq
 *  functions  local:
 *              .
 *
 ******************************************************************************/

/****** Header-Files **********************************************************/
#include "stm32l0xx.h"
#include "systick.h"

int f1ms = 0;
int f2ms = 0;
int f10ms = 0;
int f20ms = 0;
int f50ms = 0;
int f100ms = 0;
int f1s = 0;
int f2s = 0;
int f10s = 0;

int cnt2ms = 0;
int cnt10ms = 0;
int cnt20ms = 0;
int cnt50ms = 0;
int cnt100ms = 0;
int cnt1s = 0;
int cnt2s = 0;
int cnt10s = 0;

int fsleepEnable = 0;
int sleepCounter = 0;

extern void myWatchDogBOMB(void);

/****** Macros ****************************************************************/

/****** Data types ************************************************************/

/****** Fuction prototypes ****************************************************/

/****** Data ******************************************************************/
uint32_t u32Tick = 0;

/****** Implementation ********************************************************/


/*******************************************************************************
 *  function :    systick_init
 ******************************************************************************/
void
systick_init(uint32_t u32Ticks) {

    SysTick_Config(u32Ticks);
}


/*******************************************************************************
 *  function :    systick_delayMs
 ******************************************************************************/
void
systick_delayMs(uint32_t u32DelayMs) {

    uint32_t u32StartTick = u32Tick;
    while((u32DelayMs + u32StartTick) > u32Tick);
}


/*******************************************************************************
 *  function :    systick_getTick
 ******************************************************************************/
uint32_t
systick_getTick(void) {
	return (u32Tick);
}


/*******************************************************************************
 *  function :    systick_irq
 ******************************************************************************/
void systick_irq(void) {

  u32Tick++;
	
	f1ms = 1;

	
//-----------------//
//-----COUNTER-----//	
//-----------------//		
//*********	
	++cnt2ms; 
	if(cnt2ms >= 2 )
	{
		cnt2ms = 0;
		f2ms = 1;
	}
//*********		
	++cnt10ms;
	if(cnt10ms >= 10 )
	{
		cnt10ms = 0;
		f10ms = 1;
	}
//*********	
//*********		
	++cnt20ms;
	if(cnt20ms >= 20 )
	{
		cnt20ms = 0;
		f20ms = 1;
	}
//*********
//*********		
	++cnt50ms;
	if(cnt50ms >= 50 )
	{
		cnt50ms = 0;
		f50ms = 1;
	}
//*********		
	++cnt100ms;
	if(cnt100ms >= 100 )
	{
		cnt100ms = 0;
		f100ms = 1;
	}	
//*********		
	++cnt1s;
	if(cnt1s >= 1000 )
	{
		cnt1s = 0;
		f1s = 1;
	}	
//*********			
//*********		
	++cnt2s;
	if(cnt2s >= 2000 )
	{
		cnt2s = 0;
		f2s = 1;
	}	
//*********		
//*********		
	++cnt10s;
	if(cnt10s >= 10000 )
	{
		cnt10s = 0;
		f10s = 1;
	}
}

