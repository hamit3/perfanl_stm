/******************************************************************************/
/** @file       button.c
 *******************************************************************************
 *
 *  @remark     Last modifications
 *                 \li V1.0, February 2016, wht4, initial release
 *
 ******************************************************************************/
/*
 *              .
 *
 ******************************************************************************/

/****** Header-Files **********************************************************/
#include "button.h"
#include "gpio.h"
#include "led.h"
#include "systick.h"
#include "rtc.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <config.h>


unsigned char butdat = 0xFF;	// Button data
unsigned char prvdat = 0;			// Previous button data
unsigned char button = 0xFF;	// Button data store
unsigned char prvbut = 0;			// Previous button data
unsigned char presdb = 0;			// Pressed button
unsigned char vdcnt = 0;			// Valid data counter
unsigned char lvdcnt = 0;			// Long valid data counter
unsigned char lvd2cnt = 0;			// Long valid data counter

uint32_t fButtonON = 0;

/*******************************************************************************
 *  function :    led_init
 ******************************************************************************/
void button_init(void) 
{							 													 
	/* Set port parameters */			//
	struct GPIO_Parameters GPIO;	//
	GPIO.Pin	= 0;								//
	GPIO.Mode = Input;						// 
	GPIO.OType = Push_Pull;				//
	GPIO.PuPd = No_PuPd;					//
	GPIO.Speed = Medium_Speed;		//
	/* Initialize button GPIOA 15*/
	GPIO_Init(GPIOA,GPIO);	
	
	/* Line 0 is not masked */
	EXTI->IMR |= EXTI_IMR_IM0;
	
	/* Set interrupt for rising edge */
	EXTI->RTSR |= EXTI_RTSR_TR0;
	
	/* Enable interrupt for EXTI Line 0 and 1 */
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	
	NVIC_SetPriority(EXTI0_1_IRQn, CONFIG_BTN_ISR_PRIO);	
}


void Configure_DBG(void)
{
  /* Enable the peripheral clock of DBG register */
  RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
  
  DBGMCU->CR |= DBGMCU_CR_DBG_STOP; /* To be able to debug in stop mode */
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; /* To enter deep sleep when __WFI() */
  PWR->CR &=~ PWR_CR_PDDS; /* Select STOP mode */
	//PWR_CR_ULP set edilip ultralow power mode aktif edilebilir.
}

/**
  * Brief   This function configures EXTI.
  * Param   None
  * Retval  None
  */
void Configure_EXTI(void)
{
  /* Configure Syscfg, exti and nvic for pushbutton PB0 */
  /* (1) PB0 PB1 PB2 PB3 PB4 as source input */
  /* (2) Unmask port 0 */
  /* (3) Falling edge */
  /* (4) Set priority */
  /* (5) Enable EXTI0_1_IRQn */
  SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI0) | SYSCFG_EXTICR1_EXTI0_PB; /* (1) */ 
	
  EXTI->IMR |= EXTI_IMR_IM0; /* (2) */ 
  EXTI->RTSR |= EXTI_FTSR_TR0; /* (3) */ 
	//
	
	NVIC_SetPriority(EXTI0_1_IRQn, 0); /**/ 
	NVIC_EnableIRQ(EXTI0_1_IRQn);

	//			
}

void buttonPressed(void)
{
	if(fButtonON)
	{
		fButtonON = 0;
	}
	else
	{
		fButtonON = 1;
		fsleepEnable = 1;
	}
}

/*=====================================================================
				*********** BUTTON READ ************			
=====================================================================*/
void readButtonPin( void )
{
	butdat = (~(GPIOA-> IDR)) & 0x0001;	
		
	if (butdat==prvdat)
	{
		vdcnt++;
		if (button != butdat)			//Long button control
		{
			if ( vdcnt == 3)			//20ms * 3 = 60ms debounce
			{
				vdcnt = 0;
				prvbut = button;
				button = butdat;
				presdb = (butdat ^ prvbut) & prvbut;
				if (presdb != 0)
				{
					if (presdb == 0x01)				// Button Pressed			
					{
						buttonPressed();
					}
				}
			}
		}
	}		
	else
	{
		vdcnt = 0;
		lvdcnt = 0;
		lvd2cnt = 0;
		prvdat = butdat;
	}
}


