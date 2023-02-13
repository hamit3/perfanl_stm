/******************************************************************************/
/** @file       hw.c
 *******************************************************************************
 *
 *  @brief      Module for initializing the attached hw
 *
 *  @author     wht4
 *
 *  @remark     Last modifications
 *                 \li V1.0, February 2016, wht4, initial release
 *
 ******************************************************************************/
/*
 *  functions  global:
 *              hw_init
 *  functions  local:
 *              hw_initSysclock
 *
 ******************************************************************************/

/****** Header-Files **********************************************************/
#include "stm32l0xx.h"

#include "config.h"
#include "led.h"
#include "button.h"
#include "systick.h"
#include "btn.h"
#include "gpio.h"
#include "i2c.h"
#include "rtc.h"

/****** Macros ****************************************************************/

/****** Data types ************************************************************/

/****** Function prototypes ****************************************************/
void adc_con_pins_init(void);


/****** Data ******************************************************************/

/****** Implementation ********************************************************/


/*******************************************************************************
 *  function :    hw_initSysclock
 ******************************************************************************/
/** @brief        This function configures the system clock @16MHz and voltage
 *                scale 1 assuming the registers have their reset value before
 *                the call.
 *                <p>
 *                POWER SCALE   = RANGE 1
 *                SYSTEM CLOCK  = PLL MUL8 DIV2
 *                PLL SOURCE    = HSI/4
 *                FLASH LATENCY = 0
 *
 *  @copyright    Licensed under MCD-ST Liberty SW License Agreement V2,
 *                (the "License");
 *                You may not use this file except in compliance with the
 *                License. You may obtain a copy of the License at:
 *                http://www.st.com/software_license_agreement_liberty_v2
 *
 *  @type         global
 *
 *  @param[in]    in
 *  @param[out]   out
 *
 *  @return       void
 *
 ******************************************************************************/
void hw_initSysclock(void) 
{
	/* Enable power interface clock */
	RCC->APB1ENR |= (RCC_APB1ENR_PWREN);

	/* Select voltage scale 1 (1.65V - 1.95V) i.e. (01) */
	/* for VOS bits in PWR_CR */
	PWR->CR = (PWR->CR & ~(PWR_CR_VOS)) | PWR_CR_VOS_0;  // 
	
  /* Enable HSI */	
	RCC->CR |= ((uint32_t)RCC_CR_HSION);
	
	/* Wait for HSI to be ready */
  while ((RCC->CR & RCC_CR_HSIRDY) == 0){
		// Nop
	}

	/* Set HSI as the System Clock */
  RCC->CFGR = RCC_CFGR_SW_HSI;
	
	/* Wait for HSI to be used for teh system clock */
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI){
		// Nop
	}
	
	FLASH->ACR |= FLASH_ACR_PRFTEN;                          // Enable Prefetch Buffer
  FLASH->ACR |= FLASH_ACR_LATENCY;                         // Flash 1 wait state

  RCC->APB1ENR |= RCC_APB1ENR_PWREN;                       // Enable the PWR APB1 Clock
  PWR->CR = PWR_CR_VOS_0;                                  // Select the Voltage Range 1 (1.8V)
  while((PWR->CSR & PWR_CSR_VOSF) != 0);                   // Wait for Voltage Regulator Ready

  /* PLLCLK = (HSI * 4)/2 = 32 MHz */
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV);				/* Clear */
  RCC->CFGR |=  (RCC_CFGR_PLLSRC_HSI | RCC_CFGR_PLLMUL4 | RCC_CFGR_PLLDIV2);	/* Set   */
	
	/* Peripheral Clock divisors */
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                         // HCLK = SYSCLK
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;                        // PCLK1 = HCLK
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;                        // PCLK2 = HCLK

	/* Enable PLL */
  RCC->CR &= ~RCC_CR_PLLON;		/* Disable PLL */
  RCC->CR |= RCC_CR_PLLON;		/* Enable PLL	 */
	
	/* Wait until the PLL is ready */
  while((RCC->CR & RCC_CR_PLLRDY) == 0){
		//Nop
	}
	
	/* Select PLL as system Clock */
  RCC->CFGR &= ~RCC_CFGR_SW;			/* Clear */
  RCC->CFGR |=  RCC_CFGR_SW_PLL;	/* Set   */
	
	/* Wait for PLL to become system core clock */
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL){
		//Nop
	}
}



/*******************************************************************************
 *  function :    hw_init
 ******************************************************************************/
void hw_init(void) {
	//
	//
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	
	//	
	I2C_Init();
	button_init();
	Configure_DBG();
	//	
}


void arrangeGPIOforSleep(void)
{
	
	/* Set port parameters */			//
	struct GPIO_Parameters GPIO;	//
	GPIO.Pin	= 1;								//
	GPIO.Mode = Output;						// 
	GPIO.OType = Push_Pull;				//
	GPIO.PuPd = No_PuPd;				//
	GPIO.Speed = Medium_Speed;		//
	
	/* Initialize XBEE_COMM Pin GPIOA 1*/
	GPIO_Init(GPIOA,GPIO);
	GPIO_Off(GPIOA,GPIO.Pin);
	
	/* Initialize INT Pin GPIOA 2*/
	GPIO.Pin	= 2;								//
	GPIO_Init(GPIOA,GPIO);
	GPIO_Off(GPIOA,GPIO.Pin);	

//	/* Initialize I2C SCL Pin GPIOA 6*/
//	GPIO.Pin	= 6;								//
//	GPIO_Init(GPIOA,GPIO);
//	GPIO_Off(GPIOA,GPIO.Pin);	
//	
//	/* Initialize I2C SDA Pin GPIOA 7*/
//	GPIO.Pin	= 7;								//
//	GPIO_Init(GPIOA,GPIO);
//	GPIO_Off(GPIOA,GPIO.Pin);	
	
	/* Initialize UART TXD Pin GPIOA 9*/
	GPIO.Pin	= 9;								//
	GPIO_Init(GPIOA,GPIO);								
	GPIO_Off(GPIOA,GPIO.Pin);		
	
	/* Initialize UART RXD Pin GPIOA 10*/
	GPIO.Pin	= 10;								//
	GPIO_Init(GPIOA,GPIO);								
	GPIO_Off(GPIOA,GPIO.Pin);		
	
//	/* Initialize SWDIO Pin GPIOA 13*/
//	GPIO.Pin	= 13;								//
//	GPIO_Init(GPIOA,GPIO);								
//	GPIO_On(GPIOA,GPIO.Pin);		
//	
//	/* Initialize SWCLK Pin GPIOA 14*/
//	GPIO.Pin	= 14;								//
//	GPIO_Init(GPIOA,GPIO);								
//	GPIO_Off(GPIOA,GPIO.Pin);		
		
	/* Initialize RGB_RED Pin GPIOA 15*/
	GPIO.Pin	= 15;								//
	GPIO_Init(GPIOA,GPIO);								
	GPIO_Off(GPIOA,GPIO.Pin);		

	/* Initialize RF_LED Pin GPIOB 0*/
	GPIO.Pin	= 0;								//
	GPIO_Init(GPIOB,GPIO);								
	GPIO_Off(GPIOB,GPIO.Pin);		

	/* Initialize RGB_BLUE Pin GPIOB 1*/
	GPIO.Pin	= 1;								//
	GPIO_Init(GPIOB,GPIO);								
	GPIO_Off(GPIOB,GPIO.Pin);		

	/* Initialize BL_LED Pin GPIOB 2*/
	GPIO.Pin	= 2;								//
	GPIO_Init(GPIOB,GPIO);								
	GPIO_Off(GPIOB,GPIO.Pin);		

	/* Initialize RGB_GREEN Pin GPIOB 3*/
	GPIO.Pin	= 3;								//
	GPIO_Init(GPIOB,GPIO);								
	GPIO_Off(GPIOB,GPIO.Pin);
	
	/* Initialize RGB_CONTROL Pin GPIOB 4*/
	GPIO.Pin	= 4;								//
	GPIO_Init(GPIOB,GPIO);								
	GPIO_Off(GPIOB,GPIO.Pin);	
	
	/* Initialize LP_UART_RX Pin GPIOB 10*/
	GPIO.Pin	= 10;								//
	GPIO_Init(GPIOB,GPIO);								
	GPIO_Off(GPIOB,GPIO.Pin);	

	/* Initialize LP_UART_TX Pin GPIOB 11*/
	GPIO.Pin	= 11;								//
	GPIO_Init(GPIOB,GPIO);								
	GPIO_Off(GPIOB,GPIO.Pin);	

	/* Initialize REG_PWR_CON Pin GPIOB 12*/
	GPIO.Pin	= 12;								//
	GPIO.Mode = Output;						// 
	GPIO.OType = Push_Pull;				//
	GPIO_Init(GPIOB,GPIO);								
	GPIO_Off(GPIOB,GPIO.Pin);	


	/* Initialize MAGNO_INT Pin GPIOB 13*/
	GPIO.Pin	= 13;								//
	GPIO.Mode = Input;
	GPIO.OType = Push_Pull;				//
	GPIO.PuPd = Pull_Down;				//
	GPIO_Init(GPIOB,GPIO);									
	
	/* Initialize ACC_INT Pin GPIOB 14*/
	GPIO.Pin	= 14;								//
	GPIO.Mode = Input;
	GPIO.OType = Push_Pull;				//
	GPIO.PuPd = Pull_Down;				//	
	GPIO_Init(GPIOB,GPIO);									
	
	/* Initialize GYRO_INT Pin GPIOB 15*/
	GPIO.Pin	= 15;								//
	GPIO.Mode = Input;
	GPIO.OType = Push_Pull;				//
	GPIO.PuPd = Pull_Down;				//
	GPIO_Init(GPIOB,GPIO);	

	/* Initialize I2C SCL Pin GPIOA 6*/
	GPIO.Pin	= 6;								//
	GPIO.Mode = Output;
	GPIO.OType = Push_Pull;				//
	GPIO_Init(GPIOB,GPIO);
	GPIO_Off(GPIOB,GPIO.Pin);	
	
	/* Initialize I2C SDA Pin GPIOA 7*/
	GPIO.Pin	= 7;	
	GPIO.Mode = Input;
	GPIO.OType = Push_Pull;				//
	GPIO.PuPd = Pull_Down;				//
	GPIO_Init(GPIOB,GPIO);
	GPIO_Off(GPIOB,GPIO.Pin);	
	
}

