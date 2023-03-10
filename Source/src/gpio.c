/*------------------------------------------------------------------------------------------------------
 * Name:    GPIO.c
 * Purpose: Initializes GPIO and has LED and Button functions for the green LED and the 
						User button
 * Date: 		7/14/15
 * Author:	Christopher Jordan - Denny
 *------------------------------------------------------------------------------------------------------
 * Note(s):
 *----------------------------------------------------------------------------------------------------*/

/*---------------------------------------------Include Statements-------------------------------------*/
#include "stm32l053xx.h"									// Specific Device header
#include "GPIO.h"
/*---------------------------------------------Definitions--------------------------------------------*/


/**
  \fn 			void GPIO_Init(GPIO_TypeDef* GPIOx, struct GPIO_Parameters GPIO)
  \brief		Initialize GPIO
	\param		GPIO_TypeDef* GPIOx: Which port to initialize, i.e. GPIOA,GPIOB
	\param		struct GPIO_Parameters GPIO: Structure containing all GPIO parameters:
						* Pin
						*	Mode
						*	Output Type
						* Output Speed
						* Pull up / Pull down
*/

void GPIO_Init(GPIO_TypeDef* GPIOx, struct GPIO_Parameters GPIO){
	
	/* Enable GPIO Clock depending on port */
	if(GPIOx == GPIOA) RCC->IOPENR |= RCC_IOPENR_GPIOAEN;		// Enable GPIOA clock
	if(GPIOx == GPIOB) RCC->IOPENR |= RCC_IOPENR_GPIOBEN;		// Enable GPIOB clock
	if(GPIOx == GPIOC) RCC->IOPENR |= RCC_IOPENR_GPIOCEN;		// Enable GPIOC clock
	if(GPIOx == GPIOD) RCC->IOPENR |= RCC_IOPENR_GPIODEN;		// Enbale GPIOD clock
	
	/* GPIO Mode Init */
	GPIOx->MODER   &= ~((3ul << 2*GPIO.Pin));					// write 00 to pin location
  GPIOx->MODER   |=  ((GPIO.Mode << 2*GPIO.Pin));		// Choose mode
	
	/* Output Type Init */
  GPIOx->OTYPER  &= ~((~(GPIO.OType) << GPIO.Pin));
	
	/* GPIO Speed Init */
  GPIOx->OSPEEDR |=  ((GPIO.Speed << 2*GPIO.Pin));
	
	/* GPIO PULLUP/PULLDOWN Init */
  GPIOx->PUPDR   |= (GPIO.PuPd << 2*GPIO.Pin);
}

/**
  \fn					void GPIO_Uninit(GPIO_TypeDef* GPIOx)
  \brief			Reset the given port
	\param			GPIO_TypeDef* GPIOx: Which port to reset
*/

void GPIO_Uninit(GPIO_TypeDef* GPIOx){
	
	/* GPIOA has different reset parameters */
	if(GPIOx == GPIOA){
		GPIOA->OTYPER = 0X00000000;											//Reset for portA
		GPIOA->MODER = 0xEBFFFCFF;											//Reset for portA
		GPIOA->PUPDR = 0X24000000;											//Reset for portA
		GPIOA->OSPEEDR = 0x0C000000;										//Reset for portA
	}
	
	/* Normal Reset States */
	else{
		GPIOx->OTYPER = 0X00000000;											//Reset for port
		GPIOx->MODER = 0XFFFFFFFF;											//Reset for port
		GPIOx->PUPDR = 0X00000000;											//Reset for port
		GPIOx->OSPEEDR = 0x00000000; 										//Reset for port
	}
}

/**
  \fn          void Button_Initialize (void)
  \brief       Initialize User Button
*/

void Button_Initialize(void){
	
	/* Set port parameters */
	struct GPIO_Parameters GPIO;
	GPIO.Pin	= 12;
	GPIO.Mode = Input;
	GPIO.OType = Push_Pull;
	GPIO.PuPd = No_PuPd;
	GPIO.Speed = Low_Speed;
	
	/* Initialize button */
	GPIO_Init(GPIOC,GPIO);
}

/**
  \fn          void LED_Init(void)
  \brief       Initialize LD2
*/

void GPIO_Output_Init(GPIO_TypeDef* GPIOx,int LED){
	
	/* Set port parameters */
	struct GPIO_Parameters GPIO;
	GPIO.Pin = LED;
	GPIO.Mode = Output;
	GPIO.OType = Push_Pull;
	GPIO.PuPd = No_PuPd;
	GPIO.Speed = High_Speed;
	
	/* Initialize the LED */
	GPIO_Init(GPIOx,GPIO);
}

/**
  \fn          void LED_On(void)
  \brief       Turn LD2 on
*/

void GPIO_On(GPIO_TypeDef* GPIOx,int LED){
	/* Local Variables */
	const unsigned long gpio_Pin = {1UL << LED};
	
	/* Turn on GPIOA-5 */
	GPIOx->BSRR |= (gpio_Pin);
	//GPIOA->ODR = 0x20;				// Same as using the BSRR register
}

/**
  \fn          void LED_Off(void)
  \brief       Turn LD2 off
*/

void GPIO_Off(GPIO_TypeDef* GPIOx,int LED){
	/* Local Variables */
	const unsigned long gpio_Pin = {1UL << LED};
	
	/* Turn the LED off */
	GPIOx->BSRR |= (gpio_Pin << 16);
}

