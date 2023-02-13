/******************************************************************************/
/** @file       main.c
 *******************************************************************************
 *
 *  @brief      EPTSPORT
 *
 ******************************************************************************/

/****** Header-Files **********************************************************/
/****** Header-Files **********************************************************/
#include "stm32l0xx.h"
#include "system_stm32l0xx.h"
#include "hw.h"
#include "xb3_24.h"
#include "gpio.h"
#include "led.h"
#include "button.h"
#include "systick.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "mma8452q.h"
#include "fxas21002.h"
#include "iis2mdc_reg.h"
#include "i2c.h"
#include "cam_m8q.h"
#include "flags.h"
#include "xb3_24.h"
#include "rtc.h"
#include "xbee.h"
#include "Fusion.h"
#include "FusionAhrs.h"
#include "FusionTypes.h"
#include "FusionBias.h"
#include <math.h>


#define newLine	"\r\n"
#define accc	"A\r\n"
#define gyroo	"G\r\n"
#define magg	"M\r\n"
#define GyroOkMessage	"Gyro is OK\r\n"
#define GyroNotOkMessage	"Gyro is NOT OK\r\n"
#define AccOkMessage	"Acc is OK\r\n"
#define AccNotOkMessage	"Acc is NOT OK\r\n"
#define MagOkMessage	"Mag is OK\r\n"
#define MagNotOkMessage	"Mag is NOT OK\r\n"
#define changeHc05Boudrate	"AT+UART=115200,1,0\0"
#define resetHc05	"AT+RESET\0"

void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z);
void ConfigureExtendedIT(void);
void ImuIntPinsInit(void);
void counterUpdate(void);
void checkRTCtimerTrig(void);
void ledStatusFlagsUpdate(void);
void powerConPinInit(void);
void sleepCon(void);
void peripheralPowerOn(void);
void peripheralPowerOff(void);
void xbeePinsInit(void);
void xbeeReset(void);
void xbeeSet(void);
void xbeeCommOn(void);
void xbeeCommOff(void);
void waitAlittle(void);
void epts_sensor_fusion(void);
void updateNewAHRS(void);
void updateGpsFixLed(void);
void updateDgpsFixLed(void);
void updateGpsLookingForLed(void);

FusionBias fusionBias;
FusionAhrs fusionAhrs;

float samplePeriod = 0.1f; // replace this value with actual sample period in seconds

uint32_t RGB1sCnt = 0;
uint32_t RGBFix100msCnt = 0;
uint32_t RGBDfix100msCnt = 0;

uint32_t sensorReadCnt = 0;
uint32_t sensorReadErrorCnt = 0;
uint32_t fMagIsOk = 0;
uint32_t fAccIsOk = 0;
uint32_t fGyroIsOk = 0;
uint32_t fGpsFix = 0;
uint32_t fDgpsFix = 0;
uint32_t fGpsLookingFor = 0;

int cntLed = 0;

	FusionVector3 gyroscopeSensitivity = {
			.axis.x = 0.0625f,
			.axis.y = 0.0625f,
			.axis.z = 0.0625f,
	}; // replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet

	FusionVector3 accelerometerSensitivity = {
			.axis.x = 0.001f,
			.axis.y = 0.001f,
			.axis.z = 0.001f,
	}; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet

	FusionVector3 hardIronBias = {
			.axis.x = 0.0f,
			.axis.y = 0.0f,
			.axis.z = 0.0f,
	}; // replace these values with actual hard-iron bias in uT if known	



/*******************************************************************************
 *  function :    main
 ******************************************************************************/
int main(void) 
{
	fSystemIsOn = 1;

	/*************************************
	......
	....
	..
	.
	INIT SYSTEM
	.
	..
	....
	......
	**************************************/
	SystemInit();
	hw_initSysclock();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);
	waitAlittle();
	led_init();
	//
	rgb_control_set();
	rgb_blue_led_set();
	rgb_green_led_set();	
	rgb_red_led_clear();
	//
	powerConPinInit(); 
	waitAlittle();
	peripheralPowerOn();     /// GPS, IMU and ZIGBEE POWER ON
	
	/*
	waitAlittle();
	hw_init();
	waitAlittle();
	xbeePinsInit();
	waitAlittle();
	xbeeReset();
	xbeeCommOff();
	waitAlittle();		
	checkAccelerometer();
	start_acc();
	waitAlittle();
	calibrate_acc();
	waitAlittle();
	checkGyro();
	waitAlittle();
	start_gyro();
	waitAlittle();
	checkMagnetometer();
	waitAlittle();
	start_magneto();
	waitAlittle();
	USART1_Init();
	waitAlittle();
	initGPSubxData();
	ImuIntPinsInit();
	//ConfigureExtendedIT();
	Configure_GPIO_LPUART();
  Configure_LPUART_115200(); // FOR XBEE 3, IN PRODUCTION
	//Configure_LPUART_9600();
	waitAlittle(); 
	xbeeSet();
	waitAlittle(); 
	xbeeCommOn();
	systick_delayMs(1000);
	xbeeColdVerifySetup();
	systick_delayMs(2000);	

	// Initialise gyroscope bias correction algorithm
	FusionBiasInitialise(&fusionBias, 0.5f, samplePeriod); // stationary threshold = 0.5 degrees per second

	// Initialise AHRS algorithm
	FusionAhrsInitialise(&fusionAhrs, 2.0f); // demo gain = 0.5

	// Set optional magnetic field limits
	FusionAhrsSetMagneticField(&fusionAhrs, 20.0f, 70.0f); // valid magnetic field range = 20 uT to 70 uT
*/
	
	while (1) 
	{
		/*
		if (fGPSdataUpdate)
		{
			fGPSdataUpdate = 0;
			GPSdataUpdate();					/// UPDATE GPS DATA BUFER AND DECODE NMEA DATAPACK
			conGpsDataToDecDegree();	/// CONVERT GPS DATA TO DECIMAL DEGREE FORMAT			
		}
		if(f10ms)
		{
			f10ms = 0;
			read_acc();					/// READ ACCELEROMETER
			read_magneto();			/// READ MAGNETOMETER
			read_gyro();				/// READ GYROSCOPE	
		}
		if(f20ms)
		{
			f20ms = 0;
			readButtonPin();		/// READ BUTTON
		}				
		if(f100ms)
		{
			f100ms = 0; 
			if(fGpsFix){updateGpsFixLed();}  	 // GPS FIXED, RGB BLUE STANDS
			if(fDgpsFix){updateDgpsFixLed();}  // DGPS FIXED, RGB GREEN STANDS
			updateNewAHRS(); 		 /// FEED AHRS FILTER
			packDataforZigbee(); /// PACK WHOLE DATA  
			send_Msg(DataPack);  /// SEND DATAPACK VIA XBEE		
		}
		*/
		if(f1s)
		{
			f1s = 0; 
			ledStatusFlagsUpdate();				/// UPDATE INDICATOR LEDS
			if(fGpsLookingFor){updateGpsLookingForLed();}  // GPS NOT FIXED, TOOGLE RGB LEDS
		}		
		sleepCon();						/// SLEEP CONTROL
	}
}


/**********************************************************************
 * @brief		Update GPS Fix Led 
 **********************************************************************/
void updateGpsFixLed(void)
{
		rgb_control_set();
		rgb_red_led_set();
		rgb_green_led_set();
		RGBFix100msCnt += 1;

		if(RGBFix100msCnt < 30)
		{
			rgb_blue_led_clear();
		}
		else if(RGBFix100msCnt < 31)
		{
			rgb_blue_led_set();
		}
		else if(RGBFix100msCnt < 32)
		{
			rgb_blue_led_clear();
		}
		else if(RGBFix100msCnt < 33)
		{
			rgb_blue_led_set();
		}		
		else if(RGBFix100msCnt < 34)
		{
			rgb_blue_led_clear();
		}
		else if(RGBFix100msCnt < 35)
		{
			rgb_blue_led_set();
		}	
		else if(RGBFix100msCnt < 36)
		{
			rgb_blue_led_clear();
		}
		else if(RGBFix100msCnt < 37)
		{
			rgb_blue_led_set();
		}		
		else if(RGBFix100msCnt < 38)
		{
			rgb_blue_led_clear();
		}
		else if(RGBFix100msCnt < 39)
		{
			rgb_blue_led_set();
		}
		else if(RGBFix100msCnt < 40)
		{
			rgb_blue_led_clear();
		}
		else if(RGBFix100msCnt < 41)
		{
			rgb_blue_led_set();
			RGBFix100msCnt = 0;
		}					
		else
		{
			RGBFix100msCnt = 0;
		}			
}


/**********************************************************************
 * @brief		Update DGPS Fix Leds
 **********************************************************************/
void updateDgpsFixLed(void)
{
		rgb_control_set();
		rgb_blue_led_set();
		rgb_red_led_set();
		RGBDfix100msCnt += 1;
		if(RGBDfix100msCnt < 30)
		{
			rgb_green_led_clear();
		}
		else if(RGBDfix100msCnt < 31)
		{
			rgb_green_led_set();
		}
		else if(RGBDfix100msCnt < 32)
		{
			rgb_green_led_clear();
		}
		else if(RGBDfix100msCnt < 33)
		{
			rgb_green_led_set();
		}
		else if(RGBDfix100msCnt < 34)
		{
			rgb_green_led_set();
		}
		else if(RGBDfix100msCnt < 35)
		{
			rgb_green_led_clear();
		}
		else if(RGBDfix100msCnt < 36)
		{
			rgb_green_led_set();
		}		
		else if(RGBDfix100msCnt < 37)
		{
			rgb_green_led_set();
		}
		else if(RGBDfix100msCnt < 38)
		{
			rgb_green_led_clear();
		}
		else if(RGBDfix100msCnt < 39)
		{
			rgb_green_led_set();
		}		
		else if(RGBDfix100msCnt < 40)
		{
			rgb_green_led_clear();
		}
		else if(RGBDfix100msCnt < 41)
		{
			rgb_green_led_set();
			RGBDfix100msCnt = 0;
		}				
		else
		{
			RGBDfix100msCnt = 0;
		}
}

/**********************************************************************
 * @brief		Update GPS Looking For Leds
 **********************************************************************/
void updateGpsLookingForLed(void)
{
		if (RGB1sCnt == 1)		
		{
			rgb_control_set();
			rgb_red_led_set();
			rgb_green_led_set();
			rgb_blue_led_clear();
		}
		else if (RGB1sCnt == 2)		
		{
			rgb_control_set();
			rgb_blue_led_set();
			rgb_red_led_set();
			rgb_green_led_clear();	
		}
		else if (RGB1sCnt == 3)
		{
			rgb_control_clear();
			rgb_blue_led_clear();
			rgb_red_led_clear();
			rgb_green_led_clear();			
		}			
		else
		{
			rgb_control_set();
			rgb_blue_led_set();
			rgb_green_led_set();
			rgb_red_led_clear();	
			RGB1sCnt = 0;
		}
		RGB1sCnt++;
	}

/**********************************************************************
 * @brief		Update Led Status Flags
 **********************************************************************/
void ledStatusFlagsUpdate(void)
{
	// GPS STATUS RGB LED
	if (GPS_FIX_DATA_BUF[0][0] == '1') // GPS Fixed
	{
		fGpsFix = 1;
		fDgpsFix = 0;
		fGpsLookingFor = 0;
		RGBDfix100msCnt = 0;
		RGB1sCnt = 0;
	}
	else if (GPS_FIX_DATA_BUF[0][0] == '2') // DGPS Fixed
	{
		fGpsFix = 0;
		fDgpsFix = 1;
		fGpsLookingFor = 0;
		RGBFix100msCnt = 0;
		RGB1sCnt = 0;
	}
	else  // GPS not fixed, looking for...
	{
		fGpsFix = 0;
		fDgpsFix = 0;
		fGpsLookingFor = 1;
		RGBFix100msCnt = 0;
		RGBDfix100msCnt = 0;
	}
}

void ImuIntPinsInit(void) {

	/* Set port parameters */			//
	struct GPIO_Parameters GPIO;	//
	GPIO.Pin	= 15;								//
	GPIO.Mode = Input;						// 
	GPIO.OType = Push_Pull;				//
	GPIO.PuPd = Pull_Down;				//
	GPIO.Speed = Medium_Speed;		//
	
	/* Initialize INT Pin GPIOB 15*/
	GPIO_Init(GPIOB,GPIO);
	
	/* Initialize INT Pin GPIOB 14*/
	GPIO.Pin	= 14;								//
	GPIO_Init(GPIOB,GPIO);

	/* Initialize INT Pin GPIOB 13*/
	GPIO.Pin	= 13;								//
	GPIO_Init(GPIOB,GPIO);											 
}

/*******************************************************************************
 *  function :    Power Control Pin Init
 ******************************************************************************/
void powerConPinInit(void) {

	/* Set port parameters */			//
	struct GPIO_Parameters GPIO;	//
	GPIO.Pin	= 12;								//
	GPIO.Mode = Output;						// 
	GPIO.OType = Push_Pull;				//
	GPIO.PuPd = No_PuPd;					//
	GPIO.Speed = Medium_Speed;		//
	
	/* Initialize button GPIOB 12*/
	GPIO_Init(GPIOB,GPIO);
}

void xbeePinsInit(void)
{
	/* Set port parameters */			//
	struct GPIO_Parameters GPIO;	//
	GPIO.Pin	= 2;								//
	GPIO.Mode = Output;						// 
	GPIO.OType = Push_Pull;				//
	GPIO.PuPd = No_PuPd;					//
	GPIO.Speed = Medium_Speed;		//
	
	/* Initialize button GPIOA 2*/
	GPIO_Init(GPIOA,GPIO);	
	
		/* Initialize button GPIOA 1*/
	GPIO.Pin	= 1;								//
	GPIO_Init(GPIOA,GPIO);	
}

void ConfigureExtendedIT(void)
{  
  /* (1) Enable the peripheral clock of GPIOB */ 
  /* (2) Select input mode (00) on GPIOB pin 15,14,13 */  
  /* (3) Select Port B */  
  /* (4) Configure the corresponding mask bit in the EXTI_IMR register */
  /* (5) Configure the Trigger Selection bits of the Interrupt line 
         on rising edge*/
  /* (6) Configure the Trigger Selection bits of the Interrupt line 
         on falling edge*/
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN; /* (1) */
  GPIOB->MODER = (GPIOB->MODER & ~((GPIO_MODER_MODE15)|(GPIO_MODER_MODE14)|(GPIO_MODER_MODE13))); /* (2) */  
  //SYSCFG->EXTICR[0] &= (uint16_t)~SYSCFG_EXTICR1_EXTI0_PA; /* (3) */
  EXTI->IMR |= 0xE000; /* (4) */ 
  EXTI->RTSR |= 0xE000; /* (5) */     // PB15,PB14,PB13 Rising Edge
  
  /* Configure NVIC for Extended Interrupt */
  /* (6) Enable Interrupt on EXTI0_1 */
  /* (7) Set priority for EXTI0_1 */
  NVIC_EnableIRQ(EXTI4_15_IRQn); /* (6) */
  NVIC_SetPriority(EXTI4_15_IRQn,0); /* (7) */
}

/*******************************************************************************
 *  function :    WAKEUP FROM SLEEP MODE
 ******************************************************************************/

void wakeUp(void)
{
	NVIC_SystemReset();
}


/*******************************************************************************
 *  function :    BtnCallback - WAKEUP FROM SLEEP MODE with BUTTON PRESS
 ******************************************************************************/
void BtnCallback (Btn_t tBtn, BtnHandlingCtx_t tBtnHandlingCtx) 
{
	wakeUp();
	(void) tBtn;
	(void) tBtnHandlingCtx;
}



/*******************************************************************************
 *  function :    SLEEP CONTROL
 ******************************************************************************/
void sleepCon(void)
{
	if(fsleepEnable)
	{	
		int i = 0;
		fSystemIsOn = 0;
		peripheralPowerOff();     /// GPS, IMU and ZIGBEE POWER OFF
		for(i=0;i<10000;i++);
		for(i=0;i<10000;i++);
		//
		Reset_I2C();
		I2C1->CR1 &= ~I2C_CR1_PE;
		for(i=0;i<10000;i++);		
		arrangeGPIOforSleep();
		RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
		RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
		RCC->APB1ENR &= ~RCC_APB1ENR_LCDEN;		
		RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN; /* (1) */					 
		RCC->APB2SMENR &= ~RCC_APB2SMENR_SYSCFGSMEN; /* (1) */
		RCC->APB2SMENR &= ~RCC_APB2SMENR_TIM21SMEN; /* (1) */
		RCC->APB2SMENR &= ~RCC_APB2SMENR_TIM22SMEN; /* (1) */
		RCC->APB2SMENR &= ~RCC_APB2SMENR_ADC1SMEN; /* (1) */
		RCC->APB2SMENR &= ~RCC_APB2SMENR_SPI1SMEN; /* (1) */
		RCC->APB2SMENR &= ~RCC_APB2SMENR_USART1SMEN; /* (1) */
		RCC->APB2SMENR &= ~RCC_APB1SMENR_LPUART1SMEN; /* (1) */	
		//RCC->APB2SMENR &= ~RCC_APB2SMENR_DBGMCUSMEN; /* (1) */
		//
//		// Mantiksiz
//		RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
//		DBGMCU->CR &= ~DBGMCU_CR_DBG; /* To be able to debug in all power down modes */
//		RCC->APB2ENR &= ~RCC_APB2ENR_DBGMCUEN;
		
		btn_registerCallback(BTN_1, BTN_HANDLING_CTX_ISR, BtnCallback);
		//
		PWR->CSR |= PWR_CSR_EWUP1 | PWR_CSR_EWUP1 ;
//     
    PWR->CR |= PWR_CR_CWUF; // clear the WUF flag after 2 clock cycles
    PWR->CR |= PWR_CR_ULP;   // V_{REFINT} is off in low-power mode
		PWR->CR |= PWR_CR_FWU;  // enable fast wake up
		
    PWR->CR &= ~PWR_CR_PDDS; 
		//PWR->CR |= PWR_CR_PDDS; // Enter Standby mode when the CPU enters deepsleep
    //PWR->CR |= PWR_CR_LPSDSR; // *!< Low-power deepsleep/sleep/ Internal Regulator Off*/  
		
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // low-power mode = stop mode
    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk; // reenter low-power mode after ISR

		__DSB(); // Ensures SLEEPONEXIT is set immediately before sleep		

		__disable_irq();
		SysTick->CTRL  &= ~(SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |SysTick_CTRL_ENABLE_Msk);  /* Enable SysTick IRQ and SysTick Timer */
		//	
		EXTI->PR = 0xFFFFFFFF;
		__WFI(); // Go to sleep
		//
		//
		__enable_irq();
		NVIC_EnableIRQ(SysTick_IRQn);
		NVIC_SetPriority(EXTI0_1_IRQn, 0); /**/ 
		NVIC_EnableIRQ(EXTI0_1_IRQn); /*ENABLE BUTTON INTERRUPT*/ 
	}	
}

/*******************************************************************************
 *  function :    GPS, IMU and ZIGBEE POWER ON
 ******************************************************************************/
void peripheralPowerOn(void)
{
		GPIO_On(GPIOB, 12);    /// GPS, IMU and ZIGBEE POWER ON
}

/*******************************************************************************
 *  function :    GPS, IMU and ZIGBEE POWER OFF
 ******************************************************************************/
void peripheralPowerOff(void)
{
		GPIO_Off(GPIOB, 12);    /// GPS, IMU and ZIGBEE POWER OFF
}


/*******************************************************************************
 *  function :    XBEE Reset
 ******************************************************************************/
void xbeeReset(void)
{
		GPIO_Off(GPIOA, 2);
}

/*******************************************************************************
*  function :    :Xbee Set
 ******************************************************************************/
void xbeeSet(void)
{
		GPIO_On(GPIOA, 2);
}

/*******************************************************************************
 *  function :    XBEE Commm On
 ******************************************************************************/
void xbeeCommOn(void)
{
		GPIO_On(GPIOA, 1);
}

/*******************************************************************************
 *  function :    GPS, IMU and ZIGBEE POWER OFF
 ******************************************************************************/
void xbeeCommOff(void)
{
		GPIO_Off(GPIOA, 1);
}

/*******************************************************************************
 *  function :    GPS, IMU and ZIGBEE POWER OFF
 ******************************************************************************/
void waitAlittle(void)
{
		systick_delayMs(20);
}

void updateNewAHRS(void)
{
	// Calibrate gyroscope
	FusionVector3 uncalibratedGyroscope = {
			.axis.x = GYRO_X_BUF, /* replace this value with actual gyroscope x axis measurement in lsb */
			.axis.y = GYRO_Y_BUF, /* replace this value with actual gyroscope y axis measurement in lsb */
			.axis.z = GYRO_Z_BUF, /* replace this value with actual gyroscope z axis measurement in lsb */
	};
	FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

	// Calibrate accelerometer
	FusionVector3 uncalibratedAccelerometer = {
			.axis.x = ACC_X_BUF, /* replace this value with actual accelerometer x axis measurement in lsb */
			.axis.y = ACC_Y_BUF, /* replace this value with actual accelerometer y axis measurement in lsb */
			.axis.z = ACC_Z_BUF, /* replace this value with actual accelerometer z axis measurement in lsb */
	};
	FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

	// Calibrate magnetometer
	FusionVector3 uncalibratedMagnetometer = {
			.axis.x = MAG_X_BUF, /* replace this value with actual magnetometer x axis measurement in uT */
			.axis.y = MAG_Y_BUF, /* replace this value with actual magnetometer y axis measurement in uT */
			.axis.z = MAG_Z_BUF, /* replace this value with actual magnetometer z axis measurement in uT */
	};
	FusionVector3 calibratedMagnetometer = FusionCalibrationMagnetic(uncalibratedMagnetometer, FUSION_ROTATION_MATRIX_IDENTITY, hardIronBias);

	// Update gyroscope bias correction algorithm
	calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);

	// Update AHRS algorithm
	FusionAhrsUpdate(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, calibratedMagnetometer, samplePeriod);

	// Print Euler angles with the methode in FusionAhrs
	FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));
	epts_pitch = eulerAngles.angle.pitch;
	epts_roll = eulerAngles.angle.roll;
	epts_yaw = eulerAngles.angle.yaw;
	
	epts_quat_w = (float) fusionAhrs.quaternion.element.w;
	epts_quat_x = (float) fusionAhrs.quaternion.element.x;
	epts_quat_y = (float) fusionAhrs.quaternion.element.y;
	epts_quat_z = (float) fusionAhrs.quaternion.element.z;
}


/*******************************************************************************
 *  END
 ******************************************************************************/
