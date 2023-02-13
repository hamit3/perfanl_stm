/*------------------------------------------------------------------------------------------------------
 * Name:    xb3_24.c
 * Purpose: Initializes LUART and XB3_24 xbee device
 -----------------------------------------------------------------------------------------------------*/

/*---------------------------------Include Statements-------------------------------------------------*/
#include "stm32l053xx.h"			//Specific Device Header
#include <stdint.h>
#include <stdio.h>						//Standard input and output
#include <string.h>						//Various useful string functions
#include <stdlib.h>						//Various useful conversion functions
#include <math.h>	
#include "flags.h"
#include "systick.h"
#include "cam_m8q.h"
#include "xb3_24.h"
#include "mma8452q.h"  
#include "fxas21002.h"  
#include "iis2mdc_reg.h" 
#include "Fusion.h"
#include "FusionAhrs.h"
#include "FusionTypes.h"
#include "FusionBias.h"


/*---------------------------------XBee Commands----------------------------------------------------------------------*/
/* Prefix(AT) + ASCII Command + Space(Optional) + Parameter(Optional,HEX) + Carridge Return */
#define cmdMode						"+++"
#define cmdWrite					"ATWR\r"
#define cmdReset					"ATNR0\r"
//#define cmdBlEn						"ATBT1\r"  // Bluetooth Enable
#define cmdApply					"ATAC\r"
#define cmdDeviceRole			"ATCE0\r"  // Device Role --> 0 Standard Router
#define cmdBoudRate				"ATBD7\r"  //boud 115200
#define cmdExit						"ATCN\r"
#define cmdZigbeeStack		"ATZS2\r"  // 0 Network Specific, 1 Zigbee 2006, 2 Zigbee Pro
#define cmdAssoLed				"ATD55\r"  // 4 Off, 5 Onn
#define cmdPanID					"ATID0\r"
#define cmdTransferMode		"ATAP1\r"  // 0 = transparent, 1; API Mode

/*---------------------------------More defines-----------------------------------------------------------------------*/
#define TRUE	1;
#define FALSE 0;
#define PCLK	32000000								// Clock
#define HSECLK	32768									// LSE Clock

#define BAUD9600	9600									// Baud rate
#define BAUD115200	115200									// Baud rate

/*---------------------------------Globals----------------------------------------------------------------------------*/
char 										RX_Data[128] = 				"";				//Rx
uint8_t 								ChIndex =							0;				//Character Index
static const char				OK[] = 								"\0OK";			//When data has been written XBee will send an OK
static const char				cmdOK[] = 						"OK";
static const char				cmdERROR[] = 						"ERROR";
uint8_t									Device_Ack_Flag = 		0;		//XBee OK acknowledge
uint8_t									Device_Error_Flag = 		0;		//XBee ERROR acknowledge


char latString[12]  = {'0','0','0','0','0','0','0','0','0','0','0','\0'};
char lonString[12]  = {'0','0','0','0','0','0','0','0','0','0','0','\0'};

char str_epts_pitch[10];
char str_epts_yaw[10];
char str_epts_roll[10];

char str_quat_w[8] = {'0','0','0','0','0','0','0','\0'};
char str_quat_x[8] = {'0','0','0','0','0','0','0','\0'};
char str_quat_y[8] = {'0','0','0','0','0','0','0','\0'};
char str_quat_z[8] = {'0','0','0','0','0','0','0','\0'};

volatile float epts_yaw, epts_pitch, epts_roll;
volatile float epts_quat_w, epts_quat_x, epts_quat_y, epts_quat_z;

char DataPack[128];
char strAccX[10];
char strAccY[10];
char strAccZ[10];
char strGyroX[10];
char strGyroY[10];
char strGyroZ[10];
char strMagX[10];
char strMagY[10];
char strMagZ[10];
/*--------------------------------Struct Initialize-------------------------------------------------------------------*/
AT_Data AT;




void xbeeColdVerifySetup(void)
{   
	LPUART1_Send(cmdMode);
	Wait_For_OK();
	LPUART1_Send(cmdReset);
	Wait_For_OK();	
	LPUART1_Send(cmdBoudRate);
	Wait_For_OK();
//	LPUART1_Send(cmdBlEn);
//	Wait_For_OK();	
	LPUART1_Send(cmdDeviceRole);
	Wait_For_OK();
	LPUART1_Send(cmdZigbeeStack);
	Wait_For_OK();	
	LPUART1_Send(cmdTransferMode);
	Wait_For_OK();
	LPUART1_Send(cmdPanID);
	Wait_For_OK();
//	LPUART1_Send(cmdDestinationAddHigh);
//	Wait_For_OK();	
//	LPUART1_Send(cmdDestinationAddLow);
//	Wait_For_OK();	
	LPUART1_Send(cmdAssoLed);
	Wait_For_OK();
	LPUART1_Send(cmdWrite);	
	Wait_For_OK();
	LPUART1_Send(cmdExit);
	Wait_For_OK();	
}



/**
	\fn			void RNG_LPUART1_IRQHandler(void)
	\brief	Global interrupt handler for LPUART, Currently only handles RX
*/

void RNG_LPUART1_IRQHandler(void){
	if((LPUART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE){
		
		/* Read RX Data */
		RX_Data[ChIndex] = LPUART1->RDR;
		
		/* Check for RTS */
		if(RX_Data[ChIndex] == '!'){
			// RTS KULLANILIRSA
		}
		
		/* Check for end of recieved data */
		if(RX_Data[ChIndex] == '\r'){
			/* Compare string to OK to see if Acknowledged */
			if(strncmp(OK,RX_Data,(sizeof(OK)-1)) == 0){
				Device_Ack_Flag = 1;
			}
			else if(strncmp(cmdOK,RX_Data,(sizeof(cmdOK)-1)) == 0){
				Device_Ack_Flag = 1;
			}
			else if(strncmp(cmdERROR,RX_Data,(sizeof(cmdERROR)-1)) == 0){
				Device_Error_Flag = 1;
			}
			/* Clear RX_Data */
			ChIndex = 0;
			memset(RX_Data,0,sizeof(RX_Data));
		}else ChIndex++;
	}
}


/**
  * Brief   This function :
             - Enables GPIO clock
             - Configures the LPUART pins on GPIO PB10 PB11
  * Param   None
  * Retval  None
  */
void Configure_GPIO_LPUART(void)
{
  /* Enable the peripheral clock of GPIOB */
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
	
  /* GPIO configuration for LPUART signals */
  /* (1) Select AF mode (10) on PB10 and PB11 */
  /* (2) AF4 for LPUART signals */
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE10|GPIO_MODER_MODE11))\
                 | (GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1); /* (1) */
  GPIOB->AFR[1] = (GPIOB->AFR[1] &~ (0x0000FF00))\
                  | (4 << (2 * 4)) | (4 << (3 * 4)); /* (2) */

}

/**
  * Brief   This function configures LPUART.
  * Param   None
  * Retval  None
  */
//void Configure_LPUART(void)
//{
//	/* (1) Enable power interface clock */
//  /* (2) Disable back up protection register to allow the access to the RTC clock domain */
//  /* (3) LSE on */
//  /* (4) Wait LSE ready */
//  /* (5) Enable back up protection register to allow the access to the RTC clock domain */
//  /* (6) LSE mapped on LPUART */
//  /* (7) Enable the peripheral clock LPUART */
//  /* Configure LPUART */
//  /* (8) oversampling by 16, 9600 baud */
//  /* (9) 8 data bit, 1 start bit, 1 stop bit, no parity, reception mode */
//  /* (10) Set priority for LPUART1_IRQn */
//  /* (11) Enable LPUART1_IRQn */
//  RCC->APB1ENR |= (RCC_APB1ENR_PWREN); /* (1) */
//  PWR->CR |= PWR_CR_DBP; /* (2) */
//  RCC->CSR |= RCC_CSR_LSEON; /* (3) */
//  while ((RCC->CSR & (RCC_CSR_LSERDY)) != (RCC_CSR_LSERDY)) /* (4) */
//  {
//    /* add time out here for a robust application */
//  }
//  PWR->CR &=~ PWR_CR_DBP; /* (5) */
//  RCC->CCIPR |= RCC_CCIPR_LPUART1SEL; /* (6) */
//	RCC->APB1ENR |= RCC_APB1ENR_LPUART1EN; /* (7) */
//  LPUART1->BRR = 0x369; /* (8) */
//	LPUART1->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE| USART_CR1_TE; /* (9) */
//  NVIC_SetPriority(RNG_LPUART1_IRQn, 1); /* (10) */
//  NVIC_EnableIRQ(RNG_LPUART1_IRQn); /* (11) */


//}


void Configure_LPUART_9600(void){
	
	RCC->APB1ENR  |=   RCC_APB1ENR_LPUART1EN;   /* Enable LP USART#1 clock */
	
	//interrupt init
	NVIC_EnableIRQ(RNG_LPUART1_IRQn);
	NVIC_SetPriority(RNG_LPUART1_IRQn,1);
	
	LPUART1->BRR  	= (unsigned long)((256.0f/BAUD9600)*PCLK); 		/* 9600 baud @ 32MHz */
  LPUART1->CR3    = 0x0000;																	/* no flow control */
	LPUART1->CR2    = 0x0000;																	/* 1 stop bit */
	
	/* 1 stop bit, 8 data bits */
  LPUART1->CR1    = ((USART_CR1_RE) |												/* enable RX  */
                     (USART_CR1_TE) |												/* enable TX  */
                     (USART_CR1_UE) |      									/* enable USART */
										 (USART_CR1_RXNEIE));										/* Enable Interrupt */								
}

/**
	\fn				char LPUART1_PutChar(char ch)
	\brief		Sends a character to the XBEE
	\returns 	char ch: The character sent to the XBEE
*/


void Configure_LPUART_115200(void){
	
	RCC->APB1ENR  |=   RCC_APB1ENR_LPUART1EN;   /* Enable LP USART#1 clock */
	
	//interrupt init
	NVIC_EnableIRQ(RNG_LPUART1_IRQn);
	NVIC_SetPriority(RNG_LPUART1_IRQn,1);
	
	LPUART1->BRR  	= (unsigned long)((256.0f/BAUD115200)*PCLK); 		/* 115200 baud @ 32MHz */
  LPUART1->CR3    = 0x0000;																	/* no flow control */
	LPUART1->CR2    = 0x0000;																	/* 1 stop bit */
	
	/* 1 stop bit, 8 data bits */
  LPUART1->CR1    = ((USART_CR1_RE) |												/* enable RX  */
                     (USART_CR1_TE) |												/* enable TX  */
                     (USART_CR1_UE) |      									/* enable USART */
										 (USART_CR1_RXNEIE));										/* Enable Interrupt */								
}

/**
	\fn				char LPUART1_PutChar(char ch)
	\brief		Sends a character to the XBEE
	\returns 	char ch: The character sent to the XBEE
*/

char LPUART1_PutChar(char ch){

	//Wait for buffer to be empty
  while ((LPUART1->ISR & USART_ISR_TXE) == 0){
			//Nop
	}
	
	//Send character
  LPUART1->TDR = (ch);

  return (ch);
}

/**
	\fn				void LPUART1_Send(char c[])
	\brief		Sends a string to the XBEE
*/

void LPUART1_Send(char c[]){
	
	int String_Length = strlen(c);
	int Counter = 0;
	
	while(Counter < String_Length){
		LPUART1_PutChar(c[Counter]);
		Counter++;
	}
}

/**
	\fn				void LPUART1_Send(char c[])
	\brief		Sends a string to the XBEE
*/

void LPUART1_Send_DataPack(char c[]){
	
	int Counter = 0;
	
	while(Counter < 512){
		LPUART1_PutChar(c[Counter]);
		Counter++;
	}
}


/**
	\fn				void XBee_Init(void)
	\brief		Initializes the XBEE
*/

/**
	\fn				void Wait_For_OK(void)
	\brief		Waits until the XBEE has sent the OK message
						This is done when it has changed its settings
*/

void Wait_For_OK(void){
	
	/* Wait for XBee Acknowledge */
	while(!Device_Ack_Flag);
	
	/* Reset Flags */
	Device_Ack_Flag = 0;
	
}



/**********************************************************************
 * @brief		Functions to Convert Float to String
 **********************************************************************/
// reverses a string 'str' of length 'len' 
void reverse(char *str, int len) 
{ 
    int i=0, j=len-1, temp; 
    while (i<j) 
    { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; j--; 
    } 
} 
  
 // Converts a given integer x to string str[].  d is the number 
 // of digits required in output. If d is more than the number 
 // of digits in x, then 0s are added at the beginning. 
int intToStr(int x, char str[], int d) 
{ 
    int i = 0; 
    while (x) 
    { 
        str[i++] = (x%10) + '0'; 
        x = x/10; 
    }
  
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
  
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 
void ftoa(float n, char *res, int afterpoint) 
{ 
    // Extract integer part 
    int ipart = (int)n; 
  
    // Extract floating part 
    float fpart = n - (float)ipart; 
  
    // convert integer part to string 
    int i = intToStr(ipart, res, 0); 
  
    // check for display option after point 
    if (afterpoint != 0) 
    { 
        res[i] = '.';  // add dot 
  
        // Get the value of fraction part upto given no. 
        // of points after dot. The third parameter is needed 
        // to handle cases like 233.007 
        fpart = fpart * pow(10, afterpoint); 
  
        intToStr((int)fpart, res + i + 1, afterpoint); 
    } 
} 

/**********************************************************************
 * @brief		Convert GPS Data fto Decimal Degree Format
 **********************************************************************/
void conGpsDataToDecDegree(void)
{
	float lat = 0;
	float latNew = 0;
	double latInt= 0;
	float latFraction= 0;
	float lon= 0;
	float lonNew= 0;
	double lonInt= 0;
	float lonFraction= 0;

	//GPS value format dönüsümü
	lat = atof(&GPS_LAT_BUF[0][0])/100;
	lon = atof(&GPS_LON_BUF[0][0])/100;
	latFraction =100*modf(lat, &latInt)/60;
	lonFraction =100*modf(lon, &lonInt)/60;
	latNew = latInt + latFraction;
	lonNew = lonInt + lonFraction;
	ftoa(latNew, latString, 8); 
	ftoa(lonNew, lonString, 8);		
}


// inline function to swap two numbers
void swap(char *x, char *y) {
	char t = *x; *x = *y; *y = t;
}

// function to reverse buffer[i..j]
char* reverseInt(char *buffer, int i, int j)
{
	while (i < j)
		swap(&buffer[i++], &buffer[j--]);

	return buffer;
}

// Iterative function to implement itoa() function in C
char* itoa(int value, char* buffer, int base)
{
	int i = 0;
	int n = abs(value);
	// invalid input
	if (base < 2 || base > 32)
		return buffer;

	// consider absolute value of number

	while (n)
	{
		int r = n % base;

		if (r >= 10) 
			buffer[i++] = 65 + (r - 10);
		else
			buffer[i++] = 48 + r;

		n = n / base;
	}

	// if number is 0
	if (i == 0)
		buffer[i++] = '0';

	// If base is 10 and value is negative, the resulting string 
	// is preceded with a minus sign (-)
	// With any other base, value is always considered unsigned
	if (value < 0 && base == 10)
		buffer[i++] = '-';

	buffer[i] = '\0'; // null terminate string

	// reverse the string and return it
	return reverseInt(buffer, 0,(i-1));
}


/**********************************************************************
 * @brief		IMU Val to String
 **********************************************************************/
void IMUtoString(void)
{
	itoa(ACC_X_BUF, strAccX, 10);
	itoa(ACC_Y_BUF, strAccY, 10);
	itoa(ACC_Z_BUF, strAccZ, 10);
}	

/**********************************************************************
 * @brief		QUAT Val to String
 **********************************************************************/
void QUATtoString(void)
{
	double tmp_epts_quat_w,tmp_epts_quat_x,tmp_epts_quat_y,tmp_epts_quat_z = 0.0f;
	
	tmp_epts_quat_w = epts_quat_w;
	tmp_epts_quat_x = epts_quat_x;
	tmp_epts_quat_y = epts_quat_y;
	tmp_epts_quat_z = epts_quat_z;	
	
	if (epts_quat_w < 0)
	{
		tmp_epts_quat_w *= -1;
	}
		if (epts_quat_x < 0)
	{
		tmp_epts_quat_x *= -1;
	}
		if (epts_quat_y < 0)
	{
		tmp_epts_quat_y *= -1;
	}
		if (epts_quat_z < 0)
	{
		tmp_epts_quat_z *= -1;
	}
	ftoa(tmp_epts_quat_w, str_quat_w, 6);
	ftoa(tmp_epts_quat_x, str_quat_x, 6);
	ftoa(tmp_epts_quat_y, str_quat_y, 6);
	ftoa(tmp_epts_quat_z, str_quat_z, 6);
}	


/**********************************************************************
 * @brief		Pack Data for Zigbee
 **********************************************************************/
void packDataforZigbee(void)
{
	char serialized_string[128] = "";
	QUATtoString();
	IMUtoString();
	strcat(serialized_string, "$E");
	strcat(serialized_string, "&");
	strcat(serialized_string, &GPS_FIX_DATA_BUF[0][0]);
	strcat(serialized_string, "&");		
	strcat(serialized_string, &GPS_HDOP_BUF[0][0]);
	strcat(serialized_string, "&");		
	strcat(serialized_string, latString);
	strcat(serialized_string, "&");	
	strcat(serialized_string, lonString);
	strcat(serialized_string, "&");	
	strcat(serialized_string, &GPS_VELOCITY_BUF[0][0]);
	strcat(serialized_string, "&");	
	strcat(serialized_string, "98"); // HEARTRATE EKLENECEK
	strcat(serialized_string, "&");
	strcat(serialized_string, strAccX);
	strcat(serialized_string, "&");	
	strcat(serialized_string, strAccY);
	strcat(serialized_string, "&");	
	strcat(serialized_string, strAccZ);
	strcat(serialized_string, "&");	
	if(epts_quat_w < 0 ){strcat(serialized_string, "-");}
	strcat(serialized_string, "0");		
	strcat(serialized_string, str_quat_w);
	strcat(serialized_string, "&");
	if(epts_quat_x < 0 ){strcat(serialized_string, "-");}
	strcat(serialized_string, "0");	
	strcat(serialized_string, str_quat_x);
	strcat(serialized_string, "&");
	if(epts_quat_y < 0 ){strcat(serialized_string, "-");}
	strcat(serialized_string, "0");		
	strcat(serialized_string, str_quat_y);	
	strcat(serialized_string, "&");
	if(epts_quat_z < 0 ){strcat(serialized_string, "-");}
	strcat(serialized_string, "0");		
	strcat(serialized_string, str_quat_z);	
	strcat(serialized_string, "&D");	
	strcat(serialized_string, "\0");		
;
	memcpy(DataPack, serialized_string, sizeof(serialized_string)-1);
}	
