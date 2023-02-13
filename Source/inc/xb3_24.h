/*------------------------------------------------------------------------------------------------------
 * Name:    xb3_24.h 
 *----------------------------------------------------------------------------------------------------*/

/*-------------------------------------------Include Statements---------------------------------------*/

#ifndef XB3_24_H
#define XB3_24_H

#include "stm32l053xx.h"
#include "Fusion.h"
#include "FusionAhrs.h"
#include "FusionTypes.h"
#include "FusionBias.h"


//#define DEBUG_ON_EPTS_USB_STICK
#define DEBUG_ON_EPTS_GATEWAY

#if defined DEBUG_ON_EPTS_USB_STICK
	#define cmdDestinationAddHigh			"ATDH13A200\r"        // EPTS TEST USB STICK
	#define cmdDestinationAddLow			"ATDL41A1C67A\r "     // EPTS TEST USB STICK
	#define DestinationAddHigh				0x0013A200     	   	  // EPTS TEST USB STICK
	#define DestinationAddLow					0x41A1C67A   		  		// EPTS TEST USB STICK
#elif defined DEBUG_ON_EPTS_GATEWAY
	#define cmdDestinationAddHigh			"ATDH0013A200\r"      // EPTS TEST ETHERNET GATEWAY
	#define cmdDestinationAddLow			"ATDL4180D683\r "     // EPTS TEST ETHERNET GATEWAY
	#define DestinationAddHigh				0x0013A200     	    	// EPTS TEST USB STICK
	#define DestinationAddLow					0x4180D683 		  			// EPTS TEST USB STICK
#else
	#define cmdDestinationAddHigh			"ATDH00000000\r"      // DEFAULT ETHERNET GATEWAY
	#define cmdDestinationAddLow			"ATDL00000000\r "     // DEFAULT ETHERNET GATEWAY
	#define DestinationAddHigh				0x00000000   	    		// EPTS TEST USB STICK
	#define DestinationAddLow					0x00000000  		 	 	  // EPTS TEST USB STICK
#endif


/* Initialization methods */
extern void conGpsDataToDecDegree(void);
extern void packDataforZigbee(void);
extern volatile float epts_yaw, epts_pitch, epts_roll;
extern volatile float epts_quat_w, epts_quat_x, epts_quat_y, epts_quat_z;

 typedef struct AT_Data
 {
	 char MY[6];		/* Source Address unique to this device */
	 char ID[6];		/* Personal Area Network 								*/
	 char DH[9];		/* Destination register high 						*/
	 char DL[9];		/* Destination register low							*/
	 char CH[4];		/* Channel Selection										*/
 } AT_Data;
 
//extern void LPUART_Init(void);
extern char LPUART1_PutChar(char ch);
extern void Configure_GPIO_LPUART(void);
extern void Configure_LPUART_9600(void);
extern void Configure_LPUART_115200(void);
extern void XBee_ProS1_Init(void);
extern void XBee_900HP_Init(void);
extern void LPUART1_Send(char c[]);
extern void LPUART1_Send_DataPack(char c[]);
extern void Read_Xbee_ProS1_Init(void);
extern void xbeeColdVerifySetup(void);
extern void IMUtoString(void);
extern char DataPack[128];
extern void Wait_For_OK(void);
extern void Wait_For_Data(void);

#endif
