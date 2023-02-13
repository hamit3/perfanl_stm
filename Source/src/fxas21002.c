/*
 ******************************************************************************
 * @file    fxas21002.c
 * @author  EPTS Team
 * @brief   fxas210002 driver file
 ******************************************************************************
*/
#include "flags.h"
#include <stdint.h>
#include <stdbool.h>
#include "fxas21002.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>	
#include "i2c.h"
#include "math.h"
#include "systick.h"

void recordToGyroBuffer( float gyroX, float gyroY, float gyroZ);
void filterGyroData(float a[], float b[], float c[], int array_size);
//void updateGyroBuffer( void );
 
volatile int16_t newGyro[3];

float TMP_GYRO_X_BUF[sampleNumber];
float TMP_GYRO_Y_BUF[sampleNumber];
float TMP_GYRO_Z_BUF[sampleNumber];


float GYRO_X_BUF = 0;
float GYRO_Y_BUF = 0;
float GYRO_Z_BUF = 0;
float GYRO_BIAS_X_BUF = 0;
float GYRO_BIAS_Y_BUF = 0;
float GYRO_BIAS_Z_BUF = 0;

//int GYRO_ERR_BUF = 0;

int gyroErrCnt = 0;
int gyroCnt = 0;

/**********************************************************************
 * @brief		Check Gyroscope - Read "Who am I" flag
 **********************************************************************/
void checkGyro(void)
{
	I2C_Read_Reg(FXAS21002_ADD,FXAS21002_WHO_AM_I_ADD);	
	//Check if device signature is correct
	if (I2C1->RXDR == 215)
	{
		fGyroIsOk = 1;
	}
	else
	{
		fGyroIsOk = 0;
	}
}



/**********************************************************************
 * @brief		INIT AND START GYROSCOPE
 **********************************************************************/
void start_gyro (void) 
{     
	//**
	uint32_t reg_val = 0;
	uint32_t GYRO_DATA[1] = {0x00};
	//**
	GYRO_DATA[0] = 0x00;
	I2C_Write_Bulk(FXAS21002_ADD,FXAS21002_REG_CTRLREG1, GYRO_DATA, 1);	//Standby mode 	
	//**	
	systick_delayMs(100); 
	GYRO_DATA[0] = 0x40;
	I2C_Write_NoAck(FXAS21002_ADD,FXAS21002_REG_CTRLREG1, GYRO_DATA, 1);	// Reset
	/*
	On issuing a Software Reset command over an I2C interface,
	the device immediately resets and does not send any
	acknowledgment (ACK) of the written byte to the Master.
	*/
	systick_delayMs(100); 	 // a little delay
	do                                                              // Wait for the RST bit to clear     
	{         
		reg_val = I2C_Read_Reg(FXAS21002_ADD, FXAS21002_REG_CTRLREG1) & 0x40;     
	}    
	while (reg_val);

	GYRO_DATA[0] = 0xDC;  // LPF cutoff 32Hz, HPF Enable, HPF Cutoff 0.248Hz, FULL SCALE 1 1, Range +- 2000 DPS
	I2C_Write_Bulk(FXAS21002_ADD,FXAS21002_REG_CTRLREG0, GYRO_DATA, 1);		
	GYRO_DATA[0] = 0x00;
	I2C_Write_Bulk(FXAS21002_ADD,FXAS21002_REG_CTRLREG2, GYRO_DATA, 1);		// All interrupts are Disabled
	GYRO_DATA[0] = 0x0E;
	I2C_Write_Bulk(FXAS21002_ADD,FXAS21002_REG_CTRLREG1, GYRO_DATA, 1);	// ODR = 100 Hz, Active mode 	
	//**
}


/**********************************************************************
 * @brief		INIT AND START GYROSCOPE
 **********************************************************************/
void gyro_low_power_mode (void) 
{     
	//
}

/**********************************************************************
 * @brief		READ GYROSCOPE
 **********************************************************************/
void read_gyro (void)
{	
	uint32_t I2CSlaveBuffer[6] = {0,0,0,0,0,0};
	int i = 0;
	I2CSlaveBuffer[0] = I2C_Read_Reg(FXAS21002_ADD,FXAS21002_REG_OUTXMSB);
	I2CSlaveBuffer[1] = I2C_Read_Reg(FXAS21002_ADD,FXAS21002_REG_OUTXLSB);
	I2CSlaveBuffer[2] = I2C_Read_Reg(FXAS21002_ADD,FXAS21002_REG_OUTYMSB);
	I2CSlaveBuffer[3] = I2C_Read_Reg(FXAS21002_ADD,FXAS21002_REG_OUTYLSB);
	I2CSlaveBuffer[4] = I2C_Read_Reg(FXAS21002_ADD,FXAS21002_REG_OUTZMSB);
	I2CSlaveBuffer[5] = I2C_Read_Reg(FXAS21002_ADD,FXAS21002_REG_OUTZLSB);
//	I2C_Read_Reg(FXAS21002_ADD,FXAS21002_REG_DR_STATUS);

	for (i=0; i<6; i+=2)
	{
		newGyro[i/2] = ((I2CSlaveBuffer[i] << 8) | I2CSlaveBuffer[i+1]);  // Turn the MSB and LSB into a 16-bit value
	}	
	
	recordToGyroBuffer(newGyro[0],newGyro[1],newGyro[2]);	
	
}

void recordToGyroBuffer( float gyroX, float gyroY, float gyroZ)
{
	TMP_GYRO_X_BUF[gyroCnt] = gyroX;
	TMP_GYRO_Y_BUF[gyroCnt] = gyroY;
	TMP_GYRO_Z_BUF[gyroCnt] = gyroZ;
	
	gyroCnt +=1;
	if (gyroCnt >=  sampleNumber)
	{		
		gyroCnt = 0;
		filterGyroData(TMP_GYRO_X_BUF, TMP_GYRO_Y_BUF, TMP_GYRO_Z_BUF,  sampleNumber);
	}	
}

 void filterGyroData(float a[], float b[], float c[], int array_size)
 {
	int i, j, k, temp = 0;
	float TMP_GYRO_DATA = 0;
	GYRO_X_BUF = 0;
	GYRO_Y_BUF = 0;
	GYRO_Z_BUF = 0;	 
	
	if(array_size == 1)
	{
		GYRO_X_BUF = a[0];
		GYRO_Y_BUF = b[0];
		GYRO_Z_BUF = c[0];
	}
	else if(array_size == 2) // just mean
	{
		GYRO_X_BUF = (a[0] + a[1])/2;
		GYRO_Y_BUF = (b[0] + b[1])/2;
		GYRO_Z_BUF = (c[0] + c[1])/2;
	}
	else // if sample number >= 3, sort data, eleminete max and min value and then mean the total sum	
	{
	// Data sorting
		for (i = 0; i < (array_size - 1); ++i)
		{
			for (j = 0; j < array_size - 1 - i; ++j )
			{
				 if (a[j] > a[j+1])
				 {
						temp = a[j+1];
						a[j+1] = a[j];
						a[j] = temp;
				 }
				 if (b[j] > b[j+1])
				 {
						temp = b[j+1];
						b[j+1] = b[j];
						b[j] = temp;
				 }
				 if (c[j] > c[j+1])
				 {
						temp = c[j+1];
						c[j+1] = c[j];
						c[j] = temp;
				 }
			}
		}
		for(k = 1; k < (array_size - 1); k++)
		{
			GYRO_X_BUF = GYRO_X_BUF + a[k]/(array_size - 2);
			GYRO_Y_BUF = GYRO_Y_BUF + b[k]/(array_size - 2);
			GYRO_Z_BUF = GYRO_Z_BUF + c[k]/(array_size - 2);
		}
	}	
	// Due to orientation of EPTS Board, shift X and Y, reverse  X
	TMP_GYRO_DATA = GYRO_X_BUF;
	GYRO_X_BUF = GYRO_Y_BUF;
	GYRO_Y_BUF = -1 * TMP_GYRO_DATA;
} 

///*******************************************************************************
// * EOF
// ******************************************************************************/
