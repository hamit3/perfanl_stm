/*
 ******************************************************************************
 * @file    iis2mdc_reg.c
 * @author  EPTS Team
 * @brief   IIS2MDC driver file
 ******************************************************************************
*/
#include "flags.h"
#include "iis2mdc_reg.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "i2c.h"
#include "math.h"

void recordToMagBuffer(float magX, float magY, float magZ);
void filterMagData(float a[], float b[], float c[], int array_size);
	 
volatile int16_t newMag[3]; 


float TMP_MAG_X_BUF[sampleNumber];
float TMP_MAG_Y_BUF[sampleNumber];
float TMP_MAG_Z_BUF[sampleNumber];


float MAG_X_BUF = 0;
float MAG_Y_BUF = 0;
float MAG_Z_BUF = 0;

int magErrCnt = 0;
int magCnt = 0;

/**
  * @defgroup  IIS2MDC
  * @brief     This file provides a set of functions needed to drive the
  *            iis2mdc enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup  IIS2MDC_Interfaces_Functions
  * @brief     This section provide a set of functions used to read and
  *            write a generic register of the device.
  *            MANDATORY: return 0 -> no Error.
  * @{
  *
  */


/**********************************************************************
 * @brief		Check Magnetometer - Read "Who am I" flag
 **********************************************************************/
void checkMagnetometer(void)
{
	I2C_Read_Reg(IIS2MDC_I2C_ADD,0x4F);	
	//Check if device signature is correct
	if (I2C1->RXDR == 64)
	{
		fMagIsOk = 1;
	}
	else
	{
		fMagIsOk = 0;
	}
}

/**********************************************************************
 * @brief		INIT AND START MAGNETOMETER
 **********************************************************************/
void start_magneto(void)
{
	//**
	uint32_t MAG_DATA[1] = {0x00};

	MAG_DATA[0] = 0x20;
	I2C_Write_Bulk(IIS2MDC_I2C_ADD,IIS2MDC_CFG_REG_A, MAG_DATA, 1);		//  // RESET ALL REGISTERS
	//**
	MAG_DATA[0] = 0x8C;
	I2C_Write_Bulk(IIS2MDC_I2C_ADD,IIS2MDC_CFG_REG_A, MAG_DATA, 1);		// Temp Comp Enabled, 100Hz ODR, Continious Mode
	//**
	MAG_DATA[0] = 0x03;
	I2C_Write_Bulk(IIS2MDC_I2C_ADD,IIS2MDC_CFG_REG_B, MAG_DATA, 1);	// OFFSET CANCEL ENABLED, ENABLE LPF	
	//**
//	MAG_DATA[0] = 0x01;
//	I2C_Write_Bulk(IIS2MDC_I2C_ADD,IIS2MDC_CFG_REG_C, MAG_DATA, 1);	// INT ENABLED, Data ready signal enabled on register 	
	//**
	MAG_DATA[0] = 0xE3;
	I2C_Write_Bulk(IIS2MDC_I2C_ADD,IIS2MDC_INT_CRTL_REG, MAG_DATA, 1);	// INT LATCHED, INT occurded signal enabled on register 	
	//**
}

/**********************************************************************
 * @brief		ACTIVATE MAGNETOMETER LOW POWER MODE
 **********************************************************************/
void magneto_low_power_mode(void)
{
	uint32_t MAG_DATA[1] = {0x00};

	MAG_DATA[0] = 0x20;
	I2C_Write_Bulk(IIS2MDC_I2C_ADD,IIS2MDC_CFG_REG_A, MAG_DATA, 1);		//  // RESET ALL REGISTERS
	//**
	MAG_DATA[0] = 0x8C;
	I2C_Write_Bulk(IIS2MDC_I2C_ADD,IIS2MDC_CFG_REG_A, MAG_DATA, 1);		// Temp Comp Enabled, High Resolution, 100Hz ODR, Continuous Mode
	//**
	MAG_DATA[0] = 0x02;
	I2C_Write_Bulk(IIS2MDC_I2C_ADD,IIS2MDC_CFG_REG_B, MAG_DATA, 1);	// OFFSET CANCEL ENABLED, LOW PASS FILTER Disabled
	//**
}


/**********************************************************************
 * @brief		READ MAGNETOMETER ROUTINE
 **********************************************************************/
void read_magneto(void)
{
	uint32_t I2CSlaveBuffer[6] = {0,0,0,0,0,0};
	int i = 0;
	I2CSlaveBuffer[0] = I2C_Read_Reg(IIS2MDC_I2C_ADD,IIS2MDC_OUTX_H_REG);
	I2CSlaveBuffer[1] = I2C_Read_Reg(IIS2MDC_I2C_ADD,IIS2MDC_OUTX_L_REG);
	I2CSlaveBuffer[2] = I2C_Read_Reg(IIS2MDC_I2C_ADD,IIS2MDC_OUTY_H_REG);
	I2CSlaveBuffer[3] = I2C_Read_Reg(IIS2MDC_I2C_ADD,IIS2MDC_OUTY_L_REG);
	I2CSlaveBuffer[4] = I2C_Read_Reg(IIS2MDC_I2C_ADD,IIS2MDC_OUTZ_H_REG);
	I2CSlaveBuffer[5] = I2C_Read_Reg(IIS2MDC_I2C_ADD,IIS2MDC_OUTZ_L_REG);
	 for (i=0; i<6; i+=2)
	{
		newMag[i/2] = ((I2CSlaveBuffer[i] << 8) | I2CSlaveBuffer[i+1]);  // Turn the MSB and LSB into a 16-bit value
	}		
	recordToMagBuffer(newMag[0],newMag[1],newMag[2]);

	//READ INT SOURCE REGISTER TO CLEAR INTERRUPT  -- USE IF DATA READY INT ENABLED
	//I2C_Read_Reg(IIS2MDC_I2C_ADD,IIS2MDC_INT_SOURCE_REG);
	
}

void recordToMagBuffer( float magX, float magY, float magZ)
{
	// Swap x and y axis and -z because of mag orientation
	TMP_MAG_X_BUF[magCnt] = magX;
	TMP_MAG_Y_BUF[magCnt] = magY;
	TMP_MAG_Z_BUF[magCnt] = magZ;
	
	magCnt +=1;
	
	if (magCnt >=  sampleNumber)
	{		
		magCnt = 0;
		filterMagData(TMP_MAG_X_BUF, TMP_MAG_Y_BUF, TMP_MAG_Z_BUF,  sampleNumber);
	}	
}

 void filterMagData(float a[], float b[], float c[], int array_size)
 {
	int i, j, k, temp = 0;
	MAG_X_BUF = 0;
	MAG_Y_BUF = 0;
	MAG_Z_BUF = 0;	 
	
	if(array_size == 1)
	{
		MAG_X_BUF = a[0];
		MAG_Y_BUF = b[0];
		MAG_Z_BUF = c[0];
	}
	else if(array_size == 2) // just mean
	{
		MAG_X_BUF = (a[0] + a[1])/2;
		MAG_Y_BUF = (b[0] + b[1])/2;
		MAG_Z_BUF = (c[0] + c[1])/2;
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
			MAG_X_BUF = MAG_X_BUF + a[k]/(array_size - 2);
			MAG_Y_BUF = MAG_Y_BUF + b[k]/(array_size - 2);
			MAG_Z_BUF = MAG_Z_BUF + c[k]/(array_size - 2);
		}
	} 
	MAG_X_BUF = -1 * iis2mdc_from_lsb_to_uTesla(MAG_X_BUF); 
	MAG_Y_BUF = iis2mdc_from_lsb_to_uTesla(MAG_Y_BUF);
	MAG_Z_BUF = iis2mdc_from_lsb_to_uTesla(MAG_Z_BUF);	
	
 }  


float iis2mdc_from_lsb_to_mgauss(float lsb)
{
  return lsb * 1.5f;
}

float iis2mdc_from_lsb_to_uTesla(float lsb)
{
  return lsb * (1.5f / 10);
}


float iis2mdc_from_lsb_to_celsius(float lsb)
{
  return (lsb / 8.0f) + 25.0f;
}






