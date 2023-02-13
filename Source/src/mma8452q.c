/*
 ******************************************************************************
 * @file    mma8452qc.c
 * @author  EPTS Team
 * @brief   mma8452q driver file
 ******************************************************************************
*/
#include "flags.h"
#include "i2c.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>	
#include "mma8452q.h"
#include "systick.h"
#include <math.h>


float TMP_ACC_X_BUF[sampleNumber];
float TMP_ACC_Y_BUF[sampleNumber];
float TMP_ACC_Z_BUF[sampleNumber];

float ACC_X_BUF = 0;
float ACC_Y_BUF = 0;
float ACC_Z_BUF = 0;
float OFSET_ACC_X_BUF = 0;
float OFSET_ACC_Y_BUF = 0;
float OFSET_ACC_Z_BUF = 0;

int accCnt = 0;
int corde = 50, min_setting = 0, max_setting = 0, xoff_calibration = 0, yoff_calibration = 0, zoff_calibration = 0;
float x_offset = 0, y_offset = 0, z_offset = 0;


void recordToAccBuffer( float accX, float accY, float accZ);
void filterAccData(float a[], float b[], float c[], int array_size);
//void updateAccBuffer( void );	

/**********************************************************************
 * @brief		Check Accelerometer - Read "Who am I" flag
 **********************************************************************/
void checkAccelerometer(void)
{
	I2C_Read_Reg(MMA8452Q_ADD, MMA8452Q_WHO_AM_I_ADD);	
	if (I2C1->RXDR == 42)
	{
		fAccIsOk = 1;
	}
	else
	{
		fAccIsOk = 0;
	}
}

/**********************************************************************
 * @brief		INIT AND START ACCELEROMETER
 **********************************************************************/
void start_acc (void)
{	
	uint32_t ACC_DATA[1] = {0x00};
	//**
	ACC_DATA[0] = 0x00;	
	I2C_Write_Bulk(MMA8452Q_ADD, CTRL_REG1, ACC_DATA, 1);	// Reset  
	//**
	ACC_DATA[0] = 0x02;
	I2C_Write_Bulk(MMA8452Q_ADD, CTRL_REG2, ACC_DATA, 1);	//	High Resolution		
	//**
//	ACC_DATA[0] = 0x02;
//	I2C_Write_Bulk(MMA8452Q_ADD, CTRL_REG3, ACC_DATA, 1);	// Active High	
//	//**/	
	ACC_DATA[0] = 0x00;
	I2C_Write_Bulk(MMA8452Q_ADD, CTRL_REG4, ACC_DATA, 1);	//Data Ready Int Enabled
	//**
	ACC_DATA[0] = 0x00;
	I2C_Write_Bulk(MMA8452Q_ADD, XYZ_DATA_CFG, ACC_DATA, 1);	//	High Pass Filter Disable, range to +/- 2g
	//**
	ACC_DATA[0] = 0x13;
	I2C_Write_Bulk(MMA8452Q_ADD, HP_FILTER_CUTOFF, ACC_DATA, 1);	// LPF Enabled, High Pass Filter Cutoff 2Hz
	//**	
	ACC_DATA[0] = 0x1D;
	I2C_Write_Bulk(MMA8452Q_ADD, CTRL_REG1, ACC_DATA, 1);	// 	ODR 100Hz, Select mode register(0x2A), LNOISE - HIGH RESOLUTION MODE, Active mode(0x01)	
	//**
}


/**********************************************************************
 * @brief		CALIBRATE ACCELEROMETER
 **********************************************************************/
void calibrate_acc(void)
{	
	int i, k = 0;
	int calibrateCnt = 200;
	uint32_t I2CSlaveBuffer[6] = {0,0,0,0,0,0};
	int16_t newAcc[3];
	systick_delayMs(300);
	for (k = 0; k < calibrateCnt; k++)
	{			
		I2CSlaveBuffer[0] = I2C_Read_Reg(MMA8452Q_ADD,OUT_X_MSB);
		I2CSlaveBuffer[1] = I2C_Read_Reg(MMA8452Q_ADD,OUT_X_LSB);
		I2CSlaveBuffer[2] = I2C_Read_Reg(MMA8452Q_ADD,OUT_Y_MSB);
		I2CSlaveBuffer[3] = I2C_Read_Reg(MMA8452Q_ADD,OUT_Y_LSB);
		I2CSlaveBuffer[4] = I2C_Read_Reg(MMA8452Q_ADD,OUT_Z_MSB);
		I2CSlaveBuffer[5] = I2C_Read_Reg(MMA8452Q_ADD,OUT_Z_LSB);
		
		for (i=0; i<6; i+=2)
		{
			newAcc[i/2] = ((I2CSlaveBuffer[i] << 8) | I2CSlaveBuffer[i + 1]) >> 4;  // Turn the MSB and LSB into a 12-bit value	
			// If the number is negative, we have to make it so manually (no 12-bit data type)
			if ((newAcc[i/2] & 0x0800) != 0) {
				newAcc[i/2] -= 4096;
			}
		}	
		OFSET_ACC_X_BUF += newAcc[0] / calibrateCnt;
		OFSET_ACC_Y_BUF += newAcc[1] / calibrateCnt;
		OFSET_ACC_Z_BUF += newAcc[2] / calibrateCnt;	
		systick_delayMs(10);
	}
//	OFSET_ACC_X_BUF *= 0.0625f;
//	OFSET_ACC_Y_BUF *= 0.0625f;
//	OFSET_ACC_Z_BUF *= 0.0625f;
}


/**********************************************************************
 * @brief		READ ACCELEROMETER
 **********************************************************************/
void read_acc (void)
{
		int i=0;
		uint32_t I2CSlaveBuffer[6] = {0,0,0,0,0,0};
		
		volatile int16_t newAcc[3];
		
		I2CSlaveBuffer[0] = I2C_Read_Reg(MMA8452Q_ADD,OUT_X_MSB);
		I2CSlaveBuffer[1] = I2C_Read_Reg(MMA8452Q_ADD,OUT_X_LSB);
		I2CSlaveBuffer[2] = I2C_Read_Reg(MMA8452Q_ADD,OUT_Y_MSB);
		I2CSlaveBuffer[3] = I2C_Read_Reg(MMA8452Q_ADD,OUT_Y_LSB);
		I2CSlaveBuffer[4] = I2C_Read_Reg(MMA8452Q_ADD,OUT_Z_MSB);
		I2CSlaveBuffer[5] = I2C_Read_Reg(MMA8452Q_ADD,OUT_Z_LSB);
		
	  for (i=0; i<6; i+=2)
		{
			newAcc[i/2] = ((I2CSlaveBuffer[i] << 8) | I2CSlaveBuffer[i + 1]) >> 4;  // Turn the MSB and LSB into a 12-bit value
		
			// If the number is negative, we have to make it so manually (no 12-bit data type)
			if ((newAcc[i/2] & 0x0800) != 0) 
			{
				
				newAcc[i/2] -= 4096;
			}			
		}	
		recordToAccBuffer(newAcc[0],newAcc[1],newAcc[2]);
}

/**********************************************************************
 * @brief		RECORD TO ACCELEROMETER BUFFER
 **********************************************************************/
void recordToAccBuffer( float accX, float accY, float accZ)
{
	TMP_ACC_X_BUF[accCnt] = accX;
	TMP_ACC_Y_BUF[accCnt] = accY;
	TMP_ACC_Z_BUF[accCnt] = accZ;
	
	accCnt +=1;
	if (accCnt >= sampleNumber)
	{		
		accCnt = 0;
		filterAccData(TMP_ACC_X_BUF, TMP_ACC_Y_BUF, TMP_ACC_Z_BUF,  sampleNumber);
	}	
}

/**********************************************************************
 * @brief		FILTER ACCELEROMETER DATA
 **********************************************************************/
void filterAccData(float a[], float b[], float c[], int array_size)
{
	int i, j, k, temp = 0;
	
	ACC_X_BUF = 0;
	ACC_Y_BUF = 0;
	ACC_Z_BUF = 0;
	
	if(array_size == 1)
	{
		ACC_X_BUF = a[0];
		ACC_Y_BUF = b[0];
		ACC_Z_BUF = c[0];
	}
	else if(array_size == 2) // just mean
	{
		ACC_X_BUF = (a[0] + a[1])/2;
		ACC_Y_BUF = (b[0] + b[1])/2;
		ACC_Z_BUF = (c[0] + c[1])/2;
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
			ACC_X_BUF += a[k] / (array_size - 2);
			ACC_Y_BUF += b[k] / (array_size - 2);
			ACC_Z_BUF += c[k] / (array_size - 2);
		}
	}
}  



