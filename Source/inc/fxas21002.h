/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//#include <sensor.h>
#include <i2c.h>
#include <gpio.h>

//#define M_PI   3.14159265358979323
#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)



#define FXAS21002_ADD 0x40
#define FXAS21002_WHO_AM_I_ADD 0x0C

#define FXAS21002_REG_STATUS			0x00
#define FXAS21002_REG_OUTXMSB			0x01
#define FXAS21002_REG_OUTXLSB			0x02
#define FXAS21002_REG_OUTYMSB			0x03
#define FXAS21002_REG_OUTYLSB			0x04
#define FXAS21002_REG_OUTZMSB			0x05
#define FXAS21002_REG_OUTZLSB			0x06
#define FXAS21002_REG_DR_STATUS		0x07
#define FXAS21002_REG_F_STATUS		0x08
#define FXAS21002_REG_F_SETUP			0x09
#define FXAS21002_REG_F_EVENT			0x0A
#define FXAS21002_REG_INT_SOURCE	0x0b
#define FXAS21002_REG_CTRLREG0		0x0d
#define FXAS21002_REG_RT_CFG			0x0e
#define FXAS21002_REG_RT_SRC			0x0f
#define FXAS21002_RT_THS					0x10
#define FXAS21002_REG_RT_COUNT		0x11
#define FXAS21002_REG_TEMP				0x12
#define FXAS21002_REG_CTRLREG1		0x13
#define FXAS21002_REG_CTRLREG2		0x14
#define FXAS21002_REG_CTRLREG3		0x15

#define FXAS21002_INT_SOURCE_DRDY_MASK	(1 << 0)

#define FXAS21002_CTRLREG0_FS_MASK	(3 << 0)

#define FXAS21002_CTRLREG1_DR_SHIFT	2

#define FXAS21002_CTRLREG1_POWER_MASK	(3 << 0)
#define FXAS21002_CTRLREG1_DR_MASK	(7 << FXAS21002_CTRLREG1_DR_SHIFT)
#define FXAS21002_CTRLREG1_RST_MASK	(1 << 6)

#define FXAS21002_CTRLREG2_CFG_EN_MASK		(1 << 2)
#define FXAS21002_CTRLREG2_CFG_DRDY_MASK	(1 << 3)

#define FXAS21002_MAX_NUM_CHANNELS	3

#define FXAS21002_BYTES_PER_CHANNEL	2

#define FXAS21002_MAX_NUM_BYTES		(FXAS21002_BYTES_PER_CHANNEL * \
					 FXAS21002_MAX_NUM_CHANNELS)

enum fxas21002_power {
	FXAS21002_POWER_STANDBY		= 0,
	FXAS21002_POWER_READY     = 1,
	FXAS21002_POWER_ACTIVE    = 3,
};

enum fxas21002_range {
	FXAS21002_RANGE_2000DPS		= 0,
	FXAS21002_RANGE_1000DPS,
	FXAS21002_RANGE_500DPS,
	FXAS21002_RANGE_250DPS,
};

enum fxas21002_channel {
	FXAS21002_CHANNEL_GYRO_X	= 0,
	FXAS21002_CHANNEL_GYRO_Y,
	FXAS21002_CHANNEL_GYRO_Z,
};

extern void read_gyro (void);
extern void start_gyro (void);
extern void checkGyro(void);
extern void gyro_low_power_mode (void);

extern float GYRO_X_BUF;
extern float GYRO_Y_BUF;
extern float GYRO_Z_BUF;
