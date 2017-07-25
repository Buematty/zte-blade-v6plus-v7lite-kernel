/* linux/drivers/hwmon/LIS3DH.c
 *
 * (C) Copyright 2008 
 * MediaTek <www.mediatek.com>
 *
 * LIS3DH driver for MT6516
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef LIS3DSH_H
#define LIS3DSH_H
	 
#include <linux/ioctl.h>

extern struct acc_hw* lis3dsh_get_cust_acc_hw(void) ;

#define LIS3DSH_I2C_SLAVE_ADDR		0x3C
	 
/* LIS3DSH Register Map  (Please refer to LIS3DSH Specifications) */
#define LIS3DSH_REG_CTL_REG_4		0x20
#define LIS3DSH_REG_CTL_REG_5		0x24
#define LIS3DSH_REG_CTL_REG_3		0x23
#define LIS3DSH_REG_CTL_REG3		0x22

#define LIS3DSH_REG_DATAX0			0x28
#define LIS3DSH_REG_OUT_X			0x28
#define LIS3DSH_REG_OUT_Y			0x2A
#define LIS3DSH_REG_OUT_Z			0x2C

#define LIS3DSH_REG_DEVID			0x0F
#define WHO_AM_I 					0x3F
	 
#define LIS3DSH_BW_400HZ			0x70
#define LIS3DSH_BW_100HZ			0x60
#define LIS3DSH_BW_50HZ			0x50

#define	LIS3DSH_FULLRANG_LSB		0XFF
	 
#define LIS3DSH_MEASURE_MODE		0x08 
#define LIS3DSH_DATA_READY			0x07
	 
#define LIS3DSH_RANGE_2G			0x00
#define LIS3DSH_RANGE_4G			0x08
#define LIS3DSH_RANGE_6G			0x10

#define LIS3DSH_RANGE_8G			0x18
#define LIS3DSH_RANGE_16G			0x20

#define LIS3DSH_SELF_TEST           	0x10
	 
#define LIS3DSH_STREAM_MODE		0x80
#define LIS3DSH_SAMPLES_15			0x0F
#define LIS3DSH_LEFT_JUSTIFY		0x04
#define LIS3DSH_RIGHT_JUSTIFY		0x00

#define	I2C_AUTO_INCREMENT	0x80		/* Autoincrement i2c address */

#define	LIS3DSH_ACC_FS_MASK	(0x38)

/* Output Data Rates ODR */
#define LIS3DSH_ODR_MASK		0XF0
#define LIS3DSH_PM_OFF			0x00		/* OFF */
#define LIS3DSH_ODR3_125		0x10		/*    3.125 Hz */
#define LIS3DSH_ODR6_25		0x20		/*    6.25  Hz */
#define LIS3DSH_ODR12_5		0x30		/*   12.5   Hz */
#define LIS3DSH_ODR25			0x40		/*   25     Hz */
#define LIS3DSH_ODR50			0x50		/*   50     Hz */
#define LIS3DSH_ODR100			0x60		/*  100     Hz */
#define LIS3DSH_ODR400			0x70		/*  400     Hz */
#define LIS3DSH_ODR800			0x80		/*  800     Hz */
#define LIS3DSH_ODR1600		0x90		/* 1600     Hz */

/* Registers configuration Mask and settings */
/* CTRLREG1 */
#define LIS3DSH_INTEN_MASK		0x01
#define LIS3DSH_INTEN_OFF		0x00
#define LIS3DSH_INTEN_ON		0x01

/* CTRLREG2 */
#define LIS3DSH_HIST1_MASK			0xE0
#define LIS3DSH_SM1INT_PIN_MASK	0x08
#define LIS3DSH_SM1INT_PINB		0x08
#define LIS3DSH_SM1INT_PINA		0x00
#define LIS3DSH_SM1_EN_MASK		0x01
#define LIS3DSH_SM1_EN_ON			0x01
#define LIS3DSH_SM1_EN_OFF			0x00

/* CTRLREG3 */
#define LIS3DSH_HIST2_MASK			0xE0
#define LIS3DSH_SM2INT_PIN_MASK	0x08
#define LIS3DSH_SM2INT_PINB		0x08
#define LIS3DSH_SM2INT_PINA		0x00
#define LIS3DSH_SM2_EN_MASK		0x01
#define LIS3DSH_SM2_EN_ON			0x01
#define LIS3DSH_SM2_EN_OFF			0x00

/* CTRLREG4 */
#define LIS3DSH_INT_ACT_MASK		(0x01 << 6)
#define LIS3DSH_INT_ACT_H			(0x01 << 6)
#define LIS3DSH_INT_ACT_L			0x00
#define LIS3DSH_INT2_EN_MASK		(0x01 << 4)
#define LIS3DSH_INT2_EN_ON			(0x01 << 4)
#define LIS3DSH_INT2_EN_OFF		0x00
#define LIS3DSH_INT1_EN_MASK		(0x01 << 3)
#define LIS3DSH_INT1_EN_ON			(0x01 << 3)
#define LIS3DSH_INT1_EN_OFF		0x00

#define OUT_AXISDATA_REG			LIS3DSH_OUTX_L
#define WHOAMI_LIS3DSH_ACC		0x40	/* Expected content for WAI */

/*	CONTROL REGISTERS	*/
#define LIS3DSH_WHO_AM_I			0x0F	/* WhoAmI register Address */

#define LIS3DSH_OUTX_L			0x28	/* Output X LSByte */
#define LIS3DSH_OUTX_H			0x29	/* Output X MSByte */
#define LIS3DSH_OUTY_L			0x2A	/* Output Y LSByte */
#define LIS3DSH_OUTY_H			0x2B	/* Output Y MSByte */
#define LIS3DSH_OUTZ_L			0x2C	/* Output Z LSByte */
#define LIS3DSH_OUTZ_H			0x2D	/* Output Z MSByte */
#define LIS3DSH_LC_L			0x16	/* LSByte Long Counter Status */
#define LIS3DSH_LC_H			0x17	/* MSByte Long Counter Status */

#define LIS3DSH_STATUS_REG		0x27	/* Status */

#define LIS3DSH_CTRL_REG1		0x21	/* control reg 1 */
#define LIS3DSH_CTRL_REG2		0x22	/* control reg 2 */
#define LIS3DSH_CTRL_REG3		0x23	/* control reg 3 */
#define LIS3DSH_CTRL_REG4		0x20	/* control reg 4 */
#define LIS3DSH_CTRL_REG5		0x24	/* control reg 3 */
#define LIS3DSH_CTRL_REG6		0x25	/* control reg 4 */

#define LIS3DSH_OFF_X			0x10	/* Offset X Corr */
#define LIS3DSH_OFF_Y			0x11	/* Offset Y Corr */
#define LIS3DSH_OFF_Z			0x12	/* Offset Z Corr */

#define LIS3DSH_CS_X			0x13	/* Const Shift X */
#define LIS3DSH_CS_Y			0x14	/* Const Shift Y */
#define LIS3DSH_CS_Z			0x15	/* Const Shift Z */

#define LIS3DSH_VFC_1			0x1B	/* Vect Filter Coeff 1 */
#define LIS3DSH_VFC_2			0x1C	/* Vect Filter Coeff 2 */
#define LIS3DSH_VFC_3			0x1D	/* Vect Filter Coeff 3 */
#define LIS3DSH_VFC_4			0x1E	/* Vect Filter Coeff 4 */

/* RESUME STATE INDICES */
#define LIS3DSH_RES_LC_L			0
#define LIS3DSH_RES_LC_H			1

#define LIS3DSH_RES_CTRL_REG1		2
#define LIS3DSH_RES_CTRL_REG2		3
#define LIS3DSH_RES_CTRL_REG3		4
#define LIS3DSH_RES_CTRL_REG4		5
#define LIS3DSH_RES_CTRL_REG5		6

#define LIS3DSH_RES_TIM4_1			20
#define LIS3DSH_RES_TIM3_1			21
#define LIS3DSH_RES_TIM2_1_L		22
#define LIS3DSH_RES_TIM2_1_H		23
#define LIS3DSH_RES_TIM1_1_L		24
#define LIS3DSH_RES_TIM1_1_H		25

#define LIS3DSH_RES_THRS2_1		26
#define LIS3DSH_RES_THRS1_1		27
#define LIS3DSH_RES_SA_1			28
#define LIS3DSH_RES_MA_1			29
#define LIS3DSH_RES_SETT_1			30

#define LIS3DSH_RES_TIM4_2			31
#define LIS3DSH_RES_TIM3_2			32
#define LIS3DSH_RES_TIM2_2_L		33
#define LIS3DSH_RES_TIM2_2_H		34
#define LIS3DSH_RES_TIM1_2_L		35
#define LIS3DSH_RES_TIM1_2_H		36

#define LIS3DSH_RES_THRS2_2		37
#define LIS3DSH_RES_THRS1_2		38
#define LIS3DSH_RES_DES_2			39
#define LIS3DSH_RES_SA_2			40
#define LIS3DSH_RES_MA_2			41
#define LIS3DSH_RES_SETT_2			42

#define LIS3DSH_RESUME_ENTRIES	43

#define LIS3DSH_STATE_PR_SIZE		16

/* STATE PROGRAMS ENABLE CONTROLS */
#define LIS3DSH_SM1_DIS_SM2_DIS		0x00
#define LIS3DSH_SM1_DIS_SM2_EN		0x01
#define LIS3DSH_SM1_EN_SM2_DIS		0x02
#define LIS3DSH_SM1_EN_SM2_EN		0x03

/* INTERRUPTS ENABLE CONTROLS */
#define LIS3DSH_INT1_DIS_INT2_DIS		0x00
#define LIS3DSH_INT1_DIS_INT2_EN		0x01
#define LIS3DSH_INT1_EN_INT2_DIS		0x02
#define LIS3DSH_INT1_EN_INT2_EN		0x03

#define LIS3DSH_SUCCESS					0
#define LIS3DSH_ERR_I2C					-1
#define LIS3DSH_ERR_STATUS				-3
#define LIS3DSH_ERR_SETUP_FAILURE		-4
#define LIS3DSH_ERR_GETGSENSORDATA	-5
#define LIS3DSH_ERR_IDENTIFICATION	-6
 
#define LIS3DSH_BUFSIZE					256
	 
#endif
