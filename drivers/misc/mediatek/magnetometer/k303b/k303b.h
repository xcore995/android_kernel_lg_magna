/* linux/drivers/hwmon/K303B.c
 *
 * (C) Copyright 2008
 * MediaTek <www.mediatek.com>
 *
 * K303B driver for MT6516
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


#ifndef K303B
#define K303B
#include <linux/ioctl.h>

#define K303B_I2C_ADDRESS 	0x3C


/* Magnetometer Sensor Full Scale */
// conversion of magnetic data to uT units
#define CONVERT_M                   (1.0f/10.0f)
#define CONVERT_M_X                 (CONVERT_M)
#define CONVERT_M_Y                 (CONVERT_M)
#define CONVERT_M_Z                 (CONVERT_M)

#define	I2C_AUTO_INCREMENT	(0x80)


#define	K303B_REG_CTL0			0x1F
#define K303B_REG_DEVID			0x0F
#define	K303B_REG_BW_RATE			0x20
#define K303B_REG_DATA_FORMAT		0x21
#define K303B_REG_POWER_CTL  		0x22
#define K303B_REG_DATAX0		    0x28

#define K303B_REG_OFSX            0XFF




#define K303B_FIXED_DEVID			0x3d


#define K303B_MAG_FS_MASK	(0x60)
#define K303B_MAG_FS_16G	(0x60)	/* Full scale 16 gauss */


#define SENSITIVITY_MAG_16G	580	/**	ugauss/LSB	*/

#define K303B_MAG_POWER_ON	  (0x00)  /* POWER ON */
#define K303B_MAG_POWER_OFF	  (0x02)  /* POWER ON */


#define ODR_MAG_MASK		  (0X40)	/* Mask for odr change on mag */
//#define K303B_MAG_ODR3_125  (0x00)  /* 3.25Hz output data rate */
#define K303B_MAG_ODR_625	  (0x00)  /* 0.625Hz output data rate */
#define K303B_MAG_ODR1_25	  (0x04)  /* 1.25Hz output data rate */
#define K303B_MAG_ODR2_5	  (0x08)  /* 2.5Hz output data rate */
#define K303B_MAG_ODR5	  (0x0c)  /* 5Hz output data rate */
#define K303B_MAG_ODR10	  (0x10)  /* 10Hz output data rate */
#define K303B_MAG_ODR20	  (0x14)  /* 20Hz output data rate */
#define K303B_MAG_ODR40	  (0x18)  /* 20Hz output data rate */
#define K303B_MAG_ODR80	  (0x1c)  /* 20Hz output data rate */


#define K303B_SUCCESS						0
#define K303B_ERR_I2C						-1
#define K303B_ERR_STATUS					-3
#define K303B_ERR_SETUP_FAILURE			-4
#define K303B_ERR_GETGSENSORDATA			-5
#define K303B_ERR_IDENTIFICATION			-6



#define K303B_BUFSIZE				256


#endif
