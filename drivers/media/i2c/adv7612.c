/*
 * drivers/media/video/adv7612.c
 *
 * Copyright (C) 2013-2014 Renesas Electronics Corporation
 *
 * adv7612.c Analog Devices ADV7612 HDMI receiver driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <media/v4l2-ioctl.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-chip-ident.h>
#include <linux/mutex.h>
#include <media/soc_camera.h>

#define DRIVER_NAME "adv7612"

/****************************************/
/* ADV7612 I2C slave address definition */
/****************************************/
#define ADV7612_I2C_IO			0x4C	/* IO Map */
#define ADV7612_I2C_CEC			0x40	/* CEC Map */
#define ADV7612_I2C_INFOFRAME	0x3E	/* InfoFrame Map */
#define ADV7612_I2C_DPLL		0x26	/* DPLL Map */
#define ADV7612_I2C_KSV			0x32	/* KSV(Repeater) Map */
#define ADV7612_I2C_EDID		0x36	/* EDID Map */
#define ADV7612_I2C_HDMI		0x34	/* HDMI Map */
#define ADV7612_I2C_CP			0x22	/* CP Map */
#define ADV7612_I2C_EOR			0xFF	/* End Mark */


/****************************************/
/* ADV7612 IO register definition       */
/****************************************/

/* Power Management */
#define ADV7612_IO_PWR_MAN_REG	0x0C	/* Power management register */
#define ADV7612_IO_PWR_ON		0x42	/* Power on */
#define ADV7612_IO_PWR_OFF		0x62	/* Power down */


/****************************************/
/* ADV7612 CP register definition       */
/****************************************/

/* Contrast Control */
#define ADV7612_CP_CON_REG	0x3a	/* Contrast (unsigned) */
#define ADV7612_CP_CON_MIN	0		/* Minimum contrast */
#define ADV7612_CP_CON_DEF	128		/* Default */
#define ADV7612_CP_CON_MAX	255		/* Maximum contrast */

/* Saturation Control */
#define ADV7612_CP_SAT_REG	0x3b	/* Saturation (unsigned) */
#define ADV7612_CP_SAT_MIN	0		/* Minimum saturation */
#define ADV7612_CP_SAT_DEF	128		/* Default */
#define ADV7612_CP_SAT_MAX	255		/* Maximum saturation */

/* Brightness Control */
#define ADV7612_CP_BRI_REG	0x3c	/* Brightness (signed) */
#define ADV7612_CP_BRI_MIN	-128	/* Luma is -512d */
#define ADV7612_CP_BRI_DEF	0		/* Luma is 0 */
#define ADV7612_CP_BRI_MAX	127		/* Luma is 508d */

/* Hue Control */
#define ADV7612_CP_HUE_REG	0x3d	/* Hue (unsigned) */
#define ADV7612_CP_HUE_MIN	0		/* -90 degree */
#define ADV7612_CP_HUE_DEF	0		/* -90 degree */
#define ADV7612_CP_HUE_MAX	255		/* +90 degree */

/* Video adjustment register */
#define ADV7612_CP_VID_ADJ_REG		0x3e
/* Video adjustment mask */
#define ADV7612_CP_VID_ADJ_MASK		0x7F
/* Enable color controls */
#define ADV7612_CP_VID_ADJ_ENABLE	0x80


/****************************************/
/* ADV7612 HDMI register definition     */
/****************************************/

/* HDMI status register */
#define ADV7612_HDMI_STATUS1_REG		0x07
/* VERT_FILTER_LOCKED flag */
#define ADV7612_HDMI_VF_LOCKED_FLG		0x80
/* DE_REGEN_FILTER_LOCKED flag */
#define ADV7612_HDMI_DERF_LOCKED_FLG	0x20
/* LINE_WIDTH[12:8] mask */
#define ADV7612_HDMI_LWIDTH_MSBS_MASK	0x1F

/* LINE_WIDTH[7:0] register */
#define ADV7612_HDMI_LWIDTH_REG			0x08

/* FIELD0_HEIGHT[12:8] register */
#define ADV7612_HDMI_F0HEIGHT_MSBS_REG	0x09
/* FIELD0_HEIGHT[12:8] mask */
#define ADV7612_HDMI_F0HEIGHT_MSBS_MASK	0x1F

/* FIELD0_HEIGHT[7:0] register */
#define ADV7612_HDMI_F0HEIGHT_LSBS_REG	0x0A

/* HDMI status register */
#define ADV7612_HDMI_STATUS2_REG		0x0B
/* DEEP_COLOR_MODE[1:0] mask */
#define ADV7612_HDMI_DCM_MASK			0xC0
/* HDMI_INTERLACED flag */
#define ADV7612_HDMI_IP_FLAG			0x20
/* FIELD1_HEIGHT[12:8] mask */
#define ADV7612_HDMI_F1HEIGHT_MSBS_MASK	0x1F

/* FIELD1_HEIGHT[7:0] register */
#define ADV7612_HDMI_F1HEIGHT_REG		0x0C



/****************************************/
/* ADV7612 other definition             */
/****************************************/

#define ADV7612_MAX_WIDTH		1920
#define ADV7612_MAX_HEIGHT		1080



/****************************************/
/* ADV7612 structure definition         */
/****************************************/

/* Structure for register values */
struct adv7612_reg_value {
	u8 addr;				/* i2c slave address */
	u8 reg;					/* sub (register) address */
	u8 value;				/* register value */
};

/* Register default values */
/* RGB color space output  */
static const struct adv7612_reg_value adv7612_init_defaults[] = {

	/* I2C Slave Address settings */
	{ADV7612_I2C_IO, 0xF4, ADV7612_I2C_CEC*2},	/* CEC Map */
	{ADV7612_I2C_IO, 0xF5, ADV7612_I2C_INFOFRAME*2},/* INFOFRAME Map */
	{ADV7612_I2C_IO, 0xF8, ADV7612_I2C_DPLL*2},	/* DPLL Map */
	{ADV7612_I2C_IO, 0xF9, ADV7612_I2C_KSV*2},	/* KSV Map */
	{ADV7612_I2C_IO, 0xFA, ADV7612_I2C_EDID*2},	/* EDID Map */
	{ADV7612_I2C_IO, 0xFB, ADV7612_I2C_HDMI*2},	/* HDMI Map */
	{ADV7612_I2C_IO, 0xFD, ADV7612_I2C_CP*2},	/* CP Map */

	{ADV7612_I2C_IO, 0x01, 0x06},	/* V-FREQ = 60Hz, */
					/* Prim_Mode = HDMI-GR */
	{ADV7612_I2C_IO, 0x02, 0xF2},	/* Auto CSC, RGB out, */
					/* Disable op_656 bit */
	{ADV7612_I2C_IO, 0x03, 0x42},	/* 36 bit SDR 444 Mode 0 */
	{ADV7612_I2C_IO, 0x05, 0x28},	/* AV Codes Off */
	{ADV7612_I2C_IO, 0x0B, 0x44},	/* Power up part */
	{ADV7612_I2C_IO, 0x0C, 0x42},	/* Power up part */
	{ADV7612_I2C_IO, 0x14, 0x7F},	/* Max Drive Strength */
	{ADV7612_I2C_IO, 0x15, 0x80},	/* Disable Tristate of Pins */
					/*  (Audio output pins active) */
	{ADV7612_I2C_IO, 0x19, 0x83},	/* LLC DLL phase */
	{ADV7612_I2C_IO, 0x33, 0x40},	/* LLC DLL enable */

	{ADV7612_I2C_CP, 0xBA, 0x01},	/* Set HDMI FreeRun */

	{ADV7612_I2C_KSV, 0x40, 0x81},	/* Disable HDCP 1.1 features */

	{ADV7612_I2C_HDMI, 0x9B, 0x03},	/* ADI recommended setting */
	{ADV7612_I2C_HDMI, 0x00, 0x08},	/* Set HDMI Input Port A */
					/*  (BG_MEAS_PORT_SEL = 001b) */
	{ADV7612_I2C_HDMI, 0x02, 0x03},	/* Enable Ports A & B in */
					/* background mode */
	{ADV7612_I2C_HDMI, 0x6D, 0x80},	/* Enable TDM mode */
	{ADV7612_I2C_HDMI, 0x03, 0x18},	/* I2C mode 24 bits */
	{ADV7612_I2C_HDMI, 0x83, 0xFC},	/* Enable clock terminators */
					/* for port A & B */
	{ADV7612_I2C_HDMI, 0x6F, 0x0C},	/* ADI recommended setting */
	{ADV7612_I2C_HDMI, 0x85, 0x1F},	/* ADI recommended setting */
	{ADV7612_I2C_HDMI, 0x87, 0x70},	/* ADI recommended setting */
	{ADV7612_I2C_HDMI, 0x8D, 0x04},	/* LFG Port A */
	{ADV7612_I2C_HDMI, 0x8E, 0x1E},	/* HFG Port A */
	{ADV7612_I2C_HDMI, 0x1A, 0x8A},	/* unmute audio */
	{ADV7612_I2C_HDMI, 0x57, 0xDA},	/* ADI recommended setting */
	{ADV7612_I2C_HDMI, 0x58, 0x01},	/* ADI recommended setting */
	{ADV7612_I2C_HDMI, 0x75, 0x10},	/* DDC drive strength */
	{ADV7612_I2C_HDMI, 0x90, 0x04},	/* LFG Port B */
	{ADV7612_I2C_HDMI, 0x91, 0x1E},	/* HFG Port B */

	{ADV7612_I2C_HDMI, 0x04, 0x03},
	{ADV7612_I2C_HDMI, 0x14, 0x00},
	{ADV7612_I2C_HDMI, 0x15, 0x00},
	{ADV7612_I2C_HDMI, 0x16, 0x00},

	{ADV7612_I2C_INFOFRAME, 0x1C, 0x11},	/* PCM,IEC 60958-3[13], */
						/* 2 channels */
	{ADV7612_I2C_INFOFRAME, 0x1D, 0x0F},	/* 48 kHz, 24 bit */
	{ADV7612_I2C_INFOFRAME, 0x1E, 0x00},	/* Extension(unused) */
	{ADV7612_I2C_INFOFRAME, 0x1F, 0x00},	/* Channel Number=0(FL, FR) */
	{ADV7612_I2C_INFOFRAME, 0x20, 0x00},	/* Level Shift Value=0dB */
	{ADV7612_I2C_INFOFRAME, 0x21, 0x00},
	{ADV7612_I2C_INFOFRAME, 0x22, 0x00},
	{ADV7612_I2C_INFOFRAME, 0x23, 0x00},
	{ADV7612_I2C_INFOFRAME, 0x24, 0x00},
	{ADV7612_I2C_INFOFRAME, 0x25, 0x00},
	{ADV7612_I2C_INFOFRAME, 0x26, 0x00},
	{ADV7612_I2C_INFOFRAME, 0x27, 0x00},
	{ADV7612_I2C_INFOFRAME, 0x28, 0x00},
	{ADV7612_I2C_INFOFRAME, 0x29, 0x00},

	{ADV7612_I2C_KSV, 0x77, 0x00},	/* Disable the Internal EDID */
					/* for all ports */
	/* EDID */
	/* Header information(0-19th byte) */
	{ADV7612_I2C_EDID, 0x00, 0x00},	/* Fixed header pattern */
	{ADV7612_I2C_EDID, 0x01, 0xFF},
	{ADV7612_I2C_EDID, 0x02, 0xFF},
	{ADV7612_I2C_EDID, 0x03, 0xFF},
	{ADV7612_I2C_EDID, 0x04, 0xFF},
	{ADV7612_I2C_EDID, 0x05, 0xFF},
	{ADV7612_I2C_EDID, 0x06, 0xFF},
	{ADV7612_I2C_EDID, 0x07, 0x00},
	{ADV7612_I2C_EDID, 0x08, 0x00},	/* Manufacturer ID */
	{ADV7612_I2C_EDID, 0x09, 0x00},
	{ADV7612_I2C_EDID, 0x0A, 0x00},	/* Manufacturer product code */
	{ADV7612_I2C_EDID, 0x0B, 0x00},
	{ADV7612_I2C_EDID, 0x0C, 0x00},	/* Serial number */
	{ADV7612_I2C_EDID, 0x0D, 0x00},
	{ADV7612_I2C_EDID, 0x0E, 0x00},
	{ADV7612_I2C_EDID, 0x0F, 0x00},
	{ADV7612_I2C_EDID, 0x10, 0x01},	/* Week of manufacture */
	{ADV7612_I2C_EDID, 0x11, 0x0C},	/* Year of manufacture */
	{ADV7612_I2C_EDID, 0x12, 0x01},	/* EDID version */
	{ADV7612_I2C_EDID, 0x13, 0x03},
	/* Basic display parameters(20-24th byte) */
	{ADV7612_I2C_EDID, 0x14, 0x80},	/* Video input parameters bitmap */
	{ADV7612_I2C_EDID, 0x15, 0x50},	/* Maximum horizontal image size */
	{ADV7612_I2C_EDID, 0x16, 0x2D},	/* Maximum vertical image size */
	{ADV7612_I2C_EDID, 0x17, 0x78},	/* Display gamma */
	{ADV7612_I2C_EDID, 0x18, 0x0A},	/* Supported features bitmap */
	/* Chromaticity coordinates(25-34th byte) */
	{ADV7612_I2C_EDID, 0x19, 0x0D},
	{ADV7612_I2C_EDID, 0x1A, 0xC9},
	{ADV7612_I2C_EDID, 0x1B, 0xA0},
	{ADV7612_I2C_EDID, 0x1C, 0x57},
	{ADV7612_I2C_EDID, 0x1D, 0x47},
	{ADV7612_I2C_EDID, 0x1E, 0x98},
	{ADV7612_I2C_EDID, 0x1F, 0x27},
	{ADV7612_I2C_EDID, 0x20, 0x12},
	{ADV7612_I2C_EDID, 0x21, 0x48},
	{ADV7612_I2C_EDID, 0x22, 0x4C},
	/* Established timing bitmap(35-37th byte) */
	{ADV7612_I2C_EDID, 0x23, 0x20},
	{ADV7612_I2C_EDID, 0x24, 0x00},
	{ADV7612_I2C_EDID, 0x25, 0x00},
	/* Standard timing information(38-53th byte) */
	/* Because they are unused, in this field, all values are 0101h. */
	{ADV7612_I2C_EDID, 0x26, 0x01},
	{ADV7612_I2C_EDID, 0x27, 0x01},
	{ADV7612_I2C_EDID, 0x28, 0x01},
	{ADV7612_I2C_EDID, 0x29, 0x01},
	{ADV7612_I2C_EDID, 0x2A, 0x01},
	{ADV7612_I2C_EDID, 0x2B, 0x01},
	{ADV7612_I2C_EDID, 0x2C, 0x01},
	{ADV7612_I2C_EDID, 0x2D, 0x01},
	{ADV7612_I2C_EDID, 0x2E, 0x01},
	{ADV7612_I2C_EDID, 0x2F, 0x01},
	{ADV7612_I2C_EDID, 0x30, 0x01},
	{ADV7612_I2C_EDID, 0x31, 0x01},
	{ADV7612_I2C_EDID, 0x32, 0x01},
	{ADV7612_I2C_EDID, 0x33, 0x01},
	{ADV7612_I2C_EDID, 0x34, 0x01},
	{ADV7612_I2C_EDID, 0x35, 0x01},
	/* Descriptor blocks of Descriptor 1(54-71th byte) */
	{ADV7612_I2C_EDID, 0x36, 0x01},
	{ADV7612_I2C_EDID, 0x37, 0x1D},
	{ADV7612_I2C_EDID, 0x38, 0x80},
	{ADV7612_I2C_EDID, 0x39, 0x18},
	{ADV7612_I2C_EDID, 0x3A, 0x71},
	{ADV7612_I2C_EDID, 0x3B, 0x1C},
	{ADV7612_I2C_EDID, 0x3C, 0x16},
	{ADV7612_I2C_EDID, 0x3D, 0x20},
	{ADV7612_I2C_EDID, 0x3E, 0x58},
	{ADV7612_I2C_EDID, 0x3F, 0x2C},
	{ADV7612_I2C_EDID, 0x40, 0x25},
	{ADV7612_I2C_EDID, 0x41, 0x00},
	{ADV7612_I2C_EDID, 0x42, 0x20},
	{ADV7612_I2C_EDID, 0x43, 0xC2},
	{ADV7612_I2C_EDID, 0x44, 0x31},
	{ADV7612_I2C_EDID, 0x45, 0x00},
	{ADV7612_I2C_EDID, 0x46, 0x00},
	{ADV7612_I2C_EDID, 0x47, 0x98},
	/* Descriptor blocks of Descriptor 2(72-89th byte) */
	{ADV7612_I2C_EDID, 0x48, 0x8C},
	{ADV7612_I2C_EDID, 0x49, 0x0A},
	{ADV7612_I2C_EDID, 0x4A, 0xD0},
	{ADV7612_I2C_EDID, 0x4B, 0x8A},
	{ADV7612_I2C_EDID, 0x4C, 0x20},
	{ADV7612_I2C_EDID, 0x4D, 0xE0},
	{ADV7612_I2C_EDID, 0x4E, 0x2D},
	{ADV7612_I2C_EDID, 0x4F, 0x10},
	{ADV7612_I2C_EDID, 0x50, 0x10},
	{ADV7612_I2C_EDID, 0x51, 0x3E},
	{ADV7612_I2C_EDID, 0x52, 0x96},
	{ADV7612_I2C_EDID, 0x53, 0x00},
	{ADV7612_I2C_EDID, 0x54, 0x58},
	{ADV7612_I2C_EDID, 0x55, 0xC2},
	{ADV7612_I2C_EDID, 0x56, 0x21},
	{ADV7612_I2C_EDID, 0x57, 0x00},
	{ADV7612_I2C_EDID, 0x58, 0x00},
	{ADV7612_I2C_EDID, 0x59, 0x18},
	/* Descriptor blocks of Descriptor 3(90-107th byte) */
	{ADV7612_I2C_EDID, 0x5A, 0x00},
	{ADV7612_I2C_EDID, 0x5B, 0x00},
	{ADV7612_I2C_EDID, 0x5C, 0x00},
	{ADV7612_I2C_EDID, 0x5D, 0xFC},
	{ADV7612_I2C_EDID, 0x5E, 0x00},
	{ADV7612_I2C_EDID, 0x5F, 0x4D},
	{ADV7612_I2C_EDID, 0x60, 0x59},
	{ADV7612_I2C_EDID, 0x61, 0x20},
	{ADV7612_I2C_EDID, 0x62, 0x48},
	{ADV7612_I2C_EDID, 0x63, 0x44},
	{ADV7612_I2C_EDID, 0x64, 0x54},
	{ADV7612_I2C_EDID, 0x65, 0x56},
	{ADV7612_I2C_EDID, 0x66, 0x0A},
	{ADV7612_I2C_EDID, 0x67, 0x20},
	{ADV7612_I2C_EDID, 0x68, 0x20},
	{ADV7612_I2C_EDID, 0x69, 0x20},
	{ADV7612_I2C_EDID, 0x6A, 0x20},
	{ADV7612_I2C_EDID, 0x6B, 0x20},
	/* Descriptor blocks of Descriptor 4(108-125th byte) */
	{ADV7612_I2C_EDID, 0x6C, 0x00},
	{ADV7612_I2C_EDID, 0x6D, 0x00},
	{ADV7612_I2C_EDID, 0x6E, 0x00},
	{ADV7612_I2C_EDID, 0x6F, 0xFD},
	{ADV7612_I2C_EDID, 0x70, 0x00},
	{ADV7612_I2C_EDID, 0x71, 0x3B},
	{ADV7612_I2C_EDID, 0x72, 0x3D},
	{ADV7612_I2C_EDID, 0x73, 0x0F},
	{ADV7612_I2C_EDID, 0x74, 0x2E},
	{ADV7612_I2C_EDID, 0x75, 0x08},
	{ADV7612_I2C_EDID, 0x76, 0x00},
	{ADV7612_I2C_EDID, 0x77, 0x0A},
	{ADV7612_I2C_EDID, 0x78, 0x20},
	{ADV7612_I2C_EDID, 0x79, 0x20},
	{ADV7612_I2C_EDID, 0x7A, 0x20},
	{ADV7612_I2C_EDID, 0x7B, 0x20},
	{ADV7612_I2C_EDID, 0x7C, 0x20},
	{ADV7612_I2C_EDID, 0x7D, 0x20},
	/* Number of extensions to follow(126th byte) */
	{ADV7612_I2C_EDID, 0x7E, 0x01},	/* Extension enable */
	/* Checksum(127th byte) */
	{ADV7612_I2C_EDID, 0x7F, 0x75},

	/* CEA EDID Timing Extension Version 3 */
	{ADV7612_I2C_EDID, 0x80, 0x02},	/* CEA 861 Externsion Block */
	{ADV7612_I2C_EDID, 0x81, 0x03},
	{ADV7612_I2C_EDID, 0x82, 0x1E},	/* 0x0C==>0x8C (0x1E==>0x9E, */
					/* 0x26==>0xA6, 0x34==>0xB4) */
	{ADV7612_I2C_EDID, 0x83, 0x40},
	{ADV7612_I2C_EDID, 0x84, 0x83},	/* Speaker, 3byte */
	{ADV7612_I2C_EDID, 0x85, 0x7F},	/* 7.1ch */
	{ADV7612_I2C_EDID, 0x86, 0x00},
	{ADV7612_I2C_EDID, 0x87, 0x00},
	{ADV7612_I2C_EDID, 0x88, 0x35},	/* Audio, 21byte(3byte x 7) */
	{ADV7612_I2C_EDID, 0x89, 0x0F},	/* LPCM 8ch    (0x0F) */
	{ADV7612_I2C_EDID, 0x8A, 0x06},	/* 48/44.1KHz  (0x7F) */
	{ADV7612_I2C_EDID, 0x8B, 0x07},	/* 16/20/24bit */
	{ADV7612_I2C_EDID, 0x8C, 0x17},	/* AC-3 */
	{ADV7612_I2C_EDID, 0x8D, 0x1F},
	{ADV7612_I2C_EDID, 0x8E, 0x38},
	{ADV7612_I2C_EDID, 0x8F, 0x1F},	/* MPEG1 */
	{ADV7612_I2C_EDID, 0x90, 0x07},
	{ADV7612_I2C_EDID, 0x91, 0x30},
	{ADV7612_I2C_EDID, 0x92, 0x2F},	/* MPEG2 */
	{ADV7612_I2C_EDID, 0x93, 0x07},
	{ADV7612_I2C_EDID, 0x94, 0x72},
	{ADV7612_I2C_EDID, 0x95, 0x3F},	/* DTS */
	{ADV7612_I2C_EDID, 0x96, 0x7F},
	{ADV7612_I2C_EDID, 0x97, 0x72},
	{ADV7612_I2C_EDID, 0x98, 0x57},	/* DD+ */
	{ADV7612_I2C_EDID, 0x99, 0x7F},
	{ADV7612_I2C_EDID, 0x9A, 0x00},
	{ADV7612_I2C_EDID, 0x9B, 0x37},	/* AAC */
	{ADV7612_I2C_EDID, 0x9C, 0x7F},
	{ADV7612_I2C_EDID, 0x9D, 0x72},
	{ADV7612_I2C_EDID, 0x9E, 0x67},	/* Vender Specific, 7byte */
	{ADV7612_I2C_EDID, 0x9F, 0x03},
	{ADV7612_I2C_EDID, 0xA0, 0x0C},
	{ADV7612_I2C_EDID, 0xA1, 0x00},
	{ADV7612_I2C_EDID, 0xA2, 0x10},
	{ADV7612_I2C_EDID, 0xA3, 0x00},
	{ADV7612_I2C_EDID, 0xA4, 0x88},
	{ADV7612_I2C_EDID, 0xA5, 0x2D},
	{ADV7612_I2C_EDID, 0xA6, 0x4D},	/* Video, 13byte */
	{ADV7612_I2C_EDID, 0xA7, 0x82},
	{ADV7612_I2C_EDID, 0xA8, 0x05},
	{ADV7612_I2C_EDID, 0xA9, 0x04},
	{ADV7612_I2C_EDID, 0xAA, 0x01},
	{ADV7612_I2C_EDID, 0xAB, 0x10},
	{ADV7612_I2C_EDID, 0xAC, 0x11},
	{ADV7612_I2C_EDID, 0xAD, 0x14},
	{ADV7612_I2C_EDID, 0xAE, 0x13},
	{ADV7612_I2C_EDID, 0xAF, 0x1F},
	{ADV7612_I2C_EDID, 0xB0, 0x06},
	{ADV7612_I2C_EDID, 0xB1, 0x15},
	{ADV7612_I2C_EDID, 0xB2, 0x03},
	{ADV7612_I2C_EDID, 0xB3, 0x12},
	{ADV7612_I2C_EDID, 0xB4, 0x00},
	{ADV7612_I2C_EDID, 0xB5, 0x00},
	{ADV7612_I2C_EDID, 0xB6, 0x00},
	{ADV7612_I2C_EDID, 0xB7, 0xFF},
	{ADV7612_I2C_EDID, 0xB8, 0x00},
	{ADV7612_I2C_EDID, 0xB9, 0x0A},
	{ADV7612_I2C_EDID, 0xBA, 0x20},
	{ADV7612_I2C_EDID, 0xBB, 0x20},
	{ADV7612_I2C_EDID, 0xBC, 0x20},
	{ADV7612_I2C_EDID, 0xBD, 0x20},
	{ADV7612_I2C_EDID, 0xBE, 0x20},
	{ADV7612_I2C_EDID, 0xBF, 0x20},
	{ADV7612_I2C_EDID, 0xC0, 0x20},
	{ADV7612_I2C_EDID, 0xC1, 0x20},
	{ADV7612_I2C_EDID, 0xC2, 0x20},
	{ADV7612_I2C_EDID, 0xC3, 0x20},
	{ADV7612_I2C_EDID, 0xC4, 0x20},
	{ADV7612_I2C_EDID, 0xC5, 0x20},
	{ADV7612_I2C_EDID, 0xC6, 0x00},
	{ADV7612_I2C_EDID, 0xC7, 0x00},
	{ADV7612_I2C_EDID, 0xC8, 0x00},
	{ADV7612_I2C_EDID, 0xC9, 0xFF},
	{ADV7612_I2C_EDID, 0xCA, 0x00},
	{ADV7612_I2C_EDID, 0xCB, 0x0A},
	{ADV7612_I2C_EDID, 0xCC, 0x20},
	{ADV7612_I2C_EDID, 0xCD, 0x20},
	{ADV7612_I2C_EDID, 0xCE, 0x20},
	{ADV7612_I2C_EDID, 0xCF, 0x20},
	{ADV7612_I2C_EDID, 0xD0, 0x20},
	{ADV7612_I2C_EDID, 0xD1, 0x20},
	{ADV7612_I2C_EDID, 0xD2, 0x20},
	{ADV7612_I2C_EDID, 0xD3, 0x20},
	{ADV7612_I2C_EDID, 0xD4, 0x20},
	{ADV7612_I2C_EDID, 0xD5, 0x20},
	{ADV7612_I2C_EDID, 0xD6, 0x20},
	{ADV7612_I2C_EDID, 0xD7, 0x20},
	{ADV7612_I2C_EDID, 0xD8, 0x00},
	{ADV7612_I2C_EDID, 0xD9, 0x00},
	{ADV7612_I2C_EDID, 0xDA, 0x00},
	{ADV7612_I2C_EDID, 0xDB, 0xFF},
	{ADV7612_I2C_EDID, 0xDC, 0x00},
	{ADV7612_I2C_EDID, 0xDD, 0x0A},
	{ADV7612_I2C_EDID, 0xDE, 0x20},
	{ADV7612_I2C_EDID, 0xDF, 0x20},
	{ADV7612_I2C_EDID, 0xE0, 0x20},
	{ADV7612_I2C_EDID, 0xE1, 0x20},
	{ADV7612_I2C_EDID, 0xE2, 0x20},
	{ADV7612_I2C_EDID, 0xE3, 0x20},
	{ADV7612_I2C_EDID, 0xE4, 0x20},
	{ADV7612_I2C_EDID, 0xE5, 0x20},
	{ADV7612_I2C_EDID, 0xE6, 0x20},
	{ADV7612_I2C_EDID, 0xE7, 0x20},
	{ADV7612_I2C_EDID, 0xE8, 0x20},
	{ADV7612_I2C_EDID, 0xE9, 0x20},
	{ADV7612_I2C_EDID, 0xEA, 0x00},
	{ADV7612_I2C_EDID, 0xEB, 0x00},
	{ADV7612_I2C_EDID, 0xEC, 0x00},
	{ADV7612_I2C_EDID, 0xED, 0x00},
	{ADV7612_I2C_EDID, 0xEE, 0x00},
	{ADV7612_I2C_EDID, 0xEF, 0x00},
	{ADV7612_I2C_EDID, 0xF0, 0x00},
	{ADV7612_I2C_EDID, 0xF1, 0x00},
	{ADV7612_I2C_EDID, 0xF2, 0x00},
	{ADV7612_I2C_EDID, 0xF3, 0x00},
	{ADV7612_I2C_EDID, 0xF4, 0x00},
	{ADV7612_I2C_EDID, 0xF5, 0x00},
	{ADV7612_I2C_EDID, 0xF6, 0x00},
	{ADV7612_I2C_EDID, 0xF7, 0x00},
	{ADV7612_I2C_EDID, 0xF8, 0x00},
	{ADV7612_I2C_EDID, 0xF9, 0x00},
	{ADV7612_I2C_EDID, 0xFA, 0x00},
	{ADV7612_I2C_EDID, 0xFB, 0x00},
	{ADV7612_I2C_EDID, 0xFC, 0x00},
	{ADV7612_I2C_EDID, 0xFD, 0x00},
	{ADV7612_I2C_EDID, 0xFE, 0x00},
	{ADV7612_I2C_EDID, 0xFF, 0xD8},	/* Set the Most Significant Bit */
					/* of the SPA location to 0 */
	{ADV7612_I2C_KSV, 0x52, 0x20},	/* Set the SPA for port B */
	{ADV7612_I2C_KSV, 0x53, 0x00},	/* Set the SPA for port B. */
	{ADV7612_I2C_KSV, 0x70, 0x9E},	/* Set the Least Significant Byte */
					/* of the SPA location */
	{ADV7612_I2C_KSV, 0x74, 0x03},	/* Enable the Internal EDID for ports */
	{ADV7612_I2C_EOR, 0xFF, 0xFF}	/* End of register table */
};

/* Register parameters for 480p */
static const struct adv7612_reg_value adv7612_parms_480P[] = {
	/* FIX ME */

	{ADV7612_I2C_EOR, 0xFF, 0xFF}		/* End of register table */
};

/* Register parameters for 720p */
static const struct adv7612_reg_value adv7612_parms_720P60[] = {
	/* FIX ME */

	{ADV7612_I2C_EOR, 0xFF, 0xFF}		/* End of register table */
};

/* Register parameters for 1080I60 */
static const struct adv7612_reg_value adv7612_parms_1080I60[] = {
	/* FIX ME */

	{ADV7612_I2C_EOR, 0xFF, 0xFF}		/* End of register table */
};


struct adv7612_color_format {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};

/* supported color format list */
static const struct adv7612_color_format adv7612_cfmts[] = {
	{
		.code		= V4L2_MBUS_FMT_RGB888_1X24,
		.colorspace	= V4L2_COLORSPACE_SRGB,
	},
};

struct adv7612_state {
	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_subdev	sd;
	struct mutex		mutex; /* mutual excl. when accessing chip */
	bool			autodetect;
	const struct adv7612_color_format	*cfmt;
	u32			width;
	u32			height;
	enum v4l2_field		scanmode;
};



/*****************************************************************************/
/*  Private functions                                                        */
/*****************************************************************************/

#define to_adv7612_sd(_ctrl) (&container_of(_ctrl->handler,		\
					    struct adv7612_state,	\
					    ctrl_hdl)->sd)

static inline struct adv7612_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct adv7612_state, sd);
}

/*
 * adv7612_write_registers() - Write adv7612 device registers
 * @client: pointer to i2c_client structure
 * @regs: pointer to adv7612_reg_value structure
 *
 * Write the specified adv7612 register's values.
 */
static int adv7612_write_registers(struct i2c_client *client,
					const struct adv7612_reg_value *regs)
{
	struct i2c_msg msg;
	u8 data_buf[2];
	int ret = -EINVAL;

	if (!client->adapter)
		return -ENODEV;

	msg.flags = 0;
	msg.len = 2;
	msg.buf = &data_buf[0];

	while (ADV7612_I2C_EOR != regs->addr) {

		msg.addr = regs->addr;
		data_buf[0] = regs->reg;
		data_buf[1] = regs->value;

		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0)
			break;

		regs++;
	}

	return (ret < 0) ? ret : 0;
}

/*
 * adv7612_write_register() - Write adv7612 device register
 * @client: pointer to i2c_client structure
 * @addr: i2c slave address of adv7612 device
 * @reg: adv7612 device register address
 * @value: the value to be written
 *
 * Write the specified adv7612 register's value.
 */
static int adv7612_write_register(struct i2c_client *client,
		u8 addr, u8 reg, u8 value)
{
	struct adv7612_reg_value regs[2];
	int ret;

	regs[0].addr = addr;
	regs[0].reg = reg;
	regs[0].value = value;
	regs[1].addr = ADV7612_I2C_EOR;
	regs[1].reg = 0xFF;
	regs[1].value = 0xFF;

	ret = adv7612_write_registers(client, regs);

	return ret;
}

/*
 * adv7612_read_register() - Read adv7612 device register
 * @client: pointer to i2c_client structure
 * @addr: i2c slave address of adv7612 device
 * @reg: adv7612 device register address
 * @value: pointer to the value
 *
 * Read the specified adv7612 register's value.
 */
static int adv7612_read_register(struct i2c_client *client,
		u8 addr, u8 reg, u8 *value)
{
	struct i2c_msg msg[2];
	u8 reg_buf, data_buf;
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg_buf;
	msg[1].addr = addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &data_buf;

	reg_buf = reg;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0)
		return ret;

	*value = data_buf;

	return (ret < 0) ? ret : 0;
}

/*
 * adv7612_get_vid_info() - Get video information
 * @sd: pointer to standard V4L2 sub-device structure
 * @progressive: progressive or interlace
 * @width: line width
 * @height: lines per frame
 *
 * Get video information.
 */
static int adv7612_get_vid_info(struct v4l2_subdev *sd, u8 *progressive,
				u32 *width, u32 *height, u8 *signal)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 hdmi_int;
	u8 msb;
	u8 lsb;
	int ret;

	if (signal)
		*signal = 0;

	/* decide line width */
	ret = adv7612_read_register(client, ADV7612_I2C_HDMI,
			ADV7612_HDMI_STATUS1_REG, &msb);
	if (ret < 0)
		return ret;

	if (!(msb & ADV7612_HDMI_VF_LOCKED_FLG) ||
	    !(msb & ADV7612_HDMI_DERF_LOCKED_FLG))
		return -EIO;

	if (signal)
		*signal = 1;

	/* decide interlaced or progressive */
	ret = adv7612_read_register(client, ADV7612_I2C_HDMI,
			ADV7612_HDMI_STATUS2_REG, &hdmi_int);
	if (ret < 0)
		return ret;

	*progressive =  1;
	if ((hdmi_int & ADV7612_HDMI_IP_FLAG) != 0)
		*progressive =  0;

	ret = adv7612_read_register(client, ADV7612_I2C_HDMI,
			ADV7612_HDMI_LWIDTH_REG, &lsb);
	if (ret < 0)
		return ret;

	*width = (u32)(ADV7612_HDMI_LWIDTH_MSBS_MASK & msb);
	*width = (lsb | (*width << 8));

	/* decide lines per frame */
	ret = adv7612_read_register(client, ADV7612_I2C_HDMI,
			ADV7612_HDMI_F0HEIGHT_MSBS_REG, &msb);
	if (ret < 0)
		return ret;

	ret = adv7612_read_register(client, ADV7612_I2C_HDMI,
			ADV7612_HDMI_F0HEIGHT_LSBS_REG, &lsb);
	if (ret < 0)
		return ret;

	*height = (u32)(ADV7612_HDMI_F0HEIGHT_MSBS_MASK & msb);
	*height = (lsb | (*height << 8));
	if (!(*progressive))
		*height = *height * 2;

	if (*width == 0 || *height == 0)
		return -EIO;

	return 0;
}


/*****************************************************************************/
/*  V4L2 decoder i/f handler for v4l2_subdev_core_ops                        */
/*****************************************************************************/


/*
 * adv7612_g_chip_ident() - V4L2 decoder i/f handler for g_chip_ident
 * @sd: ptr to v4l2_subdev struct
 * @chip: ptr to v4l2_dbg_chip_ident struct
 *
 * Obtains the chip's identification number.
 * Currently does not return the hardware revision code.
 */
static int adv7612_g_chip_ident(struct v4l2_subdev *sd,
	struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_ADV7612, 0);
}

/*
 * adv7612_querystd() - V4L2 decoder i/f handler for querystd
 * @sd: ptr to v4l2_subdev struct
 * @std: standard input video id
 *
 * Obtains the video standard input id
 */
static int adv7612_querystd(struct v4l2_subdev *sd, v4l2_std_id *std)
{
	struct adv7612_state *state = to_state(sd);
	int err = mutex_lock_interruptible(&state->mutex);
	if (err)
		return err;

	*std = V4L2_STD_ATSC;

	mutex_unlock(&state->mutex);

	return err;
}

/*
 * adv7612_g_input_status() - V4L2 decoder i/f handler for g_input_status
 * @sd: ptr to v4l2_subdev struct
 * @status: video input status flag
 *
 * Obtains the video input status flags.
 */
static int adv7612_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7612_state *state = to_state(sd);
	u8 status1 = 0;
	int ret = mutex_lock_interruptible(&state->mutex);

	if (ret)
		return ret;

	ret = adv7612_read_register(client, ADV7612_I2C_HDMI,
			ADV7612_HDMI_STATUS1_REG, &status1);
	if (ret < 0)
		goto out;

	if (!(status1 & ADV7612_HDMI_VF_LOCKED_FLG))
		*status = V4L2_IN_ST_NO_SIGNAL;
	else if (!(status1 & ADV7612_HDMI_DERF_LOCKED_FLG))
		*status = V4L2_IN_ST_NO_SIGNAL;
	else
		*status = 0;

	ret = 0;
out:
	mutex_unlock(&state->mutex);
	return ret;
}

/*
 * adv7612_s_stream() - V4L2 decoder i/f handler for s_stream
 * @sd: pointer to standard V4L2 sub-device structure
 * @enable: streaming enable or disable
 *
 * Sets streaming to enable or disable, if possible.
 * Currently no implementation.
 */
static int adv7612_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7612_state *state = to_state(sd);

	dev_dbg(&client->dev, "format %d\n", state->cfmt->code);

	return 0;
}

/*
 * adv7612_cropcap() - V4L2 decoder i/f handler for cropcap
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 cropcap structure
 *
 * Gets cropping limits, default cropping rectangle and pixel aspect.
 */
static int adv7612_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	u8 progressive;
	u32 width;
	u32 height;
	int ret;

	/* cropping limits */
	a->bounds.left			= 0;
	a->bounds.top			= 0;

	/* Get video information */
	ret = adv7612_get_vid_info(sd, &progressive, &width, &height, NULL);
	if (ret < 0) {
		a->bounds.width			= ADV7612_MAX_WIDTH;
		a->bounds.height		= ADV7612_MAX_HEIGHT;
	} else {
		a->bounds.width			= width;
		a->bounds.height		= height;
	}

	/* default cropping rectangle */
	a->defrect				= a->bounds;

	/* does not support scaling */
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

/*
 * adv7612_cropcap() - V4L2 decoder i/f handler for g_crop
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 cropcap structure
 *
 * Gets current cropping rectangle.
 */
static int adv7612_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	u8 progressive;
	u32 width;
	u32 height;
	int ret;

	a->c.left	= 0;
	a->c.top	= 0;

	/* Get video information */
	ret = adv7612_get_vid_info(sd, &progressive, &width, &height, NULL);
	if (ret < 0) {
		a->c.width	= ADV7612_MAX_WIDTH;
		a->c.height	= ADV7612_MAX_HEIGHT;
	} else {
		a->c.width	= width;
		a->c.height	= height;
	}

	a->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

/*
 * adv7612_mbus_fmt() - V4L2 decoder i/f handler for g_mbus_fmt
 * @sd: pointer to standard V4L2 sub-device structure
 * @mf: pointer to mediabus format structure
 *
 * Negotiate the image capture size and mediabus format.
 * Get the data format.
 */
static int adv7612_g_mbus_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	struct adv7612_state *state = to_state(sd);

	if (!state->cfmt)
		state->cfmt = adv7612_cfmts;

	mf->width = state->width;
	mf->height = state->height;
	mf->code = state->cfmt->code;
	mf->field = state->scanmode;
	mf->colorspace = state->cfmt->colorspace;

	return 0;
}

/*
 * adv7612_try_mbus_fmt() - V4L2 decoder i/f handler for try_mbus_fmt
 * @sd: pointer to standard V4L2 sub-device structure
 * @mf: pointer to mediabus format structure
 *
 * Negotiate the image capture size and mediabus format.
 * Try a format.
 */
static int adv7612_try_mbus_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	struct adv7612_state *state = to_state(sd);
	int i;
	int ret;
	u8 progressive;
	u32 width;
	u32 height;

	for (i = 0; i < ARRAY_SIZE(adv7612_cfmts); i++) {
		if (mf->code == adv7612_cfmts[i].code)
			break;
	}

	if (i == ARRAY_SIZE(adv7612_cfmts)) {
		/* Unsupported format requested. Propose either */
		if (state->cfmt) {
			/* the current one or */
			mf->colorspace = state->cfmt->colorspace;
			mf->code = state->cfmt->code;
		} else {
			/* the default one */
			mf->colorspace = adv7612_cfmts[0].colorspace;
			mf->code = adv7612_cfmts[0].code;
		}
	} else {
		/* Also return the colorspace */
		mf->colorspace	= adv7612_cfmts[i].colorspace;
	}

	/* Get video information */
	ret = adv7612_get_vid_info(sd, &progressive, &width, &height, NULL);
	if (ret < 0) {
		width		= ADV7612_MAX_WIDTH;
		height		= ADV7612_MAX_HEIGHT;
		progressive	= 1;
	}

	mf->width = width;
	mf->height = height;
	if (progressive) {
		mf->field = V4L2_FIELD_NONE;
	} else {
		if (mf->field == V4L2_FIELD_NONE)
			mf->field = V4L2_FIELD_INTERLACED;
	}

	return 0;
}

/*
 * adv7612_s_mbus_fmt() - V4L2 decoder i/f handler for s_mbus_fmt
 * @sd: pointer to standard V4L2 sub-device structure
 * @mf: pointer to mediabus format structure
 *
 * Negotiate the image capture size and mediabus format.
 * Set the data format.
 */
static int adv7612_s_mbus_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7612_state *state = to_state(sd);
	int i;
	int ret;
	u8 progressive;
	u8 signal;
	u32 width;
	u32 height;

	for (i = 0; i < ARRAY_SIZE(adv7612_cfmts); i++) {
		if (mf->code == adv7612_cfmts[i].code) {
			state->cfmt = adv7612_cfmts + i;
			break;
		}
	}
	if (i >= ARRAY_SIZE(adv7612_cfmts))
		return -EINVAL;	/* no match format */

	/* Get video information */
	ret = adv7612_get_vid_info(sd, &progressive, &width, &height, &signal);
	if (ret < 0) {
		width		= ADV7612_MAX_WIDTH;
		height		= ADV7612_MAX_HEIGHT;
		progressive	= 1;
	}

	if (signal)
		dev_info(&client->dev,
			"Detected the HDMI video input signal (%dx%d%c)\n"
				, width, height, (progressive) ? 'p' : 'i');
	else
		dev_info(&client->dev,
			"Not detect any video input signal\n");

	state->width = width;
	state->height = height;
	state->scanmode =
		(progressive) ? V4L2_FIELD_NONE : V4L2_FIELD_INTERLACED;

	mf->width = state->width;
	mf->height = state->height;
	mf->code = state->cfmt->code;
	mf->field = state->scanmode;
	mf->colorspace = state->cfmt->colorspace;

	return 0;
}

/*
 * adv7612_enum_mbus_fmt() - V4L2 decoder i/f handler for enum_mbus_fmt
 * @sd: pointer to standard V4L2 sub-device structure
 * @index: format index
 * @code: pointer to mediabus format
 *
 * Enumerate supported mediabus formats.
 */
static int adv7612_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	/* Check requested format index is within range */
	if (index >= ARRAY_SIZE(adv7612_cfmts))
		return -EINVAL;

	*code = adv7612_cfmts[index].code;

	return 0;
}

/*
 * adv7612_g_mbus_config() - V4L2 decoder i/f handler for g_mbus_config
 * @sd: pointer to standard V4L2 sub-device structure
 * @cfg: pointer to V4L2 mbus_config structure
 *
 * Get mbus configuration.
 */
static int adv7612_g_mbus_config(struct v4l2_subdev *sd,
					struct v4l2_mbus_config *cfg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);

	cfg->flags = V4L2_MBUS_PCLK_SAMPLE_RISING | V4L2_MBUS_MASTER |
		V4L2_MBUS_VSYNC_ACTIVE_LOW | V4L2_MBUS_HSYNC_ACTIVE_LOW |
		V4L2_MBUS_DATA_ACTIVE_HIGH;
	cfg->type = V4L2_MBUS_BT656;
	cfg->flags = soc_camera_apply_board_flags(ssdd, cfg);

	return 0;
}


/*****************************************************************************/
/*  V4L2 decoder i/f handler for v4l2_ctrl_ops                               */
/*****************************************************************************/


/*
 * adv7612_s_ctrl() - V4L2 decoder i/f handler for s_ctrl
 * @ctrl: pointer to standard V4L2 control structure
 *
 * Set a control in ADV7612 decoder device.
 */
static int adv7612_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_adv7612_sd(ctrl);
	struct adv7612_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = mutex_lock_interruptible(&state->mutex);
	u8 val;

	if (ret)
		return ret;

	/* Enable video adjustment first */
	ret = adv7612_read_register(client, ADV7612_I2C_CP,
			ADV7612_CP_VID_ADJ_REG, &val);
	if (ret < 0)
		return ret;
	val |= ADV7612_CP_VID_ADJ_ENABLE;
	ret = adv7612_write_register(client, ADV7612_I2C_CP,
			ADV7612_CP_VID_ADJ_REG, val);
	if (ret < 0)
		return ret;

	val = ctrl->val;
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		if ((ctrl->val < ADV7612_CP_BRI_MIN) ||
					(ctrl->val > ADV7612_CP_BRI_MAX))
			ret = -ERANGE;
		else
			ret = adv7612_write_register(client, ADV7612_I2C_CP,
					ADV7612_CP_BRI_REG, val);
		break;
	case V4L2_CID_HUE:
		if ((ctrl->val < ADV7612_CP_HUE_MIN) ||
					(ctrl->val > ADV7612_CP_HUE_MAX))
			ret = -ERANGE;
		else
			ret = adv7612_write_register(client, ADV7612_I2C_CP,
					ADV7612_CP_HUE_REG, val);
		break;
	case V4L2_CID_CONTRAST:
		if ((ctrl->val < ADV7612_CP_CON_MIN) ||
					(ctrl->val > ADV7612_CP_CON_MAX))
			ret = -ERANGE;
		else
			ret = adv7612_write_register(client, ADV7612_I2C_CP,
					ADV7612_CP_CON_REG, val);
		break;
	case V4L2_CID_SATURATION:
		if ((ctrl->val < ADV7612_CP_SAT_MIN) ||
					(ctrl->val > ADV7612_CP_SAT_MAX))
			ret = -ERANGE;
		else
			ret = adv7612_write_register(client, ADV7612_I2C_CP,
					ADV7612_CP_SAT_REG, val);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&state->mutex);
	return ret;
}


static const struct v4l2_subdev_core_ops adv7612_core_ops = {
	.g_chip_ident	= adv7612_g_chip_ident,
	.queryctrl = v4l2_subdev_queryctrl,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.querymenu = v4l2_subdev_querymenu,
};

static const struct v4l2_subdev_video_ops adv7612_video_ops = {
	.querystd	= adv7612_querystd,
	.g_input_status = adv7612_g_input_status,
	.s_stream	= adv7612_s_stream,
	.cropcap	= adv7612_cropcap,
	.g_crop		= adv7612_g_crop,
	.enum_mbus_fmt	= adv7612_enum_mbus_fmt,
	.g_mbus_fmt	= adv7612_g_mbus_fmt,
	.try_mbus_fmt	= adv7612_try_mbus_fmt,
	.s_mbus_fmt	= adv7612_s_mbus_fmt,
	.g_mbus_config	= adv7612_g_mbus_config,
};

static const struct v4l2_subdev_ops adv7612_ops = {
	.core = &adv7612_core_ops,
	.video = &adv7612_video_ops,
};

static const struct v4l2_ctrl_ops adv7612_ctrl_ops = {
	.s_ctrl = adv7612_s_ctrl,
};

/*
 * adv7612_init_controls() - Init controls
 * @state: pointer to private state structure
 *
 * Init ADV7612 supported control handler.
 */
static int adv7612_init_controls(struct adv7612_state *state)
{
	v4l2_ctrl_handler_init(&state->ctrl_hdl, 4);

	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7612_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, (s32)0x80000000,
			  (s32)0x7fffffff, 1, ADV7612_CP_BRI_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7612_ctrl_ops,
			  V4L2_CID_CONTRAST, (s32)0x80000000,
			  (s32)0x7fffffff, 1, ADV7612_CP_CON_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7612_ctrl_ops,
			  V4L2_CID_SATURATION, (s32)0x80000000,
			  (s32)0x7fffffff, 1, ADV7612_CP_SAT_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv7612_ctrl_ops,
			  V4L2_CID_HUE, (s32)0x80000000,
			  (s32)0x7fffffff, 1, ADV7612_CP_HUE_DEF);
	state->sd.ctrl_handler = &state->ctrl_hdl;
	if (state->ctrl_hdl.error) {
		int err = state->ctrl_hdl.error;

		v4l2_ctrl_handler_free(&state->ctrl_hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&state->ctrl_hdl);

	return 0;
}


/*****************************************************************************/
/*  i2c driver interface handlers                                            */
/*****************************************************************************/


/*
 * adv7612_probe - Probe a ADV7612 device
 * @client: pointer to i2c_client structure
 * @id: pointer to i2c_device_id structure
 *
 * Initialize the ADV7612 device
 */
static int adv7612_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct adv7612_state *state;
	struct v4l2_subdev *sd;
	int ret;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	state = kzalloc(sizeof(struct adv7612_state), GFP_KERNEL);
	if (state == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	mutex_init(&state->mutex);
	state->autodetect = true;
	sd = &state->sd;
	state->width		= ADV7612_MAX_WIDTH;
	state->height		= ADV7612_MAX_HEIGHT;
	state->scanmode		= V4L2_FIELD_NONE;

	v4l2_i2c_subdev_init(sd, client, &adv7612_ops);

	/* Initializes AVD7612 to its default values */
	ret = adv7612_write_registers(client, adv7612_init_defaults);
	if (ret < 0)
		goto err_unreg_subdev;

	ret = adv7612_init_controls(state);
	if (ret)
		goto err_unreg_subdev;

	return 0;

err_unreg_subdev:
	mutex_destroy(&state->mutex);
	v4l2_device_unregister_subdev(sd);
	kfree(state);
err:
	dev_err(&client->dev, ": Failed to probe: %d\n", ret);
	return ret;
}

/*
 * adv7612_remove - Remove ADV7612 device support
 * @client: pointer to i2c_client structure
 *
 * Reset the ADV7612 device
 */
static int adv7612_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adv7612_state *state = to_state(sd);

	v4l2_ctrl_handler_free(&state->ctrl_hdl);
	mutex_destroy(&state->mutex);
	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id adv7612_id[] = {
	{DRIVER_NAME, 0},
	{},
};

#ifdef CONFIG_PM
/*
 * adv7612_suspend - Suspend ADV7612 device
 * @client: pointer to i2c_client structure
 * @state: power management state
 *
 * Power down the ADV7612 device
 */
static int adv7612_suspend(struct i2c_client *client, pm_message_t state)
{
	int ret;

	ret = adv7612_write_register(client, ADV7612_I2C_IO,
				ADV7612_IO_PWR_MAN_REG, ADV7612_IO_PWR_OFF);

	return ret;
}

/*
 * adv7612_resume - Resume ADV7612 device
 * @client: pointer to i2c_client structure
 *
 * Power on and initialize the ADV7612 device
 */
static int adv7612_resume(struct i2c_client *client)
{
	int ret;

	ret = adv7612_write_register(client, ADV7612_I2C_IO,
				ADV7612_IO_PWR_MAN_REG, ADV7612_IO_PWR_ON);
	if (ret < 0)
		return ret;

	/* Initializes AVD7612 to its default values */
	ret = adv7612_write_registers(client, adv7612_init_defaults);

	return ret;
}
#endif

MODULE_DEVICE_TABLE(i2c, adv7612_id);

static struct i2c_driver adv7612_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME,
	},
	.probe		= adv7612_probe,
	.remove		= adv7612_remove,
#ifdef CONFIG_PM
	.suspend = adv7612_suspend,
	.resume = adv7612_resume,
#endif
	.id_table	= adv7612_id,
};

module_i2c_driver(adv7612_driver);

MODULE_DESCRIPTION("HDMI Receiver ADV7612 video decoder driver");
MODULE_LICENSE("GPL v2");
