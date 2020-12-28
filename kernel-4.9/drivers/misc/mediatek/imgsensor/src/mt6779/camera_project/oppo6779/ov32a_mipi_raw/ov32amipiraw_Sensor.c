/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

//#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"
#include "ov32amipiraw_Sensor.h"

#undef IMGSENSOR_I2C_1000K
#include "imgsensor_i2c.h"

#ifndef VENDOR_EDIT
#define VENDOR_EDIT
#endif

#ifdef VENDOR_EDIT
/*Caohua.Lin@Camera.Driver  add for 18011  board 20180723*/
#define DEVICE_VERSION_OV32A    "ov32a"
#define IMGSENSOR_MODULE_ID_SUNNY       0x01
extern void register_imgsensor_deviceinfo(char *name, char *version, u8 module_id);
//static uint8_t deviceInfo_register_value;
/* Feiping.Li@Camera.Drv, 20190624, add for speed up sensor init*/
#endif

#define PFX "OV32A_camera_sensor"
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
static kal_uint32 streaming_control(kal_bool enable);

#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __func__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {

	/*record sensor id defined in Kd_imgsensor.h*/
	.sensor_id = OV32A_SENSOR_ID,

	.checksum_value = 0xb1893b4f, /*checksum value for Camera Auto Test*/
	.pre = {
		.pclk = 90000000,
		.linelength = 840,
		.framelength = 3570,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 300,
		.mipi_pixel_rate = 393600000,
	},
	.cap = {
		.pclk = 90000000,
		.linelength = 1200,
		.framelength = 5000,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 6528,
		.grabwindow_height = 4896,
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 150,
		.mipi_pixel_rate = 508800000,
	},
	.normal_video = {
		.pclk = 90000000,
		.linelength = 420,
		.framelength = 7140,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 300,
		.mipi_pixel_rate = 787200000,
	},
	.hs_video = {
		.pclk = 90000000,
		.linelength = 420,
		.framelength = 7140,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 300,
		.mipi_pixel_rate = 787200000,
	},
	.slim_video = {
		.pclk = 90000000,
		.linelength = 420,
		.framelength = 7140,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 300,
		.mipi_pixel_rate = 787200000,
	},

	.custom1 = {
		.pclk = 90000000,
		.linelength = 420,
		.framelength = 7140,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 120,
		.max_framerate = 300,
		.mipi_pixel_rate = 787200000,
	},


	.margin = 14,			/*sensor framelength & shutter margin*/
	.min_shutter = 8,		/*min shutter*/

	/*max framelength by sensor register's limitation*/
	.max_frame_length = 0xffff,

	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2, /*isp gain delay frame for AE cycle*/
	.frame_time_delay_frame = 2,
	.ihdr_support = 0,	  /*1, support; 0,not support*/
	.ihdr_le_firstline = 0,  /*1,le first ; 0, se first*/

	/*support sensor mode num ,don't support Slow motion*/
	.sensor_mode_num = 6,
	.cap_delay_frame = 3,		/*enter capture delay frame num*/
	.pre_delay_frame = 3,		/*enter preview delay frame num*/
	.video_delay_frame = 3,		/*enter video delay frame num*/
	.hs_video_delay_frame = 3, /*enter high speed video  delay frame num*/
	.slim_video_delay_frame = 3,/*enter slim video delay frame num*/
	.custom1_delay_frame = 3,
	.isp_driving_current = ISP_DRIVING_2MA, /*mclk driving current*/

	/*Sensor_interface_type*/
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,

	/*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
	.mipi_sensor_type = MIPI_OPHY_NCSI2,

	/*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
	.mipi_settle_delay_mode = 1, //0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL

	/*sensor output first pixel color*/
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,

	.mclk = 24,/*mclk value, suggest 24 or 26 for 24Mhz or 26Mhz*/
	.mipi_lane_num = SENSOR_MIPI_4_LANE,/*mipi lane num*/

	/*record sensor support all write id addr, only supprt 4must end with 0xff*/
	.i2c_addr_table = {0x20, 0x21, 0xff},
	.i2c_speed = 400,
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,		/*mirrorflip information*/
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x04C0,			/*current shutter*/
	.gain = 0x200,				/*current gain*/
	.dummy_pixel = 0,			/*current dummypixel*/
	.dummy_line = 0,			/*current dummyline*/

	/*full size current fps : 24fps for PIP, 30fps for Normal or ZSD*/
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,

	/*current scenario id*/
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_mode = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x6c, /*record current sensor's i2c write id*/
};


/* Sensor output window information*/
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
	{6528, 4896,    0,    0, 6528, 4896, 3264, 2448,  0,  0,3264, 2448, 0, 0, 3264, 2448},     /* preview */
	{6528, 4896,    0,    0, 6528, 4896, 6528, 4896,  0,  0,6528, 4896, 0, 0, 6528, 4896},     /* capture */
	{6528, 4896,    0,    0, 6528, 4896, 3264, 2448,  0,  0,3264, 2448, 0, 0, 3264, 2448},     /* preview */
	{6528, 4896,    0,    0, 6528, 4896, 3264, 2448,  0,  0,3264, 2448, 0, 0, 3264, 2448},     /* preview */
	{6528, 4896,    0,    0, 6528, 4896, 3264, 2448,  0,  0,3264, 2448, 0, 0, 3264, 2448},     /* preview */
	{6528, 4896,    0,    0, 6528, 4896, 3264, 2448,  0,  0,3264, 2448, 0, 0, 3264, 2448},     /* preview */
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF),
			(char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
	write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);
	write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
}	/*  set_dummy  */


static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ?
			frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length -
		imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
	imgsensor.frame_length = imgsensor_info.max_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length -
		imgsensor.min_frame_length;
	}
	if (min_framelength_en)
	imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);

	set_dummy();
}

static void set_shutter_frame_length(
				kal_uint16 shutter, kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	spin_lock(&imgsensor_drv_lock);
	/*Change frame time*/
	if (frame_length > 1) {
		dummy_line = frame_length - imgsensor.frame_length;
	}
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin) {
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	}
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;

	if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) {
		shutter = (imgsensor_info.max_frame_length - imgsensor_info.margin);
	}

	//imgsensor.frame_length = imgsensor.frame_length - imgsensor.frame_length % 2;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
			write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x380f,imgsensor.frame_length & 0xFF);
		}
	} else {
			imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
			write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x380f,imgsensor.frame_length & 0xFF);
		}

	/* Update Shutter*/
	write_cmos_sensor(0x3501, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x3502, (shutter)  & 0xFF);

	LOG_INF("Add for N3D! shutterlzl =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}



/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK
	 * to get exposure larger than frame exposure
	 */
	/* AE doesn't update sensor gain at capture mode,
	 * thus extra exposure lines must be updated here.
	 */

	/* OV Recommend Solution*/
	/* if shutter bigger than frame_length, should extend frame length first*/

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin) {
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	} else {
		imgsensor.frame_length = imgsensor.min_frame_length;
	}
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;

	if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) {
		shutter = (imgsensor_info.max_frame_length - imgsensor_info.margin);
	}
	//imgsensor.frame_length = imgsensor.frame_length - imgsensor.frame_length % 2;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
			write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x380f,imgsensor.frame_length & 0xFF);
		}
	} else {
			imgsensor.frame_length = (imgsensor.frame_length  >> 1) << 1;
			write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x380f,imgsensor.frame_length & 0xFF);
		}

	/* Update Shutter*/
	write_cmos_sensor(0x3501, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x3502, (shutter)  & 0xFF);

	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/


static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 iReg = 0x0000;

	//platform 1xgain = 64, sensor driver 1*gain = 0x100
	iReg = gain*256/BASEGAIN;

	if(iReg < 0x100)	//sensor 1xGain
	{
		iReg = 0X100;
	}
	if(iReg > 0xf80)	//sensor 15.5xGain
	{
		iReg = 0Xf80;
	}
	return iReg;		/* sensorGlobalGain */
}

static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;
	unsigned long flags;

	reg_gain = gain2reg(gain);
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.gain = reg_gain;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	write_cmos_sensor(0x03508, (reg_gain >> 8));
	write_cmos_sensor(0x03509, (reg_gain&0xff));

	return gain;
}


static void ihdr_write_shutter_gain(
			kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
}

#if 0
/*Feiping@Camera.Drv, 20190603, add for set correct mirror/flip */
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

/********************************************************
*
*   0x3820[2] ISP Vertical flip
*   0x3820[1] Sensor Vertical flip
*
*   0x3821[2] ISP Horizontal mirror
*   0x3821[1] Sensor Horizontal mirror
*
*   ISP and Sensor flip or mirror register bit should be the same!!
*
********************************************************/

	switch (image_mirror) {
	case IMAGE_NORMAL:
		break;

	case IMAGE_H_MIRROR:
		break;

	case IMAGE_V_MIRROR:
		break;

	case IMAGE_HV_MIRROR:
		break;

	default:
		LOG_INF("Error image_mirror setting\n");
	}
}
#endif


/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/

#if 0
#define I2C_BUFFER_LEN 225	/* trans# max is 255, each 3 bytes */
extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length, u16 timing);

static kal_uint16 table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;
	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)addr;
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)data;
			IDX += 2;
			addr_last = addr;
		}

		/* Write when remain buffer size is less than 4 bytes or reach end of data */
		if ((I2C_BUFFER_LEN - tosend) < 2 || IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id,
								2, imgsensor_info.i2c_speed);
			tosend = 0;
		}
	}
	return 0;
}

static kal_uint16 addr_data_pair_global[] = {

};
#endif

/*************************************************************************
 * FUNCTION
 *	sensor_init
 *
 * DESCRIPTION
 *	Sensor init
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void sensor_init(void)
{
	LOG_INF("v2 E\n");

	#if 0
	table_write_cmos_sensor(addr_data_pair_global,
		sizeof(addr_data_pair_global) / sizeof(kal_uint16));
	#else
		write_cmos_sensor(0x0103, 0x01);
		write_cmos_sensor(0x0302, 0x31);
		write_cmos_sensor(0x0323, 0x05);
		write_cmos_sensor(0x0324, 0x01);
		write_cmos_sensor(0x0325, 0x68);
		write_cmos_sensor(0x0326, 0xcb);
		write_cmos_sensor(0x0327, 0x05);
		write_cmos_sensor(0x0343, 0x05);
		write_cmos_sensor(0x0346, 0xcf);
		write_cmos_sensor(0x300e, 0x22);
		write_cmos_sensor(0x3016, 0x96);
		write_cmos_sensor(0x3017, 0x78);
		write_cmos_sensor(0x3018, 0x70);
		write_cmos_sensor(0x3019, 0xd2);
		write_cmos_sensor(0x301b, 0x16);
		write_cmos_sensor(0x3022, 0xd0);
		write_cmos_sensor(0x3028, 0xc3);
		write_cmos_sensor(0x3102, 0x00);
		write_cmos_sensor(0x3103, 0x0a);
		write_cmos_sensor(0x3221, 0x0a);
		write_cmos_sensor(0x3400, 0x04);
		write_cmos_sensor(0x3408, 0x06);
		write_cmos_sensor(0x3508, 0x08);
		write_cmos_sensor(0x3548, 0x08);
		write_cmos_sensor(0x360b, 0xfc);
		write_cmos_sensor(0x360c, 0x11);
		write_cmos_sensor(0x3619, 0x80);
		write_cmos_sensor(0x361c, 0x80);
		write_cmos_sensor(0x361e, 0x87);
		write_cmos_sensor(0x3623, 0x55);
		write_cmos_sensor(0x3626, 0x89);
		write_cmos_sensor(0x3627, 0x11);
		write_cmos_sensor(0x3628, 0x88);
		write_cmos_sensor(0x3629, 0xce);
		write_cmos_sensor(0x3634, 0x0c);
		write_cmos_sensor(0x363b, 0x09);
		write_cmos_sensor(0x363c, 0x14);
		write_cmos_sensor(0x363d, 0x24);
		write_cmos_sensor(0x363e, 0x43);
		write_cmos_sensor(0x3641, 0x0a);
		write_cmos_sensor(0x3642, 0xc8);
		write_cmos_sensor(0x3654, 0x0a);
		write_cmos_sensor(0x3655, 0xdc);
		write_cmos_sensor(0x3656, 0x0f);
		write_cmos_sensor(0x3663, 0x81);
		write_cmos_sensor(0x3664, 0x00);
		write_cmos_sensor(0x3681, 0x08);
		write_cmos_sensor(0x3683, 0x11);
		write_cmos_sensor(0x3684, 0x40);
		write_cmos_sensor(0x3685, 0x10);
		write_cmos_sensor(0x3686, 0x11);
		write_cmos_sensor(0x368b, 0x06);
		write_cmos_sensor(0x3704, 0x22);
		write_cmos_sensor(0x3706, 0x4f);
		write_cmos_sensor(0x3709, 0x8e);
		write_cmos_sensor(0x3711, 0x00);
		write_cmos_sensor(0x3724, 0x40);
		write_cmos_sensor(0x373f, 0x02);
		write_cmos_sensor(0x374f, 0x05);
		write_cmos_sensor(0x3765, 0x08);
		write_cmos_sensor(0x3767, 0x00);
		write_cmos_sensor(0x37cb, 0x11);
		write_cmos_sensor(0x37cc, 0x0f);
		write_cmos_sensor(0x37d9, 0x08);
		write_cmos_sensor(0x37dc, 0x20);
		write_cmos_sensor(0x37e3, 0x18);
		write_cmos_sensor(0x3823, 0x04);
		write_cmos_sensor(0x382a, 0x01);
		write_cmos_sensor(0x3834, 0x04);
		write_cmos_sensor(0x383d, 0x00);
		write_cmos_sensor(0x3860, 0x00);
		write_cmos_sensor(0x3861, 0x00);
		write_cmos_sensor(0x3889, 0x03);
		write_cmos_sensor(0x388b, 0x02);
		write_cmos_sensor(0x388d, 0x01);
		write_cmos_sensor(0x3d85, 0x85);
		write_cmos_sensor(0x3d8c, 0x77);
		write_cmos_sensor(0x3d8d, 0xa0);
		write_cmos_sensor(0x3d96, 0x0a);
		write_cmos_sensor(0x3f01, 0x12);
		write_cmos_sensor(0x4009, 0x02);
		write_cmos_sensor(0x4021, 0x00);
		write_cmos_sensor(0x4023, 0x00);
		write_cmos_sensor(0x4024, 0x04);
		write_cmos_sensor(0x4025, 0x00);
		write_cmos_sensor(0x4026, 0x04);
		write_cmos_sensor(0x4027, 0x00);
		write_cmos_sensor(0x40c3, 0x0a);
		write_cmos_sensor(0x4506, 0x0a);
		write_cmos_sensor(0x460a, 0x0a);
		write_cmos_sensor(0x464a, 0x0a);
		write_cmos_sensor(0x4850, 0x41);
		write_cmos_sensor(0x4a00, 0x10);
		write_cmos_sensor(0x4d01, 0x00);
		write_cmos_sensor(0x4d02, 0xb7);
		write_cmos_sensor(0x4d03, 0xca);
		write_cmos_sensor(0x4d04, 0x30);
		write_cmos_sensor(0x4d05, 0x1d);
		write_cmos_sensor(0x5004, 0x01);
		write_cmos_sensor(0x5181, 0x10);
		write_cmos_sensor(0x5182, 0x05);
		write_cmos_sensor(0x5183, 0x8f);
		write_cmos_sensor(0x5184, 0x01);
		write_cmos_sensor(0x522a, 0x44);
		write_cmos_sensor(0x522b, 0x44);
		write_cmos_sensor(0x522c, 0x14);
		write_cmos_sensor(0x522d, 0x44);
		write_cmos_sensor(0x5300, 0x7b);
		write_cmos_sensor(0x5311, 0x01);
		write_cmos_sensor(0x5312, 0x01);
		write_cmos_sensor(0x5313, 0x02);
		write_cmos_sensor(0x5314, 0x04);
		write_cmos_sensor(0x5315, 0x06);
		write_cmos_sensor(0x5316, 0x08);
		write_cmos_sensor(0x5317, 0x0a);
		write_cmos_sensor(0x5318, 0x0c);
		write_cmos_sensor(0x5319, 0x0e);
		write_cmos_sensor(0x531a, 0x10);
		write_cmos_sensor(0x531b, 0x12);
		write_cmos_sensor(0x531c, 0x14);
		write_cmos_sensor(0x531d, 0x16);
		write_cmos_sensor(0x531e, 0x18);
		write_cmos_sensor(0x5330, 0x12);
		write_cmos_sensor(0x5331, 0x15);
		write_cmos_sensor(0x5332, 0x17);
		write_cmos_sensor(0x5333, 0x19);
		write_cmos_sensor(0x5334, 0x1b);
		write_cmos_sensor(0x5335, 0x1d);
		write_cmos_sensor(0x5336, 0x1e);
		write_cmos_sensor(0x5337, 0x1f);
		write_cmos_sensor(0x5338, 0x20);
		write_cmos_sensor(0x5339, 0x20);
		write_cmos_sensor(0x533a, 0x0c);
		write_cmos_sensor(0x533b, 0x02);
		write_cmos_sensor(0x533c, 0x68);
		write_cmos_sensor(0x5d80, 0x21);
		write_cmos_sensor(0x5d82, 0x04);
		write_cmos_sensor(0x5d85, 0x19);
		write_cmos_sensor(0x5e07, 0x0a);
		write_cmos_sensor(0x600b, 0x03);
		write_cmos_sensor(0x4853, 0x04);
	#endif
}	/*	sensor_init  */

/*************************************************************************
 * FUNCTION
 *	preview_setting
 *
 * DESCRIPTION
 *	Sensor preview
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void preview_setting(void)
{
	write_cmos_sensor(0x0305, 0x64);
	write_cmos_sensor(0x0344, 0x01);
	write_cmos_sensor(0x0345, 0xe0);
	write_cmos_sensor(0x034a, 0x06);
	write_cmos_sensor(0x034b, 0x00);
	write_cmos_sensor(0x3501, 0x0d);
	write_cmos_sensor(0x3502, 0xce);
	write_cmos_sensor(0x3603, 0x0b);
	write_cmos_sensor(0x3608, 0x4a);
	write_cmos_sensor(0x360d, 0x4a);
	write_cmos_sensor(0x3622, 0x66);
	write_cmos_sensor(0x3633, 0x06);
	write_cmos_sensor(0x3635, 0x2c);
	write_cmos_sensor(0x3636, 0x2c);
	write_cmos_sensor(0x3639, 0x44);
	write_cmos_sensor(0x363a, 0x33);
	write_cmos_sensor(0x366b, 0x00);
	write_cmos_sensor(0x370b, 0xb0);
	write_cmos_sensor(0x3712, 0x00);
	write_cmos_sensor(0x3714, 0x61);
	write_cmos_sensor(0x3800, 0x00);
	write_cmos_sensor(0x3801, 0x00);
	write_cmos_sensor(0x3802, 0x00);
	write_cmos_sensor(0x3803, 0x00);
	write_cmos_sensor(0x3804, 0x19);
	write_cmos_sensor(0x3805, 0x9f);
	write_cmos_sensor(0x3806, 0x13);
	write_cmos_sensor(0x3807, 0x3f);
	write_cmos_sensor(0x3808, 0x0c);
	write_cmos_sensor(0x3809, 0xc0);
	write_cmos_sensor(0x380a, 0x09);
	write_cmos_sensor(0x380b, 0x90);
	write_cmos_sensor(0x380c, 0x03);
	write_cmos_sensor(0x380d, 0x48);
	write_cmos_sensor(0x380e, 0x0d);
	write_cmos_sensor(0x380f, 0xf2);
	write_cmos_sensor(0x3811, 0x09);
	write_cmos_sensor(0x3813, 0x08);
	write_cmos_sensor(0x3814, 0x22);
	write_cmos_sensor(0x3815, 0x22);
	write_cmos_sensor(0x3820, 0x01);
	write_cmos_sensor(0x3821, 0x0d);
	write_cmos_sensor(0x3822, 0x00);
	write_cmos_sensor(0x4012, 0x0d);
	write_cmos_sensor(0x4015, 0x02);
	write_cmos_sensor(0x4016, 0x0d);
	write_cmos_sensor(0x4018, 0x03);
	write_cmos_sensor(0x401e, 0x01);
	write_cmos_sensor(0x401f, 0x0c);
	write_cmos_sensor(0x4837, 0x06);
	write_cmos_sensor(0x5000, 0x89);
	write_cmos_sensor(0x5001, 0x02);
	write_cmos_sensor(0x5002, 0x01);
	write_cmos_sensor(0x5003, 0x7a);
	write_cmos_sensor(0x5005, 0x08);
	write_cmos_sensor(0x5014, 0x30);
	write_cmos_sensor(0x5015, 0x06);
	write_cmos_sensor(0x5035, 0x08);
	write_cmos_sensor(0x5037, 0x08);
	write_cmos_sensor(0x5038, 0x0c);
	write_cmos_sensor(0x5039, 0xc0);
	write_cmos_sensor(0x503a, 0x09);
	write_cmos_sensor(0x503b, 0x90);
	write_cmos_sensor(0x5185, 0x0b);
	write_cmos_sensor(0x518c, 0x01);
	write_cmos_sensor(0x518d, 0x01);
	write_cmos_sensor(0x518e, 0x01);
	write_cmos_sensor(0x518f, 0x01);
	write_cmos_sensor(0x5207, 0xff);
	write_cmos_sensor(0x5208, 0xc1);
	write_cmos_sensor(0x5380, 0x0c);
	write_cmos_sensor(0x5381, 0x06);
	write_cmos_sensor(0x5386, 0x14);
	write_cmos_sensor(0x5387, 0x60);
	write_cmos_sensor(0x5388, 0x0f);
	write_cmos_sensor(0x5389, 0xc8);
	write_cmos_sensor(0x5880, 0xc5);
	write_cmos_sensor(0x5884, 0x18);
	write_cmos_sensor(0x5885, 0x08);
	write_cmos_sensor(0x5886, 0x08);
	write_cmos_sensor(0x5887, 0x18);
	write_cmos_sensor(0x5889, 0x05);
	write_cmos_sensor(0x588a, 0x00);
	write_cmos_sensor(0x58c0, 0x10);
	write_cmos_sensor(0x58c2, 0x0e);
	write_cmos_sensor(0x58c3, 0x0c);
	write_cmos_sensor(0x58c4, 0x04);
	write_cmos_sensor(0x58c5, 0x01);
	write_cmos_sensor(0x58c6, 0xf7);
	write_cmos_sensor(0x58c8, 0x6f);
	write_cmos_sensor(0x58ca, 0x1d);
	write_cmos_sensor(0x58cb, 0x01);
	write_cmos_sensor(0x58cc, 0xfd);
	write_cmos_sensor(0x0305, 0x29);
	write_cmos_sensor(0x4837, 0x10);
}	/*	preview_setting  */
/*************************************************************************
 * FUNCTION
 *	Capture
 *
 * DESCRIPTION
 *	Sensor capture
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	write_cmos_sensor(0x0305, 0x3f);
	write_cmos_sensor(0x0344, 0x01);
	write_cmos_sensor(0x0345, 0xce);
	write_cmos_sensor(0x034a, 0x0a);
	write_cmos_sensor(0x034b, 0x00);
	write_cmos_sensor(0x3501, 0x13);
	write_cmos_sensor(0x3502, 0x64);
	write_cmos_sensor(0x3603, 0x0b);
	write_cmos_sensor(0x3608, 0x63);
	write_cmos_sensor(0x360d, 0x61);
	write_cmos_sensor(0x3622, 0x55);
	write_cmos_sensor(0x3633, 0x03);
	write_cmos_sensor(0x3635, 0x0c);
	write_cmos_sensor(0x3636, 0x0c);
	write_cmos_sensor(0x3639, 0xcc);
	write_cmos_sensor(0x363a, 0xcc);
	write_cmos_sensor(0x366b, 0x02);
	write_cmos_sensor(0x370b, 0xb0);
	write_cmos_sensor(0x3712, 0x00);
	write_cmos_sensor(0x3714, 0x67);
	write_cmos_sensor(0x3800, 0x00);
	write_cmos_sensor(0x3801, 0x00);
	write_cmos_sensor(0x3802, 0x00);
	write_cmos_sensor(0x3803, 0x00);
	write_cmos_sensor(0x3804, 0x19);
	write_cmos_sensor(0x3805, 0x9f);
	write_cmos_sensor(0x3806, 0x13);
	write_cmos_sensor(0x3807, 0x3f);
	write_cmos_sensor(0x3808, 0x19);
	write_cmos_sensor(0x3809, 0x80);
	write_cmos_sensor(0x380a, 0x13);
	write_cmos_sensor(0x380b, 0x20);
	write_cmos_sensor(0x380c, 0x04);
	write_cmos_sensor(0x380d, 0xb0);
	write_cmos_sensor(0x380e, 0x13);
	write_cmos_sensor(0x380f, 0x88);
	write_cmos_sensor(0x3811, 0x11);
	write_cmos_sensor(0x3813, 0x10);
	write_cmos_sensor(0x3814, 0x11);
	write_cmos_sensor(0x3815, 0x11);
	write_cmos_sensor(0x3820, 0x00);
	write_cmos_sensor(0x3821, 0x04);
	write_cmos_sensor(0x3822, 0x00);
	write_cmos_sensor(0x4012, 0x7d);
	write_cmos_sensor(0x4015, 0x04);
	write_cmos_sensor(0x4016, 0x1b);
	write_cmos_sensor(0x4018, 0x07);
	write_cmos_sensor(0x401e, 0x01);
	write_cmos_sensor(0x401f, 0x0c);
	write_cmos_sensor(0x4837, 0x0a);
	write_cmos_sensor(0x5000, 0xc9);
	write_cmos_sensor(0x5001, 0x42);
	write_cmos_sensor(0x5002, 0x01);
	write_cmos_sensor(0x5003, 0x7a);
	write_cmos_sensor(0x5005, 0x00);
	write_cmos_sensor(0x5014, 0x00);
	write_cmos_sensor(0x5015, 0x06);
	write_cmos_sensor(0x5035, 0x10);
	write_cmos_sensor(0x5037, 0x10);
	write_cmos_sensor(0x5038, 0x19);
	write_cmos_sensor(0x5039, 0x80);
	write_cmos_sensor(0x503a, 0x13);
	write_cmos_sensor(0x503b, 0x20);
	write_cmos_sensor(0x5185, 0x0c);
	write_cmos_sensor(0x518c, 0x01);
	write_cmos_sensor(0x518d, 0x01);
	write_cmos_sensor(0x518e, 0x01);
	write_cmos_sensor(0x518f, 0x01);
	write_cmos_sensor(0x5207, 0x05);
	write_cmos_sensor(0x5208, 0x41);
	write_cmos_sensor(0x5380, 0x0f);
	write_cmos_sensor(0x5381, 0x00);
	write_cmos_sensor(0x5386, 0x19);
	write_cmos_sensor(0x5387, 0xa0);
	write_cmos_sensor(0x5388, 0x13);
	write_cmos_sensor(0x5389, 0x40);
	write_cmos_sensor(0x5880, 0xc5);
	write_cmos_sensor(0x5884, 0x18);
	write_cmos_sensor(0x5885, 0x08);
	write_cmos_sensor(0x5886, 0x08);
	write_cmos_sensor(0x5887, 0x18);
	write_cmos_sensor(0x5889, 0x05);
	write_cmos_sensor(0x588a, 0x00);
	write_cmos_sensor(0x58c0, 0x10);
	write_cmos_sensor(0x58c2, 0x0e);
	write_cmos_sensor(0x58c3, 0x0c);
	write_cmos_sensor(0x58c4, 0x04);
	write_cmos_sensor(0x58c5, 0x01);
	write_cmos_sensor(0x58c6, 0xf7);
	write_cmos_sensor(0x58c8, 0x6f);
	write_cmos_sensor(0x58ca, 0x1d);
	write_cmos_sensor(0x58cb, 0x01);
	write_cmos_sensor(0x58cc, 0xfd);
	write_cmos_sensor(0x0305, 0x35);
	write_cmos_sensor(0x4837, 0x0c); 
}

static void custom1_setting(void)
{
	LOG_INF("E!\n");
	write_cmos_sensor(0xfd, 0x01);
	write_cmos_sensor(0x05, 0x02);
	write_cmos_sensor(0x06, 0x04);
	write_cmos_sensor(0x03, 0x02);
	write_cmos_sensor(0x04, 0x00);
	write_cmos_sensor(0xac, 0x00);
}



/*************************************************************************
 * FUNCTION
 *	Video
 *
 * DESCRIPTION
 *	Sensor video
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/

static void hs_video_setting(void)
{
	LOG_INF("E\n");
	preview_setting();
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	preview_setting();
}


/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
 static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x300a) << 16) |
		(read_cmos_sensor(0x300b) << 8) | read_cmos_sensor(0x300c));
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	LOG_INF("OV32A,get_imgsensor_id Begin!!\n");
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
	spin_lock(&imgsensor_drv_lock);
	imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
	spin_unlock(&imgsensor_drv_lock);
	do {
		*sensor_id = return_sensor_id();
		LOG_INF("OV32A,get_imgsensor_id: 0x%x !!\n", *sensor_id);
		if (*sensor_id == imgsensor_info.sensor_id) {
			LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
			imgsensor.i2c_write_id, *sensor_id);
			printk("OV32A check otp Begin!!\n");
			return ERROR_NONE;
		}
		retry--;
	} while (retry > 0);
	i++;
	retry = 1;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		LOG_INF("get_imgsensor_id: 0x%x fail\n", *sensor_id);
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 1;
	kal_uint32 sensor_id = 0;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
	spin_lock(&imgsensor_drv_lock);
	imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
	spin_unlock(&imgsensor_drv_lock);
	do {
	sensor_id = return_sensor_id();
	if (sensor_id == imgsensor_info.sensor_id) {
		LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
			imgsensor.i2c_write_id, sensor_id);
		break;
	}
		retry--;
	} while (retry > 0);
	i++;
	if (sensor_id == imgsensor_info.sensor_id)
	break;
	retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id) {
		LOG_INF("Open sensor id: 0x%x fail\n", sensor_id);
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	sensor_init();

	spin_lock(&imgsensor_drv_lock);
	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");
	streaming_control(KAL_FALSE);
	/*No Need to implement this function*/
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/*imgsensor.video_mode = KAL_FALSE;*/
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	//set_mirror_flip(imgsensor.mirror);
	mdelay(10);
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);

	mdelay(10);
	//set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}	/* capture() */


static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	//set_mirror_flip(imgsensor.mirror);
	mdelay(10);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	//set_mirror_flip(imgsensor.mirror);
	mdelay(10);

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				 MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	//set_mirror_flip(imgsensor.mirror);
	mdelay(10);

	return ERROR_NONE;
}	/*	slim_video	 */

static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting();
	//set_mirror_flip(imgsensor.mirror);
	mdelay(10);
	return ERROR_NONE;
}   /*  Custom1   */


static kal_uint32 get_resolution(
			MSDK_SENSOR_RESOLUTION_INFO_STRUCT(*sensor_resolution))
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;

	return ERROR_NONE;
}	/*	get_resolution	*/


static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
				MSDK_SENSOR_INFO_STRUCT *sensor_info,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
    LOG_INF("mipi_lane_num = %d\n", imgsensor_info.mipi_lane_num);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* not use */
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* inverse with datasheet*/
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */
	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;

	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame; /* The delay frame of setting frame length  */

	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x*/
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;

	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
		break;

	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
		break;

	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;

	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;

	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
		break;

	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}
	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			Custom1(image_window, sensor_config_data);  // Custom1
			break;
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate*/
	if (framerate == 0) {
		/* Dynamic frame rate*/
		return ERROR_NONE;
	}
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE)) {
		imgsensor.current_fps = 296;
	} else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE)) {
		imgsensor.current_fps = 146;
	} else {
		imgsensor.current_fps = framerate;
	}
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) {/*enable auto flicker	  */
		imgsensor.autoflicker_en = KAL_TRUE;
	} else {/*Cancel Auto flick*/
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.pre.framelength) {
			imgsensor.dummy_line = (frame_length - imgsensor_info.pre.framelength);
		} else {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;

	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0) {
			return ERROR_NONE;
		}

		frame_length = imgsensor_info.normal_video.pclk / framerate * 10;

		frame_length /= imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);

		if (frame_length > imgsensor_info.normal_video.framelength) {
			imgsensor.dummy_line = frame_length - imgsensor_info.normal_video.framelength;
		} else {
			imgsensor.dummy_line = 0;
		}

		imgsensor.frame_length =
		imgsensor_info.normal_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;

	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.cap.framelength) {
			imgsensor.dummy_line = (frame_length - imgsensor_info.cap.framelength);
		} else {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;

	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10;

		frame_length /= imgsensor_info.hs_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.hs_video.framelength) {
			imgsensor.dummy_line =(frame_length - imgsensor_info.hs_video.framelength);
		} else {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;

	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10;

		frame_length /= imgsensor_info.slim_video.linelength;

		spin_lock(&imgsensor_drv_lock);

		if (frame_length > imgsensor_info.slim_video.framelength) {
			imgsensor.dummy_line = (frame_length - imgsensor_info.slim_video.framelength);
		} else {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
		if (imgsensor.dummy_line < 0) {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter) {
			set_dummy();
		}
		break;

	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.pre.framelength) {
			imgsensor.dummy_line = (frame_length - imgsensor_info.pre.framelength);
		} else {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
			enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	default:
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
	//	write_cmos_sensor(0x5000, 0x57);
	//	write_cmos_sensor(0x5001, 0x02);
	//	write_cmos_sensor(0x5e00, 0x80);
	} else {
	//	write_cmos_sensor(0x5000, 0x77);
	//	write_cmos_sensor(0x5001, 0x0a);
	//	write_cmos_sensor(0x5e00, 0x00);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	printk("OV32A,streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor(0x0100, 0X01);
	else
		write_cmos_sensor(0x0100, 0x00);
	mdelay(10);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
	UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
				(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
    	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
						= imgsensor_info.cap.pclk;
				break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
						= imgsensor_info.normal_video.pclk;
				break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
						= imgsensor_info.hs_video.pclk;
				break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
						= imgsensor_info.slim_video.pclk;
				break;
		case MSDK_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
						= imgsensor_info.custom1.pclk;
				break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
						= imgsensor_info.pre.pclk;
				break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
						= (imgsensor_info.cap.framelength << 16)
								 + imgsensor_info.cap.linelength;
				break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
						= (imgsensor_info.normal_video.framelength << 16)
								+ imgsensor_info.normal_video.linelength;
				break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
						= (imgsensor_info.hs_video.framelength << 16)
								 + imgsensor_info.hs_video.linelength;
				break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
						= (imgsensor_info.slim_video.framelength << 16)
								 + imgsensor_info.slim_video.linelength;
				break;
		case MSDK_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
						= (imgsensor_info.custom1.framelength << 16)
								 + imgsensor_info.custom1.linelength;
				break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
						= (imgsensor_info.pre.framelength << 16)
								 + imgsensor_info.pre.linelength;
				break;
		}
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) (*feature_data));
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;

	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;

	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		break;

	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16 + 1));
		break;

	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data + 1));
		break;

	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data),(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;

	/*for factory mode auto testing*/
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;

	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = (UINT8)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			LOG_INF("lch MSDK_SCENARIO_ID_CUSTOM1 \n");
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[5], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;

	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", (UINT16)*feature_data, (UINT16)*(feature_data + 1), (UINT16)*(feature_data + 2));

		ihdr_write_shutter_gain((UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16)*(feature_data + 2));
		break;

	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data), (UINT16) (*(feature_data + 1)));
		break;

	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		LOG_INF("This sensor can't support temperature get\n");
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);

		if (*feature_data != 0) {
			set_shutter(*feature_data);
		}
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		{
			kal_uint32 rate;
			switch (*feature_data) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					rate = imgsensor_info.cap.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					rate = imgsensor_info.normal_video.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					rate = imgsensor_info.hs_video.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_CUSTOM1:
					rate = imgsensor_info.custom1.mipi_pixel_rate;
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					rate = imgsensor_info.pre.mipi_pixel_rate;
					break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
		}
		break;
	default:
		break;
	}

	return ERROR_NONE;
}    /*    feature_control()  */


static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 OV32A_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL) {
		*pfFunc = &sensor_func;
	}
	return ERROR_NONE;
}	/*	ov32a1q_MIPI_RAW_SensorInit	*/
