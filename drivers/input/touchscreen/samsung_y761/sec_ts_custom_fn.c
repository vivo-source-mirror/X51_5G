/* drivers/input/touchscreen/sec_ts_fn.c
 *
 * Copyright (C) 2015 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung TSC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/unaligned.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/ctype.h>
#include <linux/hrtimer.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>
#include "sec_ts.h"
//#include "../vts_core.h"
#include <linux/vivo_ts_function.h>


int sec_ts_i2c_write(struct sec_ts_data *ts, u8 reg, u8 *data, int len);
extern int bbk_xxsw_reset(struct sec_ts_data *ts);
int bbk_xxsw_reset(struct sec_ts_data *ts)
{
		int ret;

	VTI("sw reset");
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SW_RESET, NULL, 0);
	if (ret < 0) {
		VTE("%s: write fail, sw_reset\n", __func__);
	}

	sec_ts_delay(300);

	return ret;
}


int bbk_slsi_read_charger_bit(void)
{
	struct sec_ts_data *ts = g_ts_data;
	int ret = 0;
	char data = 0;

	ret = sec_ts_i2c_read(ts, SET_TS_CMD_SET_NOISE_MODE, &data, 1);
	if (ret < 0) {
		VTE("%s: failed to read charger stauts(%d)\n",
					__func__, ret);
		return ret;
	}

	return (int)data;
}


int bbk_slsi_get_module_id(void)
{
	int ret = 0;
	struct sec_ts_data *ts = g_ts_data;
	/*
	add get modul id
	*/
	u8 tBuff[8];
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_IMG_VERSION, tBuff, 8);
	VTI("%s: Read IMG version: %x %x %x %x\n", __func__,
			tBuff[0], tBuff[1], tBuff[2], tBuff[3]);

	return ret;
}

int bbk_slsi_gesture_point_get(u16 *data)
{
	struct sec_ts_data *ts = g_ts_data;
	int ret = 0;
	int i = 0;
	u8 buff[20] = { 0 };

	VTI("gesture point num is %d", ts->read_gesture_point_num);
	ret = sec_ts_i2c_read(ts, SEC_TS_CMD_GESTURECOORD, buff, 20);
	if (ret < 0) {
		VTE("i2c read gesture coord failed\n");
		return ret;
	}

	switch(buff[0]) {
		case GESTURE_DOUBLE_CLICK:
		case GESTURE_DOWN_SLIDE:
		case GESTURE_LEFT_SLIDE:
		case GESTURE_RIGHT_SLIDE:
		case GESTURE_M:
			return 0;
		case GESTURE_UP_SLIDE:
			ret = 2;
			break;
		case GESTURE_W:
			ret = 5;
			break;
		default:
			ret = 6;
			break;
	}

	for (i = 0; i < 6; i++) {
		if (i >= ret) {
			data[i * 2] = 0;
			data[i * 2 + 1] = 0;
			continue;
		}
		data[i * 2] = (buff[3 * i + 1] << 4) | (buff[3 * i + 3] >> 4);
		data[i * 2 + 1] = (buff[3 * i + 2] << 4) | (buff[3 * i + 3] & 0x0f);
	}
	if (buff[0] == GESTURE_O)  // orientation info for 'O'
		data[12] = buff[19];

	return ret;
}