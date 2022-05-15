/* drivers/input/touchscreen/sec_ts.c
 *
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung TSC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "sec_ts.h"
#include  <linux/vivo_ts_function.h>  
//#include "../vts_core.h"
//#include <linux/bbk_drivers_info.h>

#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <drm/drm_panel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/fs.h>




#ifdef USE_POWER_RESET_WORK
struct sec_ts_data *tsp_info;
struct sec_ts_data *g_ts_data;

static void sec_ts_reset_work(struct work_struct *work);
#endif
#if 0
static void sec_ts_read_info_work(struct work_struct *work);
#endif
#ifdef USE_OPEN_CLOSE
static int sec_ts_input_open(struct input_dev *dev);
static void sec_ts_input_close(struct input_dev *dev);
#endif

#define parse_property(np, prop_name, data_type, val, err_return, err_default) do { \
		if(of_property_read_##data_type(np, prop_name, val)) { \
			if (err_return) {\
				VTE("get property "prop_name" failed!!\n"); \
				return -EINVAL; \
			} \
			\
			*val = err_default; \
			VTI("property "prop_name" not configed, set to default value:"#err_default"\n"); \
		} \
	} while (0)

#define parse_property_u32_with_default(np, prop_name, val, err_default) parse_property(np, prop_name, u32, val, false, err_default)
#define parse_property_u8_with_default(np, prop_name, val, err_default) parse_property(np, prop_name, u8, val, false, err_default)

int sec_ts_read_information(struct sec_ts_data *ts);

//static int bbk_slsi_setEdgeRejectArea(struct vts_device *vtsdev, struct vts_edge_cmd *cmd);

void vivo_get_file_path_str(char **pp_file, char *file_path)
{
#if 0
	char *p = NULL;
	char *p1 = NULL;

	p = strrchr(file_path, '/');
	if (p != NULL) {
		strlcpy(pp_file, p + 1, 255);
		p1 = strrchr(pp_file, '.');
		if (p1 != NULL) {
			*p1 = 0;
		}
	}
#else
	char *p = NULL;
	p = strrchr(file_path, '/');
	if (p != NULL)
		*pp_file = p+1;
	else
		*pp_file = file_path;

#endif
}

int get_ts_log_switch(void)
{
	return 0;
}


int sec_ts_i2c_write(struct sec_ts_data *ts, u8 reg, u8 *data, int len)
{
	u8 buf[I2C_WRITE_BUFFER_SIZE + 1];
	int ret;
	unsigned char retry;
	//struct vts_device *vtsdev = ts->vtsdev;
	struct i2c_msg msg;
	//int i;

	if (len > I2C_WRITE_BUFFER_SIZE) {
		VTE("%s: len is larger than buffer size\n", __func__);
		return -EINVAL;
	}

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTE("%s: POWER_STATUS : OFF\n", __func__);
		goto err;
	}

	buf[0] = reg;
	memcpy(buf + 1, data, len);

	msg.addr = ts->client->addr;
	msg.flags = 0;
	msg.len = len + 1;
	msg.buf = buf;

	mutex_lock(&ts->i2c_mutex);
	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		ret = i2c_transfer(ts->client->adapter, &msg, 1);
		if (ret == 1)
			break;

		if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
			VTE("%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
			mutex_unlock(&ts->i2c_mutex);
			goto err;
		}

		usleep_range(1 * 1000, 1 * 1000);

		if (retry > 1) {
			VTE("%s: I2C retry %d, ret:%d\n", __func__, retry + 1, ret);
			ts->comm_err_count++;
		}
	}

	mutex_unlock(&ts->i2c_mutex);

	if (retry == SEC_TS_I2C_RETRY_CNT) {
		VTE("%s: I2C write over retry limit\n", __func__);
		ret = -EIO;	
#ifdef USE_POR_AFTER_I2C_RETRY
		if (ts->probe_done && !ts->reset_is_on_going) {
			if (ts->reset_count > RESET_MAX) {
				VTE("tp reset over retry limit %d", ts->reset_count -1);
				goto err;
			}
			ts->reset_count++;
			schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));	
		}							
#endif
	} else {
		ts->reset_count = 0;
	}

	/*vts_debug_code() {
		pr_info("VIVO_TS sec_input:i2c_cmd: W: %02X | ", reg);
		for (i = 0; i < len; i++)
			pr_cont("%02X ", data[i]);
		pr_cont("\n");
	}*/

	if (ret == 1)
		return 0;
err:
	return -EIO;
}


int sec_ts_i2c_read(struct sec_ts_data *ts, u8 reg, u8 *data, int len)
{
	u8 buf[4];
	int ret;
	unsigned char retry;	
	//struct vts_device *vtsdev = ts->vtsdev;
	struct i2c_msg msg[2];
	int remain = len;
//	int i;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTE("%s: POWER_STATUS : OFF\n", __func__);
		goto err;
	}

	buf[0] = reg;

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	mutex_lock(&ts->i2c_mutex);

	if (len <= ts->i2c_burstmax) {

		for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
			ret = i2c_transfer(ts->client->adapter, msg, 2);
			if (ret == 2)
				break;
			usleep_range(1 * 1000, 1 * 1000);
			if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
				VTE("%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
				mutex_unlock(&ts->i2c_mutex);
				goto err;
			}

			if (retry > 1) {
				VTE("%s: I2C retry %d, ret:%d\n",
					__func__, retry + 1, ret);
				ts->comm_err_count++;
			}
		}

	} else {
		/*
		 * I2C read buffer is 256 byte. do not support long buffer over than 256.
		 * So, try to seperate reading data about 256 bytes.
		 */

		for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
			ret = i2c_transfer(ts->client->adapter, msg, 1);
			if (ret == 1)
				break;
			usleep_range(1 * 1000, 1 * 1000);
			if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
				VTE("%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
				mutex_unlock(&ts->i2c_mutex);
				goto err;
			}

			if (retry > 1) {
				VTE("%s: I2C retry %d, ret:%d\n",
					__func__, retry + 1, ret);
				ts->comm_err_count++;
			}
		}

		do {
			if (remain > ts->i2c_burstmax)
				msg[1].len = ts->i2c_burstmax;
			else
				msg[1].len = remain;

			remain -= ts->i2c_burstmax;

			for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
				ret = i2c_transfer(ts->client->adapter, &msg[1], 1);
				if (ret == 1)
					break;
				usleep_range(1 * 1000, 1 * 1000);
				if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
					VTE("%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
					mutex_unlock(&ts->i2c_mutex);
					goto err;
				}

				if (retry > 1) {
					VTE("%s: I2C retry %d, ret:%d\n",
						__func__, retry + 1, ret);
					ts->comm_err_count++;
				}
			}

			msg[1].buf += msg[1].len;

		} while (remain > 0);

	}

	mutex_unlock(&ts->i2c_mutex);

	if (retry == SEC_TS_I2C_RETRY_CNT) {
		VTE("%s: I2C read over retry limit\n", __func__);
		ret = -EIO;
#ifdef USE_POR_AFTER_I2C_RETRY
		if (ts->probe_done && !ts->reset_is_on_going) {
			if (ts->reset_count > RESET_MAX) {
				VTE("tp reset over retry limit %d", ts->reset_count -1);
				goto err;
			}
			ts->reset_count++;
			schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));	
		}							
#endif

	} else {
		ts->reset_count = 0;
	}

	return ret;

err:
	return -EIO;
}

static int sec_ts_i2c_write_burst(struct sec_ts_data *ts, u8 *data, int len)
{
	int ret;
	int retry;

	mutex_lock(&ts->i2c_mutex);
	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		ret = i2c_master_send(ts->client, data, len);
		if (ret == len)
			break;

		usleep_range(1 * 1000, 1 * 1000);

		if (retry > 1) {
			VTE("%s: I2C retry %d, ret:%d\n", __func__, retry + 1, ret);
			ts->comm_err_count++;
		}
	}

	mutex_unlock(&ts->i2c_mutex);
	if (retry == SEC_TS_I2C_RETRY_CNT) {
		VTE("%s: I2C write over retry limit\n", __func__);
		ret = -EIO;	
	}

	return ret;
}

static int sec_ts_i2c_read_bulk(struct sec_ts_data *ts, u8 *data, int len)
{
	int ret;
	unsigned char retry;
	int remain = len;
	struct i2c_msg msg;

	msg.addr = ts->client->addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	msg.buf = data;

	mutex_lock(&ts->i2c_mutex);

	do {
		if (remain > ts->i2c_burstmax)
			msg.len = ts->i2c_burstmax;
		else
			msg.len = remain;

		remain -= ts->i2c_burstmax;

		for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
			ret = i2c_transfer(ts->client->adapter, &msg, 1);
			if (ret == 1)
				break;
			usleep_range(1 * 1000, 1 * 1000);

			if (retry > 1) {
				VTE("%s: I2C retry %d, ret:%d\n",
					__func__, retry + 1, ret);
				ts->comm_err_count++;
			}
		}

		if (retry == SEC_TS_I2C_RETRY_CNT) {
			VTE("%s: I2C read over retry limit\n", __func__);
			ret = -EIO;			
			break;
		}
		msg.buf += msg.len;

	} while (remain > 0);

	mutex_unlock(&ts->i2c_mutex);

	if (ret == 1)
		return 0;

	return -EIO;
}


static void sec_ts_check_rawdata(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data, ghost_check.work);

	if (ts->tsp_dump_lock == 1) {
		VTE("%s: ignored ## already checking..\n", __func__);
		return;
	}
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTE("%s: ignored ## IC is power off\n", __func__);
		return;
	}

	VTI("%s: start ##\n", __func__);
	sec_ts_run_rawdata_all(ts);
	msleep(100);

	VTI("%s: done ##\n", __func__);
}

void sec_ts_delay(unsigned int ms)
{
	if (ms < 20)
		usleep_range(ms * 1000, ms * 1000);
	else
		msleep(ms);
}

int sec_ts_wait_for_ready(struct sec_ts_data *ts, unsigned int ack)
{
	int rc = -1;
	int retry = 0;
	u8 tBuff[SEC_TS_EVENT_BUFF_SIZE] = {0,};
	
	while (sec_ts_i2c_read(ts, SEC_TS_READ_ONE_EVENT, tBuff, SEC_TS_EVENT_BUFF_SIZE) > 0) {
		if (((tBuff[0] >> 2) & 0xF) == TYPE_STATUS_EVENT_INFO) {
			if (tBuff[1] == ack) {
				rc = 0;
				break;
			}
		} else if (((tBuff[0] >> 2) & 0xF) == TYPE_STATUS_EVENT_VENDOR_INFO) {
			if (tBuff[1] == ack) {
				rc = 0;
				break;
			}
		}

		if (retry++ > SEC_TS_WAIT_RETRY_CNT) {
			VTE("%s: Time Over\n", __func__);
			break;
		}
		sec_ts_delay(20);
	}

	VTI("%s: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X [%d]\n",
			__func__, tBuff[0], tBuff[1], tBuff[2], tBuff[3],
			tBuff[4], tBuff[5], tBuff[6], tBuff[7], retry);

	return rc;
}

int bbk_slsi_erase_vivo_cal(u8 val)
{
	int ret = 0;
	struct sec_ts_data *ts = g_ts_data;
	u8 tBuff[1] = { 0x00 };
	VTI("enter");
	tBuff[0] = val;
	ret = sec_ts_i2c_write(ts, SEC_TS_VIVO_STATUS_COMMAND, tBuff, sizeof(tBuff));// Set to Doesn't have vivo offset calibration
	if (ret < 0) {
		VTI("fail to Erase Vivo Calibration flag!");
	}

	return ret;
}

int bbk_slsi_get_vivo_calibration_status(int *cali_satus)
{
	int ret = 0;
	struct sec_ts_data *ts = g_ts_data;
	u8 vivo_cal_status[1] = {0};
	VTI("enter");
	ret = sec_ts_i2c_read(ts, SEC_TS_VIVO_STATUS_COMMAND, vivo_cal_status, 1);

	if (ret < 0) {
		VTI("failed to read Vivo Calibration status");
		return ret;
	}

	VTI("cal status is 0x%x", vivo_cal_status[0]);
	*cali_satus = vivo_cal_status[0];
	return  ret;
}

int bbk_slsi_start_force_calibration(void)
{
	int ret = 0;
	struct sec_ts_data *ts = g_ts_data;

	VTI("enter");
	ret = sec_ts_execute_force_calibration(ts, OFFSET_CAL_SET);
	if (ret < 0) {
		VTI("fail to write OFFSET CAL SEC!");
	} else {
		VTI("success to write OFFSET CAL SEC!");
	}

	return ret;
}

void sec_ts_reinit(struct sec_ts_data *ts)
{
	u8 w_data[2] = {0x00, 0x00};
	int ret = 0;

	VTI("%s : charger=0x%x, touch_functions=0x%x, Power mode=0x%x, noise_mode=%d\n",
			__func__, ts->charger_mode, ts->touch_functions,
			ts->power_status, ts->touch_noise_status);

	ts->touch_noise_status = 0;

	/* charger mode */
	if (ts->charger_mode != SEC_TS_BIT_CHARGER_MODE_NO) {
		w_data[0] = ts->charger_mode;
		ret = ts->sec_ts_i2c_write(ts, SET_TS_CMD_SET_CHARGER_MODE, (u8 *)&w_data[0], 1);
		if (ret < 0)
			VTE("%s: Failed to send command(0x%x)",
					__func__, SET_TS_CMD_SET_CHARGER_MODE);
	}

	/* Cover mode */
	if (ts->touch_functions & SEC_TS_BIT_SETFUNC_COVER) {
		w_data[0] = ts->cover_cmd;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_COVERTYPE, (u8 *)&w_data[0], 1);
		if (ret < 0)
			VTE("%s: Failed to send command(0x%x)",
					__func__, SEC_TS_CMD_SET_COVERTYPE);
	}

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, (u8 *)&(ts->touch_functions), 2);
	if (ret < 0)
		VTE("%s: Failed to send command(0x%x)",
				__func__, SEC_TS_CMD_SET_TOUCHFUNCTION);

	/* Power mode */
	if (ts->power_status == SEC_TS_STATE_LPM) {

		w_data[0] = TO_LOWPOWER_MODE;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_POWER_MODE, (u8 *)&w_data[0], 1);
		if (ret < 0)
			VTE("%s: Failed to send command(0x%x)",
					__func__, SEC_TS_CMD_SET_POWER_MODE);

		sec_ts_delay(50);

	} else {
		sec_ts_set_grip_type(ts, ONLY_EDGE_HANDLER);
		//vts_reset(vtsdev);
	}

	ts->edge_app_flag = 0;
	
	return;
}

#define MAX_EVENT_COUNT 128
#include <uapi/linux/input.h>
int sec_ts_set_active_mode(struct sec_ts_data *ts, bool enable)
{
	int ret;
	u8 active_mode_on[] = {0x01};
	u8 active_mode_off[] = {0x00};

	if (enable) {
		VTI("sec_ts_set_active_mode : active mode on!!==\n");
		ret = ts->sec_ts_i2c_write(ts, 0x48, active_mode_on, sizeof(active_mode_on));
		if (ret < 0)
			VTE("sec_ts_set_active_mode: fail to write ACTIVE_MODE_ON\n");
	} else {
		VTI("sec_ts_set_active_mode : active mode off!!\n");
		ret = ts->sec_ts_i2c_write(ts, 0x48, active_mode_off, sizeof(active_mode_off));
		if (ret < 0)
			VTE("sec_ts_set_active_mode: fail to write ACTIVE_MODE_OFF\n");
	}

	return ret;
}

enum point_status {
	DOWN,
	UP
};

static void sec_report_point(enum point_status status, int x, int y, int id, int major)
{
	x = x * 1080 / 1440;
	y = y * 2376 / 3120;
	switch(status) {
		case DOWN:
			input_mt_slot(g_ts_data->input_dev_pad, id);
			input_report_key(g_ts_data->input_dev_pad, BTN_TOUCH, 1);
			input_report_key(g_ts_data->input_dev_pad, BTN_TOOL_FINGER, 1);
			input_mt_report_slot_state(g_ts_data->input_dev_pad, MT_TOOL_FINGER, 1);
			//input_report_abs(g_ts_data->input_dev_pad, ABS_MT_TRACKING_ID, id);
			input_report_abs(g_ts_data->input_dev_pad, ABS_MT_POSITION_X, x);
			input_report_abs(g_ts_data->input_dev_pad, ABS_MT_POSITION_Y, y);
			input_report_abs(g_ts_data->input_dev_pad, ABS_MT_PRESSURE, major);
			input_report_abs(g_ts_data->input_dev_pad, ABS_MT_TOUCH_MAJOR, major);
			input_report_abs(g_ts_data->input_dev_pad, ABS_MT_TOUCH_MINOR, major);
			input_sync(g_ts_data->input_dev_pad);
			break;
		case UP:
			input_mt_slot(g_ts_data->input_dev_pad, id);
			//input_report_abs(g_ts_data->input_dev_pad, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(g_ts_data->input_dev_pad, MT_TOOL_FINGER, 0);
			input_report_key(g_ts_data->input_dev_pad, BTN_TOOL_FINGER, 0);
			input_report_key(g_ts_data->input_dev_pad, BTN_TOUCH, 0);
			input_sync(g_ts_data->input_dev_pad);
	}
}

static void sec_ts_read_event(struct sec_ts_data *ts)
{
	int ret;
	u8 t_id;
	u8 event_id;
	u8 left_event_count;
	u8 read_event_buff[MAX_EVENT_COUNT][SEC_TS_EVENT_BUFF_SIZE] = { { 0 } };
	u8 *event_buff;
	struct sec_ts_event_coordinate *p_event_coord;
	struct sec_ts_event_status *p_event_status;
	int curr_pos;
	int remain_event_count = 0;
	int pre_ttype = 0;
#ifdef CONFIG_VIRTUAL_PROX_ON_TOUCH
	int pick[3] = {0};
#endif

	if (ts->power_status == SEC_TS_STATE_LPM) {

		/* waiting for blsp block resuming, if not occurs i2c error */
		ret = wait_for_completion_interruptible_timeout(&ts->resume_done, msecs_to_jiffies(500));
		if (ret == 0) {
			VTE("%s: LPM: pm resume is not handled\n", __func__);
			return;
		}

		if (ret < 0) {
			VTE("%s: LPM: -ERESTARTSYS if interrupted, %d\n", __func__, ret);
			return;
		}

		VTI("%s: run LPM interrupt handler, %d\n", __func__, ret);
		/* run lpm interrupt handler */
	}

	ret = t_id = event_id = curr_pos = remain_event_count = 0;
	/* repeat READ_ONE_EVENT until buffer is empty(No event) */
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_ONE_EVENT, (u8 *)read_event_buff[0], SEC_TS_EVENT_BUFF_SIZE);
	if (ret < 0) {
		VTE("%s: i2c read one event failed\n", __func__);
		return;
	}

	VTD("ONE: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			read_event_buff[0][0], read_event_buff[0][1],
			read_event_buff[0][2], read_event_buff[0][3],
			read_event_buff[0][4], read_event_buff[0][5],
			read_event_buff[0][6], read_event_buff[0][7]);

	if (read_event_buff[0][0] == 0) {
		VTI("%s: event buffer is empty\n", __func__);
		return;
	}

	left_event_count = read_event_buff[0][7] & 0x3F;
	remain_event_count = left_event_count;

	if (left_event_count > MAX_EVENT_COUNT) {
		VTE("%s: event buffer overflow\n", __func__);

		/* write clear event stack command when read_event_count > MAX_EVENT_COUNT */
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
		if (ret < 0)
			VTE("%s: i2c write clear event failed\n", __func__);

		sec_ts_unlocked_release_all_finger(ts);

		return;
	}

	if (left_event_count > 0) {
		ret = sec_ts_i2c_read(ts, SEC_TS_READ_ALL_EVENT, (u8 *)read_event_buff[1],
				sizeof(u8) * (SEC_TS_EVENT_BUFF_SIZE) * (left_event_count));
		if (ret < 0) {
			VTE("%s: i2c read one event failed\n", __func__);
			return;
		}
	}

	do {
		event_buff = read_event_buff[curr_pos];
		event_id = event_buff[0] & 0x3;

		VTD("ALL: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				event_buff[0], event_buff[1], event_buff[2], event_buff[3],
				event_buff[4], event_buff[5], event_buff[6], event_buff[7]);

		switch (event_id) {
		case SEC_TS_STATUS_EVENT:
			p_event_status = (struct sec_ts_event_status *)event_buff;

			/* tchsta == 0 && ttype == 0 && eid == 0 : buffer empty */
			if (p_event_status->stype > 0)
				VTI("%s: STATUS %x %x %x %x %x %x %x %x\n", __func__,
						event_buff[0], event_buff[1], event_buff[2],
						event_buff[3], event_buff[4], event_buff[5],
						event_buff[6], event_buff[7]);

			/* watchdog reset -> send SENSEON command */
			if ((p_event_status->stype == TYPE_STATUS_EVENT_INFO) &&
				(p_event_status->status_id == SEC_TS_ACK_BOOT_COMPLETE) &&
				(p_event_status->status_data_1 == 0x20 || event_buff[2] == 0x10)) {

				sec_ts_unlocked_release_all_finger(ts);
				//vts_report_release(vtsdev);

				ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
				if (ret < 0)
					VTE("%s: fail to write Sense_on\n", __func__);

				sec_ts_reinit(ts);
			}

			/* event queue full-> all finger release */
			if ((p_event_status->stype == TYPE_STATUS_EVENT_ERR) &&
				(p_event_status->status_id == SEC_TS_ERR_EVENT_QUEUE_FULL)) {
				VTE("%s: IC Event Queue is full\n", __func__);
				sec_ts_unlocked_release_all_finger(ts);
				//vts_report_release(vtsdev);
			}

			if ((p_event_status->stype == TYPE_STATUS_EVENT_ERR) &&
				(p_event_status->status_id == SEC_TS_ERR_EVENT_ESD)) {
				VTE("%s: ESD detected. run reset\n", __func__);
#ifdef USE_RESET_DURING_POWER_ON
				schedule_work(&ts->reset_work.work);
#endif
			}

			if ((p_event_status->stype == TYPE_STATUS_EVENT_INFO) &&
				(p_event_status->status_id == SEC_TS_ACK_WET_MODE)) {
				ts->wet_mode = p_event_status->status_data_1;
				VTI("%s: water wet mode %d\n",
						__func__, ts->wet_mode);
				if (ts->wet_mode)
					ts->wet_count++;
			}

			if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
				(p_event_status->status_id == SEC_TS_VENDOR_ACK_PALM_EVENT)) {
				if (p_event_status->status_data_1 == SEC_TS_ACK_PALM_DETECT) {
                	VTI("%s: Palm Detect", __func__);
                	/* Detect */
					ts->large_press = true;
					
       			}
        		else if (p_event_status->status_data_1 == SEC_TS_ACK_PALM_RELEASE) {
                	VTI("%s: Palm Release", __func__);
                	/* Release */
					ts->large_press = false;
        		}
				if (ts->large_press)
					ts->large_press_count++;
			}

			if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
					(p_event_status->status_id == SEC_TS_VENDOR_ACK_NOISE_STATUS_NOTI)) {

				ts->touch_noise_status = !!p_event_status->status_data_1;
				VTI("%s: TSP NOISE MODE %s[%d]\n",
						__func__, ts->touch_noise_status == 0 ? "OFF" : "ON",
						p_event_status->status_data_1);

				if (ts->touch_noise_status)
					ts->noise_count++;
			}
			if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
                (p_event_status->status_id == SEC_TS_VENDOR_ACK_CALLMODE_EVENT)) {
        		if (p_event_status->status_data_1 == SEC_TS_ACK_CALL_DETECT) {
                	VTI("%s: Proxi Detect", __func__);
                	/* Detect */
					pick[0] = 0;
					#ifdef CONFIG_VIRTUAL_PROX_ON_TOUCH
						VTI("algo-prox report near event");
					#endif
       			}
        		else if (p_event_status->status_data_1 == SEC_TS_ACK_CALL_RELEASE) {
                	VTI("%s: Proxi Release", __func__);
                	/* Release */
					pick[0] = 1;
					#ifdef CONFIG_VIRTUAL_PROX_ON_TOUCH
						VTI("algo-prox report far event");
					#endif
        		}
			}

			if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
			                (p_event_status->status_id == SEC_TS_VENDOR_ACK_FOD_EVENT)) {
			        if (p_event_status->status_data_1 == SEC_TS_ACK_FOD_DETECT) {
			                VTI("%s: FOD Detect", __func__);
			        }
			        else if (p_event_status->status_data_1 == SEC_TS_ACK_FOD_RELEASE) {
			                VTI("%s: FOD Release", __func__);
			        }
			}

			if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
			                (p_event_status->status_id == SEC_TS_VENDOR_ACK_LONGPRESS_EVENT)) {
			        if (p_event_status->status_data_1 == SEC_TS_ACK_INSIDE_SLIDE_DOWN) {
						if (p_event_status->status_data_2 == SEC_TS_ACK_INSIDE_SLIDE_LEFT) {
			                VTI("%s: Inside Slide Left Down", __func__);
							ts->inside_slide = 1;
						} else if (p_event_status->status_data_2 == SEC_TS_ACK_INSIDE_SLIDE_RIGHT) {
			                VTI("%s: Inside Slide Right Down", __func__);
							ts->inside_slide = 2;
						}
			        }
			        else if (p_event_status->status_data_1 == SEC_TS_ACK_INSIDE_SLIDE_RELEASE) {
						if (ts->inside_slide == 1) {
			                VTI("%s: Inside Slide Left Up", __func__);
						} else if (ts->inside_slide == 2) {
			                VTI("%s: Inside Slide Right Up", __func__);
		
						}
						ts->inside_slide = 0;
			        }
			}
							
			if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
							(p_event_status->status_id == SEC_TS_VENDOR_ACK_QUIT_ACTIVE_EVENT)) {
					VTI("%s: QUIT active mode", __func__);
					sec_ts_set_active_mode(ts, 0);

			}
			break;

		case SEC_TS_COORDINATE_EVENT:
			if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
				VTE("%s: device is closed\n", __func__);
				break;
			}
			p_event_coord = (struct sec_ts_event_coordinate *)event_buff;

			t_id = (p_event_coord->tid - 1);

			if (t_id < MAX_SUPPORT_TOUCH_COUNT + MAX_SUPPORT_HOVER_COUNT) {
				pre_ttype = ts->coord[t_id].ttype;
				ts->coord[t_id].id = t_id;
				ts->coord[t_id].action = p_event_coord->tchsta;
				ts->coord[t_id].x = (p_event_coord->x_11_4 << 4) | (p_event_coord->x_3_0);
				ts->coord[t_id].y = (p_event_coord->y_11_4 << 4) | (p_event_coord->y_3_0);
				ts->coord[t_id].z = p_event_coord->z & 0x3F;
				ts->coord[t_id].ttype = p_event_coord->ttype_3_2 << 2 | p_event_coord->ttype_1_0 << 0;
				ts->coord[t_id].major = p_event_coord->major;
				ts->coord[t_id].minor = p_event_coord->minor;

				if (!ts->coord[t_id].palm && (ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_PALM))
					ts->coord[t_id].palm_count++;

				ts->coord[t_id].palm = (ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_PALM);
				ts->coord[t_id].left_event = p_event_coord->left_event;

				if (ts->coord[t_id].z <= 0)
					ts->coord[t_id].z = 1;

				if ((ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_NORMAL)
						|| (ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_PALM)
						|| (ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_WET)
						|| (ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_GLOVE)) {

					if (ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_RELEASE) {
						if (ts->touch_count > 0)
							ts->touch_count--;
						sec_report_point(UP, ts->coord[t_id].x, ts->coord[t_id].y, ts->coord[t_id].id, ts->coord[t_id].major);
						if (ts->touch_count == 0) {
							ts->check_multi = 0;
						}
						
						ts->coord[t_id].action = SEC_TS_COORDINATE_ACTION_NONE;
						ts->coord[t_id].mcount = 0;
						ts->coord[t_id].palm_count = 0;


					} else if (ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_PRESS) {
						do_gettimeofday(&ts->time_pressed[t_id]);

						ts->touch_count++;
						ts->all_finger_count++;

						ts->max_z_value = max((unsigned int)ts->coord[t_id].z, ts->max_z_value);
						ts->min_z_value = min((unsigned int)ts->coord[t_id].z, ts->min_z_value);
						ts->sum_z_value += (unsigned int)ts->coord[t_id].z;
						if ((ts->touch_count > 4) && (ts->check_multi == 0)) {
							ts->check_multi = 1;
							ts->multi_count++;
						}
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
					
						sec_report_point(DOWN, ts->coord[t_id].x, ts->coord[t_id].y, ts->coord[t_id].id, ts->coord[t_id].major);
						
#else
						sec_report_point(DOWN, ts->coord[t_id].x, ts->coord[t_id].y, ts->coord[t_id].id, ts->coord[t_id].major);
						
#endif
					} else if (ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_MOVE) {
						
						sec_report_point(DOWN, ts->coord[t_id].x, ts->coord[t_id].y, ts->coord[t_id].id, ts->coord[t_id].major);
						if ((ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_GLOVE) && !ts->touchkey_glove_mode_status) {
							ts->touchkey_glove_mode_status = true;
						} else if ((ts->coord[t_id].ttype != SEC_TS_TOUCHTYPE_GLOVE) && ts->touchkey_glove_mode_status) {
							ts->touchkey_glove_mode_status = false;
						}
						ts->coord[t_id].mcount++;
					} else {
						VTD("%s: do not support coordinate action(%d)\n", __func__, ts->coord[t_id].action);
					}

					if ((ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_PRESS)
							|| (ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_MOVE)) {

						if (ts->coord[t_id].ttype != pre_ttype) {
							VTI("%s : tID:%d ttype(%x->%x)\n",
									__func__, ts->coord[t_id].id,
									pre_ttype, ts->coord[t_id].ttype);
						}
					}

				} else {
					VTD("%s: do not support coordinate type(%d)\n", __func__, ts->coord[t_id].ttype);
				}
			} else {
				VTE("%s: tid(%d) is out of range\n", __func__, t_id);
			}
			break;

		default:
			VTE("%s: unknown event %x %x %x %x %x %x\n", __func__,
					event_buff[0], event_buff[1], event_buff[2],
					event_buff[3], event_buff[4], event_buff[5]);
			break;
		}

		curr_pos++;
		remain_event_count--;
	} while (remain_event_count >= 0);

	
}

static irqreturn_t sec_ts_irq_thread(int irq, void *ptr)
{
	struct sec_ts_data *ts = (struct sec_ts_data *)ptr;
	mutex_lock(&ts->eventlock);

	sec_ts_read_event(ts);

	mutex_unlock(&ts->eventlock);

	return IRQ_HANDLED;
}

int get_tsp_status(void)
{
	return 0;
}
EXPORT_SYMBOL(get_tsp_status);

int sec_ts_set_charger(struct sec_ts_data *ts, bool enable)
{
	int ret;
	u8 noise_mode_on[] = {0x02};
	u8 noise_mode_off[] = {0x01};

	if (enable) {
		VTI("sec_ts_set_charger : ==charger CONNECTED!!==\n");
		ret = ts->sec_ts_i2c_write(ts, SET_TS_CMD_SET_CHARGER_MODE, noise_mode_on, sizeof(noise_mode_on));
		if (ret < 0)
			VTE("sec_ts_set_charger: fail to write CHARGER_MODE_ON\n");
	} else {
		VTI("sec_ts_set_charger : charger DISCONNECTED!!\n");
		ret = ts->sec_ts_i2c_write(ts, SET_TS_CMD_SET_CHARGER_MODE, noise_mode_off, sizeof(noise_mode_off));
		if (ret < 0)
			VTE("sec_ts_set_charger: fail to write CHARGER_MODE_OFF\n");
	}

	return ret;
}
EXPORT_SYMBOL(sec_ts_set_charger);

int sec_ts_glove_mode_enables(struct sec_ts_data *ts, int mode)
{
	int ret;

	if (mode)
		ts->touch_functions = (ts->touch_functions | SEC_TS_BIT_SETFUNC_GLOVE | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);
	else
		ts->touch_functions = ((ts->touch_functions & (~SEC_TS_BIT_SETFUNC_GLOVE)) | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTE("%s: pwr off, glove:%d, status:%x\n", __func__,
				mode, ts->touch_functions);
		goto glove_enable_err;
	}

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, (u8 *)&ts->touch_functions, 2);
	if (ret < 0) {
		VTE("%s: Failed to send command", __func__);
		goto glove_enable_err;
	}

	VTI("%s: glove:%d, status:%x\n", __func__,
			mode, ts->touch_functions);

	return 0;

glove_enable_err:
	return -EIO;
}
EXPORT_SYMBOL(sec_ts_glove_mode_enables);

int sec_ts_set_cover_type(struct sec_ts_data *ts, bool enable)
{
	int ret;

	VTI("%s: %d\n", __func__, ts->cover_type);


	switch (ts->cover_type) {
	case SEC_TS_VIEW_WIRELESS:
	case SEC_TS_VIEW_COVER:
	case SEC_TS_VIEW_WALLET:
	case SEC_TS_FLIP_WALLET:
	case SEC_TS_LED_COVER:
	case SEC_TS_MONTBLANC_COVER:
	case SEC_TS_CLEAR_FLIP_COVER:
	case SEC_TS_QWERTY_KEYBOARD_EUR:
	case SEC_TS_QWERTY_KEYBOARD_KOR:
		ts->cover_cmd = (u8)ts->cover_type;
		break;
	case SEC_TS_CHARGER_COVER:
	case SEC_TS_COVER_NOTHING1:
	case SEC_TS_COVER_NOTHING2:
	default:
		ts->cover_cmd = 0;
		VTE("%s: not chage touch state, %d\n",
				__func__, ts->cover_type);
		break;
	}

	if (enable)
		ts->touch_functions = (ts->touch_functions | SEC_TS_BIT_SETFUNC_COVER | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);
	else
		ts->touch_functions = ((ts->touch_functions & (~SEC_TS_BIT_SETFUNC_COVER)) | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTE("%s: pwr off, close:%d, status:%x\n", __func__,
				enable, ts->touch_functions);
		goto cover_enable_err;
	}

	if (enable) {
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_COVERTYPE, &ts->cover_cmd, 1);
		if (ret < 0) {
			VTE("%s: Failed to send covertype command: %d", __func__, ts->cover_cmd);
			goto cover_enable_err;
		}
	}

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, (u8 *)&(ts->touch_functions), 2);
	if (ret < 0) {
		VTE("%s: Failed to send command", __func__);
		goto cover_enable_err;
	}

	VTI("%s: close:%d, status:%x\n", __func__,
			enable, ts->touch_functions);

	return 0;

cover_enable_err:
	return -EIO;


}
EXPORT_SYMBOL(sec_ts_set_cover_type);

void sec_ts_set_grip_type(struct sec_ts_data *ts, u8 set_type)
{
	return;
# if 0
	u8 mode = G_NONE;

	VTI("%s: re-init grip(%d), edh:%d, edg:%d, lan:%d\n", __func__,
			set_type, ts->grip_edgehandler_direction, ts->grip_edge_range, ts->grip_landscape_mode);

	/* edge handler */
	if (ts->grip_edgehandler_direction != 0)
		mode |= G_SET_EDGE_HANDLER;

	if (set_type == GRIP_ALL_DATA) {
		/* edge */
		if (ts->grip_edge_range != 60)
			mode |= G_SET_EDGE_ZONE;

		/* dead zone */
		if (ts->grip_landscape_mode == 1)	/* default 0 mode, 32 */
			mode |= G_SET_LANDSCAPE_MODE;
		else
			mode |= G_SET_NORMAL_MODE;
	}

	if (mode)
		set_grip_data_to_ic(ts, mode);
#endif
}




int bbk_slsi_expect_mode (void) {
	return 0;
}
EXPORT_SYMBOL(bbk_slsi_expect_mode);


/*================BBK*===========================*/


int sec_ts_init_regulator(struct i2c_client *client)
{
	struct sec_ts_data *ts = i2c_get_clientdata(client);
	struct sec_ts_plat_data *pdata = ts->plat_data;
	int retval;
	if (gpio_is_valid(pdata->iovcc_gpio)) {    //1.8v
		retval = devm_gpio_request(&client->dev ,pdata->iovcc_gpio, "sec, tsp_iovcc");
		if (retval){
			VTE("fail to request iovcc gpio !!!");
		}
	} else {
		pdata->regulator_dvdd = regulator_get(&client->dev, "vcc_i2c");
		if (IS_ERR_OR_NULL(pdata->regulator_dvdd)) {
			VTE("%s: Failed to get dvdd ONE regulator.\n", __func__);
			pdata->regulator_dvdd = regulator_get(&client->dev, "vcc_i2c_s");
			if (IS_ERR_OR_NULL(pdata->regulator_dvdd)) {
				VTE("%s: Failed to get dvdd TWO regulator.\n", __func__);
				retval = PTR_ERR(pdata->regulator_dvdd);
				goto err_dvdd;
			}
		}

		retval = regulator_set_voltage(pdata->regulator_dvdd, 1800000,
								1800000);
		if (retval) {
			VTI("regulator set_vtg:vcc_i2c failed retval = %d", retval);
		}
	}
	
	if (gpio_is_valid(pdata->power_gpio)) {    //3v
		retval = devm_gpio_request(&client->dev, pdata->power_gpio, "sec, tsp_power");
		if (retval){
			VTE("fail to request power gpio !!!");
		}
	} else {
		//regulator_avdd = regulator_get(NULL, pdata->regulator_avdd);
		pdata->regulator_avdd = regulator_get(&client->dev, "avdd_pwr");
		if (IS_ERR_OR_NULL(pdata->regulator_avdd)) {
			VTE("%s: Failed to get avdd regulator.\n", __func__);
			retval = PTR_ERR(pdata->regulator_avdd);
			goto err_avdd;
		}
		retval = regulator_set_voltage(pdata->regulator_avdd, 3000000,
								3000000);
		if (retval) {
			VTI("regulator set_vtg:avdd_pwr failed retval = %d", retval);
		}
	}

	VTI("INIT REGULATOR");

	return retval;

err_avdd:
	if (pdata->regulator_avdd) {
		regulator_put(pdata->regulator_avdd);
		pdata->regulator_avdd = NULL;
	}
err_dvdd:
	if (pdata->regulator_dvdd) {
		regulator_put(pdata->regulator_dvdd);
		pdata->regulator_dvdd = NULL;
	}
	return retval;
}

int sec_ts_power(void *data, bool on)
{
	struct sec_ts_data *ts = (struct sec_ts_data *)data;
	struct sec_ts_plat_data *pdata = ts->plat_data;
	static bool enabled;
	int ret = 0;

	if (enabled == on) {
		VTI("enabled == on");
		return ret;
	}

	if (on) {
		VTI("regulator_enable");
		if (gpio_is_valid(pdata->iovcc_gpio)) {
			ret = gpio_direction_output(pdata->iovcc_gpio, 1);
			if(ret) {
				VTE("fail to set iovcc 1.8v !!!");
			}
		} else {
			ret = regulator_set_voltage(pdata->regulator_dvdd, 1800000, 1800000);
			if (ret) {
				VTI("regulator set_vtg:vcc_i2c failed retval = %d", ret);
			}
			regulator_set_load(pdata->regulator_dvdd, 30000);
			ret = regulator_enable(pdata->regulator_dvdd);
			if (ret) {
				VTE("%s: Failed to enable dvdd: %d\n", __func__, ret);
				//goto out;
			}
		}

		sec_ts_delay(1);

		if(gpio_is_valid(pdata->power_gpio)) {
			ret = gpio_direction_output(pdata->power_gpio, 1);
			if(ret) {
				VTE("fail to set avdd 3.0v !!!");
			}
		} else {
			ret = regulator_set_voltage(pdata->regulator_avdd, 3000000, 3000000);
			if (ret) {
				VTI("regulator set_vtg:avdd failed retval = %d", ret);
			}
			regulator_set_load(pdata->regulator_avdd, 30000);
			ret = regulator_enable(pdata->regulator_avdd);
			if (ret) {
				VTE("%s: Failed to enable avdd: %d\n", __func__, ret);
				//goto out;
			}
		}
		
		gpio_set_value(pdata->reset_gpio, 1);
		msleep(5);
	} else {
		gpio_set_value(pdata->reset_gpio, 0);
		msleep(5);
		if (gpio_is_valid(pdata->power_gpio)) {
			ret = gpio_direction_output(pdata->power_gpio, 0);
			if(ret) {
				VTE("fail to shutdown power 3.0v !!!");
			}
		} else {
			ret = regulator_disable(pdata->regulator_avdd);
			if (ret) {
				VTE("fail to disable avdd !!!");
			}
			if (regulator_count_voltages(pdata->regulator_avdd) > 0) {
				regulator_set_voltage(pdata->regulator_avdd, 0, 3000000);
				regulator_set_load(pdata->regulator_avdd, 0);
			}
		}
		sec_ts_delay(4);
		if (gpio_is_valid(pdata->iovcc_gpio)) {
			ret = gpio_direction_output(pdata->iovcc_gpio, 0);
			if(ret) {
				VTE("fail to shutdown iovcc 1.8v !!!");
			}
		} else {
			ret = regulator_disable(pdata->regulator_dvdd);
			if (ret) {
				VTE("fail to disable dvdd !!!");
			}
			if (regulator_count_voltages(pdata->regulator_dvdd) > 0) {
				 regulator_set_load(pdata->regulator_dvdd, 0);
				 regulator_set_voltage(pdata->regulator_dvdd, 0, 1800000);
			}
		}
	}

	enabled = on;

//out:
	if (!gpio_is_valid(pdata->power_gpio)) {
		VTI("%s: avdd:%s\n", on ? "on" : "off",
			regulator_is_enabled(pdata->regulator_avdd) ? "on" : "off");
	}
	if (!gpio_is_valid(pdata->iovcc_gpio)) {
		VTI("%s: dvdd:%s\n", on ? "on" : "off",
			regulator_is_enabled(pdata->regulator_dvdd) ? "on" : "off");
	}
	return ret;

}
#if 0
static int vts_parse_incell_panel(struct device_node *np)
{
	int i;
	int count;
	struct device_node *node;
	struct drm_panel *panel;

	g_ts_data->active_panel_v2 = NULL;

	VTI("vts_parse_incell_panel in !!!");
	count = of_count_phandle_with_args(np, "vts-incell-panel", NULL);
	if (count <= 0){
		return 0;
	}

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "vts-incell-panel", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			g_ts_data->active_panel_v2 = panel;
			VTI("find vts-incell-panel success");
			return 0;
		}
	}
	VTI("find vts-incell-panel FAIL");

	return -ENODEV;
}
#endif
static int sec_ts_parse_dt(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sec_ts_plat_data *pdata = dev->platform_data;
	struct device_node *np = dev->of_node;
	int ret = 0;	
	struct property *prop;

	prop = of_find_property(np, "samsung,power-gpio", NULL);
	if (prop && prop->length) {
		pdata->power_gpio = of_get_named_gpio_flags(np,
			"samsung,power-gpio", 0, NULL);
		VTI("dt find:samsung,power-gpio = %d", pdata->power_gpio);
	} else {
		pdata->power_gpio = -1;
	}

	prop = of_find_property(np, "samsung,iovcc-gpio", NULL);
	if (prop && prop->length) {
		pdata->iovcc_gpio = of_get_named_gpio_flags(np,
			"samsung,iovcc-gpio", 0, NULL);
		VTI("dt find:samsung,iovcc-gpio = %d", pdata->iovcc_gpio);
	} else {
		pdata->iovcc_gpio = -1;
	}

	pdata->irq_gpio = of_get_named_gpio_flags(np,
			"samsung,irq-gpio", 0,
			(enum of_gpio_flags *)&pdata->irq_type);

	if (gpio_is_valid(pdata->irq_gpio)) {
		ret = gpio_request_one(pdata->irq_gpio, GPIOF_DIR_IN, "sec,tsp_int");
		if (ret) {
			VTE("%s: Unable to request tsp_int [%d]\n", __func__, pdata->irq_gpio);
			return -EINVAL;
		}
	} else {
		VTE("%s: Failed to get irq gpio\n", __func__);
		return -EINVAL;
	}
	
	VTI("%s: irq_type property:%X, %d\n", __func__,
					pdata->irq_type, pdata->irq_type);

	client->irq = gpio_to_irq(pdata->irq_gpio);
	VTI("%s: gpio : %d IRQ : %d", __func__, pdata->irq_gpio, client->irq);

	prop = of_find_property(np, "samsung,reset-gpio", NULL);
	if (prop && prop->length) {
		pdata->reset_gpio = of_get_named_gpio_flags(np,
				"samsung,reset-gpio", 0, NULL);
		ret = gpio_request_one(pdata->reset_gpio, GPIOF_DIR_OUT, "sec,tsp_reset");
		VTI("%s: reset pin property %X, %d\n",
			__func__, pdata->reset_gpio, pdata->reset_gpio);
	} else {
		pdata->reset_gpio = -1;
		VTE("%s: parsing reset pin failed\n", __func__);
	}

	pdata->i2c_burstmax = 32;

	pdata->power = sec_ts_power;

	pdata->always_lpmode = 0;
	pdata->bringup = 0;
	pdata->mis_cal_check = 0;
    pdata->firmware_name = NULL;
	pdata->project_name = NULL;
	pdata->model_name = NULL;

	pdata->tsp_icid = 0;
	pdata->tsp_id = 0;
	pdata->tsp_vsync = 0;

	pdata->regulator_boot_on = 0;
	pdata->support_sidegesture = 0;
	pdata->support_dex = 1;

	VTI("i2c buffer limit: %d, bringup:%d, FW:%s(%d), id:%d, mis_cal:%d dex:%d, gesture:%d\n",
		pdata->i2c_burstmax, pdata->bringup, pdata->firmware_name,
			pdata->tsp_id, pdata->tsp_icid, pdata->mis_cal_check, pdata->support_dex, pdata->support_sidegesture);

	return ret;
}
static int sec_ts_parse_dt_property()
{
	//struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	struct device *dev = &g_ts_data->client->dev;
	struct device_node *np = dev->of_node;
	int i;

	parse_property_u32_with_default(np, "nail,edge_reject_height_max", (u32 *)&g_ts_data->edge_para[EDGE_NAIL].reject_height_max, 500);
	parse_property_u32_with_default(np, "nail,edge_reject_top", (u32 *)&g_ts_data->edge_para[EDGE_NAIL].reject_top, 300);
	parse_property_u32_with_default(np, "nail,edge_reject_buttom", (u32 *)&g_ts_data->edge_para[EDGE_NAIL].reject_buttom, 300);
	parse_property_u32_with_default(np, "vk,edge_reject_height_max", (u32 *)&g_ts_data->edge_para[EDGE_VIRTUAL_KEY].reject_height_max, 500);
	parse_property_u32_with_default(np, "vk,edge_reject_height_max", (u32 *)&g_ts_data->edge_para[EDGE_VIRTUAL_KEY_SINGLE].reject_height_max, 500);
	parse_property_u32_with_default(np, "vk,edge_reject_height_max", (u32 *)&g_ts_data->edge_para[EDGE_VIRTUAL_KEY_DOUBLE].reject_height_max, 500);
	parse_property_u32_with_default(np, "vk,edge_reject_top", (u32 *)&g_ts_data->edge_para[EDGE_VIRTUAL_KEY].reject_top, 300);
	parse_property_u32_with_default(np, "vk,edge_reject_top", (u32 *)&g_ts_data->edge_para[EDGE_VIRTUAL_KEY_SINGLE].reject_top, 300);
	parse_property_u32_with_default(np, "vk,edge_reject_top", (u32 *)&g_ts_data->edge_para[EDGE_VIRTUAL_KEY_DOUBLE].reject_top, 300);
	parse_property_u32_with_default(np, "vk,edge_reject_buttom", (u32 *)&g_ts_data->edge_para[EDGE_VIRTUAL_KEY].reject_buttom, 750);
	parse_property_u32_with_default(np, "vk,edge_reject_buttom", (u32 *)&g_ts_data->edge_para[EDGE_VIRTUAL_KEY_SINGLE].reject_buttom, 750);
	parse_property_u32_with_default(np, "vk,edge_reject_buttom", (u32 *)&g_ts_data->edge_para[EDGE_VIRTUAL_KEY_DOUBLE].reject_buttom, 750);

	for (i = 1; i < EDGE_PARA_NUM; i++) {
		VTI("edge_para_%d, height = %d, top = %d, buttom = %d", i, g_ts_data->edge_para[i].reject_height_max, g_ts_data->edge_para[i].reject_top, g_ts_data->edge_para[i].reject_buttom);
	}

	parse_property_u32_with_default(np, "virtual_gamekey_width", (u32 *)&g_ts_data->gamekey_size[0], 300);
	parse_property_u32_with_default(np, "virtual_gamekey_height", (u32 *)&g_ts_data->gamekey_size[1], 36);
	parse_property_u8_with_default(np, "virtual_gamekey_up_deadzone", (u8 *)&g_ts_data->vgk_deadzone[0], 0);
	parse_property_u8_with_default(np, "virtual_gamekey_down_deadzone", (u8 *)&g_ts_data->vgk_deadzone[1], 0);
#if 0
	vts_parse_incell_panel(np);
#endif
	return 0;
}

int sec_ts_read_information(struct sec_ts_data *ts)
{
	unsigned char data[13] = { 0 };
	int ret;

	memset(data, 0x0, 3);
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_ID, data, 3);
	if (ret < 0) {
		VTE("%s: failed to read device id(%d)\n",
				__func__, ret);
		return ret;
	}

	VTI("%s: %X, %X, %X\n",
			__func__, data[0], data[1], data[2]);
	memset(data, 0x0, 11);
	ret = sec_ts_i2c_read(ts,  SEC_TS_READ_PANEL_INFO, data, 11);
	if (ret < 0) {
		VTE("%s: failed to read sub id(%d)\n",
				__func__, ret);
		return ret;
	}

	VTI("%s: nTX:%X, nRX:%X, rY:%d, rX:%d\n",
			__func__, data[8], data[9],
			(data[2] << 8) | data[3], (data[0] << 8) | data[1]);

	/* Set X,Y Resolution from IC information. */
	if (((data[0] << 8) | data[1]) > 0)
		ts->plat_data->max_x = ((data[0] << 8) | data[1]) - 1;

	if (((data[2] << 8) | data[3]) > 0)
		ts->plat_data->max_y = ((data[2] << 8) | data[3]) - 1;

	ts->tx_count = data[8];
	ts->rx_count = data[9];

	data[0] = 0;
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS, data, 1);
	if (ret < 0) {
		VTE("%s: failed to read sub id(%d)\n",
				__func__, ret);
		return ret;
	}

	VTI("%s: STATUS : %X\n",
			__func__, data[0]);

	memset(data, 0x0, 4);
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_TS_STATUS, data, 4);
	if (ret < 0) {
		VTE("%s: failed to read sub id(%d)\n",
				__func__, ret);
		return ret;
	}

	VTI("%s: TOUCH STATUS : %02X, %02X, %02X, %02X\n",
			__func__, data[0], data[1], data[2], data[3]);
	ret = sec_ts_i2c_read(ts, SEC_TS_CMD_SET_TOUCHFUNCTION,  (u8 *)&(ts->touch_functions), 2);
	if (ret < 0) {
		VTE("%s: failed to read touch functions(%d)\n",
				__func__, ret);
		return ret;
	}

	VTI("%s: Functions : %02X\n",
			__func__, ts->touch_functions);

	return ret;
}

static void sec_ts_set_input_prop(struct sec_ts_data *ts, struct input_dev *dev, u8 propbit)
{
	static char sec_ts_phys[64] = { 0 };

	snprintf(sec_ts_phys, sizeof(sec_ts_phys), "%s/input1",
			dev->name);
	dev->phys = sec_ts_phys;
	dev->id.bustype = BUS_I2C;
	dev->dev.parent = &ts->client->dev;

	set_bit(EV_SYN, dev->evbit);
	set_bit(EV_KEY, dev->evbit);
	set_bit(EV_ABS, dev->evbit);
	set_bit(EV_SW, dev->evbit);
	set_bit(BTN_TOUCH, dev->keybit);
	set_bit(BTN_TOOL_FINGER, dev->keybit);
	//set_bit(KEY_BLACK_UI_GESTURE, dev->keybit);
#ifdef SEC_TS_SUPPORT_TOUCH_KEY
	if (ts->plat_data->support_mskey) {
		int i;

		for (i = 0 ; i < ts->plat_data->num_touchkey ; i++)
			set_bit(ts->plat_data->touchkey[i].keycode, dev->keybit);

		set_bit(EV_LED, dev->evbit);
		set_bit(LED_MISC, dev->ledbit);
	}
#endif
	if (ts->plat_data->support_sidegesture) {
		//set_bit(KEY_SIDE_GESTURE, dev->keybit);
		//set_bit(KEY_SIDE_GESTURE_RIGHT, dev->keybit);
		//set_bit(KEY_SIDE_GESTURE_LEFT, dev->keybit);
	}
	set_bit(propbit, dev->propbit);
	set_bit(KEY_HOMEPAGE, dev->keybit);

	//input_set_capability(dev, EV_SW, SW_GLOVE);

	input_set_abs_params(dev, ABS_MT_POSITION_X, 0, ts->plat_data->max_x, 0, 0);
	input_set_abs_params(dev, ABS_MT_POSITION_Y, 0, ts->plat_data->max_y, 0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	//input_set_abs_params(dev, ABS_MT_CUSTOM, 0, 0xFFFFFFFF, 0, 0);
	if (ts->plat_data->support_mt_pressure)
		input_set_abs_params(dev, ABS_MT_PRESSURE, 0, 255, 0, 0);

	if (propbit == INPUT_PROP_POINTER)
		input_mt_init_slots(dev, MAX_SUPPORT_TOUCH_COUNT, INPUT_MT_POINTER);
	else
		input_mt_init_slots(dev, MAX_SUPPORT_TOUCH_COUNT, INPUT_MT_DIRECT);

	input_set_drvdata(dev, ts);
}

static vts_lcm_state_process(unsigned long event, int blank)
{
	VTI("event:%ld, blank:%d", event, blank);

	if (event == DRM_PANEL_EVENT_BLANK && blank == DRM_PANEL_BLANK_UNBLANK) {
		//resume
		VTI("/---------------resume-------------/");
	} else if (event == DRM_PANEL_EARLY_EVENT_BLANK && blank == DRM_PANEL_BLANK_UNBLANK) {
		//early_resume
		VTI("/---------------early_resume-------------/");
	} else if (event == DRM_PANEL_EVENT_BLANK && blank == DRM_PANEL_BLANK_POWERDOWN) {
		//suspend
		VTI("/---------------suspend-------------/");
	} else if (event == DRM_PANEL_EARLY_EVENT_BLANK && blank == DRM_PANEL_BLANK_POWERDOWN){
		//early_suspend
		VTI("/---------------early_suspend-------------/");
	}
	return 0;
}

static int vts_lcm_state_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	int blank;
	struct fb_event *evdata = data;
	VTI("/---------------callback-------------/");
	if(evdata == NULL){
		VTE("evdata is NULL");
		return 0;
	}
	if(evdata->data == NULL){
		VTE("evdata->data is NULL");
		return 0;	
	}
	if(IS_ERR(evdata) || IS_ERR(evdata->data)){
		VTE("evdata or evdata->data is ERR point");
		return 0;	
	}
	blank = *(int *)evdata->data;
	return vts_lcm_state_process(event, blank);
}

int sec_ts_enable_irq_wake(struct sec_ts_data *ts, u8 enable)
{
	VTI("irq wake is %s", enable? "enable" : "disable");
	if (enable) {
		if(ts->irq_wake_flag == 1) 
			return 0;
		else {
			if (device_may_wakeup(&ts->client->dev)) {
				enable_irq_wake(ts->client->irq);
				ts->irq_wake_flag = 1;
			}
		}
		
	} else {
		if (ts->irq_wake_flag == 0) {
			return 0;
		} else { 
			ts->irq_wake_flag = 0;
			if (device_may_wakeup(&ts->client->dev)) 
				disable_irq_wake(ts->client->irq);
		}
	}

	return 0;
}


enum mode_state {
	SUSPEND = 0,
	RESUME
};
static int last_state = RESUME;
int bbk_slsi_mode_change(int which)
{
	struct sec_ts_data *ts = g_ts_data;
	int ret = 0;
	unsigned char read_result[4] = {0};

	if (which == last_state) {
		VTE("%s: No mode change (last_state:%d, which:%d)",
				__func__, last_state, which);
		return -1;
	}

	mutex_lock(&ts->modechange);

	switch(which) {
		case RESUME:
			VTI("change to normal mode");

			sec_ts_enable_irq_wake(ts, false);
			ret = sec_ts_start_device(ts, true);
			if (ret < 0)
					VTE("Failed to start device");

			VTI("change to normal mode end");
			ret = sec_ts_i2c_read(ts, SEC_TS_READ_TS_STATUS, read_result, 4);
			if (ret < 0) {
				VTI("read data err");
			}
			VTI("read status:0x%x, 0x%x, 0x%x, 0x%x", read_result[0], read_result[1], read_result[2], read_result[3]);
			break;
		case SUSPEND:
			VTI("change to sleep mode");
			sec_ts_enable_irq_wake(ts, false);
			ret = sec_ts_stop_device(ts, true);
			VTI("change to sleep mode end");
			break;

		default : break;
	}

	if (which == RESUME || which == SUSPEND)
		last_state = which;

	mutex_unlock(&ts->modechange);
	if (ret >= 0)
		return 0;
	else
		return ret;
}


static ssize_t dclick_lcd_state_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VTI("invalide number of parameters passed.");
		return -EINVAL;
	}

	VTI("parameter is %d", val);
	if (val == 0) {
		bbk_slsi_mode_change(SUSPEND);
	} else if(val == 1) {
		bbk_slsi_mode_change(RESUME);
	}
	g_ts_data->lcd_state = val;
	return count;
}

static ssize_t touchscreen_dclick_lcd_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	
	return snprintf(buf, 255, "has_lcd_shutoff = %d, 0--lcd on, 1---lcd off\n", g_ts_data->lcd_state);
}

static DEVICE_ATTR(dclick_lcd_state, S_IRUGO | S_IWUSR,
	touchscreen_dclick_lcd_state_show, dclick_lcd_state_store);

static struct attribute *touchscreen_sys_attrs[] = {
	&dev_attr_dclick_lcd_state.attr,
	NULL,
};

static const struct attribute_group touchscreen_sys_attrs_group = {
	.attrs = touchscreen_sys_attrs,
};

static int sec_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sec_ts_data *ts;
	struct sec_ts_plat_data *pdata;
	//struct vts_device *vtsdev = NULL;
	int ret = 0;
	bool force_update = false;
	bool valid_firmware_integrity = false;
	unsigned char data[5] = { 0 };
	unsigned char deviceID[5] = { 0 };
	unsigned char result = 0;
	struct kobject *kobj;

	kobj = kobject_create_and_add("touchscreen", NULL);
	if (kobj == NULL) {
		ret = -ENOMEM;
		VTE("--------------------------------fail to add object!");
	}
//	int i;

	VTI("%s\n", __func__);

	/* parse dt */
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct sec_ts_plat_data), GFP_KERNEL);

		if (!pdata) {
			VTE("%s: Failed to allocate platform data\n", __func__);
			goto error_allocate_pdata;
		}
		client->dev.platform_data = pdata;
		ret = sec_ts_parse_dt(client);
		if (ret) {
			VTE("%s: Failed to parse dt\n", __func__);
			goto error_allocate_mem;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			VTE("%s: No platform data found\n", __func__);
			goto error_allocate_pdata;
		}
	}

	if (!pdata->power) {
		VTE("%s: No power contorl found\n", __func__);
		goto error_allocate_mem;
	}

	ts = kzalloc(sizeof(struct sec_ts_data), GFP_KERNEL);
	if (!ts)
		goto error_allocate_mem;

	ts->client = client;
	ts->plat_data = pdata;
	ts->flash_page_size = SEC_TS_FW_BLK_SIZE_DEFAULT;
	ts->sec_ts_i2c_read = sec_ts_i2c_read;
	ts->sec_ts_i2c_write = sec_ts_i2c_write;
	ts->sec_ts_i2c_write_burst = sec_ts_i2c_write_burst;
	ts->sec_ts_i2c_read_bulk = sec_ts_i2c_read_bulk;
	ts->i2c_burstmax = pdata->i2c_burstmax;
#ifdef USE_POWER_RESET_WORK
	INIT_DELAYED_WORK(&ts->reset_work, sec_ts_reset_work);
	wakeup_source_init(&ts->reset_wakelock, "tp_reset_work");
#endif
	//INIT_DELAYED_WORK(&ts->work_read_info, sec_ts_read_info_work);
	g_ts_data = ts;
	g_ts_data->nb.notifier_call = vts_lcm_state_callback;
	//g_ts_data->touch_v1 = kzalloc(sizeof(g_ts_data->touch_v1),GFP_KERNEL);

	i2c_set_clientdata(client, ts);

	if (ts->plat_data->support_dex) {
		ts->input_dev_pad = input_allocate_device();
		if (!ts->input_dev_pad) {
			VTE("%s: allocate device err!\n", __func__);
			ret = -ENOMEM;
			goto err_allocate_input_dev_pad;
		}
	}

	ts->touch_count = 0;
	ts->max_z_value = 0;
	ts->min_z_value = 0xFFFFFFFF;
	ts->sum_z_value = 0;

	mutex_init(&ts->lock);
	mutex_init(&ts->device_mutex);
	mutex_init(&ts->i2c_mutex);
	mutex_init(&ts->eventlock);
	mutex_init(&ts->modechange);

	init_completion(&ts->resume_done);
	complete_all(&ts->resume_done);

	if (pdata->always_lpmode)
		ts->lowpower_mode |= SEC_TS_MODE_CUSTOMLIB_FORCE_KEY;
	else
		ts->lowpower_mode &= ~SEC_TS_MODE_CUSTOMLIB_FORCE_KEY;

	VTI("%s: init resource\n", __func__);

	//sec_ts_pinctrl_configure(ts, true);

	//gpio_set_value(ts->plat_data->reset_gpio, 1);
	//sec_ts_delay(5);
	ret = sysfs_create_group(kobj, &touchscreen_sys_attrs_group);
	ret = sec_ts_init_regulator(client);
	if (!ret) {
		sec_ts_power(ts, true);
		sec_ts_delay(70);
	} else {
		goto error_regulator_init;
	}
	/* power enable */
	
	VTI("%s: init power\n", __func__);
	ts->power_status = SEC_TS_STATE_POWER_ON;
	ts->external_factory = false;

	sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);
	
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_DEVICE_ID, deviceID, 5);
	if (ret < 0)
		VTE("%s: failed to read device ID(%d)\n", __func__, ret);
	else
		VTI("%s: TOUCH DEVICE ID : %02X, %02X, %02X, %02X, %02X\n", __func__,
				deviceID[0], deviceID[1], deviceID[2], deviceID[3], deviceID[4]);

	ret = sec_ts_i2c_read(ts, SEC_TS_READ_FIRMWARE_INTEGRITY, &result, 1);
	if (ret < 0) {
		VTE("%s: failed to integrity check (%d)\n", __func__, ret);
	} else {
		if (result & 0x80) {
			valid_firmware_integrity = true;
		} else if (result & 0x40) {
			valid_firmware_integrity = false;
			VTE("%s: invalid firmware (0x%x)\n", __func__, result);
		} else {
			valid_firmware_integrity = false;
			VTE("%s: invalid integrity result (0x%x)\n", __func__, result);
		}
	}

	ret = sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS, &data[0], 1);
	if (ret < 0) {
		VTE("%s: failed to read sub id(%d)\n",
				__func__, ret);
	} else {
		ret = sec_ts_i2c_read(ts, SEC_TS_READ_TS_STATUS, &data[1], 4);
		if (ret < 0) {
			VTE("%s: failed to touch status(%d)\n",
					__func__, ret);
		}
	}
	VTI("%s: TOUCH STATUS : %02X || %02X, %02X, %02X, %02X\n",
			__func__, data[0], data[1], data[2], data[3], data[4]);

	if (data[0] == SEC_TS_STATUS_BOOT_MODE)
		force_update = true;
	else if ((data[0] == SEC_TS_STATUS_APP_MODE && data[2] == TOUCH_SYSTEM_MODE_FLASH) ||
			(valid_firmware_integrity == false))
		force_update = true;
	else
		force_update = false;
	
#ifdef SEC_TS_FW_UPDATE_ON_PROBE
	sec_ts_firmware_update_on_hidden_menu(ts, 0);
	ret = sec_ts_firmware_update_on_probe(ts, force_update);
	if (ret < 0)
		VTE("calibrate on probe is failed, ret = %d", ret);
#else
	VTI("%s: fw update on probe disabled!\n", __func__);
#endif

	ret = sec_ts_read_information(ts);
	if (ret < 0) {
		VTE("%s: fail to read information 0x%x\n", __func__, ret);
		goto err_init;
	}

	ts->touch_functions |= SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC;
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, (u8 *)&ts->touch_functions, 2);
	if (ret < 0)
		VTE("%s: Failed to send touch func_mode command", __func__);

	/* Sense_on */
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0) {
		VTE("%s: fail to write Sense_on\n", __func__);
		goto err_init;
	}

	ts->pFrame = kzalloc(ts->tx_count * ts->rx_count * 2, GFP_KERNEL);
	if (!ts->pFrame) {
		ret = -ENOMEM;
		goto err_allocate_frame;
	}

	if (ts->plat_data->support_dex) {
		ts->input_dev_pad->name = "sec_touchpad";
		sec_ts_set_input_prop(ts, ts->input_dev_pad, INPUT_PROP_DIRECT);
	}
	ts->dex_name = "";

	if (ts->plat_data->support_dex) {
		ret = input_register_device(ts->input_dev_pad);
		if (ret) {
			VTE("%s: Unable to register %s input device\n", __func__, ts->input_dev_pad->name);
			goto err_input_pad_register_device;
		}
	}

	/* need remove below resource @ remove driver */
	sec_ts_raw_device_init(ts);

	device_init_wakeup(&client->dev, true);

	if (0)
		INIT_DELAYED_WORK(&ts->ghost_check, sec_ts_check_rawdata);
#if 0
	schedule_delayed_work(&ts->work_read_info, msecs_to_jiffies(50));

	schedule_delayed_work(&ts->ghost_check, msecs_to_jiffies(100));
#endif

	ts->probe_done = true;


	sec_ts_parse_dt_property();
	ret = request_threaded_irq(client->irq, NULL, sec_ts_irq_thread, IRQF_TRIGGER_LOW | IRQF_ONESHOT, "sec_irq_thread", g_ts_data);
	if (ret < 0) {
		VTE("%s: Unable to request threaded irq %d\n", __func__, ret);
		goto err_irq;
	}

	sec_ts_i2c_read(ts, SEC_TS_CMD_SELF_SENSING_MODE, &ts->self_sensing_mode, 1);
	VTI("read self sensing mode is %d", ts->self_sensing_mode);
	sec_ts_i2c_read(ts, SEC_TS_READ_CALIBRATION_REPORT, ts->ic_info, 8);
	VTI("read F1 is %d, %d, %d, %d, %d, %d, %d, %d", ts->ic_info[0], ts->ic_info[1], ts->ic_info[2], ts->ic_info[3],
		ts->ic_info[4], ts->ic_info[5], ts->ic_info[6], ts->ic_info[7]);

	VTI("%s: request_irq done = %d\n", __func__, client->irq);

	ret = fb_register_client(&g_ts_data->nb);
	if (ret) {
		VTE("fail to register notifier callback!!!");
	} else {
		VTI("success to register notifier callback !");
	}
	
	
	VTI("%s: done\n", __func__);

	return 0;

	/* need to be enabled when new goto statement is added */
#if 0
	sec_ts_fn_remove(ts);
	free_irq(client->irq, ts);
#endif
err_irq:
	//vts_unregister_driver(vtsdev);
	if (ts->plat_data->support_dex) {
		input_unregister_device(ts->input_dev_pad);
		ts->input_dev_pad = NULL;
	}
err_input_pad_register_device:
	kfree(ts->pFrame);
err_allocate_frame:
err_init:
	sec_ts_power(ts, false);
error_regulator_init:
	if ((!gpio_is_valid(pdata->power_gpio)) && pdata->regulator_avdd) {
		regulator_put(pdata->regulator_avdd);
		pdata->regulator_avdd = NULL;
	}
	if ((!gpio_is_valid(pdata->iovcc_gpio)) && pdata->regulator_dvdd) {
		regulator_put(pdata->regulator_dvdd);
		pdata->regulator_dvdd = NULL;
	}
	if (ts->plat_data->support_dex) {
		if (ts->input_dev_pad)
			input_free_device(ts->input_dev_pad);
	}
err_allocate_input_dev_pad:
#ifdef USE_POWER_RESET_WORK
	wakeup_source_trash(&ts->reset_wakelock);
#endif
	kfree(ts);
error_allocate_mem:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
	if (gpio_is_valid(pdata->tsp_id))
		gpio_free(pdata->tsp_id);
	if (gpio_is_valid(pdata->tsp_icid))
		gpio_free(pdata->tsp_icid);

error_allocate_pdata:
	/*if (vtsdev != NULL) {
		vts_device_free(vtsdev);
		vtsdev = NULL;
	}*/
	if (ret == -ECONNREFUSED)
		sec_ts_delay(100);
	ret = -ENODEV;
	VTE("%s: failed(%d)\n", __func__, ret);

	return ret;
}

void sec_ts_unlocked_release_all_finger(struct sec_ts_data *ts)
{
	int i;

	for (i = 0; i < MAX_SUPPORT_TOUCH_COUNT; i++) {
		if ((ts->coord[i].action == SEC_TS_COORDINATE_ACTION_PRESS) ||
			(ts->coord[i].action == SEC_TS_COORDINATE_ACTION_MOVE)) {

			ts->coord[i].action = SEC_TS_COORDINATE_ACTION_RELEASE;
			VTI("%s: [RA] tID:%d mc:%d tc:%d v:%02X%02X cal:%02X id(%d,%d) p:%d\n",
					__func__, i, ts->coord[i].mcount, ts->touch_count,
					ts->plat_data->img_version_of_ic[2],
					ts->plat_data->img_version_of_ic[3],
					ts->cal_status, ts->tspid_val,
					ts->tspicid_val, ts->coord[i].palm_count);

			do_gettimeofday(&ts->time_released[i]);
			
			if (ts->time_longest < (ts->time_released[i].tv_sec - ts->time_pressed[i].tv_sec))
				ts->time_longest = (ts->time_released[i].tv_sec - ts->time_pressed[i].tv_sec);
		}

		ts->coord[i].mcount = 0;
		ts->coord[i].palm_count = 0;

	}

	ts->touchkey_glove_mode_status = false;
	ts->touch_count = 0;
	ts->check_multi = 0;
}

void sec_ts_locked_release_all_finger(struct sec_ts_data *ts)
{
	int i;

	mutex_lock(&ts->eventlock);

	for (i = 0; i < MAX_SUPPORT_TOUCH_COUNT; i++) {
		if ((ts->coord[i].action == SEC_TS_COORDINATE_ACTION_PRESS) ||
			(ts->coord[i].action == SEC_TS_COORDINATE_ACTION_MOVE)) {

			ts->coord[i].action = SEC_TS_COORDINATE_ACTION_RELEASE;
			VTI("%s: [RA] tID:%d mc: %d tc:%d, v:%02X%02X, cal:%X id(%d,%d), p:%d\n",
					__func__, i, ts->coord[i].mcount, ts->touch_count,
					ts->plat_data->img_version_of_ic[2],
					ts->plat_data->img_version_of_ic[3],
					ts->cal_status, ts->tspid_val, ts->tspicid_val, ts->coord[i].palm_count);

			do_gettimeofday(&ts->time_released[i]);
			
			if (ts->time_longest < (ts->time_released[i].tv_sec - ts->time_pressed[i].tv_sec))
				ts->time_longest = (ts->time_released[i].tv_sec - ts->time_pressed[i].tv_sec);
		}

		ts->coord[i].mcount = 0;
		ts->coord[i].palm_count = 0;

	}

	ts->touchkey_glove_mode_status = false;
	ts->touch_count = 0;
	ts->check_multi = 0;

	mutex_unlock(&ts->eventlock);
}

#ifdef USE_POWER_RESET_WORK
static void sec_ts_reset_work(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
			reset_work.work);
	int ret;

	__pm_stay_awake(&ts->reset_wakelock);
	
	if (ts->reset_is_on_going) {
		VTE("%s: reset is ongoing\n", __func__);
		goto END_RELEAX;
	}

	disable_irq(ts->client->irq);
	mutex_lock(&ts->modechange);

	ts->reset_is_on_going = true;
	VTI("%s\n", __func__);

	sec_ts_stop_device(ts, true);

	sec_ts_delay(30);

	ret = sec_ts_start_device(ts, true);
	if (ret < 0) {
		VTE("%s: failed to reset, ret:%d\n", __func__, ret);
		ts->reset_is_on_going = false;
		cancel_delayed_work(&ts->reset_work);
		if (ts->reset_wakelock.active)
			__pm_relax(&ts->reset_wakelock);
		__pm_stay_awake(&ts->reset_wakelock);
		if(ts->reset_count < RESET_MAX) {
			schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
			ts->reset_count++;
		} else {
			VTE("tp reset over retry limit %d", ts->reset_count -1);
		}

		goto END_UNLOCK;
	}

	
	ts->reset_is_on_going = false;

END_UNLOCK:
	mutex_unlock(&ts->modechange);
	enable_irq(ts->client->irq);
END_RELEAX:
	if (ts->reset_wakelock.active)
		__pm_relax(&ts->reset_wakelock);
	return;
}
#endif

int sec_ts_set_lowpowermode(struct sec_ts_data *ts, u8 mode)
{
	int ret;
	int retrycnt = 0;
	//u8 data;
	char para = 0;

	VTI("%s: %s(%X)\n", __func__,
			mode == TO_LOWPOWER_MODE ? "ENTER" : "EXIT", ts->lowpower_mode);

retry_pmode:
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_POWER_MODE, &mode, 1);
	if (ret < 0) {
		VTE("%s: failed\n", __func__);
		goto i2c_error;
	}

	sec_ts_delay(50);

	ret = sec_ts_i2c_read(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
	if (ret < 0) {
		VTE("%s: read power mode failed!\n", __func__);
		goto i2c_error;
	} else {
		VTI("%s: power mode - write(%d) read(%d)\n", __func__, mode, para);
	}

	if (mode != para) {
		retrycnt++;
		if (retrycnt < 5) {
			goto retry_pmode;
		} else {
			VTE("set mode failed and retry %d times, reset IC to recovery work", retrycnt);
			if (ts->probe_done && !ts->reset_is_on_going)
				schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
		}
	}

	if (mode) {
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
		if (ret < 0) {
			VTE("%s: i2c write clear event failed\n", __func__);
			goto i2c_error;
		}
	}

	sec_ts_locked_release_all_finger(ts);

	if (device_may_wakeup(&ts->client->dev)) {
		if (mode)
			enable_irq_wake(ts->client->irq);
		else
			disable_irq_wake(ts->client->irq);
	}

	if (mode == TO_LOWPOWER_MODE)
		ts->power_status = SEC_TS_STATE_LPM;
	else
		ts->power_status = SEC_TS_STATE_POWER_ON;

i2c_error:
	VTI("%s: end %d\n", __func__, ret);

	return ret;
}

#ifdef USE_OPEN_CLOSE
static int sec_ts_input_open(struct input_dev *dev)
{
	struct sec_ts_data *ts = input_get_drvdata(dev);
	int ret;

	if (!ts->info_work_done) {
		VTE("%s not finished info work\n", __func__);
		return 0;
	}

	mutex_lock(&ts->modechange);

	ts->input_closed = false;

	VTI("%s\n", __func__);

	if (ts->power_status == SEC_TS_STATE_LPM) {
#ifdef USE_RESET_EXIT_LPM
		schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
#else
		sec_ts_set_lowpowermode(ts, TO_TOUCH_MODE);
#endif
	} else {
		ret = sec_ts_start_device(ts, true);
		if (ret < 0)
			VTE("%s: Failed to start device\n", __func__);
	}

	/* because edge and dead zone will recover soon */
	sec_ts_set_grip_type(ts, ONLY_EDGE_HANDLER);

	mutex_unlock(&ts->modechange);
	
	return 0;
}

static void sec_ts_input_close(struct input_dev *dev)
{
	struct sec_ts_data *ts = input_get_drvdata(dev);

	if (!ts->info_work_done) {
		VTE("%s not finished info work\n", __func__);
		return;
	}

	mutex_lock(&ts->modechange);

	ts->input_closed = true;

	VTI("%s\n", __func__);

#ifdef USE_POWER_RESET_WORK
	cancel_delayed_work(&ts->reset_work);
	if (ts->reset_wakelock.active)
		__pm_relax(&ts->reset_wakelock);
		
#endif

	if (ts->lowpower_mode) {
		sec_ts_set_lowpowermode(ts, TO_LOWPOWER_MODE);
	} else {
		sec_ts_stop_device(ts, true);
	}

	mutex_unlock(&ts->modechange);
}
#endif

static int sec_ts_remove(struct i2c_client *client)
{
	struct sec_ts_data *ts = i2c_get_clientdata(client);

	VTI("%s\n", __func__);

	disable_irq_nosync(ts->client->irq);
	free_irq(ts->client->irq, ts);
	VTI("%s: irq disabled\n", __func__);

#ifdef USE_POWER_RESET_WORK
	cancel_delayed_work_sync(&ts->reset_work);
	flush_delayed_work(&ts->reset_work);
	wakeup_source_trash(&ts->reset_wakelock);

	VTI("%s: flush queue\n", __func__);

#endif

	device_init_wakeup(&client->dev, false);

	ts->lowpower_mode = false;
	ts->probe_done = false;

	if (ts->plat_data->support_dex) {
		input_mt_destroy_slots(ts->input_dev_pad);
		input_unregister_device(ts->input_dev_pad);
	}

	ts->input_dev_pad = NULL;
	ts->plat_data->power(ts, false);

	kfree(ts);
	return 0;
}

static void sec_ts_shutdown(struct i2c_client *client)
{
	struct sec_ts_data *ts = i2c_get_clientdata(client);

	VTI("%s\n", __func__);

	disable_irq_nosync(ts->client->irq);
	free_irq(ts->client->irq, ts);
	VTI("%s: irq disabled\n", __func__);

#ifdef USE_POWER_RESET_WORK
	cancel_delayed_work_sync(&ts->reset_work);
	flush_delayed_work(&ts->reset_work);

	VTI("%s: flush queue\n", __func__);

#endif

	device_init_wakeup(&client->dev, false);
	//wake_lock_destroy(&ts->wakelock);

	ts->lowpower_mode = false;
	ts->probe_done = false;

	if (ts->plat_data->support_dex) {
		input_mt_destroy_slots(ts->input_dev_pad);
		input_unregister_device(ts->input_dev_pad);
	}

	ts->input_dev_pad = NULL;
	ts->plat_data->power(ts, false);
	if ((!gpio_is_valid(ts->plat_data->power_gpio)) && ts->plat_data->regulator_avdd) {
		regulator_put(ts->plat_data->regulator_avdd);
		ts->plat_data->regulator_avdd = NULL;
	}
	if ((!gpio_is_valid(ts->plat_data->iovcc_gpio)) && ts->plat_data->regulator_dvdd) {
		regulator_put(ts->plat_data->regulator_dvdd);
		ts->plat_data->regulator_dvdd = NULL;
	}
}

int sec_ts_stop_device(struct sec_ts_data *ts, bool hardware)
{
	u8 status;
	int ret = 0;
	
	VTI("stop by %s\n", hardware == true ? "hardware" : "software");

	mutex_lock(&ts->device_mutex);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTE("%s: already power off\n", __func__);
		goto out;
	}

	disable_irq(ts->client->irq);

	sec_ts_locked_release_all_finger(ts);

	if (hardware) {
		ts->plat_data->power(ts, false);
	} else {
		status = 1;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_OFF, &status, 1);
		if (ret < 0) {
			VTE("%s: failed\n", __func__);
			goto out;
		}
	}

	ts->power_status = SEC_TS_STATE_POWER_OFF;

	if (ts->plat_data->enable_sync)
		ts->plat_data->enable_sync(false);

	//sec_ts_pinctrl_configure(ts, false);

out:
	mutex_unlock(&ts->device_mutex);
	return 0;
}

int sec_ts_start_device(struct sec_ts_data *ts, bool hardware)
{
	int ret = -1;

	VTI("start by %s\n",hardware == true ? "hardware" : "software");

	//sec_ts_pinctrl_configure(ts, true);

	mutex_lock(&ts->device_mutex);

	if (ts->power_status == SEC_TS_STATE_POWER_ON) {
		VTE("%s: already power on\n", __func__);
		goto out;
	}

	sec_ts_locked_release_all_finger(ts);

	ts->power_status = SEC_TS_STATE_POWER_ON;

	if (hardware) {
		ts->plat_data->power(ts, true);
		sec_ts_delay(70);
		ts->touch_noise_status = 0;

		ret = sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);
		if (ret < 0) {
			VTE("%s: Failed to wait_for_ready\n", __func__);
			goto err;
		}

		if (ts->plat_data->enable_sync)
			ts->plat_data->enable_sync(true);

		if (ts->flip_enable) {
			ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_COVERTYPE, &ts->cover_cmd, 1);
			if (ret < 0)
				goto err;

			ts->touch_functions = ts->touch_functions | SEC_TS_BIT_SETFUNC_COVER;
			VTI("%s: cover cmd write type:%d, mode:%x, ret:%d\n",
					__func__, ts->touch_functions, ts->cover_cmd, ret);
		} else {
			ts->touch_functions = (ts->touch_functions & (~SEC_TS_BIT_SETFUNC_COVER));
			VTI("%s: cover open, not send cmd\n", __func__);
		}

		ts->touch_functions = ts->touch_functions | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, (u8 *)&ts->touch_functions, 2);
		if (ret < 0) {
			VTE("%s: Failed to send touch function command\n", __func__);
			goto err;
		}

		sec_ts_set_grip_type(ts, ONLY_EDGE_HANDLER);

		if (ts->dex_mode) {
			VTI("%s: set dex mode\n", __func__);
			ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_DEX_MODE, &ts->dex_mode, 1);
			if (ret < 0) {
				VTE("%s: failed to set dex mode %x\n", __func__, ts->dex_mode);
				goto err;
			}
		}

		if (ts->brush_mode) {
			VTI("%s: set brush mode\n", __func__);
			ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_BRUSH_MODE, &ts->brush_mode, 1);
			if (ret < 0) {
				VTE("%s: failed to set brush mode\n", __func__);
				goto err;
			}
		}

		if (ts->touchable_area) {
			VTI("%s: set 16:9 mode\n", __func__);
			ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHABLE_AREA, &ts->touchable_area, 1);
			if (ret < 0) {
				VTE("%s: failed to set 16:9 mode\n", __func__);
				goto err;
			}
		}
	}

err:
	/* Sense_on */
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0)
		VTE("%s: fail to write Sense_on\n", __func__);

	enable_irq(ts->client->irq);

out:
	mutex_unlock(&ts->device_mutex);
	return ret;
}

#ifdef CONFIG_PM
static int sec_ts_pm_suspend(struct device *dev)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (ts->lowpower_mode)
		reinit_completion(&ts->resume_done);

	return 0;
}

static int sec_ts_pm_resume(struct device *dev)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (ts->lowpower_mode)
		complete_all(&ts->resume_done);

	return 0;
}
#endif

#ifdef CONFIG_PM
static struct dev_pm_ops sec_ts_dev_pm_ops = {
	.suspend = sec_ts_pm_suspend,
	.resume = sec_ts_pm_resume,
};
#endif

static const struct i2c_device_id sec_ts_id[] = {
	{ SEC_TS_I2C_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sec_ts_id);

#ifdef CONFIG_OF
static struct of_device_id sec_ts_match_table[] = {
	{ .compatible = "samsung,y761",},
	{ },
};
MODULE_DEVICE_TABLE(of, sec_ts_match_table);
#else
#define sec_ts_match_table NULL
#endif

static struct i2c_driver sec_ts_driver = {
	.probe		= sec_ts_probe,
	.remove		= sec_ts_remove,
	.shutdown	= sec_ts_shutdown,
	.id_table	= sec_ts_id,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SEC_TS_I2C_NAME,
		.of_match_table = sec_ts_match_table,
#ifdef CONFIG_PM
		.pm = &sec_ts_dev_pm_ops,
#endif
	},
};

static int __init sec_dev_init(void)
{
	VTI("sec_dev_init i2c add driver !");
	i2c_add_driver(&sec_ts_driver);
	return 0;
}

static void __exit sec_dev_exit(void)
{
	VTI("sec_dev_exit i2c delete driver !");
	i2c_del_driver(&sec_ts_driver);
}

module_init(sec_dev_init);
module_exit(sec_dev_exit);



