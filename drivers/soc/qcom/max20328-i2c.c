/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/soc/qcom/fsa4480-i2c.h>
#include <linux/pmic-voter.h>

#define MAX20328_I2C_NAME	"max20328-driver"

#ifdef pr_debug
#undef pr_debug
#endif
#define pr_debug pr_info

#ifdef dev_dbg
#undef dev_dbg
#endif
#define dev_dbg dev_info

enum max20328_regs_def {
	MAX20328_DEVICE_ID       = 0x00,
	MAX20328_ADC_VAL         = 0x01,
	MAX20328_STATUS1         = 0x02,
	MAX20328_STATUS2         = 0x03,
	MAX20328_INTERRUPT       = 0x04,
	MAX20328_MASK            = 0x05,
	MAX20328_CONTROL1        = 0x06,
	MAX20328_CONTROL2        = 0x07,
	MAX20328_CONTROL3        = 0x08,
	MAX20328_ADC_CONTROL1    = 0x09,
	MAX20328_ADC_CONTROL2    = 0x0A,
	MAX20328_HIHS_VAL        = 0x0B,
	MAX20328_OMTP_VAL        = 0x0C,
	MAX20328_SW_DEFLT1       = 0x0D,
	MAX20328_SW_DEFLT2       = 0x0E,
	MAX20328_REG_MAX,
};

#define USB_HSPHY_3P3_VOL_MIN			3050000 /* uV */
#define USB_HSPHY_3P3_VOL_MAX			3300000 /* uV */
#define USB_HSPHY_3P3_HPM_LOAD			16000	/* uA */

static struct max20328_priv *usbc_switch_mmax_priv;
bool max20328_drp_enable;

static u8 max20328_regs[] = {
	MAX20328_DEVICE_ID,
	MAX20328_ADC_VAL,
	MAX20328_STATUS1,
	MAX20328_STATUS2,
	MAX20328_INTERRUPT,
	MAX20328_MASK,
	MAX20328_CONTROL1,
	MAX20328_CONTROL2,
	MAX20328_CONTROL3,
	MAX20328_ADC_CONTROL1,
	MAX20328_ADC_CONTROL2,
	MAX20328_HIHS_VAL,
	MAX20328_OMTP_VAL,
	MAX20328_SW_DEFLT1,
	MAX20328_SW_DEFLT2,
};

struct max20328_priv {
	struct regmap *regmap;
	struct device *dev;
	struct power_supply *usb_psy;
	struct notifier_block psy_nb;
	struct votable *drp_mode_votable;
	atomic_t usbc_mode;
	struct work_struct usbc_analog_work;
	struct work_struct usbc_removed_work;
	struct work_struct max20328_irq_handler_work;
	struct blocking_notifier_head max20328_notifier;
	struct regulator *vdda33;
	wait_queue_head_t irq_waitq;
	bool power_enabled;
	struct mutex usbc_switch_lock;
	struct wake_lock usbc_wake_lock;
	int mmax_int;
	int mmax_int_irq;
	int dev_gpio;
	int dev_gpio_irq;
	int current_plug;
	int gpio_detection_type;
	int current_switch_dev;
};

struct max20328_reg_val {
	u16 reg;
	u8 val;
};

static const struct regmap_config max20328_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX20328_REG_MAX,
};

static const struct max20328_reg_val mmax_reg_i2c_defaults[] = {
	{MAX20328_SW_DEFLT1, 0x40}, /* 0x0D*/
	{MAX20328_SW_DEFLT2, 0x00}, /* 0x0E*/
	{MAX20328_ADC_CONTROL2, 0xF0}, /* 0x0A*/
	{MAX20328_CONTROL2, 0x00}, /* 0x07*/
	{MAX20328_CONTROL3, 0x00}, /* 0x08*/
	{MAX20328_ADC_CONTROL1, 0x30}, /* 0x09*/
	{MAX20328_CONTROL1, 0x13}, /* 0x06*/
};

static int max20328_usbc_wait_event_wake(struct max20328_priv *mmax_priv)
{
	int ret;

	pr_debug("%s: enter.\n", __func__);
	wake_lock_timeout(&mmax_priv->usbc_wake_lock, 3 * HZ);
	ret = wait_event_timeout(mmax_priv->irq_waitq,
				(mmax_priv->dev->power.is_suspended == false),
				msecs_to_jiffies(1000));
	if (!ret)
		pr_err("%s: check suspend timed out.\n", __func__);

	pr_debug("%s: leave.\n", __func__);
	return ret;
}

static int max20328_usbc_enable_power(struct max20328_priv *mmax_priv, bool on)
{
	int ret = 0;

	if (!mmax_priv || !mmax_priv->vdda33)
		return -EINVAL;

	dev_dbg(mmax_priv->dev, "%s turn %s regulators. power_enabled:%d\n",
			__func__, on ? "on" : "off", mmax_priv->power_enabled);

	if (mmax_priv->power_enabled == on) {
		dev_dbg(mmax_priv->dev, "USBC' regulators are already ON.\n");
		return 0;
	}

	if (!on)
		goto disable_vdda33;

	ret = regulator_set_load(mmax_priv->vdda33, USB_HSPHY_3P3_HPM_LOAD);
	if (ret < 0) {
		dev_err(mmax_priv->dev, "Unable to set HPM of vdda33:%d\n", ret);
		goto err_vdd;
	}

	ret = regulator_set_voltage(mmax_priv->vdda33, USB_HSPHY_3P3_VOL_MIN,
						USB_HSPHY_3P3_VOL_MAX);
	if (ret) {
		dev_err(mmax_priv->dev,
				"Unable to set voltage for vdda33:%d\n", ret);
		goto put_vdda33_lpm;
	}

	ret = regulator_enable(mmax_priv->vdda33);
	if (ret) {
		dev_err(mmax_priv->dev, "Unable to enable vdda33:%d\n", ret);
		goto unset_vdd33;
	}

	mmax_priv->power_enabled = true;

	dev_dbg(mmax_priv->dev, "%s(): USBC's regulators are turned ON.\n", __func__);
	return ret;

disable_vdda33:
	ret = regulator_disable(mmax_priv->vdda33);
	if (ret)
		dev_err(mmax_priv->dev, "Unable to disable vdda33:%d\n", ret);

unset_vdd33:
	ret = regulator_set_voltage(mmax_priv->vdda33, 0, USB_HSPHY_3P3_VOL_MAX);
	if (ret)
		dev_err(mmax_priv->dev,
			"Unable to set (0) voltage for vdda33:%d\n", ret);

put_vdda33_lpm:
	ret = regulator_set_load(mmax_priv->vdda33, 0);
	if (ret < 0)
		dev_err(mmax_priv->dev, "Unable to set (0) HPM of vdda33\n");
err_vdd:
	mmax_priv->power_enabled = false;
	dev_dbg(mmax_priv->dev, "USBC's regulators are turned OFF.\n");
	return ret;
}

static void max20328_usbc_switch_enable(struct max20328_priv *mmax_priv, bool enable)
{
	unsigned int val = 0, reg_06 = 0;
	int ret;

	if (!mmax_priv->regmap) {
		dev_err(mmax_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

	val = enable ? 0x13 : 0x03;
	ret = regmap_write(mmax_priv->regmap, 0x06, val);
	if (ret)
		dev_err(mmax_priv->dev, "%s: failed %d\n", __func__, ret);
	usleep_range(5 * 1000, 5 * 1000);
	regmap_read(mmax_priv->regmap, 0x06, &reg_06);
	dev_dbg(mmax_priv->dev, "%s: enable (%d) reg_0x06 (0x%x)\n",
			__func__, enable, reg_06);
}

void max20328_usbc_set_switch_mode(struct max20328_priv *mmax_priv, int mode)
{
	if (!mmax_priv->regmap) {
		dev_err(mmax_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	dev_dbg(mmax_priv->dev, "%s: mode (%d)\n", __func__, mode);

	switch (mode) {
	case POWER_SUPPLY_TYPEC_NONE: /* USB mode */
		regmap_write(mmax_priv->regmap, 0x0E, 0x00); /* DEF register2 set 00 */
		regmap_write(mmax_priv->regmap, 0x0D, 0x40); /* DEF register1 set TOP side closed in data connection, bottom side is open */
		regmap_write(mmax_priv->regmap, 0x07, 0x00); /* CONTROL2 register, switch state NOT Force mode nor follow MODE[0:2] */
		regmap_write(mmax_priv->regmap, 0x08, 0x00); /* CONTROL3 register, force value is not use, anyway default it. */
		regmap_write(mmax_priv->regmap, 0x09, 0x30); /* ADC CONTROL1, ADC is always off on USB MODE */
		regmap_write(mmax_priv->regmap, 0x06, 0x13); /* CONTROL1 register, switch enable, default programmable with registers 0x0D and 0x0E */
		break;
	case POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY: /* USB2 mode */
		regmap_write(mmax_priv->regmap, 0x0E, 0x00); /* DEF register2 set 00 */
		regmap_write(mmax_priv->regmap, 0x0D, 0x10); /* DEF register1 set TOP side closed in data connection, bottom side is open */
		regmap_write(mmax_priv->regmap, 0x07, 0x00); /* CONTROL2 register, switch state NOT Force mode nor follow MODE[0:2] */
		regmap_write(mmax_priv->regmap, 0x08, 0x00); /* CONTROL3 register, force value is not use, anyway default it. */
		regmap_write(mmax_priv->regmap, 0x09, 0x30); /* ADC CONTROL1, ADC is always off on USB MODE */
		regmap_write(mmax_priv->regmap, 0x06, 0x13); /* CONTROL1 register, switch enable, default programmable with registers 0x0D and 0x0E */
		break;
	case POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER:
		regmap_write(mmax_priv->regmap, 0x0D, 0x03); /* DEF register */
		regmap_write(mmax_priv->regmap, 0x0E, 0x10); /* DEF register2 */
		regmap_write(mmax_priv->regmap, 0x07, 0x02); /* CONTROL2 register */
		regmap_write(mmax_priv->regmap, 0x08, 0x00); /* CONTROL3 register */
		regmap_write(mmax_priv->regmap, 0x09, 0x00); /* ADC CONTROL1, ADC is always off */
		regmap_write(mmax_priv->regmap, 0x06, 0x13); /* CONTROL1 register, switch enable, single Audio accessory */
		break;
	default:
		break;
	}

	mutex_unlock(&mmax_priv->usbc_switch_lock);
}
EXPORT_SYMBOL(max20328_usbc_set_switch_mode);

#if 0
static void max20328_usbc_update_settings(struct max20328_priv *mmax_priv,
		u32 switch_control, u32 switch_enable)
{
	if (!mmax_priv->regmap) {
		dev_err(mmax_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

	regmap_write(mmax_priv->regmap, MAX20328_SWITCH_SETTINGS, 0x80);
	regmap_write(mmax_priv->regmap, MAX20328_SWITCH_CONTROL, switch_control);
	/* MAX20328 chip hardware requirement */
	usleep_range(50, 55);
	regmap_write(mmax_priv->regmap, MAX20328_SWITCH_SETTINGS, switch_enable);
}
#endif

static int max20328_usbc_event_changed(struct notifier_block *nb,
				      unsigned long evt, void *ptr)
{
	struct max20328_priv *mmax_priv =
			container_of(nb, struct max20328_priv, psy_nb);
	struct device *dev;

	if (!mmax_priv)
		return -EINVAL;

	dev = mmax_priv->dev;
	if (!dev)
		return -EINVAL;

	if ((struct power_supply *)ptr != mmax_priv->usb_psy ||
				evt != PSY_EVENT_PROP_CHANGED) {
		dev_dbg(dev, "%s: evt: %ld, retrun!\n", __func__, evt);
		return 0;
	}

	dev_dbg(dev, "%s: queueing usbc_analog_work\n", __func__);
	pm_stay_awake(mmax_priv->dev);
	schedule_work(&mmax_priv->usbc_analog_work);

	return 0;
}

/*
 * fsa4480_reg_notifier - register notifier block with mmax driver
 *
 * @nb - notifier block of max20328
 * @node - phandle node to max20328 device
 *
 * Returns 0 on success, or error code
 */
int fsa4480_reg_notifier(struct notifier_block *nb,
			 struct device_node *node)
{
	int rc = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct max20328_priv *mmax_priv;

	if (!client)
		return -EINVAL;

	mmax_priv = (struct max20328_priv *)i2c_get_clientdata(client);
	if (!mmax_priv)
		return -EINVAL;

	rc = blocking_notifier_chain_register
				(&mmax_priv->max20328_notifier, nb);
	if (rc)
		return rc;

	/*
	 * as part of the init sequence check if there is a connected
	 * USB C analog adapter
	 */
	dev_dbg(mmax_priv->dev, "%s: verify if USB adapter is already inserted\n",
		__func__);
	atomic_set(&(mmax_priv->usbc_mode), 0);
	rc = max20328_usbc_event_changed(&mmax_priv->psy_nb,
					     PSY_EVENT_PROP_CHANGED,
					     mmax_priv->usb_psy);

	return rc;
}
EXPORT_SYMBOL(fsa4480_reg_notifier);

/*
 * fsa4480_unreg_notifier - unregister notifier block with mmax driver
 *
 * @nb - notifier block of max20328
 * @node - phandle node to max20328 device
 *
 * Returns 0 on pass, or error code
 */
int fsa4480_unreg_notifier(struct notifier_block *nb,
			     struct device_node *node)
{
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct max20328_priv *mmax_priv;

	if (!client)
		return -EINVAL;

	mmax_priv = (struct max20328_priv *)i2c_get_clientdata(client);
	if (!mmax_priv)
		return -EINVAL;

	atomic_set(&(mmax_priv->usbc_mode), 0);
	max20328_usbc_set_switch_mode(mmax_priv, POWER_SUPPLY_TYPEC_NONE);
	return blocking_notifier_chain_unregister
					(&mmax_priv->max20328_notifier, nb);
}
EXPORT_SYMBOL(fsa4480_unreg_notifier);

#if 0
static int max20328_validate_display_port_settings(struct max20328_priv *mmax_priv)
{
	u32 switch_status = 0;

	regmap_read(mmax_priv->regmap, MAX20328_SWITCH_STATUS1, &switch_status);

	if ((switch_status != 0x23) && (switch_status != 0x1C)) {
		pr_err("AUX SBU1/2 switch status is invalid = %u\n",
				switch_status);
		return -EIO;
	}

	return 0;
}
#endif
/*
 * fsa4480_switch_event - configure MMAX switch position based on event
 *
 * @node - phandle node to max20328 device
 * @event - fsa_function enum
 *
 * Returns int on whether the switch happened or not
 */
int fsa4480_switch_event(struct device_node *node,
			 enum fsa_function event)
{
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	union power_supply_propval pval = {0, };
	struct max20328_priv *mmax_priv;
	bool is_audio_adapter;
	unsigned int val = 0;
	unsigned int val_0x0D = 0, val_0x0E = 0;

	if (!client)
		return -EINVAL;

	mmax_priv = (struct max20328_priv *)i2c_get_clientdata(client);
	if (!mmax_priv)
		return -EINVAL;
	if (!mmax_priv->regmap)
		return -EINVAL;

	max20328_usbc_wait_event_wake(mmax_priv);

	mutex_lock(&mmax_priv->usbc_switch_lock);

	dev_dbg(mmax_priv->dev, "%s: max20328: event = %d mode %d\n",
			__func__, event, atomic_read(&(mmax_priv->usbc_mode)));

	if (atomic_read(&(mmax_priv->usbc_mode)) == POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER)
		is_audio_adapter = true;
	else
		is_audio_adapter = false;

	switch (event) {
	case FSA_MIC_GND_SWAP:
		if (is_audio_adapter) {
			regmap_read(mmax_priv->regmap, 0x0D, &val);
			if ((val & 0x0f) == 0x07) {
				val = 0x03 | (val & 0xf0);
				regmap_write(mmax_priv->regmap, 0x0D, val);
				regmap_write(mmax_priv->regmap, 0x0E, 0x10);
			} else {
				val = 0x07 | (val & 0xf0);
				regmap_write(mmax_priv->regmap, 0x0D, val);
				regmap_write(mmax_priv->regmap, 0x0E, 0x40);
			}
		}
		break;
	case FSA_USBC_AUIDO_HP_ON:
		if (is_audio_adapter) {
			regmap_read(mmax_priv->regmap, 0x0D, &val);
			val = 0xa0 | (val & 0x0f);
			regmap_write(mmax_priv->regmap, 0x0D, val);
		}
		break;
	case FSA_USBC_AUIDO_HP_OFF:
		if (is_audio_adapter) {
			regmap_read(mmax_priv->regmap, 0x0D, &val);
			val = val & 0x0f;
			regmap_write(mmax_priv->regmap, 0x0D, val);
		}
		break;
	case FSA_USBC_ORIENTATION_CC1:
		regmap_write(mmax_priv->regmap, 0x06, 0x14);
		break;
	case FSA_USBC_ORIENTATION_CC2:
		regmap_write(mmax_priv->regmap, 0x06, 0x34);
		break;
	case FSA_USBC_DISPLAYPORT_DISCONNECTED:
		regmap_write(mmax_priv->regmap, 0x06, 0x14);
		break;
	case FSA_USBC_FAST_CHARGE_SELECT:
		if (!is_audio_adapter)
			regmap_write(mmax_priv->regmap, 0x0D, 0x10);
		break;
	case FSA_USBC_FAST_CHARGE_EXIT:
		if (!is_audio_adapter)
			regmap_write(mmax_priv->regmap, 0x0D, 0x40);
		break;
	case FSA_USBC_AUDIO_REPORT_IN:
		if (is_audio_adapter) {
			memset(&pval, 0, sizeof(pval));
			pval.intval = true;
			if (!power_supply_set_property(mmax_priv->usb_psy,
					POWER_SUPPLY_PROP_AUDIO_ATTACHED, &pval))
				pr_info("%s: max20328: AUDIO_ATTACHED true\n",
					 __func__);
		}
		break;
	case FSA_USBC_AUDIO_REPORT_REMOVE:
		memset(&pval, 0, sizeof(pval));
		pval.intval = false;
		if (!power_supply_set_property(mmax_priv->usb_psy,
				POWER_SUPPLY_PROP_AUDIO_ATTACHED, &pval))
			pr_info("%s: max20328: AUDIO_ATTACHED false\n",
				 __func__);
		break;
	default:
		break;
	}

	regmap_read(mmax_priv->regmap, 0x0D, &val_0x0D);
	regmap_read(mmax_priv->regmap, 0x0E, &val_0x0E);
	dev_dbg(mmax_priv->dev, "%s: max20328: val_0x0D = 0x%x, val_0x0E = 0x%x, is_audio_adapter %d\n",
			__func__, val_0x0D, val_0x0E, is_audio_adapter);

	mutex_unlock(&mmax_priv->usbc_switch_lock);

	return 0;
}
EXPORT_SYMBOL(fsa4480_switch_event);

int fsa4480_switch_mode_event(enum fsa_function event)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	bool is_audio_adapter;
	unsigned int val = 0;
	unsigned int val_0x0D = 0, val_0x0E = 0;

	if (!mmax_priv) {
		pr_err("%s: max20328 mmax_priv is null\n", __func__);
		return -EINVAL;
	}
	if (!mmax_priv->regmap) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return -EINVAL;
	}

	max20328_usbc_wait_event_wake(mmax_priv);

	mutex_lock(&mmax_priv->usbc_switch_lock);

	if (atomic_read(&(mmax_priv->usbc_mode)) == POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER)
		is_audio_adapter = true;
	else
		is_audio_adapter = false;

	dev_dbg(mmax_priv->dev, "%s: max20328: event: %d, mode: %d, is_audio_adapter: %d.\n",
			__func__, event, atomic_read(&(mmax_priv->usbc_mode)), is_audio_adapter);

	switch (event) {
	case FSA_MIC_GND_SWAP:
		if (is_audio_adapter) {
			regmap_read(mmax_priv->regmap, 0x0D, &val);
			if ((val & 0x0f) == 0x07) {
				val = 0x03 | (val & 0xf0);
				regmap_write(mmax_priv->regmap, 0x0D, val);
				regmap_write(mmax_priv->regmap, 0x0E, 0x10);
			} else {
				val = 0x07 | (val & 0xf0);
				regmap_write(mmax_priv->regmap, 0x0D, val);
				regmap_write(mmax_priv->regmap, 0x0E, 0x40);
			}
		}
		break;
	case FSA_USBC_AUIDO_HP_ON:
		if (is_audio_adapter) {
			regmap_read(mmax_priv->regmap, 0x0D, &val);
			val = 0xa0 | (val & 0x0f);
			regmap_write(mmax_priv->regmap, 0x0D, val);
		}
		break;
	case FSA_USBC_AUIDO_HP_OFF:
		if (is_audio_adapter) {
			regmap_read(mmax_priv->regmap, 0x0D, &val);
			val = val & 0x0f;
			regmap_write(mmax_priv->regmap, 0x0D, val);
		}
		break;
	case FSA_USBC_ORIENTATION_CC2:
		regmap_write(mmax_priv->regmap, 0x06, 0x34);
		break;
	case FSA_USBC_DISPLAYPORT_DISCONNECTED:
		regmap_write(mmax_priv->regmap, 0x06, 0x14);
		break;
	case FSA_USBC_FAST_CHARGE_SELECT:
		if (!is_audio_adapter)
			regmap_write(mmax_priv->regmap, 0x0D, 0x10);
		break;
	case FSA_USBC_FAST_CHARGE_EXIT:
		if (!is_audio_adapter)
			regmap_write(mmax_priv->regmap, 0x0D, 0x40);
		break;
	case FSA_USBC_SWITCH_ENABLE:
		if (!is_audio_adapter)
			max20328_usbc_switch_enable(mmax_priv, true);
		break;
	case FSA_USBC_SWITCH_DISABLE:
		if (!is_audio_adapter)
			max20328_usbc_switch_enable(mmax_priv, false);
		break;
	default:
		break;
	}

	regmap_read(mmax_priv->regmap, 0x0D, &val_0x0D);
	regmap_read(mmax_priv->regmap, 0x0E, &val_0x0E);
	dev_dbg(mmax_priv->dev, "%s: max20328: val_0x0D = 0x%x, val_0x0E = 0x%x, is_audio_adapter %d\n",
			__func__, val_0x0D, val_0x0E, is_audio_adapter);

	mutex_unlock(&mmax_priv->usbc_switch_lock);

	return 0;
}
EXPORT_SYMBOL(fsa4480_switch_mode_event);

static int max20328_usbc_analog_setup_switches
			(struct max20328_priv *mmax_priv, bool active)
{
	int rc = 0;

	dev_dbg(mmax_priv->dev, "%s: setting GPIOs active = %d\n",
		__func__, active);

	if (active) {
		/* activate switches */
		max20328_usbc_set_switch_mode(mmax_priv, POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER);

		/* notify call chain on event */
		blocking_notifier_call_chain(&mmax_priv->max20328_notifier,
		POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER, NULL);
	} else {
		/* notify call chain on event */
		blocking_notifier_call_chain(&mmax_priv->max20328_notifier,
				POWER_SUPPLY_TYPEC_NONE, NULL);

		/* deactivate switches */
		max20328_usbc_set_switch_mode(mmax_priv, POWER_SUPPLY_TYPEC_NONE);
	}

	return rc;
}

static void max20328_usbc_analog_work_fn(struct work_struct *work)
{
	struct max20328_priv *mmax_priv =
		container_of(work, struct max20328_priv, usbc_analog_work);
	union power_supply_propval mode;
	struct device *dev;
	unsigned int reg_06 = 0;
	int ret = 0;

	if (!mmax_priv) {
		pr_err("%s: mmax container invalid\n", __func__);
		goto err;
	}

	dev = mmax_priv->dev;
	if (!dev) {
		pr_err("%s: mmax dev invalid\n", __func__);
		goto err;
	}

	if (!mmax_priv->regmap) {
		dev_err(mmax_priv->dev, "%s: regmap invalid\n", __func__);
		goto err;
	}

	max20328_usbc_wait_event_wake(mmax_priv);

	mutex_lock(&mmax_priv->usbc_switch_lock);
	ret = regmap_write(mmax_priv->regmap, 0x06, 0x13);
	if (ret)
		dev_err(mmax_priv->dev, "%s: failed %d\n", __func__, ret);
	usleep_range(5 * 1000, 5 * 1000);
	regmap_read(mmax_priv->regmap, 0x06, &reg_06);
	dev_dbg(mmax_priv->dev, "%s: reg_0x06 (0x%x)\n",
			__func__, reg_06);
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	ret = power_supply_get_property(mmax_priv->usb_psy,
			POWER_SUPPLY_PROP_TYPEC_MODE, &mode);
	if (ret) {
		dev_err(dev, "%s: Unable to read USB TYPEC_MODE: %d\n",
			__func__, ret);
		goto err;
	}

	dev_dbg(dev, "%s: USB change event received, supply mode %d, usbc mode %d, "
		"audio adapter expected %d\n", __func__,
		mode.intval, mmax_priv->usbc_mode.counter,
		POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER);

	switch (mode.intval) {
	case POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER:
	case POWER_SUPPLY_TYPEC_NONE:
		if (atomic_read(&(mmax_priv->usbc_mode)) == mode.intval)
			break; /* filter notifications received before */
		atomic_set(&(mmax_priv->usbc_mode), mode.intval);
		max20328_usbc_analog_setup_switches(mmax_priv,
			atomic_read(&(mmax_priv->usbc_mode)) != POWER_SUPPLY_TYPEC_NONE);
		break;
	default:
		break;
	}

err:
	pm_relax(mmax_priv->dev);
	return;
}

static void max20328_update_reg_defaults(struct regmap *regmap)
{
	int reg_00 = 0;
	int ret = 0;
	u8 i;

	ret = regmap_read(regmap, MAX20328_DEVICE_ID, &reg_00);

	pr_info("%s: MAX20328_DEVICE_ID[0x%02x]: 0x%02x\n",
		__func__, MAX20328_DEVICE_ID, reg_00);

	for (i = 0; i < ARRAY_SIZE(mmax_reg_i2c_defaults); i++)
		regmap_write(regmap, mmax_reg_i2c_defaults[i].reg,
				   mmax_reg_i2c_defaults[i].val);
}

static void max20328_usbc_removed_work_fn(struct work_struct *work)
{
	union power_supply_propval pval = {0, };
	struct max20328_priv *mmax_priv =
		container_of(work, struct max20328_priv, usbc_removed_work);

	if (!mmax_priv) {
		pr_err("%s: mmax container invalid\n", __func__);
		return;
	}

	msleep(300);
	max20328_usbc_wait_event_wake(mmax_priv);

	mutex_lock(&mmax_priv->usbc_switch_lock);
	if (!mmax_priv->current_plug) {
		memset(&pval, 0, sizeof(pval));
		if (mmax_priv->drp_mode_votable &&
			!vote(mmax_priv->drp_mode_votable, "AUDIO_DRP_VOTER", false, 1)) {
			pr_info("%s: vote AUDIO_DRP_VOTER false successful\n", __func__);
		}
		/* pval.intval = POWER_SUPPLY_TYPEC_PR_SINK_VIVO;
		if (!power_supply_set_property(mmax_priv->usb_psy,
				POWER_SUPPLY_PROP_TYPEC_POWER_ROLE, &pval))
			pr_info("%s: force PR_SINK mode successful\n",
				 __func__); */
		/*vivo chg*/
		pval.intval = USB_DET_PIN_MODE_IDLE;
		if (!power_supply_set_property(mmax_priv->usb_psy,
				POWER_SUPPLY_PROP_USB_DET_PIN_MODE, &pval)) {
			pr_info("%s: set usb_det_pin_mode %d successful\n",
				 __func__, USB_DET_PIN_MODE_IDLE);
			power_supply_changed(mmax_priv->usb_psy);
		}

		max20328_drp_enable = false;
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);
}

static irqreturn_t max20328_usbc_irq_handler(int irq, void *data)
{
	struct max20328_priv *mmax_priv = data;
	union power_supply_propval pval = {0, };
	int try_times = 3, read_times, i;
	bool new_type;

	pr_info("%s: enter\n", __func__);

	if (!mmax_priv)
		return -ENOMEM;

	disable_irq_nosync(mmax_priv->dev_gpio_irq);
	mutex_lock(&mmax_priv->usbc_switch_lock);

	if (mmax_priv->current_plug)
		read_times = 3000;
	else
		read_times = 400;

	mmax_priv->gpio_detection_type = !gpio_get_value_cansleep(mmax_priv->dev_gpio);

	for (; try_times > 0; try_times--) {
		for (i = read_times; i > 0; i--) {
			new_type = !gpio_get_value_cansleep(mmax_priv->dev_gpio);
			if (mmax_priv->gpio_detection_type != new_type) {
				mmax_priv->gpio_detection_type = new_type;
				break;
			}
		if (i == (read_times / 4))
			usleep_range(5 * 1000, 5 * 1000);
		else if (i == (read_times / 3))
			usleep_range(10 * 1000, 10 * 1000);
		else if (i == (read_times / 2))
			usleep_range(15 * 1000, 15 * 1000);
		}
		pr_info("%s: try_times %d, i %d\n",
			__func__, try_times, i);
		if (i <= 0)
			break;

		usleep_range(30 * 1000, 30 * 1000);
	}

	if (try_times <= 0) {
		pr_info("%s: detect usb failed, keep current state %d\n",
				__func__, mmax_priv->current_plug);
		goto leave;
	} else {
		pr_info("%s: detect usb success, detection_type %d\n",
			__func__, mmax_priv->gpio_detection_type);
		mmax_priv->current_plug = mmax_priv->gpio_detection_type;

		if (mmax_priv->gpio_detection_type) {
			memset(&pval, 0, sizeof(pval));
			max20328_drp_enable = true;

			/*vivo chg*/
			pval.intval = USB_DET_PIN_MODE_DET;
			if (!power_supply_set_property(mmax_priv->usb_psy,
					POWER_SUPPLY_PROP_USB_DET_PIN_MODE, &pval)) {
				pr_info("%s: set usb_det_pin_mode %d successful\n",
					 __func__, USB_DET_PIN_MODE_DET);
				power_supply_changed(mmax_priv->usb_psy);
			}

			/*pval.intval = POWER_SUPPLY_TYPEC_PR_DUAL_VIVO;
			if (!power_supply_set_property(mmax_priv->usb_psy,
					POWER_SUPPLY_PROP_TYPEC_POWER_ROLE, &pval))
				pr_info("%s: force PR_DUAL mode successful\n",
					 __func__);*/
			if (mmax_priv->drp_mode_votable &&
				!vote(mmax_priv->drp_mode_votable, "AUDIO_DRP_VOTER", true, 1)) {
				pr_info("%s: vote AUDIO_DRP_VOTER true successful\n", __func__);
			}
		} else {
			pr_info("%s: queueing usbc_removed_work\n", __func__);
			schedule_work(&mmax_priv->usbc_removed_work);
		}
	}

leave:

	if (mmax_priv->current_plug)
		irq_set_irq_type(mmax_priv->dev_gpio_irq, (IRQF_ONESHOT | IRQF_TRIGGER_HIGH));
	else
		irq_set_irq_type(mmax_priv->dev_gpio_irq, (IRQF_ONESHOT | IRQF_TRIGGER_LOW));

	mutex_unlock(&mmax_priv->usbc_switch_lock);
	enable_irq(mmax_priv->dev_gpio_irq);

	pr_info("%s: leave\n", __func__);
	return IRQ_HANDLED;
}

static void max20328_irq_handler_work_fn(struct work_struct *work)
{
	struct max20328_priv *mmax_priv =
		container_of(work, struct max20328_priv, max20328_irq_handler_work);
	unsigned int data = 0, i;
	int reg_04 = 0;

	if (!mmax_priv) {
		pr_err("%s: mmax container invalid\n", __func__);
		return;
	}

	if (!mmax_priv->regmap) {
		pr_err("%s: mmax regmap is null\n", __func__);
		return;
	}

	max20328_usbc_wait_event_wake(mmax_priv);

	mutex_lock(&mmax_priv->usbc_switch_lock);
	regmap_read(mmax_priv->regmap, MAX20328_INTERRUPT, &reg_04);
	for (i = 0; i < sizeof(max20328_regs); i++) {
		regmap_read(mmax_priv->regmap, max20328_regs[i], &data);
		pr_info("%s: reg[0x%02x]: 0x%02x\n",
			__func__, max20328_regs[i], data);
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	msleep(500);
	enable_irq(mmax_priv->mmax_int_irq);
}

static irqreturn_t max20328_irq_handler(int irq, void *data)
{
	struct max20328_priv *mmax_priv = data;

	pr_info("%s: enter\n", __func__);

	if (!mmax_priv)
		return -ENOMEM;

	disable_irq_nosync(mmax_priv->mmax_int_irq);
	schedule_work(&mmax_priv->max20328_irq_handler_work);

	pr_info("%s: leave\n", __func__);
	return IRQ_HANDLED;
}

static int max20328_usbc_irq_init(struct max20328_priv *mmax_priv)
{
	int ret;

	mmax_priv->dev_gpio = of_get_named_gpio(mmax_priv->dev->of_node,
									"vivo,usbc-irq-gpio", 0);
	if (mmax_priv->dev_gpio > 0) {
		ret = gpio_request(mmax_priv->dev_gpio, "usbc irq");
		if (ret < 0) {
			dev_err(mmax_priv->dev, "%s: gpio %d request error %d\n",
			       __func__, mmax_priv->dev_gpio, ret);
		}
		ret = gpio_direction_input(mmax_priv->dev_gpio);
		if (ret < 0)
			dev_err(mmax_priv->dev, "%s: set gpio %d input error %d\n", __func__,
					mmax_priv->dev_gpio, ret);
		mmax_priv->dev_gpio_irq = gpio_to_irq(mmax_priv->dev_gpio);
		dev_dbg(mmax_priv->dev, "%s: gpio_irq = %d\n", __func__,
				mmax_priv->dev_gpio_irq);
	} else {
		dev_dbg(mmax_priv->dev, "%s: gpio irq may not to be supported, return value %d\n",
				__func__, mmax_priv->dev_gpio);
		return 0;
	}

	if (mmax_priv->dev_gpio_irq) {
		ret = request_threaded_irq(mmax_priv->dev_gpio_irq,	NULL,
									max20328_usbc_irq_handler,
									(IRQF_ONESHOT | IRQF_TRIGGER_LOW),
									"usbc irq", mmax_priv);
		if (ret != 0) {
			dev_err(mmax_priv->dev, "%s: Failed to request IRQ: %d\n", __func__, ret);
			free_irq(mmax_priv->dev_gpio_irq, mmax_priv);
			gpio_free(mmax_priv->dev_gpio);
			return ret;
		}
	}

	ret = enable_irq_wake(mmax_priv->dev_gpio_irq);
	if (ret)
		dev_err(mmax_priv->dev, "%s: Failed to enable wake up irq %d\n",
			  __func__, mmax_priv->dev_gpio_irq);

	return 0;
}

static int max20328_irq_init(struct max20328_priv *mmax_priv)
{
	int ret;

	mmax_priv->mmax_int = of_get_named_gpio(mmax_priv->dev->of_node,
									"max20328-irq-gpio", 0);
	if (mmax_priv->mmax_int > 0) {
		ret = gpio_request(mmax_priv->mmax_int, "max20328 irq");
		if (ret < 0) {
			dev_err(mmax_priv->dev, "%s: gpio %d request error %d\n",
			       __func__, mmax_priv->mmax_int, ret);
		}
		ret = gpio_direction_input(mmax_priv->mmax_int);
		if (ret < 0)
			dev_err(mmax_priv->dev, "%s: set gpio %d input error %d\n", __func__,
					mmax_priv->mmax_int, ret);
		mmax_priv->mmax_int_irq = gpio_to_irq(mmax_priv->mmax_int);
		dev_dbg(mmax_priv->dev, "%s: gpio_irq = %d\n", __func__,
				mmax_priv->mmax_int_irq);
	} else {
		dev_dbg(mmax_priv->dev, "%s: gpio irq may not to be supported, return value %d\n",
				__func__, mmax_priv->mmax_int);
		return 0;
	}

	if (mmax_priv->mmax_int_irq) {
		ret = request_threaded_irq(mmax_priv->mmax_int_irq,	NULL,
									max20328_irq_handler,
									(IRQF_ONESHOT | IRQF_TRIGGER_LOW),
									"max20328 irq", mmax_priv);
		if (ret != 0) {
			dev_err(mmax_priv->dev, "%s: Failed to request IRQ: %d\n", __func__, ret);
			free_irq(mmax_priv->mmax_int_irq, mmax_priv);
			gpio_free(mmax_priv->mmax_int);
			return ret;
		}
	}

	ret = enable_irq_wake(mmax_priv->mmax_int_irq);
	if (ret)
		dev_err(mmax_priv->dev, "%s: Failed to enable wake up irq %d\n",
			  __func__, mmax_priv->mmax_int_irq);

	return 0;
}

int get_usbc_mg_status(void)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int val_0x0D = 0, val_0x0E = 0;
	int ret = -1;

	if (!mmax_priv) {
		pr_err("%s: mmax container invalid\n", __func__);
		return ret;
	}

	if (!mmax_priv->regmap) {
		pr_err("%s: max20328 regmap is null\n", __func__);
		return ret;
	}

	if (atomic_read(&(mmax_priv->usbc_mode)) != POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER) {
		pr_err("%s: peripheral is not audio adapter, %d, return!\n",
			__func__, atomic_read(&(mmax_priv->usbc_mode)));
		return ret;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	regmap_read(mmax_priv->regmap, MAX20328_SW_DEFLT1, &val_0x0D);
	regmap_read(mmax_priv->regmap, MAX20328_SW_DEFLT2, &val_0x0E);
	pr_info("%s: max20328: val_0x0D: 0x%x, val_0x0E: 0x%x\n",
			__func__, val_0x0D, val_0x0E);
	if (val_0x0E & 0x40) {
		ret = 1;
	} else if (val_0x0E & 0x10) {
		ret = 2;
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	return ret;
}
EXPORT_SYMBOL(get_usbc_mg_status);

int get_usbc_peripheral_status(void)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	int ret = -1;

	if (!mmax_priv)
		return ret;

	mutex_lock(&mmax_priv->usbc_switch_lock);
	ret = mmax_priv->current_plug;
	pr_info("%s: gpio_current_plug: %d.\n",
		__func__, ret);
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	return ret;
}
EXPORT_SYMBOL(get_usbc_peripheral_status);

#ifdef CONFIG_DEBUG_FS
static struct dentry *max20328_debugfs_root;
static struct dentry *max20328_debugfs_reg;
static struct dentry *max20328_debugfs_i2c;

static int max20328_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t max20328_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	unsigned int kbuf[2];
	char *temp;
	int ret = 0;

	temp = kmalloc(cnt, GFP_KERNEL);
	if (!temp) {
		return -ENOMEM;
	}

	ret = copy_from_user(temp, ubuf, cnt);
	ret = sscanf(temp, "%x %x", &kbuf[0], &kbuf[1]);
	if (!ret) {
		kfree(temp);
		return -EFAULT;
	}

	pr_info("%s: kbuf[0]: %x, kbuf[1]: %x cnt: %d\n",
		__func__, kbuf[0], kbuf[1], (int)cnt);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	if (kbuf[0] <= MAX20328_REG_MAX) {
		mutex_lock(&mmax_priv->usbc_switch_lock);
		regmap_write(mmax_priv->regmap, kbuf[0], kbuf[1]);
		mutex_unlock(&mmax_priv->usbc_switch_lock);
	} else {
		pr_err("%s: reg addr 0x%x out of range.\n", __func__, kbuf[0]);
	}

	kfree(temp);
	return cnt;
}

static ssize_t max20328_debug_read(struct file *file, char __user *buf,
					size_t count, loff_t *pos)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	const int size = 512;
	char buffer[size];
	int n = 0, i;
	unsigned int data;

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	mutex_lock(&mmax_priv->usbc_switch_lock);
	for (i = 0; i < sizeof(max20328_regs); i++) {
		regmap_read(mmax_priv->regmap, max20328_regs[i], &data);
		n += scnprintf(buffer+n, size-n, "reg[0x%02x]: 0x%02x\n", max20328_regs[i], data);
		pr_info("%s: reg[0x%02x]: 0x%02x.\n", __func__, max20328_regs[i], data);
	}
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations max20328_debugfs_fops = {
	.open = max20328_debug_open,
	.read = max20328_debug_read,
	.write = max20328_debug_write,
};

static ssize_t max20328_debug_i2c_read(struct file *file, char __user *buf,
					size_t count, loff_t *pos)
{
	struct max20328_priv *mmax_priv = usbc_switch_mmax_priv;
	struct i2c_client *i2c = NULL;
	const int size = 512;
	char buffer[size];
	int n = 0, ret = 0, reg_01 = 0;

	pr_info("%s: i2c read enter.\n", __func__);

	if (IS_ERR_OR_NULL(mmax_priv)) {
		pr_err("%s: Invalid data.\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(mmax_priv->dev)) {
		pr_err("%s: Invalid client.\n ", __func__);
		return -EFAULT;
	}

	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		pr_err("%s: Invalid regmap.\n ", __func__);
		return -EFAULT;
	}

	i2c = to_i2c_client(mmax_priv->dev);

	mutex_lock(&mmax_priv->usbc_switch_lock);
	ret = regmap_read(mmax_priv->regmap, MAX20328_DEVICE_ID, &reg_01);
	mutex_unlock(&mmax_priv->usbc_switch_lock);

	n += scnprintf(buffer+n, size-n, "MAX20328-0x%x %s\n",
		i2c->addr, ((ret < 0) || !(reg_01 & 0x80)) ? "ERROR" : "OK");
	buffer[n] = 0;
	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations max20328_i2c_debugfs_fops = {
	.open = max20328_debug_open,
	.read = max20328_debug_i2c_read,
};

static void max20328_debugfs_init(void)
{
	max20328_debugfs_root = debugfs_create_dir("audio-max20328", NULL);
	if (IS_ERR_OR_NULL(max20328_debugfs_root)) {
		pr_err("%s: debugfs create dir error\n", __func__);
		goto err1;
	}

	max20328_debugfs_reg = debugfs_create_file("reg", 0644,
		max20328_debugfs_root, NULL, &max20328_debugfs_fops);
	if (IS_ERR_OR_NULL(max20328_debugfs_reg)) {
		pr_err("%s: debugfs reg create failed\n", __func__);
		goto err2;
	}

	max20328_debugfs_i2c = debugfs_create_file("i2c", 0444,
		max20328_debugfs_root, NULL, &max20328_i2c_debugfs_fops);
	if (IS_ERR_OR_NULL(max20328_debugfs_i2c)) {
		pr_err("%s: debugfs i2c create failed\n", __func__);
		goto err3;
	}

	return;
err3:
	debugfs_remove(max20328_debugfs_reg);
	max20328_debugfs_i2c = NULL;
err2:
	debugfs_remove(max20328_debugfs_root);
	max20328_debugfs_reg = NULL;
err1:
	max20328_debugfs_root = NULL;
	return;
}

static void max20328_debugfs_deinit(void)
{
	if (!IS_ERR_OR_NULL(max20328_debugfs_i2c))
		debugfs_remove(max20328_debugfs_i2c);
	if (!IS_ERR_OR_NULL(max20328_debugfs_reg))
		debugfs_remove(max20328_debugfs_reg);
	if (!IS_ERR_OR_NULL(max20328_debugfs_root))
		debugfs_remove(max20328_debugfs_root);
	return;
}
#endif

static int max20328_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *id)
{
	struct max20328_priv *mmax_priv;
	//union power_supply_propval pval = {0, };
	int rc = 0;

	mmax_priv = devm_kzalloc(&i2c->dev, sizeof(*mmax_priv),
				GFP_KERNEL);
	if (!mmax_priv)
		return -ENOMEM;

	mmax_priv->dev = &i2c->dev;
	mutex_init(&mmax_priv->usbc_switch_lock);
	mmax_priv->usb_psy = power_supply_get_by_name("usb");
	if (!mmax_priv->usb_psy) {
		rc = -EPROBE_DEFER;
		dev_dbg(mmax_priv->dev,
			"%s: could not get USB psy info: %d\n",
			__func__, rc);
		goto err_data;
	}

	mmax_priv->drp_mode_votable = find_votable("DRP_MODE");
	if (!mmax_priv->drp_mode_votable) {
		rc = -EPROBE_DEFER;
		dev_dbg(mmax_priv->dev,
			"%s: could not get usb drp mode votable %d\n",
			__func__, rc);
		goto err_supply;
	}

	dev_dbg(mmax_priv->dev, "%s enter\n", __func__);

	mmax_priv->vdda33 = devm_regulator_get(&i2c->dev, "vdda33");
	if (IS_ERR(mmax_priv->vdda33)) {
		dev_err(&i2c->dev, "unable to get vdda33 supply\n");
		rc = PTR_ERR(mmax_priv->vdda33);
		goto err_supply;
	}

	max20328_usbc_enable_power(mmax_priv, true);

	mmax_priv->regmap = devm_regmap_init_i2c(i2c, &max20328_regmap_config);
	if (IS_ERR_OR_NULL(mmax_priv->regmap)) {
		dev_err(mmax_priv->dev, "%s: Failed to initialize regmap: %d\n",
			__func__, rc);
		if (!mmax_priv->regmap) {
			rc = -EINVAL;
			goto err_supply;
		}
		rc = PTR_ERR(mmax_priv->regmap);
		goto err_supply;
	}
	max20328_update_reg_defaults(mmax_priv->regmap);
	usbc_switch_mmax_priv = mmax_priv;
	mmax_priv->psy_nb.notifier_call = max20328_usbc_event_changed;
	mmax_priv->psy_nb.priority = 0;
	rc = power_supply_reg_notifier(&mmax_priv->psy_nb);
	if (rc) {
		dev_err(mmax_priv->dev, "%s: power supply reg failed: %d\n",
			__func__, rc);
		goto err_supply;
	}

	wake_lock_init(&mmax_priv->usbc_wake_lock, WAKE_LOCK_SUSPEND, "usbc wake lock");
	init_waitqueue_head(&mmax_priv->irq_waitq);

	rc = max20328_irq_init(mmax_priv);
	if (rc < 0)
		goto mmax_irq_err;

	i2c_set_clientdata(i2c, mmax_priv);

	INIT_WORK(&mmax_priv->usbc_analog_work,
		  max20328_usbc_analog_work_fn);
	INIT_WORK(&mmax_priv->usbc_removed_work,
			  max20328_usbc_removed_work_fn);
	INIT_WORK(&mmax_priv->max20328_irq_handler_work,
			  max20328_irq_handler_work_fn);

	mmax_priv->max20328_notifier.rwsem =
		(struct rw_semaphore)__RWSEM_INITIALIZER
		((mmax_priv->max20328_notifier).rwsem);
	mmax_priv->max20328_notifier.head = NULL;

	/*memset(&pval, 0, sizeof(pval));
	pval.intval = POWER_SUPPLY_TYPEC_PR_SINK_VIVO;
	if (!power_supply_set_property(mmax_priv->usb_psy,
			POWER_SUPPLY_PROP_TYPEC_POWER_ROLE, &pval))
		pr_info("%s: force PR_SINK mode successful\n",
			 __func__);*//*chager driver have inited sink mode,so remove*/
	max20328_drp_enable = false;

	rc = max20328_usbc_irq_init(mmax_priv);
	if (rc < 0)
		goto irq_err;

#ifdef CONFIG_DEBUG_FS
	max20328_debugfs_init();
#endif

	dev_dbg(mmax_priv->dev, "%s leave\n", __func__);
	return 0;

irq_err:
	if (mmax_priv->mmax_int > 0) {
		disable_irq_nosync(mmax_priv->mmax_int_irq);
		free_irq(mmax_priv->mmax_int_irq, mmax_priv);
		gpio_free(mmax_priv->mmax_int);
	}
mmax_irq_err:
	wake_lock_destroy(&mmax_priv->usbc_wake_lock);
err_supply:
	power_supply_put(mmax_priv->usb_psy);
err_data:
	usbc_switch_mmax_priv = NULL;
	mutex_destroy(&mmax_priv->usbc_switch_lock);
	devm_kfree(&i2c->dev, mmax_priv);
	return rc;
}

static int max20328_remove(struct i2c_client *i2c)
{
	struct max20328_priv *mmax_priv =
			(struct max20328_priv *)i2c_get_clientdata(i2c);

	if (!mmax_priv)
		return -EINVAL;

	dev_dbg(mmax_priv->dev, "%s\n", __func__);

#ifdef CONFIG_DEBUG_FS
	max20328_debugfs_deinit();
#endif

	max20328_usbc_set_switch_mode(mmax_priv, POWER_SUPPLY_TYPEC_NONE);
	if (mmax_priv->dev_gpio > 0) {
		disable_irq_nosync(mmax_priv->dev_gpio_irq);
		free_irq(mmax_priv->dev_gpio_irq, mmax_priv);
		gpio_free(mmax_priv->dev_gpio);
	}
	if (mmax_priv->mmax_int > 0) {
		disable_irq_nosync(mmax_priv->mmax_int_irq);
		free_irq(mmax_priv->mmax_int_irq, mmax_priv);
		gpio_free(mmax_priv->mmax_int);
	}
	cancel_work_sync(&mmax_priv->max20328_irq_handler_work);
	cancel_work_sync(&mmax_priv->usbc_removed_work);
	cancel_work_sync(&mmax_priv->usbc_analog_work);
	wake_lock_destroy(&mmax_priv->usbc_wake_lock);
	pm_relax(mmax_priv->dev);
	/* deregister from PMI */
	power_supply_unreg_notifier(&mmax_priv->psy_nb);
	max20328_usbc_enable_power(mmax_priv, false);
	power_supply_put(mmax_priv->usb_psy);
	dev_set_drvdata(&i2c->dev, NULL);
	mutex_destroy(&mmax_priv->usbc_switch_lock);
	usbc_switch_mmax_priv = NULL;
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max20328_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct max20328_priv *mmax_priv =
			(struct max20328_priv *)i2c_get_clientdata(i2c);

	if (!mmax_priv)
		return -EINVAL;

	dev_dbg(mmax_priv->dev, "%s\n", __func__);

	return 0;
}

static int max20328_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct max20328_priv *mmax_priv =
			(struct max20328_priv *)i2c_get_clientdata(i2c);

	if (!mmax_priv)
		return -EINVAL;

	dev_dbg(mmax_priv->dev, "%s\n", __func__);
	wake_up(&mmax_priv->irq_waitq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(max20328_pm_ops, max20328_suspend, max20328_resume);
#define MAX20328_PM_OPS (&max20328_pm_ops)
#else
#define MAX20328_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */

static const struct of_device_id max20328_i2c_dt_match[] = {
	{
		.compatible = "qcom,max20328-i2c",
	},
	{}
};

static struct i2c_driver max20328_i2c_driver = {
	.driver = {
		.name = MAX20328_I2C_NAME,
		.of_match_table = max20328_i2c_dt_match,
		.pm = MAX20328_PM_OPS,
	},
	.probe = max20328_probe,
	.remove = max20328_remove,
};

static int __init max20328_init(void)
{
	int rc;

	rc = i2c_add_driver(&max20328_i2c_driver);
	if (rc)
		pr_err("max20328: Failed to register I2C driver: %d\n", rc);

	return rc;
}
module_init(max20328_init);

static void __exit max20328_exit(void)
{
	i2c_del_driver(&max20328_i2c_driver);
}
module_exit(max20328_exit);

MODULE_DESCRIPTION("MAX20328 I2C driver");
MODULE_LICENSE("GPL v2");
