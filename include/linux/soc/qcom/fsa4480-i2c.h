/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 */
#ifndef FSA4480_I2C_H
#define FSA4480_I2C_H

#include <linux/of.h>
#include <linux/notifier.h>

enum fsa_function {
	FSA_MIC_GND_SWAP,
	FSA_USBC_AUIDO_HP_ON,
	FSA_USBC_AUIDO_HP_OFF,
	FSA_USBC_ORIENTATION_CC1,
	FSA_USBC_ORIENTATION_CC2,
	FSA_USBC_DISPLAYPORT_DISCONNECTED,
	FSA_USBC_FAST_CHARGE_SELECT,
	FSA_USBC_FAST_CHARGE_EXIT,
	FSA_USBC_SWITCH_ENABLE,
	FSA_USBC_SWITCH_DISABLE,
	FSA_USBC_AUDIO_REPORT_IN,
	FSA_USBC_AUDIO_REPORT_REMOVE,
	FSA_EVENT_MAX,
};

#ifdef CONFIG_QCOM_FSA4480_I2C
int fsa4480_switch_event(struct device_node *node,
			 enum fsa_function event);
int fsa4480_reg_notifier(struct notifier_block *nb,
			 struct device_node *node);
int fsa4480_unreg_notifier(struct notifier_block *nb,
			   struct device_node *node);
/* Add this func for AT CMD and failure detection by vivo audio team@fanyongxiang.
 * return -1: Not support;
 * return 0: No peripheral;
 * return 1: Insert peripheral.
 */
int get_usbc_peripheral_status(void);
/* Add this func for Mic and Gnd switch status to AT CMD by vivo audio team@fanyongxiang.
 * return -1: Not support;
 * return 1: MG_SR->GSNS;
 * return 2: GM_SR->GSNS.
 */
int get_usbc_mg_status(void);
#else
static inline int fsa4480_switch_event(struct device_node *node,
				       enum fsa_function event)
{
	return 0;
}

static inline int fsa4480_reg_notifier(struct notifier_block *nb,
				       struct device_node *node)
{
	return 0;
}

static inline int fsa4480_unreg_notifier(struct notifier_block *nb,
					 struct device_node *node)
{
	return 0;
}
#endif /* CONFIG_QCOM_FSA4480_I2C */

#endif /* FSA4480_I2C_H */

