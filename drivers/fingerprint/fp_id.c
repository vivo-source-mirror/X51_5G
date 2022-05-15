#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include "fp_id.h"

#define MAX_TIMES		7

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};

struct fp_id_data_t *fp_id_data_g;

/*
 *static const struct vreg_config const vreg_conf[] = {
 *	{ "vcc_fpc", 1800000UL, 1800000UL, 10, },
 *	{ "vcc_goodix", 2800000UL, 2800000UL, 10, },
 *};
 */

static const char * const pctl_names[] = {
	"fp_id_gpio_up",
	"fp_id_gpio_down",
	"fp_id_gpio_suspend"
};

enum pctl_index {
	FP_ID_PULL_UP = 0,
	FP_ID_PULL_DOWN,
	FP_ID_PULL_SUSPEND
};

struct fp_id_data_t {
	struct platform_device *pdev;
	int fp_gpio;
	int fp_id;
	int fp_id_value;
	bool fp_id_use_gpio;
	char fp_project_name[20];

	struct pinctrl *fp_pinctrl;
	struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];
};

#define DEVFS_MODE_RO (S_IRUSR|S_IRGRP|S_IROTH)
struct attribute fp_id_attr = {
	.name = "fp_id",
	.mode = DEVFS_MODE_RO,
};
static struct attribute *our_own_sys_attrs[] = {
	&fp_id_attr,
	NULL,
};

static int select_pin_ctl(struct fp_id_data_t *fp_id_data, const char *name)
{
	size_t i;
	int rc;

	for (i = 0; i < ARRAY_SIZE(fp_id_data->pinctrl_state); i++) {
		const char *n = pctl_names[i];

		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fp_id_data->fp_pinctrl, fp_id_data->pinctrl_state[i]);
			if (rc)
				dev_err(&fp_id_data->pdev->dev, "bio_fp_error cannot select '%s'\n", name);
			else
				dev_dbg(&fp_id_data->pdev->dev, "Selected '%s'\n", name);
			goto exit;
		}
	}
	rc = -EINVAL;
	dev_err(&fp_id_data->pdev->dev, "bio_fp_error %s:'%s' not found\n", __func__, name);
exit:
	return rc;
}

static ssize_t fp_id_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	char *fp_frame_id;

	if (fp_id_data_g->fp_id == FPC_FPC1022)
		fp_frame_id = "fpc_1022";
	else if (fp_id_data_g->fp_id == FPC_FPC1245)
		fp_frame_id = "fpc_1245";
	else if (fp_id_data_g->fp_id == GOODIX_GF5116M)
		fp_frame_id = "goodix_5116";
	else if (fp_id_data_g->fp_id == GOODIX_GF52X6)
		fp_frame_id = "goodix_5216";
	else if (fp_id_data_g->fp_id == GOODIX_GF318M)
		fp_frame_id = "goodix_318m";
	else if (fp_id_data_g->fp_id == GOODIX_GF3208)
		fp_frame_id = "goodix_3208b";
	else if (fp_id_data_g->fp_id == GOODIX_GF5269)
		fp_frame_id = "goodix_5269";
	else if (fp_id_data_g->fp_id == GOODIX_GF5288)
		fp_frame_id = "goodix_5288";
	else if (fp_id_data_g->fp_id == GOODIX_GF9518)
		fp_frame_id = "udfp_goodix_gf9518";
	else if (GOODIX_GF3658 == fp_id_data_g->fp_id)
		fp_frame_id = "goodix_3658";
	else if (GOODIX_GF3626 == fp_id_data_g->fp_id)
		fp_frame_id = "sidefp_goodix_3626";
	else if (FPC_FPC1540 == fp_id_data_g->fp_id)
		fp_frame_id = "sidefp_fpc_1540";
	else if (fp_id_data_g->fp_id == GOODIX_GF9518N)
		fp_frame_id = "udfp_goodix2_gf9518";
	else if (fp_id_data_g->fp_id == SYNAPTICS_FS9501)
		fp_frame_id = "udfp_syna_fs9501";
	else
		fp_frame_id = "default";

	printk("fp_project_name:%s, fp_gpio=%d, fp_id_value=%d, fp_id=%d, fp_frame_id=%s\n", \
			fp_id_data_g->fp_project_name, fp_id_data_g->fp_gpio, fp_id_data_g->fp_id_value, \
			fp_id_data_g->fp_id, fp_frame_id);
	return snprintf(buf, strlen(fp_frame_id)+2, "%s\n", fp_frame_id);
}

static ssize_t fp_id_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	/* nothing to do temply */
	printk("fp_id cannot be writed.\n");
	return 0;
}

int get_fp_id(void)
{
	return fp_id_data_g->fp_id;
}
EXPORT_SYMBOL(get_fp_id);

static void fp_id_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static const struct sysfs_ops fp_id_object_sysfs_ops = {
	.show = fp_id_object_show,
	.store = fp_id_object_store,
};
static struct kobj_type fp_id_object_type = {
	.sysfs_ops	= &fp_id_object_sysfs_ops,
	.release	= fp_id_object_release,
	.default_attrs = our_own_sys_attrs,
};

struct kobject kobj;

static int fp_id_parse_dts(struct fp_id_data_t *fp_id_data)
{
	int ret = 0;
	int i = 0;
	const char *fp_project_name_temp;

	ret = of_property_read_string(fp_id_data->pdev->dev.of_node, "vivo,project-name", &fp_project_name_temp);
	if (ret) {
		printk("%s:vivo,project-name property do not find\n", __func__);
		snprintf(fp_id_data->fp_project_name, strlen("dafult"), "dafult");
		return -ERRORFP;
	} else {
		snprintf(fp_id_data->fp_project_name, sizeof(fp_project_name_temp), "%s", fp_project_name_temp);
		printk("%s:vivo,project-name is %s\n", __func__, fp_id_data->fp_project_name);
	}

	fp_id_data->fp_id_use_gpio = of_property_read_bool(fp_id_data->pdev->dev.of_node, "fp-id-use-gpio");
	printk("%s: fp_id_detect is %s\n", __func__, fp_id_data->fp_id_use_gpio ? "true" : "false");

	if (fp_id_data->fp_id_use_gpio) {
		fp_id_data->fp_pinctrl = devm_pinctrl_get(&fp_id_data->pdev->dev);
		if (IS_ERR(fp_id_data->fp_pinctrl)) {
			if (PTR_ERR(fp_id_data->fp_pinctrl) == -EPROBE_DEFER) {
				printk("%s: fp_pinctrl not ready!\n", __func__);
			} else {
				fp_id_data->fp_pinctrl = NULL;
			}
			return -ERRORFP;
		}

		for (i = 0; i < ARRAY_SIZE(fp_id_data->pinctrl_state); i++) {
			fp_id_data->pinctrl_state[i] = 	pinctrl_lookup_state(fp_id_data->fp_pinctrl, pctl_names[i]);
			if (IS_ERR(fp_id_data->pinctrl_state[i])) {
				printk("%s: cannot find '%s'\n", __func__, pctl_names[i]);
				return -ERRORFP;
			} else {
				printk("%s: found pin control %s\n", __func__, pctl_names[i]);
			}
		}

		fp_id_data->fp_gpio = of_get_named_gpio(fp_id_data->pdev->dev.of_node, "fp_id,gpio", 0);
		if (fp_id_data->fp_gpio < 0) {
			printk("%s: get fp_id gpio failed!\n", __func__);
			return -ERRORFP;
		} else {
			printk("%s:fp gpio: %d \n", __func__, fp_id_data->fp_gpio);
		}
	}

	return 0;
}

static int fp_id_value_cal(struct fp_id_data_t *fp_id_data)
{
	int ret = 0;
	int fp_id_pullup = 0;
	int fp_id_pulldown = 0;

	ret = devm_gpio_request(&fp_id_data->pdev->dev, fp_id_data->fp_gpio, "fp_id, gpio");
	if (ret) {
		printk("%s: request fp_id gpio failed!\n", __func__);
		goto err_devm_gpio_request;
	}

	ret = select_pin_ctl(fp_id_data, pctl_names[FP_ID_PULL_UP]);
	mdelay(5);
	if (ret) {
		printk("%s: select_pin_ctl %s failed\n", __func__, pctl_names[FP_ID_PULL_UP]);
		goto err_pin_ctl;
	} else {
		fp_id_pullup = gpio_get_value(fp_id_data->fp_gpio);
		printk("%s: fp_id(%s) is %d \n", __func__, pctl_names[FP_ID_PULL_DOWN], fp_id_pullup);
	}

	ret = select_pin_ctl(fp_id_data, pctl_names[FP_ID_PULL_DOWN]);
	mdelay(5);
	if (ret) {
		printk("%s: select_pin_ctl %s failed\n", __func__, pctl_names[FP_ID_PULL_DOWN]);
		goto err_pin_ctl;
	} else {
		fp_id_pulldown = gpio_get_value(fp_id_data->fp_gpio);
		printk("%s: fp_id(%s) is %d \n", __func__, pctl_names[FP_ID_PULL_DOWN], fp_id_pulldown);
	}

	ret = select_pin_ctl(fp_id_data, pctl_names[FP_ID_PULL_SUSPEND]);
	if (ret) {
		printk("%s: select_pin_ctl %s failed\n", __func__, pctl_names[FP_ID_PULL_SUSPEND]);
	}

	devm_gpio_free(&fp_id_data->pdev->dev, fp_id_data->fp_gpio);

	printk("%s: fp_id is %d \n", __func__, fp_id_pullup + fp_id_pulldown);

	return fp_id_pullup + fp_id_pulldown;

err_pin_ctl:
	devm_gpio_free(&fp_id_data->pdev->dev, fp_id_data->fp_gpio);

err_devm_gpio_request:
	return -ERRORFP;
}

static void fp_id_detect(struct fp_id_data_t *fp_id_data)
{
	if (!strncmp(fp_id_data->fp_project_name, "TD1907", 6)) {
		fp_id_data->fp_id = GOODIX_GF9518;
		printk("%s: return gf9518  directly\n", __func__);
	}

	if (!strncmp(fp_id_data->fp_project_name, "PD1968F_EX", 6)) {
		fp_id_data->fp_id = GOODIX_GF9518;
		printk("%s: return gf9518  directly\n", __func__);
	}

	if (!strncmp(fp_id_data->fp_project_name, "PD1963", 6)) {
		fp_id_data_g->fp_id = GOODIX_GF3658;
		printk("%s: return gf3658  directly\n", __func__);
	}

	if (!strncmp(fp_id_data->fp_project_name, "PD2001", 6)) {
		fp_id_data_g->fp_id = GOODIX_GF9518;
		printk("%s: return gf9518  directly\n", __func__);
	}

	if (!strncmp(fp_id_data->fp_project_name, "PD2005", 6)) {
		fp_id_data_g->fp_id = GOODIX_GF9518;
		printk("%s: return gf9518  directly\n", __func__);
	}

	if (!strncmp(fp_id_data->fp_project_name, "TD1907", 6)) {
		if (fp_id_data->fp_id_value == 1) {
			fp_id_data->fp_id = GOODIX_GF9518;
		} else if (fp_id_data->fp_id_value == 2) {
			fp_id_data->fp_id = GOODIX_GF9518N;
		}

		printk("%s: return gf9518  directly\n", __func__);
	}
	if (!strncmp(fp_id_data->fp_project_name, "PD2012", 6)) {
		if (fp_id_data->fp_id_value == 0) {
			fp_id_data->fp_id = GOODIX_GF3626;
			printk("%s: return goodix directly\n", __func__);
		} else {
			fp_id_data->fp_id = FPC_FPC1540;
			printk("%s: return fpc directly\n", __func__);
		}
	}
}

static void fp_id_data_init(struct fp_id_data_t *fp_id_data)
{
	fp_id_data->fp_gpio = -1;
	fp_id_data->fp_id = -1;
	fp_id_data->fp_id_use_gpio = false;
	fp_id_data->fp_id_value = -1;
	fp_id_data_g->fp_id = -1;
}

static int
fp_id_probe(struct platform_device *pdev)
{
	int ret;
	struct fp_id_data_t *fp_id_data = NULL;

	printk("%s: fp_id probe\n", __func__);

	fp_id_data = kmalloc(sizeof(struct fp_id_data_t), GFP_KERNEL);
	if (!fp_id_data) {
		ret = -ERRORFP;
		goto err_malloc_fp_id_data;
	} else {
		memset(fp_id_data, 0, sizeof(struct fp_id_data_t));
		fp_id_data->pdev = pdev;
		fp_id_data_g = fp_id_data;
	}

	fp_id_data_init(fp_id_data);

	ret = fp_id_parse_dts(fp_id_data);
	if (ret) {
		printk("%s: fp_id_parse_dts failed\n", __func__);
		goto err_fp_id_parse_dts;
	}

	if (fp_id_data->fp_id_use_gpio) {
		fp_id_data->fp_id_value = fp_id_value_cal(fp_id_data);
	}

	fp_id_detect(fp_id_data);

	ret = kobject_init_and_add(&kobj, &fp_id_object_type, NULL, "fp_id");
	if (ret) {
		printk("%s: Create fp_id error!\n", __func__);
		goto err_fp_id_parse_dts;
	}

	printk("%s: fp_id probe success\n", __func__);

	return 0;

err_fp_id_parse_dts:
	kfree(fp_id_data);

err_malloc_fp_id_data:

	printk("%s: fp_id probe failed\n", __func__);

	return 	ret;

}

static int fp_id_remove(struct platform_device *pdev)
{
	printk("fp_id  remove.\n");
	kobject_del(&kobj);
    return 0;
}

static struct of_device_id fp_id_match_table[] = {
	{ .compatible = "fp-id",},
	{},
};

static struct platform_driver fp_id_driver = {
    .probe      = fp_id_probe,
    .remove     = fp_id_remove,
    .driver = {
		.name   = "fp_id",
		.owner  = THIS_MODULE,
		.of_match_table = fp_id_match_table,
	},
};

static int __init fp_id_init(void)
{
    return platform_driver_register(&fp_id_driver);
}
module_init(fp_id_init);

static void __exit fp_id_exit(void)
{
    platform_driver_unregister(&fp_id_driver);

}
module_exit(fp_id_exit);

MODULE_AUTHOR("Xiaot BBK Corporation");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
