// SPDX-License-Identifier: GPL-2.0
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#ifdef CONFIG_VIVO_LINUXBSP_EDIT
extern char qpnp_pon_reason_str[];
extern char qpnp_poff_reason_str[];
extern char *vivo_get_pmic_status(void);
static char vivo_cmdline_ext[1024];

static int cmdline_proc_show(struct seq_file *m, void *v)
{
	snprintf(vivo_cmdline_ext, sizeof(vivo_cmdline_ext),
			" cmdline.max=4096 androidboot.bootreason=%s/%s/%s",
			qpnp_pon_reason_str,
			qpnp_poff_reason_str,
			vivo_get_pmic_status());

	seq_puts(m, saved_command_line);
	seq_puts(m, vivo_cmdline_ext);
	seq_putc(m, '\n');
	return 0;
}
#else
static int cmdline_proc_show(struct seq_file *m, void *v)
{
	seq_puts(m, saved_command_line);
	seq_putc(m, '\n');
	return 0;
}
#endif	/*CONFIG_VIVO_LINUXBSP_EDIT*/

static int __init proc_cmdline_init(void)
{
	proc_create_single("cmdline", 0, NULL, cmdline_proc_show);
	return 0;
}
fs_initcall(proc_cmdline_init);
