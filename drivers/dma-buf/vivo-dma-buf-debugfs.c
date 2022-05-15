/*
 * drivers/dma-buf/vivo-dma-buf-debugfs.c
 *
 * VIVO Resource Control.
 *
 * stat task cpu usage.
 *
 * Copyright (C) 2017 VIVO Technology Co., Ltd
 */

#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/dma-buf.h>
#include <linux/dma-fence.h>
#include <linux/anon_inodes.h>
#include <linux/export.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/poll.h>
#include <linux/reservation.h>
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/atomic.h>
#include <linux/sched/signal.h>
#include <linux/fdtable.h>
#include <linux/list_sort.h>
#include <linux/hashtable.h>
#include <uapi/linux/dma-buf.h>
#include <linux/kthread.h>
#include <linux/ctype.h>
#include "dma-buf-common.h"
#include "vivo-dma-buf-debugfs.h"

#define LOG_TO_KMSG

#ifdef LOG_TO_KMSG
#define MAX_SIZE_TO_LOG		(1024*1024*1024)
#define MAX_LOG_INTERVAL	(5*60*HZ)
#define TAG     			"vivo-dma-buf: "
#define dma_buf_info(fmt, args...)		\
	pr_warn(TAG""fmt, ##args)

static size_t current_size;
static DECLARE_WAIT_QUEUE_HEAD(loging_wait);
static struct task_struct *loging_tsk;
static unsigned long last_log_jiffies;
static bool log_request_from_rms;

static inline void log_to_kmsg_anyway(void)
{
	log_request_from_rms = true;
	wake_up_interruptible(&loging_wait);
}
static inline void log_to_kmsg_if_need(void)
{
	if (time_before_eq(jiffies, last_log_jiffies + MAX_LOG_INTERVAL) || current_size < MAX_SIZE_TO_LOG)
		return;

	wake_up_interruptible(&loging_wait);
}

inline void on_dma_buf_export_locked(const struct dma_buf *dmabuf)
{
	current_size += dmabuf->size;
	log_to_kmsg_if_need();
}

inline void on_dma_buf_release_locked(const struct dma_buf *dmabuf)
{
	current_size -= dmabuf->size;
}

#else
inline void on_dma_buf_export_locked(const struct dma_buf *dmabuf) { }
inline void on_dma_buf_release_locked(const struct dma_buf *dmabuf) { }
#endif

static int dma_buf_debug_show(struct seq_file *s, void *unused)
{
	int ret;
	struct dma_buf *buf_obj;
	struct dma_buf_attachment *attach_obj;
	struct reservation_object *robj;
	struct reservation_object_list *fobj;
	struct dma_fence *fence;
	unsigned seq;
	int count = 0, attach_count, shared_count, i;
	size_t size = 0;

	ret = mutex_lock_interruptible(&db_list.lock);
	if (ret)
		return ret;

	seq_puts(s, "\nDma-buf Objects:\n");
	seq_printf(s, "%-8s\t%-8s\t%-8s\t%-8s\t%-12s\t%-s\n",
		   "size", "flags", "mode", "count", "exp_name", "buf name");

	list_for_each_entry(buf_obj, &db_list.head, list_node) {
		ret = mutex_lock_interruptible(&buf_obj->lock);

		if (ret) {
			seq_puts(s,
				 "\tERROR locking buffer object: skipping\n");
			continue;
		}

		seq_printf(s, "%08zu\t%08x\t%08x\t%08ld\t%-12s\t%-s\n",
				buf_obj->size,
				buf_obj->file->f_flags, buf_obj->file->f_mode,
				file_count(buf_obj->file),
				buf_obj->exp_name, buf_obj->name);

		robj = buf_obj->resv;
		while (true) {
			seq = read_seqcount_begin(&robj->seq);
			rcu_read_lock();
			fobj = rcu_dereference(robj->fence);
			shared_count = fobj ? fobj->shared_count : 0;
			fence = rcu_dereference(robj->fence_excl);
			if (!read_seqcount_retry(&robj->seq, seq))
				break;
			rcu_read_unlock();
		}

		if (fence)
			seq_printf(s, "\tExclusive fence: %s %s %ssignalled\n",
				   fence->ops->get_driver_name(fence),
				   fence->ops->get_timeline_name(fence),
				   dma_fence_is_signaled(fence) ? "" : "un");
		for (i = 0; i < shared_count; i++) {
			fence = rcu_dereference(fobj->shared[i]);
			if (!dma_fence_get_rcu(fence))
				continue;
			seq_printf(s, "\tShared fence: %s %s %ssignalled\n",
				   fence->ops->get_driver_name(fence),
				   fence->ops->get_timeline_name(fence),
				   dma_fence_is_signaled(fence) ? "" : "un");
			dma_fence_put(fence);
		}
		rcu_read_unlock();

		seq_puts(s, "\tAttached Devices:\n");
		attach_count = 0;

		list_for_each_entry(attach_obj, &buf_obj->attachments, node) {
			seq_printf(s, "\t%s\n", dev_name(attach_obj->dev));
			attach_count++;
		}

		seq_printf(s, "Total %d devices attached\n\n",
				attach_count);

		dma_buf_ref_show(s, buf_obj);

		count++;
		size += buf_obj->size;
		mutex_unlock(&buf_obj->lock);
	}

	seq_printf(s, "\nTotal %d objects, %zu bytes\n", count, size);

	mutex_unlock(&db_list.lock);
	return 0;
}

static int dma_buf_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, dma_buf_debug_show, NULL);
}

static const struct file_operations dma_buf_debug_fops = {
	.open           = dma_buf_debug_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int get_dma_info(const void *data, struct file *file, unsigned int n)
{
	struct dma_proc *dma_proc;
	struct dma_info *dma_info;

	dma_proc = (struct dma_proc *)data;
	if (!is_dma_buf_file_outer(file))
		return 0;

	hash_for_each_possible(dma_proc->dma_bufs, dma_info,
			       head, (unsigned long)file->private_data)
		if (file->private_data == dma_info->dmabuf)
			return 0;

	dma_info = kzalloc(sizeof(*dma_info), GFP_ATOMIC);
	if (!dma_info)
		return -ENOMEM;

	get_file(file);
	dma_info->dmabuf = file->private_data;
	dma_proc->size += dma_info->dmabuf->size / SZ_1K;
	hash_add(dma_proc->dma_bufs, &dma_info->head,
			(unsigned long)dma_info->dmabuf);
	return 0;
}

static void write_proc(struct seq_file *s, struct dma_proc *proc)
{
	struct dma_info *tmp;
	int i;

	seq_printf(s, "\n%s (PID %d) size: %ld\nDMA Buffers:\n",
		proc->name, proc->pid, proc->size);
	seq_printf(s, "%-8s\t%-8s\t%-8s\n",
		"Name", "Size (KB)", "Time Alive (sec)");

	hash_for_each(proc->dma_bufs, i, tmp, head) {
		struct dma_buf *dmabuf = tmp->dmabuf;
		ktime_t elapmstime = ktime_ms_delta(ktime_get(), dmabuf->ktime);

		elapmstime = ktime_divns(elapmstime, MSEC_PER_SEC);
		seq_printf(s, "%-8s\t%-8ld\t%-8lld\n",
				dmabuf->name,
				dmabuf->size / SZ_1K,
				elapmstime);
	}
}

static void free_proc(struct dma_proc *proc)
{
	struct dma_info *tmp;
	struct hlist_node *n;
	int i;

	hash_for_each_safe(proc->dma_bufs, i, n, tmp, head) {
		fput(tmp->dmabuf->file);
		hash_del(&tmp->head);
		kfree(tmp);
	}
	kfree(proc);
}

static int proccmp(void *unused, struct list_head *a, struct list_head *b)
{
	struct dma_proc *a_proc, *b_proc;

	a_proc = list_entry(a, struct dma_proc, head);
	b_proc = list_entry(b, struct dma_proc, head);
	return b_proc->size - a_proc->size;
}

static ssize_t dma_procs_debug_write(struct file *file, char const __user *buf,
			size_t count, loff_t *offset)
{
#ifdef LOG_TO_KMSG
	char buffer[10];
	char *magic_buf;
	int magic;
	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	magic_buf = strstrip(buffer);

	if (isdigit(*magic_buf)) {
		sscanf(magic_buf, "%u", &magic);
		if (magic == 114109115) {
			log_to_kmsg_anyway();
		}
	}
#endif
	return count;
}
static int dma_procs_debug_show(struct seq_file *s, void *unused)
{
	struct task_struct *task, *thread;
	struct files_struct *files;
	int ret = 0;
	struct dma_proc *tmp, *n;
	LIST_HEAD(plist);

	rcu_read_lock();
	for_each_process(task) {
		struct files_struct *group_leader_files = NULL;

		tmp = kzalloc(sizeof(*tmp), GFP_ATOMIC);
		if (!tmp) {
			ret = -ENOMEM;
			rcu_read_unlock();
			goto mem_err;
		}
		hash_init(tmp->dma_bufs);
		for_each_thread(task, thread) {
			task_lock(thread);
			if (unlikely(!group_leader_files))
				group_leader_files = task->group_leader->files;
			files = thread->files;
			if (files && (group_leader_files != files ||
				      thread == task->group_leader))
				ret = iterate_fd(files, 0, get_dma_info, tmp);
			task_unlock(thread);
		}
		if (ret || hash_empty(tmp->dma_bufs))
			goto skip;
		get_task_comm(tmp->name, task);
		tmp->pid = task->tgid;
		list_add(&tmp->head, &plist);
		continue;
skip:
		free_proc(tmp);
	}
	rcu_read_unlock();

	list_sort(NULL, &plist, proccmp);
	list_for_each_entry(tmp, &plist, head)
		write_proc(s, tmp);

	ret = 0;
mem_err:
	list_for_each_entry_safe(tmp, n, &plist, head) {
		list_del(&tmp->head);
		free_proc(tmp);
	}
	return ret;
}

static int dma_procs_debug_open(struct inode *f_inode, struct file *file)
{
	return single_open(file, dma_procs_debug_show, NULL);
}

static const struct file_operations dma_procs_debug_fops = {
	.open           = dma_procs_debug_open,
	.read           = seq_read,
	.write          = dma_procs_debug_write,
	.llseek         = seq_lseek,
	.release        = single_release
};

static struct dentry *dma_buf_debugfs_dir;

#ifdef LOG_TO_KMSG
static void write_proc_to_kmsg(struct dma_proc *proc)
{
	struct dma_info *tmp;
	int i;

	dma_buf_info("\n");
	dma_buf_info("%s (PID %d) size: %ld:\n",
		proc->name, proc->pid, proc->size);
	dma_buf_info("%-8s    %-8s    %-8s\n",
		"Name", "Size (KB)", "Time Alive (sec)");

	hash_for_each(proc->dma_bufs, i, tmp, head) {
		struct dma_buf *dmabuf = tmp->dmabuf;
		ktime_t elapmstime = ktime_ms_delta(ktime_get(), dmabuf->ktime);

		elapmstime = ktime_divns(elapmstime, MSEC_PER_SEC);
		dma_buf_info("%-8s    %-8ld    %-8lld\n",
				dmabuf->name,
				dmabuf->size / SZ_1K,
				elapmstime);
	}
}

static int do_loging(void)
{
	struct task_struct *task, *thread;
	struct files_struct *files;
	int ret = 0;
	struct dma_proc *tmp, *n;
	LIST_HEAD(plist);

	dma_buf_info("Total dma-buf size %zu\n", current_size);

	rcu_read_lock();
	for_each_process(task) {
		struct files_struct *group_leader_files = NULL;

		tmp = kzalloc(sizeof(*tmp), GFP_ATOMIC);
		if (!tmp) {
			ret = -ENOMEM;
			rcu_read_unlock();
			goto mem_err;
		}
		hash_init(tmp->dma_bufs);
		for_each_thread(task, thread) {
			task_lock(thread);
			if (unlikely(!group_leader_files))
				group_leader_files = task->group_leader->files;
			files = thread->files;
			if (files && (group_leader_files != files ||
				      thread == task->group_leader))
				ret = iterate_fd(files, 0, get_dma_info, tmp);
			task_unlock(thread);
		}
		if (ret || hash_empty(tmp->dma_bufs))
			goto skip;
		get_task_comm(tmp->name, task);
		tmp->pid = task->tgid;
		list_add(&tmp->head, &plist);
		continue;
skip:
		free_proc(tmp);
	}
	rcu_read_unlock();

	list_sort(NULL, &plist, proccmp);
	list_for_each_entry(tmp, &plist, head)
		write_proc_to_kmsg(tmp);

	ret = 0;
mem_err:
	list_for_each_entry_safe(tmp, n, &plist, head) {
		list_del(&tmp->head);
		free_proc(tmp);
	}
	return ret;
}
static int loging_funcion(void *arg)
{
	for (;;) {
		wait_event_interruptible(loging_wait,
			(log_request_from_rms || (time_after_eq(jiffies, last_log_jiffies + MAX_LOG_INTERVAL)
				&& current_size >= MAX_SIZE_TO_LOG))
					|| kthread_should_stop());
		if (kthread_should_stop())
			break;
		log_request_from_rms = false;
		last_log_jiffies = jiffies;
		do_loging();
	}
	return 0;
}

static int create_loging_thread(void)
{
	int ret;
	loging_tsk = kthread_create(loging_funcion, NULL, "dma-buf-log");
	if (IS_ERR(loging_tsk)) {
		ret = PTR_ERR(loging_tsk);
		loging_tsk = NULL;
		return ret;
	}
	wake_up_process(loging_tsk);
	last_log_jiffies = jiffies;
	return 0;
}

static void destroy_loging_thread(void)
{
	if (loging_tsk)
		kthread_stop(loging_tsk);
}
#endif

int vivo_dma_buf_init_debugfs(void)
{
	struct dentry *d;
	int err = 0;

	d = debugfs_create_dir("vivo_dma_buf", NULL);
	if (IS_ERR(d))
		return PTR_ERR(d);

	dma_buf_debugfs_dir = d;

	d = debugfs_create_file("bufinfo", S_IRUGO, dma_buf_debugfs_dir,
				NULL, &dma_buf_debug_fops);
	if (IS_ERR(d)) {
		pr_debug("dma_buf: debugfs: failed to create node bufinfo\n");
		debugfs_remove_recursive(dma_buf_debugfs_dir);
		dma_buf_debugfs_dir = NULL;
		err = PTR_ERR(d);
		return err;
	}

	d = debugfs_create_file("dmaprocs", S_IRWXUGO, dma_buf_debugfs_dir,
				NULL, &dma_procs_debug_fops);

	if (IS_ERR(d)) {
		pr_debug("dma_buf: debugfs: failed to create node dmaprocs\n");
		debugfs_remove_recursive(dma_buf_debugfs_dir);
		dma_buf_debugfs_dir = NULL;
		err = PTR_ERR(d);
	}

#ifdef LOG_TO_KMSG
	err = create_loging_thread();
#endif
	return err;
}

void vivo_dma_buf_uninit_debugfs(void)
{
#ifdef LOG_TO_KMSG
	destroy_loging_thread();
#endif
	debugfs_remove_recursive(dma_buf_debugfs_dir);
}
