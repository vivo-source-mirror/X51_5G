/*
 * drivers/dma-buf/dma-buf-common.h
 *
 * VIVO Resource Control.
 *
 * stat task cpu usage.
 *
 * Copyright (C) 2017 VIVO Technology Co., Ltd
 */

#ifndef _DMA_BUF_COMMON_H
#define _DMA_BUF_COMMON_H

#include <linux/atomic.h>
#include <linux/dma-buf.h>
#include <linux/kernel.h>
#include <linux/list_sort.h>

struct dma_buf_list {
	struct list_head head;
	struct mutex lock;
};

struct dma_info {
	struct dma_buf *dmabuf;
	struct hlist_node head;
};

struct dma_proc {
	char name[TASK_COMM_LEN];
	pid_t pid;
	size_t size;
	struct hlist_head dma_bufs[1 << 10];
	struct list_head head;
};

inline int is_dma_buf_file_outer(struct file *file);

extern struct dma_buf_list db_list;

#endif