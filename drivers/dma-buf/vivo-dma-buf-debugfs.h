/*
 * drivers/dma-buf/vivo-dma-buf-debugfs.h
 *
 * VIVO Resource Control.
 *
 * stat task cpu usage.
 *
 * Copyright (C) 2017 VIVO Technology Co., Ltd
 */

#ifndef _VIVO_DMA_BUF_DEBUGFS_H
#define _VIVO_DMA_BUF_DEBUGFS_H

#include <linux/dma-buf.h>

#ifdef CONFIG_VIVO_DMA_BUF_DEBUGFS
int vivo_dma_buf_init_debugfs(void);
void vivo_dma_buf_uninit_debugfs(void);
inline void on_dma_buf_export_locked(const struct dma_buf *dmabuf);
inline void on_dma_buf_release_locked(const struct dma_buf *dmabuf);
#else
static int vivo_dma_buf_init_debugfs(void) { return 0; }
static void vivo_dma_buf_uninit_debugfs(void) { }
static inline void on_dma_buf_export_locked(const struct dma_buf *dmabuf) { }
static inline void on_dma_buf_release_locked(const struct dma_buf *dmabuf) { }
#endif

#endif