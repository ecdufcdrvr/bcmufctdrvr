/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2018 Broadcom.  All Rights Reserved.
 *  The term "Broadcom" refers to Broadcom Inc. and/or its subsidiaries.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *    * Neither the name of Intel Corporation nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * Exported interface to the DSLAB component
 *
 */

#if !defined(__DSLAB_H__)
#define __DSLAB_H__

#include "ocs_list.h"

typedef struct dslab_item_s dslab_item_t;
typedef struct dslab_s dslab_t;
typedef struct dslab_entry_s dslab_entry_t;
typedef struct dslab_dir_s dslab_dir_t;

typedef struct {
	/* public */
	uintptr_t paddr;
	void *vaddr;
	uint32_t size;
	/* application private */
	void *app;
} dslab_dmabuf_t;

typedef struct {
	int (*dslab_dmabuf_alloc)(void *os, dslab_dmabuf_t *dma, uint32_t len);
	void (*dslab_dmabuf_free)(void *os, dslab_dmabuf_t *dma);
} dslab_callbacks_t;

struct dslab_item_s {
	/* DMA buffer */
	intptr_t paddr;			/*<< physical address of this item */
	void *vaddr;			/*<< virtual address of this item */
	uint32_t size;			/*<< size of this item */

	ocs_list_link_t link;		/*<< item link */
	dslab_t *dslab;			/*<< pointer back to parent dslab */
};

struct dslab_s {
	dslab_entry_t *entry;		/*<< pointer to parent entry */

	/* Dimensions */
	uint32_t item_len;		/*<< this slabs element length in bytes */
	uint32_t item_count;		/*<< count of items in this slab */
	dslab_item_t *items;		/*<< pointer to array of item_len items */

	/* DMA buffer */
	dslab_dmabuf_t dma;

	ocs_list_link_t link;		/*<< link for this slab */

};

struct dslab_entry_s {
	dslab_dir_t *dir;		/*<< pointer to parent directory */
	uint32_t item_len;		/*<< length of items in this directory entry */
	ocs_list_t dslab_list;		/*<< pointer to dslab list */
	ocs_list_t free_list;		/*<< list of free items */
	ocs_list_t inuse_list;		/*<< list of free items */
};

struct dslab_dir_s {
	void *os;
	dslab_callbacks_t *callbacks;
	uint32_t entry_count;
	uint32_t min_item_len;
	uint32_t max_item_len;
	dslab_entry_t *entries;
};

extern dslab_dir_t *dslab_dir_new(void *os, dslab_callbacks_t *callbacks, uint32_t entry_count, uint32_t max_item_len);
extern void dslab_dir_del(dslab_dir_t *dir);
extern dslab_item_t *dslab_item_new(dslab_dir_t *dir, uint32_t len);
extern void dslab_item_del(dslab_item_t *item);
extern void dslab_item_dump(dslab_item_t *item);
extern void dslab_dump(dslab_t *dslab);
extern void dslab_entry_dump(dslab_entry_t *entry);
extern void dslab_dir_dump(dslab_dir_t *dir);

#endif // __DSLAB_H__
