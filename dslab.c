/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2018 Broadcom.  All Rights Reserved.
 *  The term "Broadcom" refers to Broadcom Limited and/or its subsidiaries.
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
 * Implements a DMA buffer slab allocator.   Its used by the OCS userspace
 * driver framework, but not tied to OCS specifically
 */

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <stdarg.h>
#include <sys/user.h>

#include "dslab.h"

#define roundup(x,y)	((((x) + (y) - 1) / (y)) * (y))
#define min(a,b)		((a) < (b) ? (a) : (b))

#define DSLAB_MIN_ITEM_LEN		64

static int dslab_entry_init(dslab_dir_t *dir, dslab_entry_t *entry, uint32_t item_len);
static void dslab_entry_free(dslab_entry_t *entry);
static int dslab_slab_new(dslab_entry_t *entry);
static void dslab_slab_del(dslab_t *dslab);
static int dslab_item_init(dslab_t *dslab, dslab_item_t *item, uintptr_t paddr, void *vaddr);
static dslab_entry_t *dslab_entry_find(dslab_dir_t *dir, uint32_t len);

static void *
zalloc(uint32_t len)
{
	void *p = malloc(len);
	if (p) {
		memset(p, 0, len);
	}
	return p;
}

dslab_dir_t *
dslab_dir_new(void *os, dslab_callbacks_t *callbacks, uint32_t entry_count, uint32_t max_item_len)
{
	dslab_dir_t *dir;
	uint32_t i;
	dslab_entry_t *entry;

	dir = zalloc(sizeof(*dir));
	if (dir != NULL) {
		dir->callbacks = callbacks;
		dir->entry_count = entry_count;
		dir->min_item_len = DSLAB_MIN_ITEM_LEN;
		dir->max_item_len = max_item_len;
		dir->os = os;
		dir->entries = zalloc(sizeof(dslab_entry_t) * entry_count);
		if (dir->entries == NULL) {
			free(dir);
			dir = NULL;
		}
		for (i = 0, entry = dir->entries; i < dir->entry_count; i ++, entry ++) {
			dslab_entry_init(dir, entry, 0);
		}
	}
	return dir;
}

void
dslab_dir_del(dslab_dir_t *dir)
{
	uint32_t i;
	dslab_entry_t *entry;
	/* Free all directory entries */

	if (dir != NULL) {
		for (i = 0, entry = dir->entries; i < dir->entry_count; i ++, entry ++) {
			dslab_entry_free(entry);
		}
		if (dir->entries != NULL) {
			free(dir->entries);
		}
		free(dir);
	}
}

static int
dslab_entry_init(dslab_dir_t *dir, dslab_entry_t *entry, uint32_t item_len)
{
	if (entry != NULL) {
		entry->dir = dir;
		entry->item_len = roundup(item_len, 16);
		ocs_list_init(&entry->dslab_list, dslab_t, link);
		ocs_list_init(&entry->free_list, dslab_item_t, link);
		ocs_list_init(&entry->inuse_list, dslab_item_t, link);
	}
	return 0;
}

static void
dslab_entry_free(dslab_entry_t *entry)
{
	dslab_t *dslab;
	dslab_t *next;
	if (entry) {
		ocs_list_foreach_safe(&entry->dslab_list, dslab, next) {
			dslab_slab_del(dslab);
		}
	}
}

static int
dslab_slab_new(dslab_entry_t *entry)
{
	dslab_t *dslab;
	dslab_dir_t *dir;
	dslab_item_t *item;
	uint32_t i;
	uintptr_t paddr;
	void *vaddr;
	uint32_t reqsize;
	int rc;

	assert(entry);
	assert(entry->dir);

	dir = entry->dir;
	reqsize = roundup(entry->item_len * 64, PAGE_SIZE);
	reqsize  = min(reqsize, 1*1024*1024);

	/* Allocate a dslab_t */
	dslab = zalloc(sizeof(*dslab));
	if (dslab == NULL) {
		printf("Error: %s: malloc failed\n", __func__);
		return -1;
	}
	dslab->entry = entry;
	dslab->item_len = entry->item_len;
	dslab->item_count = reqsize / dslab->item_len;

	/* Allocate DMA memory using from the os */
	rc = (*dir->callbacks->dslab_dmabuf_alloc)(dir->os, &dslab->dma, reqsize);
	if (rc) {
		printf("%s: dmabuf failed\n", __func__);
		dslab_slab_del(dslab);
		return -1;
	}

	dslab->items = zalloc(dslab->item_count * sizeof(*dslab->items));
	if (dslab->items == NULL) {
		printf("Error: %s: malloc failed\n", __func__);
		dslab_slab_del(dslab);
		return -1;
	}
	paddr = dslab->dma.paddr;
	vaddr = dslab->dma.vaddr;
	for (i = 0, item = dslab->items; i < dslab->item_count; i ++, item ++) {
		dslab_item_init(dslab, item, paddr, vaddr);
		paddr += dslab->item_len;
		vaddr += dslab->item_len;
	}

	ocs_list_add_tail(&entry->dslab_list, dslab);
	return 0;
}

static void
dslab_slab_del(dslab_t *dslab)
{
	dslab_dir_t *dir;

	assert(dslab);
	assert(dslab->entry);
	assert(dslab->entry->dir);

	dir = dslab->entry->dir;
	if (dslab->items) {
		free(dslab->items);
	}
	(*dir->callbacks->dslab_dmabuf_free)(dir->os, &dslab->dma);
	free(dslab);
}

static int
dslab_item_init(dslab_t *dslab, dslab_item_t *item, uintptr_t paddr, void *vaddr)
{
	dslab_entry_t *entry;

	assert(dslab);
	assert(dslab->entry);
	entry = dslab->entry;

	memset(item, 0, sizeof(*item));
	item->paddr = paddr;
	item->vaddr = vaddr;
	item->size = dslab->item_len;
	item->dslab = dslab;

	/* Add to the entry's free list */
	ocs_list_add_tail(&entry->free_list, item);

	return 0;
}

dslab_item_t *
dslab_item_new(dslab_dir_t *dir, uint32_t len)
{
	dslab_item_t *item = NULL;
	dslab_entry_t *entry;

	/* round up len to multiple of 16 */
	len = roundup(len, 16);

	/* adjust for minimum length */
	if (len < dir->min_item_len) {
		len = dir->min_item_len;
	}

	/* Find an entry that this request fits into */
	entry = dslab_entry_find(dir, len);

	if (entry != NULL) {
		if (ocs_list_empty(&entry->free_list)) {
			/* Add a new slab to this entry */
			if (dslab_slab_new(entry)) {
				printf("Error: %s: dslab_slab_alloc() failed\n", __func__);
				return NULL;
			}
		}
		item = ocs_list_get_head(&entry->free_list);
		ocs_list_remove(&entry->free_list, item);
		ocs_list_add_tail(&entry->inuse_list, item);
	}

	return item;
}

void
dslab_item_del(dslab_item_t *item)
{
	dslab_entry_t *entry;
	assert(item);
	assert(item->dslab);
	assert(item->dslab->entry);
	entry = item->dslab->entry;
	ocs_list_remove(&entry->inuse_list, item);
	ocs_list_add_tail(&entry->free_list, item);
}

static dslab_entry_t *
dslab_entry_find(dslab_dir_t *dir, uint32_t len)
{
	uint32_t i;
	dslab_entry_t *entry;
	int dif;
	int bestdif = 0;
	dslab_entry_t *best;

	/* Find an entry that this one fits in */
	for (i = 0, entry = dir->entries; i < dir->entry_count; i ++, entry ++) {
		if (len == entry->item_len) {
			return entry;
		}
	}

	/* Not found, so find an empty slot and use  it */
	for (i = 0, entry = dir->entries; i < dir->entry_count; i ++, entry ++) {
		if (entry->item_len == 0) {
			dslab_entry_init(dir, entry, len);
			return entry;
		}
	}

	/* Find best fit */
	best = NULL;
	for (i = 0, entry = dir->entries; i < dir->entry_count; i ++, entry ++) {
		if(entry->item_len >= len) {
			dif = entry->item_len - len;
			if ((best == NULL) || (dif < bestdif)) {
				bestdif = dif;
				best = entry;
			}
		}
	}

	if (best) {
		return best;
	}

	return NULL;
}

#define ENABLE_DUMP 0
#if ENABLE_DUMP
void indentpf(const char *fmt, ...);

void
dslab_item_dump(dslab_item_t *item)
{
	indentpf("+item[%3d] p %016lx v %p size %d\n", item - item->dslab->items, item->paddr, item->vaddr, item->size);
	indentpf("-");
}

void
dslab_dump(dslab_t *dslab)
{
	indentpf("dslab\n");
	indentpf("+item_len %d\n", dslab->item_len);
	indentpf("item_count %d\n", dslab->item_count);
	indentpf("paddr %ld\n", dslab->dma.paddr);
	indentpf("vaddr %p\n", dslab->dma.vaddr);
	indentpf("-");
}

void
indentpf(const char *fmt, ...)
{
	va_list ap;
	static int indent = 0;
	char idbuf[128];
	memset(idbuf, ' ', sizeof(idbuf));

	if (*fmt == '+') {
		indent += 4;
		fmt ++;
	} else if (*fmt == '-') {
		indent -= 4;
		if (indent < 0)
			indent = 0;
		fmt ++;
	}
	if (strlen(fmt) == 0)
		return;
	idbuf[indent] = 0;
	printf("%s", idbuf);
	va_start(ap, fmt);
	vprintf(fmt, ap);
	fflush(stdout);
}

void
dslab_entry_dump(dslab_entry_t *entry)
{
	dslab_t *dslab;

	assert(entry);
	assert(entry->dir);
	if (entry->item_len == 0)
		return;

	indentpf("Entry[%ld] item_len %d\n", entry - entry->dir->entries, entry->item_len);

	indentpf("+dslab_list\n");
	ocs_list_foreach(&entry->dslab_list, dslab) {
		dslab_dump(dslab);
	}

	indentpf("-");
}

void
dslab_dir_dump(dslab_dir_t *dir)
{
	int i;
	dslab_entry_t *entry;

	for (i = 0, entry = dir->entries; i < dir->entry_count; i ++, entry ++) {
		dslab_entry_dump(entry);
	}
}
#endif

#if defined(TEST)
static int test_dslab_alloc(void *os, dslab_dmabuf_t *dma, uint32_t len);
static void test_dslab_free(void *os, dslab_dmabuf_t *dma);
static dslab_callbacks_t test_callbacks = {
	test_dslab_alloc,
	test_dslab_free,
};


static int
test_dslab_alloc(void *os, dslab_dmabuf_t *dma, uint32_t len)
{
	dma->vaddr = zalloc(len);
	dma->paddr = 0;
	dma->size = len;
	return 0;
}

static void
test_dslab_free(void *os, dslab_dmabuf_t *dma)
{
	if (dma->vaddr) {
		free(dma->vaddr);
	}
}

int main(int argc, char *argv[])
{
	dslab_dir_t *dir;
	dslab_item_t *item[3];

	dir = dslab_dir_new(NULL, &test_callbacks, 16, 1*1024*1024);
	if (dir == NULL) {
		printf ("dslab_dir_new failed\n");
	}
	item[0] = dslab_item_new(dir, 128);
	if (item[0] == NULL) printf("dslab_item_alloc() failed\n");
	item[1] = dslab_item_new(dir, 272);
	if (item[1] == NULL) printf("dslab_item_alloc() failed\n");
	item[2] = dslab_item_new(dir, 128);
	if (item[2] == NULL) printf("dslab_item_alloc() failed\n");
	dslab_item_del(item[0]);
	dslab_item_del(item[1]);
	dslab_item_del(item[2]);
	dslab_dir_dump(dir);

	dslab_dir_del(dir);

	return 0;
}
#endif

