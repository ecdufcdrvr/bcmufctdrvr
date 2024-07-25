/*
 * BSD LICENSE
 *
 * Copyright (C) 2024 Broadcom. All Rights Reserved.
 * The term “Broadcom” refers to Broadcom Inc. and/or its subsidiaries.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 *
 * The ocs_pool_t data structure consists of:
 *
 *	pool->a		An ocs_array_t.
 *	pool->freelist	A linked list of free items.
 *
 *	When a pool is allocated using ocs_pool_alloc(), the caller provides the
 *	size in bytes of each memory pool item (size), and a count of items (count).
 *	Since ocs_pool_alloc() has no visibility into the object the caller is
 *	allocating, a link for the linked list is "pre-pended".  Thus when allocating the
 *	ocs_array_t, the size used is the size of the pool_hdr_t plus the requested
 *	memory pool item size.
 *
 *	array item layout:
 *
 *		pool_hdr_t
 *		pool data[size]
 *
 *	The address of the pool data is returned when allocated (using ocs_pool_get(), or
 *	ocs_pool_get_instance()), and received when being freed (using ocs_pool_put().
 *	So the address returned by the array item (ocs_array_get()) must be offset by
 *	the size of pool_hdr_t.
 */

#include "ocs_os.h"
#include "ocs_pool.h"

struct ocs_pool_s {
	ocs_os_handle_t os;
	ocs_array_t *a;
	ocs_list_t freelist;
	uint32_t use_lock:1;
	ocs_lock_t lock;
};

typedef struct {
	ocs_list_link_t link;
} pool_hdr_t;


/**
 * @brief Allocate a memory pool.
 *
 * A memory pool of given size and item count is allocated.
 *
 * @param os OS handle.
 * @param size Size in bytes of item.
 * @param count Number of items in a memory pool.
 * @param use_lock TRUE to enable locking of pool.
 *
 * @return Returns pointer to allocated memory pool, or NULL.
 */
ocs_pool_t *
ocs_pool_alloc(ocs_os_handle_t os, uint32_t size, uint32_t count, uint32_t use_lock)
{
	ocs_pool_t *pool;
	uint32_t i;
	uint32_t flags = OCS_M_ZERO;

#if !defined(OCS_USPACE)
	ocs_os_t *ocs_os = os;
	if (ocs_os->hw_cmpl_context != OCS_HW_CMPL_CONTEXT_THREAD)
		flags |= OCS_M_NOWAIT;
#endif

	pool = ocs_malloc(os, sizeof(*pool), flags);
	if (pool == NULL) {
		return NULL;
	}

	pool->os = os;
	pool->use_lock = use_lock;
	if (pool->use_lock) {
		ocs_lock_init(os, &pool->lock, OCS_LOCK_ORDER_IGNORE, "ocs_pool:%p", pool);
	}

	/* Allocate an array where each array item is the size of a pool_hdr_t plus
	 * the requested memory item size (size)
	 */
	pool->a = ocs_array_alloc(os, size + sizeof(pool_hdr_t), count);
	if (pool->a == NULL) {
		ocs_pool_free(pool);
		return NULL;
	}

	ocs_list_init(&pool->freelist, pool_hdr_t, link);
	for (i = 0; i < count; i++) {
		ocs_list_add_tail(&pool->freelist, ocs_array_get(pool->a, i));
	}

	return pool;
}

/**
 * @brief Reset a memory pool.
 *
 * Place all pool elements on the free list, and zero them.
 *
 * @param pool Pointer to the pool object.
 *
 * @return None.
 */
void
ocs_pool_reset(ocs_pool_t *pool)
{
	uint32_t i;
	uint32_t count = ocs_array_get_count(pool->a);
	uint32_t size = ocs_array_get_size(pool->a);

	if (pool->use_lock) {
		ocs_lock(&pool->lock);
	}

	/*
	 * Remove all the entries from the free list, otherwise we will
	 * encountered linked list asserts when they are re-added.
	 */
	while (!ocs_list_empty(&pool->freelist)) {
		ocs_list_remove_head(&pool->freelist);
	}

	/* Reset the free list */
	ocs_list_init(&pool->freelist, pool_hdr_t, link);

	/* Return all elements to the free list and zero the elements */
	for (i = 0; i < count; i++) {
		void *buf = ocs_pool_get_instance(pool, i);

		if (!buf) {
			ocs_log_err(NULL, "ocs_pool_get_instance failed\n");
			ocs_list_assert(0);
		}

		if (size > sizeof(pool_hdr_t))
			ocs_memset(buf, 0, size - sizeof(pool_hdr_t));

		ocs_list_add_tail(&pool->freelist, ocs_array_get(pool->a, i));
	}

	if (pool->use_lock) {
		ocs_unlock(&pool->lock);
	}
}

/**
 * @brief Free a previously allocated memory pool.
 *
 * The memory pool is freed.
 *
 * @param pool Pointer to memory pool.
 *
 * @return None.
 */
void
ocs_pool_free(ocs_pool_t *pool)
{
	if (pool != NULL) {
		if (pool->a != NULL) {
			ocs_array_free(pool->a);
		}
		if (pool->use_lock) {
			ocs_lock_free(&pool->lock);
		}
		ocs_free(pool->os, pool, sizeof(*pool));
	}
}

/**
 * @brief Allocate a memory pool item
 *
 * A memory pool item is taken from the free list and returned.
 *
 * @param pool Pointer to memory pool.
 *
 * @return Pointer to allocated item, otherwise NULL if there are no unallocated
 *	   items.
 */
void *
ocs_pool_get(ocs_pool_t *pool)
{
	pool_hdr_t *h;
	void *item = NULL;

	if (pool->use_lock) {
		ocs_lock(&pool->lock);
	}

	h = ocs_list_remove_head(&pool->freelist);

	if (h != NULL) {
		/* Return the array item address offset by the size of pool_hdr_t */
		item = &h[1];
	}

	if (pool->use_lock) {
		ocs_unlock(&pool->lock);
	}
	return item;
}

/**
 * @brief free memory pool item
 *
 * A memory pool item is freed.
 *
 * @param pool Pointer to memory pool.
 * @param item Pointer to item to free.
 *
 * @return None.
 */
void
ocs_pool_put(ocs_pool_t *pool, void *item)
{
	pool_hdr_t *h;

	if (pool->use_lock) {
		ocs_lock(&pool->lock);
	}

	/* Fetch the address of the array item, which is the item address negatively offset
	 * by size of pool_hdr_t (note the index of [-1]
	 */
	h = &((pool_hdr_t*)item)[-1];

	ocs_list_add_tail(&pool->freelist, h);

	if (pool->use_lock) {
		ocs_unlock(&pool->lock);
	}
}

/**
 * @brief free memory pool item
 *
 * A memory pool item is freed to head of list.
 *
 * @param pool Pointer to memory pool.
 * @param item Pointer to item to free.
 *
 * @return None.
 */
void
ocs_pool_put_head(ocs_pool_t *pool, void *item)
{
	pool_hdr_t *h;

	if (pool->use_lock) {
		ocs_lock(&pool->lock);
	}

	/* Fetch the address of the array item, which is the item address negatively offset
	 * by size of pool_hdr_t (note the index of [-1]
	 */
	h = &((pool_hdr_t*)item)[-1];

	ocs_list_add_head(&pool->freelist, h);

	if (pool->use_lock) {
		ocs_unlock(&pool->lock);
	}
}

/**
 * @brief Return memory pool item count.
 *
 * Returns the allocated number of items.
 *
 * @param pool Pointer to memory pool.
 *
 * @return Returns count of allocated items.
 */
uint32_t
ocs_pool_get_count(ocs_pool_t *pool)
{
	uint32_t count;
	if (pool->use_lock) {
		ocs_lock(&pool->lock);
	}
	count = ocs_array_get_count(pool->a);
	if (pool->use_lock) {
		ocs_unlock(&pool->lock);
	}
	return count;
}

/**
 * @brief Return item given an index.
 *
 * A pointer to a memory pool item is returned given an index.
 *
 * @param pool Pointer to memory pool.
 * @param idx Index.
 *
 * @return Returns pointer to item, or NULL if index is invalid.
 */
void *
ocs_pool_get_instance(ocs_pool_t *pool, uint32_t idx)
{
	pool_hdr_t *h = ocs_array_get(pool->a, idx);

	if (h == NULL) {
		return NULL;
	}
	return &h[1];
}

/**
 * @brief Return count of free objects in a pool.
 *
 * The number of objects on a pool's free list.
 *
 * @param pool Pointer to memory pool.
 *
 * @return Returns count of objects on free list.
 */
uint32_t
ocs_pool_get_freelist_count(ocs_pool_t *pool)
{
	uint32_t count = 0;
	void *item;

	if (pool->use_lock) {
		ocs_lock(&pool->lock);
	}

	ocs_list_foreach(&pool->freelist, item) {
		count++;
	}

	if (pool->use_lock) {
		ocs_unlock(&pool->lock);
	}
	return count;
}
