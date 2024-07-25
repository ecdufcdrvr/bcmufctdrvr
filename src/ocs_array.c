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
 */

#include "ocs.h"
#include "ocs_array.h"

#define DEFAULT_SLAB_LEN		(16*1024)

struct ocs_array_s {
	ocs_os_handle_t os;

	uint32_t size;
	uint32_t count;

	uint32_t n_rows;
	uint32_t elems_per_row;
	uint32_t bytes_per_row;

	void **array_rows;
	uint32_t array_rows_len;
};

static uint32_t slab_len = DEFAULT_SLAB_LEN;

/**
 * @brief Set array slab allocation length
 *
 * The slab length is the maximum allocation length that the array uses.
 * The default 64k slab length may be overridden using this function.
 *
 * @param len new slab length.
 *
 * @return none
 */
void
ocs_array_set_slablen(uint32_t len)
{
	slab_len = len;
}

/**
 * @brief Allocate an array object
 *
 * An array object of size and number of elements is allocated
 *
 * @param os OS handle
 * @param size size of array elements in bytes
 * @param count number of elements in array
 *
 * @return pointer to array object or NULL
 */
ocs_array_t *
ocs_array_alloc(ocs_os_handle_t os, uint32_t size, uint32_t count)
{
	ocs_array_t *array = NULL;
	uint32_t i;
	uint32_t flags = OCS_M_ZERO;
#if !defined(OCS_USPACE)
	ocs_os_t *ocs_os = os;
#endif

	/* Fail if the item size exceeds slab_len - caller should increase slab_size,
	 * or not use this API.
	 */
	if (size > slab_len) {
		ocs_log_err(NULL, "Error: size exceeds slab length\n");
		return NULL;
	}

#if !defined(OCS_USPACE)
	if ((ocs_os == NULL) || (ocs_os->hw_cmpl_context != OCS_HW_CMPL_CONTEXT_THREAD))
		flags |= OCS_M_NOWAIT;
#endif

	array = ocs_malloc(os, sizeof(*array), flags);
	if (array == NULL) {
		return NULL;
	}

	array->os = os;
	array->size = size;
	array->count = count;
	array->elems_per_row = slab_len / size;
	array->n_rows = (count + array->elems_per_row - 1) / array->elems_per_row;
	array->bytes_per_row = array->elems_per_row * array->size;

	array->array_rows_len = array->n_rows * sizeof(*array->array_rows);
	array->array_rows = ocs_malloc(os, array->array_rows_len, flags);
	if (array->array_rows == NULL) {
		ocs_array_free(array);
		return NULL;
	}
	for (i = 0; i < array->n_rows; i++) {
		array->array_rows[i] = ocs_malloc(os, array->bytes_per_row, flags);
		if (array->array_rows[i] == NULL) {
			ocs_array_free(array);
			return NULL;
		}
	}

	return array;
}

/**
 * @brief Free an array object
 *
 * Frees a prevously allocated array object
 *
 * @param array pointer to array object
 *
 * @return none
 */
void
ocs_array_free(ocs_array_t *array)
{
	uint32_t i;

	if (array != NULL) {
		if (array->array_rows != NULL) {
			for (i = 0; i < array->n_rows; i++) {
				if (array->array_rows[i] != NULL) {
					ocs_free(array->os, array->array_rows[i], array->bytes_per_row);
				}
			}
			ocs_free(array->os, array->array_rows, array->array_rows_len);
		}
		ocs_free(array->os, array, sizeof(*array));
	}
}

/**
 * @brief Return reference to an element of an array object
 *
 * Return the address of an array element given an index
 *
 * @param array pointer to array object
 * @param idx array element index
 *
 * @return rointer to array element, or NULL if index out of range
 */
void *ocs_array_get(ocs_array_t *array, uint32_t idx)
{
	void *entry = NULL;

	if (idx < array->count) {
		uint32_t row = idx / array->elems_per_row;
		uint32_t offset = idx % array->elems_per_row;
		entry = ((uint8_t*)array->array_rows[row]) + (offset * array->size);
	}
	return entry;
}

/**
 * @brief Return number of elements in an array
 *
 * Return the number of elements in an array
 *
 * @param array pointer to array object
 *
 * @return returns count of elements in an array
 */
uint32_t
ocs_array_get_count(ocs_array_t *array)
{
	return array->count;
}

/**
 * @brief Return size of array elements in bytes
 *
 * Returns the size in bytes of each array element
 *
 * @param array pointer to array object
 *
 * @return size of array element
 */
uint32_t
ocs_array_get_size(ocs_array_t *array)
{
	return array->size;
}

/**
 * @brief Void pointer array structure
 *
 * This structure describes an object consisting of an array of void
 * pointers.   The object is allocated with a maximum array size, entries
 * are then added to the array with while maintaining an entry count.   A set of
 * iterator APIs are included to allow facilitate cycling through the array
 * entries in a circular fashion.
 *
 */
struct ocs_varray_s {
	ocs_os_handle_t os;
	uint32_t array_count;			/*>> maximum entry count in array */
	void **array;				/*>> pointer to allocated array memory */
	uint32_t entry_count;			/*>> number of entries added to the array */
	uint32_t next_index;			/*>> iterator next index */
	ocs_lock_t lock;			/*>> iterator lock */
};

/**
 * @brief Allocate a void pointer array
 *
 * A void pointer array of given length is allocated.
 *
 * @param os OS handle
 * @param array_count Array size
 *
 * @return returns a pointer to the ocs_varray_t object, other NULL on error
 */
ocs_varray_t *
ocs_varray_alloc(ocs_os_handle_t os, uint32_t array_count)
{
	ocs_varray_t *va;

	va = ocs_malloc(os, sizeof(*va), OCS_M_ZERO);
	if (va != NULL) {
		va->os = os;
		va->array_count = array_count;
		va->array = ocs_malloc(os, sizeof(*va->array) * va->array_count, OCS_M_ZERO);
		if (va->array != NULL) {
			va->next_index = 0;
			ocs_lock_init(os, &va->lock, OCS_LOCK_ORDER_IGNORE, "varray:%p", va);
		} else {
			ocs_free(os, va, sizeof(*va));
			va = NULL;
		}
	}
	return va;
}

/**
 * @brief Free a void pointer array
 *
 * The void pointer array object is free'd
 *
 * @param va Pointer to void pointer array
 *
 * @return none
 */
void
ocs_varray_free(ocs_varray_t *va)
{
	if (va != NULL) {
		ocs_lock_free(&va->lock);
		if (va->array != NULL) {
			ocs_free(va->os, va->array, sizeof(*va->array) * va->array_count);
		}
		ocs_free(va->os, va, sizeof(*va));
	}
}

/**
 * @brief Add an entry to a void pointer array
 *
 * An entry is added to the void pointer array
 *
 * @param va Pointer to void pointer array
 * @param entry Pointer to entry to add
 *
 * @return returns 0 if entry was added, -1 if there is no more space in the array
 */
int32_t
ocs_varray_add(ocs_varray_t *va, void *entry)
{
	uint32_t rc = -1;

	ocs_lock(&va->lock);
		if (va->entry_count < va->array_count) {
			va->array[va->entry_count++] = entry;
			rc = 0;
		}
	ocs_unlock(&va->lock);

	return rc;
}

/**
 * @brief Reset the void pointer array iterator
 *
 * The next index value of the void pointer array iterator is cleared.
 *
 * @param va Pointer to void pointer array
 *
 * @return none
 */
void
ocs_varray_iter_reset(ocs_varray_t *va)
{
	ocs_lock(&va->lock);
		va->next_index = 0;
	ocs_unlock(&va->lock);
}

/**
 * @brief Return next entry from a void pointer array
 *
 * The next entry in the void pointer array is returned.
 *
 * @param va Pointer to void point array
 *
 * Note: takes the void pointer array lock
 *
 * @return returns next void pointer entry
 */
void *
ocs_varray_iter_next(ocs_varray_t *va)
{
	void *rval = NULL;

	if (va != NULL) {
		ocs_lock(&va->lock);
			rval = _ocs_varray_iter_next(va);
		ocs_unlock(&va->lock);
	}
	return rval;
}

/**
 * @brief Return next entry from a void pointer array
 *
 * The next entry in the void pointer array is returned.
 *
 * @param va Pointer to void point array
 *
 * Note: doesn't take the void pointer array lock
 *
 * @return returns next void pointer entry
 */
void *
_ocs_varray_iter_next(ocs_varray_t *va)
{
	void *rval;

	rval = va->array[va->next_index];
	if (++va->next_index >= va->entry_count) {
		va->next_index = 0;
	}
	return rval;
}

/**
 * @brief Take void pointer array lock
 *
 * Takes the lock for the given void pointer array
 *
 * @param va Pointer to void pointer array
 *
 * @return none
 */
void
ocs_varray_lock(ocs_varray_t *va)
{
	ocs_lock(&va->lock);
}

/**
 * @brief Release void pointer array lock
 *
 * Releases the lock for the given void pointer array
 *
 * @param va Pointer to void pointer array
 *
 * @return none
 */
void
ocs_varray_unlock(ocs_varray_t *va)
{
	ocs_unlock(&va->lock);
}

/**
 * @brief Return entry count for a void pointer array
 *
 * The entry count for a void pointer array is returned
 *
 * @param va Pointer to void pointer array
 *
 * @return returns entry count
 */
uint32_t
ocs_varray_get_count(ocs_varray_t *va)
{
	uint32_t rc;

	ocs_lock(&va->lock);
		rc = va->entry_count;
	ocs_unlock(&va->lock);
	return rc;
}
