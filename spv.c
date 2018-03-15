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
 * @file spv.c
 * @brief Sparse Vector API.
 *
 * This is a trimmed down sparse vector implementation tuned to the problem of
 * 24-bit FC_IDs. In this case, the 24-bit index value is broken down in three
 * 8-bit values. These values are used to index up to three 256 element arrays.
 * Arrays are allocated, only when needed. @n @n
 * The lookup can complete in constant time (3 indexed array references). @n @n
 * A typical use case would be that the fabric/directory FC_IDs would cause two rows to be
 * allocated, and the fabric assigned remote nodes would cause two rows to be allocated, with
 * the root row always allocated. This gives five rows of 256 x sizeof(void*),
 * resulting in 10k.
 */

#include "ocs_os.h"
#include "spv.h"


/**
 * @ingroup spv
 * @brief Allocate a new sparse vector row.
 *
 * @param os OS handle
 * @param rowcount Count of rows.
 *
 * @par Description
 * A new sparse vector row is allocated.
 *
 * @param rowcount Number of elements in a row.
 *
 * @return Returns the pointer to a row.
 */
static void
**spv_new_row(ocs_os_handle_t os, uint32_t rowcount)
{
	return ocs_malloc(os, sizeof(void*) * rowcount, OCS_M_ZERO | OCS_M_NOWAIT);
}


#if 0
/**
 * @ingroup spv
 * @brief Delete a sparse vector row.
 *
 * @par Description
 * The resources associated with the row are freed.
 *
 * @param row Pointer to the row.
 * @param rowcount Count of the elements in the row.
 *
 * @return None.
 */
static void
spv_del_row(void **row, uint32_t rowcount)
{
	ocs_free(row, sizeof(void*) * rowcount);
}
#endif


#if 0
/**
 * @ingroup spv
 * @brief Return maximum index for a sparse vector.
 *
 * @par Description
 * The maximum index value for the sparse vector is returned.
 *
 * @param spv Pointer to the sparse vector object.
 *
 * @return Returns the maximum index value.
 */
static uint32_t
spv_get_max_idx(sparse_vector_t spv)
{
	return spv->max_idx;
}
#endif

/**
 * @ingroup spv
 * @brief Delete row recursively.
 *
 * @par Description
 * This function recursively deletes the rows in this sparse vector
 *
 * @param os OS handle
 * @param a Pointer to the row.
 * @param n Number of elements in the row.
 * @param depth Depth of deleting.
 *
 * @return None.
 */
static void
_spv_del(ocs_os_handle_t os, void **a, uint32_t n, uint32_t depth)
{
	if (a) {
		if (depth) {
			uint32_t i;

			for (i = 0; i < n; i ++) {
				_spv_del(os, a[i], n, depth-1);
			}

			ocs_free(os, a, SPV_ROWLEN*sizeof(*a));
		}
	}
}

/**
 * @ingroup spv
 * @brief Delete a sparse vector.
 *
 * @par Description
 * The sparse vector is freed.
 *
 * @param spv Pointer to the sparse vector object.
 */
void
spv_del(sparse_vector_t spv)
{
	if (spv) {
		_spv_del(spv->os, spv->array, SPV_ROWLEN, SPV_DIM);
		ocs_free(spv->os, spv, sizeof(*spv));
	}
}

/**
 * @ingroup spv
 * @brief Instantiate a new sparse vector object.
 *
 * @par Description
 * A new sparse vector is allocated.
 *
 * @param os OS handle
 *
 * @return Returns the pointer to the sparse vector, or NULL.
 */
sparse_vector_t
spv_new(ocs_os_handle_t os)
{
	sparse_vector_t spv;
	uint32_t i;

	spv = ocs_malloc(os, sizeof(*spv), OCS_M_ZERO | OCS_M_NOWAIT);
	if (!spv) {
		return NULL;
	}

	spv->os = os;
	spv->max_idx = 1;
	for (i = 0; i < SPV_DIM; i ++) {
		spv->max_idx *= SPV_ROWLEN;
	}

	return spv;
}

/**
 * @ingroup spv
 * @brief Return the address of a cell.
 *
 * @par Description
 * Returns the address of a cell, allocates sparse rows as needed if the
 *         alloc_new_rows parameter is set.
 *
 * @param sv Pointer to the sparse vector.
 * @param idx Index of which to return the address.
 * @param alloc_new_rows If TRUE, then new rows may be allocated to set values,
 *                       Set to FALSE for retrieving values.
 *
 * @return Returns the pointer to the cell, or NULL.
 */
static void
*spv_new_cell(sparse_vector_t sv, uint32_t idx, uint8_t alloc_new_rows)
{
	uint32_t a = (idx >> 16) & 0xff;
	uint32_t b = (idx >>  8) & 0xff;
	uint32_t c = (idx >>  0) & 0xff;
	void **p;

	if (idx >= sv->max_idx) {
		return NULL;
	}

	if (sv->array == NULL) {
		sv->array = (alloc_new_rows ? spv_new_row(sv->os, SPV_ROWLEN) : NULL);
		if (sv->array == NULL) {
			return NULL;
		}
	}
	p = sv->array;
	if (p[a] == NULL) {
		p[a] = (alloc_new_rows ? spv_new_row(sv->os, SPV_ROWLEN) : NULL);
		if (p[a] == NULL) {
			return NULL;
		}
	}
	p = p[a];
	if (p[b] == NULL) {
		p[b] = (alloc_new_rows ? spv_new_row(sv->os, SPV_ROWLEN) : NULL);
		if (p[b] == NULL) {
			return NULL;
		}
	}
	p = p[b];

	return &p[c];
}

/**
 * @ingroup spv
 * @brief Set the sparse vector cell value.
 *
 * @par Description
 * Sets the sparse vector at @c idx to @c value.
 *
 * @param sv Pointer to the sparse vector.
 * @param idx Index of which to store.
 * @param value Value to store.
 *
 * @return None.
 */
void
spv_set(sparse_vector_t sv, uint32_t idx, void *value)
{
	void **ref = spv_new_cell(sv, idx, TRUE);
	if (ref) {
		*ref = value;
	}
}

/**
 * @ingroup spv
 * @brief Return the sparse vector cell value.
 *
 * @par Description
 * Returns the value at @c idx.
 *
 * @param sv Pointer to the sparse vector.
 * @param idx Index of which to return the value.
 *
 * @return Returns the cell value, or NULL.
 */
void
*spv_get(sparse_vector_t sv, uint32_t idx)
{
	void **ref = spv_new_cell(sv, idx, FALSE);
	if (ref) {
		return *ref;
	}
	return NULL;
}
