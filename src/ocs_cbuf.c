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
 *
 */


#include "ocs_os.h"
#include "ocs_array.h"

struct ocs_cbuf_s {
	ocs_os_handle_t os;		/*<< OS handle */
	uint32_t entry_count;		/*<< entry count */
	void **array;			/*<< pointer to array of cbuf pointers */
	uint32_t pidx;			/*<< producer index */
	uint32_t cidx;			/*<< consumer index */
	ocs_lock_t cbuf_plock;		/*<< idx lock */
	ocs_lock_t cbuf_clock;		/*<< idx lock */
	ocs_sem_t cbuf_psem;		/*<< cbuf producer counting semaphore */
	ocs_sem_t cbuf_csem;		/*<< cbuf consumer counting semaphore */
};

/**
 * @brief Initialize a circular buffer queue
 *
 * A circular buffer with producer/consumer API is allocated
 *
 * @param os OS handle
 * @param entry_count count of entries
 *
 * @return returns pointer to circular buffer, or NULL
 */
ocs_cbuf_t*
ocs_cbuf_alloc(ocs_os_handle_t os, uint32_t entry_count)
{
	ocs_cbuf_t *cbuf;

	cbuf = ocs_malloc(os, sizeof(*cbuf), OCS_M_NOWAIT | OCS_M_ZERO);
	if (cbuf == NULL) {
		return NULL;
	}

	cbuf->os = os;
	cbuf->entry_count = entry_count;
	cbuf->pidx = 0;
	cbuf->cidx = 0;

	ocs_lock_init(NULL, &cbuf->cbuf_clock, "cbuf_c:%p", cbuf);
	ocs_lock_init(NULL, &cbuf->cbuf_plock, "cbuf_p:%p", cbuf);
	ocs_sem_init(&cbuf->cbuf_csem, 0, "cbuf:%p", cbuf);
	ocs_sem_init(&cbuf->cbuf_psem, cbuf->entry_count, "cbuf:%p", cbuf);

	cbuf->array = ocs_malloc(os, entry_count * sizeof(*cbuf->array), OCS_M_NOWAIT | OCS_M_ZERO);
	if (cbuf->array == NULL) {
		ocs_cbuf_free(cbuf);
		return NULL;
	}

	return cbuf;
}

/**
 * @brief Free a circular buffer
 *
 * The memory resources of a circular buffer are free'd
 *
 * @param cbuf pointer to circular buffer
 *
 * @return none
 */
void
ocs_cbuf_free(ocs_cbuf_t *cbuf)
{
	if (cbuf != NULL) {
		if (cbuf->array != NULL) {
			ocs_free(cbuf->os, cbuf->array, sizeof(*cbuf->array) * cbuf->entry_count);
		}
		ocs_lock_free(&cbuf->cbuf_clock);
		ocs_lock_free(&cbuf->cbuf_plock);
		ocs_free(cbuf->os, cbuf, sizeof(*cbuf));
	}
}

/**
 * @brief Get pointer to buffer
 *
 * Wait for a buffer to become available, and return a pointer to the buffer.
 *
 * @param cbuf pointer to circular buffer
 * @param timeout_usec timeout in microseconds
 *
 * @return pointer to buffer, or NULL if timeout
 */
void*
ocs_cbuf_get(ocs_cbuf_t *cbuf, int32_t timeout_usec)
{
	void *ret = NULL;

	if (likely(ocs_sem_p(&cbuf->cbuf_csem, timeout_usec) == 0)) {
		ocs_lock(&cbuf->cbuf_clock);
			ret = cbuf->array[cbuf->cidx];
			if (unlikely(++cbuf->cidx >= cbuf->entry_count)) {
				cbuf->cidx = 0;
			}
		ocs_unlock(&cbuf->cbuf_clock);
		ocs_sem_v(&cbuf->cbuf_psem);
	}
	return ret;
}

/**
 * @brief write a buffer
 *
 * The buffer is written to the circular buffer.
 *
 * @param cbuf pointer to circular buffer
 * @param elem pointer to entry
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocs_cbuf_put(ocs_cbuf_t *cbuf, void *elem)
{
	int32_t rc = 0;

	if (likely(ocs_sem_p(&cbuf->cbuf_psem, -1) == 0)) {
		ocs_lock(&cbuf->cbuf_plock);
			cbuf->array[cbuf->pidx] = elem;
			if (unlikely(++cbuf->pidx >= cbuf->entry_count)) {
				cbuf->pidx = 0;
			}
		ocs_unlock(&cbuf->cbuf_plock);
		ocs_sem_v(&cbuf->cbuf_csem);
	} else {
		rc = -1;
	}
	return rc;
}

/**
 * @brief Prime a circular buffer data
 *
 * Post array buffers to a circular buffer
 *
 * @param cbuf pointer to circular buffer
 * @param array pointer to buffer array
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocs_cbuf_prime(ocs_cbuf_t *cbuf, ocs_array_t *array)
{
	uint32_t i;
	uint32_t count = MIN(ocs_array_get_count(array), cbuf->entry_count);

	for (i = 0; i < count; i++) {
		ocs_cbuf_put(cbuf, ocs_array_get(array, i));
	}
	return 0;
}

