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
 * @file spv.h
 * @brief Sparse Vector API
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


#if !defined(__SPV_H__)
#define __SPV_H__

#define SPV_ROWLEN	256
#define SPV_DIM		3


/*!
* @defgroup spv Sparse Vector
*/

/**
 * @brief Sparse vector structure.
 */
typedef struct sparse_vector_s {
	ocs_os_handle_t os;
	uint32_t max_idx;		/**< maximum index value */
	void **array;			/**< pointer to 3D array */
} *sparse_vector_t;

extern void spv_del(sparse_vector_t spv);
extern sparse_vector_t spv_new(ocs_os_handle_t os);
extern void spv_set(sparse_vector_t sv, uint32_t idx, void *value);
extern void *spv_get(sparse_vector_t sv, uint32_t idx);

#endif // __SPV_H__
