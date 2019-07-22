/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2018, Broadcom.  All Rights Reserved.
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

#ifndef __OCS_ARRAY_H__
#define __OCS_ARRAY_H__

typedef struct ocs_array_s ocs_array_t;

extern void ocs_array_set_slablen(uint32_t len);
extern ocs_array_t *ocs_array_alloc(ocs_os_handle_t os, uint32_t size, uint32_t count);
extern void ocs_array_free(ocs_array_t *array);
extern void *ocs_array_get(ocs_array_t *array, uint32_t idx);
extern uint32_t ocs_array_get_count(ocs_array_t *array);
extern uint32_t ocs_array_get_size(ocs_array_t *array);

/* Void pointer array and iterator */
typedef struct ocs_varray_s ocs_varray_t;

extern ocs_varray_t *ocs_varray_alloc(ocs_os_handle_t os, uint32_t entry_count);
extern void ocs_varray_free(ocs_varray_t *ai);
extern int32_t ocs_varray_add(ocs_varray_t *ai, void *entry);
extern void ocs_varray_iter_reset(ocs_varray_t *ai);
extern void *ocs_varray_iter_next(ocs_varray_t *ai);
extern void *_ocs_varray_iter_next(ocs_varray_t *ai);
extern void ocs_varray_lock(ocs_varray_t *ai);
extern void ocs_varray_unlock(ocs_varray_t *ai);
extern uint32_t ocs_varray_get_count(ocs_varray_t *ai);

#endif /* __OCS_ARRAY_H__ */
