/*
 * Copyright (C) 2020 Broadcom. All Rights Reserved.
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

#if !defined(__OCS_TEXTBUF_H__)
#define __OCS_TEXTBUF_H__

#include "ocs_list.h"

#define OCS_TEXTBUF_MAX_ALLOC_LEN	(256*1024)

typedef struct {
	ocs_list_link_t link;
	uint8_t user_allocated:1;
	uint8_t *buffer;
	uint32_t buffer_length;
	uint32_t buffer_written;
} ocs_textbuf_segment_t;

typedef struct {
	ocs_t *ocs;
	ocs_list_t segment_list;
	uint8_t pre_allocated:1;
	uint32_t allocation_length;
	uint32_t total_allocation_length;
	uint32_t max_allocation_length;
	ocs_textbuf_segment_t *current_seg;
} ocs_textbuf_t;

extern int32_t ocs_textbuf_alloc(ocs_t *ocs, ocs_textbuf_t *textbuf, uint32_t length);
extern int32_t ocs_textbuf_pool_alloc(ocs_t *ocs, ocs_textbuf_t *textbuf, uint32_t length);
extern uint32_t ocs_textbuf_initialized(ocs_textbuf_t *textbuf);
extern void ocs_textbuf_free(ocs_t *ocs, ocs_textbuf_t *textbuf);
extern void ocs_textbuf_putc(ocs_textbuf_t *textbuf, uint8_t c);
extern void ocs_textbuf_puts(ocs_textbuf_t *textbuf, char *s);
__attribute__((format(printf,2,3)))
extern void ocs_textbuf_printf(ocs_textbuf_t *textbuf, const char *fmt, ...);
__attribute__((format(printf,2,0)))
extern void ocs_textbuf_vprintf(ocs_textbuf_t *textbuf, const char *fmt, va_list ap);
extern void ocs_textbuf_buffer(ocs_textbuf_t *textbuf, uint8_t *buffer, uint32_t buffer_length);
extern void ocs_textbuf_copy(ocs_textbuf_t *textbuf, uint8_t *buffer, uint32_t buffer_length);
extern int32_t ocs_textbuf_remaining(ocs_textbuf_t *textbuf);
extern void ocs_textbuf_reset(ocs_textbuf_t *textbuf);
extern uint8_t *ocs_textbuf_get_buffer(ocs_textbuf_t *textbuf);
extern int32_t ocs_textbuf_get_length(ocs_textbuf_t *textbuf);
extern int32_t ocs_textbuf_get_written(ocs_textbuf_t *textbuf);
extern uint8_t *ocs_textbuf_ext_get_buffer(ocs_textbuf_t *textbuf, uint32_t idx);
extern int32_t ocs_textbuf_ext_get_length(ocs_textbuf_t *textbuf, uint32_t idx);
extern int32_t ocs_textbuf_ext_get_written(ocs_textbuf_t *textbuf, uint32_t idx);

#endif // __OCS_TEXTBUF_H__
