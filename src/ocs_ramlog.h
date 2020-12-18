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

/**
 * @file
 *
 */


#if !defined(__RAMLOG_H__)
#define __RAMLOG_H__

#include "ocs_textbuf.h"

typedef struct ocs_ramlog_s ocs_ramlog_t;

#define OCS_RAMLOG_DEFAULT_BUFFERS		5

extern ocs_ramlog_t *ocs_ramlog_init(ocs_t *ocs, uint32_t buffer_len, uint32_t buffer_count);
extern void ocs_ramlog_free(ocs_t *ocs, ocs_ramlog_t *ramlog);
extern void ocs_ramlog_clear(ocs_t *ocs, ocs_ramlog_t *ramlog, int clear_start_of_day, int clear_recent);
__attribute__((format(printf,2,3)))
extern int32_t ocs_ramlog_printf(void *os, const char *fmt, ...);
__attribute__((format(printf,2,0)))
extern int32_t ocs_ramlog_vprintf(ocs_ramlog_t *ramlog, const char *fmt, va_list ap);
extern int32_t ocs_ddump_ramlog(ocs_textbuf_t *textbuf, ocs_ramlog_t *ramlog);

#endif // __RAMLOG_H__
