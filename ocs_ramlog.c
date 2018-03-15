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
 * RAM buffer logging code
 *
 */

#include "ocs.h"

struct ocs_ramlog_s {
	uint32_t initialized;
	uint32_t textbuf_count;
	uint32_t textbuf_base;
	ocs_textbuf_t *textbufs;
	uint32_t cur_textbuf_idx;
	ocs_textbuf_t *cur_textbuf;
	ocs_lock_t lock;
};

static uint32_t ocs_ramlog_next_idx(ocs_ramlog_t *ramlog, uint32_t idx);

/**
 * @brief Allocate a ramlog buffer.
 *
 * Initialize a RAM logging buffer with text buffers totalling buffer_len.
 *
 * @param ocs Pointer to driver structure.
 * @param buffer_len Total length of RAM log buffers.
 * @param buffer_count Number of text buffers to allocate (totalling buffer-len).
 *
 * @return Returns pointer to ocs_ramlog_t instance, or NULL.
 */
ocs_ramlog_t *
ocs_ramlog_init(ocs_t *ocs, uint32_t buffer_len, uint32_t buffer_count)
{
	uint32_t i;
	uint32_t rc;
	ocs_ramlog_t *ramlog;

	ramlog = ocs_malloc(ocs, sizeof(*ramlog), OCS_M_ZERO | OCS_M_NOWAIT);
	if (ramlog == NULL) {
		ocs_log_err(ocs, "%s: ocs_malloc ramlog failed\n", __func__);
		return NULL;
	}

	ramlog->textbuf_count = buffer_count;

	ramlog->textbufs = ocs_malloc(ocs, sizeof(*ramlog->textbufs)*buffer_count, OCS_M_ZERO | OCS_M_NOWAIT);
	if (ramlog->textbufs == NULL) {
		ocs_log_err(ocs, "%s: ocs_malloc textbufs failed\n", __func__);
		ocs_ramlog_free(ocs, ramlog);
		return NULL;
	}

	for (i = 0; i < buffer_count; i ++) {
		rc = ocs_textbuf_alloc(ocs, &ramlog->textbufs[i], buffer_len);
		if (rc) {
			ocs_log_err(ocs, "%s: ocs_textbuf_alloc failed\n", __func__);
			ocs_ramlog_free(ocs, ramlog);
			return NULL;
		}
	}

	ramlog->cur_textbuf_idx = 0;
	ramlog->textbuf_base = 1;
	ramlog->cur_textbuf = &ramlog->textbufs[0];
	ramlog->initialized = TRUE;
	ocs_lock_init(ocs, &ramlog->lock, "ramlog_lock[%d]", ocs_instance(ocs));
	return ramlog;
}

/**
 * @brief Free a ramlog buffer.
 *
 * A previously allocated RAM logging buffer is freed.
 *
 * @param ocs Pointer to driver structure.
 * @param ramlog Pointer to RAM logging buffer structure.
 *
 * @return None.
 */

void
ocs_ramlog_free(ocs_t *ocs, ocs_ramlog_t *ramlog)
{
	uint32_t i;

	if (ramlog != NULL) {
		ocs_lock_free(&ramlog->lock);
		if (ramlog->textbufs) {
			for (i = 0; i < ramlog->textbuf_count; i ++) {
				ocs_textbuf_free(ocs, &ramlog->textbufs[i]);
			}

			ocs_free(ocs, ramlog->textbufs, ramlog->textbuf_count*sizeof(*ramlog->textbufs));
			ramlog->textbufs = NULL;
		}
		ocs_free(ocs, ramlog, sizeof(*ramlog));
	}
}

/**
 * @brief Clear a ramlog buffer.
 *
 * The text in the start of day and/or recent ramlog text buffers is cleared.
 *
 * @param ocs Pointer to driver structure.
 * @param ramlog Pointer to RAM logging buffer structure.
 * @param clear_start_of_day Clear the start of day (driver init) portion of the ramlog.
 * @param clear_recent Clear the recent messages portion of the ramlog.
 *
 * @return None.
 */

void
ocs_ramlog_clear(ocs_t *ocs, ocs_ramlog_t *ramlog, int clear_start_of_day, int clear_recent)
{
	uint32_t i;

	if (clear_recent) {
		for (i = ramlog->textbuf_base; i < ramlog->textbuf_count; i ++) {
			ocs_textbuf_reset(&ramlog->textbufs[i]);
		}
		ramlog->cur_textbuf_idx = 1;
	}
	if (clear_start_of_day && ramlog->textbuf_base) {
		ocs_textbuf_reset(&ramlog->textbufs[0]);
		/* Set textbuf_base to 0, so that all buffers are available for
		 * recent logs
		 */
		ramlog->textbuf_base = 0;
	}
}

/**
 * @brief Append formatted printf data to a ramlog buffer.
 *
 * Formatted data is appended to a RAM logging buffer.
 *
 * @param os Pointer to driver structure.
 * @param fmt Pointer to printf style format specifier.
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */

int32_t
ocs_ramlog_printf(void *os, const char *fmt, ...)
{
	ocs_t *ocs = os;
	va_list ap;
	int32_t res;

	if (ocs == NULL || ocs->ramlog == NULL) {
		return -1;
	}

	va_start(ap, fmt);
	res = ocs_ramlog_vprintf(ocs->ramlog, fmt, ap);
	va_end(ap);

	return res;
}

/**
 * @brief Append formatted text to a ramlog using variable arguments.
 *
 * Formatted data is appended to the RAM logging buffer, using variable arguments.
 *
 * @param ramlog Pointer to RAM logging buffer.
 * @param fmt Pointer to printf style formatting string.
 * @param ap Variable argument pointer.
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */

int32_t
ocs_ramlog_vprintf(ocs_ramlog_t *ramlog, const char *fmt, va_list ap)
{
	if (ramlog == NULL || !ramlog->initialized) {
		return -1;
	}

	/* check the current text buffer, if it is almost full (less than 120 characaters), then
	 * roll to the next one.
	 */
	ocs_lock(&ramlog->lock);
	if (ocs_textbuf_remaining(ramlog->cur_textbuf) < 120) {
		ramlog->cur_textbuf_idx = ocs_ramlog_next_idx(ramlog, ramlog->cur_textbuf_idx);
		ramlog->cur_textbuf = &ramlog->textbufs[ramlog->cur_textbuf_idx];
		ocs_textbuf_reset(ramlog->cur_textbuf);
	}

	ocs_textbuf_vprintf(ramlog->cur_textbuf, fmt, ap);
	ocs_unlock(&ramlog->lock);

	return 0;
}

/**
 * @brief Return next ramlog buffer index.
 *
 * Given a RAM logging buffer index, return the next index.
 *
 * @param ramlog Pointer to RAM logging buffer.
 * @param idx Index value.
 *
 * @return Returns next index value.
 */

static uint32_t
ocs_ramlog_next_idx(ocs_ramlog_t *ramlog, uint32_t idx)
{
	idx = idx + 1;

	if (idx >= ramlog->textbuf_count) {
		idx = ramlog->textbuf_base;
	}

	return idx;
}

/**
 * @brief Perform ramlog buffer driver dump.
 *
 * The RAM logging buffer is appended to the driver dump data.
 *
 * @param textbuf Pointer to the driver dump text buffer.
 * @param ramlog Pointer to the RAM logging buffer.
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */

int32_t
ocs_ddump_ramlog(ocs_textbuf_t *textbuf, ocs_ramlog_t *ramlog)
{
	uint32_t i;
	ocs_textbuf_t *rltextbuf;
	int idx;

	if ((ramlog == NULL) || (ramlog->textbufs == NULL)) {
		return -1;
	}

	ocs_ddump_section(textbuf, "driver-log", 0);

	/* Dump the start of day buffer */
	ocs_ddump_section(textbuf, "startofday", 0);
	/* If textbuf_base is 0, then all buffers are used for recent */
	if (ramlog->textbuf_base) {
		rltextbuf = &ramlog->textbufs[0];
		ocs_textbuf_buffer(textbuf, ocs_textbuf_get_buffer(rltextbuf), ocs_textbuf_get_written(rltextbuf));
	}
	ocs_ddump_endsection(textbuf, "startofday", 0);

	/* Dump the most recent buffers */
	ocs_ddump_section(textbuf, "recent", 0);

	/* start with the next textbuf */
	idx = ocs_ramlog_next_idx(ramlog, ramlog->textbuf_count);

	for (i = ramlog->textbuf_base; i < ramlog->textbuf_count; i ++) {
		rltextbuf = &ramlog->textbufs[idx];
		ocs_textbuf_buffer(textbuf, ocs_textbuf_get_buffer(rltextbuf), ocs_textbuf_get_written(rltextbuf));
		idx = ocs_ramlog_next_idx(ramlog, idx);
	}
	ocs_ddump_endsection(textbuf, "recent", 0);
	ocs_ddump_endsection(textbuf, "driver-log", 0);

	return 0;
}
