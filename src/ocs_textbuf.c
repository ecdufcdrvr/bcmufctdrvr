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

#include "ocs.h"
#include "ocs_textbuf.h"

static int32_t ocs_segment_remaining(ocs_textbuf_segment_t *segment);
static ocs_textbuf_segment_t *ocs_textbuf_segment_alloc(ocs_textbuf_t *textbuf);
static void ocs_textbuf_segment_free(ocs_t *ocs, ocs_textbuf_segment_t *segment);
static ocs_textbuf_segment_t *ocs_textbuf_get_segment(ocs_textbuf_t *textbuf, uint32_t idx);

uint8_t *
ocs_textbuf_get_buffer(ocs_textbuf_t *textbuf)
{
	return ocs_textbuf_ext_get_buffer(textbuf, 0);
}

int32_t
ocs_textbuf_get_length(ocs_textbuf_t *textbuf)
{
	return ocs_textbuf_ext_get_length(textbuf, 0);
}

int32_t
ocs_textbuf_get_written(ocs_textbuf_t *textbuf)
{
	uint32_t idx;
	int32_t n;
	int32_t total = 0;

	for (idx = 0; (n = ocs_textbuf_ext_get_written(textbuf, idx)) >= 0; idx++) {
		total += n;
	}
	return total;
}

uint8_t *ocs_textbuf_ext_get_buffer(ocs_textbuf_t *textbuf, uint32_t idx)
{
	ocs_textbuf_segment_t *segment = ocs_textbuf_get_segment(textbuf, idx);
	if (segment == NULL) {
		return NULL;
	}
	return segment->buffer;
}

int32_t ocs_textbuf_ext_get_length(ocs_textbuf_t *textbuf, uint32_t idx)
{
	ocs_textbuf_segment_t *segment = ocs_textbuf_get_segment(textbuf, idx);
	if (segment == NULL) {
		return -1;
	}
	return segment->buffer_length;
}

int32_t ocs_textbuf_ext_get_written(ocs_textbuf_t *textbuf, uint32_t idx)
{
	ocs_textbuf_segment_t *segment = ocs_textbuf_get_segment(textbuf, idx);
	if (segment == NULL) {
		return -1;
	}
	return segment->buffer_written;
}

uint32_t
ocs_textbuf_initialized(ocs_textbuf_t *textbuf)
{
	uint32_t rc = false;

	if (textbuf)
		rc = (textbuf->ocs != NULL);

	return rc;
}

int32_t
ocs_textbuf_alloc(ocs_t *ocs, ocs_textbuf_t *textbuf, uint32_t length)
{
	ocs_memset(textbuf, 0, sizeof(*textbuf));

	textbuf->ocs = ocs;
	ocs_list_init(&textbuf->segment_list, ocs_textbuf_segment_t, link);

	if (length > OCS_TEXTBUF_MAX_ALLOC_LEN) {
		textbuf->allocation_length = OCS_TEXTBUF_MAX_ALLOC_LEN;
	} else {
		textbuf->allocation_length = length;
	}

	/* save maximum allocation length */
	textbuf->max_allocation_length = length;

	textbuf->current_seg = ocs_textbuf_segment_alloc(textbuf);

	/* Add first segment */
	return (textbuf->current_seg == NULL) ? -1 : 0;
}

int32_t
ocs_textbuf_pool_alloc(ocs_t *ocs, ocs_textbuf_t *textbuf, uint32_t length)
{
	uint32_t remaining_len = length;

	ocs_memset(textbuf, 0, sizeof(*textbuf));
	textbuf->ocs = ocs;
	ocs_list_init(&textbuf->segment_list, ocs_textbuf_segment_t, link);

	/* save maximum allocation length */
	textbuf->max_allocation_length = length;

	while (remaining_len) {
		textbuf->allocation_length = (remaining_len > OCS_TEXTBUF_MAX_ALLOC_LEN) ?
						OCS_TEXTBUF_MAX_ALLOC_LEN : remaining_len;

		if (ocs_textbuf_segment_alloc(textbuf) == NULL)
			goto fail;

		if (remaining_len <= OCS_TEXTBUF_MAX_ALLOC_LEN)
			break;

		remaining_len -= textbuf->allocation_length;
	}

	textbuf->current_seg = ocs_list_get_head(&textbuf->segment_list);
	return 0;

fail:
	ocs_textbuf_free(ocs, textbuf);
	return -1;
}

static ocs_textbuf_segment_t *
ocs_textbuf_segment_alloc(ocs_textbuf_t *textbuf)
{
	ocs_textbuf_segment_t *segment = NULL;

	if (textbuf->pre_allocated)
		return NULL;

	segment = ocs_malloc(textbuf->ocs, sizeof(*segment), OCS_M_ZERO | OCS_M_NONUMA);
	if (segment != NULL) {
		segment->buffer = ocs_malloc(textbuf->ocs, textbuf->allocation_length, OCS_M_ZERO | OCS_M_NONUMA);
		if (segment->buffer != NULL) {
			segment->buffer_length = textbuf->allocation_length;
			segment->buffer_written = 0;
			ocs_list_add_tail(&textbuf->segment_list, segment);
			textbuf->total_allocation_length += textbuf->allocation_length;

			/* If we've allocated our limit, then mark as not extendable */
			if (textbuf->total_allocation_length >= textbuf->max_allocation_length)
				textbuf->pre_allocated = 1;
		} else {
			ocs_textbuf_segment_free(textbuf->ocs, segment);
			segment = NULL;
		}
	}

	return segment;
}

static void
ocs_textbuf_segment_free(ocs_t *ocs, ocs_textbuf_segment_t *segment)
{
	if (segment) {
		if (segment->buffer && !segment->user_allocated) {
			ocs_free(ocs, segment->buffer, segment->buffer_length);
		}
		ocs_free(ocs, segment, sizeof(*segment));
	}
}

static ocs_textbuf_segment_t *
ocs_textbuf_get_segment(ocs_textbuf_t *textbuf, uint32_t idx)
{
	uint32_t i;
	ocs_textbuf_segment_t *segment;

	if (ocs_textbuf_initialized(textbuf)) {
		i = 0;
		ocs_list_foreach(&textbuf->segment_list, segment) {
			if (i == idx) {
				return segment;
			}
			i++;
		}
	}
	return NULL;
}

void
ocs_textbuf_free(ocs_t *ocs, ocs_textbuf_t *textbuf)
{
	ocs_textbuf_segment_t *segment;
	ocs_textbuf_segment_t *n;

	if (ocs_textbuf_initialized(textbuf)) {
		ocs_list_foreach_safe(&textbuf->segment_list, segment, n) {
			ocs_list_remove(&textbuf->segment_list, segment);
			ocs_textbuf_segment_free(ocs, segment);
		}

		ocs_memset(textbuf, 0, sizeof(*textbuf));
	}
}

/**
 * @brief Helper function to dump message
 *
 * @param textbuf pointer to driver dump text buffer.
 * If valid, dump message to textbuf
 * If NULL, dump message to kernel dmesg
 *
 * @return none
 */
void
ocs_textbuf_printf(ocs_textbuf_t *textbuf, const char *fmt, ...)
{
	va_list ap;
	char valuebuf[256];

	va_start(ap, fmt);
	if (textbuf && ocs_textbuf_initialized(textbuf)) {
		ocs_textbuf_vprintf(textbuf, fmt, ap);
	} else {
		ocs_vsnprintf(valuebuf, sizeof(valuebuf), fmt, ap);
		ocs_log_info(NULL, "%s", valuebuf);
	}
	va_end(ap);
}

void
ocs_textbuf_vprintf(ocs_textbuf_t *textbuf, const char *fmt, va_list ap)
{
	int avail;
	int written;
	ocs_textbuf_segment_t *segment;
	va_list save_ap;

	if (!ocs_textbuf_initialized(textbuf)) {
		return;
	}

	va_copy(save_ap, ap);

	/* fetch last written segment */
	segment = textbuf->current_seg;

	avail = ocs_segment_remaining(segment);
	if (avail == 0) {
		if ((segment = ocs_textbuf_segment_alloc(textbuf)) == NULL) {
			goto out;
		}
		avail = ocs_segment_remaining(segment);
	}

	written = ocs_vsnprintf(segment->buffer + segment->buffer_written, avail, fmt, ap);

	/* See if data was truncated */
	if (written >= avail) {
		written = avail;
		/* revert the partially written data */
		*(segment->buffer + segment->buffer_written) = 0;

		if (textbuf->pre_allocated) {
			segment = ocs_list_next(&textbuf->segment_list, textbuf->current_seg);
		} else {
			/* Allocate a new segment */
			segment = ocs_textbuf_segment_alloc(textbuf);
		}

		if (segment == NULL) {
			ocs_log_err(textbuf->ocs, "alloc segment failed\n");
			goto out;
		}

		textbuf->current_seg = segment;
		avail = ocs_segment_remaining(segment);

		/* Retry the write */
		written = ocs_vsnprintf(segment->buffer + segment->buffer_written, avail, fmt, save_ap);
	}
	segment->buffer_written += written;

out:
	va_end(save_ap);
}

void
ocs_textbuf_putc(ocs_textbuf_t *textbuf, uint8_t c)
{
	ocs_textbuf_segment_t *segment;

	if (ocs_textbuf_initialized(textbuf)) {
		segment = ocs_list_get_tail(&textbuf->segment_list);

		if (ocs_segment_remaining(segment)) {
			*(segment->buffer + segment->buffer_written++) = c;
		}
		if (ocs_segment_remaining(segment) == 0) {
			ocs_textbuf_segment_alloc(textbuf);
		}
	}
}

void
ocs_textbuf_puts(ocs_textbuf_t *textbuf, char *s)
{
	if (ocs_textbuf_initialized(textbuf)) {
		while(*s) {
			ocs_textbuf_putc(textbuf, *s++);
		}
	}
}

void
ocs_textbuf_buffer(ocs_textbuf_t *textbuf, uint8_t *buffer, uint32_t buffer_length)
{
	char *s;

	if (!ocs_textbuf_initialized(textbuf) || !buffer) {
		return;
	}

	s = (char*) buffer;
	while(*s) {

		/*
		 * XML escapes
		 *
		 * "   &quot;
		 * '   &apos;
		 * <   &lt;
		 * >   &gt;
		 * &   &amp;
		 */

		switch(*s) {
		case '"':	ocs_textbuf_puts(textbuf, "&quot;"); break;
		case '\'':	ocs_textbuf_puts(textbuf, "&apos;"); break;
		case '<':	ocs_textbuf_puts(textbuf, "&lt;"); break;
		case '>':	ocs_textbuf_puts(textbuf, "&gt;"); break;
		case '&':	ocs_textbuf_puts(textbuf, "&amp;"); break;
		default:	ocs_textbuf_putc(textbuf, *s); break;
		}
		s++;
	}

}

void
ocs_textbuf_copy(ocs_textbuf_t *textbuf, uint8_t *buffer, uint32_t buffer_length)
{
	char *s;

	if (!ocs_textbuf_initialized(textbuf)) {
		return;
	}

	s = (char*) buffer;
	while(*s) {
		ocs_textbuf_putc(textbuf, *s++);
	}

}

int32_t
ocs_textbuf_remaining(ocs_textbuf_t *textbuf)
{
	if (ocs_textbuf_initialized(textbuf)) {
		return ocs_segment_remaining(ocs_list_get_head(&textbuf->segment_list));
	} else {
		return 0;
	}
}

static int32_t
ocs_segment_remaining(ocs_textbuf_segment_t *segment)
{
	if (!segment)
		return 0;

	return segment->buffer_length - segment->buffer_written;
}

void
ocs_textbuf_reset(ocs_textbuf_t *textbuf)
{
	uint32_t i = 0;
	ocs_textbuf_segment_t *segment;
	ocs_textbuf_segment_t *n;

	if (ocs_textbuf_initialized(textbuf)) {
		textbuf->current_seg = ocs_list_get_head(&textbuf->segment_list);
		/* Mark the 'buffer_written' as zero for pre_allocated / first segment and free the rest */
		ocs_list_foreach_safe(&textbuf->segment_list, segment, n) {
			if (textbuf->pre_allocated || (i++ == 0)) {
				segment->buffer_written = 0;
			} else {
				ocs_list_remove(&textbuf->segment_list, segment);
				ocs_textbuf_segment_free(textbuf->ocs, segment);
			}
		}
	}
}
