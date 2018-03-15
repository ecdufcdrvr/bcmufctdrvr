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
	return (textbuf->ocs != NULL);
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

	/* mark as extendable */
	textbuf->extendable = TRUE;

	/* save maximum allocation length */
	textbuf->max_allocation_length = length;

	/* Add first segment */
	return (ocs_textbuf_segment_alloc(textbuf) == NULL) ? -1 : 0;
}

static ocs_textbuf_segment_t *
ocs_textbuf_segment_alloc(ocs_textbuf_t *textbuf)
{
	ocs_textbuf_segment_t *segment = NULL;

	if (textbuf->extendable) {
		segment = ocs_malloc(textbuf->ocs, sizeof(*segment), OCS_M_ZERO | OCS_M_NOWAIT);
		if (segment != NULL) {
			segment->buffer = ocs_malloc(textbuf->ocs, textbuf->allocation_length, OCS_M_ZERO | OCS_M_NOWAIT);
			if (segment->buffer != NULL) {
				segment->buffer_length = textbuf->allocation_length;
				segment->buffer_written = 0;
				ocs_list_add_tail(&textbuf->segment_list, segment);
				textbuf->total_allocation_length += textbuf->allocation_length;

				/* If we've allocated our limit, then mark as not extendable */
				if (textbuf->total_allocation_length >= textbuf->max_allocation_length) {
					textbuf->extendable = 0;
				}

			} else {
				ocs_textbuf_segment_free(textbuf->ocs, segment);
				segment = NULL;
			}
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

int32_t
ocs_textbuf_init(ocs_t *ocs, ocs_textbuf_t *textbuf, void *buffer, uint32_t length)
{
	int32_t rc = -1;
	ocs_textbuf_segment_t *segment;

	ocs_memset(textbuf, 0, sizeof(*textbuf));

	textbuf->ocs = ocs;
	ocs_list_init(&textbuf->segment_list, ocs_textbuf_segment_t, link);
	segment = ocs_malloc(ocs, sizeof(*segment), OCS_M_ZERO | OCS_M_NOWAIT);
	if (segment) {
		segment->buffer = buffer;
		segment->buffer_length = length;
		segment->buffer_written = 0;
		segment->user_allocated = 1;
		ocs_list_add_tail(&textbuf->segment_list, segment);
		rc = 0;
	}

	return rc;
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

void
ocs_textbuf_printf(ocs_textbuf_t *textbuf, const char *fmt, ...)
{
	va_list ap;

	if (ocs_textbuf_initialized(textbuf)) {
		va_start(ap, fmt);
		ocs_textbuf_vprintf(textbuf, fmt, ap);
		va_end(ap);
	}
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

	/* fetch last segment */
	segment = ocs_list_get_tail(&textbuf->segment_list);

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

		if (textbuf->extendable) {

			/* revert the partially written data */
			*(segment->buffer + segment->buffer_written) = 0;

			/* Allocate a new segment */
			if ((segment = ocs_textbuf_segment_alloc(textbuf)) == NULL) {
				ocs_log_err(textbuf->ocs, "%s: alloc segment failed\n", __func__);
				goto out;
			}
			avail = ocs_segment_remaining(segment);

			/* Retry the write */
			written = ocs_vsnprintf(segment->buffer + segment->buffer_written, avail, fmt, save_ap);
		}
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

	if (!ocs_textbuf_initialized(textbuf)) {
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
	return segment->buffer_length - segment->buffer_written;
}

void
ocs_textbuf_reset(ocs_textbuf_t *textbuf)
{
	uint32_t i = 0;
	ocs_textbuf_segment_t *segment;
	ocs_textbuf_segment_t *n;

	if (ocs_textbuf_initialized(textbuf)) {
		/* zero written on the first segment, free the rest */
		ocs_list_foreach_safe(&textbuf->segment_list, segment, n) {
			if (i++ == 0) {
				segment->buffer_written = 0;
			} else {
				ocs_list_remove(&textbuf->segment_list, segment);
				ocs_textbuf_segment_free(textbuf->ocs, segment);
			}
		}
	}
}
