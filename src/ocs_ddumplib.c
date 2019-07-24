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
 * General diagnostics dump API
 *
 */

#include "ocs.h"
#include "ocs_ddump.h"

/**
 * @brief Generate driver dump start of file information
 *
 * The start of file information is added to 'textbuf'
 *
 * @param textbuf pointer to driver dump text buffer
 *
 * @return none
 */

void
ocs_ddump_startfile(ocs_textbuf_t *textbuf)
{
	ocs_textbuf_printf(textbuf, "<?xml version=\"1.0\" encoding=\"ISO-8859-1\" ?>\n");
}

/**
 * @brief Generate driver dump end of file information
 *
 * The end of file information is added to 'textbuf'
 *
 * @param textbuf pointer to driver dump text buffer
 *
 * @return none
 */

void
ocs_ddump_endfile(ocs_textbuf_t *textbuf)
{
}

/**
 * @brief Generate driver dump section start data
 *
 * The driver section start information is added to textbuf
 *
 * @param textbuf pointer to text buffer
 * @param name name of section
 * @param instance instance number of this section
 *
 * @return none
 */

void
ocs_ddump_section(ocs_textbuf_t *textbuf, const char *name, uint32_t instance)
{
	ocs_textbuf_printf(textbuf, "<%s type=\"section\" instance=\"%d\">\n", name, instance);
}

/**
 * @brief Generate driver dump section end data
 *
 * The driver section end information is added to textbuf
 *
 * @param textbuf pointer to text buffer
 * @param name name of section
 * @param instance instance number of this section
 *
 * @return none
 */

void
ocs_ddump_endsection(ocs_textbuf_t *textbuf, const char *name, uint32_t instance)
{
	ocs_textbuf_printf(textbuf, "</%s>\n", name);
}

/**
 * @brief Generate driver dump data for a given value
 *
 * A value is added to textbuf
 *
 * @param textbuf pointer to text buffer
 * @param name name of variable
 * @param fmt snprintf format specifier
 *
 * @return none
 */

void
ocs_ddump_value(ocs_textbuf_t *textbuf, const char *name, const char *fmt, ...)
{
	va_list ap;
	char valuebuf[64];

	va_start(ap, fmt);
	vsnprintf(valuebuf, sizeof(valuebuf), fmt, ap);
	va_end(ap);

	ocs_textbuf_printf(textbuf, "<%s>%s</%s>\n", name, valuebuf, name);
}


/**
 * @brief Generate driver dump data for an arbitrary buffer of DWORDS
 *
 * A status value is added to textbuf
 *
 * @param textbuf pointer to text buffer
 * @param name name of status variable
 * @param instance instance number of this section
 * @param buffer buffer to print
 * @param size size of buffer in bytes
 *
 * @return none
 */

void
ocs_ddump_buffer(ocs_textbuf_t *textbuf, const char *name, uint32_t instance, void *buffer, uint32_t size)
{
	uint32_t *dword;
	uint32_t i;
	uint32_t count;

	count = size / sizeof(uint32_t);

	if (count == 0) {
		return;
	}

	ocs_textbuf_printf(textbuf, "<%s type=\"buffer\" instance=\"%d\">\n", name, instance);

	dword = buffer;
	for (i = 0; i < count; i++) {
#define OCS_NEWLINE_MOD	8
		ocs_textbuf_printf(textbuf, "%08x ", *dword++);
		if ((i % OCS_NEWLINE_MOD) == (OCS_NEWLINE_MOD - 1)) {
			ocs_textbuf_printf(textbuf, "\n");
		}
	}

	ocs_textbuf_printf(textbuf, "</%s>\n", name);
}

/**
 * @brief Generate driver dump for queue
 *
 * Add queue elements to text buffer
 *
 * @param textbuf pointer to driver dump text buffer
 * @param q_addr address of start of queue
 * @param size size of each queue entry
 * @param length number of queue entries in the queue
 * @param index current index of queue
 * @param qentries number of most recent queue entries to dump
 *
 * @return none
 */

void
ocs_ddump_queue_entries(ocs_textbuf_t *textbuf, void *q_addr, uint32_t size,
			uint32_t length, int32_t index, uint32_t qentries)
{
	uint32_t i;
	uint32_t j;
	uint8_t *entry;
	uint32_t *dword;
	uint32_t entry_count = 0;
	uint32_t entry_words = size / sizeof(uint32_t);

	if ((qentries == (uint32_t)-1) || (qentries > length)) {
		/* if qentries is -1 or larger than queue size, dump entire queue */
		entry_count = length;
		index = 0;
	} else {
		entry_count = qentries;

		index -= (qentries - 1);
		if (index < 0) {
			index += length;
		}

	}
#define OCS_NEWLINE_MOD	8
	ocs_textbuf_printf(textbuf, "<qentries>\n");
	for (i = 0; i < entry_count; i++){
		entry = q_addr;
		entry += index * size;
		dword = (uint32_t *)entry;

		ocs_textbuf_printf(textbuf, "[%04x] ", index);
		for (j = 0; j < entry_words; j++) {
			ocs_textbuf_printf(textbuf, "%08x ", *dword++);
			if (((j+1) == entry_words) ||
			    ((j % OCS_NEWLINE_MOD) == (OCS_NEWLINE_MOD - 1))) {
				ocs_textbuf_printf(textbuf, "\n");
				if ((j+1) < entry_words) {
					ocs_textbuf_printf(textbuf, "       ");
				}
			}
		}

		index++;
		if ((uint32_t)index >= length) {
			index = 0;
		}
	}
	ocs_textbuf_printf(textbuf, "</qentries>\n");
}


