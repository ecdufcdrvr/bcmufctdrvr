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
 * Common functions used for the output formatting by ocs_mgmt_* functions.
 */

#include "ocs.h"
#include "ocs_mgmt.h"

static char *mode_string(int mode);


/**
 * @ingroup mgmt
 * @brief Generate the beginning of a numbered section in a management XML document.
 *
 * @par Description
 * This function begins a section. The XML information is appended to
 * the textbuf. This form of the function is used for sections that might have
 * multiple instances, such as a node or a SLI Port (sport). The index number
 * is appended to the name.
 *
 * @param textbuf Pointer to the driver dump text buffer.
 * @param name Name of the section.
 * @param index Index number of this instance of the section.
 *
 * @return None.
 */

extern void ocs_mgmt_start_section(ocs_textbuf_t *textbuf, const char *name, int index)
{
	ocs_textbuf_printf(textbuf, "<%s instance=\"%d\">\n", name, index);
}

/**
 * @ingroup mgmt
 * @brief Generate the beginning of an unnumbered section in a management XML document.
 *
 * @par Description
 * This function begins a section. The XML information is appended to
 * the textbuf. This form of the function is used for sections that have
 * a single instance only. Therefore, no index number is needed.
 *
 * @param textbuf Pointer to the driver dump text buffer.
 * @param name Name of the section.
 *
 * @return None.
 */

extern void ocs_mgmt_start_unnumbered_section(ocs_textbuf_t *textbuf, const char *name)
{
	ocs_textbuf_printf(textbuf, "<%s>\n", name);
}

/**
 * @ingroup mgmt
 * @brief Generate the end of a section in a management XML document.
 *
 * @par Description
 * This function ends a section. The XML information is appended to
 * the textbuf.
 *
 * @param textbuf Pointer to the driver dump text buffer.
 * @param name Name of the section.
 *
 * @return None.
 */

void ocs_mgmt_end_unnumbered_section(ocs_textbuf_t *textbuf, const char *name)
{
	ocs_textbuf_printf(textbuf, "</%s>\n", name);
}

/**
 * @ingroup mgmt
 * @brief Generate the indexed end of a section in a management XML document.
 *
 * @par Description
 * This function ends a section. The XML information is appended to
 * the textbuf.
 *
 * @param textbuf Pointer to the driver dump text buffer.
 * @param name Name of the section.
 * @param index Index number of this instance of the section.
 *
 * @return None.
 */

void ocs_mgmt_end_section(ocs_textbuf_t *textbuf, const char *name, int index)
{

	ocs_textbuf_printf(textbuf, "</%s>\n", name);

}

/**
 * @ingroup mgmt
 * @brief Generate a property, with no value, in a management XML document.
 *
 * @par Description
 * This function generates a property name. The XML information is appended to
 * the textbuf. This form of the function is used by the list functions
 * when the property name only (and not the current value) is given.
 *
 * @param textbuf Pointer to the driver dump text buffer.
 * @param mode Defines whether the property is read(r)/write(w)/executable(x).
 * @param name Name of the property.
 *
 * @return None.
 */

void ocs_mgmt_emit_property_name(ocs_textbuf_t *textbuf, int mode, const char *name)
{
	ocs_textbuf_printf(textbuf, "<%s mode=\"%s\"/>\n", name, mode_string(mode));
}

/**
 * @ingroup mgmt
 * @brief Generate a property with a string value in a management XML document.
 *
 * @par Description
 * This function generates a property name and a string value.
 * The XML information is appended to the textbuf.
 *
 * @param textbuf Pointer to the driver dump text buffer.
 * @param mode Defines whether the property is read(r)/write(w)/executable(x).
 * @param name Name of the property.
 * @param value Value of the property.
 *
 * @return None.
 */

void ocs_mgmt_emit_string(ocs_textbuf_t *textbuf, int mode, const char *name, const char *value)
{
	ocs_textbuf_printf(textbuf, "<%s mode=\"%s\">%s</%s>\n", name, mode_string(mode), value, name);
}

/**
 * @ingroup mgmt
 * @brief Generate a property with an integer value in a management XML document.
 *
 * @par Description
 * This function generates a property name and an integer value.
 * The XML information is appended to the textbuf.
 *
 * @param textbuf Pointer to driver dump text buffer.
 * @param mode Defines whether the property is read(r)/write(w)/executable(x).
 * @param name Name of the property.
 * @param fmt A printf format for formatting the integer value.
 *
 * @return none
 */

void ocs_mgmt_emit_int(ocs_textbuf_t *textbuf, int mode, const char *name, const char *fmt, ...)
{
	va_list ap;
	char valuebuf[64];

	va_start(ap, fmt);
	ocs_vsnprintf(valuebuf, sizeof(valuebuf), fmt, ap);
	va_end(ap);

	ocs_textbuf_printf(textbuf, "<%s mode=\"%s\">%s</%s>\n", name, mode_string(mode), valuebuf, name);
}

/**
 * @ingroup mgmt
 * @brief Generate a property with a boolean value in a management XML document.
 *
 * @par Description
 * This function generates a property name and a boolean value.
 * The XML information is appended to the textbuf.
 *
 * @param textbuf Pointer to the driver dump text buffer.
 * @param mode Defines whether the property is read(r)/write(w)/executable(x).
 * @param name Name of the property.
 * @param value Boolean value to be added to the textbuf.
 *
 * @return None.
 */

void ocs_mgmt_emit_boolean(ocs_textbuf_t *textbuf, int mode, const char *name, int value)
{
	char *valuebuf = value ? "true" : "false";

	ocs_textbuf_printf(textbuf, "<%s mode=\"%s\">%s</%s>\n", name, mode_string(mode), valuebuf, name);
}

static char *mode_string(int mode)
{
	static char mode_str[4];

	mode_str[0] = '\0';
	if (mode & MGMT_MODE_RD) {
		strcat(mode_str, "r");
	}
	if (mode & MGMT_MODE_WR) {
		strcat(mode_str, "w");
	}
	if (mode & MGMT_MODE_EX) {
		strcat(mode_str, "x");
	}

	return mode_str;

}
