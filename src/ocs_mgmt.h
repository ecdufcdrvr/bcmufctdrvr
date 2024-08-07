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

/**
 * @file
 * Declarations for the common functions used by ocs_mgmt.
 */


#if !defined(__OCS_MGMT_H__)
#define __OCS_MGMT_H__

#include "ocs_ioctl.h"
#include "ocs_textbuf.h"

#define OCS_MGMT_MAX_NAME 128
#define OCS_MGMT_MAX_VALUE 128

#define MGMT_MODE_RD	4
#define MGMT_MODE_WR	2
#define MGMT_MODE_EX	1
#define MGMT_MODE_RW	(MGMT_MODE_RD | MGMT_MODE_WR)
#define MGMT_MODE_MASK  (MGMT_MODE_RD | MGMT_MODE_WR | MGMT_MODE_EX)
#define MGMT_MODE_MAX   (MGMT_MODE_MASK + 1)

#define OCS_ISCSI_MGMT_CMD_PARAMS_LEN 256

/*
 * This structure is used in constructing a table of internal handler functions.
 */
typedef void (*ocs_mgmt_get_func)(ocs_t *, char *, ocs_textbuf_t*);
typedef int (*ocs_mgmt_set_func)(ocs_t *, char *, char *);
typedef int (*ocs_mgmt_action_func)(ocs_t *, char *, void *, uint32_t, void *, uint32_t);
typedef struct ocs_mgmt_table_entry_s {
	char *name;
	ocs_mgmt_get_func get_handler;
	ocs_mgmt_set_func set_handler;
	ocs_mgmt_action_func action_handler;
} ocs_mgmt_table_entry_t;

/*
 * This structure is used when defining the top level management handlers for
 * different types of objects (sport, node, domain, etc)
 */
typedef void (*ocs_mgmt_get_list_handler)(ocs_textbuf_t *textbuf, void* object);
typedef void (*ocs_mgmt_get_all_handler)(ocs_textbuf_t *textbuf, void* object);
typedef int (*ocs_mgmt_get_handler)(ocs_textbuf_t *, char *parent, char *name, void *object);
typedef int (*ocs_mgmt_set_handler)(char *parent, char *name, char *value, void *object);
typedef int (*ocs_mgmt_exec_handler)(char *parent, char *action, void *arg_in, uint32_t arg_in_length,
				      void *arg_out, uint32_t arg_out_length, void *object);

typedef struct ocs_mgmt_functions_s {
	ocs_mgmt_get_list_handler	get_list_handler;
	ocs_mgmt_get_handler		get_handler;
	ocs_mgmt_get_all_handler	get_all_handler;
	ocs_mgmt_set_handler		set_handler;
	ocs_mgmt_exec_handler		exec_handler;
} ocs_mgmt_functions_t;

// Helper functions
extern void ocs_mgmt_start_section(ocs_textbuf_t *textbuf, const char *name, int index);
extern void ocs_mgmt_start_unnumbered_section(ocs_textbuf_t *textbuf, const char *name);
extern void ocs_mgmt_end_section(ocs_textbuf_t *textbuf, const char *name, int index);
extern void ocs_mgmt_end_unnumbered_section(ocs_textbuf_t *textbuf, const char *name);
extern void ocs_mgmt_emit_property_name(ocs_textbuf_t *textbuf, int access, const char *name);
extern void ocs_mgmt_emit_string(ocs_textbuf_t *textbuf, int access, const char *name, const char *value);
__attribute__((format(printf,4,5)))
extern void ocs_mgmt_emit_int(ocs_textbuf_t *textbuf, int access, const char *name, const char *fmt, ...);
extern void ocs_mgmt_emit_boolean(ocs_textbuf_t *textbuf, int access, const char *name, const int value);
extern int parse_wwn(char *wwn_in, uint64_t *wwn_out);
extern void format_wwn(uint64_t wwn_in, char *wwn_out);

// Top level management functions - called by the ioctl
extern void ocs_mgmt_get_list(ocs_t *ocs, ocs_textbuf_t *textbuf);
extern void ocs_mgmt_get_all(ocs_t *ocs, ocs_textbuf_t *textbuf);
extern void ocs_mgmt_get_hba_info(ocs_t *ocs, ocs_textbuf_t *textbuf);
extern int ocs_mgmt_get(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf);
extern int ocs_mgmt_set(ocs_t *ocs, char *name, char *value);
extern int ocs_mgmt_exec(ocs_t *ocs, char *action, void *arg_in, uint32_t arg_in_length,
			 void *arg_out, uint32_t arg_out_length);

extern void ocs_mgmt_driver_list(ocs_textbuf_t *textbuf, void *driver);
extern void ocs_mgmt_driver_get_all(ocs_textbuf_t *textbuf, void *driver);
extern int ocs_mgmt_driver_get(ocs_textbuf_t *textbuf, char *parent, char *name, void *driver);
extern int ocs_mgmt_driver_exec(char *parent, char *action, void *arg_in, uint32_t arg_in_length,
				void *arg_out, uint32_t arg_out_length, void *driver);

extern int ocs_run_link_loopback(ocs_t *ocs, void *tx_buf, uint32_t tx_buflen,
				 void *rx_buf, uint32_t rx_buflen, uint32_t num_frames);
extern int ocs_sfp_fw_upgrade(ocs_t *ocs, void *arg_in, uint32_t arg_in_len,
                     void *arg_out, uint32_t arg_out_len);
extern int ocs_run_pci_loopback(ocs_t *ocs, void *tx_buf, uint32_t tx_buflen, void *rx_buf, uint32_t rx_buflen);
extern int ocs_set_loopback_mode(ocs_t *ocs, uint32_t arg_in);
extern int ocs_set_fc_trunk_mode(ocs_t *ocs, uint32_t trunk_mode);
extern int ocs_get_fc_trunk_info(ocs_t *ocs, uint32_t *trunk_config);
extern void ocs_capture_ras_global_params(void);
extern int ocs_ras_collect_diag_logs(ocs_t *ocs, void *tx_buf, uint32_t tx_buflen, void *rx_buf, uint32_t rx_buflen);
extern int ocs_get_dual_dump_state(ocs_t *ocs, uint32_t *dump_state);
extern int32_t ocs_read_temperature_lancer(ocs_t *ocs, void *buf, uint32_t buf_len);
extern int32_t ocs_mgmt_get_sfp(ocs_t *ocs, uint16_t page, void *buf, uint32_t buf_len);

#endif
