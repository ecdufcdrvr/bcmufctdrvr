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
 * Declarations for the interface exported by ocs_unsol.c
 */

#if !defined(__OSC_UNSOL_H__)
#define __OSC_UNSOL_H__

extern int32_t ocs_unsol_rq_thread(ocs_thread_t *mythread);
extern int32_t ocs_unsolicited_cb(void *arg, ocs_hal_sequence_t *seq);
extern int32_t ocs_node_purge_pending(ocs_node_t *node);
extern int32_t ocs_process_node_pending(ocs_node_t *node);
extern int32_t ocs_domain_process_pending(ocs_domain_t *domain);
extern int32_t ocs_domain_purge_pending(ocs_domain_t *domain);
extern int32_t ocs_dispatch_unsolicited_bls(ocs_node_t *node, ocs_hal_sequence_t *seq);
extern void ocs_domain_hold_frames(ocs_domain_t *domain);
extern void ocs_domain_accept_frames(ocs_domain_t *domain);
extern void ocs_seq_coalesce_cleanup(ocs_hal_io_t *hio, uint8_t abort_io);
extern ocs_hal_sequence_t* ocs_frame_next(ocs_list_t *pend_list, ocs_lock_t *list_lock);
extern int32_t ocs_sframe_send_bls_acc(ocs_node_t *node, ocs_hal_sequence_t *seq);
extern int32_t ocs_sframe_send_bls_rjt(ocs_node_t *node, ocs_hal_sequence_t *seq);
extern int32_t ocs_sframe_send_logo(ocs_node_t *node, uint32_t s_id, uint32_t d_id,
				    uint16_t ox_id, uint16_t rx_id);
extern int32_t ocs_sframe_send_abts(ocs_node_t *node, ocs_hal_sequence_t *seq);
extern void ocs_port_owned_abort(ocs_t *ocs, ocs_hal_io_t *hio);
extern int32_t ocs_sframe_send_ls_acc(ocs_node_t *node, uint32_t s_id, uint32_t d_id,
			uint16_t ox_id, uint16_t rx_id);
extern int32_t ocs_sframe_send_prlo_acc(ocs_node_t *node, uint32_t s_id, uint32_t d_id,
			uint16_t ox_id, uint16_t rx_id, uint8_t fc_type);

typedef struct ocs_sframe_args_s {
	uint8_t r_ctl;
	uint8_t info;
	uint32_t f_ctl;
	uint8_t type;
	void *payload;
	size_t payload_len;
	uint32_t s_id;
	uint32_t d_id;
	uint16_t ox_id;
	uint16_t rx_id;
	void (*callback)(ocs_hal_t *hal, int32_t status, void *arg);
	void *cb_arg;
} ocs_sframe_args_t;

extern int32_t ocs_sframe_common_send(ocs_node_t *node, ocs_sframe_args_t *sframe_args);

#define frame_printf(ocs, hdr, fmt, ...) \
	do { \
		char s_id_text[16]; \
		ocs_node_fcid_display(fc_be24toh((hdr)->s_id), s_id_text, sizeof(s_id_text)); \
		ocs_log_debug(ocs, "[%06x.%s] %x%x/%02x %04x/%04x: " fmt, fc_be24toh((hdr)->d_id), \
			s_id_text, (hdr)->r_ctl, (hdr)->info, (hdr)->type, \
			ocs_be16toh((hdr)->ox_id), ocs_be16toh((hdr)->rx_id), ##__VA_ARGS__); \
	} while(0)

#define frame_printf_ratelimited(ocs, hdr, fmt, ...) \
	do { \
		char s_id_text[16]; \
		ocs_node_fcid_display(fc_be24toh((hdr)->s_id), s_id_text, sizeof(s_id_text)); \
		ocs_log_debug_ratelimited(ocs, "[%06x.%s] %x%x/%02x %04x/%04x: " fmt, fc_be24toh((hdr)->d_id), \
			s_id_text, (hdr)->r_ctl, (hdr)->info, (hdr)->type, \
			ocs_be16toh((hdr)->ox_id), ocs_be16toh((hdr)->rx_id), ##__VA_ARGS__); \
	} while(0)
extern int32_t
ocs_sframe_common_send(ocs_node_t *node, ocs_sframe_args_t *sframe_args);
#endif // __OSC_UNSOL_H__
