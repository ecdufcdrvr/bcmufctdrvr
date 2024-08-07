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
 * OCS FC SLI port (SPORT) exported declarations
 *
 */

#if !defined(__OCS_SPORT_H__)
#define __OCS_SPORT_H__

extern int32_t ocs_port_cb(void *arg, ocs_hal_port_event_e event, void *data);
extern ocs_sport_t *ocs_sport_alloc(ocs_domain_t *domain, uint64_t wwpn, uint64_t wwnn, uint32_t fc_id,
	uint8_t enable_ini, uint8_t ini_fc_types, uint8_t enable_tgt, uint8_t tgt_fc_types);
extern void ocs_sport_free(ocs_sport_t *sport);

static inline void
ocs_sport_lock_init(ocs_sport_t *sport)
{
}

static inline void
ocs_sport_lock_free(ocs_sport_t *sport)
{
}

static inline int32_t
ocs_sport_lock_try(ocs_sport_t *sport)
{
	/* Use the device wide lock */
	return ocs_device_lock_try(sport->ocs);
}

static inline void
ocs_sport_lock(ocs_sport_t *sport)
{
	/* Use the device wide lock */
	ocs_device_lock(sport->ocs);
}

static inline void
ocs_sport_unlock(ocs_sport_t *sport)
{
	/* Use the device wide lock */
	ocs_device_unlock(sport->ocs);
}

static inline uint32_t
ocs_sport_get_max_frame_size(ocs_sport_t *sport)
{
	fc_plogi_payload_t *sp = (fc_plogi_payload_t *)sport->service_params;

	return (ocs_be32toh(sp->common_service_parameters[1]) & FC_PLOGI_CSP_W1_MAXFRAME_SIZE_MASK);
}

static inline uint64_t
ocs_sport_get_fabric_name(ocs_sport_t *sport)
{
	fc_plogi_payload_t *sp = (fc_plogi_payload_t *)sport->domain->flogi_service_params;

	return (((uint64_t)ocs_be32toh(sp->node_name_hi) << 32ll) | (ocs_be32toh(sp->node_name_lo)));
}

extern ocs_sport_t *ocs_sport_find(ocs_domain_t *domain, uint32_t d_id);
extern ocs_sport_t *ocs_sport_find_wwn(ocs_domain_t *domain, uint64_t wwnn, uint64_t wwpn);
extern int32_t ocs_sport_attach(ocs_sport_t *sport, uint32_t fc_id);

extern void *__ocs_sport_allocated(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_sport_wait_shutdown(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_sport_wait_port_free(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_sport_vport_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_sport_vport_wait_alloc(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_sport_vport_allocated(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_sport_attached(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

extern int32_t ocs_vport_start(ocs_domain_t *domain);
extern int32_t ocs_sport_vport_new(ocs_domain_t *domain, uint64_t wwpn, uint64_t wwnn, uint32_t fc_id, uint8_t ini, uint8_t tgt, void *tgt_data, void *ini_data, uint8_t restore_vport);
extern int32_t ocs_sport_vport_del(ocs_t *ocs, ocs_domain_t *domain, uint64_t wwpn, uint64_t wwnn, uint8_t delete_vport);
extern void ocs_vport_del_all(ocs_t *ocs);
extern int8_t ocs_vport_create_spec(ocs_t *ocs, uint64_t wwnn, uint64_t wwpn, uint32_t fc_id, uint32_t enable_ini, uint32_t ini_fct_types, uint32_t enable_tgt, uint32_t tgt_fct_types, void *tgt_data, void *ini_data);

extern int ocs_ddump_sport(ocs_textbuf_t *textbuf, ocs_sport_t *sport);

/* Node group API */
extern int ocs_sparm_cmp(uint8_t *sparms1, uint8_t *sparms2);
extern ocs_node_group_dir_t *ocs_node_group_dir_alloc(ocs_sport_t *sport, uint8_t *sparms);
extern void ocs_node_group_dir_free(ocs_node_group_dir_t *node_group_dir);
extern ocs_node_group_dir_t *ocs_node_group_dir_find(ocs_sport_t *sport, uint8_t *sparms);
extern ocs_remote_node_group_t *ocs_remote_node_group_alloc(ocs_node_group_dir_t *node_group_dir);
extern void ocs_remote_node_group_free(ocs_remote_node_group_t *node_group);
extern int ocs_node_group_init(ocs_node_t *node);
extern void ocs_node_group_free(ocs_node_t *node);
extern void ocs_tdz_issue_fabric_cmd_cb(ocs_node_t *node, ocs_node_cb_t *node_data, void *cb_data);
extern void ocs_fdmi_get_cmd_cb(ocs_node_t *node, ocs_node_cb_t *node_data, void *cb_data);
extern void ocs_ganxt_get_cmd_cb(ocs_node_t *node, ocs_node_cb_t *node_data, void *cb_data);
extern void * __ocs_sport_wait_port_logo(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

typedef struct ocs_tdz_rsp_info_s {
	int32_t status;
	uint16_t cmd_code;
	void *rsp_buf;
	size_t rsp_buf_size;
	ocs_sport_exec_arg_t *sport_exec_args;
} ocs_tdz_rsp_info_t;

typedef struct ocs_fdmi_get_cmd_results_s {
	int32_t status;
	uint16_t fdmi_cmd_code;
	void *fdmi_get_cmd_rsp_buf;
	size_t fdmi_get_cmd_rsp_buf_len;
	ocs_sport_exec_arg_t *sport_exec_args;
} ocs_fdmi_get_cmd_results_t;

typedef struct ocs_ganxt_get_cmd_results_s {
	int32_t status;
	void *ganxt_get_cmd_rsp_buf;
	size_t ganxt_get_cmd_rsp_buf_len;
	ocs_sport_exec_arg_t *sport_exec_args;
} ocs_ganxt_get_cmd_results_t;

#endif // __OCS_SPORT_H__
