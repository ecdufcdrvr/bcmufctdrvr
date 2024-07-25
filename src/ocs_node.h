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
 * OCS linux driver remote node callback declarations
 */

#if !defined(__OCS_NODE_H__)
#define __OCS_NODE_H__


#define node_sm_trace()  \
	do { \
		if (OCS_LOG_ENABLE_SM_TRACE(node->ocs)) \
			ocs_log_info(node->ocs, "[%s] %-20s\n", node->display_name, ocs_sm_event_name(evt)); \
	} while (0)

#define node_printf(node, fmt, ...)		ocs_log_info(node->ocs, "[%s] " fmt, node->display_name, ##__VA_ARGS__)
#define node_printf_test(node, fmt, ...)	ocs_log_test(node->ocs, "[%s] " fmt, node->display_name, ##__VA_ARGS__)
#define node_printf_err(node, fmt, ...)		ocs_log_err(node->ocs, "[%s] " fmt, node->display_name, ##__VA_ARGS__)
#define node_printf_warn(node, fmt, ...)	ocs_log_warn(node->ocs, "[%s] " fmt, node->display_name, ##__VA_ARGS__)

#if !defined(OCS_USPACE)
#define node_printf_ratelimited(node, fmt, ...)							\
			ocs_log_info_ratelimited(node->ocs, "[%s] " fmt, node->display_name, ##__VA_ARGS__)
#define node_printf_test_ratelimited(node, fmt, ...)						\
			ocs_log_test_ratelimited(node->ocs, "[%s] " fmt, node->display_name, ##__VA_ARGS__)
#define node_printf_err_ratelimited(node, fmt, ...)						\
			ocs_log_err_ratelimited(node->ocs, "[%s] " fmt, node->display_name, ##__VA_ARGS__)
#else
#define node_printf_ratelimited(node, fmt, ...)		node_printf(node, fmt, ##__VA_ARGS__)
#define node_printf_test_ratelimited(node, fmt, ...)	node_printf_test(node, fmt, ##__VA_ARGS__)
#define node_printf_err_ratelimited(node, fmt, ...)	node_printf_err(node, fmt, ##__VA_ARGS__)
#endif

#define node_sm_prologue() \
	if (evt == OCS_EVT_ENTER) { \
		ocs_strncpy(node->current_state_name, __func__, sizeof(node->current_state_name)); \
	} else if (evt == OCS_EVT_EXIT) { \
		ocs_strncpy(node->prev_state_name, node->current_state_name, sizeof(node->prev_state_name)); \
		ocs_strncpy(node->current_state_name, "invalid", sizeof(node->current_state_name)); \
	} \
	node->prev_evt = node->current_evt; \
	node->current_evt = evt;

#define std_node_state_decl(...) \
	ocs_node_t *node = NULL; \
	ocs_t *ocs = NULL; \
	node = ctx->app; \
	ocs_assert(node, NULL); \
	ocs = node->ocs; \
	ocs_assert(ocs, NULL); \
	node_sm_prologue();

#define OCS_NODEDB_PAUSE_FABRIC_LOGIN		(1U << 0)
#define OCS_NODEDB_PAUSE_NAMESERVER		(1U << 1)
#define OCS_NODEDB_PAUSE_NEW_NODES		(1U << 2)

/**
 * @brief Node SM IO Context Callback structure
 *
 * Structure used as callback argument
 */
typedef struct ocs_node_evt_prli_ctx_s {
	ocs_ls_rsp_type_e ls_rsp_type;
} ocs_node_evt_prli_ctx_t;

struct ocs_node_cb_s {
	ocs_io_t *io;			/**< SCSI IO for sending response */
	int32_t status;			/**< completion status */
	int32_t ext_status;		/**< extended completion status */
	fc_header_t *header;		/**< completion header buffer */
	void *payload;			/**< completion payload buffers */
	size_t payload_len;
	ocs_io_t *els;			/**< ELS IO object */
	ocs_node_evt_prli_ctx_t prli_ctx;
	bool flush_rqs_completed;	/**< Used in ABTS processing */
	ocs_node_t *node;
};

/**
 * @brief hold frames in pending frame list
 *
 * Unsolicited recieve frames are held on the node pending frame list, rather than
 * being processed.
 *
 * @param node pointer to node structure
 *
 * @return none
 */

static inline void
ocs_node_hold_frames(ocs_node_t *node)
{
	ocs_assert(node);
	node->hold_frames = TRUE;
}

/**
 * @brief accept frames
 *
 * Unsolicited recieve frames processed rather than being held on the node
 * pending frame list.
 *
 * @param node pointer to node structure
 *
 * @return none
 */

static inline void
ocs_node_accept_frames(ocs_node_t *node)
{
	ocs_assert(node);
	node->hold_frames = FALSE;
}

extern int32_t ocs_node_create_pool(ocs_t *ocs, uint32_t node_count);
extern void ocs_node_free_pool(ocs_t *ocs);
extern ocs_node_t *ocs_node_get_instance(ocs_t *ocs, uint32_t index);

static inline void
ocs_node_lock_init(ocs_node_t *node)
{
	ocs_rlock_init(node->ocs, &node->lock, OCS_LOCK_ORDER_NODE, "node rlock");
}

static inline void
ocs_node_lock_free(ocs_node_t *node)
{
	ocs_rlock_free(&node->lock);
}

static inline int32_t
ocs_node_lock_try(ocs_node_t *node)
{
	return ocs_rlock_try(&node->lock);
}

static inline void
ocs_node_lock(ocs_node_t *node)
{
	ocs_rlock_acquire(&node->lock);
}
static inline void
ocs_node_unlock(ocs_node_t *node)
{
	ocs_rlock_release(&node->lock);
}

/**
 * @ingroup node_common
 *
 * @brief Return node suppress_rsp state
 *
 * @return true if suppress response is enabled for the node, otherwise false
 */
static inline bool
ocs_node_suppress_resp(ocs_remote_node_t *rnode)
{
	ocs_node_t *node = (ocs_node_t *)rnode->node;

	ocs_assert(node, false);
	return node->suppress_rsp;
}

static inline bool
ocs_node_nsler_capable(ocs_node_t *node)
{
	return node->nvme_sler && node->nvme_conf;
}

static inline bool
ocs_node_nsler_negotiated(ocs_node_t *node)
{
	return ocs_node_nsler_capable(node) && ocs_nsler_capable(node->ocs);
}

typedef void* (*ocs_node_common_func_t)(const char *funcname, ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

extern int32_t node_check_els_req(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg, uint8_t cmd, ocs_node_common_func_t node_common_func, const char *funcname);
extern int32_t node_check_ct_req(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg, uint32_t cmd, ocs_node_common_func_t node_common_func, const char *funcname);
extern int32_t node_check_ct_resp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern int32_t ocs_remote_node_cb(void *arg, ocs_hal_remote_node_event_e event, void *data);
extern int32_t ocs_node_attach(ocs_node_t *node);
extern ocs_node_t *ocs_node_find(ocs_sport_t *sport, uint32_t port_id);
extern ocs_node_t *ocs_node_lookup_get(ocs_sport_t *sport, uint32_t port_id);
extern ocs_node_t *ocs_node_find_wwpn(ocs_sport_t *sport, uint64_t wwpn);
extern ocs_node_t *ocs_node_find_wwnn(ocs_sport_t *sport, uint64_t wwnn);
extern void ocs_node_dump(ocs_t *ocs);
extern ocs_node_t *ocs_node_alloc(ocs_sport_t *sport, uint32_t port_id, uint8_t init, uint8_t targ);
extern int32_t ocs_node_free(ocs_node_t *node);
extern void ocs_scsi_notify_node_force_free(ocs_node_t *node);
extern void ocs_node_fcid_display(uint32_t fc_id, char *buffer, uint32_t buffer_length);
extern void ocs_node_update_display_name(ocs_node_t *node);

extern void *__ocs_node_shutdown(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void * __ocs_node_wait_node_free(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_node_wait_els_shutdown(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_node_wait_ios_shutdown(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void ocs_node_save_sparms(ocs_node_t *node, void *payload);
extern void ocs_node_post_event(ocs_node_t *node, ocs_sm_event_t evt, void *arg);
extern void ocs_node_transition(ocs_node_t *node, ocs_sm_function_t state, void *data);
extern void *__ocs_node_common(const char *funcname, ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);


extern void ocs_node_initiate_cleanup(ocs_node_t *node);
extern int ocs_ddump_node(ocs_textbuf_t *textbuf, ocs_node_t *node);

extern void ocs_node_build_eui_name(char *buffer, uint32_t buffer_len, uint64_t eui_name);
extern uint64_t ocs_node_get_wwpn(ocs_node_t *node);
extern uint64_t ocs_node_get_wwnn(ocs_node_t *node);
extern void ocs_node_abort_all_els(ocs_node_t *node);

extern void ocs_node_pause(ocs_node_t *node, ocs_sm_function_t state);
extern int32_t ocs_node_resume(ocs_node_t *node);
extern void *__ocs_node_paused(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

extern int ocs_node_active_ios_empty(ocs_node_t *node);
extern void ocs_node_ls_rsp_io_cleanup(ocs_node_t *node);

extern int32_t ocs_node_recv_link_services_frame(ocs_node_t *node, ocs_hal_sequence_t *seq);
extern int32_t ocs_node_recv_bls_frame(ocs_node_t *node, ocs_hal_sequence_t *seq);
extern int32_t ocs_node_recv_els_frame(ocs_node_t *node, ocs_hal_sequence_t *seq);
extern int32_t ocs_node_recv_ct_frame(ocs_node_t *node, ocs_hal_sequence_t *seq);
extern int32_t ocs_node_recv_fcp_cmd(ocs_node_t *node, ocs_hal_sequence_t *seq);
extern int32_t ocs_node_recv_tow_data(ocs_node_t *node, ocs_hal_sequence_t *seq);

extern int32_t ocs_node_recv_bls_no_sit(ocs_node_t *node, ocs_hal_sequence_t *seq);

extern int32_t ocs_node_is_remote_node(ocs_remote_node_t *rnode);
extern void ocs_node_add_shutdown_list(ocs_node_t *node);
extern void ocs_node_remove_shutdown_list(ocs_node_t *node);

#endif // __OCS_NODE_H__
