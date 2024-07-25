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
 * OCS linux driver IO declarations
 */

#if !defined(__OCS_IO_H__)
#define __OCS_IO_H__

#include "ocs_common.h"

#if !defined(OCS_DEBUG_TRACK_IO_PROC_TIME)

#define ocs_scsi_io_clear_proc_ticks(...)
#define ocs_scsi_io_get_proc_ticks(...)
#define ocs_scsi_io_set_proc_ticks(...)
#define ocs_scsi_io_proc_ticks_elapsed(...)
#define ocs_scsi_io_proc_check_time(...)
#define ocs_ddump_scsi_io_proc_time(...)

#else

inline void ocs_scsi_io_clear_proc_ticks(ocs_io_t *io, uint32_t stage);
inline void ocs_scsi_io_set_proc_ticks(ocs_io_t *io, uint32_t stage);
inline uint64_t ocs_scsi_io_get_proc_ticks(ocs_io_t *io, uint32_t stage);
inline int64_t ocs_scsi_io_proc_ticks_elapsed(uint64_t tstart, uint64_t tend);
inline void ocs_scsi_io_proc_check_time(ocs_io_t *io, uint32_t from, uint32_t to);
inline void ocs_ddump_scsi_io_proc_time(ocs_textbuf_t *textbuf, ocs_io_t *io);

#endif

#define io_error_log(io, fmt, ...)  \
	do { \
		if (OCS_LOG_ENABLE_IO_ERRORS(io->ocs)) \
			ocs_log_warn(io->ocs, fmt, ##__VA_ARGS__); \
	} while (0)

/**
 * @brief FCP IO context
 *
 * This structure is used for transport and backend IO requests and responses.
 */

#define SCSI_CMD_BUF_LENGTH		48
#define SCSI_RSP_BUF_LENGTH		sizeof(fcp_rsp_iu_t)
#define OCS_INVALID_FC_TASK_TAG		UINT32_MAX

/**
 * @brief OCS IO types
 */
typedef enum {
	OCS_IO_TYPE_IO = 0,
	OCS_IO_TYPE_ELS,
	OCS_IO_TYPE_CT,
	OCS_IO_TYPE_CT_RESP,
	OCS_IO_TYPE_BLS_RESP,
	OCS_IO_TYPE_MAX,		/**< must be last */
} ocs_io_type_e;

typedef struct els_loopback_io_evt_s {
	bool	is_loopback_frame;
	void	*loopback_rx_data;
	uint32_t loopback_rx_data_len;
} els_loopback_io_evt_t;

typedef struct bls_abort_params_s {
	ocs_io_t *tmfio;		/**< Pointer to the TMF IO context */
	void *cb_fn;			/**< TMF IO completion callback function */
	void *cb_arg;			/**< TMF IO completion callback argument */
	ocs_scsi_tmf_resp_e rspcode;	/**< TMF IO response code */
	ocs_list_link_t bls_abort_link;	/**< BLS abort req / rsp list link */
	ocs_list_link_t bls_aborted_link; /**< Local BLS aborted list link */
} bls_abort_params_t;

typedef enum {
	OCS_IO_POOL_SCSI = 1,
	OCS_IO_POOL_ELS,
} ocs_io_pool_type_e;

typedef struct ocs_io_scsi_s {
	ocs_scsi_sgl_t *sgl;		/**< SGL */
	uint32_t sgl_allocated;		/**< Number of allocated SGEs */
	uint32_t sgl_count;		/**< Number of SGEs in this SGL */
	ocs_scsi_ini_io_t ini_io;	/**< backend initiator private IO data */
	ocs_scsi_tgt_io_t tgt_io;	/**< backend target private IO data */
	ocs_scsi_rsp_io_cb_t scsi_ini_cb; /**< initiator callback function */
	void *scsi_ini_cb_arg;		/**< initiator callback function argument */
	ocs_scsi_io_cb_t scsi_tgt_cb;	/**< target callback function */
	void *scsi_tgt_cb_arg;		/**< target callback function argument */
	ocs_hal_dif_info_t hal_dif;	/**< HAL formatted DIF parameters */
	ocs_scsi_dif_info_t scsi_dif_info;	/**< DIF info saved for DIF error recovery */
	ocs_scsi_tmf_cmd_e tmf_cmd;	/**< TMF command being processed */
	uint8_t   trecv_wqe_timer;	/**< Timeout value in seconds for treceive wqe> */
	ocs_dma_t ovfl_sgl;		/**< Overflow SGL */
	ocs_dma_t cmdbuf;		/**< SCSI Command buffer, used for CDB (initiator) */
	ocs_dma_t rspbuf;		/**< SCSI Response buffer (i+t) */
	ocs_io_t *ul_io;		/**< Upper level IO */
} ocs_io_scsi_t;

typedef struct ocs_io_els_s {
	/* for ELS requests/responses */
	bool	els_pend;		/**< True if ELS is pending: locking: active_ios_lock */
	bool	els_active;		/**< True if ELS is active: locking: active_ios_lock */
	bool	els_rjt;		/**< True if ELS IO has to send reject response: locking: active_ios_lock */
	ocs_dma_t els_req;		/**< ELS request payload buffer */
	ocs_dma_t els_rsp;		/**< ELS response payload buffer */
	ocs_sm_ctx_t els_sm;		/**< EIO IO state machine context */
	uint32_t els_evtdepth;		/**< current event posting nesting depth */
	char current_state_name[OCS_DISPLAY_NAME_LENGTH];	/**< ELS SM current state */
	char prev_state_name[OCS_DISPLAY_NAME_LENGTH];		/**< ELS SM prev state */
	ocs_sm_event_t current_evt;				/**< ELS SM current event */
	ocs_sm_event_t prev_evt;				/**< ELS SM prev event */

	bool els_req_free;		/**< this els is to be free'd */
	uint32_t els_retries_remaining;	/*<< Retries remaining */
	void (*els_callback)(ocs_node_t *node, ocs_node_cb_t *cbdata, void *cbarg);
	void *els_callback_arg;
	uint32_t els_timeout_sec;	/**< timeout */
	els_loopback_io_evt_t loopback_evt_data;
	uint32_t ls_rsp_did;		/**< D_ID for pending accept */
	uint32_t ls_rsp_oxid;		/**< OX_ID for pending accept */
	ocs_ls_rsp_type_e ls_rsp_type;	/**< type of LS rsp to send */
	uint8_t ls_cmd_code;		/**< LS request command code */
} ocs_io_els_t;

struct ocs_io_s {
	ocs_t *ocs;			/**< pointer back to ocs */
	uint32_t instance_index;	/**< unique instance index value */
	const char *display_name;	/**< display name */
	ocs_node_t *node;		/**< pointer to node */
	ocs_list_link_t io_alloc_link;	/**< (io_pool->io_free_list) free list link */
#ifdef OCS_GEN_ABORTS
	ocs_list_link_t send_abort_link;	/* < Used for internal random ABORT generation */
#endif
	uint32_t eq_idx;		/**< Index of eq, if this IO is associated with unsol command */
	uint32_t init_task_tag;		/**< initiator task tag (OX_ID) for back-end and SCSI logging */
	uint32_t tgt_task_tag;		/**< target task tag (RX_ID) - for back-end and SCSI logging */
	uint32_t hw_tag;		/**< HW layer unique IO id - for back-end and SCSI logging */
	uint32_t tag;			/**< unique IO identifier */
	ocs_timer_t delay_timer;	/**< delay timer */

	ocs_io_scsi_t *scsi_info;
	ocs_io_els_t *els_info;

	uint32_t exp_xfer_len;		/**< expected data transfer length, based on FC or iSCSI header */
	ocs_mgmt_functions_t *mgmt_functions;

	/* Declarations private to HAL/SLI */
	void *hal_priv;			/**< HAL private context */

	/* Declarations private to FC Transport */
	ocs_io_type_e io_type;		/**< indicates what this ocs_io_t structure is used for */
	ocs_io_pool_type_e io_pool_type; /** < indicate which pool this IO object created from */
	ocs_ref_t ref;			/**< refcount object */
	ocs_hal_io_t *hio;		/**< HAL IO context */
	int32_t wqe_status;		/**< HAL IO latched WQE status */
	int32_t wqe_ext_status;		/**< HAL IO latched WQE extended status */
	bool	auto_resp;		/**< set if auto_trsp was set */
	bool	low_latency;		/**< set if low latency request */
	uint8_t	 wq_steering;		/**< selected WQ steering request */
	uint8_t	 wq_class;		/**< selected WQ class if steering is class */
	uint64_t xfer_req;		/**< transfer size for current request */
	size_t transferred;		/**< Number of bytes transfered so far */
	ocs_scsi_io_cb_t bls_cb;	/**< BLS callback function */
	void *bls_cb_arg;		/**< BLS callback function argument */
	uint16_t abort_rx_id;		/**< rx_id from the ABTS that initiated the command abort */

	bool	cmd_tgt;		/**< True if this is a Target command: locking: active_ios_lock  */
	bool	cmd_ini;		/**< True if this is an Initiator command: locking: active_ios_lock  */
	bool	seq_init;		/**< True if local node has sequence initiative */
	ocs_hal_io_param_t iparam;	/**< iparams for hal io send call */
	ocs_hal_io_type_e hio_type;	/**< HAL IO type */
	uint64_t wire_len;		/**< wire length */
	void *hal_cb;			/**< saved HAL callback */
	ocs_list_link_t io_pending_link;/**< Used for io_pending_list */
	ocs_list_link_t io_cancel_pending_link; /**< Used for local io_cancel_pending_list */
	ocs_list_link_t tmf_abort_link;	/**< list to store list of IOs to which
					     abort ABORT_IO_NO_RESP evt needs to be posted */
	bool tmf_abort;			/**< True when IO marked for TMF abort */
	bool abts_received;		/**< True when IO received an ABTS request */

	ocs_lock_t bls_abort_lock;	/**< Lock to synchronize multiple abort contexts */
	ocs_list_t bls_abort_req_list;	/**< List to store multiple abort request contexts */
	ocs_list_t bls_abort_rsp_list;	/**< List to store multiple abort response contexts */
	ocs_list_link_t link;		/**< linked list link */
	ocs_list_link_t pend_link;		/**< used for freeing pending els */
	uint32_t  timeout;		/**< Timeout value in seconds for this IO */
	uint32_t  app_id;		/**< Source VM header ID */
	uint8_t   cs_ctl;		/**< CS_CTL priority for this IO */
	uint8_t   io_free;		/**< Is io object in freelist > */
	ocs_rlock_t state_lock;		/**< Lock to set IO state */
#if defined(OCS_DEBUG_TRACK_IO_PROC_TIME)
	uint64_t proc_time[OCS_SCSI_IO_PROC_CMD_STAGE_MAX];
#endif
};

/**
 * @brief common IO callback argument
 *
 * Callback argument used as common I/O callback argument
 */

typedef struct {
	int32_t status;				/**< completion status */
	int32_t ext_status;			/**< extended completion status */
	void *app;				/**< application argument */
} ocs_io_cb_arg_t;

/**
 * @brief Test if IO object is busy
 *
 * Return True if IO object is busy.   Busy is defined as the IO object not being on
 * the free list
 *
 * @param io Pointer to IO object
 *
 * @return returns True if IO is busy
 */

static inline int32_t
ocs_io_busy(ocs_io_t *io)
{
	return !(io->io_free);
}

static inline void
ocs_io_invoke_hal_cb(ocs_io_t *io, int32_t status)
{
	if (io && io->hal_cb) {
		ocs_hal_done_t cb = io->hal_cb;

		io->hal_cb = NULL;
		cb(io->hio, NULL, 0, status, 0, io);
	}
}

typedef struct ocs_io_pool_cache_s ocs_io_pool_cache_t;
typedef struct ocs_io_pool_s ocs_io_pool_t;

extern ocs_io_pool_t *ocs_io_pool_create(ocs_t *ocs, uint32_t num_io, uint32_t num_sgl, bool els_pool);
extern int32_t ocs_io_pool_free(ocs_io_pool_t *io_pool);
extern uint32_t ocs_io_pool_allocated(ocs_io_pool_t *io_pool);

extern ocs_io_t *ocs_io_pool_io_alloc(ocs_io_pool_t *io_pool, uint32_t eq_idx);
extern void ocs_io_pool_io_free(ocs_io_pool_t *io_pool, ocs_io_t *io);
extern ocs_io_t *ocs_scsi_io_find_and_ref_get(ocs_io_t *tmfio, uint16_t ox_id, uint16_t rx_id);
extern ocs_io_t *ocs_io_find_init_els_io(ocs_t *ocs, ocs_node_t *node, uint16_t ox_id);
extern void ocs_ddump_io(ocs_textbuf_t *textbuf, ocs_io_t *io);

#endif // __OCS_IO_H__

/* vim: set noexpandtab textwidth=120: */
