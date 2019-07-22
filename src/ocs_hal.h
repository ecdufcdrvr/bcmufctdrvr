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
 * Defines the Hardware Abstraction Layer (HAL) interface functions.
 */

#ifndef _OCS_HAL_H
#define _OCS_HAL_H

#include "sli4.h"
#include "sli4_fc.h"
#include "ocs_hal_config.h"
#include "ocs_stats.h"
#include "ocs_array.h"

typedef struct ocs_hal_io_s ocs_hal_io_t;

#include "ocs_hal_workaround.h"

#if defined(OCS_INCLUDE_DEBUG)
#include "ocs_debug.h"
#else
#define ocs_queue_history_wq(...)
#define ocs_queue_history_cqe(...)
#define ocs_queue_history_init(...)
#define ocs_queue_history_free(...)
#endif

/**
 * @brief HAL queue forward declarations
 */
typedef struct hal_eq_s hal_eq_t;
typedef struct hal_cq_s hal_cq_t;
typedef struct hal_mq_s hal_mq_t;
typedef struct hal_wq_s hal_wq_t;
typedef struct hal_rq_s hal_rq_t;
typedef struct hal_rq_grp_s hal_rq_grp_t;

/* HAL asserts/verify
 *
 */

extern void _ocs_hal_assert(const char *cond, const char *filename, int linenum);
extern void _ocs_hal_verify(const char *cond, const char *filename, int linenum);

#if defined(HAL_NDEBUG)
#define ocs_hal_assert(cond)
#define ocs_hal_verify(cond, ...)
#else
#define ocs_hal_assert(cond) \
	do { \
		if ((!(cond))) { \
			_ocs_hal_assert(#cond, __FILE__, __LINE__); \
		} \
	} while (0)

#define ocs_hal_verify(cond, ...) \
	do { \
		if ((!(cond))) { \
			_ocs_hal_verify(#cond, __FILE__, __LINE__); \
			return __VA_ARGS__; \
		} \
	} while (0)
#endif
#define ocs_hal_verify_arg(cond)	ocs_hal_verify(cond, OCS_HAL_RTN_INVALID_ARG)

/*
 * HAL completion loop control parameters.
 *
 * The HAL completion loop must terminate periodically to keep the OS happy.  The
 * loop terminates when a predefined time has elapsed, but to keep the overhead of
 * computing time down, the time is only checked after a number of loop iterations
 * has completed.
 *
 * OCS_HAL_TIMECHECK_ITERATIONS		number of loop iterations between time checks
 *
 */

#define OCS_HAL_TIMECHECK_ITERATIONS	100
#define OCS_HAL_MAX_NUM_MQ 1

#define OCS_HAL_MAX_MRQ_SETS 2
#define OCE_HAL_MAX_NUM_MRQ_PAIRS 16 // MAX pairs in MRQ SET 

/* Max we have 8 RX filters and two can be used for MRQ set of 16.
 * So total MAX_RQ_PAIRS is 6(normal RQ) + (2 * 16)Sets = 38 RQ Pairs */
#define OCS_HAL_MAX_RQ_PAIRS 38
#define OCS_HAL_MAX_NUM_RQ (OCS_HAL_MAX_RQ_PAIRS * 2)
#define OCS_HAL_MAX_NUM_EQ 128
#define OCS_HAL_MAX_NUM_WQ 128


#define OCS_HAL_MAX_WQ_CLASS	4
#define OCS_HAL_MAX_WQ_CPU	128

/*
 * A CQ will be assinged to each WQ (CQ must have 2X entries of the WQ for abort
 * processing), plus a separate one for each RQ PAIR and one for MQ
 */
#define OCS_HAL_MAX_NUM_CQ ((OCS_HAL_MAX_NUM_WQ*2) + 1 + (OCS_HAL_MAX_RQ_PAIRS))

/*
 * Q hash - size is the maximum of all the queue sizes, rounded up to the next
 * power of 2
 */
#define OCS_HAL_Q_HASH_SIZE	B32_NEXT_POWER_OF_2(OCS_MAX(OCS_HAL_MAX_NUM_MQ, OCS_MAX(OCS_HAL_MAX_NUM_RQ, \
				OCS_MAX(OCS_HAL_MAX_NUM_EQ, OCS_MAX(OCS_HAL_MAX_NUM_WQ, \
				OCS_HAL_MAX_NUM_CQ)))))

#define OCS_HAL_RQ_HEADER_SIZE	128
#define OCS_HAL_RQ_HEADER_INDEX	0

/**
 * @brief Options for ocs_hal_command().
 */
enum {
	OCS_CMD_POLL,	/**< command executes synchronously and busy-waits for completion */
	OCS_CMD_NOWAIT,	/**< command executes asynchronously. Uses callback */
};

typedef enum {
	OCS_HAL_RTN_SUCCESS = 0,
	OCS_HAL_RTN_SUCCESS_SYNC = 1,
	OCS_HAL_RTN_ERROR = -1,
	OCS_HAL_RTN_NO_RESOURCES = -2,
	OCS_HAL_RTN_NO_MEMORY = -3,
	OCS_HAL_RTN_IO_NOT_ACTIVE = -4,
	OCS_HAL_RTN_IO_ABORT_IN_PROGRESS = -5,
	OCS_HAL_RTN_IO_PORT_OWNED_ALREADY_ABORTED = -6,
	OCS_HAL_RTN_INVALID_ARG = -7,
} ocs_hal_rtn_e;
#define OCS_HAL_RTN_IS_ERROR(e)	((e) < 0)

typedef enum {
	OCS_HAL_RESET_FUNCTION,
	OCS_HAL_RESET_FIRMWARE,
	OCS_HAL_RESET_MAX
} ocs_hal_reset_e;

typedef enum {
	OCS_HAL_N_IO,
	OCS_HAL_N_SGL,
	OCS_HAL_MAX_IO,
	OCS_HAL_MAX_SGE,
	OCS_HAL_MAX_SGL,
	OCS_HAL_MAX_NODES,
	OCS_HAL_MAX_RQ_ENTRIES,
	OCS_HAL_TOPOLOGY,	/**< auto, nport, loop */
	OCS_HAL_WWN_NODE,
	OCS_HAL_WWN_PORT,
	OCS_HAL_FW_REV,
	OCS_HAL_FW_REV2,
	OCS_HAL_IPL,
	OCS_HAL_VPD,
	OCS_HAL_VPD_LEN,
	OCS_HAL_MODE,		/**< initiator, target, both */
	OCS_HAL_LINK_SPEED,
	OCS_HAL_IF_TYPE,
	OCS_HAL_SLI_REV,
	OCS_HAL_SLI_FAMILY,
	OCS_HAL_RQ_PROCESS_LIMIT,
	OCS_HAL_RQ_DEFAULT_BUFFER_SIZE,
	OCS_HAL_AUTO_XFER_RDY_CAPABLE,
	OCS_HAL_AUTO_XFER_RDY_XRI_CNT,
	OCS_HAL_AUTO_XFER_RDY_SIZE,
	OCS_HAL_AUTO_XFER_RDY_BLK_SIZE,
	OCS_HAL_AUTO_XFER_RDY_T10_ENABLE,
	OCS_HAL_AUTO_XFER_RDY_P_TYPE,
	OCS_HAL_AUTO_XFER_RDY_REF_TAG_IS_LBA,
	OCS_HAL_AUTO_XFER_RDY_APP_TAG_VALID,
	OCS_HAL_AUTO_XFER_RDY_APP_TAG_VALUE,
	OCS_HAL_DIF_CAPABLE,
	OCS_HAL_DIF_SEED,
	OCS_HAL_DIF_MODE,
	OCS_HAL_DIF_MULTI_SEPARATE,
	OCS_HAL_DUMP_MAX_SIZE,
	OCS_HAL_DUMP_READY,
	OCS_HAL_DUMP_PRESENT,
	OCS_HAL_RESET_REQUIRED,
	OCS_HAL_FW_ERROR,
	OCS_HAL_FW_READY,
	OCS_HAL_HIGH_LOGIN_MODE,
	OCS_HAL_PREREGISTER_SGL,
	OCS_HAL_HW_REV1,
	OCS_HAL_HW_REV2,
	OCS_HAL_HW_REV3,
	OCS_HAL_LINKCFG,
	OCS_HAL_ETH_LICENSE,
	OCS_HAL_LINK_MODULE_TYPE,
	OCS_HAL_NUM_CHUTES,
	OCS_HAL_WAR_VERSION,
	OCS_HAL_DISABLE_AR_TGT_DIF,
	OCS_HAL_EMULATE_I_ONLY_AAB, /**< emulate IAAB=0 for initiator-commands only */
	OCS_HAL_EMULATE_TARGET_WQE_TIMEOUT, /**< enable driver timeouts for target WQEs */
	OCS_HAL_LINK_CONFIG_SPEED,
	OCS_HAL_CONFIG_TOPOLOGY,
	OCS_HAL_BOUNCE,
	OCS_HAL_PORTNUM,
	OCS_HAL_BIOS_VERSION_STRING,
	OCS_HAL_SGL_CHAINING_CAPABLE,
	OCS_HAL_SGL_CHAINING_ALLOWED,
	OCS_HAL_SGL_CHAINING_HOST_ALLOCATED,
	OCS_HAL_SEND_FRAME_CAPABLE,
	OCS_HAL_FILTER_DEF,
	OCS_ESOC,
} ocs_hal_property_e;

enum {
	OCS_HAL_TOPOLOGY_AUTO,
	OCS_HAL_TOPOLOGY_NPORT,
	OCS_HAL_TOPOLOGY_LOOP,
	OCS_HAL_TOPOLOGY_NONE,
	OCS_HAL_TOPOLOGY_MAX
};

enum {
	OCS_HAL_MODE_INITIATOR,
	OCS_HAL_MODE_TARGET,
	OCS_HAL_MODE_BOTH,
	OCS_HAL_MODE_MAX
};

/**
 * @brief Port protocols
 */

typedef enum {
	OCS_HAL_PORT_PROTOCOL_ISCSI,
	OCS_HAL_PORT_PROTOCOL_FCOE,
	OCS_HAL_PORT_PROTOCOL_FC,
	OCS_HAL_PORT_PROTOCOL_OTHER,
} ocs_hal_port_protocol_e;

#define OCS_HAL_MAX_PROFILES	40
/**
 * @brief A Profile Descriptor
 */
typedef struct {
	uint32_t	profile_index;
	uint32_t	profile_id;
	char		profile_description[512];
} ocs_hal_profile_descriptor_t;

/**
 * @brief A Profile List
 */
typedef struct {
	uint32_t			num_descriptors;
	ocs_hal_profile_descriptor_t	descriptors[OCS_HAL_MAX_PROFILES];
} ocs_hal_profile_list_t;


/**
 * @brief Defines DIF operation modes
 */
enum {
	OCS_HAL_DIF_MODE_INLINE,
	OCS_HAL_DIF_MODE_SEPARATE,
};

/**
 * @brief Defines the type of RQ buffer
 */
typedef enum {
	OCS_HAL_RQ_BUFFER_TYPE_HDR,
	OCS_HAL_RQ_BUFFER_TYPE_PAYLOAD,
	OCS_HAL_RQ_BUFFER_TYPE_MAX,
} ocs_hal_rq_buffer_type_e;

/**
 * @brief Defines a wrapper for the RQ payload buffers so that we can place it
 *        back on the proper queue.
 */
typedef struct {
	uint16_t rqindex;
	ocs_dma_t dma;
} ocs_hal_rq_buffer_t;

/**
 * @brief T10 DIF operations.
 */
typedef enum {
	OCS_HAL_DIF_OPER_DISABLED,
	OCS_HAL_SGE_DIF_OP_IN_NODIF_OUT_CRC,
	OCS_HAL_SGE_DIF_OP_IN_CRC_OUT_NODIF,
	OCS_HAL_SGE_DIF_OP_IN_NODIF_OUT_CHKSUM,
	OCS_HAL_SGE_DIF_OP_IN_CHKSUM_OUT_NODIF,
	OCS_HAL_SGE_DIF_OP_IN_CRC_OUT_CRC,
	OCS_HAL_SGE_DIF_OP_IN_CHKSUM_OUT_CHKSUM,
	OCS_HAL_SGE_DIF_OP_IN_CRC_OUT_CHKSUM,
	OCS_HAL_SGE_DIF_OP_IN_CHKSUM_OUT_CRC,
	OCS_HAL_SGE_DIF_OP_IN_RAW_OUT_RAW,
} ocs_hal_dif_oper_e;

#define OCS_HAL_DIF_OPER_PASS_THRU	OCS_HAL_SGE_DIF_OP_IN_CRC_OUT_CRC
#define OCS_HAL_DIF_OPER_STRIP		OCS_HAL_SGE_DIF_OP_IN_CRC_OUT_NODIF
#define OCS_HAL_DIF_OPER_INSERT		OCS_HAL_SGE_DIF_OP_IN_NODIF_OUT_CRC

/**
 * @brief T10 DIF block sizes.
 */
typedef enum {
	OCS_HAL_DIF_BK_SIZE_512,
	OCS_HAL_DIF_BK_SIZE_1024,
	OCS_HAL_DIF_BK_SIZE_2048,
	OCS_HAL_DIF_BK_SIZE_4096,
	OCS_HAL_DIF_BK_SIZE_520,
	OCS_HAL_DIF_BK_SIZE_4104,
	OCS_HAL_DIF_BK_SIZE_NA = 0
} ocs_hal_dif_blk_size_e;

/**
 * @brief Link configurations.
 */
typedef enum {
	OCS_HAL_LINKCFG_4X10G = 0,
	OCS_HAL_LINKCFG_1X40G,
	OCS_HAL_LINKCFG_2X16G,
	OCS_HAL_LINKCFG_4X8G,
	OCS_HAL_LINKCFG_4X1G,
	OCS_HAL_LINKCFG_2X10G,
	OCS_HAL_LINKCFG_2X10G_2X8G,

	/* must be last */
	OCS_HAL_LINKCFG_NA,
} ocs_hal_linkcfg_e;

/**
 * @brief link module types
 *
 * (note: these just happen to match SLI4 values)
 */

enum {
	OCS_HAL_LINK_MODULE_TYPE_1GB = 0x0004,
	OCS_HAL_LINK_MODULE_TYPE_2GB = 0x0008,
	OCS_HAL_LINK_MODULE_TYPE_4GB = 0x0040,
	OCS_HAL_LINK_MODULE_TYPE_8GB = 0x0080,
	OCS_HAL_LINK_MODULE_TYPE_10GB = 0x0100,
	OCS_HAL_LINK_MODULE_TYPE_16GB = 0x0200,
	OCS_HAL_LINK_MODULE_TYPE_32GB = 0x0400,
};

/**
 * @brief T10 DIF information passed to the transport.
 */

typedef struct ocs_hal_dif_info_s {
	ocs_hal_dif_oper_e dif_oper;
	ocs_hal_dif_blk_size_e blk_size;
	uint32_t ref_tag_cmp;
	uint32_t ref_tag_repl;
	uint32_t app_tag_cmp:16,
		app_tag_repl:16;
	uint32_t check_ref_tag:1,
		check_app_tag:1,
		check_guard:1,
		auto_incr_ref_tag:1,
		repl_app_tag:1,
		repl_ref_tag:1,
		dif:2,
		dif_separate:1,

		/* If the APP TAG is 0xFFFF, disable checking the REF TAG and CRC fields */
		disable_app_ffff:1,

		/* if the APP TAG is 0xFFFF and REF TAG is 0xFFFF_FFFF, disable checking the received CRC field. */
		disable_app_ref_ffff:1,

		:21;
	uint16_t dif_seed;
} ocs_hal_dif_info_t;

typedef enum {
	OCS_HAL_ELS_REQ,	/**< ELS request */
	OCS_HAL_ELS_RSP,	/**< ELS response */
	OCS_HAL_ELS_RSP_SID,	/**< ELS response, override the S_ID */
	OCS_HAL_FC_CT,		/**< FC Common Transport */
	OCS_HAL_FC_CT_RSP,	/**< FC Common Transport Response */
	OCS_HAL_BLS_ACC,	/**< BLS accept (BA_ACC) */
	OCS_HAL_BLS_ACC_SID,	/**< BLS accept (BA_ACC), override the S_ID */
	OCS_HAL_BLS_RJT,	/**< BLS reject (BA_RJT) */
	OCS_HAL_BCAST,		/**< Class 3 broadcast sequence */
	OCS_HAL_IO_TARGET_READ,
	OCS_HAL_IO_TARGET_WRITE,
	OCS_HAL_IO_TARGET_RSP,
	OCS_HAL_IO_INITIATOR_READ,
	OCS_HAL_IO_INITIATOR_WRITE,
	OCS_HAL_IO_INITIATOR_NODATA,
	OCS_HAL_IO_DNRX_REQUEUE,
	OCS_HAL_IO_MAX,
} ocs_hal_io_type_e;

typedef enum {
	OCS_HAL_IO_STATE_FREE,
	OCS_HAL_IO_STATE_INUSE,
	OCS_HAL_IO_STATE_WAIT_FREE,
	OCS_HAL_IO_STATE_WAIT_SEC_HIO,
} ocs_hal_io_state_e;

/* Descriptive strings for the HAL IO request types (note: these must always
 * match up with the ocs_hal_io_type_e declaration) */
#define OCS_HAL_IO_TYPE_STRINGS \
	"ELS request", \
	"ELS response", \
	"ELS response(set SID)", \
	"FC CT request", \
	"BLS accept", \
	"BLS accept(set SID)", \
	"BLS reject", \
	"target read", \
	"target write", \
	"target response", \
	"initiator read", \
	"initiator write", \
	"initiator nodata",

/**
 * @brief HAL command context.
 *
 * Stores the state for the asynchronous commands sent to the hardware.
 */
typedef struct ocs_command_ctx_s {
	ocs_list_t	link;
	/**< Callback function */
	int32_t		(*cb)(struct ocs_hal_s *, int32_t, uint8_t *, void *);
	void		*arg;	/**< Argument for callback */
	uint8_t		*buf;	/**< buffer holding command / results */
	void		*ctx;	/**< upper layer context */
} ocs_command_ctx_t;

typedef struct ocs_hal_sgl_s {
	uintptr_t	addr;
	size_t		len;
} ocs_hal_sgl_t;

/**
 * @brief HAL callback type
 *
 * Typedef for HAL "done" callback.
 */
typedef int32_t	(*ocs_hal_done_t)(struct ocs_hal_io_s *, ocs_remote_node_t *, uint32_t len, int32_t status, uint32_t ext, void *ul_arg);


typedef union ocs_hal_io_param_u {
	struct {
		uint16_t ox_id;
		uint16_t rx_id;
		uint8_t  payload[12];	/**< big enough for ABTS BA_ACC */
	} bls;
	struct {
		uint32_t s_id;
		uint16_t ox_id;
		uint16_t rx_id;
		uint8_t  payload[12];	/**< big enough for ABTS BA_ACC */
	} bls_sid;
	struct {
		uint8_t	r_ctl;
		uint8_t	type;
		uint8_t	df_ctl;
		uint8_t timeout;
	} bcast;
	struct {
		uint16_t ox_id;
		uint8_t timeout;
	} els;
	struct {
		uint32_t s_id;
		uint16_t ox_id;
		uint8_t timeout;
	} els_sid;
	struct {
		uint8_t	r_ctl;
		uint8_t	type;
		uint8_t	df_ctl;
		uint8_t timeout;
	} fc_ct;
	struct {
		uint8_t	r_ctl;
		uint8_t	type;
		uint8_t	df_ctl;
		uint8_t timeout;
		uint16_t ox_id;
	} fc_ct_rsp;
	struct {
		uint32_t offset;
		uint16_t ox_id;
		uint16_t flags;
		uint8_t	cs_ctl;
		ocs_hal_dif_oper_e dif_oper;
		ocs_hal_dif_blk_size_e blk_size;
		uint8_t		timeout;
	} fcp_tgt;
	struct {
		ocs_dma_t	*cmnd;
		ocs_dma_t	*rsp;
		ocs_hal_dif_oper_e dif_oper;
		ocs_hal_dif_blk_size_e blk_size;
		uint32_t	cmnd_size;
		uint16_t	flags;
		uint8_t		timeout;
		uint32_t	first_burst;
	} fcp_ini;
} ocs_hal_io_param_t;

/**
 * @brief WQ steering mode
 */
typedef enum {
	OCS_HAL_WQ_STEERING_CLASS,
	OCS_HAL_WQ_STEERING_REQUEST,
	OCS_HAL_WQ_STEERING_CPU,
} ocs_hal_wq_steering_e;

/**
 * @brief HAL wqe object
 */
typedef struct {
	uint32_t	abort_wqe_submit_needed:1,	/**< set if abort wqe needs to be submitted */
			send_abts:1,			/**< set to 1 to have hardware to automatically send ABTS */
			auto_xfer_rdy_dnrx:1,		/**< TRUE if DNRX was set on this IO */
			:29;
	uint32_t	id;
	uint32_t	abort_reqtag;
	ocs_list_link_t link;
	uint8_t         *wqebuf;                        /**< work queue entry buffer */
} ocs_hal_wqe_t;

/**
 * @brief HAL IO object.
 *
 * Stores the per-IO information necessary for both the lower (SLI) and upper
 * layers (ocs).
 */
struct ocs_hal_io_s {
	// Owned by HAL
	ocs_list_link_t	link;		/**< used for busy, wait_free, free lists */
	ocs_list_link_t	wqe_link;	/**< used for timed_wqe list */
	ocs_list_link_t	dnrx_link;	/**< used for io posted dnrx list */
	ocs_hal_io_state_e state;	/**< state of IO: free, busy, wait_free */
	ocs_hal_wqe_t	wqe;		/**< Work queue object, with link for pending */
	ocs_lock_t	axr_lock;	/**< Lock to synchronize TRSP and AXT Data/Cmd Cqes */
	ocs_hal_t	*hal;		/**< pointer back to hardware context */
	ocs_remote_node_t	*rnode;
	struct ocs_hal_auto_xfer_rdy_buffer_s *axr_buf;
	ocs_dma_t	xfer_rdy;
	uint16_t	type;
	uint32_t	port_owned_abort_count; /**< IO abort count */
	hal_wq_t	*wq;		/**< WQ assigned to the exchange */
	uint32_t	xbusy;		/**< Exchange is active in FW */
	ocs_hal_done_t  done;		/**< Function called on IO completion */
	void		*arg;		/**< argument passed to "IO done" callback */
	ocs_hal_done_t  abort_done;	/**< Function called on abort completion */
	void		*abort_arg;	/**< argument passed to "abort done" callback */
	ocs_ref_t	ref;		/**< refcount object */
	size_t		length;		/**< needed for bug O127585: length of IO */
	uint8_t		tgt_wqe_timeout; /**< timeout value for target WQEs */
	uint64_t	submit_ticks;	/**< timestamp when current WQE was submitted */

	uint32_t	status_saved:1, /**< if TRUE, latched status should be returned */
			abort_in_progress:1, /**< if TRUE, abort is in progress */
			quarantine:1,	/**< set if IO to be quarantined */
			quarantine_first_phase:1,	/**< set if first phase of IO */
			is_port_owned:1,	/**< set if POST_XRI was used to send XRI to th chip */
			auto_xfer_rdy_dnrx:1,	/**< TRUE if DNRX was set on this IO */
			:26;
	uint32_t	saved_status;	/**< latched status */
	uint32_t	saved_len;	/**< latched length */
	uint32_t	saved_ext;	/**< latched extended status */

	hal_eq_t	*eq;		/**< EQ that this HIO came up on */
	ocs_hal_wq_steering_e	wq_steering;	/**< WQ steering mode request */
	uint8_t		wq_class;	/**< WQ class if steering mode is Class */

	// Owned by SLI layer
	uint16_t	reqtag;		/**< request tag for this HAL IO */
	uint32_t	abort_reqtag;	/**< request tag for an abort of this HAL IO (note: this is a 32 bit value
					     to allow us to use UINT32_MAX as an uninitialized value) */
	uint32_t	indicator;	/**< XRI */
	ocs_dma_t	def_sgl;	/**< default scatter gather list */
	uint32_t	def_sgl_count;	/**< count of SGEs in default SGL */
	ocs_dma_t	*sgl;		/**< pointer to current active SGL */
	uint32_t	sgl_count;	/**< count of SGEs in io->sgl */
	uint32_t	first_data_sge;	/**< index of first data SGE */
	ocs_dma_t	*ovfl_sgl;	/**< overflow SGL */
	uint32_t	ovfl_sgl_count;	/**< count of SGEs in default SGL */
	sli4_lsp_sge_t	*ovfl_lsp;	/**< pointer to overflow segment length */
	ocs_hal_io_t	*ovfl_io;	/**< Used for SGL chaining on skyhawk */
	uint32_t	n_sge;		/**< number of active SGEs */
	uint32_t	sge_offset;

	/* BZ 161832 Workaround: */
	struct ocs_hal_io_s	*sec_hio; /**< Secondary HAL IO context */
	ocs_hal_io_param_t sec_iparam;	/**< Secondary HAL IO context saved iparam */
	uint32_t	sec_len;	/**< Secondary HAL IO context saved len */

	/* Owned by upper layer */
	void		*ul_io;		/**< where upper layer can store reference to its IO */
};

typedef enum {
	OCS_HAL_PORT_INIT,
	OCS_HAL_PORT_SHUTDOWN,
	OCS_HAL_PORT_SET_LINK_CONFIG,
} ocs_hal_port_e;

/**
 * @brief Fabric/Domain events
 */
typedef enum {
	OCS_HAL_DOMAIN_ALLOC_OK,	/**< domain successfully allocated */
	OCS_HAL_DOMAIN_ALLOC_FAIL,	/**< domain allocation failed */
	OCS_HAL_DOMAIN_ATTACH_OK,	/**< successfully attached to domain */
	OCS_HAL_DOMAIN_ATTACH_FAIL,	/**< domain attach failed */
	OCS_HAL_DOMAIN_FREE_OK,		/**< successfully freed domain */
	OCS_HAL_DOMAIN_FREE_FAIL,	/**< domain free failed */
	OCS_HAL_DOMAIN_LOST,		/**< previously discovered domain no longer available */
	OCS_HAL_DOMAIN_FOUND,		/**< new domain discovered */
	OCS_HAL_DOMAIN_CHANGED,		/**< previously discovered domain properties have changed */
} ocs_hal_domain_event_e;

typedef enum {
	OCS_HAL_PORT_ALLOC_OK,		/**< port successfully allocated */
	OCS_HAL_PORT_ALLOC_FAIL,	/**< port allocation failed */
	OCS_HAL_PORT_ATTACH_OK,		/**< successfully attached to port */
	OCS_HAL_PORT_ATTACH_FAIL,	/**< port attach failed */
	OCS_HAL_PORT_FREE_OK,		/**< successfully freed port */
	OCS_HAL_PORT_FREE_FAIL,		/**< port free failed */
} ocs_hal_port_event_e;

typedef enum {
	OCS_HAL_NODE_ATTACH_OK,
	OCS_HAL_NODE_ATTACH_FAIL,
	OCS_HAL_NODE_FREE_OK,
	OCS_HAL_NODE_FREE_FAIL,
	OCS_HAL_NODE_FREE_ALL_OK,
	OCS_HAL_NODE_FREE_ALL_FAIL,
} ocs_hal_remote_node_event_e;

typedef enum {
	OCS_HAL_CB_DOMAIN,
	OCS_HAL_CB_PORT,
	OCS_HAL_CB_REMOTE_NODE,
	OCS_HAL_CB_UNSOLICITED,
	OCS_HAL_CB_BOUNCE,
	OCS_HAL_CB_MAX,			/**< must be last */
} ocs_hal_callback_e;

/**
 * @brief HAL unsolicited callback status
 */
typedef enum {
	OCS_HAL_UNSOL_SUCCESS,
	OCS_HAL_UNSOL_ERROR,
	OCS_HAL_UNSOL_ABTS_RCVD,
	OCS_HAL_UNSOL_MAX,		/**< must be last */
} ocs_hal_unsol_status_e;

/**
 * @brief Node group rpi reference
 */
typedef struct {
	ocs_atomic_t rpi_count;
	ocs_atomic_t rpi_attached;
} ocs_hal_rpi_ref_t;

/**
 * @brief HAL link stat types
 */
typedef enum {
	OCS_HAL_LINK_STAT_LINK_FAILURE_COUNT,
	OCS_HAL_LINK_STAT_LOSS_OF_SYNC_COUNT,
	OCS_HAL_LINK_STAT_LOSS_OF_SIGNAL_COUNT,
	OCS_HAL_LINK_STAT_PRIMITIVE_SEQ_COUNT,
	OCS_HAL_LINK_STAT_INVALID_XMIT_WORD_COUNT,
	OCS_HAL_LINK_STAT_CRC_COUNT,
	OCS_HAL_LINK_STAT_PRIMITIVE_SEQ_TIMEOUT_COUNT,
	OCS_HAL_LINK_STAT_ELASTIC_BUFFER_OVERRUN_COUNT,
	OCS_HAL_LINK_STAT_ARB_TIMEOUT_COUNT,
	OCS_HAL_LINK_STAT_ADVERTISED_RCV_B2B_CREDIT,
	OCS_HAL_LINK_STAT_CURR_RCV_B2B_CREDIT,
	OCS_HAL_LINK_STAT_ADVERTISED_XMIT_B2B_CREDIT,
	OCS_HAL_LINK_STAT_CURR_XMIT_B2B_CREDIT,
	OCS_HAL_LINK_STAT_RCV_EOFA_COUNT,
	OCS_HAL_LINK_STAT_RCV_EOFDTI_COUNT,
	OCS_HAL_LINK_STAT_RCV_EOFNI_COUNT,
	OCS_HAL_LINK_STAT_RCV_SOFF_COUNT,
	OCS_HAL_LINK_STAT_RCV_DROPPED_NO_AER_COUNT,
	OCS_HAL_LINK_STAT_RCV_DROPPED_NO_RPI_COUNT,
	OCS_HAL_LINK_STAT_RCV_DROPPED_NO_XRI_COUNT,
	OCS_HAL_LINK_STAT_MAX,		/**< must be last */
} ocs_hal_link_stat_e;

typedef enum {
	OCS_HAL_HOST_STAT_TX_KBYTE_COUNT,
	OCS_HAL_HOST_STAT_RX_KBYTE_COUNT,
	OCS_HAL_HOST_STAT_TX_FRAME_COUNT,
	OCS_HAL_HOST_STAT_RX_FRAME_COUNT,
	OCS_HAL_HOST_STAT_TX_SEQ_COUNT,
	OCS_HAL_HOST_STAT_RX_SEQ_COUNT,
	OCS_HAL_HOST_STAT_TOTAL_EXCH_ORIG,
	OCS_HAL_HOST_STAT_TOTAL_EXCH_RESP,
	OCS_HAL_HOSY_STAT_RX_P_BSY_COUNT,
	OCS_HAL_HOST_STAT_RX_F_BSY_COUNT,
	OCS_HAL_HOST_STAT_DROP_FRM_DUE_TO_NO_RQ_BUF_COUNT,
	OCS_HAL_HOST_STAT_EMPTY_RQ_TIMEOUT_COUNT,
	OCS_HAL_HOST_STAT_DROP_FRM_DUE_TO_NO_XRI_COUNT,
	OCS_HAL_HOST_STAT_EMPTY_XRI_POOL_COUNT,
	OCS_HAL_HOST_STAT_MAX /* MUST BE LAST */
} ocs_hal_host_stat_e;

typedef enum {
	OCS_HAL_STATE_UNINITIALIZED,		/* power-on, no allocations, no initializations */
	OCS_HAL_STATE_QUEUES_ALLOCATED,		/* chip is reset, allocations are complete (queues not registered) */
	OCS_HAL_STATE_ACTIVE,			/* chip is up an running */
	OCS_HAL_STATE_RESET_IN_PROGRESS,	/* chip is being reset */
	OCS_HAL_STATE_TEARDOWN_IN_PROGRESS,	/* teardown has been started */
} ocs_hal_state_e;

/**
 * @brief Defines a general FC sequence object, consisting of a header, payload buffers
 *	  and a HAL IO in the case of port owned XRI
 */
typedef struct {
	ocs_hal_t *hal;			/**< HAL that owns this sequence */
	/* sequence information */
	uint8_t fcfi;		/**< FCFI associated with sequence */
	uint8_t auto_xrdy;	/**< If auto XFER_RDY was generated */
	uint8_t out_of_xris;	/**< If IO would have been assisted if XRIs were available */
	ocs_hal_rq_buffer_t *header;
	ocs_hal_rq_buffer_t *payload;	/**< received frame payload buffer */

	/* other "state" information from the SRB (sequence coalescing) */
	ocs_hal_unsol_status_e status;
	uint32_t xri;		/**< XRI associated with sequence; sequence coalescing only */
	ocs_hal_io_t *hio;	/**< HAL IO */

	ocs_list_link_t link;
	void *hal_priv;		/**< HAL private context */
} ocs_hal_sequence_t;

/**
 * @brief Structure to track optimized write buffers posted to chip owned XRIs.
 *
 * Note: The rqindex will be set the following "fake" indexes. This will be used
 *       when the buffer is returned via ocs_seq_free() to make the buffer available
 *       for re-use on another XRI.
 *
 *       The dma->alloc pointer on the dummy header will be used to get back to this structure when the buffer is freed.
 *
 *       More of these object may be allocated on the fly if more XRIs are pushed to the chip.
 */
#define OCS_HAL_RQ_INDEX_DUMMY_HDR	0xFF00
#define OCS_HAL_RQ_INDEX_DUMMY_DATA	0xFF01
typedef struct ocs_hal_auto_xfer_rdy_buffer_s {
	fc_header_t hdr;		/**< used to build a dummy data header for unsolicited processing */
	ocs_hal_rq_buffer_t header;	/**< Points to the dummy data header */
	ocs_hal_rq_buffer_t payload;	/**< received frame payload buffer */
	ocs_hal_sequence_t seq;         /**< sequence for passing the buffers */
	uint8_t data_cqe;
	uint8_t cmd_cqe;

	/* fields saved from the command header that are needed when the data arrives */
	uint8_t fcfi;

	/* To handle outof order completions save AXR cmd and data cqes */
	uint8_t call_axr_cmd;
	uint8_t call_axr_data;
	ocs_hal_sequence_t *cmd_seq;
} ocs_hal_auto_xfer_rdy_buffer_t;

/**
 * @brief Node group rpi reference
 */
typedef struct {
	uint8_t overflow;
	uint32_t counter;
} ocs_hal_link_stat_counts_t;

/**
 * @brief HAL object describing fc host stats
 */
typedef struct {
	uint32_t counter;
} ocs_hal_host_stat_counts_t;

#define TID_HASH_BITS	8
#define TID_HASH_LEN	(1U << TID_HASH_BITS)

typedef struct ocs_hal_iopt_s {
	char		name[32];
	uint32_t	instance_index;
	ocs_thread_t	iopt_thread;
	ocs_cbuf_t	*iopt_free_queue;	// multiple reader, multiple writer
	ocs_cbuf_t	*iopt_work_queue;
	ocs_array_t	*iopt_cmd_array;
} ocs_hal_iopt_t;

typedef enum {
	HAL_CQ_HANDLER_LOCAL,
	HAL_CQ_HANDLER_THREAD,
} hal_cq_handler_e;

#include "ocs_hal_queues.h"

/**
 * @brief Stucture used for the hash lookup of queue IDs
 */
typedef struct {
	uint32_t id:16,
		in_use:1,
		index:15;
} ocs_queue_hash_t;

/**
 * @brief Define the fields required to implement the skyhawk DIF quarantine.
 */
#define OCS_HAL_QUARANTINE_QUEUE_DEPTH	4

typedef struct {
	uint32_t	quarantine_index;
	ocs_hal_io_t	*quarantine_ios[OCS_HAL_QUARANTINE_QUEUE_DEPTH];
} ocs_quarantine_info_t;

/**
 * @brief Define the WQ callback object
 */
typedef struct {
	uint16_t instance_index;	/**< use for request tag */
	void (*callback)(void *arg, uint8_t *cqe, int32_t status);
	void *arg;
} hal_wq_callback_t;

/**
 * @brief HAL object
 */
struct ocs_hal_s {
	ocs_os_handle_t	os;
	sli4_t		sli;
	uint16_t	ulp_start;
	uint16_t	ulp_max;
	uint32_t	dump_size;
	ocs_hal_state_e state;
	uint8_t		hal_setup_called;
	uint8_t		sliport_healthcheck;
	uint16_t        watchdog_timeout;

	/** HAL configuration, subject to ocs_hal_set()  */
	struct {
		uint32_t	n_eq; /**< number of event queues */
		uint32_t	n_cq; /**< number of completion queues */
		uint32_t	n_mq; /**< number of mailbox queues */
		uint32_t	n_rq; /**< number of receive queues */
		uint32_t	n_wq; /**< number of work queues */
		uint32_t	n_io; /**< total number of IO objects */
		uint32_t	n_sgl;/**< length of SGL */
		uint32_t	speed;	/** requested link speed in Mbps */
		uint32_t	topology;  /** requested link topology */
		uint32_t	rq_default_buffer_size;	/** size of the buffers for first burst */
		uint32_t	auto_xfer_rdy_xri_cnt;	/** Initial XRIs to post to chip at initialization */
		uint32_t	auto_xfer_rdy_size;	/** max size IO to use with this feature */
		uint8_t		auto_xfer_rdy_blk_size_chip;	/** block size to use with this feature */
		uint8_t         esoc;
		uint16_t	dif_seed; /** The seed for the DIF CRC calculation */
		uint16_t	auto_xfer_rdy_app_tag_value;
		uint8_t		dif_mode; /**< DIF mode to use */
		uint8_t		i_only_aab; /** Enable initiator-only auto-abort */
		uint8_t		emulate_tgt_wqe_timeout; /** Enable driver target wqe timeouts */
		uint32_t	bounce:1;
		const char      *queue_topology;
		uint8_t		auto_xfer_rdy_t10_enable;	/** Enable t10 PI for auto xfer ready */
		uint8_t		auto_xfer_rdy_p_type;	/** p_type for auto xfer ready */
		uint8_t		auto_xfer_rdy_ref_tag_is_lba;
		uint8_t		auto_xfer_rdy_app_tag_valid;
		uint32_t	filter_def[SLI4_CMD_REG_FCFI_NUM_RQ_CFG];
	} config;

	/* calculated queue sizes for each type */
	uint32_t	num_qentries[SLI_QTYPE_MAX];

	/* Storage for SLI queue objects */
	sli4_queue_t	wq[OCS_HAL_MAX_NUM_WQ];
	sli4_queue_t	rq[OCS_HAL_MAX_NUM_RQ];
	uint16_t	hal_rq_lookup[OCS_HAL_MAX_NUM_RQ];
	sli4_queue_t	mq[OCS_HAL_MAX_NUM_MQ];
	sli4_queue_t	cq[OCS_HAL_MAX_NUM_CQ];
	sli4_queue_t	eq[OCS_HAL_MAX_NUM_EQ];

	/* HAL queue */
	uint32_t	eq_count;
	uint32_t	cq_count;
	uint32_t	mq_count;
	uint32_t	wq_count;
	uint32_t	rq_count;			/**< count of SLI RQs */
	ocs_list_t	eq_list;

	ocs_queue_hash_t cq_hash[OCS_HAL_Q_HASH_SIZE];
	ocs_queue_hash_t rq_hash[OCS_HAL_Q_HASH_SIZE];
	ocs_queue_hash_t wq_hash[OCS_HAL_Q_HASH_SIZE];

	/* Storage for HAL queue objects */
	hal_wq_t	*hal_wq[OCS_HAL_MAX_NUM_WQ];
	hal_rq_t	*hal_rq[OCS_HAL_MAX_NUM_RQ];
	hal_mq_t	*hal_mq[OCS_HAL_MAX_NUM_MQ];
	hal_cq_t	*hal_cq[OCS_HAL_MAX_NUM_CQ];
	hal_eq_t	*hal_eq[OCS_HAL_MAX_NUM_EQ];
	uint32_t	hal_rq_count;			/**< count of hal_rq[] entries */
	bool		hal_mrq_used;

	ocs_varray_t	*wq_class_array[OCS_HAL_MAX_WQ_CLASS];	/**< pool per class WQs */
	ocs_varray_t	*wq_cpu_array[OCS_HAL_MAX_WQ_CPU];	/**< pool per CPU WQs */

	/* Sequence objects used in incoming frame processing */
	ocs_array_t	*seq_pool;

	/* Auto XFER RDY Buffers - protect with io_lock */
	uint32_t	auto_xfer_rdy_enabled:1,	/**< TRUE if auto xfer rdy is enabled */
			:31;
	ocs_pool_t	*auto_xfer_rdy_buf_pool;	/**< pool of ocs_hal_auto_xfer_rdy_buffer_t objects */

	/** Maintain an ordered, linked list of outstanding HAL commands. */
	ocs_lock_t	cmd_lock;
	ocs_list_t	cmd_head;
	ocs_list_t	cmd_pending;
	uint32_t	cmd_head_count;


	sli4_link_event_t link;
	ocs_hal_linkcfg_e linkcfg; /**< link configuration setting */
	uint32_t eth_license;	   /**< Ethernet license; to enable FCoE on Lancer */

	struct {
		/**
		 * Function + argument used to notify upper layer of domain events.
		 *
		 * The final argument to the callback is a generic data pointer:
		 *  - ocs_domain_record_t on OCS_HAL_DOMAIN_FOUND
		 *  - ocs_domain_t on OCS_HAL_DOMAIN_ALLOC_FAIL, OCS_HAL_DOMAIN_ALLOC_OK,
		 * OCS_HAL_DOMAIN_FREE_FAIL, OCS_HAL_DOMAIN_FREE_OK,
		 * OCS_HAL_DOMAIN_ATTACH_FAIL, OCS_HAL_DOMAIN_ATTACH_OK, and
		 * OCS_HAL_DOMAIN_LOST.
		 */
		int32_t	(*domain)(void *, ocs_hal_domain_event_e, void *);
		/**
		 * Function + argument used to notify upper layers of port events.
		 *
		 * The final argument to the callback is a pointer to the effected
		 * SLI port for all events.
		 */
		int32_t (*port)(void *, ocs_hal_port_event_e, void *);
		/** Function + argument used to announce arrival of unsolicited frames */
		int32_t (*unsolicited)(void *, ocs_hal_sequence_t *);
		int32_t (*rnode)(void *, ocs_hal_remote_node_event_e, void *);
		int32_t (*bounce)(void (*)(void *arg), void *arg, uint32_t s_id, uint32_t d_id, uint32_t ox_id);
	} callback;
	struct {
		void *domain;
		void *port;
		void *unsolicited;
		void *rnode;
		void *bounce;
	} args;

	/* OCS domain objects index by FCFI */
	int32_t		first_domain_idx;		/* Workaround for srb->fcfi == 0 */
	ocs_domain_t	*domains[SLI4_MAX_FCFI];

	/* Table of FCFI values index by FCF_index */
	uint16_t	fcf_index_fcfi[SLI4_MAX_FCF_INDEX];

	uint16_t	fcf_indicator;

	ocs_hal_io_t	*io;		/**< array of IO objects */
	uint8_t         *wqe_buffs;     /**< array of WQE buffs mapped to IO objects */

	ocs_lock_t	io_lock;		/**< IO lock to synchronize list access */
	ocs_lock_t	io_abort_lock;		/**< IO lock to synchronize IO aborting */
	ocs_list_t	io_inuse;		/**< List of IO objects in use */
	ocs_list_t	io_timed_wqe;		/**< List of IO objects with a timed target WQE */
	ocs_list_t	io_wait_free;		/**< List of IO objects waiting to be freed */
	ocs_list_t	io_free;		/**< List of IO objects available for allocation */
	ocs_list_t	io_port_owned;		/**< List of IO objects posted for chip use */
	ocs_list_t	io_port_dnrx;		/**< List of IO objects needing auto xfer rdy buffers */

	ocs_dma_t	loop_map;

	ocs_dma_t	xfer_rdy;

	ocs_dma_t	dump_sges;

	ocs_dma_t	rnode_mem;

	ocs_hal_rpi_ref_t *rpi_ref;

	char		*hal_war_version;
	ocs_hal_workaround_t workaround;

	ocs_atomic_t io_alloc_failed_count;

#if defined(OCS_DEBUG_QUEUE_HISTORY)
	ocs_hal_q_hist_t q_hist;
#endif

	ocs_list_t	sec_hio_wait_list;	/**< BZ 161832 Workaround: Secondary HAL IO context wait list */
	uint32_t	sec_hio_wait_count;	/**< BZ 161832 Workaround: Count of IOs that were put on the
						 * Secondary HAL IO wait list
						 */

#define HAL_MAX_TCMD_THREADS		16
	ocs_hal_qtop_t	*qtop;					/**< pointer to queue topology */

	uint32_t	tcmd_wq_submit[OCS_HAL_MAX_NUM_WQ];	/**< stat: wq sumbit count */
	uint32_t	tcmd_wq_complete[OCS_HAL_MAX_NUM_WQ];	/**< stat: wq complete count */

	ocs_timer_t	wqe_timer;		/**< Timer to periodically check for WQE timeouts */
	ocs_timer_t	watchdog_timer;		/**< Timer for heartbeat */
	uint32_t	in_active_wqe_timer:1,	/**< TRUE if currently in active wqe timer handler */
			active_wqe_timer_shutdown:1, /** TRUE if wqe timer is to be shutdown */
			:30;

	ocs_list_t	iopc_list;		/**< list of IO processing contexts */
	ocs_lock_t	iopc_list_lock;		/**< lock for iopc_list */

	ocs_pool_t	*wq_reqtag_pool;	/**< pool of hal_wq_callback_t objects */

	ocs_atomic_t	send_frame_seq_id;	/**< send frame sequence ID */
};


typedef enum {
	OCS_HAL_IO_INUSE_COUNT,
	OCS_HAL_IO_FREE_COUNT,
	OCS_HAL_IO_WAIT_FREE_COUNT,
	OCS_HAL_IO_PORT_OWNED_COUNT,
	OCS_HAL_IO_N_TOTAL_IO_COUNT,
} ocs_hal_io_count_type_e;

typedef void (*tcmd_cq_handler)(ocs_hal_t *hal, uint32_t cq_idx, void *cq_handler_arg);

/*
 * HAL queue data structures
 */

struct hal_eq_s {
	ocs_list_link_t link;		/**< must be first */
	sli4_qtype_e type;		/**< must be second */
	uint32_t instance;
	uint32_t entry_count;
	uint32_t entry_size;
	ocs_hal_t *hal;
	sli4_queue_t *queue;
	ocs_list_t cq_list;
#if OCS_STAT_ENABLE
	uint32_t use_count;
#endif
	ocs_varray_t *wq_array;		/*<< array of WQs */
};

struct hal_cq_s {
	ocs_list_link_t link;		/*<< must be first */
	sli4_qtype_e type;		/**< must be second */
	uint32_t instance;		/*<< CQ instance (cq_idx) */
	uint32_t entry_count;		/*<< Number of entries */
	uint32_t entry_size;		/*<< entry size */
	hal_eq_t *eq;			/*<< parent EQ */
	sli4_queue_t *queue;		/**< pointer to SLI4 queue */
	ocs_list_t q_list;		/**< list of children queues */

#if OCS_STAT_ENABLE
	uint32_t use_count;
#endif
};

typedef struct {
	ocs_list_link_t link;		/*<< must be first */
	sli4_qtype_e type;		/*<< must be second */
} hal_q_t;

struct hal_mq_s {
	ocs_list_link_t link;		/*<< must be first */
	sli4_qtype_e type;		/*<< must be second */
	uint32_t instance;

	uint32_t entry_count;
	uint32_t entry_size;
	hal_cq_t *cq;
	sli4_queue_t *queue;

#if OCS_STAT_ENABLE
	uint32_t use_count;
#endif
};

struct hal_wq_s {
	ocs_list_link_t link;		/*<< must be first */
	sli4_qtype_e type;		/*<< must be second */
	uint32_t instance;
	ocs_hal_t *hal;

	uint32_t entry_count;
	uint32_t entry_size;
	hal_cq_t *cq;
	sli4_queue_t *queue;
	uint32_t class;
	uint8_t ulp;

	/* WQ consumed */
	uint32_t wqec_set_count;		/*<< how often IOs are submitted with wqce set */
	uint32_t wqec_count;			/*<< current wqce counter */
	uint32_t free_count;			/*<< free count */
	uint32_t total_submit_count;		/*<< total submit count */
	ocs_list_t pending_list;		/*<< list of IOs pending for this WQ */

	/*
	 * ---Skyhawk only ---
	 * BZ 160124 - Driver must quarantine XRIs for target writes and
	 * initiator read when using DIF separates. Throw them on a
	 * queue until another 4 similar requests are completed to ensure they
	 * are flushed from the internal chip cache before being re-used.
	 * The must be a separate queue per CQ because the actual chip completion
	 * order cannot be determined. Since each WQ has a separate CQ, use the wq
	 * associated with the IO.
	 *
	 * Note: Protected by queue->lock
	 */
	ocs_quarantine_info_t quarantine_info;

	/*
	 * HAL IO allocated for use with Send Frame
	 */
	ocs_hal_io_t *send_frame_io;

	/* Stats */
#if OCS_STAT_ENABLE
	uint32_t use_count;			/*<< use count */
	uint32_t wq_pending_count;		/*<< count of HAL IOs that were queued on the WQ pending list */
#endif
};

struct hal_rq_s {
	ocs_list_link_t link;			/*<< must be first */
	sli4_qtype_e type;			/*<< must be second */
	uint32_t instance;

	uint32_t entry_count;
	uint32_t hdr_entry_size;
	uint32_t first_burst_entry_size;
	uint32_t data_entry_size;
	uint8_t ulp;
	uint8_t policy;
	bool is_mrq;
	uint32_t base_mrq_id;
	uint8_t mrq_set_count;
	uint8_t mrq_set_num;

	hal_cq_t *cq;

	uint8_t filter_mask;			/* Filter mask value */
	bool protocol_valid;
	uint8_t protocol;
	sli4_queue_t *hdr;
	sli4_queue_t *first_burst;
	sli4_queue_t *data;

	ocs_hal_rq_buffer_t *hdr_buf;
	ocs_hal_rq_buffer_t *fb_buf;
	ocs_hal_rq_buffer_t *payload_buf;

	ocs_hal_sequence_t **rq_tracker;	/* RQ tracker for this RQ */
#if OCS_STAT_ENABLE
	uint32_t use_count;
	uint32_t hdr_use_count;
	uint32_t fb_use_count;
	uint32_t payload_use_count;
#endif
};

typedef struct ocs_hal_global_s {
	const char	*queue_topology_string;			/**< queue topology string */
} ocs_hal_global_t;
extern ocs_hal_global_t hal_global;

extern hal_eq_t *hal_new_eq(ocs_hal_t *hal, uint32_t entry_count);
extern hal_cq_t *hal_new_cq(hal_eq_t *eq, uint32_t entry_count);
extern uint32_t hal_new_cq_set(hal_eq_t *eqs[], hal_cq_t *cqs[], uint32_t num_cqs, uint32_t entry_count);
extern hal_mq_t *hal_new_mq(hal_cq_t *cq, uint32_t entry_count);
extern hal_wq_t *hal_new_wq(hal_cq_t *cq, uint32_t entry_count, uint32_t class, uint32_t ulp);
extern hal_rq_t *hal_new_rq(hal_cq_t *cq, uint32_t entry_count, uint32_t ulp);
extern uint32_t hal_new_rq_set(hal_cq_t *cqs[], hal_rq_t *rqs[], uint32_t num_rq_pairs, uint32_t entry_count, uint32_t ulp);
extern void hal_del_eq(hal_eq_t *eq);
extern void hal_del_cq(hal_cq_t *cq);
extern void hal_del_mq(hal_mq_t *mq);
extern void hal_del_wq(hal_wq_t *wq);
extern void hal_del_rq(hal_rq_t *rq);
extern void hal_queue_dump(ocs_hal_t *hal);
extern void hal_queue_teardown(ocs_hal_t *hal);
extern int32_t hal_route_rqe(ocs_hal_t *hal, ocs_hal_sequence_t *seq);
extern int32_t ocs_hal_queue_hash_find(ocs_queue_hash_t *, uint16_t);
extern ocs_hal_rtn_e ocs_hal_setup(ocs_hal_t *, ocs_os_handle_t, sli4_port_type_e);
extern ocs_hal_rtn_e ocs_hal_init(ocs_hal_t *);
extern ocs_hal_rtn_e ocs_hal_teardown(ocs_hal_t *);
extern ocs_hal_rtn_e ocs_hal_reset(ocs_hal_t *, ocs_hal_reset_e);
extern int32_t ocs_hal_get_num_eq(ocs_hal_t *);
extern ocs_hal_rtn_e ocs_hal_get(ocs_hal_t *, ocs_hal_property_e, uint32_t *);
extern void *ocs_hal_get_ptr(ocs_hal_t *, ocs_hal_property_e);
extern ocs_hal_rtn_e ocs_hal_set(ocs_hal_t *, ocs_hal_property_e, uint32_t);
extern ocs_hal_rtn_e ocs_hal_set_ptr(ocs_hal_t *, ocs_hal_property_e, void*);
extern int32_t ocs_hal_event_check(ocs_hal_t *, uint32_t);
extern int32_t ocs_hal_process(ocs_hal_t *, uint32_t, uint32_t);
extern ocs_hal_rtn_e ocs_hal_command(ocs_hal_t *, uint8_t *, uint32_t, void *, void *);
extern ocs_hal_rtn_e ocs_hal_callback(ocs_hal_t *, ocs_hal_callback_e, void *, void *);
extern ocs_hal_rtn_e ocs_hal_port_alloc(ocs_hal_t *, ocs_sli_port_t *, ocs_domain_t *, uint8_t *);
extern ocs_hal_rtn_e ocs_hal_port_attach(ocs_hal_t *, ocs_sli_port_t *, uint32_t);
typedef void (*ocs_hal_port_control_cb_t)(int32_t status, uintptr_t value, void *arg);
extern ocs_hal_rtn_e ocs_hal_port_control(ocs_hal_t *, ocs_hal_port_e, uintptr_t, ocs_hal_port_control_cb_t, void *);
extern ocs_hal_rtn_e ocs_hal_port_free(ocs_hal_t *, ocs_sli_port_t *);
extern ocs_hal_rtn_e ocs_hal_domain_alloc(ocs_hal_t *, ocs_domain_t *, uint32_t, uint32_t);
extern ocs_hal_rtn_e ocs_hal_domain_attach(ocs_hal_t *, ocs_domain_t *, uint32_t);
extern ocs_hal_rtn_e ocs_hal_domain_free(ocs_hal_t *, ocs_domain_t *);
extern ocs_hal_rtn_e ocs_hal_domain_force_free(ocs_hal_t *, ocs_domain_t *);
extern ocs_domain_t * ocs_hal_domain_get(ocs_hal_t *, uint16_t);
extern ocs_hal_rtn_e ocs_hal_node_alloc(ocs_hal_t *, ocs_remote_node_t *, uint32_t, ocs_sli_port_t *);
extern ocs_hal_rtn_e ocs_hal_node_free_all(ocs_hal_t *);
extern ocs_hal_rtn_e ocs_hal_node_attach(ocs_hal_t *, ocs_remote_node_t *, ocs_dma_t *);
extern ocs_hal_rtn_e ocs_hal_node_detach(ocs_hal_t *, ocs_remote_node_t *);
extern ocs_hal_rtn_e ocs_hal_node_free_resources(ocs_hal_t *, ocs_remote_node_t *);
extern ocs_hal_rtn_e ocs_hal_node_group_alloc(ocs_hal_t *, ocs_remote_node_group_t *);
extern ocs_hal_rtn_e ocs_hal_node_group_attach(ocs_hal_t *, ocs_remote_node_group_t *, ocs_remote_node_t *);
extern ocs_hal_rtn_e ocs_hal_node_group_free(ocs_hal_t *, ocs_remote_node_group_t *);
extern ocs_hal_io_t *ocs_hal_io_alloc(ocs_hal_t *);
extern ocs_hal_io_t *ocs_hal_io_activate_port_owned(ocs_hal_t *, ocs_hal_io_t *);
extern int32_t ocs_hal_io_free(ocs_hal_t *, ocs_hal_io_t *);
extern uint8_t ocs_hal_io_inuse(ocs_hal_t *hal, ocs_hal_io_t *io);
typedef int32_t (*ocs_hal_srrs_cb_t)(ocs_hal_io_t *io, ocs_remote_node_t *rnode, uint32_t length, int32_t status, uint32_t ext_status, void *arg);
extern ocs_hal_rtn_e ocs_hal_srrs_send(ocs_hal_t *, ocs_hal_io_type_e, ocs_hal_io_t *, ocs_dma_t *, uint32_t, ocs_dma_t *, ocs_remote_node_t *, ocs_hal_io_param_t *, ocs_hal_srrs_cb_t, void *);
extern ocs_hal_rtn_e ocs_hal_io_send(ocs_hal_t *, ocs_hal_io_type_e, ocs_hal_io_t *, uint32_t, ocs_hal_io_param_t *, ocs_remote_node_t *, void *, void *);
extern ocs_hal_rtn_e _ocs_hal_io_send(ocs_hal_t *hal, ocs_hal_io_type_e type, ocs_hal_io_t *io,
				      uint32_t len, ocs_hal_io_param_t *iparam, ocs_remote_node_t *rnode,
				      void *cb, void *arg);
extern ocs_hal_rtn_e ocs_hal_io_register_sgl(ocs_hal_t *, ocs_hal_io_t *, ocs_dma_t *, uint32_t);
extern ocs_hal_rtn_e ocs_hal_io_init_sges(ocs_hal_t *hal, ocs_hal_io_t *io, ocs_hal_io_type_e type);
extern ocs_hal_rtn_e ocs_hal_io_add_seed_sge(ocs_hal_t *hal, ocs_hal_io_t *io, ocs_hal_dif_info_t *dif_info);
extern ocs_hal_rtn_e ocs_hal_io_add_sge(ocs_hal_t *, ocs_hal_io_t *, uintptr_t, uint32_t);
extern ocs_hal_rtn_e ocs_hal_io_add_dif_sge(ocs_hal_t *hal, ocs_hal_io_t *io, uintptr_t addr);
extern ocs_hal_rtn_e ocs_hal_io_abort(ocs_hal_t *, ocs_hal_io_t *, uint32_t, void *, void *);
extern int32_t ocs_hal_io_get_xid(ocs_hal_t *, ocs_hal_io_t *);
extern uint32_t ocs_hal_io_get_count(ocs_hal_t *, ocs_hal_io_count_type_e);
extern uint32_t ocs_hal_get_rqes_produced_count(ocs_hal_t *hal);

typedef void (*ocs_hal_fw_cb_t)(int32_t status, uint32_t bytes_written, uint32_t change_status, void *arg);
extern ocs_hal_rtn_e ocs_hal_firmware_write(ocs_hal_t *, ocs_dma_t *, uint32_t, uint32_t, int, ocs_hal_fw_cb_t, void*);

/* Function for retrieving SFP data */
typedef void (*ocs_hal_sfp_cb_t)(int32_t, uint32_t, uint32_t *, void *);
extern ocs_hal_rtn_e ocs_hal_get_sfp(ocs_hal_t *, uint16_t, ocs_hal_sfp_cb_t, void *);

/* Function for retrieving temperature data */
typedef void (*ocs_hal_temp_cb_t)(int32_t status,
				  uint32_t curr_temp,
				  uint32_t crit_temp_thrshld,
				  uint32_t warn_temp_thrshld,
				  uint32_t norm_temp_thrshld,
				  uint32_t fan_off_thrshld,
				  uint32_t fan_on_thrshld,
				  void *arg);
extern ocs_hal_rtn_e ocs_hal_get_temperature(ocs_hal_t *, ocs_hal_temp_cb_t, void*);

/* Function for retrieving link statistics */
typedef void (*ocs_hal_link_stat_cb_t)(int32_t status,
				       uint32_t num_counters,
				       ocs_hal_link_stat_counts_t *counters,
				       void *arg);
extern ocs_hal_rtn_e ocs_hal_get_link_stats(ocs_hal_t *,
					    uint8_t req_ext_counters,
					    uint8_t clear_overflow_flags,
					    uint8_t clear_all_counters,
					    ocs_hal_link_stat_cb_t, void*);
/* Function for retrieving host statistics */
typedef void (*ocs_hal_host_stat_cb_t)(int32_t status,
				       uint32_t num_counters,
				       ocs_hal_host_stat_counts_t *counters,
				       void *arg);
extern ocs_hal_rtn_e ocs_hal_get_host_stats(ocs_hal_t *hal, uint8_t cc, ocs_hal_host_stat_cb_t, void *arg);

extern ocs_hal_rtn_e ocs_hal_raise_ue(ocs_hal_t *, uint8_t);
typedef void (*ocs_hal_dump_get_cb_t)(int32_t status, uint32_t bytes_read, uint8_t eof, void *arg);
extern ocs_hal_rtn_e ocs_hal_dump_get(ocs_hal_t *, ocs_dma_t *, uint32_t, uint32_t, ocs_hal_dump_get_cb_t, void *);
extern ocs_hal_rtn_e ocs_hal_set_dump_location(ocs_hal_t *, uint32_t, ocs_dma_t *);

typedef void (*ocs_get_port_protocol_cb_t)(int32_t status, ocs_hal_port_protocol_e port_protocol, void *arg);
extern ocs_hal_rtn_e ocs_hal_get_port_protocol(ocs_hal_t *hal, uint32_t pci_func, ocs_get_port_protocol_cb_t mgmt_cb, void* ul_arg);
typedef void (*ocs_set_port_protocol_cb_t)(int32_t status,  void *arg);
extern ocs_hal_rtn_e ocs_hal_set_port_protocol(ocs_hal_t *hal, ocs_hal_port_protocol_e profile,
					       uint32_t pci_func, ocs_set_port_protocol_cb_t mgmt_cb,
					       void* ul_arg);

typedef void (*ocs_get_profile_list_cb_t)(int32_t status,  ocs_hal_profile_list_t*, void *arg);
extern ocs_hal_rtn_e ocs_hal_get_profile_list(ocs_hal_t *hal, ocs_get_profile_list_cb_t mgmt_cb, void *arg);
typedef void (*ocs_get_active_profile_cb_t)(int32_t status,  uint32_t active_profile, void *arg);
extern ocs_hal_rtn_e ocs_hal_get_active_profile(ocs_hal_t *hal, ocs_get_active_profile_cb_t mgmt_cb, void *arg);
typedef void (*ocs_set_active_profile_cb_t)(int32_t status, void *arg);
extern ocs_hal_rtn_e ocs_hal_set_active_profile(ocs_hal_t *hal, ocs_set_active_profile_cb_t mgmt_cb,
		uint32_t profile_id, void *arg);
typedef void (*ocs_get_nvparms_cb_t)(int32_t status, uint8_t *wwpn, uint8_t *wwnn, uint8_t hard_alpa,
		uint32_t preferred_d_id, void *arg);
extern ocs_hal_rtn_e ocs_hal_get_nvparms(ocs_hal_t *hal, ocs_get_nvparms_cb_t mgmt_cb, void *arg);
typedef void (*ocs_set_nvparms_cb_t)(int32_t status, void *arg);
extern ocs_hal_rtn_e ocs_hal_set_nvparms(ocs_hal_t *hal, ocs_set_nvparms_cb_t mgmt_cb, uint8_t *wwpn,
		uint8_t *wwnn, uint8_t hard_alpa, uint32_t preferred_d_id, void *arg);
extern int32_t ocs_hal_eq_process(ocs_hal_t *hal, hal_eq_t *eq, uint32_t max_isr_time_msec);
extern void ocs_hal_cq_process(ocs_hal_t *hal, hal_cq_t *cq);
extern void ocs_hal_wq_process(ocs_hal_t *hal, hal_cq_t *cq, uint8_t *cqe, int32_t status, uint16_t rid);
extern void ocs_hal_xabt_process(ocs_hal_t *hal, hal_cq_t *cq, uint8_t *cqe, uint16_t rid);
extern int32_t hal_wq_write(hal_wq_t *wq, ocs_hal_wqe_t *wqe);

typedef void (*ocs_hal_dump_clear_cb_t)(int32_t status, void *arg);
extern ocs_hal_rtn_e ocs_hal_dump_clear(ocs_hal_t *, ocs_hal_dump_clear_cb_t, void *);

extern uint8_t ocs_hal_is_io_port_owned(ocs_hal_t *hal, ocs_hal_io_t *io);


extern uint8_t ocs_hal_is_xri_port_owned(ocs_hal_t *hal, uint32_t xri);
extern ocs_hal_io_t * ocs_hal_io_lookup(ocs_hal_t *hal, uint32_t indicator);
extern uint32_t ocs_hal_xri_move_to_port_owned(ocs_hal_t *hal, uint32_t num_xri);
extern ocs_hal_rtn_e ocs_hal_xri_move_to_host_owned(ocs_hal_t *hal, uint8_t num_xri);
extern int32_t ocs_hal_reque_xri(ocs_hal_t *hal, ocs_hal_io_t *io);


typedef struct {
	/* structure elements used by HAL */
	ocs_hal_t *hal;			/**> pointer to HAL */
	hal_wq_callback_t *wqcb;	/**> WQ callback object, request tag */
	ocs_hal_wqe_t wqe;		/**> WQE buffer object (may be queued on WQ pending list) */
	void (*callback)(int32_t status, void *arg);	/**> final callback function */
	void *arg;			/**> final callback argument */

	/* General purpose elements */
	ocs_hal_sequence_t *seq;
	ocs_dma_t payload;		/**> a payload DMA buffer */
} ocs_hal_send_frame_context_t;

ocs_hal_rtn_e
ocs_hal_send_frame(ocs_hal_t *hal, fc_header_le_t *hdr, uint8_t sof, uint8_t eof, ocs_dma_t *payload,
		   ocs_hal_send_frame_context_t *ctx,
		   void (*callback)(void *arg, uint8_t *cqe, int32_t status), void *arg);

/* RQ completion handlers for RQ pair mode */
extern int32_t ocs_hal_rqpair_process_rq(ocs_hal_t *hal, hal_cq_t *cq, uint8_t *cqe);
extern ocs_hal_rtn_e ocs_hal_rqpair_sequence_free(ocs_hal_t *hal, ocs_hal_sequence_t *seq);
extern int32_t ocs_hal_rqpair_process_auto_xfr_rdy_cmd(ocs_hal_t *hal, hal_cq_t *cq, uint8_t *cqe);
extern int32_t ocs_hal_rqpair_process_auto_xfr_rdy_data(ocs_hal_t *hal, hal_cq_t *cq, uint8_t *cqe);
extern ocs_hal_rtn_e ocs_hal_rqpair_init(ocs_hal_t *hal);
extern ocs_hal_rtn_e ocs_hal_rqpair_auto_xfer_rdy_buffer_alloc(ocs_hal_t *hal, uint32_t num_buffers);
extern uint8_t ocs_hal_rqpair_auto_xfer_rdy_buffer_post(ocs_hal_t *hal, ocs_hal_io_t *io, int reuse_buf);
extern ocs_hal_rtn_e ocs_hal_rqpair_auto_xfer_rdy_move_to_port(ocs_hal_t *hal, ocs_hal_io_t *io);
extern void ocs_hal_rqpair_auto_xfer_rdy_move_to_host(ocs_hal_t *hal, ocs_hal_io_t *io);
extern void ocs_hal_rqpair_teardown(ocs_hal_t *hal);

extern ocs_hal_rtn_e ocs_hal_rx_allocate(ocs_hal_t *hal);
extern ocs_hal_rtn_e ocs_hal_rx_post(ocs_hal_t *hal);
extern void ocs_hal_rx_free(ocs_hal_t *hal);

extern void ocs_hal_unsol_process_bounce(void *arg);

typedef int32_t (*ocs_hal_async_cb_t)(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg);
extern int32_t ocs_hal_async_call(ocs_hal_t *hal, ocs_hal_async_cb_t callback, void *arg);

static inline void
ocs_hal_sequence_copy(ocs_hal_sequence_t *dst, ocs_hal_sequence_t *src)
{
	/* Copy the src to dst, then zero out the linked list link */
	*dst = *src;
	ocs_memset(&dst->link, 0, sizeof(dst->link));
}

static inline ocs_hal_rtn_e
ocs_hal_sequence_free(ocs_hal_t *hal, ocs_hal_sequence_t *seq)
{
	/* Only RQ pair mode is supported */
	return ocs_hal_rqpair_sequence_free(hal, seq);
}

/* HAL WQ request tag API */
extern ocs_hal_rtn_e ocs_hal_reqtag_init(ocs_hal_t *hal);
extern hal_wq_callback_t *ocs_hal_reqtag_alloc(ocs_hal_t *hal,
					       void (*callback)(void *arg, uint8_t *cqe, int32_t status), void *arg);
extern void ocs_hal_reqtag_free(ocs_hal_t *hal, hal_wq_callback_t *wqcb);
extern hal_wq_callback_t *ocs_hal_reqtag_get_instance(ocs_hal_t *hal, uint32_t instance_index);
extern void ocs_hal_reqtag_reset(ocs_hal_t *hal);

/* Uncomment to enable CPUTRACE */
//#define ENABLE_CPUTRACE
#ifdef ENABLE_CPUTRACE
#define CPUTRACE(t) ocs_printf("trace: %-20s %2s %-16s cpu %2d\n", __func__, t, \
	({ocs_thread_t *self = ocs_thread_self(); self != NULL ? self->name : "unknown";}), ocs_thread_getcpu());
#else
#define CPUTRACE(...)
#endif

#endif /* !_OCS_HAL_H */

/* vim: set noexpandtab textwidth=120: */
