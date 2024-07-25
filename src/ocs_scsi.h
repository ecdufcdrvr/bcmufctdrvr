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
 * OCS SCSI API declarations
 *
 */

#if !defined(__OCS_SCSI_H__)
#define __OCS_SCSI_H__

#include "ocs_ddump.h"
#include "ocs_mgmt.h"
#include "ocs_fcp.h"

/* ocs_scsi_rcv_cmd() ocs_scsi_rcv_tmf() flags */
#define OCS_SCSI_CMD_DIR_IN		(1U << 0)
#define OCS_SCSI_CMD_DIR_OUT		(1U << 1)
#define OCS_SCSI_CMD_SIMPLE		(1U << 2)
#define OCS_SCSI_CMD_HEAD_OF_QUEUE	(1U << 3)
#define OCS_SCSI_CMD_ORDERED		(1U << 4)
#define OCS_SCSI_CMD_UNTAGGED		(1U << 5)
#define OCS_SCSI_CMD_ACA		(1U << 6)
#define OCS_SCSI_FIRST_BURST_ERR	(1U << 7)
#define OCS_SCSI_FIRST_BURST_ABORTED	(1U << 8)

/* ocs_scsi_send_rd_data/recv_wr_data/send_resp flags */
#define OCS_SCSI_LAST_DATAPHASE		(1U << 0)
#define OCS_SCSI_NO_AUTO_RESPONSE	(1U << 1)
#define OCS_SCSI_LOW_LATENCY		(1U << 2)

#define OCS_SCSI_MAX_CDB_LEN		128
#define OCS_SCSI_SNS_BUF_VALID(sense)	((sense) && (0x70 == (((const uint8_t *)(sense))[0] & 0x70)))

#define OCS_SCSI_WQ_STEERING_SHIFT	(16)
#define OCS_SCSI_WQ_STEERING_MASK	(0xf << OCS_SCSI_WQ_STEERING_SHIFT)
#define OCS_SCSI_WQ_STEERING_CLASS	(1 << OCS_SCSI_WQ_STEERING_SHIFT)
#define OCS_SCSI_WQ_STEERING_REQUEST	(2 << OCS_SCSI_WQ_STEERING_SHIFT)
#define OCS_SCSI_WQ_STEERING_CPU	(3 << OCS_SCSI_WQ_STEERING_SHIFT)

#define OCS_SCSI_WQ_CLASS_SHIFT		(20)
#define OCS_SCSI_WQ_CLASS_MASK		(0xf << OCS_SCSI_WQ_CLASS_SHIFT)
#define OCS_SCSI_WQ_CLASS(x)		((x & OCS_SCSI_WQ_CLASS_MASK) << OCS_SCSI_WQ_CLASS_SHIFT)

#define OCS_SCSI_WQ_CLASS_LOW_LATENCY	(1)

#define OCS_SCSI_MAX_LOG_LEN		256

typedef enum {
	OCS_SCSI_IO_PROC_CMD_RECVD_TSTART = 0,
	OCS_SCSI_IO_PROC_CMD_RECVD_TFIN,
	OCS_SCSI_IO_PROC_CMD_SUBMITTED_TSTART,
	OCS_SCSI_IO_PROC_CMD_SUBMITTED_TFIN,

	OCS_SCSI_IO_PROC_CMD_STAGE_MAX,
} ocs_scsi_io_proc_time_e;

/*!
 * @defgroup scsi_api_base SCSI Base Target/Initiator
 * @defgroup scsi_api_target SCSI Target
 * @defgroup scsi_api_initiator SCSI Initiator
 */

/**
 * @brief SCSI command response.
 *
 * This structure is used by target-servers to specify SCSI status and
 * sense data.  In this case all but the @b residual element are used. For
 * initiator-clients, this structure is used by the SCSI API to convey the
 * response data for issued commands, including the residual element.
 */
typedef struct {
	uint8_t scsi_status;			/**< SCSI status */
	uint16_t scsi_status_qualifier;		/**< SCSI status qualifier */
	uint8_t *response_data;			/**< pointer to response data buffer */
	uint32_t response_data_length;		/**< length of response data buffer (bytes) */
	uint8_t *sense_data;			/**< pointer to sense data buffer */
	uint32_t sense_data_length;		/**< length of sense data buffer (bytes) */
	int32_t residual;			/**< command residual (not used for target), positive value
						  *  indicates an underflow, negative value indicates overflow
						  */
	uint32_t response_wire_length;		/**< Command response length received in wcqe */
} ocs_scsi_cmd_resp_t;

/* Status values returned by IO callbacks */
typedef enum {
	OCS_SCSI_STATUS_GOOD = 0,
	OCS_SCSI_STATUS_ABORTED,
	OCS_SCSI_STATUS_ERROR,
	OCS_SCSI_STATUS_DIF_GUARD_ERROR,
	OCS_SCSI_STATUS_DIF_REF_TAG_ERROR,
	OCS_SCSI_STATUS_DIF_APP_TAG_ERROR,
	OCS_SCSI_STATUS_DIF_UNKNOWN_ERROR,
	OCS_SCSI_STATUS_PROTOCOL_CRC_ERROR,
	OCS_SCSI_STATUS_NO_IO,
	OCS_SCSI_STATUS_ABORT_IN_PROGRESS,
	OCS_SCSI_STATUS_CHECK_RESPONSE,
	OCS_SCSI_STATUS_COMMAND_TIMEOUT,
	OCS_SCSI_STATUS_TIMEDOUT_AND_ABORTED,
	OCS_SCSI_STATUS_SHUTDOWN,
	OCS_SCSI_STATUS_NEXUS_LOST,
} ocs_scsi_io_status_e;

/* Sense Keys */
#define SNS_NOT_READY				0x02	/* sense key - not ready */
#define SNS_ILLEGAL_REQ				0x05	/* sense key is byte 3 ([2]) */
#define SNS_UNIT_ATTENTION			0x06
#define SNS_ABORTED_CMD				0x0B	/* sense key - aborted command */

/* Sense Codes */
#define SNSCODE_INV_CMDIU			0x0E	/* INVALID FIELD IN COMMAND IU */
#define SNSCODE_DIF_ERR				0x10	/* ID CRC OR ECC ERROR */
#define SNSCODE_BAD_CMD				0x20	/* INVALID COMMAND OPERATION CODE */
#define SNSCODE_LBA_OOR				0x21	/* LOGICAL BLOCK ADDRESS OUT OF RANGE */
#define SNSCODE_INV_CDB				0x24	/* INVALID FIELD IN CDB */
#define SNSCODE_INV_PRM				0x26	/* INVALID FIELD IN PARAMETER LIST */
#define SNSCODE_SAVE_NOT_SUPPORTED		0x39	/* SAVING PARAMETERS NOT SUPPORTED */
#define SNSCODE_ABORTED_CMD			0x47	/* Aborted command */
#define SNSCODE_OFF_ERR				0x4B	/* DATA PHASE ERROR */

/* Additional Sense Code Qualifiers */
#define SNSCODEQUAL_INV_CMDIU			0x03	/* INVALID FIELD IN COMMAND IU */

/**
 * @brief Defines the standard sense response bits per the
 * <i>SCSI Primary Commands (SPC) specification</i>.
 */
typedef struct {
	uint8_t response_code:7,
		:1;
	uint8_t :8;
	uint8_t sense_key:4,
		:1,
		ili:1,
		eom:1,
		filemark:1;
	uint8_t information[4];
	uint8_t additional_sense_length;
	uint8_t command_specific_information[4];
	uint8_t asc;
	uint8_t ascq;
	uint8_t fru_code;
	uint8_t sense_key_specific_hi:7,
		sksv:1;
	uint8_t sense_key_specific_lo[2];
} fixed_sense_data_t;

/* Callback used by send_rd_data(), recv_wr_data(), send_resp() */
typedef int32_t (*ocs_scsi_io_cb_t)(ocs_io_t *io, ocs_scsi_io_status_e status, uint32_t flags, void *arg);

/* Callback used by send_rd_io(), send_wr_io() */
typedef int32_t (*ocs_scsi_rsp_io_cb_t)(ocs_io_t *io, ocs_scsi_io_status_e status, ocs_scsi_cmd_resp_t *rsp,
	uint32_t flags, void *arg);

/* ocs_scsi_cb_t flags */
#define OCS_SCSI_IO_CMPL		(1U << 0)	/* IO completed */
#define OCS_SCSI_IO_CMPL_RSP_SENT	(1U << 1)	/* IO completed, response sent */
#define OCS_SCSI_IO_ABORTED		(1U << 2)	/* IO was aborted */

/* ocs_scsi_recv_tmf() request values */
typedef enum {
	OCS_SCSI_TMF_NONE = 0,
	OCS_SCSI_TMF_ABORT_TASK,
	OCS_SCSI_TMF_QUERY_TASK_SET,
	OCS_SCSI_TMF_ABORT_TASK_SET,
	OCS_SCSI_TMF_CLEAR_TASK_SET,
	OCS_SCSI_TMF_QUERY_ASYNCHRONOUS_EVENT,
	OCS_SCSI_TMF_LOGICAL_UNIT_RESET,
	OCS_SCSI_TMF_CLEAR_ACA,
	OCS_SCSI_TMF_TARGET_RESET,
	OCS_SCSI_TMF_IT_NEXUS_LOSS,
} ocs_scsi_tmf_cmd_e;

/* ocs_scsi_send_tmf_resp() response values */
typedef enum {
	OCS_SCSI_TMF_FUNCTION_COMPLETE = 1,
	OCS_SCSI_TMF_FUNCTION_IO_NOT_FOUND,
	OCS_SCSI_TMF_FUNCTION_REJECTED,
	OCS_SCSI_TMF_INCORRECT_LOGICAL_UNIT_NUMBER,
	OCS_SCSI_TMF_SERVICE_DELIVERY,
} ocs_scsi_tmf_resp_e;

/**
 * @brief property names for ocs_scsi_get_property() functions
 */
typedef enum {
	OCS_SCSI_MAX_SGE,
	OCS_SCSI_MAX_SGL,
	OCS_SCSI_WWNN,
	OCS_SCSI_WWPN,
	OCS_SCSI_SERIALNUMBER,
	OCS_SCSI_PARTNUMBER,
	OCS_SCSI_PORTNUM,
	OCS_SCSI_BIOS_VERSION_STRING,
	OCS_SCSI_MAX_IOS,
	OCS_SCSI_DIF_CAPABLE,
	OCS_SCSI_DIF_MULTI_SEPARATE,
	OCS_SCSI_MAX_FIRST_BURST,
	OCS_SCSI_ENABLE_TASK_SET_FULL,
} ocs_scsi_property_e;

#define DIF_SIZE		8

/**
 * @brief T10 DIF operations
 *
 *	WARNING: do not reorder or insert to this list without making appropriate changes in ocs_dif.c
 */
typedef enum {
	OCS_SCSI_DIF_OPER_DISABLED,
	OCS_SCSI_DIF_OPER_IN_NODIF_OUT_CRC,
	OCS_SCSI_DIF_OPER_IN_CRC_OUT_NODIF,
	OCS_SCSI_DIF_OPER_IN_NODIF_OUT_CHKSUM,
	OCS_SCSI_DIF_OPER_IN_CHKSUM_OUT_NODIF,
	OCS_SCSI_DIF_OPER_IN_CRC_OUT_CRC,
	OCS_SCSI_DIF_OPER_IN_CHKSUM_OUT_CHKSUM,
	OCS_SCSI_DIF_OPER_IN_CRC_OUT_CHKSUM,
	OCS_SCSI_DIF_OPER_IN_CHKSUM_OUT_CRC,
	OCS_SCSI_DIF_OPER_IN_RAW_OUT_RAW,
} ocs_scsi_dif_oper_e;

#define OCS_SCSI_DIF_OPER_PASS_THRU	OCS_SCSI_DIF_OPER_IN_CRC_OUT_CRC
#define OCS_SCSI_DIF_OPER_STRIP		OCS_SCSI_DIF_OPER_IN_CRC_OUT_NODIF
#define OCS_SCSI_DIF_OPER_INSERT	OCS_SCSI_DIF_OPER_IN_NODIF_OUT_CRC

/**
 * @brief T10 DIF block sizes
 */
typedef enum {
	OCS_SCSI_DIF_BK_SIZE_512,
	OCS_SCSI_DIF_BK_SIZE_1024,
	OCS_SCSI_DIF_BK_SIZE_2048,
	OCS_SCSI_DIF_BK_SIZE_4096,
	OCS_SCSI_DIF_BK_SIZE_520,
	OCS_SCSI_DIF_BK_SIZE_4104
} ocs_scsi_dif_blk_size_e;

/**
 * @brief generic scatter-gather list structure
 */
typedef struct ocs_scsi_sgl_s {
	uintptr_t	addr;			/**< physical address */
	uintptr_t	dif_addr;		/**< address of DIF segment, zero if DIF is interleaved */
	size_t		len;			/**< length */
} ocs_scsi_sgl_t;

/**
 * @brief T10 DIF information passed to the transport
 */
typedef struct {
	ocs_scsi_dif_oper_e dif_oper;
	ocs_scsi_dif_blk_size_e blk_size;
	uint32_t ref_tag;
	uint16_t app_tag;
	bool	check_ref_tag;
	bool	check_app_tag;
	bool	check_guard;
	bool	dif_separate;

		/* If the APP TAG is 0xFFFF, disable checking the REF TAG and CRC fields */
	bool	disable_app_ffff;

		/* if the APP TAG is 0xFFFF and REF TAG is 0xFFFF_FFFF, disable checking the received CRC field. */
	bool	disable_app_ref_ffff;
	uint64_t lba;
} ocs_scsi_dif_info_t;

/* Return values for calls from base driver to target-server/initiator-client */
#define OCS_CALL_COMPLETE	0 // All work is done
#define OCS_CALL_ASYNC		1 // Work will be completed asynchronously
#define OCS_SCSI_CALL_COMPLETE	0 // All work is done
#define OCS_SCSI_CALL_ASYNC	1 // Work will be completed asynchronously
#define OCS_NVME_CALL_COMPLETE	0 // All work is done
#define OCS_NVME_CALL_ASYNC	1 // Work will be completed asynchronously

/* Calls from target/initiator to base driver */

typedef enum {
	OCS_SCSI_IO_ROLE_ORIGINATOR,
	OCS_SCSI_IO_ROLE_RESPONDER,
} ocs_scsi_io_role_e;

typedef struct ocs_sframe_rsp_args_s {
	uint32_t s_id;
	uint32_t d_id;
	uint32_t ox_id;
	uint32_t rx_id;
	void (*callback)(ocs_hal_t *hal, int32_t status, void *arg);
	void *cb_arg;
	fcp_rsp_iu_t *fcprsp;
	uint32_t fcprsp_len;
} ocs_sframe_rsp_args_t;

extern int32_t ocs_sframe_send_fcp_rsp(ocs_node_t *node, ocs_sframe_rsp_args_t *fcp_args);

extern void ocs_scsi_io_alloc_enable(ocs_node_t *node);
extern void ocs_scsi_io_alloc_disable(ocs_node_t *node);
extern int32_t ocs_scsi_io_alloc_enabled(ocs_node_t *node);
extern ocs_io_t *ocs_scsi_io_alloc(ocs_node_t *node, ocs_scsi_io_role_e role, uint32_t eq_idx);
extern void ocs_scsi_io_free(ocs_io_t *io);
extern ocs_io_t *ocs_io_get_instance(ocs_t *ocs, uint32_t index);
extern void ocs_scsi_register_bounce(ocs_t *ocs, void(*fctn)(void(*fctn)(void *arg), void *arg,
				     uint32_t s_id, uint32_t d_id, uint32_t ox_id));

/* Calls from base driver to target-server */

extern int32_t ocs_scsi_tgt_driver_init(void);
extern int32_t ocs_scsi_tgt_driver_exit(void);
extern uint32_t ocs_scsi_tgt_io_state(ocs_io_t *io);
extern int32_t ocs_scsi_tgt_io_init(ocs_io_t *io);
extern int32_t ocs_scsi_tgt_io_exit(ocs_io_t *io);
extern int32_t ocs_scsi_tgt_new_device(ocs_t *ocs);
extern void ocs_tgt_common_new_device(ocs_t *ocs);
extern void ocs_tgt_common_del_device(ocs_t *ocs);
extern int32_t ocs_scsi_tgt_del_device(ocs_t *ocs);
extern int32_t ocs_scsi_tgt_new_domain(ocs_domain_t *domain);
extern int ocs_scsi_tgt_del_domain(ocs_domain_t *domain);
extern int32_t ocs_scsi_tgt_new_sport(ocs_sport_t *sport);
extern void ocs_scsi_tgt_del_sport(ocs_sport_t *sport);
extern int32_t ocs_scsi_validate_initiator(ocs_node_t *node);
extern int32_t ocs_scsi_new_initiator(ocs_node_t *node, void *cbdata);
extern int32_t ocs_scsi_del_initiator(ocs_node_t *node, void *cbdata);
extern bool ocs_tgt_scsi_backend_enabled(ocs_t *ocs, ocs_sport_t *sport);
extern bool ocs_ini_scsi_backend_enabled(ocs_t *ocs, ocs_sport_t *sport);
extern bool ocs_scsi_backend_enabled(ocs_t *ocs, ocs_sport_t *sport);

extern int32_t ocs_scsi_recv_cmd(ocs_io_t *io, uint64_t lun, uint8_t *cdb, uint32_t cdb_len, uint32_t flags);
extern int32_t ocs_scsi_recv_cmd_first_burst(ocs_io_t *io, uint64_t lun, uint8_t *cdb, uint32_t cdb_len,
	uint32_t flags, ocs_dma_t first_burst_buffers[], uint32_t first_burst_bytes);
extern int32_t ocs_scsi_recv_tmf(ocs_io_t *tmfio, uint64_t lun, ocs_scsi_tmf_cmd_e cmd, ocs_io_t *abortio,
	uint32_t flags);
extern ocs_sport_t *ocs_sport_get_instance(ocs_domain_t *domain, uint32_t index);
extern ocs_domain_t *ocs_domain_get_instance(ocs_t *ocs, uint32_t index);

/* Calls from target-server to base driver */

extern int32_t ocs_scsi_send_rd_data(ocs_io_t *io, uint32_t flags,
	ocs_scsi_dif_info_t *dif_info,
	ocs_scsi_sgl_t *sgl, uint32_t sgl_count,
	uint64_t wire_len, ocs_scsi_io_cb_t cb, void *arg);
extern int32_t ocs_scsi_recv_wr_data(ocs_io_t *io, uint32_t flags,
	ocs_scsi_dif_info_t *dif_info,
	ocs_scsi_sgl_t *sgl, uint32_t sgl_count,
	uint64_t wire_len, ocs_scsi_io_cb_t cb, void *arg);
extern int32_t ocs_scsi_send_resp(ocs_io_t *io, uint32_t flags, ocs_scsi_cmd_resp_t *rsp,
		ocs_scsi_io_cb_t cb, void *arg);
extern int32_t ocs_scsi_send_tmf_resp(ocs_io_t *io, ocs_scsi_tmf_resp_e rspcode, uint8_t addl_rsp_info[3],
		ocs_scsi_io_cb_t cb, void *arg);
extern int32_t ocs_scsi_tgt_abort_io(ocs_io_t *io, bool send_abts, ocs_scsi_io_cb_t cb, void *arg);
extern void ocs_scsi_io_complete(ocs_io_t *io);
extern uint32_t ocs_scsi_get_property(ocs_t *ocs, ocs_scsi_property_e prop);
extern void *ocs_scsi_get_property_ptr(ocs_t *ocs, ocs_scsi_property_e prop);
extern void ocs_scsi_del_initiator_complete(ocs_node_t *node, void *cbdata);
extern void ocs_scsi_update_first_burst_transferred(ocs_io_t *io, uint32_t transferred);

/* Calls from base driver to initiator-client */

extern int32_t ocs_scsi_ini_driver_init(int32_t initiator);
extern int32_t ocs_scsi_ini_driver_exit(int32_t initiator);
extern int32_t ocs_scsi_ini_io_init(ocs_io_t *io);
extern int32_t ocs_scsi_ini_io_exit(ocs_io_t *io);
extern int32_t ocs_scsi_host_ml_registered(ocs_t *ocs);
extern int ocs_scsi_ini_update_fc_host(ocs_t *ocs, uint64_t port_name, uint64_t node_name);
extern void ocs_scsi_ini_update_symbolic_name(ocs_t *ocs);
extern int32_t ocs_scsi_ini_new_device(ocs_t *ocs);
extern int32_t ocs_scsi_ini_del_device(ocs_t *ocs);
extern int32_t ocs_scsi_ini_new_domain(ocs_domain_t *domain);
extern int32_t ocs_scsi_host_update_supported_speed(ocs_t *ocs);
extern void ocs_scsi_host_set_supported_mode(ocs_t *ocs);
extern void ocs_scsi_host_set_active_mode(ocs_t *ocs);
extern void ocs_scsi_ini_del_domain(ocs_domain_t *domain);
extern int32_t ocs_scsi_ini_new_sport(ocs_sport_t *sport);
extern void ocs_scsi_ini_del_sport(ocs_sport_t *sport);
extern int32_t ocs_scsi_new_target(ocs_node_t *node);
extern int32_t ocs_scsi_del_target(ocs_node_t *node);

extern void ocs_get_os_host_name(ocs_t *ocs, void *os_ver_name_str, size_t size);
extern void ocs_get_os_version_and_name(ocs_t *ocs, void *os_ver_name_str, size_t size);
extern void ocs_scsi_get_host_devname(ocs_t *ocs, void *host_devname, size_t size);

/* Calls from the initiator-client to the base driver */

extern int32_t ocs_scsi_send_rd_io(ocs_node_t *node, ocs_io_t *io, uint64_t lun, void *cdb, uint32_t cdb_len,
	ocs_scsi_dif_info_t *dif_info,
	ocs_scsi_sgl_t *sgl, uint32_t sgl_count, uint32_t wire_len, ocs_scsi_rsp_io_cb_t cb, void *arg);
extern int32_t ocs_scsi_send_wr_io(ocs_node_t *node, ocs_io_t *io, uint64_t lun, void *cdb, uint32_t cdb_len,
	ocs_scsi_dif_info_t *dif_info,
	ocs_scsi_sgl_t *sgl, uint32_t sgl_count, uint32_t wire_len, ocs_scsi_rsp_io_cb_t cb, void *arg);
extern int32_t ocs_scsi_send_wr_io_first_burst(ocs_node_t *node, ocs_io_t *io, uint64_t lun, void *cdb, uint32_t cdb_len,
	ocs_scsi_dif_info_t *dif_info,
	ocs_scsi_sgl_t *sgl, uint32_t sgl_count, uint32_t wire_len, uint32_t first_burst,
	ocs_scsi_rsp_io_cb_t cb, void *arg);
extern int32_t ocs_scsi_send_tmf(ocs_node_t *node, ocs_io_t *io, ocs_io_t *io_to_abort, uint64_t lun,
	ocs_scsi_tmf_cmd_e tmf, ocs_scsi_sgl_t *sgl, uint32_t sgl_count, uint32_t len, ocs_scsi_rsp_io_cb_t cb, void *arg);
extern int32_t ocs_scsi_send_nodata_io(ocs_node_t *node, ocs_io_t *io, uint64_t lun, void *cdb, uint32_t cdb_len, ocs_scsi_rsp_io_cb_t cb, void *arg);
extern void ocs_scsi_del_target_complete(ocs_node_t *node);
extern void ocs_scsi_io_timedout(ocs_io_t *io);

typedef enum {
	OCS_SCSI_DDUMP_DEVICE,
	OCS_SCSI_DDUMP_DOMAIN,
	OCS_SCSI_DDUMP_SPORT,
	OCS_SCSI_DDUMP_NODE,
	OCS_SCSI_DDUMP_IO,
} ocs_scsi_ddump_type_e;

/* base driver to target/initiator */

typedef struct {
	void *vaddr;
	uint32_t length;
} ocs_scsi_vaddr_len_t;

extern int32_t ocs_scsi_get_block_vaddr(ocs_io_t *io, uint64_t blocknumber, ocs_scsi_vaddr_len_t addrlen[],
	uint32_t max_addrlen, void **dif_vaddr);

extern void ocs_fc_encode_lun(ocs_node_t *node, uint64_t lun, uint8_t *lu);
extern uint64_t ocs_fc_decode_lun(ocs_node_t *node, uint8_t *lu);

extern void ocs_scsi_ini_ddump(ocs_textbuf_t *textbuf, ocs_scsi_ddump_type_e type, void *obj);
extern void ocs_scsi_tgt_ddump(ocs_textbuf_t *textbuf, ocs_scsi_ddump_type_e type, void *obj);
extern void ocs_scsi_tgt_notify_node_force_free(ocs_node_t *node);

void ocs_scsi_tgt_cancel_io(ocs_t *ocs);
void ocs_scsi_tgt_sess_del_wait(ocs_sport_t *sport);
void ocs_scsi_tgt_vport_node_shutdown(ocs_sport_t *sport);
void ocs_scsi_tgt_count_sessions(ocs_t *ocs, uint64_t wwpn);
int32_t ocs_scsi_tgt_port_online(ocs_t *ocs);
extern void ocs_vport_logout_notify(ocs_sport_t *sport);

#endif // __OCS_SCSI_H__
