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

#if !defined(__OCS_SCSI_INI_H__)
#define __OCS_SCSI_INI_H__

#define OCS_INCLUDE_INI_STUB

#include "spdk_internal/event.h"


#define LUNFMTx                 PRIx64
#define LUNFMTd                 PRId64

/*
 * The Linux SCSI mid-layer converts the 8 byte LUN:
 * aa bb cc dd 00 00 00 00
 * into the integer:
 * 0xccddaabb
 * Use this mask to convert the Linux LUN definition to the
 * OCS LUN definition
 */
#define OCS_SCSI_ADDRESS_METHOD_MASK            0x0000c000
#define OCS_SCSI_ADDRESS_METHOD_PERIPHERAL      0x00000000
#define OCS_SCSI_ADDRESS_METHOD_FLAT            0x00004000
#define OCS_SCSI_ADDRESS_METHOD_EXTENDED        0x0000c000

#define SAM_STAT_GOOD            0x00
#define SAM_STAT_CHECK_CONDITION 0x02
#define SAM_STAT_CONDITION_MET   0x04
#define SAM_STAT_BUSY            0x08
#define SAM_STAT_INTERMEDIATE    0x10
#define SAM_STAT_INTERMEDIATE_CONDITION_MET 0x14
#define SAM_STAT_RESERVATION_CONFLICT 0x18
#define SAM_STAT_TASK_SET_FULL   0x28
#define SAM_STAT_ACA_ACTIVE      0x30
#define SAM_STAT_TASK_ABORTED    0x40

/* FC Initiator commands */
typedef enum _SPDK_FC_INI_CMD_TYPE {
	SPDK_FC_INI_CMD_TYPE_READ = 1,
	SPDK_FC_INI_CMD_TYPE_WRITE,
	SPDK_FC_INI_CMD_TYPE_SCSI_PASS_THRU,
	SPDK_FC_INI_CMD_TYPE_RESET

} SPDK_FC_INI_CMD_TYPE;

/* FC Initiator Reset types commands */
typedef enum _SPDK_FC_INI_RESET_TYPE {
	SPDK_FC_INI_BDEV_RESET = 1,
	SPDK_FC_INI_BDEV_FLUSH,

} SPDK_FC_INI_RESET_TYPE;


/* SCSI cmd result from FC initiator */
typedef struct ocs_blk_req_result {
	uint32_t host_result;         /* cmd. status from initiator  */
	uint32_t tgt_result;          /* cmd. status from target  */
	uint64_t data_transferred;    /* actual amount of data transferred */
#define OCS_SENSE_BUF_LEN 32
	uint8_t scsi_resp[OCS_SENSE_BUF_LEN]; /* SCSI sense data  */
	uint32_t scsi_resp_len;       /* length of SCSI sense data */

} ocs_blk_req_result_t;

/* FC initiator command completion info structure */
typedef struct spdk_fc_ini_cmd_complete_info {
	SPDK_FC_INI_CMD_TYPE cmd_type;     /* see SPDK_FC_INI_CMD_TYPE */
	struct spdk_fc_ini_bdev *bdev;     /* FC block device for command */
	void *cb_data;                     /* user defined callback data (cb_func arg) */
	ocs_blk_req_result_t cmd_result;   /* cmd result data */

} spdk_fc_ini_cmd_complete_info_t;

/* FC initiator command complete callback */
typedef void (*spdk_fc_ini_cmd_completion_cb)(struct spdk_fc_ini_cmd_complete_info *cb_info);

/* scsi block request structure */
typedef struct ocs_blk_req_s {
	ocs_node_t *node;		/* the target node */
	uint32_t blk_req_tag;		/* unique request identifier */
	uint64_t lun;			/* LUN number for cmd */
	uint8_t scsi_cdb[32];		/* SCSI CDB for command */
	int cdb_len;			/* SCSI CDB length */
	struct iovec* iov;		/* data buffers for cmd */
	uint32_t iovcnt;		/* size of iov */
	uint64_t data_len;		/* data xfer length */
	int data_dir;			/* data direction */
	uint32_t timeout;		/* command timeout */
	struct spdk_fc_ini_cmd_complete_info cmd_compl_info;
	struct spdk_event *done_event;	/* done event called by FC ini when cmd. completes */
	spdk_fc_ini_cmd_completion_cb cb_func; /* CAS callback function when cmd. completes */
	uint32_t gencnt;

} ocs_blk_req_t;


typedef struct {
} ocs_scsi_ini_t;

typedef struct {
} ocs_scsi_ini_domain_t;

typedef struct {
} ocs_scsi_ini_sport_t;

typedef struct {
} ocs_scsi_ini_node_t;

typedef struct {
	ocs_blk_req_t *req;
} ocs_scsi_ini_io_t;

int ocs_ml_fc_queue_cmd(ocs_blk_req_t *req);
int ocs_lun_reset(ocs_blk_req_t *req);

void ocsu_ini_init(void);

#endif // __OCS_SCSI_INI_H__
