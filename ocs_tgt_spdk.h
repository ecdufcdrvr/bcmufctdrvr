/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2018 Broadcom.  All Rights Reserved.
 *  The term "Broadcom" refers to Broadcom Limited and/or its subsidiaries.
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
 *
 */
#if !defined(__OCS_TGT_SPDK_H__)
#define __OCS_TGT_SPDK_H__

#define BUILD_ETC_IDS "/usr/local/etc/ids"
#define DEFAULT_CONFIG BUILD_ETC_IDS "/ids.conf"

#include "ocs_scsi_tgt.h"
#include <rte_lcore.h>
#include <rte_per_lcore.h>
#include <rte_eal.h>
#include <rte_launch.h>
#include <rte_mempool.h>
#include "spdk/log.h"

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

typedef struct {
        uint8_t response_code:7,
                :1;
        uint8_t sense_key:4,
		:4;
        uint8_t asc;
        uint8_t ascq;
        uint8_t rsvd1:7,
                sdat_ovfl:1;
        uint8_t rsvd2[2];
        uint8_t additional_sense_length;
        uint8_t descriptors[0];
} descriptor_sense_data_t;

typedef union {
        uint8_t response_code:7,
                :1;
        fixed_sense_data_t fixed;
        descriptor_sense_data_t	descriptor;
} sense_data_t;

struct sense_info {
	uint8_t response_code;
	uint8_t sense_key;
	uint8_t asc;
	uint8_t ascq;
};

/* Handler return codes */
typedef enum {
        SCSI_HANDLER_DATAPHASE_STARTED = 1,
        SCSI_HANDLER_RESP_STARTED,
        SCSI_HANDLER_VALIDATED_DATAPHASE_STARTED,
        SCSI_HANDLER_WRITE_SAME_DATAPHASE_STARTED,
        SCSI_HANDLER_NOT_STARTED,
        SCSI_CMD_NOT_SUPPORTED,
} ocs_spdk_hndlr_rtn_e;

/* Sense Keys */
#define SNS_ILLEGAL_REQ   0x05   /* sense key is byte 3 ([2]) */
#define SNS_UNIT_ATTENTION      0x06
#define SNS_ABORTED_CMD   0x0B   /* sense key - aborted command */

/* Additional Sense Codes */
#define SNSCODE_INV_CMDIU 0x0E    /* INVALID FIELD IN COMMAND IU */
#define SNSCODE_DIF_ERR   0x10   /* ID CRC OR ECC ERROR */
#define SNSCODE_BAD_CMD   0x20   /* INVALID COMMAND OPERATION CODE */
#define SNSCODE_LBA_OOR   0x21   /* LOGICAL BLOCK ADDRESS OUT OF RANGE */
#define SNSCODE_INV_CDB   0x24   /* INVALID FIELD IN CDB */
#define SNSCODE_INV_PRM   0x26   /* INVALID FIELD IN PARAMETER LIST */
#define SNSCODE_ABORTED_CMD 0x47   /* Aborted command */
#define SNSCODE_OFF_ERR   0x4B   /* DATA PHASE ERROR */
#define SNSCODE_SAVE_NOT_SUPPORTED 0x39 /* SAVING PARAMETERS NOT SUPPORTED */

/* Additional Sense Code Qualifiers */
#define SNSCODEQUAL_INV_CMDIU 0x03  /* INVALID FIELD IN COMMAND IU */

void
ocs_scsi_tgt_mgmt_status(ocs_textbuf_t *textbuf, ocs_scsi_ddump_type_e type, void *obj);
void
ocs_scsi_tgt_mgmt_config(ocs_textbuf_t *textbuf, ocs_scsi_ddump_type_e type, void *obj);

int ocs_get_sense_info(uint8_t *sense_buf, uint8_t sense_len,
	struct sense_info *sense_info);
#endif // __OCS_TGT_SPDK_H__
