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
 * Declares the target specific structures required by the base driver
 */

#if !defined(__OCS_SCSI_TGT_H__)
#define __OCS_SCSI_TGT_H__

#include "ocs_scsi.h"

#define OCS_SPDK_BUILD_NODE_NAME \
	do { \
		uint8_t *pwwn; \
		uint64_t wwpn; \
		wwpn = ocs_node_get_wwpn(node); \
		pwwn = (uint8_t*)(&wwpn); \
		ocs_snprintf(pname, sizeof(pname), "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", \
				pwwn[7], \
				pwwn[6], \
				pwwn[5], \
				pwwn[4], \
				pwwn[3], \
				pwwn[2], \
				pwwn[1], \
				pwwn[0]); \
	} while(0)

struct spdk_nvmf_fc_hw_port_init_args;

/**
 * @brief target private ocs structure
 */
typedef struct {
	uint32_t max_sge;
	uint32_t max_sgl;

	/*
	 * Variables used to send task set full. We are using a high watermark
	 * method to send task set full. We will reserve a fixed number of IOs
	 * per initiator plus a fudge factor. Once we reach this number,
	 * then the target will start sending task set full/busy responses.
	 */
	ocs_atomic_t initiator_count;		/**< count of initiators */
	ocs_atomic_t ios_in_use;		/**< num of IOs in use */
	ocs_atomic_t io_high_watermark;		/**< used to send task set full */
	ocs_atomic_t watermark_hit;		/**< used to track how often IO pool almost empty */
	int32_t	watermark_min;			/**< lower limit for watermark */
	int32_t	watermark_max;			/**< upper limit for watermark */
} ocs_scsi_tgt_t;

/**
 * @brief target private domain structure
 */

typedef struct {
	;
} ocs_scsi_tgt_domain_t;

/**
 * @brief target private sport structure
 */
typedef struct {
} ocs_scsi_tgt_sport_t;


/**
 * @brief target private node structure
 *
 * This structure holds information that relates to a node (I_T nexus)
 * that is owned and used by the target.
 *
 */

typedef struct {
} ocs_scsi_tgt_node_t;
/**
 * @brief target private IO structure
 */

#define MAX_SPDK_SGE 1024
typedef struct {
	uint32_t flags;
	uint8_t	 cdb[OCS_SCSI_MAX_CDB_LEN];
	uint32_t cdb_len;
	uint64_t lun;
	// For TMF
	ocs_scsi_tmf_cmd_e cmd;
	uint32_t abort_tag;
} ocs_scsi_tgt_io_t;

#endif /* __OCS_SCSI_TGT_H__ */
