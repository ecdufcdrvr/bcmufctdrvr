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

#include "ocs.h"
#include "ocs_os.h"
#include "ocs_scsi.h"
#include "scsi_cmds.h"
#include "fc.h"

int32_t ocs_nvme_new_target(ocs_node_t *node);
int32_t ocs_nvme_del_target(ocs_node_t *node);
int32_t ocs_nvme_process_flush_bls(ocs_node_t *node, uint16_t oxid, uint16_t rxid,
			uint32_t sler_qual, bool ht, uint16_t flush_count);

int32_t
ocs_scsi_validate_initiator(ocs_node_t *node)
{
	if (FC_ADDR_IS_DOMAIN_CTRL(node->rnode.fc_id))
		return 1;
	else
		return 0;
}

int32_t
ocs_scsi_new_initiator(ocs_node_t *node)
{
	return 0;
}

int32_t
ocs_scsi_del_initiator(ocs_node_t *node, ocs_scsi_del_initiator_reason_e reason)
{
	return 0;
}

int32_t
ocs_scsi_recv_cmd(ocs_io_t *io, uint64_t lun, uint8_t *cdb, uint32_t cdb_len, uint32_t flags)
{
	return 0;
}

int32_t
ocs_scsi_recv_cmd_first_burst(ocs_io_t *io, uint64_t lun, uint8_t *cdb, uint32_t cdb_len,
			uint32_t flags, ocs_dma_t first_burst_buffers[],
			uint32_t first_burst_buffer_count)
{
	return 0;
}

int32_t
ocs_scsi_recv_tmf(ocs_io_t *tmfio, uint64_t lun, ocs_scsi_tmf_cmd_e cmd, ocs_io_t *abortio,
	uint32_t flags)
{
	return 0;
}

void
ocs_scsi_tgt_ddump(ocs_textbuf_t *textbuf, ocs_scsi_ddump_type_e type, void *obj)
{
}

int32_t
ocs_scsi_tgt_new_domain(ocs_domain_t *domain)
{
	return 0;
}

int32_t
ocs_scsi_tgt_del_domain(ocs_domain_t *domain)
{
	return 0;
}
int32_t
ocs_scsi_tgt_new_device(ocs_t *ocs)
{
	return 0;
}

int32_t
ocs_scsi_tgt_del_device(ocs_t *ocs)
{
	return 0;
}

int32_t
ocs_scsi_tgt_new_sport(ocs_sport_t *sport)
{
	return 0;
}

void
ocs_scsi_tgt_del_sport(ocs_sport_t *sport)
{
	/* No SCSI SPDK call to delete port. */
}	

void
ocs_vport_logout_notify(ocs_node_t *node)
{
}

int32_t
ocs_nvme_new_target(ocs_node_t *node)
{
	return 0;
}

int32_t
ocs_nvme_del_target(ocs_node_t *node)
{
	return 0;
}

int32_t
ocs_nvme_process_flush_bls(ocs_node_t *node, uint16_t oxid, uint16_t rxid,
			uint32_t sler_qual, bool ht, uint16_t flush_count)
{
	return 0;
}
