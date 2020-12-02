/*
 * Copyright (C) 2020 Broadcom. All Rights Reserved.
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
 * Elxutility interface code
 */


#include "ocs.h"
#include "ocs_ioctl.h"
#include "ocs_elxu.h"

static int32_t __ocs_ioctl_mbox_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg);

int32_t
ocs_process_sli_config(ocs_t *ocs, ocs_ioctl_elxu_mbox_t *mcmd, ocs_dma_t *dma)
{
	sli4_cmd_sli_config_t *sli_config = (sli4_cmd_sli_config_t *)mcmd->payload;

	if (sli_config->emb) {
		sli4_req_hdr_t *req = (sli4_req_hdr_t *)sli_config->payload.embed;

		switch (req->opcode) {
		case SLI4_OPC_COMMON_READ_OBJECT:
			if (mcmd->out_bytes) {
				sli4_req_common_read_object_t *rdobj =
				  	(sli4_req_common_read_object_t *)sli_config->payload.embed;

				if (ocs_dma_alloc(ocs, dma, mcmd->out_bytes, 4096)) {
					ocs_log_err(ocs, "COMMON_READ_OBJECT - %" PRIX64 " allocation failed\n",
						    mcmd->out_bytes);
					return ENXIO;
				}

				ocs_memset(dma->virt, 0, mcmd->out_bytes);

				rdobj->host_buffer_descriptor[0].bde_type = SLI4_BDE_TYPE_BDE_64;
				rdobj->host_buffer_descriptor[0].buffer_length = mcmd->out_bytes;
				rdobj->host_buffer_descriptor[0].u.data.buffer_address_low = ocs_addr32_lo(dma->phys);
				rdobj->host_buffer_descriptor[0].u.data.buffer_address_high = ocs_addr32_hi(dma->phys);

				if (ocs->fw_dump.type == OCS_FW_DUMP_TYPE_FLASH) {
					if (0 == ocs_strncmp(rdobj->object_name, READ_OBJECT_NAME_DUMP,
							     sizeof(rdobj->object_name))) {
						/*
						 * Reset FW dump flags here when the first readobject for dump
						 * was requested.
						 */
						ocs_fw_dump_state_set(ocs, OCS_FW_DUMP_STATE_NONE);
					}
				}
			}
			break;
		case SLI4_OPC_COMMON_WRITE_OBJECT:
		{
			sli4_req_common_write_object_t *wrobj =
			  	(sli4_req_common_write_object_t *)sli_config->payload.embed;

			if (ocs_dma_alloc(ocs, dma, wrobj->desired_write_length, 4096)) {
				ocs_log_err(ocs, "COMMON_WRITE_OBJECT - %d allocation failed\n",
					    wrobj->desired_write_length);
				return ENXIO;
			}
			// setup the descriptor
			wrobj->host_buffer_descriptor[0].bde_type = SLI4_BDE_TYPE_BDE_64;
			wrobj->host_buffer_descriptor[0].buffer_length = wrobj->desired_write_length;
			wrobj->host_buffer_descriptor[0].u.data.buffer_address_low = ocs_addr32_lo(dma->phys);
			wrobj->host_buffer_descriptor[0].u.data.buffer_address_high = ocs_addr32_hi(dma->phys);

			// copy the data into the DMA buffer
			if (ocs_copy_from_user(dma->virt, (void *)mcmd->in_addr, mcmd->in_bytes)) {
				ocs_log_warn(ocs, "copy_from_user() failed\n");
				return EFAULT;
			}

		}
			break;
		case SLI4_OPC_COMMON_DELETE_OBJECT:
			break;
		case SLI4_OPC_COMMON_READ_OBJECT_LIST:
			if (mcmd->out_bytes) {
				sli4_req_common_read_object_list_t *rdobj =
					(sli4_req_common_read_object_list_t *)sli_config->payload.embed;
				if (ocs_dma_alloc(ocs, dma, mcmd->out_bytes, 4096)) {
					ocs_log_err(ocs, "COMMON_READ_OBJECT_LIST - %" PRIX64 " allocation failed\n",
						    mcmd->out_bytes);
					return ENXIO;
				}

				ocs_memset(dma->virt, 0, mcmd->out_bytes);

				rdobj->host_buffer_descriptor[0].bde_type = SLI4_BDE_TYPE_BDE_64;
				rdobj->host_buffer_descriptor[0].buffer_length = mcmd->out_bytes;
				rdobj->host_buffer_descriptor[0].u.data.buffer_address_low = ocs_addr32_lo(dma->phys);
				rdobj->host_buffer_descriptor[0].u.data.buffer_address_high = ocs_addr32_hi(dma->phys);

			}
			break;
		case SLI4_OPC_COMMON_READ_TRANSCEIVER_DATA:
			break;
		case SLI4_OPC_FCOE_SET_LINK_DIAG_STATE:
		case SLI4_OPC_FCOE_SET_LINK_DIAG_LOOPBACK:
		case SLI4_OPC_FCOE_SET_DPORT_MODE:
		case SLI4_OPC_FCOE_GET_DPORT_RESULTS:
			break;

		default:
			ocs_log_info(ocs, "in=%p (%" PRIX64 ") out=%p (%" PRIX64 ")\n",
					(void *)mcmd->in_addr, mcmd->in_bytes,
					(void *)mcmd->out_addr, mcmd->out_bytes);
			ocs_log_info(ocs, "unknown (opc=%#x)\n", req->opcode);
			break;
		}
	} else {
		uint32_t max_bytes = max(mcmd->in_bytes, mcmd->out_bytes);
		if (ocs_dma_alloc(ocs, dma, max_bytes, 4096)) {
			ocs_log_err(ocs, "non-embedded - %u allocation failed\n", max_bytes);
			return ENXIO;
		}

		if (ocs_copy_from_user(dma->virt, (void *)mcmd->in_addr, mcmd->in_bytes)) {
			ocs_log_warn(ocs, "copy_from_user failed\n");
			return ENXIO;
		}

		sli_config->payload.mem.address_low  = ocs_addr32_lo(dma->phys);
		sli_config->payload.mem.address_high = ocs_addr32_hi(dma->phys);
		sli_config->payload.mem.length = max_bytes;
	}

	return 0;
}

int32_t
ocs_elxu(ocs_t *ocs, ocs_ioctl_elxu_mbox_t *mcmd)
{
	int32_t rc = 0;
	ocs_dma_t dma = { 0 };
	ocs_sem_t waitsem;

	// sanity checks
	if ((ELXU_BSD_MAGIC != mcmd->magic) ||
			(sizeof(*mcmd) != mcmd->size)) {
		ocs_log_debug(ocs, "malformed command m=%08x s=%08x\n", mcmd->magic, mcmd->size);
		return EINVAL;
	}

	switch(((sli4_mbox_command_header_t *)mcmd->payload)->command)
	{
	case SLI4_MBOX_COMMAND_SLI_CONFIG:
		if (ENXIO == ocs_process_sli_config(ocs, mcmd, &dma))
			return ENXIO;
		break;

	case SLI4_MBOX_COMMAND_READ_REV:
	case SLI4_MBOX_COMMAND_READ_STATUS:
	case SLI4_MBOX_COMMAND_READ_LNK_STAT:
	case SLI4_MBOX_COMMAND_DUMP:
	case SLI4_OPC_COMMON_RUN_BIU_DIAG:
		break;

	default:
		ocs_log_debug(NULL, "command %d not supported\n",
			((sli4_mbox_command_header_t *)mcmd->payload)->command);
		goto no_support;
		break;

	} // end for switch( ... -> command )

	ocs_sem_init(&waitsem, 0, "elxu_mbox");

	rc = ocs_hal_command(&ocs->hal, mcmd->payload, OCS_CMD_NOWAIT, __ocs_ioctl_mbox_cb, &waitsem);
	if (rc)
		goto dma_free;

	if (ocs_sem_p(&waitsem, OCS_SEM_FOREVER) == 0) {
		if(SLI4_MBOX_COMMAND_SLI_CONFIG == ((sli4_mbox_command_header_t *)mcmd->payload)->command &&
			mcmd->out_bytes && dma.virt) {
			if (ocs_copy_to_user((void *)mcmd->out_addr, dma.virt, mcmd->out_bytes)) {
				ocs_log_warn(ocs, "copy_to_user failed\n");
				rc = -ENXIO;
			}
		}
	} else {
		ocs_log_warn(ocs, "hal command was interrupted\n");
		rc = -ENXIO;
	}

dma_free:
	ocs_dma_free(ocs, &dma);

no_support:
	return rc;
}

static int32_t
__ocs_ioctl_mbox_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	ocs_sem_t *sem = arg;

	ocs_sem_v(sem);

	return 0;

}
