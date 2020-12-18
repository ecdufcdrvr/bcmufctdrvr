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

#include "ocs.h"
#include "ocs_os.h"
#include "ocs_ioctl.h"
#include "ocs_ioctl_xport.h"
#include "ocs_vpd.h"
#include "ocs_gendump.h"
#include "ocs_elxu.h"

#if defined(OCS_ENABLE_ECD_HELPER)
static int ocs_process_ecd_helper (ocs_t *ocs, ocs_ioctl_ecd_helper_t *req);
#endif

/**
 * @brief ocs driver ioctl method
 *
 * Called from ocs_ioctl
 *
 * @param ocs pointer to ocs structure
 * @param cmd ioctl command to execute
 * @param arg ioctl command specitic data
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocs_device_ioctl(ocs_t *ocs, unsigned int cmd, unsigned long arg)
{
	int rc = 0;

	switch(cmd) {

	case OCS_IOCTL_CMD_TEST: {
		ocs_ioctl_test_t *buf;

		buf = (ocs_ioctl_test_t *)ocs_ioctl_preprocess(ocs, (void *)arg, sizeof(*buf));
		if (buf == NULL) {
			ocs_log_test(ocs, "copy from user failed\n");
			return -EFAULT;
		}

		ocs_log_debug(ocs, "CMD_TEST: %s\n", buf->string);
		buf->string[0] ++;

		rc = ocs_ioctl_postprocess(ocs, (void *)arg, buf, sizeof(*buf));
		if (rc) {
			ocs_log_test(ocs, "Error: copy to user failed: %d\n", rc);
			return -EFAULT;
		}

		break;
	}
	case OCS_IOCTL_CMD_ELXU_MBOX: {
		ocs_ioctl_elxu_mbox_t *mcmd;

		mcmd = (ocs_ioctl_elxu_mbox_t *)ocs_ioctl_preprocess(ocs, (void *)arg, sizeof(*mcmd));
		if (mcmd == NULL) {
			ocs_log_test(ocs, "Error: copy from user failed\n");
			return -EFAULT;
		}

		rc = ocs_elxu(ocs, mcmd);

		/* copy response back */
		if (rc == 0) {
			if (ocs_ioctl_postprocess(ocs, (void *)arg, mcmd, sizeof(*mcmd))) {
				ocs_log_test(ocs, "Error: copy to user failed\n");
				return -EFAULT;
			}
		} else {
			ocs_ioctl_free(ocs, mcmd, sizeof(*mcmd));
		}

		break;

	}
	case OCS_IOCTL_CMD_DRIVER_INFO: {
		ocs_ioctl_driver_info_t *info;
		uint8_t *pserial, *pvpd;
		uint32_t len;
		uint32_t vpd_len;

		if (ocs_hal_get(&ocs->hal, OCS_HAL_VPD_LEN, &vpd_len)) {
			ocs_log_test(ocs, "Can't get VPD length\n");
			return -EFAULT;
		}

		info = (ocs_ioctl_driver_info_t *)ocs_ioctl_preprocess(ocs, (void *)arg, sizeof(*info));
		if (info == NULL) {
			ocs_log_test(ocs, "Error: copy from user failed\n");
			return -EFAULT;
		}

		info->pci_vendor = ocs->pci_vendor;
		info->pci_device = ocs->pci_device;
		ocs_strncpy(info->businfo, ocs->businfo, sizeof(info->businfo));

		ocs_strncpy(info->desc, ocs->desc, sizeof(info->desc));
		ocs_strncpy(info->fw_rev, ocs_hal_get_ptr(&ocs->hal, OCS_HAL_FW_REV), sizeof(info->fw_rev));

		ocs_info_get_xport_address(ocs, info);

		/* fetch serial number from VPD */
		pvpd = ocs_hal_get_ptr(&ocs->hal, OCS_HAL_VPD);
		if (pvpd) {
			pserial = ocs_find_vpd(pvpd, vpd_len, "SN");
			if (pserial) {
				len = *pserial ++;
				if (len > sizeof(info->serialnum)) {
					len = sizeof(info->serialnum);
				}
				ocs_memcpy(info->serialnum, pserial, len);
			}
		}

		info->sli_intf = ocs_config_read32(ocs, SLI4_INTF_REG);

		if (ocs_ioctl_postprocess(ocs, (void*) arg, info, sizeof(*info))) {
			ocs_log_test(ocs, "Error: copy to user failed\n");
			return -EFAULT;
		}

		break;
	}
	case OCS_IOCTL_CMD_MGMT_EXEC: {
		ocs_ioctl_action_t *req;
		char action_name[128];

		req = (ocs_ioctl_action_t *)ocs_ioctl_preprocess(ocs, (void *)arg, sizeof(*req));
		if (req == NULL) {
			ocs_log_test(ocs, "Error: copy from user failed\n");
			return -EFAULT;
		}

		if (ocs_copy_from_user(action_name, req->name, (uint16_t) OCS_MIN(OCS_MGMT_MAX_NAME, req->name_len + 1))) {
			ocs_log_test(ocs, "Error: copy req.name from user failed\n");
			ocs_ioctl_free(ocs, req, sizeof(*req));
			return -EFAULT;
		}

		/* Notes: handlers must do their own user space copy in's and out's */
		req->result = ocs_mgmt_exec(ocs, action_name, req->arg_in, req->arg_in_length,
				req->arg_out, req->arg_out_length);

		if (ocs_ioctl_postprocess(ocs, (void*) arg, req, sizeof(*req))) {
			ocs_log_err(ocs, "Error: ocs_copy_to_user failed\n");
			return -EFAULT;
		}

		break;
	}

	case OCS_IOCTL_CMD_MGMT_LIST: {
		ocs_ioctl_mgmt_buffer_t *req;
		ocs_textbuf_t textbuf;

		req = (ocs_ioctl_mgmt_buffer_t *)ocs_ioctl_preprocess(ocs, (void *)arg, sizeof(*req));
		if (req == NULL) {
			ocs_log_test(ocs, "Error: ocs_copy from user failed\n");
			return -EFAULT;
		}

		/* Build a text buffer */
		if (ocs_textbuf_alloc(ocs, &textbuf, req->user_buffer_len)) {
			ocs_log_err(ocs, "Error: ocs_textbuf_alloc failed\n");
			ocs_ioctl_free(ocs, req, sizeof(*req));
			return -EFAULT;
		}

		ocs_mgmt_get_list(ocs, &textbuf);

		if (ocs_textbuf_get_written(&textbuf)) {
			if (ocs_copy_to_user(req->user_buffer, ocs_textbuf_get_buffer(&textbuf),
					     ocs_textbuf_get_written(&textbuf)))
				ocs_log_test(ocs, "Error: ocs_copy_to_user failed\n");
		}
		req->bytes_written = ocs_textbuf_get_written(&textbuf);

		if (ocs_ioctl_postprocess(ocs, (void*) arg, req, sizeof(*req))) {
			ocs_log_test(ocs, "Error: ocs_copy_to_user failed\n");
			ocs_textbuf_free(ocs, &textbuf);
			return -EFAULT;
		}

		ocs_textbuf_free(ocs, &textbuf);

		break;
	}

	case OCS_IOCTL_CMD_MGMT_GET_ALL: {
		ocs_ioctl_mgmt_buffer_t *req;
		ocs_textbuf_t textbuf;
		int32_t n;
		uint32_t idx;
		uint32_t copied = 0;

		req = (ocs_ioctl_mgmt_buffer_t *)ocs_ioctl_preprocess(ocs, (void *)arg, sizeof(*req));
		if (req == NULL) {
			ocs_log_test(ocs, "Error: ocs_copy from user failed\n");
			return -EFAULT;
		}

		/* Build a text buffer */
		if (ocs_textbuf_alloc(ocs, &textbuf, req->user_buffer_len)) {
			ocs_log_err(ocs, "Error: ocs_textbuf_alloc failed\n");
			ocs_ioctl_free(ocs, req, sizeof(*req));
			return -EFAULT;
		}

		ocs_mgmt_get_all(ocs, &textbuf);

		for (idx = 0; (n = ocs_textbuf_ext_get_written(&textbuf, idx)) > 0; idx++) {
			if (ocs_copy_to_user(req->user_buffer + copied,
					ocs_textbuf_ext_get_buffer(&textbuf, idx), n)) {
				ocs_log_test(ocs, "Error: ocs_copy_to_user failed\n");
				break;
			}
			copied += n;
		}
		req->bytes_written = copied;

		if (ocs_ioctl_postprocess(ocs, (void*) arg, req, sizeof(*req))) {
			ocs_log_test(ocs, "Error: ocs_copy_to_user failed\n");
			ocs_textbuf_free(ocs, &textbuf);
			return -EFAULT;
		}

		ocs_textbuf_free(ocs, &textbuf);

		break;
	}

	case OCS_IOCTL_CMD_MGMT_GET: {
		ocs_ioctl_cmd_get_t *req;
		ocs_textbuf_t textbuf;
		char name[OCS_MGMT_MAX_NAME];

		req = (ocs_ioctl_cmd_get_t *)ocs_ioctl_preprocess(ocs, (void *)arg, sizeof(ocs_ioctl_cmd_get_t));
		if (req == NULL) {
			ocs_log_test(ocs, "copy from user failed\n");
			return -EFAULT;
		}

		/* Copy the name value in from user space */
		if (ocs_copy_from_user(name, req->name, (uint16_t) OCS_MIN(OCS_MGMT_MAX_NAME, req->name_len + 1))) {
			ocs_log_test(ocs, "ocs_copy_from_user failed\n");
			ocs_ioctl_free(ocs, req, sizeof(ocs_ioctl_cmd_get_t));
			return -EFAULT;
		}

		/* Build a text buffer */
		if (ocs_textbuf_alloc(ocs, &textbuf, req->value_length)) {
			ocs_log_err(ocs, "Error: ocs_textbuf_alloc failed\n");
			ocs_ioctl_free(ocs, req, sizeof(ocs_ioctl_cmd_get_t));
			return -EFAULT;
		}

		rc = ocs_mgmt_get(ocs, name, &textbuf);
		if (rc) {
			ocs_log_err(ocs, "ocs_mgmt_get %s failed with %d status\n", name, rc);
			req->status = rc;
			if (rc == -EBUSY)
				ocs_log_err(ocs, "Device is busy in other operations");
		} else if (ocs_textbuf_get_written(&textbuf)) {
			if (ocs_copy_to_user(req->value, ocs_textbuf_get_buffer(&textbuf),
					     ocs_textbuf_get_written(&textbuf)))
				ocs_log_test(ocs, "Error: ocs_copy_to_user failed\n");
		}

		req->value_length = ocs_textbuf_get_written(&textbuf);

		if (ocs_ioctl_postprocess(ocs, (void *)arg, req, sizeof(ocs_ioctl_cmd_get_t))) {
			ocs_log_err(ocs, "Error: ocs_copy_to_user failed\n");
			ocs_textbuf_free(ocs, &textbuf);
			return -EFAULT;
		}

		ocs_textbuf_free(ocs, &textbuf);

		break;
	}

	case OCS_IOCTL_CMD_MGMT_SET: {
		ocs_ioctl_cmd_set_t *req;
		char name[OCS_MGMT_MAX_NAME];
		char value[OCS_MGMT_MAX_VALUE];

		req = ocs_ioctl_preprocess(ocs, (void *)arg, sizeof(*req));
		if (req == NULL) {
			ocs_log_test(ocs, "Error: copy from user failed\n");
			return -EFAULT;
		}

		// Copy the name  in from user space
		if (ocs_copy_from_user(name, req->name, (uint16_t)  OCS_MIN(OCS_MGMT_MAX_NAME, (req->name_len + 1)))) {
			ocs_log_test(ocs, "Error: copy from user failed\n");
			ocs_ioctl_free(ocs, req, sizeof(*req));
			return -EFAULT;
		}

		// Copy the  value in from user space
		if (ocs_copy_from_user(value, req->value, OCS_MGMT_MAX_VALUE)) {
			ocs_log_test(ocs, "Error: copy from user failed\n");
			ocs_ioctl_free(ocs, req, sizeof(*req));
			return -EFAULT;
		}

		req->result = ocs_mgmt_set(ocs, name, value);
		if (req->result) {
			ocs_log_err(ocs, "ocs_mgmt_set failed(%d) to set mgmt cmd: %s\n", req->result, name);
			if (req->result == -EBUSY)
				ocs_log_err(ocs, "Device is busy in other operations");
		}

		if (ocs_ioctl_postprocess(ocs, (void*) arg, req, sizeof(*req))) {
			ocs_log_test(ocs, "Error: copy_to_user failed\n");
			return -EFAULT;
		}

		break;
	}

	case OCS_IOCTL_CMD_GET_DDUMP: {
		ocs_ioctl_ddump_t *req;
		ocs_textbuf_t textbuf;
		int x;

		req = (ocs_ioctl_ddump_t *)ocs_ioctl_preprocess(ocs, (void *)arg, sizeof(*req));
		if (req == NULL) {
			ocs_log_test(ocs, "Error: ocs_copy from user failed\n");
			return -EFAULT;
		}

		switch (req->args.action) {
		case OCS_IOCTL_DDUMP_GET:
		case OCS_IOCTL_DDUMP_GET_SAVED: {
			uint32_t remaining;
			uint32_t written;
			uint32_t idx;
			int32_t n;
			ocs_textbuf_t *ptbuf = NULL;
			uint32_t flags = 0;

			ocs_log_debug(ocs, "%sdriver dump, max buffer size %d\n",
				(req->args.action == OCS_IOCTL_DDUMP_GET_SAVED)  ? "saved " : "",
				req->user_buffer_len);

			if (req->args.action == OCS_IOCTL_DDUMP_GET_SAVED) {
				if (!ocs_ddump_state_set(ocs, OCS_DDUMP_RETRIEVING)) {
					ocs_ioctl_free(ocs, req, sizeof(*req));
					return -EFAULT;
				}

				if (ocs_textbuf_initialized(&ocs->ddump_saved))
					ptbuf = &ocs->ddump_saved;
			} else {
				if (ocs_textbuf_pool_alloc(ocs, &textbuf, req->user_buffer_len)) {
					ocs_log_err(ocs, "Error: ocs_textbuf_alloc failed\n");
					ocs_ioctl_free(ocs, req, sizeof(*req));
					return -EFAULT;
				}

				/* translate IOCTL ddump flags to ddump flags */
				if (req->args.flags & OCS_IOCTL_DDUMP_FLAGS_WQES) {
					flags |= OCS_DDUMP_FLAGS_WQES;
				}
				if (req->args.flags & OCS_IOCTL_DDUMP_FLAGS_CQES) {
					flags |= OCS_DDUMP_FLAGS_CQES;
				}
				if (req->args.flags & OCS_IOCTL_DDUMP_FLAGS_MQES) {
					flags |= OCS_DDUMP_FLAGS_MQES;
				}
				if (req->args.flags & OCS_IOCTL_DDUMP_FLAGS_RQES) {
					flags |= OCS_DDUMP_FLAGS_RQES;
				}
				if (req->args.flags & OCS_IOCTL_DDUMP_FLAGS_EQES) {
					flags |= OCS_DDUMP_FLAGS_EQES;
				}

				/* Try 3 times to get the dump */
				for(x=0; x<3; x++) {
					if (ocs_ddump(ocs, &textbuf, flags, req->args.q_entries) != 0) {
						ocs_textbuf_reset(&textbuf);
					} else {
						/* Success */
						x = 0;
						break;
					}
				}
				if (x != 0 ) {
					/* Retries failed */
					ocs_log_test(ocs, "ocs_ddump failed\n");
				} else {
					ptbuf = &textbuf;
				}
			}

			written = 0;
			if (ptbuf != NULL) {
				/* Process each textbuf segment */
				remaining = req->user_buffer_len;
				for (idx = 0; remaining; idx++) {
					n = ocs_textbuf_ext_get_written(ptbuf, idx);
					if (n < 0) {
						break;
					}
					if ((uint32_t)n >= remaining) {
						n = (int32_t)remaining;
					}
					if (ocs_copy_to_user(req->user_buffer + written,
						ocs_textbuf_ext_get_buffer(ptbuf, idx), n)) {
						ocs_log_test(ocs, "Error: ocs_copy_to_user failed\n");
					}
					written += n;
					remaining -= (uint32_t)n;
				}
			}
			req->bytes_written = written;
			ocs_log_debug(ocs, "%sdriver dump size %d, written %d, req buffer size %d\n",
				(req->args.action == OCS_IOCTL_DDUMP_GET_SAVED)  ? "saved " : "",
				ocs_textbuf_get_written(&ocs->ddump_saved), req->bytes_written, req->user_buffer_len);
			if (ptbuf == &textbuf) {
				ocs_textbuf_free(ocs, &textbuf);
			}

			/* Overkill for elxsdkutil to send another ioctl when it has already consumed the saved ddump */
			if (req->args.action == OCS_IOCTL_DDUMP_GET_SAVED) {
				if (ocs->ddump_pre_alloc) {
					ocs_log_debug(ocs, "clear ddump textbuf\n");
					ocs_clear_saved_ddump(ocs);
					ocs_ddump_state_set(ocs, OCS_DDUMP_ALLOCATED);
				} else {
					ocs_log_debug(ocs, "Free ddump textbuf\n");
					ocs_textbuf_free(ocs, &ocs->ddump_saved);
					ocs_ddump_state_set(ocs, OCS_DDUMP_NONE);
				}
			}
			break;
		}
		case OCS_IOCTL_DDUMP_CLR_SAVED:
			if (ocs->ddump_pre_alloc) {
				ocs_log_debug(ocs, "clear ddump textbuf\n");
				ocs_clear_saved_ddump(ocs);
				ocs_ddump_state_set(ocs, OCS_DDUMP_ALLOCATED);
			} else {
				ocs_log_debug(ocs, "Free ddump textbuf\n");
				ocs_textbuf_free(ocs, &ocs->ddump_saved);
				ocs_ddump_state_set(ocs, OCS_DDUMP_NONE);
			}
			break;
		default:
			ocs_log_err(ocs, "Error: ocs_textbuf_alloc failed\n");
			break;
		}

		if (ocs_ioctl_postprocess(ocs, (void*) arg, req, sizeof(*req))) {
			ocs_log_test(ocs, "Error: ocs_copy_to_user failed\n");
			return -EFAULT;
		}

		break;
	}



#if defined(OCS_ENABLE_ECD_HELPER)
	case OCS_IOCTL_CMD_ECD_HELPER: {
		ocs_ioctl_ecd_helper_t *req;

		req = (ocs_ioctl_ecd_helper_t *)ocs_ioctl_preprocess(ocs, (void *)arg, sizeof(*req));
		if (req == NULL) {
			ocs_log_test(ocs, "Error: copy from user failed\n");
			return -EFAULT;
		}

		req->status = ocs_process_ecd_helper(ocs, req);

		if (ocs_ioctl_postprocess(ocs, (void*) arg, req, sizeof(*req))) {
			ocs_log_test(ocs, "Error: copy to user failed\n");
			return -EFAULT;
		}

		break;
	}
#endif

#if defined(OCS_INCLUDE_WORKLOAD)
	case OCS_IOCTL_CMD_SCSI_CMD: {
		ocs_ioctl_scsi_cmd_t *cmd;

		cmd = (ocs_ioctl_scsi_cmd_t *)ocs_ioctl_preprocess(ocs, (void *)arg, sizeof(*cmd));
		if (cmd == NULL) {
			ocs_log_test(ocs, "Error: OCS_IOCTL_CMD_SCSI_CMD: copy from user space failed\n");
			return -EFAULT;
		}

		rc = ocs_workload_run(ocs, cmd);

		if (ocs_ioctl_postprocess(ocs, (void*)arg, cmd, sizeof(*cmd))) {
			ocs_log_test(ocs, "Error: OCS_IOCTL_CMD_SCSI_CMD: copy to user space failed\n");
			return -EFAULT;
		}

		break;
	}

#if defined(OCS_INCLUDE_FC)
	case OCS_IOCTL_CMD_SEND_FRAME: {
		ocs_ioctl_send_frame_t *cmd;

		/* Fetch command */
		cmd = ocs_ioctl_preprocess(ocs, (void *)arg, sizeof(*cmd));

		/* Process command */
		ocs_workload_send_frame(ocs, cmd);

		/* Free command */
		if (ocs_ioctl_postprocess(ocs, NULL, cmd, sizeof(*cmd))) {
			ocs_log_test(ocs, "Error: OCS_IOCTL_CMD_SCSI_CMD: copy to user space failed\n");
			return -EFAULT;
		}

		break;
	}
#endif
#endif

	default:
		rc = ocs_device_ioctl_xport(ocs, cmd, arg);
		break;
	}

	return rc;
}

/**
 * @brief perform requested Elx CoreDump helper function
 *
 * The Elx CoreDump facility used for BE3 diagnostics uses the OCS_IOCTL_CMD_ECD_HELPER
 * ioctl function to execute requested "help" functions
 *
 * @param ocs pointer to ocs structure
 * @param req pointer to helper function request
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

#if defined(OCS_ENABLE_ECD_HELPER)
static int
ocs_process_ecd_helper (ocs_t *ocs, ocs_ioctl_ecd_helper_t *req)
{
	int32_t rc = 0;
	uint8_t v8;
	uint16_t v16;
	uint32_t v32;
	void *pmem = NULL;

	/* Check the BAR read/write commands for valid bar */
	switch(req->cmd) {
	case OCS_ECD_HELPER_BAR_READ8:
	case OCS_ECD_HELPER_BAR_READ16:
	case OCS_ECD_HELPER_BAR_READ32:
	case OCS_ECD_HELPER_BAR_WRITE8:
	case OCS_ECD_HELPER_BAR_WRITE16:
	case OCS_ECD_HELPER_BAR_WRITE32:
		if (req->bar >= PCI_MAX_BAR) {
			ocs_log_test(ocs, "Error: bar %d out of range\n", req->bar);
			return -EFAULT;
		}
		if (ocs->ocs_os.reg[req->bar] == NULL) {
			ocs_log_test(ocs, "Error: bar %d not defined\n", req->bar);
			return -EFAULT;
		}
		pmem = ocs->ocs_os.reg[req->bar] + req->offset;
		break;
	default:
		break;
	}
	switch(req->cmd) {
	case OCS_ECD_HELPER_CFG_READ8:
		rc = pci_read_config_byte(ocs->ocs_os.pdev, req->offset, &v8);
		if (rc == 0)
			req->data = v8;
		break;
	case OCS_ECD_HELPER_CFG_READ16:
		rc = pci_read_config_word(ocs->ocs_os.pdev, req->offset, &v16);
		if (rc == 0)
			req->data = v16;
		break;
	case OCS_ECD_HELPER_CFG_READ32:
		rc = pci_read_config_dword(ocs->ocs_os.pdev, req->offset, &v32);
		if (rc == 0)
			req->data = v32;
		break;
	case OCS_ECD_HELPER_CFG_WRITE8:
		rc = pci_write_config_byte(ocs->ocs_os.pdev, req->offset, req->data);
		break;
	case OCS_ECD_HELPER_CFG_WRITE16:
		rc = pci_write_config_word(ocs->ocs_os.pdev, req->offset, req->data);
		break;
	case OCS_ECD_HELPER_CFG_WRITE32:
		rc = pci_write_config_dword(ocs->ocs_os.pdev, req->offset, req->data);
		break;
	case OCS_ECD_HELPER_BAR_READ8:
		req->data = readb(pmem);
		break;
	case OCS_ECD_HELPER_BAR_READ16:
		req->data = readw(pmem);
		break;
	case OCS_ECD_HELPER_BAR_READ32:
		req->data = readl(pmem);
		break;
	case OCS_ECD_HELPER_BAR_WRITE8:
		writeb(req->data, pmem);
		break;
	case OCS_ECD_HELPER_BAR_WRITE16:
		writew(req->data, pmem);
		break;
	case OCS_ECD_HELPER_BAR_WRITE32:
		writel(req->data, pmem);
		break;
	}
	return rc;
}
#endif
