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
 *
 */


#include "ocs.h"
#include "ocs_ioctl.h"
#include "ocs_ioctl_xport.h"
#include "ocs_gendump.h"

/**
 * @brief Retrieve transport port address
 *
 * Returns the transport's port address into the appropriate element
 * of the info structure argument
 *
 * @param ocs pointer device structure
 * @param info pointer to the driver info structure
 *
 * @return none
 */
void
ocs_info_get_xport_address(ocs_t *ocs, ocs_ioctl_driver_info_t *info)
{
		ocs_memcpy(info->hw_addr.fc.wwnn, ocs_hal_get_ptr(&ocs->hal, OCS_HAL_WWN_NODE),
			sizeof(info->hw_addr.fc.wwnn));
		ocs_memcpy(info->hw_addr.fc.wwpn, ocs_hal_get_ptr(&ocs->hal, OCS_HAL_WWN_PORT),
			sizeof(info->hw_addr.fc.wwpn));
}

/**
 * @brief Perform transport specific ioctl operation
 *
 * Transport specific IOCTL code is found here
 *
 * @param ocs pointer to device structure
 * @param cmd IOCTL command to execute
 * @param arg IOCTL specific command argument
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocs_device_ioctl_xport(ocs_t *ocs, unsigned int cmd, unsigned long arg)
{
	int rc = 0;

	switch(cmd) {

	case OCS_IOCTL_CMD_VPORT: {
		ocs_ioctl_vport_t *req;
		ocs_domain_t *domain;
		uint32_t port_id;

#if defined(OCS_INCLUDE_SCST)
		ocs_log_test(ocs, "Error: vport commands must be sent via SCST using the mgmt cmd in "
			     "/sys/kernel/scst_tgt/targets/ocs_scst\n");
		return -1;
#endif


		req = (ocs_ioctl_vport_t *)ocs_ioctl_preprocess(ocs, (void *)arg, sizeof(*req));
		if (req == NULL) {
			ocs_log_test(ocs, "Error: copy from user failed\n");
			return -EFAULT;
		}

		domain = ocs_domain_get_instance(ocs, req->domain_index);
		if (domain == NULL) {
			ocs_log_test(ocs, "Error: no domain found\n");
			ocs_ioctl_free(ocs, req, sizeof(*req));
			return -EINVAL;
		}

		if (req->req_create) {
			port_id = UINT32_MAX;
#if defined(ENABLE_FABRIC_EMULATION)
			if (domain->femul_enable) {
				port_id = ocs_femul_portid_alloc(domain);
			}
#endif
			rc = ocs_sport_vport_new(domain, req->wwpn, req->wwnn, port_id, req->enable_ini,
				req->enable_tgt, NULL, NULL, TRUE);
		} else {
			rc = ocs_xport_control(ocs->xport, OCS_XPORT_POST_VPORT_SHUTDOWN,
					req->wwpn, req->wwnn, req->domain_index, true);
		}

		if (ocs_ioctl_postprocess(ocs, (void*) arg, req, sizeof(*req))) {
			ocs_log_test(ocs, "Error: copy to user failed\n");
			return -EFAULT;
		}

		return rc;
	}

	case OCS_IOCTL_CMD_GEN_DUMP: {
		ocs_log_info(ocs, "Calling ocsu_gendump\n");
		rc = ocsu_gendump(ocs);
		if(rc) {
			ocs_log_test(ocs, "OCS_IOCTL_CMD_GEN_DUMP failed: %d\n", rc);
		}
		break;
	}

	case OCS_IOCTL_CMD_LINK_ONLINE: {
		rc = ocs_xport_control(ocs->xport, OCS_XPORT_PORT_ONLINE);
		if (rc) {
			ocs_log_test(ocs, "OCS_XPORT_PORT_ONLINE failed: %d\n", rc);
		}
		break;
	}

	default:
		rc = -1;
		break;
	}
	return rc;
}
