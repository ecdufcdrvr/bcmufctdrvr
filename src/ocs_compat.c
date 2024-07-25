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
#include "ocs_compat.h"
#include "ocs_scsi_tgt.h"
#include "ocs_gendump.h"

bool
ocs_virtfn(ocs_t *ocs)
{
	return ocs->hal.sli.virtfn;
}

#if !defined(OCS_USPACE)
void
ocs_set_kernel_io_task(ocs_os_intr_context_t *intr_context)
{
	/* Nothing to do here */
	return;
}
#endif

void
ocs_notify_link_state_change(ocs_t *ocs, uint8_t link_state)
{
	/* Nothing to do here */
	return;
}

void
ocs_notify_peer_zone_rscn(ocs_t *ocs, uint32_t fc_id)
{
	/* Nothing to do here */
	return;
}

void
ocs_device_send_fw_dump_uevent(ocs_t *ocs, uint8_t dump_level)
{
	/* Nothing to do here */
	return;
}

void
ocs_handle_userapp_lost(ocs_t *ocs)
{
#if !defined(OCS_USPACE)
	ocs_log_err(ocs, "resetting firmware\n");

	if (!test_bit(OCS_PCI_DEV_REMOVING, &ocs->ocs_pci_flags)) {
		ocs->fw_dump.recover_func = TRUE;
		ocs_hal_raise_ue(&ocs->hal, OCS_FW_FUNC_DESC_DUMP, true);
	}
#endif
}

void
ocs_handle_uapi_rdy(ocs_t *ocs)
{
#if defined(OCS_INCLUDE_RAMD)
	ocs_xport_control(ocs->xport, OCS_XPORT_PORT_ONLINE);
#endif
}

void
ocs_nvme_irq_notify(uint32_t eq_index, int cpu)
{

}

#if !defined(OCS_USPACE_SPDK) && !defined(OCS_USPACE_SPDK_UPSTREAM)
int32_t
ocs_set_soft_wwns(ocs_t *ocs)
{
	/* Nothing to do here */
	return 0;
}

int
ocs_device_add_pdev_attrs(ocs_t *ocs)
{
	/* Nothing to do here */
	return 0;
}

void
ocs_device_remove_pdev_attrs(ocs_t *ocs)
{
	/* Nothing to do here */
	return;
}

int
ocs_device_add_early_pdev_attrs(ocs_t *ocs)
{
	return 0;
}

void
ocs_device_remove_early_pdev_attrs(ocs_t *ocs)
{
	return;
}

void
ocs_destroy_device_files(void)
{
	/* Nothing to do here */
	return;
}

int
ocs_create_device_files(void)
{
	/* Nothing to do here */
	return 0;
}
#endif
