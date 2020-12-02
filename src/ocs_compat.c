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
 *
 */

#include "ocs.h"
#include "ocs_scsi_tgt.h"
#include "ocs_gendump.h"

#if defined(OCS_INCLUDE_FC)
void
ocs_fc_encode_lun(ocs_node_t *node, uint64_t lun, uint8_t *lu)
{
	uint8_t abort_test_bits = (lun >> 24);

	lun &= 0xffffff;
	if (lun <= FCP_LUN_ADDR_SIMPLE_MAX) {
		lu[1] = lun & 0xff;
	} else if (lun <= FCP_LUN_ADDR_FLAT_MAX) {
		/*
		 * Use single level, flat space LUN
		 */
		lu[0] = (FCP_LUN_ADDR_METHOD_FLAT << FCP_LUN_ADDRESS_METHOD_SHIFT) |
			((lun >> 8) & FCP_LUN_ADDRESS_METHOD_MASK);
		lu[1] = lun & 0xff;
	} else {
		ocs_log_err(node->ocs, "unsupported LU %08X\n", lun);
		// TODO: need an error code that indicates LUN INVALID
		return;
	}

	lu[2] = abort_test_bits;
}

uint64_t
ocs_fc_decode_lun(ocs_node_t *node, uint8_t *lu)
{
	uint64_t lun = ULONG_MAX;
	uint8_t	 address_method = -1;

	address_method = lu[0] >> FCP_LUN_ADDRESS_METHOD_SHIFT;

	switch (address_method) {
	case FCP_LUN_ADDR_METHOD_PERIPHERAL:
	{
		uint8_t	bus_identifier = lu[0] & ~FCP_LUN_ADDRESS_METHOD_MASK;

		if (0 == bus_identifier) {
			lun = lu[1];
		} else {
			ocs_log_test(node->ocs, "unsupported bus identifier %#02x (LU %02x %02x %02x %02x ...)\n",
				     bus_identifier, lu[0], lu[1], lu[2], lu[3]);
		}
		break;
	}
	case FCP_LUN_ADDR_METHOD_FLAT:
	{
		lun = (lu[0] & ~FCP_LUN_ADDRESS_METHOD_MASK) << 8 | lu[1];
		break;
	}
	case FCP_LUN_ADDR_METHOD_EXTENDED:
	{
		uint8_t	length, extended_address_method;

		length = (lu[0] & 0x30) >> 4;
		extended_address_method = lu[0] & 0xf;

		if ((1 == length) && (2 == extended_address_method)) {
			lun = (lu[1] << 16) | (lu[2] << 8) | lu[3];
		} else {
			ocs_log_test(node->ocs, "unsupported extended addressing method (length=%#x method=%#x)\n",
				     length, extended_address_method);
		}
		break;
	}
	default:
		ocs_log_test(node->ocs, "unsupported LU address method %#02x\n", address_method);
	}

	ocs_assert(lun < UINT32_MAX, ULONG_MAX);
	return lun;
}
#endif

#if !defined(OCSU_FC_RAMD) && !defined(OCSU_FC_WORKLOAD) && !defined(OCS_USPACE_SPDK) && !defined(OCSU_ISCSI)
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
ocs_device_send_fw_dump_uevent(ocs_t *ocs, uint8_t dump_level)
{
	/* Nothing to do here */
	return;
}

#if !defined(OCS_USPACE_SPDK)
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
