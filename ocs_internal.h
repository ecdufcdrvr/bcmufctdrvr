/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2018 Broadcom.  All Rights Reserved.
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

#ifndef __OCS_INTERNAL_H__
#define __OCS_INTERNAL_H__

#include "ocs_spdk.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "spdk/queue.h"
#include "spdk/mmio.h"

#define MAX_OCS_DEVICES	64
#define SLI4_BMBX_SIZE	256

/* One of these per allocated PCI device. */
struct spdk_ocs_t {
	uint32_t instance_index;
	void	*pdev;
	spdk_ocs_memref_t bars[PCI_MAX_BAR];
	uint32_t bar_count;
	const char *desc;
	uint16_t pci_vendor;
	uint16_t pci_device;
	char businfo[16];
	uint8_t if_type;
	spdk_ocs_memref_t bmbx;

	/* tailq entry for attached_chans */
	TAILQ_ENTRY(spdk_ocs_t)	tailq;
};

extern struct spdk_ocs_t *spdk_ocs_devices[MAX_OCS_DEVICES];
#endif /* __OCS_INTERNAL_H__ */
