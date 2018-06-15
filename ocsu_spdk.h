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

#if !defined(__OCSU_SPDK_H__)
#define __OCSU_SPDK_H__

#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdbool.h>
#include <rte_config.h>
#include <rte_malloc.h>
#include <rte_atomic.h>
#include <rte_cycles.h>

#include "spdk/env.h"


#ifdef USE_PCIACCESS
#include <pciaccess.h>
#else
#include <rte_pci.h>
#endif

typedef struct ocs_uspace {
        uint32_t instance_index;
        struct spdk_pci_device *pdev;
        ocsu_memref_t bars[PCI_MAX_BAR];
        uint32_t bar_count;

        const char *desc;
        uint16_t pci_vendor;
        uint16_t pci_device;
        char businfo[16];
        uint32_t n_msix_vec;
        int intr_enabled;
        struct device *ocs_dev;

        uint32_t dmabuf_next_instance;
        uint8_t  if_type;
        ocsu_memref_t bmbx;
} ocs_uspace_t;

#define ocsu_free(buf)                  rte_free(buf)
#endif
