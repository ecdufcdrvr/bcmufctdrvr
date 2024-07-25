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
 * OCS linux driver UAPI protocol definitions
 */

#if !defined(__OCS_UAPI_H__)
#define __OCS_UAPI_H__

#include "ocs_os.h"
#include "ocs_list.h"
#include "ocs_ioctl.h"

#define OCS_UAPI_TIMER_SEC 30
#define OCS_UAPI_MMAP_MAX_CNT 65535

struct ocs_uapi_mmap_tag {
	uint16_t instance;
	ocs_dma_t dma;
};

typedef void (*ocs_uapi_req_callback)(ocs_t *ocs, ocs_uapi_msg_type_t msg_type,
		void *arg, ocs_uapi_status_t status);

typedef struct ocs_ioctl_uapi_list_elem_s {
	ocs_list_link_t req_link;
	ocs_list_link_t rsp_link;
	ocs_t *ocs;
	void *args;
	ocs_uapi_req_callback cb;
	uint64_t submit_ticks;
	ocs_ioctl_get_uapi_req_t *req;
} ocs_ioctl_uapi_list_elem_t;

void ocs_uapi_init(ocs_t *ocs);
void ocs_uapi_clean(ocs_t *ocs);
void ocs_uapi_clean_reqs(ocs_t *ocs);
void ocs_uapi_app_lost(ocs_t *ocs);
void ocs_uapi_rdy_notify(ocs_t *ocs);
ocs_ioctl_get_uapi_req_t * ocs_uapi_get_req(ocs_t *ocs);

void ocs_uapi_queue_req(ocs_t *ocs, ocs_ioctl_get_uapi_req_t *req, ocs_uapi_req_callback cb, void *args);

ocs_ioctl_uapi_list_elem_t *
ocs_uapi_get_pend_req(ocs_t *ocs, ocs_ioctl_set_uapi_resp_t *rsp);

int32_t ocs_ioctl_uapi_process_rsp(ocs_t *ocs, ocs_ioctl_set_uapi_resp_t *rsp);

struct ocs_uapi_mmap_tag * ocs_uapi_mmap_tag_get(ocs_t *ocs);
void ocs_uapi_mmap_tag_put(ocs_t *ocs, struct ocs_uapi_mmap_tag *tag);
struct ocs_uapi_mmap_tag * ocs_uapi_mmap_tag_pool_instance(ocs_t *ocs, uint32_t idx);
extern char *ocs_uapi_msg_type_strs[OCS_UAPI_FC_EVENT_MAX + 1];

#endif
