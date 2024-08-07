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
 * OCS linux driver common include file
 */


#if !defined(__OCS_H__)
#define __OCS_H__

#include "version.h"

#define OCS_ENABLE_VPD_SUPPORT

#define DRV_NAME	"ocsu"
#define DRV_VERSION	STR_BE_MAJOR "." STR_BE_MINOR "." STR_BE_BUILD "." STR_BE_BRANCH

#include "ocs_os.h"
#include "ocs_driver.h"
#include "ocs_impl_spdk.h"

#define BIT_2		(1<<2)

typedef struct {
	ocs_thread_t ioctl_thr;
	dslab_dir_t *slabdir;		/*>> dma slab allocator */
	uint32_t attached;
	bool shutting_down;		/* target shutdown initiated */
} ocs_drv_t;

#include "ocs_spdk_nvmet.h"
#include "ocs_drv_fc.h"

#endif // __OCS_H__
