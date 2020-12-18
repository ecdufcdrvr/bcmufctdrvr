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
 * Common (transport agnostic) shared declarations
 *
 */
#if !defined(__OCS_COMMON_SHARED_H__)
#define __OCS_COMMON_SHARED_H__

#define OCS_CTRLMASK_XPORT_DISABLE_AUTORSP_TSEND	(1U << 0)
#define OCS_CTRLMASK_XPORT_DISABLE_AUTORSP_TRECEIVE	(1U << 1)
#define OCS_CTRLMASK_XPORT_ENABLE_TARGET_RSCN		(1U << 3)
#define OCS_CTRLMASK_TGT_ALWAYS_VERIFY_DIF		(1U << 4)
#define OCS_CTRLMASK_TGT_SET_DIF_REF_TAG_CRC		(1U << 5)
#define OCS_CTRLMASK_TEST_CHAINED_SGLS			(1U << 6)
#define OCS_CTRLMASK_ISCSI_ISNS_ENABLE			(1U << 7)
#define OCS_CTRLMASK_ENABLE_FABRIC_EMULATION		(1U << 8)
#define OCS_CTRLMASK_INHIBIT_INITIATOR			(1U << 9)
#define OCS_CTRLMASK_CRASH_RESET			(1U << 10)

#define enable_target_rscn(ocs)	((ocs->ctrlmask & OCS_CTRLMASK_XPORT_ENABLE_TARGET_RSCN) != 0)

/* Used for error injection testing. */
typedef enum {
	NO_ERR_INJECT = 0,
	INJECT_DROP_CMD,
	INJECT_FREE_DROPPED,
	INJECT_DROP_DATA,
	INJECT_DROP_RESP,
	INJECT_DELAY_CMD,
} ocs_err_injection_e;

#define MAX_OCS_DEVICES                 64

#endif // __OCS_COMMON_SHARED_H__
