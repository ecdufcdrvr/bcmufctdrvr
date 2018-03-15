/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2018 Broadcom.  All Rights Reserved.
 *  The term "Broadcom" refers to Broadcom Limited and/or its subsidiaries.
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

/**
 * @file
 * HAL workarounds
 *
 * This file contains the declarations that allow run-time selection of workarounds
 * based on the asic type and revision, and range of fw revision.
 */

#if !defined(__OCS_HAL_WORKAROUND_H__)
#define __OCS_HAL_WORKAROUND_H__

/**
 * @brief pack fw revision values into a single uint64_t
 */

/* Two levels of macro needed due to expansion */
#define HAL_FWREV(a,b,c,d) (((uint64_t)(a) << 48) | ((uint64_t)(b) << 32) | ((uint64_t)(c) << 16) | ((uint64_t)(d)))
#define HAL_FWREV_1(x) HAL_FWREV(x)

#define OCS_FW_VER_STR2(a,b,c,d) #a "." #b "." #c "." #d
#define OCS_FW_VER_STR(x) OCS_FW_VER_STR2(x)

#define OCS_MIN_FW_VER_LANCER 10,4,255,0
#define OCS_MIN_FW_VER_SKYHAWK 10,4,255,0

typedef struct {
	uint64_t fwrev;

	/* Control Declarations here ...*/

	uint8_t retain_tsend_io_length;

	/* Use unregistered RPI */
	uint8_t use_unregistered_rpi;
	uint32_t unregistered_rid;
	uint32_t unregistered_index;

	uint8_t disable_ar_tgt_dif;	/* Disable auto response if target DIF */
	uint8_t disable_dump_loc;
	uint8_t use_dif_quarantine;
	uint8_t use_dif_sec_xri;

	uint8_t override_fcfi;

	uint8_t fw_version_too_low;

	uint8_t sglc_misreported;

	uint8_t ignore_send_frame;

} ocs_hal_workaround_t;

extern void ocs_hal_workaround_setup(struct ocs_hal_s *hal);

#endif // __OCS_HAL_WORKAROUND_H__
