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

#if !defined(__OCS_RECOVERY_H__)
#define __OCS_RECOVERY_H__

#define OCS_RECOVERY_MODE_PORT_MIGRATION	0
#define OCS_RECOVERY_MODE_MANUAL		1
#define OCS_RECOVERY_MODE_AUTO			2

/* FW error recovery states */
#define OCS_RECOVERY_STATE_IDLE		0x0
#define OCS_RECOVERY_STATE_IN_PROGRESS	0x1

/* FW error recovery status codes */
#define OCS_RECOVERY_STATUS_SUCCESS	0
#define OCS_RECOVERY_STATUS_IN_PROGRESS 1
#define OCS_RECOVERY_STATUS_FAILED	-1

typedef struct ocs_fw_error_recovery_s {
	ocs_thread_t thread;

	ocs_lock_t lock;
	uint32_t state;
} ocs_fw_error_recovery_t;

extern bool ocs_recovery_state_check(uint32_t state);
extern bool ocs_recovery_state_set(uint32_t state);
extern int32_t ocs_recovery_reset(ocs_t *ocs, uint32_t reset_mode, uint8_t reset_level, bool reset_fw);
extern int32_t ocs_fw_err_recovery_setup(void);
extern void ocs_fw_err_recovery_stop(void);

#endif
