/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2015, Broadcom.  All Rights Reserved.
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
 * API for OCS power managment
 *
 */

#if !defined(__OCS_PM_H__)
#define __OCS_PM_H__
/**
 * @brief Power State change message types
 *
 */
typedef enum {
	OCS_PM_PREPARE = 1,
	OCS_PM_SLEEP,
	OCS_PM_HIBERNATE,
	OCS_PM_RESUME,
} ocs_pm_msg_e;

/**
 * @brief Power State values
 *
 */
typedef enum {
	OCS_PM_STATE_S0 = 0,
	OCS_PM_STATE_S1,
	OCS_PM_STATE_S2,
	OCS_PM_STATE_S3,
	OCS_PM_STATE_S4,
} ocs_pm_state_e;

typedef struct {
	ocs_pm_state_e pm_state;		/*<< Current PM state */
} ocs_pm_context_t;

extern int32_t ocs_pm_request(ocs_t *ocs, ocs_pm_msg_e msg, int32_t (*callback)(ocs_t *ocs, int32_t status, void *arg),
	void *arg);
extern ocs_pm_state_e ocs_pm_get_state(ocs_t *ocs);
extern const char *ocs_pm_get_state_string(ocs_t *ocs);
#endif // __OCS_PM_H__
