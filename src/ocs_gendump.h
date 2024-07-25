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

#if !defined(__OCS_GENDUMP_H__)
#define __OCS_GENDUMP_H__

/* FW dump level flags */
#define OCS_FW_DUMP_LEVEL_NONE			0
#define OCS_FW_CHIP_LEVEL_DUMP			1
#define OCS_FW_FUNC_DESC_DUMP			2

/* FW dump state flags; should maintain 1:1 mapping with FW dump level flags */
#define OCS_FW_DUMP_STATE_NONE			OCS_FW_DUMP_LEVEL_NONE
#define OCS_FW_CHIP_DUMP_STATE_VALID		OCS_FW_CHIP_LEVEL_DUMP
#define OCS_FW_FUNC_DUMP_STATE_VALID		OCS_FW_FUNC_DESC_DUMP

/* OCS reset level flags; should maintain 1:1 mapping with FW dump level flags */
#define OCS_RESET_LEVEL_NONE			OCS_FW_DUMP_LEVEL_NONE
#define OCS_RESET_LEVEL_CHIP			OCS_FW_CHIP_LEVEL_DUMP
#define OCS_RESET_LEVEL_PORT			OCS_FW_FUNC_DESC_DUMP

/* FW dump type flags */
#define OCS_FW_DUMP_TYPE_NONE			0
#define OCS_FW_DUMP_TYPE_DUMPTOHOST		1
#define OCS_FW_DUMP_TYPE_FLASH			2

struct ocs_fw_dump {
	uint8_t *saved_buff;
	uint32_t size;
	int32_t type;
	bool	recover_func; /* locking: recovery lock */

	uint32_t num_chip_dump_buffers;
	uint32_t num_func_dump_buffers;
	ocs_dma_t *chip_dump_buffers;
	ocs_dma_t *func_dump_buffers;

	/* lock is used around state */
	ocs_lock_t lock;
	uint32_t state;
};

extern bool ocs_fw_dump_type_validate(ocs_t *ocs);
extern int32_t ocs_trigger_reset_dump(ocs_t *ocs, uint8_t dump_level, bool trigger_dump);
extern int32_t ocs_fw_dump_state_check(ocs_t *ocs, uint32_t state);
extern void ocs_fw_dump_state_set(ocs_t *ocs, uint32_t state);
extern int32_t ocs_gendump(ocs_t *ocs, uint8_t dump_level, bool force);
extern bool ocs_fw_dump_buffers_allocated(ocs_t *ocs, uint8_t dump_level);
extern int32_t ocs_fw_dump_copy_to_user(ocs_t *ocs, uint8_t *user_buff, uint32_t user_buflen, uint8_t dump_state);
extern int32_t ocs_dump_to_host(ocs_t *ocs, void *buf, uint32_t buflen, uint8_t dump_level);
extern void ocs_device_send_fw_dump_uevent(ocs_t *ocs, uint8_t dump_level);
extern int32_t ocs_device_trigger_fw_dump(ocs_t *ocs, uint8_t dump_level);
extern void ocs_device_trigger_fw_error(ocs_t *ocs, uint8_t dump_level, bool trigger_dump);
extern int32_t ocs_fw_dump_save(ocs_t *ocs, uint8_t dump_level);
extern int32_t ocs_device_request_reset_dump(ocs_t *ocs, uint8_t dump_level, bool trigger_dump);
extern int32_t ocs_fw_reset(ocs_t *ocs, bool trigger_dump);
extern int32_t ocsu_gendump(ocs_t *ocs);

#endif // __OCS_GENDUMP_H__
