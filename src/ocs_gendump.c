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

#include "ocs.h"
#include "ocs_gendump.h"
#include "ocs_recovery.h"

#define OCS_MAX_HBA_PORTS			4
#define OCS_XPORT_QUIESCED_TIMEOUT_MSEC		60000

ocs_lock_t ocs_dump_lock;

/**
 * @brief Initialize fw dump context
 */
void
ocs_fw_dump_init(ocs_t *ocs, int32_t fw_dump_type)
{
	if (ocs_instance(ocs) == 0) {
		ocs_lock_init(NULL, &ocs_dump_lock, "dump_access_lock");
	}

	/* Init fw dump lock */
	ocs_lock_init(ocs, &ocs->fw_dump.lock, "fw_dump_lock[%d]", ocs_instance(ocs));
	ocs->fw_dump.state = OCS_FW_DUMP_STATE_NONE;
	ocs->fw_dump.type = fw_dump_type;
	ocs->fw_dump.chip_dump_buffers = NULL;
	ocs->fw_dump.func_dump_buffers = NULL;
	ocs->fw_dump.num_chip_dump_buffers = 0;
	ocs->fw_dump.num_func_dump_buffers = 0;
}

static uint32_t
ocs_fw_dump_present(ocs_t *ocs, uint32_t *ms_waited)
{
	uint32_t dump_present = OCS_FW_DUMP_READY_STATUS_NOT_READY;

	/* Wait for the dump to complete */
	for (*ms_waited = 0; *ms_waited < SLI4_FW_DUMP_TIMEOUT_MSEC; *ms_waited += 100) {
		/*
		 * First, wait for the status register to reflect steady state
		 * values as a result of writing to control/physdev registers.
		 */
		ocs_delay_msec(100);
		ocs_hal_get(&ocs->hal, OCS_HAL_DUMP_READY, &dump_present);
		if (dump_present)
			break;
	}

	return dump_present;
}

int32_t
ocs_trigger_dump(ocs_t *ocs, uint8_t dump_level)
{
	uint32_t ms_waited = 0;
	uint32_t dump_status = OCS_FW_DUMP_READY_STATUS_NOT_READY;

	/* If the chip is in an error state (UE'd), wait for the already triggered dump to complete */
	if (ocs_hal_reset_pending(&ocs->hal)) {
		ocs_log_info(ocs, "Waiting for the recovery dump to complete\n");
		dump_status = ocs_fw_dump_present(ocs, &ms_waited);
	}

	if (OCS_FW_DUMP_READY_STATUS_NOT_READY == dump_status) {
		ocs_log_info(ocs, "Initiating the FW dump forcefully\n");

		if (OCS_HAL_RTN_SUCCESS != ocs_hal_raise_ue(&ocs->hal, dump_level)) {
			ocs_log_err(ocs, "Failed to trigger the FW dump\n");
			return OCS_FW_DUMP_STATUS_FAILED;
		}

		ocs_log_info(ocs, "Dump requested, waiting for it to complete\n");
		dump_status = ocs_fw_dump_present(ocs, &ms_waited);
	}

	if (OCS_FW_DUMP_READY_STATUS_NOT_READY == dump_status) {
		ocs_log_err(ocs, "Failed to see the FW dump (%d) after %d msec\n", dump_status, ms_waited);
		return OCS_FW_DUMP_STATUS_FAILED;
	}

	if (OCS_FW_DUMP_READY_STATUS_SKIP_DUMP == dump_status) {
		ocs_log_info(ocs, "Skipped the FW dump\n");
		return OCS_FW_DUMP_STATUS_SKIP;
	}

	if (ms_waited) {
		ocs_log_info(ocs, "FW dump was generated successfully in %d msec\n", ms_waited);
	} else {
		ocs_log_info(ocs, "FW dump was already present\n");
	}

	return OCS_FW_DUMP_STATUS_SUCCESS;
}

/**
 * @brief Check the fw dump state
 *
 * @return TRUE if the state is set, FALSE otherwise
 */
int32_t
ocs_fw_dump_state_check(ocs_t *ocs, uint32_t state)
{
	ocs_t *other_ocs;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	int32_t rc = FALSE;
	uint32_t index = 0;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	while ((other_ocs = ocs_get_instance(index++)) != NULL) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			/* For CLD, check the dump state only on pci func 0 */
			if ((OCS_FW_CHIP_DUMP_STATE_VALID == state) && (0 != other_func))
				continue;

			/* For FDD, check the dump state only on the specified pci func */
			if ((OCS_FW_FUNC_DUMP_STATE_VALID == state) && (func != other_func))
				continue;

			/* Return the existing dump irrespective of CLD / FDD request */
			ocs_lock(&other_ocs->fw_dump.lock);
				if (other_ocs->fw_dump.state) {
					rc = TRUE;
					ocs_unlock(&other_ocs->fw_dump.lock);
					break;
				}
			ocs_unlock(&other_ocs->fw_dump.lock);
		}
	}

	return rc;
}

/**
 * @brief Set fW dump state
 *
 * @return void
 */
void
ocs_fw_dump_state_set(ocs_t *ocs, uint32_t state)
{
	ocs_t *other_ocs;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	uint32_t index = 0;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	while ((other_ocs = ocs_get_instance(index++)) != NULL) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			/* For CLD, set the dump state only on pci func 0 */
			if ((OCS_FW_CHIP_DUMP_STATE_VALID == state) && (0 != other_func))
				continue;

			/* For FDD, set the dump state only on the specified pci func */
			if ((OCS_FW_FUNC_DUMP_STATE_VALID == state) && (func != other_func))
				continue;

			ocs_lock(&other_ocs->fw_dump.lock);
				other_ocs->fw_dump.state = state;
			ocs_unlock(&other_ocs->fw_dump.lock);
		}
	}
}

static int32_t
ocs_initiate_gendump(ocs_t *ocs, uint8_t dump_level)
{
	int32_t rc = OCS_FW_DUMP_STATUS_SUCCESS;

	rc = ocs_recovery_reset(ocs, OCS_RECOVERY_MODE_MANUAL, dump_level, TRUE);
	if (rc) {
		if (OCS_RECOVERY_STATUS_IN_PROGRESS == rc) {
			ocs_log_info(ocs, "FW dump is already in progress\n");
			rc = OCS_FW_DUMP_STATUS_IN_PROGRESS;
		} else {
			ocs_log_err(ocs, "FW dump failed\n");
			rc = OCS_FW_DUMP_STATUS_FAILED;
		}
	} else {
		ocs_log_info(ocs, "FW dump is generated\n");
		rc = OCS_FW_DUMP_STATUS_SUCCESS;
	}

	return rc;
}

int32_t
ocs_gendump(ocs_t *ocs, uint8_t dump_level, bool force)
{
	int32_t rc = OCS_FW_DUMP_STATUS_FAILED;
	uint32_t dump_present;

	if (OCS_FW_DUMP_TYPE_FLASH != ocs->fw_dump.type) {
		ocs_log_err(ocs, "Flash gendump is disabled\n");
		return OCS_FW_DUMP_STATUS_DUMP_TYPE_INVALID;
	}

	/**
	 * If FW dump exists already due to recovery processs,
	 * notify user. Else, do gendump and reset the adapter.
	 *
	 * And, skip this check when requested to force the dump.
	 */
	ocs_hal_get(&ocs->hal, OCS_HAL_DUMP_PRESENT, &dump_present);
	if (!force && dump_present) {
		ocs_log_info(ocs, "FW dump is already present\n");
		return OCS_FW_DUMP_STATUS_ALREADY_PRESENT;
	}

	rc = ocs_initiate_gendump(ocs, dump_level);
	if (rc < 0)
		ocs_log_err(ocs, "ocs_initiate_gendump() failed\n");

	return rc;
}

bool
ocs_fw_dump_buffers_allocated(ocs_t *ocs, uint8_t dump_level)
{
	ocs_t *other_ocs;
	ocs_dma_t *dump_buffers = NULL;
	bool rc = TRUE;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	uint32_t index = 0;
	uint32_t num_buffers = 0;

	if (OCS_FW_FUNC_DESC_DUMP == dump_level) {
		dump_buffers = ocs->fw_dump.func_dump_buffers;
		num_buffers = ocs->fw_dump.num_func_dump_buffers;
	} else {
		ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
		while ((other_ocs = ocs_get_instance(index++)) != NULL) {
			ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
			if ((bus == other_bus) && (dev == other_dev) && (0 == other_func)) {
				dump_buffers = other_ocs->fw_dump.chip_dump_buffers;
				num_buffers = other_ocs->fw_dump.num_chip_dump_buffers;
				break;
			}
		}
	}

	if (!dump_buffers || !num_buffers)
		rc = FALSE;

	return rc;
}

/**
 * @brief Waits for FW_DUMP_PRESENT flag to be set in fw_dump.flags context,
 *	  and if recovery is started, waits for recovery completion.
 *
 * @return returns 0 for success, a negative error code value for failure
 */
static int32_t
ocs_fw_dump_ready_wait(ocs_t *ocs, uint8_t dump_state)
{
	int32_t rc = 0;
	int32_t ms_waited = 0;
	int32_t wait_time = SLI4_FW_DUMP_TIMEOUT_MSEC;

	ocs_log_info(ocs, "Waiting for FW dump to be completed\n");
	while (!ocs_fw_dump_state_check(ocs, dump_state) && (ms_waited < wait_time)) {
		ocs_delay_usec(10000);
		ms_waited += 10;
	}

	if (!ocs_fw_dump_state_check(ocs, dump_state)) {
		ocs_log_err(ocs, "FW dump timedout=%d(msec)\n", ms_waited);
		rc = -1;
		goto exit_err;
	}

	ms_waited = 0;
	wait_time = SLI4_FW_READY_TIMEOUT_MSEC;
	if (ocs_recovery_state_check(OCS_RECOVERY_STATE_IN_PROGRESS)) {
		/* Wait for the recovery process to complete */
		while (!ocs_recovery_state_check(OCS_RECOVERY_STATE_IDLE) && (ms_waited < wait_time)) {
			ocs_delay_usec(10000);
			ms_waited += 10;
		}

		if (!ocs_recovery_state_check(OCS_RECOVERY_STATE_IDLE)) {
			ocs_log_err(ocs, "Recovery timedout=%d(msec)\n", ms_waited);
			rc = -1;
			goto exit_err;
		}
	}

exit_err:
	return rc;
}

/**
 * @brief Copy dump buffers to user space buffer
 */
int32_t
ocs_fw_dump_copy_to_user(ocs_t *ocs, uint8_t *user_buff, uint32_t user_buflen, uint8_t dump_state)
{
	uint8_t *saved_buff = ocs->fw_dump.saved_buff;
	uint32_t size = ocs->fw_dump.size;

	if (!ocs_fw_dump_state_check(ocs, dump_state)) {
		ocs_log_warn(ocs, "FW dump is not present in the OCS buffers\n");
		return -1;
	}

	/* Copy the dump from the saved buffer to the user buffer */
	ocs_assert(user_buflen >= size, -1);
	if (ocs_copy_to_user(user_buff, saved_buff, size)) {
		ocs_log_err(ocs, "ocs_copy_to_user() failed\n");
		return -1;
	}

	return 0;
}

/**
 * @brief Create a Lancer dump into a memory buffer
 * @par Description
 * This function creates a DMA buffer to hold a Lancer dump,
 * sets the dump location to point to that buffer, then calls
 * ocs_gen_dump to cause a dump to be transfered to the buffer.
 * After the dump is complete it copies the dump to the provided
 * user space buffer.
 *
 * @param ocs Pointer to ocs structure
 * @param buf User space buffer in which to store the dump
 * @param buflen Length of the user buffer in bytes
 * @param dump_level Gendump request level can be either chip-level or fdb
 *
 * @return Returns 0 on success, non-zero on error.
 */
int32_t
ocs_dump_to_host(ocs_t *ocs, void *buf, uint32_t buflen, uint8_t dump_level)
{
	int32_t rc = 0;

	if (OCS_FW_DUMP_TYPE_DUMPTOHOST != ocs->fw_dump.type) {
		ocs_log_err(ocs, "Dump to host is disabled\n");
		return OCS_FW_DUMP_STATUS_DUMP_TYPE_INVALID;
	}

	if (!buflen) {
		ocs_log_err(ocs, "Zero buffer length is invalid\n");
		return -1;
	}

	if (!ocs_fw_dump_buffers_allocated(ocs, dump_level)) {
		ocs_log_err(ocs, "No pre-allocated FW dump memory\n");
		return -1;
	}

	ocs_lock(&ocs_dump_lock);

	/**
	 * If FW dump exists already due to recovery processs,
	 * wait for it to complete, and then return the dump.
	 */
	if (ocs_fw_dump_state_check(ocs, dump_level)) {
		ocs_log_info(ocs, "FW dump is already present\n");
		rc = OCS_FW_DUMP_STATUS_ALREADY_PRESENT;
		goto check_ready;
	}

	rc = ocs_initiate_gendump(ocs, dump_level);
	if (OCS_FW_DUMP_STATUS_FAILED == rc) {
		ocs_log_err(ocs, "ocs_initiate_gendump() failed\n");
		goto done;
	}

check_ready:
	if ((OCS_FW_DUMP_STATUS_IN_PROGRESS == rc) || (OCS_FW_DUMP_STATUS_ALREADY_PRESENT == rc)) {
		ocs_log_debug(ocs, "Waiting for the already triggered FW dump to complete\n");
		rc = ocs_fw_dump_ready_wait(ocs, dump_level);
	}

	if (OCS_FW_DUMP_STATUS_SUCCESS == rc) {
		/* Copy the dump from the DMA buffer into the user buffer */
		rc = ocs_fw_dump_copy_to_user(ocs, (uint8_t *)buf, buflen, dump_level);
		if (rc) {
			ocs_log_err(ocs, "FW dump copy to user failed\n");
		} else {
			ocs_log_info(ocs, "FW dump to host is successful\n");
		}
	}

done:
	ocs_fw_dump_state_set(ocs, OCS_FW_DUMP_STATE_NONE);
	ocs_unlock(&ocs_dump_lock);
	return rc;
}

int32_t
ocs_device_trigger_fw_dump(ocs_t *ocs, uint8_t dump_level)
{
	int32_t rc = OCS_FW_DUMP_STATUS_FAILED;
	char *str = ((OCS_FW_FUNC_DESC_DUMP == dump_level) ? "Func Level" : "Chip Level");

	if (!ocs_fw_dump_buffers_allocated(ocs, dump_level)) {
		ocs_log_err(ocs, "No pre-allocated FW dump memory\n");
		return rc;
	}

	rc = ocs_initiate_gendump(ocs, dump_level);
	if (OCS_FW_DUMP_STATUS_SUCCESS == rc) {
		ocs_log_debug(ocs, "Captured the %s dump into host memory successfully\n", str);
		ocs_device_send_fw_dump_uevent(ocs, dump_level);
	} else {
		/* If we return success here, dump won't be retried */
		if (OCS_FW_DUMP_STATUS_IN_PROGRESS == rc)
			return OCS_FW_DUMP_STATUS_SUCCESS;

		ocs_log_err(ocs, "Failed to trigger the %s dump\n", str);
	}

	return rc;
}

void
ocs_device_trigger_fw_error(ocs_t *ocs, uint8_t dump_level)
{
	char *str = ((OCS_FW_FUNC_DESC_DUMP == dump_level) ? "Func Level" : "Chip Level");

	if (ocs_recovery_state_check(OCS_RECOVERY_STATE_IDLE)) {
		ocs_log_debug(ocs, "Triggering the %s FW error\n", str);
		ocs_hal_raise_ue(&ocs->hal, dump_level);
		if (OCS_FW_FUNC_DESC_DUMP == dump_level)
			ocs->fw_dump.recover_func = TRUE;
	} else {
		ocs_log_debug(ocs, "FW error recovery is already in progress\n");
	}
}

/**
 * @brief Save dump data to OCS buffer; saved_buff will be copied to the
 *        user space buffer when an application requests for it
 */
int32_t
ocs_fw_dump_save(ocs_t *ocs, uint8_t dump_level)
{
	ocs_t *other_ocs;
	ocs_dma_t *dump_buffers = NULL;
	uint8_t *saved_buff = NULL;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	uint32_t index = 0;
	uint32_t offset = 0;
	uint32_t num_buffers = 0;
	uint32_t saved_buff_size = 0;

	if (OCS_FW_DUMP_TYPE_DUMPTOHOST != ocs->fw_dump.type)
		return 0;

	if (OCS_FW_FUNC_DESC_DUMP == dump_level) {
		dump_buffers = ocs->fw_dump.func_dump_buffers;
		num_buffers = ocs->fw_dump.num_func_dump_buffers;
		saved_buff = ocs->fw_dump.saved_buff;
		saved_buff_size = ocs->fw_dump.size;
	} else {
		ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
		while ((other_ocs = ocs_get_instance(index++)) != NULL) {
			ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
			if ((bus == other_bus) && (dev == other_dev) && (0 == other_func)) {
				dump_buffers = other_ocs->fw_dump.chip_dump_buffers;
				num_buffers = other_ocs->fw_dump.num_chip_dump_buffers;
				saved_buff = other_ocs->fw_dump.saved_buff;
				saved_buff_size = other_ocs->fw_dump.size;
				break;
			}
		}
	}

	if (!dump_buffers || !num_buffers || !saved_buff) {
		ocs_log_err(ocs, "Failed to locate dump buffers\n");
		return -1;
	}

	ocs_memset(saved_buff, 0, saved_buff_size);
	for (index = 0; index < num_buffers; index++) {
		if (ocs_memcpy(saved_buff + offset, dump_buffers[index].virt,
				dump_buffers[index].size) == NULL) {
			ocs_log_err(ocs, "ocs_memcpy() failed\n");
			return -1;
		}

		offset += dump_buffers[index].size;
	}

	ocs_log_info(ocs, "FW dump is saved\n");
	return 0;
}

/**
 * @brief Request FW dump
 *
 * @param ocs Pointer to ocs instance
 * @param dump_level Gendump request level can be either chip-level or fdb
 *
 * @return OCS_FW_DUMP_STATUS_SUCCESS on success,
 *         OCS_FW_DUMP_STATUS_IN_PROGRESS if dump is already in progress, or
 *         OCS_FW_DUMP_STATUS_FAILED on failure.
 */
int32_t
ocs_fw_dump_req(ocs_t *ocs, uint8_t dump_level)
{
	int32_t rc = OCS_FW_DUMP_STATUS_SUCCESS;
	uint32_t skip_dump = FALSE;

	/* Validate SLI pause errors */
	if (sli_is_paused(&ocs->hal.sli)) {
		sli_validate_pause_errors(&ocs->hal.sli, &skip_dump);
		/* Don't collect the FW dump if skip_dump is set */
		if (skip_dump)
			dump_level = OCS_FW_DUMP_LEVEL_NONE;
	}

	rc = ocs_trigger_dump(ocs, dump_level);
	if (OCS_FW_DUMP_STATUS_SUCCESS == rc) {
		ocs_fw_dump_save(ocs, dump_level);
		ocs_fw_dump_state_set(ocs, dump_level);
	} else {
		ocs_fw_dump_state_set(ocs, OCS_FW_DUMP_STATE_NONE);
	}

	return rc;
}

int32_t
ocs_fw_reset(ocs_t *ocs, uint8_t dump_level)
{
	ocs_recovery_reset(ocs, OCS_RECOVERY_MODE_MANUAL, dump_level, TRUE);

	while (!ocs_recovery_state_check(OCS_RECOVERY_STATE_IDLE)) {
		/* If recovery has been triggered from a different path, wait till it completes */
		ocs_msleep(100);
	}

	ocs_device_send_fw_dump_uevent(ocs, dump_level);
	return 0;
}

int32_t
ocsu_gendump(ocs_t *ocs)
{
	int32_t rc = 0;
	uint32_t reset_required;

	/* Collect the FW dump */
	rc = ocs_trigger_dump(ocs, OCS_FW_CHIP_LEVEL_DUMP);
	if (rc)
		return rc;

	/* now reset the adapter */
	ocs_hal_get(&ocs->hal, OCS_HAL_RESET_REQUIRED, &reset_required);
	ocs_log_info(ocs, "Reset required=%d\n", reset_required);
	if (reset_required) {
		if (0 == ocs_fw_reset(ocs, OCS_FW_DUMP_LEVEL_NONE)) {
			ocs_log_info(ocs, "All devices reset\n");
		} else {
			ocs_log_err(ocs, "All devices NOT reset\n");
		}
	}

	return rc;
}
