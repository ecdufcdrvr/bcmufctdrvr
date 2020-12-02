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
#include "ocs_recovery.h"

/* Define FW error recovery context */
ocs_fw_error_recovery_t ocs_recovery;

/**
 * @brief Check recovery state
 *
 * @return TRUE if the state matches, FALSE otherwise
 */
bool
ocs_recovery_state_check(uint32_t state)
{
	bool rc = FALSE;

	ocs_lock(&ocs_recovery.lock);
		if (ocs_recovery.state == state)
			rc = TRUE;
	ocs_unlock(&ocs_recovery.lock);

	return rc;
}

/**
 * @brief Set recovery state
 *
 * @return TRUE if the state is modified as requested, FALSE otherwise
 */
bool
ocs_recovery_state_set(uint32_t state)
{
	bool rc = FALSE;

	ocs_lock(&ocs_recovery.lock);
		if (ocs_recovery.state != state) {
			ocs_recovery.state = state;
			rc = TRUE;
		}
	ocs_unlock(&ocs_recovery.lock);

	return rc;
}

/**
 * @brief Handle the EQ processing for all the ocs instances of the adapter
 */
static void
ocs_adapter_event_processing(ocs_t *ocs, bool port_reset, bool start)
{
#if !defined(OCSU_FC_WORKLOAD)
	ocs_t *other_ocs = NULL;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	uint32_t index = 0;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	while ((other_ocs = ocs_get_instance(index++)) != NULL) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			if (port_reset && (func != other_func))
				continue;

			if (start) {
				ocs_log_info(other_ocs, "Start EQ processing\n");
				ocs_start_event_processing(&other_ocs->ocs_os);
			} else {
				ocs_log_info(other_ocs, "Stop EQ processing\n");
				ocs_stop_event_processing(&other_ocs->ocs_os);
			}
		}
	}
#endif
}

/**
 * @brief Set OCS HAL state to inactive for all the ocs instances of the adapter
 */
static void
ocs_adapter_hal_inactive(ocs_t *ocs, bool port_reset)
{
	ocs_t *other_ocs = NULL;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	uint32_t index = 0;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	while ((other_ocs = ocs_get_instance(index++)) != NULL) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			if (port_reset && (func != other_func))
				continue;

			if (OCS_HAL_STATE_ACTIVE == other_ocs->hal.state)
				other_ocs->hal.state = OCS_HAL_STATE_QUEUES_ALLOCATED;
		}
	}
}

/**
 * @brief Shutdown sport(s) for all the ocs instances of the adapter
 */
static void
ocs_adapter_domain_shutdown(ocs_t *ocs, bool port_reset)
{
	ocs_t *other_ocs;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	uint32_t index = 0;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	while ((other_ocs = ocs_get_instance(index++)) != NULL) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			if (port_reset && (func != other_func))
				continue;

			ocs_scsi_tgt_cancel_io(other_ocs);
			if (other_ocs->domain) {
				other_ocs->hal.callback.domain(other_ocs->domain, OCS_HAL_DOMAIN_LOST, other_ocs->domain);
			}

			if (ocs_tgt_nvme_enabled(other_ocs))
				ocs_nvme_hw_port_quiesce(other_ocs);

		}
	}
}

/**
 * @brief Shutdown xport(s) for all the ocs instances of the adapter
 */
static void
ocs_adapter_xport_shutdown(ocs_t *ocs, bool port_reset)
{
	ocs_t *other_ocs;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	uint32_t index = 0;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	while ((other_ocs = ocs_get_instance(index++)) != NULL) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			if (port_reset && (func != other_func))
				continue;

			ocs_log_debug(other_ocs, "Initiate transport shutdown\n");
			if (ocs_xport_control(other_ocs->xport, OCS_XPORT_SHUTDOWN)) {
				ocs_log_err(other_ocs, "Tranport shutdown failed\n");
			}
		}
	}
}

/**
 * @brief Free the NVMe ports
 */
static void
ocs_adapter_nvme_free_ports(ocs_t *ocs, bool port_reset)
{
	ocs_t *other_ocs;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	uint32_t index = 0;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	while ((other_ocs = ocs_get_instance(index++)) != NULL) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			if (port_reset && (func != other_func))
				continue;

			if (ocs_tgt_nvme_enabled(other_ocs))
				ocs_nvme_hw_port_free(other_ocs);
		}
	}
}

static int32_t
ocs_port_reset(ocs_t *ocs)
{
	int rc = 0;

	if (ocs_hal_reset(&ocs->hal, OCS_HAL_RESET_FUNCTION)) {
		ocs_log_err(ocs, "Failed to reset port\n");
		rc = -1;
	} else {
		ocs_log_info(ocs, "Successfully reset port\n");

		/* now initialize hal so user can read the dump in */
		if (ocs_hal_init(&ocs->hal)) {
			ocs_log_err(ocs, "Failed to initialize hal\n");
			rc = -1;
		} else {
			ocs_log_info(ocs, "Successfully initialized hal\n");
		}
	}

	return rc;
}

/* Reset all the functions associated with a bus/dev */
static int32_t
ocs_adapter_reset(ocs_t *ocs)
{
	ocs_t *other_ocs = NULL;
	uint8_t bus, dev, func;
	int rc = 0;
	uint32_t index = 0;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	while ((other_ocs = ocs_get_instance(index++)) != NULL) {
		uint8_t other_bus, other_dev, other_func;

		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			if (0 != ocs_port_reset(other_ocs))
				rc = -1;
		}
	}

	return rc;
}

/**
 * @brief Bring the links online
 */
static int32_t
ocs_adapter_xport_online(ocs_t *ocs, bool port_reset)
{
	ocs_t *other_ocs;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	int32_t rc = 0, ret = 0;
	uint32_t index = 0;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	while ((other_ocs = ocs_get_instance(index++)) != NULL) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			if (port_reset && (func != other_func))
				continue;

			rc = ocs_scsi_tgt_port_online(other_ocs);
			if (rc) {
				ocs_log_err(other_ocs, "Unable to bring-up the link\n");
				ret = rc;
			}
		}
	}

	return ret;
}

/**
 * @brief Initiate Recovery and reset. Generate FW dump if requested.
 *
 * @param ocs Pointer to ocs instance
 * @param recovery_mode Recovery mode can be auto-recovery / manual / port-migration
 * @param reset_level Reset level requested (either chip-level or function-level)
 * @param reset_fw Reset firmware if requested
 *
 * @return OCS_RECOVERY_STATUS_SUCCESS on success,
 *         OCS_RECOVERY_STATUS_IN_PROGRESS if recovery is already in progress, or
 *         OCS_RECOVERY_STATUS_FAILED on failure.
 */
int32_t
ocs_recovery_reset(ocs_t *ocs, uint32_t recovery_mode, uint8_t reset_level, bool reset_fw)
{
	bool port_reset = (reset_level == OCS_RESET_LEVEL_PORT);
	uint8_t bus, dev, func;
	int32_t rc;

	/* Check if recovery state started */
	if (!ocs_recovery_state_set(OCS_RECOVERY_STATE_IN_PROGRESS)) {
		ocs_log_info(ocs, "FW error recovery is already in progress\n");
		return OCS_RECOVERY_STATUS_IN_PROGRESS;
	}

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);

	/* Check if we have to generate the FW dump */
	if (OCS_RECOVERY_MODE_PORT_MIGRATION != recovery_mode) {
		/* Collect the driver dump before triggering the FW dump */
		ocs_adapter_save_ddump(ocs, OCS_DDUMP_FLAGS_WQES|OCS_DDUMP_FLAGS_CQES|OCS_DDUMP_FLAGS_MQES,
				       -1, TRUE, !port_reset);
		if (reset_fw) {
			/* Trigger firmware reset */
			rc = ocs_fw_dump_req(ocs, reset_level);
			if (OCS_FW_DUMP_STATUS_IN_PROGRESS == rc) {
				rc = OCS_RECOVERY_STATUS_IN_PROGRESS;
				ocs_log_info(ocs, "FW dump is already in progress\n");
				return rc;
			}

			if (OCS_FW_DUMP_STATUS_FAILED == rc) {
				rc = OCS_RECOVERY_STATUS_FAILED;
				ocs_log_err(ocs, "FW dump has failed\n");
				goto exit_err;
			}
		}
	} else {
		rc = ocs_hal_port_migration(&ocs->hal);
		if (rc) {
			ocs_log_err(ocs, "Port migration has failed\n");
			goto exit_err;
		}

		port_reset = TRUE;
	}

	rc = OCS_RECOVERY_STATUS_SUCCESS;

	/* Wait for all the pending events to be processed */
	ocs_adapter_event_processing(ocs, port_reset, FALSE);

	/* Clear the 'recover_func' flag blindly before doing a HAL reset */
	ocs->fw_dump.recover_func = FALSE;

	/* Check and set HAL state inactive so that we don't post any more WQE's */
	ocs_adapter_hal_inactive(ocs, port_reset);

	/* Shutdown sport(s) for all the ocs instances of the adapter */
	ocs_adapter_domain_shutdown(ocs, port_reset);

	/* Shutdown xport(s) on all the ports to remove domain / sport forcefully */
	ocs_adapter_xport_shutdown(ocs, port_reset);

	ocs_adapter_nvme_free_ports(ocs, port_reset);

	/* now reset the port / adapter */
	if (port_reset) {
		rc = ocs_port_reset(ocs);
		ocs_log_info(ocs, "OCS port (%02d:%02d:%02d) reset has %s\n",
				bus, dev, func, (rc ? "failed" : "succeeded"));
	} else {
		rc = ocs_adapter_reset(ocs);
		ocs_log_info(ocs, "OCS adapter (%02d:%02d) reset has %s\n",
				bus, dev, (rc ? "failed" : "succeeded"));
	}

	/* Re-enable the interrupts */
	ocs_adapter_event_processing(ocs, port_reset, TRUE);

	/* Bring links on each port to the previous state */
	rc = ocs_adapter_xport_online(ocs, port_reset);
	if (rc)
		ocs_log_err(ocs, "Failed to bring ports online\n");

exit_err:
	/* Set recovery state to default, irrespective of the rc */
	ocs_recovery_state_set(OCS_RECOVERY_STATE_IDLE);
	return rc;
}

/**
 * @brief Recover adapter from chip error.
 */
static void
ocs_adapter_recover(ocs_t *ocs, uint32_t mode, uint8_t reset_level)
{
	bool reset_fw = TRUE;
	uint8_t bus, dev, func;
	int32_t rc = OCS_RECOVERY_STATUS_SUCCESS;
	uint32_t err1 = sli_reg_read(&ocs->hal.sli, SLI4_REG_SLIPORT_ERROR1);
	uint32_t err2 = sli_reg_read(&ocs->hal.sli, SLI4_REG_SLIPORT_ERROR2);

	if (ocs_recovery_state_check(OCS_RECOVERY_STATE_IN_PROGRESS)) {
		ocs_log_info(ocs, "FW error recovery is already in progress\n");
		return;
	}

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	ocs_log_crit(ocs, "OCS adapter %02d:%02d recovery started for UE "
			"status=%#x error1=%#x error2=%#x\n", bus, dev,
			sli_reg_read(&ocs->hal.sli, SLI4_REG_SLIPORT_STATUS),
			err1, err2);

	switch (err1) {
		/* Add the err1 codes here that impact only a single function */
		case SLI4_SLIPORT_ERROR1_RQ_EMPTY_TIMEOUT:
			reset_level = OCS_RESET_LEVEL_PORT;
			reset_fw = FALSE;
			/* fall-through */
		default:
			break;
	}

	/* gendump and reset adapter */
	rc = ocs_recovery_reset(ocs, mode, reset_level, reset_fw);
	if (OCS_RECOVERY_STATUS_SUCCESS == rc) {
		ocs_log_info(ocs, "FW error recovery is successful\n");
		ocs_device_send_fw_dump_uevent(ocs, reset_level);
	} else if (OCS_RECOVERY_STATUS_IN_PROGRESS == rc) {
		ocs_log_info(ocs, "FW error recovery is already in progress\n");
	} else {
		ocs_log_err(ocs, "FW error recovery has failed\n");
	}
}

/**
 * @ingroup os
 * @brief OCS FW error recovery thread
 *
 * @return 0 if success; < 0 otherwise.
 */
static int32_t
ocs_recovery_thread(ocs_thread_t *th)
{
	ocs_t *ocs;
	int32_t index;
	uint8_t reset_level;

	while (!ocs_thread_terminate_requested(th)) {
		ocs_msleep(1000);

		if (ocs_recovery_state_check(OCS_RECOVERY_STATE_IN_PROGRESS))
			continue;

		index = 0;
		reset_level = OCS_RESET_LEVEL_NONE;
		while ((ocs = ocs_get_instance(index++)) != NULL) {
			reset_level = ocs_hal_reset_pending(&ocs->hal);
			if (reset_level)
				break;
		}

		if (OCS_RESET_LEVEL_NONE == reset_level)
			continue;

		ocs_adapter_recover(ocs, OCS_RECOVERY_MODE_AUTO, reset_level);
	}

	ocs_recovery_state_set(OCS_RECOVERY_STATE_IDLE);
	return 0;
}

/**
 * @brief Create a FW error recovery thread
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocs_fw_err_recovery_setup(void)
{
	int32_t rc;

	ocs_lock_init(NULL, &ocs_recovery.lock, "fw_err_recovery_lock");
	ocs_recovery.state = OCS_RECOVERY_STATE_IDLE;

	rc = ocs_thread_create(NULL, &ocs_recovery.thread, ocs_recovery_thread,
			       "ocs_recovery", NULL, OCS_THREAD_RUN);
	if (rc < 0)
		ocs_log_err(NULL, "Failed to create FW err detection thread\n");

	return rc;
}

/**
 * @brief Stop FW error recovery thread
 *
 * @return void
 */
void
ocs_fw_err_recovery_stop(void)
{
	ocs_thread_terminate(&ocs_recovery.thread);
	ocs_lock_free(&ocs_recovery.lock);
	ocs_log_info(NULL, "FW error recovery thread terminated\n");
}
