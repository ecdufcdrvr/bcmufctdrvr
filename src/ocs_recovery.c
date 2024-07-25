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

#include "ocs.h"
#include "ocs_recovery.h"
#include "ocs_compat.h"

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

#define OCS_STATE_SET_SUCCESS	0
#define OCS_STATE_SET_FAILED	1
#define OCS_STATE_EXISTS	2
#define OCS_STATE_INVALID	3

/**
 * @brief set PCI recovery state
 */
void
ocs_recovery_pci_err_state(bool enable)
{
	if (enable) {
		int rc;

		/* Wait for any ongoing FW dump/recovery to complete */
		rc = ocs_recovery_state_set_stop();
		if ((rc == OCS_STATE_EXISTS) || (rc == OCS_STATE_SET_SUCCESS)) {
			ocs_atomic_add_return(&ocs_recovery.pci_err_ref, 1);
		}
	} else {
		if (ocs_atomic_read(&ocs_recovery.pci_err_ref) == 0) {
			ocs_log_err(NULL, "pci_err_ref is 0 already\n");
			return;
		}

		if (ocs_atomic_sub_and_test(&ocs_recovery.pci_err_ref, 1))
			ocs_recovery_state_set(OCS_RECOVERY_STATE_IDLE);
	}
}

/**
 * @brief Set recover state to stop
 */
int
ocs_recovery_state_set_stop(void)
{
	int rc = OCS_STATE_INVALID;

	/* Wait for any ongoing FW dump/recovery to complete */
	while (true) {
		rc = ocs_recovery_state_set(OCS_RECOVERY_STATE_STOP);
		if (rc != OCS_STATE_SET_FAILED) {
			break;
		}
		ocs_msleep(100);
	}

	return rc;
}

/**
 * @brief Set recovery state
 *
 * @return 0 if the state is modified as requested, 1 if failed, 2 if the same state exists
 */
uint8_t
ocs_recovery_state_set(uint32_t new_state)
{
	uint8_t rc = OCS_STATE_SET_FAILED;

	ocs_lock(&ocs_recovery.lock);
	/* return failure if we try to set same state */
	if (ocs_recovery.state == new_state) {
		ocs_unlock(&ocs_recovery.lock);
		return OCS_STATE_EXISTS;
	}

	switch (ocs_recovery.state) {
	case OCS_RECOVERY_STATE_IDLE:
		ocs_recovery.state = new_state;
		rc = OCS_STATE_SET_SUCCESS;
		break;
	case OCS_RECOVERY_STATE_NOT_INIT:
		if (new_state != OCS_RECOVERY_STATE_IDLE) {
			rc = OCS_STATE_INVALID;
			break;
		}
		FALL_THROUGH; /* fall-through */
	case OCS_RECOVERY_STATE_STOP:
	case OCS_RECOVERY_STATE_IN_PROGRESS:
	default:
		if (new_state == OCS_RECOVERY_STATE_IDLE) {
			ocs_recovery.state = new_state;
			rc = OCS_STATE_SET_SUCCESS;
		}
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
	ocs_t *other_ocs = NULL;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	uint32_t index = 0;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	for_each_active_ocs(index, other_ocs) {
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
}

/**
 * @brief Set OCS HAL state to inactive for all the ocs instances of the adapter
 */
void
ocs_adapter_hal_inactive(ocs_t *ocs, bool port_reset)
{
	ocs_t *other_ocs = NULL;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	uint32_t index = 0;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	for_each_active_ocs(index, other_ocs) {
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
	for_each_active_ocs(index, other_ocs) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			if (port_reset && (func != other_func))
				continue;

			/*
			 * As we are going to cleanup domain, complete any
			 * pending mailbox commands with firmware here.
			 */
			ocs_hal_flush(&other_ocs->hal);
			ocs_hal_command_cancel(&other_ocs->hal);
			ocs_scsi_tgt_cancel_io(other_ocs);
			shutdown_target_wqe_timer(&other_ocs->hal);

			/* Complete pending flush request */
			ocs_hal_rq_marker_clean(&other_ocs->hal);

			if (other_ocs->domain)
				other_ocs->hal.callback.domain(other_ocs->hal.args.domain,
							       OCS_HAL_DOMAIN_LOST,
							       other_ocs->domain);

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
	for_each_active_ocs(index, other_ocs) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			if (port_reset && (func != other_func))
				continue;

			ocs_log_debug(other_ocs, "Initiate transport shutdown\n");
			if (ocs_xport_control(other_ocs->xport, OCS_XPORT_SHUTDOWN))
				ocs_log_err(other_ocs, "Transport shutdown failed\n");
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
	for_each_active_ocs(index, other_ocs) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			if (port_reset && (func != other_func))
				continue;

			if (ocs_tgt_nvme_enabled(other_ocs))
				ocs_nvme_hw_port_free(other_ocs);
		}
	}
}

/**
 * @brief Reinit NVMe ports
 */
static void
ocs_adapter_nvme_reinit_ports(ocs_t *ocs, bool port_reset)
{
	ocs_t *other_ocs;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	uint32_t index = 0;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	for_each_active_ocs(index, other_ocs) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			if (port_reset && (func != other_func))
				continue;

			if (ocs_tgt_nvme_enabled(other_ocs))
				ocs_nvme_hw_port_reinit(other_ocs);
		}
	}
}

/**
 * @brief Shutdown FC stats timer
 */
static void
ocs_adapter_shutdown_stats_timer(ocs_t *ocs, bool port_reset)
{
	ocs_t *other_ocs;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	uint32_t index = 0;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	for_each_active_ocs(index, other_ocs) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			if (port_reset && (func != other_func))
				continue;

			/*Shutdown FC Statistics timer*/
			ocs_del_timer(&other_ocs->xport->fc_stats_timer);
		}
	}
}

#define OCS_NODE_SHUT_MAX_TIME 70 /* seconds */

/* @Brief: Check and report if any node is taking longer time
 * 	   during shutdown
 *
 * @arg: OCS device
 *
 * @return: True if any node shutdown is timedout;else false
 *
 */
bool
ocs_node_shutdown_timedout(ocs_t *ocs)
{
	ocs_domain_t *domain;
	ocs_sport_t *sport;
	ocs_node_t *node = NULL;
	bool rc = false;

	ocs_device_lock(ocs);
	domain = ocs->domain;
	if (domain) {
		ocs_domain_lock(domain);
		ocs_list_foreach(&domain->sport_list, sport) {
			ocs_sport_lock(sport);
			ocs_lock(&sport->node_shutdown_lock);
			node = ocs_list_get_head(&sport->node_shutdown_list);
			ocs_unlock(&sport->node_shutdown_lock);
			if (node) {
				uint64_t ticks_timeout = node->shutdown_start_time +
							 OCS_NODE_SHUT_MAX_TIME * ocs_get_os_tick_freq();

				ocs_node_lock(node);

				if (ocs_time_after64(ocs_get_os_ticks(), ticks_timeout)) {
					node_printf(node,
							"Node(curr_state: %s curr_event: %s) shutdown is timed-out\n",
							node->current_state_name,
							ocs_sm_event_name(node->current_evt));
					rc = true;
				}
				ocs_node_unlock(node);
			}
			ocs_sport_unlock(sport);
			if (rc)
				break;
		}
		ocs_domain_unlock(domain);
	}
	ocs_device_unlock(ocs);

	return rc;
}

/**
 * @brief Setup FC stats timer
 */
static void
ocs_adapter_setup_stats_timer(ocs_t *ocs, bool port_reset)
{
	ocs_t *other_ocs;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	uint32_t index = 0;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	for_each_active_ocs(index, other_ocs) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			if (port_reset && (func != other_func))
				continue;

			ocs_xport_setup_stats_timer(other_ocs);
		}
	}
}

static int32_t
ocs_port_update_protocols(ocs_t *ocs)
{
	if (!ocs->user_ini_fc_types && !ocs->user_tgt_fc_types) {
		ocs_log_err(ocs, "Invalid fc_types init = %d tgt = %d\n",
			    ocs->user_ini_fc_types, ocs->user_tgt_fc_types);
		return -1;
	}

	ocs->ini_fc_types = ocs->user_ini_fc_types;
	if (ocs->ini_fc_types)
		ocs->enable_ini = 1;
	else
		ocs->enable_ini = 0;

	ocs->tgt_fc_types = ocs->user_tgt_fc_types;
	if (ocs->tgt_fc_types)
		ocs->enable_tgt = 1;
	else
		ocs->enable_tgt = 0;

	ocs_strncpy(ocs->filter_def, ocs->user_filter_def, sizeof(ocs->filter_def));
	ocs_hal_set_ptr(&ocs->hal, OCS_HAL_FILTER_DEF, (void *) ocs->filter_def);

	if (ocs_hal_setup_queue_topology(&ocs->hal, ocs->user_queue_topology) != OCS_HAL_RTN_SUCCESS) {
		ocs_log_err(ocs, "Failed to parse queue_topology\n");
		return -1;
	}

	if (ocs_hal_xri_resource_reinit(&ocs->hal)) {
		ocs_log_err(ocs, "HAL xri reinit failed.\n");
		return -1;
	}

	return 0;
}

static int32_t
ocs_port_reset(ocs_t *ocs)
{
	int rc = 0;

	if (ocs_hal_reset(&ocs->hal, OCS_HAL_RESET_FUNCTION)) {
		ocs_log_err(ocs, "Failed to reset port\n");
		rc = -1;
	} else {
		bool protocols_updated = false;

		ocs_log_info(ocs, "Successfully reset port\n");

		if (ocs->update_protocols) {
			/* Backends are going to change. Cleanup them */
			ocs_xport_cleanup_backends(ocs);

			if (ocs_port_update_protocols(ocs)) {
				ocs_log_err(ocs, "Port protocol update failed\n");
				return -1;
			}

			ocs->update_protocols = false;
			protocols_updated = true;
		}

		if (protocols_updated)
			ocs_log_info(ocs, "Successfully updated protocols\n");

#if !defined(OCS_USPACE)
		if (ocs_virtfn(ocs) || protocols_updated) {
			if (ocs_msix_reset(ocs)) {
				ocs_log_err(ocs, "MSIx reset failed\n");
				return -1;
			}
		}
#endif
		/* now initialize hal so user can read the dump in */
		if (ocs_hal_init(&ocs->hal)) {
			ocs_log_err(ocs, "Failed to initialize hal\n");
			rc = -1;
		} else {
			ocs_log_info(ocs, "Successfully initialized hal\n");

			/* Firmware version could have changed, update it
			 * with SCSI ML
			 */
#if !defined(OCS_USPACE)
			ocs_scsi_ini_update_symbolic_name(ocs);
#endif
			if (protocols_updated && ocs_xport_init_backends(ocs)) {
				ocs_log_err(ocs, "ocs_xport_init_backends failed.\n");
				return -1;
			}
		}

		ocs->need_reset = false;
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
	for_each_active_ocs(index, other_ocs) {
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
	for_each_active_ocs(index, other_ocs) {
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
 * @param trigger_dump Trigger dump if requested
 *
 * @return OCS_RECOVERY_STATUS_SUCCESS on success,
 *         OCS_RECOVERY_STATUS_IN_PROGRESS if recovery is already in progress, or
 *         OCS_RECOVERY_STATUS_FAILED on failure.
 */
int32_t
ocs_recovery_reset(ocs_t *ocs, uint32_t recovery_mode, uint8_t reset_level, bool trigger_dump)
{
	bool port_reset = (reset_level == OCS_RESET_LEVEL_PORT);
	uint8_t bus, dev, func;
	int32_t rc;

	if (reset_level == OCS_RESET_LEVEL_NONE) {
		ocs_log_info(ocs, "Reset level NONE, skip recovery reset\n");
		return OCS_RECOVERY_STATUS_SUCCESS;
	}
 
	/* If ocs_req_fw_reset is set, the recovery state is either STATE_NOT_INIT 
	 * or STATE_STOP. In either of these states, transition to STATE_IN_PROGRESS
	 * is not allowed, so skip the state transition. 
	 * Otherwise, check if recovery is in progress already. */
	if (!ocs->ocs_req_fw_reset && 
			ocs_recovery_state_set(OCS_RECOVERY_STATE_IN_PROGRESS)) {
		ocs_log_info(ocs, "FW error recovery is already in progress\n");
		return OCS_RECOVERY_STATUS_IN_PROGRESS;
	}

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	ocs_log_info(ocs, "OCS %s reset started\n",
		     port_reset ? "port level" : "chip level");

	/* Check if we have to generate the FW dump */
	if (OCS_RECOVERY_MODE_PORT_MIGRATION != recovery_mode) {
		/* Collect the driver dump before triggering the FW dump */
		ocs_adapter_save_ddump(ocs, OCS_DDUMP_FLAGS_WQES|OCS_DDUMP_FLAGS_CQES|OCS_DDUMP_FLAGS_MQES,
				       -1, TRUE, !port_reset);

		rc = ocs_device_request_reset_dump(ocs, reset_level, trigger_dump);
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

		if (!ocs_hal_wait_for_fw_ready(&ocs->hal)) {
			rc = OCS_RECOVERY_STATUS_FAILED;
			ocs_log_err(ocs, "FW recovery has failed\n");
			goto exit_err;
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

	/* Check and set HAL state inactive so that we don't post any more WQE's */
	ocs_adapter_hal_inactive(ocs, port_reset);

	/* Wait for all the pending events to be processed */
	ocs_adapter_event_processing(ocs, port_reset, FALSE);

	/* Stop the rq threads */
	ocs_xport_rq_threads_teardown(ocs->xport);

#ifdef OCS_GEN_ABORTS
	if (ocs->gen_aborts) {
		ocs->gen_aborts = false;
		ocs->gen_single_abort = false;
		ocs->duplicate_aborts = 0;
		ocs->gen_abort_interval_msecs = 0;
		ocs_thread_terminate(&ocs->abts_gen_thread);
	}
#endif

	/* Clear the 'recover_func' flag blindly before doing a HAL reset */
	ocs->fw_dump.recover_func = FALSE;

	/* Shutdown sport(s) for all the ocs instances of the adapter */
	ocs_adapter_domain_shutdown(ocs, port_reset);

	/* Shutdown xport(s) on all the ports to remove domain / sport forcefully */
	ocs_adapter_xport_shutdown(ocs, port_reset);

	ocs_adapter_nvme_free_ports(ocs, port_reset);

	ocs_adapter_shutdown_stats_timer(ocs, port_reset);

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

	ocs_adapter_nvme_reinit_ports(ocs, port_reset);

	/* Re-enable the interrupts */
	ocs_adapter_event_processing(ocs, port_reset, TRUE);

	/* Re-start the rq threads if required */
	rc = ocs_xport_rq_threads_create(ocs->xport, ocs->rq_threads);
	if (rc) 
		ocs_log_err(ocs, "fail to create RQ threads\n");

	ocs_adapter_setup_stats_timer(ocs, port_reset);

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
	bool trigger_dump = TRUE;
	uint8_t bus, dev, func;
	int32_t rc = OCS_RECOVERY_STATUS_SUCCESS;
	uint32_t err1 = sli_reg_read(&ocs->hal.sli, SLI4_REG_SLIPORT_ERROR1);
	uint32_t err2 = sli_reg_read(&ocs->hal.sli, SLI4_REG_SLIPORT_ERROR2);
	uint32_t fw_error, reset_required;
	uint32_t dump_present;

	if (ocs_recovery_state_check(OCS_RECOVERY_STATE_IN_PROGRESS)) {
		ocs_log_info(ocs, "FW error recovery is already in progress\n");
		return;
	}

	if ((ocs_hal_get(&ocs->hal, OCS_HAL_FW_ERROR, &fw_error) == OCS_HAL_RTN_SUCCESS) &&
	    (ocs_hal_get(&ocs->hal, OCS_HAL_RESET_REQUIRED, &reset_required) == OCS_HAL_RTN_SUCCESS)) {
		if ((fw_error == 1) && (reset_required == 0)) {
			ocs_log_crit(ocs, "Unrecoverable chip error detected; host needs "
					  "to be power cycled or reset for recovery\n");
			return;
		}
	}

	switch (err1) {
		/* Add the err1 codes here that impact only a single function */
		case SLI4_SLIPORT_ERROR1_MBX:
			if (err2 == SLI4_SLIPORT_ERROR2_MBX_TIMEOUT)
				reset_level = OCS_RESET_LEVEL_PORT;
			break;
		case SLI4_SLIPORT_ERROR1_WDT_TIMEOUT:
			if (err2 == SLI4_SLIPORT_ERROR2_WDT_TIMEOUT) {
				ocs_log_crit(ocs, "FW WDT timedout after %d seconds. Skip recovery\n",
					     ocs->hal.watchdog_timeout);
				reset_level = OCS_RESET_LEVEL_NONE;
				trigger_dump = FALSE;
			}
			break;
		case SLI4_SLIPORT_ERROR1_RQ_EMPTY_TIMEOUT:
			reset_level = OCS_RESET_LEVEL_PORT;
			trigger_dump = FALSE;
			FALL_THROUGH; /* fall-through */
		default:
			break;
	}

	if (reset_level != OCS_RESET_LEVEL_NONE) {
		ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
		ocs_log_crit(ocs, "OCS adapter %02d:%02d recovery started for "
			     "status=%#x error1=%#x error2=%#x\n",
			     bus, dev, sli_reg_read(&ocs->hal.sli, SLI4_REG_SLIPORT_STATUS),
			     err1, err2);

		/* gendump and reset adapter */
		rc = ocs_recovery_reset(ocs, mode, reset_level, trigger_dump);
		if (OCS_RECOVERY_STATUS_SUCCESS == rc) {
			ocs_log_info(ocs, "FW error recovery is successful\n");
		} else if (OCS_RECOVERY_STATUS_IN_PROGRESS == rc) {
			ocs_log_info(ocs, "FW error recovery is already in progress\n");
		} else {
			ocs_log_err(ocs, "FW error recovery has failed\n");
		}

		ocs_hal_get(&ocs->hal, OCS_HAL_DUMP_PRESENT, &dump_present);
		if (dump_present != OCS_FW_DUMP_STATUS_NOT_PRESENT)
			ocs_device_send_fw_dump_uevent(ocs, reset_level);
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

		if (!ocs_recovery_state_check(OCS_RECOVERY_STATE_IDLE))
			continue;

		index = 0;
		reset_level = OCS_RESET_LEVEL_NONE;
		for_each_active_ocs(index, ocs) {
			if (!ocs->drv_ocs.attached)
				continue;

			reset_level = ocs_hal_reset_pending(&ocs->hal);
			if (reset_level) {
				break;
			} else if (ocs_node_shutdown_timedout(ocs)) {
				ocs_log_debug(ocs, "Resetting function due to node shutdown timeout\n");
				reset_level = OCS_RESET_LEVEL_PORT;
				break;
			}

#if !defined(OCS_USPACE)
			/* Honor WDT timeout checking in absense of g_ocs_app_watchdog_kthreads */
			if (!ocs_app_watchdog_thread_configured() && ocs_watchdog_timedout(ocs)) {
				/*
				 * FW will refresh the WDT timer when there are other
				 * outstanding mbx commands. Stop submitting new mbx commands and
				 * let FW detect WDT timeout.
				 */
				ocs_lock(&ocs->hal.cmd_lock);
				ocs->hal.cmd_submission_disabled = true;
				ocs_unlock(&ocs->hal.cmd_lock);

				ocs_log_crit(ocs, "APP FW heartbeat failed; disable mbx cmd submission\n");
			}
#endif
		}

		if (OCS_RESET_LEVEL_NONE == reset_level)
			continue;

		ocs_adapter_recover(ocs, OCS_RECOVERY_MODE_AUTO, reset_level);
	}

	ocs_recovery_state_set(OCS_RECOVERY_STATE_IDLE);
	return 0;
}

/**
 * @brief Initilize FW recovery lock and state
 *
 * @return NONE*/
void
ocs_fw_err_recovery_init(void)
{
	ocs_recovery.state = OCS_RECOVERY_STATE_NOT_INIT;
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

	ocs_lock_init(NULL, &ocs_recovery.lock, OCS_LOCK_ORDER_IGNORE, "fw_err_recovery_lock");
	ocs_recovery.state = OCS_RECOVERY_STATE_IDLE;
	ocs_atomic_init(&ocs_recovery.pci_err_ref, 0);

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
