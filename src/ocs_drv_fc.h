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
 * OCS linux driver common include file
 */


#if !defined(__OCS_DRV_FC_H__)
#define __OCS_DRV_FC_H__

#define OCS_INCLUDE_FC

#include "ocs_os.h"
#include "ocs_debug.h"
#include "ocs_fc_config.h"
#include "ocs_common.h"
#include "ocs_scsi.h"
#include "ocs_hal.h"
#include "ocs_io.h"
#include "ocs_node.h"
#include "ocs_unsol.h"
#include "ocs_ioctl.h"
#include "ocs_elxu.h"
#include "ocs_pm.h"
#include "ocs_xport.h"
#include "ocs_gendump.h"

#if defined(ENABLE_FABRIC_EMULATION)
#include "ocs_femul.h"
#endif
#include "ocs_stats.h"
#include "ocs_mgmt.h"

#define OCS_SUPPORT_NPIV	BIT(10)
#define OCS_FLOGI_FINISH	BIT(14)

struct ocs_s {
	ocs_os_t ocs_os;
	char display_name[OCS_DISPLAY_NAME_LENGTH];
	ocs_rlock_t lock;			/*>> Device wide lock */
	ocs_list_t domain_list;			/*>> linked list of virtual fabric objects */
	ocs_io_pool_t *io_pool;			/**< pointer to IO pool */
	ocs_ramlog_t *ramlog;
	ocs_drv_t drv_ocs;
#if defined(OCS_USPACE_SPDK_UPSTREAM)
	struct ocs_nvme_tgt tgt_ocs;
#else
	ocs_scsi_tgt_t tgt_ocs;
#endif
	ocs_scsi_ini_t ini_ocs;
	ocs_xport_e ocs_xport;
	ocs_xport_t *xport;			/*>> Pointer to transport object */
	bool enable_ini;
	uint8_t ini_fc_types;			/*>> Initiator type - SCSI/NVME */
	bool enable_tgt;
	uint8_t tgt_fc_types;			/*>> Target type - SCSI/NVME */
	int ctrlmask;
	int logmask;
	uint32_t max_isr_time_msec;		/*>> Maximum ISR time */
	char *hal_war_version;
	ocs_pm_context_t pm_context;		/*<< power management context */
	ocs_mgmt_functions_t *mgmt_functions;
	ocs_mgmt_functions_t *tgt_mgmt_functions;
	ocs_mgmt_functions_t *ini_mgmt_functions;
	ocs_err_injection_e err_injection;	/**< for error injection testing */
	uint32_t cmd_err_inject;		/**< specific cmd to inject error into */
	time_t delay_value_msec;		/**< for injecting delays */
	uint16_t sriov_nr_vfs;
	bool sriov_probe_vfs;

	const char *desc;
	uint32_t instance_index;
	uint16_t pci_vendor;
	uint16_t pci_device;
	uint16_t pci_subsystem_vendor;
	uint16_t pci_subsystem_device;
	char businfo[OCS_DISPLAY_BUS_INFO_LENGTH];

	const char *model;
	const char *driver_version;
	const char *fw_version;

	ocs_hal_t hal;

	ocs_domain_t *domain;			/*>> pointer to first (physical) domain (also on domain_list) */
	uint32_t domain_instance_count;			/*>> domain instance count */
	void (*domain_list_empty_cb)(ocs_t *ocs, void *arg); /*>> domain list empty callback */
	void *domain_list_empty_cb_arg;                 /*>> domain list empty callback argument */
	ocs_lock_t domain_list_empty_cb_lock;		/* lock for domain list empty cb */
	ocs_sem_t domain_list_empty_cb_sem;

	int32_t holdoff_link_online;
	bool explicit_buffer_list;
	bool external_loopback;
	uint32_t num_vports;
	uint32_t hal_bounce;
	uint32_t rq_threads;
	uint32_t rq_selection_policy;
	uint32_t worker_poll_no_tmo;
	uint32_t rr_quanta;
	char *filter_def;
	uint32_t max_remote_nodes;
	uint32_t enable_dual_dump;
	uint64_t soft_wwpn_restore;
	uint64_t soft_wwnn_restore;

	/*
	 * tgt_rscn_delay - delay in kicking off RSCN processing (nameserver queries)
	 * after receiving an RSCN on the target. This prevents thrashing of nameserver
	 * requests due to a huge burst of RSCNs received in a short period of time
	 * Note: this is only valid when target RSCN handling is enabled -- see ctrlmask.
	 */
	time_t tgt_rscn_delay_msec;		/*>> minimum target RSCN delay */

	/*
	 * tgt_rscn_period - determines maximum frequency when processing back-to-back
	 * RSCNs; e.g. if this value is 30, there will never be any more than 1 RSCN
	 * handling per 30s window. This prevents initiators on a faulty link generating
	 * many RSCN from causing the target to continually query the nameserver. Note:
	 * this is only valid when target RSCN handling is enabled
	 */
	time_t tgt_rscn_period_msec;		/*>> minimum target RSCN period */

	/*
	 * Target IO timer value:
	 * Zero: target command timeout disabled.
	 * Non-zero: Timeout value, in seconds, for target commands
	 */
	uint32_t target_io_timer_sec;

	/*
	 * FCP_TRECEIVE64_WQE timer value, in seconds.
	 */
	uint8_t tgt_wqe_timer;

	int speed;
	int topology;
	int ethernet_license;
	int num_scsi_ios;
	bool soft_wwn_enable;
	bool enable_hlm;			/*>> high login mode is enabled */
	bool enable_dpp;			/*>> DPP Mode enabled */
	bool enable_poll_mode;			/*>> Poll Mode enabled */
	uint32_t hlm_group_size;		/*>> RPI count for high login mode */
	char *wwn_bump;
	uint32_t nodedb_mask;			/*>> Node debugging mask */
	bool cfg_suppress_rsp;			/*>> Config Suppress Response feature */

	bool esoc;
	uint8_t ocs_req_fw_upgrade;
	uint8_t enable_auto_recovery;
	uint16_t sw_feature_cap;

	ocs_textbuf_t ddump_saved;
	uint8_t ddump_state;
	bool ddump_pre_alloc;
	struct ocs_fw_dump fw_dump;
	int ddump_max_size;

#if !defined(OCSU_FC_RAMD) && !defined(OCSU_FC_WORKLOAD) && !defined(OCS_USPACE_SPDK)
	unsigned long ocs_pci_flags;
	struct work_struct pci_dev_disable;
#endif

	uint8_t domain_shutdown_timedout;	/* Track shutdown timeout for userspace drivers */
	uint8_t sliport_healthcheck;
	uint8_t disable_fec;
	/*
	 * Enable firmware path to send auto good response.
	 * This will force the auto-response param field to zero.
	 */
	uint8_t enable_fw_ag_rsp;

	uint8_t enable_bbcr;
	uint16_t watchdog_timeout;
	const char *sliport_pause_errors;
#ifdef OCS_USPACE_SPDK
	bool dont_linkup;
	uint64_t scsi_lcore_id;
#endif

	/* Auth cfg data store */
	ocs_dma_t			auth_cfg_mem;
	uint32_t			num_auth_cfg_entries;
	bool				auth_cfg_updated;
	struct ocs_auth_cfg_info	auth_cfg_info;
	/* Lock used for safe access to cfg_page and cfg_info */
	ocs_lock_t			auth_cfg_lock;
};

#define ocs_is_fc_initiator_enabled()		(ocs->enable_ini)
#define ocs_is_fc_target_enabled()		(ocs->enable_tgt)
#define ocs_worker_indefinite_wait_enabled()	(ocs->worker_poll_no_tmo)

#define OCS_SIGNAL_WAIT_TMO_NONE		-1
#define OCS_SIGNAL_WAIT_TMO_INTR_THREAD		100000

static inline void
ocs_device_lock_init(ocs_t *ocs)
{
	ocs_rlock_init(ocs, &ocs->lock, "ocsdevicelock");
}

static inline void
ocs_device_lock_free(ocs_t *ocs)
{
	ocs_rlock_free(&ocs->lock);
}

static inline int32_t
ocs_device_lock_try(ocs_t *ocs)
{
	return ocs_rlock_try(&ocs->lock);
}

static inline int32_t
ocs_device_lock_try_timeout(ocs_t *ocs, uint32_t timeout_ms, uint8_t retries)
{
	uint8_t	i;
	bool lock_acquired = FALSE;

	for (i = 0; i < retries; i++) {
		if (ocs_rlock_try_timeout(&ocs->lock, timeout_ms)) {
			lock_acquired = TRUE;
			break;
		}

		ocs_log_info(ocs, "device lock timeout. Retrying(%d) attempts\n", (i + 1));
	}

	return lock_acquired;
}

static inline void
ocs_device_lock(ocs_t *ocs)
{
	ocs_rlock_acquire(&ocs->lock);
}

static inline void
ocs_device_unlock(ocs_t *ocs)
{
	ocs_rlock_release(&ocs->lock);
}

extern ocs_t *ocs_get_instance(uint32_t index);
extern int32_t ocs_get_bus_dev_func(ocs_t *ocs, uint8_t* bus, uint8_t* dev, uint8_t* func);

static inline ocs_io_t *
ocs_io_alloc(ocs_t *ocs, ocs_io_pool_type_e pool_type)
{
	ocs_io_t *io;

	if (pool_type == OCS_IO_POOL_ELS)
		io = ocs_io_pool_io_alloc(ocs->xport->els_io_pool);
	else
		io = ocs_io_pool_io_alloc(ocs->xport->io_pool);

	if (io)
		io->io_pool_type = pool_type;

	return io;
}

static inline void
ocs_io_free(ocs_t *ocs, ocs_io_t *io)
{
	ocs_assert(io->io_type != OCS_IO_TYPE_MAX);
 
	/* Release to respective IO pool based on the type */
	if (io->io_pool_type == OCS_IO_POOL_ELS)
		ocs_io_pool_io_free(ocs->xport->els_io_pool, io);
	else
		ocs_io_pool_io_free(ocs->xport->io_pool, io);
}
static inline bool
ocs_bbcr_enabled(ocs_t *ocs)
{
	/* FW should be capable & module param must be set */
	return (ocs->hal.sli.config.enable_bbcr && ocs->enable_bbcr);
}

static inline bool
ocs_tgt_nvme_enabled(ocs_t *ocs)
{
	return ((ocs->tgt_fc_types & OCS_TARGET_TYPE_NVME && ocs->enable_tgt) ? 1 : 0);
}

static inline bool
ocs_tgt_scsi_enabled(ocs_t *ocs)
{
	return ((ocs->tgt_fc_types & OCS_TARGET_TYPE_FCP && ocs->enable_tgt) ? 1 : 0);
}

static inline bool
ocs_ini_nvme_enabled(ocs_t *ocs)
{
	return ((ocs->ini_fc_types & OCS_INITIATOR_TYPE_NVME && ocs->enable_ini) ? 1 : 0);
}

static inline bool
ocs_ini_scsi_enabled(ocs_t *ocs)
{
	return ((ocs->ini_fc_types & OCS_INITIATOR_TYPE_FCP && ocs->enable_ini) ? 1 : 0);
}

static inline bool
ocs_nvme_protocol_enabled(ocs_t *ocs)
{
	return (ocs_ini_nvme_enabled(ocs) || ocs_tgt_nvme_enabled(ocs));
}

static inline bool
ocs_scsi_protocol_enabled(ocs_t *ocs)
{
	return (ocs_ini_scsi_enabled(ocs) || ocs_tgt_scsi_enabled(ocs));
}

static inline bool
ocs_dual_protocol_enabled(ocs_t *ocs)
{
	return (ocs_nvme_protocol_enabled(ocs) && ocs_scsi_protocol_enabled(ocs));
}

extern void ocs_stop_event_processing(ocs_os_t *ocs_os);
extern int32_t ocs_start_event_processing(ocs_os_t *ocs_os);
extern void ocs_notify_link_state_change(ocs_t *ocs, uint8_t link_state);

extern uint16_t ocs_sriov_get_max_nr_vfs(ocs_t *ocs);
extern uint16_t ocs_sriov_get_nr_vfs(ocs_t *ocs);
extern bool ocs_sriov_config_required(ocs_t *ocs);
extern int32_t ocs_set_soft_wwpn(ocs_t *ocs, const char *buf, size_t count);
extern int32_t ocs_set_soft_wwnn(ocs_t *ocs, const char *buf, size_t count);

/* useful for logging WWN */
#define WWN_FMT         "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x"
#define WWN_ARGS(x)     x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7]

#include "ocs_domain.h"
#include "ocs_sport.h"

#endif // __OCS_DRV_FC_H__
