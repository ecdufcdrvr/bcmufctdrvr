/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2018 Broadcom.  All Rights Reserved.
 *  The term "Broadcom" refers to Broadcom Inc. and/or its subsidiaries.
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
 * OCS linux driver common include file
 */


#if !defined(__OCS_DRV_ISCSI_H__)
#define __OCS_DRV_ISCSI_H__

#define OCS_INCLUDE_ISCSI

#include "ocs_os.h"
#include "ocs_debug.h"
#include "ocs_iscsi_config.h"

#include "ocs_common.h"
#include "ocs_hal.h"
#include "ocs_io.h"
#include "ocs_pm.h"
#include "ocs_iscsi_api.h"
#include "ocs_iscsi.h"
#include "ocs_ioctl.h"
#include "ocs_elxu.h"

#define OCS_MAX_SPORTS 32

/**
 * @brief A structure used to contain all data related to an adapter for this driver.
 */
struct ocs_s {

	ocs_os_t ocs_os;
	char display_name[OCS_DISPLAY_NAME_LENGTH];
	ocs_rlock_t lock;			/*>> Device wide lock */
	ocs_list_t domain_list;			/*>> linked list of virtual fabric objects */
	ocs_io_pool_t *io_pool;			/**< pointer to IO pool */
	ocs_ramlog_t *ramlog;
	ocs_drv_t drv_ocs;
	ocs_scsi_tgt_t tgt_ocs;
	ocs_scsi_ini_t ini_ocs;
	ocs_xport_e ocs_xport;
	ocs_xport_t *xport;			/*>> Pointer to transport object */
	bool enable_ini;
	bool enable_tgt;
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

	uint32_t instance_index;

	uint16_t pci_vendor;
	uint16_t pci_device;
	uint16_t pci_subsystem_vendor;
	uint16_t pci_subsystem_device;
	char businfo[16];

	const char *model;
	const char *driver_version;
	const char *fw_version;

	const char *desc;

	ocs_hal_t hal;
	ocs_iscsi_t iscsi;
	struct iscsi_api_s *iscsi_api;

	ocs_domain_t *domain;				/*>> pointer to first (physical) domain (also on domain_list) */
	uint32_t domain_instance_count;			/*>> domain instance count */
	ocs_sli_port_t sport[OCS_MAX_SPORTS];

	bool enable_sgl_chaining;
	bool enable_immediate_data;
	bool enable_hdr_digest;
	bool enable_data_digest;
	bool enable_jumbo_frame;
	uint32_t max_sge_size;
	uint32_t post_sgl_config;
	uint32_t num_scsi_ios;
	uint32_t max_cxns_per_session;
	bool require_chap;

	/* Driver command line parameters */
	uint32_t num_tgts;
	uint32_t global_ramdisc;
	char	 local_ip[64];
	char	 local_subnet[64];
	char	 gateway[64];
	uint32_t tcp_window_size;
	uint8_t  tcp_window_scale;
	uint32_t num_cxns;

	ocs_atomic_t io_alloc_failed_count;	/**< used to track how often IO pool is empty */
	ocs_list_t io_pending_list;		/**< list of IOs waiting for HAL resources
						 **  lock: ocs_driver_lock()
						 **  link: ocs_io_t->io_pending_link
						 */
	ocs_atomic_t io_total_alloc;		/**< count of totals IOS allocated */
	ocs_atomic_t io_total_free;		/**< count of totals IOS free'd */
	ocs_atomic_t io_total_pending;		/**< count of totals IOS that were pended */
	ocs_atomic_t io_active_count;		/**< count of active IOS */
	ocs_atomic_t io_pending_count;		/**< count of pending IOS */
	ocs_atomic_t io_pending_recursing;	/**< non-zero if ocs_scsi_check_pending is executing */

	uint32_t target_io_timer_sec;		/**< target IO timer not supported in iSCSI */
	ocs_textbuf_t ddump_saved;
};

#define ocs_is_initiator_enabled()	(ocs->enable_ini)
#define ocs_is_target_enabled()	(ocs->enable_tgt)

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
extern int32_t ocs_device_ioctl(ocs_t *ocs, unsigned int cmd, unsigned long arg);

static inline ocs_io_t *
ocs_io_alloc(ocs_t *ocs)
{
	return ocs_io_pool_io_alloc(ocs->io_pool);
}

static inline void
ocs_io_free(ocs_t *ocs, ocs_io_t *io)
{
	ocs_io_pool_io_free(ocs->io_pool, io);
}

extern void ocs_stop_event_processing(ocs_os_t *ocs_os);
extern int32_t ocs_start_event_processing(ocs_os_t *ocs_os);

#include "ocs_xport.h"
#include "ocs_scsi.h"
#include "ocs_domain.h"
#include "ocs_sport.h"
#include "ocs_cxn.h"
#include "ocs_io.h"

#endif // __OCS_DRV_ISCSI_H__
