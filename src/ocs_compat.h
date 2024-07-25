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

/**
 * @file
 *
 */

#if !defined(__OCS_COMPAT_H__)
#define __OCS_COMPAT_H__

#define OCS_BE_MAJOR				0
#define OCS_BE_MINOR				0
#define OCS_BE_BUILD				9999
#define OCS_BE_BRANCH				0

/* Enable GPL license only if a feature that requires it is enabled */
#ifdef OCS_CONFIG_GPL_API
#define OCS_MODULE_LICENSE			"GPL"
#else
#define OCS_MODULE_LICENSE			"BSD"
#endif

#define OCS_FDMI_MODEL_MANUFACTURER		"Emulex Corporation"
#define OCS_FDMI_MODEL_NAME			sport->ocs->model
#define OCS_FDMI_MODEL_DESCRIPTION		sport->ocs->desc

/* Bit mask for FDMI-1 defined HBA attributes */
#define FDMI_RHBA_MASK_V1			0x000003ff

/* Bit mask for FDMI-2 defined HBA attributes */
#define FDMI_RHBA_MASK_V2			0x0002efff

#define OCS_SYSFS_CREATE(_kobj, _attr)		0
#define OCS_SYSFS_REMOVE(_kobj, _attr)

#define OCS_INIT_WORK(_work, _func)
#define OCS_SCHED_WORK(_work)
#define OCS_CANCEL_WORK(_work)

extern struct bin_attribute ocs_sysfs_fw_dump_attr;
extern struct bin_attribute ocs_sysfs_fw_error_attr;

#if !defined(OCS_USPACE)
static inline void
ocs_mm_read_lock(struct mm_struct *mm)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,8,0)
	down_read(&mm->mmap_sem);
#else
	down_read(&mm->mmap_lock);
#endif
}

static inline void
ocs_mm_read_unlock(struct mm_struct *mm)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,8,0)
	up_read(&mm->mmap_sem);
#else
	up_read(&mm->mmap_lock);
#endif
}
#endif

void ocs_handle_userapp_lost(ocs_t *ocs);
void ocs_handle_uapi_rdy(ocs_t *ocs);
void ocs_nvme_irq_notify(uint32_t eq_index, int cpu);

static inline void
ocs_io_setup_wq_steering(ocs_io_t *io, uint32_t flags)
{
	/* If no steering request is set, use request based steering as default */
	io->wq_steering = (flags & OCS_SCSI_WQ_STEERING_MASK) >> OCS_SCSI_WQ_STEERING_SHIFT;
	if (!io->wq_steering)
		io->wq_steering = OCS_SCSI_WQ_STEERING_REQUEST >> OCS_SCSI_WQ_STEERING_SHIFT;
	io->wq_class = (flags & OCS_SCSI_WQ_CLASS_MASK) >> OCS_SCSI_WQ_CLASS_SHIFT;
}

static inline uint32_t
ocs_get_cq_config_process_limit(ocs_t *ocs)
{
	return ocs->cq_process_limit;
}

static inline bool
ocs_eq_process_watch_yield(void)
{
#if !defined(OCS_USPACE)
	/* Kernel mode drivers must yield */
	return true;
#else
	return false;
#endif
}

extern bool ocs_virtfn(ocs_t *ocs);

#endif // __OCS_COMPAT_H__
