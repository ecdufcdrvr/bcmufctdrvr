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
 * OCS userspace driver OS specific API declarations
 *
 */

#include "ocs.h"
#include "ocs_spdk.h"
#include "ocs_impl_spdk.h"
#include "spdk/env.h"
#include "spdk/event.h"

/* @brief Select DMA buffer allocation method
 */
#define ENABLE_DMABUF_SLAB		1
#define ENABLE_DMABUF_USER		0

#if ENABLE_DMABUF_SLAB
typedef struct {
	uint32_t tag;
	char name[64];
} ocs_dslab_app_t;

/**
 * @brief Allocate a dma buffer slab
 *
 * Allocates a dma buffer slab
 *
 * @param os pointer to OS context
 * @param dma pointer to dma slab to allocate
 * @param len length of this dma slab
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int
ocsu_dslab_dmabuf_alloc(void *os, dslab_dmabuf_t *dma, uint32_t len)
{
	ocs_t *ocs = os;

#ifndef OCS_USPACE_SPDK 
	ocsu_mmoffset_t mmoffset;
#endif
	ocs_dslab_app_t *app;

	ocs_memset(dma, 0, sizeof(*dma));

	/* Allocate the app specific data */
	app = calloc(1, sizeof(*app));
	if (app == NULL) {
		ocs_log_err(ocs, "malloc failed\n");
		return -1;
	}
	snprintf(app->name, sizeof(app->name), "ocs%d-ocs_dma_buff-%d",
			ocs->instance_index, ocs->ocs_os.dmabuf_next_instance);
	
	/* Submit a driver request to allocate a buffer */
	dma->vaddr = ocs_spdk_zmalloc(app->name, len, 64, &(dma->paddr));
	if (dma->vaddr == NULL) {
		free(app);
		ocs_log_err(ocs, "mmap failed\n");
		return -1;
	}

	/* Fill in the rest of the dlab dma buffer information */
	dma->app  = app;
	dma->size = len;
	app->tag  = ocs->ocs_os.dmabuf_next_instance;

	ocs->ocs_os.dmabuf_next_instance ++;
	return 0;
}

static void
ocsu_dslab_dmabuf_free(void *os, dslab_dmabuf_t *dma)
{
	ocs_assert(dma);

	/* Don't free if virtual address is NULL */
	if (dma->vaddr) {
		/* Unmap the address range */
		ocs_spdk_free(dma->vaddr);

		free(dma->app);
		memset(dma, 0, sizeof(*dma));
	}

	return;
}

dslab_callbacks_t ocs_dslab_callbacks = {
	ocsu_dslab_dmabuf_alloc,
	ocsu_dslab_dmabuf_free
};
#endif

/* Timer routines */
/* Start a micro-sec timer */
int32_t
ocs_setup_timer_us(ocs_os_handle_t os, ocs_timer_t *timer,
		   void (*func)(void *arg), void *data, uint32_t timeout_us)
{
        struct sigevent evt;
        struct itimerspec its;
        int rc;

        memset(&evt, 0, sizeof(evt));
        evt.sigev_notify = SIGEV_THREAD;
        evt.sigev_notify_function = (void *)func;
        evt.sigev_value.sival_ptr = data;

        if ((rc = timer_create(CLOCK_REALTIME, &evt, &timer->timer)) < 0) {
                ocs_log_err(os, "Timer create failed rc=%d errno=%s\n", rc, strerror(errno));
                return rc;
        }
	timer->is_set = TRUE;

	/* if the timeout value is 0, leave the timer disarmed */
	if (!timeout_us)
		return 0;

        /* Start timer */
        memset(&its, 0, sizeof(its));
	its.it_value.tv_sec = timeout_us / 1000000;
	its.it_value.tv_nsec = (timeout_us % 1000000) * 1000l;

        rc = timer_settime(timer->timer, 0, &its, NULL);
        if (rc) {
                ocs_log_err(os, "timer_settime failed: errno=%s\n", strerror(errno));
                return rc;
        }

	return 0;
}

/* Start a milli-sec timer */
int32_t
ocs_setup_timer(ocs_os_handle_t os, ocs_timer_t *timer, void(*func)(void *arg),
		void *data, uint32_t timeout_ms, bool sync)
{
	return ocs_setup_timer_us(os, timer, func, data, timeout_ms * 1000);
}

int32_t
ocs_mod_timer_us(ocs_timer_t *timer, uint32_t timeout_us)
{
        struct itimerspec its;
	int rc;

        /* Start timer */
        memset(&its, 0, sizeof(its));
        its.it_value.tv_sec = timeout_us / 1000000;
	its.it_value.tv_nsec = (timeout_us % 1000000) * 1000l;

        rc = timer_settime(timer->timer, 0, &its, NULL);
        if (rc) {
                printf("%s: Error: timer_settime failed: %s\n",
		       __func__, strerror(errno));
		timer->is_set = false;
                return rc;
        }
	timer->is_set = TRUE;

	return 0;
}

int32_t
ocs_mod_timer(ocs_timer_t *timer, uint32_t timeout_ms)
{
	return ocs_mod_timer_us(timer, timeout_ms * 1000);
}

int32_t
ocs_timer_pending(ocs_timer_t *timer)
{
	return timer->is_set;
}

int32_t
ocs_del_timer(ocs_timer_t *timer)
{
	int rc;

	if (!timer->is_set)
		return 0;

	timer->is_set = false;
	rc = timer_delete(timer->timer);
	if (rc)
		printf("ocs_del_timer: error=%d\n", rc);
	return rc;
}

/**
 * @ingroup os
 * @brief Read a 32 bit value from the specified configuration register
 *
 * @param os OS specific handle or driver context
 * @param reg_offset register offset
 *
 * @return The 32 bit value
 */
uint32_t
ocs_config_read32(void *os, uint32_t off)
{
	ocs_t		*ocs = os;
	uint32_t data;
	spdk_pci_device_cfg_read32(ocs->ocs_os.spdk_pdev, &data, off);
	return data;
}

/**
 * @ingroup os
 * @brief Write a 32 bit value to the specified configuration
 *        register
 *
 * @param os OS specific handle or driver context
 * @param off register offset
 * @param val value to write
 *
 * @return None
 */
void
ocs_config_write32(void *os, uint32_t off, uint32_t val)
{
	ocs_t		*ocs = os;
	spdk_pci_device_cfg_write32(ocs->ocs_os.spdk_pdev, val, off);
}

int32_t
ocs_thread_getcpu(void)
{
	return sched_getcpu();
}

/**
 * @brief return CPU information
 *
 * This function populates the ocs_cpuinfo_t buffer with CPU information
 *
 * @param cpuinfo pointer to ocs_cpuinfo_t buffer
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocs_get_cpuinfo(ocs_cpuinfo_t *cpuinfo)
{
	ocs_memset(cpuinfo, 0, sizeof(*cpuinfo));
	cpuinfo->num_cpus = sysconf(_SC_NPROCESSORS_CONF);
	cpuinfo->hyper = FALSE;
	return 0;
}

uint32_t
ocs_get_num_cpus(void)
{
	static ocs_cpuinfo_t cpuinfo;

	if (cpuinfo.num_cpus == 0) {
		ocs_get_cpuinfo(&cpuinfo);
	}
	return cpuinfo.num_cpus;
}

int32_t
ocs_scsi_tgt_io_init(ocs_io_t *io)
{
        return 0;
}

int32_t
ocs_scsi_tgt_io_exit(ocs_io_t *io)
{
        return 0;
}

uint32_t
ocs_sched_cpu(unsigned i)
{
	/*
	 * ocs_sched_cpu is called to allot ELS EQ processing.
	 * Upstream ELS EQ is designed to be processed from
	 * a dedicated pthread.
	 * Naturally, pthread context can run from any core.
	 */
	return spdk_env_get_last_core();
}

int32_t
ocs_scsi_get_block_vaddr(ocs_io_t *io, uint64_t blocknumber, ocs_scsi_vaddr_len_t addrlen[],
        uint32_t max_addrlen, void **dif_vaddr)
{
        return -1;
}

/**
 *  *  * @brief Cancel HAL IOs
 *   *   */
void ocs_scsi_tgt_cancel_io(ocs_t *ocs)
{
        /* Flush the WCQE's before cleaning-up the io_inuse list */
        ocs_hal_flush(&ocs->hal);
        ocs_hal_io_cancel(&ocs->hal);
}

int32_t ocs_scsi_tgt_port_online(ocs_t *ocs)
{
	int rc;
	ocs_log_debug(ocs, "Restore port ONLINE\n");
	rc = ocs_xport_control(ocs->xport, OCS_XPORT_PORT_ONLINE);
	if (rc)
		ocs_log_err(ocs, "Failed to bring port online\n");

	return rc;
}
