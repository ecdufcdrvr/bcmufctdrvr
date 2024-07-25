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

#ifndef __ocs_IMPL_H__
#define __ocs_IMPL_H__

#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <rte_config.h>
#include <rte_malloc.h>
#include <rte_atomic.h>
#include <rte_cycles.h>
#include <rte_version.h>
#include <rte_bus_pci.h>
#include <stdbool.h>

#include "spdk/env.h"
#include "env_internal.h"

/**
 * \file
 *
 * This file describes the functions required to integrate
 * the userspace OCS driver for a specific implementation.  This
 * implementation is specific for DPDK.  Users would revise it as
 * necessary for their own particular environment if not using it
 * within the SPDK framework.
 */
/*
 * OCS_OS_MAX_ISR_TIME_MSEC -  maximum time driver code should spend in an interrupt
 * or kernel thread context without yielding
 */
#define OCS_OS_MAX_ISR_TIME_MSEC	100

#define ocs_delay_usec(usec)	usleep(usec)
#define ocs_msleep(msec)	usleep(msec*1000)
#define ocs_delay_msec(msec)	ocs_msleep(msec)

/**
 * Free a memory buffer previously allocated with ocsu_zmalloc.
 */
#define ocs_spdk_free(buff)		spdk_dma_free(buff)

/**
 * Return the physical address for the specified virtual address.
 */
#define ocs_vtophys(buf)		spdk_vtophys(buf, NULL)

/**
 * Delay us.
 */
#define ocs_delay_us(us)		rte_delay_us(us)

/**
 * Assert a condition and panic/abort as desired.  Failures of these
 *  assertions indicate catastrophic failures within the driver.
 */
#define ocs_spdk_assert(check)		assert(check)

/**
 * Log or print a message from the driver.
 */
#define ocs_spdk_printf(chan, fmt, args...) printf(fmt, ##args)

/*****************************************************************************
 *
 * CPU topology API
 **/

typedef struct {
	uint32_t num_cpus;	/* Number of CPU cores */
	bool hyper;		/* TRUE if threaded CPUs */
} ocs_cpuinfo_t;

extern int32_t ocs_get_cpuinfo(ocs_cpuinfo_t *cpuinfo);
extern uint32_t ocs_get_num_cpus(void);
extern dslab_callbacks_t ocs_dslab_callbacks;

/**
 * Allocate a pinned, physically contiguous memory buffer with the
 * given size and alignment.
 */
static inline void *
ocs_spdk_zmalloc(const char *tag, size_t size, unsigned align, uint64_t *phys_addr)
{
	void *buf_ptr = NULL;
	if (tag) {
		buf_ptr = spdk_dma_zmalloc(size, align, NULL);
		if (buf_ptr) {
			*phys_addr = spdk_vtophys(buf_ptr, &size);
		}
	}
	else {
		ocs_spdk_printf(NULL, "ocs_zmalloc() call without a tag!\n");
	}
	return buf_ptr;
}

#define ocs_pcicfg_read32(handle, var, offset)  spdk_pci_device_cfg_read32(handle, var, offset)
#define ocs_pcicfg_write32(handle, var, offset) spdk_pci_device_cfg_write32(handle, var, offset)

typedef pthread_mutex_t ocs_mutex_t;

#define ocs_mutex_lock pthread_mutex_lock
#define ocs_mutex_unlock pthread_mutex_unlock
#define OCS_MUTEX_INITIALIZER PTHREAD_MUTEX_INITIALIZER

/***************************************************************************
 * Timer Routines
 *
 * Functions for setting, querying and canceling timers.
 */
typedef struct {
	uint8_t is_set;
	timer_t timer;
} ocs_timer_t;

/**
 * @ingroup os
 * @brief Initialize and set a timer
 *
 * @param os OS handle
 * @param timer    pointer to the structure allocated for this timer
 * @param func     the function to call when the timer expires
 * @param data     Data to pass to the provided timer function when the timer
 *                 expires.
 * @param timeout_ms the timeout in milliseconds
 */
extern int32_t ocs_setup_timer(ocs_os_handle_t os, ocs_timer_t *timer,
		void(*func)(void *arg), void *data, uint32_t timeout_ms, bool sync);

/**
 * @ingroup os
 * @brief Initialize and set a micro-sec timer
 *
 * @param os OS handle
 * @param timer    pointer to the structure allocated for this timer
 * @param func     the function to call when the timer expires
 * @param data     Data to pass to the provided timer function when the timer
 *                 expires.
 * @param timeout_us the timeout in micro-seconds
 */
int32_t
ocs_setup_timer_us(ocs_os_handle_t os, ocs_timer_t *timer,
		   void (*func)(void *arg), void *data, uint32_t timeout_us);

/**
 * @ingroup os
 * @brief Modify a timer's expiration
 *
 * @param timer    pointer to the structure allocated for this timer
 * @param timeout_msec    the timeout in milliseconds
 */
extern int32_t ocs_mod_timer(ocs_timer_t *timer, uint32_t timeout_ms);
extern int32_t ocs_mod_timer_us(ocs_timer_t *timer, uint32_t timeout_us);


/**
 * @ingroup os
 * @brief Queries to see if a timer is pending.
 *
 * @param timer    pointer to the structure allocated for this timer
 *
 * @return non-zero if the timer is pending
 */
extern int32_t ocs_timer_pending(ocs_timer_t *timer);

/**
 * @ingroup os
 * @brief Remove a pending timer
 *
 * @param timer    pointer to the structure allocated for this timer
 *                 expires.
 */
extern int32_t ocs_del_timer(ocs_timer_t *timer);

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
ocs_config_read32(void *os, uint32_t off);

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
ocs_config_write32(void *os, uint32_t off, uint32_t val);

extern int32_t ocs_thread_getcpu(void);
extern uint32_t ocs_sched_cpu(unsigned instance);

#endif /* __OCS_IMPL_H__ */
