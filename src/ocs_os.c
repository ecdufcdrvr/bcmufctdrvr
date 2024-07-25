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
 * OCS userspace driver OS specific API declarations
 *
 */

#include "ocs.h"
#include <sys/utsname.h>

static ocs_list_t ocs_thread_list;
static ocs_lock_t ocs_thread_list_lock;
static void *ocs_pthread_start(void *arg);
pid_t gettid(void);
int32_t ocsu_process_events(ocs_t *ocs);

#if defined(ENABLE_DMABUF_SLAB)
int32_t ocs_dma_alloc_slab(void *os, ocs_dma_t *dma, size_t size, size_t align, uint8_t flags);
int32_t ocs_dma_free_slab(void *os, ocs_dma_t *dma);

static int
ocsu_dslab_init(ocs_t *ocs, uint32_t entry_count, uint32_t max_item_len)
{
	ocs->drv_ocs.slabdir = dslab_dir_new(ocs, &ocs_dslab_callbacks, entry_count, max_item_len);
	if (ocs->drv_ocs.slabdir == NULL) {
		ocs_log_err(ocs, "dslab_dir_new() failed\n");
		return -1;
	}
	return 0;
}

static void
ocsu_dslab_teardown(ocs_t *ocs)
{
	dslab_dir_del(ocs->drv_ocs.slabdir);
}

/**
 * @ingroup os
 * @brief Allocate a DMA capable block of memory
 *
 * @param os OS specific handle or driver context
 * @param dma DMA descriptor containing results of memory allocation
 * @param size Size in bytes of desired allocation
 * @param align Alignment in bytes of the requested allocation
 *
 * @return 0 on success, non-zero otherwise
 */
int32_t
ocs_dma_alloc_slab(void *os, ocs_dma_t *dma, size_t size, size_t align, uint8_t flags)
{
	ocs_t *ocs = os;

	ocs_assert(ocs, -1);
	ocs_assert(ocs->drv_ocs.slabdir, -1);
	ocs_assert(dma, -1);

	ocs_memset(dma, 0, sizeof(*dma));

	dslab_item_t *item = dslab_item_new(ocs->drv_ocs.slabdir, size, align);
	if (item == NULL) {
		ocs_log_err(ocs, "dslab_item_new() failed\n");
		return -1;
	}

	dma->ocs = ocs;
	dma->dslab_item = item;
	dma->phys = item->paddr;
	dma->virt = item->vaddr;
	dma->alloc = dma->virt;
	dma->size = size; // this is the allocaetd size: item->size;
	dma->len = dma->size;

	return 0;
}

/**
 * @ingroup os
 * @brief Free a DMA capable block of memory
 *
 * @param os OS specific handle or driver context
 * @param dma DMA descriptor for memory to be freed
 *
 * @return 0 if memory is de-allocated, non-zero otherwise
 */
int32_t
ocs_dma_free_slab(void *os, ocs_dma_t *dma)
{
	if (!os)
		return -EPERM;

	if (dma->virt) {
		dslab_item_del(dma->dslab_item);
		ocs_memset(dma, 0, sizeof(*dma));
	}
	return 0;
}
#endif

#if defined(ENABLE_DMABUF_KERNEL)
int32_t
ocs_dma_alloc_kernel(void *os, ocs_dma_t *dma, size_t size, size_t align, uint8_t flags)
{
	ocs_t *ocs = os;
	int rc = -1;
	ocsu_ioctl_dmabuf_alloc_t req;
	ocsu_mmoffset_t mmoffset;

	ocs_memset(dma, 0, sizeof(*dma));
	ocs_memset(&req, 0, sizeof(req));
	req.req_size = size;
	rc = ioctl(ocs->ocs_os.fd, OCSU_IOCTL_CMD_DMABUF_ALLOC, &req);
	if (rc) {
		ocs_log_err(ocs, "ioctl/DMA_ALLOC failed: %d\n", rc);
		return -1;
	}

	memset(&mmoffset, 0, sizeof(mmoffset));
	mmoffset.u.t.mm_type = OCSU_MMOFFSET_DMA;
	mmoffset.u.t.mm_type_specific = req.tag;
	dma->virt = mmap(NULL, roundup(size, ocs->ocs_os.pagesize), PROT_READ | PROT_WRITE, MAP_SHARED,
		ocs->ocs_os.fd, mmoffset.u.l);
	if (dma->virt == MAP_FAILED) {
		ocs_log_err(ocs, "mmap failed\n");
		ioctl(ocs->ocs_os.fd, OCSU_IOCTL_CMD_DMABUF_FREE, req.tag);
		return -1;
	}
	dma->alloc = dma->virt;
	dma->ocs = ocs;
	dma->phys = req.paddr;
	dma->size = size;
	dma->len = size;
	dma->tag = req.tag;

	return 0;
}

int32_t
ocs_dma_free_kernel(void *os, ocs_dma_t *dma)
{
	ocs_t *ocs = os;
	ocsu_ioctl_dmabuf_free_t req;
	int rc = 0;

	if (!os)
		return -EPERM;

	if (dma->virt) {
		rc = munmap(dma->virt, roundup(dma->size, ocs->ocs_os.pagesize));
		if (rc) {
			ocs_log_err(ocs, "Error: %s: munmap failed: %d\n", rc);
		}

		memset(&req, 0, sizeof(req));
		req.tag = dma->tag;
		rc = ioctl(ocs->ocs_os.fd, OCSU_IOCTL_CMD_DMABUF_FREE, &req);
		if (rc) {
			ocs_log_err(ocs, "Error: %s: ioctl/DMABUF_FREE failed: %d\n", rc);
		}

		memset(dma, 0, sizeof(*dma));
	}

	return rc;
}

/**
 * @ingroup os
 * @brief Synchronize the DMA buffer memory
 *
 * Ensures memory coherency between the CPU and device
 *
 * @param dma DMA descriptor of memory to synchronize
 * @param flags Describes direction of synchronization
 *   - OCS_DMASYNC_PREREAD sync needed before hardware updates host memory
 *   - OCS_DMASYNC_PREWRITE sync needed after CPU updates host memory but before hardware can access
 *   - OCS_DMASYNC_POSTREAD sync needed after hardware updates host memory but before CPU can access
 *   - OCS_DMASYNC_POSTWRITE sync needed after hardware updates host memory
 */
void
ocs_dma_sync(ocs_dma_t *dma, uint32_t flags)
{
	ocs_t *ocs = dma->ocs;
	ocsu_ioctl_dmabuf_sync_t req;
	int rc;

	memset(&req, 0, sizeof(req));
	switch (flags) {
	case OCS_DMASYNC_POSTREAD:
		req.flags = OCSU_DMASYNC_POSTREAD;
		break;
	case OCS_DMASYNC_PREWRITE:
		req.flags = OCSU_DMASYNC_PREWRITE;
		break;
	default:
		printf("%s: bad flags %#x\n", __func__, flags);
		return;
	}
	req.paddr = dma->phys;
	req.size = dma->size;
	rc = ioctl(ocs->ocs_os.fd, OCSU_IOCTL_CMD_DMABUF_SYNC, &req);
	if (rc) {
		ocs_log_err(ocs, "Error: ioctl/DMABUF_SYNC failed: %d\n", rc);
	}
	return;
}
#endif

#if defined(ENABLE_DMABUF_SPDK)
int32_t ocs_dma_alloc_spdk(void *os, ocs_dma_t *dma, size_t size, size_t align, uint8_t flags);
int32_t ocs_dma_free_spdk(void *os, ocs_dma_t *dma);

/**
 * @ingroup os
 * @brief Allocate a DMA capable block of memory
 *
 * @param os OS specific handle or driver context
 * @param dma DMA descriptor containing results of memory allocation
 * @param size Size in bytes of desired allocation
 * @param align Alignment in bytes of the requested allocation
 *
 * @return 0 on success, non-zero otherwise
 */
int32_t
ocs_dma_alloc_spdk(void *os, ocs_dma_t *dma, size_t size, size_t align, uint8_t flags)
{
	ocs_t *ocs = os;
	uint8_t name[32];

	ocs_memset(dma, 0, sizeof(*dma));

	snprintf(name, sizeof(name), "ocs%d-ocs_dma_buff-%d",
		 ocs->instance_index, ocs->ocs_os.dmabuf_next_instance);

	/* Submit a driver request to allocate a buffer */
	dma->virt = ocs_spdk_zmalloc(name, size, align, &(dma->phys));
	if (dma->virt == NULL) {
		ocs_log_err(os, "Failed to do ocs_spdk_zmalloc\n");
		return -1;
	}

	dma->ocs = os;
	dma->alloc = dma->virt;
	dma->size = size; // this is the allocaetd size: item->size;
	dma->len = dma->size;

	return 0;
}

/**
 * @ingroup os
 * @brief Free a DMA capable block of memory
 *
 * @param os OS specific handle or driver context
 * @param dma DMA descriptor for memory to be freed
 *
 * @return 0 if memory is de-allocated, non-zero otherwise
 */
int32_t
ocs_dma_free_spdk(void *os, ocs_dma_t *dma)
{
	ocs_assert(dma, -1);

	/* Don't free if virtual address is NULL */
	if (dma->virt) {
		/* Unmap the address range */
		ocs_spdk_free(dma->virt);
		memset(dma, 0, sizeof(*dma));
	}

	return 0;
}
#endif

/**
 * @brief copy into dma buffer
 *
 * Copies into a dma buffer, updates the len element
 *
 * @param dma DMA descriptor
 * @param buffer address of buffer to copy from
 * @param buffer_length buffer length in bytes
 *
 * @return returns bytes copied for success, a negative error code value for failure.
 */
int32_t
ocs_dma_copy_in(ocs_dma_t *dma, void *buffer, uint32_t buffer_length)
{
	if (!dma)
		return -1;
	if (!buffer)
		return -1;
	if (buffer_length == 0)
		return 0;
	if (buffer_length > dma->size)
		buffer_length = dma->size;
	ocs_memcpy(dma->virt, buffer, buffer_length);
	dma->len = buffer_length;
	return buffer_length;
}

/**
 * @brief copy out of dma buffer
 *
 * Copy data from dma buffer.   No more than dma->len bytes are copied
 *
 * @param dma DMA descriptor
 * @param buffer address of buffer to copy into
 * @param buffer_length buffer length in bytes
 *
 * @return returns bytes copied for success, a negative error code value for failure.
 */

int32_t
ocs_dma_copy_out(ocs_dma_t *dma, void *buffer, uint32_t buffer_length)
{
	if (!dma)
		return -1;
	if (!buffer)
		return -1;
	if (buffer_length == 0)
		return 0;
	if (buffer_length > dma->len)
		buffer_length = dma->len;
	ocs_memcpy(buffer, dma->virt, buffer_length);
	return buffer_length;
}

/**
 * @ingroup os
 * @brief Read a PCI register
 *
 * @param os OS specific handle or driver context
 * @param rset Which "register set" to use
 * @param off  Register offset
 *
 * @return 32 bit contents of the register
 */
uint32_t
ocs_reg_read32(void *os, uint32_t rset, uint32_t off)
{
	ocs_t	*ocs = os;

	return *((uint32_t*)(ocs->ocs_os.bars[rset].vaddr + off));
}

/**
 * @ingroup os
 * @brief Read a PCI register
 *
 * @param os OS specific handle or driver context
 * @param rset Which "register set" to use
 * @param off  Register offset
 *
 * @return 16 bit conents of the register
 */
uint16_t
ocs_reg_read16(void *os, uint32_t rset, uint32_t off)
{
	ocs_t	*ocs = os;
	return *((uint16_t*)(ocs->ocs_os.bars[rset].vaddr + off));
}

/**
 * @ingroup os
 * @brief Read a PCI register
 *
 * @param os OS specific handle or driver context
 * @param rset Which "register set" to use
 * @param off  Register offset
 *
 * @return 8 bit conents of the register
 */
uint8_t
ocs_reg_read8(void *os, uint32_t rset, uint32_t off)
{
	ocs_t	*ocs = os;
	return *((uint8_t*)(ocs->ocs_os.bars[rset].vaddr + off));
}

/**
 * @ingroup os
 * @brief Write a PCI register
 *
 * @param os OS specific handle or driver context
 * @param rset Which "register set" to use
 * @param off  Register offset
 * @param val  32-bit value to write
 */
void
ocs_reg_write32(void *os, uint32_t rset, uint32_t off, uint32_t val)
{
	ocs_t	*ocs = os;

	ocs_assert(ocs);
	ocs_assert(rset < ocs->ocs_os.bar_count);
	*((uint32_t*)(ocs->ocs_os.bars[rset].vaddr + off)) = val;
}

/**
 * @ingroup os
 * @brief Write a PCI register
 *
 * @param os OS specific handle or driver context
 * @param rset Which "register set" to use
 * @param off  Register offset
 * @param val  16-bit value to write
 */
void
ocs_reg_write16(void *os, uint32_t rset, uint32_t off, uint16_t val)
{
	ocs_t	*ocs = os;
	*((uint16_t*)(ocs->ocs_os.bars[rset].vaddr + off)) = val;
}

/**
 * @ingroup os
 * @brief Write a PCI register
 *
 * @param os OS specific handle or driver context
 * @param rset Which "register set" to use
 * @param off  Register offset
 * @param val  8-bit value to write
 */
void
ocs_reg_write8(void *os, uint32_t rset, uint32_t off, uint8_t val)
{
	ocs_t	*ocs = os;
	*((uint8_t*)(ocs->ocs_os.bars[rset].vaddr + off)) = val;
}

/**
 * @brief Initialize recursive lock
 *
 * Initialize a recursive lock
 *
 * @param ocs pointer to ocs structure
 * @param lock pointer to recursive lock
 * @param lock order
 * @param name text
 *
 * @return none
 */
void
ocs_rlock_init(ocs_t *ocs, ocs_rlock_t *lock, ocs_lock_order_e order, const char *name)
{
	lock->ocs = ocs;
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&lock->mutex, &attr);
}

/**
 * @brief Initialize recursive lock
 *
 * Initialize a recursive lock
 *
 * @param ocs pointer to ocs structure
 *
 * @return none
 */
void
ocs_rlock_free(ocs_rlock_t *lock)
{
	ocs_memset(lock, 0, sizeof(*lock));
}

/**
 * @brief try to acquire a recursive lock
 *
 * Attempt to acquire a recursive lock, return TRUE if successful
 *
 * @param lock pointer to recursive lock
 *
 * @return TRUE if lock was acquired, FALSE if not
 */
int32_t
ocs_rlock_try(ocs_rlock_t *lock)
{
	int rc = pthread_mutex_trylock(&lock->mutex);

	/* A return of 0 means success */
	return (rc == 0);
}
/**
 * @brief try to acquire a recursive lock
 *
 * Attempt to acquire a recursive lock, return TRUE if successful
 *
 * @param lock pointer to recursive lock
 * @timeout_ms to speficy how much time that need to be spend in trying for lock
 *
 * @return TRUE if lock was acquired, FALSE if not
 */

int32_t
ocs_rlock_try_timeout(ocs_rlock_t *lock, uint32_t timeout_ms)
{
	uint32_t itreations = timeout_ms/10;
	int32_t	lock_acquired = FALSE;

	while (itreations && !lock_acquired) {
		lock_acquired = ocs_rlock_try(lock);
		itreations--;	
		/* wait for 10 ms before retrying for the lock */
		ocs_msleep(10);
	}
	return lock_acquired;
}

/**
 * @brief acquire recursive lock
 *
 * Acquire recursive lock, return when successful
 *
 * @param lock pointer to recursive lock
 *
 * @return none
 */

void
ocs_rlock_acquire(ocs_rlock_t *lock)
{
	pthread_mutex_lock(&lock->mutex);
}

/**
 * @brief release recursive lock
 *
 * Release a previously acquired recursive lock
 *
 * @param lock pointer to recursive lock
 *
 * @return none
 */

void
ocs_rlock_release(ocs_rlock_t *lock)
{
	pthread_mutex_unlock(&lock->mutex);
}

pid_t
gettid(void)
{
	return syscall(SYS_gettid);
}

static void *
ocs_pthread_start(void *arg)
{
	ocs_thread_t *thread = arg;

	prctl(PR_SET_NAME, thread->name);
	thread->tid = gettid();
	ocs_lock(&ocs_thread_list_lock);
		ocs_list_add_tail(&ocs_thread_list, thread);
	ocs_unlock(&ocs_thread_list_lock);

	if (thread->dont_start) {
		if (ocs_sem_p(&thread->sem, OCS_SEM_FOREVER) != 0) {
			/* Undefined failure */
			ocs_log_err(NULL, "ocs_sem_p failed\n");
			return NULL;
		}
	}

	(*thread->fctn)(thread);
	ocs_lock(&ocs_thread_list_lock);
		ocs_list_remove(&ocs_thread_list, thread);
	ocs_unlock(&ocs_thread_list_lock);
	free((char*)thread->name);
	return &thread->retval;
}

int32_t
ocs_thread_set_priority(ocs_thread_t *thread, uint32_t priority)
{
	int rc;
	struct sched_param param;
	int policy = SCHED_OTHER;

	switch(priority) {
	case OCS_THREAD_PRIORITY_HIGH:
		policy = SCHED_FIFO;
		param.sched_priority =
			((sched_get_priority_max(SCHED_FIFO) + sched_get_priority_min(SCHED_FIFO)) / 2) + 5;
		break;
	case OCS_THREAD_PRIORITY_NORMAL:
		policy = SCHED_RR;
		param.sched_priority =
			((sched_get_priority_max(SCHED_RR) + sched_get_priority_min(SCHED_RR)) / 2) - 5;
		break;
	default:
	case OCS_THREAD_PRIORITY_LOW:
		policy = SCHED_OTHER;
		param.sched_priority = (sched_get_priority_max(SCHED_OTHER) + sched_get_priority_min(SCHED_OTHER)) / 2;
		break;
	}

	rc = pthread_setschedparam(pthread_self(), policy, &param);
	if (rc) {
		ocs_log_err(NULL, "pthread_setschedparma() failed: %d %s\n", rc, strerror(errno));
	}

	return rc;
}

int32_t
ocs_thread_setcpu(ocs_thread_t *thread, uint32_t cpu)
{
	int rc;
	cpu_set_t cpuset;

	cpu = cpu % sysconf(_SC_NPROCESSORS_ONLN);

	CPU_ZERO(&cpuset);

	CPU_SET(cpu, &cpuset);

	rc = pthread_setaffinity_np(thread->thr, sizeof(cpu_set_t), &cpuset);
	if (rc) {
		printf("%s: pthread_setaffinity_np() failed: %d %s\n", __func__, rc, strerror(errno));
		return -1;
	}
	thread->cpu_affinity = cpu;
	return 0;
}

void
ocs_thread_init(void)
{
	ocs_list_init(&ocs_thread_list, ocs_thread_t, link);
	ocs_lock_init(NULL, &ocs_thread_list_lock, OCS_LOCK_ORDER_IGNORE, "ocs_thread_list_lock");
}

/**
 * @brief Create a kernel thread
 *
 * Creates a kernel thread and optionally starts it.
 *
 * @param os OS handle
 * @param thread pointer to thread object
 * @param fctn function for thread to be begin executing
 * @param name text name to identify thread
 * @param arg application specific argument passed to thread function
 * @param start_option start option, OCS_THREAD_RUN will start the thread immediately,
 *				     OCS_THREAD_CREATE will create but not start the thread
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocs_thread_create(ocs_os_handle_t os, ocs_thread_t *thread, ocs_thread_fctn fctn, const char *name, void *arg, ocs_thread_start_e start)
{
	int rc;
	pthread_attr_t attr;
	pthread_attr_init(&attr);

	thread->os = os;
	thread->fctn = fctn;
	thread->name = ocs_strdup(name);

	if (thread->name == NULL) {
		thread->name = "unknown";
	}

	thread->arg = arg;
	ocs_sem_init(&thread->sem, 0, "%s", name);

	if (start == OCS_THREAD_CREATE) {
		thread->dont_start = 1;
	}

	rc = pthread_create(&thread->thr, &attr, ocs_pthread_start, thread);
	thread->created = rc ? false : true;

	return rc;
}

/**
 * @brief Join threads
 *
 * The caller waits for a thread to exit.
 *
 * @param thread thread to wait for
 *
 * @return returns status of pthread_join command
 */
int32_t
ocs_thread_join(ocs_thread_t *thread)
{
	int32_t rc = 0;

	if (thread->created)
		rc = pthread_join(thread->thr, NULL);

	return rc;
}

/**
 * @brief Start a thread
 *
 * Starts a thread that was created with OCS_THREAD_CREATE rather than OCS_THREAD_RUN
 *
 * @param thread pointer to thread object
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

int32_t
ocs_thread_start(ocs_thread_t *thread)
{
	ocs_assert(thread->dont_start == 1, -1);
	if (thread->dont_start) {
		ocs_sem_v(&thread->sem);
	}
	return 0;
}

/**
 * @brief return thread argument
 *
 * Returns a pointer to the thread's application specific argument
 *
 * @param thread pointer to the thread object
 *
 * @return pointer to application specific argument
 */
void *
ocs_thread_get_arg(ocs_thread_t *mythread)
{
	return mythread->arg;
}

/**
 * @brief Request thread stop
 *
 * A stop request is made to the thread
 *
 * @param thread pointer to thread object
 *
 * @return returns threads return value
 */
int32_t
ocs_thread_terminate(ocs_thread_t *thread)
{
	void *pretval;

	/* set request flag */
	thread->terminate_req = 1;

	if (thread->created) {
		/* Unblock system calls in progress */
		pthread_kill(thread->thr, SIGRTMIN);

		/* wait for thread to terminate */
		pthread_join(thread->thr, &pretval);

		thread->created = false;
		if (pretval != NULL) {
			return *((int32_t*)pretval);
		}
	}

	return 0;
}

/**
 * @brief See if a terminate request has been made
 *
 * Check to see if a stop request has been made to the current thread.  This
 * function would be used by a thread to see if it should terminate.
 *
 * @return returns non-zero if a stop has been requested
 */

int32_t
ocs_thread_terminate_requested(ocs_thread_t *thread)
{
	return thread->terminate_req;
}

/**
 * @brief Retrieve threads return value
 *
 * After a thread has terminated, it's return value may be retrieved with this function.
 *
 * @param thread pointer to thread object
 *
 * @return return value from thread function
 */

int32_t
ocs_thread_get_retval(ocs_thread_t *thread)
{
	return thread->retval;
}

/**
 * @brief Request that the currently running thread yield
 *
 * The currently running thread yields to the scheduler
 *
 * @param thread pointer to thread (ignored)
 *
 * @return none
 */

void
ocs_thread_yield(ocs_thread_t *thread) {
	sched_yield();
}

ocs_thread_t*
ocs_thread_self(void)
{
	pthread_t pthr = pthread_self();
	ocs_thread_t *thr;
	static __thread ocs_thread_t *me = NULL;

	if (me == NULL) {
		ocs_lock(&ocs_thread_list_lock);
			ocs_list_foreach(&ocs_thread_list, thr) {
				if (thr->thr == pthr) {
					me = thr;
					ocs_unlock(&ocs_thread_list_lock);
					return me;
				}
			}
		ocs_unlock(&ocs_thread_list_lock);
	}
	return me;
}

int
ocs_sem_init(ocs_sem_t *sem, int val, const char *name, ...)
{
	va_list ap;

	va_start(ap, name);
	ocs_vsnprintf(sem->name, sizeof(sem->name), name, ap);
	va_end(ap);
	return sem_init(&sem->sem, 0, val);
}

/**
 * @ingroup os
 * @brief get timestamp counter
 *
 * This function reads the tsc register. This is useful for
 * measuring latencies.
 *
 * Notes:
 * To convert the tsc value into microseconds, take this value
 * and divide by the CPU frequency (in MHz), which can be found
 * on the "cpu MHz" line of /proc/cpuinfo.
 *
 * If the OS is throttling the CPU, these values will not be
 * consistent so it's best to disable CPU throttling if using
 * this function.
 *
 * @return timestamp counter
 */
uint64_t
ocs_get_tsc(void)
{
	uint32_t hi;
	uint32_t lo;
	__asm__ volatile("rdtsc" : "=a" (lo), "=d" (hi));
	return (((uint64_t)lo) | (((uint64_t)hi) << 32));
}

void ocs_print_stack(void)
{
        void *stack[20];
        int nptrs;

        nptrs = backtrace(stack, 20);

        backtrace_symbols_fd(stack, nptrs, STDOUT_FILENO);
}

int32_t
ocs_get_bus_dev_func(ocs_t *ocs, uint8_t* bus, uint8_t* dev, uint8_t* func)
{
	*bus = ocs->ocs_os.bus;
	*dev = ocs->ocs_os.dev;
	*func= ocs->ocs_os.func;
	return 0;
}

void _ocs_log(void *os, const char *func_name, int line, const char *fmt, ...)
{
	va_list ap;
	char buf[200];
	char *p = buf;

	va_start(ap, fmt);

	if (logdest & 4) {
		struct timeval tv;

		gettimeofday(&tv, NULL);

		p += snprintf(p, sizeof(buf) - (p - buf), "[%10ld.%06ld] ", tv.tv_sec, tv.tv_usec);
	}

	p += snprintf(p, sizeof(buf) - (p - buf), "[%d]: ", gettid());
	p += snprintf(p, sizeof(buf) - (p - buf), "%s: ", DRV_NAME);
	p += snprintf(p, sizeof(buf) - (p - buf), "%s:", func_name);
	p += snprintf(p, sizeof(buf) - (p - buf), "%i:", line);
	p += snprintf(p, sizeof(buf) - (p - buf), "%s", (os != NULL) ? ocs_display_name(os) : "");
	p += vsnprintf(p, sizeof(buf) - (p - buf), fmt, ap);

	va_end(ap);

	if (logdest & 1) {
		printf("%s", buf);
	}

	if (logdest & 2) {
		ocs_ramlog_printf(os, "%s", buf);
	}
	fflush(stdout);
}

void
_ocs_assert(const char *cond, const char *filename, int linenum)
{
	const char *fn = strrchr(__FILE__, '/');

	ocs_log_err(NULL, "%s(%d) assertion (%s) failed\n", (fn ? fn + 1 : filename), linenum, cond);
	ocs_print_stack();
	ocs_save_ddump_all(OCS_DDUMP_FLAGS_WQES|OCS_DDUMP_FLAGS_CQES|OCS_DDUMP_FLAGS_MQES, -1, TRUE);
}

static void
ocs_xport_domain_shutdown_expires(void *arg)
{
	ocs_t *ocs = (ocs_t *)arg;
	ocs->domain_shutdown_timedout = 1;
}

int
ocs_drain_shutdown_events(ocs_t *ocs, ocs_sem_t *sem)
{
	ocs_timer_t     domain_shutdown_timer;
	ocs_xport_t *xport = ocs->xport;
	xport->ocs->domain_shutdown_timedout = 0;

	ocs_log_debug(ocs, "Waiting %d seconds for domain shutdown.\n",
			   (OCS_FC_DOMAIN_SHUTDOWN_TIMEOUT_USEC/1000000));

	ocs_setup_timer(ocs, &domain_shutdown_timer, ocs_xport_domain_shutdown_expires,
			xport->ocs, OCS_FC_DOMAIN_SHUTDOWN_TIMEOUT_USEC/1000, false);
	/*Wait for domain to shutdown within timeout */
	while (!ocs_list_empty(&ocs->domain_list) && 
			!(xport->ocs->domain_shutdown_timedout)) {
		ocsu_process_events(ocs);
	}
	ocs_del_timer(&domain_shutdown_timer);

	return xport->ocs->domain_shutdown_timedout;
}

/**
 * @brief Get OS Host name 
 *
 * @param pointer to OCS
 * @param pointer to Host name buffer
 * @param pointer to size of Host name buffer
 *
 * @return Returns version string
 */

void
ocs_get_os_host_name(ocs_t *ocs, void *os_host_name_str, size_t size)
{
	char *hostname = (char *)os_host_name_str;

	if (gethostname(hostname, size - 1)) {
		ocs_snprintf(hostname, size, "%s", "UNKNOWN");
		return;
	}

	/*
	 * If Host name is too large, gethostname() truncates the hostname
	 * to fit in a given size. In such case null termination is not guaranteed.
	 * Append null terminate sentinel in any case.
	 */
	hostname[size-1] = '\0';
}

/**
 * @brief Get OS version and name
 *
 * @param pointer to OCS
 *
 * @return Returns version string
 */
void
ocs_get_os_version_and_name(ocs_t *ocs, void *os_ver_name_str, size_t size)
{
	struct utsname utsname;

	if (0 == uname(&utsname)) {
		ocs_snprintf(os_ver_name_str, size, "%s %s %s", utsname.sysname, utsname.release,
			     utsname.version);
	} else {
		ocs_snprintf(os_ver_name_str, size, "%s", "UNKNOWN");
	}
}

/**
 * @brief Get SCSI Host device name 
 *
 * @param pointer to OCS
 * @param host_devname pointer to update
 * @param size of input buffer
 *
 *
 * @return None
 */
void    
ocs_scsi_get_host_devname(ocs_t *ocs, void *host_devname, size_t size)
{
	ocs_snprintf(host_devname, size, "Emulex %s", ocs->businfo);
}

void
ocs_fc_encode_lun(ocs_node_t *node, uint64_t lun, uint8_t *lu)
{
	uint8_t abort_test_bits = (lun >> 24);

	lun &= 0xffffff;
	if (lun <= FCP_LUN_ADDR_SIMPLE_MAX) {
		lu[1] = lun & 0xff;
	} else if (lun <= FCP_LUN_ADDR_FLAT_MAX) {
		/*
		 * Use single level, flat space LUN
		 */
		lu[0] = (FCP_LUN_ADDR_METHOD_FLAT << FCP_LUN_ADDRESS_METHOD_SHIFT) |
			((lun >> 8) & FCP_LUN_ADDRESS_METHOD_MASK);
		lu[1] = lun & 0xff;
	} else {
		ocs_log_err(node->ocs, "unsupported LU %08lX\n", lun);
		// TODO: need an error code that indicates LUN INVALID
		return;
	}

	lu[2] = abort_test_bits;
}

uint64_t
ocs_fc_decode_lun(ocs_node_t *node, uint8_t *lu)
{
	uint64_t lun = ULONG_MAX;
	uint8_t	 address_method = -1;

	address_method = lu[0] >> FCP_LUN_ADDRESS_METHOD_SHIFT;

	switch (address_method) {
	case FCP_LUN_ADDR_METHOD_PERIPHERAL:
	{
		uint8_t	bus_identifier = lu[0] & ~FCP_LUN_ADDRESS_METHOD_MASK;

		if (0 == bus_identifier) {
			lun = lu[1];
		} else {
			ocs_log_test(node->ocs, "unsupported bus identifier %#02x (LU %02x %02x %02x %02x ...)\n",
				     bus_identifier, lu[0], lu[1], lu[2], lu[3]);
		}
		break;
	}
	case FCP_LUN_ADDR_METHOD_FLAT:
	{
		lun = (lu[0] & ~FCP_LUN_ADDRESS_METHOD_MASK) << 8 | lu[1];
		break;
	}
	case FCP_LUN_ADDR_METHOD_EXTENDED:
	{
		uint8_t	length, extended_address_method;

		length = (lu[0] & 0x30) >> 4;
		extended_address_method = lu[0] & 0xf;

		if ((1 == length) && (2 == extended_address_method)) {
			lun = (lu[1] << 16) | (lu[2] << 8) | lu[3];
		} else {
			ocs_log_test(node->ocs, "unsupported extended addressing method (length=%#x method=%#x)\n",
				     length, extended_address_method);
		}
		break;
	}
	default:
		ocs_log_test(node->ocs, "unsupported LU address method %#02x\n", address_method);
	}

	return lun;
}

/**
 *      ocs_get_options - Parse a string into a list of ints
 *      @str: String to be parsed
 *      @nints: size of integer array
 *      @ints: integer array
 *
 *      This function parses a string containing a comma-separated
 *      list of integers, a hyphen-separated range of _positive_ integers,
 *      or a combination of both, into ints[] starting from ints[1].
 *	The parse halts when the array is full, or when no more numbers
 *	can be retrieved from the string.
 *	Number of integers parsed is placed in ints[0]
 *	Returns NULL
 */
char *ocs_get_options(const char *str, int nints, int *ints)
{
	char *buf = NULL, *saveptr = NULL, *opt = NULL;
	int i;

	if (!str)
		return NULL;

	for (i = 1, buf = (char *)str; i < nints; buf = NULL, i++) {
		opt = strtok_r(buf, ",", &saveptr);
		if (!opt)
			break;
		ints[i] = ocs_strtoul(opt, 0, 0);
	}
	ints[0] = i - 1;
	return NULL;
}

int32_t
ocs_dma_init(void *os)
{
#if defined(ENABLE_DMABUF_SLAB)
	return ocsu_dslab_init((ocs_t *)os, DSLAB_MAX_SLAB_ENTRIES, DSLAB_MAX_ITEM_LEN);
#else
	return 0;
#endif
}

void
ocs_dma_teardown(void *os)
{
#if defined(ENABLE_DMABUF_SLAB)
	ocs_t *ocs = os;
	/* tear down the DMA buffer slab allocator */
	ocsu_dslab_teardown(ocs);
#endif
}

/**
 * @ingroup os
 * @brief Allocate a DMA capable block of memory
 *
 * @param os OS specific handle or driver context
 * @param dma DMA descriptor containing results of memory allocation
 * @param size Size in bytes of desired allocation
 * @param align Alignment in bytes of the requested allocation
 *
 * @return 0 on success, non-zero otherwise
 */
int32_t
ocs_dma_alloc(void *os, ocs_dma_t *dma, size_t size, size_t align, uint8_t flags)
{
#if defined(ENABLE_DMABUF_SLAB)
	return ocs_dma_alloc_slab(os, dma, size, align, flags);
#elif defined(ENABLE_DMABUF_SPDK)
	return ocs_dma_alloc_spdk(os, dma, size, align, flags);
#elif defined(ENABLE_DMABUF_KERNEL)
	return ocs_dma_alloc_kernel(os, dma, size, align, flags);
#else
	ocs_log_err(os, "Unsupported\n");
	return -1;
#endif
}

/**
 * @ingroup os
 * @brief Free a DMA capable block of memory
 *
 * @param os OS specific handle or driver context
 * @param dma DMA descriptor for memory to be freed
 *
 * @return 0 if memory is de-allocated, non-zero otherwise
 */
int32_t
ocs_dma_free(void *os, ocs_dma_t *dma)
{
#if defined(ENABLE_DMABUF_SLAB)
	return ocs_dma_free_slab(os, dma);
#elif defined(ENABLE_DMABUF_SPDK)
	return ocs_dma_free_spdk(os, dma);
#elif defined(ENABLE_DMABUF_KERNEL)
	return ocs_dma_free_kernel(os, dma);
#else
	ocs_log_err(os, "Unsupported\n");
	return -1;
#endif
}

int32_t
ocs_dma_free_unbound(ocs_dma_t *dma)
{
	return ocs_dma_free(NULL, dma);
}
