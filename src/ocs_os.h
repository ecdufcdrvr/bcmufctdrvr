/*
 * Copyright (c) 2011-2015, Emulex
 * All rights reserved.
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
 * userspace specific headers common to the driver
 */
#ifndef _OCS_OS_H
#define _OCS_OS_H

#if defined(OCS_SHARED_LIB)
#define OCS_LINKAGE_SYM __attribute__((__visibility__("default")))
#else
#define OCS_LINKAGE_SYM
#endif

#define OCS_HAS_TLS

typedef struct ocs_s ocs_t;

/***************************************************************************
 * OS specific includes
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <inttypes.h>
#include <unistd.h>
#include <asm-generic/unistd.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <sys/time.h>
#include <sys/syscall.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <poll.h>
#include <semaphore.h>
#include <execinfo.h>
#include <linux/types.h>
//TODO: include when user space NUMA is implemented
//#include <numa.h>

#include "ocs_ramlog.h"

#include "ocs_list.h"
#include "ocs_uspace.h"
#include "ocsu_ioctl.h"
#include "dslab.h"
#include <stdbool.h>

/**
 * @ingroup os
 * @typedef ocs_os_handle_t
 * @brief OS specific handle or driver context
 *
 * This can be anything from a void * to some other OS specific type. The lower
 * layers make no assumption about its value and pass it back as the first
 * parameter to most OS functions.
 */
typedef void * ocs_os_handle_t;

#include "ocsu_ioctl.h"
#ifdef OCS_USPACE_SPDK
#include "ocs_impl_spdk.h"
#else
#include "ocs_impl_uspace.h"
#endif
#include "ocs_debug.h"

#ifndef UINT16_MAX
#define UINT16_MAX		(65535U)
#endif

#ifndef INT32_MAX
#define INT32_MAX		(2147483647)
#endif

#ifndef UINT32_MAX
#define UINT32_MAX		(4294967295U)
#endif

#ifndef ULONG_MAX
#define ULONG_MAX		(0xFFFFFFFFFFFFFFFF)
#endif

#define KERN_ERR		"Error: "

#define max(a,b)		((a) > (b) ? (a) : (b))
#define min(a,b)		((a) < (b) ? (a) : (b))
#define roundup(x,y)		((((x) + ((y)-1)) / (y)) * (y))

#define TRUE			1
#define FALSE			0

#ifndef OCS_USPACE
#define OCS_USPACE
#endif

/* Linux driver specific definitions */

#define OCS_MIN_DMA_ALIGNMENT		16
#define OCS_MAX_DMA_ALLOC		(64*1024)	/* maxium DMA allocation that is expected to reliably succeed  */
#define OCS_FW_DUMP_CHUNK_SIZE		(16*1024)       /* FW dump DMA allocation size */

#define OCS_MAX_LUN			256
#define OCS_NUM_UNSOLICITED_FRAMES	1024

/* Per driver instance (ocs_t) definitions */
#define OCS_MAX_DOMAINS			1		/* maximum number of domains */
#define OCS_MAX_REMOTE_NODES		2048		/* maximum number of remote nodes */
//#define OCS_DEBUG_MEMORY

extern void ocs_print_stack(void);

#define ocs_abort(...)			exit(-1)

/* These datatypes are used prolifically in the linux kernel */
typedef uint8_t		u8;
typedef uint16_t	u16;
typedef uint32_t	u32;
typedef uint64_t	u64;

/* Crypto lib API - not supported currently in uspace */
/*
 *  MPI - multi precision integer (aka BN - big number) arithmetic API helper
 */
static inline int
ocs_mpi_powm(u8 *base, u32 base_len, u8 *exp, u32 exp_len,
	     u8 *mod, u32 mod_len, u8 *ans, u32 ans_len)
{
	return -EOPNOTSUPP;
}


/*
 * Hash digest API
 */
static inline int
ocs_hash(char *alg, u8 *val1, u32 val1_len, u8 *val2, u32 val2_len,
	 u8 *val3, u32 val3_len, u8 *result, u32 result_len)
{
	return -EOPNOTSUPP;
}

static inline bool
ocs_gpl_api_enabled(void)
{
#ifdef OCS_CONFIG_GPL_API
	return true;
#else
	return false;
#endif
}

/*
 * Macros used to size the CQ hash table. We want to round up to the next
 * power of 2 for the hash.
 */
#define B2(x)   (   (x) | (   (x) >> 1) )
#define B4(x)   ( B2(x) | ( B2(x) >> 2) )
#define B8(x)   ( B4(x) | ( B4(x) >> 4) )
#define B16(x)  ( B8(x) | ( B8(x) >> 8) )
#define B32(x)  (B16(x) | (B16(x) >>16) )
#define B32_NEXT_POWER_OF_2(x)      (B32((x)-1) + 1)


/***************************************************************************
 * OS abstraction
 */

/*
 * Include definitions for the State Machines used internally
 */
#include "ocs_sm.h"

/**
 * @brief Min/Max macros
 *
 */
#define OCS_MAX(x, y)		((x) > (y) ? (x) : (y))
#define OCS_MIN(x, y)		((x) < (y) ? (x) : (y))

/**
 * @brief Set the Nth bit
 *
 * @todo move to a private file used internally?
 */
#ifndef BIT
#define BIT(n)		(1U << (n))
#endif

/**
 * @brief Get minimum of two values
 */
#if defined(MIN)
#undef MIN
#endif
#define MIN(x, y)	((x) < (y) ? (x) : (y))


/***************************************************************************
 * Platform specific operations
 */

/**
 * @ingroup os
 * @brief return the lower 32-bits of a bus address
 *
 * @param addr Physical or bus address to convert
 *
 * @note this may be a good cadidate for an inline or macro
 */
static inline uint32_t ocs_addr32_lo(uintptr_t addr)
{
#if defined(__LP64__)
	return (uint32_t)(addr & 0xffffffffUL);
#else
	return addr;
#endif
}

/**
 * @ingroup os
 * @brief return the upper 32-bits of a bus address
 *
 * @param addr Physical or bus address to convert
 *
 * @note this may be a good cadidate for an inline or macro
 */
static inline uint32_t ocs_addr32_hi(uintptr_t addr)
{
#if defined(__LP64__)
	return (uint32_t)(addr >> 32);
#else
	return 0;
#endif
}

/**
 * @ingroup os
 * @brief return the log2(val)
 *
 * @param val number to use (assumed to be exact power of 2)
 *
 * @return log base 2 of val
 */
static inline uint32_t ocs_lg2(uint32_t val)
{
#if defined(__GNUC__)
	/*
	 * clz = "count leading zero's"
	 *
	 * Assuming val is an exact power of 2, the most significant bit
	 * will be the log base 2 of val
	 */
	return 31 - __builtin_clz(val);
#else
#error You need to provide a non-GCC version of this function
#endif
}

/**
 * @ingroup os
 * @brief convert a big endian 32 bit value to the host's native format
 *
 * @param val 32 bit big endian value
 *
 * @return value converted to the host's native endianess
 */
static inline uint32_t ocs_swap32(uint32_t val)
{
	return  ((val >> 24) & 0x000000ff) |
		((val >>  8) & 0x0000ff00) |
		((val <<  8) & 0x00ff0000) |
		((val << 24) & 0xff000000);
}

static inline uint64_t ocs_swap64(uint64_t val)
{
	return (((uint64_t) ocs_swap32((uint32_t)(val))) << 32ll) | ((uint64_t) ocs_swap32((uint32_t)(val >> 32ll)));
}

static inline uint32_t ocs_swap16(uint16_t val)
{
	return  ((val >> 8) & 0x00ff) |
		((val << 8) & 0xff00);
}


/**
 * @ingroup os
 * @brief optimization barrier
 *
 * Optimization barrier. Prevents compiler re-ordering
 * instructions across barrier.
 *
 * @return none
 */

#define ocs_barrier()	asm volatile("" : : : "memory")

#define ocs_be32toh(val)	ocs_swap32(val)

/**
 * @ingroup os
 * @brief convert a 32 bit value from the host's native format to big endian
 *
 * @param val 32 bit native endian value
 *
 * @return value converted to big endian
 */
#define ocs_htobe32(val)	ocs_swap32(val)

/**
 * @ingroup os
 * @brief convert a big endian 16 bit value to the host's native format
 *
 * @param val 16 bit big endian value
 *
 * @return value converted to the host's native endianess
 */
#define ocs_be16toh(val)	ocs_swap16(val)

/**
 * @ingroup os
 * @brief convert a 16 bit value from the host's native format to big endian
 *
 * @param val 16 bit native endian value
 *
 * @return value converted to big endian
 */
#define ocs_htobe16(val)	ocs_swap16(val)

#define ocs_htobe64(val)	ocs_swap64(val)
#define ocs_be64toh(val)	ocs_swap64(val)


/**
 * @ingroup os
 * @brief Copy length number of bytes from the source to destination address
 *
 * @param dst pointer to the destination memory
 * @param src pointer to the source memory
 * @param len number of bytes to copy
 *
 * @return original value of dst pointer
 */
#define ocs_memcpy(dst, src, len)	memcpy(dst, src, len)
#define ocs_memcmp(dst, src, len)	memcmp(dst, src, len)
#define ocs_strlen(src)			strlen((char*)src)
#define ocs_strnlen(src, len)		strnlen((char*)src, len)
#define ocs_strcmp(dst,src)		strcmp((char*)dst, (char*)src)
#define ocs_strncmp(dst, src, n)	strncmp((char*)dst, (char*)src, n)
#define ocs_strcasecmp(dst, src)	strcasecmp((char*)dst, (char*)src)
#define ocs_strcpy(dst,src)		strcpy((char*)dst, (char*)src)
#define ocs_strncpy(dst, src, n)	do{\
					strncpy((char*)dst, (char*)src, n); \
					if(dst[n-1] != '\0')\
						ocs_log_warn(NULL, "Not a NULL-terminated string!\n"); \
					}while(0)
#define ocs_strcat(dst, src)		strcat((char*)dst, (char*)src)
#define ocs_strstr(h,n)			strstr(h,n)
#define ocs_strsep(h, n)		strsep(h, n)
#define ocs_strchr(src,c)		strchr(src,c)
#define ocs_strtoul(src,ep,b)		strtoul((char*)src,ep,b)
#define ocs_strtoull(src,ep,b)		strtoull((char*)src,ep,b)
#define ocs_atoi(s)			strtol(s, 0, 0)
#define ocs_copy_from_user(dst, src, n) (memcpy((char*)dst, (char*)src, n), 0)
#define ocs_copy_to_user(dst, src, n)	(memcpy((char*)dst, (char*)src, n), 0)
#define ocs_snprintf(buf, n, fmt, ...)	ocs_safe_snprintf((char*)buf, n, fmt, ##__VA_ARGS__)
#define ocs_vsnprintf(buf, n, fmt, ap)	vsnprintf(((char*)buf), n, fmt, ap)
#define ocs_sscanf(buf,fmt, ...)	sscanf(buf, fmt, ##__VA_ARGS__)
#define ocs_printf			printf
#define ocs_isspace(c)			isspace(c)
#define ocs_isdigit(c)			isdigit(c)
#define ocs_isxdigit(c)			isxdigit(c)
#define ocs_strdup			strdup

extern uint64_t ocs_get_tsc(void);
char *ocs_get_options(const char *str, int nints, int *ints);

/**
 * @ingroup os
 * @brief Set the value of each byte in memory
 *
 * @param mem pointer to the memory
 * @param c value used to set memory
 * @param len number of bytes to set
 *
 * @return original value of mem pointer
 */
#define ocs_memset(mem, c, len) memset(mem, c, len)

#if !defined(OCS_USPACE_SPDK_UPSTREAM)
/*
 * SPDK Upsream picks these defines (except for LOG_TEST)
 * from /usr/include/sys/syslog.h
 */
#define LOG_CRIT	0
#define LOG_ERR		1
#define LOG_WARN	2
#define LOG_INFO	3
#define LOG_TEST	4
#define LOG_DEBUG	5
#else
#define LOG_WARN        LOG_WARNING
#define LOG_TEST	LOG_WARNING
#endif

extern const char *ocs_display_name(void *os);
extern uint32_t ocs_instance(void *os);
extern int logdest;
extern int loglevel;

extern void _ocs_log(void *os, const char *func, int line, const char *fmt, ...);

#define ocs_log_crit(os, fmt, ...)      ocs_log(os, LOG_CRIT, "CRIT: " fmt, ##__VA_ARGS__);
#define ocs_log_err(os, fmt, ...)       ocs_log(os, LOG_ERR, "ERR: " fmt, ##__VA_ARGS__);
#define ocs_log_warn(os, fmt, ...)      ocs_log(os, LOG_WARN, "WARN: " fmt, ##__VA_ARGS__);
#define ocs_log_info(os, fmt, ...)      ocs_log(os, LOG_INFO, fmt, ##__VA_ARGS__);
#define ocs_log_test(os, fmt, ...)      ocs_log(os, LOG_TEST, "TEST: " fmt, ##__VA_ARGS__);
#define ocs_log_debug(os, fmt, ...)     ocs_log(os, LOG_DEBUG, "DEBUG: " fmt, ##__VA_ARGS__);

#define ocs_log(os, level, fmt, ...) \
	do { \
		if (level <= loglevel) { \
			 _ocs_log(os, __func__, __LINE__, fmt, ##__VA_ARGS__); \
		} \
	} while(0)

#define ocs_log_crit_ratelimited(os, fmt, ...)  ocs_log_crit(os, fmt, ##__VA_ARGS__);
#define ocs_log_err_ratelimited(os, fmt, ...)   ocs_log_err(os, fmt, ##__VA_ARGS__);
#define ocs_log_warn_ratelimited(os, fmt, ...)  ocs_log_warn(os, fmt, ##__VA_ARGS__);
#define ocs_log_info_ratelimited(os, fmt, ...)  ocs_log_info(os, fmt, ##__VA_ARGS__);
#define ocs_log_test_ratelimited(os, fmt, ...)  ocs_log_test(os, fmt, ##__VA_ARGS__);
#define ocs_log_debug_ratelimited(os, fmt, ...) ocs_log_debug(os, fmt, ##__VA_ARGS__);

static inline uint32_t ocs_roundup(uint32_t x, uint32_t y)
{
	return (((x + y - 1) / y) * y);
}

static inline uint32_t ocs_rounddown(uint32_t x, uint32_t y)
{
	return ((x / y) * y);
}

/**
 * @brief OCS OS structure
 *
 * This is the OCS OS dependent structure, residing at the front of the ocs_t structure.
 *
 */
typedef struct {
	int fd;
	uint32_t pagesize;
	uint32_t bar_count;
	ocsu_memref_t  bars[PCI_MAX_BAR];
	void (*interrupt_cb)(ocs_t *ocs);
	uint8_t bus;
	uint8_t dev;
	uint8_t func;
	uint8_t num_msix;

#if defined(ENABLE_LOCK_DEBUG)
	ocs_lock_t locklist_lock;
	ocs_list_t locklist;
#endif
	int32_t numa_node;

#ifdef OCS_USPACE_SPDK
	uint64_t lcore_mask;
	struct spdk_pci_device  *spdk_pdev;
	char queue_topology[256];
	uint32_t dmabuf_next_instance;
#endif

#if defined(OCS_USPACE_SPDK_UPSTREAM)
	uint32_t num_cores;
#endif
} ocs_os_t;

/***************************************************************************
 * Memory allocation interfaces
 */

#define OCS_M_ZERO	BIT(0)
#define OCS_M_NOWAIT	BIT(1)
#define OCS_M_NONUMA	BIT(2)

#ifdef OCS_DEBUG_MEMORY
void ocs_track_memory_allocation(void* ptr, size_t size, uint8_t isDma, void*);
void ocs_track_memory_free(void* ptr, size_t size, uint8_t isDma);
#endif

/**
 * @ingroup os
 * @brief Allocate host memory
 *
 * @param os OS context
 * @param size number of bytes to allocate
 * @param flags additional options
 *
 * Flags include
 *  - OCS_M_ZERO zero memory after allocating
 *  - OCS_M_NOWAIT do not block/sleep waiting for an allocation request
 *
 * @return pointer to allocated memory, NULL otherwise
 */
static inline void *
ocs_malloc(ocs_os_handle_t os, size_t size, int32_t flags)
{
	void	*ptr = NULL;

	if (os == NULL) {
		ptr = malloc(size);
	} else {
//TODO: can't really use this yet, as numa_alloc_onnode() rounds up size to system
//	PAGE_SIZE which will be too ineffienct.   Leave this as a placeholder
//	for now
//		ptr = numa_alloc_onnode(size, ((ocs_os_t*)os)->numa_node);
		ptr = malloc(size);
	}

	if ((ptr != NULL) && (flags & OCS_M_ZERO)) {
		memset(ptr, 0, size);
	}

#ifdef OCS_DEBUG_MEMORY
	ocs_track_memory_allocation(ptr, size, FALSE, __builtin_return_address(0));
#endif
	return ptr;
}

/**
 * @ingroup os
 * @brief Free host memory
 *
 * @param os OS handle
 * @param addr pointer to memory
 * @param size bytes to free
 */
static inline void
ocs_free(ocs_os_handle_t os, void *addr, size_t size)
{
#ifdef OCS_DEBUG_MEMORY
	ocs_track_memory_free(addr, size, FALSE);
#endif
	free(addr);
}

static inline void *
ocs_vmalloc(ocs_os_handle_t os, size_t size, int32_t flags)
{
	return ocs_malloc(os, size, flags);
}

static inline void
ocs_vfree(ocs_os_handle_t os, void *addr, size_t size)
{
	ocs_free(os, addr, size);
}

/**
 * @ingroup os
 * @brief generic DMA memory descriptor for driver allocations
 *
 * Memory regions ultimately used by the hardware are described using
 * this structure. All implementations must include the structure members
 * defined in the first section, and may also add their own structure
 * members in the second section.
 *
 * Note that each region described by ocs_dma_s is assumed to be physically
 * contiguous.
 */
typedef struct ocs_dma_s {
	ocs_t *ocs;		/**< device reference */
	/*
	 * OCS layer requires the following members
	 */
	void		*virt;	/**< virtual address of the memory used by the CPU */
	void		*alloc;	/**< originally allocated virtual address used to restore virt if modified */
	uintptr_t	phys;	/**< physical or bus address of the memory used by the hardware */
	size_t		size;	/**< size in bytes of the memory */
	/*
	 * Implementation specific fields allowed here
	 */
	size_t		len;	/**< application specific length */
	uint32_t	tag;	/**< tag from DMA allocation */
	dslab_item_t	*dslab_item;
} ocs_dma_t;

extern int32_t ocs_dma_init(ocs_os_handle_t os);
extern void ocs_dma_teardown(void *os);

/**
 * @ingroup os
 * @brief Returns maximum supported DMA allocation size
 *
 * @param os OS specific handle or driver context
 * @param align alignment requirement for DMA allocation
 *
 * Return maximum supported DMA allocation size, given alignment
 * requirement.
 *
 * @return maxiumum supported DMA allocation size
 */
static inline uint32_t ocs_max_dma_alloc(ocs_os_handle_t os, size_t align)
{
	return ~((uint32_t)0); /* no max */
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
extern int32_t ocs_dma_alloc(ocs_os_handle_t os, ocs_dma_t *dma, size_t size, size_t align);

/**
 * @ingroup os
 * @brief Free a DMA capable block of memory
 *
 * @param os OS specific handle or driver context
 * @param dma DMA descriptor for memory to be freed
 *
 * @return 0 if memory is de-allocated, non-zero otherwise
 */
extern int32_t ocs_dma_free(ocs_os_handle_t os, ocs_dma_t *dma);
extern int32_t ocs_dma_free_unbound(ocs_dma_t *dma);
extern int32_t ocs_dma_copy_in(ocs_dma_t *dma, void *buffer, uint32_t buffer_length);
extern int32_t ocs_dma_copy_out(ocs_dma_t *dma, void *buffer, uint32_t buffer_length);
static inline int32_t ocs_dma_valid(ocs_dma_t *dma)
{
	return (dma->size != 0);
}

#define OCS_DMASYNC_PREWRITE	BIT(0)
#define OCS_DMASYNC_POSTREAD	BIT(1)

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
 *
 * Note: we do not need to sync dma buffer since we use coherent mapping.  This function
 * becomes a no-op for user space so we don't incurr the overhead of the ioctl() call
 * to the kernel module.
 */
static inline void
ocs_dma_sync(ocs_dma_t *dma, uint32_t flags)
{
}

/**
 * @brief Return current process ID
 *
 * For userspace drivers, the process ID is the thread ID
 *
 * @return returns the thread ID
 */

typedef union {
	pid_t pid;
	uint32_t l;
} ocs_pid_t;

static inline ocs_pid_t
ocs_mkpid(void)
{
	ocs_pid_t pid;
	pid.pid = syscall(SYS_gettid);
	return pid;
}

/***************************************************************************
 * Locking
 */

/**
 * @ingroup os
 * @typedef ocs_lock_t
 * @brief Define the type used implement locking
 */
typedef struct ocs_lock_s {
	pthread_mutex_t mutex;
#if defined(ENABLE_LOCK_DEBUG)
	void		*os;
        char            name[64];
        uint8_t         inuse;
        void            *caller[1];
        ocs_pid_t       pid;
	ocs_list_link_t	link;
#endif
} ocs_lock_t;

/**
 * @ingroup os
 * @brief Initialize a lock
 *
 * @param lock lock to initialize
 * @param name string identifier for the lock
 */
static inline void
ocs_lock_init(void *osarg, ocs_lock_t *lock, const char *name, ...)
{
#if defined(ENABLE_LOCK_DEBUG)
	ocs_os_t *os = osarg;
	va_list ap;
#endif

	pthread_mutex_init(&lock->mutex, NULL);

#if defined(ENABLE_LOCK_DEBUG)
	lock->os = os;
	lock->inuse = 0;
	lock->caller[0] = NULL;
	lock->pid.l = ~0;
	va_start(ap, name);
	ocs_vsnprintf(lock->name, sizeof(lock->name), name, ap);
	va_end(ap);
	if (os != NULL) {
		pthread_mutex_lock(&os->locklist_lock.mutex);
			ocs_list_add_tail(&os->locklist, lock);
		pthread_mutex_unlock(&os->locklist_lock.mutex);
	}

#endif
}

/**
 * @ingroup os
 * @brief Free a previously allocated lock
 *
 * @param lock lock to free
 */
static inline void
ocs_lock_free(ocs_lock_t *lock)
{
#if defined(ENABLE_LOCK_DEBUG)
	ocs_os_t *os = lock->os;

	if (os != NULL) {
		pthread_mutex_lock(&os->locklist_lock.mutex);
			ocs_list_remove(&os->locklist, lock);
		pthread_mutex_unlock(&os->locklist_lock.mutex);
	}
#endif
}

/**
 * @ingroup os
 * @brief Acquire a lock
 *
 * @param lock lock to obtain
 */
static inline void
ocs_lock(ocs_lock_t *lock)
{
#if defined(ENABLE_LOCK_DEBUG)
        if (lock->inuse && (lock->pid.pid == ocs_mkpid().pid)) {
                ocs_log_debug(NULL, "ERROR: %s: lock '%s' is inuse, owner called from %p\n", lock->name,
                        lock->caller[0]);
                ocs_print_stack();
                return;
        }
#endif
	pthread_mutex_lock(&lock->mutex);
#if defined(ENABLE_LOCK_DEBUG)
        lock->inuse = 1;
        lock->pid = ocs_mkpid();
        lock->caller[0] = __builtin_return_address(0);
#endif
}

/**
 * @ingroup os
 * @brief Try to acquire a lock
 *
 * @param lock lock to obtain
 *
 * @return non-zero if lock was acquired
 */
static inline int32_t
ocs_trylock(ocs_lock_t *lock)
{
	int rc = pthread_mutex_trylock(&lock->mutex);
#if defined(ENABLE_LOCK_DEBUG)
	if (!rc) {
		lock->inuse = 1;
		lock->pid = ocs_mkpid();
		lock->caller[0] = __builtin_return_address(0);
	}
#endif
	return !rc;
}

/**
 * @ingroup os
 * @brief Release a lock
 *
 * @param lock lock to release
 */
static inline void
ocs_unlock(ocs_lock_t *lock)
{
#if defined(ENABLE_LOCK_DEBUG)
	if (!lock->inuse) {
		ocs_log_debug(NULL, "ERROR: %s: lock '%s' is not in use\n", lock->name);
		ocs_print_stack();
		return;
	}
	lock->inuse = 0;
	lock->caller[0] = NULL;
	lock->pid.l = ~0;
#endif
	pthread_mutex_unlock(&lock->mutex);
}

/**
 * @brief recursive lock structure
 *
 * The recursive lock is implemented by maintaining a thread id (pid) and a count.
 * A count value of zero means the lock is not taken.
 *
 */

typedef struct  {
	ocs_t *ocs;
	pthread_mutex_t mutex;		/**< pthread mutex */
} ocs_rlock_t;

/**
 * @brief counting semaphore
 *
 * Declaration of the counting semaphore object
 *
 */
typedef struct {
	char *name[32];
	sem_t sem;
} ocs_sem_t;

#define OCS_SEM_FOREVER		(-1)
#define OCS_SEM_TRY		(0)

extern int ocs_sem_init(ocs_sem_t *sem, int val, const char *name, ...) __attribute__((format(printf,3,4)));

static inline int
ocs_sem_p(ocs_sem_t *sem, int32_t timeout_usec)
{
	int rc;

	if (timeout_usec <= 0) {
		rc = sem_wait(&sem->sem);
	} else {
		struct timespec ts;

		/* Compute abstime for timeout */
		clock_gettime (CLOCK_REALTIME, & ts);
		ts.tv_sec += (timeout_usec / 1000000);
		ts.tv_nsec += (timeout_usec % 1000000);
		if (ts.tv_nsec >= 1000000000l) {
			ts.tv_sec++;
			ts.tv_nsec -= 1000000000l;
		}
		rc = sem_timedwait(&sem->sem, &ts);

	}

	return rc;
}

static inline void
ocs_sem_v(ocs_sem_t *sem)
{
	(void)sem_post(&sem->sem);
}

extern void ocs_rlock_init(ocs_t *ocs, ocs_rlock_t *lock, const char *name);
extern void ocs_rlock_free(ocs_rlock_t *lock);
extern int32_t ocs_rlock_try(ocs_rlock_t *lock);
extern int32_t ocs_rlock_try_timeout(ocs_rlock_t *lock, uint32_t timeout_ms);
extern void ocs_rlock_acquire(ocs_rlock_t *lock);
extern void ocs_rlock_release(ocs_rlock_t *lock);

/***************************************************************************
 * Bitmap
 */

typedef uint32_t ocs_bitmap_t;

/**
 * @ingroup os
 * @brief Allocate a bitmap
 *
 * @param n_bits Minimum number of entries in the bit-map
 *
 * @return pointer to the bit-map or NULL on error
 */

#define BITMAP_BITS_PER_WORD		(sizeof(ocs_bitmap_t)*8)

static inline ocs_bitmap_t*
ocs_bitmap_alloc(uint32_t n_bits)
{
	void *bmap;
	uint32_t nwords = (n_bits + BITMAP_BITS_PER_WORD - 1) / BITMAP_BITS_PER_WORD;
	uint32_t nbytes = nwords * sizeof(uint32_t);

	bmap = malloc(nbytes);
	if (bmap == NULL) {
		printf("Error: %s: malloc failed\n", __func__);
		return NULL;
	}
	memset(bmap, 0, nbytes);
	return bmap;
}

/**
 * @ingroup os
 * @brief Free a bit-map
 *
 * @param bitmap Bit-map to free
 */
static inline void
ocs_bitmap_free(ocs_bitmap_t *bitmap)
{
	if (bitmap) {
		free(bitmap);
	}
}

/**
 * @ingroup os
 * @brief search for next (un)set bit
 *
 * @param bitmap bit map to search
 * @param set search for a set or unset bit
 * @param n_bits number of bits in map
 *
 * @return bit position or -1
 */
static inline int32_t
ocs_bitmap_search(ocs_bitmap_t *bitmap, uint8_t set, uint32_t n_bits)
{
	uint32_t i;
	ocs_bitmap_t setmask;
	ocs_bitmap_t mask;
	ocs_bitmap_t v;

	setmask =  (set) ? 0 : ~0;

	for (i = 0; i < n_bits; i += BITMAP_BITS_PER_WORD) {
		v = (*bitmap ++) ^ setmask;
		if (v) {
			for (mask = 1; mask; i ++, mask <<= 1) {
				if (mask & v) {
					return i;
				}
			}
		}
	}

	return -1;
}

/**
 * @ingroup os
 * @brief clear the specified bit
 *
 * @param bitmap pointer to bit map
 * @param bit bit number to clear
 */
static inline void
ocs_bitmap_clear(ocs_bitmap_t *bitmap, uint32_t bit)
{
	uint32_t idx = bit / BITMAP_BITS_PER_WORD;
	ocs_bitmap_t mask = (1U << (bit % BITMAP_BITS_PER_WORD));
	bitmap[idx] &= ~mask;
}

/**
 * @ingroup os
 * @brief set the specified bit
 *
 * @param bitmap pointer to bit map
 * @param bit bit number to set
 */
static inline void
ocs_bitmap_set(ocs_bitmap_t *bitmap, uint32_t bit)
{
	uint32_t idx = bit / BITMAP_BITS_PER_WORD;
	ocs_bitmap_t mask = (1U << (bit % BITMAP_BITS_PER_WORD));
	bitmap[idx] |= mask;
}

/**
 * @ingroup os
 * @brief Find next unset bit and set it
 *
 * @param bitmap bit map to search
 * @param n_bits number of bits in map
 *
 * @return bit position or -1 if map is full
 */
static inline int32_t
ocs_bitmap_find(ocs_bitmap_t *bitmap, uint32_t n_bits)
{
	int idx = ocs_bitmap_search(bitmap, FALSE, n_bits);

	if (idx >= 0) {
		ocs_bitmap_set(bitmap, idx);
	}
	return idx;
}

extern int32_t ocs_get_property(const char *prop_name, char *buffer, uint32_t buffer_len);


/***************************************************************************
 * Time Routines
 *
 * Functions for getting times
 */

/**
 * @ingroup os
 * @brief Get time of day in msec
 *
 * @return time of day in msec
 */
static inline time_t
ocs_msectime(void)
{
	struct timeval tv;

	gettimeofday(&tv, NULL);
	return (tv.tv_sec*1000) + (tv.tv_usec / 1000);
}

/**
 * @ingroup os
 * @brief Get time of day in usec
 *
 * @return time of day in usec
 */
static inline time_t
ocs_usectime(void)
{
	struct timeval tv;

	gettimeofday(&tv, NULL);
	return (tv.tv_sec*1000000) + (tv.tv_usec);
}

/***************************************************************************
 * Atomics
 *
 */

typedef struct ocs_atomic_s {
	int32_t value;
	ocs_lock_t lock;
} ocs_atomic_t;

/**
 * @ingroup os
 * @brief initialize an atomic
 *
 * @param a    pointer to the atomic object
 * @param v    initial value
 *
 * @return none
 */
static inline void
ocs_atomic_init(ocs_atomic_t *a, int v)
{
	ocs_lock_init(NULL, &a->lock, "atomic_lock");
	a->value = v;
}

/**
 * @ingroup os
 * @brief adds an integer to an atomic value
 *
 * @param a    pointer to the atomic object
 * @param v    value to increment
 *
 * @return the value of the atomic before incrementing.
 */
static inline int
ocs_atomic_add_return(ocs_atomic_t *a, int v)
{
	uint32_t _v;
	ocs_lock(&a->lock);
		_v = a->value;
		a->value += v;
	ocs_unlock(&a->lock);
	return _v;
}

/**
 * @ingroup os
 * @brief subtracts an integer to an atomic value
 *
 * @param a    pointer to the atomic object
 * @param v    value to increment
 *
 * @return the value of the atomic before subtracting.
 */
#define ocs_atomic_sub_return(a, v)	ocs_atomic_add_return(a, (-v))

/**
 * @ingroup os
 * @brief subtract value and test
 *
 * @param a    pointer to the atomic object
 * @param i    decrement value
 *
 * @return non-zero if subtraction resulted in a value of zero;
 * 0 otherwise.
 */
static inline int
ocs_atomic_sub_and_test(ocs_atomic_t *a, int i)
{
	int v;
	ocs_lock(&a->lock);
		a->value -= i;
		v = a->value;
	ocs_unlock(&a->lock);
	return (v == 0);
}

/**
 * @ingroup os
 * @brief Add to refcount if current value != to given value.
 *
 * @param a    pointer to the atomic object
 * @param i    value to be added
 * @param v    if current value equal to 'v' do not add
 *
 * @return non-zero if refcount was increment occurred; 0
 * otherwise.
 */
static inline int
ocs_atomic_add_unless(ocs_atomic_t *a, int i, int v)
{
	int rc = 0;
	ocs_lock(&a->lock);
		if (a->value != v) {
			a->value += i;
			rc = 1;
		}
	ocs_unlock(&a->lock);
	return rc;
}

/**
 * @ingroup os
 * @brief Sets atomic to 0, returns previous value
 *
 * @param a    pointer to the atomic object
 *
 * @return the value of the atomic before the operation.
 */
static inline int
ocs_atomic_read_and_clear(ocs_atomic_t *a)
{
	uint32_t _v;
	ocs_lock(&a->lock);
		_v = a->value;
		a->value = 0;
	ocs_unlock(&a->lock);
	return _v;
}

/**
 * @ingroup os
 * @brief returns the current value of an atomic object
 *
 * @param a    pointer to the atomic object
 *
 * @return the value of the atomic.
 */
static inline int
ocs_atomic_read(ocs_atomic_t *a)
{
	int _v;
	ocs_lock(&a->lock);
		_v = a->value;
	ocs_unlock(&a->lock);
	return _v;
}

/**
 * @ingroup os
 * @brief sets the current value of an atomic object
 *
 * @param a    pointer to the atomic object
 * @param v    value to store
 */
#define ocs_atomic_set(a, v)		(a)->value = v


#define ARRAY_SIZE(x)	(sizeof(x) / sizeof(x[0]))

/***************************************************************************
 * Message Queues
 *
 */

/**
 * @brief OCS message queue message
 *
 */

typedef struct ocs_mqueue_hdr_s {
	ocs_list_link_t link;			/**< linked list link */
	void *msgdata;				/**< message data (payload) */
} ocs_mqueue_hdr_t;

/**
 * @brief OCS message queue object
 *
 * The OCS message queue may be used to pass messages between two threads (or an ISR and thread).
 * A message is defined here as a pointer to an instance of application specific data (message data).
 * The message queue allocates a message header, saves the message data pointer, and places the
 * header on the message queue's linked list.   A counting semaphore is used to synchronize access
 * to the message queue consumer.
 *
 */

typedef struct ocs_mqueue_s {
	ocs_os_handle_t os;
	ocs_lock_t lock;		/**< message queue lock */
	ocs_sem_t prod_sem;		/**< producer semaphore */
	ocs_list_t queue;		/**< list of messages */
} ocs_mqueue_t;

/**
 * @brief initialize an OCS message queue
 *
 * The elements of the message queue  are initialized
 *
 * @param q pointer to message queue
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

static inline int32_t
ocs_mqueue_init(ocs_os_handle_t os, ocs_mqueue_t *q)
{
	ocs_memset(q, 0, sizeof(*q));
	ocs_lock_init(NULL, &q->lock, "mqueue_lock");
	ocs_sem_init(&q->prod_sem, 0, "mqueue_prod");
	ocs_list_init(&q->queue, ocs_mqueue_hdr_t, link);
	q->os = os;
	return 0;
}

/**
 * @brief put a message in a message queue
 *
 * A message header is allocated, it's payload set to point to the requested message data, and the
 * header posted to the message queue.
 *
 * @param q pointer to message queue
 * @param msgdata pointer to message data
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

static inline int32_t
ocs_mqueue_put(ocs_mqueue_t *q, void *msgdata)
{
	ocs_mqueue_hdr_t *hdr = NULL;

	hdr = ocs_malloc(q->os, sizeof(*hdr), OCS_M_NOWAIT | OCS_M_ZERO);

	if (hdr == NULL) {
		ocs_log_err(NULL, "ocs_malloc() failed\n");
		return -1;
	}

	hdr->msgdata = msgdata;

	/* lock the queue wide lock, add to tail of linked list and increment the semaphore */
	ocs_lock(&q->lock);
		ocs_list_add_tail(&q->queue, hdr);
	ocs_unlock(&q->lock);
	ocs_sem_v(&q->prod_sem);
	return 0;
}

/**
 * @brief read next message
 *
 * Reads next message header from the message queue, or times out.  The timeout_usec value
 * if zero will try one time, if negative will try forever, and if positive will try
 * for that many micro-seconds.
 *
 * @param q pointer to message queue
 * @param timeout_usec timeout (0 - try once, < 0 try forever, > 0 try micro-seconds)
 *
 * @return returns pointer to next message, or NULL
 */

static inline void *
ocs_mqueue_get(ocs_mqueue_t *q, int32_t timeout_usec)
{
	int32_t rc;
	ocs_mqueue_hdr_t *hdr = NULL;
	void *msgdata = NULL;

	if (q == NULL) {
		ocs_log_err(NULL, "q is NULL\n");
		return NULL;
	}

	rc = ocs_sem_p(&q->prod_sem, timeout_usec);
	if (rc != 0) {
		return NULL;
	}

	ocs_lock(&q->lock);
	if (!ocs_list_empty(&q->queue)) {
		hdr = ocs_list_get_head(&q->queue);
		ocs_list_remove_head(&q->queue);
	}
	ocs_unlock(&q->lock);
	if (hdr != NULL) {
		msgdata = hdr->msgdata;
		ocs_free(q->os, hdr, sizeof(*hdr));
	}
	return msgdata;
}

/**
 * @brief free an OCS message queue
 *
 * The message queue and its resources are free'd.   In this case, the message queue is
 * drained, and all the messages free'd
 *
 * @param q pointer to message queue
 *
 * @return none
 */

static inline void
ocs_mqueue_free(ocs_mqueue_t *q)
{
	ocs_mqueue_hdr_t *hdr;
	ocs_mqueue_hdr_t *next;

	ocs_lock(&q->lock);
	ocs_list_foreach_safe(&q->queue, hdr, next) {
		ocs_log_test(NULL, "Warning: freeing message queue, payload %p may leak\n", hdr->msgdata);
		ocs_free(q->os, hdr, sizeof(*hdr));
	}
	ocs_unlock(&q->lock);
}

/***************************************************************************
 * Threading
 *
 */

/**
 * @brief OCS thread structure
 *
 */

typedef struct ocs_thread_s ocs_thread_t;

typedef int32_t (*ocs_thread_fctn)(ocs_thread_t *mythread);

struct ocs_thread_s  {
	ocs_os_handle_t os;
	pthread_t thr;
	ocs_thread_fctn fctn;
	const char *name;			/*<< name of thread */
	pid_t tid;				/*<< process ID */
	uint32_t cpu_affinity;			/*<< cpu affinity */
	void *arg;				/*<< pointer to thread argument */
	uint32_t dont_start:1,			/*<< don't start */
		terminate_req:1;		/*<< terminate request */
	ocs_sem_t sem;				/*<< start/stop semaphore */
	int32_t retval;				/*<< return value */
	ocs_list_link_t link;
};

/**
 * @brief OCS thread start options
 *
 */

typedef enum {
	OCS_THREAD_RUN,				/*<< run immediately */
	OCS_THREAD_CREATE,			/*<< create and wait for start request */
} ocs_thread_start_e;


extern void ocs_thread_init(void);		/*<< called once at start of day */
extern int32_t ocs_thread_create(ocs_os_handle_t os, ocs_thread_t *thread, ocs_thread_fctn fctn,
				 const char *name, void *arg, ocs_thread_start_e start_option);
extern int32_t ocs_thread_start(ocs_thread_t *thread);

enum {	OCS_THREAD_PRIORITY_HIGH,
	OCS_THREAD_PRIORITY_NORMAL,
	OCS_THREAD_PRIORITY_LOW
};

extern int32_t ocs_thread_join(ocs_thread_t *thread);
extern int32_t ocs_thread_set_priority(ocs_thread_t *thread, uint32_t priority);
extern int32_t ocs_thread_setcpu(ocs_thread_t *thread, uint32_t cpu);
extern void *ocs_thread_get_arg(ocs_thread_t *thread);
extern int32_t ocs_thread_terminate(ocs_thread_t *thread);
extern int32_t ocs_thread_terminate_requested(ocs_thread_t *thread);
extern int32_t ocs_thread_get_retval(ocs_thread_t *thread);
extern void ocs_thread_yield(ocs_thread_t *thread);
extern ocs_thread_t *ocs_thread_self(void);

/***************************************************************************
 * Random numbers
 *
 */
static inline void ocs_get_random_bytes (uint8_t *buff, uint32_t nbytes)
{
	uint32_t i;
	uint32_t r_off;
	int rvalue;

	r_off = 0;
	rvalue = rand();
	for (i = 0; i < nbytes; i++) {
		buff[i] = ((rvalue >> (r_off * 8)) & 0xFF);
		r_off++;
		if (r_off == sizeof(int) - 1) {
			r_off = 0;
			rvalue = rand();
		}
	}
}

#define	ocs_rand()				rand()

/***************************************************************************
 * PCI
 *
 * Several functions below refer to a "register set". This is one or
 * more PCI BARs that constitute a PCI address. For example, if a MMIO
 * region is described using both BAR[0] and BAR[1], the combination of
 * BARs defines register set 0.
 */

#define PCI_MAX_BAR				6

#define PCI_VENDOR_EMULEX       		0x10df  /* Emulex */
#define PCI_VENDOR_SERVERENGINES		0x19a2	/* ServerEngines */
#define PCI_VENDOR_ATTO                         0x117c  /* ATTO   */ 

#define PCI_PRODUCT_EMULEX_OCE11102N		0x0710	/* OneCore 10Gb NIC (be3) */
#define PCI_PRODUCT_EMULEX_OCE11102F		0x0714	/* OneCore 10Gb FCoE (be3) */
#define PCI_PRODUCT_EMULEX_OCE14000		0x0724  /* OneCore 10Gb FCoE (be4) */
#define PCI_PRODUCT_EMULEX_OCE16001		0xe200	/* OneCore 16Gb FC (lancer) */
#define PCI_PRODUCT_EMULEX_OCE16002		0xe200	/* OneCore 16Gb FC (lancer) */
#define PCI_PRODUCT_EMULEX_OCE1600_VF		0xe208
#define PCI_PRODUCT_EMULEX_OCE50102		0xe260	/* OneCore FCoE (lancer) */
#define PCI_PRODUCT_EMULEX_OCE50102_VF		0xe268
#define PCI_PRODUCT_BE3_INI			0x0712  /* OneCore 10Gb iSCSI (be3) */
#define PCI_PRODUCT_BE3_TGT			0x0713  /* OneCore 10Gb iSCSI (be3) */
#define PCI_PRODUCT_ATTO_LPE31004               0x0094  /* LightPulse 32Gb x 2 FC (lancer-g6) */

/* OneCore 10Gb/40Gb iSCSI (skyhawk) Initiator
 */
#define PCI_PRODUCT_SH_INI			0x0722

/* OneCore 10Gb/40Gb iSCSI (skyhawk) Target
 */
#define PCI_PRODUCT_SH_TGT			0x0723

//@@@TODO: Add PCI Ids for Single Function I+T Skyhawk chip (mostly 0x722)??

/**
 * @ingroup os
 * @brief Get the PCI bus, device, and function values
 *
 * @param ocs OS specific handle or driver context
 * @param bus Pointer to location to store the bus number.
 * @param dev Pointer to location to store the device number.
 * @param func Pointer to location to store the function number.
 *
 * @return Returns 0.
 */
extern int32_t
ocs_get_bus_dev_func(ocs_t *ocs, uint8_t* bus, uint8_t* dev, uint8_t* func);


/**
 * @ingroup os
 * @brief Read a PCI register
 *
 * @param os OS specific handle or driver context
 * @param rset Which "register set" to use
 * @param off  Register offset
 *
 * @return 32 bit conents of the register
 */
extern uint32_t ocs_reg_read32(ocs_os_handle_t os, uint32_t rset, uint32_t off);

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
extern uint16_t ocs_reg_read16(ocs_os_handle_t os, uint32_t rset, uint32_t off);

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
extern uint8_t ocs_reg_read8(ocs_os_handle_t os, uint32_t rset, uint32_t off);

/**
 * @ingroup os
 * @brief Write a PCI register
 *
 * @param os OS specific handle or driver context
 * @param rset Which "register set" to use
 * @param off  Register offset
 * @param val  32-bit value to write
 */
extern void ocs_reg_write32(ocs_os_handle_t os, uint32_t rset, uint32_t off, uint32_t val);

/**
 * @ingroup os
 * @brief Write a PCI register
 *
 * @param os OS specific handle or driver context
 * @param rset Which "register set" to use
 * @param off  Register offset
 * @param val  16-bit value to write
 */
extern void ocs_reg_write16(ocs_os_handle_t os, uint32_t rset, uint32_t off, uint16_t val);

/**
 * @ingroup os
 * @brief Write a PCI register
 *
 * @param os OS specific handle or driver context
 * @param rset Which "register set" to use
 * @param off  Register offset
 * @param val  8-bit value to write
 */
extern void ocs_reg_write8(ocs_os_handle_t os, uint32_t rset, uint32_t off, uint8_t val);


/**
 * @ingroup os
 * @brief Return model string
 *
 * @param os OS specific handle or driver context
 */
extern const char *ocs_pci_model(uint16_t vendor, uint16_t device);


extern uint32_t ocs_instance(void *os);

/***************************************************************************
 * Reference counting
 *
 */

/**
 * @ingroup os
 * @brief reference counter object
 */
typedef void (*ocs_ref_release_t)(void *arg);
typedef struct ocs_ref_s {
	ocs_ref_release_t release; /* release function to call */
	void *arg;
	ocs_atomic_t count;
} ocs_ref_t;

/**
 * @ingroup os
 * @brief initialize given reference object
 *
 * @param ref Pointer to reference object
 * @param release Function to be called when count is 0.
 * @param arg Argument to be passed to release function.
 */
static inline void
ocs_ref_init(ocs_ref_t *ref, ocs_ref_release_t release, void *arg)
{
	// assert(release != NULL) ?
	ref->release = release;
	ref->arg = arg;
	ocs_atomic_init(&ref->count, 1);
}

/**
 * @ingroup os
 * @brief Return reference count value
 *
 * @param ref Pointer to reference object
 *
 * @return Count value of given reference object
 */
static inline uint32_t
ocs_ref_read_count(ocs_ref_t *ref)
{
	return ocs_atomic_read(&ref->count);
}

/**
 * @ingroup os
 * @brief Set count on given reference object to a value.
 *
 * @param ref Pointer to reference object
 * @param i Set count to this value
 */
static inline void
ocs_ref_set(ocs_ref_t *ref, int i)
{
	ocs_atomic_set(&ref->count, i);
}

/**
 * @ingroup os
 * @brief Take a reference on given object.
 *
 * @par Description
 * This function takes a reference on an object.
 *
 * Note: this function should only be used if the caller can
 * guarantee that the reference count is >= 1 and will stay >= 1
 * for the duration of this call (i.e. won't go to zero). If it
 * can't (the refcount may go to zero during this call),
 * ocs_ref_get_unless_zero() should be used instead.
 *
 * @param ref Pointer to reference object
 *
 */
static inline void
ocs_ref_get(ocs_ref_t *ref)
{
	ocs_atomic_add_return(&ref->count, 1);
}

/**
 * @ingroup os
 * @brief Take a reference on given object if count is not zero.
 *
 * @par Description
 * This function takes a reference on an object if and only if
 * the given reference object is "active" or valid.
 *
 * @param ref Pointer to reference object
 *
 * @return non-zero if "get" succeeded; Return zero if ref count
 * is zero.
 */
static inline uint32_t
ocs_ref_get_unless_zero(ocs_ref_t *ref)
{
	return ocs_atomic_add_unless(&ref->count, 1, 0);
}

/**
 * @ingroup os
 * @brief Decrement reference on given object
 *
 * @par Description
 * This function decrements the reference count on the given
 * reference object. If the reference count becomes zero, the
 * "release" function (set during "init" time) is called.
 *
 * @param ref Pointer to reference object
 *
 * @return non-zero if release function was called; zero
 * otherwise.
 */
static inline uint32_t
ocs_ref_put(ocs_ref_t *ref)
{
	uint32_t rc = 0;
	if (ocs_atomic_sub_and_test(&ref->count, 1)) {
		ref->release(ref->arg);
		rc = 1;
	}
	return rc;
}

/**
 * @ingroup os
 * @brief Decrement reference on given object, override release
 * function
 *
 * @par Description
 * This function decrements the reference count on the given
 * reference object. If the reference count becomes zero, the
 * given "release" function and argument is called, rather than
 * the release/arg set at ocs_ref_init() time.
 *
 * @param ref Pointer to reference object
 * @param release Function to be called if count becomes 0.
 * @param arg Argument to be passed to release function.
 *
 * @return non-zero if release function was called; zero
 * otherwise.
 */
static inline uint32_t
ocs_ref_put_release(ocs_ref_t *ref, ocs_ref_release_t release, void *arg)
{
	uint32_t rc = 0;
	if (ocs_atomic_sub_and_test(&ref->count, 1)) {
		release(arg);
		rc = 1;
	}
	return rc;
}

/**
 * @ingroup os
 * @brief Get the OS system ticks
 *
 * @return number of ticks that have occurred since the system
 * booted.
 */
static inline uint64_t
ocs_get_os_ticks(void)
{
	/*
	 * Since we can't get jiffies or HZ from userpace, return msec in
	 * ocs_get_os_ticks() and return 1000 for ocs_get_os_tick_freq()
	 */
	return ocs_msectime();
}

/**
 * @ingroup os
 * @brief Convert the OS system ticks to ms.
 *
 * @param number of ticks that have occured since the system booted.
 *
 * @return the milisecond value for the number of ticks that have occurred since the system
 * booted.
 */
static inline uint64_t ocs_ticks_to_ms(uint64_t t)
{
        return t;
}

/**
 * @ingroup os
 * @brief Convert the OS system ms to ticks
 *
 *
 * @return the ticks value for the number of ms
 */
static inline uint64_t ocs_ms_to_ticks(uint64_t msec)
{
	return msec;
}


/**
 * @ingroup os
 * @brief Get the OS system tick frequency
 *
 * @return frequency of system ticks.
 */
static inline uint32_t
ocs_get_os_tick_freq(void)
{
	/*
	 * Since we can't get jiffies or HZ from userpace, return msec in
	 * ocs_get_os_ticks() and return 1000 for ocs_get_os_tick_freq()
	 */
	return 1000;
}

/*
 * @brief Return HW completion context mode
 *
 * @return true for interrupt context mode, false for thread context mode.
 */
static inline bool ocs_in_interrupt_context(void)
{
	return FALSE;
}

#ifndef OCS_USPACE_SPDK
/**
 * @ingroup os
 * @brief Delay execution by the given number of micro-seconds
 *
 * @param usec number of micro-seconds to "busy-wait"
 *
 * @note The value of usec may be greater than 1,000,000
 */
#define ocs_udelay(usec) usleep(usec)
#define ocs_msleep(msec) usleep(msec*1000)
#define ocs_delay_msec(msec) ocs_msleep(msec)

/**
 * @ocs_delay_usec
 * @brief Do delay/sleep based on the atomic context
 *
 * @param Pointer to ocs context
 * @param delay in usec
 */
static inline void ocs_delay_usec(unsigned int udelay)
{
	if (ocs_in_interrupt_context())
		ocs_udelay(udelay);
	else
		ocs_msleep(udelay/1000);
}
#endif

extern int ocs_drain_shutdown_events(ocs_t *ocs, ocs_sem_t *sem);

static inline int
ocs_safe_snprintf(char *buf, int n, const char *format, ...)
{
	va_list args;
	int length;

	va_start(args, format);

	length = vsnprintf(buf, n, format, args);
	if(length < 0)
		 ocs_log_err(NULL, "vsnprintf failed \n");

	va_end(args);
	return (length);

}

#include "ocs_pool.h"
#include "ocs_cbuf.h"
#include "ocs_common.h"

#if defined(__GNUC__) && __GNUC__ >= 7
#define FALL_THROUGH __attribute__ ((fallthrough))
#else
#define FALL_THROUGH ((void)0)
#endif /* __GNUC__ >= 7 */

#define FD_INVALID -1

#define OCS_FD_CLOSE(fd) \
	do { \
		if (close(fd) < 0) { \
			ocs_log_warn(NULL, "fd close failed"); \
		} else { \
			(fd) = FD_INVALID; \
		} \
	} while (0)

#endif /* !_OCS_OS_H */

/* vim: set noexpandtab textwidth=120: */
