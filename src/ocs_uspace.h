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
 * Linux-specific headers common to the user space driver.
 */

 /**
 * @page kernel_mod_overview Kernel Module Overview
 *
 * <div class="overview">
 * <img src="elx_uspace_ocs_fc_ramd_kernel.jpg" alt="Kernel Module" title="Kernel Module" align="right"/>
 * The Uspace ocs_fc_ramd driver includes a small kernel module (ocs_uspace.ko).
 * This kernel module provides access to PCI resources, as well as managing DMA
 * memory buffers.<br> <br>
 * Kernel module functionality includes:
 *  - Read/write PCI configuration space registers
 *  - Read/write per PCI function BAR memory regions
 *  - Mapping of per PCI function BAR memory regions to user space
 *  - Allocation of and freeing of kernel resident DMA buffers, mapping buffers to
 * user space
 *  - DMA buffer synchronization (maintaining cache coherency)
 *  - Wait on interrupt
 *
 * The files for the kernel module are located in the
 * <a href="../../../../linux/ocs_uspace/" target="_blank">
 * driver/linux/ocs_uspace/</a> directory.
 * </div>
 *
 */

#if !defined(__OCS_USPACE_H__)
#define __OCS_USPACE_H__

#if defined(__KERNEL__)
#include <stdarg.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <asm-generic/ioctl.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/bitmap.h>
#include <linux/spinlock.h>
#include <asm/byteorder.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <asm/current.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include "linux/seq_file.h"
#include <asm/poll.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
#define __devinit
#define __devinitdata
#define __devexit
#define __devexit_p(x) x
#endif
#endif

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
#define PCI_PRODUCT_EMULEX_LPE31004		0xe300	/* LightPulse 16Gb x 4 FC (Lancer-G6) */
#define PCI_PRODUCT_EMULEX_LPE32002		0xe300  /* LightPulse 32Gb x 2 FC (lancer-g6) */
#define PCI_PRODUCT_BE3_INI			0x0712  /* OneCore 10Gb iSCSI (be3) */
#define PCI_PRODUCT_BE3_TGT			0x0713  /* OneCore 10Gb iSCSI (be3) */
#define PCI_PRODUCT_ATTO_LPE31004               0x0094  /* LightPulse 32Gb x 2 FC (lancer-g6) */

/* OneCore 10Gb/40Gb iSCSI (skyhawk) Initiator
 */
#define PCI_PRODUCT_SH_INI			0x0722

/* OneCore 10Gb/40Gb iSCSI (skyhawk) Target
 */
#define PCI_PRODUCT_SH_TGT			0x0723

/* Subsystem Device
 */
#define PCI_SUBDEVICE_LPE15004			0xe292  /* 4x8G FC (peregrine) */
#define PCI_SUBDEVICE_LPE16204			0xe26f  /* 2x8G FC, 2x10G FCoE Mixed mode (Sheba) */
#define PCI_SUBDEVICE_LPE31004			0xe312  /* 4x16G FC (lancer g6) */
#define PCI_SUBDEVICE_LPE31004_P		0xe380  /* 4x16G FC (lancer g6 Penelope) */
#define PCI_SUBDEVICE_LPE32002			0xe300  /* 2x32G FC (lancer g6 Grenoble) */
#define PCI_SUBDEVICE_LPE32000			0xe3f1  /* 2x32G FC (lancer g6 test) */

#define MAX_PCI_INTERRUPTS			16
#define DEFAULT_MSIX_INTERRUPTS			3

typedef struct {
	void *vaddr;
	uintptr_t paddr;
	size_t size;
} ocsu_memref_t;

/* mmap64 offset overloading */
enum {
	OCSU_MMOFFSET_BAR = 1,
	OCSU_MMOFFSET_DMA,
};

//note: little endian only
typedef struct {
	union {
		struct {
			uint64_t
				:12,
				mm_type_specific:44,
				mm_type:8;
		} t;
		uint64_t l;
	} u;
} ocsu_mmoffset_t;

typedef struct {
	uint16_t vendor;
	uint16_t device;
	uint32_t bar_count;
	ocsu_memref_t bars[PCI_MAX_BAR];
	uint8_t bus;
	uint8_t dev;
	uint8_t func;
	uint8_t num_msix;
	uint8_t numa_node;
} ocsu_ioctl_get_pci_config_t;

typedef struct {
	uint32_t req_size;
	uintptr_t paddr;
	uint32_t tag;
} ocsu_ioctl_dmabuf_alloc_t;

typedef struct {
	uint32_t tag;
} ocsu_ioctl_dmabuf_free_t;

enum {
	OCSU_DMASYNC_PREWRITE=1,
	OCSU_DMASYNC_POSTREAD,
	};

typedef struct {
	uint32_t flags;
	uintptr_t paddr;
	uint32_t size;
} ocsu_ioctl_dmabuf_sync_t;

/**
 * @brief coredump helper function command values
 */

typedef enum {
        OCSU_IOCTL_PCICFG_READ8,
        OCSU_IOCTL_PCICFG_READ16,
        OCSU_IOCTL_PCICFG_READ32,
        OCSU_IOCTL_PCICFG_WRITE8,
        OCSU_IOCTL_PCICFG_WRITE16,
        OCSU_IOCTL_PCICFG_WRITE32,
        OCSU_IOCTL_PCIBAR_READ8,
        OCSU_IOCTL_PCIBAR_READ16,
        OCSU_IOCTL_PCIBAR_READ32,
        OCSU_IOCTL_PCIBAR_WRITE8,
        OCSU_IOCTL_PCIBAR_WRITE16,
        OCSU_IOCTL_PCIBAR_WRITE32,
} ocsu_pciref_t;

/**
 * @brief OCS_IOCTL_CMD_ECD_HELPER ioctl structure
 */

typedef struct {
	ocsu_pciref_t cmd;
	uint32_t bar;		/*<< BAR value to use */
	uint32_t offset;	/*<< offset value to use */
	uint32_t data;		/*<< 32 bit data value to write or return read data in */
	int status;		/*<< status of helper function request */
} ocsu_ioctl_pciref_t;

#if defined(__KERNEL__)
#include <stdarg.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <asm-generic/ioctl.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/bitmap.h>
#include <linux/spinlock.h>
#include <asm/byteorder.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <asm/current.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>

typedef struct ocs_uspace ocs_uspace_t;

typedef struct {
	ocsu_memref_t memref;
	uint32_t instance_index;
	struct list_head link;
	uint32_t tag;
} ocsu_dmabuf_t;

typedef struct {
	ocs_uspace_t *ocs;
	uint32_t index;
	struct semaphore intr_sem;
} ocsu_intr_context_t;

struct ocs_uspace {
	atomic_t refcount;
	uint32_t instance_index;
	struct pci_dev	*pdev;
	ocsu_memref_t bars[PCI_MAX_BAR];
	uint32_t bar_count;

	const char *desc;
	uint16_t pci_vendor;
	uint16_t pci_device;
	char businfo[16];

	struct msix_entry msix_vec[MAX_PCI_INTERRUPTS];
	uint32_t n_msix_vec;
	int intr_enabled;

	struct semaphore intr_sem;
	ocsu_intr_context_t intr_context[MAX_PCI_INTERRUPTS];

	void (*interrupt_cb)(ocs_uspace_t *ocs);

	struct device *ocs_dev;

	uint32_t dmabuf_next_instance;
	struct list_head dmabuf_list;

	uint8_t  if_type;
	ocsu_memref_t bmbx;
};

#endif // __KERNEL__

/* Roll the IOCTL_CMD_BASE as the ioctl API changes */
#define OCSU_IOCTL_CMD_BASE			2
#define OCSU_IOCTL_CMD_GET_PCI_CONFIG		_IOR(OCSU_IOCTL_CMD_BASE, 1, ocsu_ioctl_get_pci_config_t)
#define OCSU_IOCTL_CMD_PCIREF			_IOWR(OCSU_IOCTL_CMD_BASE, 2, ocsu_ioctl_pciref_t)
#define OCSU_IOCTL_CMD_DMABUF_ALLOC		_IOWR(OCSU_IOCTL_CMD_BASE, 3, ocsu_ioctl_dmabuf_alloc_t)
#define OCSU_IOCTL_CMD_DMABUF_FREE		_IOWR(OCSU_IOCTL_CMD_BASE, 4, ocsu_ioctl_dmabuf_free_t)
#define OCSU_IOCTL_CMD_DMABUF_SYNC		_IOWR(OCSU_IOCTL_CMD_BASE, 5, ocsu_ioctl_dmabuf_sync_t)
#define OCSU_IOCTL_CMD_WAIT_EVENT		_IO(OCSU_IOCTL_CMD_BASE, 6)
#define OCSU_IOCTL_CMD_INTR_DISABLE		_IO(OCSU_IOCTL_CMD_BASE, 7)
#define OCSU_IOCTL_CMD_INTR_ENABLE		_IO(OCSU_IOCTL_CMD_BASE, 8)
#define OCSU_IOCTL_CMD_WAIT_EVENT_VEC		_IO(OCSU_IOCTL_CMD_BASE, 9)

#define SLI4_BMBX_SIZE				256

#include "ocs_common_shared.h"

/* vim: set noexpandtab textwidth=120: */
#endif // __OCS_USPACE_H__
