/*
 *   BSD LICENSE
 *
 *   Copyright (c) 2018 Broadcom.  All Rights Reserved.
 *   The term "Broadcom" refers to Broadcom Inc. and/or its subsidiaries.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#if !defined(__SPDK_NVMF_XPORT_H__)
#define __SPDK_NVMF_XPORT_H__

#include "nvmf_fc.h"

#define SPDK_SUCCESS		0
#define SPDK_ERR_INVALID_ARGS	-1
#define SPDK_ERR_INTERNAL	-2
#define SPDK_ERR_NOMEM		-3

/* maximum number of IO queues for NVME over FC */
#define OCS_NVME_FC_MAX_IO_QUEUES  16

#define BCM_MAX_IOVECS (SPDK_NVMF_MAX_SGL_ENTRIES + 2) /* 2 for skips */

/* HWQP to assign NMVE admin queue to */
#define OCS_NVME_FC_AQ_IND 0

/* SGE structure */
typedef struct bcm_sge {
	uint32_t	buffer_address_high;
	uint32_t	buffer_address_low;
	uint32_t	data_offset: 27,
			sge_type: 4,
			last: 1;
	uint32_t	buffer_length;
} bcm_sge_t;

#define BCM_SGE_SIZE sizeof(struct bcm_sge)

/* Common queue definition structure */
typedef struct bcm_sli_queue {
	/* general queue housekeeping fields */
	uint16_t  head, tail, used;
	uint32_t  posted_limit;    /* number of CQE/EQE to process before ringing doorbell */
	uint32_t  processed_limit; /* number of CQE/EQE to process in a shot */
	uint16_t  type;            /* bcm_fc_queue_type_e queue type */

	/* the following fields set by the FC driver */
	uint16_t  qid;           /* f/w Q_ID */
	uint16_t  size;          /* size of each entry */
	uint16_t  max_entries;   /* number of entries */
	uint32_t  if_type;	 /* Differentiates the eq/cq version */
	uint16_t  phase;	 /* For if_type = 6, this value toggle for each iteration
                                    of the queue, a queue entry is valid when a cqe valid
                                    bit matches this value */
	void 	  *address;      /* queue address */
	void 	  *doorbell_reg; /* queue doorbell register address */

	/* dpp queue info */
	uint8_t	  dpp_enabled;
	uint8_t   dpp_id;
	void	  *dpp_doorbell_reg;	/* doorbell register for dpp queues */
	char	  name[64];      	/* unique name */ 
} bcm_sli_queue_t;

/* EQ/CQ structure */
typedef struct fc_eventq {
	bcm_sli_queue_t q;
	bool auto_arm_flag;     /* set by poller thread only */
} fc_eventq_t;

/* WQ related */
typedef void (*bcm_fc_wqe_cb)(void *hwqp, uint8_t *cqe, int32_t status, void *args);

#define MAX_WQ_WQEC_CNT 5
#define MAX_REQTAG_POOL_SIZE 8191 /* Should be one less than DPDK ring */
typedef struct fc_wqe_reqtag {
	uint16_t index;
	bcm_fc_wqe_cb cb;
	void *cb_args;
} fc_reqtag_t;

#define MAX_WQ_ENTRIES 4096
typedef struct fc_wrkq {
	bcm_sli_queue_t q;
	uint32_t num_buffers;
	struct spdk_nvmf_fc_buffer_desc *buffer;  /* BDE buffer descriptor array */

	/* internal */
	uint32_t wqec_count;
	struct spdk_ring *reqtag_ring;
	fc_reqtag_t *reqtag_objs;
	fc_reqtag_t *p_reqtags[MAX_REQTAG_POOL_SIZE];
} fc_wrkq_t;

#define MAX_RQ_ENTRIES 4096
/* RQ structure */
typedef struct fc_rcvq {
	bcm_sli_queue_t q;
	uint32_t num_buffers;
	struct spdk_nvmf_fc_buffer_desc *buffer;      /* RQ buffer descriptor array */
	/* internal */
	uint32_t rq_map[MAX_RQ_ENTRIES];
} fc_rcvq_t;

struct fc_sgl_list {
	void *virt;
	uint64_t phys;
};

struct bcm_nvmf_fc_port {
	uint32_t xri_base;
	uint32_t xri_count;
	struct fc_sgl_list *sgl_list;
	bool sgl_preregistered;

	/* Internal */
	struct spdk_mempool *xri_pool;
	uint32_t num_cores;
};

/*
 * Hardware queues structure.
 * Structure passed from master thread to poller thread.
 */
struct bcm_nvmf_hw_queues {
	struct fc_eventq eq;
	struct fc_eventq cq_wq;
	struct fc_eventq cq_sfwq;
	struct fc_eventq cq_rq;
	struct fc_wrkq wq;
	struct fc_wrkq sfwq;
	struct fc_rcvq rq_hdr;
	struct fc_rcvq rq_payload;
	bool sfwq_configured;

	uint16_t cid_cnt;   /* used to generate unique connection id for MRQ */
	TAILQ_HEAD(, spdk_nvmf_fc_xchg) pending_xri_list;
	uint32_t send_frame_xri;
	uint8_t frame_seqid;
};

/* functions to manage XRI's (for each port) */
struct spdk_mempool*
spdk_nvmf_fc_create_xri_pool(uint32_t port_handle, uint32_t xri_base, uint32_t xri_count,
			     uint32_t num_cpus, struct fc_sgl_list *sgl_list);
void spdk_nvmf_fc_delete_xri_pool(struct spdk_mempool *xri_pool);
void nvmf_fc_wrkq_reqtag_copy(struct fc_wrkq *wq_prev, struct fc_wrkq *wq_curr);
void nvmf_fc_tgt_free_xri_pending(struct spdk_nvmf_fc_port *fc_port);
void spdk_nvmf_hwqp_free_wq_reqtags(struct spdk_nvmf_fc_port *fc_port);

#endif
