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

#if !defined(__OCS_UAPI_INTERFACE_H__)
#define __OCS_UAPI_INTERFACE_H__

#define NVMF_FC_MAX_IO_QUEUES 16

struct ocs_uapi_dma_desc {
	uint32_t mm_type_specific;
	uint64_t phys;
	void *vaddr;
	size_t len;
};
/* Max size of DMA list that can be used. */
#define OCS_MAX_UAPI_DMA_LIST_LEN (1024 * 1024 * 17)
/* Max size of DMA segment that can be used. */
#define OCS_MAX_UAPI_DMA_DESC_LEN (1024 * 256)

struct ocs_uapi_dma_list_desc {
	/* Array of dma segments. */
	struct ocs_uapi_dma_desc dma[OCS_MAX_UAPI_DMA_LIST_LEN / OCS_MAX_UAPI_DMA_DESC_LEN];

	/* Actual number of entries used in the array of dma segments. */
	int num_desc;

	/* Number of buffers in each dma segment except last one. */
	int num_desc_entries;

	/* Number of buffers in last dma segment. */
	int num_last_desc_entries;

	/* Total number of buffers in all the dma segments together. */
	int num_buffers;

	/* Actual size of each buffer. */
	size_t buffer_size;

	/* Size of each buffer with padding. */
	size_t buffer_size_with_padding;
};

struct ocs_uapi_fc_wwn {
	union {
		uint64_t wwn;
		uint8_t octets[sizeof(uint64_t)];
	} u;
};

#define UAPI_SLI_Q_DMA_CHUNKS 8
struct ocs_uapi_sli_queue {
	uint16_t  type;

	uint16_t  qid;
	uint16_t  size;
	uint32_t  max_entries;
	uint32_t  if_type;
	uint16_t  phase;

	struct ocs_uapi_dma_desc address[UAPI_SLI_Q_DMA_CHUNKS];
	uint32_t  address_chunk_size;

	uint16_t doorbell_rset;
	uint32_t doorbell_offset;

	uint8_t  dpp_enabled;
	uint8_t  dpp_id;
	uint16_t dpp_doorbell_rset;
	uint32_t dpp_doorbell_offset;
};

struct ocs_uapi_eventq {
	struct ocs_uapi_sli_queue q;
};

struct ocs_uapi_wrkq {
	struct ocs_uapi_sli_queue q;
};

struct ocs_uapi_rcvq {
	struct ocs_uapi_sli_queue q;
	struct ocs_uapi_dma_list_desc buffer_list;
};

struct ocs_uapi_hw_queues {
	uint32_t irq_vector;
	struct ocs_uapi_eventq eq;
	struct ocs_uapi_eventq cq_wq;
	struct ocs_uapi_eventq cq_rq;
	struct ocs_uapi_wrkq wq;
	struct ocs_uapi_rcvq rq_hdr;
	struct ocs_uapi_rcvq rq_payload;
};

struct ocs_uapi_hw_port_init_args {
	uint8_t                           port_handle;
	uint16_t                          fcp_rq_id;

	/* LS queue */
	struct ocs_uapi_hw_queues ls_queue;

	/* IO queue's info */
	uint32_t                          io_queue_cnt;
	struct ocs_uapi_hw_queues	  io_queues[NVMF_FC_MAX_IO_QUEUES];

	/* XRI */
	uint32_t                          xri_base;
	uint32_t                          xri_count;
	bool                              is_sgl_preregistered;
	struct ocs_uapi_dma_list_desc     sgl_list;

	uint8_t				  marker_category;
};

struct ocs_uapi_hw_port_online_args {
	uint8_t port_handle;
	uint32_t speed;
};

struct ocs_uapi_hw_port_offline_args {
	uint8_t port_handle;
};

struct ocs_uapi_hw_port_reset_args {
	uint8_t port_handle;
	bool dump_queues;
};

struct ocs_uapi_hw_port_free_args {
	uint8_t port_handle;
};

struct ocs_uapi_hw_port_reinit_args {
	uint8_t port_handle;
};

struct ocs_uapi_nport_create_args {
	uint8_t	 port_handle;
	uint16_t nport_handle;
	uint32_t d_id;
	struct ocs_uapi_fc_wwn wwnn;
	struct ocs_uapi_fc_wwn wwpn;
};

struct ocs_uapi_nport_delete_args {
	uint8_t  port_handle;
	uint32_t nport_handle;
};

struct ocs_uapi_it_add_args {
	uint8_t  port_handle;
	uint32_t nport_handle;
	uint32_t rpi;
	uint32_t s_id;
	uint32_t initiator_prli_info;
	uint32_t target_prli_info;
	struct ocs_uapi_fc_wwn wwnn;
	struct ocs_uapi_fc_wwn wwpn;
	bool is_remote_tgt;
	bool is_bidir_capable;
	bool is_bidir_done;
	bool nsler_negotiated;
};

struct ocs_uapi_it_del_args {
	uint8_t  port_handle;
	uint32_t nport_handle;
	uint32_t rpi;
	uint32_t s_id;
	struct ocs_uapi_fc_wwn local_wwpn;
	struct ocs_uapi_fc_wwn remote_wwpn;	
	bool is_remote_tgt;
	bool is_bidir_capable;
	bool is_bidir_done;
};

struct ocs_uapi_bls_flush_args {
	bool ht;
	uint32_t sler_qual;
	uint16_t count;
};

struct ocs_uapi_bls_args {
	uint8_t  port_handle;
	uint32_t nport_handle;
	uint32_t rpi;
	uint32_t s_id;
	uint16_t oxid;
	uint16_t rxid;
	union {
		struct ocs_uapi_bls_flush_args flush;
	} u;
};

union ocs_uapi_generic_args {
	struct ocs_uapi_hw_port_free_args hw_free;
	struct ocs_uapi_hw_port_online_args hw_online;
	struct ocs_uapi_hw_port_offline_args hw_offline;
	struct ocs_uapi_hw_port_reset_args hw_reset;
	struct ocs_uapi_nport_create_args nport_create;
	struct ocs_uapi_nport_delete_args nport_del;
	struct ocs_uapi_it_add_args it_add;
	struct ocs_uapi_it_del_args it_del;
	struct ocs_uapi_bls_args bls;
};

typedef enum {
	OCS_UAPI_FC_HW_PORT_FREE,
	OCS_UAPI_FC_HW_PORT_REINIT,
	OCS_UAPI_FC_HW_PORT_ONLINE,
	OCS_UAPI_FC_HW_PORT_OFFLINE,
	OCS_UAPI_FC_HW_PORT_RESET,
	OCS_UAPI_FC_NPORT_CREATE,
	OCS_UAPI_FC_NPORT_DELETE,
	OCS_UAPI_FC_IT_ADD,
	OCS_UAPI_FC_IT_DELETE,
	OCS_UAPI_FC_ABTS_RECV,
	OCS_UAPI_FC_FLUSH_BLS_RECV,
	OCS_UAPI_FC_LINK_BREAK,
	OCS_UAPI_FC_HW_PORT_DUMP,
	OCS_UAPI_FC_UNRECOVERABLE_ERR,
	OCS_UAPI_FC_EVENT_MAX,
} ocs_uapi_msg_type_t;

typedef void (*ocs_uapi_unknown_frame_callback)(uint8_t port_handle, uint32_t s_id, uint32_t d_id);

#endif
