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
 * Define Fibre Channel SLI-4 structures and function prototypes.
 */

#ifndef _SLI4_FC_H
#define _SLI4_FC_H

#include "sli4.h"
#include "ocs_fcp.h"

/**
 * @brief Maximum value for a FCFI
 *
 * Note that although most commands provide a 16 bit field for the FCFI,
 * the FC/FCoE Asynchronous Recived CQE format only provides 6 bits for
 * the returned FCFI. Then effectively, the FCFI cannot be larger than
 * 1 << 6 or 64.
 */
#define SLI4_MAX_FCFI	64

/**
 * @brief Maximum value for FCF index
 *
 * The SLI-4 specification uses a 16 bit field in most places for the FCF
 * index, but practically, this value will be much smaller. Arbitrarily
 * limit the max FCF index to match the max FCFI value.
 */
#define SLI4_MAX_FCF_INDEX	SLI4_MAX_FCFI

/*************************************************************************
 * SLI-4 FC/FCoE mailbox command formats and definitions.
 */

/**
 * FC/FCoE opcode (OPC) values.
 */
#define SLI4_OPC_FCOE_WQ_CREATE			0x1
#define SLI4_OPC_FCOE_WQ_DESTROY		0x2
#define SLI4_OPC_FCOE_POST_SGL_PAGES		0x3
#define SLI4_OPC_FCOE_RQ_CREATE			0x5
#define SLI4_OPC_FCOE_RQ_DESTROY		0x6
#define SLI4_OPC_FCOE_READ_FCF_TABLE		0x8
#define SLI4_OPC_FCOE_POST_HDR_TEMPLATES	0xb
#define SLI4_OPC_FCOE_REDISCOVER_FCF		0x10

/* Use the default CQ associated with the WQ */
#define SLI4_CQ_DEFAULT 0xffff

typedef struct sli4_physical_page_descriptor_s {
	uint32_t	low;
	uint32_t	high;
} sli4_physical_page_descriptor_t;

/**
 * @brief FCOE_WQ_CREATE
 *
 * Create a Work Queue for FC/FCoE use.
 */
#define SLI4_FCOE_WQ_CREATE_V0_MAX_PAGES	4

typedef struct sli4_req_fcoe_wq_create_s {
	sli4_req_hdr_t	hdr;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	num_pages:8,
			dua:1,
			:7,
			cq_id:16;
	sli4_physical_page_descriptor_t page_physical_address[SLI4_FCOE_WQ_CREATE_V0_MAX_PAGES];
	uint32_t	bqu:1,
			:7,
			ulp:8,
			:16;
#else
#error big endian version not defined
#endif
} sli4_req_fcoe_wq_create_t;

/**
 * @brief FCOE_WQ_CREATE_V1
 *
 * Create a version 1 Work Queue for FC/FCoE use.
 */
typedef struct sli4_req_fcoe_wq_create_v1_s {
	sli4_req_hdr_t	hdr;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	num_pages:16,
			cq_id:16;
	uint32_t	page_size:8,
			wqe_size:4,
			:3,
			dpp:1,
			wqe_count:16;
	uint32_t	sfq:1,
			rsvd6:31;
	sli4_physical_page_descriptor_t page_physical_address[8];
#else
#error big endian version not defined
#endif
} sli4_req_fcoe_wq_create_v1_t;

#define SLI4_FCOE_WQ_CREATE_V1_MAX_PAGES	8

/**
 * @brief FCOE_WQ_DESTROY
 *
 * Destroy an FC/FCoE Work Queue.
 */
typedef struct sli4_req_fcoe_wq_destroy_s {
	sli4_req_hdr_t	hdr;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	wq_id:16,
			:16;
#else
#error big endian version not defined
#endif
} sli4_req_fcoe_wq_destroy_t;

/**
 * @brief FCOE_POST_SGL_PAGES
 *
 * Register the scatter gather list (SGL) memory and associate it with an XRI.
 */
typedef struct sli4_fcoe_post_sgl_page_desc_s {
	uint32_t	page0_low;
	uint32_t	page0_high;
	uint32_t	page1_low;
	uint32_t	page1_high;
} sli4_fcoe_post_sgl_page_desc_t;

typedef struct sli4_req_fcoe_post_sgl_pages_s {
	sli4_req_hdr_t	hdr;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	xri_start:16,
			xri_count:16;
	sli4_fcoe_post_sgl_page_desc_t page_desc[0];
#else
#error big endian version not defined
#endif
} sli4_req_fcoe_post_sgl_pages_t;

/**
 * @brief FCOE_RQ_CREATE
 *
 * Create a Receive Queue for FC/FCoE use.
 */
typedef struct sli4_req_fcoe_rq_create_s {
	sli4_req_hdr_t	hdr;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	num_pages:16,
			dua:1,
			bqu:1,
			:6,
			ulp:8;
	uint32_t	:16,
			rqe_count:4,
			:12;
	uint32_t	rsvd6;
	uint32_t	buffer_size:16,
			cq_id:16;
	uint32_t	rsvd8;
	sli4_physical_page_descriptor_t page_physical_address[8];
#else
#error big endian version not defined
#endif
} sli4_req_fcoe_rq_create_t;

#define SLI4_FCOE_RQ_CREATE_V0_MAX_PAGES	8
#define SLI4_FCOE_RQ_CREATE_V0_MIN_BUF_SIZE	128
#define SLI4_FCOE_RQ_CREATE_V0_MAX_BUF_SIZE	2048

/**
 * @brief FCOE_RQ_CREATE_V1
 *
 * Create a version 1 Receive Queue for FC/FCoE use.
 */
typedef struct sli4_req_fcoe_rq_create_v1_s {
	sli4_req_hdr_t	hdr;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	num_pages:16,
			:13,
			dim:1,
			dfd:1,
			dnb:1;
	uint32_t	page_size:8,
			rqe_size:4,
			rqe_count_hi:4,
			rqe_count:16;
	uint32_t	rsvd6;
	uint32_t	:16,
			cq_id:16;
	uint32_t	buffer_size;
	sli4_physical_page_descriptor_t page_physical_address[8];
#else
#error big endian version not defined
#endif
} sli4_req_fcoe_rq_create_v1_t;


/**
 * @brief FCOE_RQ_CREATE_V2
 *
 * Create a version 2 Receive Queue for FC/FCoE use.
 */
typedef struct sli4_req_fcoe_rq_create_v2_s {
	sli4_req_hdr_t	hdr;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	num_pages:16,
			rq_count:8,
			:5,
			dim:1,
			dfd:1,
			dnb:1;
	uint32_t	page_size:8,
			rqe_size:4,
			rqe_count_hi:4,
			rqe_count:16;
	uint32_t	hdr_buffer_size:16,
			payload_buffer_size:16;
	uint32_t	base_cq_id:16,
			:16;
	uint32_t	rsvd;
	sli4_physical_page_descriptor_t page_physical_address[0];
#else
#error big endian version not defined
#endif
} sli4_req_fcoe_rq_create_v2_t;


#define SLI4_FCOE_RQ_CREATE_V1_MAX_PAGES	8
#define SLI4_FCOE_RQ_CREATE_V1_MIN_BUF_SIZE	64
#define SLI4_FCOE_RQ_CREATE_V1_MAX_BUF_SIZE	2048

#define SLI4_FCOE_RQE_SIZE_8			0x2
#define SLI4_FCOE_RQE_SIZE_16			0x3
#define SLI4_FCOE_RQE_SIZE_32			0x4
#define SLI4_FCOE_RQE_SIZE_64			0x5
#define SLI4_FCOE_RQE_SIZE_128			0x6

#define SLI4_FCOE_RQ_PAGE_SIZE_4096		0x1
#define SLI4_FCOE_RQ_PAGE_SIZE_8192		0x2
#define SLI4_FCOE_RQ_PAGE_SIZE_16384		0x4
#define SLI4_FCOE_RQ_PAGE_SIZE_32768		0x8
#define SLI4_FCOE_RQ_PAGE_SIZE_64536		0x10

#define SLI4_FCOE_RQE_SIZE			8

/**
 * @brief FCOE_RQ_DESTROY
 *
 * Destroy an FC/FCoE Receive Queue.
 */
typedef struct sli4_req_fcoe_rq_destroy_s {
	sli4_req_hdr_t	hdr;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	rq_id:16,
			:16;
#else
#error big endian version not defined
#endif
} sli4_req_fcoe_rq_destroy_t;

/**
 * @brief FCOE_READ_FCF_TABLE
 *
 * Retrieve a FCF database (also known as a table) entry created by the SLI Port
 * during FIP discovery.
 */
typedef struct sli4_req_fcoe_read_fcf_table_s {
	sli4_req_hdr_t	hdr;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	fcf_index:16,
			:16;
#else
#error big endian version not defined
#endif
} sli4_req_fcoe_read_fcf_table_t;

/* A FCF index of -1 on the request means return the first valid entry */
#define SLI4_FCOE_FCF_TABLE_FIRST		(UINT16_MAX)

/**
 * @brief FCF table entry
 *
 * This is the information returned by the FCOE_READ_FCF_TABLE command.
 */
typedef struct sli4_fcf_entry_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	max_receive_size;
	uint32_t	fip_keep_alive;
	uint32_t	fip_priority;
	uint8_t		fcf_mac_address[6];
	uint8_t		fcf_available;
	uint8_t		mac_address_provider;
	uint8_t		fabric_name_id[8];
	uint8_t		fc_map[3];
	uint8_t		val:1,
			fc:1,
			:5,
			sol:1;
	uint32_t	fcf_index:16,
			fcf_state:16;
	uint8_t		vlan_bitmap[512];
	uint8_t		switch_name[8];
#else
#error big endian version not defined
#endif
} sli4_fcf_entry_t;

/**
 * @brief FCOE_READ_FCF_TABLE response.
 */
typedef struct sli4_res_fcoe_read_fcf_table_s {
	sli4_res_hdr_t	hdr;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	event_tag;
	uint32_t	next_index:16,
			:16;
	sli4_fcf_entry_t fcf_entry;
#else
#error big endian version not defined
#endif
} sli4_res_fcoe_read_fcf_table_t;

/* A next FCF index of -1 in the response means this is the last valid entry */
#define SLI4_FCOE_FCF_TABLE_LAST		(UINT16_MAX)


/**
 * @brief FCOE_POST_HDR_TEMPLATES
 */
typedef struct sli4_req_fcoe_post_hdr_templates_s {
	sli4_req_hdr_t	hdr;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	rpi_offset:16,
			page_count:16;
	sli4_physical_page_descriptor_t page_descriptor[0];
#else
#error big endian version not defined
#endif
} sli4_req_fcoe_post_hdr_templates_t;

#define SLI4_FCOE_HDR_TEMPLATE_SIZE	64

/**
 * @brief FCOE_REDISCOVER_FCF
 */
typedef struct sli4_req_fcoe_rediscover_fcf_s {
	sli4_req_hdr_t	hdr;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	fcf_count:16,
			:16;
	uint32_t	rsvd5;
	uint16_t	fcf_index[16];
#else
#error big endian version not defined
#endif
} sli4_req_fcoe_rediscover_fcf_t;


/**
 * Work Queue Entry (WQE) types.
 */
#define SLI4_WQE_ABORT			0x0f
#define SLI4_WQE_ELS_REQUEST64		0x8a
#define SLI4_WQE_FCP_IBIDIR64		0xac
#define SLI4_WQE_FCP_IREAD64		0x9a
#define SLI4_WQE_FCP_IWRITE64		0x98
#define SLI4_WQE_FCP_ICMND64		0x9c
#define SLI4_WQE_FCP_TRECEIVE64		0xa1
#define SLI4_WQE_FCP_CONT_TRECEIVE64	0xe5
#define SLI4_WQE_FCP_TRSP64		0xa3
#define SLI4_WQE_FCP_TSEND64		0x9f
#define SLI4_WQE_GEN_REQUEST64		0xc2
#define SLI4_WQE_SEND_FRAME		0xe1
#define SLI4_WQE_XMIT_BCAST64		0X84
#define SLI4_WQE_XMIT_BLS_RSP		0x97
#define SLI4_WQE_ELS_RSP64		0x95
#define SLI4_WQE_XMIT_SEQUENCE64	0x82
#define SLI4_WQE_REQUEUE_XRI		0x93
#define SLI4_WQE_RQ_MARKER_REQUEST	0xe6

/**
 * WQE command types.
 */
#define SLI4_CMD_FCP_IREAD64_WQE	0x00
#define SLI4_CMD_FCP_ICMND64_WQE	0x00
#define SLI4_CMD_FCP_IWRITE64_WQE	0x01
#define SLI4_CMD_FCP_TRECEIVE64_WQE	0x02
#define SLI4_CMD_FCP_TRSP64_WQE		0x03
#define SLI4_CMD_FCP_TSEND64_WQE	0x07
#define SLI4_CMD_GEN_REQUEST64_WQE	0x08
#define SLI4_CMD_XMIT_BCAST64_WQE	0x08
#define SLI4_CMD_XMIT_BLS_RSP64_WQE	0x08
#define SLI4_CMD_ABORT_WQE		0x08
#define SLI4_CMD_XMIT_SEQUENCE64_WQE	0x08
#define SLI4_CMD_REQUEUE_XRI_WQE	0x0A
#define SLI4_CMD_SEND_FRAME_WQE		0x0a
#define	SLI4_CMD_RQ_MARKER_REQ_WQE	0x0a

#define SLI4_WQE_SIZE			0x05
#define SLI4_WQE_EXT_SIZE		0x06

#define SLI4_WQE_BYTES			(16 * sizeof(uint32_t))
#define SLI4_WQE_EXT_BYTES		(32 * sizeof(uint32_t))

/* Mask for ccp (CS_CTL) */
#define SLI4_MASK_CCP			0xff

/**
 * @brief Generic WQE
 */
typedef struct sli4_generic_wqe_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	cmd_spec0_5[6];
	uint32_t	xri_tag:16,
			context_tag:16;
	uint32_t	:2,
			ct:2,
			:4,
			command:8,
			class:3,
			:1,
			pu:2,
			:2,
			timer:8;
	uint32_t	abort_tag;
	uint32_t	request_tag:16,
			:16;
	uint32_t	ebde_cnt:4,
			:3,
			len_loc:2,
			qosd:1,
			:1,
			xbl:1,
			hlm:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			:3,
			wqec:1,
			:8,
			cq_id:16;
#else
#error big endian version not defined
#endif
} sli4_generic_wqe_t;

/**
 * @brief WQE used to abort exchanges.
 */
typedef struct sli4_abort_wqe_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	rsvd0;
	uint32_t	rsvd1;
	uint32_t	ext_t_tag;
	uint32_t	ia:1,
			ir:1,
			:6,
			criteria:8,
			:16;
	uint32_t	ext_t_mask;
	uint32_t	t_mask;
	uint32_t	xri_tag:16,
			context_tag:16;
	uint32_t	:2,
			ct:2,
			:4,
			command:8,
			class:3,
			:1,
			pu:2,
			:2,
			timer:8;
	uint32_t	t_tag;
	uint32_t	request_tag:16,
			:16;
	uint32_t	ebde_cnt:4,
			:3,
			len_loc:2,
			qosd:1,
			:1,
			xbl:1,
			:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			:3,
			wqec:1,
			:8,
			cq_id:16;
#else
#error big endian version not defined
#endif
} sli4_abort_wqe_t;

#define SLI4_ABORT_CRITERIA_XRI_TAG		0x01
#define SLI4_ABORT_CRITERIA_ABORT_TAG		0x02
#define SLI4_ABORT_CRITERIA_REQUEST_TAG		0x03
#define SLI4_ABORT_CRITERIA_EXT_ABORT_TAG	0x04

typedef enum {
	SLI_ABORT_XRI,
	SLI_ABORT_ABORT_ID,
	SLI_ABORT_REQUEST_ID,
	SLI_ABORT_MAX,		// must be last
} sli4_abort_type_e;

/**
 * @brief WQE used to create an ELS request.
 */
typedef struct sli4_els_request64_wqe_s {
	sli4_bde_t	els_request_payload;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	els_request_payload_length;
	uint32_t	sid:24,
			sp:1,
			:7;
	uint32_t	remote_id:24,
			:8;
	uint32_t	xri_tag:16,
			context_tag:16;
	uint32_t	:2,
			ct:2,
			:4,
			command:8,
			class:3,
			ar:1,
			pu:2,
			:2,
			timer:8;
	uint32_t	abort_tag;
	uint32_t	request_tag:16,
			temporary_rpi:16;
	uint32_t	ebde_cnt:4,
			:3,
			len_loc:2,
			qosd:1,
			:1,
			xbl:1,
			hlm:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			els_id:3,
			wqec:1,
			:8,
			cq_id:16;
	sli4_bde_t	els_response_payload_bde;
	uint32_t	max_response_payload_length;
#else
#error big endian version not defined
#endif
} sli4_els_request64_wqe_t;

#define SLI4_ELS_REQUEST64_CONTEXT_RPI	0x0
#define SLI4_ELS_REQUEST64_CONTEXT_VPI	0x1
#define SLI4_ELS_REQUEST64_CONTEXT_VFI	0x2
#define SLI4_ELS_REQUEST64_CONTEXT_FCFI	0x3

#define SLI4_ELS_REQUEST64_CLASS_2	0x1
#define SLI4_ELS_REQUEST64_CLASS_3	0x2

#define SLI4_ELS_REQUEST64_DIR_WRITE	0x0
#define SLI4_ELS_REQUEST64_DIR_READ	0x1

#define SLI4_ELS_REQUEST64_OTHER	0x0
#define SLI4_ELS_REQUEST64_LOGO		0x1
#define SLI4_ELS_REQUEST64_FDISC	0x2
#define SLI4_ELS_REQUEST64_FLOGIN	0x3
#define SLI4_ELS_REQUEST64_PLOGI	0x4

#define SLI4_ELS_REQUEST64_CMD_GEN		0x08
#define SLI4_ELS_REQUEST64_CMD_NON_FABRIC	0x0c
#define SLI4_ELS_REQUEST64_CMD_FABRIC		0x0d

/**
 * @brief WQE used to create an FCP initiator no data command.
 */
typedef struct sli4_fcp_icmnd64_wqe_s {
	sli4_bde_t	bde;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	payload_offset_length:16,
			fcp_cmd_buffer_length:16;
	uint32_t	rsvd4;
	uint32_t	remote_n_port_id:24,
			:8;
	uint32_t	xri_tag:16,
			context_tag:16;
	uint32_t	dif:2,
			ct:2,
			bs:3,
			:1,
			command:8,
			class:3,
			:1,
			pu:2,
			erp:1,
			lnk:1,
			timer:8;
	uint32_t	abort_tag;
	uint32_t	request_tag:16,
			:16;
	uint32_t	ebde_cnt:4,
			:3,
			len_loc:2,
			qosd:1,
			:1,
			xbl:1,
			hlm:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			:3,
			wqec:1,
			:8,
			cq_id:16;
	uint32_t	rsvd12;
	uint32_t	rsvd13;
	uint32_t	rsvd14;
	uint32_t	rsvd15;
#else
#error big endian version not defined
#endif
} sli4_fcp_icmnd64_wqe_t;

/**
 * @brief WQE used to create an FCP initiator read.
 */
typedef struct sli4_fcp_iread64_wqe_s {
	sli4_bde_t	bde;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	payload_offset_length:16,
			fcp_cmd_buffer_length:16;
	uint32_t	total_transfer_length;
	uint32_t	remote_n_port_id:24,
			:8;
	uint32_t	xri_tag:16,
			context_tag:16;
	uint32_t	dif:2,
			ct:2,
			bs:3,
			:1,
			command:8,
			class:3,
			:1,
			pu:2,
			erp:1,
			lnk:1,
			timer:8;
	uint32_t	abort_tag;
	uint32_t	request_tag:16,
			:16;
	uint32_t	ebde_cnt:4,
			:3,
			len_loc:2,
			qosd:1,
			:1,
			xbl:1,
			hlm:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			:3,
			wqec:1,
			:8,
			cq_id:16;
	uint32_t	rsvd12;
#else
#error big endian version not defined
#endif
	sli4_bde_t	first_data_bde;	/* For performance hints */
} sli4_fcp_iread64_wqe_t;

/**
 * @brief WQE used to create an FCP initiator write.
 */
typedef struct sli4_fcp_iwrite64_wqe_s {
	sli4_bde_t	bde;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	payload_offset_length:16,
			fcp_cmd_buffer_length:16;
	uint32_t	total_transfer_length;
	uint32_t	initial_transfer_length;
	uint32_t	xri_tag:16,
			context_tag:16;
	uint32_t	dif:2,
			ct:2,
			bs:3,
			:1,
			command:8,
			class:3,
			:1,
			pu:2,
			erp:1,
			lnk:1,
			timer:8;
	uint32_t	abort_tag;
	uint32_t	request_tag:16,
			:16;
	uint32_t	ebde_cnt:4,
			:3,
			len_loc:2,
			qosd:1,
			:1,
			xbl:1,
			hlm:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			:3,
			wqec:1,
			:8,
			cq_id:16;
	uint32_t	remote_n_port_id:24,
			:8;
#else
#error big endian version not defined
#endif
	sli4_bde_t	first_data_bde;	/* For performance hints */
} sli4_fcp_iwrite64_wqe_t;

typedef struct sli4_fcp_128byte_wqe_s {
	uint32_t dw[32];
} sli4_fcp_128byte_wqe_t;

/**
 * @brief WQE used to create an FCP target receive, and FCP target
 * receive continue.
 */
typedef struct sli4_fcp_treceive64_wqe_s {
	sli4_bde_t	bde;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	payload_offset_length;
	uint32_t	relative_offset;
	/**
	 * DWord 5 can either be the task retry identifier (HLM=0) or
	 * the remote N_Port ID (HLM=1), or if implementing the Skyhawk
	 * T10-PI workaround, the secondary xri tag
	 */
	union {
		uint32_t	sec_xri_tag:16,
				:16;
		uint32_t	dword;
	} dword5;
	uint32_t	xri_tag:16,
			context_tag:16;
	uint32_t	dif:2,
			ct:2,
			bs:3,
			:1,
			command:8,
			class:3,
			ar:1,
			pu:2,
			conf:1,
			lnk:1,
			timer:8;
	uint32_t	abort_tag;
	uint32_t	request_tag:16,
			remote_xid:16;
	uint32_t	ebde_cnt:4,
			:1,
			app_id_valid:1,
			:1,
			len_loc:2,
			qosd:1,
			wchn:1,
			xbl:1,
			hlm:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			sr:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			irsp:1,
			pbde:1,
			:1,
			wqec:1,
			:8,
			cq_id:16;
	uint32_t	fcp_data_receive_length;
#else
#error big endian version not defined
#endif
	sli4_bde_t	first_data_bde;	/* For performance hints */
} sli4_fcp_treceive64_wqe_t;

/**
 * @brief WQE used to create an FCP target response.
 */
typedef struct sli4_fcp_trsp64_wqe_s {
	sli4_bde_t	bde;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	fcp_response_length;
	uint32_t	rsvd4;
	/**
	 * DWord 5 can either be the task retry identifier (HLM=0) or
	 * the remote N_Port ID (HLM=1)
	 */
	uint32_t	dword5;
	uint32_t	xri_tag:16,
			rpi:16;
	uint32_t	:2,
			ct:2,
			dnrx:1,
			:3,
			command:8,
			class:3,
			ag:1,
			pu:2,
			conf:1,
			lnk:1,
			timer:8;
	uint32_t	abort_tag;
	uint32_t	request_tag:16,
			remote_xid:16;
	uint32_t	ebde_cnt:4,
			:1,
			app_id_valid:1,
			:1,
			len_loc:2,
			qosd:1,
			wchn:1,
			xbl:1,
			hlm:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			sr:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			:3,
			wqec:1,
			:8,
			cq_id:16;
	uint32_t	rsvd12;
	uint32_t	rsvd13;
	uint32_t	rsvd14;
	uint32_t	rsvd15;
#else
#error big endian version not defined
#endif
} sli4_fcp_trsp64_wqe_t;

/**
 * @brief WQE used to create an FCP target send (DATA IN).
 */
typedef struct sli4_fcp_tsend64_wqe_s {
	sli4_bde_t	bde;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	payload_offset_length;
	uint32_t	relative_offset;
	/**
	 * DWord 5 can either be the task retry identifier (HLM=0) or
	 * the remote N_Port ID (HLM=1)
	 */
	uint32_t	dword5;
	uint32_t	xri_tag:16,
			rpi:16;
	uint32_t	dif:2,
			ct:2,
			bs:3,
			:1,
			command:8,
			class:3,
			ar:1,
			pu:2,
			conf:1,
			lnk:1,
			timer:8;
	uint32_t	abort_tag;
	uint32_t	request_tag:16,
			remote_xid:16;
	uint32_t	ebde_cnt:4,
			:1,
			app_id_valid:1,
			:1,
			len_loc:2,
			qosd:1,
			wchn:1,
			xbl:1,
			hlm:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			sr:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			:2,
			suppress_rsp:1,
			wqec:1,
			:8,
			cq_id:16;
	uint32_t	fcp_data_transmit_length;
#else
#error big endian version not defined
#endif
	sli4_bde_t	first_data_bde;	/* For performance hints */
} sli4_fcp_tsend64_wqe_t;

#define SLI4_IO_CONTINUATION		BIT(0)	/** The XRI associated with this IO is already active */
#define SLI4_IO_AUTO_GOOD_RESPONSE	BIT(1)	/** Automatically generate a good RSP frame */
#define SLI4_IO_NO_ABORT		BIT(2)
#define SLI4_IO_DNRX			BIT(3)	/** Set the DNRX bit because no auto xref rdy buffer is posted */
#define SLI4_IO_SUPPRESS_RESPONSE	BIT(4)	/** Suppress resp is supported only when AutoResp is enabled */

/* WQE DIF field contents */
#define SLI4_DIF_DISABLED		0
#define SLI4_DIF_PASS_THROUGH		1
#define SLI4_DIF_STRIP			2
#define SLI4_DIF_INSERT			3

/**
 * @brief WQE used to create a general request.
 */
typedef struct sli4_gen_request64_wqe_s {
	sli4_bde_t	bde;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	request_payload_length;
	uint32_t	relative_offset;
	uint32_t	:8,
			df_ctl:8,
			type:8,
			r_ctl:8;
	uint32_t	xri_tag:16,
			context_tag:16;
	uint32_t	:2,
			ct:2,
			:4,
			command:8,
			class:3,
			:1,
			pu:2,
			:2,
			timer:8;
	uint32_t	abort_tag;
	uint32_t	request_tag:16,
			:16;
	uint32_t	ebde_cnt:4,
			:3,
			len_loc:2,
			qosd:1,
			:1,
			xbl:1,
			hlm:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			:3,
			wqec:1,
			:8,
			cq_id:16;
	uint32_t	remote_n_port_id:24,
			:8;
	uint32_t	rsvd13;
	uint32_t	rsvd14;
	uint32_t	max_response_payload_length;
#else
#error big endian version not defined
#endif
} sli4_gen_request64_wqe_t;

/**
 * @brief WQE used to create a send frame request.
 */
typedef struct sli4_send_frame_wqe_s {
	sli4_bde_t	bde;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	frame_length;
	uint32_t	fc_header_0_1[2];
	uint32_t	xri_tag:16,
			context_tag:16;
	uint32_t	:2,
			ct:2,
			:4,
			command:8,
			class:3,
			:1,
			pu:2,
			:2,
			timer:8;
	uint32_t	abort_tag;
	uint32_t	request_tag:16,
			eof:8,
			sof:8;
	uint32_t	ebde_cnt:4,
			:3,
			lenloc:2,
			qosd:1,
			wchn:1,
			xbl:1,
			hlm:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			:3,
			wqec:1,
			:8,
			cq_id:16;
	uint32_t	fc_header_2_5[4];
#else
#error big endian version not defined
#endif
} sli4_send_frame_wqe_t;

/**
 * @brief WQE used to create a transmit sequence.
 */
typedef struct sli4_xmit_sequence64_wqe_s {
	sli4_bde_t	bde;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	remote_n_port_id:24,
			:8;
	uint32_t	relative_offset;
	uint32_t	:2,
			si:1,
			ft:1,
			:2,
			xo:1,
			ls:1,
			df_ctl:8,
			type:8,
			r_ctl:8;
	uint32_t	xri_tag:16,
			context_tag:16;
	uint32_t	dif:2,
			ct:2,
			bs:3,
			:1,
			command:8,
			class:3,
			:1,
			pu:2,
			:2,
			timer:8;
	uint32_t	abort_tag;
	uint32_t	request_tag:16,
			remote_xid:16;
	uint32_t	ebde_cnt:4,
			:3,
			len_loc:2,
			qosd:1,
			:1,
			xbl:1,
			hlm:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			sr:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			:3,
			wqec:1,
			:8,
			cq_id:16;
	uint32_t	sequence_payload_len;
	uint32_t	rsvd13;
	uint32_t	rsvd14;
	uint32_t	rsvd15;
#else
#error big endian version not defined
#endif
} sli4_xmit_sequence64_wqe_t;

/**
 * @brief WQE used unblock the specified XRI and to release it to the SLI Port's free pool.
 */
typedef struct sli4_requeue_xri_wqe_s {
	uint32_t	rsvd0;
	uint32_t	rsvd1;
	uint32_t	rsvd2;
	uint32_t	rsvd3;
	uint32_t	rsvd4;
	uint32_t	rsvd5;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	xri_tag:16,
			context_tag:16;
	uint32_t	:2,
			ct:2,
			:4,
			command:8,
			class:3,
			:1,
			pu:2,
			:2,
			timer:8;
	uint32_t	rsvd8;
	uint32_t	request_tag:16,
			:16;
	uint32_t	ebde_cnt:4,
			rsvd_w10b4:1,
			disable_cqe:1,
			oas:1,
			len_loc:2,
			qosd:1,
			wchn:1,
			xbl:1,
			hlm:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			:3,
			wqec:1,
			:8,
			cq_id:16;
	uint32_t	rsvd12;
	uint32_t	rsvd13;
	uint32_t	rsvd14;
	uint32_t	rsvd15;
#else
#error big endian version not defined
#endif
} sli4_requeue_xri_wqe_t;

/**
 * @brief WQE used to send a single frame sequence to broadcast address
 */
typedef struct sli4_xmit_bcast64_wqe_s {
	sli4_bde_t	sequence_payload;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	sequence_payload_length;
	uint32_t	rsvd4;
	uint32_t	:8,
			df_ctl:8,
			type:8,
			r_ctl:8;
	uint32_t	xri_tag:16,
			context_tag:16;
	uint32_t	:2,
			ct:2,
			:4,
			command:8,
			class:3,
			:1,
			pu:2,
			:2,
			timer:8;
	uint32_t	abort_tag;
	uint32_t	request_tag:16,
			temporary_rpi:16;
	uint32_t	ebde_cnt:4,
			:3,
			len_loc:2,
			qosd:1,
			:1,
			xbl:1,
			hlm:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			:3,
			wqec:1,
			:8,
			cq_id:16;
	uint32_t	rsvd12;
	uint32_t	rsvd13;
	uint32_t	rsvd14;
	uint32_t	rsvd15;
#else
#error big endian version not defined
#endif
} sli4_xmit_bcast64_wqe_t;

/**
 * @brief WQE used to create a BLS response.
 */
typedef struct sli4_xmit_bls_rsp_wqe_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	payload_word0;
	uint32_t	rx_id:16,
			ox_id:16;
	uint32_t	high_seq_cnt:16,
			low_seq_cnt:16;
	uint32_t	rsvd3;
	uint32_t	local_n_port_id:24,
			:8;
	uint32_t	remote_id:24,
			:6,
			ar:1,
			xo:1;
	uint32_t	xri_tag:16,
			context_tag:16;
	uint32_t	:2,
			ct:2,
			:4,
			command:8,
			class:3,
			:1,
			pu:2,
			:2,
			timer:8;
	uint32_t	abort_tag;
	uint32_t	request_tag:16,
			:16;
	uint32_t	ebde_cnt:4,
			:3,
			len_loc:2,
			qosd:1,
			:1,
			xbl:1,
			hlm:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			:3,
			wqec:1,
			:8,
			cq_id:16;
	uint32_t	temporary_rpi:16,
			:16;
	uint32_t	rsvd13;
	uint32_t	rsvd14;
	uint32_t	rsvd15;
#else
#error big endian version not defined
#endif
} sli4_xmit_bls_rsp_wqe_t;

typedef enum {
	SLI_BLS_ACC,
	SLI_BLS_RJT,
	SLI_BLS_MAX
} sli_bls_type_e;

typedef struct sli_bls_payload_s {
	sli_bls_type_e	type;
	uint16_t	ox_id;
	uint16_t	rx_id;
	union {
		struct {
			uint32_t	seq_id_validity:8,
					seq_id_last:8,
					:16;
			uint16_t	ox_id;
			uint16_t	rx_id;
			uint16_t	low_seq_cnt;
			uint16_t	high_seq_cnt;
		} acc;
		struct {
			uint32_t	vendor_unique:8,
					reason_explanation:8,
					reason_code:8,
					:8;
		} rjt;
	} u;
} sli_bls_payload_t;

/**
 * @brief WQE used to create an ELS response.
 */
typedef struct sli4_xmit_els_rsp64_wqe_s {
	sli4_bde_t	els_response_payload;
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	els_response_payload_length;
	uint32_t	s_id:24,
			sp:1,
			:7;
	uint32_t	remote_id:24,
			:8;
	uint32_t	xri_tag:16,
			context_tag:16;
	uint32_t	:2,
			ct:2,
			:4,
			command:8,
			class:3,
			:1,
			pu:2,
			:2,
			timer:8;
	uint32_t	abort_tag;
	uint32_t	request_tag:16,
			ox_id:16;
	uint32_t	ebde_cnt:4,
			:3,
			len_loc:2,
			qosd:1,
			:1,
			xbl:1,
			hlm:1,
			iod:1,
			dbde:1,
			wqes:1,
			pri:3,
			pv:1,
			eat:1,
			xc:1,
			:1,
			ccpe:1,
			ccp:8;
	uint32_t	cmd_type:4,
			:3,
			wqec:1,
			:8,
			cq_id:16;
	uint32_t	temporary_rpi:16,
			:16;
	uint32_t	rsvd13;
	uint32_t	rsvd14;
	uint32_t	rsvd15;
#else
#error big endian version not defined
#endif
} sli4_xmit_els_rsp64_wqe_t;

#define SLI4_RQ_MARKER_CATEGORY_NONE		0x0
#define SLI4_RQ_MARKER_CATEGORY_ALL		0x1
#define SLI4_RQ_MARKER_CATEGORY_ALL_MRQ_SETS	0x3
#define SLI4_RQ_MARKER_CATEGORY_MRQ_SET_1	0x4
#define SLI4_RQ_MARKER_CATEGORY_MRQ_SET_2	0x5
#define SLI4_RQ_MARKER_TYPE_SCSI		0x80000000

typedef struct sli4_fcoe_marker_request_wqe_s {
        uint32_t        rsvd0[3];
#if BYTE_ORDER == LITTLE_ENDIAN
        uint32_t        marker_category: 4,
                        : 28;
        uint32_t        tag_lower;
        uint32_t        tag_higher;
        uint32_t        rsvd1;
        uint32_t        : 8,
                        command: 8,
                        : 16;
        uint32_t        rsvd2;
        uint32_t        req_tag: 16,
                        rq_id: 16;
        uint32_t        ebde_cnt: 4,
                        : 3,
                        len_loc: 2,
                        qosd: 1,
                        rsvd3: 22;
        uint32_t        cmd_type: 4,
                        : 3,
                        wqec: 1,
                        : 8,
                        cq_id: 16;
        uint32_t        rsvd4[4];
#else
#error big endian version not defined
#endif
} sli4_fcoe_marker_request_wqe_t;


/**
 * @brief Asynchronouse Event: Link State ACQE.
 */
typedef struct sli4_link_state_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	link_number:6,
			link_type:2,
			port_link_status:8,
			port_duplex:8,
			port_speed:8;
	uint32_t	port_fault:8,
			:8,
			logical_link_speed:16;
	uint32_t	event_tag;
	uint32_t	:8,
			event_code:8,
			event_type:8,	/** values are protocol specific */
			:6,
			ae:1,		/** async event - this is an ACQE */
			val:1;		/** valid - contents of CQE are valid */
#else
#error big endian version not defined
#endif
} sli4_link_state_t;


#define SLI4_LINK_ATTN_TYPE_LINK_UP		0x01
#define SLI4_LINK_ATTN_TYPE_LINK_DOWN		0x02
#define SLI4_LINK_ATTN_TYPE_NO_HARD_ALPA	0x03
#define SLI4_LINK_ATTN_TYPE_TRUNK_EVENT		0x07

#define SLI4_LINK_ATTN_P2P			0x01
#define SLI4_LINK_ATTN_FC_AL			0x02
#define SLI4_LINK_ATTN_INTERNAL_LOOPBACK	0x03
#define SLI4_LINK_ATTN_SERDES_LOOPBACK		0x04

#define SLI4_LINK_ATTN_1G			0x01
#define SLI4_LINK_ATTN_2G			0x02
#define SLI4_LINK_ATTN_4G			0x04
#define SLI4_LINK_ATTN_8G			0x08
#define SLI4_LINK_ATTN_10G			0x0a
#define SLI4_LINK_ATTN_16G			0x10
#define SLI4_LINK_ATTN_32G			0x20
#define SLI4_LINK_ATTN_64G			0x21
#define SLI4_LINK_ATTN_128G			0x22
#define SLI4_LINK_ATTN_256G			0x23

#define SLI4_LINK_TYPE_ETHERNET			0x0
#define SLI4_LINK_TYPE_FC			0x1

#define SLI4_FC_TRUNK_CONFIG_MASK		0xF0
#define SLI4_FC_TRUNK_LINK_STATE_MASK		0x0F

/*
 * 2 links per trunk config, trunk incorporates either,
 *   a) port 0/1 of a 4-port device
 *   b) both ports of a 2-port device
 */
#define SLI4_FC_TRUNK_2_PORT_CONFIG 		0x30

/* 2 links per trunk config that incorporates port 2/3 of a 4-port device */
#define SLI4_FC_TRUNK_2_PORT_HI_CONFIG		0xC0

/* 4 links per trunk config that incorporates all ports of a 4-port device */
#define SLI4_FC_TRUNK_4_PORT_CONFIG		0xF0

/**
 * @brief Asynchronouse Event: FC Link Attention Event.
 */
typedef struct sli4_link_attention_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	link_number:8,
			attn_type:8,
			topology:8,
			port_speed:8;
	uint32_t	port_fault:8,
			shared_link_status:8,
			logical_link_speed:16;
	uint32_t	event_tag;
	uint32_t	:8,
			event_code:8,
			event_type:8,	/** values are protocol specific */
			:6,
			ae:1,		/** async event - this is an ACQE */
			val:1;		/** valid - contents of CQE are valid */
#else
#error big endian version not defined
#endif
} sli4_link_attention_t;

/**
 * @brief FC/FCoE event types.
 */
#define SLI4_LINK_STATE_PHYSICAL		0x00
#define SLI4_LINK_STATE_LOGICAL			0x01

#define SLI4_FCOE_FIP_FCF_DISCOVERED		0x01
#define SLI4_FCOE_FIP_FCF_TABLE_FULL		0x02
#define SLI4_FCOE_FIP_FCF_DEAD			0x03
#define SLI4_FCOE_FIP_FCF_CLEAR_VLINK		0x04
#define SLI4_FCOE_FIP_FCF_MODIFIED		0x05

#define SLI4_GRP5_QOS_SPEED			0x01

#define SLI4_FC_EVENT_LINK_ATTENTION		0x01
#define SLI4_FC_EVENT_SHARED_LINK_ATTENTION	0x02

#define SLI4_PORT_SPEED_NO_LINK			0x0
#define SLI4_PORT_SPEED_10_MBPS			0x1
#define SLI4_PORT_SPEED_100_MBPS		0x2
#define SLI4_PORT_SPEED_1_GBPS			0x3
#define SLI4_PORT_SPEED_10_GBPS			0x4

#define SLI4_PORT_DUPLEX_NONE			0x0
#define SLI4_PORT_DUPLEX_HALF			0x1
#define SLI4_PORT_DUPLEX_FULL			0x2

#define SLI4_PORT_LINK_STATUS_PHYSICAL_DOWN	0x0
#define SLI4_PORT_LINK_STATUS_PHYSICAL_UP	0x1
#define SLI4_PORT_LINK_STATUS_LOGICAL_DOWN	0x2
#define SLI4_PORT_LINK_STATUS_LOGICAL_UP	0x3

/**
 * @brief Asynchronouse Event: FCoE/FIP ACQE.
 */
typedef struct sli4_fcoe_fip_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	event_information;
	uint32_t	fcf_count:16,
			fcoe_event_type:16;
	uint32_t	event_tag;
	uint32_t	:8,
			event_code:8,
			event_type:8,	/** values are protocol specific */
			:6,
			ae:1,		/** async event - this is an ACQE */
			val:1;		/** valid - contents of CQE are valid */
#else
#error big endian version not defined
#endif
} sli4_fcoe_fip_t;

/**
 * @brief FC/FCoE WQ completion queue entry.
 */
typedef struct sli4_fc_wcqe_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	hw_status:8,
			status:8,
			request_tag:16;
	uint32_t	wqe_specific_1;
	uint32_t	wqe_specific_2;
	uint32_t	:15,
			qx:1,
			code:8,
			pri:3,
			pv:1,
			xb:1,
			rha:1,
			:1,
			vld:1;
#else
#error big endian version not defined
#endif
} sli4_fc_wcqe_t;

/**
 * @brief FC/FCoE WQ consumed CQ queue entry.
 */
typedef struct sli4_fc_wqec_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	:32;
	uint32_t	:32;
	uint32_t	wqe_index:16,
			wq_id:16;
	uint32_t	:16,
			code:8,
			:7,
			vld:1;
#else
#error big endian version not defined
#endif
} sli4_fc_wqec_t;

/**
 * @brief FC/FCoE Completion Status Codes.
 */
#define SLI4_FC_WCQE_STATUS_SUCCESS		0x00
#define SLI4_FC_WCQE_STATUS_FCP_RSP_FAILURE	0x01
#define SLI4_FC_WCQE_STATUS_REMOTE_STOP		0x02
#define SLI4_FC_WCQE_STATUS_LOCAL_REJECT	0x03
#define SLI4_FC_WCQE_STATUS_NPORT_RJT		0x04
#define SLI4_FC_WCQE_STATUS_FABRIC_RJT		0x05
#define SLI4_FC_WCQE_STATUS_NPORT_BSY		0x06
#define SLI4_FC_WCQE_STATUS_FABRIC_BSY		0x07
#define SLI4_FC_WCQE_STATUS_LS_RJT		0x09
#define SLI4_FC_WCQE_STATUS_CMD_REJECT		0x0b
#define SLI4_FC_WCQE_STATUS_FCP_TGT_LENCHECK	0x0c
#define SLI4_FC_WCQE_STATUS_RQ_BUF_LEN_EXCEEDED	0x11
#define SLI4_FC_WCQE_STATUS_RQ_INSUFF_BUF_NEEDED 0x12
#define SLI4_FC_WCQE_STATUS_RQ_INSUFF_FRM_DISC	0x13
#define SLI4_FC_WCQE_STATUS_RQ_DMA_FAILURE	0x14
#define SLI4_FC_WCQE_STATUS_FCP_RSP_TRUNCATE	0x15
#define SLI4_FC_WCQE_STATUS_DI_ERROR		0x16
#define SLI4_FC_WCQE_STATUS_BA_RJT		0x17
#define SLI4_FC_WCQE_STATUS_RQ_INSUFF_XRI_NEEDED 0x18
#define SLI4_FC_WCQE_STATUS_RQ_INSUFF_XRI_DISC	0x19
#define SLI4_FC_WCQE_STATUS_RX_ERROR_DETECT	0x1a
#define SLI4_FC_WCQE_STATUS_RX_ABORT_REQUEST	0x1b

/* driver generated status codes; better not overlap with chip's status codes! */
#define SLI4_FC_WCQE_STATUS_TARGET_WQE_TIMEOUT  0xff
#define SLI4_FC_WCQE_STATUS_SHUTDOWN		0xfe
#define SLI4_FC_WCQE_STATUS_DISPATCH_ERROR	0xfd

/**
 * @brief DI_ERROR Extended Status
 */
#define SLI4_FC_DI_ERROR_GE	(1 << 0) /* Guard Error */
#define SLI4_FC_DI_ERROR_AE	(1 << 1) /* Application Tag Error */
#define SLI4_FC_DI_ERROR_RE	(1 << 2) /* Reference Tag Error */
#define SLI4_FC_DI_ERROR_TDPV	(1 << 3) /* Total Data Placed Valid */
#define SLI4_FC_DI_ERROR_UDB	(1 << 4) /* Uninitialized DIF Block */
#define SLI4_FC_DI_ERROR_EDIR	(1 << 5) /* Error direction */

/**
 * @brief Local Reject Reason Codes.
 */
#define SLI4_FC_LOCAL_REJECT_MISSING_CONTINUE	0x01
#define SLI4_FC_LOCAL_REJECT_SEQUENCE_TIMEOUT	0x02
#define SLI4_FC_LOCAL_REJECT_INTERNAL_ERROR	0x03
#define SLI4_FC_LOCAL_REJECT_INVALID_RPI	0x04
#define SLI4_FC_LOCAL_REJECT_NO_XRI		0x05
#define SLI4_FC_LOCAL_REJECT_ILLEGAL_COMMAND	0x06
#define SLI4_FC_LOCAL_REJECT_XCHG_DROPPED	0x07
#define SLI4_FC_LOCAL_REJECT_ILLEGAL_FIELD	0x08
#define SLI4_FC_LOCAL_REJECT_RPI_SUSPENDED	0x09
#define SLI4_FC_LOCAL_REJECT_NO_ABORT_MATCH	0x0c
#define SLI4_FC_LOCAL_REJECT_TX_DMA_FAILED	0x0d
#define SLI4_FC_LOCAL_REJECT_RX_DMA_FAILED	0x0e
#define SLI4_FC_LOCAL_REJECT_ILLEGAL_FRAME	0x0f
#define SLI4_FC_LOCAL_REJECT_NO_RESOURCES	0x11
#define SLI4_FC_LOCAL_REJECT_FCP_CONF_FAILURE	0x12
#define SLI4_FC_LOCAL_REJECT_ILLEGAL_LENGTH	0x13
#define SLI4_FC_LOCAL_REJECT_UNSUPPORTED_FEATURE 0x14
#define SLI4_FC_LOCAL_REJECT_ABORT_IN_PROGRESS	0x15
#define SLI4_FC_LOCAL_REJECT_ABORT_REQUESTED	0x16
#define SLI4_FC_LOCAL_REJECT_RCV_BUFFER_TIMEOUT	0x17
#define SLI4_FC_LOCAL_REJECT_LOOP_OPEN_FAILURE	0x18
#define SLI4_FC_LOCAL_REJECT_LINK_DOWN		0x1a
#define SLI4_FC_LOCAL_REJECT_CORRUPTED_DATA	0x1b
#define SLI4_FC_LOCAL_REJECT_CORRUPTED_RPI	0x1c
#define SLI4_FC_LOCAL_REJECT_OUT_OF_ORDER_DATA	0x1d
#define SLI4_FC_LOCAL_REJECT_OUT_OF_ORDER_ACK	0x1e
#define SLI4_FC_LOCAL_REJECT_DUP_FRAME		0x1f
#define SLI4_FC_LOCAL_REJECT_LINK_CONTROL_FRAME	0x20
#define SLI4_FC_LOCAL_REJECT_BAD_HOST_ADDRESS	0x21
#define SLI4_FC_LOCAL_REJECT_MISSING_HDR_BUFFER	0x23
#define SLI4_FC_LOCAL_REJECT_MSEQ_CHAIN_CORRUPTED 0x24
#define SLI4_FC_LOCAL_REJECT_ABORTMULT_REQUESTED 0x25
#define SLI4_FC_LOCAL_REJECT_BUFFER_SHORTAGE	0x28
#define SLI4_FC_LOCAL_REJECT_RCV_XRIBUF_WAITING	0x29
#define SLI4_FC_LOCAL_REJECT_INVALID_VPI	0x2e
#define SLI4_FC_LOCAL_REJECT_MISSING_XRIBUF	0x30
#define SLI4_FC_LOCAL_REJECT_INVALID_RELOFFSET	0x40
#define SLI4_FC_LOCAL_REJECT_MISSING_RELOFFSET	0x41
#define SLI4_FC_LOCAL_REJECT_INSUFF_BUFFERSPACE	0x42
#define SLI4_FC_LOCAL_REJECT_MISSING_SI		0x43
#define SLI4_FC_LOCAL_REJECT_MISSING_ES		0x44
#define SLI4_FC_LOCAL_REJECT_INCOMPLETE_XFER	0x45
#define SLI4_FC_LOCAL_REJECT_SLER_FAILURE	0x46
#define SLI4_FC_LOCAL_REJECT_SLER_CMD_RCV_FAILURE 0x47
#define SLI4_FC_LOCAL_REJECT_SLER_REC_RJT_ERR	0x48
#define SLI4_FC_LOCAL_REJECT_SLER_REC_SRR_RETRY_ERR 0x49
#define SLI4_FC_LOCAL_REJECT_SLER_SRR_RJT_ERR	0x4a
#define SLI4_FC_LOCAL_REJECT_SLER_RRQ_RJT_ERR	0x4c
#define SLI4_FC_LOCAL_REJECT_SLER_RRQ_RETRY_ERR	0x4d
#define SLI4_FC_LOCAL_REJECT_SLER_ABTS_ERR	0x4e

typedef struct sli4_fc_async_rcqe_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	:8,
			status:8,
			rq_element_index:12,
			:4;
	uint32_t	rsvd1;
	uint32_t	fcfi:6,
			rq_id:10,
			payload_data_placement_length:16;
	uint32_t	sof_byte:8,
			eof_byte:8,
			code:8,
			header_data_placement_length:6,
			:1,
			vld:1;
#else
#error big endian version not defined
#endif
} sli4_fc_async_rcqe_t;

typedef struct sli4_fc_async_rcqe_v1_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	:8,
			status:8,
			rq_element_index:15,
			:1;
	uint32_t	fcfi:6,
			:26;
	uint32_t	rq_id:16,
			payload_data_placement_length:16;
	uint32_t	sof_byte:8,
			eof_byte:8,
			code:8,
			header_data_placement_length:6,
			:1,
			vld:1;
#else
#error big endian version not defined
#endif
} sli4_fc_async_rcqe_v1_t;

typedef struct sli4_fc_marker_rcqe_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	rsvd0:8,
			status:8,
			rq_element_index:15,
			rsvd1:1;
	uint32_t	tag_lower;
	uint32_t	tag_higher;
	uint32_t	rq_id:16,
			code:8,
			:7,
			vld:1;
#else
#error big endian version not defined
#endif
} sli4_fc_marker_rcqe_t;

#define SLI4_FC_ASYNC_RQ_SUCCESS		0x10
#define SLI4_FC_ASYNC_RQ_BUF_LEN_EXCEEDED	0x11
#define SLI4_FC_ASYNC_RQ_INSUFF_BUF_NEEDED	0x12
#define SLI4_FC_ASYNC_RQ_INSUFF_BUF_FRM_DISC	0x13
#define SLI4_FC_ASYNC_RQ_DMA_FAILURE		0x14

typedef struct sli4_fc_coalescing_rcqe_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	:8,
			status:8,
			rq_element_index:12,
			:4;
	uint32_t	rsvd1;
	uint32_t	rq_id:16,
			sequence_reporting_placement_length:16;
	uint32_t	:16,
			code:8,
			:7,
			vld:1;
#else
#error big endian version not defined
#endif
} sli4_fc_coalescing_rcqe_t;

#define SLI4_FC_COALESCE_RQ_SUCCESS		0x10
#define SLI4_FC_COALESCE_RQ_INSUFF_XRI_NEEDED	0x18

typedef struct sli4_fc_optimized_write_cmd_cqe_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	:8,
			status:8,
			rq_element_index:15,
			iv:1;
	uint32_t	fcfi:6,
			:8,
			oox:1,
			tow:1,
			xri:16;
	uint32_t	rq_id:16,
			payload_data_placement_length:16;
	uint32_t	rpi:16,
			code:8,
			header_data_placement_length:6,
			:1,
			vld:1;
#else
#error big endian version not defined
#endif
} sli4_fc_optimized_write_cmd_cqe_t;

typedef struct sli4_fc_optimized_write_data_cqe_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	hw_status:8,
			status:8,
			xri:16;
	uint32_t	total_data_placed;
	uint32_t	extended_status;
	uint32_t	:16,
			code:8,
			pri:3,
			pv:1,
			xb:1,
			rha:1,
			:1,
			vld:1;
#else
#error big endian version not defined
#endif
} sli4_fc_optimized_write_data_cqe_t;

typedef struct sli4_fc_xri_aborted_cqe_s {
#if BYTE_ORDER == LITTLE_ENDIAN
	uint32_t	:8,
			status:8,
			:16;
	uint32_t	extended_status;
	uint32_t	xri:16,
			remote_xid:16;
	uint32_t	:16,
			code:8,
			xr:1,
			:3,
			eo:1,
			br:1,
			ia:1,
			vld:1;
#else
#error big endian version not defined
#endif
} sli4_fc_xri_aborted_cqe_t;

/**
 * Code definitions applicable to all FC/FCoE CQE types.
 */
#define SLI4_CQE_CODE_OFFSET		14

#define SLI4_CQE_CODE_WORK_REQUEST_COMPLETION	0x01
#define SLI4_CQE_CODE_RELEASE_WQE		0x02
#define SLI4_CQE_CODE_RQ_ASYNC			0x04
#define SLI4_CQE_CODE_XRI_ABORTED		0x05
#define SLI4_CQE_CODE_RQ_COALESCING		0x06
#define SLI4_CQE_CODE_RQ_CONSUMPTION		0x07
#define SLI4_CQE_CODE_MEASUREMENT_REPORTING	0x08
#define SLI4_CQE_CODE_RQ_ASYNC_V1		0x09
#define SLI4_CQE_CODE_OPTIMIZED_WRITE_CMD	0x0B
#define SLI4_CQE_CODE_OPTIMIZED_WRITE_DATA	0x0C
#define SLI4_CQE_CODE_MARKER			0x1D

extern int32_t sli_fc_process_link_state(sli4_t *, void *);
extern int32_t sli_fc_process_link_attention(sli4_t *, void *);
extern int32_t sli_fc_process_sli_port_event(sli4_t *, void *);
extern int32_t sli_fc_cqe_parse(sli4_t *, sli4_queue_t *, uint8_t *, sli4_qentry_e *, uint16_t *);
extern uint32_t sli_fc_response_length(sli4_t *, uint8_t *);
extern uint32_t sli_fc_io_length(sli4_t *, uint8_t *);
extern int32_t sli_fc_els_did(sli4_t *, uint8_t *, uint32_t *);
extern uint32_t sli_fc_ext_status(sli4_t *, uint8_t *);
extern int32_t sli_fc_rqe_rqid_and_index(sli4_t *, uint8_t *, uint16_t *, uint32_t *);
extern int32_t sli_fc_process_fcoe(sli4_t *, void *);
extern int32_t sli_cmd_fcoe_wq_create(sli4_t *, void *, size_t, ocs_dma_t *, uint16_t, uint16_t, bool);
extern int32_t sli_cmd_fcoe_wq_create_v1(sli4_t *, void *, size_t, ocs_dma_t *, uint16_t, uint16_t, bool);
extern int32_t sli_cmd_fcoe_wq_destroy(sli4_t *, void *, size_t, uint16_t);
extern int32_t sli_cmd_fcoe_post_sgl_pages(sli4_t *, void *, size_t, uint16_t, uint32_t, ocs_dma_t **, ocs_dma_t *);
extern int32_t sli_cmd_fcoe_rq_create(sli4_t *, void *, size_t, ocs_dma_t *, uint16_t, uint16_t, uint16_t);
extern int32_t sli_cmd_fcoe_rq_create_v1(sli4_t *, void *, size_t, ocs_dma_t *, uint16_t, uint16_t, uint16_t);
extern int32_t sli_cmd_fcoe_rq_destroy(sli4_t *, void *, size_t, uint16_t);
extern int32_t sli_cmd_fcoe_read_fcf_table(sli4_t *, void *, size_t, ocs_dma_t *, uint16_t);
extern int32_t sli_cmd_fcoe_post_hdr_templates(sli4_t *, void *, size_t, ocs_dma_t *, uint16_t, ocs_dma_t *);
extern int32_t sli_cmd_fcoe_rediscover_fcf(sli4_t *, void *, size_t, uint16_t);
extern int32_t sli_fc_rq_alloc(sli4_t *, sli4_queue_t *, uint32_t, uint32_t, sli4_queue_t *, uint16_t, uint8_t);
extern int32_t sli_fc_rq_set_alloc(sli4_t *, uint32_t, sli4_queue_t *[], uint32_t, uint32_t, uint32_t, uint32_t, uint16_t);
extern uint32_t sli_fc_get_rpi_requirements(sli4_t *, uint32_t);
extern int32_t sli_abort_wqe(sli4_t *, void *, size_t, sli4_abort_type_e, uint32_t, uint32_t, uint32_t, uint16_t, uint16_t);

extern int32_t sli_els_request64_wqe(sli4_t *, void *, size_t, ocs_dma_t *, uint8_t, uint32_t, uint32_t, uint8_t, uint16_t, uint16_t, uint16_t, ocs_remote_node_t *, ocs_sport_t *);
extern int32_t sli_fcp_iread64_wqe(sli4_t *, void *, size_t, ocs_dma_t *, uint32_t, uint32_t, uint16_t, uint16_t, uint16_t, uint32_t, ocs_remote_node_t *, uint8_t, uint8_t, uint8_t);
extern int32_t sli_fcp_iwrite64_wqe(sli4_t *, void *, size_t, ocs_dma_t *, uint32_t, uint32_t, uint32_t, uint16_t, uint16_t, uint16_t, uint32_t, ocs_remote_node_t *, uint8_t, uint8_t, uint8_t);
extern int32_t sli_fcp_icmnd64_wqe(sli4_t *, void *, size_t, ocs_dma_t *, uint16_t, uint16_t, uint16_t, uint32_t, ocs_remote_node_t *, uint8_t);

extern int32_t sli_fcp_treceive64_wqe(sli4_t *, void *, size_t, ocs_dma_t *, uint32_t, uint32_t, uint32_t, uint16_t, uint16_t, uint16_t, uint16_t, uint32_t, ocs_remote_node_t *, uint32_t, uint8_t, uint8_t, uint8_t, uint32_t, uint8_t);
extern int32_t sli_fcp_trsp64_wqe(sli4_t *, void *, size_t, ocs_dma_t *, uint32_t, uint16_t, uint16_t, uint16_t, uint16_t, uint32_t, ocs_remote_node_t *, uint32_t, uint8_t, uint8_t, uint32_t);
extern int32_t sli_fcp_tsend64_wqe(sli4_t *, void *, size_t, ocs_dma_t *, uint32_t, uint32_t, uint32_t, uint16_t, uint16_t, uint16_t, uint16_t, uint32_t, ocs_remote_node_t *, uint32_t, uint8_t, uint8_t, uint8_t, uint32_t);
extern int32_t sli_fcp_cont_treceive64_wqe(sli4_t *, void*, size_t, ocs_dma_t *, uint32_t, uint32_t, uint32_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint32_t, ocs_remote_node_t *, uint32_t, uint8_t, uint8_t, uint8_t, uint32_t, uint8_t);
extern int32_t sli_gen_request64_wqe(sli4_t *, void *, size_t, ocs_dma_t *, uint32_t, uint32_t,uint8_t, uint16_t, uint16_t, uint16_t, ocs_remote_node_t *, uint8_t, uint8_t, uint8_t);
extern int32_t sli_send_frame_wqe(sli4_t *sli4, void *buf, size_t size, uint8_t sof, uint8_t eof, uint32_t *hdr,
				  ocs_dma_t *payload, uint32_t req_len, uint8_t timeout,
				  uint16_t xri, uint16_t req_tag);
extern void sli4_fcoe_marker_request_wqe(sli4_t *sli4, void *buf, uint32_t ctx_tag, uint16_t req_tag, uint8_t category);
extern int32_t sli_xmit_sequence64_wqe(sli4_t *, void *, size_t, ocs_dma_t *, uint32_t, uint8_t, uint16_t, uint16_t, uint16_t, ocs_remote_node_t *, uint8_t, uint8_t, uint8_t);
extern int32_t sli_xmit_bcast64_wqe(sli4_t *, void *, size_t, ocs_dma_t *, uint32_t, uint8_t, uint16_t, uint16_t, uint16_t, ocs_remote_node_t *, uint8_t, uint8_t, uint8_t);
extern int32_t sli_xmit_bls_rsp64_wqe(sli4_t *, void *, size_t, sli_bls_payload_t *, uint16_t, uint16_t, uint16_t, ocs_remote_node_t *, uint32_t);
extern int32_t sli_xmit_els_rsp64_wqe(sli4_t *, void *, size_t, ocs_dma_t *, uint32_t, uint16_t, uint16_t, uint16_t, uint16_t, ocs_remote_node_t *, uint32_t, uint32_t);
extern int32_t sli_requeue_xri_wqe(sli4_t *, void *, size_t, uint16_t, uint16_t, uint16_t);
extern void sli4_cmd_lowlevel_set_watchdog(sli4_t *sli4, void *buf, size_t size, uint16_t timeout);
extern void sli4_cmd_lowlevel_enable_ras(sli4_t *sli4, void *buf, size_t size, int loglevel, uint32_t unit_size, int32_t buf_count, uint32_t *phys_addr, uintptr_t lwpd_phys_addr);
extern void sli4_cmd_lowlevel_disable_ras(sli4_t *sli4, void *buf, size_t size);
extern void sli4_cmd_lowlevel_get_itcm_parity_stats(sli4_t *sli4, void *buf, size_t size, bool clear_stats);

/**
 * @ingroup sli_fc
 * @brief Retrieve the received header and payload length.
 *
 * @param sli4 SLI context.
 * @param cqe Pointer to the CQ entry.
 * @param len_hdr Pointer where the header length is written.
 * @param len_data Pointer where the payload length is written.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static inline int32_t
sli_fc_rqe_length(sli4_t *sli4, void *cqe, uint32_t *len_hdr, uint32_t *len_data)
{
	sli4_fc_async_rcqe_t	*rcqe = cqe;

	*len_hdr = *len_data = 0;

	if (SLI4_FC_ASYNC_RQ_SUCCESS == rcqe->status) {
		*len_hdr  = rcqe->header_data_placement_length;
		*len_data = rcqe->payload_data_placement_length;
		return 0;
	} else {
		return -1;
	}
}

/**
 * @ingroup sli_fc
 * @brief Retrieve the received FCFI.
 *
 * @param sli4 SLI context.
 * @param cqe Pointer to the CQ entry.
 *
 * @return Returns the FCFI in the CQE. or UINT8_MAX if invalid CQE code.
 */
static inline uint8_t
sli_fc_rqe_fcfi(sli4_t *sli4, void *cqe)
{
	uint8_t code = ((uint8_t*)cqe)[SLI4_CQE_CODE_OFFSET];
	uint8_t fcfi = UINT8_MAX;

	switch(code) {
	case SLI4_CQE_CODE_RQ_ASYNC: {
		sli4_fc_async_rcqe_t *rcqe = cqe;
		fcfi = rcqe->fcfi;
		break;
	}
	case SLI4_CQE_CODE_RQ_ASYNC_V1: {
		sli4_fc_async_rcqe_v1_t *rcqev1 = cqe;
		fcfi = rcqev1->fcfi;
		break;
	}
	case SLI4_CQE_CODE_OPTIMIZED_WRITE_CMD: {
		sli4_fc_optimized_write_cmd_cqe_t *opt_wr = cqe;
		fcfi = opt_wr->fcfi;
		break;
	}
	}

	return fcfi;
}

extern const char *sli_fc_get_status_string(uint32_t status);

#endif /* !_SLI4_FC_H */

/* vim: set noexpandtab textwidth=120: */

 /**
 * @page fc_sli4_api_overview SLI-4 APIs
 * - @ref sli
 * - @ref sli_fc
 *
 * <div class="overview">
 * <img src="elx_fc_sli4.jpg" alt="FC/FCoE SLI-4" title="FC/FCoE SLI-4" align="right"/>
 *
 * <h2>FC/FCoE SLI-4</h2>
 *
 * The FC/FCoE SLI-4 component implements the commands and processing defined by
 * the <i>SLI-4 Architecture Specification</i> and the <i>SLI-4 FC and FCoE Command Reference</i>
 * through a lightly-abstracted API. The primary objective of this component is to provide
 * a mechanism to access the Broadcom SLI-4 hardware without enforcing a specific policy.
 * For example, this API provides a mechanism to unregister an RPI (sli_cmd_unreg_rpi()),
 * but does not provide details on when to invoke this function.<br><br>
 *
 * All meaningful communication between the driver and hardware occurs via queues. To
 * issue commands and work requests to the hardware, the driver writes work queue
 * entries (WQEs). Conversely, to get information about asynchronous events (for
 * example, link events or previously submitted commands), the driver reads queue
 * entries created by the hardware. The SLI component defines a set of common queue
 * operations used to interact with the hardware, simplifying some of the underlying
 * interface complexities. You can use the queue API to perform the following operations:
 *
 * <ul><li>Allocating a queue type with the given number of entries. Since most queues
 * are closely coupled to an associated queue (for example, a completion queue is
 * tied to an event queue), this interface allows specifying the associated queue.
 * <li>Freeing a previously allocated queue.</li>
 * <li>Determining if the next queue entry is valid (that is, the hardware has written a
 * new entry).</li>
 * <li>Reading the next valid entry from the queue to a supplied buffer.</li>
 * <li>Writing an entry from a supplied buffer to the next available queue location.</li></ul>
 *
 * The SLI-4 documentation defines registers that are predominantly used in conjunction
 * with the previously mentioned queue operations. One exception to this
 * characterization of SLI-4 registers is the bootstrap mailbox register (bmbx). The
 * bootstrap mailbox register allows the driver to send commands before any of the other
 * queues exist. Similar to the standard queue operations, the SLI component encapsulates
 * the standard mechanism used to communicate via the bootstrap mailbox.<br><br>
 *
 * The OneCore Storage driver writes two types of queue entries: commands and I/O
 * work requests. In both cases, the SLI-4 component provides helper functions to
 * correctly format these queue entries.<br><br>
 *
 * The basic format of the command helper functions includes:
 *
 * <ul><li>A name corresponding to the SLI-4 documentation. For example, the helper
 * function for the INIT_LINK command is sli_cmd_init_link().</li>
 * <li>A buffer to hold the formatted queue entry.</li>
 * <li>The size of the allocated buffer.</li>
 * <li>Parameter options, where appropriate. For example, the INIT_VPI command
 * requires values for the VPI and VFI. Therefore, the sli_cmd_init_vpi() function
 * provides two additional parameters to supply the VPI and VFI values.</li></ul>
 *
 * The basic format of the I/O work request helper functions follow a similar format, and
 * includes:
 *
 * <ul><li>A name corresponding to the SLI-4 documentation. For example, the helper
 * function to create an ELS_REQUEST64_WQE is sli_els_request64_wqe().</li>
 * <li>A buffer to hold the formatted queue entry.</li>
 * <li>The size of the allocated buffer.</li>
 * <li>Parameterized command options where appropriate. For example, the
 * ELS_REQUEST64_WQE work request requires values for the XRI and request
 * tag. Therefore, the sli_els_request64_wqe() function has additional parameters
 * for the XRI and tag.</li></ul>
 *
 * Once formatted with a helper function, the queue entry can be passed to
 * sli_queue_write() to post it to the hardware.<br><br>
 *
 * @b Note: The formatted command buffers may also be passed to the hardware through
 * the bootstrap mailbox interface (sli_bmbx_command()).<br><br>
 *
 * The SLI-4 component provides assistance in decoding entries created by the hardware
 * and read by the driver. In some cases, a SLI-4 function receives the raw buffer as input,
 * and then returns more relevant values. For example, the sli_cq_parse() function
 * receives the completion queue entry as input, and then returns the completion type and
 * queue ID. In other cases, the parsing function decodes the queue entry, and then
 * returns the results through registered callback functions. For example, the
 * sli_cqe_async() helper function decodes asynchronous events, such as FC link events
 * and FCoE FIP events. The drivers can register callback functions for each of these
 * events using the SLI component.<br><br>
 *
 * In contrast to SLI-2 and SLI-3 drivers, SLI-4 drivers play a greater role in the allocation
 * and assignment of hardware resources (for example, XRI, RPI, and VPI). However, the
 * inner workings of these resources differ between different hardware. The SLI-4
 * component encapsulates these differences and presents a common interface to upper
 * layers. The API presents a simple allocate or free interface.<br><br>
 *
 * The SLI-4 covers a range of hardware, each with different capabilities. The SLI
 * component API provides functions to manage these differences. These functions allow
 * drivers to query hardware capabilities (for example, the maximum SGL length), as well
 * as configure capabilities (for example, setting the FC topology to private loop).
 * <br><br>
 * </div><!-- overview -->
 */
