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
 * @defgroup sli_fc SLI-4 FC/FCoE APIs
 */

/**
 * @file
 * Implementation of Fibre Channel SLI-4 functions.
 */

#include "ocs_os.h"
#include "sli4_fc.h"
#if defined(OCS_INCLUDE_DEBUG)
#include "ocs_debug.h"
#endif

extern const char *SLI_QNAME[];
extern const sli4_reg_t regmap[SLI4_REG_MAX][SLI4_MAX_IF_TYPES];

/**
 * @ingroup sli_fc
 * @brief Write an FCOE_WQ_CREATE command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param qmem DMA memory for the queue.
 * @param cq_id Associated CQ_ID.
 * @param ulp The ULP to bind
 * @param ignored Ignored (Used for consistency among queue creation functions)
 *
 * @note This creates a Version 0 message.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_fcoe_wq_create(sli4_t *sli4, void *buf, size_t size,
		       ocs_dma_t *qmem, uint16_t cq_id, uint16_t ulp, bool ignored)
{
	sli4_req_fcoe_wq_create_t	*wq = NULL;
	uint32_t	p, sli_config_off = 0;
	uintptr_t	addr;
	size_t		qmem_size = sli_get_qmem_size(qmem);
	uint32_t	qpage_size = sli_get_qpage_size(qmem_size);

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		uint32_t payload_size;

		/* Payload length must accomodate both request and response */
		payload_size = OCS_MAX(sizeof(sli4_req_fcoe_wq_create_t),
				sizeof(sli4_res_common_create_queue_t));
		sli_config_off = sli_cmd_sli_config(sli4, buf, size, payload_size, NULL);
	}
	wq = (sli4_req_fcoe_wq_create_t *)((uint8_t *)buf + sli_config_off);

	wq->hdr.opcode = SLI4_OPC_FCOE_WQ_CREATE;
	wq->hdr.subsystem = SLI4_SUBSYSTEM_FCFCOE;
	wq->hdr.request_length = sizeof(sli4_req_fcoe_wq_create_t) - sizeof(sli4_req_hdr_t);

	/* valid values for number of pages: 1-4 (sec 4.5.1) */
	wq->num_pages = sli_page_count(qmem_size, qpage_size);
	if (!wq->num_pages || (wq->num_pages > SLI4_FCOE_WQ_CREATE_V0_MAX_PAGES)) {
		return 0;
	}

	wq->cq_id = cq_id;

	if (sli4->config.dual_ulp_capable) {
		wq->dua = 1;
		wq->bqu = 1;
		wq->ulp = ulp;
	}

	for (p = 0; p < wq->num_pages; p++) {
		addr = qmem[p].phys;
		wq->page_physical_address[p].low  = ocs_addr32_lo(addr);
		wq->page_physical_address[p].high = ocs_addr32_hi(addr);
	}

	return(sli_config_off + sizeof(sli4_req_fcoe_wq_create_t));
}

/**
 * @ingroup sli_fc
 * @brief Write an FCOE_WQ_CREATE_V1 command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param qmem DMA memory for the queue.
 * @param cq_id Associated CQ_ID.
 * @param ignored This parameter carries the ULP for WQ (ignored for V1)
 * @param sfq WQ for Send Frames
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_fcoe_wq_create_v1(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *qmem,
			  uint16_t cq_id, uint16_t ignored, bool sfq)
{
	sli4_req_fcoe_wq_create_v1_t	*wq = NULL;
	uint32_t	p, sli_config_off = 0;
	uintptr_t	addr;
	size_t		qmem_size = sli_get_qmem_size(qmem);
	uint32_t	qpage_size = sli_get_qpage_size(qmem_size);

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		uint32_t payload_size;

		/* Payload length must accomodate both request and response */
		payload_size = OCS_MAX(sizeof(sli4_req_fcoe_wq_create_v1_t),
				sizeof(sli4_res_common_create_queue_t));

		sli_config_off = sli_cmd_sli_config(sli4, buf, size, payload_size, NULL);
	}
	wq = (sli4_req_fcoe_wq_create_v1_t *)((uint8_t *)buf + sli_config_off);

	wq->hdr.opcode = SLI4_OPC_FCOE_WQ_CREATE;
	wq->hdr.subsystem = SLI4_SUBSYSTEM_FCFCOE;
	wq->hdr.request_length = sizeof(sli4_req_fcoe_wq_create_v1_t) - sizeof(sli4_req_hdr_t);
	wq->hdr.version = 1;

	/* valid values for number of pages: 1-8 */
	wq->num_pages = sli_page_count(qmem_size, qpage_size);
	if (!wq->num_pages || (wq->num_pages > SLI4_FCOE_WQ_CREATE_V1_MAX_PAGES)) {
		return 0;
	}

	wq->cq_id = cq_id;

	/* Fill the page_size_multiplier */
	wq->page_size = (qpage_size / SLI_PAGE_SIZE);

	if (sli4->config.wqe_size == SLI4_WQE_EXT_BYTES)
		wq->wqe_size = SLI4_WQE_EXT_SIZE;
	else
		wq->wqe_size = SLI4_WQE_SIZE;

	if (sli4->config.dpp)
		wq->dpp = 1;

	wq->sfq = sfq;
	wq->wqe_count = (qmem_size / sli4->config.wqe_size);

	for (p = 0; p < wq->num_pages; p++) {
		addr = qmem[p].phys;
		wq->page_physical_address[p].low  = ocs_addr32_lo(addr);
		wq->page_physical_address[p].high = ocs_addr32_hi(addr);
	}

	return(sli_config_off + sizeof(sli4_req_fcoe_wq_create_v1_t));
}

/**
 * @ingroup sli_fc
 * @brief Write an FCOE_WQ_DESTROY command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param wq_id WQ_ID.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_fcoe_wq_destroy(sli4_t *sli4, void *buf, size_t size, uint16_t wq_id)
{
	sli4_req_fcoe_wq_destroy_t	*wq = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		uint32_t payload_size;

		/* Payload length must accomodate both request and response */
		payload_size = max(sizeof(sli4_req_fcoe_wq_destroy_t),
				sizeof(sli4_res_hdr_t));

		sli_config_off = sli_cmd_sli_config(sli4, buf, size, payload_size,
				NULL);
	}
	wq = (sli4_req_fcoe_wq_destroy_t *)((uint8_t *)buf + sli_config_off);

	wq->hdr.opcode = SLI4_OPC_FCOE_WQ_DESTROY;
	wq->hdr.subsystem = SLI4_SUBSYSTEM_FCFCOE;
	wq->hdr.request_length = sizeof(sli4_req_fcoe_wq_destroy_t) -
					sizeof(sli4_req_hdr_t);

	wq->wq_id = wq_id;

	return(sli_config_off + sizeof(sli4_req_fcoe_wq_destroy_t));
}

/**
 * @ingroup sli_fc
 * @brief Write an FCOE_POST_SGL_PAGES command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param xri starting XRI
 * @param xri_count XRI
 * @param SGL memory.
 * @param dma DMA buffer for non-embedded mailbox command (options)
 *
 * if non-embedded mbx command is used, dma buffer must be at least (32 + xri_count*16) in length
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_fcoe_post_sgl_pages(sli4_t *sli4, void *buf, size_t size,
		uint16_t xri, uint32_t xri_count, ocs_dma_t *sgl[], ocs_dma_t *dma)
{
	sli4_req_fcoe_post_sgl_pages_t	*post = NULL;
	uint32_t	sli_config_off = 0;
	uint32_t	i;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		uint32_t payload_size;

		/* Payload length must accomodate both request and response */
		payload_size = max(sizeof(sli4_req_fcoe_post_sgl_pages_t) +
				   xri_count*sizeof(sli4_fcoe_post_sgl_page_desc_t),
				   sizeof(sli4_res_hdr_t));

		sli_config_off = sli_cmd_sli_config(sli4, buf, size, payload_size,
				dma);
	}
	if (dma) {
		post = dma->virt;
		ocs_memset(post, 0, dma->size);
	} else {
		post = (sli4_req_fcoe_post_sgl_pages_t *)((uint8_t *)buf + sli_config_off);
	}

	post->hdr.opcode = SLI4_OPC_FCOE_POST_SGL_PAGES;
	post->hdr.subsystem = SLI4_SUBSYSTEM_FCFCOE;
	// payload size calculation
	//   4 = xri_start + xri_count
	//   xri_count = # of XRI's registered
	//   sizeof(uint64_t) = physical address size
	//   2 = # of physical addresses per page set
	post->hdr.request_length = 4 + (xri_count * (sizeof(uint64_t) * 2));

	post->xri_start = xri;
	post->xri_count = xri_count;

	for (i = 0; i < xri_count; i++) {
		post->page_desc[i].page0_low  = ocs_addr32_lo(sgl[i]->phys);
		post->page_desc[i].page0_high = ocs_addr32_hi(sgl[i]->phys);

		if (sgl[i]->size > SLI_PAGE_SIZE) {
			post->page_desc[i].page1_low  = ocs_addr32_lo(sgl[i]->phys + SLI_PAGE_SIZE);
			post->page_desc[i].page1_high = ocs_addr32_hi(sgl[i]->phys + SLI_PAGE_SIZE);
		}
	}

	return dma ? sli_config_off : (sli_config_off + sizeof(sli4_req_fcoe_post_sgl_pages_t));
}

/**
 * @ingroup sli_fc
 * @brief Write an FCOE_RQ_CREATE command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param qmem DMA memory for the queue.
 * @param cq_id Associated CQ_ID.
 * @param ulp This parameter carries the ULP for the RQ
 * @param buffer_size Buffer size pointed to by each RQE.
 *
 * @note This creates a Version 0 message.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_fcoe_rq_create(sli4_t *sli4, void *buf, size_t size,
		ocs_dma_t *qmem, uint16_t cq_id, uint16_t ulp, uint16_t buffer_size)
{
	sli4_req_fcoe_rq_create_t	*rq = NULL;
	uint32_t	p, sli_config_off = 0;
	uintptr_t	addr;
	size_t		qmem_size = sli_get_qmem_size(qmem);
	uint32_t	qpage_size = sli_get_qpage_size(qmem_size);

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		uint32_t payload_size;

		/* Payload length must accomodate both request and response */
		payload_size = OCS_MAX(sizeof(sli4_req_fcoe_rq_create_t),
				sizeof(sli4_res_common_create_queue_t));

		sli_config_off = sli_cmd_sli_config(sli4, buf, size, payload_size, NULL);
	}
	rq = (sli4_req_fcoe_rq_create_t *)((uint8_t *)buf + sli_config_off);

	rq->hdr.opcode = SLI4_OPC_FCOE_RQ_CREATE;
	rq->hdr.subsystem = SLI4_SUBSYSTEM_FCFCOE;
	rq->hdr.request_length = sizeof(sli4_req_fcoe_rq_create_t) - sizeof(sli4_req_hdr_t);

	/* valid values for number of pages: 1-8 (sec 4.5.6) */
	rq->num_pages = sli_page_count(qmem_size, qpage_size);
	if (!rq->num_pages || (rq->num_pages > SLI4_FCOE_RQ_CREATE_V0_MAX_PAGES)) {
		ocs_log_test(sli4->os, "num_pages %d not valid\n", rq->num_pages);
		return 0;
	}

	/*
	 * RQE count is the log base 2 of the total number of entries
	 */
	rq->rqe_count = ocs_lg2(qmem_size / SLI4_FCOE_RQE_SIZE);

	if ((buffer_size < SLI4_FCOE_RQ_CREATE_V0_MIN_BUF_SIZE) ||
			(buffer_size > SLI4_FCOE_RQ_CREATE_V0_MAX_BUF_SIZE)) {
		ocs_log_err(sli4->os, "buffer_size %d out of range (%d-%d)\n",
				buffer_size,
				SLI4_FCOE_RQ_CREATE_V0_MIN_BUF_SIZE,
				SLI4_FCOE_RQ_CREATE_V0_MAX_BUF_SIZE);
		return -1;
	}
	rq->buffer_size = buffer_size;

	rq->cq_id = cq_id;

	if (sli4->config.dual_ulp_capable) {
		rq->dua = 1;
		rq->bqu = 1;
		rq->ulp = ulp;
	}

	for (p = 0; p < rq->num_pages; p++) {
		addr = qmem[p].phys;
		rq->page_physical_address[p].low  = ocs_addr32_lo(addr);
		rq->page_physical_address[p].high = ocs_addr32_hi(addr);
	}

	return(sli_config_off + sizeof(sli4_req_fcoe_rq_create_t));
}

/**
 * @ingroup sli_fc
 * @brief Write an FCOE_RQ_CREATE_V1 command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param qmem DMA memory for the queue.
 * @param cq_id Associated CQ_ID.
 * @param ulp This parameter carries the ULP for RQ (ignored for V1)
 * @param buffer_size Buffer size pointed to by each RQE.
 *
 * @note This creates a Version 0 message
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_fcoe_rq_create_v1(sli4_t *sli4, void *buf, size_t size,
		ocs_dma_t *qmem, uint16_t cq_id, uint16_t ulp, uint16_t buffer_size)
{
	sli4_req_fcoe_rq_create_v1_t	*rq = NULL;
	uintptr_t	addr;
	uint32_t	p, sli_config_off = 0;
	size_t		qmem_size = sli_get_qmem_size(qmem);
	uint32_t	qpage_size = sli_get_qpage_size(qmem_size);
	uint32_t	total_rq_entries;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		uint32_t payload_size;

		/* Payload length must accomodate both request and response */
		payload_size = OCS_MAX(sizeof(sli4_req_fcoe_rq_create_v1_t),
				sizeof(sli4_res_common_create_queue_t));

		sli_config_off = sli_cmd_sli_config(sli4, buf, size, payload_size, NULL);
	}
	rq = (sli4_req_fcoe_rq_create_v1_t *)((uint8_t *)buf + sli_config_off);

	rq->hdr.opcode = SLI4_OPC_FCOE_RQ_CREATE;
	rq->hdr.subsystem = SLI4_SUBSYSTEM_FCFCOE;
	rq->hdr.request_length = sizeof(sli4_req_fcoe_rq_create_v1_t) - sizeof(sli4_req_hdr_t);
	rq->hdr.version = 1;

	rq->dnb = sli4->config.rq_dnb;
	/* valid values for number of pages: 1-8 (sec 4.5.6) */
	rq->num_pages = sli_page_count(qmem_size, qpage_size);
	if (!rq->num_pages || (rq->num_pages > SLI4_FCOE_RQ_CREATE_V1_MAX_PAGES)) {
		ocs_log_test(sli4->os, "num_pages %d not valid, max %d\n",
				rq->num_pages, SLI4_FCOE_RQ_CREATE_V1_MAX_PAGES);
		return 0;
	}

	/*
	 * RQE count is the total number of entries (note not lg2(# entries))
	 */
	total_rq_entries = (qmem_size / SLI4_FCOE_RQE_SIZE);
	if (sli4->asic_type == SLI4_ASIC_TYPE_LANCERG7PLUS) {
		/* Prism+ device with larger queues */
		rq->rqe_count = total_rq_entries & 0xFFFF;
		rq->rqe_count_hi = (total_rq_entries >> 16) & 0xF;	/* MSB 4 bits */
	} else {
		rq->rqe_count = total_rq_entries;
	}

	rq->rqe_size = SLI4_FCOE_RQE_SIZE_8;

	/* Fill the page_size_multiplier */
	rq->page_size = qpage_size / SLI_PAGE_SIZE;

	if ((buffer_size < sli4->config.rq_min_buf_size) ||
	    (buffer_size > sli4->config.rq_max_buf_size)) {
		ocs_log_err(sli4->os, "buffer_size %d out of range (%d-%d)\n",
				buffer_size,
				sli4->config.rq_min_buf_size,
				sli4->config.rq_max_buf_size);
		return -1;
	}
	rq->buffer_size = buffer_size;

	rq->cq_id = cq_id;

	for (p = 0; p < rq->num_pages; p++) {
		addr = qmem[p].phys;
		rq->page_physical_address[p].low  = ocs_addr32_lo(addr);
		rq->page_physical_address[p].high = ocs_addr32_hi(addr);
	}

	return(sli_config_off + sizeof(sli4_req_fcoe_rq_create_v1_t));
}

/**
 * @ingroup sli_fc
 * @brief Write an FCOE_RQ_DESTROY command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param rq_id RQ_ID.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_fcoe_rq_destroy(sli4_t *sli4, void *buf, size_t size, uint16_t rq_id)
{
	sli4_req_fcoe_rq_destroy_t	*rq = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		uint32_t payload_size;

		/* Payload length must accomodate both request and response */
		payload_size = max(sizeof(sli4_req_fcoe_rq_destroy_t),
				sizeof(sli4_res_hdr_t));

		sli_config_off = sli_cmd_sli_config(sli4, buf, size, payload_size,
				NULL);
	}
	rq = (sli4_req_fcoe_rq_destroy_t *)((uint8_t *)buf + sli_config_off);

	rq->hdr.opcode = SLI4_OPC_FCOE_RQ_DESTROY;
	rq->hdr.subsystem = SLI4_SUBSYSTEM_FCFCOE;
	rq->hdr.request_length = sizeof(sli4_req_fcoe_rq_destroy_t) -
					sizeof(sli4_req_hdr_t);

	rq->rq_id = rq_id;

	return(sli_config_off + sizeof(sli4_req_fcoe_rq_destroy_t));
}

/**
 * @ingroup sli_fc
 * @brief Write an FCOE_READ_FCF_TABLE command.
 *
 * @note
 * The response of this command exceeds the size of an embedded
 * command and requires an external buffer with DMA capability to hold the results.
 * The caller should allocate the ocs_dma_t structure / memory.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param dma Pointer to DMA memory structure. This is allocated by the caller.
 * @param index FCF table index to retrieve.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_fcoe_read_fcf_table(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *dma, uint16_t index)
{
	sli4_req_fcoe_read_fcf_table_t *read_fcf = NULL;

	if (SLI4_PORT_TYPE_FC != sli4->port_type) {
		ocs_log_test(sli4->os, "FCOE_READ_FCF_TABLE only supported on FC\n");
		return -1;
	}

	read_fcf = dma->virt;

	ocs_memset(read_fcf, 0, sizeof(sli4_req_fcoe_read_fcf_table_t));

	read_fcf->hdr.opcode = SLI4_OPC_FCOE_READ_FCF_TABLE;
	read_fcf->hdr.subsystem = SLI4_SUBSYSTEM_FCFCOE;
	read_fcf->hdr.request_length = dma->size -
		sizeof(sli4_req_fcoe_read_fcf_table_t);
	read_fcf->fcf_index = index;

	return sli_cmd_sli_config(sli4, buf, size, 0, dma);
}

/**
 * @ingroup sli_fc
 * @brief Write an FCOE_POST_HDR_TEMPLATES command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param dma Pointer to DMA memory structure. This is allocated by the caller.
 * @param rpi Starting RPI index for the header templates.
 * @param payload_dma Pointer to DMA memory used to hold larger descriptor counts.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_fcoe_post_hdr_templates(sli4_t *sli4, void *buf, size_t size,
		ocs_dma_t *dma, uint16_t rpi, ocs_dma_t *payload_dma)
{
	sli4_req_fcoe_post_hdr_templates_t *template = NULL;
	uint32_t	sli_config_off = 0;
	uintptr_t	phys = 0;
	uint32_t	i = 0;
	uint32_t	page_count;
	uint32_t	payload_size;

	page_count = sli_page_count(dma->size, SLI_PAGE_SIZE);

	payload_size = sizeof(sli4_req_fcoe_post_hdr_templates_t) +
				page_count * sizeof(sli4_physical_page_descriptor_t);

	if (page_count > 16) {
		/* We can't fit more than 16 descriptors into an embedded mailbox
		   command, it has to be non-embedded */
		if (ocs_dma_alloc(sli4->os, payload_dma, payload_size, 4, OCS_M_FLAGS_NONE)) {
			ocs_log_err(sli4->os, "mailbox payload memory allocation fail\n");
			return 0;
		}
		sli_config_off = sli_cmd_sli_config(sli4, buf, size, payload_size, payload_dma);
		template = (sli4_req_fcoe_post_hdr_templates_t *)payload_dma->virt;
	} else {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size, payload_size, NULL);
		template = (sli4_req_fcoe_post_hdr_templates_t *)((uint8_t *)buf + sli_config_off);
	}

	if (UINT16_MAX == rpi) {
		rpi = sli4->config.extent[SLI_RSRC_FCOE_RPI].base[0];
	}

	template->hdr.opcode = SLI4_OPC_FCOE_POST_HDR_TEMPLATES;
	template->hdr.subsystem = SLI4_SUBSYSTEM_FCFCOE;
	template->hdr.request_length = sizeof(sli4_req_fcoe_post_hdr_templates_t) -
					sizeof(sli4_req_hdr_t);

	template->rpi_offset = rpi;
	template->page_count = page_count;
	phys = dma->phys;
	for (i = 0; i < template->page_count; i++) {
		template->page_descriptor[i].low  = ocs_addr32_lo(phys);
		template->page_descriptor[i].high = ocs_addr32_hi(phys);

		phys += SLI_PAGE_SIZE;
	}

	return(sli_config_off + payload_size);
}

int32_t
sli_cmd_fcoe_rediscover_fcf(sli4_t *sli4, void *buf, size_t size, uint16_t index)
{
	sli4_req_fcoe_rediscover_fcf_t *redisc = NULL;
	uint32_t	sli_config_off = 0;

	sli_config_off = sli_cmd_sli_config(sli4, buf, size,
			sizeof(sli4_req_fcoe_rediscover_fcf_t),
			NULL);

	redisc = (sli4_req_fcoe_rediscover_fcf_t *)((uint8_t *)buf + sli_config_off);

	redisc->hdr.opcode = SLI4_OPC_FCOE_REDISCOVER_FCF;
	redisc->hdr.subsystem = SLI4_SUBSYSTEM_FCFCOE;
	redisc->hdr.request_length = sizeof(sli4_req_fcoe_rediscover_fcf_t) -
					sizeof(sli4_req_hdr_t);

	if (index == UINT16_MAX) {
		redisc->fcf_count = 0;
	} else {
		redisc->fcf_count = 1;
		redisc->fcf_index[0] = index;
	}

	return(sli_config_off + sizeof(sli4_req_fcoe_rediscover_fcf_t));
}

/**
 * @ingroup sli_fc
 * @brief Write an ABORT_WQE work queue entry.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param size Buffer size, in bytes.
 * @param type Abort type, such as XRI, abort tag, and request tag.
 * @param send_abts Boolean to cause the hardware to automatically generate an ABTS.
 * @param ids ID of IOs to abort.
 * @param mask Mask applied to the ID values to abort.
 * @param tag Tag value associated with this abort.
 * @param cq_id The id of the completion queue where the WQE response is sent.
 * @param dnrx When set to 1, this field indicates that the SLI Port must not return the associated XRI to the SLI
 *             Port's optimized write XRI pool.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_abort_wqe(sli4_t *sli4, void *buf, size_t size, sli4_abort_type_e type, uint32_t send_abts,
	      uint32_t ids, uint32_t mask, uint16_t tag, uint16_t cq_id)
{
	sli4_abort_wqe_t	*abort = buf;

	ocs_memset(buf, 0, size);

	switch (type) {
	case SLI_ABORT_XRI:
		abort->criteria = SLI4_ABORT_CRITERIA_XRI_TAG;
		if (mask) {
			ocs_log_warn(sli4->os, "warning non-zero mask %#x when aborting XRI %#x\n", mask, ids);
			mask = 0;
		}
		break;
	case SLI_ABORT_ABORT_ID:
		abort->criteria = SLI4_ABORT_CRITERIA_ABORT_TAG;
		break;
	case SLI_ABORT_REQUEST_ID:
		abort->criteria = SLI4_ABORT_CRITERIA_REQUEST_TAG;
		break;
	default:
		ocs_log_test(sli4->os, "unsupported type %#x\n", type);
		return -1;
	}

	abort->ia = send_abts ? 0 : 1;

	/* Supress ABTS retries */
	abort->ir = 1;

	abort->t_mask = mask;
	abort->t_tag  = ids;
	abort->command = SLI4_WQE_ABORT;
	abort->request_tag = tag;
	abort->qosd = TRUE;
	abort->cq_id = cq_id;
	abort->cmd_type = SLI4_CMD_ABORT_WQE;

	return 0;
}

/**
 * @ingroup sli_fc
 * @brief Write an ELS_REQUEST64_WQE work queue entry.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param size Buffer size, in bytes.
 * @param sgl DMA memory for the ELS request.
 * @param req_type ELS request type.
 * @param req_len Length of ELS request in bytes.
 * @param max_rsp_len Max length of ELS response in bytes.
 * @param timeout Time, in seconds, before an IO times out. Zero means 2 * R_A_TOV.
 * @param xri XRI for this exchange.
 * @param tag IO tag value.
 * @param cq_id The id of the completion queue where the WQE response is sent.
 * @param rnode Destination of ELS request (that is, the remote node).
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_els_request64_wqe(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *sgl, uint8_t req_type,
		      uint32_t req_len, uint32_t max_rsp_len, uint8_t timeout,
		      uint16_t xri, uint16_t tag, uint16_t cq_id, ocs_remote_node_t *rnode, ocs_sport_t *sport)
{
	sli4_els_request64_wqe_t	*els = buf;
	sli4_sge_t	*sge = sgl->virt;
	uint8_t		is_fabric = FALSE;
	uint32_t rnode_indicator, sport_indicator, rnode_fc_id, sport_fc_id;

	ocs_memset(buf, 0, size);

	if (sli4->config.sgl_pre_registered) {
		els->xbl = FALSE;

		els->dbde = TRUE;
		els->els_request_payload.bde_type = SLI4_BDE_TYPE_BDE_64;

		els->els_request_payload.buffer_length = req_len;
		els->els_request_payload.u.data.buffer_address_low  = sge[0].buffer_address_low;
		els->els_request_payload.u.data.buffer_address_high = sge[0].buffer_address_high;
	} else {
		els->xbl = TRUE;

		els->els_request_payload.bde_type = SLI4_BDE_TYPE_BLP;

		els->els_request_payload.buffer_length = 2 * sizeof(sli4_sge_t);
		els->els_request_payload.u.blp.sgl_segment_address_low  = ocs_addr32_lo(sgl->phys);
		els->els_request_payload.u.blp.sgl_segment_address_high = ocs_addr32_hi(sgl->phys);
	}

	els->els_request_payload_length = req_len;
	els->max_response_payload_length = max_rsp_len;

	els->xri_tag = xri;
	els->timer = timeout;
	els->class = SLI4_ELS_REQUEST64_CLASS_3;

	els->command = SLI4_WQE_ELS_REQUEST64;

	els->request_tag = tag;

	if (rnode) {
		rnode_fc_id = rnode->fc_id; 
		rnode_indicator = rnode->indicator;
		sport_fc_id = rnode->sport->fc_id; 
		sport_indicator = rnode->sport->indicator;
		if (rnode->node_group) {
			els->hlm = TRUE;
			els->remote_id = rnode->fc_id & 0x00ffffff;
		}
	} else if (sport) {
		rnode_fc_id = FC_ADDR_FABRIC;
		rnode_indicator = sport->fabric_rpi;
		sport_fc_id = sport->indicator;
		sport_indicator = sport->indicator;
	} else {
		ocs_log_err(sli4->os, "invalid input received\n");
		return -1;
	}
	els->iod = SLI4_ELS_REQUEST64_DIR_READ;

	els->qosd = TRUE;

	// figure out the ELS_ID value from the request buffer

	switch (req_type) {
	case FC_ELS_CMD_LOGO:
		els->els_id = SLI4_ELS_REQUEST64_LOGO;
		if (rnode && rnode->attached) {
			els->ct = SLI4_ELS_REQUEST64_CONTEXT_RPI;
			els->context_tag = rnode_indicator;
		} else {
			els->ct = SLI4_ELS_REQUEST64_CONTEXT_VPI;
			els->context_tag = sport_indicator;
		}
		if (FC_ADDR_FABRIC == rnode_fc_id) {
			is_fabric = TRUE;
		}
		break;
	case FC_ELS_CMD_FDISC:
		if (FC_ADDR_FABRIC == rnode_fc_id) {
			is_fabric = TRUE;
		}
		if (0 == sport_fc_id) {
			els->els_id = SLI4_ELS_REQUEST64_FDISC;
			is_fabric = TRUE;
		} else {
			els->els_id = SLI4_ELS_REQUEST64_OTHER;
		}
		els->ct = SLI4_ELS_REQUEST64_CONTEXT_VPI;
		els->context_tag = sport_indicator;
		els->sp = TRUE;
		break;
	case FC_ELS_CMD_FLOGI:
		els->els_id = SLI4_ELS_REQUEST64_FLOGIN;
		is_fabric = TRUE;
		if (SLI4_IF_TYPE_BE3_SKH_PF == sli4->if_type) {
			if (!rnode->sport->domain) {
				ocs_log_test(sli4->os, "invalid domain handle\n");
				return -1;
			}
			/*
			 * IF_TYPE 0 skips INIT_VFI/INIT_VPI and therefore must use the
			 * FCFI here
			 */
			els->ct = SLI4_ELS_REQUEST64_CONTEXT_FCFI;
			els->context_tag = rnode->sport->domain->fcf_indicator;
			els->sp = TRUE;
		} else {
			els->ct = SLI4_ELS_REQUEST64_CONTEXT_VPI;
			els->context_tag = rnode->sport->indicator;

			/*
			 * Set SP here ... we haven't done a REG_VPI yet
			 * TODO: need to maybe not set this when we have
			 *       completed VFI/VPI registrations ...
			 *
			 * Use the FC_ID of the SPORT if it has been allocated, otherwise
			 * use an S_ID of zero.
			 */
			els->sp = TRUE;
			if (sport_fc_id != UINT32_MAX) {
				els->sid = sport_fc_id;
			}
		}
		break;
	case FC_ELS_CMD_PLOGI:
		els->els_id = SLI4_ELS_REQUEST64_PLOGI;
		els->ct = SLI4_ELS_REQUEST64_CONTEXT_VPI;
		els->context_tag = sport_indicator;
		break;
	case FC_ELS_CMD_SCR:
		els->els_id = SLI4_ELS_REQUEST64_OTHER;
		els->ct = SLI4_ELS_REQUEST64_CONTEXT_VPI;
		els->context_tag = sport_indicator;
		break;
	case FC_ELS_CMD_AUTH:
		els->els_id = SLI4_ELS_REQUEST64_OTHER;
		if (rnode->fc_id == FC_ADDR_FABRIC) {
			els->ct = SLI4_ELS_REQUEST64_CONTEXT_VPI;
			els->context_tag = sport_indicator;
			is_fabric = TRUE;
		} else {
			/* TODO RPI: not used yet */
		}
		break;
	default:
		els->els_id = SLI4_ELS_REQUEST64_OTHER;
		if (rnode->attached) {
			els->ct = SLI4_ELS_REQUEST64_CONTEXT_RPI;
			els->context_tag = rnode_indicator;
		} else {
			els->ct = SLI4_ELS_REQUEST64_CONTEXT_VPI;
			els->context_tag = sport_indicator;
		}
		break;
	}

	if (is_fabric) {
		els->cmd_type = SLI4_ELS_REQUEST64_CMD_FABRIC;
	} else {
		els->cmd_type = SLI4_ELS_REQUEST64_CMD_NON_FABRIC;
	}

	els->cq_id = cq_id;

	if (SLI4_ELS_REQUEST64_CONTEXT_RPI != els->ct) {
		els->remote_id = rnode_fc_id;
	}
	if (SLI4_ELS_REQUEST64_CONTEXT_VPI == els->ct) {
		els->temporary_rpi = rnode_indicator;
	}

	return 0;
}


/**
 * @ingroup sli_fc
 * @brief Write an FCP_ICMND64_WQE work queue entry.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param size Buffer size, in bytes.
 * @param sgl DMA memory for the scatter gather list.
 * @param xri XRI for this exchange.
 * @param tag IO tag value.
 * @param cq_id The id of the completion queue where the WQE response is sent.
 * @param rpi remote node indicator (RPI)
 * @param rnode Destination request (that is, the remote node).
 * @param timeout Time, in seconds, before an IO times out. Zero means no timeout.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_fcp_icmnd64_wqe(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *sgl,
		    uint16_t xri, uint16_t tag, uint16_t cq_id,
		    uint32_t rpi, ocs_remote_node_t *rnode, uint8_t timeout)
{
	sli4_fcp_icmnd64_wqe_t *icmnd = buf;
	sli4_sge_t	*sge = NULL;

	ocs_memset(buf, 0, size);

	if (!sgl || !sgl->virt) {
		ocs_log_err(sli4->os, "bad parameter sgl=%p virt=%p\n", sgl, sgl ? sgl->virt : NULL);
		return -1;
	}
	sge = sgl->virt;

	if (sli4->config.sgl_pre_registered) {
		icmnd->xbl = FALSE;

		icmnd->dbde = TRUE;
		icmnd->bde.bde_type = SLI4_BDE_TYPE_BDE_64;

		icmnd->bde.buffer_length = sge[0].buffer_length;
		icmnd->bde.u.data.buffer_address_low  = sge[0].buffer_address_low;
		icmnd->bde.u.data.buffer_address_high = sge[0].buffer_address_high;
	} else {
		icmnd->xbl = TRUE;

		icmnd->bde.bde_type = SLI4_BDE_TYPE_BLP;

		icmnd->bde.buffer_length = sgl->size;
		icmnd->bde.u.blp.sgl_segment_address_low  = ocs_addr32_lo(sgl->phys);
		icmnd->bde.u.blp.sgl_segment_address_high = ocs_addr32_hi(sgl->phys);
	}

	icmnd->payload_offset_length = sge[0].buffer_length + sge[1].buffer_length;
	icmnd->xri_tag = xri;
	icmnd->context_tag = rpi;
	icmnd->timer = timeout;

	icmnd->pu = 2;	// WQE word 4 contains read transfer length
	icmnd->class = SLI4_ELS_REQUEST64_CLASS_3;
	icmnd->command = SLI4_WQE_FCP_ICMND64;
	icmnd->ct = SLI4_ELS_REQUEST64_CONTEXT_RPI;

	icmnd->abort_tag = xri;

	icmnd->request_tag = tag;
	icmnd->len_loc = 3;
	if (rnode->node_group) {
		icmnd->hlm = TRUE;
		icmnd->remote_n_port_id = rnode->fc_id & 0x00ffffff;
	}
	icmnd->cmd_type = SLI4_CMD_FCP_ICMND64_WQE;
	icmnd->cq_id = cq_id;

	return  0;
}

/**
 * @ingroup sli_fc
 * @brief Write an FCP_IREAD64_WQE work queue entry.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param size Buffer size, in bytes.
 * @param sgl DMA memory for the scatter gather list.
 * @param first_data_sge Index of first data sge (used if perf hints are enabled)
 * @param xfer_len Data transfer length.
 * @param xri XRI for this exchange.
 * @param tag IO tag value.
 * @param cq_id The id of the completion queue where the WQE response is sent.
 * @param rpi remote node indicator (RPI)
 * @param rnode Destination request (i.e. remote node).
 * @param dif T10 DIF operation, or 0 to disable.
 * @param bs T10 DIF block size, or 0 if DIF is disabled.
 * @param timeout Time, in seconds, before an IO times out. Zero means no timeout.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_fcp_iread64_wqe(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *sgl, uint32_t first_data_sge,
		    uint32_t xfer_len, uint16_t xri, uint16_t tag, uint16_t cq_id,
		    uint32_t rpi, ocs_remote_node_t *rnode,
		    uint8_t dif, uint8_t bs, uint8_t timeout)
{
	sli4_fcp_iread64_wqe_t *iread = buf;
	sli4_sge_t	*sge = NULL;

	ocs_memset(buf, 0, size);

	if (!sgl || !sgl->virt) {
		ocs_log_err(sli4->os, "bad parameter sgl=%p virt=%p\n", sgl, sgl ? sgl->virt : NULL);
		return -1;
	}
	sge = sgl->virt;

	if (sli4->config.sgl_pre_registered) {
		iread->xbl = FALSE;

		iread->dbde = TRUE;
		iread->bde.bde_type = SLI4_BDE_TYPE_BDE_64;

		iread->bde.buffer_length = sge[0].buffer_length;
		iread->bde.u.data.buffer_address_low  = sge[0].buffer_address_low;
		iread->bde.u.data.buffer_address_high = sge[0].buffer_address_high;
	} else {
		iread->xbl = TRUE;

		iread->bde.bde_type = SLI4_BDE_TYPE_BLP;

		iread->bde.buffer_length = sgl->size;
		iread->bde.u.blp.sgl_segment_address_low  = ocs_addr32_lo(sgl->phys);
		iread->bde.u.blp.sgl_segment_address_high = ocs_addr32_hi(sgl->phys);

		/* fill out fcp_cmnd buffer len and change resp buffer to be of type
		 * "skip" (note: response will still be written to sge[1] if necessary) */
		iread->fcp_cmd_buffer_length = sge[0].buffer_length;
		sge[1].sge_type = SLI4_SGE_TYPE_SKIP;
	}

	iread->payload_offset_length = sge[0].buffer_length + sge[1].buffer_length;
	iread->total_transfer_length = xfer_len;

	iread->xri_tag = xri;
	iread->context_tag = rpi;

	iread->timer = timeout;

	iread->pu = 2;	// WQE word 4 contains read transfer length
	iread->class = SLI4_ELS_REQUEST64_CLASS_3;
	iread->command = SLI4_WQE_FCP_IREAD64;
	iread->ct = SLI4_ELS_REQUEST64_CONTEXT_RPI;
	iread->dif = dif;
	iread->bs  = bs;

	iread->abort_tag = xri;

	iread->request_tag = tag;
	iread->len_loc = 3;
	if (rnode->node_group) {
		iread->hlm = TRUE;
		iread->remote_n_port_id = rnode->fc_id & 0x00ffffff;
	}
	iread->iod = 1;
	iread->cmd_type = SLI4_CMD_FCP_IREAD64_WQE;
	iread->cq_id = cq_id;

	if (sli4->config.perf_hint) {
		iread->first_data_bde.bde_type = SLI4_BDE_TYPE_BDE_64;
		iread->first_data_bde.buffer_length = sge[first_data_sge].buffer_length;
		iread->first_data_bde.u.data.buffer_address_low  = sge[first_data_sge].buffer_address_low;
		iread->first_data_bde.u.data.buffer_address_high = sge[first_data_sge].buffer_address_high;
	}

	return  0;
}


/**
 * @ingroup sli_fc
 * @brief Write an FCP_IWRITE64_WQE work queue entry.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param size Buffer size, in bytes.
 * @param sgl DMA memory for the scatter gather list.
 * @param first_data_sge Index of first data sge (used if perf hints are enabled)
 * @param xfer_len Data transfer length.
 * @param first_burst The number of first burst bytes
 * @param xri XRI for this exchange.
 * @param tag IO tag value.
 * @param cq_id The id of the completion queue where the WQE response is sent.
 * @param rpi remote node indicator (RPI)
 * @param rnode Destination request (i.e. remote node)
 * @param dif T10 DIF operation, or 0 to disable
 * @param bs T10 DIF block size, or 0 if DIF is disabled
 * @param timeout Time, in seconds, before an IO times out. Zero means no timeout.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_fcp_iwrite64_wqe(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *sgl, uint32_t first_data_sge,
		     uint32_t xfer_len, uint32_t first_burst, uint16_t xri, uint16_t tag, uint16_t cq_id,
		     uint32_t rpi, ocs_remote_node_t *rnode,
		     uint8_t dif, uint8_t bs, uint8_t timeout)
{
	sli4_fcp_iwrite64_wqe_t *iwrite = buf;
	sli4_sge_t	*sge = NULL;

	ocs_memset(buf, 0, size);

	if (!sgl || !sgl->virt) {
		ocs_log_err(sli4->os, "bad parameter sgl=%p virt=%p\n", sgl, sgl ? sgl->virt : NULL);
		return -1;
	}
	sge = sgl->virt;

	if (sli4->config.sgl_pre_registered) {
		iwrite->xbl = FALSE;

		iwrite->dbde = TRUE;
		iwrite->bde.bde_type = SLI4_BDE_TYPE_BDE_64;

		iwrite->bde.buffer_length = sge[0].buffer_length;
		iwrite->bde.u.data.buffer_address_low  = sge[0].buffer_address_low;
		iwrite->bde.u.data.buffer_address_high = sge[0].buffer_address_high;
	} else {
		iwrite->xbl = TRUE;

		iwrite->bde.bde_type = SLI4_BDE_TYPE_BLP;

		iwrite->bde.buffer_length = sgl->size;
		iwrite->bde.u.blp.sgl_segment_address_low  = ocs_addr32_lo(sgl->phys);
		iwrite->bde.u.blp.sgl_segment_address_high = ocs_addr32_hi(sgl->phys);

		/* fill out fcp_cmnd buffer len and change resp buffer to be of type
		 * "skip" (note: response will still be written to sge[1] if necessary) */
		iwrite->fcp_cmd_buffer_length = sge[0].buffer_length;
		sge[1].sge_type = SLI4_SGE_TYPE_SKIP;
	}

	iwrite->payload_offset_length = sge[0].buffer_length + sge[1].buffer_length;
	iwrite->total_transfer_length = xfer_len;
	iwrite->initial_transfer_length = MIN(xfer_len, first_burst);

	iwrite->xri_tag = xri;
	iwrite->context_tag = rpi;

	iwrite->timer = timeout;

	iwrite->pu = 2;	// WQE word 4 contains read transfer length
	iwrite->class = SLI4_ELS_REQUEST64_CLASS_3;
	iwrite->command = SLI4_WQE_FCP_IWRITE64;
	iwrite->ct = SLI4_ELS_REQUEST64_CONTEXT_RPI;
	iwrite->dif = dif;
	iwrite->bs  = bs;

	iwrite->abort_tag = xri;

	iwrite->request_tag = tag;
	iwrite->len_loc = 3;
	if (rnode->node_group) {
		iwrite->hlm = TRUE;
		iwrite->remote_n_port_id = rnode->fc_id & 0x00ffffff;
	}
	iwrite->cmd_type = SLI4_CMD_FCP_IWRITE64_WQE;
	iwrite->cq_id = cq_id;

	if (sli4->config.perf_hint) {
		iwrite->first_data_bde.bde_type = SLI4_BDE_TYPE_BDE_64;
		iwrite->first_data_bde.buffer_length = sge[first_data_sge].buffer_length;
		iwrite->first_data_bde.u.data.buffer_address_low  = sge[first_data_sge].buffer_address_low;
		iwrite->first_data_bde.u.data.buffer_address_high = sge[first_data_sge].buffer_address_high;
	}

	return  0;
}

/**
 * @ingroup sli_fc
 * @brief Write an FCP_TRECEIVE64_WQE work queue entry.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param size Buffer size, in bytes.
 * @param sgl DMA memory for the Scatter-Gather List.
 * @param first_data_sge Index of first data sge (used if perf hints are enabled)
 * @param relative_off Relative offset of the IO (if any).
 * @param xfer_len Data transfer length.
 * @param xri XRI for this exchange.
 * @param tag IO tag value.
 * @param xid OX_ID for the exchange.
 * @param cq_id The id of the completion queue where the WQE response is sent.
 * @param rpi remote node indicator (RPI)
 * @param rnode Destination request (i.e. remote node).
 * @param flags Optional attributes, including:
 *  - ACTIVE - IO is already active.
 *  - AUTO RSP - Automatically generate a good FCP_RSP.
 * @param dif T10 DIF operation, or 0 to disable.
 * @param bs T10 DIF block size, or 0 if DIF is disabled.
 * @param csctl value of csctl field.
 * @param app_id value for VM application header.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_fcp_treceive64_wqe(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *sgl, uint32_t first_data_sge,
		       uint32_t relative_off, uint32_t xfer_len, uint16_t xri, uint16_t tag, uint16_t cq_id,
		       uint16_t xid, uint32_t rpi, ocs_remote_node_t *rnode, uint32_t flags, uint8_t dif, uint8_t bs,
		       uint8_t csctl, uint32_t app_id, uint8_t timer)
{
	sli4_fcp_treceive64_wqe_t *trecv = buf;
	sli4_fcp_128byte_wqe_t *trecv_128 = buf;
	sli4_sge_t	*sge = NULL;
	bool iftype_g7 = (SLI4_IF_TYPE_LANCER_G7 == sli4->if_type) ? true : false;
	bool disable_pbde = true; /* BZ 217434: Disable pbde use till we have a proper fix to work around prism asic issue */

	ocs_memset(buf, 0, size);

	if (!sgl || !sgl->virt) {
		ocs_log_err(sli4->os, "bad parameter sgl=%p virt=%p\n", sgl, sgl ? sgl->virt : NULL);
		return -1;
	}
	sge = sgl->virt;

	if (sli4->config.sgl_pre_registered) {
		trecv->xbl = FALSE;

		if (!dif && !disable_pbde && iftype_g7 && (xfer_len <= sge[2].buffer_length) && (!relative_off)) {
			/* For single bde, use the pbde optimization to get the latency benefit with G7 */
			trecv->pbde = true;
			trecv->first_data_bde.bde_type = SLI4_BDE_TYPE_BDE_64;
			trecv->first_data_bde.buffer_length = sge[2].buffer_length;
			trecv->first_data_bde.u.data.buffer_address_low = sge[2].buffer_address_low;
			trecv->first_data_bde.u.data.buffer_address_high = sge[2].buffer_address_high;
		} else {
			trecv->dbde = TRUE;
			trecv->bde.bde_type = SLI4_BDE_TYPE_BDE_64;

			trecv->bde.buffer_length = sge[0].buffer_length;
			trecv->bde.u.data.buffer_address_low  = sge[0].buffer_address_low;
			trecv->bde.u.data.buffer_address_high = sge[0].buffer_address_high;

			trecv->payload_offset_length = sge[0].buffer_length;
		}
	} else {
		trecv->xbl = TRUE;

		// if data is a single physical address, use a BDE
		if (!dif && (xfer_len <= sge[2].buffer_length)) {
			trecv->dbde = TRUE;
			trecv->bde.bde_type = SLI4_BDE_TYPE_BDE_64;

			trecv->bde.buffer_length = sge[2].buffer_length;
			trecv->bde.u.data.buffer_address_low  = sge[2].buffer_address_low;
			trecv->bde.u.data.buffer_address_high = sge[2].buffer_address_high;
		} else {
			trecv->bde.bde_type = SLI4_BDE_TYPE_BLP;
			trecv->bde.buffer_length = sgl->size;
			trecv->bde.u.blp.sgl_segment_address_low  = ocs_addr32_lo(sgl->phys);
			trecv->bde.u.blp.sgl_segment_address_high = ocs_addr32_hi(sgl->phys);
		}
	}

	trecv->relative_offset = relative_off;

	if (flags & SLI4_IO_CONTINUATION) {
		trecv->xc = TRUE;
	}
	trecv->xri_tag = xri;

	trecv->context_tag = rpi;

	trecv->pu = TRUE;	// WQE uses relative offset

	if (flags & SLI4_IO_AUTO_GOOD_RESPONSE) {
		trecv->ar = TRUE;
	}

	trecv->command = SLI4_WQE_FCP_TRECEIVE64;
	trecv->class = SLI4_ELS_REQUEST64_CLASS_3;
	trecv->ct = SLI4_ELS_REQUEST64_CONTEXT_RPI;
	trecv->dif = dif;
	trecv->bs = bs;
	trecv->timer = timer;

	trecv->remote_xid = xid;

	trecv->request_tag = tag;

	trecv->iod = 1;

	trecv->len_loc = 0x2;

	if (rnode->node_group) {
		trecv->hlm = TRUE;
		trecv->dword5.dword = rnode->fc_id & 0x00ffffff;
	}

	trecv->cmd_type = SLI4_CMD_FCP_TRECEIVE64_WQE;

	trecv->cq_id = cq_id;

	trecv->fcp_data_receive_length = xfer_len;

	if (sli4->config.perf_hint) {
		trecv->first_data_bde.bde_type = SLI4_BDE_TYPE_BDE_64;
		trecv->first_data_bde.buffer_length = sge[first_data_sge].buffer_length;
		trecv->first_data_bde.u.data.buffer_address_low  = sge[first_data_sge].buffer_address_low;
		trecv->first_data_bde.u.data.buffer_address_high = sge[first_data_sge].buffer_address_high;
	}

	/* The upper 7 bits of csctl is the priority */
	if (csctl & SLI4_MASK_CCP) {
		trecv->ccpe = 1;
		trecv->ccp = (csctl & SLI4_MASK_CCP);
	}

	if (app_id && (sli4->config.wqe_size == SLI4_WQE_EXT_BYTES) && !trecv->eat) {
		trecv->app_id_valid = 1;
		trecv->wqes = 1;
		trecv_128->dw[31] = app_id;
	}

	return 0;
}

/**
 * @ingroup sli_fc
 * @brief Write an FCP_CONT_TRECEIVE64_WQE work queue entry.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param size Buffer size, in bytes.
 * @param sgl DMA memory for the Scatter-Gather List.
 * @param first_data_sge Index of first data sge (used if perf hints are enabled)
 * @param relative_off Relative offset of the IO (if any).
 * @param xfer_len Data transfer length.
 * @param xri XRI for this exchange.
 * @param sec_xri Secondary XRI for this exchange. (BZ 161832 workaround)
 * @param tag IO tag value.
 * @param xid OX_ID for the exchange.
 * @param cq_id The id of the completion queue where the WQE response is sent.
 * @param rpi remote node indicator (RPI)
 * @param rnode Destination request (i.e. remote node).
 * @param flags Optional attributes, including:
 *  - ACTIVE - IO is already active.
 *  - AUTO RSP - Automatically generate a good FCP_RSP.
 * @param dif T10 DIF operation, or 0 to disable.
 * @param bs T10 DIF block size, or 0 if DIF is disabled.
 * @param csctl value of csctl field.
 * @param app_id value for VM application header.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_fcp_cont_treceive64_wqe(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *sgl, uint32_t first_data_sge,
			uint32_t relative_off, uint32_t xfer_len, uint16_t xri, uint16_t sec_xri, uint16_t tag,
			uint16_t cq_id, uint16_t xid, uint32_t rpi, ocs_remote_node_t *rnode, uint32_t flags,
			uint8_t dif, uint8_t bs, uint8_t csctl, uint32_t app_id, uint8_t timer)
{
	int32_t rc;

	rc = sli_fcp_treceive64_wqe(sli4, buf, size, sgl, first_data_sge, relative_off, xfer_len, xri, tag,
				    cq_id, xid, rpi, rnode, flags, dif, bs, csctl, app_id, timer);
	if (rc == 0) {
		sli4_fcp_treceive64_wqe_t *trecv = buf;

		trecv->command = SLI4_WQE_FCP_CONT_TRECEIVE64;
		trecv->dword5.sec_xri_tag = sec_xri;
	}

	return rc;
}

/**
 * @ingroup sli_fc
 * @brief Write an FCP_TRSP64_WQE work queue entry.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param size Buffer size, in bytes.
 * @param sgl DMA memory for the Scatter-Gather List.
 * @param rsp_len Response data length.
 * @param xri XRI for this exchange.
 * @param tag IO tag value.
 * @param cq_id The id of the completion queue where the WQE response is sent.
 * @param xid OX_ID for the exchange.
 * @param rpi remote node indicator (RPI)
 * @param rnode Destination request (i.e. remote node).
 * @param flags Optional attributes, including:
 *  - ACTIVE - IO is already active
 *  - AUTO RSP - Automatically generate a good FCP_RSP.
 * @param csctl value of csctl field.
 * @param port_owned 0/1 to indicate if the XRI is port owned (used to set XBL=0)
 * @param app_id value for VM application header.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_fcp_trsp64_wqe(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *sgl, uint32_t rsp_len,
		   uint16_t xri, uint16_t tag, uint16_t cq_id, uint16_t xid, uint32_t rpi, ocs_remote_node_t *rnode,
		   uint32_t flags, uint8_t csctl, uint8_t port_owned, uint32_t app_id)
{
	sli4_fcp_trsp64_wqe_t *trsp = buf;
	sli4_fcp_128byte_wqe_t *trsp_128 = buf;

	ocs_memset(buf, 0, size);

	if (flags & SLI4_IO_AUTO_GOOD_RESPONSE) {
		trsp->ag = TRUE;
		/*
		 * The SLI-4 documentation states that the BDE is ignored when
		 * using auto-good response, but, at least for IF_TYPE 0 devices,
		 * this does not appear to be true.
		 */
		if (SLI4_IF_TYPE_BE3_SKH_PF == sli4->if_type) {
			trsp->bde.buffer_length = 12;	// byte size of RSP
		}
	} else {
		sli4_sge_t	*sge = sgl->virt;

		if (sli4->config.sgl_pre_registered || port_owned) {
			trsp->dbde = TRUE;
		} else {
			trsp->xbl = TRUE;
		}

		trsp->bde.bde_type = SLI4_BDE_TYPE_BDE_64;
		trsp->bde.buffer_length = sge[0].buffer_length;
		trsp->bde.u.data.buffer_address_low  = sge[0].buffer_address_low;
		trsp->bde.u.data.buffer_address_high = sge[0].buffer_address_high;

		trsp->fcp_response_length = rsp_len;
	}

	if (flags & SLI4_IO_CONTINUATION) {
		trsp->xc = TRUE;
	}

	if (rnode->node_group) {
		trsp->hlm = TRUE;
		trsp->dword5 = rnode->fc_id & 0x00ffffff;
	}

	trsp->xri_tag = xri;
	trsp->rpi = rpi;

	trsp->command = SLI4_WQE_FCP_TRSP64;
	trsp->class = SLI4_ELS_REQUEST64_CLASS_3;

	trsp->remote_xid = xid;
	trsp->request_tag = tag;
	trsp->dnrx = ((flags & SLI4_IO_DNRX) == 0 ? 0 : 1);
	trsp->len_loc = 0x1;
	trsp->cq_id = cq_id;
	trsp->cmd_type = SLI4_CMD_FCP_TRSP64_WQE;

	/* The upper 7 bits of csctl is the priority */
	if (csctl & SLI4_MASK_CCP) {
		trsp->ccpe = 1;
		trsp->ccp = (csctl & SLI4_MASK_CCP);
	}

	if (app_id && (sli4->config.wqe_size == SLI4_WQE_EXT_BYTES) && !trsp->eat) {
		trsp->app_id_valid = 1;
		trsp->wqes = 1;
		trsp_128->dw[31] = app_id;
	}

	return 0;
}

/**
 * @ingroup sli_fc
 * @brief Write an FCP_TSEND64_WQE work queue entry.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param size Buffer size, in bytes.
 * @param sgl DMA memory for the scatter gather list.
 * @param first_data_sge Index of first data sge (used if perf hints are enabled)
 * @param relative_off Relative offset of the IO (if any).
 * @param xfer_len Data transfer length.
 * @param xri XRI for this exchange.
 * @param tag IO tag value.
 * @param cq_id The id of the completion queue where the WQE response is sent.
 * @param xid OX_ID for the exchange.
 * @param rpi remote node indicator (RPI)
 * @param rnode Destination request (i.e. remote node).
 * @param flags Optional attributes, including:
 *  - ACTIVE - IO is already active.
 *  - AUTO RSP - Automatically generate a good FCP_RSP.
 * @param dif T10 DIF operation, or 0 to disable.
 * @param bs T10 DIF block size, or 0 if DIF is disabled.
 * @param csctl value of csctl field.
 * @param app_id value for VM application header.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_fcp_tsend64_wqe(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *sgl, uint32_t first_data_sge,
		    uint32_t relative_off, uint32_t xfer_len,
		    uint16_t xri, uint16_t tag, uint16_t cq_id, uint16_t xid, uint32_t rpi, ocs_remote_node_t *rnode,
		    uint32_t flags, uint8_t dif, uint8_t bs, uint8_t csctl, uint32_t app_id)
{
	sli4_fcp_tsend64_wqe_t *tsend = buf;
	sli4_fcp_128byte_wqe_t *tsend_128 = buf;
	sli4_sge_t	*sge = NULL;

	ocs_memset(buf, 0, size);

	if (!sgl || !sgl->virt) {
		ocs_log_err(sli4->os, "bad parameter sgl=%p virt=%p\n", sgl, sgl ? sgl->virt : NULL);
		return -1;
	}
	sge = sgl->virt;

	if (sli4->config.sgl_pre_registered) {
		tsend->xbl = FALSE;

		tsend->dbde = TRUE;
		tsend->bde.bde_type = SLI4_BDE_TYPE_BDE_64;

		// TSEND64_WQE specifies first two SGE are skipped (i.e. 3rd is valid)
		tsend->bde.buffer_length = sge[2].buffer_length;
		tsend->bde.u.data.buffer_address_low  = sge[2].buffer_address_low;
		tsend->bde.u.data.buffer_address_high = sge[2].buffer_address_high;
	} else {
		tsend->xbl = TRUE;

		// if data is a single physical address, use a BDE
		if (!dif && (xfer_len <= sge[2].buffer_length)) {
			tsend->dbde = TRUE;
			tsend->bde.bde_type = SLI4_BDE_TYPE_BDE_64;
			// TSEND64_WQE specifies first two SGE are skipped (i.e. 3rd is valid)
			tsend->bde.buffer_length = sge[2].buffer_length;
			tsend->bde.u.data.buffer_address_low  = sge[2].buffer_address_low;
			tsend->bde.u.data.buffer_address_high = sge[2].buffer_address_high;
		} else {
			tsend->bde.bde_type = SLI4_BDE_TYPE_BLP;
			tsend->bde.buffer_length = sgl->size;
			tsend->bde.u.blp.sgl_segment_address_low  = ocs_addr32_lo(sgl->phys);
			tsend->bde.u.blp.sgl_segment_address_high = ocs_addr32_hi(sgl->phys);
		}
	}

	tsend->relative_offset = relative_off;

	if (flags & SLI4_IO_CONTINUATION) {
		tsend->xc = TRUE;
	}
	tsend->xri_tag = xri;

	tsend->rpi = rpi;

	tsend->pu = TRUE;	// WQE uses relative offset

	if (flags & SLI4_IO_AUTO_GOOD_RESPONSE) {
		tsend->ar = TRUE;
		if (flags & SLI4_IO_SUPPRESS_RESPONSE)
			tsend->suppress_rsp = TRUE;
	}

	tsend->command = SLI4_WQE_FCP_TSEND64;
	tsend->class = SLI4_ELS_REQUEST64_CLASS_3;
	tsend->ct = SLI4_ELS_REQUEST64_CONTEXT_RPI;
	tsend->dif = dif;
	tsend->bs  = bs;

	tsend->remote_xid = xid;

	tsend->request_tag = tag;

	tsend->len_loc = 0x2;

	if (rnode->node_group) {
		tsend->hlm = TRUE;
		tsend->dword5 = rnode->fc_id & 0x00ffffff;
	}

	tsend->cq_id = cq_id;

	tsend->cmd_type = SLI4_CMD_FCP_TSEND64_WQE;

	tsend->fcp_data_transmit_length = xfer_len;

	if (sli4->config.perf_hint) {
		tsend->first_data_bde.bde_type = SLI4_BDE_TYPE_BDE_64;
		tsend->first_data_bde.buffer_length = sge[first_data_sge].buffer_length;
		tsend->first_data_bde.u.data.buffer_address_low  = sge[first_data_sge].buffer_address_low;
		tsend->first_data_bde.u.data.buffer_address_high = sge[first_data_sge].buffer_address_high;
	}

	/* The upper 7 bits of csctl is the priority */
	if (csctl & SLI4_MASK_CCP) {
		tsend->ccpe = 1;
		tsend->ccp = (csctl & SLI4_MASK_CCP);
	}

	if (app_id && (sli4->config.wqe_size == SLI4_WQE_EXT_BYTES) && !tsend->eat) {
		tsend->app_id_valid = 1;
		tsend->wqes = 1;
		tsend_128->dw[31] = app_id;
	}

	return 0;
}

/**
 * @ingroup sli_fc
 * @brief Write a GEN_REQUEST64 work queue entry.
 *
 * @note This WQE is only used to send FC-CT commands.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param size Buffer size, in bytes.
 * @param sgl DMA memory for the request.
 * @param req_len Length of request.
 * @param max_rsp_len Max length of response.
 * @param timeout Time, in seconds, before an IO times out. Zero means infinite.
 * @param xri XRI for this exchange.
 * @param tag IO tag value.
 * @param cq_id The id of the completion queue where the WQE response is sent.
 * @param rnode Destination of request (that is, the remote node).
 * @param r_ctl R_CTL value for sequence.
 * @param type TYPE value for sequence.
 * @param df_ctl DF_CTL value for sequence.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_gen_request64_wqe(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *sgl,
		      uint32_t req_len, uint32_t max_rsp_len, uint8_t timeout,
		      uint16_t xri, uint16_t tag, uint16_t cq_id, ocs_remote_node_t *rnode,
		      uint8_t r_ctl, uint8_t type, uint8_t df_ctl)
{
	sli4_gen_request64_wqe_t	*gen = buf;
	sli4_sge_t	*sge = NULL;

	ocs_memset(buf, 0, size);

	if (!sgl || !sgl->virt) {
		ocs_log_err(sli4->os, "bad parameter sgl=%p virt=%p\n", sgl, sgl ? sgl->virt : NULL);
		return -1;
	}
	sge = sgl->virt;

	if (sli4->config.sgl_pre_registered) {
		gen->xbl = FALSE;

		gen->dbde = TRUE;
		gen->bde.bde_type = SLI4_BDE_TYPE_BDE_64;

		gen->bde.buffer_length = req_len;
		gen->bde.u.data.buffer_address_low  = sge[0].buffer_address_low;
		gen->bde.u.data.buffer_address_high = sge[0].buffer_address_high;
	} else {
		gen->xbl = TRUE;

		gen->bde.bde_type = SLI4_BDE_TYPE_BLP;

		gen->bde.buffer_length = 2 * sizeof(sli4_sge_t);
		gen->bde.u.blp.sgl_segment_address_low  = ocs_addr32_lo(sgl->phys);
		gen->bde.u.blp.sgl_segment_address_high = ocs_addr32_hi(sgl->phys);
	}

	gen->request_payload_length = req_len;
	gen->max_response_payload_length = max_rsp_len;

	gen->df_ctl = df_ctl;
	gen->type = type;
	gen->r_ctl = r_ctl;

	gen->xri_tag = xri;

	gen->ct = SLI4_ELS_REQUEST64_CONTEXT_RPI;
	gen->context_tag = rnode->indicator;

	gen->class = SLI4_ELS_REQUEST64_CLASS_3;

	gen->command = SLI4_WQE_GEN_REQUEST64;

	gen->timer = timeout;

	gen->request_tag = tag;

	gen->iod = SLI4_ELS_REQUEST64_DIR_READ;

	gen->qosd = TRUE;

	if (rnode->node_group) {
		gen->hlm = TRUE;
		gen->remote_n_port_id = rnode->fc_id & 0x00ffffff;
	}

	gen->cmd_type = SLI4_CMD_GEN_REQUEST64_WQE;

	gen->cq_id = cq_id;

	return 0;
}

/**
 * @ingroup sli_fc
 * @brief Write a SEND_FRAME work queue entry
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param size Buffer size, in bytes.
 * @param sof Start of frame value
 * @param eof End of frame value
 * @param hdr Pointer to FC header data
 * @param payload DMA memory for the payload.
 * @param req_len Length of payload.
 * @param timeout Time, in seconds, before an IO times out. Zero means infinite.
 * @param xri XRI for this exchange.
 * @param req_tag IO tag value.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_send_frame_wqe(sli4_t *sli4, void *buf, size_t size, uint8_t sof, uint8_t eof, uint32_t *hdr,
		   ocs_dma_t *payload, uint32_t req_len, uint8_t timeout,
		   uint16_t xri, uint16_t req_tag)
{
	sli4_send_frame_wqe_t *sf = buf;

	ocs_memset(buf, 0, size);

	if (req_len) {
		sf->dbde = TRUE;
		sf->bde.buffer_length = req_len;
		sf->bde.u.data.buffer_address_low = ocs_addr32_lo(payload->phys);
		sf->bde.u.data.buffer_address_high = ocs_addr32_hi(payload->phys);
		sf->frame_length = req_len;
	}

	/* Copy FC header */
	sf->fc_header_0_1[0] = hdr[0];
	sf->fc_header_0_1[1] = hdr[1];
	sf->fc_header_2_5[0] = hdr[2];
	sf->fc_header_2_5[1] = hdr[3];
	sf->fc_header_2_5[2] = hdr[4];
	sf->fc_header_2_5[3] = hdr[5];


	sf->xri_tag = xri;
	sf->pu = 0;
	sf->context_tag = 0;


	sf->ct = 0;
	sf->command = SLI4_WQE_SEND_FRAME;
	sf->class = 0;
	sf->timer = timeout;

	sf->request_tag = req_tag;
	sf->eof = eof;
	sf->sof = sof;

	sf->qosd = 0;
	sf->lenloc = 1;
	sf->xc = 1;
	sf->xbl = 1;

	sf->cmd_type = SLI4_CMD_SEND_FRAME_WQE;
	sf->cq_id = 0xffff;

	return 0;
}

/**
 * @ingroup sli_fc
 * @brief Write a XMIT_SEQUENCE64 work queue entry.
 *
 * This WQE is used to send FC-CT response frames.
 *
 * @note This API implements a restricted use for this WQE, a TODO: would
 * include passing in sequence initiative, and full SGL's
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param size Buffer size, in bytes.
 * @param payload DMA memory for the request.
 * @param payload_len Length of request.
 * @param timeout Time, in seconds, before an IO times out. Zero means infinite.
 * @param ox_id originator exchange ID
 * @param xri XRI for this exchange.
 * @param tag IO tag value.
 * @param rnode Destination of request (that is, the remote node).
 * @param r_ctl R_CTL value for sequence.
 * @param type TYPE value for sequence.
 * @param df_ctl DF_CTL value for sequence.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_xmit_sequence64_wqe(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *payload,
		      uint32_t payload_len, uint8_t timeout, uint16_t ox_id,
		      uint16_t xri, uint16_t tag, ocs_remote_node_t *rnode,
		      uint8_t r_ctl, uint8_t type, uint8_t df_ctl)
{
	sli4_xmit_sequence64_wqe_t	*xmit = buf;

	ocs_memset(buf, 0, size);

	if ((payload == NULL) || (payload->virt == NULL)) {
		ocs_log_err(sli4->os, "bad parameter sgl=%p virt=%p\n", payload, payload ? payload->virt : NULL);
		return -1;
	}

	if (sli4->config.sgl_pre_registered) {
		xmit->dbde = TRUE;
	} else {
		xmit->xbl = TRUE;
	}

	xmit->bde.bde_type = SLI4_BDE_TYPE_BDE_64;
	xmit->bde.buffer_length = payload_len;
	xmit->bde.u.data.buffer_address_low  = ocs_addr32_lo(payload->phys);
	xmit->bde.u.data.buffer_address_high = ocs_addr32_hi(payload->phys);
	xmit->sequence_payload_len = payload_len;

	xmit->remote_n_port_id = rnode->fc_id & 0x00ffffff;

	xmit->relative_offset = 0;

	xmit->si = 0;			/* sequence initiative - this matches what is seen from
					 * FC switches in response to FCGS commands */
	xmit->ft = 0;			/* force transmit */
	xmit->xo = 0;			/* exchange responder */
	xmit->ls = 1;			/* last in seqence */
	xmit->df_ctl = df_ctl;
	xmit->type = type;
	xmit->r_ctl = r_ctl;

	xmit->xri_tag = xri;
	xmit->context_tag = rnode->indicator;

	xmit->dif = 0;
	xmit->ct = SLI4_ELS_REQUEST64_CONTEXT_RPI;
	xmit->bs = 0;

	xmit->command = SLI4_WQE_XMIT_SEQUENCE64;
	xmit->class = SLI4_ELS_REQUEST64_CLASS_3;
	xmit->pu = 0;
	xmit->timer = timeout;

	xmit->abort_tag = 0;
	xmit->request_tag = tag;
	xmit->remote_xid = ox_id;

	xmit->iod = SLI4_ELS_REQUEST64_DIR_READ;

	if (rnode->node_group) {
		xmit->hlm = TRUE;
		xmit->remote_n_port_id = rnode->fc_id & 0x00ffffff;
	}

	xmit->cmd_type = SLI4_CMD_XMIT_SEQUENCE64_WQE;

	xmit->len_loc = 2;

	xmit->cq_id = 0xFFFF;

	return 0;
}

/**
 * @ingroup sli_fc
 * @brief Write a REQUEUE_XRI_WQE work queue entry.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param size Buffer size, in bytes.
 * @param xri XRI for this exchange.
 * @param tag IO tag value.
 * @param cq_id The id of the completion queue where the WQE response is sent.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_requeue_xri_wqe(sli4_t *sli4, void *buf, size_t size, uint16_t xri, uint16_t tag, uint16_t cq_id)
{
	sli4_requeue_xri_wqe_t	*requeue = buf;

	ocs_memset(buf, 0, size);

	requeue->command = SLI4_WQE_REQUEUE_XRI;
	requeue->xri_tag = xri;
	requeue->request_tag = tag;
	requeue->xc = 1;
	requeue->qosd = 1;
	requeue->cq_id = cq_id;
	requeue->disable_cqe = 1;	// Disable Completion CQE
	requeue->cmd_type = SLI4_CMD_REQUEUE_XRI_WQE;
	return 0;
}

int32_t
sli_xmit_bcast64_wqe(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *payload,
		uint32_t payload_len, uint8_t timeout, uint16_t xri, uint16_t tag,
		uint16_t cq_id, ocs_remote_node_t *rnode,
		uint8_t r_ctl, uint8_t type, uint8_t df_ctl)
{
	sli4_xmit_bcast64_wqe_t *bcast = buf;

	/* Command requires a temporary RPI (i.e. unused remote node) */
	if (rnode->attached) {
		ocs_log_test(sli4->os, "remote node %d in use\n", rnode->indicator);
		return -1;
	}

	ocs_memset(buf, 0, size);

	bcast->dbde = TRUE;
	bcast->sequence_payload.bde_type = SLI4_BDE_TYPE_BDE_64;
	bcast->sequence_payload.buffer_length = payload_len;
	bcast->sequence_payload.u.data.buffer_address_low  = ocs_addr32_lo(payload->phys);
	bcast->sequence_payload.u.data.buffer_address_high = ocs_addr32_hi(payload->phys);

	bcast->sequence_payload_length = payload_len;

	bcast->df_ctl = df_ctl;
	bcast->type = type;
	bcast->r_ctl = r_ctl;

	bcast->xri_tag = xri;

	bcast->ct = SLI4_ELS_REQUEST64_CONTEXT_VPI;
	bcast->context_tag = rnode->sport->indicator;

	bcast->class = SLI4_ELS_REQUEST64_CLASS_3;

	bcast->command = SLI4_WQE_XMIT_BCAST64;

	bcast->timer = timeout;

	bcast->request_tag = tag;

	bcast->temporary_rpi = rnode->indicator;

	bcast->len_loc = 0x1;

	bcast->iod = SLI4_ELS_REQUEST64_DIR_WRITE;

	bcast->cmd_type = SLI4_CMD_XMIT_BCAST64_WQE;

	bcast->cq_id = cq_id;

	return 0;
}

/**
 * @ingroup sli_fc
 * @brief Write an XMIT_BLS_RSP64_WQE work queue entry.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param size Buffer size, in bytes.
 * @param payload Contents of the BLS payload to be sent.
 * @param xri XRI for this exchange.
 * @param tag IO tag value.
 * @param cq_id The id of the completion queue where the WQE response is sent.
 * @param rnode Destination of request (that is, the remote node).
 * @param s_id Source ID to use in the response. If UINT32_MAX, use SLI Port's ID.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_xmit_bls_rsp64_wqe(sli4_t *sli4, void *buf, size_t size, sli_bls_payload_t *payload,
		       uint16_t xri, uint16_t tag, uint16_t cq_id, ocs_remote_node_t *rnode, uint32_t s_id)
{
	sli4_xmit_bls_rsp_wqe_t *bls = buf;

	/*
	 * Callers can either specify RPI or S_ID, but not both
	 */
	if (rnode->attached && (s_id != UINT32_MAX)) {
		ocs_log_test(sli4->os, "S_ID specified for attached remote node %d\n", rnode->indicator);
		return -1;
	}

	ocs_memset(buf, 0, size);

	if (SLI_BLS_ACC == payload->type) {
		bls->payload_word0 = (payload->u.acc.seq_id_last << 16) |
			(payload->u.acc.seq_id_validity << 24);
		bls->high_seq_cnt = payload->u.acc.high_seq_cnt;
		bls->low_seq_cnt = payload->u.acc.low_seq_cnt;
	} else if (SLI_BLS_RJT == payload->type) {
		bls->payload_word0 = *((uint32_t *)&payload->u.rjt);
		bls->ar = TRUE;
	} else {
		ocs_log_test(sli4->os, "bad BLS type %#x\n", payload->type);
		return -1;
	}

	bls->ox_id = payload->ox_id;
	bls->rx_id = payload->rx_id;

	if (rnode->attached) {
		bls->ct = SLI4_ELS_REQUEST64_CONTEXT_RPI;
		bls->context_tag = rnode->indicator;
	} else {
		bls->ct = SLI4_ELS_REQUEST64_CONTEXT_VPI;
		bls->context_tag = rnode->sport->indicator;

		if (UINT32_MAX != s_id) {
			bls->local_n_port_id = s_id & 0x00ffffff;
		} else {
			bls->local_n_port_id = rnode->sport->fc_id & 0x00ffffff;
		}
		bls->remote_id = rnode->fc_id & 0x00ffffff;

		bls->temporary_rpi = rnode->indicator;
	}

	bls->xri_tag = xri;

	bls->class = SLI4_ELS_REQUEST64_CLASS_3;

	bls->command = SLI4_WQE_XMIT_BLS_RSP;

	bls->request_tag = tag;

	bls->qosd = TRUE;

	if (rnode->node_group) {
		bls->hlm = TRUE;
		bls->remote_id = rnode->fc_id & 0x00ffffff;
	}

	bls->cq_id = cq_id;

	bls->cmd_type = SLI4_CMD_XMIT_BLS_RSP64_WQE;

	return 0;
}

/**
 * @ingroup sli_fc
 * @brief Write a XMIT_ELS_RSP64_WQE work queue entry.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param size Buffer size, in bytes.
 * @param rsp DMA memory for the ELS response.
 * @param rsp_len Length of ELS response, in bytes.
 * @param xri XRI for this exchange.
 * @param tag IO tag value.
 * @param cq_id The id of the completion queue where the WQE response is sent.
 * @param ox_id OX_ID of the exchange containing the request.
 * @param rnode Destination of the ELS response (that is, the remote node).
 * @param flags Optional attributes, including:
 *  - SLI4_IO_CONTINUATION - IO is already active.
 * @param s_id S_ID used for special responses.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_xmit_els_rsp64_wqe(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *rsp,
		       uint32_t rsp_len, uint16_t xri, uint16_t tag, uint16_t cq_id,
		       uint16_t ox_id, ocs_remote_node_t *rnode, uint32_t flags, uint32_t s_id)
{
	sli4_xmit_els_rsp64_wqe_t	*els = buf;

	ocs_memset(buf, 0, size);

	if (sli4->config.sgl_pre_registered) {
		els->dbde = TRUE;
	} else {
		els->xbl = TRUE;
	}

	els->els_response_payload.bde_type = SLI4_BDE_TYPE_BDE_64;
	els->els_response_payload.buffer_length = rsp_len;
	els->els_response_payload.u.data.buffer_address_low  = ocs_addr32_lo(rsp->phys);
	els->els_response_payload.u.data.buffer_address_high = ocs_addr32_hi(rsp->phys);

	els->els_response_payload_length = rsp_len;

	els->xri_tag = xri;

	els->class = SLI4_ELS_REQUEST64_CLASS_3;

	els->command = SLI4_WQE_ELS_RSP64;

	els->request_tag = tag;

	els->ox_id = ox_id;

	els->iod = SLI4_ELS_REQUEST64_DIR_WRITE;

	els->qosd = TRUE;

	if (flags & SLI4_IO_CONTINUATION) {
		els->xc = TRUE;
	}

	if (rnode->attached) {
		els->ct = SLI4_ELS_REQUEST64_CONTEXT_RPI;
		els->context_tag = rnode->indicator;
	} else {
		els->ct = SLI4_ELS_REQUEST64_CONTEXT_VPI;
		els->context_tag = rnode->sport->indicator;
		els->remote_id = rnode->fc_id & 0x00ffffff;
		els->temporary_rpi = rnode->indicator;
		if (UINT32_MAX != s_id) {
			els->sp = TRUE;
			els->s_id = s_id & 0x00ffffff;
		}
	}

	if (rnode->node_group) {
		els->hlm = TRUE;
		els->remote_id = rnode->fc_id & 0x00ffffff;
	}

	els->cmd_type = SLI4_ELS_REQUEST64_CMD_GEN;

	els->cq_id = cq_id;

	return 0;
}


/*
 * @ingroup sli_fc
 * @brief Write a XMIT_ELS_RSP64_WQE work queue entry.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the WQE.
 * @param ctx_tag context tag in marker request which will be filled in cqe
 * @param req_tag to associate compl with wqe
 * @param category Marker request category
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
void
sli4_fcoe_marker_request_wqe(sli4_t *sli4, void *buf, uint32_t ctx_tag,
			     uint16_t req_tag, uint8_t category)
{
	sli4_fcoe_marker_request_wqe_t	*req = buf;

	req->marker_category = category;
	req->command = SLI4_WQE_RQ_MARKER_REQUEST;
	req->cmd_type = SLI4_CMD_RQ_MARKER_REQ_WQE;
	req->qosd = 1;
	req->cq_id = 0xffff;
	req->req_tag = req_tag;

	req->tag_higher = SLI4_RQ_MARKER_TYPE_SCSI;
	req->tag_lower = ctx_tag;

	return;
}

/**
 * @ingroup sli_fc
 * @brief Process an asynchronous Link State event entry.
 *
 * @par Description
 * Parses Asynchronous Completion Queue Entry (ACQE),
 * creates an abstracted event, and calls registered callback functions.
 *
 * @param sli4 SLI context.
 * @param acqe Pointer to the ACQE.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_fc_process_link_state(sli4_t *sli4, void *acqe)
{
	sli4_link_state_t	*link_state = acqe;
	sli4_link_event_t	event = { 0 };
	int32_t			rc = 0;

	if (!sli4->link) {
		// bail if there is no callback
		return 0;
	}

	if (SLI4_LINK_TYPE_ETHERNET == link_state->link_type) {
		event.topology = SLI_LINK_TOPO_NPORT;
		event.medium   = SLI_LINK_MEDIUM_ETHERNET;
	} else {
		// TODO is this supported for anything other than FCoE?
		ocs_log_test(sli4->os, "unsupported link type %#x\n", link_state->link_type);
		event.topology = SLI_LINK_TOPO_MAX;
		event.medium   = SLI_LINK_MEDIUM_MAX;
		rc = -1;
	}

	switch (link_state->port_link_status) {
	case SLI4_PORT_LINK_STATUS_PHYSICAL_DOWN:
	case SLI4_PORT_LINK_STATUS_LOGICAL_DOWN:
		event.status = SLI_LINK_STATUS_DOWN;
		break;
	case SLI4_PORT_LINK_STATUS_PHYSICAL_UP:
	case SLI4_PORT_LINK_STATUS_LOGICAL_UP:
		event.status = SLI_LINK_STATUS_UP;
		break;
	default:
		ocs_log_test(sli4->os, "unsupported link status %#x\n", link_state->port_link_status);
		event.status = SLI_LINK_STATUS_MAX;
		rc = -1;
	}

	switch (link_state->port_speed) {
	case 0:
		event.speed = 0;
		break;
	case 1:
		event.speed = 10;
		break;
	case 2:
		event.speed = 100;
		break;
	case 3:
		event.speed = 1000;
		break;
	case 4:
		event.speed = 10000;
		break;
	case 5:
		event.speed = 20000;
		break;
	case 6:
		event.speed = 25000;
		break;
	case 7:
		event.speed = 40000;
		break;
	case 8:
		event.speed = 100000;
		break;
	default:
		ocs_log_test(sli4->os, "unsupported port_speed %#x\n",
				link_state->port_speed);
		rc = -1;
	}

	sli4->link(sli4->link_arg, (void *)&event);

	return rc;
}

typedef struct sli4_tcm_parity_event_s {
	uint32_t	tcm_parity_event;
	uint32_t	tcm_address:24,
			ulp_id:8;
	uint32_t	pbec:16,
			tpec:16;
	uint32_t	:8,
			event_code:8,
			event_type:8,
			:6,
			async_event:1,
			valid:1;
} sli4_tcm_parity_event_t;

static void
sli_fc_log_link_state(sli4_t *sli4, uint8_t state)
{
	switch (state) {
		case 0:
			ocs_log_info(sli4->os, "Link is valid\n");
			break;
		case 1:
			ocs_log_info(sli4->os, "Link not present\n");
			break;
		case 2:
			ocs_log_info(sli4->os, "Link is present but incompatible physical media type\n");
			break;
		case 3:
			ocs_log_info(sli4->os, "Present, Unsupported PHY\n");
			break;
		default:
			ocs_log_err(sli4->os, "Unknown link state %d\n", state);
			break;
	}
}

static void
sli_fc_report_link_state(sli4_t *sli4, sli4_port_event_acqe_t *pe_acqe)
{
	switch (sli4->config.port_number) {
		case 0:
			sli_fc_log_link_state(sli4, pe_acqe->p0_linkstate);
			break;
		case 1:
			sli_fc_log_link_state(sli4, pe_acqe->p1_linkstate);
			break;
		case 2:
			sli_fc_log_link_state(sli4, pe_acqe->p2_linkstate);
			break;
		case 3:
			sli_fc_log_link_state(sli4, pe_acqe->p3_linkstate);
			break;
		default:
			ocs_log_err(sli4->os, "Invalid port number %d\n", sli4->config.port_number);
			break;
	}
}

/**
 * @ingroup sli_fc
 * @brief Process an asynchronous sli port event.
 *
 * @par Description
 * Parses Asynchronous Completion Queue Entry (ACQE)
 * and logs driver debug messages for correct SFP link status
 *
 * @param sli4 SLI context.
 * @param acqe Pointer to the ACQE.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_fc_process_sli_port_event(sli4_t *sli4, void *acqe)
{
	sli4_port_event_acqe_t	*pe_acqe = acqe;
	sli4_link_event_t	event = { 0 };

	if (!sli4->link)
		return 0;

	switch (pe_acqe->event_type) {
		case SLI4_ACQE_PORT_EVENT_TYPE_ITCM_PARITY_EVENT: {
			sli4_tcm_parity_event_t *evt = acqe;

			ocs_log_warn(sli4->os, "ITCM parity event detected; ULP id = %d, "\
				"parity_bit_error_count = %d, total_parity_error_count = %d\n",
				evt->ulp_id, evt->pbec, evt->tpec);
			break;
		}
		case SLI4_ACQE_PORT_EVENT_TYPE_MISCONF_PHYPORTS:
			sli_fc_report_link_state(sli4, pe_acqe);
			break;
		case SLI4_ACQE_PORT_EVENT_TYPE_PORT_INOPERABLE: {
			ocs_log_err(sli4->os, "Port inoperable event detected.\n");
			event.status = SLI_LINK_STATUS_INOPERABLE;
			break;
		}
		default:
			ocs_log_err(sli4->os, "Unhandled async event type %d\n", pe_acqe->event_type);
			break;
	}

	if (event.status != SLI_LINK_STATUS_INOPERABLE)
		event.status = SLI_LINK_STATUS_CHANGED;

	sli4->link(sli4->link_arg, (void *)&event);
	return 0;
}

static uint32_t
sli_fc_decode_port_speed(uint32_t speed_code)
{
	uint32_t speed_gbps = 0;

	/* Speed in Gbps */
	switch (speed_code) {
	case SLI4_LINK_ATTN_64G:
		speed_gbps = 0x40;
		break;
	case SLI4_LINK_ATTN_128G:
		speed_gbps = 0x80;
		break;
	case SLI4_LINK_ATTN_256G:
		speed_gbps = 0x100;
		break;
	default:
		speed_gbps = speed_code;
	}

	/* Return Mbps value */
	return speed_gbps * 1000;
}

static int32_t
sli_fc_process_fc_link_attention(sli4_t *sli4, sli4_link_attention_t *link_attn, sli4_link_event_t *event)
{
	int32_t rc = 0;

	switch (link_attn->attn_type) {
	case SLI4_LINK_ATTN_TYPE_LINK_UP:
		event->status = SLI_LINK_STATUS_UP;
		break;
	case SLI4_LINK_ATTN_TYPE_LINK_DOWN:
		event->status = SLI_LINK_STATUS_DOWN;
		break;
	case SLI4_LINK_ATTN_TYPE_NO_HARD_ALPA:
		ocs_log_debug(sli4->os, "attn_type: no hard alpa\n");
		event->status = SLI_LINK_STATUS_NO_ALPA;
		break;
	default:
		ocs_log_test(sli4->os, "attn_type: unknown\n");
		return -1;
	}

	switch (link_attn->topology) {
	case SLI4_LINK_ATTN_P2P:
		event->topology = SLI_LINK_TOPO_NPORT;
		break;
	case SLI4_LINK_ATTN_FC_AL:
		event->topology = SLI_LINK_TOPO_LOOP;
		break;
	case SLI4_LINK_ATTN_INTERNAL_LOOPBACK:
		ocs_log_debug(sli4->os, "topology Internal loopback\n");
		event->topology = SLI_LINK_TOPO_LOOPBACK_INTERNAL;
		break;
	case SLI4_LINK_ATTN_SERDES_LOOPBACK:
		ocs_log_debug(sli4->os, "topology serdes loopback\n");
		event->topology = SLI_LINK_TOPO_LOOPBACK_EXTERNAL;
		break;
	default:
		ocs_log_test(sli4->os, "topology: unknown\n");
		rc = -1;
		break;
	}

	event->speed = sli_fc_decode_port_speed(link_attn->port_speed);
	return rc;
}

static void
sli_fc_report_trunk_links(sli4_t *sli4, uint8_t ports_active, char *active_ports_str, size_t max_str_len)
{
	char tmp[8];
	char *ptr = active_ports_str;
	int i;

	ocs_memset(active_ports_str, 0x0, max_str_len);
	for (i = 0; i < 4; i++) {
		if (!(ports_active & (1 << i)))
			continue;

		if (!ocs_strlen(active_ports_str)) {
			ocs_snprintf(tmp, sizeof(tmp), "%d", i);
		} else {
			ocs_snprintf(tmp, sizeof(tmp), ",%d", i);
		}

		if (ocs_strlen(active_ports_str) + ocs_strlen(tmp) > max_str_len)
			break;

		ocs_strcpy(ptr, tmp);
		ptr += ocs_strlen(tmp);
	}

	if (!ocs_strlen(active_ports_str))
		ocs_strcpy(active_ports_str, "None");
}

static void
sli_fc_process_trunk_link_attention(sli4_t *sli4, sli4_link_attention_t *link_attn)
{
	uint8_t ports_active;
	uint32_t potential_agg_speed;
	uint32_t current_trunk_speed;
	char active_ports_str[16];

	potential_agg_speed = sli_fc_decode_port_speed(link_attn->port_speed);
	current_trunk_speed = link_attn->logical_link_speed * 10;	/* is in units of 10 Mbps */
	ports_active = link_attn->topology & SLI4_FC_TRUNK_LINK_STATE_MASK;

	switch (link_attn->topology & SLI4_FC_TRUNK_CONFIG_MASK) {
	case SLI4_FC_TRUNK_2_PORT_CONFIG:
		/* 2 port trunk, member ports 0/1 */
		sli_fc_report_trunk_links(sli4, ports_active, active_ports_str, sizeof(active_ports_str));
		ocs_log_debug(sli4->os,
			"[ Trunk ] Links=2, Ports={ Member={0,1}, Active={%s} }, Speed={max=%dMbps, current=%dMbps}\n",
			active_ports_str, potential_agg_speed, current_trunk_speed);
		break;
	case SLI4_FC_TRUNK_2_PORT_HI_CONFIG:
		/* 2 port trunk, member ports 2/3 */
		sli_fc_report_trunk_links(sli4, ports_active, active_ports_str, sizeof(active_ports_str));
		ocs_log_debug(sli4->os,
			"[ Trunk ] Links=2, Ports={ Member={2,3}, Active={%s} }, Speed={max=%dMbps, current=%dMbps}\n",
			active_ports_str, potential_agg_speed, current_trunk_speed);
		break;
	case SLI4_FC_TRUNK_4_PORT_CONFIG:
		/* All 4 ports are members of the same trunk */
		sli_fc_report_trunk_links(sli4, ports_active, active_ports_str, sizeof(active_ports_str));
		ocs_log_debug(sli4->os,
			"[ Trunk ] Links=4, Ports={ Member={0,1,2,3}, Active={%s} }, Speed={max=%dMbps, current=%dMbps}\n",
			active_ports_str, potential_agg_speed, current_trunk_speed);
		break;
	default:
		ocs_log_debug(sli4->os,
			"[ Trunk ] Invalid config: topology=%#x, reported max speed=%d Mbps, current speed=%d Mbps\n",
			link_attn->topology, potential_agg_speed, current_trunk_speed);
	}
}

/**
 * @ingroup sli_fc
 * @brief Process an asynchronous Link Attention event entry.
 *
 * @par Description
 * Parses Asynchronous Completion Queue Entry (ACQE),
 * creates an abstracted event, and calls the registered callback functions.
 *
 * @param sli4 SLI context.
 * @param acqe Pointer to the ACQE.
 *
 * @todo XXX all events return LINK_UP.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_fc_process_link_attention(sli4_t *sli4, void *acqe)
{
	sli4_link_attention_t	*link_attn = acqe;
	sli4_link_event_t	event = { 0 };
	int32_t rc = 0;

	ocs_log_debug(sli4->os, "link_number=%d attn_type=%#x topology=%#x port_speed=%#x "
			"port_fault=%#x shared_link_status=%#x logical_link_speed=%#x "
			"event_tag=%#x\n", link_attn->link_number, link_attn->attn_type,
			link_attn->topology, link_attn->port_speed, link_attn->port_fault,
			link_attn->shared_link_status, link_attn->logical_link_speed,
			link_attn->event_tag);

	if (!sli4->link) {
		return 0;
	}

	switch (link_attn->event_type) {
	case SLI4_FC_EVENT_LINK_ATTENTION:
		ocs_log_debug(sli4->os, "event_type: SLI4_FC_EVENT_LINK_ATTENTION\n");
		break;
	case SLI4_FC_EVENT_SHARED_LINK_ATTENTION:
		ocs_log_debug(sli4->os, "event_type: FC shared link event \n");
		break;
	default:
		ocs_log_test(sli4->os, "event_type: unknown, %d\n", link_attn->event_type);
		break;
	}

	event.medium   = SLI_LINK_MEDIUM_FC;

	switch (link_attn->attn_type) {
	case SLI4_LINK_ATTN_TYPE_LINK_UP:
	case SLI4_LINK_ATTN_TYPE_LINK_DOWN:
	case SLI4_LINK_ATTN_TYPE_NO_HARD_ALPA:
		rc = sli_fc_process_fc_link_attention(sli4, link_attn, &event);
		sli4->link(sli4->link_arg, (void *)&event);
		break;
	case SLI4_LINK_ATTN_TYPE_TRUNK_EVENT:
		sli_fc_process_trunk_link_attention(sli4, link_attn);
		event.status = SLI_LOGICAL_LINK_SPEED_CHANGED;
		event.logical_link_speed = link_attn->logical_link_speed * 10; /* is in units of 10 Mbps */
		event.aggregate_link_speed = sli_fc_decode_port_speed(link_attn->port_speed);
		sli4->link(sli4->link_arg, (void *)&event);
		break;
	default:
		ocs_log_test(sli4->os, "attn_type: unknown\n");
		rc = -1;
		break;
	}

	return rc;
}

/**
 * @ingroup sli_fc
 * @brief Parse an FC/FCoE work queue CQ entry.
 *
 * @param sli4 SLI context.
 * @param cq CQ to process.
 * @param cqe Pointer to the CQ entry.
 * @param etype CQ event type.
 * @param r_id Resource ID associated with this completion message (such as the IO tag).
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_fc_cqe_parse(sli4_t *sli4, sli4_queue_t *cq, uint8_t *cqe, sli4_qentry_e *etype,
		uint16_t *r_id)
{
	uint8_t		code = cqe[SLI4_CQE_CODE_OFFSET];
	int32_t		rc = -1;

	switch (code) {
	case SLI4_CQE_CODE_WORK_REQUEST_COMPLETION:
	{
		sli4_fc_wcqe_t *wcqe = (void *)cqe;

		*etype = SLI_QENTRY_WQ;
		*r_id = wcqe->request_tag;
		rc = wcqe->status;

		// Flag errors except for FCP_RSP_FAILURE
		if (rc && (rc != SLI4_FC_WCQE_STATUS_FCP_RSP_FAILURE)) {
			ocs_log_err(sli4->os, "WCQE: [ %08X %08X %08X %08X ]"
				    " status=%#x hw_status=%#x"
				    " tag=%#x w1=%#x w2=%#x xb=%d\n",
				    ((uint32_t *)cqe)[0], ((uint32_t *)cqe)[1],
				    ((uint32_t *)cqe)[2], ((uint32_t *)cqe)[3],
				    wcqe->status, wcqe->hw_status,
				    wcqe->request_tag, wcqe->wqe_specific_1,
				    wcqe->wqe_specific_2, wcqe->xb);
		}

//TODO: need to pass additional status back out of here as well as status (could overload rc as status/addlstatus
//	are only 8 bits each)
		break;
	}
	case SLI4_CQE_CODE_RQ_ASYNC:
	{
		sli4_fc_async_rcqe_t *rcqe = (void *)cqe;

		*etype = SLI_QENTRY_RQ;
		*r_id = rcqe->rq_id;
		rc = rcqe->status;
		break;
	}
	case SLI4_CQE_CODE_RQ_ASYNC_V1:
	{
		sli4_fc_async_rcqe_v1_t *rcqe = (void *)cqe;

		*etype = SLI_QENTRY_RQ;
		*r_id = rcqe->rq_id;
		rc = rcqe->status;
		break;
	}
	case SLI4_CQE_CODE_OPTIMIZED_WRITE_CMD:
	{
		sli4_fc_optimized_write_cmd_cqe_t *optcqe = (void *)cqe;

		*etype = SLI_QENTRY_OPT_WRITE_CMD;
		*r_id = optcqe->rq_id;
		rc = optcqe->status;
		break;
	}
	case SLI4_CQE_CODE_OPTIMIZED_WRITE_DATA:
	{
		sli4_fc_optimized_write_data_cqe_t *dcqe = (void *)cqe;

		*etype = SLI_QENTRY_OPT_WRITE_DATA;
		*r_id = dcqe->xri;
		rc = dcqe->status;

		// Flag errors
		if (rc != SLI4_FC_WCQE_STATUS_SUCCESS) {
			ocs_log_err(sli4->os, "Optimized DATA CQE: status=%#x ext_status=%#x hw_status=%#x xri=%#x dpl=%#x w3=%#x xb=%d\n",
				dcqe->status, ((uint32_t*) cqe)[2], dcqe->hw_status,
				dcqe->xri, dcqe->total_data_placed,
				((uint32_t*) cqe)[3], dcqe->xb);
		}
		break;
	}
	case SLI4_CQE_CODE_RQ_COALESCING:
	{
		sli4_fc_coalescing_rcqe_t *rcqe = (void *)cqe;

		*etype = SLI_QENTRY_RQ;
		*r_id = rcqe->rq_id;
		rc = rcqe->status;
		break;
	}
	case SLI4_CQE_CODE_XRI_ABORTED:
	{
		sli4_fc_xri_aborted_cqe_t *xa = (void *)cqe;

		*etype = SLI_QENTRY_XABT;
		*r_id = xa->xri;
		rc = 0;
		break;
	}
	case SLI4_CQE_CODE_RELEASE_WQE: {
		sli4_fc_wqec_t *wqec = (void*) cqe;

		*etype = SLI_QENTRY_WQ_RELEASE;
		*r_id = wqec->wq_id;
		rc = 0;
		break;
	}
	case SLI4_CQE_CODE_MARKER: {
		sli4_fc_marker_rcqe_t *marker_cqe = (void *)cqe;

		*etype = SLI_QENTRY_RQ;
		*r_id = marker_cqe->rq_id;
		rc = marker_cqe->status;
		break;
	}
	default:
		ocs_log_test(sli4->os, "CQE completion code %d not handled\n", code);
		*etype = SLI_QENTRY_MAX;
		*r_id = UINT16_MAX;
	}

	return rc;
}

/**
 * @ingroup sli_fc
 * @brief Return the ELS/CT response length.
 *
 * @param sli4 SLI context.
 * @param cqe Pointer to the CQ entry.
 *
 * @return Returns the length, in bytes.
 */
uint32_t
sli_fc_response_length(sli4_t *sli4, uint8_t *cqe)
{
	sli4_fc_wcqe_t *wcqe = (void *)cqe;

	return wcqe->wqe_specific_1;
}

/**
 * @ingroup sli_fc
 * @brief Return the FCP IO length.
 *
 * @param sli4 SLI context.
 * @param cqe Pointer to the CQ entry.
 *
 * @return Returns the length, in bytes.
 */
uint32_t
sli_fc_io_length(sli4_t *sli4, uint8_t *cqe)
{
	sli4_fc_wcqe_t *wcqe = (void *)cqe;

	return wcqe->wqe_specific_1;
}

/**
 * @ingroup sli_fc
 * @brief Retrieve the D_ID from the completion.
 *
 * @param sli4 SLI context.
 * @param cqe Pointer to the CQ entry.
 * @param d_id Pointer where the D_ID is written.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_fc_els_did(sli4_t *sli4, uint8_t *cqe, uint32_t *d_id)
{
	sli4_fc_wcqe_t *wcqe = (void *)cqe;

	*d_id = 0;

	if (wcqe->status) {
		return -1;
	} else {
		*d_id = wcqe->wqe_specific_2 & 0x00ffffff;
		return 0;
	}
}

uint32_t
sli_fc_ext_status(sli4_t *sli4, uint8_t *cqe)
{
	sli4_fc_wcqe_t *wcqe = (void *)cqe;
	uint32_t	mask;

	switch (wcqe->status) {
	case SLI4_FC_WCQE_STATUS_FCP_RSP_FAILURE:
		mask = UINT32_MAX;
		break;
	case SLI4_FC_WCQE_STATUS_LOCAL_REJECT:
	case SLI4_FC_WCQE_STATUS_CMD_REJECT:
		mask = 0xff;
		break;
	case SLI4_FC_WCQE_STATUS_NPORT_RJT:
	case SLI4_FC_WCQE_STATUS_FABRIC_RJT:
	case SLI4_FC_WCQE_STATUS_NPORT_BSY:
	case SLI4_FC_WCQE_STATUS_FABRIC_BSY:
	case SLI4_FC_WCQE_STATUS_LS_RJT:
		mask = UINT32_MAX;
		break;
	case SLI4_FC_WCQE_STATUS_DI_ERROR:
		mask = UINT32_MAX;
		break;
	default:
		mask = 0;
	}

	return wcqe->wqe_specific_2 & mask;
}

/**
 * @ingroup sli_fc
 * @brief Retrieve the RQ index from the completion.
 *
 * @param sli4 SLI context.
 * @param cqe Pointer to the CQ entry.
 * @param rq_id Pointer where the rq_id is written.
 * @param index Pointer where the index is written.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_fc_rqe_rqid_and_index(sli4_t *sli4, uint8_t *cqe, uint16_t *rq_id, uint32_t *index)
{
	sli4_fc_async_rcqe_t	*rcqe = (void *)cqe;
	sli4_fc_async_rcqe_v1_t	*rcqe_v1 = (void *)cqe;
	int32_t	rc = -1;
	uint8_t	code = 0;

	*rq_id = 0;
	*index = UINT32_MAX;

	code = cqe[SLI4_CQE_CODE_OFFSET];

	if (code == SLI4_CQE_CODE_RQ_ASYNC) {
		*rq_id = rcqe->rq_id;
		if (SLI4_FC_ASYNC_RQ_SUCCESS == rcqe->status) {
			*index = rcqe->rq_element_index;
			rc = 0;
		} else {
			*index = rcqe->rq_element_index;
			rc = rcqe->status;
			ocs_log_err_ratelimited(sli4->os, "RCQE: status=%02x (%s) rq_id=%d, index=%x pdpl=%x sof=%02x eof=%02x hdpl=%x\n",
				rcqe->status, sli_fc_get_status_string(rcqe->status), rcqe->rq_id,
				rcqe->rq_element_index, rcqe->payload_data_placement_length, rcqe->sof_byte,
				rcqe->eof_byte, rcqe->header_data_placement_length);
			ocs_log_err_ratelimited(sli4->os, "      %08X %08X %08X %08X\n", ((uint32_t *)cqe)[0],
				((uint32_t *)cqe)[1], ((uint32_t *)cqe)[2], ((uint32_t *)cqe)[3]);
		}
	} else if (code == SLI4_CQE_CODE_RQ_ASYNC_V1) {
		*rq_id = rcqe_v1->rq_id;
		if (SLI4_FC_ASYNC_RQ_SUCCESS == rcqe_v1->status) {
			*index = rcqe_v1->rq_element_index;
			rc = 0;
		} else {
			*index = rcqe_v1->rq_element_index;
			rc = rcqe_v1->status;
			ocs_log_err_ratelimited(sli4->os, "RCQE: status=%02x (%s) rq_id=%d, index=%x pdpl=%x sof=%02x eof=%02x hdpl=%x\n",
				rcqe_v1->status, sli_fc_get_status_string(rcqe_v1->status), rcqe_v1->rq_id,
				rcqe_v1->rq_element_index, rcqe_v1->payload_data_placement_length, rcqe_v1->sof_byte,
				rcqe_v1->eof_byte, rcqe_v1->header_data_placement_length);
			ocs_log_err_ratelimited(sli4->os, "      %08X %08X %08X %08X\n", ((uint32_t *)cqe)[0],
				((uint32_t *)cqe)[1], ((uint32_t *)cqe)[2], ((uint32_t *)cqe)[3]);
		}
	} else if (code == SLI4_CQE_CODE_OPTIMIZED_WRITE_CMD) {
		sli4_fc_optimized_write_cmd_cqe_t *optcqe = (void *)cqe;

		*rq_id = optcqe->rq_id;
		if (SLI4_FC_ASYNC_RQ_SUCCESS == optcqe->status) {
			*index = optcqe->rq_element_index;
			rc = 0;
		} else {
			*index = optcqe->rq_element_index;
			rc = optcqe->status;
			ocs_log_test_ratelimited(sli4->os, "status=%02x (%s) rq_id=%d, index=%x pdpl=%x hdpl=%x oox=%d agxr=%d xri=0x%x rpi=0x%x\n",
				optcqe->status, sli_fc_get_status_string(optcqe->status), optcqe->rq_id,
				optcqe->rq_element_index, optcqe->payload_data_placement_length,
				optcqe->header_data_placement_length, optcqe->oox, optcqe->tow, optcqe->xri,
				optcqe->rpi);
		}
	} else if (code == SLI4_CQE_CODE_RQ_COALESCING) {
		sli4_fc_coalescing_rcqe_t	*rcqe = (void *)cqe;

		*rq_id = rcqe->rq_id;
		if (SLI4_FC_COALESCE_RQ_SUCCESS == rcqe->status) {
			*index = rcqe->rq_element_index;
			rc = 0;
		} else {
			*index = UINT32_MAX;
			rc = rcqe->status;

			ocs_log_test_ratelimited(sli4->os, "status=%02x (%s) rq_id=%d, index=%x rq_id=%#x sdpl=%x\n",
				rcqe->status, sli_fc_get_status_string(rcqe->status), rcqe->rq_id,
				rcqe->rq_element_index, rcqe->rq_id, rcqe->sequence_reporting_placement_length);
		}
	} else if (code == SLI4_CQE_CODE_MARKER) {
		sli4_fc_marker_rcqe_t *rcqe = (void *)cqe;
		*rq_id = rcqe->rq_id;

		if (SLI4_FC_ASYNC_RQ_SUCCESS == rcqe->status) {
			*index = rcqe->rq_element_index;
			rc = 0;
		} else {
			*index = UINT32_MAX;
			rc = rcqe->status;
			ocs_log_test_ratelimited(sli4->os, "marker rcqe failed, status=%d\n", rc);
		}
	} else {
		*index = UINT32_MAX;

		rc = rcqe->status;

		ocs_log_debug_ratelimited(sli4->os, "status=%02x rq_id=%d, index=%x pdpl=%x sof=%02x eof=%02x hdpl=%x\n",
			rcqe->status, rcqe->rq_id, rcqe->rq_element_index, rcqe->payload_data_placement_length,
			rcqe->sof_byte, rcqe->eof_byte, rcqe->header_data_placement_length);
	}

	return rc;
}

/**
 * @ingroup sli_fc
 * @brief Process an asynchronous FCoE event entry.
 *
 * @par Description
 * Parses Asynchronous Completion Queue Entry (ACQE),
 * creates an abstracted event, and calls the registered callback functions.
 *
 * @param sli4 SLI context.
 * @param acqe Pointer to the ACQE.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_fc_process_fcoe(sli4_t *sli4, void *acqe)
{
	sli4_fcoe_fip_t	*fcoe = acqe;
	sli4_fip_event_t event = { 0 };
	uint32_t	mask = UINT32_MAX;

	ocs_log_debug(sli4->os, "ACQE FCoE FIP type=%02x count=%d tag=%#x\n",
			fcoe->event_type,
			fcoe->fcf_count,
			fcoe->event_tag);

	if (!sli4->fip) {
		return 0;
	}

	event.type = fcoe->event_type;
	event.index = UINT32_MAX;

	switch (fcoe->event_type) {
	case SLI4_FCOE_FIP_FCF_DISCOVERED:
		ocs_log_debug(sli4->os, "FCF Discovered index=%d\n", fcoe->event_information);
		break;
	case SLI4_FCOE_FIP_FCF_TABLE_FULL:
		ocs_log_debug(sli4->os, "FCF Table Full\n");
		mask = 0;
		break;
	case SLI4_FCOE_FIP_FCF_DEAD:
		ocs_log_debug(sli4->os, "FCF Dead/Gone index=%d\n", fcoe->event_information);
		break;
	case SLI4_FCOE_FIP_FCF_CLEAR_VLINK:
		mask = UINT16_MAX;
		ocs_log_debug(sli4->os, "Clear VLINK Received VPI=%#x\n", fcoe->event_information & mask);
		break;
	case SLI4_FCOE_FIP_FCF_MODIFIED:
		ocs_log_debug(sli4->os, "FCF Modified\n");
		break;
	default:
		ocs_log_test(sli4->os, "bad FCoE type %#x", fcoe->event_type);
		mask = 0;
	}

	if (mask != 0) {
		event.index = fcoe->event_information & mask;
	}

	sli4->fip(sli4->fip_arg, &event);

	return 0;
}

/**
 * @ingroup sli_fc
 * @brief Allocate a receive queue.
 *
 * @par Description
 * Allocates DMA memory and configures the requested queue type.
 *
 * @param sli4 SLI context.
 * @param q Pointer to the queue object for the header.
 * @param n_entries Number of entries to allocate.
 * @param buffer_size buffer size for the queue.
 * @param cq Associated CQ.
 * @param ulp The ULP to bind
 * @param is_hdr Used to validate the rq_id and set the type of queue
 *
 * @return Returns 0 on success, or -1 on failure.
 */
int32_t
sli_fc_rq_alloc(sli4_t *sli4, sli4_queue_t *q,
		uint32_t n_entries, uint32_t buffer_size,
		sli4_queue_t *cq, uint16_t ulp, uint8_t is_hdr)
{
	int32_t (*rq_create)(sli4_t *, void *, size_t, ocs_dma_t *, uint16_t, uint16_t, uint16_t);

	if ((sli4 == NULL) || (q == NULL)) {
		void *os = sli4 != NULL ? sli4->os : NULL;

		ocs_log_err(os, "bad parameter sli4=%p q=%p\n", sli4, q);
		return -1;
	}

	if (__sli_queue_init(sli4, q, SLI_QTYPE_RQ, SLI4_FCOE_RQE_SIZE, n_entries, SLI_PAGE_SIZE)) {
		return -1;
	}

	if (sli4->if_type == SLI4_IF_TYPE_BE3_SKH_PF) {
		rq_create = sli_cmd_fcoe_rq_create;
	} else {
		rq_create = sli_cmd_fcoe_rq_create_v1;
	}

	if (rq_create(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE, q->dma, cq->id, ulp, buffer_size)) {
		if (__sli_create_queue(sli4, q)) {
			ocs_log_err(sli4->os, "create %s failed\n", SLI_QNAME[SLI_QTYPE_RQ]);
			return -1;
		}

		if (is_hdr && q->id & 1) {
			ocs_log_test(sli4->os, "bad header RQ_ID %d\n", q->id);
			sli_queue_dma_free(sli4, q);
			return -1;
		} else if (!is_hdr  && (q->id & 1) == 0) {
			ocs_log_test(sli4->os, "bad data RQ_ID %d\n", q->id);
			sli_queue_dma_free(sli4, q);
			return -1;
		}
	} else {
		return -1;
	}

	q->u.flag.is_hdr = is_hdr;
	if (SLI4_IF_TYPE_BE3_SKH_PF == sli4->if_type) {
		q->u.flag.rq_batch = TRUE;
	}

	return 0;
}


/**
 * @ingroup sli_fc
 * @brief Allocate a receive queue set.
 *
 * @param sli4 SLI context.
 * @param num_rq_pairs to create
 * @param qs Pointers to the queue objects for both header and data.
 *	Length of this arrays should be 2 * num_rq_pairs
 * @param base_cq_id. Assumes base_cq_id : (base_cq_id + num_rq_pairs) cqs as allotted.
 * @param n_entries number of entries in each RQ queue.
 * @param header_buffer_size
 * @param payload_buffer_size
 * @param ulp The ULP to bind
 *
 * @return Returns 0 on success, or -1 on failure.
 */
int32_t
sli_fc_rq_set_alloc(sli4_t *sli4, uint32_t num_rq_pairs,
		    sli4_queue_t *qs[], uint32_t base_cq_id,
		    uint32_t n_entries, uint32_t header_buffer_size,
		    uint32_t payload_buffer_size,  uint16_t ulp)
{
	sli4_req_fcoe_rq_create_v2_t *req = NULL;
	sli4_res_common_create_queue_set_t *rsp = NULL;
	uint32_t	i, p, offset = 0;
	uint32_t	cmd_size, payload_size;
	uint32_t	total_page_count;
	uintptr_t	addr;
	ocs_dma_t	dma;
	size_t		qmem_size;
	uint32_t	qpage_size;

	ocs_memset(&dma, 0, sizeof(dma));

	for (i = 0; i < (num_rq_pairs * 2); i++) {
		if (__sli_queue_init(sli4, qs[i], SLI_QTYPE_RQ, SLI4_FCOE_RQE_SIZE, n_entries, SLI_PAGE_SIZE)) {
			goto error;
		}
	}

	qmem_size = sli_get_qmem_size(qs[0]->dma);
	qpage_size = sli_get_qpage_size(qmem_size);
	total_page_count = (sli_page_count(qmem_size, qpage_size) * num_rq_pairs * 2);

	/* Payload length must accomodate both request and response */
	cmd_size = (sizeof(sli4_req_fcoe_rq_create_v2_t) + (8 * total_page_count));
	payload_size = OCS_MAX((size_t)cmd_size, sizeof(sli4_res_common_create_queue_set_t));

	if (ocs_dma_alloc(sli4->os, &dma, payload_size, SLI_PAGE_SIZE, OCS_M_FLAGS_NONE)) {
		ocs_log_err(sli4->os, "DMA allocation failed\n");
		goto error;
	}
	ocs_memset(dma.virt, 0, payload_size);

	if (sli_cmd_sli_config(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE, payload_size, &dma) == -1) {
		goto error;
	}
	req = (sli4_req_fcoe_rq_create_v2_t *)((uint8_t *)dma.virt);

	/* Fill Header fields */
	req->hdr.opcode    = SLI4_OPC_FCOE_RQ_CREATE;
	req->hdr.subsystem = SLI4_SUBSYSTEM_FCFCOE;
	req->hdr.version   = 2;
	req->hdr.request_length = cmd_size - sizeof(sli4_req_hdr_t);

	/* Fill Payload fields */
	req->dnb 	   = sli4->config.rq_dnb;
	req->num_pages     = sli_page_count(qmem_size, qpage_size);
	req->rqe_count     = (qmem_size / SLI4_FCOE_RQE_SIZE);
	req->rqe_size      = SLI4_FCOE_RQE_SIZE_8;
	/* Fill the page_size_multiplier */
	req->page_size     = qpage_size / SLI_PAGE_SIZE;
	req->rq_count      = num_rq_pairs * 2;
	req->base_cq_id    = base_cq_id;
	req->hdr_buffer_size     = header_buffer_size;
	req->payload_buffer_size = payload_buffer_size;

	for (i = 0; i < (num_rq_pairs * 2); i++) {
		for (p = 0; p < req->num_pages; p++) {
			addr = qs[i]->dma[p].phys;
			req->page_physical_address[offset].low  = ocs_addr32_lo(addr);
			req->page_physical_address[offset].high = ocs_addr32_hi(addr);
			offset++;
		}
	}

	if (sli_bmbx_command(sli4)){
		ocs_log_crit(sli4->os, "bootstrap mailbox write faild RQSet\n");
		goto error;
	}

	rsp = (void *)((uint8_t *)dma.virt);
	if (rsp->hdr.status) {
		ocs_log_err(sli4->os, "bad create RQSet status=%#x addl=%#x\n",
			rsp->hdr.status, rsp->hdr.additional_status);
		goto error;
	} else {
		for (i = 0; i < (num_rq_pairs * 2); i++) {
			qs[i]->id = i + rsp->q_id;
			if ((qs[i]->id & 1) == 0) {
				qs[i]->u.flag.is_hdr = TRUE;
			} else {
				qs[i]->u.flag.is_hdr = FALSE;
			}
			qs[i]->doorbell_offset = regmap[SLI4_REG_FCOE_RQ_DOORBELL][sli4->if_type].off;
			qs[i]->doorbell_rset = regmap[SLI4_REG_FCOE_RQ_DOORBELL][sli4->if_type].rset;
		}
	}

	ocs_dma_free(sli4->os, &dma);

	return 0;

error:
	for (i = 0; i < (num_rq_pairs * 2); i++) {
		sli_queue_dma_free(sli4, qs[i]);
	}

	if (dma.size) {
		ocs_dma_free(sli4->os, &dma);
	}

	return -1;
}

/**
 * @ingroup sli_fc
 * @brief Get the RPI resource requirements.
 *
 * @param sli4 SLI context.
 * @param n_rpi Number of RPIs desired.
 *
 * @return Returns the number of bytes needed. This value may be zero.
 */
uint32_t
sli_fc_get_rpi_requirements(sli4_t *sli4, uint32_t n_rpi)
{
	uint32_t	bytes = 0;

	/* Check if header templates needed */
	if (sli4->config.hdr_template_req) {
		/* round up to a page */
		bytes = SLI_ROUND_PAGE(n_rpi * SLI4_FCOE_HDR_TEMPLATE_SIZE);
	}

	return bytes;
}

/**
 * @ingroup sli_fc
 * @brief Return a text string corresponding to a CQE status value
 *
 * @param status Status value
 *
 * @return Returns corresponding string, otherwise "unknown"
 */
const char *
sli_fc_get_status_string(uint32_t status)
{
	static struct {
		uint32_t code;
		const char *label;
	} lookup[] = {
		{SLI4_FC_WCQE_STATUS_SUCCESS,			"SUCCESS"},
		{SLI4_FC_WCQE_STATUS_FCP_RSP_FAILURE,		"FCP_RSP_FAILURE"},
		{SLI4_FC_WCQE_STATUS_REMOTE_STOP,		"REMOTE_STOP"},
		{SLI4_FC_WCQE_STATUS_LOCAL_REJECT,		"LOCAL_REJECT"},
		{SLI4_FC_WCQE_STATUS_NPORT_RJT,			"NPORT_RJT"},
		{SLI4_FC_WCQE_STATUS_FABRIC_RJT,		"FABRIC_RJT"},
		{SLI4_FC_WCQE_STATUS_NPORT_BSY,			"NPORT_BSY"},
		{SLI4_FC_WCQE_STATUS_FABRIC_BSY,		"FABRIC_BSY"},
		{SLI4_FC_WCQE_STATUS_LS_RJT,			"LS_RJT"},
		{SLI4_FC_WCQE_STATUS_CMD_REJECT,		"CMD_REJECT"},
		{SLI4_FC_WCQE_STATUS_FCP_TGT_LENCHECK,		"FCP_TGT_LENCHECK"},
		{SLI4_FC_WCQE_STATUS_RQ_BUF_LEN_EXCEEDED,	"BUF_LEN_EXCEEDED"},
		{SLI4_FC_WCQE_STATUS_RQ_INSUFF_BUF_NEEDED,	"RQ_INSUFF_BUF_NEEDED"},
		{SLI4_FC_WCQE_STATUS_RQ_INSUFF_FRM_DISC,	"RQ_INSUFF_FRM_DESC"},
		{SLI4_FC_WCQE_STATUS_RQ_DMA_FAILURE,		"RQ_DMA_FAILURE"},
		{SLI4_FC_WCQE_STATUS_FCP_RSP_TRUNCATE,		"FCP_RSP_TRUNCATE"},
		{SLI4_FC_WCQE_STATUS_DI_ERROR,			"DI_ERROR"},
		{SLI4_FC_WCQE_STATUS_BA_RJT,			"BA_RJT"},
		{SLI4_FC_WCQE_STATUS_RQ_INSUFF_XRI_NEEDED,	"RQ_INSUFF_XRI_NEEDED"},
		{SLI4_FC_WCQE_STATUS_RQ_INSUFF_XRI_DISC,	"INSUFF_XRI_DISC"},
		{SLI4_FC_WCQE_STATUS_RX_ERROR_DETECT,		"RX_ERROR_DETECT"},
		{SLI4_FC_WCQE_STATUS_RX_ABORT_REQUEST,		"RX_ABORT_REQUEST"},
		};
	uint32_t i;

	for (i = 0; i < ARRAY_SIZE(lookup); i++) {
		if (status == lookup[i].code) {
			return lookup[i].label;
		}
	}
	return "unknown";
}
/* vim: set noexpandtab textwidth=120: */
