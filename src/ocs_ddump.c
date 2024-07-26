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
 * generate FC ddump
 *
 */

#include "ocs.h"
#include "ocs_ddump.h"
#include "ocs_compat.h"

#define DEFAULT_SAVED_DUMP_SIZE		(4*1024*1024)

void hal_queue_ddump(ocs_textbuf_t *textbuf, ocs_hal_t *hal);

/**
 * @brief Generate sli4 queue ddump
 *
 * Generates sli4 queue ddump data
 *
 * @param textbuf pointer to text buffer
 * @param name name of SLI4 queue
 * @param hal pointer HAL context
 * @param q pointer to SLI4 queues array
 * @param q_count count of SLI4 queues
 * @param qentries number of SLI4 queue entries to dump
 *
 * @return none
 */

static void
ocs_ddump_sli4_queue(ocs_textbuf_t *textbuf, const char *name, ocs_hal_t *hal,
			sli4_queue_t **queue, uint32_t q_count, uint32_t qentries)
{
	uint32_t i, j;

	for (i = 0; i < q_count; i++) {
		sli4_queue_t *q = queue[i];

		ocs_assert(q);
		ocs_lock(&q->lock);
		ocs_ddump_section(textbuf, name, i);
		ocs_ddump_value(textbuf, "index", "%d", q->index);
		ocs_ddump_value(textbuf, "size", "%d", q->size);
		ocs_ddump_value(textbuf, "length", "%d", q->length);
		ocs_ddump_value(textbuf, "n_posted", "%d", q->n_posted);
		ocs_ddump_value(textbuf, "id", "%d", q->id);
		ocs_ddump_value(textbuf, "type", "%d", q->type);
		ocs_ddump_value(textbuf, "proc_limit", "%d", q->proc_limit);
		ocs_ddump_value(textbuf, "posted_limit", "%d", q->posted_limit);
		ocs_ddump_value(textbuf, "max_num_processed", "%d", q->max_num_processed);
		ocs_ddump_value(textbuf, "max_process_time", "%ld", (long)q->max_process_time);

		/* queue-specific information */
		switch (q->type) {
		case SLI_QTYPE_MQ:
			ocs_ddump_value(textbuf, "r_idx", "%d", q->u.r_idx);
			break;
		case SLI_QTYPE_CQ:
			ocs_ddump_value(textbuf, "is_mq", "%d", q->u.flag.is_mq);
			ocs_ddump_value(textbuf, "cq_last_processed_time", "%u", q->last_processed_time);
			break;
		case SLI_QTYPE_WQ:
			break;
		case SLI_QTYPE_EQ:
			ocs_ddump_value(textbuf, "eq_last_processed_time", "%u", q->last_processed_time);
			break;
		case SLI_QTYPE_RQ: {
			uint32_t i;
			uint32_t j;
			uint32_t rqe_count = 0;
			hal_rq_t *rq;

			ocs_ddump_value(textbuf, "is_hdr", "%d", q->u.flag.is_hdr);
			ocs_ddump_value(textbuf, "rq_batch", "%d", q->u.flag.rq_batch);

			/* loop through RQ tracker to see how many RQEs were produced */
			for (i = 0; i < hal->hal_rq_count; i++) {
				rq = hal->hal_rq[i];
				if (rq) {
					for (j = 0; j < rq->entry_count; j++) {
						if (rq->rq_tracker[j] != NULL) {
							rqe_count++;
						}
					}
				}
			}
			ocs_ddump_value(textbuf, "rqes_produced", "%d", rqe_count);
			break;
		}
		}

		for (j = 0; j < SLI_Q_DMA_CHUNKS && q->dma[j].virt; j++) {
			uint32_t page_num_qes = (q->dma[j].size / q->size); // (dma_page_size / qentry_size)

			ocs_ddump_value(textbuf, "virt_addr", "[%d]: %p", j, q->dma[j].virt);
			ocs_ddump_value(textbuf, "phys_addr", "[%d]: %lx", j, q->dma[j].phys);
			ocs_ddump_queue_entries(textbuf, q->dma[j].virt, q->size, page_num_qes,
				((q->type == SLI_QTYPE_MQ) ? q->u.r_idx : q->index), qentries);
		}
		ocs_unlock(&q->lock);
		ocs_ddump_endsection(textbuf, name, i);
	}
}


/**
 * @brief Generate SLI4 ddump
 *
 * Generates sli4 ddump
 *
 * @param textbuf pointer to text buffer
 * @param sli4 pointer SLI context
 * @param qtype SLI4 queue type
 *
 * @return none
 */

static void
ocs_ddump_sli_q_fields(ocs_textbuf_t *textbuf, sli4_t *sli4, sli4_qtype_e qtype)
{
	char * q_desc;

	switch(qtype) {
	case SLI_QTYPE_EQ: q_desc = "EQ"; break;
	case SLI_QTYPE_CQ: q_desc = "CQ"; break;
	case SLI_QTYPE_MQ: q_desc = "MQ"; break;
	case SLI_QTYPE_WQ: q_desc = "WQ"; break;
	case SLI_QTYPE_RQ: q_desc = "RQ"; break;
	default: q_desc = "unknown"; break;
	}

	ocs_ddump_section(textbuf, q_desc, qtype);

	ocs_ddump_value(textbuf, "max_qcount", "%d", sli4->config.max_qcount[qtype]);
	ocs_ddump_value(textbuf, "max_qentries", "%d", sli4->config.max_qentries[qtype]);
	ocs_ddump_value(textbuf, "qpage_count", "%d", sli4->config.qpage_count[qtype]);
	ocs_ddump_endsection(textbuf, q_desc, qtype);
}


/**
 * @brief Generate SLI4 ddump
 *
 * Generates sli4 ddump
 *
 * @param textbuf pointer to text buffer
 * @param sli4 pointer SLI context
 *
 * @return none
 */

static void
ocs_ddump_sli(ocs_textbuf_t *textbuf, sli4_t *sli4)
{
	sli4_sgl_chaining_params_t *cparams = &sli4->config.sgl_chaining_params;
	const char *p;
	int i;
	sli4_reg_t sli_reg;

	ocs_ddump_section(textbuf, "sli4_register_set_dump", 0);

	for (i = 0 ; i < SLI4_REG_MAX; i++) {
		char buf[256] = {0,};

		sli_reg_get(sli4, i, &sli_reg);
		if ((UINT32_MAX != sli_reg.rset) && (UINT32_MAX != sli_reg.off)) {
			ocs_snprintf(buf, sizeof(buf), "%x set: 0x%x offset: 0x%x value: %08x",
					i, sli_reg.rset, sli_reg.off,
					ocs_reg_read32(sli4->os, sli_reg.rset, sli_reg.off));
                	ocs_ddump_value(textbuf, "register", "%s", buf);
		}
	}

	ocs_ddump_endsection(textbuf, "sli4_register_set_dump", 0);

	ocs_ddump_section(textbuf, "sli4", 0);

	ocs_ddump_value(textbuf, "sli_rev", "%d", sli4->sli_rev);
	ocs_ddump_value(textbuf, "sli_family", "%d", sli4->sli_family);
	ocs_ddump_value(textbuf, "if_type", "%d", sli4->if_type);

	switch(sli4->asic_type) {
	case SLI4_ASIC_TYPE_BE3:
		p = "BE3";
		break;
	case SLI4_ASIC_TYPE_SKYHAWK:
		p = "Skyhawk";
		break;
	case SLI4_ASIC_TYPE_LANCER:
		p = "Lancer";
		break;
	case SLI4_ASIC_TYPE_LANCERG6:
		p = "LancerG6";
		break;
	case SLI4_ASIC_TYPE_LANCERG7:
		p = "LancerG7";
		break;
	case SLI4_ASIC_TYPE_LANCERG7PLUS:
		p = "LancerG7 PLUS";
		break;
	default:
		p = "Unknown";
		break;
	}
	ocs_ddump_value(textbuf, "asic_type", "%s", p);

	switch(sli4->asic_rev) {
	case SLI4_ASIC_REV_FPGA:
		p = "FPGA";
		break;
	case SLI4_ASIC_REV_A0:
		p = "A0";
		break;
	case SLI4_ASIC_REV_A1:
		p = "A1";
		break;
	case SLI4_ASIC_REV_A2:
		p = "A2";
		break;
	case SLI4_ASIC_REV_A3:
		p = "A3";
		break;
	case SLI4_ASIC_REV_B0:
		p = "B0";
		break;
	case SLI4_ASIC_REV_C0:
		p = "C0";
		break;
	case SLI4_ASIC_REV_D0:
		p = "D0";
		break;
	case SLI4_ASIC_REV_T:
		p = "T";
		break;
	default:
		p = "Unknown";
		break;
	}
	ocs_ddump_value(textbuf, "asic_rev", "%s", p);

	ocs_ddump_value(textbuf, "e_d_tov", "%d", sli4->config.e_d_tov);
	ocs_ddump_value(textbuf, "r_a_tov", "%d", sli4->config.r_a_tov);
	ocs_ddump_value(textbuf, "link_module_type", "%d", sli4->config.link_module_type);
	ocs_ddump_value(textbuf, "rq_batch", "%d", sli4->config.rq_batch);
	ocs_ddump_value(textbuf, "topology", "%d", sli4->config.topology);
	ocs_ddump_value(textbuf, "wwpn", "%02x%02x%02x%02x%02x%02x%02x%02x",
			 sli4->config.wwpn[0],
			 sli4->config.wwpn[1],
			 sli4->config.wwpn[2],
			 sli4->config.wwpn[3],
			 sli4->config.wwpn[4],
			 sli4->config.wwpn[5],
			 sli4->config.wwpn[6],
			 sli4->config.wwpn[7]);
	ocs_ddump_value(textbuf, "wwnn", "%02x%02x%02x%02x%02x%02x%02x%02x",
			 sli4->config.wwnn[0],
			 sli4->config.wwnn[1],
			 sli4->config.wwnn[2],
			 sli4->config.wwnn[3],
			 sli4->config.wwnn[4],
			 sli4->config.wwnn[5],
			 sli4->config.wwnn[6],
			 sli4->config.wwnn[7]);
	ocs_ddump_value(textbuf, "fw_rev0", "%d", sli4->config.fw_rev[0]);
	ocs_ddump_value(textbuf, "fw_rev1", "%d", sli4->config.fw_rev[1]);
	ocs_ddump_value(textbuf, "fw_name0", "%s", (char*)sli4->config.fw_name[0]);
	ocs_ddump_value(textbuf, "fw_name1", "%s", (char*)sli4->config.fw_name[1]);
	ocs_ddump_value(textbuf, "hw_rev0", "%x", sli4->config.hw_rev[0]);
	ocs_ddump_value(textbuf, "hw_rev1", "%x", sli4->config.hw_rev[1]);
	ocs_ddump_value(textbuf, "hw_rev2", "%x", sli4->config.hw_rev[2]);
	ocs_ddump_value(textbuf, "sge_supported_length", "%x", sli4->config.sge_supported_length);
	ocs_ddump_value(textbuf, "sgl_page_sizes", "%x", sli4->config.sgl_page_sizes);
	ocs_ddump_value(textbuf, "max_sgl_pages", "%x", sli4->config.max_sgl_pages);
	ocs_ddump_value(textbuf, "high_login_mode", "%x", sli4->config.high_login_mode);
	ocs_ddump_value(textbuf, "sgl_pre_registered", "%x", sli4->config.sgl_pre_registered);
	ocs_ddump_value(textbuf, "sgl_pre_registration_required", "%x", sli4->config.sgl_pre_registration_required);

	ocs_ddump_value(textbuf, "sgl_chaining_capable", "%x", cparams->chaining_capable);
	ocs_ddump_value(textbuf, "frag_num_field_offset", "%x", cparams->frag_num_field_offset);
	ocs_ddump_value(textbuf, "frag_num_field_mask", "%016" PRIx64 "", cparams->frag_num_field_mask);
	ocs_ddump_value(textbuf, "sgl_index_field_offset", "%x", cparams->sgl_index_field_offset);
	ocs_ddump_value(textbuf, "sgl_index_field_mask", "%016" PRIx64 "", cparams->sgl_index_field_mask);
	ocs_ddump_value(textbuf, "chain_sge_initial_value_lo", "%x", cparams->chain_sge_initial_value_lo);
	ocs_ddump_value(textbuf, "chain_sge_initial_value_hi", "%x", cparams->chain_sge_initial_value_hi);

	ocs_ddump_value(textbuf, "max_vfi", "%d", sli_get_max_rsrc(sli4, SLI_RSRC_FCOE_VFI));
	ocs_ddump_value(textbuf, "max_vpi", "%d", sli_get_max_rsrc(sli4, SLI_RSRC_FCOE_VPI));
	ocs_ddump_value(textbuf, "max_rpi", "%d", sli_get_max_rsrc(sli4, SLI_RSRC_FCOE_RPI));
	ocs_ddump_value(textbuf, "max_xri", "%d", sli_get_max_rsrc(sli4, SLI_RSRC_FCOE_XRI));
	ocs_ddump_value(textbuf, "max_fcfi", "%d", sli_get_max_rsrc(sli4, SLI_RSRC_FCOE_FCFI));

	ocs_ddump_sli_q_fields(textbuf, sli4, SLI_QTYPE_EQ);
	ocs_ddump_sli_q_fields(textbuf, sli4, SLI_QTYPE_CQ);
	ocs_ddump_sli_q_fields(textbuf, sli4, SLI_QTYPE_MQ);
	ocs_ddump_sli_q_fields(textbuf, sli4, SLI_QTYPE_WQ);
	ocs_ddump_sli_q_fields(textbuf, sli4, SLI_QTYPE_RQ);

	ocs_ddump_endsection(textbuf, "sli4", 0);
}


/**
 * @brief Dump HAL IO
 *
 * Dump HAL IO
 *
 * @param textbuf pointer to text buffer
 * @param io pointer to HAL IO object
 *
 * @return none
 */

static void
ocs_ddump_hal_io(ocs_textbuf_t *textbuf, ocs_hal_io_t *io)
{
	ocs_assert(io);

	ocs_ddump_section(textbuf, "hal_io", io->indicator);

	ocs_ddump_value(textbuf, "state", "%d", io->state);
	ocs_ddump_value(textbuf, "xri", "0x%x", io->indicator);
	ocs_ddump_value(textbuf, "tag", "0x%x", io->reqtag);
	ocs_ddump_value(textbuf, "abort_reqtag", "0x%x", io->abort_reqtag);
	ocs_ddump_value(textbuf, "ref_count", "%d", ocs_ref_read_count(&io->ref));

	/* just to make it obvious, display abort bit from tag */
	ocs_ddump_value(textbuf, "abort_issued", "0x%x", io->abort_issued);
	ocs_ddump_value(textbuf, "abort_completed", "0x%x", io->abort_completed);
	ocs_ddump_value(textbuf, "wq_index", "%d", (io->wq == NULL ? 0xffff : io->wq->instance));
	ocs_ddump_value(textbuf, "type", "%d", io->type);
	ocs_ddump_value(textbuf, "xbusy", "%d", io->xbusy);
	ocs_ddump_value(textbuf, "active_wqe_link", "%d", ocs_list_on_list(&io->wqe_link));
	ocs_ddump_value(textbuf, "def_sgl_count", "%d", io->def_sgl_count);
	ocs_ddump_value(textbuf, "n_sge", "%d", io->n_sge);
	ocs_ddump_value(textbuf, "has_ovfl_sgl", "%s", (io->ovfl_sgl != NULL ? "TRUE" : "FALSE"));
	ocs_ddump_value(textbuf, "has_ovfl_io", "%s", (io->ovfl_io != NULL ? "TRUE" : "FALSE"));

	ocs_ddump_endsection(textbuf, "hal_io", io->indicator);
}

#if defined(OCS_DEBUG_QUEUE_HISTORY)
/**
 * @brief Generate queue history ddump
 *
 * @param textbuf pointer to text buffer
 * @param q_hist Pointer to queue history object.
 */
static void
ocs_ddump_queue_history(ocs_textbuf_t *textbuf, ocs_hal_q_hist_t *q_hist)
{
	uint32_t x;

	if (!textbuf)
		return;

	ocs_ddump_section(textbuf, "q_hist", 0);
	ocs_ddump_value(textbuf, "count", "%ld", OCS_Q_HIST_SIZE);
	ocs_ddump_value(textbuf, "index", "%d", q_hist->q_hist_index);

	if (q_hist->q_hist == NULL) {
		ocs_ddump_section(textbuf, "history", 0);
		ocs_textbuf_printf(textbuf, "No history available\n");
		ocs_ddump_endsection(textbuf, "history", 0);
		ocs_ddump_endsection(textbuf, "q_hist", 0);
		return;
	}

	/* start from last entry and go backwards */
	ocs_textbuf_printf(textbuf, "<history>\n");
	ocs_textbuf_printf(textbuf, "(newest first):\n");

	ocs_lock(&q_hist->q_hist_lock);
	x = ocs_queue_history_prev_index(q_hist->q_hist_index);
	do {
		int i;
		ocs_q_hist_ftr_t ftr;
		uint32_t mask;


		/* footer's mask indicates what words were captured */
		ftr.word = q_hist->q_hist[x];
		mask = ftr.s.mask;
		i = 0;

		/* if we've encountered a mask of 0, must be done */
		if (mask == 0) {
			break;
		}

		/* display entry type */
		ocs_textbuf_printf(textbuf, "%s:\n",
				   ocs_queue_history_type_name(ftr.s.type));

		if (ocs_queue_history_timestamp_enabled()) {
			uint64_t tsc_value;
			x = ocs_queue_history_prev_index(x);
			tsc_value = ((q_hist->q_hist[x]) & 0x00000000FFFFFFFFull);
			x = ocs_queue_history_prev_index(x);
			tsc_value |= (((uint64_t)q_hist->q_hist[x] << 32) & 0xFFFFFFFF00000000ull);
			ocs_textbuf_printf(textbuf, " t: %" PRIu64 "\n", tsc_value);
		}

		if (ocs_queue_history_q_info_enabled()) {
			if (ftr.s.type == OCS_Q_HIST_TYPE_CWQE ||
			    ftr.s.type == OCS_Q_HIST_TYPE_CXABT ||
			    ftr.s.type == OCS_Q_HIST_TYPE_WQE) {
				x = ocs_queue_history_prev_index(x);
				ocs_textbuf_printf(textbuf, " qid=0x%x idx=0x%x\n",
						   ((q_hist->q_hist[x] >> 16) & 0xFFFF),
						   ((q_hist->q_hist[x] >> 0) & 0xFFFF));
			}
		}

		while (mask) {
			if ((mask & 1) && (x != q_hist->q_hist_index)){
				/* get next word */
				x = ocs_queue_history_prev_index(x);
				ocs_textbuf_printf(textbuf, " [%d]=%x\n",
						   i, q_hist->q_hist[x]);
			}
			mask = (mask >> 1UL);
			i++;
		}

		/* go backwards to next element */
		x = ocs_queue_history_prev_index(x);
	} while (x != ocs_queue_history_prev_index(q_hist->q_hist_index));
	ocs_unlock(&q_hist->q_hist_lock);

	ocs_textbuf_printf(textbuf, "</history>\n");
	ocs_ddump_endsection(textbuf, "q_hist", 0);
}
#endif

/**
 * @brief Generate hal ddump
 *
 * Generates hal ddump
 *
 * @param textbuf pointer to text buffer
 * @param hal pointer HAL context
 * @param flags ddump flags
 * @param qentries number of qentries to dump
 *
 * @return none
 */

static void
ocs_ddump_hal(ocs_textbuf_t *textbuf, ocs_hal_t *hal, uint32_t flags, uint32_t qentries)
{
	ocs_t *ocs = hal->os;
	uint32_t cnt = 0;
	ocs_hal_io_t *io = NULL;
	uint32_t i;
	uint32_t j;
	uint32_t max_rpi = sli_get_max_rsrc(&hal->sli, SLI_RSRC_FCOE_RPI);

	ocs_assert(ocs);

	ocs_ddump_section(textbuf, "hal", ocs->instance_index);

	/* device specific information */
	switch(hal->sli.if_type) {
	case 0:
		ocs_ddump_value(textbuf, "uerr_mask_hi", "%08x",
				 sli_reg_read(&hal->sli, SLI4_REG_UERR_MASK_HI));
		ocs_ddump_value(textbuf, "uerr_mask_lo", "%08x",
				 sli_reg_read(&hal->sli, SLI4_REG_UERR_MASK_LO));
		ocs_ddump_value(textbuf, "uerr_status_hi", "%08x",
				 sli_reg_read(&hal->sli, SLI4_REG_UERR_STATUS_HI));
		ocs_ddump_value(textbuf, "uerr_status_lo", "%08x",
				 sli_reg_read(&hal->sli, SLI4_REG_UERR_STATUS_LO));
		break;
	case 2:
		ocs_ddump_value(textbuf, "sliport_status", "%08x",
				 sli_reg_read(&hal->sli, SLI4_REG_SLIPORT_STATUS));
		ocs_ddump_value(textbuf, "sliport_error1", "%08x",
				 sli_reg_read(&hal->sli, SLI4_REG_SLIPORT_ERROR1));
		ocs_ddump_value(textbuf, "sliport_error2", "%08x",
				 sli_reg_read(&hal->sli, SLI4_REG_SLIPORT_ERROR2));
		break;
	}

	ocs_ddump_value(textbuf, "link_status", "%d", hal->link.status);
	ocs_ddump_value(textbuf, "link_speed", "%d", hal->link.speed);
	ocs_ddump_value(textbuf, "link_topology", "%d", hal->link.topology);
	ocs_ddump_value(textbuf, "state", "%d", hal->state);
	ocs_ddump_value(textbuf, "io_alloc_failed_count", "%d", ocs_atomic_read(&hal->io_alloc_failed_count));
	ocs_ddump_value(textbuf, "n_io", "%d", hal->config.n_io);

	ocs_ddump_value(textbuf, "queue_topology", "%s", hal->config.queue_topology);
	ocs_ddump_value(textbuf, "rr_quanta", "%d", hal->config.rr_quanta);
	for (i = 0; i < ARRAY_SIZE(hal->config.filter_def); i++) {
		ocs_ddump_value(textbuf, "filter_def", "%08X", hal->config.filter_def[i]);
	}
	ocs_ddump_value(textbuf, "n_eq", "%d", hal->eq_count);
	ocs_ddump_value(textbuf, "n_cq", "%d", hal->cq_count);
	ocs_ddump_value(textbuf, "n_mq", "%d", hal->mq_count);
	ocs_ddump_value(textbuf, "n_rq", "%d", hal->rq_count);
	ocs_ddump_value(textbuf, "n_wq", "%d", hal->wq_count);
	ocs_ddump_value(textbuf, "n_sgl", "%d", hal->config.n_sgl);
	ocs_ddump_value(textbuf, "watchdog_timeout", "%d", hal->watchdog_timeout);

	ocs_ddump_sli(textbuf, &hal->sli);

	ocs_ddump_sli4_queue(textbuf, "wq", hal, hal->wq, hal->wq_count,
			     ((flags & OCS_DDUMP_FLAGS_WQES) ? qentries : 0));
	ocs_ddump_sli4_queue(textbuf, "rq", hal, hal->rq, hal->rq_count,
			     ((flags & OCS_DDUMP_FLAGS_RQES) ? qentries : 0));
	ocs_ddump_sli4_queue(textbuf, "mq", hal, hal->mq, hal->mq_count,
			     ((flags & OCS_DDUMP_FLAGS_MQES) ? qentries : 0));
	ocs_ddump_sli4_queue(textbuf, "cq", hal, hal->cq, hal->cq_count,
			     ((flags & OCS_DDUMP_FLAGS_CQES) ? qentries : 0));
	ocs_ddump_sli4_queue(textbuf, "eq", hal, hal->eq, hal->eq_count,
			     ((flags & OCS_DDUMP_FLAGS_EQES) ? qentries : 0));

	/* dump the IO quarantine list */
	for (i = 0; i < hal->wq_count; i++) {
		ocs_ddump_section(textbuf, "io_quarantine", i);
		ocs_ddump_value(textbuf, "quarantine_index", "%d", hal->hal_wq[i]->quarantine_info.quarantine_index);
		for (j = 0; j < OCS_HAL_QUARANTINE_QUEUE_DEPTH; j++) {
			if (hal->hal_wq[i]->quarantine_info.quarantine_ios[j] != NULL) {
				ocs_ddump_hal_io(textbuf, hal->hal_wq[i]->quarantine_info.quarantine_ios[j]);
			}
		}
		ocs_ddump_endsection(textbuf, "io_quarantine", i);
	}

	ocs_ddump_section(textbuf, "workaround", ocs->instance_index);
	ocs_ddump_value(textbuf, "fwrev", "%08" PRIx64, hal->workaround.fwrev);
	ocs_ddump_endsection(textbuf, "workaround", ocs->instance_index);

	ocs_lock(&hal->io_lock);
		ocs_ddump_section(textbuf, "io_inuse", ocs->instance_index);
		ocs_list_foreach(&hal->io_inuse, io) {
			ocs_ddump_hal_io(textbuf, io);
		}
		ocs_ddump_endsection(textbuf, "io_inuse", ocs->instance_index);

		ocs_ddump_section(textbuf, "io_wait_free", ocs->instance_index);
		ocs_list_foreach(&hal->io_wait_free, io) {
			ocs_ddump_hal_io(textbuf, io);
		}
		ocs_ddump_endsection(textbuf, "io_wait_free", ocs->instance_index);
		ocs_ddump_section(textbuf, "io_free", ocs->instance_index);
		ocs_list_foreach(&hal->io_free, io) {
			if (io->xbusy) {
				/* only display free ios if they're active */
				ocs_ddump_hal_io(textbuf, io);
			}
			cnt++;
		}
		ocs_ddump_endsection(textbuf, "io_free", ocs->instance_index);
		ocs_ddump_value(textbuf, "ios_free", "%d", cnt);

	ocs_ddump_value(textbuf, "sec_hio_wait_count", "%d", hal->sec_hio_wait_count);
	ocs_unlock(&hal->io_lock);

	/* now check the IOs not in a list; i.e. sequence coalescing xris */
	ocs_ddump_section(textbuf, "port_owned_ios", ocs->instance_index);
	for (i = 0; i < hal->config.n_io; i++) {
		io = hal->io[i];
		if (!io)
			continue;

		if (ocs_hal_is_xri_port_owned(hal, io->indicator)) {
			if (ocs_ref_read_count(&io->ref)) {
				/* only display free ios if they're active */
				ocs_ddump_hal_io(textbuf, io);
			}
		}
	}
	ocs_ddump_endsection(textbuf, "port_owned_ios", ocs->instance_index);

	ocs_textbuf_printf(textbuf, "<rpi_ref>");
	for (i = 0; i < max_rpi; i++) {
		if (ocs_atomic_read(&hal->rpi_ref[i].rpi_attached) ||
			ocs_atomic_read(&hal->rpi_ref[i].rpi_count) ) {
			ocs_textbuf_printf(textbuf, "[%d] att=%d cnt=%d\n", i,
				ocs_atomic_read(&hal->rpi_ref[i].rpi_attached),
				ocs_atomic_read(&hal->rpi_ref[i].rpi_count));
		}
	}
	ocs_textbuf_printf(textbuf, "</rpi_ref>");

	for (i = 0; i < hal->wq_count; i++) {
		ocs_ddump_value(textbuf, "wq_submit", "%d", hal->tcmd_wq_submit[i]);
	}
	for (i = 0; i < hal->wq_count; i++) {
		ocs_ddump_value(textbuf, "wq_complete", "%d", hal->tcmd_wq_complete[i]);
	}

	hal_queue_ddump(textbuf, hal);

	ocs_ddump_endsection(textbuf, "hal", ocs->instance_index);

	//sli4_link_event_t link;
}

void
hal_queue_ddump(ocs_textbuf_t *textbuf, ocs_hal_t *hal)
{
	hal_eq_t *eq;
	hal_cq_t *cq;
	hal_q_t *q;
	hal_mq_t *mq;
	hal_wq_t *wq;
	hal_rq_t *rq;

	ocs_ddump_section(textbuf, "hal_queue", 0);
	ocs_list_foreach(&hal->eq_list, eq) {
		ocs_ddump_section(textbuf, "eq", eq->instance);
		ocs_ddump_value(textbuf, "queue-id", "%d", eq->queue->id);
		OCS_STAT(ocs_ddump_value(textbuf, "use_count", "%d", eq->use_count));
		ocs_list_foreach(&eq->cq_list, cq) {
			ocs_ddump_section(textbuf, "cq", cq->instance);
			ocs_ddump_value(textbuf, "queue-id", "%d", cq->queue->id);
			OCS_STAT(ocs_ddump_value(textbuf, "use_count", "%d", cq->use_count));
			ocs_list_foreach(&cq->q_list, q) {
				switch(q->type) {
				case SLI_QTYPE_MQ:
					mq = (hal_mq_t *) q;
					ocs_ddump_section(textbuf, "mq", mq->instance);
					ocs_ddump_value(textbuf, "queue-id", "%d", mq->queue->id);
					OCS_STAT(ocs_ddump_value(textbuf, "use_count", "%d", mq->use_count));
					ocs_ddump_endsection(textbuf, "mq", mq->instance);
					break;
				case SLI_QTYPE_WQ:
					wq = (hal_wq_t *) q;
					ocs_ddump_section(textbuf, "wq", wq->instance);
					ocs_ddump_value(textbuf, "queue-id", "%d", wq->queue->id);
					OCS_STAT(ocs_ddump_value(textbuf, "use_count", "%d", wq->use_count));
					ocs_ddump_value(textbuf, "wqec_count", "%d", wq->wqec_count);
					ocs_ddump_value(textbuf, "free_count", "%d", wq->free_count);
					ocs_ddump_value(textbuf, "fw_sfq_enabled", "%d", wq->fw_sfq_enabled);
					OCS_STAT(ocs_ddump_value(textbuf, "wq_pending_count", "%d",
								 wq->wq_pending_count));
					ocs_ddump_endsection(textbuf, "wq", wq->instance);
					break;
				case SLI_QTYPE_RQ:
					rq = (hal_rq_t *) q;
					ocs_ddump_section(textbuf, "rq", rq->instance);
					OCS_STAT(ocs_ddump_value(textbuf, "use_count", "%d", rq->use_count));
					ocs_ddump_value(textbuf, "filter_mask", "%d", rq->filter_mask);
					if (rq->hdr != NULL) {
						ocs_ddump_value(textbuf, "hdr-id", "%d", rq->hdr->id);
						OCS_STAT(ocs_ddump_value(textbuf, "hdr_use_count", "%d", rq->hdr_use_count));
					}
					if (rq->first_burst != NULL) {
						OCS_STAT(ocs_ddump_value(textbuf, "fb-id", "%d", rq->first_burst->id));
						OCS_STAT(ocs_ddump_value(textbuf, "fb_use_count", "%d", rq->fb_use_count));
					}
					if (rq->data != NULL) {
						OCS_STAT(ocs_ddump_value(textbuf, "payload-id", "%d", rq->data->id));
						OCS_STAT(ocs_ddump_value(textbuf, "payload_use_count", "%d", rq->payload_use_count));
					}
					ocs_ddump_value(textbuf, "rq_empty_warn_count", "%d", rq->rq_empty_warn_count);
					ocs_ddump_value(textbuf, "rq_empty_err_count", "%d", rq->rq_empty_err_count);
					ocs_ddump_endsection(textbuf, "rq", rq->instance);
					break;
				default:
					break;
				}
			}
			ocs_ddump_endsection(textbuf, "cq", cq->instance);
		}
		ocs_ddump_endsection(textbuf, "eq", eq->instance);
	}
	ocs_ddump_endsection(textbuf, "hal_queue", 0);
}

#define OCS_PCI_CONFIG_SPACE_SIZE	4096

void
ocs_ddump_pci_config_space(ocs_t *ocs, ocs_textbuf_t *textbuf)
{
	void *config_space;
	uint32_t offset;

	config_space = ocs_malloc(ocs, OCS_PCI_CONFIG_SPACE_SIZE, OCS_M_NOWAIT | OCS_M_ZERO);
	if (!config_space) {
		ocs_log_err(ocs, "failed to allocate memory\n");
		return;
	}

	
	ocs_ddump_section(textbuf, "PCI config space data", ocs->instance_index);
	for (offset = 0; offset < OCS_PCI_CONFIG_SPACE_SIZE; offset += sizeof(uint32_t)) {
		*((uint32_t*)(config_space + offset)) = ocs_config_read32(ocs, offset);
	}

	ocs_ddump_buffer(textbuf, "PCI config space data", ocs->instance_index,
			config_space, OCS_PCI_CONFIG_SPACE_SIZE);

	ocs_ddump_endsection(textbuf, "PCI config space data", ocs->instance_index);

	ocs_ddump_section(textbuf, "PCI bar map dump ", ocs->instance_index);

	/* Dump BAR0 address space */
	if (ocs->ocs_os.bars[0].vaddr) {
		ocs_ddump_buffer(textbuf, "PCI_bar_address_map_dump_BAR0", ocs->instance_index,
				ocs->ocs_os.bars[0].vaddr, ocs->ocs_os.bars[0].size);
	}
	ocs_ddump_endsection(textbuf, "PCI bar map dump", ocs->instance_index);

	ocs_free(ocs, config_space, OCS_PCI_CONFIG_SPACE_SIZE);
}

/**
 * @brief Initiate ddump
 *
 * Traverses the ocs/domain/port/node/io data structures to generate a driver
 * dump.
 *
 * @param ocs pointer to device context
 * @param textbuf pointer to text buffer
 * @param flags ddump flags
 * @param qentries number of queue entries to dump
 *
 * @return Returns 0 on success, or a negative value on failure.
 */

int
ocs_ddump(ocs_t *ocs, ocs_textbuf_t *textbuf, uint32_t flags, uint32_t qentries)
{
	ocs_xport_t *xport = ocs->xport;
	ocs_domain_t *domain;
	uint32_t instance;
	ocs_vport_spec_t *vport;
	ocs_io_t *io;
	int retval = 0;
	uint32_t i;

	ocs_ddump_startfile(textbuf);

	ocs_ddump_section(textbuf, "ocs", ocs->instance_index);

	ocs_ddump_section(textbuf, "ocs_os", ocs->instance_index);
	ocs_ddump_value(textbuf, "numa_node", "%d", ocs->ocs_os.numa_node);

	/* PCI config dump */
	ocs_ddump_pci_config_space(ocs, textbuf);

	ocs_ddump_endsection(textbuf, "ocs_os", ocs->instance_index);

	ocs_ddump_value(textbuf, "drv_name", "%s", DRV_NAME);
	ocs_ddump_value(textbuf, "drv_version", "%s", DRV_VERSION);
	ocs_ddump_value(textbuf, "display_name", "%s", ocs->display_name);
	ocs_ddump_value(textbuf, "enable_ini", "%d", ocs->enable_ini);
	ocs_ddump_value(textbuf, "enable_tgt", "%d", ocs->enable_tgt);
	ocs_ddump_value(textbuf, "enable_hlm", "%d", ocs->enable_hlm);
	ocs_ddump_value(textbuf, "hlm_group_size", "%d", ocs->hlm_group_size);
	ocs_ddump_value(textbuf, "fw_dump_type", "%d", ocs->fw_dump.type);
	ocs_ddump_value(textbuf, "max_isr_time_msec", "%d", ocs->max_isr_time_msec);

	/* Dump XPORT data */
	if (xport) {
		ocs_ddump_value(textbuf, "lip_count", "%d", xport->lip_count);
		ocs_ddump_value(textbuf, "nodes_count", "%d", xport->nodes_count);
		ocs_ddump_value(textbuf, "io_alloc_failed_count", "%d", ocs_atomic_read(&xport->io_alloc_failed_count));
		ocs_ddump_value(textbuf, "io_active_count", "%d", ocs_atomic_read(&xport->io_active_count));
		ocs_ddump_value(textbuf, "io_total_alloc", "%d", ocs_atomic_read(&xport->io_total_alloc));
		ocs_ddump_value(textbuf, "io_total_free", "%d", ocs_atomic_read(&xport->io_total_free));
		ocs_ddump_value(textbuf, "io_pending_count", "%d", ocs_atomic_read(&xport->io_pending_count));
		ocs_ddump_value(textbuf, "io_total_pending", "%d", ocs_atomic_read(&xport->io_total_pending));
		ocs_ddump_value(textbuf, "io_pending_recursing", "%d", ocs_atomic_read(&xport->io_pending_recursing));

		for (i = 0; i < SLI4_MAX_FCFI; i++) {
			ocs_lock(&xport->fcfi[i].pend_frames_lock);
			if (!ocs_list_empty(&xport->fcfi[i].pend_frames)) {
				ocs_hal_sequence_t *frame;
				ocs_ddump_section(textbuf, "pending_frames", i);
				ocs_ddump_value(textbuf, "hold_frames", "%d", xport->fcfi[i].hold_frames);
				ocs_list_foreach(&xport->fcfi[i].pend_frames, frame) {
					fc_header_t *hdr;
					char buf[256];

					hdr = frame->header.data;
					ocs_snprintf(buf, sizeof(buf), "%02x/%06x/%02x/%02x/%04x/%04x len 0x%x",
						     hdr->type, fc_be24toh(hdr->f_ctl), hdr->r_ctl, hdr->info,
						     ocs_be16toh(hdr->ox_id), ocs_be16toh(hdr->rx_id),
						     frame->payload.data_len);
					ocs_ddump_value(textbuf, "frame", "%s", buf);
				}
				ocs_ddump_endsection(textbuf, "pending_frames", i);
			}
			ocs_unlock(&xport->fcfi[i].pend_frames_lock);
		}

		ocs_lock(&xport->io_pending_lock);
		ocs_ddump_section(textbuf, "io_pending_list", ocs->instance_index);
		ocs_list_foreach(&xport->io_pending_list, io) {
			ocs_ddump_io(textbuf, io);
		}
		ocs_ddump_endsection(textbuf, "io_pending_list", ocs->instance_index);
		ocs_unlock(&xport->io_pending_lock);


		/* Dump any pending vports */
		if (ocs_device_lock_try(ocs) != TRUE) {
			/* Didn't get the lock */
			return -1;
		}
		instance = 0;
		ocs_list_foreach(&xport->vport_list, vport) {
			ocs_ddump_section(textbuf, "vport_spec", instance);
			ocs_ddump_value(textbuf, "domain_instance", "%d", vport->domain_instance);
			ocs_ddump_value(textbuf, "wwnn", "%" PRIx64, vport->wwnn);
			ocs_ddump_value(textbuf, "wwpn", "%" PRIx64, vport->wwpn);
			ocs_ddump_value(textbuf, "fc_id", "0x%x", vport->fc_id);
			ocs_ddump_value(textbuf, "enable_tgt", "%d", vport->enable_tgt);
			ocs_ddump_value(textbuf, "enable_ini", "%d" PRIx64, vport->enable_ini);
			ocs_ddump_endsection(textbuf, "vport_spec", instance ++);
		}
		ocs_device_unlock(ocs);
	}
#if defined(ENABLE_LOCK_DEBUG)
	/* Dump the lock list */
	ocs_ddump_section(textbuf, "locks", 0);
	ocs_lock(&ocs->ocs_os.locklist_lock); {
		ocs_lock_t *l;
		uint32_t idx = 0;
		ocs_list_foreach(&ocs->ocs_os.locklist, l) {
			ocs_ddump_section(textbuf, "lock", idx);
			ocs_ddump_value(textbuf, "name", "%s", l->name);
			ocs_ddump_value(textbuf, "inuse", "%d", l->inuse);
			ocs_ddump_value(textbuf, "caller", "%p", l->caller[0]);
			ocs_ddump_value(textbuf, "pid", "%08x", l->pid.id);
			ocs_ddump_endsection(textbuf, "lock", idx);
			idx++;
		}
	} ocs_unlock(&ocs->ocs_os.locklist_lock);
	ocs_ddump_endsection(textbuf, "locks", 0);
#endif
	/* Dump target and initiator private data */
	ocs_scsi_ini_ddump(textbuf, OCS_SCSI_DDUMP_DEVICE, ocs);
	ocs_scsi_tgt_ddump(textbuf, OCS_SCSI_DDUMP_DEVICE, ocs);

	ocs_ddump_hal(textbuf, &ocs->hal, flags, qentries);

	if (ocs_device_lock_try(ocs) != TRUE) {
		/* Didn't get the lock */
		return -1;
	}
		/* Here the device lock is held */
		ocs_list_foreach(&ocs->domain_list, domain) {
			retval = ocs_ddump_domain(textbuf, domain);
			if (retval != 0) {
				break;
			}
		}

		/* Dump ramlog */
		ocs_ddump_ramlog(textbuf, ocs->ramlog);
	ocs_device_unlock(ocs);

#if !defined(OCS_DEBUG_QUEUE_HISTORY)
	ocs_ddump_section(textbuf, "q_hist", ocs->instance_index);
	ocs_textbuf_printf(textbuf, "<history>\n");
	ocs_textbuf_printf(textbuf, "No history available\n");
	ocs_textbuf_printf(textbuf, "</history>\n");
	ocs_ddump_endsection(textbuf, "q_hist", ocs->instance_index);
#else
	ocs_ddump_queue_history(textbuf, &ocs->hal.q_hist);
#endif

#if defined(OCS_DEBUG_MEMORY)
	ocs_memory_allocated_ddump(textbuf);
#endif

	ocs_ddump_endsection(textbuf, "ocs", ocs->instance_index);

	ocs_ddump_endfile(textbuf);

	return retval;
}

/**
 * @brief Capture and save ddump
 *
 * Captures and saves a ddump to the ocs_t structure to save the
 * current state. The goal of this function is to save a ddump
 * as soon as an issue is encountered. The saved ddump will be
 * kept until the user reads it.
 *
 * @param ocs pointer to device context
 * @param flags ddump flags
 * @param qentries number of queue entries to dump
 *
 * @return 0 if ddump was saved; > 0 of one already exists; < 0
 *         error
 */

int32_t
ocs_save_ddump(ocs_t *ocs, uint32_t flags, uint32_t qentries)
{
	if (!ocs_textbuf_initialized(&ocs->ddump_saved)) {
		ocs_log_err(ocs, "Saved ddump not allocated\n");
		return -1;
	}

	ocs_log_debug(ocs, "Saving ddump\n");
	ocs_ddump(ocs, &ocs->ddump_saved, flags, qentries);
	ocs_log_debug(ocs, "Saved ddump: %d bytes written, max size = %d\n", ocs_textbuf_get_written(&ocs->ddump_saved), ocs->ddump_max_size);

	return 0;
}

/**
 * @brief Set the driver dump state
 *
 * Valid state changes
 * DDUMP_NONE ---> DDUMP_ALLOCATING
 * DDUMP_ALLOCATING --> DDUMP_ALLOCATED
 * DDUMP_ALLOCATED --> DDUMP_STARTED
 * DDUMP_STARTED --> DDUMP_PRESENT
 * DDUMP_PRESENT --> DDUMP_RETRIEVING
 * DDUMP_RETRIEVING --> DDUMP_NONE
 *
 * A thread has the ownership of the ddump data structure when in the following states:
 *
 *  - DDUMP_ALLOCATING
 *  - DDUMP_STARTED
 *  - DDUMP_RETRIEVING
 *
 * so it should move the state to one of the other 3 states after finishing processing
 * to givie up the ownership
 *
 * @param ocs pointer to device context
 * @param state driver dump state to change
 *
 * @return: TRUE <if status changed successfully> or
 *          FALSE <failed to set dump status>
 */

bool
ocs_ddump_state_set(ocs_t *ocs, uint8_t state)
{
	bool set_state = FALSE;

	ocs_assert(ocs, FALSE);

	ocs_device_lock(ocs);

	switch (state) {
	/* caller setting these states should have the ownership */
	case OCS_DDUMP_NONE:
	case OCS_DDUMP_ALLOCATED:
	case OCS_DDUMP_PRESENT:
		ocs->ddump_state = state;
		set_state = TRUE;
		break;

	/* caller setting these states is trying to "win" the ownership */
	case OCS_DDUMP_ALLOCATING:
		if (ocs->ddump_state == OCS_DDUMP_NONE) {
			ocs->ddump_state = state;
			set_state = TRUE;
		}
		break;
	case OCS_DDUMP_STARTED:
		if (ocs->ddump_state == OCS_DDUMP_ALLOCATED) {
			ocs->ddump_state = state;
			set_state = TRUE;
		}
		break;
	case OCS_DDUMP_RETRIEVING:
		if (ocs->ddump_state == OCS_DDUMP_PRESENT) {
			ocs->ddump_state = state;
			set_state = TRUE;
		}
		break;
	}

	if (set_state) {
		ocs_log_debug(ocs, "Driver dump state set to %d successfully\n", state);
	} else {
		ocs_log_err(ocs, "failed to set driver dump state(%d) present state: %d\n", state, ocs->ddump_state);
	}

	ocs_device_unlock(ocs);

	return set_state;
}

/**
 * @brief Capture and save ddump for all OCS instances
 *
 * Calls ocs_save_ddump() for each OCS instance.
 *
 * @param flags ddump flags
 * @param qentries number of queue entries to dump
 * @param alloc_flag allocate dump buffer if not already allocated
 *
 * @return 0 if ddump was saved; > 0 of one already exists; < 0
 *         error
 */

int32_t
ocs_save_ddump_all(uint32_t flags, uint32_t qentries, uint32_t alloc_flag)
{
	ocs_t *ocs;
	uint32_t i;
	int32_t rc = 0;

	for_each_active_ocs(i, ocs) {
		rc = ocs_get_saved_ddump(ocs, flags, qentries, alloc_flag);
		if (rc < 0) {
			ocs_log_err(ocs, "save ddump failed\n");
			break;
		}
	}
	return rc;
}

/**
 * @brief Clear the previously saved ddump, and collect
 *        the ddump for all OCS instances of the adapter
 */
int32_t
ocs_adapter_ddump_clear_and_save(ocs_t *ocs, uint8_t bus, uint8_t dev,
				 uint32_t flags, uint32_t qentries, uint32_t alloc_flag)
{
	ocs_t *other_ocs;
	uint8_t other_bus, other_dev, other_func;
	int32_t index = 0, rc = 0;

	for_each_active_ocs(index, other_ocs) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			if (alloc_flag) {
				/* Clear the previously saved ddump */
				ocs_clear_saved_ddump(other_ocs);

				rc = ocs_textbuf_alloc(other_ocs, &other_ocs->ddump_saved, DEFAULT_SAVED_DUMP_SIZE);
				if (rc) {
					ocs_log_err(other_ocs, "textbuf_alloc failed\n");
					goto clear_ddump;
				}
			}

			rc = ocs_save_ddump(other_ocs, flags, qentries);
			if (rc < 0) {
				ocs_log_err(other_ocs, "Save ddump failed\n");
				goto clear_ddump;
			}

			ocs_log_debug(other_ocs, "Saved the driver dump successfully\n");
		}
	}

	return rc;

clear_ddump:
	index = 0;
	for_each_active_ocs(index, other_ocs) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev))
			ocs_clear_saved_ddump(other_ocs);
	}

	return rc;
}

int8_t
ocs_get_saved_ddump(ocs_t *ocs, uint32_t flags, uint32_t qentries, uint32_t alloc_flag)
{
	int rc = 0;

	if (alloc_flag && ocs_ddump_state_set(ocs, OCS_DDUMP_ALLOCATING)) {
		if (sli_get_asic_type(&ocs->hal.sli) == SLI4_ASIC_TYPE_LANCERG7PLUS)
			ocs->ddump_max_size = OCS_DDUMP_MAX_SIZE;
		else
			ocs->ddump_max_size = OCS_DDUMP_MAX_SIZE / 3;

		ocs_log_debug(ocs, "Allocating driver dump buffer pool\n");
		rc = ocs_textbuf_pool_alloc(ocs, &ocs->ddump_saved, ocs->ddump_max_size);
		if (rc) {
			ocs_log_err(ocs, "textbuf_alloc failed\n");
			ocs_ddump_state_set(ocs, OCS_DDUMP_NONE);
			return -1;
		}
		ocs_ddump_state_set(ocs, OCS_DDUMP_ALLOCATED);
	}

	if (!ocs_ddump_state_set(ocs, OCS_DDUMP_STARTED))
		return 1;

	/* Clear ddump if it's already exists */
	ocs_clear_saved_ddump(ocs);

	rc = ocs_save_ddump(ocs, flags, qentries);
	if (rc == 0) {
		ocs_ddump_state_set(ocs, OCS_DDUMP_PRESENT);
	} else {
		if (ocs->ddump_pre_alloc) {
			ocs_clear_saved_ddump(ocs);
			ocs_ddump_state_set(ocs, OCS_DDUMP_ALLOCATED);
		} else {
			ocs_textbuf_free(ocs, &ocs->ddump_saved);
			ocs_ddump_state_set(ocs, OCS_DDUMP_NONE);
		}
	}

	return rc;
}

/**
 * @brief Capture and save ddump for all OCS instances of adapter
 *
 * Calls ocs_save_ddump() for each OCS instance of HBA
 *
 * @param flags ddump flags
 * @param qentries number of queue entries to dump
 * @param alloc_flag allocate dump buffer if not already allocated
 * @param adapter_dump function or adapter specific dump
 *
 * @return 0 if ddump was saved; < 0 error
 */

int32_t
ocs_adapter_save_ddump(ocs_t *ocs, uint32_t flags, uint32_t qentries, uint32_t alloc_flag, bool adapter_dump)
{
	ocs_t *other_ocs;
	uint32_t i;
	int32_t rc = 0;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;

	/* Check if the device is initialized and attached */
	if (!ocs->drv_ocs.attached) {
		ocs_log_info(ocs, "ocs device not initialized yet, skip driver dump\n");
		return rc;
	}

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	for_each_active_ocs(i, other_ocs) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			if (!adapter_dump && (func != other_func))
				continue;

			rc = ocs_get_saved_ddump(other_ocs, flags, qentries, alloc_flag);
			if (rc < 0)
				return rc;
		}
	}

	return rc;
}

/**
 * @brief Clear saved ddump
 *
 * Clears saved ddump to make room for next one.
 *
 * @param ocs pointer to device context
 *
 * @return 0 if ddump was cleared; > 0 no saved ddump found
 */

int32_t
ocs_clear_saved_ddump(ocs_t *ocs)
{
	/* if there's a saved ddump, copy to newly allocated textbuf */
	if (ocs_textbuf_get_written(&ocs->ddump_saved)) {
		ocs_log_debug(ocs, "saved ddump cleared\n");
		ocs_textbuf_reset(&ocs->ddump_saved);
		return 0;
	} else {
		ocs_log_debug(ocs, "no saved ddump found\n");
		return 1;
	}
}

