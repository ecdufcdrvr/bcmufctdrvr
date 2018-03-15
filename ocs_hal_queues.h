/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2018 Broadcom.  All Rights Reserved.
 *  The term "Broadcom" refers to Broadcom Limited and/or its subsidiaries.
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
 *
 */

#ifndef __OCS_HAL_QUEUES_H__
#define __OCS_HAL_QUEUES_H__

#define OCS_HAL_MQ_DEPTH	128

typedef enum {
	QTOP_EQ = 0,
	QTOP_CQ,
	QTOP_WQ,
	QTOP_RQ,
	QTOP_MQ,
	QTOP_LAST,
} ocs_hal_qtop_entry_e;

typedef struct {
	ocs_hal_qtop_entry_e entry;
	uint8_t set_default;
	uint32_t len;
	uint8_t class;
	uint8_t ulp;
	uint8_t filter_mask;
	uint8_t policy;
	bool protocol_valid;
	uint8_t protocol;
} ocs_hal_qtop_entry_t;

typedef struct {
	struct rq_config {
		hal_eq_t *eq;
		uint32_t len;
		uint8_t class;
		uint8_t ulp;
		uint8_t filter_mask;
		uint8_t policy;
		bool protocol_valid;
		uint8_t protocol;
	} rq_cfg[OCS_HAL_MAX_RQ_PAIRS]; // 8 Filters
	uint32_t num_pairs;
} ocs_hal_rqs_info_t;

typedef struct {
	uint32_t num_pairs;
	uint32_t len;
	uint8_t class;
	uint8_t ulp;
	uint8_t filter_mask;
	uint8_t policy;
	bool protocol_valid;
	uint8_t protocol;
	hal_eq_t *eqs[OCE_HAL_MAX_NUM_MRQ_PAIRS];
} ocs_hal_mrq_info_t;


#define MAX_TOKENS			256
#define OCS_HAL_MAX_QTOP_ENTRIES	200

typedef struct {
	ocs_os_handle_t os;
	ocs_hal_qtop_entry_t *entries;
	uint32_t alloc_count;
	uint32_t inuse_count;
	uint32_t entry_counts[QTOP_LAST];
	uint32_t rptcount[10];
	uint32_t rptcount_idx;
} ocs_hal_qtop_t;

extern ocs_hal_qtop_t *ocs_hal_qtop_parse(ocs_hal_t *hal, const char *qtop_string);
extern void ocs_hal_qtop_free(ocs_hal_qtop_t *qtop);
extern const char *ocs_hal_qtop_entry_name(ocs_hal_qtop_entry_e entry);
extern uint32_t ocs_hal_qtop_eq_count(ocs_hal_t *hal);

extern ocs_hal_rtn_e ocs_hal_init_queues(ocs_hal_t *hal, ocs_hal_qtop_t *qtop);
extern void hal_thread_eq_handler(ocs_hal_t *hal, hal_eq_t *eq, uint32_t max_isr_time_msec);
extern void hal_thread_cq_handler(ocs_hal_t *hal, hal_cq_t *cq);
extern  hal_wq_t *ocs_hal_queue_next_wq(ocs_hal_t *hal, ocs_hal_io_t *io);

#endif /* __OCS_HAL_QUEUES_H__ */
