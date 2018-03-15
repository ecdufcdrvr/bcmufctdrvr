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
 * OCS debug declarations
 */

/**
 * @page fc_general_purpose_overview General Purpose APIs
 * - @ref debug
 * - @ref spv
 *
 * <div class="overview">
 *
 * <h2>Debug Support Functions</h2>
 *
 * These functions provide general purpose debug facilities.
 *
 * <h2>Sparse Vector</h2>
 *
 * See @ref spv and the description in spv.h.
 *
 * <br><br>
 * </div><!-- overview -->
 */

#ifndef _OCS_DEBUG_H
#define _OCS_DEBUG_H

/* Uncomment this line to enable logging extended queue history
 */
//#define OCS_DEBUG_QUEUE_HISTORY


/* Allocate maximum allowed (4M) */
#if defined(OCS_DEBUG_QUEUE_HISTORY)
#define OCS_Q_HIST_SIZE (1000000UL)		/* Size in words */
#endif

#define OCS_LOG_ENABLE_SM_TRACE(ocs)		(((ocs) != NULL) ? (((ocs)->logmask & (1U << 0)) != 0) : 0)
#define OCS_LOG_ENABLE_ELS_TRACE(ocs)		(((ocs) != NULL) ? (((ocs)->logmask & (1U << 1)) != 0) : 0)
#define OCS_LOG_ENABLE_SCSI_TRACE(ocs)		(((ocs) != NULL) ? (((ocs)->logmask & (1U << 2)) != 0) : 0)
#define OCS_LOG_ENABLE_SCSI_TGT_TRACE(ocs)	(((ocs) != NULL) ? (((ocs)->logmask & (1U << 3)) != 0) : 0)
#define OCS_LOG_ENABLE_DOMAIN_SM_TRACE(ocs)	(((ocs) != NULL) ? (((ocs)->logmask & (1U << 4)) != 0) : 0)
#define OCS_LOG_ENABLE_Q_FULL_BUSY_MSG(ocs)	(((ocs) != NULL) ? (((ocs)->logmask & (1U << 5)) != 0) : 0)
#define OCS_LOG_ENABLE_IO_ERRORS(ocs)		(((ocs) != NULL) ? (((ocs)->logmask & (1U << 6)) != 0) : 0)


extern void ocs_dump32(uint32_t, ocs_os_handle_t, const char *, void *, uint32_t);
extern void ocs_debug_enable(uint32_t mask);
extern void ocs_debug_disable(uint32_t mask);
extern int ocs_debug_is_enabled(uint32_t mask);
extern void ocs_debug_attach(void *);
extern void ocs_debug_detach(void *);

#if defined(OCS_DEBUG_QUEUE_HISTORY)

/**
 * @brief Queue history footer
 */
typedef union ocs_q_hist_ftr_u {
	uint32_t word;
	struct {
#define Q_HIST_TYPE_LEN 		3
#define Q_HIST_MASK_LEN 		29
		uint32_t mask:Q_HIST_MASK_LEN,
			 type:Q_HIST_TYPE_LEN;
	} s;
} ocs_q_hist_ftr_t;


/**
 * @brief WQE command mask lookup
 */
typedef struct ocs_q_hist_wqe_mask_s {
	uint8_t command;
	uint32_t mask;
} ocs_q_hist_wqe_mask_t;

/**
 * @brief CQE mask lookup
 */
typedef struct ocs_q_hist_cqe_mask_s {
	uint8_t ctype;
	uint32_t :Q_HIST_MASK_LEN,
		 type:Q_HIST_TYPE_LEN;
	uint32_t mask;
	uint32_t mask_err;
} ocs_q_hist_cqe_mask_t;

/**
 * @brief Queue history type
 */
typedef enum {
	/* changes need to be made to ocs_queue_history_type_name() as well */
	OCS_Q_HIST_TYPE_WQE = 0,
	OCS_Q_HIST_TYPE_CWQE,
	OCS_Q_HIST_TYPE_CXABT,
	OCS_Q_HIST_TYPE_MISC,
} ocs_q_hist_type_t;

static __inline const char *
ocs_queue_history_type_name(ocs_q_hist_type_t type)
{
	switch (type) {
	case OCS_Q_HIST_TYPE_WQE: return "wqe"; break;
	case OCS_Q_HIST_TYPE_CWQE: return "wcqe"; break;
	case OCS_Q_HIST_TYPE_CXABT: return "xacqe"; break;
	case OCS_Q_HIST_TYPE_MISC: return "misc"; break;
	default: return "unknown"; break;
	}
}

typedef struct {
	ocs_t		*ocs;
	uint32_t	*q_hist;
	uint32_t	q_hist_index;
	ocs_lock_t	q_hist_lock;
} ocs_hal_q_hist_t;

extern void ocs_queue_history_cqe(ocs_hal_q_hist_t*, uint8_t, uint32_t *, uint8_t, uint32_t, uint32_t);
extern void ocs_queue_history_wq(ocs_hal_q_hist_t*, uint32_t *, uint32_t, uint32_t);
extern void ocs_queue_history_misc(ocs_hal_q_hist_t*, uint32_t *, uint32_t);
extern void ocs_queue_history_init(ocs_t *, ocs_hal_q_hist_t*);
extern void ocs_queue_history_free(ocs_hal_q_hist_t*);
extern uint32_t ocs_queue_history_prev_index(uint32_t);
extern uint8_t ocs_queue_history_q_info_enabled(void);
extern uint8_t ocs_queue_history_timestamp_enabled(void);
#else
#define ocs_queue_history_wq(...)
#define ocs_queue_history_cqe(...)
#define ocs_queue_history_misc(...)
#define ocs_queue_history_init(...)
#define ocs_queue_history_free(...)
#endif

#define OCS_DEBUG_ALWAYS		(1U << 0)
#define OCS_DEBUG_ENABLE_MQ_DUMP	(1U << 1)
#define OCS_DEBUG_ENABLE_CQ_DUMP	(1U << 2)
#define OCS_DEBUG_ENABLE_WQ_DUMP	(1U << 3)
#define OCS_DEBUG_ENABLE_EQ_DUMP	(1U << 4)
#define OCS_DEBUG_ENABLE_SPARAM_DUMP	(1U << 5)

extern void _ocs_assert(const char *cond, const char *filename, int linenum);

#define ocs_assert(cond, ...) \
	do { \
		if (!(cond)) { \
			_ocs_assert(#cond, __FILE__, __LINE__); \
			return __VA_ARGS__; \
		} \
	} while (0)

extern void ocs_dump_service_params(const char *label, void *sparms);
extern void ocs_display_sparams(const char *prelabel, const char *reqlabel, int dest, void *textbuf, void *sparams);

#endif /* !_OCS_DEBUG_H */
