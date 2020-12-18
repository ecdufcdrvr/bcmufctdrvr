/*
 * Copyright (C) 2020 Broadcom. All Rights Reserved.
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
 * Provides general purpose debug facilities
 */
/*!
@defgroup debug Debug Support Functions
*/

#include "ocs.h"

#define OCS_DEBUG_ENABLE(x)	(x ? ~0 : 0)

// change '0' to '1' to enable debug feature
#define OCS_DEBUG_MASK \
	(OCS_DEBUG_ENABLE(1)	& OCS_DEBUG_ALWAYS)  | \
	(OCS_DEBUG_ENABLE(0)	& OCS_DEBUG_ENABLE_MQ_DUMP) | \
	(OCS_DEBUG_ENABLE(0)	& OCS_DEBUG_ENABLE_CQ_DUMP) | \
	(OCS_DEBUG_ENABLE(0)	& OCS_DEBUG_ENABLE_WQ_DUMP) | \
	(OCS_DEBUG_ENABLE(0)	& OCS_DEBUG_ENABLE_EQ_DUMP) | \
	(OCS_DEBUG_ENABLE(0)	& OCS_DEBUG_ENABLE_SPARAM_DUMP)

static uint32_t ocs_debug_mask = OCS_DEBUG_MASK;

static int
_isprint(int c) {
	return ((c > 32) && (c < 127));
}

/**
 * @ingroup debug
 * @brief enable debug options
 *
 * Enables debug options by or-ing in <b>mask</b> into the currently enabled
 * debug mask.
 *
 * @param mask mask bits to enable
 *
 * @return none
 */

void ocs_debug_enable(uint32_t mask) {
	ocs_debug_mask |= mask;
}

/**
 * @ingroup debug
 * @brief disable debug options
 *
 * Disables debug options by clearing bits in <b>mask</b> into the currently enabled
 * debug mask.
 *
 * @param mask mask bits to enable
 *
 * @return none
 */

void ocs_debug_disable(uint32_t mask) {
	ocs_debug_mask &= ~mask;
}

/**
 * @ingroup debug
 * @brief return true if debug bits are enabled
 *
 * Returns true if the request debug bits are set.
 *
 * @param mask debug bit mask
 *
 * @return true if corresponding bits are set
 *
 * @note Passing in a mask value of zero always returns true
 */

int ocs_debug_is_enabled(uint32_t mask) {
	return (ocs_debug_mask & mask) == mask;
}


/**
 * @ingroup debug
 * @brief Dump 32 bit hex/ascii data
 *
 * Dumps using ocs_log a buffer of data as 32 bit hex and ascii
 *
 * @param mask debug enable bits
 * @param os os handle
 * @param label text label for the display (may be NULL)
 * @param buf pointer to data buffer
 * @param buf_length length of data buffer
 *
 * @return none
 *
 */

void
ocs_dump32(uint32_t mask, ocs_os_handle_t os, const char *label, void *buf, uint32_t buf_length)
{
	uint32_t word_count = buf_length / sizeof(uint32_t);
	uint32_t i;
	uint32_t columns = 8;
	uint32_t n;
	uint32_t *wbuf;
	char *cbuf;
	uint32_t addr = 0;
	char linebuf[200];
	char *pbuf = linebuf;

	if (!ocs_debug_is_enabled(mask))
		return;

	if (label)
		ocs_log_info(os, "%s\n", label);

	wbuf = buf;
	while (word_count > 0) {
		pbuf = linebuf;
		pbuf += ocs_snprintf(pbuf, sizeof(linebuf) - (pbuf-linebuf), "%08X:  ", addr);

		n = word_count;
		if (n > columns)
			n = columns;

		for (i = 0; i < n; i ++)
			pbuf += ocs_snprintf(pbuf, sizeof(linebuf) - (pbuf-linebuf), "%08X ", wbuf[i]);

		for (; i < columns; i ++)
			pbuf += ocs_snprintf(pbuf, sizeof(linebuf) - (pbuf-linebuf), "%8s ", "");

		pbuf += ocs_snprintf(pbuf, sizeof(linebuf) - (pbuf-linebuf), "    ");
		cbuf = (char*)wbuf;
		for (i = 0; i < n*sizeof(uint32_t); i ++)
			pbuf += ocs_snprintf(pbuf, sizeof(linebuf) - (pbuf-linebuf), "%c", _isprint(cbuf[i]) ? cbuf[i] : '.');
		pbuf += ocs_snprintf(pbuf, sizeof(linebuf) - (pbuf-linebuf), "\n");

		ocs_log_info(os, "%s", linebuf);

		wbuf += n;
		word_count -= n;
		addr += n*sizeof(uint32_t);
	}
}


#if defined(OCS_DEBUG_QUEUE_HISTORY)

/* each bit corresponds to word to capture */
#define OCS_Q_HIST_WQE_WORD_MASK_DEFAULT	(BIT(4) | BIT(6) | BIT(7) | BIT(9) | BIT(12))
#define OCS_Q_HIST_TRECV_CONT_WQE_WORD_MASK	(BIT(4) | BIT(5) | BIT(6) | BIT(7) | BIT(9) | BIT(12))
#define OCS_Q_HIST_IWRITE_WQE_WORD_MASK		(BIT(4) | BIT(5) | BIT(6) | BIT(7) | BIT(9))
#define OCS_Q_HIST_IREAD_WQE_WORD_MASK		(BIT(4) | BIT(6) | BIT(7) | BIT(9))
#define OCS_Q_HIST_ABORT_WQE_WORD_MASK		(BIT(3) | BIT(7) | BIT(8) | BIT(9))
#define OCS_Q_HIST_WCQE_WORD_MASK		(BIT(0) | BIT(3))
#define OCS_Q_HIST_WCQE_WORD_MASK_ERR		(BIT(0) | BIT(1) | BIT(2) | BIT(3))
#define OCS_Q_HIST_CQXABT_WORD_MASK		(BIT(0) | BIT(1) | BIT(2) | BIT(3))

/* if set, will provide extra queue information in each entry */
#define OCS_Q_HIST_ENABLE_Q_INFO	0
uint8_t ocs_queue_history_q_info_enabled(void)
{
	return OCS_Q_HIST_ENABLE_Q_INFO;
}

/* if set, will provide timestamps in each entry */
#define OCS_Q_HIST_ENABLE_TIMESTAMPS	0
uint8_t ocs_queue_history_timestamp_enabled(void)
{
	return OCS_Q_HIST_ENABLE_TIMESTAMPS;
}

/* Add WQEs and masks to override default WQE mask */
ocs_q_hist_wqe_mask_t ocs_q_hist_wqe_masks[] = {
	/* WQE command   Word mask */
	{SLI4_WQE_ABORT, OCS_Q_HIST_ABORT_WQE_WORD_MASK},
	{SLI4_WQE_FCP_IREAD64, OCS_Q_HIST_IREAD_WQE_WORD_MASK},
	{SLI4_WQE_FCP_IWRITE64, OCS_Q_HIST_IWRITE_WQE_WORD_MASK},
	{SLI4_WQE_FCP_CONT_TRECEIVE64, OCS_Q_HIST_TRECV_CONT_WQE_WORD_MASK},
};

/* CQE masks */
ocs_q_hist_cqe_mask_t ocs_q_hist_cqe_masks[] = {
	/* CQE type     Q_hist_type		mask (success) 	mask (non-success) */
	{SLI_QENTRY_WQ, OCS_Q_HIST_TYPE_CWQE, 	OCS_Q_HIST_WCQE_WORD_MASK, OCS_Q_HIST_WCQE_WORD_MASK_ERR},
	{SLI_QENTRY_XABT, OCS_Q_HIST_TYPE_CXABT, OCS_Q_HIST_CQXABT_WORD_MASK, OCS_Q_HIST_WCQE_WORD_MASK},
};

static uint32_t ocs_q_hist_get_wqe_mask(sli4_generic_wqe_t *wqe)
{
	uint32_t i;
	for (i = 0; i < ARRAY_SIZE(ocs_q_hist_wqe_masks); i++) {
		if (ocs_q_hist_wqe_masks[i].command == wqe->command) {
			return ocs_q_hist_wqe_masks[i].mask;
		}
	}
	/* return default WQE mask */
	return OCS_Q_HIST_WQE_WORD_MASK_DEFAULT;
}

/**
 * @ingroup debug
 * @brief Initialize resources for queue history
 *
 * @param os os handle
 * @param q_hist Pointer to the queue history object.
 *
 * @return none
 */
void
ocs_queue_history_init(ocs_t *ocs, ocs_hal_q_hist_t *q_hist)
{
	q_hist->ocs = ocs;
	if (q_hist->q_hist != NULL) {
		/* Setup is already done */
		ocs_log_debug(ocs, "q_hist not NULL, skipping init\n");
		return;
	}

	q_hist->q_hist = ocs_malloc(ocs, sizeof(*q_hist->q_hist) * OCS_Q_HIST_SIZE, OCS_M_ZERO | OCS_M_NONUMA);
	if (q_hist->q_hist == NULL) {
		ocs_log_err(ocs, "Could not allocate queue history buffer\n");
	} else {
		ocs_lock_init(ocs, &q_hist->q_hist_lock, "queue history lock[%d]", ocs_instance(ocs));
	}

	q_hist->q_hist_index = 0;
}

/**
 * @ingroup debug
 * @brief Free resources for queue history
 *
 * @param q_hist Pointer to the queue history object.
 *
 * @return none
 */
void
ocs_queue_history_free(ocs_hal_q_hist_t *q_hist)
{
	ocs_t *ocs = q_hist->ocs;

	if (q_hist->q_hist != NULL) {
		ocs_free(ocs, q_hist->q_hist, sizeof(*q_hist->q_hist)*OCS_Q_HIST_SIZE);
		ocs_lock_free(&q_hist->q_hist_lock);
		q_hist->q_hist = NULL;
	}
}

static void
ocs_queue_history_add_q_info(ocs_hal_q_hist_t *q_hist, uint32_t qid, uint32_t qindex)
{
	if (ocs_queue_history_q_info_enabled()) {
		/* write qid, index */
		q_hist->q_hist[q_hist->q_hist_index] = (qid << 16) | qindex;
		q_hist->q_hist_index++;
		q_hist->q_hist_index = q_hist->q_hist_index % OCS_Q_HIST_SIZE;
	}
}

static void
ocs_queue_history_add_timestamp(ocs_hal_q_hist_t *q_hist)
{
	if (ocs_queue_history_timestamp_enabled()) {
		/* write tsc */
		uint64_t tsc_value;
		tsc_value = ocs_get_tsc();
		q_hist->q_hist[q_hist->q_hist_index] = ((tsc_value >> 32 ) & 0xFFFFFFFF);
		q_hist->q_hist_index++;
		q_hist->q_hist_index = q_hist->q_hist_index % OCS_Q_HIST_SIZE;
		q_hist->q_hist[q_hist->q_hist_index] = (tsc_value & 0xFFFFFFFF);
		q_hist->q_hist_index++;
		q_hist->q_hist_index = q_hist->q_hist_index % OCS_Q_HIST_SIZE;
	}
}

/**
 * @ingroup debug
 * @brief Log work queue entry (WQE) into history array
 *
 * @param q_hist Pointer to the queue history object.
 * @param entryw Work queue entry in words
 * @param qid Queue ID
 * @param qindex Queue index
 *
 * @return none
 */
void
ocs_queue_history_wq(ocs_hal_q_hist_t *q_hist, uint32_t *entryw, uint32_t qid, uint32_t qindex)
{
	int i;
	ocs_q_hist_ftr_t ftr;
	uint32_t wqe_word_mask = ocs_q_hist_get_wqe_mask((sli4_generic_wqe_t *)entryw);

	if (q_hist->q_hist == NULL) {
		/* Can't save anything */
		return;
	}

	ftr.word = 0;
	ftr.s.type = OCS_Q_HIST_TYPE_WQE;
	ocs_lock(&q_hist->q_hist_lock);
		/* Capture words in reverse order since we'll be interpretting them LIFO */
		for (i = ((sizeof(wqe_word_mask)*8) - 1); i >= 0; i--){
			if ((wqe_word_mask >> i) & 1) {
				q_hist->q_hist[q_hist->q_hist_index] = entryw[i];
				q_hist->q_hist_index++;
				q_hist->q_hist_index = q_hist->q_hist_index % OCS_Q_HIST_SIZE;
			}
		}

		ocs_queue_history_add_q_info(q_hist, qid, qindex);
		ocs_queue_history_add_timestamp(q_hist);

		/* write footer */
		if (wqe_word_mask) {
			ftr.s.mask = wqe_word_mask;
			q_hist->q_hist[q_hist->q_hist_index] = ftr.word;
			q_hist->q_hist_index++;
			q_hist->q_hist_index = q_hist->q_hist_index % OCS_Q_HIST_SIZE;
		}

	ocs_unlock(&q_hist->q_hist_lock);
}

/**
 * @ingroup debug
 * @brief Log misc words
 *
 * @param q_hist Pointer to the queue history object.
 * @param entryw array of words
 * @param num_words number of words in entryw
 *
 * @return none
 */
void
ocs_queue_history_misc(ocs_hal_q_hist_t *q_hist, uint32_t *entryw, uint32_t num_words)
{
	int i;
	ocs_q_hist_ftr_t ftr;
	uint32_t mask = 0;

	if (q_hist->q_hist == NULL) {
		/* Can't save anything */
		return;
	}

	ftr.word = 0;
	ftr.s.type = OCS_Q_HIST_TYPE_MISC;
	ocs_lock(&q_hist->q_hist_lock);
		/* Capture words in reverse order since we'll be interpretting them LIFO */
		for (i = num_words-1; i >= 0; i--) {
			q_hist->q_hist[q_hist->q_hist_index] = entryw[i];
			q_hist->q_hist_index++;
			q_hist->q_hist_index = q_hist->q_hist_index % OCS_Q_HIST_SIZE;
			mask |= BIT(i);
		}

		ocs_queue_history_add_timestamp(q_hist);

		/* write footer */
		if (num_words) {
			ftr.s.mask = mask;
			q_hist->q_hist[q_hist->q_hist_index] = ftr.word;
			q_hist->q_hist_index++;
			q_hist->q_hist_index = q_hist->q_hist_index % OCS_Q_HIST_SIZE;
		}

	ocs_unlock(&q_hist->q_hist_lock);
}

/**
 * @ingroup debug
 * @brief Log work queue completion (CQE) entry into history
 *        array
 *
 * @param q_hist Pointer to the queue history object.
 * @param ctype Type of completion entry
 * @param entryw Completion queue entry in words
 * @param status Completion queue status
 * @param qid Queue ID
 * @param qindex Queue index
 *
 * @return none
 */
void
ocs_queue_history_cqe(ocs_hal_q_hist_t *q_hist, uint8_t ctype, uint32_t *entryw, uint8_t status, uint32_t qid, uint32_t qindex)
{
	int i;
	unsigned j;
	uint32_t cqe_word_mask = 0;
	ocs_q_hist_ftr_t ftr;

	if (q_hist->q_hist == NULL) {
		/* Can't save anything */
		return;
	}

	ftr.word = 0;
	for (j = 0; j < ARRAY_SIZE(ocs_q_hist_cqe_masks); j++) {
		if (ocs_q_hist_cqe_masks[j].ctype == ctype) {
			ftr.s.type = ocs_q_hist_cqe_masks[j].type;
			if (status != 0) {
				cqe_word_mask = ocs_q_hist_cqe_masks[j].mask_err;
			} else {
				cqe_word_mask = ocs_q_hist_cqe_masks[j].mask;
			}
		}
	}
	ocs_lock(&q_hist->q_hist_lock);
		/* Capture words in reverse order since we'll be interpretting them LIFO */
		for (i = ((sizeof(cqe_word_mask)*8) - 1); i >= 0; i--){
			if ((cqe_word_mask >> i) & 1) {
				q_hist->q_hist[q_hist->q_hist_index] = entryw[i];
				q_hist->q_hist_index++;
				q_hist->q_hist_index = q_hist->q_hist_index % OCS_Q_HIST_SIZE;
			}
		}
		ocs_queue_history_add_q_info(q_hist, qid, qindex);
		ocs_queue_history_add_timestamp(q_hist);

		/* write footer */
		if (cqe_word_mask) {
			ftr.s.mask = cqe_word_mask;
			q_hist->q_hist[q_hist->q_hist_index] = ftr.word;
			q_hist->q_hist_index++;
			q_hist->q_hist_index = q_hist->q_hist_index % OCS_Q_HIST_SIZE;
		}

	ocs_unlock(&q_hist->q_hist_lock);
}

/**
 * @brief Get previous index
 *
 * @param index Index from which previous index is derived.
 */
uint32_t
ocs_queue_history_prev_index(uint32_t index)
{
	if (index == 0) {
		return OCS_Q_HIST_SIZE - 1;
	} else {
		return index - 1;
	}
}

#endif // OCS_DEBUG_QUEUE_HISTORY

/**
 * @brief Display service parameters
 *
 * <description>
 *
 * @param prelabel leading display label
 * @param reqlabel display label
 * @param dest destination 0=ocs_log, 1=textbuf
 * @param textbuf text buffer destination (if dest==1)
 * @param sparams pointer to service parameter
 *
 * @return none
 */

void
ocs_display_sparams(const char *prelabel, const char *reqlabel, int dest, void *textbuf, void *sparams)
{
	char label[64];

	if (sparams == NULL) {
		return;
	}

	switch(dest) {
	case 0:
		if (prelabel != NULL) {
			ocs_snprintf(label, sizeof(label), "[%s] sparam: %s", prelabel, reqlabel);
		} else {
			ocs_snprintf(label, sizeof(label), "sparam: %s", reqlabel);
		}

		ocs_dump32(OCS_DEBUG_ENABLE_SPARAM_DUMP, NULL, label, sparams, sizeof(fc_plogi_payload_t));
		break;
	case 1:
		ocs_ddump_buffer((ocs_textbuf_t*) textbuf, reqlabel, 0, sparams, sizeof(fc_plogi_payload_t));
		break;
	}
}

