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
 * This file contains all HAL runtime workaround code.  Based on the asic type,
 * asic revision, and range of fw revisions, a particular workaround may be enabled.
 *
 * A workaround may consist of overriding a particular HAL/SLI4 value that was initialized
 * during ocs_hal_setup() (for example the MAX_QUEUE overrides for mis-reported queue
 * sizes). Or if required, elements of the ocs_hal_workaround_t structure may be set to
 * control specific runtime behavior.
 *
 * It is intended that the controls in ocs_hal_workaround_t be defined functionally.  So we
 * would have the driver look like:  "if (hal->workaround.enable_xxx) then ...", rather than
 * what we might previously see as "if this is a BE3, then do xxx"
 *
 */

#include "ocs_hal.h"

#define HAL_FWREV_ZERO		(0ull)
#define HAL_FWREV_MAX		(~0ull)

/**
 * @brief Internal definition of workarounds
 */

typedef enum {
	HAL_WORKAROUND_TEST = 1,
	HAL_WORKAROUND_MAX_QUEUE,	/**< Limits all queues */
	HAL_WORKAROUND_MAX_RQ,		/**< Limits only the RQ */
	HAL_WORKAROUND_RETAIN_TSEND_IO_LENGTH,
	HAL_WORKAROUND_WQE_COUNT_METHOD,
	HAL_WORKAROUND_RQE_COUNT_METHOD,
	HAL_WORKAROUND_USE_UNREGISTERD_RPI,
	HAL_WORKAROUND_DISABLE_AR_TGT_DIF, /**< Disable of auto-response target DIF */
	HAL_WORKAROUND_DISABLE_SET_DUMP_LOC,
	HAL_WORKAROUND_USE_DIF_QUARANTINE,
	HAL_WORKAROUND_USE_DIF_SEC_XRI,		/**< Use secondary xri for multiple data phases */
	HAL_WORKAROUND_OVERRIDE_FCFI_IN_SRB,	/**< FCFI reported in SRB not correct, use "first" registered domain */
	HAL_WORKAROUND_FW_VERSION_TOO_LOW,	/**< The FW version is not the min version supported by this driver */
	HAL_WORKAROUND_SGLC_MISREPORTED,	/**< Chip supports SGL Chaining but SGLC is not set in SLI4_PARAMS */
} hal_workaround_e;

/**
 * @brief Internal workaround structure instance
 */

typedef struct {
	sli4_asic_type_e asic_type;
	sli4_asic_rev_e asic_rev;
	uint64_t fwrev_low;
	uint64_t fwrev_high;

	hal_workaround_e workaround;
	uint32_t value;
} hal_workaround_t;

static hal_workaround_t hal_workarounds[] = {
	{SLI4_ASIC_TYPE_ANY,	SLI4_ASIC_REV_ANY, HAL_FWREV_ZERO, HAL_FWREV_MAX,
		HAL_WORKAROUND_TEST, 999},

	/* Bug: 127585: if_type == 2 returns 0 for total length placed on
	 * FCP_TSEND64_WQE completions.   Note, original driver code enables this
	 * workaround for all asic types
	 */
	{SLI4_ASIC_TYPE_ANY,	SLI4_ASIC_REV_ANY, HAL_FWREV_ZERO, HAL_FWREV_MAX,
		HAL_WORKAROUND_RETAIN_TSEND_IO_LENGTH, 0},

	/* Bug: unknown, Lancer A0 has mis-reported max queue depth */
	{SLI4_ASIC_TYPE_LANCER,	SLI4_ASIC_REV_A0, HAL_FWREV_ZERO, HAL_FWREV_MAX,
		HAL_WORKAROUND_MAX_QUEUE, 2048},

	/* Bug: 143399, BE3 has mis-reported max RQ queue depth */
	{SLI4_ASIC_TYPE_BE3,	SLI4_ASIC_REV_ANY, HAL_FWREV_ZERO, HAL_FWREV(4,6,293,0),
		HAL_WORKAROUND_MAX_RQ, 2048},

	/* Bug: 143399, skyhawk has mis-reported max RQ queue depth */
	{SLI4_ASIC_TYPE_SKYHAWK, SLI4_ASIC_REV_ANY, HAL_FWREV_ZERO, HAL_FWREV(10,0,594,0),
		HAL_WORKAROUND_MAX_RQ, 2048},

	/* Bug: 103487, BE3 before f/w 4.2.314.0 has mis-reported WQE count method */
	{SLI4_ASIC_TYPE_BE3,	SLI4_ASIC_REV_ANY, HAL_FWREV_ZERO, HAL_FWREV(4,2,314,0),
		HAL_WORKAROUND_WQE_COUNT_METHOD, 1},

	/* Bug: 103487, BE3 before f/w 4.2.314.0 has mis-reported RQE count method */
	{SLI4_ASIC_TYPE_BE3,	SLI4_ASIC_REV_ANY, HAL_FWREV_ZERO, HAL_FWREV(4,2,314,0),
		HAL_WORKAROUND_RQE_COUNT_METHOD, 1},

	/* Bug: 142968, BE3 UE with RPI == 0xffff */
	{SLI4_ASIC_TYPE_BE3,	SLI4_ASIC_REV_ANY, HAL_FWREV_ZERO, HAL_FWREV_MAX,
		HAL_WORKAROUND_USE_UNREGISTERD_RPI, 0},

	/* Bug: unknown, Skyhawk won't support auto-response on target T10-PI  */
	{SLI4_ASIC_TYPE_SKYHAWK, SLI4_ASIC_REV_ANY, HAL_FWREV_ZERO, HAL_FWREV_MAX,
		HAL_WORKAROUND_DISABLE_AR_TGT_DIF, 0},

	{SLI4_ASIC_TYPE_LANCER,	SLI4_ASIC_REV_ANY, HAL_FWREV_ZERO, HAL_FWREV(1,1,65,0),
		HAL_WORKAROUND_DISABLE_SET_DUMP_LOC, 0},

	/* Bug: 160124, Skyhawk quarantine DIF XRIs  */
	{SLI4_ASIC_TYPE_SKYHAWK, SLI4_ASIC_REV_ANY, HAL_FWREV_ZERO, HAL_FWREV_MAX,
		HAL_WORKAROUND_USE_DIF_QUARANTINE, 0},

	/* Bug: 161832, Skyhawk use secondary XRI for multiple data phase TRECV */
	{SLI4_ASIC_TYPE_SKYHAWK, SLI4_ASIC_REV_ANY, HAL_FWREV_ZERO, HAL_FWREV_MAX,
		HAL_WORKAROUND_USE_DIF_SEC_XRI, 0},

	/* Bug: xxxxxx, FCFI reported in SRB not corrrect */
	{SLI4_ASIC_TYPE_LANCER, SLI4_ASIC_REV_ANY, HAL_FWREV_ZERO, HAL_FWREV_MAX,
		HAL_WORKAROUND_OVERRIDE_FCFI_IN_SRB, 0},

	/* Bug: 165642, FW version check for driver */
	{SLI4_ASIC_TYPE_LANCER, SLI4_ASIC_REV_ANY, HAL_FWREV_ZERO, HAL_FWREV_1(OCS_MIN_FW_VER_LANCER),
		HAL_WORKAROUND_FW_VERSION_TOO_LOW, 0},

	{SLI4_ASIC_TYPE_SKYHAWK, SLI4_ASIC_REV_ANY, HAL_FWREV_ZERO, HAL_FWREV_1(OCS_MIN_FW_VER_SKYHAWK),
		HAL_WORKAROUND_FW_VERSION_TOO_LOW, 0},

	/* Bug 177061, Lancer FW does not set the SGLC bit */
	{SLI4_ASIC_TYPE_LANCER, SLI4_ASIC_REV_ANY, HAL_FWREV_ZERO, HAL_FWREV_MAX,
		HAL_WORKAROUND_SGLC_MISREPORTED, 0},
};

/**
 * @brief Function prototypes
 */

static int32_t ocs_hal_workaround_match(ocs_hal_t *hal, hal_workaround_t *w);

/**
 * @brief Parse the firmware version (name)
 *
 * Parse a string of the form a.b.c.d, returning a uint64_t packed as defined
 * by the HAL_FWREV() macro
 *
 * @param fwrev_string pointer to the firmware string
 *
 * @return packed firmware revision value
 */

static uint64_t
parse_fw_version(const char *fwrev_string)
{
	int v[4] = {0};
	const char *p;
	int i;

	for (p = fwrev_string, i = 0; *p && (i < 4); i ++) {
		v[i] = ocs_strtoul(p, 0, 0);
		while(*p && *p != '.') {
			p ++;
		}
		if (*p) {
			p ++;
		}
	}

	/* Special case for bootleg releases with f/w rev 0.0.9999.0, set to max value */
	if (v[2] == 9999) {
		return HAL_FWREV_MAX;
	} else {
		return HAL_FWREV(v[0], v[1], v[2], v[3]);
	}
}

/**
 * @brief Test for a workaround match
 *
 * Looks at the asic type, asic revision, and fw revision, and returns TRUE if match.
 *
 * @param hal Pointer to the HAL structure
 * @param w Pointer to a workaround structure entry
 *
 * @return Return TRUE for a match
 */

static int32_t
ocs_hal_workaround_match(ocs_hal_t *hal, hal_workaround_t *w)
{
	return (((w->asic_type == SLI4_ASIC_TYPE_ANY) || (w->asic_type == hal->sli.asic_type)) &&
		    ((w->asic_rev == SLI4_ASIC_REV_ANY) || (w->asic_rev == hal->sli.asic_rev)) &&
		    (w->fwrev_low <= hal->workaround.fwrev) &&
		    ((w->fwrev_high == HAL_FWREV_MAX) || (hal->workaround.fwrev < w->fwrev_high)));
}

/**
 * @brief Setup HAL runtime workarounds
 *
 * The function is called at the end of ocs_hal_setup() to setup any runtime workarounds
 * based on the HAL/SLI setup.
 *
 * @param hal Pointer to HAL structure
 *
 * @return none
 */

void
ocs_hal_workaround_setup(struct ocs_hal_s *hal)
{
	hal_workaround_t *w;
	sli4_t *sli4 = &hal->sli;
	uint32_t i;

	/* Initialize the workaround settings */
	ocs_memset(&hal->workaround, 0, sizeof(hal->workaround));

	/* If hal_war_version is non-null, then its a value that was set by a module parameter
	 * (sorry for the break in abstraction, but workarounds are ... well, workarounds)
	 */

	if (hal->hal_war_version) {
		hal->workaround.fwrev = parse_fw_version(hal->hal_war_version);
	} else {
		hal->workaround.fwrev = parse_fw_version((char*) sli4->config.fw_name[0]);
	}

	/* Walk the workaround list, if a match is found, then handle it */
	for (i = 0, w = hal_workarounds; i < ARRAY_SIZE(hal_workarounds); i++, w++) {
		if (ocs_hal_workaround_match(hal, w)) {
			switch(w->workaround) {

			case HAL_WORKAROUND_TEST: {
				ocs_log_debug(hal->os, "Override: test: %d\n", w->value);
				break;
			}

			case HAL_WORKAROUND_RETAIN_TSEND_IO_LENGTH: {
				ocs_log_debug(hal->os, "HAL Workaround: retain TSEND IO length\n");
				hal->workaround.retain_tsend_io_length = 1;
				break;
			}
			case HAL_WORKAROUND_MAX_QUEUE: {
				sli4_qtype_e q;

				ocs_log_debug(hal->os, "HAL Workaround: override max_qentries: %d\n", w->value);
				for (q = SLI_QTYPE_EQ; q < SLI_QTYPE_MAX; q++) {
					if (hal->num_qentries[q] > w->value) {
						hal->num_qentries[q] = w->value;
					}
				}
				break;
			}
			case HAL_WORKAROUND_MAX_RQ: {
				ocs_log_debug(hal->os, "HAL Workaround: override RQ max_qentries: %d\n", w->value);
				if (hal->num_qentries[SLI_QTYPE_RQ] > w->value) {
					hal->num_qentries[SLI_QTYPE_RQ] = w->value;
				}
				break;
			}
			case HAL_WORKAROUND_WQE_COUNT_METHOD: {
				ocs_log_debug(hal->os, "HAL Workaround: set WQE count method=%d\n", w->value);
				sli4->config.count_method[SLI_QTYPE_WQ] = w->value;
				sli_calc_max_qentries(sli4);
				break;
			}
			case HAL_WORKAROUND_RQE_COUNT_METHOD: {
				ocs_log_debug(hal->os, "HAL Workaround: set RQE count method=%d\n", w->value);
				sli4->config.count_method[SLI_QTYPE_RQ] = w->value;
				sli_calc_max_qentries(sli4);
				break;
			}
			case HAL_WORKAROUND_USE_UNREGISTERD_RPI:
				ocs_log_debug(hal->os, "HAL Workaround: use unreg'd RPI if rnode->indicator == 0xFFFF\n");
				hal->workaround.use_unregistered_rpi = TRUE;
				/*
				 * Allocate an RPI that is never registered, to be used in the case where
				 * a node has been unregistered, and its indicator (RPI) value is set to 0xFFFF
				 */
				if (sli_resource_alloc(&hal->sli, SLI_RSRC_FCOE_RPI, &hal->workaround.unregistered_rid,
					&hal->workaround.unregistered_index)) {
					ocs_log_err(hal->os, "sli_resource_alloc unregistered RPI failed\n");
					hal->workaround.use_unregistered_rpi = FALSE;
				}
				break;
			case HAL_WORKAROUND_DISABLE_AR_TGT_DIF:
				ocs_log_debug(hal->os, "HAL Workaround: disable AR on T10-PI TSEND\n");
				hal->workaround.disable_ar_tgt_dif = TRUE;
				break;
			case HAL_WORKAROUND_DISABLE_SET_DUMP_LOC:
				ocs_log_debug(hal->os, "HAL Workaround: disable set_dump_loc\n");
				hal->workaround.disable_dump_loc = TRUE;
				break;
			case HAL_WORKAROUND_USE_DIF_QUARANTINE:
				ocs_log_debug(hal->os, "HAL Workaround: use DIF quarantine\n");
				hal->workaround.use_dif_quarantine = TRUE;
				break;
			case HAL_WORKAROUND_USE_DIF_SEC_XRI:
				ocs_log_debug(hal->os, "HAL Workaround: use DIF secondary xri\n");
				hal->workaround.use_dif_sec_xri = TRUE;
				break;
			case HAL_WORKAROUND_OVERRIDE_FCFI_IN_SRB:
				ocs_log_debug(hal->os, "HAL Workaround: override FCFI in SRB\n");
				hal->workaround.override_fcfi = TRUE;
				break;

			case HAL_WORKAROUND_FW_VERSION_TOO_LOW:
				ocs_log_debug(hal->os, "HAL Workaround: fw version is below the minimum for this driver\n");
				hal->workaround.fw_version_too_low = TRUE;
				break;
			case HAL_WORKAROUND_SGLC_MISREPORTED:
				ocs_log_debug(hal->os, "HAL Workaround: SGLC misreported - chaining is enabled\n");
				hal->workaround.sglc_misreported = TRUE;
				break;
			} /* switch(w->workaround) */
		}
	}
}
