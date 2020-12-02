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
 * @defgroup sli SLI-4 Base APIs
 */

/**
 * @file
 * All common (i.e. transport-independent) SLI-4 functions are implemented
 * in this file.
 */

#include "sli4.h"
#include "sli4_fc.h"
#include "ocs.h"
#if defined(OCS_INCLUDE_DEBUG)
#include "ocs_debug.h"
#endif

#define SLI4_BMBX_DELAY_US 1000 /* 1 ms */
#define SLI4_INIT_PORT_DELAY_US 10000 /* 10 ms */

static int32_t sli_sriov_setup(sli4_t *sli4);
static int32_t sli_fw_init(sli4_t *);
static int32_t sli_fw_term(sli4_t *);
static int32_t sli_sliport_control(sli4_t *sli4, uint32_t endian);
static int32_t sli_cmd_fw_deinitialize(sli4_t *, void *, size_t);
static int32_t sli_cmd_fw_initialize(sli4_t *, void *, size_t);
static int32_t sli_queue_doorbell(sli4_t *, sli4_queue_t *);
static uint8_t sli_queue_entry_is_valid(sli4_queue_t *, uint8_t *, uint8_t);

const uint8_t sli4_fw_initialize[] = {
	0xff, 0x12, 0x34, 0xff,
	0xff, 0x56, 0x78, 0xff,
};

const uint8_t sli4_fw_deinitialize[] = {
	0xff, 0xaa, 0xbb, 0xff,
	0xff, 0xcc, 0xdd, 0xff,
};

typedef struct {
	uint32_t family;	/* generation */
	sli4_asic_type_e type;
} sli4_asic_family_t;

sli4_asic_family_t sli4_asic_family_table[] = {
	{0x01, SLI4_ASIC_TYPE_BE3	},
	{0x02, SLI4_ASIC_TYPE_SKYHAWK	},
	{0x04, SLI4_ASIC_TYPE_SKYHAWK	},
	{0x0a, SLI4_ASIC_TYPE_LANCER	},
	{0x0b, SLI4_ASIC_TYPE_LANCER	},
	{0x0c, SLI4_ASIC_TYPE_LANCERG6	},
	{0x0d, SLI4_ASIC_TYPE_LANCERG7	},
	{0x05, SLI4_ASIC_TYPE_CORSAIR	},
};

typedef struct {
	uint32_t rev_id;
	sli4_asic_rev_e rev;
} sli4_asic_revision_t;

sli4_asic_revision_t sli4_asic_rev_table[] = {
	{0x00, SLI4_ASIC_REV_A0},
	{0x01, SLI4_ASIC_REV_A1},
	{0x02, SLI4_ASIC_REV_A2},
	{0x03, SLI4_ASIC_REV_A3},
	{0x10, SLI4_ASIC_REV_B0},
	{0x11, SLI4_ASIC_REV_B1},
	{0x12, SLI4_ASIC_REV_B2},
	{0x13, SLI4_ASIC_REV_B3},
	{0x14, SLI4_ASIC_REV_B4},
	{0x20, SLI4_ASIC_REV_C0},
	{0x21, SLI4_ASIC_REV_C1},
	{0x22, SLI4_ASIC_REV_C2},
	{0x23, SLI4_ASIC_REV_C3},
	{0x24, SLI4_ASIC_REV_C4},
	{0x30, SLI4_ASIC_REV_D0},
};

/*
 * @brief Convert queue type enum (SLI_QTYPE_*) into a string.
 */
const char *SLI_QNAME[] = {
	"Event Queue",
	"Completion Queue",
	"Mailbox Queue",
	"Work Queue",
	"Receive Queue",
	"Undefined"
};

/**
 * @brief Define the mapping of registers to their BAR and offset.
 *
 * @par Description
 * Although SLI-4 specification defines a common set of registers, their locations
 * (both BAR and offset) depend on the interface type. This array maps a register
 * enum to an array of BAR/offset pairs indexed by the interface type. For
 * example, to access the bootstrap mailbox register on an interface type 0
 * device, code can refer to the offset using regmap[SLI4_REG_BMBX][0].offset.
 *
 * @b Note: A value of UINT32_MAX for either the register set (rset) or offset (off)
 * indicates an invalid mapping.
 */
const sli4_reg_t regmap[SLI4_REG_MAX][SLI4_MAX_IF_TYPES] = {
	// SLI4_REG_BMBX
	{
		{ 2, SLI4_BMBX_REG }, { 0, SLI4_BMBX_REG }, { 0, SLI4_BMBX_REG }, { 0, SLI4_BMBX_REG },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { 0, SLI4_BMBX_REG },
	},
	// SLI4_REG_EQ_DOORBELL
	{
		{ 2, SLI4_EQCQ_DOORBELL_REG }, { 0, SLI4_EQCQ_DOORBELL_REG },
		{ 0, SLI4_EQCQ_DOORBELL_REG }, { 0, SLI4_EQCQ_DOORBELL_REG },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX },
		{ 1, SLI4_IF6_EQ_DOORBELL_REG }
	},
	// SLI4_REG_CQ_DOORBELL
	{
		{ 2, SLI4_EQCQ_DOORBELL_REG }, { 0, SLI4_EQCQ_DOORBELL_REG },
		{ 0, SLI4_EQCQ_DOORBELL_REG }, { 0, SLI4_EQCQ_DOORBELL_REG },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX },
		{ 1, SLI4_IF6_CQ_DOORBELL_REG }
	},
	// SLI4_REG_FCOE_RQ_DOORBELL
	{
		{ 2, SLI4_RQ_DOORBELL_REG }, { 0, SLI4_RQ_DOORBELL_REG },
		{ 0, SLI4_RQ_DOORBELL_REG }, { UINT32_MAX, UINT32_MAX },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX },
		{ 1, SLI4_IF6_RQ_DOORBELL_REG }
	},
	// SLI4_REG_IO_WQ_DOORBELL
	{
		{ 2, SLI4_IO_WQ_DOORBELL_REG }, { 0, SLI4_IO_WQ_DOORBELL_REG },
		{ 0, SLI4_IO_WQ_DOORBELL_REG }, { UINT32_MAX, UINT32_MAX },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX },
		{ 1, SLI4_IF6_WQ_DOORBELL_REG }
	},
	// SLI4_REG_MQ_DOORBELL
	{
		{ 2, SLI4_MQ_DOORBELL_REG }, { 0, SLI4_MQ_DOORBELL_REG },
		{ 0, SLI4_MQ_DOORBELL_REG }, { 0, SLI4_MQ_DOORBELL_REG },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX },
		{ 1, SLI4_IF6_MQ_DOORBELL_REG }
	},
	// SLI4_REG_PHYSDEV_CONTROL
	{
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { 0, SLI4_PHSDEV_CONTROL_REG_236 }, { 0, SLI4_PHSDEV_CONTROL_REG_236 },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { 0, SLI4_PHSDEV_CONTROL_REG_236 },
	},
	// SLI4_REG_SLIPORT_CONTROL
	{
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { 0, SLI4_SLIPORT_CONTROL_REG }, { UINT32_MAX, UINT32_MAX },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { 0, SLI4_SLIPORT_CONTROL_REG },
	},
	// SLI4_REG_SLIPORT_ERROR1
	{
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { 0, SLI4_SLIPORT_ERROR1 }, { UINT32_MAX, UINT32_MAX },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { 0, SLI4_SLIPORT_ERROR1 },
	},
	// SLI4_REG_SLIPORT_ERROR2
	{
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { 0, SLI4_SLIPORT_ERROR2 }, { UINT32_MAX, UINT32_MAX },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { 0, SLI4_SLIPORT_ERROR2 },
	},
	// SLI4_REG_SLIPORT_SEMAPHORE
	{
		{ 1, SLI4_PORT_SEMAPHORE_REG_0 },  { 0, SLI4_PORT_SEMAPHORE_REG_1 },
		{ 0, SLI4_PORT_SEMAPHORE_REG_236 }, { 0, SLI4_PORT_SEMAPHORE_REG_236 },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX },
		{ 0, SLI4_PORT_SEMAPHORE_REG_236 },
	},
	// SLI4_REG_SLIPORT_STATUS
	{
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { 0, SLI4_PORT_STATUS_REG_236 }, { 0, SLI4_PORT_STATUS_REG_236 },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { 0, SLI4_PORT_STATUS_REG_236 },
	},
	// SLI4_REG_UERR_MASK_HI
	{
		{ 0, SLI4_UERR_MASK_HIGH_REG }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }
	},
	// SLI4_REG_UERR_MASK_LO
	{
		{ 0, SLI4_UERR_MASK_LOW_REG }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }
	},
	// SLI4_REG_UERR_STATUS_HI
	{
		{ 0, SLI4_UERR_STATUS_HIGH_REG }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }
	},
	// SLI4_REG_UERR_STATUS_LO
	{
		{ 0, SLI4_UERR_STATUS_LOW_REG }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }
	},
	// SLI4_REG_SW_UE_CSR1
	{
		{ 1, SLI4_SW_UE_CSR1}, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }
	},
	// SLI4_REG_SW_UE_CSR2
	{
		{ 1, SLI4_SW_UE_CSR2}, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX },
		{ UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }, { UINT32_MAX, UINT32_MAX }
	},
};

/**
 * @brief Read the given SLI register.
 *
 * @param sli Pointer to the SLI context.
 * @param reg Register name enum.
 *
 * @return Returns the register value.
 */
uint32_t
sli_reg_read(sli4_t *sli, sli4_regname_e reg)
{
	const sli4_reg_t	*r = &(regmap[reg][sli->if_type]);

	// sanity check
	if ((UINT32_MAX == r->rset) || (UINT32_MAX == r->off)) {
		ocs_log_err(sli->os, "regname %d not defined for if_type %d\n", reg, sli->if_type);
		return UINT32_MAX;
	}

	return ocs_reg_read32(sli->os, r->rset, r->off);
}

/**
 * @brief Write the value to the given SLI register.
 *
 * @param sli Pointer to the SLI context.
 * @param reg Register name enum.
 * @param val Value to write.
 *
 * @return None.
 */
void
sli_reg_write(sli4_t *sli, sli4_regname_e reg, uint32_t val)
{
	const sli4_reg_t	*r = &(regmap[reg][sli->if_type]);

	// sanity check
	if ((UINT32_MAX == r->rset) || (UINT32_MAX == r->off)) {
		ocs_log_err(sli->os, "regname %d not defined for if_type %d\n", reg, sli->if_type);
		return;
	}

	ocs_reg_write32(sli->os, r->rset, r->off, val);
}

/**
 * @brief Check if the SLI_INTF register is valid.
 *
 * @param val 32-bit SLI_INTF register value.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static uint8_t
sli_intf_valid_check(uint32_t val)
{
	return ((val >> SLI4_INTF_VALID_SHIFT) & SLI4_INTF_VALID_MASK) != SLI4_INTF_VALID;
}

/**
 * @brief Retrieve the SLI revision level.
 *
 * @param val 32-bit SLI_INTF register value.
 *
 * @return Returns the SLI revision level.
 */
static uint8_t
sli_intf_sli_revision(uint32_t val)
{
	return ((val >> SLI4_INTF_SLI_REVISION_SHIFT) & SLI4_INTF_SLI_REVISION_MASK);
}

static uint8_t
sli_intf_sli_family(uint32_t val)
{
	return ((val >> SLI4_INTF_SLI_FAMILY_SHIFT) & SLI4_INTF_SLI_FAMILY_MASK);
}

/**
 * @brief Retrieve the SLI interface type.
 *
 * @param val 32-bit SLI_INTF register value.
 *
 * @return Returns the SLI interface type.
 */
static uint8_t
sli_intf_if_type(uint32_t val)
{
	return ((val >> SLI4_INTF_IF_TYPE_SHIFT) & SLI4_INTF_IF_TYPE_MASK);
}

/**
 * @brief Retrieve PCI revision ID.
 *
 * @param val 32-bit PCI CLASS_REVISION register value.
 *
 * @return Returns the PCI revision ID.
 */
static uint8_t
sli_pci_rev_id(uint32_t val)
{
	return ((val >> SLI4_PCI_REV_ID_SHIFT) & SLI4_PCI_REV_ID_MASK);
}

/**
 * @brief retrieve SLI ASIC generation
 *
 * @param val 32-bit SLI_ASIC_ID register value
 *
 * @return SLI ASIC generation
 */
static uint8_t
sli_asic_gen(uint32_t val)
{
	return ((val >> SLI4_ASIC_GEN_SHIFT) & SLI4_ASIC_GEN_MASK);
}

/**
 * @brief Wait for the bootstrap mailbox to report "ready".
 *
 * @param sli4 SLI context pointer.
 * @param msec Number of milliseconds to wait.
 *
 * @return Returns 0 if BMBX is ready, or non-zero otherwise (i.e. time out occurred).
 */
static int32_t
sli_bmbx_wait(sli4_t *sli4, uint32_t msec)
{
	uint32_t	val = 0;

	do {
		ocs_delay_usec(SLI4_BMBX_DELAY_US);	// 1 ms
		val = sli_reg_read(sli4, SLI4_REG_BMBX);
		msec--;
	} while(msec && !(val & SLI4_BMBX_RDY));

	return(!(val & SLI4_BMBX_RDY));
}

/**
 * @brief Write bootstrap mailbox.
 *
 * @param sli4 SLI context pointer.
 *
 * @return Returns 0 if command succeeded, or non-zero otherwise.
 */
static int32_t
sli_bmbx_write(sli4_t *sli4)
{
	uint32_t	val = 0;

	// write buffer location to bootstrap mailbox register
	ocs_dma_sync(&sli4->bmbx, OCS_DMASYNC_PREWRITE);
	val = SLI4_BMBX_WRITE_HI(sli4->bmbx.phys);
	sli_reg_write(sli4, SLI4_REG_BMBX, val);

	if (sli_bmbx_wait(sli4, SLI4_BMBX_DELAY_US)) {
		ocs_log_crit(sli4->os, "BMBX WRITE_HI failed\n");
		return -1;
	}
	val = SLI4_BMBX_WRITE_LO(sli4->bmbx.phys);
	sli_reg_write(sli4, SLI4_REG_BMBX, val);

	// wait for SLI Port to set ready bit
	return sli_bmbx_wait(sli4, SLI4_BMBX_TIMEOUT_MSEC/*XXX*/);
}

#if defined(OCS_INCLUDE_DEBUG)
/**
 * @ingroup sli
 * @brief Dump BMBX mailbox command.
 *
 * @par Description
 * Convenience function for dumping BMBX mailbox commands. Takes
 * into account which mailbox command is given since SLI_CONFIG
 * commands are special.
 *
 * @b Note: This function takes advantage of
 * the one-command-at-a-time nature of the BMBX to be able to
 * display non-embedded SLI_CONFIG commands. This will not work
 * for mailbox commands on the MQ. Luckily, all current non-emb
 * mailbox commands go through the BMBX.
 *
 * @param sli4 SLI context pointer.
 * @param mbx Pointer to mailbox command to dump.
 * @param prefix Prefix for dump label.
 *
 * @return None.
 */
static void
sli_dump_bmbx_command(sli4_t *sli4, void *mbx, const char *prefix)
{
	uint32_t size = 0;
	char label[64];
	uint32_t i;
	// Mailbox diagnostic logging
	sli4_mbox_command_header_t *hdr = (sli4_mbox_command_header_t *)mbx;

	if (!ocs_debug_is_enabled(OCS_DEBUG_ENABLE_MQ_DUMP)) {
		return;
	}

	if (hdr->command == SLI4_MBOX_COMMAND_SLI_CONFIG) {
		sli4_cmd_sli_config_t *sli_config = (sli4_cmd_sli_config_t *)hdr;
		sli4_req_hdr_t	*sli_config_hdr;
		if (sli_config->emb) {
			ocs_snprintf(label, sizeof(label), "%s (emb)", prefix);

			/*  if embedded, dump entire command */
			sli_config_hdr = (sli4_req_hdr_t *)sli_config->payload.embed;
			size = sizeof(*sli_config) - sizeof(sli_config->payload) +
				sli_config_hdr->request_length + (4*sizeof(uint32_t));
			ocs_dump32(OCS_DEBUG_ENABLE_MQ_DUMP, sli4->os, label,
				   (uint8_t *)sli4->bmbx.virt, size);
		} else {
			sli4_sli_config_pmd_t *pmd;
			ocs_snprintf(label, sizeof(label), "%s (non-emb hdr)", prefix);

			/* if non-embedded, break up into two parts: SLI_CONFIG hdr
			   and the payload(s) */
			size = sizeof(*sli_config) - sizeof(sli_config->payload) + (12 * sli_config->pmd_count);
			ocs_dump32(OCS_DEBUG_ENABLE_MQ_DUMP, sli4->os, label,
				   (uint8_t *)sli4->bmbx.virt, size);

			/* as sanity check, make sure first PMD matches what was saved */
			pmd = &sli_config->payload.mem;
			if ((pmd->address_high == ocs_addr32_hi(sli4->bmbx_non_emb_pmd->phys)) &&
			    (pmd->address_low == ocs_addr32_lo(sli4->bmbx_non_emb_pmd->phys))) {
				for (i = 0; i < sli_config->pmd_count; i++, pmd++) {
					sli_config_hdr = sli4->bmbx_non_emb_pmd->virt;
					ocs_snprintf(label, sizeof(label), "%s (non-emb pay[%d])", prefix, i);
					ocs_dump32(OCS_DEBUG_ENABLE_MQ_DUMP, sli4->os, label,
						   (uint8_t *)sli4->bmbx_non_emb_pmd->virt,
						   sli_config_hdr->request_length + (4*sizeof(uint32_t)));
				}
			} else {
				ocs_log_debug(sli4->os, "pmd addr does not match pmd:%x %x (%x %x)\n",
					pmd->address_high, pmd->address_low,
					ocs_addr32_hi(sli4->bmbx_non_emb_pmd->phys),
					ocs_addr32_lo(sli4->bmbx_non_emb_pmd->phys));
			}

		}
	} else {
		/* not an SLI_CONFIG command, just display first 64 bytes, like we do
		   for MQEs */
		size = 64;
		ocs_dump32(OCS_DEBUG_ENABLE_MQ_DUMP, sli4->os, prefix,
			   (uint8_t *)mbx, size);
	}
}
#endif

/**
 * @ingroup sli
 * @brief Submit a command to the bootstrap mailbox and check the status.
 *
 * @param sli4 SLI context pointer.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_bmbx_command(sli4_t *sli4)
{
	void *cqe = (uint8_t *)sli4->bmbx.virt + SLI4_BMBX_SIZE;

#if defined(OCS_INCLUDE_DEBUG)
	sli_dump_bmbx_command(sli4, sli4->bmbx.virt, "bmbx cmd");
#endif

	if (sli4->reset_pending && sli4->reset_pending(sli4->reset_pending_arg, FALSE))
		return -1;

	if (sli_bmbx_write(sli4)) {
		ocs_log_crit(sli4->os, "bootstrap mailbox write fail phys=%p reg=%#x\n",
			(void*)sli4->bmbx.phys, sli_reg_read(sli4, SLI4_REG_BMBX));
		return -1;
	}

	// check completion queue entry status
	ocs_dma_sync(&sli4->bmbx, OCS_DMASYNC_POSTREAD);
	if (((sli4_mcqe_t *)cqe)->val) {
#if defined(OCS_INCLUDE_DEBUG)
		sli_dump_bmbx_command(sli4, sli4->bmbx.virt, "bmbx cmpl");
		ocs_dump32(OCS_DEBUG_ENABLE_CQ_DUMP, sli4->os, "bmbx cqe", cqe, sizeof(sli4_mcqe_t));
#endif
		return sli_cqe_mq(cqe);
	} else {
		ocs_log_err(sli4->os, "invalid or wrong type\n");
		return -1;
	}
}

/****************************************************************************
 * Messages
 */

/**
 * @ingroup sli
 * @brief Write a CONFIG_LINK command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_config_link(sli4_t *sli4, void *buf, size_t size)
{
	sli4_cmd_config_link_t	*config_link = buf;

	ocs_memset(buf, 0, size);

	config_link->hdr.command = SLI4_MBOX_COMMAND_CONFIG_LINK;

	if (ocs_bbcr_enabled(sli4->os)) {
		config_link->cscn = 1;
		config_link->bbscn = sli4->config.bbscn_def;
	}

	ocs_log_debug(sli4->os, "config_link->bbscn: %d\n", config_link->bbscn);

	/* Port interprets zero in a field as "use default value" */

	return sizeof(sli4_cmd_config_link_t);
}

/**
 * @ingroup sli
 * @brief Write a DOWN_LINK command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_down_link(sli4_t *sli4, void *buf, size_t size)
{
	sli4_mbox_command_header_t	*hdr = buf;

	ocs_memset(buf, 0, size);

	hdr->command = SLI4_MBOX_COMMAND_DOWN_LINK;

	/* Port interprets zero in a field as "use default value" */

	return sizeof(sli4_mbox_command_header_t);
}

/**
 * @ingroup sli
 * @brief Write a DUMP Type 2 command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param region_id BOOT region ID.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_dump_type2(sli4_t *sli4, void *buf, size_t size, uint16_t region_id, ocs_dma_t *dma)
{
	sli4_cmd_dump2_t	*cmd = buf;

	ocs_memset(buf, 0, size);

	cmd->hdr.command = SLI4_MBOX_COMMAND_DUMP;
	cmd->type = 2;
	cmd->region_id = region_id;
	cmd->phys_addr_low = ocs_addr32_lo(dma->phys);
	cmd->phys_addr_high = ocs_addr32_hi(dma->phys);
	cmd->aval_len = size;

	return sizeof(sli4_cmd_dump2_t);
}

/**
 * @ingroup sli
 * @brief Write a DUMP Type 4 command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param wki The well known item ID.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_dump_type4(sli4_t *sli4, void *buf, size_t size, uint16_t wki)
{
	sli4_cmd_dump4_t	*cmd = buf;

	ocs_memset(buf, 0, size);

	cmd->hdr.command = SLI4_MBOX_COMMAND_DUMP;
	cmd->type = 4;
	cmd->wki_selection = wki;
	return sizeof(sli4_cmd_dump4_t);
}

/**
 * @ingroup sli
 * @brief update SLI Port non-volatile configuration data.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param region_id BOOT region ID.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_update_cfg(sli4_t *sli4, void *buf, size_t size, uint16_t region_id, ocs_dma_t *dma)
{
	sli4_cmd_update_cfg_t	*cmd = buf;

	ocs_memset(buf, 0, size);

	cmd->hdr.command = SLI4_MBOX_COMMAND_UPDATE_CFG;
	cmd->req_type = SLI4_REQ_UPDATE_CFG;
	cmd->version = 1;
	cmd->di = 1;
	cmd->region_id = region_id;
	cmd->byte_cnt = size;
	cmd->entry_len = size;
	cmd->bde.bde_type = SLI4_BDE_TYPE_BDE_64;
	cmd->bde.buffer_length = size;
	cmd->bde.u.data.buffer_address_low = ocs_addr32_lo(dma->phys);
	cmd->bde.u.data.buffer_address_high = ocs_addr32_hi(dma->phys);

	return sizeof(sli4_cmd_update_cfg_t);
}

/**
 * @ingroup sli
 * @brief Write a COMMON_READ_TRANSCEIVER_DATA command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param page_num The page of SFP data to retrieve (0xa0 or 0xa2).
 * @param dma DMA structure from which the data will be copied.
 *
 * @note This creates a Version 0 message.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_read_transceiver_data(sli4_t *sli4, void *buf, size_t size, uint32_t page_num,
				     ocs_dma_t *dma)
{
	sli4_req_common_read_transceiver_data_t *req = NULL;
	uint32_t	sli_config_off = 0;
	uint32_t	payload_size;

	if (dma == NULL) {
		/* Payload length must accomodate both request and response */
		payload_size = max(sizeof(sli4_req_common_read_transceiver_data_t),
				   sizeof(sli4_res_common_read_transceiver_data_t));
	} else {
		payload_size = dma->size;
	}

	if (sli4->port_type == SLI4_PORT_TYPE_FC) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size, payload_size, dma);
	}

	if (dma == NULL) {
		req = (sli4_req_common_read_transceiver_data_t *)((uint8_t *)buf + sli_config_off);
	} else {
		req = (sli4_req_common_read_transceiver_data_t *)dma->virt;
		ocs_memset(req, 0, dma->size);
	}

	req->hdr.opcode = SLI4_OPC_COMMON_READ_TRANSCEIVER_DATA;
	req->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
	req->hdr.request_length = payload_size - sizeof(sli4_req_hdr_t);

	req->page_number = page_num;
	req->port = sli4->physical_port;

	return(sli_config_off + sizeof(sli4_req_common_read_transceiver_data_t));
}

/**
 * @ingroup sli
 * @brief Write a READ_LINK_STAT command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param req_ext_counters If TRUE, then the extended counters will be requested.
 * @param clear_overflow_flags If TRUE, then overflow flags will be cleared.
 * @param clear_all_counters If TRUE, the counters will be cleared.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_read_link_stats(sli4_t *sli4, void *buf, size_t size,
			uint8_t req_ext_counters,
			uint8_t clear_overflow_flags,
			uint8_t clear_all_counters)
{
	sli4_cmd_read_link_stats_t	*cmd = buf;

	ocs_memset(buf, 0, size);

	cmd->hdr.command = SLI4_MBOX_COMMAND_READ_LNK_STAT;
	cmd->rec = req_ext_counters;
	cmd->clrc = clear_all_counters;
	cmd->clof = clear_overflow_flags;
	return sizeof(sli4_cmd_read_link_stats_t);
}

/**
 * @ingroup sli
 * @brief Write a READ_STATUS command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param clear_counters If TRUE, the counters will be cleared.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_read_status(sli4_t *sli4, void *buf, size_t size,
			uint8_t clear_counters)
{
	sli4_cmd_read_status_t	*cmd = buf;

	ocs_memset(buf, 0, size);

	cmd->hdr.command = SLI4_MBOX_COMMAND_READ_STATUS;
	cmd->cc = clear_counters;
	return sizeof(sli4_cmd_read_status_t);
}

/**
 * @brief Write a FW_DEINITIALIZE command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 *
 * @return Returns the number of bytes written.
 */
static int32_t
sli_cmd_fw_deinitialize(sli4_t *sli4, void *buf, size_t size)
{

	// TODO assert sizeof(sli4_fw_initialize) <= size
	ocs_memset(buf, 0, size);
	ocs_memcpy(buf, sli4_fw_deinitialize, sizeof(sli4_fw_deinitialize));

	return sizeof(sli4_fw_deinitialize);
}

/**
 * @brief Write a FW_INITIALIZE command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 *
 * @return Returns the number of bytes written.
 */
static int32_t
sli_cmd_fw_initialize(sli4_t *sli4, void *buf, size_t size)
{

	// TODO assert sizeof(sli4_fw_initialize) <= size
	ocs_memset(buf, 0, size);
	ocs_memcpy(buf, sli4_fw_initialize, sizeof(sli4_fw_initialize));

	return sizeof(sli4_fw_initialize);
}

/**
 * @ingroup sli
 * @brief Write an INIT_LINK command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param speed Link speed.
 * @param reset_alpa For native FC, this is the selective reset AL_PA
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_init_link(sli4_t *sli4, void *buf, size_t size, uint32_t speed, uint8_t reset_alpa)
{
	sli4_cmd_init_link_t	*init_link = buf;

	ocs_memset(buf, 0, size);

	init_link->hdr.command = SLI4_MBOX_COMMAND_INIT_LINK;

	/* Most fields only have meaning for FC links */
	if (sli4->config.topology != SLI4_READ_CFG_TOPO_FCOE) {
		init_link->selective_reset_al_pa = reset_alpa;
		init_link->link_flags.loopback = FALSE;

		init_link->link_speed_selection_code = speed;
		switch (speed) {
		case FC_LINK_SPEED_1G:
		case FC_LINK_SPEED_2G:
		case FC_LINK_SPEED_4G:
		case FC_LINK_SPEED_8G:
		case FC_LINK_SPEED_16G:
		case FC_LINK_SPEED_32G:
		case FC_LINK_SPEED_64G:
		case FC_LINK_SPEED_128G:
		case FC_LINK_SPEED_256G:
			init_link->link_flags.fixed_speed = TRUE;
			break;
		case FC_LINK_SPEED_10G:
			ocs_log_test(sli4->os, "unsupported FC speed %d\n", speed);
			return 0;
		}

		switch (sli4->config.topology) {
		case SLI4_READ_CFG_TOPO_FC:
			// Attempt P2P but failover to FC-AL
			init_link->link_flags.enable_topology_failover = TRUE;
			init_link->link_flags.topology = SLI4_INIT_LINK_F_P2P_FAIL_OVER;
			break;
		case SLI4_READ_CFG_TOPO_FC_AL:
			init_link->link_flags.topology = SLI4_INIT_LINK_F_FCAL_ONLY;

			if ((init_link->link_speed_selection_code == FC_LINK_SPEED_16G) ||
			    (init_link->link_speed_selection_code == FC_LINK_SPEED_32G) ||
			    (init_link->link_speed_selection_code >= FC_LINK_SPEED_AUTO_32_16)) {
				ocs_log_err(sli4->os, "unsupported FC-AL speed (speed_code: %d)\n", speed);
				return 0;
			}

			break;
		case SLI4_READ_CFG_TOPO_FC_DA:
			init_link->link_flags.topology = FC_TOPOLOGY_P2P;
			break;
		default:
			ocs_log_err(sli4->os, "unsupported topology %#x\n", sli4->config.topology);
			return 0;
		}

		init_link->link_flags.unfair = FALSE;
		init_link->link_flags.skip_lirp_lilp = FALSE;
		init_link->link_flags.gen_loop_validity_check = FALSE;
		init_link->link_flags.skip_lisa = FALSE;
		init_link->link_flags.select_hightest_al_pa = FALSE;
	}

	return sizeof(sli4_cmd_init_link_t);
}

/**
 * @ingroup sli
 * @brief Write an INIT_VFI command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param vfi VFI
 * @param fcfi FCFI
 * @param vpi VPI (Set to -1 if unused.)
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_init_vfi(sli4_t *sli4, void *buf, size_t size, uint16_t vfi,
		uint16_t fcfi, uint16_t vpi)
{
	sli4_cmd_init_vfi_t	*init_vfi = buf;

	ocs_memset(buf, 0, size);

	init_vfi->hdr.command = SLI4_MBOX_COMMAND_INIT_VFI;

	init_vfi->vfi = vfi;
	init_vfi->fcfi = fcfi;

	/*
	 * If the VPI is valid, initialize it at the same time as
	 * the VFI
	 */
	if (0xffff != vpi) {
		init_vfi->vp  = TRUE;
		init_vfi->vpi = vpi;
	}

	return sizeof(sli4_cmd_init_vfi_t);
}

/**
 * @ingroup sli
 * @brief Write an INIT_VPI command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param vpi VPI allocated.
 * @param vfi VFI associated with this VPI.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_init_vpi(sli4_t *sli4, void *buf, size_t size, uint16_t vpi, uint16_t vfi)
{
	sli4_cmd_init_vpi_t	*init_vpi = buf;

	ocs_memset(buf, 0, size);

	init_vpi->hdr.command = SLI4_MBOX_COMMAND_INIT_VPI;
	init_vpi->vpi = vpi;
	init_vpi->vfi = vfi;

	return sizeof(sli4_cmd_init_vpi_t);
}

/**
 * @ingroup sli
 * @brief Write a POST_XRI command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param xri_base Starting XRI value for range of XRI given to SLI Port.
 * @param xri_count Number of XRIs provided to the SLI Port.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_post_xri(sli4_t *sli4, void *buf, size_t size,  uint16_t xri_base, uint16_t xri_count)
{
	sli4_cmd_post_xri_t	*post_xri = buf;

	ocs_memset(buf, 0, size);

	post_xri->hdr.command = SLI4_MBOX_COMMAND_POST_XRI;
	post_xri->xri_base = xri_base;
	post_xri->xri_count = xri_count;

	if ((!sli_get_auto_xfer_rdy_capable(sli4)) && (!sli_get_tow_capable(sli4))) {
		post_xri->enx = TRUE;
		post_xri->val = TRUE;
	}

	return sizeof(sli4_cmd_post_xri_t);
}

/**
 * @ingroup sli
 * @brief Write a RELEASE_XRI command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param num_xri The number of XRIs to be released.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_release_xri(sli4_t *sli4, void *buf, size_t size,  uint8_t num_xri)
{
	sli4_cmd_release_xri_t	*release_xri = buf;

	ocs_memset(buf, 0, size);

	release_xri->hdr.command = SLI4_MBOX_COMMAND_RELEASE_XRI;
	release_xri->xri_count = num_xri;

	return sizeof(sli4_cmd_release_xri_t);
}

/**
 * @brief Write a READ_CONFIG command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_read_config(sli4_t *sli4, void *buf, size_t size)
{
	sli4_cmd_read_config_t	*read_config = buf;

	ocs_memset(buf, 0, size);

	read_config->hdr.command = SLI4_MBOX_COMMAND_READ_CONFIG;

	return sizeof(sli4_cmd_read_config_t);
}

/**
 * @brief Write a READ_NVPARMS command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_read_nvparms(sli4_t *sli4, void *buf, size_t size)
{
	sli4_cmd_read_nvparms_t	*read_nvparms = buf;

	ocs_memset(buf, 0, size);

	read_nvparms->hdr.command = SLI4_MBOX_COMMAND_READ_NVPARMS;

	return sizeof(sli4_cmd_read_nvparms_t);
}

/**
 * @brief Write a WRITE_NVPARMS command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param wwpn WWPN to write - pointer to array of 8 uint8_t.
 * @param wwnn WWNN to write - pointer to array of 8 uint8_t.
 * @param hard_alpa Hard ALPA to write.
 * @param preferred_d_id  Preferred D_ID to write.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_write_nvparms(sli4_t *sli4, void *buf, size_t size, uint8_t *wwpn, uint8_t *wwnn, uint8_t hard_alpa,
		uint32_t preferred_d_id)
{
	sli4_cmd_write_nvparms_t	*write_nvparms = buf;

	ocs_memset(buf, 0, size);

	write_nvparms->hdr.command = SLI4_MBOX_COMMAND_WRITE_NVPARMS;
	ocs_memcpy(write_nvparms->wwpn, wwpn, 8);
	ocs_memcpy(write_nvparms->wwnn, wwnn, 8);
	write_nvparms->hard_alpa = hard_alpa;
	write_nvparms->preferred_d_id = preferred_d_id;

	return sizeof(sli4_cmd_write_nvparms_t);
}

/**
 * @brief Write a READ_REV command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param vpd Pointer to the buffer.
 *
 * @return Returns the number of bytes written.
 */
static int32_t
sli_cmd_read_rev(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *vpd)
{
	sli4_cmd_read_rev_t	*read_rev = buf;

	ocs_memset(buf, 0, size);

	read_rev->hdr.command = SLI4_MBOX_COMMAND_READ_REV;

	if (vpd && vpd->size) {
		read_rev->vpd = TRUE;

		read_rev->available_length = vpd->size;

		read_rev->physical_address_low  = ocs_addr32_lo(vpd->phys);
		read_rev->physical_address_high = ocs_addr32_hi(vpd->phys);
	}

	return sizeof(sli4_cmd_read_rev_t);
}

/**
 * @ingroup sli
 * @brief Write a READ_SPARM64 command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param dma DMA buffer for the service parameters.
 * @param vpi VPI used to determine the WWN.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_read_sparm64(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *dma,
		uint16_t vpi)
{
	sli4_cmd_read_sparm64_t	*read_sparm64 = buf;

	ocs_memset(buf, 0, size);

	if (SLI4_READ_SPARM64_VPI_SPECIAL == vpi) {
		ocs_log_test(sli4->os, "special VPI not supported!!!\n");
		return -1;
	}

	if (!dma || !dma->phys) {
		ocs_log_test(sli4->os, "bad DMA buffer\n");
		return -1;
	}

	read_sparm64->hdr.command = SLI4_MBOX_COMMAND_READ_SPARM64;

	read_sparm64->bde_64.bde_type = SLI4_BDE_TYPE_BDE_64;
	read_sparm64->bde_64.buffer_length = dma->size;
	read_sparm64->bde_64.u.data.buffer_address_low  = ocs_addr32_lo(dma->phys);
	read_sparm64->bde_64.u.data.buffer_address_high = ocs_addr32_hi(dma->phys);

	read_sparm64->vpi = vpi;

	return sizeof(sli4_cmd_read_sparm64_t);
}

/**
 * @ingroup sli
 * @brief Write a READ_TOPOLOGY command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param dma DMA buffer for loop map (optional).
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_read_topology(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *dma)
{
	sli4_cmd_read_topology_t *read_topo = buf;

	ocs_memset(buf, 0, size);

	read_topo->hdr.command = SLI4_MBOX_COMMAND_READ_TOPOLOGY;

	if (dma && dma->size) {
		if (dma->size < SLI4_MIN_LOOP_MAP_BYTES) {
			ocs_log_test(sli4->os, "loop map buffer too small %jd\n", dma->size);
			return 0;
		}

		ocs_memset(dma->virt, 0, dma->size);

		read_topo->bde_loop_map.bde_type = SLI4_BDE_TYPE_BDE_64;
		read_topo->bde_loop_map.buffer_length = dma->size;
		read_topo->bde_loop_map.u.data.buffer_address_low  = ocs_addr32_lo(dma->phys);
		read_topo->bde_loop_map.u.data.buffer_address_high = ocs_addr32_hi(dma->phys);
	}

	return sizeof(sli4_cmd_read_topology_t);
}

/**
 * @ingroup sli
 * @brief Write a REG_FCFI command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param index FCF index returned by READ_FCF_TABLE.
 * @param rq_cfg RQ_ID/R_CTL/TYPE routing information
 * @param vlan_id VLAN ID tag.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_reg_fcfi(sli4_t *sli4, void *buf, size_t size, uint16_t index, sli4_cmd_rq_cfg_t rq_cfg[SLI4_CMD_REG_FCFI_NUM_RQ_CFG], uint16_t vlan_id)
{
	sli4_cmd_reg_fcfi_t	*reg_fcfi = buf;
	uint32_t		i;

	ocs_memset(buf, 0, size);

	reg_fcfi->hdr.command = SLI4_MBOX_COMMAND_REG_FCFI;

	reg_fcfi->fcf_index = index;

	for (i = 0; i < SLI4_CMD_REG_FCFI_NUM_RQ_CFG; i++) {
		switch(i) {
		case 0:	reg_fcfi->rq_id_0 = rq_cfg[0].rq_id; break;
		case 1:	reg_fcfi->rq_id_1 = rq_cfg[1].rq_id; break;
		case 2:	reg_fcfi->rq_id_2 = rq_cfg[2].rq_id; break;
		case 3:	reg_fcfi->rq_id_3 = rq_cfg[3].rq_id; break;
		}
		reg_fcfi->rq_cfg[i].r_ctl_mask = rq_cfg[i].r_ctl_mask;
		reg_fcfi->rq_cfg[i].r_ctl_match = rq_cfg[i].r_ctl_match;
		reg_fcfi->rq_cfg[i].type_mask = rq_cfg[i].type_mask;
		reg_fcfi->rq_cfg[i].type_match = rq_cfg[i].type_match;
	}

	if (vlan_id) {
		reg_fcfi->vv = TRUE;
		reg_fcfi->vlan_tag = vlan_id;
	}

	return sizeof(sli4_cmd_reg_fcfi_t);
}

/**
 * @brief Write REG_FCFI_MRQ to provided command buffer
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param fcf_index FCF index returned by READ_FCF_TABLE.
 * @param vlan_id VLAN ID tag.
 * @param rq_cfg RQ_ID/R_CTL/TYPE routing information
 * @param rq_cfg_cnt Number of rq_cfg's
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
sli_cmd_reg_fcfi_mrq(sli4_t *sli4, void *buf, size_t size, uint8_t mode,
		     uint16_t fcf_index, uint16_t vlan_id,
		     sli4_cmd_rq_cfg_t rq_cfg[SLI4_CMD_REG_FCFI_NUM_RQ_CFG], int32_t rq_cfg_cnt)
{
	sli4_cmd_reg_fcfi_mrq_t	*reg_fcfi_mrq = buf;
	uint32_t i;

	ocs_memset(buf, 0, size);

	reg_fcfi_mrq->hdr.command = SLI4_MBOX_COMMAND_REG_FCFI_MRQ;
	reg_fcfi_mrq->mode = mode;
	if (mode == SLI4_CMD_REG_FCFI_SET_FCFI_MODE) {
		reg_fcfi_mrq->fcf_index = fcf_index;
		if (vlan_id) {
			reg_fcfi_mrq->vv = TRUE;
			reg_fcfi_mrq->vlan_tag = vlan_id;
		}
		goto done;
	}

	if (rq_cfg_cnt > 4)
		reg_fcfi_mrq->xmv = true;

	for (i = 0; i < SLI4_CMD_REG_FCFI_NUM_RQ_CFG; i++) {
		switch (i) {
		case 7:	reg_fcfi_mrq->rq_id_7 = rq_cfg[i].rq_id; break;
		case 6:	reg_fcfi_mrq->rq_id_6 = rq_cfg[i].rq_id; break;
		case 5:	reg_fcfi_mrq->rq_id_5 = rq_cfg[i].rq_id; break;
		case 4:	reg_fcfi_mrq->rq_id_4 = rq_cfg[i].rq_id; break;
		case 3:	reg_fcfi_mrq->rq_id_3 = rq_cfg[i].rq_id; break;
		case 2:	reg_fcfi_mrq->rq_id_2 = rq_cfg[i].rq_id; break;
		case 1:	reg_fcfi_mrq->rq_id_1 = rq_cfg[i].rq_id; break;
		case 0:	reg_fcfi_mrq->rq_id_0 = rq_cfg[i].rq_id; break;
		}

		if (rq_cfg[i].rq_id == 0xffff) /* Not valid */
			continue;

		if (i < 4) {
			/* First set of filters */
			reg_fcfi_mrq->rq_cfg_1[i].r_ctl_mask = rq_cfg[i].r_ctl_mask;
			reg_fcfi_mrq->rq_cfg_1[i].r_ctl_match = rq_cfg[i].r_ctl_match;
			reg_fcfi_mrq->rq_cfg_1[i].type_mask = rq_cfg[i].type_mask;
			reg_fcfi_mrq->rq_cfg_1[i].type_match = rq_cfg[i].type_match;
		} else {
			reg_fcfi_mrq->rq_cfg_2[i - 4].r_ctl_mask = rq_cfg[i].r_ctl_mask;
			reg_fcfi_mrq->rq_cfg_2[i - 4].r_ctl_match = rq_cfg[i].r_ctl_match;
			reg_fcfi_mrq->rq_cfg_2[i - 4].type_mask = rq_cfg[i].type_mask;
			reg_fcfi_mrq->rq_cfg_2[i - 4].type_match = rq_cfg[i].type_match;
		}

		/* Check if protocol needs to be set */
		if (rq_cfg[i].protocol_valid) {
			reg_fcfi_mrq->ptc |= (1 << i);
			reg_fcfi_mrq->pt  |= ((rq_cfg[i].protocol & 0x1) << i);
		}

		/* Check if this filter points to MRQ */
		if (rq_cfg[i].is_mrq) {
			if (rq_cfg[i].mrq_set_num == SLI4_CMD_REG_FCFI_MRQ_SET_0) {
				reg_fcfi_mrq->num_mrq_pairs_1 = rq_cfg[i].mrq_set_count;

				if (reg_fcfi_mrq->xmv) /* extended filter */
					reg_fcfi_mrq->alt_mrq_filter_bit_mask_1 = rq_cfg[i].filtermask;
				else
					reg_fcfi_mrq->mrq_filter_bitmask_1 = rq_cfg[i].filtermask;

				reg_fcfi_mrq->rq_selection_policy_1 = rq_cfg[i].mrq_policy;
			} else if (rq_cfg[i].mrq_set_num == SLI4_CMD_REG_FCFI_MRQ_SET_1) {
				reg_fcfi_mrq->num_mrq_pairs_2 = rq_cfg[i].mrq_set_count;

				if (reg_fcfi_mrq->xmv)
					reg_fcfi_mrq->alt_mrq_filter_bit_mask_2 = rq_cfg[i].filtermask;
				else
					reg_fcfi_mrq->mrq_filter_bitmask_2 = rq_cfg[i].filtermask;

				reg_fcfi_mrq->rq_selection_policy_2 = rq_cfg[i].mrq_policy;
			} else {
				return 0;
			}
		}
	}

done:
	return sizeof(sli4_cmd_reg_fcfi_mrq_t);
}

/**
 * @ingroup sli
 * @brief Write a REG_RPI command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param nport_id Remote F/N_Port_ID.
 * @param rpi Previously-allocated Remote Port Indicator.
 * @param vpi Previously-allocated Virtual Port Indicator.
 * @param dma DMA buffer that contains the remote port's service parameters.
 * @param update Boolean indicating an update to an existing RPI (TRUE)
 * or a new registration (FALSE).
 * @param hlm Indicating an RPI being used for HLM RPI
 * @param nsler NVMe Sequence Level Error Recovery
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_reg_rpi(sli4_t *sli4, void *buf, size_t size, uint32_t nport_id, uint16_t rpi,
		uint16_t vpi, ocs_dma_t *dma, uint8_t update,
		uint8_t enable_t10_pi, uint8_t hlm, uint8_t nsler)
{
	sli4_cmd_reg_rpi_t *reg_rpi = buf;

	ocs_memset(buf, 0, size);

	reg_rpi->hdr.command = SLI4_MBOX_COMMAND_REG_RPI;

	reg_rpi->rpi = rpi;
	reg_rpi->remote_n_port_id = nport_id;
	reg_rpi->upd = update;
	reg_rpi->etow = enable_t10_pi;
	reg_rpi->hlm = hlm;
	reg_rpi->nsler = nsler;

	if (dma) {
		reg_rpi->bde_64.bde_type = SLI4_BDE_TYPE_BDE_64;
		reg_rpi->bde_64.buffer_length = SLI4_REG_RPI_BUF_LEN;
		reg_rpi->bde_64.u.data.buffer_address_low  = ocs_addr32_lo(dma->phys);
		reg_rpi->bde_64.u.data.buffer_address_high = ocs_addr32_hi(dma->phys);
	}

	reg_rpi->vpi = vpi;

	return sizeof(sli4_cmd_reg_rpi_t);
}

/**
 * @ingroup sli
 * @brief Write a REG_VFI command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param domain Pointer to the domain object.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_reg_vfi(sli4_t *sli4, void *buf, size_t size, ocs_domain_t *domain)
{
	sli4_cmd_reg_vfi_t	*reg_vfi = buf;
	fc_plogi_payload_t	*flogi_rsp_sparams = NULL;
	uint8_t			bbscn_fabric = 0;

	if (!sli4 || !buf || !domain) {
		return 0;
	}

	ocs_memset(buf, 0, size);

	reg_vfi->hdr.command = SLI4_MBOX_COMMAND_REG_VFI;

	reg_vfi->vfi = domain->indicator;

	reg_vfi->fcfi = domain->fcf_indicator;

	// TODO contents of domain->dma only valid if topo == FABRIC
	reg_vfi->sparm.bde_type = SLI4_BDE_TYPE_BDE_64;
	reg_vfi->sparm.buffer_length = 0x70;
	reg_vfi->sparm.u.data.buffer_address_low  = ocs_addr32_lo(domain->dma.phys);
	reg_vfi->sparm.u.data.buffer_address_high = ocs_addr32_hi(domain->dma.phys);

	reg_vfi->e_d_tov = sli4->config.e_d_tov;
	reg_vfi->r_a_tov = sli4->config.r_a_tov;

	reg_vfi->vp = TRUE;
	reg_vfi->vpi = domain->sport->indicator;
	ocs_memcpy(reg_vfi->wwpn, &domain->sport->sli_wwpn, sizeof(reg_vfi->wwpn));
	reg_vfi->local_n_port_id = domain->sport->fc_id;

	flogi_rsp_sparams = (fc_plogi_payload_t *)domain->flogi_service_params;
	bbscn_fabric = (ocs_be32toh(flogi_rsp_sparams->common_service_parameters[1]) >> 12) & 0xF;
	ocs_log_debug(sli4->os, "bbscn_fabric: %d\n", bbscn_fabric);

	/**
	 * If our advertised service parameter BB_SC_N is 0 or
	 * accept BB_SC_N is 0, send REG_VFI with BBCR set to 0
	 */
	if (ocs_bbcr_enabled(sli4->os) && bbscn_fabric) {
		/**
		 * If the accept BB_SC_N value is greater than our maximum supported value
		 * (BBSCN_MAX from READ_CONFIG), then BBCR should not be enabled
		 */
		if (bbscn_fabric <= sli4->config.bbscn_max) {
			/* BB_SC_N value should be set to max of (advertised value, accept value) */
			reg_vfi->bbscn = OCS_MAX(sli4->config.bbscn_def, bbscn_fabric);
			reg_vfi->bbcr = 1;
			ocs_log_info(sli4->os, "Enabling BBCR with BB_SC_N: %d\n", reg_vfi->bbscn);
		}
	}

	return sizeof(sli4_cmd_reg_vfi_t);
}

/**
 * @ingroup sli
 * @brief Write a REG_VPI command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param sport Point to SLI Port object.
 * @param update Boolean indicating whether to update the existing VPI (true)
 * or create a new VPI (false).
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_reg_vpi(sli4_t *sli4, void *buf, size_t size, ocs_sli_port_t *sport, uint8_t update)
{
	sli4_cmd_reg_vpi_t	*reg_vpi = buf;

	if (!sli4 || !buf || !sport) {
		return 0;
	}

	ocs_memset(buf, 0, size);

	reg_vpi->hdr.command = SLI4_MBOX_COMMAND_REG_VPI;

	reg_vpi->local_n_port_id = sport->fc_id;
	reg_vpi->upd = update != 0;
	ocs_memcpy(reg_vpi->wwpn, &sport->sli_wwpn, sizeof(reg_vpi->wwpn));
	reg_vpi->vpi = sport->indicator;
	reg_vpi->vfi = sport->domain->indicator;

	return sizeof(sli4_cmd_reg_vpi_t);
}

/**
 * @brief Write a REQUEST_FEATURES command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param mask Features to request.
 * @param query Use feature query mode (does not change FW).
 *
 * @return Returns the number of bytes written.
 */
static int32_t
sli_cmd_request_features(sli4_t *sli4, void *buf, size_t size, sli4_features_t mask, uint8_t query)
{
	sli4_cmd_request_features_t *features = buf;

	ocs_memset(buf, 0, size);

	features->hdr.command = SLI4_MBOX_COMMAND_REQUEST_FEATURES;

	if (query) {
		features->qry = TRUE;
	}
	features->command.dword = mask.dword;

	return sizeof(sli4_cmd_request_features_t);
}

/**
 * @ingroup sli
 * @brief Write a SLI_CONFIG command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param length Length in bytes of attached command.
 * @param dma DMA buffer for non-embedded commands.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_sli_config(sli4_t *sli4, void *buf, size_t size, uint32_t length, ocs_dma_t *dma)
{
	sli4_cmd_sli_config_t	*sli_config = NULL;

	if ((length > sizeof(sli_config->payload.embed)) && (dma == NULL)) {
		ocs_log_test(sli4->os, "length(%d) > payload(%ld)\n", length, sizeof(sli_config->payload.embed));
		return -1;
	}

	sli_config = buf;

	ocs_memset(buf, 0, size);

	sli_config->hdr.command = SLI4_MBOX_COMMAND_SLI_CONFIG;
	if (NULL == dma) {
		sli_config->emb = TRUE;
		sli_config->payload_length = length;
	} else {
		sli_config->emb = FALSE;

		sli_config->pmd_count = 1;

		sli_config->payload.mem.address_low = ocs_addr32_lo(dma->phys);
		sli_config->payload.mem.address_high = ocs_addr32_hi(dma->phys);
		sli_config->payload.mem.length = dma->size;
		sli_config->payload_length = dma->size;
#if defined(OCS_INCLUDE_DEBUG)
		/* save pointer to DMA for BMBX dumping purposes */
		sli4->bmbx_non_emb_pmd = dma;
#endif

	}

	return offsetof(sli4_cmd_sli_config_t, payload.embed);
}

/**
 * @brief Initialize SLI Port control register.
 *
 * @param sli4 SLI context pointer.
 * @param endian Endian value to write.
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */

static int32_t
sli_sliport_control(sli4_t *sli4, uint32_t endian)
{
	uint32_t iter;
	int32_t rc;

	rc = -1;

	/* Initialize port, endian */
	sli_reg_write(sli4, SLI4_REG_SLIPORT_CONTROL, endian | SLI4_SLIPORT_CONTROL_IP);

	for (iter = 0; iter < 3000; iter ++) {
		ocs_delay_usec(SLI4_INIT_PORT_DELAY_US);	/* 10 ms */
		/* Wait for FW to become ready & error to be cleared */
		if ((sli_fw_ready(sli4) == 1) && !sli_fw_error_status(sli4)) {
			rc = 0;
			break;
		}
	}

	if (rc)
		ocs_log_crit(sli4->os, "port failed to become ready after initialization\n");

	return rc;
}

/**
 * @ingroup sli
 * @brief Write a UNREG_FCFI command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param indicator Indicator value.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_unreg_fcfi(sli4_t *sli4, void *buf, size_t size, uint16_t indicator)
{
	sli4_cmd_unreg_fcfi_t	*unreg_fcfi = buf;

	if (!sli4 || !buf) {
		return 0;
	}

	ocs_memset(buf, 0, size);

	unreg_fcfi->hdr.command = SLI4_MBOX_COMMAND_UNREG_FCFI;

	unreg_fcfi->fcfi = indicator;

	return sizeof(sli4_cmd_unreg_fcfi_t);
}

/**
 * @ingroup sli
 * @brief Write an UNREG_RPI command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param indicator Indicator value.
 * @param which Type of unregister, such as node, port, domain, or FCF.
 * @param fc_id FC address.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_unreg_rpi(sli4_t *sli4, void *buf, size_t size, uint16_t indicator, sli4_resource_e which,
		uint32_t fc_id)
{
	sli4_cmd_unreg_rpi_t	*unreg_rpi = buf;
	uint8_t		index_indicator = 0;

	if (!sli4 || !buf) {
		return 0;
	}

	ocs_memset(buf, 0, size);

	unreg_rpi->hdr.command = SLI4_MBOX_COMMAND_UNREG_RPI;

	switch (which) {
	case SLI_RSRC_FCOE_RPI:
		index_indicator = SLI4_UNREG_RPI_II_RPI;
		if (fc_id != UINT32_MAX) {
			unreg_rpi->dp = TRUE;
			unreg_rpi->destination_n_port_id = fc_id & 0x00ffffff;
		}
		break;
	case SLI_RSRC_FCOE_VPI:
		index_indicator = SLI4_UNREG_RPI_II_VPI;
		break;
	case SLI_RSRC_FCOE_VFI:
		index_indicator = SLI4_UNREG_RPI_II_VFI;
		break;
	case SLI_RSRC_FCOE_FCFI:
		index_indicator = SLI4_UNREG_RPI_II_FCFI;
		break;
	default:
		ocs_log_test(sli4->os, "unknown type %#x\n", which);
		return 0;
	}

	unreg_rpi->ii = index_indicator;
	unreg_rpi->index = indicator;

	return sizeof(sli4_cmd_unreg_rpi_t);
}

/**
 * @ingroup sli
 * @brief Write an UNREG_VFI command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param domain Pointer to the domain object
 * @param which Type of unregister, such as domain, FCFI, or everything.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_unreg_vfi(sli4_t *sli4, void *buf, size_t size, ocs_domain_t *domain, uint32_t which)
{
	sli4_cmd_unreg_vfi_t	*unreg_vfi = buf;

	if (!sli4 || !buf || !domain) {
		return 0;
	}

	ocs_memset(buf, 0, size);

	unreg_vfi->hdr.command = SLI4_MBOX_COMMAND_UNREG_VFI;
	switch (which) {
	case SLI4_UNREG_TYPE_DOMAIN:
		unreg_vfi->index = domain->indicator;
		break;
	case SLI4_UNREG_TYPE_FCF:
		unreg_vfi->index = domain->fcf_indicator;
		break;
	case SLI4_UNREG_TYPE_ALL:
		unreg_vfi->index = UINT16_MAX;
		break;
	default:
		return 0;
	}

	if (SLI4_UNREG_TYPE_DOMAIN != which) {
		unreg_vfi->ii = SLI4_UNREG_VFI_II_FCFI;
	}

	return sizeof(sli4_cmd_unreg_vfi_t);
}

/**
 * @ingroup sli
 * @brief Write an UNREG_VPI command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param indicator Indicator value.
 * @param which Type of unregister: port, domain, FCFI, everything
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_unreg_vpi(sli4_t *sli4, void *buf, size_t size, uint16_t indicator, uint32_t which)
{
	sli4_cmd_unreg_vpi_t	*unreg_vpi = buf;

	if (!sli4 || !buf) {
		return 0;
	}

	ocs_memset(buf, 0, size);

	unreg_vpi->hdr.command = SLI4_MBOX_COMMAND_UNREG_VPI;
	unreg_vpi->index = indicator;
	switch (which) {
	case SLI4_UNREG_TYPE_PORT:
		unreg_vpi->ii = SLI4_UNREG_VPI_II_VPI;
		break;
	case SLI4_UNREG_TYPE_DOMAIN:
		unreg_vpi->ii = SLI4_UNREG_VPI_II_VFI;
		break;
	case SLI4_UNREG_TYPE_FCF:
		unreg_vpi->ii = SLI4_UNREG_VPI_II_FCFI;
		break;
	case SLI4_UNREG_TYPE_ALL:
		unreg_vpi->index = UINT16_MAX;	/* override indicator */
		unreg_vpi->ii = SLI4_UNREG_VPI_II_FCFI;
		break;
	default:
		return 0;
	}

	return sizeof(sli4_cmd_unreg_vpi_t);
}

/**
 * @ingroup sli
 * @brief Write an CONFIG_AUTO_XFER_RDY command to the provided buffer.
 *        Command to enable AXR in G5.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param max_burst_len if the write FCP_DL is less than this size,
 * then the SLI port will generate the auto XFER_RDY.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_config_auto_xfer_rdy(sli4_t *sli4, void *buf, size_t size, uint32_t max_burst_len)
{
	sli4_cmd_config_auto_xfer_rdy_t	*req = buf;

	if (!sli4 || !buf) {
		return 0;
	}

	ocs_memset(buf, 0, size);

	req->hdr.command = SLI4_MBOX_COMMAND_CONFIG_TOW;
	req->max_burst_len = max_burst_len;

	return sizeof(sli4_cmd_config_auto_xfer_rdy_t);
}

/**
 * @ingroup sli
 * @brief Write an CONFIG_AUTO_XFER_RDY_HP command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param max_burst_len if the write FCP_DL is less than this size,
 * @param esoc enable start offset computation,
 * @param block_size block size,
 * then the SLI port will generate the auto XFER_RDY.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_config_auto_xfer_rdy_hp(sli4_t *sli4, void *buf, size_t size,
				uint32_t max_burst_len, uint32_t block_size)
{
	sli4_cmd_config_auto_xfer_rdy_hp_t *req = buf;

	if (!sli4 || !buf)
		return 0;

	ocs_memset(buf, 0, size);

	req->hdr.command = SLI4_MBOX_COMMAND_CONFIG_TOW_HP;
	req->max_burst_len = max_burst_len;
	req->esoc = 1;
	req->block_size = block_size;

	return sizeof(sli4_cmd_config_auto_xfer_rdy_hp_t);
}

/**
 * @ingroup sli
 * @brief Write an CONFIG_OPTIMIZED_WRITE command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param max_burst_len if the write FCP_DL is less than this size,
 * @param xri_cnt Total XRIs that will be posted,
 * then the SLI port will generate the auto XFER_RDY.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_config_optimized_write(sli4_t *sli4, void *buf, size_t size,
			       uint32_t max_burst_len, uint32_t xri_cnt,
			       uint32_t tow_feature)
{
	sli4_cmd_config_tow_t *req = buf;

	if (!sli4 || !buf) {
		return 0;
	}

	ocs_memset(buf, 0, size);

	req->hdr.command = SLI4_MBOX_COMMAND_CONFIG_TOW;
	if (tow_feature & OCS_TOW_FEATURE_AXR)
		req->axs = 1;
	if (tow_feature & OCS_TOW_FEATURE_TFB)
		req->fbs = 1;

	req->xpns = 0;
	req->scsi_max_data_len = max_burst_len;
	req->version = 1;
	req->pool0_xri_cnt = xri_cnt;

	return sizeof(sli4_cmd_config_tow_t);
}

/**
 * @ingroup sli
 * @brief Write an CONFIG_OPTIMIZED_WRITE_HP command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to the destination buffer.
 * @param size Buffer size, in bytes.
 * @param max_burst_len if the write FCP_DL is less than this size,
 * @param xri_cnt Total XRIs that will be posted,
 * @param block_size block size,
 * then the SLI port will generate the auto XFER_RDY.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_config_optimized_write_hp(sli4_t *sli4, void *buf,
				  size_t size, uint32_t max_burst_len,
				  uint32_t xri_cnt, uint32_t block_size,
				  uint32_t tow_feature)
{
	sli4_cmd_config_tow_hp_t *req = buf;

	if (!sli4 || !buf) {
		return 0;
	}

	ocs_memset(buf, 0, size);

	req->hdr.command = SLI4_MBOX_COMMAND_CONFIG_TOW_HP;
	if (tow_feature & OCS_TOW_FEATURE_AXR)
		req->axs = 1;
	if (tow_feature & OCS_TOW_FEATURE_TFB)
		req->fbs = 1;

	req->xpns = 0;
	req->scsi_max_data_len = max_burst_len;
	req->esoc = 1;
	req->scsi_block_size = block_size;
	req->pool0_xri_cnt =  xri_cnt;

	return sizeof(sli4_cmd_config_tow_hp_t);
}

/**
 * @brief Write a COMMON_FUNCTION_RESET command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 *
 * @return Returns the number of bytes written.
 */
static int32_t
sli_cmd_common_function_reset(sli4_t *sli4, void *buf, size_t size)
{
	sli4_req_common_function_reset_t *reset = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		uint32_t payload_size;

		/* Payload length must accomodate both request and response */
		payload_size = max(sizeof(sli4_req_common_function_reset_t),
				sizeof(sli4_res_common_function_reset_t));

		sli_config_off = sli_cmd_sli_config(sli4, buf, size, payload_size,
				NULL);
	}
	reset = (sli4_req_common_function_reset_t *)((uint8_t *)buf + sli_config_off);

	reset->hdr.opcode = SLI4_OPC_COMMON_FUNCTION_RESET;
	reset->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;

	return(sli_config_off + sizeof(sli4_req_common_function_reset_t));
}

/**
 * @brief Write a COMMON_CREATE_CQ command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param qmem DMA memory for the queue.
 * @param eq_id Associated EQ_ID
 * @param ignored1 This parameter carries the ULP which is only used for WQ and RQs
 * @param ignored2 Ignored (Used for consistency among queue creation functions)
 *
 * @note This creates a Version 0 message.
 *
 * @return Returns the number of bytes written.
 */
static int32_t
sli_cmd_common_create_cq(sli4_t *sli4, void *buf, size_t size,
			 ocs_dma_t *qmem, uint16_t eq_id,
			 uint16_t ignored1, bool ignored2)
{
	sli4_req_common_create_cq_v0_t	*cqv0 = NULL;
	sli4_req_common_create_cq_v2_t	*cqv2 = NULL;
	uint32_t	p, sli_config_off = 0;
	uintptr_t	addr;
	uint32_t	if_type = sli4->if_type;
	uint32_t	num_pages = 0;
	uint32_t 	cmd_size = 0;
	size_t		qmem_size = sli_get_qmem_size(qmem);
	uint32_t	qpage_size = sli_get_qpage_size(qmem_size);

	/* First calculate number of pages and the mailbox cmd length */
	/* Any 'case' added to this switch must be added to the subsequent switch-case as well */
	switch (if_type)
	{
	case SLI4_IF_TYPE_BE3_SKH_PF:
		num_pages = sli_page_count(qmem_size, qpage_size);
		cmd_size = sizeof(sli4_req_common_create_cq_v0_t) + (8 * num_pages);
		break;
	case SLI4_IF_TYPE_LANCER_FC_ETH:
	case SLI4_IF_TYPE_LANCER_G7:
		num_pages = sli_page_count(qmem_size, qpage_size);
		cmd_size = sizeof(sli4_req_common_create_cq_v2_t) + (8 * num_pages);
		break;
	default:
		ocs_log_test(sli4->os, "unsupported IF_TYPE %d\n", if_type);
		return -1;
	}

	/* now that we have the mailbox command size, we can set SLI_CONFIG fields */
	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		uint32_t payload_size;

		/* Payload length must accomodate both request and response */
		payload_size = OCS_MAX((size_t)cmd_size, sizeof(sli4_res_common_create_queue_t));
		sli_config_off = sli_cmd_sli_config(sli4, buf, size, payload_size, NULL);
	}

	/* Expect only 3 values for 'if_type', rest would have been filtered by the above switch case */
	switch (if_type)
	{
	case SLI4_IF_TYPE_BE3_SKH_PF:
		cqv0 = (sli4_req_common_create_cq_v0_t *)((uint8_t *)buf + sli_config_off);
		cqv0->hdr.opcode = SLI4_OPC_COMMON_CREATE_CQ;
		cqv0->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
		cqv0->hdr.version = 0;
		cqv0->hdr.request_length = cmd_size - sizeof(sli4_req_hdr_t);

		/* valid values for number of pages: 1, 2, 4 (sec 4.4.3) */
		cqv0->num_pages = num_pages;
		switch (cqv0->num_pages) {
		case 1:
			cqv0->cqecnt = SLI4_CQ_CNT_256;
			break;
		case 2:
			cqv0->cqecnt = SLI4_CQ_CNT_512;
			break;
		case 4:
			cqv0->cqecnt = SLI4_CQ_CNT_1024;
			break;
		default:
			ocs_log_test(sli4->os, "num_pages %d not valid\n", cqv0->num_pages);
			return -1;
		}
		cqv0->evt = TRUE;
		cqv0->valid = TRUE;
		/* TODO cq->nodelay = ???; */
		/* TODO cq->clswm = ???; */
		cqv0->arm = FALSE;
		cqv0->eq_id = eq_id;

		for (p = 0; p < cqv0->num_pages; p++) {
			addr = qmem[p].phys;
			cqv0->page_physical_address[p].low = ocs_addr32_lo(addr);
			cqv0->page_physical_address[p].high = ocs_addr32_hi(addr);
		}

		break;
	case SLI4_IF_TYPE_LANCER_FC_ETH:
		/* fall-through */
		FALL_THROUGH;
	case SLI4_IF_TYPE_LANCER_G7:
		cqv2 = (sli4_req_common_create_cq_v2_t *)((uint8_t *)buf + sli_config_off);
		cqv2->hdr.opcode = SLI4_OPC_COMMON_CREATE_CQ;
		cqv2->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
		cqv2->hdr.version = 2;
		cqv2->hdr.request_length = cmd_size - sizeof(sli4_req_hdr_t);

		if (if_type == SLI4_IF_TYPE_LANCER_G7)
			cqv2->autovalid = TRUE;

		/* Fill the page_size_multiplier */
		cqv2->page_size = (qpage_size / SLI_PAGE_SIZE);

		/* valid values for number of pages: 1, 2, 4, 8 (sec 4.4.3) */
		cqv2->num_pages = num_pages;
		if (!cqv2->num_pages || (cqv2->num_pages > SLI4_COMMON_CREATE_CQ_V2_MAX_PAGES)) {
			return 0;
		}

		switch (cqv2->num_pages) {
		case 1:
			cqv2->cqecnt = SLI4_CQ_CNT_256;
			break;
		case 2:
			cqv2->cqecnt = SLI4_CQ_CNT_512;
			break;
		case 4:
			cqv2->cqecnt = SLI4_CQ_CNT_1024;
			break;
		case 8:
			cqv2->cqecnt = SLI4_CQ_CNT_LARGE;
			cqv2->cqe_count = (qmem_size / SLI4_CQE_BYTES);
			break;
		default:
			ocs_log_test(sli4->os, "num_pages %d not valid\n", cqv2->num_pages);
			return -1;
		}

		cqv2->evt = TRUE;
		cqv2->valid = TRUE;
		/* TODO cq->nodelay = ???; */
		/* TODO cq->clswm = ???; */
		cqv2->arm = FALSE;
		cqv2->eq_id = eq_id;

		for (p = 0; p < cqv2->num_pages; p++) {
			addr = qmem[p].phys;
			cqv2->page_physical_address[p].low = ocs_addr32_lo(addr);
			cqv2->page_physical_address[p].high = ocs_addr32_hi(addr);
		}

		break;
	default:
		ocs_log_test(sli4->os, "unsupported IF_TYPE %d\n", if_type);
		return -1;
	}

	return (sli_config_off + cmd_size);
}

/**
 * @brief Write a COMMON_DESTROY_CQ command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param cq_id CQ ID
 *
 * @note This creates a Version 0 message.
 *
 * @return Returns the number of bytes written.
 */
static int32_t
sli_cmd_common_destroy_cq(sli4_t *sli4, void *buf, size_t size, uint16_t cq_id)
{
	sli4_req_common_destroy_cq_t	*cq = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				/* Payload length must accomodate both request and response */
				max(sizeof(sli4_req_common_destroy_cq_t),
					sizeof(sli4_res_hdr_t)),
				NULL);
	}
	cq = (sli4_req_common_destroy_cq_t *)((uint8_t *)buf + sli_config_off);

	cq->hdr.opcode = SLI4_OPC_COMMON_DESTROY_CQ;
	cq->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
	cq->hdr.request_length = sizeof(sli4_req_common_destroy_cq_t) -
					sizeof(sli4_req_hdr_t);
	cq->cq_id = cq_id;

	return(sli_config_off + sizeof(sli4_req_common_destroy_cq_t));
}

/**
 * @brief Write a COMMON_MODIFY_EQ_DELAY command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param q Queue object array.
 * @param num_q Queue object array count.
 * @param shift Phase shift for staggering interrupts.
 * @param delay_mult Delay multiplier for limiting interrupt frequency.
 *
 * @return Returns the number of bytes written.
 */
static int32_t
sli_cmd_common_modify_eq_delay(sli4_t *sli4, void *buf, size_t size, sli4_queue_t *q, int num_q, uint32_t shift,
				uint32_t delay_mult)
{
	sli4_req_common_modify_eq_delay_t *modify_delay = NULL;
	uint32_t	sli_config_off = 0;
	int i;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				/* Payload length must accomodate both request and response */
				max(sizeof(sli4_req_common_modify_eq_delay_t), sizeof(sli4_res_hdr_t)),
				NULL);
	}

	modify_delay = (sli4_req_common_modify_eq_delay_t *)((uint8_t *)buf + sli_config_off);

	modify_delay->hdr.opcode = SLI4_OPC_COMMON_MODIFY_EQ_DELAY;
	modify_delay->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
	modify_delay->hdr.request_length = sizeof(sli4_req_common_modify_eq_delay_t) -
					sizeof(sli4_req_hdr_t);

	modify_delay->num_eq = num_q;

	for (i = 0; i<num_q; i++) {
		modify_delay->eq_delay_record[i].eq_id = q[i].id;
		modify_delay->eq_delay_record[i].phase = shift;
		modify_delay->eq_delay_record[i].delay_multiplier = delay_mult;
	}

	return(sli_config_off + sizeof(sli4_req_common_modify_eq_delay_t));
}

/**
 * @brief Write a COMMON_CREATE_EQ command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param qmem DMA memory for the queue.
 * @param ignored1 Ignored (used for consistency among queue creation functions).
 * @param ignored2 Ignored (used for consistency among queue creation functions).
 * @param ignored3 Ignored (used for consistency among queue creation functions).
 *
 * @note Other queue creation routines use the last parameter to pass in
 * the associated Q_ID and ULP. EQ doesn't have an associated queue or ULP,
 * so these parameters are ignored
 *
 * @note This creates a Version 0 message
 *
 * @return Returns the number of bytes written.
 */
static int32_t
sli_cmd_common_create_eq(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *qmem,
			 uint16_t ignored1, uint16_t ignored2, bool ignored3)
{
	sli4_req_common_create_eq_t	*eq = NULL;
	uint32_t	p, sli_config_off = 0;
	uintptr_t	addr;
	size_t		qmem_size = sli_get_qmem_size(qmem);
	uint32_t	qpage_size = sli_get_qpage_size(qmem_size);

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		uint32_t payload_size;

		/* Payload length must accomodate both request and response */
		payload_size = OCS_MAX(sizeof(sli4_req_common_create_eq_t),
				sizeof(sli4_res_common_create_queue_t));
		sli_config_off = sli_cmd_sli_config(sli4, buf, size, payload_size, NULL);
	}
	eq = (sli4_req_common_create_eq_t *)((uint8_t *)buf + sli_config_off);

	eq->hdr.opcode = SLI4_OPC_COMMON_CREATE_EQ;
	eq->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
	eq->hdr.request_length = sizeof(sli4_req_common_create_eq_t) - sizeof(sli4_req_hdr_t);

	if (sli4->if_type == SLI4_IF_TYPE_LANCER_G7) {
		eq->hdr.version = 2;
		eq->autovalid = TRUE;
	}

	/* valid values for number of pages: 1, 2, 4 (sec 4.4.3) */
	eq->num_pages = sli_page_count(qmem_size, qpage_size);
	switch (eq->num_pages) {
	case 1:
		eq->eqesz = SLI4_EQE_SIZE_4;
		eq->count = SLI4_EQ_CNT_1024;
		break;
	case 2:
		eq->eqesz = SLI4_EQE_SIZE_4;
		eq->count = SLI4_EQ_CNT_2048;
		break;
	case 4:
		eq->eqesz = SLI4_EQE_SIZE_4;
		eq->count = SLI4_EQ_CNT_4096;
		break;
	default:
		ocs_log_test(sli4->os, "num_pages %d not valid\n", eq->num_pages);
		return -1;
	}

	eq->valid = TRUE;
	eq->arm = FALSE;
	eq->delay_multiplier = 32;

	for (p = 0; p < eq->num_pages; p++) {
		addr = qmem[p].phys;
		eq->page_address[p].low = ocs_addr32_lo(addr);
		eq->page_address[p].high = ocs_addr32_hi(addr);
	}

	return(sli_config_off + sizeof(sli4_req_common_create_eq_t));
}


/**
 * @brief Write a COMMON_DESTROY_EQ command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param eq_id Queue ID to destroy.
 *
 * @note Other queue creation routines use the last parameter to pass in
 * the associated Q_ID. EQ doesn't have an associated queue so this
 * parameter is ignored.
 *
 * @note This creates a Version 0 message.
 *
 * @return Returns the number of bytes written.
 */
static int32_t
sli_cmd_common_destroy_eq(sli4_t *sli4, void *buf, size_t size, uint16_t eq_id)
{
	sli4_req_common_destroy_eq_t	*eq = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				/* Payload length must accomodate both request and response */
				max(sizeof(sli4_req_common_destroy_eq_t),
					sizeof(sli4_res_hdr_t)),
				NULL);
	}
	eq = (sli4_req_common_destroy_eq_t *)((uint8_t *)buf + sli_config_off);

	eq->hdr.opcode = SLI4_OPC_COMMON_DESTROY_EQ;
	eq->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
	eq->hdr.request_length = sizeof(sli4_req_common_destroy_eq_t) -
					sizeof(sli4_req_hdr_t);

	eq->eq_id = eq_id;

	return(sli_config_off + sizeof(sli4_req_common_destroy_eq_t));
}

/**
 * @brief Write a LOWLEVEL_SET_WATCHDOG command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param timeout watchdog timer timeout in seconds
 *
 * @return void
 */
void
sli4_cmd_lowlevel_set_watchdog(sli4_t *sli4, void *buf, size_t size, uint16_t timeout)
{

	sli4_req_lowlevel_set_watchdog_t *req = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				/* Payload length must accomodate both request and response */
				max(sizeof(sli4_req_lowlevel_set_watchdog_t),
					sizeof(sli4_res_lowlevel_set_watchdog_t)),
				NULL);
	}
	req = (sli4_req_lowlevel_set_watchdog_t *)((uint8_t *)buf + sli_config_off);

	req->hdr.opcode = SLI4_OPC_LOWLEVEL_SET_WATCHDOG;
	req->hdr.subsystem = SLI4_SUBSYSTEM_LOWLEVEL;
	req->hdr.request_length = sizeof(sli4_req_lowlevel_set_watchdog_t) - sizeof(sli4_req_hdr_t);
	req->watchdog_timeout = timeout;

	return;
}

/**
 * @brief Write a LOWLEVEL_SET_DIAG_LOG_OPTIONS command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param timeout watchdog timer timeout in seconds
 *
 * @return void
 */
void
sli4_cmd_lowlevel_enable_ras(sli4_t *sli4, void *buf, size_t size, int log_level,
			     uint32_t unit_size, int32_t buf_count, uint32_t *phys_addr,
			     uintptr_t lwpd_phys_addr)
{
	int32_t i;
	uint32_t sli_config_off = 0;
	sli4_req_lowlevel_set_diag_log_options_t *req = NULL;

	ocs_memset(buf, 0x00, sizeof(sli4_req_lowlevel_set_diag_log_options_t) + sizeof(sli4_req_hdr_t));

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				/* Payload length must accomodate both request and response */
				max(sizeof(sli4_req_lowlevel_set_diag_log_options_t),
				    sizeof(sli4_res_lowlevel_set_diag_log_options_t)),
				NULL);
	}
	req = (sli4_req_lowlevel_set_diag_log_options_t *)((uint8_t *)buf + sli_config_off);

	req->hdr.opcode = SLI4_OPC_LOWLEVEL_SET_DIAG_LOG_OPTIONS;
	req->hdr.subsystem = SLI4_SUBSYSTEM_LOWLEVEL;
	req->hdr.request_length = sizeof(sli4_req_lowlevel_set_diag_log_options_t) -
				  sizeof(sli4_req_hdr_t) + sizeof(uint32_t) * 2 * buf_count;
	req->enable = 1;
	req->reset_action = 0;
	req->log_level = log_level;
	req->buffer_size = (uint8_t)(unit_size / 4096);
	req->buffer_cnt = (uint8_t)buf_count;
	req->acqe_interval = 0;	/* Disabled */
	req->cq_id = 0;
	req->lwpd_addr_lo = (uint32_t)(lwpd_phys_addr & 0xFFFFFFFF);
	req->lwpd_addr_hi = (uint32_t)(lwpd_phys_addr >> 32);
	for (i = 0; i < buf_count; i++) {
		req->buffer_desc[i].addr_lo = phys_addr[i * 2];
		req->buffer_desc[i].addr_hi = phys_addr[i * 2 + 1];
	}
}

/**
 * @brief Write a LOWLEVEL_SET_DIAG_LOG_OPTIONS command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param timeout watchdog timer timeout in seconds
 *
 * @return void
 */
void
sli4_cmd_lowlevel_disable_ras(sli4_t *sli4, void *buf, size_t size)
{
	uint32_t sli_config_off = 0;
	sli4_req_lowlevel_set_diag_log_options_t *req = NULL;

	ocs_memset(buf, 0x00, sizeof(sli4_req_lowlevel_set_diag_log_options_t) + sizeof(sli4_req_hdr_t));

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				/* Payload length must accomodate both request and response */
				max(sizeof(sli4_req_lowlevel_set_diag_log_options_t),
				    sizeof(sli4_res_lowlevel_set_diag_log_options_t)),
				NULL);
	}
	req = (sli4_req_lowlevel_set_diag_log_options_t *)((uint8_t *)buf + sli_config_off);

	req->hdr.opcode = SLI4_OPC_LOWLEVEL_SET_DIAG_LOG_OPTIONS;
	req->hdr.subsystem = SLI4_SUBSYSTEM_LOWLEVEL;
	req->hdr.request_length = sizeof(sli4_req_lowlevel_set_diag_log_options_t) - sizeof(sli4_req_hdr_t);
	req->enable = 0;
}

void
sli4_cmd_lowlevel_get_itcm_parity_stats(sli4_t *sli4, void *buf, size_t size, bool clear_stats)
{
	sli4_req_lowlevel_get_itcm_parity_stats_t *req = NULL;
	uint32_t sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type)
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				/* Payload length must accomodate both request and response */
				max(sizeof(*req), sizeof(sli4_res_lowlevel_get_itcm_parity_stats_t)),
				NULL);

	req = (sli4_req_lowlevel_get_itcm_parity_stats_t *)SLI4_GET_SLI_CONFIG_OFFSET(buf, sli_config_off);
	req->hdr.opcode = SLI4_OPC_LOWLEVEL_GET_ITCM_PARITY_STATS;
	req->hdr.subsystem = SLI4_SUBSYSTEM_LOWLEVEL;
	req->hdr.request_length = (sizeof(*req) - sizeof(sli4_req_hdr_t));
	if (clear_stats) {
		ocs_log_info(sli4->os, "Clearing the ITCM parity stats\n");
		req->clr_pbec = 1;
		req->clr_tpec = 1;
	}
}

static int32_t
sli_cmd_common_get_cntl_attributes(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *dma)
{
	sli4_req_hdr_t *hdr = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				sizeof(sli4_req_hdr_t),
				dma);
	}

	if (dma == NULL) {
		return 0;
	}

	ocs_memset(dma->virt, 0, dma->size);

	hdr = dma->virt;

	hdr->opcode = SLI4_OPC_COMMON_GET_CNTL_ATTRIBUTES;
	hdr->subsystem = SLI4_SUBSYSTEM_COMMON;
	hdr->request_length = dma->size;

	return(sli_config_off + sizeof(sli4_req_hdr_t));
}

/**
 * @brief Write a COMMON_GET_CNTL_ADDL_ATTRIBUTES command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param dma DMA structure from which the data will be copied.
 *
 * @note This creates a Version 0 message.
 *
 * @return Returns the number of bytes written.
 */
static int32_t
sli_cmd_common_get_cntl_addl_attributes(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *dma)
{
	sli4_req_hdr_t *hdr = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size, sizeof(sli4_req_hdr_t), dma);
	}

	if (dma == NULL) {
		return 0;
	}

	ocs_memset(dma->virt, 0, dma->size);

	hdr = dma->virt;

	hdr->opcode = SLI4_OPC_COMMON_GET_CNTL_ADDL_ATTRIBUTES;
	hdr->subsystem = SLI4_SUBSYSTEM_COMMON;
	hdr->request_length = dma->size;

	return(sli_config_off + sizeof(sli4_req_hdr_t));
}

/**
 * @brief Write a COMMON_CREATE_MQ_EXT command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param qmem DMA memory for the queue.
 * @param cq_id Associated CQ_ID.
 * @param ignored1 This parameter carries the ULP which is only used for WQ and RQs
 * @param ignored2 Ignored (Used for consistency among queue creation functions)
 *
 * @note This creates a Version 0 message.
 *
 * @return Returns the number of bytes written.
 */
static int32_t
sli_cmd_common_create_mq_ext(sli4_t *sli4, void *buf, size_t size,
			     ocs_dma_t *qmem, uint16_t cq_id,
			     uint16_t ignored1, bool ignored2)
{
	sli4_req_common_create_mq_ext_t	*mq = NULL;
	uint32_t	p, sli_config_off = 0;
	uintptr_t	addr;
	size_t		qmem_size = sli_get_qmem_size(qmem);
	uint32_t	qpage_size = sli_get_qpage_size(qmem_size);

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		uint32_t payload_size;

		/* Payload length must accomodate both request and response */
		payload_size = OCS_MAX(sizeof(sli4_req_common_create_mq_ext_t),
				sizeof(sli4_res_common_create_queue_t));
		sli_config_off = sli_cmd_sli_config(sli4, buf, size, payload_size, NULL);
	}
	mq = (sli4_req_common_create_mq_ext_t *)((uint8_t *)buf + sli_config_off);

	mq->hdr.opcode = SLI4_OPC_COMMON_CREATE_MQ_EXT;
	mq->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
	mq->hdr.request_length = sizeof(sli4_req_common_create_mq_ext_t) - sizeof(sli4_req_hdr_t);

	/* valid values for number of pages: 1, 2, 4, 8 (sec 4.4.12) */
	mq->num_pages = sli_page_count(qmem_size, qpage_size);
	switch (mq->num_pages) {
	case 1:
		mq->ring_size = SLI4_MQE_SIZE_16;
		break;
	case 2:
		mq->ring_size = SLI4_MQE_SIZE_32;
		break;
	case 4:
		mq->ring_size = SLI4_MQE_SIZE_64;
		break;
	case 8:
		mq->ring_size = SLI4_MQE_SIZE_128;
		break;
	default:
		ocs_log_test(sli4->os, "num_pages %d not valid\n", mq->num_pages);
		return -1;
	}

	/* TODO break this down by sli4->config.topology */
	mq->async_event_bitmap = SLI4_ASYNC_EVT_FC_FCOE;

	if (sli4->config.mq_create_version) {
		mq->cq_id_v1 = cq_id;
		mq->hdr.version = 1;
	}
	else {
		mq->cq_id_v0 = cq_id;
	}
	mq->val = TRUE;

	for (p = 0; p < mq->num_pages; p++) {
		addr = qmem[p].phys;
		mq->page_physical_address[p].low = ocs_addr32_lo(addr);
		mq->page_physical_address[p].high = ocs_addr32_hi(addr);
	}

	return(sli_config_off + sizeof(sli4_req_common_create_mq_ext_t));
}

/**
 * @brief Write a COMMON_DESTROY_MQ command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param mq_id MQ ID
 *
 * @note This creates a Version 0 message.
 *
 * @return Returns the number of bytes written.
 */
static int32_t
sli_cmd_common_destroy_mq(sli4_t *sli4, void *buf, size_t size, uint16_t mq_id)
{
	sli4_req_common_destroy_mq_t	*mq = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				/* Payload length must accomodate both request and response */
				max(sizeof(sli4_req_common_destroy_mq_t),
					sizeof(sli4_res_hdr_t)),
				NULL);
	}
	mq = (sli4_req_common_destroy_mq_t *)((uint8_t *)buf + sli_config_off);

	mq->hdr.opcode = SLI4_OPC_COMMON_DESTROY_MQ;
	mq->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
	mq->hdr.request_length = sizeof(sli4_req_common_destroy_mq_t) -
					sizeof(sli4_req_hdr_t);

	mq->mq_id = mq_id;

	return(sli_config_off + sizeof(sli4_req_common_destroy_mq_t));
}

/**
 * @ingroup sli
 * @brief Write a COMMON_NOP command
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param context NOP context value (passed to response, except on FC/FCoE).
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_nop(sli4_t *sli4, void *buf, size_t size, uint64_t context)
{
	sli4_req_common_nop_t *nop = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				/* Payload length must accomodate both request and response */
				max(sizeof(sli4_req_common_nop_t), sizeof(sli4_res_common_nop_t)),
				NULL);
	}

	nop = (sli4_req_common_nop_t *)((uint8_t *)buf + sli_config_off);

	nop->hdr.opcode = SLI4_OPC_COMMON_NOP;
	nop->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
	nop->hdr.request_length = 8;

	ocs_memcpy(&nop->context, &context, sizeof(context));

	return(sli_config_off + sizeof(sli4_req_common_nop_t));
}

/**
 * @ingroup sli
 * @brief Write a COMMON_GET_RESOURCE_EXTENT_INFO command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param rtype Resource type (for example, XRI, VFI, VPI, and RPI).
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_get_resource_extent_info(sli4_t *sli4, void *buf, size_t size, uint16_t rtype)
{
	sli4_req_common_get_resource_extent_info_t *extent = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				sizeof(sli4_req_common_get_resource_extent_info_t),
				NULL);
	}

	extent = (sli4_req_common_get_resource_extent_info_t *)((uint8_t *)buf + sli_config_off);

	extent->hdr.opcode = SLI4_OPC_COMMON_GET_RESOURCE_EXTENT_INFO;
	extent->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
	extent->hdr.request_length = 4;

	extent->resource_type = rtype;

	return(sli_config_off + sizeof(sli4_req_common_get_resource_extent_info_t));
}

/**
 * @ingroup sli
 * @brief Write a COMMON_GET_SLI4_PARAMETERS command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_get_sli4_parameters(sli4_t *sli4, void *buf, size_t size)
{
	sli4_req_hdr_t	*hdr = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				sizeof(sli4_res_common_get_sli4_parameters_t),
				NULL);
	}

	hdr = (sli4_req_hdr_t *)((uint8_t *)buf + sli_config_off);

	hdr->opcode = SLI4_OPC_COMMON_GET_SLI4_PARAMETERS;
	hdr->subsystem = SLI4_SUBSYSTEM_COMMON;
	hdr->request_length = 0x50;

	return(sli_config_off + sizeof(sli4_req_hdr_t));
}

/**
 * @brief Write a COMMON_QUERY_FW_CONFIG command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to destination buffer.
 * @param size Buffer size in bytes.
 *
 * @return Returns the number of bytes written
 */
static int32_t
sli_cmd_common_query_fw_config(sli4_t *sli4, void *buf, size_t size)
{
	sli4_req_common_query_fw_config_t   *fw_config;
	uint32_t	sli_config_off = 0;
	uint32_t payload_size;

	/* Payload length must accomodate both request and response */
	payload_size = max(sizeof(sli4_req_common_query_fw_config_t),
			   sizeof(sli4_res_common_query_fw_config_t));

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				payload_size,
				NULL);
	}

	fw_config = (sli4_req_common_query_fw_config_t*)((uint8_t*)buf + sli_config_off);
	fw_config->hdr.opcode	      = SLI4_OPC_COMMON_QUERY_FW_CONFIG;
	fw_config->hdr.subsystem      = SLI4_SUBSYSTEM_COMMON;
	fw_config->hdr.request_length = payload_size - sizeof(sli4_req_hdr_t);
	return sli_config_off + sizeof(sli4_req_common_query_fw_config_t);
}

/**
 * @brief Write a COMMON_GET_PORT_NAME command to the provided buffer.
 *
 * @param sli4 SLI context pointer.
 * @param buf Virtual pointer to destination buffer.
 * @param size Buffer size in bytes.
 *
 * @note Function supports both version 0 and 1 forms of this command via
 * the IF_TYPE.
 *
 * @return Returns the number of bytes written.
 */
static int32_t
sli_cmd_common_get_port_name(sli4_t *sli4, void *buf, size_t size)
{
	sli4_req_common_get_port_name_t	*port_name;
	uint32_t	sli_config_off = 0;
	uint32_t	payload_size;
	uint8_t		version = 0;
	uint8_t		pt = 0;

	/* Select command version according to IF_TYPE */
	switch (sli4->if_type) {
	case SLI4_IF_TYPE_BE3_SKH_PF:
	case SLI4_IF_TYPE_BE3_SKH_VF:
		version = 0;
		break;
	case SLI4_IF_TYPE_LANCER_FC_ETH:
	case SLI4_IF_TYPE_LANCER_RDMA:
	case SLI4_IF_TYPE_LANCER_G7:
		version = 1;
		break;
	default:
		ocs_log_test(sli4->os, "unsupported IF_TYPE %d\n", sli4->if_type);
		return 0;
	}

	/* Payload length must accomodate both request and response */
	payload_size = max(sizeof(sli4_req_common_get_port_name_t),
			   sizeof(sli4_res_common_get_port_name_t));

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				payload_size,
				NULL);

		pt = 1;
	}

	port_name = (sli4_req_common_get_port_name_t *)((uint8_t *)buf + sli_config_off);

	port_name->hdr.opcode		= SLI4_OPC_COMMON_GET_PORT_NAME;
	port_name->hdr.subsystem	= SLI4_SUBSYSTEM_COMMON;
	port_name->hdr.request_length	= sizeof(sli4_req_hdr_t) + (version * sizeof(uint32_t));
	port_name->hdr.version		= version;

	/* Set the port type value (ethernet=0, FC=1) for V1 commands */
	if (version == 1) {
		port_name->pt = pt;
	}

	return sli_config_off + port_name->hdr.request_length;
}


/**
 * @ingroup sli
 * @brief Write a COMMON_WRITE_OBJECT command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param noc True if the object should be written but not committed to flash.
 * @param eof True if this is the last write for this object.
 * @param desired_write_length Number of bytes of data to write to the object.
 * @param offset Offset, in bytes, from the start of the object.
 * @param object_name Name of the object to write.
 * @param dma DMA structure from which the data will be copied.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_write_object(sli4_t *sli4, void *buf, size_t size,
		uint16_t noc, uint16_t eof, uint32_t desired_write_length,
		uint32_t offset,
		char *object_name,
		ocs_dma_t *dma)
{
	sli4_req_common_write_object_t *wr_obj = NULL;
	uint32_t	sli_config_off = 0;
	sli4_bde_t *host_buffer;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				sizeof (sli4_req_common_write_object_t) + sizeof (sli4_bde_t),
				NULL);
	}

	wr_obj = (sli4_req_common_write_object_t *)((uint8_t *)buf + sli_config_off);

	wr_obj->hdr.opcode = SLI4_OPC_COMMON_WRITE_OBJECT;
	wr_obj->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
	wr_obj->hdr.request_length = sizeof(*wr_obj) - 4*sizeof(uint32_t) + sizeof(sli4_bde_t);
	wr_obj->hdr.timeout = 0;
	wr_obj->hdr.version = 0;

	wr_obj->noc = noc;
	wr_obj->eof = eof;
	wr_obj->desired_write_length = desired_write_length;
	wr_obj->write_offset = offset;
	ocs_strncpy(wr_obj->object_name, object_name, sizeof(wr_obj->object_name));
	wr_obj->host_buffer_descriptor_count = 1;

	host_buffer = (sli4_bde_t *)wr_obj->host_buffer_descriptor;

	// Setup to transfer xfer_size bytes to device
	host_buffer->bde_type = SLI4_BDE_TYPE_BDE_64;
	host_buffer->buffer_length = desired_write_length;
	host_buffer->u.data.buffer_address_low = ocs_addr32_lo(dma->phys);
	host_buffer->u.data.buffer_address_high = ocs_addr32_hi(dma->phys);


	return(sli_config_off + sizeof(sli4_req_common_write_object_t) + sizeof (sli4_bde_t));
}


/**
 * @ingroup sli
 * @brief Write a COMMON_DELETE_OBJECT command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param object_name Name of the object to write.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_delete_object(sli4_t *sli4, void *buf, size_t size,
		char *object_name)
{
	sli4_req_common_delete_object_t *del_obj = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				sizeof (sli4_req_common_delete_object_t),
				NULL);
	}

	del_obj = (sli4_req_common_delete_object_t *)((uint8_t *)buf + sli_config_off);

	del_obj->hdr.opcode = SLI4_OPC_COMMON_DELETE_OBJECT;
	del_obj->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
	del_obj->hdr.request_length = sizeof(*del_obj);
	del_obj->hdr.timeout = 0;
	del_obj->hdr.version = 0;

	ocs_strncpy(del_obj->object_name, object_name, sizeof(del_obj->object_name));
	return(sli_config_off + sizeof(sli4_req_common_delete_object_t));
}

/**
 * @ingroup sli
 * @brief Write a COMMON_READ_OBJECT command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param desired_read_length Number of bytes of data to read from the object.
 * @param offset Offset, in bytes, from the start of the object.
 * @param object_name Name of the object to read.
 * @param dma DMA structure from which the data will be copied.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_read_object(sli4_t *sli4, void *buf, size_t size,
		uint32_t desired_read_length,
		uint32_t offset,
		char *object_name,
		ocs_dma_t *dma)
{
	sli4_req_common_read_object_t *rd_obj = NULL;
	uint32_t	sli_config_off = 0;
	sli4_bde_t *host_buffer;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				sizeof (sli4_req_common_read_object_t) + sizeof (sli4_bde_t),
				NULL);
	}

	rd_obj = (sli4_req_common_read_object_t *)((uint8_t *)buf + sli_config_off);

	rd_obj->hdr.opcode = SLI4_OPC_COMMON_READ_OBJECT;
	rd_obj->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
	rd_obj->hdr.request_length = sizeof(*rd_obj) - 4*sizeof(uint32_t) + sizeof(sli4_bde_t);
	rd_obj->hdr.timeout = 0;
	rd_obj->hdr.version = 0;

	rd_obj->desired_read_length = desired_read_length;
	rd_obj->read_offset = offset;
	ocs_strncpy(rd_obj->object_name, object_name, sizeof(rd_obj->object_name));
	rd_obj->host_buffer_descriptor_count = 1;

	host_buffer = (sli4_bde_t *)rd_obj->host_buffer_descriptor;

	// Setup to transfer xfer_size bytes to device
	host_buffer->bde_type = SLI4_BDE_TYPE_BDE_64;
	host_buffer->buffer_length = desired_read_length;
	if (dma != NULL) {
		host_buffer->u.data.buffer_address_low = ocs_addr32_lo(dma->phys);
		host_buffer->u.data.buffer_address_high = ocs_addr32_hi(dma->phys);
	} else {
		host_buffer->u.data.buffer_address_low = 0;
		host_buffer->u.data.buffer_address_high = 0;
	}


	return(sli_config_off + sizeof(sli4_req_common_read_object_t) + sizeof (sli4_bde_t));
}

/**
 * @ingroup sli
 * @brief Write a DMTF_EXEC_CLP_CMD command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param cmd DMA structure that describes the buffer for the command.
 * @param resp DMA structure that describes the buffer for the response.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_dmtf_exec_clp_cmd(sli4_t *sli4, void *buf, size_t size,
		ocs_dma_t *cmd,
		ocs_dma_t *resp)
{
	sli4_req_dmtf_exec_clp_cmd_t *clp_cmd = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				sizeof (sli4_req_dmtf_exec_clp_cmd_t),
				NULL);
	}

	clp_cmd = (sli4_req_dmtf_exec_clp_cmd_t*)((uint8_t *)buf + sli_config_off);

	clp_cmd->hdr.opcode = SLI4_OPC_DMTF_EXEC_CLP_CMD;
	clp_cmd->hdr.subsystem = SLI4_SUBSYSTEM_DMTF;
	clp_cmd->hdr.request_length = sizeof(sli4_req_dmtf_exec_clp_cmd_t) -
					sizeof(sli4_req_hdr_t);
	clp_cmd->hdr.timeout = 0;
	clp_cmd->hdr.version = 0;
	clp_cmd->cmd_buf_length = cmd->size;
	clp_cmd->cmd_buf_addr_low = ocs_addr32_lo(cmd->phys);
	clp_cmd->cmd_buf_addr_high = ocs_addr32_hi(cmd->phys);
	clp_cmd->resp_buf_length = resp->size;
	clp_cmd->resp_buf_addr_low = ocs_addr32_lo(resp->phys);
	clp_cmd->resp_buf_addr_high = ocs_addr32_hi(resp->phys);

	return(sli_config_off + sizeof(sli4_req_dmtf_exec_clp_cmd_t));
}

/**
 * @ingroup sli
 * @brief Write a COMMON_SET_DUMP_LOCATION command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param query Zero to set dump location, non-zero to query dump size
 * @param is_buffer_list Set to one if the buffer is a set of buffer descriptors or
 *                       set to 0 if the buffer is a contiguous dump area.
 * @param buffer DMA structure to which the dump will be copied.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_set_dump_location(sli4_t *sli4, void *buf, size_t size,
				 uint8_t query, uint8_t is_buffer_list,
				 ocs_dma_t *buffer, uint8_t fdb)
{
	sli4_req_common_set_dump_location_t *set_dump_loc = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				sizeof (sli4_req_common_set_dump_location_t),
				NULL);
	}

	set_dump_loc = (sli4_req_common_set_dump_location_t *)((uint8_t *)buf + sli_config_off);

	set_dump_loc->hdr.opcode = SLI4_OPC_COMMON_SET_DUMP_LOCATION;
	set_dump_loc->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
	set_dump_loc->hdr.request_length = sizeof(sli4_req_common_set_dump_location_t) - sizeof(sli4_req_hdr_t);
	set_dump_loc->hdr.timeout = 0;
	set_dump_loc->hdr.version = 0;

	set_dump_loc->blp = is_buffer_list;
	set_dump_loc->qry = query;
	set_dump_loc->fdb = fdb;

	if (buffer) {
		set_dump_loc->buf_addr_low = ocs_addr32_lo(buffer->phys);
		set_dump_loc->buf_addr_high = ocs_addr32_hi(buffer->phys);
		set_dump_loc->buffer_length = buffer->len;
	} else {
		set_dump_loc->buf_addr_low = 0;
		set_dump_loc->buf_addr_high = 0;
		set_dump_loc->buffer_length = 0;
	}

	return(sli_config_off + sizeof(sli4_req_common_set_dump_location_t));
}

/**
 * @ingroup sli
 * @brief Write a RUN_BIU_DIAG command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param TX DMA buffer structure
 * @param TX DMA buffer size
 * @param RX DMA buffer structure
 * @param RX DMA buffer size
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_config_run_biu_diag(sli4_t *sli4, void *buf, size_t size,
			    ocs_dma_t *tx_buffer, size_t tx_buf_len,
			    ocs_dma_t *rx_buffer, size_t rx_buf_len)
{
	sli4_cmd_config_run_biu_diag_t *req = buf;

	if (!sli4 || !buf) {
		ocs_log_err(NULL, "Invalid parameters\n");
		return 0;
	}

	if (!tx_buffer || !rx_buffer) {
		ocs_log_err(sli4->os, "Invalid Tx/Rx buffers\n");
		return 0;
	}
	ocs_memset(buf, 0, size);

	req->hdr.command = SLI4_OPC_COMMON_RUN_BIU_DIAG;

	req->tx_phy_addr_low = ocs_addr32_lo(tx_buffer->phys);
	req->tx_phy_addr_high = ocs_addr32_hi(tx_buffer->phys);
	req->tx_buf_len = tx_buf_len;

	req->rx_phy_addr_low = ocs_addr32_lo(rx_buffer->phys);
	req->rx_phy_addr_high = ocs_addr32_hi(rx_buffer->phys);
	req->rx_buf_len = rx_buf_len;

	return(sizeof(sli4_cmd_config_run_biu_diag_t));
}
/**
 * @ingroup sli
 * @brief Write a  SLI4_OPC_FCOE_SET_LINK_DIAG_LOOPBACK command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param type Loopback type.
 *
 * @return Returns the number of bytes written.
 */

int32_t
sli_cmd_fcoe_set_loopback_mode(sli4_t *sli4, void *buf, size_t size, int type)
{
	sli4_req_common_set_loopback_mode_t *set_loopback_mode = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				sizeof (sli4_req_common_set_loopback_mode_t),
				NULL);
	}

	set_loopback_mode = (sli4_req_common_set_loopback_mode_t *)((uint8_t *)buf + sli_config_off);

	set_loopback_mode->hdr.opcode = SLI4_OPC_FCOE_SET_LINK_DIAG_LOOPBACK;
	set_loopback_mode->hdr.subsystem = SLI4_SUBSYSTEM_FCFCOE;
	set_loopback_mode->hdr.request_length = sizeof(sli4_req_common_set_loopback_mode_t) - sizeof(sli4_req_hdr_t);
	set_loopback_mode->hdr.timeout = 0;
	set_loopback_mode->hdr.version = 0;

	set_loopback_mode->typ = type; /* Loopback type */
	set_loopback_mode->link_number = sli4->config.port_number; /* Link number */
	set_loopback_mode->link_type = 0x1; /* FC Link */

	return(sli_config_off + sizeof(sli4_req_common_set_loopback_mode_t));
}

/**
 * @ingroup sli
 * @brief Write a SLI4_OPC_FC_SET_TRUNK_MODE command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param trunk_mode Trunk mode.
 *
 * @return Returns the number of bytes written or '-1' if an invalid trunk mode is requested.
 */
int32_t
sli_cmd_fc_set_trunk_mode(sli4_t *sli4, void *buf, size_t size, uint32_t trunk_mode)
{
	sli4_req_set_trunk_mode_t *req = NULL;
	uint32_t sli_config_off = 0;

	if (trunk_mode > SLI4_TRUNK_MODE_4_LINKS_PER_TRUNK)
		return -1;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				/* Payload length must accomodate both request and response */
				max(sizeof(sli4_req_set_trunk_mode_t),
				    sizeof(sli4_res_set_trunk_mode_t)),
				NULL);
	}

	req = (sli4_req_set_trunk_mode_t *)((uint8_t *)buf + sli_config_off);
	req->hdr.opcode = SLI4_OPC_FC_SET_TRUNK_MODE;
	req->hdr.subsystem = SLI4_SUBSYSTEM_FCFCOE;
	req->hdr.request_length = sizeof(sli4_req_set_trunk_mode_t) - sizeof(sli4_req_hdr_t);
	req->trunk_mode = trunk_mode;

	return (sli_config_off + sizeof(sli4_req_set_trunk_mode_t));
}

/**
 * @ingroup sli
 * @brief Write a COMMON_SET_FEATURES command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param feature Feature to set.
 * @param param_len Length of the parameter (must be a multiple of 4 bytes).
 * @param parameter Pointer to the parameter value.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_set_features(sli4_t *sli4, void *buf, size_t size,
			    uint32_t feature,
			    uint32_t param_len,
			    void* parameter)
{
	sli4_req_common_set_features_t *cmd = NULL;
	uint32_t	sli_config_off = 0;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				sizeof (sli4_req_common_set_features_t),
				NULL);
	}

	cmd = (sli4_req_common_set_features_t *)((uint8_t *)buf + sli_config_off);

	cmd->hdr.opcode = SLI4_OPC_COMMON_SET_FEATURES;
	cmd->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
	cmd->hdr.request_length = sizeof(sli4_req_common_set_features_t) - sizeof(sli4_req_hdr_t);
	cmd->hdr.timeout = 0;
	cmd->hdr.version = 0;

	cmd->feature = feature;
	cmd->param_len = param_len;
	ocs_memcpy(cmd->params, parameter, MIN(param_len, sizeof(cmd->params)));

	return(sli_config_off + sizeof(sli4_req_common_set_features_t));
}

/**
 * @ingroup sli
 * @brief Write a COMMON_COMMON_GET_PROFILE_CONFIG command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size in bytes.
 * @param dma DMA capable memory used to retrieve profile.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_get_profile_config(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *dma)
{
        sli4_req_common_get_profile_config_t *req = NULL;
	uint32_t	sli_config_off = 0;
	uint32_t	payload_size;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
				sizeof (sli4_req_common_get_profile_config_t),
				dma);
	}

	if (dma != NULL) {
		req = dma->virt;
		ocs_memset(req, 0, dma->size);
		payload_size = dma->size;
	} else {
		req = (sli4_req_common_get_profile_config_t *)((uint8_t *)buf + sli_config_off);
		payload_size = sizeof(sli4_req_common_get_profile_config_t);
	}

        req->hdr.opcode = SLI4_OPC_COMMON_GET_PROFILE_CONFIG;
        req->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
        req->hdr.request_length = payload_size - sizeof(sli4_req_hdr_t);
        req->hdr.version = 1;

        return(sli_config_off + sizeof(sli4_req_common_get_profile_config_t));
}

/**
 * @ingroup sli
 * @brief Write a COMMON_COMMON_SET_PROFILE_CONFIG command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param dma DMA capable memory containing profile.
 * @param profile_id Profile ID to configure.
 * @param descriptor_count Number of descriptors in DMA buffer.
 * @param isap Implicit Set Active Profile value to use.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_set_profile_config(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *dma,
		uint8_t profile_id, uint32_t descriptor_count, uint8_t isap)
{
        sli4_req_common_set_profile_config_t *req = NULL;
        uint32_t cmd_off = 0;
        uint32_t payload_size;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		cmd_off = sli_cmd_sli_config(sli4, buf, size,
						    sizeof (sli4_req_common_set_profile_config_t),
						    dma);
	}

	if (dma != NULL) {
		req = dma->virt;
		ocs_memset(req, 0, dma->size);
		payload_size = dma->size;
	} else {
		req = (sli4_req_common_set_profile_config_t *)((uint8_t *)buf + cmd_off);
		payload_size = sizeof(sli4_req_common_set_profile_config_t);
	}

        req->hdr.opcode = SLI4_OPC_COMMON_SET_PROFILE_CONFIG;
        req->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
        req->hdr.request_length = payload_size - sizeof(sli4_req_hdr_t);
        req->hdr.version = 1;
        req->profile_id = profile_id;
        req->desc_count = descriptor_count;
        req->isap = isap;

        return(cmd_off + sizeof(sli4_req_common_set_profile_config_t));
}

/**
 * @ingroup sli
 * @brief Write a COMMON_COMMON_GET_PROFILE_LIST command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size in bytes.
 * @param start_profile_index First profile index to return.
 * @param dma Buffer into which the list will be written.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_get_profile_list(sli4_t *sli4, void *buf, size_t size,
                                   uint32_t start_profile_index, ocs_dma_t *dma)
{
        sli4_req_common_get_profile_list_t *req = NULL;
        uint32_t cmd_off = 0;
        uint32_t payload_size;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		cmd_off = sli_cmd_sli_config(sli4, buf, size,
					     sizeof (sli4_req_common_get_profile_list_t),
					     dma);
	}

	if (dma != NULL) {
		req = dma->virt;
		ocs_memset(req, 0, dma->size);
		payload_size = dma->size;
	} else {
		req = (sli4_req_common_get_profile_list_t *)((uint8_t *)buf + cmd_off);
		payload_size = sizeof(sli4_req_common_get_profile_list_t);
	}

        req->hdr.opcode = SLI4_OPC_COMMON_GET_PROFILE_LIST;
        req->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
        req->hdr.request_length = payload_size - sizeof(sli4_req_hdr_t);
        req->hdr.version = 0;

        req->start_profile_index = start_profile_index;

        return(cmd_off + sizeof(sli4_req_common_get_profile_list_t));
}

/**
 * @ingroup sli
 * @brief Write a COMMON_COMMON_GET_ACTIVE_PROFILE command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size in bytes.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_get_active_profile(sli4_t *sli4, void *buf, size_t size)
{
        sli4_req_common_get_active_profile_t *req = NULL;
        uint32_t cmd_off = 0;
        uint32_t payload_size;

        /* Payload length must accomodate both request and response */
        payload_size = max(sizeof(sli4_req_common_get_active_profile_t),
                           sizeof(sli4_res_common_get_active_profile_t));

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		cmd_off = sli_cmd_sli_config(sli4, buf, size,
				payload_size,
				NULL);
	}

        req = (sli4_req_common_get_active_profile_t *)
                ((uint8_t*)buf + cmd_off);

        req->hdr.opcode = SLI4_OPC_COMMON_GET_ACTIVE_PROFILE;
        req->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
        req->hdr.request_length = payload_size - sizeof(sli4_req_hdr_t);
        req->hdr.version = 0;

        return(cmd_off + sizeof(sli4_req_common_get_active_profile_t));
}

/**
 * @ingroup sli
 * @brief Write a COMMON_COMMON_SET_ACTIVE_PROFILE command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size in bytes.
 * @param fd If non-zero, set profile to factory default.
 * @param active_profile_id ID of new active profile.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_set_active_profile(sli4_t *sli4, void *buf, size_t size,
                                  uint32_t fd, uint32_t active_profile_id)
{
        sli4_req_common_set_active_profile_t *req = NULL;
        uint32_t cmd_off = 0;
        uint32_t payload_size;

        /* Payload length must accomodate both request and response */
        payload_size = max(sizeof(sli4_req_common_set_active_profile_t),
                           sizeof(sli4_res_common_set_active_profile_t));

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		cmd_off = sli_cmd_sli_config(sli4, buf, size,
				payload_size,
				NULL);
	}

        req = (sli4_req_common_set_active_profile_t *)
                ((uint8_t*)buf + cmd_off);

        req->hdr.opcode = SLI4_OPC_COMMON_SET_ACTIVE_PROFILE;
        req->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
        req->hdr.request_length = payload_size - sizeof(sli4_req_hdr_t);
        req->hdr.version = 0;
        req->fd = fd;
        req->active_profile_id = active_profile_id;

        return(cmd_off + sizeof(sli4_req_common_set_active_profile_t));
}

/**
 * @ingroup sli
 * @brief Write a COMMON_GET_RECONFIG_LINK_INFO command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size in bytes.
 * @param dma Buffer to store the supported link configuration modes from the physical device.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_get_reconfig_link_info(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *dma)
{
        sli4_req_common_get_reconfig_link_info_t *req = NULL;
        uint32_t cmd_off = 0;
        uint32_t payload_size;

        /* Payload length must accomodate both request and response */
        payload_size = max(sizeof(sli4_req_common_get_reconfig_link_info_t),
                           sizeof(sli4_res_common_get_reconfig_link_info_t));

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		cmd_off = sli_cmd_sli_config(sli4, buf, size,
				payload_size,
				dma);
	}

	if (dma != NULL) {
		req = dma->virt;
		ocs_memset(req, 0, dma->size);
		payload_size = dma->size;
	} else {
		req = (sli4_req_common_get_reconfig_link_info_t *)((uint8_t *)buf + cmd_off);
		payload_size = sizeof(sli4_req_common_get_reconfig_link_info_t);
	}

        req->hdr.opcode = SLI4_OPC_COMMON_GET_RECONFIG_LINK_INFO;
        req->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
        req->hdr.request_length = payload_size - sizeof(sli4_req_hdr_t);
        req->hdr.version = 0;

        return(cmd_off + sizeof(sli4_req_common_get_reconfig_link_info_t));
}

/**
 * @ingroup sli
 * @brief Write a COMMON_SET_RECONFIG_LINK_ID command.
 *
 * @param sli4 SLI context.
 * @param buf destination buffer for the command.
 * @param size buffer size in bytes.
 * @param fd If non-zero, set link config to factory default.
 * @param active_link_config_id ID of new active profile.
 * @param dma Buffer to assign the link configuration mode that is to become active from the physical device.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_set_reconfig_link_id(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *dma,
                                  uint32_t fd, uint32_t active_link_config_id)
{
        sli4_req_common_set_reconfig_link_id_t *req = NULL;
        uint32_t cmd_off = 0;
        uint32_t payload_size;

        /* Payload length must accomodate both request and response */
        payload_size = max(sizeof(sli4_req_common_set_reconfig_link_id_t),
                           sizeof(sli4_res_common_set_reconfig_link_id_t));

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		cmd_off = sli_cmd_sli_config(sli4, buf, size,
				payload_size,
				NULL);
	}

		if (dma != NULL) {
		req = dma->virt;
		ocs_memset(req, 0, dma->size);
		payload_size = dma->size;
	} else {
		req = (sli4_req_common_set_reconfig_link_id_t *)((uint8_t *)buf + cmd_off);
		payload_size = sizeof(sli4_req_common_set_reconfig_link_id_t);
	}

        req->hdr.opcode = SLI4_OPC_COMMON_SET_RECONFIG_LINK_ID;
        req->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
        req->hdr.request_length = payload_size - sizeof(sli4_req_hdr_t);
        req->hdr.version = 0;
        req->fd = fd;
        req->next_link_config_id = active_link_config_id;

        return(cmd_off + sizeof(sli4_req_common_set_reconfig_link_id_t));
}


/**
 * @ingroup sli
 * @brief Check the mailbox/queue completion entry.
 *
 * @param buf Pointer to the MCQE.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_cqe_mq(void *buf)
{
	sli4_mcqe_t	*mcqe = buf;

	/*
	 * Firmware can split mbx completions into two MCQEs: first with only
	 * the "consumed" bit set and a second with the "complete" bit set.
	 * Thus, ignore MCQE unless "complete" is set.
	 */
	if (!mcqe->cmp) {
		ocs_log_err(NULL, "Ignoring MCQE, completion bit not set\n");
		return -2;
	}

	if (mcqe->completion_status) {
		ocs_log_err(NULL, "MCQE: bad status (cmpl=%#x ext=%#x con=%d cmp=%d ae=%d val=%d)\n",
			mcqe->completion_status, mcqe->extended_status,
			mcqe->con, mcqe->cmp, mcqe->ae, mcqe->val);
		ocs_log_err(NULL, "      %08X %08X %08X %08X\n", ((uint32_t *)buf)[0],
			((uint32_t *)buf)[1], ((uint32_t *)buf)[2], ((uint32_t *)buf)[3]);
	}

	return mcqe->completion_status;
}

/**
 * @ingroup sli
 * @brief Check the asynchronous event completion entry.
 *
 * @param sli4 SLI context.
 * @param buf Pointer to the ACQE.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_cqe_async(sli4_t *sli4, void *buf)
{
	sli4_acqe_t	*acqe = buf;
	int32_t		rc = -1;

	if (!sli4 || !buf) {
		ocs_log_err(NULL, "bad parameter sli4=%p buf=%p\n", sli4, buf);
		return -1;
	}

	switch (acqe->event_code) {
	case SLI4_ACQE_EVENT_CODE_LINK_STATE:
		rc = sli_fc_process_link_state(sli4, buf);
		break;
	case SLI4_ACQE_EVENT_CODE_FCOE_FIP:
		rc = sli_fc_process_fcoe(sli4, buf);
		break;
	case SLI4_ACQE_EVENT_CODE_GRP_5:
		/*TODO*/
		ocs_log_debug(sli4->os, "ACQE GRP5\n");
		break;
	case SLI4_ACQE_EVENT_CODE_SLI_PORT_EVENT:
		ocs_log_debug(sli4->os,"ACQE SLI Port, type=0x%x, data1,2=0x%08x,0x%08x\n",
			      acqe->event_type, acqe->event_data[0], acqe->event_data[1]);
#if defined(OCS_INCLUDE_DEBUG)
		ocs_dump32(OCS_DEBUG_ALWAYS, sli4->os, "acq", acqe, sizeof(*acqe));
#endif
		rc = sli_fc_process_sli_port_event(sli4, buf);
		break;

	case SLI4_ACQE_EVENT_CODE_FC_LINK_EVENT:
		rc = sli_fc_process_link_attention(sli4, buf);
		break;
	default:
		/*TODO*/
		ocs_log_test(sli4->os, "ACQE unknown=%#x\n", acqe->event_code);
	}

	return rc;
}

/**
 * @brief Check the SLI_CONFIG response.
 *
 * @par Description
 * Function checks the SLI_CONFIG response and the payload status.
 *
 * @param buf Pointer to SLI_CONFIG response.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
sli_res_sli_config(void *buf)
{
	sli4_cmd_sli_config_t	*sli_config = buf;

	// sanity check
	if (!buf || (SLI4_MBOX_COMMAND_SLI_CONFIG != sli_config->hdr.command)) {
		ocs_log_err(NULL, "bad parameter buf=%p cmd=%#x\n", buf, (buf ? sli_config->hdr.command : -1));
		return -1;
	}

	if (sli_config->hdr.status) {
		return sli_config->hdr.status;
	}

	if (sli_config->emb) {
		return sli_config->payload.embed[4];
	} else {
		ocs_log_test(NULL, "external buffers not supported\n");
		return -1;
	}
}

/**
 * @brief Issue a COMMON_FUNCTION_RESET command.
 *
 * @param sli4 SLI context.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
sli_common_function_reset(sli4_t *sli4)
{

	if (sli_cmd_common_function_reset(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE)) {
		if (sli_bmbx_command(sli4)) {
			ocs_log_crit(sli4->os, "bootstrap mailbox write fail (COM_FUNC_RESET)\n");
			return -1;
		}
		if (sli_res_sli_config(sli4->bmbx.virt)) {
			ocs_log_err(sli4->os, "bad status COM_FUNC_RESET\n");
			return -1;
		}
	} else {
		ocs_log_err(sli4->os, "bad COM_FUNC_RESET write\n");
		return -1;
	}

	return 0;
}


/**
 * @brief check to see if the FW is ready.
 *
 * @par Description
 * Based on <i>SLI-4 Architecture Specification, Revision 4.x0-13 (2012).</i>.
 *
 * @param sli4 SLI context.
 * @param timeout_ms Time, in milliseconds, to wait for the port to be ready
 *  before failing.
 *
 * @return Returns TRUE for ready, or FALSE otherwise.
 */
static int32_t
sli_wait_for_fw_ready(sli4_t *sli4, uint32_t timeout_ms)
{
	uint32_t	iter = timeout_ms / (SLI4_INIT_PORT_DELAY_US / 1000);
	uint32_t	ready = FALSE;

	do {
		iter--;
		ocs_delay_usec(SLI4_INIT_PORT_DELAY_US);	/* 10 ms */
		if (sli_fw_ready(sli4) == 1) {
			ready = TRUE;
		}
	} while (!ready && (iter > 0));

	return ready;
}

/**
 * @brief Initialize the firmware.
 *
 * @par Description
 * Based on <i>SLI-4 Architecture Specification, Revision 4.x0-13 (2012).</i>.
 *
 * @param sli4 SLI context.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
sli_fw_init(sli4_t *sli4)
{
	uint32_t ready;
	uint32_t endian;

	/*
	 * Is firmware ready for operation?
	 */
	ready = sli_wait_for_fw_ready(sli4, SLI4_FW_READY_TIMEOUT_MSEC);
	if (!ready) {
		ocs_log_crit(sli4->os, "FW status is NOT ready\n");
		return -1;
	}

	/*
	 * Reset port to a known state
	 */
	switch (sli4->if_type) {
	case SLI4_IF_TYPE_BE3_SKH_PF:
	case SLI4_IF_TYPE_BE3_SKH_VF:
		/* No SLIPORT_CONTROL register so use command sequence instead */
		if (sli_bmbx_wait(sli4, SLI4_BMBX_DELAY_US)) {
			ocs_log_crit(sli4->os, "bootstrap mailbox not ready\n");
			return -1;
		}

		if (sli_cmd_fw_initialize(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE)) {
			if (sli_bmbx_command(sli4)) {
				ocs_log_crit(sli4->os, "bootstrap mailbox write fail (FW_INIT)\n");
				return -1;
			}
		} else {
			ocs_log_crit(sli4->os, "bad FW_INIT write\n");
			return -1;
		}

		if (sli_common_function_reset(sli4)) {
			ocs_log_err(sli4->os, "bad COM_FUNC_RESET write\n");
			return -1;
		}
		break;
	case SLI4_IF_TYPE_LANCER_FC_ETH:
	case SLI4_IF_TYPE_LANCER_G7:
#if BYTE_ORDER == LITTLE_ENDIAN
		endian = SLI4_SLIPORT_CONTROL_LITTLE_ENDIAN;
#else
		endian = SLI4_SLIPORT_CONTROL_BIG_ENDIAN;
#endif

		if (sli_sliport_control(sli4, endian))
			return -1;
		break;
	default:
		ocs_log_test(sli4->os, "if_type %d not supported\n", sli4->if_type);
		return -1;
	}

	return 0;
}

/**
 * @brief Terminate the firmware.
 *
 * @param sli4 SLI context.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
sli_fw_term(sli4_t *sli4)
{
	uint32_t endian;

	if (sli4->if_type == SLI4_IF_TYPE_BE3_SKH_PF ||
	    sli4->if_type == SLI4_IF_TYPE_BE3_SKH_VF) {
		/* No SLIPORT_CONTROL register so use command sequence instead */
		if (sli_bmbx_wait(sli4, SLI4_BMBX_DELAY_US)) {
			ocs_log_crit(sli4->os, "bootstrap mailbox not ready\n");
			return -1;
		}

		if (sli_common_function_reset(sli4)) {
			ocs_log_err(sli4->os, "bad COM_FUNC_RESET write\n");
			return -1;
		}

		if (sli_cmd_fw_deinitialize(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE)) {
			if (sli_bmbx_command(sli4)) {
				ocs_log_crit(sli4->os, "bootstrap mailbox write fail (FW_DEINIT)\n");
				return -1;
			}
		} else {
			ocs_log_test(sli4->os, "bad FW_DEINIT write\n");
			return -1;
		}
	} else {
#if BYTE_ORDER == LITTLE_ENDIAN
		endian = SLI4_SLIPORT_CONTROL_LITTLE_ENDIAN;
#else
		endian = SLI4_SLIPORT_CONTROL_BIG_ENDIAN;
#endif
		// type 2 etc. use SLIPORT_CONTROL to initialize port
		sli_sliport_control(sli4, endian);
	}
	return 0;
}

/**
 * @brief Write the doorbell register associated with the queue object.
 *
 * @param sli4 SLI context.
 * @param q Queue object.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
sli_queue_doorbell(sli4_t *sli4, sli4_queue_t *q)
{
	uint32_t	val = 0;

	switch (q->type) {
	case SLI_QTYPE_EQ:
		if (sli4->if_type == SLI4_IF_TYPE_LANCER_G7)
			val = sli_iftype6_eq_doorbell(q->n_posted, q->id, FALSE);
		else
			val = sli_eq_doorbell(q->n_posted, q->id, FALSE);

		ocs_reg_write32(sli4->os, q->doorbell_rset, q->doorbell_offset, val);
		break;
	case SLI_QTYPE_CQ:
		if (sli4->if_type == SLI4_IF_TYPE_LANCER_G7)
			val = sli_iftype6_cq_doorbell(q->n_posted, q->id, FALSE);
		else
			val = sli_cq_doorbell(q->n_posted, q->id, FALSE);

		ocs_reg_write32(sli4->os, q->doorbell_rset, q->doorbell_offset, val);
		break;
	case SLI_QTYPE_MQ:
		val = SLI4_MQ_DOORBELL(q->n_posted, q->id);
		ocs_reg_write32(sli4->os, q->doorbell_rset, q->doorbell_offset, val);
		break;
	case SLI_QTYPE_RQ:
	{
		uint32_t	n_posted = q->n_posted;
		/*
		 * FC/FCoE has different rules for Receive Queues. The host
		 * should only update the doorbell of the RQ-pair containing
		 * the headers since the header / payload RQs are treated
		 * as a matched unit.
		 */
		if (SLI4_PORT_TYPE_FC == sli4->port_type) {
			/*
			 * In RQ-pair, an RQ either contains the FC header
			 * (i.e. is_hdr == TRUE) or the payload.
			 *
			 * Don't ring doorbell for payload RQ
			 */
			if (!q->u.flag.is_hdr) {
				break;
			}
			/*
			 * Some RQ cannot be incremented one entry at a time. Instead,
			 * the driver collects a number of entries and updates the
			 * RQ in batches.
			 */
			if (q->u.flag.rq_batch) {
				if (((q->index + q->n_posted) % SLI4_QUEUE_RQ_BATCH)) {
					break;
				}
				n_posted = SLI4_QUEUE_RQ_BATCH;
			}
		}

		val = SLI4_RQ_DOORBELL(n_posted, q->id);
		ocs_reg_write32(sli4->os, q->doorbell_rset, q->doorbell_offset, val);
		break;
	}
	case SLI_QTYPE_WQ:
		if (sli4->if_type == SLI4_IF_TYPE_LANCER_G7) {
			if (q->u.dpp.enabled) {
				/* Ring the doorbell with dpp bit set + dpp id + q id */
				val = SLI4_DPP_WQ_DOORBELL(q->n_posted, q->u.dpp.id, q->id);
			} else {
				/* non-dpp write for iftype = 6 */
				val = SLI4_WQ_DOORBELL(q->n_posted, 0, q->id);
			}
		} else {
			/* For iftype = 2 and 3, q->index value is ignored */
			val = SLI4_WQ_DOORBELL(q->n_posted, q->index, q->id);
		}

		ocs_reg_write32(sli4->os, q->doorbell_rset, q->doorbell_offset, val);
		break;
	default:
		ocs_log_test(sli4->os, "bad queue type %d\n", q->type);
		return -1;
	}

	return 0;
}

static int32_t
sli_request_features(sli4_t *sli4, sli4_features_t *features, uint8_t query)
{
	if (sli_cmd_request_features(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE,
				*features, query)) {
		sli4_cmd_request_features_t *req_features = sli4->bmbx.virt;

		if (sli_bmbx_command(sli4)) {
			ocs_log_crit(sli4->os, "bootstrap mailbox write fail (REQUEST_FEATURES)\n");
			return -1;
		}

		if (req_features->hdr.status) {
			ocs_log_err(sli4->os, "REQUEST_FEATURES bad status %#x\n", req_features->hdr.status);
			return -1;
		}

		features->dword = req_features->response.dword;
	} else {
		ocs_log_err(sli4->os, "bad REQUEST_FEATURES write\n");
		return -1;
	}

	return 0;
}

/**
 * @brief Calculate max queue entries.
 *
 * @param sli4 SLI context.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
void
sli_calc_max_qentries(sli4_t *sli4)
{
	sli4_qtype_e q;
	uint32_t alloc_size, qentries, qentry_size;

	for (q = SLI_QTYPE_EQ; q < SLI_QTYPE_MAX; q++) {
		sli4->config.max_qentries[q] = sli_convert_mask_to_count(sli4->config.count_method[q],
									 sli4->config.count_mask[q]);
	}

	/* single, continguous DMA allocations will be called for each queue
	 * of size (max_qentries * queue entry size); since these can be large,
	 * check against the OS max DMA allocation size
	 */
	for (q = SLI_QTYPE_EQ; q < SLI_QTYPE_MAX; q++) {
		qentries = sli4->config.max_qentries[q];
		qentry_size = sli_get_queue_entry_size(sli4, q);
		alloc_size = qentries * qentry_size;
		if (alloc_size > ocs_max_dma_alloc(sli4->os, SLI_PAGE_SIZE)) {
			while (alloc_size > ocs_max_dma_alloc(sli4->os, SLI_PAGE_SIZE)) {
				/* cut the qentries in half until alloc_size <= max DMA alloc size */
				qentries >>= 1;
				alloc_size = qentries * qentry_size;
			}
			ocs_log_debug(sli4->os, "[%s]: max_qentries from %d to %d (max dma %d)\n",
				SLI_QNAME[q], sli4->config.max_qentries[q],
				qentries, ocs_max_dma_alloc(sli4->os, SLI_PAGE_SIZE));
			sli4->config.max_qentries[q] = qentries;
		}
	}
}

/**
 * @brief Issue a FW_CONFIG mailbox command and store the results.
 *
 * @param sli4 SLI context.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
sli_query_fw_config(sli4_t *sli4)
{
	/*
	 * Read the device configuration
	 *
	 * Note: Only ulp0 fields contain values
	 */
	if (sli_cmd_common_query_fw_config(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE)) {
		sli4_res_common_query_fw_config_t   *fw_config =
			(sli4_res_common_query_fw_config_t *)
			(((uint8_t *)sli4->bmbx.virt) + offsetof(sli4_cmd_sli_config_t, payload.embed));

		if (sli_bmbx_command(sli4)) {
			ocs_log_crit(sli4->os, "bootstrap mailbox write fail (QUERY_FW_CONFIG)\n");
			return -1;
		}

		if (fw_config->hdr.status) {
			ocs_log_err(sli4->os, "COMMON_QUERY_FW_CONFIG bad status %#x\n", fw_config->hdr.status);
			return -1;
		}

		sli4->physical_port = fw_config->physical_port;
		sli4->config.dual_ulp_capable = ((fw_config->function_mode & SLI4_FUNCTION_MODE_DUA_MODE) == 0 ? 0 : 1);
		sli4->config.is_ulp_fc[0] = ((fw_config->ulp0_mode &
					      (SLI4_ULP_MODE_FCOE_INI |
					       SLI4_ULP_MODE_FCOE_TGT)) == 0 ? 0 : 1);
		sli4->config.is_ulp_fc[1] = ((fw_config->ulp1_mode &
					      (SLI4_ULP_MODE_FCOE_INI |
					       SLI4_ULP_MODE_FCOE_TGT)) == 0 ? 0 : 1);

		if (sli4->config.dual_ulp_capable) {
			/*
			 * Lancer will not support this, so we use the values
			 * from the READ_CONFIG.
			 */
			if (sli4->config.is_ulp_fc[0] &&
			    sli4->config.is_ulp_fc[1]) {
				sli4->config.max_qcount[SLI_QTYPE_WQ] = fw_config->ulp0_toe_wq_total + fw_config->ulp1_toe_wq_total;
				sli4->config.max_qcount[SLI_QTYPE_RQ] = fw_config->ulp0_toe_defrq_total + fw_config->ulp1_toe_defrq_total;
			} else if (sli4->config.is_ulp_fc[0]) {
				sli4->config.max_qcount[SLI_QTYPE_WQ] = fw_config->ulp0_toe_wq_total;
				sli4->config.max_qcount[SLI_QTYPE_RQ] = fw_config->ulp0_toe_defrq_total;
			} else {
				sli4->config.max_qcount[SLI_QTYPE_WQ] = fw_config->ulp1_toe_wq_total;
				sli4->config.max_qcount[SLI_QTYPE_RQ] = fw_config->ulp1_toe_defrq_total;
			}
		}
	} else {
		ocs_log_err(sli4->os, "bad QUERY_FW_CONFIG write\n");
		return -1;
	}
	return 0;
}

#define SLI4_MIN_XRI_FOR_DISCOVERY 100

static int32_t
sli_get_config(sli4_t *sli4)
{
	ocs_dma_t	get_cntl_addl_data;

	/*
	 * Read the device configuration
	 */
	if (sli_cmd_read_config(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE)) {
		sli4_res_read_config_t	*read_config = sli4->bmbx.virt;
		uint32_t	i;
		uint32_t	total;

		if (sli_bmbx_command(sli4)) {
			ocs_log_crit(sli4->os, "bootstrap mailbox write fail (READ_CONFIG)\n");
			return -1;
		}

		if (read_config->hdr.status) {
			ocs_log_err(sli4->os, "READ_CONFIG bad status %#x\n", read_config->hdr.status);
			return -1;
		}

		sli4->config.has_extents = read_config->ext;
		if (FALSE == sli4->config.has_extents) {
			uint32_t	i = 0;
			uint32_t	*base = sli4->config.extent[0].base;

			if (!base) {
				if (NULL == (base = ocs_malloc(sli4->os, SLI_RSRC_MAX * sizeof(uint32_t),
								OCS_M_ZERO | OCS_M_NOWAIT))) {
					ocs_log_err(sli4->os, "memory allocation failed for sli4_resource_t\n");
					return -1;
				}
			}

			for (i = 0; i < SLI_RSRC_MAX; i++) {
				sli4->config.extent[i].number = 1;
				sli4->config.extent[i].n_alloc = 0;
				sli4->config.extent[i].base = &base[i];
				sli4->config.extent[i].nvme_size = 0;
			}

			sli4->config.extent[SLI_RSRC_FCOE_VFI].base[0] = read_config->vfi_base;
			sli4->config.extent[SLI_RSRC_FCOE_VFI].size = read_config->vfi_count;

			sli4->config.extent[SLI_RSRC_FCOE_VPI].base[0] = read_config->vpi_base;
			sli4->config.extent[SLI_RSRC_FCOE_VPI].size = read_config->vpi_count;

			sli4->config.extent[SLI_RSRC_FCOE_RPI].base[0] = read_config->rpi_base;
			sli4->config.extent[SLI_RSRC_FCOE_RPI].size = read_config->rpi_count;
			sli4->config.extent[SLI_RSRC_FCOE_XRI].base[0] = read_config->xri_base;
			if (ocs_dual_protocol_enabled(sli4->os)) {
				sli4->config.extent[SLI_RSRC_FCOE_XRI].size = read_config->xri_count / 2;
				sli4->config.extent[SLI_RSRC_FCOE_XRI].nvme_size = 
					read_config->xri_count - sli4->config.extent[SLI_RSRC_FCOE_XRI].size;
			} else if (ocs_scsi_protocol_enabled(sli4->os)) {	
				sli4->config.extent[SLI_RSRC_FCOE_XRI].size = read_config->xri_count;
			} else {
				/* OCS only used for discovery */
				sli4->config.extent[SLI_RSRC_FCOE_XRI].size = SLI4_MIN_XRI_FOR_DISCOVERY;
				sli4->config.extent[SLI_RSRC_FCOE_XRI].nvme_size = 
					read_config->xri_count - SLI4_MIN_XRI_FOR_DISCOVERY;
			}

			ocs_log_info(sli4->os, "SLI4 SCSI/DISC Xri base = %d cnt = %d\n",
				     read_config->xri_base, sli4->config.extent[SLI_RSRC_FCOE_XRI].size);

			sli4->config.extent[SLI_RSRC_FCOE_FCFI].base[0] = 0;
			sli4->config.extent[SLI_RSRC_FCOE_FCFI].size = read_config->fcfi_count;
		} else {
			// TODO extents
			;
		}

		for (i = 0; i < SLI_RSRC_MAX; i++) {
			total = sli4->config.extent[i].number * sli4->config.extent[i].size;
			sli4->config.extent[i].use_map = ocs_bitmap_alloc(total);
			if (NULL == sli4->config.extent[i].use_map) {
				ocs_log_err(sli4->os, "bitmap memory allocation failed resource %d\n", i);
				return -1;
			}

			sli4->config.extent[i].map_size = total;
			ocs_lock_init(sli4->os, &sli4->config.extent[i].lock, "rsrc_lock-%d", i);
		}

		sli4->config.topology = read_config->topology;
		switch (sli4->config.topology) {
		case SLI4_READ_CFG_TOPO_FCOE:
			ocs_log_debug(sli4->os, "FCoE\n");
			break;
		case SLI4_READ_CFG_TOPO_FC:
			ocs_log_debug(sli4->os, "FC (unknown)\n");
			break;
		case SLI4_READ_CFG_TOPO_FC_DA:
			ocs_log_debug(sli4->os, "FC (direct attach)\n");
			break;
		case SLI4_READ_CFG_TOPO_FC_AL:
			ocs_log_debug(sli4->os, "FC (arbitrated loop)\n");
			break;
		default:
			ocs_log_test(sli4->os, "bad topology %#x\n", sli4->config.topology);
		}

		sli4->config.e_d_tov = read_config->e_d_tov;
		sli4->config.r_a_tov = read_config->r_a_tov;

		if (read_config->bbscn_def) {
			sli4->config.enable_bbcr = TRUE;
			sli4->config.bbscn_max = read_config->bbscn_max;
			sli4->config.bbscn_def = read_config->bbscn_def;
		}

		ocs_log_debug(sli4->os, "bbscn_max: %d, bbscn_def: %d\n",
			read_config->bbscn_max, read_config->bbscn_def);

		sli4->config.link_module_type = read_config->lmt;

		sli4->config.max_qcount[SLI_QTYPE_EQ] = read_config->eq_count;
		sli4->config.max_qcount[SLI_QTYPE_CQ] = read_config->cq_count;
		sli4->config.max_qcount[SLI_QTYPE_WQ] = read_config->wq_count;
		sli4->config.max_qcount[SLI_QTYPE_RQ] = read_config->rq_count;

		/*
		 * READ_CONFIG doesn't give the max number of MQ. Applications
		 * will typically want 1, but we may need another at some future
		 * date. Dummy up a "max" MQ count here.
		 */
		sli4->config.max_qcount[SLI_QTYPE_MQ] = SLI_USER_MQ_COUNT;
	} else {
		ocs_log_err(sli4->os, "bad READ_CONFIG write\n");
		return -1;
	}

	if (sli_cmd_common_get_sli4_parameters(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE)) {
		sli4_res_common_get_sli4_parameters_t	*parms = (sli4_res_common_get_sli4_parameters_t *)
			(((uint8_t *)sli4->bmbx.virt) + offsetof(sli4_cmd_sli_config_t, payload.embed));

		if (sli_bmbx_command(sli4)) {
			ocs_log_crit(sli4->os, "bootstrap mailbox write fail (COMMON_GET_SLI4_PARAMETERS)\n");
			return -1;
		} else if (parms->hdr.status) {
			ocs_log_err(sli4->os, "COMMON_GET_SLI4_PARAMETERS bad status %#x att'l %#x\n",
					parms->hdr.status, parms->hdr.additional_status);
			return -1;
		}

		sli4->config.auto_reg = parms->areg;
		sli4->config.auto_xfer_rdy = parms->agxf;
		sli4->config.tow = parms->tow;
		sli4->config.max_tow_xris = parms->dw20w0;

		sli4->config.hdr_template_req = parms->hdrr;
		sli4->config.t10_dif_inline_capable = parms->timm;
		sli4->config.t10_dif_separate_capable = parms->tsmm;
		sli4->config.fdd_present = parms->fdb;

		sli4->config.xib_capable = parms->xib;
		sli4->config.suppress_rsp_not_supported = parms->sriuns;
		sli4->config.nsler_capable = parms->nsler;

		sli4->config.mq_create_version = parms->mqv;
		sli4->config.cq_create_version = parms->cqv;
		sli4->config.rq_min_buf_size = parms->min_rq_buffer_size;
		sli4->config.rq_max_buf_size = parms->max_rq_buffer_size;

		sli4->config.qpage_count[SLI_QTYPE_EQ] = parms->eq_page_cnt;
		sli4->config.qpage_count[SLI_QTYPE_CQ] = parms->cq_page_cnt;
		sli4->config.qpage_count[SLI_QTYPE_MQ] = parms->mq_page_cnt;
		sli4->config.qpage_count[SLI_QTYPE_WQ] = parms->wq_page_cnt;
		sli4->config.qpage_count[SLI_QTYPE_RQ] = parms->rq_page_cnt;

		/* save count methods and masks for each queue type */
		sli4->config.count_mask[SLI_QTYPE_EQ] = parms->eqe_count_mask;
		sli4->config.count_method[SLI_QTYPE_EQ] = parms->eqe_count_method;
		sli4->config.count_mask[SLI_QTYPE_CQ] = parms->cqe_count_mask;
		sli4->config.count_method[SLI_QTYPE_CQ] = parms->cqe_count_method;
		sli4->config.count_mask[SLI_QTYPE_MQ] = parms->mqe_count_mask;
		sli4->config.count_method[SLI_QTYPE_MQ] = parms->mqe_count_method;
		sli4->config.count_mask[SLI_QTYPE_WQ] = parms->wqe_count_mask;
		sli4->config.count_method[SLI_QTYPE_WQ] = parms->wqe_count_method;
		sli4->config.count_mask[SLI_QTYPE_RQ] = parms->rqe_count_mask;
		sli4->config.count_method[SLI_QTYPE_RQ] = parms->rqe_count_method;

		/* now calculate max queue entries */
		sli_calc_max_qentries(sli4);

		sli4->config.max_sgl_pages = parms->sgl_page_cnt;	// max # of pages
		sli4->config.sgl_page_sizes = parms->sgl_page_sizes;	// bit map of available sizes
		// ignore HLM here. Use value from REQUEST_FEATURES
		sli4->config.sge_supported_length = parms->sge_supported_length;
		sli4->config.sgl_pre_registration_required = parms->sglr;
		// default to using pre-registered SGL's
		sli4->config.sgl_pre_registered = TRUE;

		sli4->config.perf_hint = parms->phon;
		sli4->config.perf_wq_id_association = parms->phwq;

		sli4->config.rq_batch = parms->rq_db_window;

		/* save the fields for skyhawk SGL chaining */
		sli4->config.sgl_chaining_params.chaining_capable =
			(parms->sglc == 1);
		sli4->config.sgl_chaining_params.frag_num_field_offset =
			parms->dw20w0;
		sli4->config.sgl_chaining_params.frag_num_field_mask =
			(1ull << parms->dw20w1) - 1;
		sli4->config.sgl_chaining_params.sgl_index_field_offset =
			parms->sgl_index_field_offset;
		sli4->config.sgl_chaining_params.sgl_index_field_mask =
			(1ull << parms->sgl_index_field_size) - 1;
		sli4->config.sgl_chaining_params.chain_sge_initial_value_lo =
			parms->chain_sge_initial_value_lo;
		sli4->config.sgl_chaining_params.chain_sge_initial_value_hi =
			parms->chain_sge_initial_value_hi;

		/* Use the highest available WQE size */
		if (parms->wqe_sizes & SLI4_128BYTE_WQE_SUPPORT)
			sli4->config.wqe_size = SLI4_WQE_EXT_BYTES;
		else
			sli4->config.wqe_size = SLI4_WQE_BYTES;
	}

	if (sli_query_fw_config(sli4)) {
		ocs_log_err(sli4->os, "Error sending QUERY_FW_CONFIG\n");
		return -1;
	}

	sli4->config.port_number = 0;

	/*
	 * Issue COMMON_GET_CNTL_ATTRIBUTES to get port_number. Temporarily
	 * uses VPD DMA buffer as the response won't fit in the embedded
	 * buffer.
	 */
	if (sli_cmd_common_get_cntl_attributes(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE, &sli4->vpd.data)) {
		sli4_res_common_get_cntl_attributes_t *attr = sli4->vpd.data.virt;

		if (sli_bmbx_command(sli4)) {
			ocs_log_crit(sli4->os, "bootstrap mailbox write fail (COMMON_GET_CNTL_ATTRIBUTES)\n");
			return -1;
		} else if (attr->hdr.status) {
			ocs_log_err(sli4->os, "COMMON_GET_CNTL_ATTRIBUTES bad status %#x att'l %#x\n",
					attr->hdr.status, attr->hdr.additional_status);
			return -1;
		}

		sli4->config.port_number = attr->port_number;

		ocs_memcpy(sli4->config.modeldesc, attr->description, sizeof(sli4->config.modeldesc));
		ocs_memcpy(sli4->config.bios_version_string, attr->bios_version_string,
				sizeof(sli4->config.bios_version_string));
	} else {
		ocs_log_err(sli4->os, "bad COMMON_GET_CNTL_ATTRIBUTES write\n");
		return -1;
	}

	if (ocs_dma_alloc(sli4->os, &get_cntl_addl_data, sizeof(sli4_res_common_get_cntl_addl_attributes_t),
			  OCS_MIN_DMA_ALIGNMENT)) {
		ocs_log_err(sli4->os, "Failed to allocate memory for GET_CNTL_ADDL_ATTR data\n");
	} else {
		if (sli_cmd_common_get_cntl_addl_attributes(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE,
							    &get_cntl_addl_data)) {
			sli4_res_common_get_cntl_addl_attributes_t *attr = get_cntl_addl_data.virt;

			if (sli_bmbx_command(sli4)) {
				ocs_log_crit(sli4->os, "bootstrap mailbox write fail (COMMON_GET_CNTL_ADDL_ATTRIBUTES)\n");
				ocs_dma_free(sli4->os, &get_cntl_addl_data);
				return -1;
			}
			if (attr->hdr.status) {
				ocs_log_err(sli4->os, "COMMON_GET_CNTL_ADDL_ATTRIBUTES bad status %#x\n",
					    attr->hdr.status);
				ocs_dma_free(sli4->os, &get_cntl_addl_data);
				return -1;
			}

			ocs_memcpy(sli4->config.ipl_name, attr->ipl_file_name, sizeof(sli4->config.ipl_name));

			ocs_log_debug(sli4->os, "IPL:%s \n", (char*)sli4->config.ipl_name);
		} else {
			ocs_log_err(sli4->os, "bad COMMON_GET_CNTL_ADDL_ATTRIBUTES write\n");
			ocs_dma_free(sli4->os, &get_cntl_addl_data);
			return -1;
		}

		ocs_dma_free(sli4->os, &get_cntl_addl_data);
	}

	if (sli_cmd_common_get_port_name(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE)) {
		sli4_res_common_get_port_name_t	*port_name = (sli4_res_common_get_port_name_t *)(((uint8_t *)sli4->bmbx.virt) +
			offsetof(sli4_cmd_sli_config_t, payload.embed));

		if (sli_bmbx_command(sli4)) {
			ocs_log_crit(sli4->os, "bootstrap mailbox write fail (COMMON_GET_PORT_NAME)\n");
			return -1;
		}

		sli4->config.port_name[0] = port_name->port_name[sli4->config.port_number];
	}
	sli4->config.port_name[1] = '\0';

	if (sli_cmd_read_rev(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE, &sli4->vpd.data)) {
		sli4_cmd_read_rev_t	*read_rev = sli4->bmbx.virt;

		if (sli_bmbx_command(sli4)) {
			ocs_log_crit(sli4->os, "bootstrap mailbox write fail (READ_REV)\n");
			return -1;
		}
		if (read_rev->hdr.status) {
			ocs_log_err(sli4->os, "READ_REV bad status %#x\n", read_rev->hdr.status);
			return -1;
		}

		sli4->config.fw_rev[0] = read_rev->first_fw_id;
		ocs_memcpy(sli4->config.fw_name[0],read_rev->first_fw_name, sizeof(sli4->config.fw_name[0]));

		sli4->config.fw_rev[1] = read_rev->second_fw_id;
		ocs_memcpy(sli4->config.fw_name[1],read_rev->second_fw_name, sizeof(sli4->config.fw_name[1]));

		sli4->config.hw_rev[0] = read_rev->first_hw_revision;
		sli4->config.hw_rev[1] = read_rev->second_hw_revision;
		sli4->config.hw_rev[2] = read_rev->third_hw_revision;

		ocs_log_info(sli4->os, "FW1:%s (%08x) / FW2:%s (%08x)\n",
				read_rev->first_fw_name, read_rev->first_fw_id,
				read_rev->second_fw_name, read_rev->second_fw_id);

		ocs_log_info(sli4->os, "HW1: %08x / HW2: %08x\n", read_rev->first_hw_revision,
				read_rev->second_hw_revision);

		/* Check that all VPD data was returned */
		if (read_rev->returned_vpd_length != read_rev->actual_vpd_length) {
			ocs_log_test(sli4->os, "VPD length: available=%d returned=%d actual=%d\n",
					read_rev->available_length,
					read_rev->returned_vpd_length,
					read_rev->actual_vpd_length);
		}
		sli4->vpd.length = read_rev->returned_vpd_length;
	} else {
		ocs_log_err(sli4->os, "bad READ_REV write\n");
		return -1;
	}

	if (sli_cmd_read_nvparms(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE)) {
		sli4_cmd_read_nvparms_t	*read_nvparms = sli4->bmbx.virt;

		if (sli_bmbx_command(sli4)) {
			ocs_log_crit(sli4->os, "bootstrap mailbox write fail (READ_NVPARMS)\n");
			return -1;
		}
		if (read_nvparms->hdr.status) {
			ocs_log_err(sli4->os, "READ_NVPARMS bad status %#x\n", read_nvparms->hdr.status);
			return -1;
		}

		ocs_memcpy(sli4->config.wwpn, read_nvparms->wwpn, sizeof(sli4->config.wwpn));
		ocs_memcpy(sli4->config.wwnn, read_nvparms->wwnn, sizeof(sli4->config.wwnn));

		ocs_log_debug(sli4->os, "WWPN %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
				sli4->config.wwpn[0],
				sli4->config.wwpn[1],
				sli4->config.wwpn[2],
				sli4->config.wwpn[3],
				sli4->config.wwpn[4],
				sli4->config.wwpn[5],
				sli4->config.wwpn[6],
				sli4->config.wwpn[7]);
		ocs_log_debug(sli4->os, "WWNN %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
				sli4->config.wwnn[0],
				sli4->config.wwnn[1],
				sli4->config.wwnn[2],
				sli4->config.wwnn[3],
				sli4->config.wwnn[4],
				sli4->config.wwnn[5],
				sli4->config.wwnn[6],
				sli4->config.wwnn[7]);
	} else {
		ocs_log_err(sli4->os, "bad READ_NVPARMS write\n");
		return -1;
	}

	return 0;
}

/****************************************************************************
 * Public functions
 */

/**
 * @ingroup sli
 * @brief Set up the SLI context.
 *
 * @param sli4 SLI context.
 * @param os Device abstraction.
 * @param port_type Protocol type of port (for example, FC and NIC).
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_setup(sli4_t *sli4, ocs_os_handle_t os, sli4_port_type_e port_type)
{
	uint32_t sli_intf;
	uint32_t pci_class_rev;
	uint32_t family;
	sli4_asic_family_t *asic_family;
	sli4_asic_revision_t *asic_revision;
	uint8_t rev_id;
	uint32_t i;

	ocs_memset(sli4, 0, sizeof(sli4_t));

	sli4->os = os;
	sli4->port_type = port_type;

	/*
	 * Read the SLI_INTF register to discover the register layout
	 * and other capability information
	 */
	sli_intf = ocs_config_read32(os, SLI4_INTF_REG);
	if (sli_intf_valid_check(sli_intf)) {
		ocs_log_err(os, "SLI_INTF is not valid\n");
		return -1;
	}

	/* driver only support SLI-4 */
	sli4->sli_rev = sli_intf_sli_revision(sli_intf);
	if (4 != sli4->sli_rev) {
		ocs_log_err(os, "Unsupported SLI revision (intf=%#x)\n", sli_intf);
		return -1;
	}

	sli4->sli_family = sli_intf_sli_family(sli_intf);
	sli4->if_type = sli_intf_if_type(sli_intf);

	/*
	 * set the ASIC type and revision
	 */
	pci_class_rev = ocs_config_read32(os, SLI4_PCI_CLASS_REVISION);
	rev_id = sli_pci_rev_id(pci_class_rev);
	family = sli4->sli_family;
	if (family == SLI4_FAMILY_CHECK_ASIC_TYPE) {
		uint32_t asic_id = ocs_config_read32(os, SLI4_ASIC_ID_REG);
		family = sli_asic_gen(asic_id);
	}

	for (i = 0, asic_family = sli4_asic_family_table;
	     i < ARRAY_SIZE(sli4_asic_family_table); i++, asic_family++) {
		if (family == asic_family->family) {
			sli4->asic_type = asic_family->type;
			break;
		}
	}

	if (!sli4->asic_type) {
		ocs_log_err(os, "Unsupported ASIC family %02x-%02x\n", family, rev_id);
		return -1;
	}

	sli4->asic_rev = SLI4_ASIC_REV_ANY;
	for (i = 0, asic_revision = sli4_asic_rev_table;
	     i < ARRAY_SIZE(sli4_asic_rev_table); i++, asic_revision++) {
		if (rev_id == asic_revision->rev_id) {
			sli4->asic_rev = asic_revision->rev;
			break;
		}
	}

	/*
	 * The bootstrap mailbox is equivalent to a MQ with a single 256 byte
	 * entry, a CQ with a single 16 byte entry, and no event queue.
	 * Alignment must be 16 bytes as the low order address bits in the
	 * address register are also control / status.
	 */
	if (ocs_dma_alloc(sli4->os, &sli4->bmbx, SLI4_BMBX_SIZE +
				sizeof(sli4_mcqe_t), 16)) {
		ocs_log_err(os, "bootstrap mailbox allocation failed\n");
		return -1;
	}

	if (sli4->bmbx.phys & SLI4_BMBX_MASK_LO) {
		ocs_log_err(os, "bad alignment for bootstrap mailbox\n");
		return -1;
	}

	ocs_log_debug(os, "bmbx v=%p p=0x%x %08x s=%zd\n", sli4->bmbx.virt,
		ocs_addr32_hi(sli4->bmbx.phys),
		ocs_addr32_lo(sli4->bmbx.phys),
		sli4->bmbx.size);

	// TODO 4096 is arbitrary. What should this value actually be?
	if (ocs_dma_alloc(sli4->os, &sli4->vpd.data, 4096/*TODO*/, 4096)) {
		/* Note that failure isn't fatal in this specific case */
		sli4->vpd.data.size = 0;
		ocs_log_test(os, "VPD buffer allocation failed\n");
	}

	if (sli_fw_init(sli4)) {
		ocs_log_err(sli4->os, "FW initialization failed\n");
		return -1;
	}

	if ((SLI4_IF_TYPE_LANCER_FC_ETH == sli4->if_type) ||
	    (SLI4_IF_TYPE_LANCER_G7 == sli4->if_type)) {
		ocs_log_debug(os, "status=%#x error1=%#x error2=%#x\n",
				sli_reg_read(sli4, SLI4_REG_SLIPORT_STATUS),
				sli_reg_read(sli4, SLI4_REG_SLIPORT_ERROR1),
				sli_reg_read(sli4, SLI4_REG_SLIPORT_ERROR2));
	}

	/*
	 * Set SRIOV default config, if requested.
	 * This command must be the first command to be executed.
	 */
	if (ocs_sriov_config_required(sli4->os) && sli_sriov_setup(sli4)) {
		ocs_log_err(sli4->os, "SRIOV setup failed\n");
		return -1;
	}

	/*
	 * Set one of fcpi(initiator), fcpt(target), fcpc(combined) to true
	 * in addition to any other desired features
	 */
	sli4->config.features.flag.iaab = TRUE;
	sli4->config.features.flag.npiv = TRUE;
	sli4->config.features.flag.dif = TRUE;
	sli4->config.features.flag.vf = TRUE;
	sli4->config.features.flag.fcpc = TRUE;
	sli4->config.features.flag.iaar = TRUE;
	sli4->config.features.flag.hlm = TRUE;
	sli4->config.features.flag.perfh = TRUE;
	sli4->config.features.flag.rxseq = TRUE;
	sli4->config.features.flag.rxri = TRUE;
	sli4->config.features.flag.mrqp = TRUE;
	sli4->config.features.flag.ashdr = TRUE;

	// use performance hints if available
	if (sli4->config.perf_hint) {
		sli4->config.features.flag.perfh = TRUE;
	}

	if (sli_request_features(sli4, &sli4->config.features, TRUE)) {
		return -1;
	}

	if (sli_get_config(sli4)) {
		return -1;
	}

	return 0;
}

bool
sli_feature_enabled(sli4_t *sli4, uint32_t feature)
{
	return (sli4->config.features.dword & feature);
}

void
sli_config_set_features(sli4_t *sli4, uint32_t features)
{
	sli4->config.features.dword = features;
}

uint32_t
sli_config_get_features(sli4_t *sli4)
{
	return sli4->config.features.dword;
}

int32_t
sli_init(sli4_t *sli4)
{
	if (sli4->config.has_extents) {
		/* TODO COMMON_ALLOC_RESOURCE_EXTENTS */
		ocs_log_warn(sli4->os, "Need to implement extent allocation\n");
		return -1;
	}

	return sli_request_features(sli4, &sli4->config.features, false);
}

int32_t
sli_reset(sli4_t *sli4)
{
	uint32_t	i;

	if (sli_fw_init(sli4)) {
		ocs_log_crit(sli4->os, "FW initialization failed\n");
		return -1;
	}

	if (sli4->config.extent[0].base) {
		ocs_free(sli4->os, sli4->config.extent[0].base, SLI_RSRC_MAX * sizeof(uint32_t));
		sli4->config.extent[0].base = NULL;
	}

	for (i = 0; i < SLI_RSRC_MAX; i++) {
		if (sli4->config.extent[i].use_map) {
			ocs_bitmap_free(sli4->config.extent[i].use_map);
			sli4->config.extent[i].use_map = NULL;
		}

		sli4->config.extent[i].base = NULL;
		ocs_lock_free(&sli4->config.extent[i].lock);
	}

	if (sli_get_config(sli4))
		return -1;

	return 0;
}

/**
 * @ingroup sli
 * @brief Issue a Firmware Reset.
 *
 * @par Description
 * Issues a Firmware Reset to the chip.  This reset affects the entire chip,
 * so all PCI function on the same PCI bus and device are affected.
 * @n @n This type of reset can be used to activate newly downloaded firmware.
 * @n @n The driver should be considered to be in an unknown state after this
 * reset and should be reloaded.
 *
 * @param sli4 SLI context.
 *
 * @return Returns 0 on success, or -1 otherwise.
 */

int32_t
sli_fw_reset(sli4_t *sli4)
{
	uint32_t val;
	uint32_t ready;

	/*
	 * Firmware must be ready before issuing the reset.
	 */
	ready = sli_wait_for_fw_ready(sli4, SLI4_FW_READY_TIMEOUT_MSEC);
	if (!ready) {
		ocs_log_crit(sli4->os, "FW status is NOT ready\n");
		return -1;
	}
	switch(sli4->if_type) {
	case SLI4_IF_TYPE_BE3_SKH_PF:
		/* BE3 / Skyhawk use PCICFG_SOFT_RESET_CSR */
		val = ocs_config_read32(sli4->os, SLI4_PCI_SOFT_RESET_CSR);
		val |= SLI4_PCI_SOFT_RESET_MASK;
		ocs_config_write32(sli4->os, SLI4_PCI_SOFT_RESET_CSR, val);
		break;
	case SLI4_IF_TYPE_LANCER_FC_ETH:
	case SLI4_IF_TYPE_LANCER_G7:
		/* Lancer uses PHYDEV_CONTROL */

		val = SLI4_PHYDEV_CONTROL_FRST;
		sli_reg_write(sli4, SLI4_REG_PHYSDEV_CONTROL, val);
		break;
	default:
		ocs_log_test(sli4->os, "Unexpected iftype %d\n", sli4->if_type);
		return -1;
		break;
	}

	/* wait for the FW to become ready after the reset */
	ready = sli_wait_for_fw_ready(sli4, SLI4_FW_READY_TIMEOUT_MSEC);
	if (!ready) {
		ocs_log_crit(sli4->os, "Failed to become ready after firmware reset\n");
		return -1;
	}
	return 0;
}

int32_t
sli_port_migration(sli4_t *sli4)
{
	uint32_t ready;
	uint32_t val;

	/*
	 * Firmware must be ready before setting port migration.
	 */
	ready = sli_wait_for_fw_ready(sli4, SLI4_FW_READY_TIMEOUT_MSEC);
	if (!ready) {
		ocs_log_crit(sli4->os, "FW status is NOT ready\n");
		return -1;
	}

	switch (sli4->if_type) {
	case SLI4_IF_TYPE_LANCER_G7:
		val = BIT(SLI4_PORT_MIGRATION_BIT + sli4->physical_port);
		ocs_log_debug(sli4->os, "PM: Setting SLI4_REG_PHYSDEV_CONTROL to val: 0x%x\n", val);
		sli_reg_write(sli4, SLI4_REG_PHYSDEV_CONTROL, val);
		break;
	default:
		ocs_log_test(sli4->os, "Unexpected iftype %d\n", sli4->if_type);
		return -1;
	}

	/* wait for the FW to become ready after the reset */
	ready = sli_wait_for_fw_ready(sli4, SLI4_FW_READY_TIMEOUT_MSEC);
	if (!ready) {
		ocs_log_crit(sli4->os, "Failed to become ready after firmware reset\n");
	}

	return 0;
}
/**
 * @ingroup sli
 * @brief Tear down a SLI context.
 *
 * @param sli4 SLI context.
 *
 * @return Returns 0 on success, or non-zero otherwise.
 */
int32_t
sli_teardown(sli4_t *sli4)
{
	uint32_t i;

	if (sli4->config.extent[0].base) {
		ocs_free(sli4->os, sli4->config.extent[0].base, SLI_RSRC_MAX * sizeof(uint32_t));
		sli4->config.extent[0].base = NULL;
	}

	for (i = 0; i < SLI_RSRC_MAX; i++) {
		if (sli4->config.has_extents) {
			/* TODO COMMON_DEALLOC_RESOURCE_EXTENTS */;
		}

		sli4->config.extent[i].base = NULL;
		ocs_bitmap_free(sli4->config.extent[i].use_map);
		sli4->config.extent[i].use_map = NULL;
		ocs_lock_free(&sli4->config.extent[i].lock);
	}

	if (sli_fw_term(sli4))
		ocs_log_err(sli4->os, "FW deinitialization failed\n");

	ocs_dma_free(sli4->os, &sli4->vpd.data);
	ocs_dma_free(sli4->os, &sli4->bmbx);

	return 0;
}

/**
 * @ingroup sli
 * @brief Register a callback for the given event.
 *
 * @param sli4 SLI context.
 * @param which Event of interest.
 * @param func Function to call when the event occurs.
 * @param arg Argument passed to the callback function.
 *
 * @return Returns 0 on success, or non-zero otherwise.
 */
int32_t
sli_callback(sli4_t *sli4, sli4_callback_e which, void *func, void *arg)
{

	if (!sli4 || !func || (which >= SLI4_CB_MAX)) {
		ocs_log_err(NULL, "bad parameter sli4=%p which=%#x func=%p\n", sli4, which, func);
		return -1;
	}

	switch (which) {
	case SLI4_CB_LINK:
		sli4->link = func;
		sli4->link_arg = arg;
		break;
	case SLI4_CB_FIP:
		sli4->fip = func;
		sli4->fip_arg = arg;
		break;
	case SLI4_CB_ERR:
		sli4->reset_pending = func;
		sli4->reset_pending_arg = arg;
		break;
	default:
		ocs_log_test(sli4->os, "unknown callback %#x\n", which);
		return -1;
	}

	return 0;
}

int32_t
sli_queue_dma_free(sli4_t *sli4, sli4_queue_t *q)
{
	int32_t rc = 0;
	uint32_t i;

#if defined(OCS_USPACE_SPDK)
	for (i = 0; i < SLI_Q_DMA_CHUNKS && q->dma[i].virt; i++) {
		q->dma[i].virt = NULL;
		q->dma[i].phys = 0;
		q->dma[i].size = 0;
	}

	if (q->dma_single_chunk.virt) {
		if (ocs_dma_free(sli4->os, &q->dma_single_chunk)) {
			ocs_log_err(sli4->os, "%s queue ID %d DMA free failed\n", SLI_QNAME[q->type], q->id);
			rc = -1;
		}
	}
#else
	for (i = 0; i < SLI_Q_DMA_CHUNKS && q->dma[i].virt; i++) {
		if (ocs_dma_free(sli4->os, &q->dma[i])) {
			ocs_log_err(sli4->os, "%s queue ID %d DMA[%d] free failed\n", SLI_QNAME[q->type], q->id, i);
			rc = -1;
		}
	}
#endif

	return rc;
}

static int32_t
sli_queue_dma_alloc(sli4_t *sli4, sli4_queue_t *q, size_t qmem_size, uint32_t qtype, uint32_t align)
{
	int32_t rc = 0;
	uint32_t i, qpage_size, num_dma_chunks;

	qpage_size = sli_get_qpage_size(qmem_size);
	num_dma_chunks = sli_page_count(qmem_size, qpage_size);

#if defined(OCS_USPACE_SPDK)
	if (ocs_dma_alloc(sli4->os, &q->dma_single_chunk, qmem_size, align)) {
		ocs_log_err(sli4->os, "%s DMA allocation failed\n", SLI_QNAME[qtype]);
		rc = -1;
		goto dma_err;
	}

	ocs_memset(q->dma_single_chunk.virt, 0, qmem_size);

	for (i = 0; i < num_dma_chunks; i++) {
		q->dma[i].virt = q->dma_single_chunk.virt + i * qpage_size;
		q->dma[i].phys = q->dma_single_chunk.phys + i * qpage_size;
		q->dma[i].size = qpage_size;
	}
#else
	for (i = 0; i < num_dma_chunks; i++) {
		if (ocs_dma_alloc(sli4->os, &q->dma[i], qpage_size, align)) {
			ocs_log_err(sli4->os, "%s DMA[%d] allocation failed\n", SLI_QNAME[qtype], i);
			rc = -1;
			goto dma_err;
		}

		ocs_memset(q->dma[i].virt, 0, qpage_size);
	}
#endif

dma_err:
	if (rc)
		sli_queue_dma_free(sli4, q);

	return rc;
}

/**
 * @ingroup sli
 * @brief Initialize a queue object.
 *
 * @par Description
 * This initializes the sli4_queue_t object members, including the underlying
 * DMA memory.
 *
 * @param sli4 SLI context.
 * @param q Pointer to queue object.
 * @param qtype Type of queue to create.
 * @param size Size of each entry.
 * @param n_entries Number of entries to allocate.
 * @param align Starting memory address alignment.
 *
 * @note Checks if using the existing DMA memory (if any) is possible. If not,
 * it frees the existing memory and re-allocates.
 *
 * @return Returns 0 on success, or non-zero otherwise.
 */
int32_t
__sli_queue_init(sli4_t *sli4, sli4_queue_t *q, uint32_t qtype,
		size_t size, uint32_t n_entries, uint32_t align)
{
	if ((!q->dma[0].size) || (size != q->size) || (n_entries != q->length)) {
		size_t qmem_size = (size * n_entries);

		/* 'qmem_size' should be power of 2 */
		ocs_hal_assert(ocs_power_of_2(qmem_size));

		sli_queue_dma_free(sli4, q);
		ocs_memset(q, 0, sizeof(sli4_queue_t));

		if (sli_queue_dma_alloc(sli4, q, qmem_size, qtype, align))
			return -1;

		ocs_lock_init(sli4->os, &q->lock, "%s lock[%d]", SLI_QNAME[qtype], ocs_instance(sli4->os));

		q->type = qtype;
		q->size = size;
		q->length = n_entries;

		/* Limit to half the queue size per interrupt */
		q->proc_limit = n_entries / 2;

		if ((q->type == SLI_QTYPE_EQ) || (q->type == SLI_QTYPE_CQ)) {
			/* For prism, phase will be flipped after a sweep through eq and cq */
			q->phase = 1;
		}

		switch(q->type) {
		case SLI_QTYPE_EQ:
			q->posted_limit = q->length / 2;
			break;
		default:
			if ((sli4->if_type == SLI4_IF_TYPE_BE3_SKH_PF) ||
			    (sli4->if_type == SLI4_IF_TYPE_BE3_SKH_VF)) {
				/* For Skyhawk, ring the doorbell more often */
				q->posted_limit = 8;
			} else {
				q->posted_limit = 64;
			}
			break;
		}
	}

	return 0;
}

/**
 * @ingroup sli
 * @brief Issue the command to create a queue.
 *
 * @param sli4 SLI context.
 * @param q Pointer to queue object.
 *
 * @return Returns 0 on success, or non-zero otherwise.
 */
int32_t
__sli_create_queue(sli4_t *sli4, sli4_queue_t *q)
{
	sli4_res_common_create_queue_t *res_q = NULL;
	sli4_res_create_wq_t *wq_res_q = NULL;

	if (sli_bmbx_command(sli4)){
		ocs_log_crit(sli4->os, "bootstrap mailbox write fail %s\n", SLI_QNAME[q->type]);
		sli_queue_dma_free(sli4, q);
		return -1;
	}
	if (sli_res_sli_config(sli4->bmbx.virt)) {
		ocs_log_err(sli4->os, "bad status create %s\n", SLI_QNAME[q->type]);
		sli_queue_dma_free(sli4, q);
		return -1;
	}
	res_q = (void *)((uint8_t *)sli4->bmbx.virt +
			offsetof(sli4_cmd_sli_config_t, payload));

	if (res_q->hdr.status) {
		ocs_log_err(sli4->os, "bad create %s status=%#x addl=%#x\n", SLI_QNAME[q->type],
				res_q->hdr.status, res_q->hdr.additional_status);
		sli_queue_dma_free(sli4, q);
		return -1;
	} else {
		q->id = res_q->q_id;

		switch (q->type) {
		case SLI_QTYPE_EQ:
			/* No doorbell information in response for EQs */
			q->doorbell_offset = regmap[SLI4_REG_EQ_DOORBELL][sli4->if_type].off;
			q->doorbell_rset = regmap[SLI4_REG_EQ_DOORBELL][sli4->if_type].rset;
			break;
		case SLI_QTYPE_CQ:
			/* No doorbell information in response for CQs */
			q->doorbell_offset = regmap[SLI4_REG_CQ_DOORBELL][sli4->if_type].off;
			q->doorbell_rset = regmap[SLI4_REG_CQ_DOORBELL][sli4->if_type].rset;
			break;
		case SLI_QTYPE_MQ:
			/* No doorbell information in response for MQs */
			q->doorbell_offset = regmap[SLI4_REG_MQ_DOORBELL][sli4->if_type].off;
			q->doorbell_rset = regmap[SLI4_REG_MQ_DOORBELL][sli4->if_type].rset;
			break;
		case SLI_QTYPE_RQ:
			if (!sli4->config.dual_ulp_capable) {
				/* set the doorbell for non-skyhawks */
				q->doorbell_offset = regmap[SLI4_REG_FCOE_RQ_DOORBELL][sli4->if_type].off;
				q->doorbell_rset = regmap[SLI4_REG_FCOE_RQ_DOORBELL][sli4->if_type].rset;
			} else {
				q->doorbell_offset = res_q->db_offset;
				q->doorbell_rset = res_q->db_rs;
			}
			break;
		case SLI_QTYPE_WQ:
			wq_res_q = (sli4_res_create_wq_t *) res_q;
			q->sfq_resp = wq_res_q->sfq;

			if (sli4->config.dpp) {
				q->u.dpp.enabled = wq_res_q->dpp;
				/* Check if dpp is enabled */
				if (wq_res_q->dpp) {
					q->doorbell_rset = wq_res_q->wq_reg_set;
					q->doorbell_offset = wq_res_q->wq_db_offset;
					q->u.dpp.id = wq_res_q->dpp_id;
					q->u.dpp.db_offset = wq_res_q->dpp_offset;
					q->u.dpp.db_rset = wq_res_q->dpp_reg_set;
					ocs_log_info(sli4->os, "DPP WQ Create success dpp_id = %d \n",
						     wq_res_q->dpp_id);
					break;
				}
			}
			/* DPP Not enabled. Set the regualr wq doorbell register and offset */
			if (!sli4->config.dual_ulp_capable) {
				/* set the doorbell for non-skyhawks */
				q->doorbell_offset = regmap[SLI4_REG_IO_WQ_DOORBELL][sli4->if_type].off;
				q->doorbell_rset = regmap[SLI4_REG_IO_WQ_DOORBELL][sli4->if_type].rset;
			} else {
				q->doorbell_offset = res_q->db_offset;
				q->doorbell_rset = res_q->db_rs;
			}
			break;
		default:
			break;
		}
	}

	return 0;
}

/**
 * @ingroup sli
 * @brief Get queue entry size.
 *
 * Get queue entry size given queue type.
 *
 * @param sli4 SLI context
 * @param qtype Type for which the entry size is returned.
 *
 * @return Returns > 0 on success (queue entry size), or a negative value on failure.
 */
int32_t
sli_get_queue_entry_size(sli4_t *sli4, uint32_t qtype)
{
	uint32_t	size = 0;

	if (!sli4) {
		ocs_log_err(NULL, "bad parameter sli4=%p\n", sli4);
		return -1;
	}

	switch (qtype) {
	case SLI_QTYPE_EQ:
		size = SLI4_EQE_BYTES;
		break;
	case SLI_QTYPE_CQ:
		size = SLI4_CQE_BYTES;
		break;
	case SLI_QTYPE_MQ:
		size = SLI4_MQE_BYTES;
		break;
	case SLI_QTYPE_WQ:
		if (SLI4_PORT_TYPE_FC == sli4->port_type) {
			size = sli4->config.wqe_size;
		} else {
			/* TODO */
			ocs_log_test(sli4->os, "unsupported queue entry size\n");
			return -1;
		}
		break;
	case SLI_QTYPE_RQ:
		size = SLI4_FCOE_RQE_SIZE;
		break;
	default:
		ocs_log_test(sli4->os, "unknown queue type %d\n", qtype);
		return -1;
	}
	return size;
}

/**
 * @ingroup sli
 * @brief Modify the delay timer for all the EQs
 *
 * @param sli4 SLI context.
 * @param eq Array of EQs.
 * @param num_eq Count of EQs.
 * @param shift Phase shift for staggering interrupts.
 * @param delay_mult Delay multiplier for limiting interrupt frequency.
 *
 * @return Returns 0 on success, or -1 otherwise.
 */
int32_t
sli_eq_modify_delay(sli4_t *sli4, sli4_queue_t *eq, uint32_t num_eq, uint32_t shift, uint32_t delay_mult)
{

	sli_cmd_common_modify_eq_delay(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE, eq, num_eq, shift, delay_mult);

	if (sli_bmbx_command(sli4)) {
		ocs_log_crit(sli4->os, "bootstrap mailbox write fail (MODIFY EQ DELAY)\n");
		return -1;
	}
	if (sli_res_sli_config(sli4->bmbx.virt)) {
		ocs_log_err(sli4->os, "bad status MODIFY EQ DELAY\n");
		return -1;
	}

	return 0;
}

/**
 * @ingroup sli
 * @brief Allocate a queue.
 *
 * @par Description
 * Allocates DMA memory and configures the requested queue type.
 *
 * @param sli4 SLI context.
 * @param qtype Type of queue to create.
 * @param q Pointer to the queue object.
 * @param n_entries Number of entries to allocate.
 * @param assoc Associated queue (that is, the EQ for a CQ, the CQ for a MQ, and so on).
 * @param ulp The ULP to bind, which is only used for WQ and RQs
 * @param sfq Send Frame WQ, valid for WQ creation only
 *
 * @return Returns 0 on success, or -1 otherwise.
 */
int32_t
sli_queue_alloc(sli4_t *sli4, uint32_t qtype, sli4_queue_t *q, uint32_t n_entries,
		sli4_queue_t *assoc, uint16_t ulp, bool sfq)
{
	int32_t		size;
	sli4_create_q_fn_t create = NULL;

	if (!sli4 || !q) {
		ocs_log_err(NULL, "bad parameter sli4=%p q=%p\n", sli4, q);
		return -1;
	}

	/* get queue size */
	size = sli_get_queue_entry_size(sli4, qtype);
	if (size < 0)
		return -1;

	switch (qtype) {
	case SLI_QTYPE_EQ:
		create = sli_cmd_common_create_eq;
		break;
	case SLI_QTYPE_CQ:
		create = sli_cmd_common_create_cq;
		break;
	case SLI_QTYPE_MQ:
		assoc->u.flag.is_mq = TRUE;
		create = sli_cmd_common_create_mq_ext;
		break;
	case SLI_QTYPE_WQ:
		if (SLI4_PORT_TYPE_FC == sli4->port_type) {
			if (sli4->if_type == SLI4_IF_TYPE_BE3_SKH_PF) {
				create = sli_cmd_fcoe_wq_create;
			} else {
				create = sli_cmd_fcoe_wq_create_v1;
			}
		} else {
			/* TODO */
			ocs_log_test(sli4->os, "unsupported WQ create\n");
			return -1;
		}
		break;
	default:
		ocs_log_test(sli4->os, "unknown queue type %d\n", qtype);
		return -1;
	}


	if (__sli_queue_init(sli4, q, qtype, size, n_entries, SLI_PAGE_SIZE)) {
		ocs_log_err(sli4->os, "%s allocation failed\n", SLI_QNAME[qtype]);
		return -1;
	}

	if (create(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE, q->dma,
		   assoc ? assoc->id : 0, ulp, sfq)) {

		if (__sli_create_queue(sli4, q)) {
			ocs_log_err(sli4->os, "create %s failed\n", SLI_QNAME[qtype]);
			return -1;
		}
		q->ulp = ulp;
	} else {
		ocs_log_err(sli4->os, "cannot create %s\n", SLI_QNAME[qtype]);
		return -1;
	}

	return 0;
}


/**
 * @ingroup sli
 * @brief Allocate a c queue set.
 *
 * @param sli4 SLI context.
 * @param num_cqs to create
 * @param qs Pointers to the queue objects.
 * @param n_entries Number of entries to allocate per CQ.
 * @param eqs Associated event queues
 *
 * @return Returns 0 on success, or -1 otherwise.
 */
int32_t
sli_cq_alloc_set(sli4_t *sli4, sli4_queue_t *qs[], uint32_t num_cqs,
			uint32_t n_entries, sli4_queue_t *eqs[])
{
	sli4_req_common_create_cq_set_v0_t *req = NULL;
	sli4_res_common_create_queue_set_t *res = NULL;
	uint32_t	i, p, offset = 0;
	uint32_t	cmd_size, payload_size;
	uint32_t	num_pages_cq;
	uintptr_t	addr;
	ocs_dma_t	dma;
	size_t		qmem_size;
	uint32_t	qpage_size;

	if (!sli4) {
		ocs_log_err(NULL, "bad parameter sli4=%p\n", sli4);
		return -1;
	}

	ocs_memset(&dma, 0, sizeof(dma));

	/* Align the queue DMA memory */
	for (i = 0; i < num_cqs; i++) {
		if (__sli_queue_init(sli4, qs[i], SLI_QTYPE_CQ, SLI4_CQE_BYTES, n_entries, SLI_PAGE_SIZE)) {
			ocs_log_err(sli4->os, "Queue init failed\n");
			goto error;
		}
	}

	qmem_size = sli_get_qmem_size(qs[0]->dma);
	qpage_size = sli_get_qpage_size(qmem_size);
	num_pages_cq = sli_page_count(qmem_size, qpage_size);
	cmd_size = (sizeof(sli4_req_common_create_cq_set_v0_t) + (8 * num_pages_cq * num_cqs));
	payload_size = OCS_MAX((size_t)cmd_size, sizeof(sli4_res_common_create_queue_set_t));

	if (ocs_dma_alloc(sli4->os, &dma, payload_size, SLI_PAGE_SIZE)) {
		ocs_log_err(sli4->os, "DMA allocation failed\n");
		goto error;
	}
	ocs_memset(dma.virt, 0, payload_size);

	if (sli_cmd_sli_config(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE, payload_size, &dma) == -1) {
		goto error;
	}

	/* Fill the request structure */
	req = (sli4_req_common_create_cq_set_v0_t *)((uint8_t *)dma.virt);
	req->hdr.opcode = SLI4_OPC_COMMON_CREATE_CQ_SET;
	req->hdr.subsystem = SLI4_SUBSYSTEM_FCFCOE;
	req->hdr.version = 0;
	req->hdr.request_length = cmd_size - sizeof(sli4_req_hdr_t);

	/* Fill the page_size_multiplier */
	req->page_size = (qpage_size / SLI_PAGE_SIZE);

	req->num_pages = num_pages_cq;
	switch (req->num_pages) {
	case 1:
		req->cqecnt = SLI4_CQ_CNT_256;
		break;
	case 2:
		req->cqecnt = SLI4_CQ_CNT_512;
		break;
	case 4:
		req->cqecnt = SLI4_CQ_CNT_1024;
		break;
	case 8:
		req->cqecnt = SLI4_CQ_CNT_LARGE;
		req->cqe_count = (qmem_size / SLI4_CQE_BYTES);
		break;
	default:
		ocs_log_test(sli4->os, "num_pages %d not valid\n", req->num_pages);
		goto error;
	}

	req->evt = TRUE;
	req->valid = TRUE;
	req->arm = FALSE;
	req->num_cq_req = num_cqs;

	if (sli4->if_type == SLI4_IF_TYPE_LANCER_G7)
		req->autovalid = TRUE;

	/* Fill page addresses of all the CQs */
	for (i = 0; i < num_cqs; i++) {
		req->eq_id[i] = eqs[i]->id;
		for (p = 0; p < req->num_pages; p++) {
			addr = qs[i]->dma[p].phys;
			req->page_physical_address[offset].low = ocs_addr32_lo(addr);
			req->page_physical_address[offset].high = ocs_addr32_hi(addr);
			offset++;
		}
	}

	if (sli_bmbx_command(sli4)) {
		ocs_log_crit(sli4->os, "bootstrap mailbox write fail CQSet\n");
		goto error;
	}

	res = (void *)((uint8_t *)dma.virt);
	if (res->hdr.status) {
		ocs_log_err(sli4->os, "bad create CQSet status=%#x addl=%#x\n",
			res->hdr.status, res->hdr.additional_status);
		goto error;
	} else {
		/* Check if we got all requested CQs */
		if (res->num_q_allocated != num_cqs) {
			ocs_log_crit(sli4->os, "Requested count CQs doesnt match\n");
			goto error;
		}

		/* Fill the resp cq ids */
		for (i = 0; i < num_cqs; i++) {
			qs[i]->id = res->q_id + i;
			qs[i]->doorbell_offset = regmap[SLI4_REG_CQ_DOORBELL][sli4->if_type].off;
			qs[i]->doorbell_rset   = regmap[SLI4_REG_CQ_DOORBELL][sli4->if_type].rset;
		}
	}

	ocs_dma_free(sli4->os, &dma);
	return 0;

error:
	for (i = 0; i < num_cqs; i++) {
		sli_queue_dma_free(sli4, qs[i]);
	}

	if (dma.size) {
		ocs_dma_free(sli4->os, &dma);
	}

	return -1;
}

/**
 * @ingroup sli
 * @brief Free a queue.
 *
 * @par Description
 * Frees DMA memory and de-registers the requested queue.
 *
 * @param sli4 SLI context.
 * @param q Pointer to the queue object.
 * @param destroy_queues Non-zero if the mailbox commands should be sent to destroy the queues.
 * @param free_memory Non-zero if the DMA memory associated with the queue should be freed.
 *
 * @return Returns 0 on success, or -1 otherwise.
 */
int32_t
sli_queue_free(sli4_t *sli4, sli4_queue_t *q, uint32_t destroy_queues, uint32_t free_memory)
{
	sli4_destroy_q_fn_t destroy = NULL;
	int32_t		rc = -1;

	if (!sli4 || !q) {
		ocs_log_err(NULL, "bad parameter sli4=%p q=%p\n", sli4, q);
		return -1;
	}

	if (destroy_queues) {
		switch (q->type) {
		case SLI_QTYPE_EQ:
			destroy = sli_cmd_common_destroy_eq;
			break;
		case SLI_QTYPE_CQ:
			destroy = sli_cmd_common_destroy_cq;
			break;
		case SLI_QTYPE_MQ:
			destroy = sli_cmd_common_destroy_mq;
			break;
		case SLI_QTYPE_WQ:
			if (SLI4_PORT_TYPE_FC == sli4->port_type) {
				destroy = sli_cmd_fcoe_wq_destroy;
			} else {
				/* TODO */
				ocs_log_test(sli4->os, "unsupported WQ destroy\n");
				return -1;
			}
			break;
		case SLI_QTYPE_RQ:
			if (SLI4_PORT_TYPE_FC == sli4->port_type) {
				destroy = sli_cmd_fcoe_rq_destroy;
			} else {
				/* TODO */
				ocs_log_test(sli4->os, "unsupported RQ destroy\n");
				return -1;
			}
			break;
		default:
			ocs_log_test(sli4->os, "bad queue type %d\n", q->type);
			return -1;
		}

		/*
		 * Destroying queues makes BE3 sad (version 0 interface type). Rely
		 * on COMMON_FUNCTION_RESET to free host allocated queue resources
		 * inside the SLI Port.
		 */
		if (SLI4_IF_TYPE_BE3_SKH_PF == sli4->if_type) {
			destroy = NULL;
		}

		// Destroy the queue if the operation is defined
		if (destroy && destroy(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE, q->id)) {
			sli4_res_hdr_t	*res = NULL;

			if (sli_bmbx_command(sli4)) {
				ocs_log_crit(sli4->os, "bootstrap mailbox write fail destroy %s\n", SLI_QNAME[q->type]);
			} else if (sli_res_sli_config(sli4->bmbx.virt)) {
				ocs_log_err(sli4->os, "bad status destroy %s\n", SLI_QNAME[q->type]);
			} else {
				res = (void *)((uint8_t *)sli4->bmbx.virt + offsetof(sli4_cmd_sli_config_t, payload));
				if (res->status) {
					ocs_log_err(sli4->os, "bad destroy %s status=%#x addl=%#x\n",
						    SLI_QNAME[q->type], res->status, res->additional_status);
				} else {
					rc = 0;
				}
			}
		}
	}

	sli_queue_reset(sli4, q);

	if (free_memory) {
		ocs_lock_free(&q->lock);
		rc = sli_queue_dma_free(sli4, q);
	}

	return rc;
}

int32_t
sli_queue_reset(sli4_t *sli4, sli4_queue_t *q)
{
	uint32_t i;

	ocs_lock(&q->lock);

	q->index = 0;
	q->n_posted = 0;

	if (SLI_QTYPE_MQ == q->type) {
		q->u.r_idx = 0;
	}

	for (i = 0; i < SLI_Q_DMA_CHUNKS && q->dma[i].virt; i++) {
		ocs_memset(q->dma[i].virt, 0, q->dma[i].size);
	}

	ocs_unlock(&q->lock);

	return 0;
}

/**
 * @ingroup sli
 * @brief Check if the given queue is empty.
 *
 * @par Description
 * If the valid bit of the current entry is unset, the queue is empty.
 *
 * @param sli4 SLI context.
 * @param q Pointer to the queue object.
 *
 * @return Returns TRUE if empty, or FALSE otherwise.
 */
int32_t
sli_queue_is_empty(sli4_t *sli4, sli4_queue_t *q)
{
	int32_t		rc = TRUE;
	uint8_t		*qe;
	uint32_t	index, offset, qpage_size;

	ocs_lock(&q->lock);

	qpage_size = sli_get_qpage_size((size_t)(q->size * q->length));

	index = ((q->index * q->size) / qpage_size);
	offset = ((q->index * q->size) % qpage_size);

	ocs_dma_sync(&q->dma[index], OCS_DMASYNC_POSTREAD);

	qe = (uint8_t *)(((uint8_t *)q->dma[index].virt) + offset);

	rc = !sli_queue_entry_is_valid(q, qe, FALSE);

	ocs_unlock(&q->lock);

	return rc;
}

/**
 * @ingroup sli
 * @brief Arm an EQ.
 *
 * @param sli4 SLI context.
 * @param q Pointer to queue object.
 * @param arm If TRUE, arm the EQ.
 *
 * @return Returns 0 on success, or non-zero otherwise.
 */
int32_t
sli_queue_eq_arm(sli4_t *sli4, sli4_queue_t *q, uint8_t arm)
{
	uint32_t	val = 0;

	ocs_lock(&q->lock);
		if (sli4->if_type == SLI4_IF_TYPE_LANCER_G7)
			val = sli_iftype6_eq_doorbell(q->n_posted, q->id, arm);
		else
			val = sli_eq_doorbell(q->n_posted, q->id, arm);

		ocs_reg_write32(sli4->os, q->doorbell_rset, q->doorbell_offset, val);
		q->n_posted = 0;
	ocs_unlock(&q->lock);

	return 0;
}

/**
 * @ingroup sli
 * @brief Arm a queue.
 *
 * @param sli4 SLI context.
 * @param q Pointer to queue object.
 * @param arm If TRUE, arm the queue.
 *
 * @return Returns 0 on success, or non-zero otherwise.
 */
int32_t
sli_queue_arm(sli4_t *sli4, sli4_queue_t *q, uint8_t arm)
{
	uint32_t	val = 0;

	ocs_lock(&q->lock);

	switch (q->type) {
	case SLI_QTYPE_EQ:
		if (sli4->if_type == SLI4_IF_TYPE_LANCER_G7)
			val = sli_iftype6_eq_doorbell(q->n_posted, q->id, arm);
		else
			val = sli_eq_doorbell(q->n_posted, q->id, arm);

		ocs_reg_write32(sli4->os, q->doorbell_rset, q->doorbell_offset, val);
		q->n_posted = 0;
		break;
	case SLI_QTYPE_CQ:
		if (sli4->if_type == SLI4_IF_TYPE_LANCER_G7)
			val = sli_iftype6_cq_doorbell(q->n_posted, q->id, arm);
		else
			val = sli_cq_doorbell(q->n_posted, q->id, arm);

		ocs_reg_write32(sli4->os, q->doorbell_rset, q->doorbell_offset, val);
		q->n_posted = 0;
		break;
	default:
		ocs_log_test(sli4->os, "Function should only be used for EQ/CQ, not %s\n", SLI_QNAME[q->type]);
	}

	ocs_unlock(&q->lock);

	return 0;
}

/**
 * @ingroup sli
 * @brief Write an entry to the queue object.
 *
 * Note: Assumes the q->lock will be locked and released by the caller.
 *
 * @param sli4 SLI context.
 * @param q Pointer to the queue object.
 * @param entry Pointer to the entry contents.
 *
 * @return Returns queue index on success, or negative error value otherwise.
 */
int32_t
_sli_queue_write(sli4_t *sli4, sli4_queue_t *q, uint8_t *entry)
{
	int32_t		rc = 0;
	uint8_t		*qe;
	uint8_t		*tmp;
	uint8_t		dpp = FALSE;
	uint32_t	qindex = q->index;
	uint32_t	i, index, offset, qpage_size;

	qpage_size = sli_get_qpage_size((size_t)(q->size * q->length));

	index = ((q->index * q->size) / qpage_size);
	offset = ((q->index * q->size) % qpage_size);

	qe = (uint8_t *)(((uint8_t *)q->dma[index].virt) + offset);

	if (entry) {
		if (SLI_QTYPE_WQ == q->type) {
			if (sli4->config.perf_wq_id_association)
				sli_set_wq_id_association(entry, q->id);

			if (q->u.dpp.enabled)
				dpp = TRUE;
		}
#if defined(OCS_INCLUDE_DEBUG)
		switch (q->type) {
		case SLI_QTYPE_WQ: {
			ocs_dump32(OCS_DEBUG_ENABLE_WQ_DUMP, sli4->os, "wqe", entry, q->size);
			break;

		}
		case SLI_QTYPE_MQ:
			// Note: we don't really need to dump the whole 256 bytes, just do 64
			ocs_dump32(OCS_DEBUG_ENABLE_MQ_DUMP, sli4->os, "mqe outbound", entry, 64);
			break;

		default:
			break;
		}
#endif
		ocs_memcpy(qe, entry, q->size);
		if (dpp) {
			tmp = (uint8_t *)entry;
			for (i = 0; i < q->size; i += sizeof(uint32_t)) {
				ocs_reg_write32(sli4->os,
						q->u.dpp.db_rset,
						q->u.dpp.db_offset + i,
						*((uint32_t *)(tmp + i)));
			}
		}
		q->n_posted = 1;
	}

	ocs_dma_sync(&q->dma[index], OCS_DMASYNC_PREWRITE);

	rc = sli_queue_doorbell(sli4, q);

	q->index = (q->index + q->n_posted) & (q->length - 1);
	q->n_posted = 0;

	if (rc < 0) {
		/* failure */
		return rc;
	} else if (rc > 0) {
		/* failure, but we need to return a negative value on failure */
		return -rc;
	} else {
		return qindex;
	}
}

/**
 * @ingroup sli
 * @brief Write an entry to the queue object.
 *
 * Note: Assumes the q->lock will be locked and released by the caller.
 *
 * @param sli4 SLI context.
 * @param q Pointer to the queue object.
 * @param entry Pointer to the entry contents.
 *
 * @return Returns queue index on success, or negative error value otherwise.
 */
int32_t
sli_queue_write(sli4_t *sli4, sli4_queue_t *q, uint8_t *entry)
{
	int32_t rc;

	ocs_lock(&q->lock);
		rc = _sli_queue_write(sli4, q, entry);
	ocs_unlock(&q->lock);

	return rc;
}

/**
 * @brief Check if the current queue entry is valid.
 *
 * @param q Pointer to the queue object.
 * @param qe Pointer to the queue entry.
 * @param clear Boolean to clear valid bit.
 *
 * @return Returns TRUE if the entry is valid, or FALSE otherwise.
 */
static uint8_t
sli_queue_entry_is_valid(sli4_queue_t *q, uint8_t *qe, uint8_t clear)
{
	uint8_t		valid = FALSE;
	uint8_t		valid_bit_set = 0;
	uint32_t	index, qpage_size;

	switch (q->type) {
	case SLI_QTYPE_EQ:
		valid = (((sli4_eqe_t *)qe)->vld == q->phase) ? 1 : 0;
		if (valid && clear) {
			((sli4_eqe_t *)qe)->vld = 0;
		}
		break;
	case SLI_QTYPE_CQ:
		/*
		 * For both MCQE and WCQE/RCQE, the valid bit
		 * is bit 31 of dword 3 (0 based)
		 */
		valid_bit_set = (qe[15] & 0x80) != 0;
		if (valid_bit_set == q->phase)
			valid = 1;

		if (valid & clear) {
			qe[15] &= ~0x80;
		}
		break;
	case SLI_QTYPE_MQ:
		valid = q->index != q->u.r_idx;
		break;
	case SLI_QTYPE_RQ:
		valid = TRUE;
		clear = FALSE;
		break;
	default:
		ocs_log_test(NULL, "Function doesn't handle type=%#x\n", q->type);
	}

	if (clear) {
		qpage_size = sli_get_qpage_size((size_t)(q->size * q->length));
		index = ((q->index * q->size) / qpage_size);

		ocs_dma_sync(&q->dma[index], OCS_DMASYNC_PREWRITE);
	}

	return valid;
}

/**
 * @ingroup sli
 * @brief Read an entry from the queue object.
 *
 * @param sli4 SLI context.
 * @param q Pointer to the queue object.
 * @param entry Destination pointer for the queue entry contents.
 *
 * @return Returns 0 on success, or non-zero otherwise.
 */
int32_t
sli_queue_read(sli4_t *sli4, sli4_queue_t *q, uint8_t *entry)
{
	int32_t		rc = 0;
	uint8_t		*qe;
	uint32_t	*qindex = NULL;
	uint32_t	index, offset, qpage_size;
	uint8_t 	clear = (SLI4_IF_TYPE_LANCER_G7 == sli_get_if_type(sli4)) ?  FALSE : TRUE;

	if (SLI_QTYPE_MQ == q->type) {
		qindex = &q->u.r_idx;
	} else {
		qindex = &q->index;
	}

	ocs_lock(&q->lock);

	qpage_size = sli_get_qpage_size((size_t)(q->size * q->length));

	index = ((*qindex * q->size) / qpage_size);
	offset = ((*qindex * q->size) % qpage_size);

	ocs_dma_sync(&q->dma[index], OCS_DMASYNC_POSTREAD);

	qe = (uint8_t *)(((uint8_t *)q->dma[index].virt) + offset);

	if (!sli_queue_entry_is_valid(q, qe, clear)) {
		ocs_unlock(&q->lock);
		return -1;
	}

	if (entry) {
		ocs_memcpy(entry, qe, q->size);
#if defined(OCS_INCLUDE_DEBUG)
		switch(q->type) {
		case SLI_QTYPE_CQ:
			ocs_dump32(OCS_DEBUG_ENABLE_CQ_DUMP, sli4->os, "cq", entry, q->size);
			break;
		case SLI_QTYPE_MQ:
			ocs_dump32(OCS_DEBUG_ENABLE_MQ_DUMP, sli4->os, "mq Compl", entry, 64);
			break;
		case SLI_QTYPE_EQ:
			ocs_dump32(OCS_DEBUG_ENABLE_EQ_DUMP, sli4->os, "eq Compl", entry, q->size);
			break;
		default:
			break;
		}
#endif
	}

	switch (q->type) {
		case SLI_QTYPE_EQ:
		case SLI_QTYPE_CQ:
		case SLI_QTYPE_MQ:
			*qindex = (*qindex + 1) & (q->length - 1);
			if (SLI_QTYPE_MQ != q->type) {
				q->n_posted++;
				/*
				 * For prism, the phase value will be used to check the validity of eq/cq entries.
				 * The value toggles after a complete sweep through the queue.
				 */
				if ((SLI4_IF_TYPE_LANCER_G7 == sli_get_if_type(sli4)) && (*qindex == 0)) {
					q->phase ^= (uint16_t) 0x1;
				}
			}
			break;
		default:
			/* reads don't update the index */
			break;
	}

	ocs_unlock(&q->lock);

	return rc;
}

int32_t
sli_queue_index(sli4_t *sli4, sli4_queue_t *q)
{

	if (q) {
		return q->index;
	} else {
		return -1;
	}
}

int32_t
sli_queue_poke(sli4_t *sli4, sli4_queue_t *q, uint32_t index, uint8_t *entry)
{
	int32_t rc;

	ocs_lock(&q->lock);
		rc = _sli_queue_poke(sli4, q, index, entry);
	ocs_unlock(&q->lock);

	return rc;
}

int32_t
_sli_queue_poke(sli4_t *sli4, sli4_queue_t *q, uint32_t q_elem_index, uint8_t *entry)
{
	int32_t		rc = 0;
	uint8_t		*qe;
	uint32_t	index, offset, qpage_size;

	if (q_elem_index >= q->length) {
		return -1;
	}

	qpage_size = sli_get_qpage_size((size_t)(q->size * q->length));

	index = ((q_elem_index * q->size) / qpage_size);
	offset = ((q_elem_index * q->size) % qpage_size);

	qe = (uint8_t *)(((uint8_t *)q->dma[index].virt) + offset);

	if (entry) {
		ocs_memcpy(qe, entry, q->size);
	}

	ocs_dma_sync(&q->dma[index], OCS_DMASYNC_PREWRITE);

	return rc;
}

/**
 * @ingroup sli
 * @brief Allocate SLI Port resources.
 *
 * @par Description
 * Allocate port-related resources, such as VFI, RPI, XRI, and so on.
 * Resources are modeled using extents, regardless of whether the underlying
 * device implements resource extents. If the device does not implement
 * extents, the SLI layer models this as a single (albeit large) extent.
 *
 * @param sli4 SLI context.
 * @param rtype Resource type (for example, RPI or XRI)
 * @param rid Allocated resource ID.
 * @param index Index into the bitmap.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_resource_alloc(sli4_t *sli4, sli4_resource_e rtype, uint32_t *rid, uint32_t *index)
{
	int32_t		rc = 0;
	uint32_t	size;
	uint32_t	extent_idx;
	uint32_t	item_idx;
	int		status;

	switch (rtype) {
	case SLI_RSRC_FCOE_VFI:
	case SLI_RSRC_FCOE_VPI:
	case SLI_RSRC_FCOE_RPI:
	case SLI_RSRC_FCOE_XRI:
		ocs_lock(&sli4->config.extent[rtype].lock);

		*rid = UINT32_MAX;
		*index = UINT32_MAX;
		status = ocs_bitmap_find(sli4->config.extent[rtype].use_map,
				sli4->config.extent[rtype].map_size);
		if (status < 0) {
			ocs_log_err(sli4->os, "out of resource %d (alloc=%d, status=%d, map_size=%d)\n",
					rtype, sli4->config.extent[rtype].n_alloc,
					status, sli4->config.extent[rtype].map_size);
			rc = -1;
			ocs_unlock(&sli4->config.extent[rtype].lock);
			break;
		} else {
			*index = status;
		}

		size = sli4->config.extent[rtype].size;

		extent_idx = *index / size;
		item_idx   = *index % size;

		*rid = sli4->config.extent[rtype].base[extent_idx] + item_idx;

		sli4->config.extent[rtype].n_alloc++;
		ocs_unlock(&sli4->config.extent[rtype].lock);
		break;
	default:
		rc = -1;
	}

	return rc;
}

/**
 * @ingroup sli
 * @brief Free the SLI Port resources.
 *
 * @par Description
 * Free port-related resources, such as VFI, RPI, XRI, and so. See discussion of
 * "extent" usage in sli_resource_alloc.
 *
 * @param sli4 SLI context.
 * @param rtype Resource type (for example, RPI or XRI).
 * @param rid Allocated resource ID.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
sli_resource_free(sli4_t *sli4, sli4_resource_e rtype, uint32_t rid)
{
	int32_t		rc = -1;
	uint32_t	x;
	uint32_t	size, *base;

	switch (rtype) {
	case SLI_RSRC_FCOE_VFI:
	case SLI_RSRC_FCOE_VPI:
	case SLI_RSRC_FCOE_RPI:
	case SLI_RSRC_FCOE_XRI:
		ocs_lock(&sli4->config.extent[rtype].lock);

		/*
		 * Figure out which extent contains the resource ID. I.e. find
		 * the extent such that
		 *   extent->base <= resource ID < extent->base + extent->size
		 */
		base = sli4->config.extent[rtype].base;
		size = sli4->config.extent[rtype].size;

		/*
		 * In the case of FW reset, this may be cleared but the force_free path will
		 * still attempt to free the resource. Prevent a NULL pointer access.
		 */
		if (base != NULL) {
			for (x = 0; x < sli4->config.extent[rtype].number; x++) {
				if ((rid >= base[x]) && (rid < (base[x] + size))) {
					rid -= base[x];
					ocs_bitmap_clear(sli4->config.extent[rtype].use_map,
							 (x * size) + rid);
					sli4->config.extent[rtype].n_alloc--;
					rc = 0;
					break;
				}
			}
		}

		ocs_unlock(&sli4->config.extent[rtype].lock);
		break;
	default:
		;
	}

	return rc;
}

int32_t
sli_resource_reset(sli4_t *sli4, sli4_resource_e rtype)
{
	int32_t		rc = -1;
	uint32_t	i;

	switch (rtype) {
	case SLI_RSRC_FCOE_VFI:
	case SLI_RSRC_FCOE_VPI:
	case SLI_RSRC_FCOE_RPI:
	case SLI_RSRC_FCOE_XRI:
		ocs_lock(&sli4->config.extent[rtype].lock);

		for (i = 0; i < sli4->config.extent[rtype].map_size; i++) {
			ocs_bitmap_clear(sli4->config.extent[rtype].use_map, i);
		}

		ocs_unlock(&sli4->config.extent[rtype].lock);
		rc = 0;
		break;
	default:
		;
	}

	return rc;
}

/**
 * @ingroup sli
 * @brief Parse an EQ entry to retrieve the CQ_ID for this event.
 *
 * @param sli4 SLI context.
 * @param buf Pointer to the EQ entry.
 * @param cq_id CQ_ID for this entry (only valid on success).
 *
 * @return
 * - 0 if success.
 * - < 0 if error.
 * - > 0 if firmware detects EQ overflow.
 */
int32_t
sli_eq_parse(sli4_t *sli4, uint8_t *buf, uint16_t *cq_id)
{
	sli4_eqe_t	*eqe = (void *)buf;
	int32_t		rc = 0;

	if (!sli4 || !buf || !cq_id) {
		ocs_log_err(NULL, "bad parameters sli4=%p buf=%p cq_id=%p\n", sli4, buf, cq_id);
		return -1;
	}

	switch (eqe->major_code) {
	case SLI4_MAJOR_CODE_STANDARD:
		*cq_id = eqe->resource_id;
		break;
	case SLI4_MAJOR_CODE_SENTINEL:
		ocs_log_debug(sli4->os, "sentinel EQE\n");
		rc = 1;
		break;
	default:
		ocs_log_test(sli4->os, "Unsupported EQE: major %x minor %x\n",
				eqe->major_code, eqe->minor_code);
		rc = -1;
	}

	return rc;
}

/**
 * @ingroup sli
 * @brief Parse a CQ entry to retrieve the event type and the associated queue.
 *
 * @param sli4 SLI context.
 * @param cq CQ to process.
 * @param cqe Pointer to the CQ entry.
 * @param etype CQ event type.
 * @param q_id Queue ID associated with this completion message
 * (that is, MQ_ID, RQ_ID, and so on).
 *
 * @return
 * - 0 if call completed correctly and CQE status is SUCCESS.
 * - -1 if call failed (no CQE status).
 * - Other value if call completed correctly and return value is a CQE status value.
 */
int32_t
sli_cq_parse(sli4_t *sli4, sli4_queue_t *cq, uint8_t *cqe, sli4_qentry_e *etype,
		uint16_t *q_id)
{
	int32_t	rc = 0;

	if (!sli4 || !cq || !cqe || !etype) {
		ocs_log_err(NULL, "bad parameters sli4=%p cq=%p cqe=%p etype=%p q_id=%p\n",
				sli4, cq, cqe, etype, q_id);
		return -1;
	}

	if (cq->u.flag.is_mq) {
		sli4_mcqe_t	*mcqe = (void *)cqe;

		if (mcqe->ae) {
			*etype = SLI_QENTRY_ASYNC;
		} else {
			*etype = SLI_QENTRY_MQ;
			rc = sli_cqe_mq(mcqe);
		}
		*q_id = -1;
	} else if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		rc = sli_fc_cqe_parse(sli4, cq, cqe, etype, q_id);
	} else {
		ocs_log_test(sli4->os, "implement CQE parsing type = %#x\n", sli4->port_type);
		rc = -1;
	}

	return rc;
}

/**
 * @ingroup sli
 * @brief Cause chip to enter an unrecoverable error state.
 *
 * @par Description
 * Cause chip to enter an unrecoverable error state. This is
 * used when detecting unexpected FW behavior so FW can be
 * halted from the driver as soon as error is detected.
 *
 * @param sli4 SLI context.
 * @param dump Generate dump as part of reset.
 *
 * @return Returns 0 if call completed correctly, or -1 if call failed (unsupported chip).
 */
int32_t sli_raise_ue(sli4_t *sli4, uint8_t dump)
{
	if (SLI4_IF_TYPE_BE3_SKH_PF == sli_get_if_type(sli4)) {
		switch(sli_get_asic_type(sli4)) {
		case SLI4_ASIC_TYPE_BE3: {
			sli_reg_write(sli4, SLI4_REG_SW_UE_CSR1, 0xffffffff);
			sli_reg_write(sli4, SLI4_REG_SW_UE_CSR2, 0);
			break;
		}
		case SLI4_ASIC_TYPE_SKYHAWK: {
			uint32_t value;
			value = ocs_config_read32(sli4->os, SLI4_SW_UE_REG);
			ocs_config_write32(sli4->os, SLI4_SW_UE_REG, (value | (1U << 24)));
			break;
		}
		default:
			ocs_log_test(sli4->os, "invalid asic type %d\n", sli_get_asic_type(sli4));
			return -1;
		}
	} else if ((SLI4_IF_TYPE_LANCER_FC_ETH == sli_get_if_type(sli4)) ||
		   (SLI4_IF_TYPE_LANCER_G7 == sli_get_if_type(sli4))) {
		if (OCS_FW_FUNC_DESC_DUMP == dump) {
			sli_reg_write(sli4, SLI4_REG_SLIPORT_CONTROL, SLI4_SLIPORT_CONTROL_FDD | SLI4_SLIPORT_CONTROL_IP);
		} else {
			uint32_t value = SLI4_PHYDEV_CONTROL_FRST;

			if (OCS_FW_CHIP_LEVEL_DUMP == dump)
				value |= SLI4_PHYDEV_CONTROL_DD;

			sli_reg_write(sli4, SLI4_REG_PHYSDEV_CONTROL, value);
		}
	} else {
		ocs_log_err(sli4->os, "invalid iftype=%d\n", sli_get_if_type(sli4));
		return -1;
	}

	return 0;
}

/**
 * @ingroup sli
 * @brief Read the SLIPORT_STATUS register to to check if a dump is present.
 *
 * @param sli4 SLI context.
 *
 * @return
 * - -1 if the call failed.
 * - 0 if the chip is not ready.
 * - 1 if the chip dump is ready.
 * - 2 if the func dump is ready.
 * - 3 if the FW has skipped the dump generation.
 */
int32_t sli_dump_is_ready(sli4_t *sli4)
{
	int32_t	rc = OCS_FW_DUMP_READY_STATUS_NOT_READY;
	uint32_t port_val;
	uint32_t bmbx_val;
	uint32_t uerr_lo;
	uint32_t uerr_hi;
	uint32_t uerr_mask_lo;
	uint32_t uerr_mask_hi;

	if (SLI4_IF_TYPE_BE3_SKH_PF == sli_get_if_type(sli4)) {
		/* for iftype=0, dump ready when UE is encountered */
		uerr_lo = sli_reg_read(sli4, SLI4_REG_UERR_STATUS_LO);
		uerr_hi = sli_reg_read(sli4, SLI4_REG_UERR_STATUS_HI);
		uerr_mask_lo = sli_reg_read(sli4, SLI4_REG_UERR_MASK_LO);
		uerr_mask_hi = sli_reg_read(sli4, SLI4_REG_UERR_MASK_HI);
		if ((uerr_lo & ~uerr_mask_lo) || (uerr_hi & ~uerr_mask_hi)) {
			rc = OCS_FW_DUMP_READY_STATUS_DD_PRESENT;
		}
	} else if ((SLI4_IF_TYPE_LANCER_FC_ETH == sli_get_if_type(sli4)) ||
		   (SLI4_IF_TYPE_LANCER_G7 == sli_get_if_type(sli4))) {
		/*
		 * Ensure that the port is ready AND the mailbox is
		 * ready before signaling that the dump is ready to go.
		 */
		port_val = sli_reg_read(sli4, SLI4_REG_SLIPORT_STATUS);
		bmbx_val = sli_reg_read(sli4, SLI4_REG_BMBX);

		if ((bmbx_val & SLI4_BMBX_RDY) && SLI4_PORT_STATUS_READY(port_val)) {
			if (SLI4_PORT_STATUS_DUMP_PRESENT(port_val)) {
				rc = OCS_FW_DUMP_READY_STATUS_DD_PRESENT;
			} else if (SLI4_PORT_STATUS_FDP_PRESENT(port_val)) {
				rc = OCS_FW_DUMP_READY_STATUS_FDB_PRESENT;
			} else {
				/* Chip is in ready state but no dump was generated by the FW */
				rc = OCS_FW_DUMP_READY_STATUS_SKIP_DUMP;
			}
		} else if (SLI4_PORT_STATUS_SPP & port_val) {
			uint32_t skip_dump = FALSE;
			uint32_t flags = SLI4_PHYDEV_CONTROL_CFP;

			/*
			 * If SLI is in a paused state, set CFP bit to resume and
			 * skip the dump based on pause errors list.
			 */
			sli_validate_pause_errors(sli4, &skip_dump);
			if (skip_dump)
				rc = OCS_FW_DUMP_READY_STATUS_SKIP_DUMP;
			else
				flags |= SLI4_PHYDEV_CONTROL_DD;

			sli_reg_write(sli4, SLI4_REG_PHYSDEV_CONTROL, flags);
		}
	} else {
		ocs_log_test(sli4->os, "invalid iftype=%d\n", sli_get_if_type(sli4));
		rc = OCS_FW_DUMP_READY_STATUS_FAILED;
	}

	return rc;
}

/**
 * @ingroup sli
 * @brief Read the SLIPORT_STATUS register to check if a dump is present.
 *
 * @param sli4 SLI context.
 *
 * @return
 * - 0 if call completed correctly and no dump is present.
 * - 1 if call completed and dump is present.
 * - -1 if call failed (unsupported chip).
 */
int32_t sli_dump_is_present(sli4_t *sli4)
{
	uint32_t val;
	uint32_t ready;

	if ((SLI4_IF_TYPE_LANCER_FC_ETH != sli_get_if_type(sli4)) &&
	    (SLI4_IF_TYPE_LANCER_G7 != sli_get_if_type(sli4))) {
		ocs_log_test(sli4->os, "Function only supported for I/F type 2/6");
		return -1;
	}

	/* If the chip is not ready, then there cannot be a dump */
	ready = sli_wait_for_fw_ready(sli4, SLI4_INIT_PORT_DELAY_US);
	if (!ready) {
		ocs_log_err(sli4->os, "Chip is not in ready state\n");
		return 0;
	}

	val = sli_reg_read(sli4, SLI4_REG_SLIPORT_STATUS);
	if (UINT32_MAX == val) {
		ocs_log_err(sli4->os, "error reading SLIPORT_STATUS\n");
		return -1;
	}

	return ((SLI4_PORT_STATUS_DUMP_PRESENT(val) || SLI4_PORT_STATUS_FDP_PRESENT(val)) ? 1 : 0);
}

/**
 * @ingroup sli
 * @brief Read the SLIPORT_STATUS register to check if the reset required is set.
 *
 * @param sli4 SLI context.
 *
 * @return
 * - 0 if call completed correctly and reset is not required.
 * - 1 if call completed and reset is required.
 * - -1 if call failed.
 */
int32_t sli_reset_required(sli4_t *sli4)
{
	uint32_t val;

	if (SLI4_IF_TYPE_BE3_SKH_PF == sli_get_if_type(sli4)) {
		ocs_log_test(sli4->os, "reset required N/A for iftype 0\n");
		return 0;
	}

	val = sli_reg_read(sli4, SLI4_REG_SLIPORT_STATUS);
	if (UINT32_MAX == val) {
		ocs_log_err(sli4->os, "error reading SLIPORT_STATUS\n");
		return -1;
	} else {
		return ((val & SLI4_PORT_STATUS_RN) ? 1 : 0);
	}
}

/**
 * @ingroup sli
 * @brief Read the SLIPORT_SEMAPHORE and SLIPORT_STATUS registers to check if
 * the port status indicates that a FW error has occurred.
 *
 * @param sli4 SLI context.
 *
 * @return
 * - 0 if call completed correctly and no FW error occurred.
 * - > 0 which indicates that a FW error has occurred.
 * - -1 if call failed.
 */
int32_t sli_fw_error_status(sli4_t *sli4)
{
	uint32_t sliport_semaphore;
	int32_t rc = 0;

	sliport_semaphore = sli_reg_read(sli4, SLI4_REG_SLIPORT_SEMAPHORE);
	if (UINT32_MAX == sliport_semaphore) {
		ocs_log_err(sli4->os, "error reading SLIPORT_SEMAPHORE register\n");
		return -1;
	}
	rc = (SLI4_PORT_SEMAPHORE_IN_ERR(sliport_semaphore) ? 1 : 0);

	if (rc == 0) {
		if (SLI4_IF_TYPE_BE3_SKH_PF == sli4->if_type ||
		    (SLI4_IF_TYPE_BE3_SKH_VF == sli4->if_type)) {
			uint32_t uerr_mask_lo, uerr_mask_hi;
			uint32_t uerr_status_lo, uerr_status_hi;

			uerr_mask_lo = sli_reg_read(sli4, SLI4_REG_UERR_MASK_LO);
			uerr_mask_hi = sli_reg_read(sli4, SLI4_REG_UERR_MASK_HI);
			uerr_status_lo = sli_reg_read(sli4, SLI4_REG_UERR_STATUS_LO);
			uerr_status_hi = sli_reg_read(sli4, SLI4_REG_UERR_STATUS_HI);
			if ((uerr_mask_lo & uerr_status_lo) != 0 ||
			    (uerr_mask_hi & uerr_status_hi) != 0) {
				rc = 1;
			}
		} else if ((SLI4_IF_TYPE_LANCER_FC_ETH == sli4->if_type) ||
			   (SLI4_IF_TYPE_LANCER_G7 == sli4->if_type)) {
			uint32_t sliport_status;

			sliport_status = sli_reg_read(sli4, SLI4_REG_SLIPORT_STATUS);
			rc = (SLI4_PORT_STATUS_ERROR(sliport_status) ? 1 : 0);
		}
	}
	return rc;
}

/**
 * @ingroup sli
 * @brief Determine if the chip FW is in a ready state
 *
 * @param sli4 SLI context.
 *
 * @return
 * - 0 if call completed correctly and FW is not ready.
 * - 1 if call completed correctly and FW is ready.
 * - -1 if call failed.
 */
int32_t
sli_fw_ready(sli4_t *sli4)
{
	uint32_t val;
	int32_t rc = -1;

	/*
	 * Is firmware ready for operation? Check needed depends on IF_TYPE
	 */
	if (SLI4_IF_TYPE_BE3_SKH_PF == sli4->if_type ||
	    SLI4_IF_TYPE_BE3_SKH_VF == sli4->if_type) {
		val = sli_reg_read(sli4, SLI4_REG_SLIPORT_SEMAPHORE);
		rc = ((SLI4_PORT_SEMAPHORE_STATUS_POST_READY ==
		       SLI4_PORT_SEMAPHORE_PORT(val)) &&
		      (!SLI4_PORT_SEMAPHORE_IN_ERR(val)) ? 1 : 0);
	} else if ((SLI4_IF_TYPE_LANCER_FC_ETH == sli4->if_type) ||
		   (SLI4_IF_TYPE_LANCER_G7 == sli4->if_type)) {
		val = sli_reg_read(sli4, SLI4_REG_SLIPORT_STATUS);
		rc = (SLI4_PORT_STATUS_READY(val) ? 1 : 0);
	}

	return rc;
}

/**
 * @ingroup sli
 * @brief Determine if the link can be configured
 *
 * @param sli4 SLI context.
 *
 * @return
 * - 0 if link is not configurable.
 * - 1 if link is configurable.
 */
int32_t sli_link_is_configurable(sli4_t *sli)
{
	int32_t rc = 0;
	/*
	 * Link config works on: Skyhawk and Lancer
	 * Link config does not work on: LancerG6
	 */

	switch (sli_get_asic_type(sli)) {
	case SLI4_ASIC_TYPE_SKYHAWK:
	case SLI4_ASIC_TYPE_LANCER:
	case SLI4_ASIC_TYPE_CORSAIR:
		rc = 1;
		break;
	case SLI4_ASIC_TYPE_LANCERG6:
	case SLI4_ASIC_TYPE_LANCERG7:
	case SLI4_ASIC_TYPE_BE3:
	default:
		rc = 0;
		break;
	}

	return rc;

}

int32_t
sli_is_paused(sli4_t *sli4)
{
	int32_t rc = FALSE;

	if (SLI4_PORT_STATUS_SPP & sli_reg_read(sli4, SLI4_REG_SLIPORT_STATUS))
		rc = TRUE;

	return rc;
}

void
sli_validate_pause_errors(sli4_t *sli4, uint32_t *skip_dump)
{
	int32_t i;
	uint32_t err1 = sli_reg_read(sli4, SLI4_REG_SLIPORT_ERROR1);
	uint32_t err2 = sli_reg_read(sli4, SLI4_REG_SLIPORT_ERROR2);

	ocs_log_debug(sli4->os, "error1: 0x%x\n", err1);
	ocs_log_debug(sli4->os, "error2: 0x%x\n", err2);

	for (i = 0; i < SLI4_PAUSE_ERRX_PAIR_CNT; i++) {
		if ((err1 == sli4->pause_errx_pair[i].err1) &&
		    ((0xFFFFFFFF == sli4->pause_errx_pair[i].err2) ||
		     (err2 == sli4->pause_errx_pair[i].err2))) {
			*skip_dump = TRUE;
			break;
		}
	}

	if (*skip_dump)
		ocs_log_crit(sli4->os, "Skipping diagnostic dump for (error1, error2) = (%#x, %#x)\n", err1, err2);
}

/**
 * @brief Decode the link speed
 *
 * @param in_link_speed: bits 15:8 of the field 'Current Link Speed' from 'Read Topo SLI CMD'
 *                       Refer table 12 SLI4_FC_FCOE_CMDREF
 * @param out_link_speed: link speed in Mbps
 *
 * @return None.
 */
void
sli4_decode_link_speed(uint32_t in_link_speed, uint32_t *out_link_speed)
{
	switch (in_link_speed) {
	case SLI4_READ_TOPOLOGY_SPEED_1G:
		*out_link_speed =  1 * 1000;
		break;
	case SLI4_READ_TOPOLOGY_SPEED_2G:
		*out_link_speed =  2 * 1000;
		break;
	case SLI4_READ_TOPOLOGY_SPEED_4G:
		*out_link_speed =  4 * 1000;
		break;
	case SLI4_READ_TOPOLOGY_SPEED_8G:
		*out_link_speed =  8 * 1000;
		break;
	case SLI4_READ_TOPOLOGY_SPEED_16G:
		*out_link_speed = 16 * 1000;
		break;
	case SLI4_READ_TOPOLOGY_SPEED_32G:
		*out_link_speed = 32 * 1000;
		break;
	case SLI4_READ_TOPOLOGY_SPEED_64G:
		*out_link_speed = 64 * 1000;
		break;
	case SLI4_READ_TOPOLOGY_SPEED_128G:
		*out_link_speed = 128 * 1000;
		break;
	case SLI4_READ_TOPOLOGY_SPEED_256G:
		*out_link_speed = 256 * 1000;
		break;
	default:
		ocs_log_err(NULL, "Invalid link_speed = %d\n", in_link_speed);
	}
}

int32_t
sli_parse_fw_dual_dump_state(sli4_t *sli4, void *buf, size_t size, bool *dd_state)
{
	sli4_res_common_set_features_t *resp = NULL;
	uint32_t sli_config_off = offsetof(sli4_cmd_sli_config_t, payload);
	sli4_res_common_set_features_dual_dump_param_t *param;
	uint32_t exp_param_len = sizeof(*param);

	resp = (sli4_res_common_set_features_t *)((uint8_t *)buf + sli_config_off);
	param = (sli4_res_common_set_features_dual_dump_param_t *)resp->params;

	if (resp->hdr.status || resp->hdr.additional_status) {
		ocs_log_err(sli4->os, "Dual dump: request failed with status=%#x, additional status=%#x\n",
			    resp->hdr.status, resp->hdr.additional_status);
		return -1;
	}

	if (resp->feature != SLI4_SET_FEATURES_CONFIG_DUAL_DUMP) {
		ocs_log_err(sli4->os, "Dual dump: invalid feature code: expected=%#x, received=%#x\n",
			    SLI4_SET_FEATURES_CONFIG_DUAL_DUMP, resp->feature);
		return -1;
	}

	if (resp->param_len != exp_param_len) {
		/* Potentially a FW book keeping mistake. Treat this as a non-fatal error,
		 * log the event and continue to process the response.
		 */
		ocs_log_err(sli4->os, "Dual dump: invalid param length: expected=%#x, received=%#x\n",
			    exp_param_len, resp->param_len);
	}

	*dd_state = (bool)param->dual_dump;
	ocs_log_debug(sli4->os, "Dual dump: feature is %s\n", param->dual_dump ? "enabled" : "disabled");

	return 0;
}

/**
 * @ingroup sli
 * @brief Set SRIOV default config 
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param nr_vfs Number of VFs to be configured 
 * @param wwna WWN assign requests Port to assign WWNs to VFs
 * @param priv_mask Privilege mask for the VFs
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_set_sriov_default_config_v2(sli4_t *sli4, void *buf, size_t size,
					   uint8_t nr_vfs, bool wwna, uint32_t priv_mask)
{
	sli4_req_common_set_sriov_default_config_v2_t *req_v2;
	uint32_t sli_config_off = 0;
	uint32_t payload_size;

	/* Payload length must accomodate both request and response */
	payload_size = max(sizeof(sli4_req_common_set_sriov_default_config_v2_t),
			   sizeof(sli4_res_common_query_fw_config_t));

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		sli_config_off = sli_cmd_sli_config(sli4, buf, size,
						    payload_size,
						    NULL);
	}

	req_v2 = (sli4_req_common_set_sriov_default_config_v2_t *)((uint8_t*)buf + sli_config_off);
	req_v2->vf_count = nr_vfs;
	req_v2->wwna = wwna;
	req_v2->privilege_mask = priv_mask;
	req_v2->hdr.opcode = SLI4_OPC_COMMON_SET_SRIOV_DEFAULT_CONFIG;
	req_v2->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
	req_v2->hdr.request_length = payload_size - sizeof(sli4_req_hdr_t);
	req_v2->hdr.version = 2;
	
	return sli_config_off + sizeof(sli4_req_common_set_sriov_default_config_v2_t);

}

/**
 * @ingroup sli
 * @brief Set SRIOV default config 
 *
 * @param sli4 SLI context.
 * @param nr_vfs Number of VFs to be configured 
 * @param wwna WWN assign requests Port to assign WWNs to VFs
 * @param priv_mask Privilege mask for the VFs
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_sriov_set_default_config(sli4_t *sli4, uint16_t nr_vfs,
			     bool wwna, uint32_t priv_mask)
{
	if (sli_cmd_common_set_sriov_default_config_v2(sli4, sli4->bmbx.virt, SLI4_BMBX_SIZE,
						       nr_vfs, wwna, priv_mask)) {
		sli4_res_hdr_t *resp;

		if (sli_bmbx_command(sli4)) {
			ocs_log_crit(sli4->os, "Bootstrap SET_SRIOV_DEFAULT_CONFIG failed\n");
			return -1;
		}

		resp = (sli4_res_hdr_t *)((uint8_t *)sli4->bmbx.virt +
					  offsetof(sli4_cmd_sli_config_t, payload));

		if (resp->status == SLI4_MGMT_STATUS_COMMAND_NOT_SUPPORTED &&
		    resp->additional_status == SLI4_CFG_ADD_STATUS_NO_STATUS) {
			ocs_log_err(sli4->os, "Set SRIOV default config command is unsupported\n");
			return -1;
		} else if (resp->status) {
			ocs_log_err(sli4->os, "Set SRIOV default config command bad status %#x (%#x) \n",
				    resp->status, resp->additional_status);
			return -1;
		}
	} else {
		ocs_log_err(sli4->os, "Error in preparing SET_SRIOV_DEFAULT_CONFIG req\n");
		return -1;
	}

	return 0;
}

/**
 * @ingroup sli
 * @brief COMMON_COMMON_SET_PROFILE_CONFIG V0 command.
 *
 * @param sli4 SLI context.
 * @param buf Destination buffer for the command.
 * @param size Buffer size, in bytes.
 * @param dma DMA capable memory containing profile.
 * @param profile_id Profile ID to configure.
 * @param descriptor_count Number of descriptors in DMA buffer.
 *
 * @return Returns the number of bytes written.
 */
int32_t
sli_cmd_common_set_profile_config_v0(sli4_t *sli4, void *buf, size_t size, ocs_dma_t *dma,
				     uint8_t profile_id, uint32_t descriptor_count)
{
	sli4_req_common_set_profile_config_v0_t *req = NULL;
	uint32_t cmd_off = 0;
	uint32_t payload_size;

	if (SLI4_PORT_TYPE_FC == sli4->port_type) {
		cmd_off = sli_cmd_sli_config(sli4, buf, size,
					     sizeof(sli4_req_common_set_profile_config_v0_t),
					     dma);
	}

	if (dma != NULL) {
		req = dma->virt;
		ocs_memset(req, 0, dma->size);
		payload_size = dma->size;
	} else {
		req = (sli4_req_common_set_profile_config_v0_t *)((uint8_t *)buf + cmd_off);
		payload_size = sizeof(sli4_req_common_set_profile_config_v0_t);
	}

        req->hdr.opcode = SLI4_OPC_COMMON_SET_PROFILE_CONFIG;
        req->hdr.subsystem = SLI4_SUBSYSTEM_COMMON;
        req->hdr.request_length = payload_size - sizeof(sli4_req_hdr_t);
        req->hdr.version = 0;
        req->profile_id = profile_id;
        req->desc_count = descriptor_count;

        return(cmd_off + sizeof(sli4_req_common_set_profile_config_t));
}

static int32_t
sli_sriov_setup(sli4_t *sli4)
{
	uint16_t nr_vfs = ocs_sriov_get_nr_vfs(sli4->os);
	uint16_t max_nr_vfs = ocs_sriov_get_max_nr_vfs(sli4->os);

	if (nr_vfs > max_nr_vfs) {
		ocs_log_err(sli4->os,
			    "Requested vfs (%d) greater than supported vfs (%d)\n",
			    nr_vfs, max_nr_vfs);
		return -1;
	}

	if (sli_sriov_set_default_config(sli4, nr_vfs, true,
					 SLI4_FUNC_PRIV_LINK_DEVSEC |
					 SLI4_FUNC_PRIV_LINK_DEVCFG)) {
		ocs_log_err(sli4->os, "Set SRIOV default config failed\n");
		return -1;
	}

	ocs_log_info(sli4->os, "SRIOV functions (nr_vfs %d) privileged successfully\n",
		     nr_vfs);
	return 0;
}
