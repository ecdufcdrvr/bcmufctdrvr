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
 *
 */

#include "ocs.h"
#include "ocs_dif.h"
#include "t10crc16.h"

/**
 * @brief Calculate the T10 PI CRC guard value for a block.
 *
 * @param buffer Pointer to the data buffer.
 * @param size Number of bytes.
 * @param crc Previously-calculated CRC, or 0 for a new block.
 *
 * @return Returns the calculated CRC, which may be passed back in for partial blocks.
 *
 */

uint16_t
ocs_scsi_dif_calc_crc(const uint8_t *buffer, uint32_t size, uint16_t crc)
{
	return t10crc16(buffer, size, crc);
}

/**
 * @brief Calculate the IP-checksum guard value for a block.
 *
 * @param addrlen array of address length pairs
 * @param addrlen_count number of entries in the addrlen[] array
 *
 * Algorithm:
 *    Sum all all the 16-byte words in the block
 *    Add in the "carry", which is everything in excess of 16-bits
 *    Flip all the bits
 *
 * @return Returns the calculated checksum
 */

uint16_t
ocs_scsi_dif_calc_checksum(ocs_scsi_vaddr_len_t addrlen[], uint32_t addrlen_count)
{
	uint32_t i, j;
	uint16_t checksum;
	uint32_t intermediate; /* Use an intermediate to hold more than 16 bits during calculations */
	uint32_t count;
	uint16_t *buffer;

	intermediate = 0;
	for (j = 0; j < addrlen_count; j++) {
		buffer = addrlen[j].vaddr;
		count = addrlen[j].length / 2;
		for (i=0; i < count; i++) {
			intermediate += buffer[i];
		}
	}

	/* Carry is everything over 16 bits */
	intermediate += ((intermediate & 0xffff0000) >> 16);

	/* Flip all the bits */
	intermediate = ~intermediate;

	checksum = intermediate;

	return checksum;
}

/**
 * @brief Return blocksize given SCSI API DIF block size
 *
 * Given the DIF block size enumerated value, return the block size value. (e.g.
 * OCS_SCSI_DIF_BLK_SIZE_512 returns 512)
 *
 * @param dif_info Pointer to SCSI API DIF info block
 *
 * @return returns block size, or 0 if SCSI API DIF blocksize is invalid
 */

uint32_t
ocs_scsi_dif_blocksize(ocs_scsi_dif_info_t *dif_info)
{
	uint32_t blocksize = 0;

	switch(dif_info->blk_size) {
	case OCS_SCSI_DIF_BK_SIZE_512:	blocksize = 512; break;
	case OCS_SCSI_DIF_BK_SIZE_1024:	blocksize = 1024; break;
	case OCS_SCSI_DIF_BK_SIZE_2048:	blocksize = 2048; break;
	case OCS_SCSI_DIF_BK_SIZE_4096:	blocksize = 4096; break;
	case OCS_SCSI_DIF_BK_SIZE_520:	blocksize = 520; break;
	case OCS_SCSI_DIF_BK_SIZE_4104:	blocksize = 4104; break;
	default:
		break;
	}

	return blocksize;
}

/**
 * @brief Set SCSI API DIF blocksize
 *
 * Given a blocksize value (512, 1024, etc.), set the SCSI API DIF blocksize
 * in the DIF info block
 *
 * @param dif_info Pointer to the SCSI API DIF info block
 * @param blocksize Block size
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

int32_t
ocs_scsi_dif_set_blocksize(ocs_scsi_dif_info_t *dif_info, uint32_t blocksize)
{
	int32_t rc = 0;

	switch(blocksize) {
	case 512:	dif_info->blk_size = OCS_SCSI_DIF_BK_SIZE_512; break;
	case 1024:	dif_info->blk_size = OCS_SCSI_DIF_BK_SIZE_1024; break;
	case 2048:	dif_info->blk_size = OCS_SCSI_DIF_BK_SIZE_2048; break;
	case 4096:	dif_info->blk_size = OCS_SCSI_DIF_BK_SIZE_4096; break;
	case 520:	dif_info->blk_size = OCS_SCSI_DIF_BK_SIZE_520; break;
	case 4104:	dif_info->blk_size = OCS_SCSI_DIF_BK_SIZE_4104; break;
	default:
		rc = -1;
		break;
	}
	return rc;

}

/**
 * @brief Return memory block size given SCSI DIF API
 *
 * The blocksize in memory for the DIF transfer is returned, given the SCSI DIF info
 * block and the direction of transfer.
 *
 * @param dif_info Pointer to DIF info block
 * @param wiretomem Transfer direction, 1 is wire to memory, 0 is memory to wire
 *
 * @return Memory blocksize, or negative error value
 *
 * WARNING: the order of initialization of the adj[] arrays MUST match the declarations
 * of OCS_SCSI_DIF_OPER_*
 */

int32_t
ocs_scsi_dif_mem_blocksize(ocs_scsi_dif_info_t *dif_info, int wiretomem)
{
	uint32_t blocksize;
	uint8_t wiretomem_adj[] = {
		0,		// OCS_SCSI_DIF_OPER_DISABLED,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_NODIF_OUT_CRC,
		0,		// OCS_SCSI_DIF_OPER_IN_CRC_OUT_NODIF,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_NODIF_OUT_CHKSUM,
		0,		// OCS_SCSI_DIF_OPER_IN_CHKSUM_OUT_NODIF,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CRC_OUT_CRC,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CHKSUM_OUT_CHKSUM,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CRC_OUT_CHKSUM,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CHKSUM_OUT_CRC,
		DIF_SIZE};	// OCS_SCSI_DIF_OPER_IN_RAW_OUT_RAW,
	uint8_t memtowire_adj[] = {
		0,		// OCS_SCSI_DIF_OPER_DISABLED,
		0,		// OCS_SCSI_DIF_OPER_IN_NODIF_OUT_CRC,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CRC_OUT_NODIF,
		0,		// OCS_SCSI_DIF_OPER_IN_NODIF_OUT_CHKSUM,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CHKSUM_OUT_NODIF,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CRC_OUT_CRC,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CHKSUM_OUT_CHKSUM,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CRC_OUT_CHKSUM,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CHKSUM_OUT_CRC,
		DIF_SIZE};	// OCS_SCSI_DIF_OPER_IN_RAW_OUT_RAW,

	blocksize = ocs_scsi_dif_blocksize(dif_info);
	if (blocksize == 0) {
		return -1;
	}

	if (wiretomem) {
		ocs_assert(dif_info->dif_oper < ARRAY_SIZE(wiretomem_adj), 0);
		blocksize += wiretomem_adj[dif_info->dif_oper];
	} else {	/* mem to wire */
		ocs_assert(dif_info->dif_oper < ARRAY_SIZE(memtowire_adj), 0);
		blocksize += memtowire_adj[dif_info->dif_oper];
	}
	return blocksize;
}

/**
 * @brief Return wire block size given SCSI DIF API
 *
 * The blocksize on the wire for the DIF transfer is returned, given the SCSI DIF info
 * block and the direction of transfer.
 *
 * @param dif_info Pointer to DIF info block
 * @param wiretomem Transfer direction, 1 is wire to memory, 0 is memory to wire
 *
 * @return Wire blocksize or negative error value
 *
 * WARNING: the order of initialization of the adj[] arrays MUST match the declarations
 * of OCS_SCSI_DIF_OPER_*
 */

int32_t
ocs_scsi_dif_wire_blocksize(ocs_scsi_dif_info_t *dif_info, int wiretomem)
{
	uint32_t blocksize;
	uint8_t wiretomem_adj[] = {
		0,		// OCS_SCSI_DIF_OPER_DISABLED,
		0,		// OCS_SCSI_DIF_OPER_IN_NODIF_OUT_CRC,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CRC_OUT_NODIF,
		0,		// OCS_SCSI_DIF_OPER_IN_NODIF_OUT_CHKSUM,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CHKSUM_OUT_NODIF,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CRC_OUT_CRC,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CHKSUM_OUT_CHKSUM,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CRC_OUT_CHKSUM,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CHKSUM_OUT_CRC,
		DIF_SIZE};	// OCS_SCSI_DIF_OPER_IN_RAW_OUT_RAW,
	uint8_t memtowire_adj[] = {
		0,		// OCS_SCSI_DIF_OPER_DISABLED,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_NODIF_OUT_CRC,
		0,		// OCS_SCSI_DIF_OPER_IN_CRC_OUT_NODIF,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_NODIF_OUT_CHKSUM,
		0,		// OCS_SCSI_DIF_OPER_IN_CHKSUM_OUT_NODIF,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CRC_OUT_CRC,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CHKSUM_OUT_CHKSUM,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CRC_OUT_CHKSUM,
		DIF_SIZE,	// OCS_SCSI_DIF_OPER_IN_CHKSUM_OUT_CRC,
		DIF_SIZE};	// OCS_SCSI_DIF_OPER_IN_RAW_OUT_RAW,


	blocksize = ocs_scsi_dif_blocksize(dif_info);
	if (blocksize == 0) {
		return -1;
	}

	if (wiretomem) {
		ocs_assert(dif_info->dif_oper < ARRAY_SIZE(wiretomem_adj), 0);
		blocksize += wiretomem_adj[dif_info->dif_oper];
	} else {	/* mem to wire */
		ocs_assert(dif_info->dif_oper < ARRAY_SIZE(memtowire_adj), 0);
		blocksize += memtowire_adj[dif_info->dif_oper];
	}

	return blocksize;
}
/**
 * @brief Return blocksize given HAL API DIF block size
 *
 * Given the DIF block size enumerated value, return the block size value. (e.g.
 * OCS_SCSI_DIF_BLK_SIZE_512 returns 512)
 *
 * @param dif_info Pointer to HAL API DIF info block
 *
 * @return returns block size, or 0 if HAL API DIF blocksize is invalid
 */

uint32_t
ocs_hal_dif_blocksize(ocs_hal_dif_info_t *dif_info)
{
	uint32_t blocksize = 0;

	switch(dif_info->blk_size) {
	case OCS_HAL_DIF_BK_SIZE_512:	blocksize = 512; break;
	case OCS_HAL_DIF_BK_SIZE_1024:	blocksize = 1024; break;
	case OCS_HAL_DIF_BK_SIZE_2048:	blocksize = 2048; break;
	case OCS_HAL_DIF_BK_SIZE_4096:	blocksize = 4096; break;
	case OCS_HAL_DIF_BK_SIZE_520:	blocksize = 520; break;
	case OCS_HAL_DIF_BK_SIZE_4104:	blocksize = 4104; break;
	default:
		break;
	}

	return blocksize;
}

/**
 * @brief Return memory block size given HAL DIF API
 *
 * The blocksize in memory for the DIF transfer is returned, given the HAL DIF info
 * block and the direction of transfer.
 *
 * @param dif_info Pointer to DIF info block
 * @param wiretomem Transfer direction, 1 is wire to memory, 0 is memory to wire
 *
 * @return Memory blocksize, or negative error value
 *
 * WARNING: the order of initialization of the adj[] arrays MUST match the declarations
 * of OCS_HAL_DIF_OPER_*
 */

int32_t
ocs_hal_dif_mem_blocksize(ocs_hal_dif_info_t *dif_info, int wiretomem)
{
	uint32_t blocksize;
	uint8_t wiretomem_adj[] = {
		0,		// OCS_HAL_DIF_OPER_DISABLED,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_NODIF_OUT_CRC,
		0,		// OCS_HAL_DIF_OPER_IN_CRC_OUT_NODIF,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_NODIF_OUT_CHKSUM,
		0,		// OCS_HAL_DIF_OPER_IN_CHKSUM_OUT_NODIF,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CRC_OUT_CRC,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CHKSUM_OUT_CHKSUM,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CRC_OUT_CHKSUM,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CHKSUM_OUT_CRC,
		DIF_SIZE};	// OCS_HAL_DIF_OPER_IN_RAW_OUT_RAW,
	uint8_t memtowire_adj[] = {
		0,		// OCS_HAL_DIF_OPER_DISABLED,
		0,		// OCS_HAL_DIF_OPER_IN_NODIF_OUT_CRC,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CRC_OUT_NODIF,
		0,		// OCS_HAL_DIF_OPER_IN_NODIF_OUT_CHKSUM,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CHKSUM_OUT_NODIF,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CRC_OUT_CRC,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CHKSUM_OUT_CHKSUM,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CRC_OUT_CHKSUM,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CHKSUM_OUT_CRC,
		DIF_SIZE};	// OCS_HAL_DIF_OPER_IN_RAW_OUT_RAW,

	blocksize = ocs_hal_dif_blocksize(dif_info);
	if (blocksize == 0) {
		return -1;
	}

	if (wiretomem) {
		ocs_assert(dif_info->dif_oper < ARRAY_SIZE(wiretomem_adj), 0);
		blocksize += wiretomem_adj[dif_info->dif_oper];
	} else {	/* mem to wire */
		ocs_assert(dif_info->dif_oper < ARRAY_SIZE(memtowire_adj), 0);
		blocksize += memtowire_adj[dif_info->dif_oper];
	}
	return blocksize;
}

/**
 * @brief Return wire block size given HAL DIF API
 *
 * The blocksize on the wire for the DIF transfer is returned, given the HAL DIF info
 * block and the direction of transfer.
 *
 * @param dif_info Pointer to DIF info block
 * @param wiretomem Transfer direction, 1 is wire to memory, 0 is memory to wire
 *
 * @return Wire blocksize or negative error value
 *
 * WARNING: the order of initialization of the adj[] arrays MUST match the declarations
 * of OCS_HAL_DIF_OPER_*
 */

int32_t
ocs_hal_dif_wire_blocksize(ocs_hal_dif_info_t *dif_info, int wiretomem)
{
	uint32_t blocksize;
	uint8_t wiretomem_adj[] = {
		0,		// OCS_HAL_DIF_OPER_DISABLED,
		0,		// OCS_HAL_DIF_OPER_IN_NODIF_OUT_CRC,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CRC_OUT_NODIF,
		0,		// OCS_HAL_DIF_OPER_IN_NODIF_OUT_CHKSUM,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CHKSUM_OUT_NODIF,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CRC_OUT_CRC,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CHKSUM_OUT_CHKSUM,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CRC_OUT_CHKSUM,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CHKSUM_OUT_CRC,
		DIF_SIZE};	// OCS_HAL_DIF_OPER_IN_RAW_OUT_RAW,
	uint8_t memtowire_adj[] = {
		0,		// OCS_HAL_DIF_OPER_DISABLED,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_NODIF_OUT_CRC,
		0,		// OCS_HAL_DIF_OPER_IN_CRC_OUT_NODIF,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_NODIF_OUT_CHKSUM,
		0,		// OCS_HAL_DIF_OPER_IN_CHKSUM_OUT_NODIF,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CRC_OUT_CRC,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CHKSUM_OUT_CHKSUM,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CRC_OUT_CHKSUM,
		DIF_SIZE,	// OCS_HAL_DIF_OPER_IN_CHKSUM_OUT_CRC,
		DIF_SIZE};	// OCS_HAL_DIF_OPER_IN_RAW_OUT_RAW,


	blocksize = ocs_hal_dif_blocksize(dif_info);
	if (blocksize == 0) {
		return -1;
	}

	if (wiretomem) {
		ocs_assert(dif_info->dif_oper < ARRAY_SIZE(wiretomem_adj), 0);
		blocksize += wiretomem_adj[dif_info->dif_oper];
	} else {	/* mem to wire */
		ocs_assert(dif_info->dif_oper < ARRAY_SIZE(memtowire_adj), 0);
		blocksize += memtowire_adj[dif_info->dif_oper];
	}

	return blocksize;
}
