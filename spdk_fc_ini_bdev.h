/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2016-2018 Broadcom.  All Rights Reserved.
 *  The term "Broadcom" refers to Broadcom Inc. and/or its subsidiaries.
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
 * SPDK fc block device data definitions.
 */

#ifndef _SPDK_FC_INI_BDEV_H
#define _SPDK_FC_INI_BDEV_H

#include "ocs_scsi_ini.h"

/* not defined in scsi.h */
#define READ_16         0x88
#define WRITE_16		0x8a

#define READ_16_CMDLEN  16
#define WRITE_16_CMDLEN 16

#pragma pack(push, 1)

/* CDB structures */
typedef struct read16_cdb {
	uint8_t opcode;
	uint8_t options;
	uint8_t lba[8];
	uint8_t length[4];
	uint8_t group_number;
	uint8_t control;
} read16_cdb_t;

typedef struct write16_cdb {
	uint8_t opcode;
	uint8_t options;
	uint8_t lba[8];
	uint8_t length[4];
	uint8_t group_number;
	uint8_t control;
} write16_cdb_t;

typedef struct tur_cdb {
	uint8_t opcode;
	uint8_t rsvd[4];
	uint8_t control;
} tur_cdb_t;

#pragma pack(pop)

/* FC Port Address Info */
typedef struct spdk_fc_ini_port_address {
	uint64_t wwpn;   /* port's Worldwide Port Name */
	uint64_t wwnn;   /* port's Worldwide Node Name */
} spdk_fc_ini_port_address_t;

/* FC Initiator SCSI device definition */
typedef struct spdk_fc_ini_scsi_dev {
	uint8_t vendor_id[8];    /* LUN vendor identifier - from SCSI INQUIRY */
	uint8_t product_id[16];  /* LUN product identifier - from SCSI INQUIRY */
	uint64_t lun_no;         /* LUN number - from SCSI REPORT LUNS */
	uint32_t blk_len;        /* LUN block size */
	uint64_t blk_cnt;        /* number of blocks on LUN */
	void* di_tgt;            /* discovered target ptr for LUN (do not modify) */
} spdk_fc_scsi_dev_t;


/* FC Initiator block device definition */
typedef struct spdk_fc_ini_bdev {

	/* is_ready indicates that the device can take a request. For new devices
	   is_ready will always be true. For existing devices, a false value
	   indicates the the device is no longer available to take requests */
	bool is_ready;
	uint32_t lcore_mask;     /* lcore's available for i/o on this device */
	struct spdk_fc_ini_port_address ini_address; /* initiator address */
	struct spdk_fc_ini_port_address tgt_address; /* target address */
	struct spdk_fc_ini_scsi_dev fc_dev;
	uint32_t gencnt;
	void *target_id;         /* target identifer (do not modify) */

} spdk_fc_ini_bdev_t;

/* SCSI read/write CBD optional byte definitions */
typedef struct spdk_fc_ini_io_cdb_options {
	union {
		uint8_t cdb_byte1_raw;  /* SCSI READ/WRITE CBD byte 1 */
		struct {
			uint8_t rsvd0       : 1; /* not used */
			uint8_t fua_nv      : 1; /* forced unit access - NV */
			uint8_t rsvd1       : 1; /* not used */
			uint8_t fua         : 1; /* forced unit access */
			uint8_t dpo         : 1; /* disable page out */
			uint8_t rw_protect  : 3; /* rd / wr protection */
		} cdb_b1;
	} u;

	uint8_t group_number		: 5;   /* SCSI read/write command group number */
	uint8_t reserved1			: 2;
	uint8_t mmc4				: 1;   /* restricted for MMC-4 */
	uint8_t control;				   /* control byte */
	uint8_t pad0;

} spdk_fc_ini_io_cdb_options_t;

typedef struct spdk_fc_ini_bdev_update {

/* bit values for update_mask */
#define SPDK_FC_INI_BLKDEV_UPDATE_NEW_DEVICE          1U
#define SPDK_FC_INI_BLKDEV_UPDATE_DEVICE_NOT_READY    (1U << 1)
#define SPDK_FC_INI_BLKDEV_UPDATE_DEVICE_READY        (1U << 2)
#define SPDK_FC_INI_BLKDEV_UPDATE_DEVICE_SIZE_CHANGE  (1U << 3)
#define SPDK_FC_INI_BLKDEV_UPDATE_DEVICE_REMOVED      (1U << 4)

	uint32_t update_mask;
	struct spdk_fc_ini_bdev *bdev;

} spdk_fc_ini_bdev_update_t;

/* FC initiator block device list update callback */
typedef void (*spdk_fc_ini_bdev_list_update_cb)(uint32_t bdev_cnt,  /* number of entries in bdev_list */
	spdk_fc_ini_bdev_update_t bdev_list[]);  /* list of updated bdev's */

//!
//! \fn spdk_fc_ini_init
//! \brief Performs initialization of the FC initiator components.
//! This must be called once before any other FC Initiator calls are made.
//! Additionally, this function can register a callback function to be called
//! when the there is a change to the block device list (e.g. added or changed
//! devices).  Upon receiving this callback, the code should call
//! spdk_fc_ini_get_bdevs() to get an updated block device list.
//!
//! \param cb - Address of a spdk_fc_ini_bdev_list_update_cb callback function
//! to be called when block device list needs to be updated.  Set to NULL if
//! no callback is desired.
//!
void spdk_fc_ini_init(
	spdk_fc_ini_bdev_list_update_cb cb);

//!
//! \fn spdk_fc_ini_writev
//! \brief Performs a SCSI write to the FC LUN associated with the passed in
//! block device bdev and registers the callback function to be called when the
//! write completes .  When the write completes the cb function will be called.
//! The cmd_type field of the spdk_fc_ini_cmd_completion_cb_info structure
//! will be set to SPDK_FC_INI_CMD_TYPE_WRITE.
//!
//! \param bdev - The block device (returned from spdk_fc_ini_enumerate_bdevs)
//! that is to be written to.
//!
//! \param iov - Array of iovec structures containing SGL of the buffers to
//! write.
//!
//! \param iovcnt - Number of iovec structures in iov.
//!
//! \param len - Total write length (in bytes).
//!
//! \param offset - Offset (LBA) into LUN to write data.
//!
//! \param spdk_fc_ini_io_cdb_options - Specifies SCSI WRITE CDB optional
//! values (byte 1 & group number).
//!
//! \param cb - spdk_fc_ini_cmd_completion_cb callback function to call upon
//! completion.
//!
//! \param timeout - Time (in seconds) to wait for write to complete before
//! returning timeout error.  A timeout value of 0 will use the driver default
//! timeout value.
//!
//! \return int - error code
//! 0	Success.  Write command was successfully queued for processing.
//! SPDK_FC_INI_ERROR_NOT_INIT	FC Initiator has not been initialized.
//! Call spdk_fc_ini_init() first.
//! SPDK_FC_INI_ERROR_DEVICE_NOT_READY	Device is not ready to take a command.
//! [Additional Errors – TBD]
//!
//! Notes:The buffers in iov must be allocated from DPDK rte_mempool.
//!
int spdk_fc_ini_writev(
	struct spdk_fc_ini_bdev *bdev,
	struct iovec *iov,
	int iovcnt,
	uint64_t len,
	uint64_t offset,
	struct spdk_fc_ini_io_cdb_options cbd_options,
	spdk_fc_ini_cmd_completion_cb cb,
	void *cb_data,
	uint32_t timeout);

//!
//! \fn spdk_fc_ini_readv
//! \brief Performs a SCSI read on the FC LUN associated with the passed in
//! block device bdev and registers the callback function to be called when
//! the read completes.  When the read completes the cb function will be
//! called.  The cmd_type field of the spdk_fc_ini_cmd_completion_cb_info
//! structure will be set to SPDK_FC_INI_CMD_TYPE_READ.
//!
//! \param bdev - The block device (returned from spdk_fc_ini_enumerate_bdevs)
//! that is to be read  from.
//!
//! \param iov - Array of iovec structures containing SGL of the
//! buffers to read into.
//!
//! \param iovcnt - Number of iovec structures in iov.
//!
//! \param len - Total read length (in bytes).
//!
//! \param offset - Offset (LBA) into LUN to write data.
//!
//! \param spdk_fc_ini_io_cdb_options - Specifies SCSI READ CDB optional
//! values (byte 1 & group number).
//!
//! \param cb - spdk_fc_ini_cmd_completion_cb callback function to call
//! upon completion.
//!
//! \param cb_data - Pointer to callback data to provide in callback
//! function. Defined by application (e.g. unique command identifier).
//!
//! \param timeout - Time (in seconds) to wait for read to complete before
//! returning timeout error.  A timeout value of 0 will use the driver
//! default timeout value.
//!
//! \return int - error code
//! 0  Success.  Read command was successfully queued for processing.
//! SPDK_FC_INI_ERROR_NOT_INIT	FC Initiator has not been initialized.
//! Call spdk_fc_ini_init() first.
//! SPDK_FC_INI_ERROR_DEVICE_NOT_READY	Device is not ready to take a command.
//! [Additional Errors – TBD]
//!
//! Notes:The buffers in iov must be allocated from DPDK rte_mempool.
//!
int spdk_fc_ini_readv(
	struct spdk_fc_ini_bdev *bdev,
	struct iovec *iov,
	int iovcnt,
	uint64_t len,
	uint64_t offset,
	struct spdk_fc_ini_io_cdb_options cbd_options,
	spdk_fc_ini_cmd_completion_cb cb,
	void *cb_data,
	uint32_t timeout);

//!
//! \fn spdk_fc_ini_scsi_pass_thru
//! \brief Performs a SCSI command on the FC LUN (or target) specified by
//! the bdev and registers the callback function to be called when the command
//! completes.  buf can used for commands requiring data or commands that
//! return data.  When the command completes the cb function will be called.
//! The cmd_type field of the spdk_fc_ini_cmd_completion_cb_info structure
//! will be set to SPDK_FC_INI_CMD_TYPE_SCSI_PASS_THRU.
//!
//! \param bdev - The block device (returned from spdk_fc_ini_enumerate_bdevs)
//! that the pass-thru command should be executed on.
//!
//! \param scsi_cdb - The CDB containing the SCSI command to execute.
//!
//! \param cdb_len - The size of scsi_cdb.
//!
//! \param iov - Array of iovec structures containing SGL of the buffers
//! containing data for command.
//!
//! \param iovcnt - Number of iovec structures in iov.
//!
//! \param data_len - The length (in bytes) of data to transfer. 0 if no
//! data is transfered
//!
//! \param data_dir - The direction of the data flow (see SPDK scsi.h).
//! Ignored when buf is NULL and buf_size is 0.  Possible values are:
//! SPDK_SCSI_DIR_NONE – no data transferred
//! SPDK_SCSI_DIR_TO_DEV – data transferred from initiator.
//! SPDK_SCSI_DIR_FROM_DEV – data transferred into initiator.
//!
//! \param cb - spdk_fc_ini_cmd_completion_cb callback function to call upon
//! completion.
//!
//! \param cb_data - Pointer to callback data to provide in callback function.
//! Defined by application (e.g. unique command identifier).
//!
//! \param timeout - Time (in seconds) to wait for command to complete before
//! returning timeout error.  A timeout value of 0 will use the driver
//! default timeout value.
//!
//! \return int - error code
//! 0	Success.  SCSI pass-thru command was successfully queued for
//! processing.
//! SPDK_FC_INI_ERROR_NOT_INIT	FC Initiator has not been initialized.
//! Call spdk_fc_ini_init() first.
//! SPDK_FC_INI_ERROR_DEVICE_NOT_READY	Device is not ready to take a command.
//! [Additional Errors – TBD]
//!
//! Notes: The buf buffer must be allocated from DPDK rte_mempool.
//!
int spdk_fc_ini_scsi_pass_thru(
	struct spdk_fc_ini_bdev *bdev,
	uint8_t *scsi_cdb,
	uint32_t cdb_len,
	struct iovec *iov,
	int iovcnt,
	int data_len,
	int data_dir,
	spdk_fc_ini_cmd_completion_cb cb,
	void *cb_data,
	uint32_t timeout);

//!
//! \fn spdk_fc_ini_reset
//! \brief Performs a reset operation on the block device or all block devices
//! and registers the callback function to be called when the reset completes.
//! All I/O’s currently queued for the device(s) are cancelled.
//!
//! \param bdev - The block device (i.e. LUN) or target (returned from
//! spdk_fc_ini_enumerate_bdevs) that the reset should be done on.  If set to
//! NULL all block devices are reset.
//!
//! \param reset_type - Type of reset  SPDK_FC_INI_BDEV_RESET, SPDK_FC_INI_BDEV_FLUSH
//!
//! \param cb - spdk_fc_ini_cmd_completion_cb callback function to call upon
//! completion.
//!
//! \param cb_data - Pointer to callback data to provide in callback function.
//! Defined by application (e.g. unique command identifier).
//!
//! \return int
int spdk_fc_ini_reset(
	struct spdk_fc_ini_bdev *bdev,
	uint8_t reset_type,
	spdk_fc_ini_cmd_completion_cb cb,
	void *cb_data);

//!
//! \fn spdk_fc_ini_scsi_passthru_completion
//! \brief  Callback event from driver layer for SCSI pass-thru command.
//!
//!
void spdk_fc_ini_scsi_passthru_completion(void *arg1, void *arg2);

//!
//! \fn spdk_fc_ini_reset_completion
//! \brief Callback event from driver for reset function completion.
//!
//!
void spdk_fc_ini_reset_completion(void *arg1, void *arg2);

//!
//! \fn issue_test_unit_ready
//! \brief Function to issue a scsi test unit ready
//!
//! \param bdev - The block device (i.e. LUN) or target (returned from
//! spdk_fc_ini_enumerate_bdevs) that the command should issued for.
//!
//! \param cb - spdk_fc_ini_cmd_completion_cb callback function to call upon
//! completion.
//!
//! \return int
//static int issue_test_unit_ready(struct spdk_fc_ini_bdev *bdev,
//	spdk_fc_ini_cmd_completion_cb cb);

void spdk_fc_ini_blkreq_free(void);

#endif
