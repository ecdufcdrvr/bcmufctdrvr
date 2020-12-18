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
 * Common declartaions for the driver's IOCTL interface
 */

#if !defined(__OCS_IOCTL_H__)
#define __OCS_IOCTL_H__

#define MAX_FDMI_HBA_COUNT		2048	/*Enough for 2K remote initiator nodes */
#define MAX_FDMI_PORT_COUNT		256	/*Enough for 2K remote initiator nodes */

/* FW dump status */
#define OCS_FW_DUMP_STATUS_SUCCESS		0
#define OCS_FW_DUMP_STATUS_IN_PROGRESS		1
#define OCS_FW_DUMP_STATUS_SKIP			2
#define OCS_FW_DUMP_STATUS_ALREADY_PRESENT	3
#define OCS_FW_DUMP_STATUS_DUMP_TYPE_INVALID	4
#define OCS_FW_DUMP_STATUS_FAILED		-1

#define OCS_SRIOV_VFS_MAX 8
typedef struct ocs_sriov_vfs_params_s {
	int32_t num_wwpn;
	uint64_t wwnn;
	uint64_t wwpn[OCS_SRIOV_VFS_MAX];
} ocs_sriov_vfs_params_t;

/**
 * @brief OCS test ioctl
 *
 * Simple structure for testing the IOCTL interface
 */

typedef struct {
	char string[32];		/**< fixed string buffer */
} ocs_ioctl_test_t;

/**
 * @brief DRIVER_INFO ioctl structure
 *
 * Structure is returned whtn the OCS_IOCTL_CMD_DRIVER_INFO is issued by a user space program.
 */

typedef struct {
	uint16_t pci_vendor;		/**< PCI vender ID value (binary) */
	uint16_t pci_device;		/**< PCI device ID value (binary) */
	char businfo[16];		/**< Bus information (text) */
	uint32_t sli_intf;		/**< SLI_INTF register value (binary) */
	char desc[64];			/**< description (text) */
	char fw_rev[32];		/**< firmware revision (text) */
	union {
		struct {
			uint8_t wwnn[8];	/**< WWNN (binary) */
			uint8_t wwpn[8];	/**< WWPN (binary) */
		} fc;
		struct {
			uint8_t mac_addr[6];	/**< MAC address (binary) */
			uint8_t reserved[10];
		} iscsi;
	} hw_addr;
	char serialnum[32];		/**< board serial number (text) */
} ocs_ioctl_driver_info_t;

#define ELXU_BSD_MAGIC			0x30584c45

/**
 * @brief IOCTL_CMD_IOCTL_ELXU_MBOX ioctl structure
 *
 * Structure used to submit elxsdkutil mailbox requests for firmware download and
 * dump file retrieveal.
 */

typedef struct {
	uint32_t	magic;		/**< magic number */
	uint32_t	size;		/**< size of MBX command */
	uint8_t		payload[256];	/**< MBX command in/out payload buffer */
	uint64_t	in_addr;	/**< user space address of input buffer */
	uint64_t	in_bytes;	/**< length of user space input buffer in bytes */
	uint64_t	out_addr;	/**< user space address of output buffer */
	uint64_t	out_bytes;	/**< length of user space output buffer in bytes */
} ocs_ioctl_elxu_mbox_t;

enum {
	ocs_ioctl_scsi_cmd_loop,	/**< Start command loop */
	ocs_ioctl_scsi_cmd_loop_wait,	/**< Start command loop and wait for completion */
	ocs_ioctl_scsi_cmd_stop,	/**< Stop command loop */
	ocs_ioctl_scsi_cmd,		/**< Start one command */
	ocs_ioctl_scsi_cmd_wait,	/**< Wait for a command to complete */
	ocs_ioctl_scsi_cmd_abort,	/**< Start an abort */
	ocs_ioctl_scsi_cmd_tmf,		/**< Start a tmf */
	ocs_ioctl_els_send,		/**< Start an ELS */
	ocs_ioctl_tgt_logout,		/**< logout of a target */
	ocs_ioctl_scsi_cmd_wait_any,	/**< Wait for any command to complete */
};

enum {
	ocs_ioctl_scsi_cmd_rd = (1U << 0),	/**< direction is read */
	ocs_ioctl_scsi_cmd_wr = (1U << 1),	/**< direction is write */
};

/**
 * @brief OCS_IOCTL_CMD_SCSI_CMD ioctl command structure
 */

typedef enum {
	DIF_OP_DISABLE = 0,
	DIF_OP_IN_NODIF_OUT_CRC,
	DIF_OP_IN_CRC_OUT_NODIF,
	DIF_OP_IN_NODIF_OUT_CHKSUM,
	DIF_OP_IN_CHKSUM_OUT_NODIF,
	DIF_OP_IN_CRC_OUT_CRC,
	DIF_OP_IN_CHKSUM_OUT_CHKSUM,
	DIF_OP_IN_CRC_OUT_CHKSUM,
	DIF_OP_IN_CHKSUM_OUT_CRC,
	DIF_OP_IN_RAW_OUT_RAW,
	} dif_op_t;

#define DIF_OP_PASS_THRU		DIF_OP_IN_CRC_OUT_CRC
#define DIF_OP_STRIP			DIF_OP_IN_CRC_OUT_NODIF
#define DIF_OP_INSERT			DIF_OP_IN_NODIF_OUT_CRC

typedef struct {
	dif_op_t dif_op;
	uint32_t
		check_ref_tag:1,	/* check reference tag on initiator writes */
		check_app_tag:1,	/* check application tag on initiator writes */
		check_guard:1,		/* check CRC on initiator writes */
		dif_separate:1;		/* use DIF separate transfers */
	uint32_t ref_tag;		/* DIF reference tag */
	uint16_t app_tag;		/* DIF application tag */
	uint32_t blocksize;		/* DIF blocksize */
} dif_info_t;

typedef struct {
	int command;			/**< SCSI command request command */
	uint32_t target_idx;		/**< Target device index */
	uint32_t dir;			/**< rd or wr */
	uint64_t lun;			/**< lun value */
	int32_t  tmf;			/**< TMF */
	uint8_t cdb[32];		/**< SCSI CDB */
	uint32_t cdb_len;		/**< SCSI CDB length in bytes */
	uint32_t els_cmd;		/**< ELS command */
	uint32_t flags;			/**< command flags */
	uint32_t queue_depth;		/**< queue depth for command looping */
	uint32_t payload_length;	/**< payload length for command */
	uint32_t dif_payload_length;	/**< DIF payload length for command if separate */
	uint32_t io_count;		/**< command count for looping */
	uint32_t io_timeout;		/**< IO timeout in seconds (0 = no timeout) */

	uint32_t directio;		/**< If set, DMA to and from user buffers */

	uint32_t first_burst:1;		/**< If true send IO writes with first burst */
	uint32_t first_burst_size;	/**< If first burst is enabled, then this size */

	int32_t wait_timeout_usec;	/**< Wait timeout (usec) for wait, wait_any */

	/* T10-PI */
	dif_info_t dif;			/* DIF info */

	/* user space buffers */
	void *payload;			/**< pointer to user space payload buffer */
	void *dif_payload;		/**< pointer to DIF block data if separate */
	uint8_t scsi_status;		/**< SCSI status */
	uint16_t scsi_status_qualifier;	/**< SCSI status qualifier */
	void *sense_data;		/**< pointer to sense data buffer */
	uint32_t sense_data_length;	/**< length of sense data buffer (call=buffer leng, return=data written) */
	int32_t residual;		/**< residual */
	uint32_t tag_to_abort;		/**< tag to abort for an abort task request */

	/* return value */
	int32_t status;			/**< returned status */
	uint32_t data_transferred;	/**< updated with data transferred */
	uint32_t tag;			/**< returned unique I/O context tag */

	/* for scsi loop */
	uint32_t submit_count;		/**< count of submitted IOs */
	uint32_t complete_count;	/**< count of completed IOs */
} ocs_ioctl_scsi_cmd_t;

/**
 * @brief coredump helper function command values
 */

typedef enum {
	OCS_ECD_HELPER_CFG_READ8,
	OCS_ECD_HELPER_CFG_READ16,
	OCS_ECD_HELPER_CFG_READ32,
	OCS_ECD_HELPER_CFG_WRITE8,
	OCS_ECD_HELPER_CFG_WRITE16,
	OCS_ECD_HELPER_CFG_WRITE32,
	OCS_ECD_HELPER_BAR_READ8,
	OCS_ECD_HELPER_BAR_READ16,
	OCS_ECD_HELPER_BAR_READ32,
	OCS_ECD_HELPER_BAR_WRITE8,
	OCS_ECD_HELPER_BAR_WRITE16,
	OCS_ECD_HELPER_BAR_WRITE32,
} ocs_ecd_helper_cmd_t;

/**
 * @brief OCS_IOCTL_CMD_ECD_HELPER ioctl structure
 */

typedef struct {
	ocs_ecd_helper_cmd_t cmd;	/*<< coredump helper function command */
	uint32_t bar;			/*<< BAR value to use */
	uint32_t offset;		/*<< offset value to use */
	uint32_t data;			/*<< 32 bit data value to write or return read data in */
	int status;			/*<< status of helper function request */
} ocs_ioctl_ecd_helper_t;

/**
 * @brief OCS_IOCTL_CMD_VPORT ioctl structure
 */

typedef struct {
	uint32_t domain_index;		/*<< domain instance index */
	uint32_t req_create:1,		/*<< 1 = create vport, zero = remove vport */
		 enable_ini:1,		/*<< 1 = enable vport as an initiator */
		 enable_tgt:1;		/*<< 1 = enable vport as a target */
	uint64_t wwpn;			/*<< wwpn to create or delete */
	uint64_t wwnn;			/*<< wwnn to create or delete */
	int status;			/*<< status of helper function request */
} ocs_ioctl_vport_t;

/**
 * @brief connection info ioctl structure
 *
 * Structure is returned when the OCS_IOCTL_CMD_CONNECTION_INFO is issued by a user space program.
 */
typedef struct {
	uint32_t connection_handle;
	uint16_t connection_id;
	uint8_t source_ip_type;
	uint8_t source_ip[16];
	uint16_t source_port;
	uint8_t dest_ip_type;
	uint8_t dest_ip[16];
	uint16_t dest_port;
} ocs_ioctl_connection_info_t;

typedef struct {
	uint32_t max_connections;
	uint32_t num_connections;
	ocs_ioctl_connection_info_t *connections;
} ocs_ioctl_connections_t;

/*
 * boardtemp (DUMP_TYPE4_WKI_TAG_SAT_TEM) response buffer length.
 */
#define OCS_DUMP_TYPE4_WKI_TAG_SAT_TEM_RESP_LEN	(6 * 4)

#define OCS_CHAP_INCOMING		"incoming"
#define OCS_CHAP_OUTGOING		"outgoing"
#define OCS_CHAP_TARGET			"target"
#define OCS_CHAP_INITIATOR		"initiator"
#define OCS_CHAP_TYPE_INCOMING		1
#define OCS_CHAP_TYPE_OUTGOING		2
#define OCS_CHAP_MODE_TARGET		1
#define OCS_CHAP_MODE_INITIATOR		2

typedef struct ocs_chap_user_buffer_s {
	char username[32];
	char secret[32];
	uint32_t type;
	uint32_t mode;
} ocs_chap_user_buffer_t;

#define OCS_ISCSI_ROUTE_TABLE_ENTRIES_MAX	1024

typedef struct {
	int32_t ip_type;
	uint8_t ip_addr[64];
	uint8_t subnet[64];
	uint8_t gateway[64];
} ocs_route_table_entry_t;

typedef struct {
	uint32_t entry_count;
	ocs_route_table_entry_t entry[OCS_ISCSI_ROUTE_TABLE_ENTRIES_MAX];
} ocs_route_table_t;

/**
 * @brief driver-dump actions
 */

typedef enum {
	OCS_IOCTL_DDUMP_GET,
	OCS_IOCTL_DDUMP_GET_SAVED,
	OCS_IOCTL_DDUMP_CLR_SAVED,
} ocs_ddump_action_t;

#define OCS_IOCTL_DDUMP_FLAGS_WQES	(1U << 0)
#define OCS_IOCTL_DDUMP_FLAGS_CQES	(1U << 1)
#define OCS_IOCTL_DDUMP_FLAGS_MQES	(1U << 2)
#define OCS_IOCTL_DDUMP_FLAGS_RQES	(1U << 3)
#define OCS_IOCTL_DDUMP_FLAGS_EQES	(1U << 4)

typedef struct {
	ocs_ddump_action_t action;
	uint32_t flags;
	uint32_t q_entries;
} ocs_ioctl_ddump_arg_t;

/**
 * @brief OCS_CTL_CMD_GET_DDUMP ioctl structure
 */

typedef struct {
	ocs_ioctl_ddump_arg_t args;	/*<< arguments for ddump */
	uint8_t *user_buffer;		/*<< pointer to user space buffer */
	uint32_t user_buffer_len;	/*<< length in bytes of user space buffer */
	uint32_t bytes_written;		/*<< number of bytes written */
} ocs_ioctl_ddump_t;

/**
 * @brief OCS_CTL_CMD_GET_STATUS, OCS_CTL_CMD_GET_CONFIG
 */

typedef struct {
	uint8_t *user_buffer;		/*<< pointer to user space buffer */
	uint32_t user_buffer_len;	/*<< length in bytes of user space buffer */
	uint32_t bytes_written;		/*<< number of bytes written */
} ocs_ioctl_mgmt_buffer_t;

typedef struct {
	uint8_t		*name;			/*<< Input: name of property to retrieve */
	uint16_t	name_len;		/*<< Input: Length of name */
	uint8_t		*value;			/*<< Output: user space buffer in which to place the response */
	uint32_t	value_length;		/*<< Input: size of the user space buffer */
	int		status;			/*<< Output: command execution status */
} ocs_ioctl_cmd_get_t;

typedef struct {
	uint8_t		*name;			/*<< Input: name of property to set */
	uint16_t	name_len;		/*<< Input: Length of name */
	uint8_t		*value;			/*<< Input: user space buffer which contains the new value */
	int32_t		result;			/*<< Output: result */
} ocs_ioctl_cmd_set_t;

typedef struct {
	uint8_t		*name;			/*<< Input: name of action to execute */
	uint16_t	name_len;		/*<< Input: Length of name */
	void		*arg_in;		/*<< Input: pointer to argument in user space */
	uint32_t	arg_in_length;		/*<< Input: size of arg_in in bytes */
	void		*arg_out;		/*<< Output: pointer to argument from kernel to user */
	uint32_t	arg_out_length;		/*<< Input: size of arg_out in bytes */
	int32_t		result;			/*<< Output: result */
} ocs_ioctl_action_t;

#define FC_HEADER_LEN			24

typedef struct {
	uint8_t		fc_header[FC_HEADER_LEN]; /*<< FC Header to send */
	uint8_t		*payload;		/*<< payload */
	uint32_t	payload_len;		/*<< payload length (bytes) */
	uint8_t		sof;			/*<< SOF value */
	uint8_t		eof;			/*<< EOF Value */
} ocs_ioctl_send_frame_t;

typedef struct ocs_fdmi_hba_list_info_s {
	uint32_t entry_count;
	uint64_t hba_identifier[MAX_FDMI_HBA_COUNT];
} ocs_fdmi_hba_list_info_t;

typedef struct ocs_fdmi_port_list_info_s {
	uint32_t entry_count;
	uint64_t port_identifier[MAX_FDMI_PORT_COUNT];
} ocs_fdmi_port_list_info_t;

typedef struct ocs_fdmi_hba_attr_info_s {
	uint32_t num_port_entries;
	uint64_t port_identifier[MAX_FDMI_PORT_COUNT];
	uint32_t node_name_valid:1,
		 manufacturer_valid:1,
		 serial_num_valid:1,
		 model_name_valid:1,
		 model_desc_valid:1,
		 hw_ver_valid:1,
		 driver_ver_valid:1,
		 opt_rom_ver_valid:1,
		 fw_ver_valid:1,
		 os_name_ver_valid:1,
		 max_ct_payload_valid:1,
		 node_symb_name_valid:1,
		 vendor_info_valid:1,
		 num_ports_valid:1,
		 fabric_name_valid:1,
		 bios_ver_valid:1,
		 boot_state_valid:1,
		 vendor_id_valid:1,
		 reserved:14;
	uint64_t node_name;
	uint8_t	 manufacturer[64];
	uint8_t	 serial_num[64];
	uint8_t	 model_name[256];
	uint8_t	 model_desc[256];
	uint8_t	 hw_ver[256];
	uint8_t	 driver_ver[256];
	uint8_t	 opt_rom_ver[256];
	uint8_t	 fw_ver[256];
	uint8_t	 os_name_ver[256];
	uint32_t max_ct_payload_len;
	uint8_t	 node_symb_name[256];
	uint32_t vendor_info;
	uint32_t num_ports;
	uint64_t fabric_name;
	uint8_t	 bios_ver[256];
	uint32_t boot_state;
	char	 vendor_id[8];
} ocs_fdmi_hba_attr_info_t;

typedef struct ocs_fdmi_port_attr_info_s {
	uint32_t num_port_attr_entries;
	uint32_t supported_fc_types_valid:1,
		 supported_speed_valid:1,
		 current_speed_valid:1,
		 max_frame_size_valid:1,
		 os_dev_name_valid:1,
		 host_name_valid:1,
		 node_name_valid:1,
		 port_name_valid:1,
		 port_symb_name_valid:1,
		 port_type_valid:1,
		 supported_cos_valid:1,
		 port_fabric_name_valid:1,
		 active_fc_types_valid:1,
		 port_state_valid:1,
		 num_disc_ports_valid:1,
		 port_id_valid:1,
		 reserved:16;
	uint8_t	 supported_fc4_types[32];
	uint32_t supported_speed;
	uint32_t current_port_speed;
	uint32_t max_frame_size;
	uint8_t	 os_dev_name[256];
	uint8_t	 host_name[256];
	uint64_t node_name;
	uint64_t port_name;
	uint8_t	 port_symb_name[256];
	uint32_t port_type;
	uint32_t supported_class_of_service;
	uint64_t port_fabric_name;
	uint8_t	 active_fc4_types[32];
	uint32_t port_state;
	uint32_t num_disc_ports;
	uint32_t port_identifier;
} ocs_fdmi_port_attr_info_t;

typedef struct ocs_rq_empty_gen_args_s {
	int rq_empty_enable;
	uint32_t drop_count;
	uint32_t drop_trigger;
	int rq_thread_halt_time;
	int rq_thread_halt_start_delay;
} ocs_rq_empty_gen_args_t;

typedef struct ocs_io_delay_args_s {
	uint32_t delay_min_ms;
	uint32_t delay_max_ms;
	uint64_t delay_intervel_counter;
} ocs_io_delay_args_t;

#define OCS_MAX_SLI4_ULP		8

typedef struct ocs_parity_err_count_s {
	uint32_t	pbec:16,
			tpec:16;
} ocs_parity_err_count_t;

#define OCS_UEFI_BIOS_SIGNATURE		0x42494645	/* EFI BIOS signature = "EFIB" */
#define OCS_X86_BIOS_SIGNATURE		0x534f4942	/* x86 BIOS signature = "BIOS" */
#define OCS_BIOS_REGION_ID		8
#define OCS_BIOS_REGION_ID1		9
#define OCS_EFIBIOS_REGION_ID		10
#define MAX_BOOT_LIST_ENTRIES		8

/* UEFI define flags */
#define OCS_EFI_BIOS_ENABLED		0x00000001
#define OCS_EFI_SPINUP_ENABLE		0x00000002
#define OCS_ALL_BOOTSCAN		0x00000004
#define OCS_NO_BOOTSCAN			0x00000008
#define OCS_START_STOP_ENABLE		0x00000010
#define OCS_SCSI_PATH			0x00000020
#define OCS_TIERED_BOOTSCAN		0x00000040

#define NVRAM_BOOTSCAN			0x00000000	/* no bits set in boot mask means scan by NVRAM */
#define BOOTSCAN_MASK			(ALL_BOOTSCAN | TIERED_BOOTSCAN | NO_BOOTSCAN)	/* Use mask function first */

typedef struct ocs_bios_dev_s {
	uint8_t adapt;
	uint8_t alpa;
	uint8_t select_id;
	uint8_t lun;
} ocs_bios_dev_t;

typedef struct ocs_bios_enable_entry_s {
	uint32_t enable_wwpn[2];
	uint32_t enable_did:24;
	uint32_t enable_lun:8;
} ocs_bios_enable_entry_t;

typedef union ocs_bios_port_did_s {
	struct {
		uint32_t	d_id:24;
		uint32_t	valid_flag:8;
	} bits;
	struct {
		uint8_t		alpa;
		uint8_t		area;
		uint8_t		domain;
		uint8_t		valid_flag;
	} bytes;
} ocs_bios_port_did_t;

typedef struct ocs_bios_boot_dev_s {
	uint32_t		WWPN[2];
	uint8_t			lun[8];
	ocs_bios_port_did_t	PortId;
	uint32_t		Reserved;		/* Future Use */
} ocs_bios_boot_dev_t;

/* X86 BIOS region defines */
#define X86_VALID_HOST_DID		0x0001
#define X86_VALID_ENABLE_FLAG		0x0002
#define X86_VALID_PLOGI_RTY_TMR		0x0040
#define X86_VALID_FABRIC_BOOT_DID	0x0080
#define X86_VALID_FABRIC_BOOT_WWPN	0x0100
#define X86_VALID_FABRIC_ENABLE		0x0200
#define X86_NEW_DEVICE_CONFIG		0x0400
#define X86_BOOT_ENTRY_SET		0x0780
#define X86_LUN_8_BYTE			0x0800
#define X86_ENABLE_START		0x1000
#define X86_ENABLE_EV			0x2000
#define X86_ENABLE_AUTOSECTOR		0x4000
#define X86_BIOS_ENABLED		0x01

/******************************************************************************
 *
 * Region 8 - x86 Boot BIOS data
 *
 ******************************************************************************/
typedef struct ocs_bios_struct_s {
	uint8_t			signature[4];
	uint32_t		valid_flag;
	uint8_t			host_did;
	uint8_t			enable_flag;
	ocs_bios_dev_t		boot_dev;
	uint16_t		reserved1;
	uint8_t			plogi_timer;
	uint8_t			wwpn[4];
	uint32_t		f_boot_did:24;
	uint32_t		f_boot_lun:8;
	uint8_t			byte0_lun[8];
	uint32_t		link_speed;
	uint8_t			enable_fa_wwn;		/* EFI needs a byte to create VFR token */
	uint8_t			reserved2[11];
	uint8_t			reserved3[3];
	uint32_t		reserved4:28;
	uint32_t		boot_once:4;
	ocs_bios_enable_entry_t	enable_entry[7];	/* x86 Region 9 data (boot entries 2-8) */
} ocs_bios_struct_t;

/******************************************************************************
 *
 * Region 10 - UEFI Boot BIOS data
 *
 ******************************************************************************/
typedef struct ocs_uefi_bios_struct_s {
	uint8_t			signature[4];		/* Signature = "EFIB" */
	ocs_bios_boot_dev_t	boot_dev_list[MAX_BOOT_LIST_ENTRIES];
	uint32_t		efi_flag;
	uint16_t		max_luns;
	uint8_t			reserved_hard_alpa;
	uint8_t			plogi_timer;
	uint8_t			topology;
	uint8_t			reset_delay_timer;
	uint8_t			link_speed;
	uint8_t			reserved[49];
} ocs_uefi_bios_struct_t;

/**
 * @brief linkcfg strings
 */
#define OCS_CONFIG_LINKCFG_4X10G	"ETH_4x10G"
#define OCS_CONFIG_LINKCFG_1X40G	"ETH_1x40G"
#define OCS_CONFIG_LINKCFG_2X16G	"FC_2x16G"
#define OCS_CONFIG_LINKCFG_4X8G		"FC_4x8G"
#define OCS_CONFIG_LINKCFG_4X1G		"FC_4x1G"
#define OCS_CONFIG_LINKCFG_2X10G	"ETH_2x10G"
#define OCS_CONFIG_LINKCFG_2X10G_2X8G	"ETH_2x10G_FC_2x8G"
#define OCS_CONFIG_LINKCFG_UNKNOWN	"UNKNOWN"

/* IOCTL interface for DHCHAP config */
struct ocs_ioctl_dhchap_config {
#define IOCTL_DHCHAP_MODE_DISABLE	1
#define IOCTL_DHCHAP_MODE_ACTIVE	2
#define IOCTL_DHCHAP_MODE_PASSIVE	3
	uint8_t		mode;
#define IOCTL_DHCHAP_FLAGS_BIDI		(1U << 0)
#define IOCTL_DHCHAP_FLAGS_VALID	(1U << 7)
	uint8_t		flags;

#define IOCTL_HASH_MD5			1
#define IOCTL_HASH_SHA1			2
#define IOCTL_MAX_HASH			2
	/* list in the order of priority */
	uint8_t		hash_list[IOCTL_MAX_HASH];

#define IOCTL_DH_GROUP_NULL		1
#define IOCTL_DH_GROUP_1024		2
#define IOCTL_DH_GROUP_1280		3
#define IOCTL_DH_GROUP_1536		4
#define IOCTL_DH_GROUP_2048		5
#define IOCTL_MAX_DH_GROUP		5
	/* list in the order of priority */
	uint8_t		dh_grp_list[IOCTL_MAX_DH_GROUP];
	uint32_t	reauth_interval;
};

/* ensure that this stays the same as struct ocs_auth_cfg_id{} */
struct ocs_ioctl_dhchap_id {
	uint8_t		local_wwpn[8];
	uint8_t		remote_wwpn[8];
};

/* IOCTL interface for DHCHAP secret key pair configured by user */
struct ocs_ioctl_dhchap_secret {
	uint8_t		local_wwpn[8];
	uint8_t		remote_wwpn[8];
#define IOCTL_PSSWD_MIN_LEN 		12
#define IOCTL_PSSWD_MAX_LEN 		40
	/* Null terminated strings */
	char		local_pass[IOCTL_PSSWD_MAX_LEN + 1];
	char		remote_pass[IOCTL_PSSWD_MAX_LEN + 1];
};

#define OCS_IOCTL_CMD_BASE		'o'
#define OCS_IOCTL_CMD_TEST		_IOWR(OCS_IOCTL_CMD_BASE, 1, ocs_ioctl_test_t)
#define OCS_IOCTL_CMD_ELXU_MBOX		_IOWR(OCS_IOCTL_CMD_BASE, 2, ocs_ioctl_elxu_mbox_t)
#define OCS_IOCTL_CMD_SCSI_CMD		_IOWR(OCS_IOCTL_CMD_BASE, 3, ocs_ioctl_scsi_cmd_t)
#define OCS_IOCTL_CMD_DRIVER_INFO	_IOWR(OCS_IOCTL_CMD_BASE, 4, ocs_ioctl_driver_info_t)
#define OCS_IOCTL_CMD_ECD_HELPER	_IOWR(OCS_IOCTL_CMD_BASE, 5, ocs_ioctl_ecd_helper_t)
#define OCS_IOCTL_CMD_CONNECTION_INFO	_IOWR(OCS_IOCTL_CMD_BASE, 6, ocs_ioctl_connection_info_t)
#define OCS_IOCTL_CMD_VPORT		_IOWR(OCS_IOCTL_CMD_BASE, 7, ocs_ioctl_vport_t)
#define OCS_IOCTL_CMD_GET_DDUMP		_IOWR(OCS_IOCTL_CMD_BASE, 8, ocs_ioctl_ddump_t)
#define OCS_IOCTL_CMD_MGMT_GET		_IOWR(OCS_IOCTL_CMD_BASE, 9, ocs_ioctl_cmd_get_t)
#define OCS_IOCTL_CMD_MGMT_GET_ALL	_IOWR(OCS_IOCTL_CMD_BASE, 10, ocs_ioctl_mgmt_buffer_t)
#define OCS_IOCTL_CMD_MGMT_SET		_IOWR(OCS_IOCTL_CMD_BASE, 11, ocs_ioctl_cmd_set_t)
#define OCS_IOCTL_CMD_MGMT_LIST		_IOWR(OCS_IOCTL_CMD_BASE, 12, ocs_ioctl_mgmt_buffer_t)
#define OCS_IOCTL_CMD_MGMT_EXEC		_IOWR(OCS_IOCTL_CMD_BASE, 13, ocs_ioctl_action_t)
#define OCS_IOCTL_CMD_LINK_ONLINE	_IOWR(OCS_IOCTL_CMD_BASE, 16, int)
#define OCS_IOCTL_CMD_GEN_DUMP		_IOWR(OCS_IOCTL_CMD_BASE, 17, int)
#define OCS_IOCTL_CMD_UNLOAD		_IO(OCS_IOCTL_CMD_BASE, 18)
#define OCS_IOCTL_CMD_SEND_FRAME	_IOWR(OCS_IOCTL_CMD_BASE, 19, ocs_ioctl_send_frame_t)

#endif // __OCS_IOCTL_H__
