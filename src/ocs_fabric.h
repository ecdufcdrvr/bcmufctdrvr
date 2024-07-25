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
 * Declarations for the interface exported by ocs_fabric
 */


#if !defined(__OCS_FABRIC_H__)
#define __OCS_FABRIC_H__
extern void *__ocs_fabric_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_fabric_flogi_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_fabric_domain_attach_wait(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_fabric_wait_domain_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

extern void *__ocs_vport_fabric_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_fabric_fdisc_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_fabric_wait_sport_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

extern void *__ocs_ns_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_ns_plogi_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_ns_rftid_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_ns_rffid_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void * __ocs_ns_nvme_rffid_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_ns_wait_node_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_fabric_wait_attach_evt_shutdown(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_ns_logo_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_ns_gidpt_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_ns_gffid_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_ns_idle(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_ns_gidpt_delay(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_ns_ganxt_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

extern void *__ocs_fdmi_reg_port_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_fdmi_rhba_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_fdmi_dprt_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_fdmi_dhba_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

extern void *__ocs_mgmt_srv_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_mgmt_srv_plogi_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_mgmt_srv_wait_node_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_mgmt_srv_idle(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

extern void *__ocs_fabctl_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_fabctl_wait_node_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_fabctl_wait_scr_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_fabctl_wait_rdf_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_fabctl_ready(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_fabctl_wait_ls_acc_cmpl(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_fabric_idle(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

extern void *__ocs_p2p_rnode_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_p2p_domain_attach_wait(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_p2p_wait_flogi_acc_cmpl(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_p2p_wait_plogi_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_p2p_wait_plogi_rsp_recvd_prli(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_p2p_wait_domain_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_p2p_wait_node_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

extern int32_t ocs_rnode_is_nport(fc_plogi_payload_t *remote_sparms);
extern int32_t ocs_rnode_is_npiv_capable(fc_plogi_payload_t *remote_sparms);
extern int32_t ocs_p2p_setup(ocs_sport_t *sport);
extern void ocs_fabric_set_topology(ocs_node_t *node, ocs_sport_topology_e topology);
extern void ocs_fabric_notify_topology(ocs_node_t *node);
extern void *__ocs_ns_wait_domain_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_mgmt_srv_wait_domain_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

/*
 * HBA Attribute Types
 */
#define  RHBA_NODENAME           0x1 /* 8 byte WWNN */
#define  RHBA_MANUFACTURER       0x2 /* 4 to 64 byte ASCII string */
#define  RHBA_SERIAL_NUMBER      0x3 /* 4 to 64 byte ASCII string */
#define  RHBA_MODEL              0x4 /* 4 to 256 byte ASCII string */
#define  RHBA_MODEL_DESCRIPTION  0x5 /* 4 to 256 byte ASCII string */
#define  RHBA_HARDWARE_VERSION   0x6 /* 4 to 256 byte ASCII string */
#define  RHBA_DRIVER_VERSION     0x7 /* 4 to 256 byte ASCII string */
#define  RHBA_OPTION_ROM_VERSION 0x8 /* 4 to 256 byte ASCII string */
#define  RHBA_FIRMWARE_VERSION   0x9 /* 4 to 256 byte ASCII string */
#define  RHBA_OS_NAME_VERSION    0xa /* 4 to 256 byte ASCII string */
#define  RHBA_MAX_CT_PAYLOAD_LEN 0xb /* 32-bit unsigned int */
#define  RHBA_SYM_NODENAME       0xc /* 4 to 256 byte ASCII string */
#define  RHBA_VENDOR_INFO        0xd  /* 32-bit unsigned int */
#define  RHBA_NUM_PORTS          0xe  /* 32-bit unsigned int */
#define  RHBA_FABRIC_NAME        0xf  /* 8 byte WWNN */
#define  RHBA_BIOS_VERSION       0x10 /* 4 to 256 byte ASCII string */
#define  RHBA_BIOS_STATE         0x11 /* 32-bit unsigned int */
#define  RHBA_VENDOR_ID          0xe0 /* 8 byte ASCII string */

/*
 * Port Attrubute Types
 */
#define  RPRT_SUPPORTED_FC4_TYPES     0x1 /* 32 byte binary array */
#define  RPRT_SUPPORTED_SPEED         0x2 /* 32-bit unsigned int */
#define  RPRT_PORT_SPEED              0x3 /* 32-bit unsigned int */
#define  RPRT_MAX_FRAME_SIZE          0x4 /* 32-bit unsigned int */
#define  RPRT_OS_DEVICE_NAME          0x5 /* 4 to 256 byte ASCII string */
#define  RPRT_HOST_NAME               0x6 /* 4 to 256 byte ASCII string */
#define  RPRT_NODENAME                0x7 /* 8 byte WWNN */
#define  RPRT_PORTNAME                0x8 /* 8 byte WWPN */
#define  RPRT_SYM_PORTNAME            0x9 /* 4 to 256 byte ASCII string */
#define  RPRT_PORT_TYPE               0xa /* 32-bit unsigned int */
#define  RPRT_SUPPORTED_CLASS         0xb /* 32-bit unsigned int */
#define  RPRT_FABRICNAME              0xc /* 8 byte Fabric WWPN */
#define  RPRT_ACTIVE_FC4_TYPES        0xd /* 32 byte binary array */
#define  RPRT_PORT_STATE              0x101 /* 32-bit unsigned int */
#define  RPRT_DISC_PORT               0x102 /* 32-bit unsigned int */
#define  RPRT_PORT_ID                 0x103 /* 32-bit unsigned int */
#define  RPRT_SMART_SECURITY          0xf106 /* 32-bit unsigned int */

/* Bit mask for FDMI-1 defined PORT attributes */
#define FDMI_PORT_MASK_V1 0x0000003f

/* Bit mask for FDMI-2 defined PORT attributes */
#define FDMI_PORT_MASK_V2 0x0000ffff

/* Bit mask for FDMI-2 defined HBA attributes */
/* Definitions for HBA / Port attribute entries */
typedef struct ocs_fdmi_attr_def_s { /* Defined in TLV format */
	uint16_t attr_type;
	uint16_t attr_len;
	ocs_fdmi_attr_entry_t attr_value;  /* Marks start of Value (ATTRIBUTE_ENTRY) */
} __attribute__ ((packed)) ocs_fdmi_attr_def_t;

extern int ocs_fdmi_hba_attr_wwnn(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_manufacturer(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_sn(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_model(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_description(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_drvr_ver(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_rom_ver(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_fmw_ver(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_os_ver(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_ct_len(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_symbolic_name(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_vendor_info(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_num_ports(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_fabric_name(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_bios_ver(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_bios_state(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_hba_attr_vendor_id(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);

extern int ocs_fdmi_port_attr_supported_fc4type(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_port_attr_supported_speed(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_port_attr_speed(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_port_attr_max_frame(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_port_attr_os_devname(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_port_attr_host_name(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_port_attr_wwnn(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_port_attr_wwpn(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_port_attr_symbolic_name(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_port_attr_port_type(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_port_attr_class(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_port_attr_fabric_name(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_port_attr_active_fc4type(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_port_attr_port_state(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);
extern int ocs_fdmi_port_attr_nportid(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);

extern int32_t
ocs_process_tdz_rsp_hdr(ocs_node_t *node, fcct_iu_header_t *hdr, ocs_tdz_status_t *tdz_status);
extern int32_t
ocs_process_tdz_gfez_rsp_buf(ocs_node_t *node, fcct_tdz_gfez_rsp_t *gfez_rsp, ocs_tdz_status_t *tdz_status);
extern int32_t
ocs_process_tdz_gapz_rsp_buf(ocs_node_t *node, fcct_tdz_gapz_rsp_t *gapz_rsp, ocs_tdz_get_peer_zone_rsp_t *tdz_rsp);

extern int32_t
ocs_process_fdmi_grhl_payload(ocs_node_t *node, fcct_fdmi_grhl_rsp_t *grhl,
			      uint32_t grhl_len, void *buf);
extern int32_t
ocs_process_fdmi_grpl_payload(ocs_node_t *node, fcct_fdmi_grpl_rsp_t *grpl,
			      uint32_t grpl_len, void *buf);
extern int32_t
ocs_process_fdmi_ghat_payload(ocs_node_t *node, fcct_fdmi_ghat_rsp_t *ghat,
			      uint32_t ghat_len, void *buf);
extern int32_t
ocs_process_fdmi_gpat_payload(ocs_node_t *node, fcct_fdmi_gpat_rsp_t *gpat,
			      uint32_t gpat_len, void *buf);
extern int32_t
ocs_process_ganxt_payload(ocs_node_t *node, fcct_ganxt_acc_t *ganxt,
			      uint32_t ganxt_len, void *buf);
extern int ocs_sport_fpin_send_event(ocs_sport_t *sport, void *args);
#endif // __OCS_FABRIC_H__
