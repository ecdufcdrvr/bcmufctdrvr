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
 * The ocs_mgmt top level functions for Fibre Channel.
 */

/**
 * @defgroup mgmt Management Functions
 */

#include "ocs.h"
#include "ocs_mgmt.h"
#include "ocs_vpd.h"
#include "ocs_recovery.h"

/*
 * Lancer board temperature command helper structure.
 */
typedef struct ocs_mgmt_lancer_temp_result_s {
	ocs_t *ocs;
	ocs_sem_t waitsem;
	uint32_t resp[OCS_DUMP_TYPE4_WKI_TAG_SAT_TEM_RESP_LEN / 4];
	int32_t status;
} ocs_mgmt_lancer_temp_result_t;

/* Executables*/

static int ocs_mgmt_firmware_write(ocs_t *ocs, char *, void *buf, uint32_t buf_len, void*, uint32_t);
static int ocs_mgmt_firmware_reset(ocs_t *ocs, char *name, void *arg_in, uint32_t arg_in_length,
				void *arg_out, uint32_t arg_out_length);
static int ocs_mgmt_trigger_error(ocs_t *ocs, char *name, void *arg_in, uint32_t arg_in_length,
				void *arg_out, uint32_t arg_out_length);

#if !defined(OCS_USPACE_SPDK)
static int32_t
ocs_mgmt_gen_rq_empty(ocs_t *ocs, char *name, void *buf, uint32_t buf_len, void *arg_out, uint32_t arg_out_length);
#endif

#if defined(OCS_INCLUDE_IO_DELAY)
static int32_t
ocs_mgmt_io_delay_config(ocs_t *ocs, char *name, void *buf, uint32_t buf_len, void *arg_out, uint32_t arg_out_length);
#endif

static int ocs_mgmt_function_reset(ocs_t *ocs, char *, void *buf, uint32_t buf_len, void*, uint32_t);
static int ocs_mgmt_port_migration(ocs_t *ocs, char *, void *buf, uint32_t buf_len, void*, uint32_t);
static int ocs_mgmt_read_parity_stats(ocs_t *ocs, char *name, void *arg_in, uint32_t arg_in_length,
				void *arg_out, uint32_t arg_out_length);

static void ocs_mgmt_fw_write_cb(int32_t status, int32_t ext_status, uint32_t actual_write_length,
				 uint32_t change_status, void *arg);
static int ocs_mgmt_force_assert(ocs_t *ocs, char *, void *buf, uint32_t buf_len, void*, uint32_t);

#if defined(OCS_INCLUDE_RAMD)
static int32_t
ocs_mgmt_read_phys(ocs_t *ocs, char *, void *, uint32_t , void *, uint32_t);
#endif

static int
ocs_mgmt_get_dhchap_config(ocs_t *ocs, char *name,
			   void *arg_in, uint32_t arg_in_len,
			   void *arg_out, uint32_t arg_out_len);
static int
ocs_mgmt_set_dhchap_config(ocs_t *ocs, char *name,
			   void *arg_in, uint32_t arg_in_len,
			   void *arg_out, uint32_t arg_out_len);
static int
ocs_mgmt_add_dhchap_secret(ocs_t *ocs, char *name,
			   void *arg_in, uint32_t arg_in_len,
			   void *arg_out, uint32_t arg_out_len);
static int
ocs_mgmt_list_dhchap_secrets(ocs_t *ocs, char *name,
			     void *arg_in, uint32_t arg_in_len,
			     void *arg_out, uint32_t arg_out_len);
static int
ocs_mgmt_num_dhchap_secrets(ocs_t *ocs, char *name,
			    void *arg_in, uint32_t arg_in_len,
			    void *arg_out, uint32_t arg_out_len);

/* Getters */

static void get_nodes_count(ocs_t *, char *, ocs_textbuf_t*);
static void get_desc(ocs_t *, char *, ocs_textbuf_t*);
static void get_fw_rev(ocs_t *, char *, ocs_textbuf_t*);
static void get_fw_rev2(ocs_t *, char *, ocs_textbuf_t*);
static void get_ipl(ocs_t *, char *, ocs_textbuf_t*);
static void get_wwnn(ocs_t *, char *, ocs_textbuf_t*);
static void get_wwpn(ocs_t *, char *, ocs_textbuf_t*);
static void get_sn(ocs_t *, char *, ocs_textbuf_t*);
static void get_pn(ocs_t *, char *, ocs_textbuf_t*);
static void get_sli4_intf_reg(ocs_t *, char *, ocs_textbuf_t*);
static void get_phy_port_num(ocs_t *, char *, ocs_textbuf_t*);
static void get_asic_id(ocs_t *, char *, ocs_textbuf_t*);
static void get_pci_vendor(ocs_t *, char *, ocs_textbuf_t*);
static void get_pci_device(ocs_t *, char *, ocs_textbuf_t*);
static void get_pci_subsystem_vendor(ocs_t *, char *, ocs_textbuf_t*);
static void get_pci_subsystem_device(ocs_t *, char *, ocs_textbuf_t*);
static void get_businfo(ocs_t *, char *, ocs_textbuf_t*);
static void get_sfp_a0(ocs_t *, char *, ocs_textbuf_t*);
static void get_sfp_a2(ocs_t *, char *, ocs_textbuf_t*);
static void get_active_xri_count(ocs_t *, char *, ocs_textbuf_t*);
static void get_active_rpi_count(ocs_t *, char *, ocs_textbuf_t*);
static void get_lip_count(ocs_t *, char *, ocs_textbuf_t*);
static void get_hw_rev1(ocs_t *, char *, ocs_textbuf_t*);
static void get_hw_rev2(ocs_t *, char *, ocs_textbuf_t*);
static void get_hw_rev3(ocs_t *, char *, ocs_textbuf_t*);
static void get_fw_dump_present(ocs_t *, char *, ocs_textbuf_t*);
static void get_fw_dump_max_size(ocs_t *, char *, ocs_textbuf_t*);
static void get_debug_mq_dump(ocs_t*, char*, ocs_textbuf_t*);
static void get_debug_cq_dump(ocs_t*, char*, ocs_textbuf_t*);
static void get_debug_wq_dump(ocs_t*, char*, ocs_textbuf_t*);
static void get_debug_eq_dump(ocs_t*, char*, ocs_textbuf_t*);
static void get_logmask(ocs_t*, char*, ocs_textbuf_t*);
static void get_current_speed(ocs_t*, char*, ocs_textbuf_t*);
static void get_current_topology(ocs_t*, char*, ocs_textbuf_t*);
static void get_current_link_state(ocs_t*, char*, ocs_textbuf_t*);
static void get_logical_link_speed(ocs_t*, char*, ocs_textbuf_t*);
static void get_configured_speed(ocs_t*, char*, ocs_textbuf_t*);
static void get_configured_topology(ocs_t*, char*, ocs_textbuf_t*);
static void get_configured_link_state(ocs_t*, char*, ocs_textbuf_t*);
#if defined(ENABLE_FABRIC_EMULATION)
static void get_femul_enable(ocs_t*, char*, ocs_textbuf_t*);
#endif
static void get_linkcfg(ocs_t*, char*, ocs_textbuf_t*);
static void get_req_wwnn(ocs_t*, char*, ocs_textbuf_t*);
static void get_req_wwpn(ocs_t*, char*, ocs_textbuf_t*);
static void get_nodedb_mask(ocs_t*, char*, ocs_textbuf_t*);
static void get_profile_list(ocs_t*, char*, ocs_textbuf_t*);
static void get_active_profile(ocs_t*, char*, ocs_textbuf_t*);
static void get_port_protocol(ocs_t*, char*, ocs_textbuf_t*);
static void get_driver_version(ocs_t*, char*, ocs_textbuf_t*);
static void get_chip_type(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf);
static void get_supported_speeds(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf);
static void get_tgt_rscn_delay(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf);
static void get_tgt_rscn_period(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf);
static void get_inject_drop_cmd(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf);
static void get_inject_free_drop_cmd(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf);
static void get_inject_drop_data(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf);
static void get_inject_drop_resp(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf);
static void get_cmd_err_inject(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf);
static void get_cmd_delay_value(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf);
static void get_nv_wwpn(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf);
static void get_nv_wwnn(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf);
static void get_loglevel(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf);
static void get_node_abort_cnt(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf);

/* Setters */
static int set_debug_mq_dump(ocs_t*, char*, char*);
static int set_debug_cq_dump(ocs_t*, char*, char*);
static int set_debug_wq_dump(ocs_t*, char*, char*);
static int set_debug_eq_dump(ocs_t*, char*, char*);
static int set_logmask(ocs_t*, char*, char*);
static int set_configured_speed(ocs_t*, char*, char*);
static int set_configured_topology(ocs_t*, char*, char*);
static int set_configured_link_state(ocs_t*, char*, char*);
#if defined(ENABLE_FABRIC_EMULATION)
static int set_femul_enable(ocs_t*, char*, char*);
#endif
static int set_linkcfg(ocs_t*, char*, char*);
static int set_req_wwnn(ocs_t*, char*, char*);
static int set_req_wwpn(ocs_t*, char*, char*);
static int set_nodedb_mask(ocs_t*, char*, char*);
static int set_port_protocol(ocs_t*, char*, char*);
static int set_active_profile(ocs_t*, char*, char*);
static int set_tgt_rscn_delay(ocs_t*, char*, char*);
static int set_tgt_rscn_period(ocs_t*, char*, char*);
static int set_inject_drop_cmd(ocs_t*, char*, char*);
static int set_inject_free_drop_cmd(ocs_t*, char*, char*);
static int set_inject_drop_data(ocs_t*, char*, char*);
static int set_inject_drop_resp(ocs_t*, char*, char*);
static int set_cmd_err_inject(ocs_t*, char*, char*);
static int set_cmd_delay_value(ocs_t*, char*, char*);
static int set_nv_wwn(ocs_t*, char*, char*);
static int set_loglevel(ocs_t*, char*, char*);

static int32_t
ocs_mgmt_get_bios_config(ocs_t *ocs, char *name,
			 void *arg_in, uint32_t arg_in_len,
			 void *arg_out, uint32_t arg_out_len);

static int32_t
ocs_mgmt_update_uefi_bios_config(ocs_t *ocs, char *name,
				 void *arg_in, uint32_t arg_in_len,
				 void *arg_out, uint32_t arg_out_len);

static int32_t
ocs_mgmt_update_x86_bios_config(ocs_t *ocs, char *name,
				void *arg_in, uint32_t arg_in_len,
				void *arg_out, uint32_t arg_out_len);

static void ocs_mgmt_linkcfg_cb(int32_t status, uintptr_t value, void *arg);
extern int32_t ocs_read_temperature_lancer(ocs_t *ocs, void *buf, uint32_t buf_len);

#if defined(OCS_INCLUDE_RAMD)
static void* find_address_in_target(ocs_ramdisc_t **ramdisc_array, uint32_t ramdisc_count, uintptr_t target_addr);
#endif

ocs_mgmt_table_entry_t mgmt_table[] = {
		{"nodes_count", get_nodes_count, NULL, NULL},
		{"desc", get_desc, NULL, NULL},
		{"fw_rev", get_fw_rev, NULL, NULL},
		{"fw_rev2", get_fw_rev2, NULL, NULL},
		{"ipl", get_ipl, NULL, NULL},
		{"hw_rev1", get_hw_rev1, NULL, NULL},
		{"hw_rev2", get_hw_rev2, NULL, NULL},
		{"hw_rev3", get_hw_rev3, NULL, NULL},
		{"wwnn", get_wwnn, NULL, NULL},
		{"wwpn", get_wwpn, NULL, NULL},
		{"sn", get_sn, NULL, NULL},
		{"pn", get_pn, NULL, NULL},
		{"sli4_intf_reg", get_sli4_intf_reg, NULL, NULL},
		{"phy_port_num", get_phy_port_num, NULL, NULL},
		{"asic_id_reg", get_asic_id, NULL, NULL},
		{"pci_vendor", get_pci_vendor, NULL, NULL},
		{"pci_device", get_pci_device, NULL, NULL},
		{"pci_subsystem_vendor", get_pci_subsystem_vendor, NULL, NULL},
		{"pci_subsystem_device", get_pci_subsystem_device, NULL, NULL},
		{"businfo", get_businfo, NULL, NULL},
		{"sfp_a0", get_sfp_a0, NULL, NULL},
		{"sfp_a2", get_sfp_a2, NULL, NULL},
		{"active_rpi_count", get_active_rpi_count, NULL, NULL},
		{"active_xri_count", get_active_xri_count, NULL, NULL},
		{"lip_count", get_lip_count, NULL, NULL},
		{"profile_list", get_profile_list, NULL, NULL},
		{"driver_version", get_driver_version, NULL, NULL},
		{"current_speed", get_current_speed, NULL, NULL},
		{"current_topology", get_current_topology, NULL, NULL},
		{"current_link_state", get_current_link_state, NULL, NULL},
		{"logical_link_speed", get_logical_link_speed, NULL, NULL},
		{"chip_type", get_chip_type, NULL, NULL},
		{"supported_speeds", get_supported_speeds, NULL, NULL},
		{"configured_speed", get_configured_speed, set_configured_speed, NULL},
		{"configured_topology", get_configured_topology, set_configured_topology, NULL},
		{"configured_link_state", get_configured_link_state, set_configured_link_state, NULL},
#if defined(ENABLE_FABRIC_EMULATION)
		{"femul_enable", get_femul_enable, set_femul_enable, NULL},
#endif
		{"debug_mq_dump", get_debug_mq_dump, set_debug_mq_dump, NULL},
		{"debug_cq_dump", get_debug_cq_dump, set_debug_cq_dump, NULL},
		{"debug_wq_dump", get_debug_wq_dump, set_debug_wq_dump, NULL},
		{"debug_eq_dump", get_debug_eq_dump, set_debug_eq_dump, NULL},
		{"logmask", get_logmask, set_logmask, NULL},
		{"loglevel", get_loglevel, set_loglevel, NULL},
		{"linkcfg", get_linkcfg, set_linkcfg, NULL},
		{"requested_wwnn", get_req_wwnn, set_req_wwnn, NULL},
		{"requested_wwpn", get_req_wwpn, set_req_wwpn, NULL},
		{"nodedb_mask", get_nodedb_mask, set_nodedb_mask, NULL},
		{"port_protocol", get_port_protocol, set_port_protocol, NULL},
		{"active_profile", get_active_profile, set_active_profile, NULL},
		{"fw_dump_present", get_fw_dump_present, NULL, NULL},
		{"fw_dump_max_size", get_fw_dump_max_size, NULL, NULL},
		{"firmware_write", NULL, NULL, ocs_mgmt_firmware_write},
		{"firmware_reset", NULL, NULL, ocs_mgmt_firmware_reset},
		{"trigger_error", NULL, NULL, ocs_mgmt_trigger_error},
#if !defined(OCS_USPACE_SPDK)
		/* Commands not supported in uspace SPDK driver */
		{"gen_rq_empty", NULL, NULL, ocs_mgmt_gen_rq_empty},
#endif
		{"function_reset", NULL, NULL, ocs_mgmt_function_reset},
		{"port_migration", NULL, NULL, ocs_mgmt_port_migration},
#if defined(OCS_INCLUDE_RAMD)
		{"read_phys", NULL, NULL, ocs_mgmt_read_phys},
#endif
#if defined(OCS_INCLUDE_IO_DELAY)
		{"io_delay_config", NULL, NULL, ocs_mgmt_io_delay_config},
#endif
		{"force_assert", NULL, NULL, ocs_mgmt_force_assert},
		{"get_parity_stats", NULL, NULL, ocs_mgmt_read_parity_stats},
		{"tgt_rscn_delay", get_tgt_rscn_delay, set_tgt_rscn_delay, NULL},
		{"tgt_rscn_period", get_tgt_rscn_period, set_tgt_rscn_period, NULL},
		{"inject_drop_cmd", get_inject_drop_cmd, set_inject_drop_cmd, NULL},
		{"inject_free_drop_cmd", get_inject_free_drop_cmd, set_inject_free_drop_cmd, NULL},
		{"inject_drop_data", get_inject_drop_data, set_inject_drop_data, NULL},
		{"inject_drop_resp", get_inject_drop_resp, set_inject_drop_resp, NULL},
		{"cmd_err_inject", get_cmd_err_inject, set_cmd_err_inject, NULL},
		{"cmd_delay_value", get_cmd_delay_value, set_cmd_delay_value, NULL},
		{"nv_wwpn", get_nv_wwpn, NULL, NULL},
		{"nv_wwnn", get_nv_wwnn, NULL, NULL},
		{"nv_wwn", NULL, set_nv_wwn, NULL},
		{"node_abort_cnt", get_node_abort_cnt, NULL, NULL},
		{"set_dhchap_config", NULL, NULL, ocs_mgmt_set_dhchap_config},
		{"get_dhchap_config", NULL, NULL, ocs_mgmt_get_dhchap_config},
		{"add_dhchap_secret", NULL, NULL, ocs_mgmt_add_dhchap_secret},
		{"list_dhchap_secrets", NULL, NULL, ocs_mgmt_list_dhchap_secrets},
		{"num_dhchap_secrets", NULL, NULL, ocs_mgmt_num_dhchap_secrets},
		{"get_bios_config_info", NULL, NULL, ocs_mgmt_get_bios_config},
		{"update_uefi_bios_config_info", NULL, NULL, ocs_mgmt_update_uefi_bios_config},
		{"update_x86_bios_config_info", NULL, NULL, ocs_mgmt_update_x86_bios_config},
};

/**
 * @ingroup mgmt
 * @brief Get a list of options supported by the driver.
 *
 * @par Description
 * This is the top level "get list" handler for the driver. It
 * performs the following:
 *  - Adds entries to the textbuf for any actions supported by this level in the driver.
 *  - Calls a back-end function to add any actions supported by the back-end.
 *  - Calls a function on each child (domain) to recursively add supported actions.
 *
 * @param ocs Pointer to the ocs structure.
 * @param textbuf Pointer to an ocs_textbuf, which is used to accumulate the results.
 *
 * @return Returns 0 on success, or a negative value on failure.
 */

void
ocs_mgmt_get_list(ocs_t *ocs, ocs_textbuf_t *textbuf)
{
	ocs_domain_t *domain;
	uint32_t i;
	int access;

	ocs_mgmt_start_unnumbered_section(textbuf, "ocs");

	for (i=0;i<ARRAY_SIZE(mgmt_table);i++) {
		access = 0;
		if (mgmt_table[i].get_handler) {
			access |= MGMT_MODE_RD;
		}
		if (mgmt_table[i].set_handler) {
			access |= MGMT_MODE_WR;
		}
		if (mgmt_table[i].action_handler) {
			access |= MGMT_MODE_EX;
		}
		ocs_mgmt_emit_property_name(textbuf, access, mgmt_table[i].name);
	}

	if ((ocs->mgmt_functions) && (ocs->mgmt_functions->get_list_handler)) {
		ocs->mgmt_functions->get_list_handler(textbuf, ocs);
	}

	if ((ocs->tgt_mgmt_functions) && (ocs->tgt_mgmt_functions->get_list_handler)) {
		ocs->tgt_mgmt_functions->get_list_handler(textbuf, &(ocs->tgt_ocs));
	}

	/* Have each of my children add their actions */
	if (ocs_device_lock_try(ocs) == TRUE) {

		/* If we get here then we are holding the device lock */
		ocs_list_foreach(&ocs->domain_list, domain) {
			if ((domain->mgmt_functions) && (domain->mgmt_functions->get_list_handler)) {
				domain->mgmt_functions->get_list_handler(textbuf, domain);
			}
		}
		ocs_device_unlock(ocs);
	}

	ocs_mgmt_end_unnumbered_section(textbuf, "ocs");

}

/**
 * @ingroup mgmt
 * @brief Return the value of a management item.
 *
 * @par Description
 * This is the top level "get" handler for the driver. It
 * performs the following:
 *  - Checks that the qualifier portion of the name begins with my qualifier (ocs).
 *  - If the remaining part of the name matches a parameter that is known at this level,
 *    writes the value into textbuf.
 *  - If the name is not known, sends the request to the back-ends to fulfill (if possible).
 *  - If the request has not been fulfilled by the back-end,
 *    passes the request to each of the children (domains) to
 *    have them (recursively) try to respond.
 *
 *  In passing the request to other entities, the request is considered to be answered
 *  when a response has been written into textbuf, indicated by textbuf->buffer_written
 *  being non-zero.
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Name of the status item to be retrieved.
 * @param textbuf Pointer to an ocs_textbuf, which is used to return the results.
 *
 * @return Returns 0 if the value was found and returned, or -1 if an error occurred.
 */


int
ocs_mgmt_get(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_domain_t *domain;
	char qualifier[6];
	int retval = -1;
	uint32_t i;

	ocs_mgmt_start_unnumbered_section(textbuf, "ocs");


	snprintf(qualifier, sizeof(qualifier), "/ocs");

	/* See if the name starts with my qualifier.  If not then this request isn't for me */
	if (ocs_strncmp(name, qualifier, strlen(qualifier)) == 0) {
		char *unqualified_name = name + strlen(qualifier) + 1;

		for (i=0;i<ARRAY_SIZE(mgmt_table);i++) {
			if (ocs_strcmp(unqualified_name, mgmt_table[i].name) == 0) {
				if (mgmt_table[i].get_handler) {
					mgmt_table[i].get_handler(ocs, name, textbuf);
					ocs_mgmt_end_unnumbered_section(textbuf, "ocs");
					return 0;
				}
			}
		}

		if ((ocs->mgmt_functions) && (ocs->mgmt_functions->get_handler)) {
			retval = ocs->mgmt_functions->get_handler(textbuf, qualifier, (char*)name, ocs);
		}

		if (retval != 0) {
			if ((ocs->tgt_mgmt_functions) && (ocs->tgt_mgmt_functions->get_handler)) {
				retval = ocs->tgt_mgmt_functions->get_handler(textbuf, qualifier,
						(char*)name, &(ocs->tgt_ocs));
			}
		}

		/* The driver didn't handle it, pass it to each domain */
		if (retval != 0) {
			if (!ocs_device_lock_try_timeout(ocs, 1000, 3)) {
				ocs_log_err(ocs, "Failed to acquire device lock\n");
				ocs_mgmt_end_unnumbered_section(textbuf, "ocs");
				return -EBUSY;
			}

			/* Return success if domain list is empty */
			if (ocs_list_empty(&ocs->domain_list)) {
				ocs_device_unlock(ocs);
				ocs_mgmt_end_unnumbered_section(textbuf, "ocs");
				return 0;
			}

			ocs_list_foreach(&ocs->domain_list, domain) {
				if ((domain->mgmt_functions) && (domain->mgmt_functions->get_handler)) {
					retval = domain->mgmt_functions->get_handler(textbuf, qualifier, (char*)name, domain);
				}

				if (retval ==  0) {
					break;
				}
			}
			ocs_device_unlock(ocs);
		}
	}

	ocs_mgmt_end_unnumbered_section(textbuf, "ocs");

	return retval;
}


/**
 * @ingroup mgmt
 * @brief Set the value of a mgmt item.
 *
 * @par Description
 * This is the top level "set" handler for the driver. It
 * performs the following:
 *  - Checks that the qualifier portion of the name begins with my qualifier (ocs).
 *  - If the remaining part of the name matches a parameter that is known at this level,
 *    calls the correct function to change the configuration.
 *  - If the name is not known, sends the request to the back-ends to fulfill (if possible).
 *  - If the request has not been fulfilled by the back-end, passes the request to each of the
 *    children (domains) to have them (recursively) try to respond.
 *
 *  In passing the request to other entities, the request is considered to be handled
 *  if the function returns 0.
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Name of the property to be changed.
 * @param value Requested new value of the property.
 *
 * @return Returns 0 if the configuration value was updated, or -1 otherwise.
 */

int
ocs_mgmt_set(ocs_t *ocs, char *name, char *value)
{
	ocs_domain_t *domain;
	int result = -1;
	char qualifier[80];
	uint32_t i;

	snprintf(qualifier, sizeof(qualifier), "/ocs");

	/* If it doesn't start with my qualifier I don't know what to do with it */
	if (ocs_strncmp(name, qualifier, strlen(qualifier)) == 0) {
		char *unqualified_name = name + strlen(qualifier) +1;

		/* See if it's a value I can set */
		for (i=0;i<ARRAY_SIZE(mgmt_table);i++) {
			if (ocs_strcmp(unqualified_name, mgmt_table[i].name) == 0) {
				if (mgmt_table[i].set_handler) {
					return mgmt_table[i].set_handler(ocs, name, value);
				}
			}
		}

		if ((ocs->mgmt_functions) && (ocs->mgmt_functions->set_handler)) {
			result = ocs->mgmt_functions->set_handler(qualifier, name, (char *)value, ocs);
		}

		if (result != 0) {
			if ((ocs->tgt_mgmt_functions) && (ocs->tgt_mgmt_functions->set_handler)) {
				result = ocs->tgt_mgmt_functions->set_handler(qualifier, name,
						(char *)value, &(ocs->tgt_ocs));
			}
		}

		/* If I didn't know how to set this config value pass the request to each of my children */
		if (result != 0) {
			if (!ocs_device_lock_try_timeout(ocs, 1000, 3)) {
				ocs_log_err(ocs, "Failed to acquire device lock\n");
				return -EBUSY;
			}

			ocs_list_foreach(&ocs->domain_list, domain) {
				if ((domain->mgmt_functions) && (domain->mgmt_functions->set_handler)) {
					result = domain->mgmt_functions->set_handler(qualifier, name, (char*)value, domain);
				}
				if (result == 0) {
					break;
				}
			}
			ocs_device_unlock(ocs);
		}


	}

	return result;
}

/**
 * @ingroup mgmt
 * @brief Perform a management action.
 *
 * @par Description
 * This is the top level "exec" handler for the driver. It
 * performs the following:
 *  - Checks that the qualifier portion of the name begins with my qualifier (ocs).
 *  - If the remaining part of the name matches an action that is known at this level,
 *    calls the correct function to perform the action.
 *  - If the name is not known, sends the request to the back-ends to fulfill (if possible).
 *  - If the request has not been fulfilled by the back-end, passes the request to each of the
 *    children (domains) to have them (recursively) try to respond.
 *
 *  In passing the request to other entities, the request is considered to be handled
 *  if the function returns 0.
 *
 * @param ocs Pointer to the ocs structure.
 * @param action Name of the action to be performed.
 * @param arg_in Pointer to an argument being passed to the action.
 * @param arg_in_length Length of the argument pointed to by @c arg_in.
 * @param arg_out Pointer to an argument being passed to the action.
 * @param arg_out_length Length of the argument pointed to by @c arg_out.
 *
 * @return Returns 0 if the action was completed, or -1 otherwise.
 *
 *
 */

int
ocs_mgmt_exec(ocs_t *ocs, char *action, void *arg_in,
		uint32_t arg_in_length, void *arg_out, uint32_t arg_out_length)
{
	ocs_domain_t *domain;
	int result = -EOPNOTSUPP;
	char qualifier[80];
	uint32_t i;

	snprintf(qualifier, sizeof(qualifier), "/ocs");

	/* If it doesn't start with my qualifier I don't know what to do with it */
	if (ocs_strncmp(action, qualifier, strlen(qualifier)) == 0) {
		char *unqualified_name = action + strlen(qualifier) +1;

		/* See if it's an action I can perform */
		for (i=0;i<ARRAY_SIZE(mgmt_table); i++) {
			if (ocs_strcmp(unqualified_name, mgmt_table[i].name) == 0) {
				if (mgmt_table[i].action_handler) {
					return mgmt_table[i].action_handler(ocs, action, arg_in, arg_in_length,
							arg_out, arg_out_length);
				}
			}
		}

		if ((ocs->mgmt_functions) && (ocs->mgmt_functions->exec_handler)) {
			result = ocs->mgmt_functions->exec_handler(qualifier, action, arg_in, arg_in_length,
								   arg_out, arg_out_length, ocs);
		}

		if (result == -EOPNOTSUPP) {
			if ((ocs->tgt_mgmt_functions) && (ocs->tgt_mgmt_functions->exec_handler)) {
				result = ocs->tgt_mgmt_functions->exec_handler(qualifier, action,
						arg_in, arg_in_length, arg_out, arg_out_length,
						&(ocs->tgt_ocs));
			}
		}

		/* If I didn't know how to do this action pass the request to each of my children */
		if (result == -EOPNOTSUPP) {
			ocs_domain_exec_arg_t *domain_exec_args;

			if (ocs_list_empty(&ocs->domain_list)) {
				ocs_log_err(ocs, "Error: domain is empty\n");
				return -ENODEV;
			}

			domain_exec_args = ocs_malloc(ocs, sizeof(ocs_domain_exec_arg_t), OCS_M_NOWAIT | OCS_M_ZERO);
			if (domain_exec_args == NULL) {
				ocs_log_err(ocs, "Failed to allocate memory\n");
				return -ENOMEM;
			}

			if (arg_in_length && arg_in) {
				domain_exec_args->arg_in = ocs_malloc(ocs, arg_in_length, OCS_M_NOWAIT | OCS_M_ZERO);
				if (domain_exec_args->arg_in == NULL) {
					ocs_log_err(ocs, "Failed to allocate arg_in memory\n");
					result = -ENOMEM;
					goto free_and_exit;
				}

				if (ocs_copy_from_user(domain_exec_args->arg_in, (uint8_t *)arg_in, arg_in_length)) {
					ocs_log_err(ocs, "Failed to copy addr from user\n");
					result = -EFAULT;
					goto free_and_exit;
				}
				domain_exec_args->arg_in_length = arg_in_length;
			}

			domain_exec_args->arg_out = ocs_malloc(ocs, arg_out_length, OCS_M_NOWAIT | OCS_M_ZERO);
			if (domain_exec_args->arg_out == NULL) {
				ocs_log_err(ocs, "Failed to allocate arg_out memory\n");
				result = -ENOMEM;
				goto free_and_exit;
			}

			domain_exec_args->arg_out_length = arg_out_length;
			domain_exec_args->exec_cb = ocs_mgmt_domain_exec_cb;
			ocs_sem_init(&domain_exec_args->sem, 0, "domain_exec_sem");
			ocs_atomic_init(&domain_exec_args->ref_cnt, 2);

			ocs_device_lock(ocs);
			ocs_list_foreach(&ocs->domain_list, domain) {
				if ((domain->mgmt_functions) && (domain->mgmt_functions->exec_handler)) {
					result = domain->mgmt_functions->exec_handler(qualifier, action,
								domain_exec_args, sizeof(ocs_domain_exec_arg_t),
								NULL, 0, domain);
					if (result == 0)
						break;
				}
			}
			ocs_device_unlock(ocs);

			/* None of the domains could successfully process the action */
			if (result < 0)
				goto free_and_exit;

			if (ocs_sem_p(&domain_exec_args->sem, OCS_SEM_FOREVER)) {
				ocs_log_err(ocs, "ocs_sem_p failed\n");
				result = -ENXIO;
			} else if (domain_exec_args->sport_exec_args.retval) {
				result = domain_exec_args->retval;
			} else {
				if (ocs_copy_to_user(arg_out, domain_exec_args->arg_out, arg_out_length)) {
					ocs_log_err(ocs, "Error: copy to user buffer failed\n");
					result = -EFAULT;
				}
			}

			if (!ocs_atomic_sub_and_test(&domain_exec_args->ref_cnt, 1))
				return result;

free_and_exit:
			if (domain_exec_args->arg_in)
				ocs_free(ocs, domain_exec_args->arg_in, arg_in_length);

			if (domain_exec_args->arg_out)
				ocs_free(ocs, domain_exec_args->arg_out, arg_out_length);

			ocs_free(ocs, domain_exec_args, sizeof(ocs_domain_exec_arg_t));
		}
	}

	return result;
}

void
ocs_mgmt_get_all(ocs_t *ocs, ocs_textbuf_t *textbuf)
{
	ocs_domain_t *domain;
	uint32_t i;

	ocs_mgmt_start_unnumbered_section(textbuf, "ocs");

	for (i=0;i<ARRAY_SIZE(mgmt_table);i++) {
		if (mgmt_table[i].get_handler) {
			mgmt_table[i].get_handler(ocs, mgmt_table[i].name, textbuf);
		} else if (mgmt_table[i].action_handler) {
			/* No get_handler, but there's an action_handler. Just report
			   the name */
			ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_EX, mgmt_table[i].name);
		}
	}

	if ((ocs->mgmt_functions) && (ocs->mgmt_functions->get_all_handler)) {
		ocs->mgmt_functions->get_all_handler(textbuf, ocs);
	}

	if ((ocs->tgt_mgmt_functions) && (ocs->tgt_mgmt_functions->get_all_handler)) {
		ocs->tgt_mgmt_functions->get_all_handler(textbuf, &(ocs->tgt_ocs));
	}

	if (!ocs_device_lock_try_timeout(ocs, 1000, 3)) {
		ocs_log_err(ocs, "Failed to acquire device lock\n");
		ocs_mgmt_end_unnumbered_section(textbuf, "ocs");
		return;
	}

	ocs_list_foreach(&ocs->domain_list, domain) {
		if ((domain->mgmt_functions) && (domain->mgmt_functions->get_all_handler)) {
			domain->mgmt_functions->get_all_handler(textbuf, domain);
		}
	}
	ocs_device_unlock(ocs);

	ocs_mgmt_end_unnumbered_section(textbuf, "ocs");
}

#if defined(OCS_INCLUDE_RAMD)
static int32_t
ocs_mgmt_read_phys(ocs_t *ocs, char *name, void *arg_in, uint32_t arg_in_length, void *arg_out, uint32_t arg_out_length)
{
        uint32_t length;
        char addr_str[80];
        uintptr_t target_addr;
        void* vaddr = NULL;
        ocs_ramdisc_t **ramdisc_array;
        uint32_t ramdisc_count;


        if ((arg_in == NULL) ||
            (arg_in_length == 0) ||
            (arg_out == NULL) ||
            (arg_out_length == 0)) {
                return -1;
        }

        if (arg_in_length > 80) {
                arg_in_length = 80;
        }

        if (ocs_copy_from_user(addr_str, arg_in, arg_in_length)) {
                ocs_log_test(ocs, "Failed to copy addr from user\n");
                return -EFAULT;
        }

        target_addr = (uintptr_t)ocs_strtoul(addr_str, NULL, 0);
        /* addr_str must be the physical address of a buffer that was reported
         * in an SGL.  Search ramdiscs looking for a segment that contains that
         * physical address
         */

        if (ocs->tgt_ocs.use_global_ramd) {
                /* Only one target */
                ramdisc_count = ocs->tgt_ocs.rdisc_count;
                ramdisc_array = ocs->tgt_ocs.rdisc;
                vaddr = find_address_in_target(ramdisc_array, ramdisc_count, target_addr);
        } else {
                /* Multiple targets.  Each target is on a sport */
		uint32_t domain_idx;

		for (domain_idx=0; domain_idx<ocs->domain_instance_count; domain_idx++) {
			ocs_domain_t *domain;
			uint32_t sport_idx;

			domain = ocs_domain_get_instance(ocs, domain_idx);
			for (sport_idx=0; sport_idx < domain->sport_instance_count; sport_idx++) {
				ocs_sport_t *sport;

				sport = ocs_sport_get_instance(domain, sport_idx);
				ramdisc_count = sport->tgt_sport.rdisc_count;
				ramdisc_array = sport->tgt_sport.rdisc;
				vaddr = find_address_in_target(ramdisc_array, ramdisc_count, target_addr);

				if (vaddr != NULL) {
					break;
				}
			}
                }
        }




        length = arg_out_length;

        if (vaddr != NULL) {

                if (ocs_copy_to_user(arg_out, vaddr, length)) {
                        ocs_log_test(ocs, "Failed to copy buffer to user\n");
                        return -EFAULT;
                }

                return 0;
        } else {

                return -EFAULT;
	}

}

/*
 * This function searches a target for a given physical address.
 * The target is made up of a number of LUNs, each represented by
 * a ocs_ramdisc_t.
 */
static void* find_address_in_target(ocs_ramdisc_t **ramdisc_array, uint32_t ramdisc_count, uintptr_t target_addr)
{
	void *vaddr = NULL;
	uint32_t ramdisc_idx;

	/* Check each ramdisc */
	for (ramdisc_idx=0; ramdisc_idx<ramdisc_count; ramdisc_idx++) {
		uint32_t segment_idx;
		ocs_ramdisc_t *rdisc;
		rdisc = ramdisc_array[ramdisc_idx];
		/* Check each segment in the ramdisc */
		for (segment_idx=0; segment_idx<rdisc->segment_count; segment_idx++) {
			ramdisc_segment_t *segment = rdisc->segments[segment_idx];
			uintptr_t segment_start;
			uintptr_t segment_end;
			uint32_t offset;

			segment_start = segment->data_segment.phys;
			segment_end = segment->data_segment.phys + segment->data_segment.size - 1;
			if ((target_addr >= segment_start) && (target_addr <= segment_end)) {
				/* Found the target address */
				offset = target_addr - segment_start;
				vaddr = (uint32_t*)segment->data_segment.virt + offset;
			}

			if (rdisc->dif_separate) {
				segment_start = segment->dif_segment.phys;
				segment_end = segment->data_segment.phys + segment->dif_segment.size - 1;
				if ((target_addr >= segment_start) && (target_addr <= segment_end)) {
					/* Found the target address */
					offset = target_addr - segment_start;
					vaddr = (uint32_t*)segment->dif_segment.virt + offset;
				}
			}

			if (vaddr != NULL) {
				break;
			}

		}

		if (vaddr != NULL) {
			break;
		}


	}

	return vaddr;
}
#endif

static int32_t
ocs_mgmt_firmware_reset(ocs_t *ocs, char *name, void *arg_in, uint32_t arg_in_length, void *arg_out, uint32_t arg_out_length)
{
	char arg_str[12] = { '\0' };
	uint8_t dump_level = OCS_FW_DUMP_LEVEL_NONE;

	if (arg_in_length && arg_in) {
		if (arg_in_length > 12)
			arg_in_length = 12;

		if (ocs_copy_from_user(arg_str, arg_in, arg_in_length)) {
			ocs_log_err(ocs, "Failed to copy arg from user\n");
			return -EFAULT;
		}

		/* Check if we have to generate a FW dump */
		if (0 == ocs_strncmp(arg_str, "force_dump", ocs_strlen("force_dump")))
			dump_level = OCS_FW_CHIP_LEVEL_DUMP;
	}

	return ocs_fw_reset(ocs, dump_level);
}

static int32_t
ocs_mgmt_trigger_error(ocs_t *ocs, char *name, void *arg_in, uint32_t arg_in_length, void *arg_out, uint32_t arg_out_length)
{
	char arg_str[8] = { '\0' };
	uint8_t dump_level = OCS_FW_CHIP_LEVEL_DUMP;
	
	if (arg_in_length && arg_in) {
		if (arg_in_length > 8)
			arg_in_length = 8;

		if (ocs_copy_from_user(arg_str, arg_in, arg_in_length)) {
			ocs_log_err(ocs, "Failed to copy arg from user\n");
			return -EFAULT;
		}

		if (0 == ocs_strncmp(arg_str, "fdb", ocs_strlen("fdb")))
			dump_level = OCS_FW_FUNC_DESC_DUMP;
	}
	ocs_device_trigger_fw_error(ocs, dump_level);
	return 0;
}

static int32_t
ocs_mgmt_function_reset(ocs_t *ocs, char *name, void *buf, uint32_t buf_len, void *arg_out, uint32_t arg_out_length)
{
	ocs_log_info(ocs, "Initiate function reset\n");
	ocs_recovery_reset(ocs, OCS_RECOVERY_MODE_MANUAL, OCS_RESET_LEVEL_PORT, FALSE);

	while (!ocs_recovery_state_check(OCS_RECOVERY_STATE_IDLE)) {
		/* If recovery has been triggered from a different path, wait till it completes */
		ocs_msleep(100);
	}

	return 0;
}

static int32_t
ocs_mgmt_port_migration(ocs_t *ocs, char *name, void *buf, uint32_t buf_len, void *arg_out, uint32_t arg_out_length)
{
#if !defined(OCS_USPACE_SPDK)
	return ocs_recovery_reset(ocs, OCS_RECOVERY_MODE_PORT_MIGRATION, OCS_RESET_LEVEL_NONE, FALSE);
#else
	return -1;
#endif
}

#define FW_WRITE_BUFSIZE 64*1024

typedef struct ocs_mgmt_fw_write_result {
	ocs_sem_t semaphore;
	int32_t status;
	uint32_t actual_xfer;
	uint32_t change_status;
	uint32_t ext_status;
} ocs_mgmt_fw_write_result_t;

static int32_t
ocs_mgmt_firmware_write(ocs_t *ocs, char *name, void *buf, uint32_t buf_len, void *arg_out, uint32_t arg_out_length)
{
	int rc = 0;
	uint32_t bytes_left;
	uint32_t xfer_size;
	uint32_t offset;
	uint8_t *userp;
	ocs_dma_t dma;
	int last = 0;
	ocs_mgmt_fw_write_result_t result;
	uint32_t change_status = 0;
        char status_str[80];

	ocs_sem_init(&(result.semaphore), 0, "fw_write");

	bytes_left = buf_len;
	offset = 0;
	userp = (uint8_t *)buf;

	if (ocs_dma_alloc(ocs, &dma, FW_WRITE_BUFSIZE, 4096)) {
		ocs_log_err(ocs, "ocs_mgmt_firmware_write: malloc failed");
		return -ENOMEM;
	}

	while (bytes_left > 0) {
		if (bytes_left > FW_WRITE_BUFSIZE) {
			xfer_size = FW_WRITE_BUFSIZE;
		} else {
			xfer_size = bytes_left;
		}

		// Copy xfer_size bytes from user space to kernel buffer
		if (ocs_copy_from_user(dma.virt, userp, xfer_size)) {
			rc = -EFAULT;
			break;
		}

		// See if this is the last block
		if (bytes_left == xfer_size) {
			last = 1;
		}

		// Send the HAL command
		ocs_hal_firmware_write(&ocs->hal, &dma, xfer_size, offset, last, ocs_mgmt_fw_write_cb, &result);

		// Wait for semaphore to be signaled when the command completes
		// TODO:  Should there be a timeout on this?  If so, how long?
		if (ocs_sem_p(&(result.semaphore), OCS_SEM_FOREVER) != 0) {
			// Undefined failure
			ocs_log_err(ocs, "ocs_sem_p failed\n");
			rc = -ENXIO;
			break;
		}

		if (result.actual_xfer == 0) {
			ocs_log_test(ocs, "actual_write_length is %d\n", result.actual_xfer);
			rc = -EFAULT;
			break;
		}

		// Check status
		if (result.status != 0) {
			ocs_log_err(ocs, "write returned status %d, additional status %d\n",
				    result.status, result.ext_status);
			if (result.ext_status == SLI4_MBOX_EXT_STATUS_OBJ_UNSUPPORTED_FIRMWARE) {
				ocs_log_err(ocs, "Firmware does not meet minimum version requirement for this IPL\n");
				rc = -EACCES;
			} else {
				rc = -EFAULT;
			}
			break;
		}

		if (last) {
			change_status = result.change_status;
		}

		bytes_left -= result.actual_xfer;
		offset += result.actual_xfer;
		userp += result.actual_xfer;

	}

	/* Create string with status and copy to userland */
	if ((arg_out_length > 0) && (arg_out != NULL)) {
		if (arg_out_length > sizeof(status_str)) {
			arg_out_length = sizeof(status_str);
		}
		ocs_snprintf(status_str, arg_out_length, "%d", change_status);
		if (ocs_copy_to_user(arg_out, status_str, arg_out_length))
			ocs_log_test(ocs, "copy to user failed for change_status\n");
	}


	ocs_dma_free(ocs, &dma);

	return rc;
}

static void
ocs_mgmt_fw_write_cb(int32_t status, int32_t ext_status, uint32_t actual_write_length,
		     uint32_t change_status, void *arg)
{
	ocs_mgmt_fw_write_result_t *result = arg;

	result->status = status;
	result->actual_xfer = actual_write_length;
	result->change_status = change_status;
	result->ext_status = ext_status;

	ocs_sem_v(&(result->semaphore));
}

typedef struct ocs_mgmt_parity_stats_result_s {
	ocs_sem_t semaphore;
	int32_t status;
	ocs_parity_err_count_t counter[OCS_MAX_SLI4_ULP];
} ocs_mgmt_parity_stats_result_t;

static void
ocs_mgmt_read_parity_stats_cb(int32_t status, uint8_t num_ulp,
		ocs_parity_err_count_t *counter, void *arg)
{
	uint8_t i;
	ocs_mgmt_parity_stats_result_t *result = (ocs_mgmt_parity_stats_result_t *)arg;

	result->status = status;
	if (0 == status) {
		for (i = 0; i < num_ulp; i++) {
			result->counter[i].pbec = counter[i].pbec;
			result->counter[i].tpec = counter[i].tpec;
		}
	}

	ocs_sem_v(&result->semaphore);
}

static int32_t
ocs_mgmt_read_parity_stats(ocs_t *ocs, char *name, void *arg_in, uint32_t arg_in_length,
				void *arg_out, uint32_t arg_out_length)
{
	int32_t rc = -1;
	bool clear_stats = FALSE;
	char arg_str[8] = { '\0' };
	uint32_t stats_counter_size = (OCS_MAX_SLI4_ULP * sizeof(ocs_parity_err_count_t));
	ocs_mgmt_parity_stats_result_t result;

	if (!arg_out_length || !arg_out)
		return rc;

	if (arg_in_length && arg_in) {
		if (arg_in_length > 8)
			arg_in_length = 8;

		if (ocs_copy_from_user(arg_str, arg_in, arg_in_length)) {
			ocs_log_err(ocs, "Failed to copy arg from user\n");
			return -EFAULT;
		}

		/* Check if we have to clear the ITCM parity stats */
		if (0 == ocs_strncmp(arg_str, "clear", ocs_strlen("clear")))
			clear_stats = TRUE;
	}

	ocs_memset(&result, 0, sizeof(result));
	ocs_sem_init(&result.semaphore, 0, "get_parity_stats");
	rc = ocs_hal_get_itcm_parity_stats(&ocs->hal, clear_stats, ocs_mgmt_read_parity_stats_cb, &result);
	if (rc)
		return rc;

	if (ocs_sem_p(&result.semaphore, OCS_SEM_FOREVER)) {
		/* Undefined failure */
		ocs_log_err(ocs, "ocs_sem_p failed\n");
		return -ENXIO;
	}

	if (result.status) {
		ocs_log_err(ocs, "Read parity stats failed; status: %d\n", result.status);
		return -EFAULT;
	}

	/* Success; copy the parity stats from result struct to arg_out */
	if (arg_out_length > stats_counter_size)
		arg_out_length = stats_counter_size;

	if (ocs_copy_to_user(arg_out, result.counter, arg_out_length)) {
		ocs_log_err(ocs, "Failed to copy buffer to user\n");
		return -EFAULT;
	}

	return rc;
}

void ocs_read_temperature_lancer_cb(int32_t status, uint32_t curr_temp,
				    uint32_t crit_temp_thr, uint32_t warn_temp_thr,
				    uint32_t norm_temp_thr, uint32_t fan_off_thr,
				    uint32_t fan_on_thr, void *arg)
{
	ocs_mgmt_lancer_temp_result_t *result = (ocs_mgmt_lancer_temp_result_t *)arg;
	ocs_t *ocs = result->ocs;

	ocs_log_debug(ocs, "CB status %08x, curr_temp=%d crit_temp_thr=%d "
		      "warn_temp_thr %d norm_temp_thr %d foff_thr %d fon_thr %d\n",
		      status, curr_temp, crit_temp_thr, warn_temp_thr,
		      norm_temp_thr, fan_off_thr, fan_on_thr);

	result->status = status;
	result->resp[0] = curr_temp;
	result->resp[1] = crit_temp_thr;
	result->resp[2] = warn_temp_thr;
	result->resp[3] = norm_temp_thr;
	result->resp[4] = fan_off_thr;
	result->resp[5] = fan_on_thr;

	ocs_sem_v(&result->waitsem);
}

int32_t ocs_read_temperature_lancer(ocs_t *ocs, void *buf, uint32_t buf_len)
{
	int32_t rc = -1;
	ocs_mgmt_lancer_temp_result_t *result;

	if (buf_len < OCS_DUMP_TYPE4_WKI_TAG_SAT_TEM_RESP_LEN) {
		ocs_log_err(ocs, "Insufficient output buffer\n");
		return rc;
	}

	result = ocs_malloc(ocs, sizeof(ocs_mgmt_lancer_temp_result_t),
			    OCS_M_ZERO | OCS_M_NOWAIT);
	if (result == NULL) {
		ocs_log_err(ocs, "Failed to allcoate memory\n");
		return rc;
	}

	ocs_sem_init(&result->waitsem, 0, "dump_type4_mbox");
	rc = ocs_hal_get_temperature(&ocs->hal, ocs_read_temperature_lancer_cb,
				     result);
	if (rc) {
		ocs_log_err(ocs, "HAL get temperature failed\n");
		goto exit_err;
	}

	if (ocs_sem_p(&result->waitsem, OCS_SEM_FOREVER)) {
		ocs_log_err(ocs, "ocs_sem_p failed\n");
		goto exit_err;
	}

	if (result->status != 0) {
		ocs_log_err(ocs, "Read temperature command failed status %04x\n",
			    result->status);
		rc = -EFAULT;
		goto exit_err;
	}

	/* Copy resp to user buffer */
	if (ocs_copy_to_user(buf, (uint8_t *)result->resp, sizeof(result->resp))) {
		ocs_log_err(ocs, "Failed to copy resp to user buffer\n");
		rc = -EFAULT;
	}

exit_err:
	if (result)
		ocs_free(ocs, result, sizeof(ocs_mgmt_lancer_temp_result_t));

	return rc;
}

static int32_t
ocs_mgmt_get_bios_config(ocs_t *ocs, char *name,
			 void *arg_in, uint32_t arg_in_len,
			 void *arg_out, uint32_t arg_out_len)
{
	int32_t rc;
	unsigned long signature;

	if (ocs_copy_from_user(&signature, arg_in, MIN(arg_in_len, sizeof(unsigned long)))) {
		ocs_log_err(ocs, "failed to copy from user buff\n");
		return -EFAULT;
	}

	if (signature == OCS_UEFI_BIOS_SIGNATURE) {
		ocs_uefi_bios_struct_t *uefi_bios_region_info = ocs_malloc(ocs, sizeof(ocs_uefi_bios_struct_t),
									   OCS_M_ZERO | OCS_M_NOWAIT);
		if (uefi_bios_region_info == NULL) {
			ocs_log_err(ocs, "failed to allocate memory\n");
			return -ENOSPC;
		}

		rc = ocs_hal_dump_type2(&ocs->hal, OCS_EFIBIOS_REGION_ID, uefi_bios_region_info, sizeof(ocs_uefi_bios_struct_t));
		if (rc) {
			ocs_log_err(ocs, "Failed to read UEFI BIOS config data\n");
			ocs_free(ocs, uefi_bios_region_info, sizeof(ocs_uefi_bios_struct_t));
			return -1;
		}

		if (ocs_copy_to_user(arg_out, uefi_bios_region_info,
					MIN(arg_out_len, sizeof(ocs_uefi_bios_struct_t)))) {
			ocs_log_err(ocs, "copy to user buffer failed\n");
			ocs_free(ocs, uefi_bios_region_info, sizeof(ocs_uefi_bios_struct_t));
			return -1;
		}
	} else if (signature == OCS_X86_BIOS_SIGNATURE) {
		ocs_bios_struct_t *bios_region_info = ocs_malloc(ocs, sizeof(ocs_bios_struct_t),
				OCS_M_ZERO | OCS_M_NOWAIT);

		if (bios_region_info == NULL) {
			ocs_log_err(ocs, "failed to allocate memory\n");
			return -ENOSPC;
		}

		rc = ocs_hal_dump_type2(&ocs->hal, OCS_BIOS_REGION_ID, bios_region_info, sizeof(ocs_bios_struct_t));
		if (rc) {
			ocs_log_err(ocs, "Failed to read BIOS config data\n");
			ocs_free(ocs, bios_region_info, sizeof(ocs_bios_struct_t));
			return -1;
		}

		if (ocs_copy_to_user(arg_out, bios_region_info,
					MIN(arg_out_len, sizeof(ocs_bios_struct_t)))) {
			ocs_log_err(ocs, "copy to user buffer failed\n");
			ocs_free(ocs, bios_region_info, sizeof(ocs_bios_struct_t));
			return -1;
		}
	} else {
		ocs_log_err(ocs, "Invalid BIOS signature(0x%lx) requested\n", signature);
		rc = -EFAULT;
	}

	return rc;
}

static int32_t
ocs_mgmt_update_uefi_bios_config(ocs_t *ocs, char *name,
				 void *arg_in, uint32_t arg_in_len,
				 void *arg_out, uint32_t arg_out_len)
{
	int32_t rc = 0;
	ocs_uefi_bios_struct_t *bios_region_info = ocs_malloc(ocs, sizeof(ocs_uefi_bios_struct_t),
							      OCS_M_ZERO | OCS_M_NOWAIT);

	if (bios_region_info == NULL) {
		ocs_log_err(ocs, "failed to allocate memory\n");
		return -ENOSPC;
	}

	if (ocs_copy_from_user(bios_region_info, arg_in, MIN(arg_in_len, sizeof(ocs_uefi_bios_struct_t)))) {
		ocs_log_err(ocs, "failed to copy from user buff\n");
		rc = -EFAULT;
		goto exit;
	}

	rc = ocs_hal_update_cfg(&ocs->hal, OCS_EFIBIOS_REGION_ID, bios_region_info, sizeof(ocs_uefi_bios_struct_t));
	if (rc) {
		ocs_log_err(ocs, "Failed to update UEFI BIOS config data\n");
		rc = -1;
	}

exit:
	ocs_free(ocs, bios_region_info, sizeof(ocs_uefi_bios_struct_t));
	return rc;
}

static int32_t
ocs_mgmt_update_x86_bios_config(ocs_t *ocs, char *name,
				void *arg_in, uint32_t arg_in_len,
				void *arg_out, uint32_t arg_out_len)
{
	int32_t rc = 0;
	ocs_bios_struct_t *bios_region_info = ocs_malloc(ocs, sizeof(ocs_bios_struct_t),
							 OCS_M_ZERO | OCS_M_NOWAIT);

	if (bios_region_info == NULL) {
		ocs_log_err(ocs, "failed to allocate memory\n");
		return -ENOSPC;
	}

	if (ocs_copy_from_user(bios_region_info, arg_in, MIN(arg_in_len, sizeof(ocs_bios_struct_t)))) {
		ocs_log_err(ocs, "failed to copy from user buff\n");
		rc = -EFAULT;
		goto exit;
	}

	rc = ocs_hal_update_cfg(&ocs->hal, OCS_BIOS_REGION_ID, bios_region_info, sizeof(ocs_bios_struct_t));
	if (rc) {
		ocs_log_err(ocs, "Failed to update X86 BIOS config data\n");
		rc = -1;
	}

exit:
	ocs_free(ocs, bios_region_info, sizeof(ocs_bios_struct_t));
	return rc;
}

typedef struct ocs_mgmt_sfp_result {
	ocs_sem_t semaphore;
	int32_t status;
	ocs_atomic_t refcnt;
	uint32_t bytes_read;
	uint32_t page_data[32];
} ocs_mgmt_sfp_result_t;

static void
ocs_mgmt_sfp_cb(void *os, int32_t status, uint32_t bytes_read, uint32_t *data, void *arg)
{
	ocs_mgmt_sfp_result_t *result = arg;
	ocs_t *ocs = os;

	if (result) {
		result->status = status;
		result->bytes_read = bytes_read;
		ocs_memcpy(&result->page_data, data, SFP_PAGE_SIZE);

		if (ocs_atomic_sub_and_test(&result->refcnt, 1)) {
			ocs_free(ocs, result, sizeof(*result));
			return;
		}
		ocs_sem_v(&(result->semaphore));
	}
}

/**
 * @brief Set DHCHAP config
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Not used.
 * @param value Value to which the dhchap config is set.
 *
 * @return Returns 0 on success.
 */
static int
ocs_mgmt_set_dhchap_config(ocs_t *ocs, char *name,
			   void *arg_in, uint32_t arg_in_len,
			   void *arg_out, uint32_t arg_out_len)
{
	struct ocs_ioctl_dhchap_config cfg;
	struct ocs_auth_cfg_info auth_cfg = { 0 };
	int i, rc;

	ocs_log_debug(ocs, "\n");
	rc = ocs_copy_from_user(&cfg, arg_in, sizeof(cfg));
	if (rc)
		return rc;

	auth_cfg.mode = cfg.mode;
	auth_cfg.flags = cfg.flags;
	for (i = 0; i < IOCTL_MAX_HASH; i++)
		auth_cfg.hash_list[i] = cfg.hash_list[i];
	for (i = 0; i < IOCTL_MAX_DH_GROUP; i++)
		auth_cfg.dh_grp_list[i] = cfg.dh_grp_list[i];
	auth_cfg.flags |= OCS_AUTH_FLAGS_VALID;

	ocs_log_debug(ocs, "mode=%d, flags=0x%x\n",
		      auth_cfg.mode, auth_cfg.flags);
	rc = ocs_auth_cfg_info_update(ocs, &auth_cfg);
	if (rc)
		return rc;

	return ocs_auth_cfg_commit(ocs);
}

/**
 * @brief Get DHCHAP config
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Not used.
 *
 * @return None.
 */
static int
ocs_mgmt_get_dhchap_config(ocs_t *ocs, char *name,
			   void *arg_in, uint32_t arg_in_len,
			   void *arg_out, uint32_t arg_out_len)
{
	struct ocs_ioctl_dhchap_config cfg = { 0 };
	struct ocs_auth_cfg_info auth_cfg = { 0 };
	int i, rc;

	ocs_log_debug(ocs, "\n");

	rc = ocs_auth_cfg_info_get(ocs, &auth_cfg);
	if (rc)
		return rc;

	cfg.mode = auth_cfg.mode;
	cfg.flags = auth_cfg.flags;
	for (i = 0; i < IOCTL_MAX_HASH; i++)
		cfg.hash_list[i] = auth_cfg.hash_list[i];
	for (i = 0; i < IOCTL_MAX_DH_GROUP; i++)
		cfg.dh_grp_list[i] = auth_cfg.dh_grp_list[i];
	cfg.reauth_interval = auth_cfg.reauth_interval;

	ocs_log_debug(ocs, "mode=%d, flags=0x%x\n", cfg.mode, cfg.flags);
	return ocs_copy_to_user(arg_out, &cfg, arg_out_len);
}

static int
ocs_mgmt_add_dhchap_secret(ocs_t *ocs, char *name,
			   void *arg_in, uint32_t arg_in_len,
			   void *arg_out, uint32_t arg_out_len)
{
	struct ocs_ioctl_dhchap_secret secret;
	struct ocs_auth_cfg_pass pass = { 0 };
	int rc;

	ocs_log_debug(ocs, "\n");
	rc = ocs_copy_from_user(&secret, arg_in, sizeof(secret));
	if (rc)
		return rc;

	pass.local_len = ocs_strlen(secret.local_pass);
	pass.remote_len = ocs_strlen(secret.remote_pass);
	ocs_strcpy(pass.local, secret.local_pass);
	ocs_strcpy(pass.remote, secret.remote_pass);

	rc = ocs_auth_cfg_entry_update(ocs, secret.local_wwpn,
				       secret.remote_wwpn, &pass);
	if (rc)
		return rc;

	return ocs_auth_cfg_commit(ocs);
}

static int
ocs_mgmt_list_dhchap_secrets(ocs_t *ocs, char *name,
			     void *arg_in, uint32_t arg_in_len,
			     void *arg_out, uint32_t arg_out_len)
{
	struct ocs_auth_cfg_id *ids;
	int rc;

	ocs_log_debug(ocs, "\n");
	ids = ocs_malloc(ocs, arg_out_len, OCS_M_ZERO);
	if (!ids)
		return -ENOMEM;

	rc = ocs_auth_cfg_entries_get(ocs, ids, arg_out_len / sizeof(*ids));
	if (rc) {
		ocs_free(ocs, ids, arg_out_len);
		return rc;
	}

	rc = ocs_copy_to_user(arg_out, ids, arg_out_len);
	if (rc) 
		ocs_log_err(ocs, "Failed to copy buffer\n");

	ocs_free(ocs, ids, arg_out_len);
	return rc;
}

static int
ocs_mgmt_num_dhchap_secrets(ocs_t *ocs, char *name,
			    void *arg_in, uint32_t arg_in_len,
			    void *arg_out, uint32_t arg_out_len)
{
	uint32_t num;
	int rc;

	ocs_log_debug(ocs, "\n");

	rc = ocs_auth_cfg_num_entries_get(ocs, &num);
	if (rc)
		return rc;

	return ocs_copy_to_user(arg_out, &num, arg_out_len);
}

int32_t
ocs_mgmt_get_sfp(ocs_t *ocs, uint16_t page, void *buf, uint32_t buf_len)
{
	int rc = OCS_HAL_RTN_SUCCESS;
	ocs_mgmt_sfp_result_t *result = ocs_malloc(ocs, sizeof(ocs_mgmt_sfp_result_t),  OCS_M_ZERO | OCS_M_NOWAIT);
	if (result == NULL) {
		ocs_log_err(ocs, "failed to allocate memory for ocs_mgmt_sfp_result_t\n");
		return -ENOMEM;
	}

	ocs_sem_init(&(result->semaphore), 0, "get_sfp");
	ocs_atomic_init(&result->refcnt, 2); /* one for self and one for callback */

	/* Send the HAL command */
	rc = ocs_hal_get_sfp(&ocs->hal, page, ocs_mgmt_sfp_cb, result);

	if (rc != OCS_HAL_RTN_SUCCESS) {
		ocs_log_err(ocs, "failed to submit HAL command rc: 0x%x\n", rc);
		ocs_free(ocs, result, sizeof(*result));
		return -1;
	}

	/* Wait for semaphore to be signaled when the command completes */
	if (ocs_sem_p(&(result->semaphore), 5 * 1000 * 1000) != 0) {
		/* Undefined failure */
		ocs_log_err(ocs, "ocs_sem_p failed\n");
		rc = -ENXIO;
		goto exit_ocs_mgmt_get_sfp;
	}

	/* Check status */
	if (result->status != 0) {
		ocs_log_test(ocs, "read_transceiver_data returned status %d\n", result->status);
		if (result->status == 0x42)
			rc = -ENODEV;
		else
			rc = -EFAULT;
	}

	if (rc == OCS_HAL_RTN_SUCCESS) {
		rc = (result->bytes_read > buf_len ? buf_len : result->bytes_read);
		/* Copy the results back to the supplied buffer */
		ocs_memcpy(buf, result->page_data, rc);
	}

exit_ocs_mgmt_get_sfp:
	if (ocs_atomic_sub_and_test(&result->refcnt, 1))
		ocs_free(ocs, result, sizeof(*result));

	return rc;
}

static int32_t
ocs_mgmt_force_assert(ocs_t *ocs, char *name, void *buf, uint32_t buf_len, void *arg_out, uint32_t arg_out_length)
{
	ocs_assert(FALSE, 0);
}

static void
get_nodes_count(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_xport_t *xport = ocs->xport;

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "nodes_count", "%d", xport->nodes_count);
}

static void
get_driver_version(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "driver_version", ocs->driver_version);
}

static void
get_desc(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "desc", ocs->desc);
}

static void
get_fw_rev(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "fw_rev", ocs_hal_get_ptr(&ocs->hal, OCS_HAL_FW_REV));
}

static void
get_fw_rev2(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "fw_rev2", ocs_hal_get_ptr(&ocs->hal, OCS_HAL_FW_REV2));
}

static void
get_ipl(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "ipl", ocs_hal_get_ptr(&ocs->hal, OCS_HAL_IPL));
}

static void
get_hw_rev1(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint32_t value;

	ocs_hal_get(&ocs->hal, OCS_HAL_HW_REV1, &value);

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "hw_rev1", "%u", value);
}

static void
get_hw_rev2(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint32_t value;

	ocs_hal_get(&ocs->hal, OCS_HAL_HW_REV2, &value);

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "hw_rev2", "%u", value);
}

static void
get_hw_rev3(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint32_t value;
	ocs_hal_get(&ocs->hal, OCS_HAL_HW_REV3, &value);

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "hw_rev3", "%u", value);
}

static void
get_wwnn(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint64_t *wwnn;

	wwnn = ocs_hal_get_ptr(&ocs->hal, OCS_HAL_WWN_NODE);

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "wwnn", "0x%" PRIx64 , ocs_htobe64(*wwnn));
}

static void
get_wwpn(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint64_t *wwpn;

	wwpn = ocs_hal_get_ptr(&ocs->hal, OCS_HAL_WWN_PORT);

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "wwpn", "0x%" PRIx64 , ocs_htobe64(*wwpn));
}

static void
get_sn(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint8_t *pserial;
	uint32_t len;
	char sn_buf[256];

	pserial = ocs_scsi_get_property_ptr(ocs, OCS_SCSI_SERIALNUMBER);
	if (pserial) {
		len = *pserial ++;
		strncpy(sn_buf, (char*)pserial, len);
		sn_buf[len] = '\0';
		ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "sn", sn_buf);
	}
}

static void
get_pn(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint8_t *pserial;
	uint32_t len;
	char sn_buf[256];

	pserial = ocs_scsi_get_property_ptr(ocs, OCS_SCSI_PARTNUMBER);
	if (pserial) {
		len = *pserial ++;
		strncpy(sn_buf, (char*)pserial, len);
		sn_buf[len] = '\0';
		ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "pn", sn_buf);
	} else {
		ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "pn", ocs->model);
	}
}

static void
get_sli4_intf_reg(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "sli4_intf_reg", "0x%04x",
		ocs_config_read32(ocs, SLI4_INTF_REG));
}

static void
get_phy_port_num(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint32_t phy_port;

	phy_port = ocs_scsi_get_property(ocs, OCS_SCSI_PORTNUM);
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "phy_port_num", "0x%04x", phy_port);
}

static void
get_asic_id(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "asic_id_reg", "0x%04x",
		ocs_config_read32(ocs, SLI4_ASIC_ID_REG));
}

static void
get_chip_type(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint32_t family;
	uint32_t asic_id;
	uint32_t asic_gen_num;
	uint32_t asic_rev_num;
	uint32_t rev_id;
	char result_buf[80];
	char tmp_buf[80];

	family = (ocs_config_read32(ocs, SLI4_INTF_REG) & 0x00000f00) >> 8;
	asic_id = ocs_config_read32(ocs, SLI4_ASIC_ID_REG);
	asic_rev_num = asic_id & 0xff;
	asic_gen_num = (asic_id & 0xff00) >> 8;

	rev_id = ocs_config_read32(ocs, SLI4_PCI_CLASS_REVISION) & 0xff;

	switch(family) {
	case 0x00:
		// BE2
		ocs_strncpy(result_buf,  "BE2 A", sizeof(result_buf));
		ocs_snprintf(tmp_buf, 2, "%d", rev_id);
		strcat(result_buf, tmp_buf);
		break;
	case 0x01:
		// BE3
		ocs_strncpy(result_buf, "BE3", sizeof(result_buf));
		if (rev_id >= 0x10) {
			strcat(result_buf, "-R");
		}
		ocs_snprintf(tmp_buf, 3, " %c", ((rev_id & 0xf0) >> 4) + 'A');
		strcat(result_buf, tmp_buf);
		ocs_snprintf(tmp_buf, 2, "%d", rev_id & 0x0f);
		strcat(result_buf, tmp_buf);
		break;
	case 0x02:
		// Skyhawk A0
		ocs_strncpy(result_buf, "Skyhawk A0", sizeof(result_buf));
		break;
	case 0x0a:
		// Lancer A0
		ocs_strncpy(result_buf, "Lancer A", sizeof(result_buf));
		ocs_snprintf(tmp_buf, 2, "%d", rev_id & 0x0f);
		strcat(result_buf, tmp_buf);
		break;
	case 0x0b:
		// Lancer B0 or D0
		ocs_strncpy(result_buf, "Lancer", sizeof(result_buf));
		ocs_snprintf(tmp_buf, 3, " %c", ((rev_id & 0xf0) >> 4) + 'A');
		strcat(result_buf, tmp_buf);
		ocs_snprintf(tmp_buf, 2, "%d", rev_id & 0x0f);
		strcat(result_buf, tmp_buf);
		break;
	case 0x0c:
		ocs_strncpy(result_buf, "Lancer G6", sizeof(result_buf));
		break;
	case 0x0d:
		ocs_strncpy(result_buf, "Prism", sizeof(result_buf));
		break;
	case 0x0f:
		// Refer to ASIC_ID
		switch(asic_gen_num) {
		case 0x00:
			ocs_strncpy(result_buf, "BE2", sizeof(result_buf));
			break;
		case 0x03:
			ocs_strncpy(result_buf, "BE3-R", sizeof(result_buf));
			break;
		case 0x04:
			ocs_strncpy(result_buf, "Skyhawk-R", sizeof(result_buf));
			break;
		case 0x05:
			ocs_strncpy(result_buf, "Corsair", sizeof(result_buf));
			break;
		case 0x0b:
			ocs_strncpy(result_buf, "Lancer", sizeof(result_buf));
			break;
		case 0x0c:
			ocs_strncpy(result_buf, "LancerG6", sizeof(result_buf));
			break;
		case 0x0d:
			ocs_strncpy(result_buf, "Prism", sizeof(result_buf));
			break;
		default:
			ocs_strncpy(result_buf, "Unknown", sizeof(result_buf));
		}
		if (ocs_strcmp(result_buf, "Unknown") != 0) {
			ocs_snprintf(tmp_buf, 3, " %c", ((asic_rev_num & 0xf0) >> 4) + 'A');
			strcat(result_buf, tmp_buf);
			ocs_snprintf(tmp_buf, 2, "%d", asic_rev_num & 0x0f);
			strcat(result_buf, tmp_buf);
		}
		break;
	default:
		ocs_strncpy(result_buf, "Unknown", sizeof(result_buf));
	}

	ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "chip_type", result_buf);

}

static void
get_supported_speeds(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	char supported_speeds[256] = { 0 };
	ocs_xport_stats_t speed;
	size_t length = 0;

	speed.value = 1000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		strcat(supported_speeds, "1 Gbit, ");
	}
	speed.value = 2000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		strcat(supported_speeds, "2 Gbit, ");
	}
	speed.value = 4000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		strcat(supported_speeds, "4 Gbit, ");
	}
	speed.value = 8000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		strcat(supported_speeds, "8 Gbit, ");
	}
	speed.value = 16000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		strcat(supported_speeds, "16 Gbit, ");
	}
	speed.value = 32000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		strcat(supported_speeds, "32 Gbit, ");
	}
	speed.value = 64000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		strcat(supported_speeds, "64 Gbit, ");
	}
	speed.value = 128000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		strcat(supported_speeds, "128 Gbit, ");
	}
	speed.value = 256000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		strcat(supported_speeds, "256 Gbit, ");
	}

	/* Trim trailing ', ' */
	length = strlen(supported_speeds);
	if (length > 2) {
		supported_speeds[length-2] = 0;
	}

	ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "supported_speeds", supported_speeds);
}

static void
get_pci_vendor(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "pci_vendor", "0x%04x", ocs->pci_vendor);
}

static void
get_pci_device(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "pci_device", "0x%04x", ocs->pci_device);
}

static void
get_pci_subsystem_vendor(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "pci_subsystem_vendor", "0x%04x", ocs->pci_subsystem_vendor);
}

static void
get_pci_subsystem_device(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "pci_subsystem_device", "0x%04x", ocs->pci_subsystem_device);
}

static void
get_tgt_rscn_delay(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "tgt_rscn_delay", "%ld", ocs->tgt_rscn_delay_msec / 1000);
}

static void
get_tgt_rscn_period(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "tgt_rscn_period", "%ld", ocs->tgt_rscn_period_msec / 1000);
}

static void
get_inject_drop_cmd(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "inject_drop_cmd", "%d",
			(ocs->err_injection == INJECT_DROP_CMD ? 1:0));
}

static void
get_inject_free_drop_cmd(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "inject_free_drop_cmd", "%d",
			(ocs->err_injection == INJECT_FREE_DROPPED ? 1:0));
}

static void
get_inject_drop_data(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "inject_drop_data", "%d",
			(ocs->err_injection == INJECT_DROP_DATA ? 1:0));
}

static void
get_inject_drop_resp(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "inject_drop_resp", "%d",
			(ocs->err_injection == INJECT_DROP_RESP ? 1:0));
}

static void
get_cmd_err_inject(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "cmd_err_inject", "0x%02x", ocs->cmd_err_inject);
}

static void
get_cmd_delay_value(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "cmd_delay_value", "%ld", ocs->delay_value_msec);
}

static void
get_businfo(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "businfo", ocs->businfo);
}

static void
get_lip_count(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "lip_count", "%d",
			  ocs->xport? ocs->xport->lip_count: 0);
}

static void
get_active_xri_count(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "active_xri_count", "%d",
			  ocs_hal_io_get_count(&ocs->hal, OCS_HAL_IO_INUSE_COUNT));
}
static void
get_active_rpi_count(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "active_rpi_count", "%d",
			  ocs_hal_rpi_active_count(&ocs->hal));
}

static void
get_sfp_a0(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint8_t *page_data;
	char *buf;
	int i;
	int32_t bytes_read;

	page_data = ocs_malloc(ocs, SFP_PAGE_SIZE, OCS_M_ZERO);
	if (page_data == NULL) {
		return;
	}

	buf = ocs_malloc(ocs, (SFP_PAGE_SIZE * 3) + 1, OCS_M_ZERO);
	if (buf == NULL) {
		ocs_free(ocs, page_data, SFP_PAGE_SIZE);
		return;
	}

	bytes_read = ocs_mgmt_get_sfp(ocs, SFP_PAGE_A0, page_data, SFP_PAGE_SIZE);
	if (bytes_read <= 0) {
		if (bytes_read == -ENODEV) {
			ocs_log_debug(ocs, "No SFP module present on this device\n");
			ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "sfp_a0", "no-sfp");
		} else {
			ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "sfp_a0", "(unknown)");
		}
	} else {
		char *d = buf;
		uint8_t *s = page_data;
		int buffer_remaining = (SFP_PAGE_SIZE * 3) + 1;
		int bytes_added;

		for (i = 0; i < bytes_read; i++) {
			bytes_added = ocs_snprintf(d, buffer_remaining, "%02x ", *s);
			++s;
			d += bytes_added;
			buffer_remaining -= bytes_added;
		}
		*d = '\0';
		ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "sfp_a0", buf);
	}

	ocs_free(ocs, page_data, SFP_PAGE_SIZE);
	ocs_free(ocs, buf, (3 * SFP_PAGE_SIZE) + 1);
}

static void
get_sfp_a2(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint8_t *page_data;
	char *buf;
	int i;
	int32_t bytes_read;

	page_data = ocs_malloc(ocs, SFP_PAGE_SIZE, OCS_M_ZERO);
	if (page_data == NULL) {
		return;
	}

	buf = ocs_malloc(ocs, (SFP_PAGE_SIZE * 3) + 1, OCS_M_ZERO);
	if (buf == NULL) {
		ocs_free(ocs, page_data, SFP_PAGE_SIZE);
		return;
	}

	bytes_read = ocs_mgmt_get_sfp(ocs, SFP_PAGE_A2, page_data, SFP_PAGE_SIZE);
	if (bytes_read <= 0) {
		ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "sfp_a2", "(unknown)");
	} else {
		char *d = buf;
		uint8_t *s = page_data;
		int buffer_remaining = (SFP_PAGE_SIZE * 3) + 1;
		int bytes_added;

		for (i=0; i < bytes_read; i++) {
			bytes_added = ocs_snprintf(d, buffer_remaining, "%02x ", *s);
			++s;
			d += bytes_added;
			buffer_remaining -= bytes_added;
		}
		*d = '\0';
		ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "sfp_a2", buf);
	}

	ocs_free(ocs, page_data, SFP_PAGE_SIZE);
	ocs_free(ocs, buf, (3 * SFP_PAGE_SIZE) + 1);
}

static void
get_fw_dump_present(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint32_t dump_present = OCS_FW_DUMP_STATE_NONE;

	ocs_hal_get(&ocs->hal, OCS_HAL_DUMP_PRESENT, &dump_present);
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "fw_dump_present", "%d", dump_present);
}

static void
get_fw_dump_max_size(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint32_t max_dump_size = 0;

	ocs_hal_get(&ocs->hal, OCS_HAL_DUMP_MAX_SIZE, &max_dump_size);
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "fw_dump_max_size", "%d", max_dump_size);
}

static void
get_debug_mq_dump(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{

	ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RW, "debug_mq_dump",
		ocs_debug_is_enabled(OCS_DEBUG_ENABLE_MQ_DUMP));
}

static void
get_debug_cq_dump(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{

	ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RW, "debug_cq_dump",
		ocs_debug_is_enabled(OCS_DEBUG_ENABLE_CQ_DUMP));
}

static void
get_debug_wq_dump(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RW, "debug_wq_dump",
		ocs_debug_is_enabled(OCS_DEBUG_ENABLE_WQ_DUMP));
}

static void
get_debug_eq_dump(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RW, "debug_eq_dump",
		ocs_debug_is_enabled(OCS_DEBUG_ENABLE_EQ_DUMP));
}

static void
get_logmask(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "logmask", "0x%02x", ocs->logmask);

}

static void
get_loglevel(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "loglevel", "%d", loglevel);

}

static void
get_current_speed(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint32_t value;

	ocs_hal_get(&(ocs->hal), OCS_HAL_LINK_SPEED, &value);

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "current_speed", "%d", value);
}

static void
get_configured_speed(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint32_t value;

	ocs_hal_get(&(ocs->hal), OCS_HAL_LINK_CONFIG_SPEED, &value);
	if (value == 0) {
		ocs_mgmt_emit_string(textbuf, MGMT_MODE_RW, "configured_speed", "auto");
	} else {
		ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "configured_speed", "%d", value);
	}

}

static void
get_current_topology(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint32_t value;

	ocs_hal_get(&(ocs->hal), OCS_HAL_TOPOLOGY, &value);
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "current_topology", "%d", value);

}

static void
get_configured_topology(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint32_t value;

	ocs_hal_get(&(ocs->hal), OCS_HAL_CONFIG_TOPOLOGY, &value);
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "configured_topology", "%d", value);

}

static void
get_current_link_state(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_xport_stats_t value;

	if (ocs_xport_status(ocs->xport, OCS_XPORT_PORT_STATUS, &value) == 0) {
		if (value.value == OCS_XPORT_PORT_ONLINE) {
			ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "current_link_state", "online");
		} else {
			ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "current_link_state", "offline");
		}
	}
}

static void
get_logical_link_speed(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_xport_stats_t value;

	ocs_xport_status(ocs->xport, OCS_XPORT_LOGICAL_LINK_SPEED, &value);
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "logical_link_speed", "%d", value.value);
}

static void
get_configured_link_state(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_xport_stats_t value;

	if (ocs_xport_status(ocs->xport, OCS_XPORT_CONFIG_PORT_STATUS, &value) == 0) {
		if (value.value == OCS_XPORT_PORT_ONLINE) {
			ocs_mgmt_emit_string(textbuf, MGMT_MODE_RW, "configured_link_state", "online");
		} else {
			ocs_mgmt_emit_string(textbuf, MGMT_MODE_RW, "configured_link_state", "offline");
		}
	}
}

#if defined(ENABLE_FABRIC_EMULATION)
static void
get_femul_enable(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "femul_enable",  "%d",
		(ocs->ctrlmask & OCS_CTRLMASK_ENABLE_FABRIC_EMULATION) != 0);
}
#endif

/**
 * @brief HAL link config enum to mgmt string value mapping.
 *
 * This structure provides a mapping from the ocs_hal_linkcfg_e
 * enum (enum exposed for the OCS_HAL_PORT_SET_LINK_CONFIG port
 * control) to the mgmt string that is passed in by the mgmt application
 * (elxsdkutil).
 */
typedef struct ocs_mgmt_linkcfg_map_s {
	ocs_hal_linkcfg_e linkcfg;
	const char *mgmt_str;
} ocs_mgmt_linkcfg_map_t;

static ocs_mgmt_linkcfg_map_t mgmt_linkcfg_map[] = {
	{OCS_HAL_LINKCFG_4X10G, OCS_CONFIG_LINKCFG_4X10G},
	{OCS_HAL_LINKCFG_1X40G, OCS_CONFIG_LINKCFG_1X40G},
	{OCS_HAL_LINKCFG_2X16G, OCS_CONFIG_LINKCFG_2X16G},
	{OCS_HAL_LINKCFG_4X8G, OCS_CONFIG_LINKCFG_4X8G},
	{OCS_HAL_LINKCFG_4X1G, OCS_CONFIG_LINKCFG_4X1G},
	{OCS_HAL_LINKCFG_2X10G, OCS_CONFIG_LINKCFG_2X10G},
	{OCS_HAL_LINKCFG_2X10G_2X8G, OCS_CONFIG_LINKCFG_2X10G_2X8G}};

/**
 * @brief Get the HAL linkcfg enum from the mgmt config string.
 *
 * @param mgmt_str mgmt string value.
 *
 * @return Returns the HAL linkcfg enum corresponding to clp_str.
 */
static ocs_hal_linkcfg_e
ocs_hal_linkcfg_from_mgmt(const char *mgmt_str)
{
	uint32_t i;
	for (i = 0; i < ARRAY_SIZE(mgmt_linkcfg_map); i++) {
		if (ocs_strncmp(mgmt_linkcfg_map[i].mgmt_str,
				mgmt_str, ocs_strlen(mgmt_str)) == 0) {
			return mgmt_linkcfg_map[i].linkcfg;
		}
	}
	return OCS_HAL_LINKCFG_NA;
}

/**
 * @brief Get the mgmt string value from the HAL linkcfg enum.
 *
 * @param linkcfg HAL linkcfg enum.
 *
 * @return Returns the mgmt string value corresponding to the given HAL linkcfg.
 */
static const char *
ocs_mgmt_from_hal_linkcfg(ocs_hal_linkcfg_e linkcfg)
{
	uint32_t i;
	for (i = 0; i < ARRAY_SIZE(mgmt_linkcfg_map); i++) {
		if (mgmt_linkcfg_map[i].linkcfg == linkcfg) {
			return mgmt_linkcfg_map[i].mgmt_str;
		}
	}
	return OCS_CONFIG_LINKCFG_UNKNOWN;
}

/**
 * @brief Link configuration callback argument
 */
typedef struct ocs_mgmt_linkcfg_arg_s {
	ocs_sem_t semaphore;
	int32_t status;
	ocs_hal_linkcfg_e linkcfg;
} ocs_mgmt_linkcfg_arg_t;

/**
 * @brief Get linkcfg config value
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Not used.
 * @param textbuf The textbuf to which the result is written.
 *
 * @return None.
 */
static void
get_linkcfg(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	const char *linkcfg_str = NULL;
	uint32_t value;
	ocs_hal_linkcfg_e linkcfg;
	ocs_hal_get(&ocs->hal, OCS_HAL_LINKCFG, &value);
	linkcfg = (ocs_hal_linkcfg_e)value;
	linkcfg_str = ocs_mgmt_from_hal_linkcfg(linkcfg);
	ocs_mgmt_emit_string(textbuf, MGMT_MODE_RW, "linkcfg", linkcfg_str);
}

/**
 * @brief Get requested WWNN config value
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Not used.
 * @param textbuf The textbuf to which the result is written.
 *
 * @return None.
 */
static void
get_req_wwnn(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_xport_t *xport = ocs->xport;

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "requested_wwnn", "0x%" PRIx64 , xport->req_wwnn);
}

/**
 * @brief Get requested WWPN config value
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Not used.
 * @param textbuf The textbuf to which the result is written.
 *
 * @return None.
 */
static void
get_req_wwpn(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_xport_t *xport = ocs->xport;

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "requested_wwpn", "0x%" PRIx64 , xport->req_wwpn);
}

/**
 * @brief Get requested nodedb_mask config value
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Not used.
 * @param textbuf The textbuf to which the result is written.
 *
 * @return None.
 */
static void
get_nodedb_mask(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RW, "nodedb_mask", "0x%08x", ocs->nodedb_mask);
}

/**
 * @brief Set requested WWNN value.
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Not used.
 * @param value Value to which the linkcfg is set.
 *
 * @return Returns 0 on success.
 */

static int
set_req_wwnn(ocs_t *ocs, char *name, char *value)
{
	int rc;
	uint64_t wwnn;

	if (ocs_strcasecmp(value, "default") == 0) {
		wwnn = 0;
	}
	else if (parse_wwn(value, &wwnn) != 0) {
		ocs_log_test(ocs, "Invalid WWNN: %s\n", value);
		return -1;
	}

	rc = ocs_xport_control(ocs->xport, OCS_XPORT_WWNN_SET, wwnn);

	if(rc) {
		ocs_log_test(ocs, "OCS_XPORT_WWNN_SET failed: %d\n", rc);
	}

	return rc;
}

/**
 * @brief Set requested WWNP value.
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Not used.
 * @param value Value to which the linkcfg is set.
 *
 * @return Returns 0 on success.
 */

static int
set_req_wwpn(ocs_t *ocs, char *name, char *value)
{
	int rc;
	uint64_t wwpn;

	if (ocs_strcasecmp(value, "default") == 0) {
		wwpn = 0;
	}
	else if (parse_wwn(value, &wwpn) != 0) {
		ocs_log_test(ocs, "Invalid WWPN: %s\n", value);
		return -1;
	}

	rc = ocs_xport_control(ocs->xport, OCS_XPORT_WWPN_SET, wwpn);

	if(rc) {
		ocs_log_test(ocs, "OCS_XPORT_WWPN_SET failed: %d\n", rc);
	}

	return rc;
}

/**
 * @brief Set node debug mask value
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Not used.
 * @param value Value to which the nodedb_mask is set.
 *
 * @return Returns 0 on success.
 */
static int
set_nodedb_mask(ocs_t *ocs, char *name, char *value)
{
	ocs->nodedb_mask = ocs_strtoul(value, 0, 0);
	return 0;
}

/**
 * @brief Set linkcfg config value.
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Not used.
 * @param value Value to which the linkcfg is set.
 *
 * @return Returns 0 on success.
 */
static int
set_linkcfg(ocs_t *ocs, char *name, char *value)
{
	ocs_hal_linkcfg_e linkcfg;
	ocs_mgmt_linkcfg_arg_t cb_arg;
	ocs_hal_rtn_e status;

	ocs_sem_init(&cb_arg.semaphore, 0, "mgmt_linkcfg");

	/* translate mgmt linkcfg string to HAL linkcfg enum */
	linkcfg = ocs_hal_linkcfg_from_mgmt(value);

	/* set HAL linkcfg */
	status = ocs_hal_port_control(&ocs->hal, OCS_HAL_PORT_SET_LINK_CONFIG,
				      (uintptr_t)linkcfg, ocs_mgmt_linkcfg_cb, &cb_arg);
	if (status) {
		ocs_log_test(ocs, "ocs_hal_set_linkcfg failed\n");
		return -1;
	}

	if (ocs_sem_p(&cb_arg.semaphore, OCS_SEM_FOREVER)) {
		ocs_log_err(ocs, "ocs_sem_p failed\n");
		return -1;
	}

	if (cb_arg.status) {
		ocs_log_test(ocs, "failed to set linkcfg from HAL status=%d\n", cb_arg.status);
		return -1;
	}

	return 0;
}

/**
 * @brief Linkcfg callback
 *
 * @param status Result of the linkcfg get/set operation.
 * @param value Resulting linkcfg value.
 * @param arg Callback argument.
 *
 * @return None.
 */
static void
ocs_mgmt_linkcfg_cb(int32_t status, uintptr_t value, void *arg)
{
	ocs_mgmt_linkcfg_arg_t *cb_arg = (ocs_mgmt_linkcfg_arg_t *)arg;
	cb_arg->status = status;
	cb_arg->linkcfg = (ocs_hal_linkcfg_e)value;
	ocs_sem_v(&cb_arg->semaphore);
}

static int
set_debug_mq_dump(ocs_t *ocs, char *name, char *value)
{
	int result;

	if (ocs_strcasecmp(value, "false") == 0) {
		ocs_debug_disable(OCS_DEBUG_ENABLE_MQ_DUMP);
		result = 0;
	} else if (ocs_strcasecmp(value, "true") == 0) {
		ocs_debug_enable(OCS_DEBUG_ENABLE_MQ_DUMP);
		result = 0;
	} else {
		result = -1;
	}

	return result;
}

static int
set_debug_cq_dump(ocs_t *ocs, char *name, char *value)
{
	int result;

	if (ocs_strcasecmp(value, "false") == 0) {
		ocs_debug_disable(OCS_DEBUG_ENABLE_CQ_DUMP);
		result = 0;
	} else if (ocs_strcasecmp(value, "true") == 0) {
		ocs_debug_enable(OCS_DEBUG_ENABLE_CQ_DUMP);
		result = 0;
	} else {
		result = -1;
	}

	return result;
}

static int
set_debug_wq_dump(ocs_t *ocs, char *name, char *value)
{
	int result;

	if (ocs_strcasecmp(value, "false") == 0) {
		ocs_debug_disable(OCS_DEBUG_ENABLE_WQ_DUMP);
		result = 0;
	} else if (ocs_strcasecmp(value, "true") == 0) {
		ocs_debug_enable(OCS_DEBUG_ENABLE_WQ_DUMP);
		result = 0;
	} else {
		result = -1;
	}

	return result;
}

static int
set_debug_eq_dump(ocs_t *ocs, char *name, char *value)
{
	int result;

	if (ocs_strcasecmp(value, "false") == 0) {
		ocs_debug_disable(OCS_DEBUG_ENABLE_EQ_DUMP);
		result = 0;
	} else if (ocs_strcasecmp(value, "true") == 0) {
		ocs_debug_enable(OCS_DEBUG_ENABLE_EQ_DUMP);
		result = 0;
	} else {
		result = -1;
	}

	return result;
}

static int
set_logmask(ocs_t *ocs, char *name, char *value)
{

	ocs->logmask = ocs_strtoul(value, NULL, 0);

	return 0;
}

static int
set_loglevel(ocs_t *ocs, char *name, char *value)
{

	loglevel = ocs_strtoul(value, NULL, 0);

	return 0;
}

static int
set_configured_speed(ocs_t *ocs, char *name, char *value)
{
	int result = 0;
	ocs_hal_rtn_e hal_rc;
	int xport_rc;
	uint32_t spd;

	spd = ocs_strtoul(value, NULL, 0);

	ocs_log_debug(ocs, "Taking port offline\n");

	ocs->xport->forced_link_down = 1;
	xport_rc = ocs_xport_control(ocs->xport, OCS_XPORT_PORT_OFFLINE);
	if (xport_rc != 0) {
		ocs_log_test(ocs, "Port offline failed\n");
		ocs->xport->forced_link_down = 0;
		result = -1;
	} else {
		ocs_log_debug(ocs, "Setting port to speed %d\n", spd);
		hal_rc = ocs_hal_set(&ocs->hal, OCS_HAL_LINK_SPEED, spd);
		if (hal_rc != OCS_HAL_RTN_SUCCESS) {
			ocs_log_test(ocs, "Speed set failed\n");
			result = -1;
		}

		// If we failed to set the speed we still want to try to bring
		// the port back online

		ocs_log_debug(ocs, "Bringing port online\n");
		xport_rc = ocs_xport_control(ocs->xport, OCS_XPORT_PORT_ONLINE);
		if (xport_rc != 0) {
			result = -1;
		}
	}

	return result;
}

static int
set_configured_topology(ocs_t *ocs, char *name, char *value)
{
	int32_t result = 0;
	ocs_hal_rtn_e hal_rc;
	int32_t xport_rc;
	uint32_t topo;

	topo = ocs_strtoul(value, NULL, 0);
	if (topo >= OCS_HAL_TOPOLOGY_MAX)
		return -EINVAL;

	ocs_log_debug(ocs, "Taking port offline\n");
	ocs->xport->forced_link_down = 1;
	xport_rc = ocs_xport_control(ocs->xport, OCS_XPORT_PORT_OFFLINE);
	if (xport_rc != 0) {
		ocs_log_err(ocs, "Port offline failed\n");
		ocs->xport->forced_link_down = 0;
		result = -EIO;
	} else {
		ocs_log_debug(ocs, "Setting port to topology %d\n", topo);
		hal_rc = ocs_hal_set(&ocs->hal, OCS_HAL_TOPOLOGY, topo);
		if (hal_rc != OCS_HAL_RTN_SUCCESS) {
			ocs_log_err(ocs, "Topology set failed\n");
			result = -EIO;
		}

		// If we failed to set the topology we still want to try to bring
		// the port back online

		ocs_log_debug(ocs, "Bringing port online\n");
		xport_rc = ocs_xport_control(ocs->xport, OCS_XPORT_PORT_ONLINE);
		if (xport_rc != 0) {
			ocs_log_err(ocs, "Port online failed\n");
			result = -EIO;
		}
	}

	return result;
}

static int
set_configured_link_state(ocs_t *ocs, char *name, char *value)
{
	int result = 0;
	int xport_rc;

	if (ocs_strcasecmp(value, "offline") == 0) {
		ocs_log_debug(ocs, "Setting port to %s\n", value);
		ocs->xport->forced_link_down = 1;
		xport_rc = ocs_xport_control(ocs->xport, OCS_XPORT_PORT_OFFLINE);
		if (xport_rc != 0) {
			ocs_log_test(ocs, "Setting port to offline failed\n");
			ocs->xport->forced_link_down = 0;
			result = -1;
		}
	} else if (ocs_strcasecmp(value, "online") == 0) {
		ocs_log_debug(ocs, "Setting port to %s\n", value);
		xport_rc = ocs_xport_control(ocs->xport, OCS_XPORT_PORT_ONLINE);
		if (xport_rc != 0) {
			ocs_log_test(ocs, "Setting port to online failed\n");
			result = -1;
		}
	} else {
		ocs_log_test(ocs, "Unsupported link state \"%s\"\n", value);
		result = -1;
	}

	return result;
}

#if defined(ENABLE_FABRIC_EMULATION)
static int
set_femul_enable(ocs_t *ocs, char *name, char *value)
{
	if (ocs_strtoul(value, 0, 0) != 0) {
		ocs->ctrlmask |= OCS_CTRLMASK_ENABLE_FABRIC_EMULATION;
	} else {
		ocs->ctrlmask &= ~OCS_CTRLMASK_ENABLE_FABRIC_EMULATION;
	}
	return 0;
}
#endif

typedef struct ocs_mgmt_get_port_protocol_result {
	ocs_sem_t semaphore;
	int32_t status;
	ocs_hal_port_protocol_e port_protocol;
} ocs_mgmt_get_port_protocol_result_t;


static void
ocs_mgmt_get_port_protocol_cb(int32_t status,
			      ocs_hal_port_protocol_e port_protocol,
			      void    *arg)
{
	ocs_mgmt_get_port_protocol_result_t *result = arg;

	result->status = status;
	result->port_protocol = port_protocol;

	ocs_sem_v(&(result->semaphore));
}

static void
get_port_protocol(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_get_port_protocol_result_t result;
	uint8_t bus;
	uint8_t dev;
	uint8_t func;

	ocs_sem_init(&(result.semaphore), 0, "get_port_protocol");

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);

	if(ocs_hal_get_port_protocol(&ocs->hal, func, ocs_mgmt_get_port_protocol_cb, &result) == OCS_HAL_RTN_SUCCESS) {
		if (ocs_sem_p(&(result.semaphore), OCS_SEM_FOREVER) != 0) {
			// Undefined failure
			ocs_log_err(ocs, "ocs_sem_p failed\n");
			return;
		}
		if (result.status == 0) {
			switch (result.port_protocol) {
			case OCS_HAL_PORT_PROTOCOL_ISCSI:
				ocs_mgmt_emit_string(textbuf, MGMT_MODE_RW, "port_protocol", "iSCSI");
				break;
			case OCS_HAL_PORT_PROTOCOL_FCOE:
				ocs_mgmt_emit_string(textbuf, MGMT_MODE_RW, "port_protocol", "FCoE");
				break;
			case OCS_HAL_PORT_PROTOCOL_FC:
				ocs_mgmt_emit_string(textbuf, MGMT_MODE_RW, "port_protocol", "FC");
				break;
			case OCS_HAL_PORT_PROTOCOL_OTHER:
				ocs_mgmt_emit_string(textbuf, MGMT_MODE_RW, "port_protocol", "Other");
				break;
			}
		} else {
			ocs_log_test(ocs, "getting port profile status 0x%x\n", result.status);
			ocs_mgmt_emit_string(textbuf, MGMT_MODE_RW, "port_protocol", "Unknown");
		}
	}
}

typedef struct ocs_mgmt_set_port_protocol_result {
	ocs_sem_t semaphore;
	int32_t status;
} ocs_mgmt_set_port_protocol_result_t;



static void
ocs_mgmt_set_port_protocol_cb(int32_t status,
			      void    *arg)
{
	ocs_mgmt_get_port_protocol_result_t *result = arg;

	result->status = status;

	ocs_sem_v(&(result->semaphore));
}

/**
 * @brief  Set port protocol
 * @par Description
 * This is a management action handler to set the current
 * port protocol.  Input value should be one of iSCSI,
 * FC, or FCoE.
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Name of the action being performed.
 * @param value The value to be assigned
 *
 * @return Returns 0 on success, non-zero on failure.
 */
static int32_t
set_port_protocol(ocs_t *ocs, char *name, char *value)
{
	ocs_mgmt_set_port_protocol_result_t result;
	int32_t rc = 0;
	ocs_hal_port_protocol_e new_protocol;
	uint8_t bus;
	uint8_t dev;
	uint8_t func;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);

	ocs_sem_init(&(result.semaphore), 0, "set_port_protocol");

	if (ocs_strcasecmp(value, "iscsi") == 0) {
		new_protocol = OCS_HAL_PORT_PROTOCOL_ISCSI;
	} else if (ocs_strcasecmp(value, "fc") == 0) {
		new_protocol = OCS_HAL_PORT_PROTOCOL_FC;
	} else if (ocs_strcasecmp(value, "fcoe") == 0) {
		new_protocol = OCS_HAL_PORT_PROTOCOL_FCOE;
	} else {
		return -1;
	}

	rc = ocs_hal_set_port_protocol(&ocs->hal, new_protocol, func,
				       ocs_mgmt_set_port_protocol_cb, &result);
	if (rc == OCS_HAL_RTN_SUCCESS) {
		if (ocs_sem_p(&(result.semaphore), OCS_SEM_FOREVER) != 0) {
			// Undefined failure
			ocs_log_err(ocs, "ocs_sem_p failed\n");
			return -ENXIO;
		}
		if (result.status == 0) {
			// Success.
			rc = 0;
		} else {
			rc = -1;
			ocs_log_test(ocs, "setting active profile status 0x%x\n", result.status);
		}
	}

	return rc;
}

typedef struct ocs_mgmt_get_profile_list_result_s {
	ocs_sem_t semaphore;
	int32_t status;
	ocs_hal_profile_list_t *list;
} ocs_mgmt_get_profile_list_result_t;

static void
ocs_mgmt_get_profile_list_cb(int32_t status, ocs_hal_profile_list_t *list, void *ul_arg)
{
	ocs_mgmt_get_profile_list_result_t *result = ul_arg;

	result->status = status;
	result->list = list;

	ocs_sem_v(&(result->semaphore));
}

/**
 * @brief  Get list of profiles
 * @par Description
 * This is a management action handler to get the list of
 * profiles supported by the SLI port.  Although the spec says
 * that all SLI platforms support this, only Skyhawk actually
 * has a useful implementation.
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Name of the action being performed.
 * @param textbuf Pointer to an ocs_textbuf, which is used to return the results.
 *
 * @return none
 */
static void
get_profile_list(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	ocs_mgmt_get_profile_list_result_t result;

	ocs_sem_init(&(result.semaphore), 0, "get_profile_list");

	if(ocs_hal_get_profile_list(&ocs->hal, ocs_mgmt_get_profile_list_cb, &result) == OCS_HAL_RTN_SUCCESS) {
		if (ocs_sem_p(&(result.semaphore), OCS_SEM_FOREVER) != 0) {
			// Undefined failure
			ocs_log_err(ocs, "ocs_sem_p failed\n");
			return;
		}
		if (result.status == 0) {
			// Success.
#define MAX_LINE_SIZE 520
#define BUFFER_SIZE MAX_LINE_SIZE*40
			char *result_buf;
			char result_line[MAX_LINE_SIZE];
			uint32_t bytes_left;
			uint32_t i;

			result_buf = ocs_malloc(ocs, BUFFER_SIZE, OCS_M_ZERO);
			bytes_left = BUFFER_SIZE;

			for (i=0; i<result.list->num_descriptors; i++) {
				sprintf(result_line, "0x%02x:%s\n", result.list->descriptors[i].profile_id,
					result.list->descriptors[i].profile_description);
				if (strlen(result_line) < bytes_left) {
					strcat(result_buf, result_line);
					bytes_left -= strlen(result_line);
				}
			}


			ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "profile_list", result_buf);

			ocs_free(ocs, result_buf, BUFFER_SIZE);
			ocs_free(ocs, result.list, sizeof(ocs_hal_profile_list_t));
		} else {
			ocs_log_test(ocs, "getting profile list status 0x%x\n", result.status);
		}
	}
}

typedef struct ocs_mgmt_get_active_profile_result {
	ocs_sem_t semaphore;
	int32_t status;
	uint32_t active_profile_id;
} ocs_mgmt_get_active_profile_result_t;

static void
ocs_mgmt_get_active_profile_cb(int32_t status, uint32_t active_profile, void *ul_arg)
{
	ocs_mgmt_get_active_profile_result_t *result = ul_arg;

	result->status = status;
	result->active_profile_id = active_profile;

	ocs_sem_v(&(result->semaphore));
}

#define MAX_PROFILE_LENGTH 5

/**
 * @brief  Get active profile
 * @par Description
 * This is a management action handler to get the currently
 * active profile for an SLI port.  Although the spec says that
 * all SLI platforms support this, only Skyhawk actually has a
 * useful implementation.
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Name of the action being performed.
 * @param textbuf Pointer to an ocs_textbuf, which is used to return the results.
 *
 * @return none
 */
static void
get_active_profile(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	char result_string[MAX_PROFILE_LENGTH];
	ocs_mgmt_get_active_profile_result_t result;

	ocs_sem_init(&(result.semaphore), 0, "get_active_profile");

	if(ocs_hal_get_active_profile(&ocs->hal, ocs_mgmt_get_active_profile_cb, &result) == OCS_HAL_RTN_SUCCESS) {
		if (ocs_sem_p(&(result.semaphore), OCS_SEM_FOREVER) != 0) {
			// Undefined failure
			ocs_log_err(ocs, "ocs_sem_p failed\n");
			return;
		}
		if (result.status == 0) {
			// Success.
			sprintf(result_string, "0x%02x", result.active_profile_id);
			ocs_mgmt_emit_string(textbuf, MGMT_MODE_RW, "active_profile", result_string);
		} else {
			ocs_log_test(ocs, "getting active profile status 0x%x\n", result.status);
		}
	}
}

typedef struct ocs_mgmt_set_active_profile_result {
	ocs_sem_t semaphore;
	int32_t status;
} ocs_mgmt_set_active_profile_result_t;


static void
ocs_mgmt_set_active_profile_cb(int32_t status, void *ul_arg)
{
	ocs_mgmt_get_profile_list_result_t *result = ul_arg;

	result->status = status;

	ocs_sem_v(&(result->semaphore));
}

/**
 * @brief  Set active profile
 * @par Description
 * This is a management action handler to set the currently
 * active profile for an SLI port.  Although the spec says that
 * all SLI platforms support this, only Skyhawk actually has a
 * useful implementation.
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Name of the action being performed.
 * @param value Requested new value of the property.
 *
 * @return Returns 0 on success, non-zero on failure.
 */
static int32_t
set_active_profile(ocs_t *ocs, char *name, char *value)
{
	ocs_mgmt_set_active_profile_result_t result;
	int32_t rc = 0;
	int32_t new_profile;

	new_profile = ocs_strtoul(value, NULL, 0);

	ocs_sem_init(&(result.semaphore), 0, "set_active_profile");

	rc = ocs_hal_set_active_profile(&ocs->hal, ocs_mgmt_set_active_profile_cb, new_profile, &result);
	if (rc == OCS_HAL_RTN_SUCCESS) {
		if (ocs_sem_p(&(result.semaphore), OCS_SEM_FOREVER) != 0) {
			// Undefined failure
			ocs_log_err(ocs, "ocs_sem_p failed\n");
			return -ENXIO;
		}
		if (result.status == 0) {
			// Success.
			rc = 0;
		} else {
			rc = -1;
			ocs_log_test(ocs, "setting active profile status 0x%x\n", result.status);
		}
	}

	return rc;
}

typedef struct ocs_mgmt_get_nvparms_result {
	ocs_sem_t semaphore;
	int32_t status;
	uint8_t	wwpn[8];
	uint8_t wwnn[8];
	uint8_t hard_alpa;
	uint32_t preferred_d_id;
	ocs_atomic_t refcnt;
} ocs_mgmt_get_nvparms_result_t;

static void
ocs_mgmt_get_nvparms_cb(void *os, int32_t status, uint8_t *wwpn, uint8_t *wwnn, uint8_t hard_alpa,
		uint32_t preferred_d_id, void *ul_arg)
{
	ocs_mgmt_get_nvparms_result_t *result = ul_arg;

	result->status = status;
	ocs_memcpy(result->wwpn, wwpn, sizeof(result->wwpn));
	ocs_memcpy(result->wwnn, wwnn, sizeof(result->wwnn));
	result->hard_alpa = hard_alpa;
	result->preferred_d_id = preferred_d_id;

	ocs_sem_v(&result->semaphore);

	if (ocs_atomic_sub_and_test(&result->refcnt, 1))
		ocs_free(os, result, sizeof(ocs_mgmt_get_nvparms_result_t));
}

/**
 * @brief  Get wwpn
 * @par Description
 *
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Name of the action being performed.
 * @param textbuf Pointer to an ocs_textbuf, which is used to return the results.
 *
 * @return none
 */
static void
get_nv_wwpn(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	char result_string[24];
	ocs_mgmt_get_nvparms_result_t *result;

	result = ocs_malloc(ocs, sizeof(ocs_mgmt_get_nvparms_result_t), OCS_M_NOWAIT | OCS_M_ZERO);
	if (result == NULL) {
		ocs_log_err(ocs, "failed to allocate memory\n");
		return;
	}

	ocs_sem_init(&result->semaphore, 0, "get_nv_wwpn");
	ocs_atomic_init(&result->refcnt, 2);

	if(ocs_hal_get_nvparms(&ocs->hal, ocs_mgmt_get_nvparms_cb, result) == OCS_HAL_RTN_SUCCESS) {
		if (ocs_sem_p(&result->semaphore, OCS_SEM_FOREVER) != 0) {
			// Undefined failure
			ocs_log_err(ocs, "ocs_sem_p failed\n");
			goto exit;
		}
		if (result->status == 0) {
			// Success.  Copy wwpn from result struct to result string
			sprintf(result_string, "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
					result->wwpn[0], result->wwpn[1], result->wwpn[2],
					result->wwpn[3], result->wwpn[4], result->wwpn[5],
					result->wwpn[6], result->wwpn[7]);
			ocs_mgmt_emit_string(textbuf, MGMT_MODE_RW, "nv_wwpn", result_string);
		} else {
			ocs_log_test(ocs, "getting wwpn status 0x%x\n", result->status);
		}
	} else {
		ocs_atomic_sub_return(&result->refcnt, 1);	
	}
exit:
	if (ocs_atomic_sub_and_test(&result->refcnt, 1))
		ocs_free(ocs, result, sizeof(ocs_mgmt_get_nvparms_result_t));
}

/**
 * @brief  Get wwnn
 * @par Description
 *
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Name of the action being performed.
 * @param textbuf Pointer to an ocs_textbuf, which is used to return the results.
 *
 * @return none
 */
static void
get_nv_wwnn(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	char result_string[24];
	ocs_mgmt_get_nvparms_result_t *result;

	result = ocs_malloc(ocs, sizeof(ocs_mgmt_get_nvparms_result_t), OCS_M_NOWAIT | OCS_M_ZERO);
	if (result == NULL) {
		ocs_log_err(ocs, "failed to allocate memory\n");
		return;
	}
	ocs_sem_init(&result->semaphore, 0, "get_nv_wwnn");
	ocs_atomic_init(&result->refcnt, 2);

	if(ocs_hal_get_nvparms(&ocs->hal, ocs_mgmt_get_nvparms_cb, result) == OCS_HAL_RTN_SUCCESS) {
		if (ocs_sem_p(&result->semaphore, OCS_SEM_FOREVER) != 0) {
			// Undefined failure
			ocs_log_err(ocs, "ocs_sem_p failed\n");
			goto exit;
		}
		if (result->status == 0) {
			// Success.  Copy wwnn from result struct to result string
			ocs_snprintf(result_string, sizeof(result_string), "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
					result->wwnn[0], result->wwnn[1], result->wwnn[2],
					result->wwnn[3], result->wwnn[4], result->wwnn[5],
					result->wwnn[6], result->wwnn[7]);
			ocs_mgmt_emit_string(textbuf, MGMT_MODE_RW, "nv_wwnn", result_string);
		} else {
			ocs_log_test(ocs, "getting wwnn status 0x%x\n", result->status);
		}
	} else {
		ocs_atomic_sub_return(&result->refcnt, 1);	
	}
exit:
	if (ocs_atomic_sub_and_test(&result->refcnt, 1))
		ocs_free(ocs, result, sizeof(ocs_mgmt_get_nvparms_result_t));
}

/**
 * @brief Get accumulated node abort counts
 * @par Description Get the sum of all nodes abort count.
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Name of the action being performed.
 * @param textbuf Pointer to an ocs_textbuf, which is used to return the results.
 *
 * @return None.
 */
static void
get_node_abort_cnt(ocs_t *ocs, char *name, ocs_textbuf_t *textbuf)
{
	uint32_t abort_counts = 0;
	ocs_domain_t *domain;
	ocs_sport_t *sport;
	ocs_node_t *node;

	if (ocs_device_lock_try(ocs) != TRUE) {
		/* Didn't get the lock */
		return;
	}

		/* Here the Device lock is held */
		ocs_list_foreach(&ocs->domain_list, domain) {
			if (ocs_domain_lock_try(domain) != TRUE) {
				/* Didn't get the lock */
				ocs_device_unlock(ocs);
				return;
			}

				/* Here the Domain lock is held */
				ocs_list_foreach(&domain->sport_list, sport) {
					if (ocs_sport_lock_try(sport) != TRUE) {
						/* Didn't get the lock */
						ocs_domain_unlock(domain);
						ocs_device_unlock(ocs);
						return;
					}

						/* Here the sport lock is held */
						ocs_list_foreach(&sport->node_list, node) {
							abort_counts += node->abort_cnt;
						}

					ocs_sport_unlock(sport);
				}

			ocs_domain_unlock(domain);
		}

	ocs_device_unlock(ocs);

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "node_abort_cnt", "%d" , abort_counts);
}

typedef struct ocs_mgmt_set_nvparms_result {
	ocs_sem_t semaphore;
	int32_t status;
} ocs_mgmt_set_nvparms_result_t;


static void
ocs_mgmt_set_nvparms_cb(int32_t status, void *ul_arg)
{
	ocs_mgmt_get_profile_list_result_t *result = ul_arg;

	result->status = status;

	ocs_sem_v(&(result->semaphore));
}

/**
 * @brief  Set wwn
 * @par Description Sets the Non-volatile worldwide names,
 * if provided.
 *
 * @param ocs Pointer to the ocs structure.
 * @param name Name of the action being performed.
 * @param wwn_p Requested new WWN values.
 *
 * @return Returns 0 on success, non-zero on failure.
 */
static int32_t
set_nv_wwn(ocs_t *ocs, char *name, char *wwn_p)
{
	ocs_mgmt_get_nvparms_result_t result;
	uint8_t new_wwpn[8];
	uint8_t new_wwnn[8];
	char *wwpn_p = NULL;
	char *wwnn_p = NULL;
	int32_t rc = -1;
	int wwpn = 0;
	int wwnn = 0;
	int i;

	/* This is a read-modify-write operation, so first we have to read
	 * the current values
	 */
	ocs_sem_init(&(result.semaphore), 0, "set_nv_wwn1");

	rc = ocs_hal_get_nvparms(&ocs->hal, ocs_mgmt_get_nvparms_cb, &result);

	if (rc == OCS_HAL_RTN_SUCCESS) {
		if (ocs_sem_p(&(result.semaphore), OCS_SEM_FOREVER) != 0) {
			/* Undefined failure */
			ocs_log_err(ocs, "ocs_sem_p failed\n");
			return -ENXIO;
		}
		if (result.status != 0) {
			ocs_log_test(ocs, "getting nvparms status 0x%x\n", result.status);
			return -1;
		}
	}

	/* wwn_p contains wwpn_p@wwnn_p values */
	if (wwn_p != NULL) {
		wwpn_p = ocs_strsep(&wwn_p, "@");
		wwnn_p = wwn_p;
	}

	if (wwpn_p != NULL) {
		wwpn = ocs_strcmp(wwpn_p, "NA");
	}

	if (wwnn_p != NULL) {
		wwnn = ocs_strcmp(wwnn_p, "NA");
	}

	/* Parse the new WWPN */
	if (wwpn != 0) {
		if (ocs_sscanf(wwpn_p, "%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx",
				&(new_wwpn[0]), &(new_wwpn[1]), &(new_wwpn[2]),
				&(new_wwpn[3]), &(new_wwpn[4]), &(new_wwpn[5]),
				&(new_wwpn[6]), &(new_wwpn[7])) != 8) {
			ocs_log_test(ocs, "can't parse WWPN %s\n", wwpn_p);
			return -1;
		}
	}

	/* Parse the new WWNN */
	if (wwnn != 0) {
		if (ocs_sscanf(wwnn_p, "%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx",
				&(new_wwnn[0]), &(new_wwnn[1]), &(new_wwnn[2]),
				&(new_wwnn[3]), &(new_wwnn[4]), &(new_wwnn[5]),
				&(new_wwnn[6]), &(new_wwnn[7])) != 8) {
			ocs_log_test(ocs, "can't parse WWNN %s\n", wwnn_p);
			return -1;
		}
	}

	for (i = 0; i < 8; i++) {
		/* Use active wwpn, if new one is not provided */
		if (wwpn == 0) {
			new_wwpn[i] = result.wwpn[i];
		}

		/* Use active wwnn, if new one is not provided */
		if (wwnn == 0) {
			new_wwnn[i] = result.wwnn[i];
		}
	}

	/* Modify the nv_wwnn and nv_wwpn, then write it back */
	ocs_sem_init(&(result.semaphore), 0, "set_nv_wwn2");

	rc = ocs_hal_set_nvparms(&ocs->hal, ocs_mgmt_set_nvparms_cb, new_wwpn,
				 new_wwnn, result.hard_alpa, result.preferred_d_id,
				 &result);
	if (rc == OCS_HAL_RTN_SUCCESS) {
		if (ocs_sem_p(&(result.semaphore), OCS_SEM_FOREVER) != 0) {
			/* Undefined failure */
			ocs_log_err(ocs, "ocs_sem_p failed\n");
			return -ENXIO;
		}
		if (result.status != 0) {
			ocs_log_test(ocs, "setting wwn status 0x%x\n", result.status);
			return -1;
		}
	}

	return rc;
}

#if defined(OCS_INCLUDE_IO_DELAY)
static int32_t
ocs_mgmt_io_delay_config(ocs_t *ocs, char *name, void *buf, uint32_t buf_len,
		void *arg_out, uint32_t arg_out_length)
{
	#define MAX_LIO_DELAY_MS 60000 /*1000 ms */
	ocs_io_delay_args_t io_delay_args;
	
	if (ocs_copy_from_user(&io_delay_args, buf, buf_len)) {
		ocs_log_err(ocs, "error: failed to copy from user addr\n");
		return -EFAULT;
	}

        if (io_delay_args.delay_max_ms < io_delay_args.delay_min_ms) {
                ocs_log_err(ocs, "delay_max_ms: min must be less than or equal to max\n");
                return -EINVAL;
        } else if (io_delay_args.delay_max_ms > MAX_LIO_DELAY_MS) {
                ocs_log_err(ocs, "delay_max_ms: range values must be less than %dms\n",
				MAX_LIO_DELAY_MS);
                return -EINVAL;
        }

	ocs->tgt_ocs.io_delay_min_ms = io_delay_args.delay_min_ms;
	ocs->tgt_ocs.io_delay_max_ms = io_delay_args.delay_max_ms;
	ocs->tgt_ocs.delay_intervel_counter = io_delay_args.delay_intervel_counter;

	return 0;
}
#endif

#if !defined(OCS_USPACE_SPDK)
static int32_t
ocs_mgmt_gen_rq_empty(ocs_t *ocs, char *name, void *buf, uint32_t buf_len, void *arg_out, uint32_t arg_out_length)
{
	ocs_rq_empty_gen_args_t rq_empty_args;
	ocs_hal_t *hal= &ocs->hal;

	if (ocs_copy_from_user(&rq_empty_args, buf, buf_len)) {
		ocs_log_err(ocs, "error: failed to copy from user addr\n");
		return -EFAULT;
	}

	if (rq_empty_args.rq_empty_enable) {
		if (rq_empty_args.drop_count) {
			if (rq_empty_args.drop_count < hal->rq_buf_drop.max_rq_buffer_drop_count) {
				ocs_log_err(ocs, "Cannot decrement drop counter\n");
			} else {
				uint32_t max_rq_entries;

				ocs_hal_get(&ocs->hal, OCS_HAL_MAX_RQ_ENTRIES, &max_rq_entries);
				if (rq_empty_args.drop_count < max_rq_entries - 100)
					hal->rq_buf_drop.max_rq_buffer_drop_count = rq_empty_args.drop_count;

				if (rq_empty_args.drop_trigger) {
					hal->rq_buf_drop.rq_buffer_drop_trigger = rq_empty_args.drop_trigger;
				} else {
					#define DEF_RQ_BUF_DROP_TRIGGER 1000
					hal->rq_buf_drop.rq_buffer_drop_trigger = DEF_RQ_BUF_DROP_TRIGGER;
				}

				ocs_log_info(ocs, "rq_buffer_drop_counter changed to : %d drop trigger: %d\n",
					     rq_empty_args.drop_count, hal->rq_buf_drop.rq_buffer_drop_trigger);
			}
		} else if (rq_empty_args.rq_thread_halt_time) {
			if (hal->rq_buf_drop.rq_pause_running) {
				ocs_log_info(ocs, "RQ halting operation already in progress\n");
				return 0;
			}

			hal->rq_buf_drop.rq_pause_running = true;
			ocs_log_info(ocs, "Setting rq_thread_pause secs to %d\n", rq_empty_args.rq_thread_halt_time);
			hal->rq_buf_drop.ms_to_sleep = rq_empty_args.rq_thread_halt_time * 1000;
			if (rq_empty_args.rq_thread_halt_start_delay) {
				ocs_log_info(ocs, "Setting RQ thread halt delay to %d secs\n",
					     rq_empty_args.rq_thread_halt_time);
				hal->rq_buf_drop.rq_thread_pause_start = ocs_get_os_ticks() +
					ocs_ms_to_ticks((uint64_t)rq_empty_args.rq_thread_halt_start_delay * 1000);
			}
		}

		hal->rq_buf_drop.enable = 1;
	} else {
		ocs_log_info(ocs, "Disabling diag RQ empty generation\n");
		ocs_memset(&hal->rq_buf_drop, 0, sizeof(ocs_hal_rq_buf_drop_t));
	}

	return 0;
}
#endif

static int
set_tgt_rscn_delay(ocs_t *ocs, char *name, char *value)
{
	ocs->tgt_rscn_delay_msec = ocs_strtoul(value, NULL, 0) * 1000;
	ocs_log_debug(ocs, "mgmt set: %s %s\n", name, value);
	return 0;
}

static int
set_tgt_rscn_period(ocs_t *ocs, char *name, char *value)
{
	ocs->tgt_rscn_period_msec = ocs_strtoul(value, NULL, 0) * 1000;
	ocs_log_debug(ocs, "mgmt set: %s %s\n", name, value);
	return 0;
}

static int
set_inject_drop_cmd(ocs_t *ocs, char *name, char *value)
{
	ocs->err_injection = (ocs_strtoul(value, NULL, 0) == 0 ? NO_ERR_INJECT : INJECT_DROP_CMD);
	ocs_log_debug(ocs, "mgmt set: %s %s\n", name, value);
	return 0;
}

static int
set_inject_free_drop_cmd(ocs_t *ocs, char *name, char *value)
{
	ocs->err_injection = (ocs_strtoul(value, NULL, 0) == 0 ? NO_ERR_INJECT : INJECT_FREE_DROPPED);
	ocs_log_debug(ocs, "mgmt set: %s %s\n", name, value);
	return 0;
}

static int
set_inject_drop_data(ocs_t *ocs, char *name, char *value)
{
	ocs->err_injection = (ocs_strtoul(value, NULL, 0) == 0 ? NO_ERR_INJECT : INJECT_DROP_DATA);
	ocs_log_debug(ocs, "mgmt set: %s %s\n", name, value);
	return 0;
}

static int
set_inject_drop_resp(ocs_t *ocs, char *name, char *value)
{
	ocs->err_injection = (ocs_strtoul(value, NULL, 0) == 0 ? NO_ERR_INJECT : INJECT_DROP_RESP);
	ocs_log_debug(ocs, "mgmt set: %s %s\n", name, value);
	return 0;
}

static int
set_cmd_err_inject(ocs_t *ocs, char *name, char *value)
{
	ocs->cmd_err_inject = ocs_strtoul(value, NULL, 0);
	ocs_log_debug(ocs, "mgmt set: %s %s\n", name, value);
	return 0;
}

static int
set_cmd_delay_value(ocs_t *ocs, char *name, char *value)
{
	ocs->delay_value_msec = ocs_strtoul(value, NULL, 0);
	ocs->err_injection = (ocs->delay_value_msec == 0 ? NO_ERR_INJECT : INJECT_DELAY_CMD);
	ocs_log_debug(ocs, "mgmt set: %s %s\n", name, value);
	return 0;
}

#if 0
static void
ocs_mgmt_status_hal(ocs_textbuf_t *textbuf, ocs_hal_t *hal)
{
	ocs_t *ocs = hal->os;

	ocs_assert(ocs);

	ocs_mgmt_start_section(textbuf, "hal", ocs->ocs_os.instance_index);


	ocs_mgmt_sli4_queue_status(textbuf, "wq", hal->wq, ARRAY_SIZE(hal->wq));
	ocs_mgmt_sli4_queue_status(textbuf, "rq", hal->rq, ARRAY_SIZE(hal->rq));
	ocs_mgmt_sli4_queue_status(textbuf, "mq", hal->mq, ARRAY_SIZE(hal->mq));
	ocs_mgmt_sli4_queue_status(textbuf, "cq", hal->cq, ARRAY_SIZE(hal->cq));
	ocs_mgmt_sli4_queue_status(textbuf, "eq", hal->eq, ARRAY_SIZE(hal->eq));

	ocs_mgmt_end_section(textbuf);
	//sli4_link_event_t link;
}

static void
ocs_mgmt_sli4_queue_status(ocs_textbuf_t *textbuf, const char *name, sli4_queue_t *q, uint32_t q_count)
{
	uint32_t i;

	for (i = 0; i < q_count; i ++, q ++) {
		ocs_mgmt_start_section(textbuf, name, i);
		ocs_mgmt_status_int(textbuf, "index", "%d", q->index);
		ocs_mgmt_status_int(textbuf, "size", "%d", q->size);
		ocs_mgmt_status_int(textbuf, "length", "%d", q->length);
		ocs_mgmt_status_int(textbuf, "n_posted", "%d", q->n_posted);
		ocs_mgmt_status_int(textbuf, "id", "%d", q->id);
		ocs_mgmt_status_int(textbuf, "type", "%d", q->type);
		ocs_mgmt_status_int(textbuf, "limit", "%d", q->limit);
		ocs_mgmt_end_section(textbuf);
	}
}
#endif

/**
 * @brief parse a WWN from a string into a 64-bit value
 *
 * Given a pointer to a string, parse the string into a 64-bit
 * WWN value.  The format of the string must be xx:xx:xx:xx:xx:xx:xx:xx
 *
 * @param wwn_in pointer to the string to be parsed
 * @param wwn_out pointer to uint64_t in which to put the parsed result
 *
 * @return 0 if successful, non-zero if the WWN is malformed and couldn't be parsed
 */
int
parse_wwn(char *wwn_in, uint64_t *wwn_out)
{
	uint8_t byte0;
	uint8_t byte1;
	uint8_t byte2;
	uint8_t byte3;
	uint8_t byte4;
	uint8_t byte5;
	uint8_t byte6;
	uint8_t byte7;
	int rc;

	if (ocs_strchr(wwn_in, ':') == NULL) {
		rc = ocs_sscanf(wwn_in, "%2hhx%2hhx%2hhx%2hhx%2hhx%2hhx%2hhx%2hhx",
				&byte0, &byte1, &byte2, &byte3,
				&byte4, &byte5, &byte6, &byte7);
	} else {
		rc = ocs_sscanf(wwn_in, "%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx",
				&byte0, &byte1, &byte2, &byte3,
				&byte4, &byte5, &byte6, &byte7);
	}

	if (rc == 8) {
		*wwn_out = ((uint64_t)byte0 << 56) |
				((uint64_t)byte1 << 48) |
				((uint64_t)byte2 << 40) |
				((uint64_t)byte3 << 32) |
				((uint64_t)byte4 << 24) |
				((uint64_t)byte5 << 16) |
				((uint64_t)byte6 <<  8) |
				((uint64_t)byte7);
		return 0;

	} else {
		return -1;
	}
}


/**
 * @page mgmt_api_overview Management APIs
 * - @ref mgmt
 *
 * <div class="overview">
 * The management interface provides a mechanism by which a program in user space
 * can interact with the running driver. Through the ioctl system call, a user program
 * can get a list of the information available, and then read/modify
 * individual values. There are three categories of management information:<br/><br/>
 *
 * - Status - A status is a read-only value. The ocs_mgmt_get_status_list() function
 * returns a list of available status properties. The ocs_mgmt_get_status()
 * function returns the value of a particular property.
 *
 * - Configuration - A configuration is a read/write value. The
 * ocs_mgmt_get_config_list() function
 * returns a list of configuration properties. The ocs_mgmt_get_config() function
 * returns the current value of a property. The ocs_mgmt_set_config() function
 * changes the value of a property.
 *
 * - Action - An action allows a user to perform an action in the driver, such as
 * updating firmware or bringing a link up/down. The ocs_mgmt_get_action_list()
 * function returns a list of available actions. The ocs_mgmt_do_action() function
 * invokes a particular action.
 *
 * </div>
 */

