/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2018 Broadcom.  All Rights Reserved.
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
 *
 * APIs exported to applications.
 */

#include "ocs.h"
#include "ocs_os.h"
#include "ocs_ocsu.h"
#include "ocs_tgt_api.h"
#include "ocs_spdk_conf.h"
#include "ocs_els.h"
#include "ocs_fabric.h"
#include "ocsu_scsi_if.h"
#include "fc_subsystem.h"
#include "spdk/scsi_spec.h"

bool
spdk_fc_api_subsystem_init(void)
{
	return spdk_fc_subsystem_init();
}

bool
spdk_fc_api_subsystem_exit(void)
{
	return spdk_fc_subsystem_fini();
}

struct spdk_fc_api_ig_list *
spdk_fc_api_get_igs(void)
{
	struct spdk_fc_api_ig_list *ig_list = NULL;
	struct spdk_fc_ig *ig = NULL, *tmp;
	int i = 0;

	ocs_lock(&g_spdk_config_lock);
	
	ig_list = ocs_malloc(NULL, sizeof(struct spdk_fc_api_ig_list), OCS_M_ZERO);
	if (!ig_list) {
		goto fail;
	}

	/* Find the list of igs */
	TAILQ_COUNT(ig_list->igs_count, &g_spdk_fc_igs.head, tailq, tmp);
	if (!ig_list->igs_count) {
		goto fail;
	}
	
	ig_list->igs = ocs_malloc(NULL, ig_list->igs_count * 
				sizeof(struct spdk_fc_ig), OCS_M_ZERO);
	if (!ig_list->igs) {
		goto fail;
	}
	
	TAILQ_FOREACH(ig, &g_spdk_fc_igs.head, tailq) {
		ocs_memcpy(&ig_list->igs[i], ig, sizeof(struct spdk_fc_ig));
		i ++;
	}

	ocs_unlock(&g_spdk_config_lock);
	return ig_list;
fail:
	if (ig_list) {
		free(ig_list);
	}	
	ocs_unlock(&g_spdk_config_lock);
	return NULL;
}

bool
spdk_fc_api_ig_add_initiator(char *initiator, int ig_id)
{
	struct spdk_fc_ig *ig = NULL, *tmp;
	int i;
	uint64_t wwn;

	ocs_lock(&g_spdk_config_lock);

	if (!initiator) {
		ocs_log_err(NULL, "Invalid initiator pointer.\n");
		goto fail;
	}

	FIND_NODE_BY_ID(g_spdk_fc_igs, ig_id, ig, tmp);
	if (!ig) {
		ocs_log_err(NULL, "IG not found.\n");
		goto fail; // Invalid IG
	}

	if (ig->num_initiators == MAX_INITIATORS_IN_IG) {
		ocs_log_err(NULL, "IG initiators exceeds Max supported %d\n",
				MAX_INITIATORS_IN_IG);
		goto fail; // No Room
	}

	
	if (ocs_strncmp(initiator, "ALL", 3) != 0) {
		// Make sure its in proper format.
		if (parse_wwn(initiator, &wwn) != 0) {
			ocs_log_err(NULL, "Initiator WWN(%s) wrong format.\n",
					initiator);
			goto fail;
		}
	}

	/* find a slot and add the initiator WWN */
	for (i = 0; i < ig->num_initiators; i ++) {
		if (ocs_strlen(ig->initiators[i]) == 0) {
			// use this index
			break;
		}
	}

	ocs_strncpy(ig->initiators[i], initiator, MAX_WWN_NAME_LEN);
	ig->num_initiators ++;

	ocs_unlock(&g_spdk_config_lock);
	return true;

fail:
	ocs_unlock(&g_spdk_config_lock);
	return false;
}

bool
spdk_fc_api_ig_del_initiator(char *initiator, int ig_id)
{
	struct spdk_fc_ig *ig = NULL, *tmp;
	int i;

	ocs_lock(&g_spdk_config_lock);
	if (!initiator) {
		ocs_log_err(NULL, "Invalid initiator pointer.\n");
		goto fail;
	}

	FIND_NODE_BY_ID(g_spdk_fc_igs, ig_id, ig, tmp);
	if (!ig) {
		ocs_log_err(NULL, "IG not found.\n");
		goto fail;
	}

	/* find the initiator WWN and delete*/
	for (i = 0; i < ig->num_initiators; i ++) {
		if (ocs_strcasecmp(ig->initiators[i], initiator) == 0) {
			spdk_fc_cf_delete_ig_initiator(ig->id, initiator);
			memset(ig->initiators[i], 0, MAX_WWN_NAME_LEN);
			ocs_unlock(&g_spdk_config_lock);
			return true;
		}
	}

fail:
	ocs_unlock(&g_spdk_config_lock);
	return false;
}


bool
spdk_fc_api_add_ig(struct spdk_fc_ig *ig)
{
	struct spdk_fc_ig *tmp = NULL, *tmp1, *new_ig = NULL;
	uint64_t wwn;
	int i;
	
	ocs_lock(&g_spdk_config_lock);
	if (!ig) {
		ocs_log_err(NULL, "Invalid IG pointer\n");
		goto fail;
	}

	if (ig->num_initiators > MAX_INITIATORS_IN_IG) {
		ocs_log_err(NULL, "IG initiators exceeds Max supported %d\n",
				MAX_INITIATORS_IN_IG);
		goto fail;
	}

	FIND_NODE_BY_ID(g_spdk_fc_igs, ig->id, tmp, tmp1);
	if (tmp) {
		ocs_log_err(NULL, "IG with id = %d already exists\n", ig->id);
		goto fail; // Already Exists	
	}

	/* Check all names are in proper format. */
	for (i = 0; i < ig->num_initiators; i ++) {
		if (ocs_strncmp(ig->initiators[i], "ALL", 3) != 0) {
			// Make sure its in proper format.
			if (parse_wwn(ig->initiators[i], &wwn) != 0) {
				ocs_log_err(NULL, "Initiator WWN(%s) wrong format.\n",
						ig->initiators[i]);
				goto fail;
			}
		}
	}

	new_ig = ocs_malloc(NULL, sizeof(struct spdk_fc_ig), OCS_M_ZERO);
	if (!new_ig) {
		ocs_log_err(NULL, "Memory allocation failed.\n");
		goto fail;
	}

	ocs_memcpy(new_ig, ig, sizeof(struct spdk_fc_ig));

	TAILQ_INSERT_TAIL(&g_spdk_fc_igs.head, new_ig, tailq);
	
	ocs_unlock(&g_spdk_config_lock);
	return true;
fail:
	if (new_ig) {
		free(new_ig);
	}
	ocs_unlock(&g_spdk_config_lock);
	return false;
}

bool
spdk_fc_api_del_ig(int ig_id)
{
	struct spdk_fc_ig *ig = NULL, *tmp;

	ocs_lock(&g_spdk_config_lock);

	FIND_NODE_BY_ID(g_spdk_fc_igs, ig_id, ig, tmp);
	if (!ig) {
		ocs_log_err(NULL, "IG not found.\n");
		goto fail; // Invalid IG
	}

	spdk_fc_cf_delete_ig(ig_id);
	
	TAILQ_REMOVE(&g_spdk_fc_igs.head, ig, tailq);
	free(ig);	

	ocs_unlock(&g_spdk_config_lock);
	return true;
fail:
	ocs_unlock(&g_spdk_config_lock);
	return false;
}


struct spdk_fc_api_lun_map_list *
spdk_fc_api_get_lun_maps(void)
{
	struct spdk_fc_api_lun_map_list *lun_map_list = NULL;
	struct spdk_fc_lun_map *lun_map = NULL, *tmp;
	int i = 0;

	ocs_lock(&g_spdk_config_lock);

	lun_map_list = ocs_malloc(NULL, sizeof(struct spdk_fc_api_lun_map_list),
			OCS_M_ZERO);
	if (!lun_map_list) {
		ocs_log_err(NULL, "Memory allocation failed\n");
		goto fail;
	}

	TAILQ_COUNT(lun_map_list->lun_maps_count, &g_spdk_fc_lun_maps.head, tailq, tmp);
	if (!lun_map_list->lun_maps_count) {
		ocs_log_err(NULL, "No Lunmaps\n");
		goto fail;
	}

	lun_map_list->lun_maps = ocs_malloc(NULL, 
			lun_map_list->lun_maps_count * sizeof(struct spdk_fc_lun_map),
			OCS_M_ZERO);
	if (!lun_map_list->lun_maps) {
		ocs_log_err(NULL, "Memory allocation failed\n");
		goto fail;
	}

	TAILQ_FOREACH(lun_map, &g_spdk_fc_lun_maps.head, tailq) {
		ocs_memcpy(&lun_map_list->lun_maps[i], lun_map, sizeof(struct spdk_fc_lun_map));
		i ++;
	}

	ocs_unlock(&g_spdk_config_lock);
	return lun_map_list;
fail:
	if (lun_map_list) {
		free(lun_map_list);
	}	
	ocs_unlock(&g_spdk_config_lock);
	return NULL;
}


bool
spdk_fc_api_add_lun_map(struct spdk_fc_lun_map *lun_map)
{
	struct spdk_fc_lun_map *tmp, *tmp1, *new_lun_map = NULL;
	struct spdk_fc_ig *ig, *ig_tmp;
	struct spdk_fc_hba_port *hba_port, *hba_port_tmp;
	char *p_lun_names[SPDK_SCSI_DEV_MAX_LUN] = { NULL };
	char name[1024];
	int i;

	ocs_lock(&g_spdk_config_lock);
	if (!lun_map) {
		ocs_log_err(NULL, "lunmap null pointer\n\n");
		goto fail;
	}

	FIND_NODE_BY_ID(g_spdk_fc_lun_maps, lun_map->id, tmp, tmp1);
	if (tmp) {
		ocs_log_err(NULL, "lun_map not found\n");
		goto fail; // Already Exists	
	}

	/* Validate igs and hba ports */
	for (i = 0; i < lun_map->num_mappings; i ++) {
		FIND_NODE_BY_ID(g_spdk_fc_igs, lun_map->ig_ids[i], ig, ig_tmp);
		if (!ig) {
			ocs_log_err(NULL, "IG(%d) not found\n", lun_map->ig_ids[i]);
			goto fail;
		}

		FIND_NODE_BY_ID(g_spdk_fc_hba_ports, lun_map->hba_port_ids[i],
				hba_port, hba_port_tmp);
		if (!hba_port) {
			ocs_log_err(NULL, "HBAPort(%d) not found\n", 
					lun_map->hba_port_ids[i]);
			goto fail;
		}
	}

	for (i = 0; i < lun_map->num_luns; i ++) {
		p_lun_names[i] = lun_map->lun_names[i];
	}

	new_lun_map = ocs_malloc(NULL, sizeof(struct spdk_fc_lun_map), OCS_M_ZERO);
	if (!new_lun_map) {
		ocs_log_err(NULL, "Memory allocation failed\n");
		goto fail;
	}

	ocs_memcpy(new_lun_map, lun_map, sizeof(struct spdk_fc_lun_map));
	snprintf(name, sizeof(name), "LunMapping%d", lun_map->id);

	new_lun_map->scsi_dev = spdk_scsi_dev_construct(name, (const char **)p_lun_names,
			new_lun_map->lun_ids, new_lun_map->num_luns,
			SPDK_SPC_PROTOCOL_IDENTIFIER_FC, NULL, NULL);	
	if (!new_lun_map->scsi_dev) {
		ocs_log_err(NULL, "SCSI dev creation failed\n");
		goto fail;
	}

	TAILQ_INSERT_TAIL(&g_spdk_fc_lun_maps.head, new_lun_map, tailq);
	
	ocs_unlock(&g_spdk_config_lock);
	return true;
fail:
	if (new_lun_map) {
		free(new_lun_map);
	}
	ocs_unlock(&g_spdk_config_lock);
	return false;
}

bool
spdk_fc_api_add_lun_to_lun_map(struct spdk_fc_api_lun_info *lun,
		spdk_lunmap_status cb)
{
	struct spdk_fc_lun_map *lun_map = NULL, *tmp;
	struct spdk_event *event;
	struct spdk_fc_api_lun_info *copy;
	bool rc = false;

	ocs_lock(&g_spdk_config_lock);

	if (!lun) {
		ocs_log_err(NULL, "Invalid lun_info\n");
		goto done;
	}

	if (!cb) {
		ocs_log_err(NULL, "Invalid callback\n");
		goto done;
	}

	FIND_NODE_BY_ID(g_spdk_fc_lun_maps, lun->lun_map_id, lun_map, tmp);
	if (!lun_map) {
		ocs_log_err(NULL, "Invalid lun_map\n");
		goto done; // Invalid
	}

	copy = ocs_malloc(NULL, sizeof(struct spdk_fc_api_lun_info), OCS_M_ZERO);
	if (!copy) {
		ocs_log_err(NULL, "Memory alloc failure\n");
		goto done;
	}
	ocs_memcpy(copy, lun, sizeof(struct spdk_fc_api_lun_info));

	/* Schedule this. */
	spdk_fc_cf_get_io_channel(lun_map->scsi_dev);

	event = spdk_event_allocate(lun_map->core_id,
			spdk_fc_cf_add_lun_to_lun_map,
			copy, cb);
	spdk_event_call(event);

	rc = true;	
done:
	ocs_unlock(&g_spdk_config_lock);
	return rc;
}

bool
spdk_fc_api_del_lun_from_lun_map(struct spdk_fc_api_lun_info *lun,
		spdk_lunmap_status cb)
{
	struct spdk_fc_lun_map *lun_map = NULL, *tmp;
	struct spdk_event *event;
	struct spdk_fc_api_lun_info *copy;
	bool rc = false;

	ocs_lock(&g_spdk_config_lock);

	FIND_NODE_BY_ID(g_spdk_fc_lun_maps, lun->lun_map_id, lun_map, tmp);
	if (!lun_map) {
		ocs_log_err(NULL, "Invalid lun_map\n");
		goto done; // Invalid
	}

	copy = ocs_malloc(NULL, sizeof(struct spdk_fc_api_lun_info), OCS_M_ZERO);
	if (!copy) {
		ocs_log_err(NULL, "Memory alloc failure\n");
		goto done;
	}
	ocs_memcpy(copy, lun, sizeof(struct spdk_fc_api_lun_info));

	/* Schedule this. */
	spdk_fc_cf_get_io_channel(lun_map->scsi_dev);

	event = spdk_event_allocate(lun_map->core_id,
			spdk_fc_cf_delete_lun_from_lun_map,
			copy, cb);
	spdk_event_call(event);
	rc = true;	
done:
	ocs_unlock(&g_spdk_config_lock);
	return rc;
}

bool
spdk_fc_api_del_lun_map(int lun_map_id)
{
	struct spdk_fc_lun_map *lun_map = NULL, *tmp;

	ocs_lock(&g_spdk_config_lock);

	FIND_NODE_BY_ID(g_spdk_fc_lun_maps, lun_map_id, lun_map, tmp);
	if (!lun_map) {
		ocs_log_err(NULL, "Invalid lun_map\n");
		goto fail; // Invalid
	}

	spdk_fc_cf_delete_lun_map(lun_map_id);
	spdk_scsi_dev_free_io_channels(lun_map->scsi_dev);
	spdk_scsi_dev_destruct(lun_map->scsi_dev, NULL, NULL);
	
	TAILQ_REMOVE(&g_spdk_fc_lun_maps.head, lun_map, tailq);
	free(lun_map);	
	
	ocs_unlock(&g_spdk_config_lock);

	return true;
fail:
	ocs_unlock(&g_spdk_config_lock);
	return false;
}

bool
spdk_fc_unplug_device(struct spdk_pci_addr *pci_addr)
{
	return ocsu_device_remove(pci_addr);
}

bool
spdk_fc_plug_device(struct spdk_pci_addr *pci_addr)
{
	return ocsu_device_add(pci_addr);
}


static void
ga_next_port_cb(ocs_node_t *node, ocs_node_cb_t *cbdata, void *arg)
{
	ocs_t *ocs = node->ocs;
	spdk_ga_next_cb cb  = arg;
	struct spdk_ga_next_acc *acc = NULL;
	fcct_iu_header_t *hdr = cbdata->els->els_rsp.virt;
	int status = -1;
	
	/* check status and if there is any residual */
	if (cbdata->status || ocs_be16toh(hdr->max_residual_size)) {
		ocs_log_err(ocs, "Failed. Status = %d, Residual = %d\n", 
			cbdata->status, ocs_be16toh(hdr->max_residual_size));
		goto done;
	}

	acc = ocs_malloc(node->ocs, sizeof(struct spdk_ga_next_acc), OCS_M_ZERO);;
	if (!acc) {
		ocs_log_err(ocs, "memory allocation failed.\n");
		goto done;
	}

	status = cbdata->status;
	ocs_memcpy(acc, ((char *)hdr + sizeof(fcct_iu_header_t)),
		sizeof(struct spdk_ga_next_acc));
done:
	cb(status, acc);
}


int
ocs_spdk_get_next_port(int targetnode_index, uint32_t port_id, spdk_ga_next_cb cb)
{
	ocs_t *ocs;
	ocs_node_t *node  = NULL;

	ocs = ocs_get_instance(targetnode_index);
	if (!ocs) {
		ocs_log_err(ocs, "Target instance not found.\n");
		goto fail;
	}

	if (ocs->domain && ocs->domain->sport) {
		node  = ocs_node_find(ocs->domain->sport, FC_ADDR_NAMESERVER);
		if (node) {
			ocs_ns_send_ganxt(node, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
					OCS_FC_ELS_DEFAULT_RETRIES, ga_next_port_cb, cb, port_id);
			ocs_node_transition(node, __ocs_ns_ganxt_wait_rsp, NULL);

			return 0;
		}
	} else {
		ocs_log_err(ocs, "Port down.\n");
		goto fail;
	}
fail:
	ocs_log_err(ocs, "ocs_spdk_get_next_port failed.\n");
	return -1;
}

