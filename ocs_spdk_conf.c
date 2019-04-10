/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2018 Broadcom.  All Rights Reserved.
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

#include "ocs_spdk_conf.h"
#include "spdk/log.h"
#include "spdk/scsi.h"
#include "spdk/scsi_spec.h"
#include "spdk/conf.h"
#include "spdk/env.h"
#include "ocs_node.h"
#include "ocs_device.h"
#include "ocs_els.h"
#include "ocs_tgt_api.h"
#include "scsi_internal.h"

struct spdk_fc_igs_list g_spdk_fc_igs;
struct spdk_fc_lun_map_list g_spdk_fc_lun_maps;
struct spdk_fc_hba_ports_list g_spdk_fc_hba_ports;
ocs_lock_t g_spdk_config_lock;

int
spdk_fc_cf_init_hba_ports(void)
{
	struct spdk_fc_hba_port *hba_port;
	struct spdk_conf_section *sp;
	const char *val = NULL;
	uint64_t wwn;

	TAILQ_INIT(&g_spdk_fc_hba_ports.head);
	ocs_lock_init(NULL, &g_spdk_config_lock, "SPDK FC Config Lock");

	sp = spdk_conf_first_section(NULL);
	while (sp != NULL) {
		if (spdk_conf_section_match_prefix(sp, "HBAPort")) {
			hba_port = ocs_malloc(NULL, sizeof(struct spdk_fc_hba_port), OCS_M_ZERO);
			if (!hba_port) {
				SPDK_ERRLOG("hba_port malloc failed.\n");
				goto error;
			}

			hba_port->id = spdk_conf_section_get_num(sp);
			
			val = spdk_conf_section_get_val(sp, "Initiator");
			if (val == NULL) {
				hba_port->initiator = false;
			} else if (strcasecmp(val, "Yes") == 0) {
				hba_port->initiator = true;
			} else if (strcasecmp(val, "No") == 0) {
				hba_port->initiator = false;
			} else {
				SPDK_ERRLOG("hba_port invalid initiator val.\n");
				goto error;
			}

			val = spdk_conf_section_get_val(sp, "Target");
			if (val == NULL) {
				hba_port->target = true;
			} else if (strcasecmp(val, "Yes") == 0) {
				hba_port->target = true;
			} else if (strcasecmp(val, "No") == 0) {
				hba_port->target = false;
			} else {
				SPDK_ERRLOG("hba_port invalid target val.\n");
				goto error;
			}

			hba_port->mrqs = spdk_conf_section_get_intval(sp, "MRQ");
			if (hba_port->mrqs < 1) {
				// Default use 1.
				hba_port->mrqs = 1;
			}

			if (hba_port->mrqs > (int)ocs_get_num_cpus()) {
				SPDK_ERRLOG("MRQ count cant be greater than number of CPUs.\n");
				goto error;
			}

			/* Check if WWNN is provided. */
			val = spdk_conf_section_get_val(sp, "WWNN");
			if (val) {
				if (parse_wwn((char *)val, &wwn) != 0) {
					SPDK_ERRLOG("hba_port parse WWNN(%s) failed.\n", val);
					goto error;
				}
				ocs_snprintf(hba_port->wwnn, MAX_WWN_NAME_LEN, "%s", val);	
			}
				
			/* Check if WWPN is provided. */
			val = spdk_conf_section_get_val(sp, "WWPN");
			if (val) {
				if (parse_wwn((char *)val, &wwn) != 0) {
					SPDK_ERRLOG("hba_port parse WWPN(%s) failed.\n", val);
					goto error;
				}
				ocs_snprintf(hba_port->wwpn, MAX_WWN_NAME_LEN, "%s", val);	
			}

			TAILQ_INSERT_TAIL(&g_spdk_fc_hba_ports.head, hba_port, tailq);
		}
		sp = spdk_conf_next_section(sp);	
	}

	return 0;
error:
	if (hba_port) {
		free(hba_port);
	}
	return -1;
}


int
spdk_fc_cf_init_igs(void)
{
	struct spdk_fc_ig *ig;
	struct spdk_conf_section *sp;
	const char *val = NULL;
	uint64_t wwn;
	int i;

	TAILQ_INIT(&g_spdk_fc_igs.head);

	/* Parse config options here */
	sp = spdk_conf_first_section(NULL);
	while (sp != NULL) {
		if (spdk_conf_section_match_prefix(sp, "InitiatorGroup")) {
			ig = ocs_malloc(NULL, sizeof(struct spdk_fc_ig), OCS_M_ZERO);	
			if (!ig) {
				SPDK_ERRLOG("IG malloc failed.\n");
				goto error;
			}

			ig->id = spdk_conf_section_get_num(sp);

			/* Get all initiator names */
			for (i = 0; ; i++) {
				val = spdk_conf_section_get_nval(sp, "WWN", i);
				if (val == NULL) {
					break; // No more entries
				}

				if (i >= MAX_INITIATORS_IN_IG) {
					SPDK_ERRLOG("IG Group Max error.\n");
					goto error;
				}

				if (ocs_strncmp(val, "ALL", 3) != 0) {
					if (parse_wwn((char *)val, &wwn) != 0) {
						SPDK_ERRLOG("IG parse WWN(%s) failed.\n", val);
						goto error;
					}
				}
				ocs_snprintf(&ig->initiators[i][0], MAX_WWN_NAME_LEN, "%s", val);	

				ig->num_initiators ++;
			}

			TAILQ_INSERT_TAIL(&g_spdk_fc_igs.head, ig, tailq);
		}
		sp = spdk_conf_next_section(sp);	
	}

	return 0;
error:
	if (ig) {
		free(ig);
	}
	return -1;
}

int
spdk_fc_cf_init_lun_maps(void)
{
	struct spdk_fc_lun_map *lun_map;
	char *p_lun_names[SPDK_SCSI_DEV_MAX_LUN] = { NULL };
	struct spdk_conf_section *sp;
	const char *val = NULL;
	char buf[1024];
	int rc = -1, i;

	TAILQ_INIT(&g_spdk_fc_lun_maps.head);

	/* Parse config options here */
	sp = spdk_conf_first_section(NULL);
	while (sp != NULL) {
		if (spdk_conf_section_match_prefix(sp, "LunMapping")) {
			lun_map = ocs_malloc(NULL, sizeof(struct spdk_fc_lun_map), OCS_M_ZERO);	
			if (!lun_map) {
				SPDK_ERRLOG("Lunmap malloc failed.\n");
				goto error;
			}

			lun_map->id = spdk_conf_section_get_num(sp);

			/* Get all LUN information */
			for (i = 0; i < SPDK_SCSI_DEV_MAX_LUN; i++) {
				snprintf(buf, sizeof(buf), "LUN%d", i);
				val = spdk_conf_section_get_val(sp, buf);
				if (val == NULL) {
					continue; // This is not configured.
				}

				snprintf(lun_map->lun_names[lun_map->num_luns], SPDK_SCSI_LUN_MAX_NAME_LENGTH, "%s", val);
				p_lun_names[lun_map->num_luns] = lun_map->lun_names[lun_map->num_luns]; 
				lun_map->lun_ids[lun_map->num_luns] = i;
				lun_map->num_luns ++;	
			}

			if (!lun_map->num_luns) {
				SPDK_ERRLOG("No luns Configured for LunMapping%d.\n", lun_map->id);
				goto error;
			}

			/* Asuuming IG groups And HBA ports are already initialised. */
			for (i = 0; ; i++) {
				struct spdk_fc_ig *ig = NULL, *tmp;
				struct spdk_fc_hba_port *hba_port = NULL, *tmp1;
				int id = 0;

				val = spdk_conf_section_get_nmval(sp, "Mapping", i, 1);
				if (val == NULL) {
					break; // no more entries.
				}

				SPDK_GET_SECTION_ID(val, id);
				lun_map->ig_ids[lun_map->num_mappings] = id;
				FIND_NODE_BY_ID(g_spdk_fc_igs, id, ig, tmp);
				if (!ig) {
					SPDK_ERRLOG("InitiatorGroup not found.\n");
					goto error;
				}
				
				val = spdk_conf_section_get_nmval(sp, "Mapping", i, 0);
				if (val == NULL) {
					goto error; //Should not happen.
				}

				SPDK_GET_SECTION_ID(val, id);
				lun_map->hba_port_ids[lun_map->num_mappings] = id;
				FIND_NODE_BY_ID(g_spdk_fc_hba_ports, id, hba_port, tmp1);
				if (!hba_port) {
					SPDK_ERRLOG("HBAPort not found.\n");
					goto error;
				}

				lun_map->num_mappings ++;
			}
		
			/* Create a SCSI device */
			lun_map->scsi_dev = spdk_scsi_dev_construct(spdk_conf_section_get_name(sp),
					(const char **)p_lun_names, lun_map->lun_ids, lun_map->num_luns,
					SPDK_SPC_PROTOCOL_IDENTIFIER_FC, NULL, NULL);
			if (!lun_map->scsi_dev) {
				SPDK_ERRLOG("Unable to create SCSI dev for lunmapping%d.\n", lun_map->id);
				goto error;
			}
			lun_map->scsi_dev_thread = spdk_get_thread();
			lun_map->io_ch_thread = NULL;
			lun_map->core_id = -1;
	
			TAILQ_INSERT_TAIL(&g_spdk_fc_lun_maps.head, lun_map, tailq);
		}
		sp = spdk_conf_next_section(sp);	
	}

	return 0;
error:
	if (lun_map) {
		free(lun_map);
	}
	return rc;
}

static void
spdk_fc_cf_cleanup_scsidevs_done(void *ctx)
{
	struct spdk_fc_lun_map *tmp_lun_map = NULL, *lun_map = NULL;
	struct spdk_fc_ig *tmp_ig = NULL, *ig = NULL;
	struct spdk_fc_hba_port *tmp_hba_port = NULL, *hba_port = NULL;

	/* Free the conf resources */
	TAILQ_FOREACH_SAFE(lun_map, &g_spdk_fc_lun_maps.head, tailq, tmp_lun_map) {
		free(lun_map);
	}
	TAILQ_FOREACH_SAFE(ig, &g_spdk_fc_igs.head, tailq, tmp_ig) {
		free(ig);
	}
	TAILQ_FOREACH_SAFE(hba_port, &g_spdk_fc_hba_ports.head, tailq, tmp_hba_port) {
		free(hba_port);
	}
	ocs_lock_free(&g_spdk_config_lock);
}

static void
spdk_fc_cf_thread_cleanup_scsidevs(void *ctx)
{
	struct spdk_fc_lun_map *lun_map = NULL;

	TAILQ_FOREACH(lun_map, &g_spdk_fc_lun_maps.head, tailq) {
		if (lun_map->scsi_dev_thread == spdk_get_thread() && lun_map->scsi_dev) {
			spdk_scsi_dev_destruct(lun_map->scsi_dev);
			lun_map->scsi_dev = NULL;
		}
	}
}

static void
spdk_fc_cf_cleanup_io_channels_done(void *ctx)
{
	spdk_for_each_thread(spdk_fc_cf_thread_cleanup_scsidevs,
			     NULL, spdk_fc_cf_cleanup_scsidevs_done);
}

static void
spdk_fc_cf_thread_cleanup_io_channels(void *ctx)
{
	struct spdk_fc_lun_map *lun_map = NULL;

	TAILQ_FOREACH(lun_map, &g_spdk_fc_lun_maps.head, tailq) {
		if (lun_map->io_ch_thread == spdk_get_thread()) {
			spdk_scsi_dev_free_io_channels(lun_map->scsi_dev);
		}
	}
}

void
spdk_fc_cf_cleanup_cfg(void)
{
	/* Cleanup io_channels, scsidev and then resources in order. */
	spdk_for_each_thread(spdk_fc_cf_thread_cleanup_io_channels,
			     NULL, spdk_fc_cf_cleanup_io_channels_done);
}

void
ocs_sport_logout_all_initiators(ocs_sport_t *sport)
{
	ocs_node_t *node = NULL, *next_node;
	
	ocs_sport_lock(sport);
	ocs_list_foreach_safe(&sport->node_list, node, next_node) {
		if (!node->init) {
			continue;	
		}
		// Logout node.
		if (ocs_send_logo(node, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
					0, NULL, NULL) == NULL) {
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		} else {
			ocs_node_transition(node, __ocs_d_wait_logo_rsp, NULL);
		}	
	}
	ocs_sport_unlock(sport);
}

void
ocs_sport_logout_initiator(ocs_sport_t *sport, uint64_t wwn)
{
	ocs_node_t *node = NULL;
	
	node = ocs_node_find_wwnn(sport, wwn);
	if (!node) {
		return;
	}

	if (!node->init) {
		return;	
	}

	if (ocs_send_logo(node, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
				0, NULL, NULL) == NULL) {
		ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
	} else {
		ocs_node_transition(node, __ocs_d_wait_logo_rsp, NULL);
	}	
}


// IMPORTANT.
// Below calls should be called with config lock acquired.

struct spdk_scsi_dev *
spdk_fc_cf_get_initiator_scsidev(int hba_port_id, uint64_t wwn)
{
	struct spdk_fc_lun_map *lun_map = NULL;
	uint8_t *pwwn;
	int i, j;
	char wwn_name[MAX_WWN_NAME_LEN] = { 0 };

	/* Convert to WWN to name */
	pwwn = (uint8_t *) &wwn;
	ocs_snprintf(wwn_name, MAX_WWN_NAME_LEN,
			"%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
			pwwn[7],
			pwwn[6],
			pwwn[5],
			pwwn[4],
			pwwn[3],
			pwwn[2],
			pwwn[1],
			pwwn[0]);

	TAILQ_FOREACH(lun_map, &g_spdk_fc_lun_maps.head, tailq) {
		for (i = 0; i < lun_map->num_mappings; i ++) {
			if (lun_map->hba_port_ids[i] == hba_port_id) {
				struct spdk_fc_ig *ig = NULL, *tmp;

				FIND_NODE_BY_ID(g_spdk_fc_igs, lun_map->ig_ids[i], ig, tmp);
				if (!ig) {
					continue;
				} 
				
				/* Now check is this wwn is part of IG */
				for (j = 0; j < ig->num_initiators; j ++) {
					if (((ocs_strncmp(ig->initiators[j], "ALL", 3) == 0)) ||
							(strncasecmp(ig->initiators[j],
								wwn_name, MAX_WWN_NAME_LEN) == 0)) {
						return lun_map->scsi_dev;
					} 
				}
			}
		}
	}

	return NULL;
}

int
spdk_fc_cf_add_scsidev_port(struct spdk_fc_hba_port *hba_port, uint64_t tgt_id)
{
	struct spdk_fc_lun_map *lun_map = NULL;
	int i;
	bool created = false;

	if (!hba_port) {
		return -1;
	}

	TAILQ_FOREACH(lun_map, &g_spdk_fc_lun_maps.head, tailq) {
		for (i = 0; i < lun_map->num_mappings; i ++) {
			if (lun_map->hba_port_ids[i] == hba_port->id) {
				if (!spdk_scsi_dev_find_port_by_id(lun_map->scsi_dev, tgt_id)) {
					spdk_scsi_dev_add_port(lun_map->scsi_dev, 
							tgt_id, hba_port->wwpn);
				}
				created = true;
			}
		}
	}

	if (created)
		return 0;
	return -1;
}

int
spdk_fc_cf_get_io_channel(struct spdk_scsi_dev *scsi_dev)
{
	struct spdk_fc_lun_map *lun_map = NULL;

	if (!scsi_dev) {
		return -1;
	}

	TAILQ_FOREACH(lun_map, &g_spdk_fc_lun_maps.head, tailq) {
		if (lun_map->scsi_dev == scsi_dev) {
			if (lun_map->core_id < 0) {
				// Assign current lcore id.
				if (spdk_scsi_dev_allocate_io_channels(scsi_dev) != 0) {
					return -1;
				}
				lun_map->core_id = spdk_env_get_current_core();
				lun_map->io_ch_thread = spdk_get_thread();
			}
			return lun_map->core_id;
		}
	}

	return -1;
}

void
spdk_fc_cf_delete_ig_initiator(int ig_id, char *wwn_name)
{
	struct spdk_fc_lun_map *lun_map = NULL;
	int i;
	uint64_t wwn;

	parse_wwn(wwn_name, &wwn);

	TAILQ_FOREACH(lun_map, &g_spdk_fc_lun_maps.head, tailq) {
		for (i = 0; i < lun_map->num_mappings; i ++) {
			if (lun_map->ig_ids[i] == ig_id) {
				struct spdk_fc_hba_port *hba_port = NULL, *tmp;
				ocs_t *ocs  	 = NULL;

				FIND_NODE_BY_ID(g_spdk_fc_hba_ports, lun_map->hba_port_ids[i],
						 hba_port, tmp);
				if (!hba_port) {
					continue;
				} 
			
				/* Shutdown node */
				ocs = ocs_get_instance(hba_port->id);
				if (ocs && ocs->domain && ocs->domain->sport) {

					if (!ocs_strncmp(wwn_name, "ALL", 3)) {
						ocs_sport_logout_all_initiators(ocs->domain->sport);
					} else {
						ocs_sport_logout_initiator(ocs->domain->sport, wwn);
					}
				}
			}
		}
	}
}


void
spdk_fc_cf_delete_ig(int ig_id)
{
	struct spdk_fc_lun_map *lun_map = NULL;
	int i, j;

	TAILQ_FOREACH(lun_map, &g_spdk_fc_lun_maps.head, tailq) {
		for (i = 0; i < lun_map->num_mappings; i ++) {
			if (lun_map->ig_ids[i] == ig_id) {
				struct spdk_fc_hba_port *hba_port = NULL, *tmp;
				struct spdk_fc_ig *ig = NULL, *tmp1;
				ocs_t *ocs  	 = NULL;

				FIND_NODE_BY_ID(g_spdk_fc_igs, lun_map->ig_ids[i], ig, tmp1);
				if (!ig) {
					continue;
				} 

				FIND_NODE_BY_ID(g_spdk_fc_hba_ports, lun_map->hba_port_ids[i],
						 hba_port, tmp);
				if (!hba_port) {
					continue;
				}

				ocs = ocs_get_instance(hba_port->id);
				if (ocs && ocs->domain && ocs->domain->sport) {
					for (j = 0; j < ig->num_initiators; j ++) {
						uint64_t wwn;

						if (!ocs_strncmp(ig->initiators[j], "ALL", 3)) {
							ocs_sport_logout_all_initiators(ocs->domain->sport);
							break;
						}

						if (parse_wwn(ig->initiators[j], &wwn) != 0) {
							SPDK_ERRLOG("Parse WWN(%s) failed.\n",
									ig->initiators[j]);
							continue;
						}

						ocs_sport_logout_initiator(ocs->domain->sport, wwn);
					}
				}
			}
		}
	}
}

void
spdk_fc_cf_delete_lun_map(int lun_map_id)
{
	struct spdk_fc_lun_map *lun_map = NULL, *tmp;
	int i, j;

	FIND_NODE_BY_ID(g_spdk_fc_lun_maps, lun_map_id, lun_map, tmp);
	if (!lun_map) {
		return;
	}

	for (i = 0; i < lun_map->num_mappings; i ++) {
		struct spdk_fc_hba_port *hba_port = NULL, *tmp1;
		struct spdk_fc_ig *ig = NULL, *tmp2;
		ocs_t *ocs = NULL;

		FIND_NODE_BY_ID(g_spdk_fc_igs, lun_map->ig_ids[i], ig, tmp2);
		if (!ig) {
			continue;
		} 

		FIND_NODE_BY_ID(g_spdk_fc_hba_ports, lun_map->hba_port_ids[i],
				hba_port, tmp1);
		if (!hba_port) {
			continue;
		}

		ocs = ocs_get_instance(hba_port->id);
		if (ocs && ocs->domain && ocs->domain->sport) {
			for (j = 0; j < ig->num_initiators; j ++) {
				uint64_t wwn;

				if (!ocs_strncmp(ig->initiators[j], "ALL", 3)) {
					ocs_sport_logout_all_initiators(ocs->domain->sport);
					break;
				}

				if (parse_wwn(ig->initiators[j], &wwn) != 0) {
					SPDK_ERRLOG("Parse WWN(%s) failed.\n", 
							ig->initiators[j]);
					continue;
				}

				ocs_sport_logout_initiator(ocs->domain->sport, wwn);
			}
		}
	}
}

void
ocs_scsi_dev_set_nodes_dirty(struct spdk_scsi_dev *dev)
{
	int i;
	ocs_t *ocs;
	ocs_node_t *node;
	ocs_sport_t *sport;

	for_each_ocs(i, ocs) {
		if (ocs && ocs->domain && ocs->domain->sport) {
			sport = ocs->domain->sport;

			ocs_sport_lock(sport);

			ocs_list_foreach(&sport->node_list, node) {
				if (node->tgt_node.scsi_dev == dev) {
					ocs_node_lock(node);
					node->tgt_node.dirty = true;
					ocs_node_unlock(node);
				}
			}

			ocs_sport_unlock(sport);
		}
	}
}

// This should be sheduled on the lunmap core if core is assigned.
void
spdk_fc_cf_delete_lun_from_lun_map(void *arg1, void *arg2) 
{
	struct spdk_fc_lun_map *lun_map, *tmp;
	struct spdk_fc_api_lun_info *lun = arg1;
	spdk_lunmap_status cb 		 = arg2;
	bool status = false;
	int i;

	ocs_lock(&g_spdk_config_lock);

	if (!lun || !cb) {
		ocs_log_err(NULL, "Invalid params\n");
		goto done;
	}
	
	FIND_NODE_BY_ID(g_spdk_fc_lun_maps, lun->lun_map_id, lun_map, tmp);
	if (!lun_map) {
		ocs_log_err(NULL, "lun_map not found\n");
		goto done;
	}

	for (i = 0; i < lun_map->num_luns; i ++) {
		if (lun_map->lun_ids[i] == lun->id) {
			lun_map->lun_ids[i] = -1;
			spdk_scsi_dev_delete_lun(lun_map->scsi_dev, 
					lun_map->scsi_dev->lun[lun->id]);
			break;
		}
	}

	if (i != lun_map->num_luns) {
		// Lun deleted. Fix the hole in the array.
		lun_map->num_luns --;
		for (; i < lun_map->num_luns; i ++) {
			lun_map->lun_ids[i] = lun_map->lun_ids[i + 1];
			snprintf(lun_map->lun_names[i], SPDK_SCSI_LUN_MAX_NAME_LENGTH,
					"%s", lun_map->lun_names[i + 1]);
		}
		ocs_scsi_dev_set_nodes_dirty(lun_map->scsi_dev);	
		status = true;
	} else {
		ocs_log_err(NULL, "lun not found\n");
	}

done:
	ocs_unlock(&g_spdk_config_lock);
	if (cb)
		cb(status);
	if (lun)
		free(lun);
}

void
spdk_fc_cf_add_lun_to_lun_map(void *arg1, void *arg2)
{
	struct spdk_fc_lun_map *lun_map, *tmp;
	struct spdk_fc_api_lun_info *lun = arg1;
	spdk_lunmap_status cb 		 = arg2;
	struct spdk_bdev *bdev;
	struct spdk_scsi_lun *scsilun;
	bool status = false;

	ocs_lock(&g_spdk_config_lock);

	if (!lun || !cb) {
		ocs_log_err(NULL, "Invalid params\n");
		goto done;
	}
	
	FIND_NODE_BY_ID(g_spdk_fc_lun_maps, lun->lun_map_id, lun_map, tmp);
	if (!lun_map) {
		ocs_log_err(NULL, "lun_map not found\n");
		goto done;
	}

	bdev = spdk_bdev_get_by_name(lun->name);
	if (bdev == NULL) {
		SPDK_ERRLOG("bdev not found %s\n", lun->name);
		goto done;
	}

	scsilun = spdk_scsi_lun_construct(bdev, NULL, NULL);
	if (scsilun == NULL) {
		SPDK_ERRLOG("lun construct failed. %s\n", lun->name);
		goto done;
	}

	/* make sure this is not already added. */
	if (lun_map->scsi_dev->lun[lun->id] == scsilun) {
		SPDK_ERRLOG("lun %s already added.\n", lun->name);
		goto done;
	}

	/* Add lun device */
	// spdk_scsi_dev_add_lun(lun_map->scsi_dev, scsilun, lun->id); // Export this from scsi

	/* Modify lun_map with new lun info */
	snprintf(lun_map->lun_names[lun_map->num_luns], 
			SPDK_SCSI_LUN_MAX_NAME_LENGTH, "%s", lun->name);
	lun_map->lun_ids[lun_map->num_luns] = lun->id;
	lun_map->num_luns ++;
	
	/* Notify loggged initiators using this lun_map */	
	ocs_scsi_dev_set_nodes_dirty(lun_map->scsi_dev);
	status = true;

done:
	ocs_unlock(&g_spdk_config_lock);
	if (cb)
		cb(status);
	if (lun)
		free(lun);
}


