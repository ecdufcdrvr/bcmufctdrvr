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
 */
#if !defined(__SPDK_FC_TGT_API_H__)
#define __SPDK_FC_TGT_API_H__

#include "spdk/scsi.h"


#define MAX_GRP_NAME_LEN	256
#define MAX_WWN_NAME_LEN	128
#define MAX_INITIATORS_IN_IG	256
#define MAX_MAPPINGS_IN_LUNMAP	64

#define TAILQ_COUNT(count, head, field, var) do {	\
	count = 0;					\
	for (var = TAILQ_FIRST(head); var;		\
			 var = TAILQ_NEXT(var, field))	\
	{						\
		count ++;				\
	}						\
} while (0)

#define FIND_NODE_BY_ID(list, idx, node, tvar) do {	\
	node = NULL;					\
							\
	TAILQ_FOREACH(tvar, &list.head, tailq) {	\
		if (tvar->id == idx) {			\
			node = tvar;			\
			break;				\
		}					\
	}						\
} while (0)


#define SPDK_GET_SECTION_ID(name, id) do {			\
	const char *p; 						\
	for (p = name; *p != '\0' && !isdigit((int) *p); p++)	\
		;					\
	if (*p != '\0') {					\
		id = (int)strtol(p, NULL, 10);			\
	} else {						\
		id = 0;						\
	}							\
} while (0)


#define CLEANUP_NODE_LIST(list, tvar) do {		\
	TAILQ_FOREACH(tvar, &list.head, tailq) { 	\
		free(tvar);				\
	}						\
} while (0)

struct spdk_fc_ig {
	int  id;

	/* List of initiators in this group */
	int  num_initiators;
	char initiators[MAX_INITIATORS_IN_IG][MAX_WWN_NAME_LEN];

	/* For internal use only. */
	TAILQ_ENTRY(spdk_fc_ig) tailq;
};

struct spdk_fc_hba_port {
	int  id;
	
	int mrqs;
	bool initiator;
	bool target;

	char wwnn[MAX_WWN_NAME_LEN];
	char wwpn[MAX_WWN_NAME_LEN];

	TAILQ_ENTRY(spdk_fc_hba_port) tailq;
};

#define SPDK_SCSI_LUN_MAX_NAME_LENGTH 16

struct spdk_fc_lun_map {
	int id;

	int num_luns;
	int lun_ids[SPDK_SCSI_DEV_MAX_LUN];
	char lun_names[SPDK_SCSI_DEV_MAX_LUN][SPDK_SCSI_LUN_MAX_NAME_LENGTH];

	/* Ports and groups that have access to this lunmap */
	int num_mappings;
	int ig_ids[MAX_MAPPINGS_IN_LUNMAP];
	int hba_port_ids[MAX_MAPPINGS_IN_LUNMAP];

	/* Below for internal purpose only */

	/* scsi device for this lunmap */
	struct spdk_scsi_dev *scsi_dev;
	int core_id;

	TAILQ_ENTRY(spdk_fc_lun_map) tailq;

	/* cleanup context */
	struct spdk_thread *io_ch_thread;
	struct spdk_thread *scsi_dev_thread;
};


struct spdk_fc_api_ig_list {
	struct spdk_fc_ig *igs;
	int igs_count;
};

struct spdk_fc_api_lun_map_list {
	struct spdk_fc_lun_map *lun_maps;
	int lun_maps_count;
};

struct spdk_fc_api_hba_port_list {
	struct spdk_fc_hba_port *hba_ports;
	int hba_ports_count;
};

struct spdk_fc_api_lun_info {
	int id;
	char name[SPDK_SCSI_LUN_MAX_NAME_LENGTH];
	
	int lun_map_id;
};

/* GA_NEXT CT command response structure.
 * Keep this in sync with fcct_ganxt_acc_t in "lib/fc/ocs_fcp.h"
 */
#pragma pack(1)
struct spdk_ga_next_acc {
	uint32_t port_type:8,
		 port_id:24;
	uint64_t port_name;
	uint8_t	sym_port_name_len;
	uint8_t	sym_port_name[255];
	uint64_t node_name;
	uint8_t	sym_node_name_len;
	uint8_t	sym_node_name[255];
	uint64_t ip_associator;
	uint8_t	ip_addr[16];
	uint32_t class_of_serv;
	uint8_t	protocol_types[32];
};
#pragma pack()

typedef void (*spdk_ga_next_cb)(int status, struct spdk_ga_next_acc *);




/* @brief  Get the list of igs
 * @return List of igs or NULL.
 *
 * Note:   On successfull return user is responsible to free
 *         spdk_fc_api_ig_list.igs and spdk_fc_api_ig_list
 */
struct spdk_fc_api_ig_list *
spdk_fc_api_get_igs(void);


/* @brief  Add initiator(Param0) to IG Group(Param1)
 * @Param0 Initiator name
 * @Param1 Initiator Group id.
 * @return True or False.
 */
bool
spdk_fc_api_ig_add_initiator(char *initiator, int ig_id);


/* @brief  Delete initiator(Param0) from IG Group(Param1)
 * @Param0 Initiator name
 * @Param1 Initiator Group id.
 * @return True or False.
 */
bool
spdk_fc_api_ig_del_initiator(char *initiator, int ig_id);


/* @brief  Add IG Group.
 * @Param0 Ig Group information
 * @return True or False.
 */
bool
spdk_fc_api_add_ig(struct spdk_fc_ig *ig);


/* @brief  Delete IG Group.
 * @Param0 Ig Group id
 * @return True or False.
 */
bool
spdk_fc_api_del_ig(int ig_id);


/* @brief  Get the list of lunmaps 
 * @return List of lunmaps or NULL.
 *
 * Note:   On successfull return user is responsible to free
 *         spdk_fc_api_lunmap_list.lunmaps and spdk_fc_api_lunmap_list
 */
struct spdk_fc_api_lun_map_list *
spdk_fc_api_get_lun_maps(void);


/* @brief  Add Lun Mapping.
 * @Param0 Lun Mapping information
 * @return True or False.
 */
bool
spdk_fc_api_add_lun_map(struct spdk_fc_lun_map *lun_map);


/* @brief  Delete Lun Mapping.
 * @Param0 Lun Mapping id
 * @return True or False.
 */
bool
spdk_fc_api_del_lun_map(int lun_map_id);


/* @brief  Get the list of FC HBA Port node details
 * @return List of FC HBA ports or NULL.
 *
 * Note:   On successfull return user is responsible to free
 *         spdk_fc_api_hba_port_list.ports and spdk_fc_api_hba_port_list
 */
struct spdk_fc_api_hba_port_list *
spdk_fc_api_get_hba_ports(void);

int
ocs_spdk_get_next_port(int targetnode_index, uint32_t port_id, spdk_ga_next_cb cb);


bool
spdk_fc_unplug_device(struct spdk_pci_addr *pci_addr);

bool
spdk_fc_plug_device(struct spdk_pci_addr *pci_addr);


typedef void (*spdk_lunmap_status)(bool status);

bool
spdk_fc_api_del_lun_from_lun_map(struct spdk_fc_api_lun_info *lun_info,
		spdk_lunmap_status cb);

bool
spdk_fc_api_add_lun_to_lun_map(struct spdk_fc_api_lun_info *lun_info,
		spdk_lunmap_status cb);

bool
spdk_fc_api_subsystem_init(void);

bool
spdk_fc_api_subsystem_exit(void);

#endif // __SPDK_FC_TGT_API_H__
