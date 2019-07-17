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

#if !defined(__OCS_SPDK_CONF_H__)
#define __OCS_SPDK_CONF_H__

#include "ocs.h"
#include "ocs_os.h"
#include "spdk/scsi.h"
#include "spdk/conf.h"

#define SPDK_FC_LARGE_RBUF_MAX_SIZE (64 * 1024)
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


struct spdk_fc_igs_list {
	TAILQ_HEAD(, spdk_fc_ig) head;
};


struct spdk_fc_lun_map_list {
	TAILQ_HEAD(, spdk_fc_lun_map) head;
};

struct spdk_fc_hba_ports_list {
	TAILQ_HEAD(, spdk_fc_hba_port) head;
};

int spdk_fc_cf_init_hba_ports(void);
int spdk_fc_cf_init_igs(void);
int spdk_fc_cf_init_lun_maps(void);
void spdk_fc_cf_cleanup_cfg(void);

int
spdk_fc_cf_add_scsidev_port(struct spdk_fc_hba_port *hba_port, uint64_t tgt_id);

int
spdk_fc_cf_get_io_channel(struct spdk_scsi_dev *scsi_dev);

void
ocs_sport_logout_all_initiators(ocs_sport_t *sport);

void
ocs_sport_logout_initiator(ocs_sport_t *sport, uint64_t wwn);

struct spdk_scsi_dev *
spdk_fc_cf_get_initiator_scsidev(int hba_port_id, uint64_t wwn);

// Global
extern struct spdk_fc_igs_list g_spdk_fc_igs;
extern struct spdk_fc_lun_map_list g_spdk_fc_lun_maps;
extern struct spdk_fc_hba_ports_list g_spdk_fc_hba_ports;
extern ocs_lock_t g_spdk_config_lock;


#endif
