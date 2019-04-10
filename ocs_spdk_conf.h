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
#include "ocs_tgt_api.h"
#include "spdk/conf.h"

#define SPDK_FC_LARGE_RBUF_MAX_SIZE (64 * 1024)

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

struct spdk_scsi_dev *
spdk_fc_cf_get_initiator_scsidev(int hba_port_id, uint64_t wwn);

void
spdk_fc_cf_delete_ig_initiator(int ig_id, char *wwn_name);

void
spdk_fc_cf_delete_ig(int ig_id);

void
spdk_fc_cf_delete_lun_map(int lun_map_id);

int
spdk_fc_cf_add_scsidev_port(struct spdk_fc_hba_port *hba_port, uint64_t tgt_id);

int
spdk_fc_cf_get_io_channel(struct spdk_scsi_dev *scsi_dev);

void
ocs_sport_logout_all_initiators(ocs_sport_t *sport);

void
ocs_sport_logout_initiator(ocs_sport_t *sport, uint64_t wwn);

void
spdk_fc_cf_delete_lun_from_lun_map(void *arg1, void *arg2);

void
spdk_fc_cf_add_lun_to_lun_map(void *arg1, void *arg2);

void
ocs_scsi_dev_set_nodes_dirty(struct spdk_scsi_dev *dev);

// Global
extern struct spdk_fc_igs_list g_spdk_fc_igs;
extern struct spdk_fc_lun_map_list g_spdk_fc_lun_maps;
extern struct spdk_fc_hba_ports_list g_spdk_fc_hba_ports;
extern ocs_lock_t g_spdk_config_lock;

#endif
