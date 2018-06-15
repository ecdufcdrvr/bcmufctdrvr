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
 * SPDK fc initiator discovery data definitions.
 */

#ifndef _SPDK_FC_INI_DI_H
#define _SPDK_FC_INI_DI_H

#include "spdk_fc_ini_bdev.h"
#include "ocs_common_shared.h"

/* event type passed in spdk_fc_ini_di_update_devs_notify()
   to indicate update node type */
typedef enum  spdk_fc_ini_port_tgt_event_type {

	INI_NODE_EVENT_TYPE_NEW_TARGET,    /* new target found */
	INI_NODE_EVENT_TYPE_TARGET_DOWN,   /* target has gone down */
	INI_NODE_EVENT_TYPE_TARGET_UP,     /* target has come back up */
	INI_NODE_EVENT_TYPE_TGT_LUNS,      /* need to update the LUNs on target */
	INI_NODE_EVENT_TYPE_PORT_DOWN,     /* port has gone down */

} SPDK_FC_INI_PORT_TGT_EVENT_TYPE;

/* Notification function called from OCS initiator to block layer that
   device updates are needed */
/* For all but INI_NODE_EVENT_TYPE_PORT_DOWN, update_node is a ocs_node_t*
   (target ptr) */
/* For INI_NODE_EVENT_TYPE_PORT_DOWN, update_node is ocs_t* (ini port ptr) */
void spdk_fc_ini_di_update_devs_notify(SPDK_FC_INI_PORT_TGT_EVENT_TYPE type,
	void *update_node);

/* called when shutting down (TBD) */
int spdk_fc_ini_di_shutdown(void);

#define SPDK_FC_INI_DI_MAX_LUNS                 256  /* max. LUNs per target */
/* maximum initiator ports */
#define SPDK_FC_INI_DI_MAX_PORTS                MAX_OCS_DEVICES
#define SPDK_FC_INI_DI_SCSI_BUFFER_SIZE         4096
/* number of SCSI cmd buffers to allocate */
#define SPDK_FC_INI_DI_NUM_SCSI_BUFFERS         SPDK_FC_INI_DI_MAX_PORTS 
/* command recovery timer interval (msecs) */
#define SPDK_FC_INI_DI_RECOVERY_TIMER_INTERVAL  5000  

#define SPDK_FC_INI_DI_LIST(name, type) TAILQ_HEAD(name, type) name; \
	pthread_mutex_t lock_##name
#define SPDK_FC_INI_DI_LIST_INIT(o, l) \
	TAILQ_INIT(&(o->l)); \
	pthread_mutex_init(&(o->lock_##l), NULL)
#define SPDK_FC_INI_DI_LIST_EMPTY(o, l) TAILQ_EMPTY(&(o->l))
#define SPDK_FC_INI_DI_LIST_LOCK(o, l) pthread_mutex_lock(&(o->lock_##l))
#define SPDK_FC_INI_DI_LIST_UNLOCK(o,l) pthread_mutex_unlock(&(o->lock_##l))
#define SPDK_FC_INI_DI_LIST_INSERT(o, l, e, ln) \
	SPDK_FC_INI_DI_LIST_LOCK(o, l); \
	TAILQ_INSERT_TAIL(&(o->l), (e), ln); \
	SPDK_FC_INI_DI_LIST_UNLOCK(o, l)
#define SPDK_FC_INI_DI_LIST_NL_INSERT(o, l, e, ln) \
	TAILQ_INSERT_TAIL(&(o->l), (e), ln) 
#define SPDK_FC_INI_DI_LIST_REMOVE(o, l, e, ln) \
	SPDK_FC_INI_DI_LIST_LOCK(o, l); \
	TAILQ_REMOVE(&(o->l), (e), ln); \
	SPDK_FC_INI_DI_LIST_UNLOCK(o, l)
#define SPDK_FC_INI_DI_LIST_NL_REMOVE(o, l, e, ln) \
	TAILQ_REMOVE(&(o->l), e, ln)
#define SPDK_FC_INI_DI_LIST_FIRST(o, l) TAILQ_FIRST(&(o->l))
#define SPDK_FC_INI_DI_LIST_FOREACH(e, o, l, ln) \
	SPDK_FC_INI_DI_LIST_LOCK(o, l); \
	TAILQ_FOREACH(e, &(o->l), ln)
#define SPDK_FC_INI_DI_LIST_NL_FOREACH(e, o, l, ln) TAILQ_FOREACH(e, &(o->l), ln)

/* discovered block device */
typedef struct _di_bdev {
	struct spdk_fc_ini_bdev bdev;  /* the FC initiator block device */
	bool new_lun;                /* true when bdev is first added - for cb */
	TAILQ_ENTRY(_di_bdev) link;
} spdk_fc_ini_di_bdev_t;

/* FC initiator port */
typedef struct _di_port {
	ocs_t *ini_ocs_port;           /* initiator port (in FC driver) */
	uint64_t wwpn;                 /* wwpn of port */
	/* list of tgts on port */
	SPDK_FC_INI_DI_LIST(target_list, _di_target); 
	/* list of items (the port or targets) that need processing done on them */
	SPDK_FC_INI_DI_LIST(discovery_work_q, _di_wqe);
	struct rte_mempool *mp;      /* allocated DPDK memory pool for SCSI cmds */
} spdk_fc_ini_di_port_t;

/* FC initiator target */
typedef struct _di_target {
	ocs_node_t *target_node;       /* initiator target node (in FC driver) */
	uint64_t wwpn;                 /* wwpn of target */
	struct _di_port *ini_port;     /* back ptr to ini port for target */
	/* list of _di_bdev's belonging to target */
	SPDK_FC_INI_DI_LIST(bdev_list, _di_bdev);
	TAILQ_ENTRY(_di_target) link;
} spdk_fc_ini_di_target_t;

/* discovery work queue entry */
typedef struct _di_wqe {
	struct _di_port *ini_port;     /* initiator port being procossed */
	struct _di_target *ini_target; /* initiator target being processed */
	SPDK_FC_INI_PORT_TGT_EVENT_TYPE event_type;
	bool scsi_retry_in_progress;    /* true=peforming a SCSI command retry */
	struct spdk_fc_ini_di_mobj *cmd_buf; /* SCSI cmd buf ptr */
	/* SCSI REPORT LUNS buffer (256 LUNs max) */
	uint8_t report_luns_buf[(SPDK_FC_INI_DI_MAX_LUNS + 1)* 8];
	uint32_t num_reported_luns;  /* number of reported luns */
	/* index into report_luns_buf for current LUN being processed */
	uint32_t curr_lun_index;
	TAILQ_ENTRY(_di_wqe) link;
} spdk_fc_ini_di_wqe_t;

/* structure for storing info on DPDK buffers used in SCSI cmds. */
typedef struct spdk_fc_ini_di_mobj {
	struct rte_mempool *mp;
	void *buf;
	size_t len;
	uint64_t phys_addr;
	uint64_t reserved; /* do not use */
} spdk_fc_ini_di_mobj_t;

typedef struct spdk_fc_ini_di_cmd_recovery_q {
	SPDK_FC_INI_DI_LIST(cmd_recovery_work_q, _di_wqe);
} spdk_fc_ini_di_cmd_recovery_q_t;

#endif
