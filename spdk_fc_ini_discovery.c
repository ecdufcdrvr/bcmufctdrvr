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

#ifndef DEBUG
//#define DEBUG 1  /* uncomment this line to turn on trace logging */
#endif
//#define DEBUG_TEST_DI_NOTIFICATIONS 1

#include <inttypes.h>
#include <stdlib.h>
#include <unistd.h>

#include <rte_config.h>
#include <rte_common.h>
#include <rte_log.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_memzone.h>
#include <rte_tailq.h>
#include <rte_debug.h>
#include <rte_eal.h>
#include <rte_per_lcore.h>
#include <rte_launch.h>
#include <rte_atomic.h>
#include <rte_lcore.h>
#include <rte_branch_prediction.h>
#include <rte_ring.h>
#include <rte_log.h>
#include <rte_mempool.h>
#include <rte_string_fns.h>
#include <rte_cycles.h>
#include <rte_malloc.h>
#include <rte_version.h>

#include <scsi/scsi.h>
#include <spdk/scsi.h>
#include <spdk/scsi_spec.h>
#include <spdk/endian.h>
#include <spdk_internal/log.h>
#include <spdk/event.h>
#include <spdk/io_channel.h>

#include "ocs.h"
#include "ocs_os.h"
#include "ocs_fcp.h"
#include "ocs_drv_fc.h"
#include "ocs_fc_ini_errors.h"
#include "ocs_tgt_spdk.h"
#include "scsi_cmds.h"
#include "spdk_fc_ini_bdev.h"
#include "spdk_fc_ini_discovery.h"


/* global list of initiator ports from FC driver */
static spdk_fc_ini_di_port_t g_fc_ini_port_list[SPDK_FC_INI_DI_MAX_PORTS];
static uint32_t g_fc_ini_num_ports = 0;

/* callback function pointer (passed from client) for device list update */
static spdk_fc_ini_bdev_list_update_cb g_spdk_fc_ini_di_dev_list_update_cb = 0;
static bool g_spdk_fc_ini_di_running = false;

/* command error recovery */
static spdk_fc_ini_di_cmd_recovery_q_t g_spdk_fc_ini_di_cmd_recovery_q;
static struct spdk_poller *g_cmd_recovery_timer;

/* prototypes */
static void spdk_fc_ini_di_process_target_down(spdk_fc_ini_di_wqe_t *);
static void spdk_fc_ini_di_process_port_down(spdk_fc_ini_di_wqe_t *);
static void spdk_fc_ini_di_process_target_update(spdk_fc_ini_di_wqe_t *);
static void spdk_fc_ini_di_report_luns_complete(
	struct spdk_fc_ini_cmd_complete_info *);
static void spdk_fc_ini_di_process_next_lun(spdk_fc_ini_di_wqe_t *);
static void spdk_fc_ini_di_inquiry_complete(
	struct spdk_fc_ini_cmd_complete_info *);
static void spdk_fc_ini_di_process_inquiry(uint64_t, spdk_fc_ini_di_wqe_t *);
static void spdk_fc_ini_di_send_read_capacity(uint64_t, spdk_fc_ini_di_wqe_t *);
static void spdk_fc_ini_di_read_capacity_complete(
struct spdk_fc_ini_cmd_complete_info *);
static spdk_fc_ini_di_port_t* spdk_fc_ini_di_find_port_in_port_list(
	ocs_t *ocs_port);
static spdk_fc_ini_di_target_t *spdk_fc_ini_di_find_tgt_in_tgt_list(
	spdk_fc_ini_di_port_t *port, ocs_node_t *node);
static void spdk_fc_ini_di_mp_ctor(struct rte_mempool *mp,
	__attribute__((unused))void *arg, void *_m,
	__attribute__((unused))unsigned in);
static bool spdk_fc_ini_di_log_scsi_send_cmd_error(char *scsi_cmd_str, int rc,
	spdk_fc_ini_di_wqe_t *dwqe);
static bool spdk_fc_ini_di_log_scsi_cmd_resp_error(char *scsi_cmd_str,
	struct spdk_fc_ini_cmd_complete_info *cci);
static void spdk_fc_ini_di_queue_dwqe(spdk_fc_ini_di_wqe_t *dwqe);
static void spdk_fc_ini_di_queue_discovery_work_request(
	spdk_fc_ini_di_port_t *port,
	spdk_fc_ini_di_target_t *tgt,
	SPDK_FC_INI_PORT_TGT_EVENT_TYPE event_type);
static void spdk_fc_ini_di_new_target(ocs_node_t *node);
static void spdk_fc_ini_di_target_update(ocs_node_t *node,
	SPDK_FC_INI_PORT_TGT_EVENT_TYPE event_type);
static void spdk_fc_ini_di_port_down(ocs_t *port_node);
static void spdk_fc_ini_di_process_dwqe(spdk_fc_ini_di_wqe_t *dwqe);
static void spdk_fc_ini_di_process_next_dwqe(spdk_fc_ini_di_port_t *port);
static void spdk_fc_ini_di_complete_dwqe(spdk_fc_ini_di_wqe_t *dwqe);
static void spdk_fc_ini_di_start_process_dwq(void *arg1, void *arg2);
static int spdk_fc_ini_di_command_recovery_startup(void *arg);
static void spdk_fc_ini_queue_on_recovery_q(
	spdk_fc_ini_di_cmd_recovery_q_t *rq,
	spdk_fc_ini_di_wqe_t *dwqe);
void spdk_fc_ini_di_init(spdk_fc_ini_bdev_list_update_cb cb);

/* The DI_RETURN_IF_NOT_RUNNING macro is used in many discovery functions
   to check if a discovery cycle should continue */
#define DI_RETURN_IF_NOT_RUNNING if (!g_spdk_fc_ini_di_running) return
#define DI_DWQE_PORT_WWPN(dwqe) (dwqe)->ini_port->wwpn
#define DI_DWQE_TGT_WWPN(dwqe) (dwqe)->ini_target->wwpn

static inline
uint64_t spdk_fc_ini_di_port_node_wwpn(ocs_t* port)
{
	return (port->xport->req_wwpn == 0 ?
			ocs_get_wwn(&port->hal, OCS_HAL_WWN_PORT) :
			port->xport->req_wwpn);
}

static inline
uint64_t spdk_fc_ini_di_port_node_wwnn(ocs_t* port)
{
	return (port->xport->req_wwpn == 0 ?
			ocs_get_wwn(&port->hal, OCS_HAL_WWN_NODE) :
			port->xport->req_wwpn);
}

static inline
uint64_t spdk_fc_ini_di_tgt_node_wwpn(ocs_node_t *node)
{
	fc_plogi_payload_t *sp = (fc_plogi_payload_t *)node->service_params;
#ifdef DEBUG_TEST_DI_NOTIFICATIONS
	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY, 
		"display_name[0] = %d\n", node->display_name[0]);
	if (node->display_name[0] > 0 &&  node->display_name[0] <= 10) {
		uint64_t wwpn = 	
			((uint64_t)from_be32(&sp->port_name_hi) << 32ll) |
			(from_be32(&sp->port_name_lo));
		return (wwpn | ((uint64_t) node->display_name[0] << 56));
	}
	else
#endif

	return (((uint64_t)from_be32(&sp->port_name_hi) << 32ll) |
			(from_be32(&sp->port_name_lo)));
}

static inline
uint64_t spdk_fc_ini_di_tgt_node_wwnn(ocs_node_t *node)
{
	fc_plogi_payload_t *sp = (fc_plogi_payload_t *)node->service_params;
	return (((uint64_t)from_be32(&sp->node_name_hi) << 32ll) |
			(from_be32(&sp->node_name_lo)));
}

static
bool spdk_fc_ini_di_log_scsi_send_cmd_error(char *scsi_cmd_str, int rc,
	spdk_fc_ini_di_wqe_t *dwqe)
{   /* log sending a SCSI cmd. send error - return false if retrying */
	uint64_t port_wwpn = spdk_fc_ini_di_port_node_wwpn(dwqe->ini_port->ini_ocs_port);
	uint64_t tgt_wwpn = DI_DWQE_TGT_WWPN(dwqe);
	SPDK_ERRLOG(
		"Error sending SCSI '%s' to port: %lx, tgt: %lx (err %d)\n",
		scsi_cmd_str, port_wwpn, tgt_wwpn, rc);

	/* see if it's ok to start discovery task again a little while later */
	if (rc == SPDK_FC_INI_ERROR_NO_MEM 
		&& dwqe->scsi_retry_in_progress == false) {
		dwqe->scsi_retry_in_progress = true;
		spdk_fc_ini_queue_on_recovery_q(&g_spdk_fc_ini_di_cmd_recovery_q,
			dwqe);
		return false;
	}

	return true;
}

static
bool spdk_fc_ini_di_log_scsi_cmd_resp_error(char *scsi_cmd_str,
	struct spdk_fc_ini_cmd_complete_info *cci)
{   /* log a SCSI cmd. result error - return false if retrying */
	spdk_fc_ini_di_wqe_t *dwqe = (spdk_fc_ini_di_wqe_t *)cci->cb_data;
	struct sense_info sense_info = { 0 };

	SPDK_ERRLOG(
		"SCSI '%s' error (host_rslt=%d, tgt_rslt=%d) for port: %lx, tgt: %lx\n",
		scsi_cmd_str,
		cci->cmd_result.host_result, cci->cmd_result.tgt_result,
		DI_DWQE_PORT_WWPN(dwqe), DI_DWQE_TGT_WWPN(dwqe));

	if (cci->cmd_result.tgt_result == SAM_STAT_CHECK_CONDITION) {
		if (ocs_get_sense_info(cci->cmd_result.scsi_resp,
			cci->cmd_result.scsi_resp_len, &sense_info) == 0) {
			SPDK_ERRLOG("Sense data - rc=%x, sk=%x, asc=%x, ascq=%x\n",
				sense_info.response_code, sense_info.sense_key,
				sense_info.asc, sense_info.ascq);
		} else {
			SPDK_ERRLOG("Sense data not available\n");
		}
	}

	/* see if it's ok to start discovery task again a little while later */
	if ((cci->cmd_result.host_result == SPDK_FC_INI_ERROR_TIMEOUT
		 || cci->cmd_result.tgt_result == SAM_STAT_BUSY 
		 || cci->cmd_result.tgt_result == SAM_STAT_TASK_SET_FULL
		 || (cci->cmd_result.tgt_result == SAM_STAT_CHECK_CONDITION 
			 && sense_info.sense_key  == SPDK_SCSI_SENSE_UNIT_ATTENTION))
		 && dwqe->scsi_retry_in_progress == false) {
		dwqe->scsi_retry_in_progress = true;
		spdk_fc_ini_queue_on_recovery_q(&g_spdk_fc_ini_di_cmd_recovery_q,
			dwqe);
		return false;
	} else {
		dwqe->scsi_retry_in_progress = false;
	}

	return true;
}

static
uint32_t spdk_fc_ini_di_get_lcore(spdk_fc_ini_di_wqe_t *dwqe)
{
	// TODO: Determine which lcore to do discovery on
	// For now just use current core
	return rte_lcore_id(); 
}

static void
spdk_fc_ini_di_mp_ctor(struct rte_mempool *mp,
	__attribute__((unused))void *arg,
	void *_m,
	__attribute__((unused))unsigned in)
{   /* SCSI data buffer mempool initializer */
	struct spdk_fc_ini_di_mobj *m = (struct spdk_fc_ini_di_mobj *)_m;
	off_t off;

	m->mp = mp;
	m->buf = (uint8_t *)m + sizeof(struct spdk_fc_ini_di_mobj);
	m->buf = (void *)((unsigned long)((uint8_t *)m->buf + 512) & ~511UL);
	off = (uint64_t)(uint8_t *)m->buf - (uint64_t)(uint8_t *)m;

	m->phys_addr = spdk_vtophys(m) + off;
}

static
spdk_fc_ini_di_port_t* spdk_fc_ini_di_find_port_in_port_list(ocs_t *ocs_port)
{   /* find spdk_fc_ini_di_port_t that ocs_t port belongs to */
	spdk_fc_ini_di_port_t *port = NULL;
	uint64_t find_port_wwpn = spdk_fc_ini_di_port_node_wwpn(ocs_port);
	uint32_t pind;

	for (pind = 0; pind < g_fc_ini_num_ports && port == NULL; pind++) {
		if (g_fc_ini_port_list[pind].wwpn == find_port_wwpn) { /* found it */
			port = &g_fc_ini_port_list[pind];
		}
	}

	return port;
}

static
spdk_fc_ini_di_target_t *spdk_fc_ini_di_find_tgt_in_tgt_list(
	spdk_fc_ini_di_port_t *port, ocs_node_t *node)
{
	struct _di_target *tgt = NULL;
	uint64_t find_tgt_wwpn = spdk_fc_ini_di_tgt_node_wwpn(node);

	SPDK_FC_INI_DI_LIST_FOREACH(tgt, port, target_list, link) {
		if (tgt->wwpn == find_tgt_wwpn) {
			break;
		}
	}
	SPDK_FC_INI_DI_LIST_UNLOCK(port, target_list);
	return tgt;
}

static
void spdk_fc_ini_di_queue_dwqe(spdk_fc_ini_di_wqe_t *dwqe)
{
	bool start_q = false;
	spdk_fc_ini_di_port_t *port = dwqe->ini_port;

	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY,
		"queueing work request for port: %lx tgt: %lx\n", port->wwpn,
		dwqe->ini_target ? dwqe->ini_target->wwpn : 0);
	SPDK_FC_INI_DI_LIST_LOCK(port, discovery_work_q);
	DI_RETURN_IF_NOT_RUNNING;
	start_q = SPDK_FC_INI_DI_LIST_EMPTY(port, discovery_work_q);
	SPDK_FC_INI_DI_LIST_NL_INSERT(port, discovery_work_q, dwqe, link);
	SPDK_FC_INI_DI_LIST_UNLOCK(port, discovery_work_q);
	DI_RETURN_IF_NOT_RUNNING;
	if (start_q) {
		struct spdk_event *ev = spdk_event_allocate(
			spdk_fc_ini_di_get_lcore(dwqe),
			spdk_fc_ini_di_start_process_dwq,
			port, 0);

		spdk_event_call(ev);
		SPDK_DEBUGLOG(
			SPDK_LOG_FC_DISCOVERY, "create spdk event to process dwq\n");
	}
}

static
void spdk_fc_ini_di_queue_discovery_work_request(
	spdk_fc_ini_di_port_t *port,
	spdk_fc_ini_di_target_t *tgt,
	SPDK_FC_INI_PORT_TGT_EVENT_TYPE event_type)
{   /* queue a work request on a port's discovery work queue */
	spdk_fc_ini_di_wqe_t *dwqe = (spdk_fc_ini_di_wqe_t *)malloc(
		sizeof(spdk_fc_ini_di_wqe_t));
	if (dwqe == NULL) {
		SPDK_ERRLOG(
			"Unable to create dwqe for tgt: %lx on port: %lx\n",
			tgt->wwpn, port->wwpn);
	} else {
		memset(dwqe, 0, sizeof(spdk_fc_ini_di_wqe_t));
		dwqe->ini_port = port;
		dwqe->ini_target = tgt;
		dwqe->event_type = event_type;
		spdk_fc_ini_di_queue_dwqe(dwqe);
	}
}

static
void spdk_fc_ini_di_new_target(ocs_node_t *node)
{   /* handle new target event from driver */
	SPDK_FC_INI_PORT_TGT_EVENT_TYPE event_type = INI_NODE_EVENT_TYPE_NEW_TARGET;
	spdk_fc_ini_di_port_t *port = spdk_fc_ini_di_find_port_in_port_list(
								  node->ocs);

	if (port == NULL) {   /* did not find port */
		SPDK_ERRLOG(
			"ERROR - port list corrupted - %lx not found\n",
			spdk_fc_ini_di_port_node_wwpn(node->ocs));
	} else {
		/* look for existing target (even thouugh driver said it was new */
		spdk_fc_ini_di_target_t *new_tgt =
			spdk_fc_ini_di_find_tgt_in_tgt_list(port, node);

		if (new_tgt == NULL) {
			/* it is a new target */
			new_tgt = (spdk_fc_ini_di_target_t *)malloc(
				sizeof(spdk_fc_ini_di_target_t));

			if (new_tgt == NULL) {
				SPDK_ERRLOG(
					"Unable to allocate new tgt: %lx for port: %lx\n",
					spdk_fc_ini_di_tgt_node_wwpn(node), port->wwpn);
			} else {
				new_tgt->ini_port = port;
				new_tgt->target_node = node;
				new_tgt->wwpn = spdk_fc_ini_di_tgt_node_wwpn(node);
				SPDK_FC_INI_DI_LIST_INIT(new_tgt, bdev_list);
				SPDK_FC_INI_DI_LIST_INSERT(port, target_list, new_tgt, link);
				SPDK_DEBUGLOG(
					SPDK_LOG_FC_DISCOVERY,
					"new target - port: %lx (%s), tgt: %lx (%s)\n",
					port->wwpn, port->ini_ocs_port->display_name,
					new_tgt->wwpn, new_tgt->target_node->display_name);
			}
		} else { /* it's an existing target - driver reported as new */
			spdk_fc_ini_di_bdev_t* di_bdev;
			SPDK_DEBUGLOG(
				SPDK_LOG_FC_DISCOVERY,
				"new tgt event for existing tgt - port: %lx, tgt: %lx\n",
				port->wwpn, new_tgt->wwpn);
			/* update existing target and it's bdev's with new node ptr */
			new_tgt->target_node = node;
			SPDK_FC_INI_DI_LIST_FOREACH(di_bdev, new_tgt, bdev_list, link) {
				di_bdev->bdev.target_id = node;
			}
			SPDK_FC_INI_DI_LIST_UNLOCK(new_tgt, bdev_list);
			event_type = INI_NODE_EVENT_TYPE_TARGET_UP;
		}

		if (new_tgt != NULL) {
			spdk_fc_ini_di_queue_discovery_work_request(port, new_tgt,
				event_type);
		}
	}
}

static
void spdk_fc_ini_di_target_update(ocs_node_t *node,
	SPDK_FC_INI_PORT_TGT_EVENT_TYPE event_type)
{   /* handle new target, target link up, or lun update event from driver */
	spdk_fc_ini_di_port_t *port;/* = spdk_fc_ini_di_find_port_in_port_list(
								  node->ocs); */
	port = spdk_fc_ini_di_find_port_in_port_list(
								  node->ocs);
	if (port == NULL) {   /* did not find port */
		SPDK_ERRLOG(
			"ERROR - port list corrupted - %lx not found\n",
			spdk_fc_ini_di_port_node_wwpn(node->ocs));
	} else {   /* now find target */
		struct _di_target *tgt =
			spdk_fc_ini_di_find_tgt_in_tgt_list(port, node);

		if (tgt == NULL) {   /* did not find target */
			SPDK_ERRLOG(
			"ERROR - target list corrupted, couldn't find tgt: %lx port: %lx\n",
			spdk_fc_ini_di_tgt_node_wwpn(node), port->wwpn);
		} else {
			spdk_fc_ini_di_queue_discovery_work_request(port, tgt, event_type);
		}
	}
}

static
void spdk_fc_ini_di_port_down(ocs_t *port_node)
{   /* handle port down event from driver */
	spdk_fc_ini_di_port_t *port = spdk_fc_ini_di_find_port_in_port_list(port_node);

	if (port == NULL) {   /* did not find port */
		SPDK_ERRLOG(
			"ERROR - port list corrupted - %lx not found\n",
			spdk_fc_ini_di_port_node_wwpn(port_node));
	} else {
		spdk_fc_ini_di_queue_discovery_work_request(port, NULL,
			INI_NODE_EVENT_TYPE_PORT_DOWN);
	}
}

static
void spdk_fc_ini_di_start_process_dwq(void *arg1, void *arg2)
{   /* spdk event cb function to start processing a port's discovery work q */
	spdk_fc_ini_di_port_t *port =
		(spdk_fc_ini_di_port_t *)arg1;

	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY,
		"port %lx\n",
		 spdk_fc_ini_di_port_node_wwpn(port->ini_ocs_port));

	spdk_fc_ini_di_process_next_dwqe(port);
}

static 
void spdk_fc_ini_di_complete_dwqe(spdk_fc_ini_di_wqe_t *dwqe)
{   /* free the previous dwqe and process the next one */
	spdk_fc_ini_di_port_t *port = dwqe->ini_port;
	
	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY, "port %lx scsi_retry_in_progress:%s\n",
		port->wwpn, dwqe->scsi_retry_in_progress ? "true" : "false");

	if (dwqe->cmd_buf)
		rte_mempool_put(port->mp, (void **)&dwqe->cmd_buf);
	free(dwqe);
	spdk_fc_ini_di_process_next_dwqe(port);
}

static
void spdk_fc_ini_di_process_next_dwqe(spdk_fc_ini_di_port_t *port)
{   /* process next entry on port's discovery work queue */
	spdk_fc_ini_di_wqe_t *dwqe;
	DI_RETURN_IF_NOT_RUNNING;
	SPDK_FC_INI_DI_LIST_LOCK(port, discovery_work_q);
	dwqe = SPDK_FC_INI_DI_LIST_FIRST(port, discovery_work_q);
	if (dwqe)
	{
		SPDK_FC_INI_DI_LIST_NL_REMOVE(port, discovery_work_q, dwqe, link);
		SPDK_FC_INI_DI_LIST_UNLOCK(port, discovery_work_q);
		spdk_fc_ini_di_process_dwqe(dwqe);
	}
	else
	{
		SPDK_FC_INI_DI_LIST_UNLOCK(port, discovery_work_q);
		SPDK_DEBUGLOG(
			SPDK_LOG_FC_DISCOVERY, "no more dwqe's\n");
	}
}

static
void spdk_fc_ini_di_process_dwqe(spdk_fc_ini_di_wqe_t *dwqe)
{   /* process a discovery work queue entry */
	if (dwqe) {
		SPDK_DEBUGLOG(
			SPDK_LOG_FC_DISCOVERY, "port %lx, type=%d\n",
			DI_DWQE_PORT_WWPN(dwqe), dwqe->event_type);
		switch (dwqe->event_type) {
		case INI_NODE_EVENT_TYPE_NEW_TARGET:
		case INI_NODE_EVENT_TYPE_TARGET_UP:
		case INI_NODE_EVENT_TYPE_TGT_LUNS:
			spdk_fc_ini_di_process_target_update(dwqe);
			break;

		case INI_NODE_EVENT_TYPE_TARGET_DOWN:
			spdk_fc_ini_di_process_target_down(dwqe);
			break;

		case INI_NODE_EVENT_TYPE_PORT_DOWN:
			spdk_fc_ini_di_process_port_down(dwqe);
			break;

		default:
			{
				SPDK_WARNLOG(
					"Unknown update type %d for port: %lx\n",
					dwqe->event_type, DI_DWQE_PORT_WWPN(dwqe));
			}
			break;
		}
	} else {
		/* dwqe is NULL */
		SPDK_DEBUGLOG(SPDK_LOG_FC_DISCOVERY, "dwq empty\n");
	}
}

static
void spdk_fc_ini_di_process_target_down(spdk_fc_ini_di_wqe_t *dwqe)
{   /* process target link-down event */
	if (dwqe->ini_target) {
		spdk_fc_ini_di_bdev_t *di_bdev;
		uint32_t bdev_ready_cnt = 0;

		/* iterate target's devices to count ready devices */
		SPDK_FC_INI_DI_LIST_FOREACH(di_bdev, dwqe->ini_target, bdev_list, link) {
			if (di_bdev->bdev.is_ready == true) {
				bdev_ready_cnt++;
			}
		}
		SPDK_FC_INI_DI_LIST_UNLOCK(dwqe->ini_target, bdev_list);

		if (bdev_ready_cnt > 0) {
			/* do "not ready" callback for all ready luns on this target */
			uint32_t bdev_update_cnt = 0;
			spdk_fc_ini_bdev_update_t *update_list = 
				(spdk_fc_ini_bdev_update_t *)
				malloc(sizeof(spdk_fc_ini_bdev_update_t) * bdev_ready_cnt);

			SPDK_FC_INI_DI_LIST_FOREACH(di_bdev, dwqe->ini_target, bdev_list, link) {
				if (bdev_update_cnt < bdev_ready_cnt &&
						di_bdev->bdev.is_ready == true) {
					di_bdev->bdev.is_ready = false;
					update_list[bdev_update_cnt].bdev = &di_bdev->bdev;
					update_list[bdev_update_cnt++].update_mask =
						SPDK_FC_INI_BLKDEV_UPDATE_DEVICE_NOT_READY;
				}
			}
			SPDK_FC_INI_DI_LIST_UNLOCK(dwqe->ini_target, bdev_list);
			if (g_spdk_fc_ini_di_dev_list_update_cb) {
				g_spdk_fc_ini_di_dev_list_update_cb(bdev_update_cnt, update_list);
			}
			free(update_list);
		}
	}

	spdk_fc_ini_di_complete_dwqe(dwqe);
}


static
void spdk_fc_ini_di_process_port_down(spdk_fc_ini_di_wqe_t *dwqe)
{   /* process port link down event */
	if (dwqe->ini_port) {   /* iterate each target */
		uint32_t bdev_ready_cnt = 0;
		struct _di_target *tgt;

		SPDK_FC_INI_DI_LIST_FOREACH(tgt, dwqe->ini_port, target_list, link) {
			spdk_fc_ini_di_bdev_t *di_bdev;

			/* iterate target's devices and mark each not ready */
			SPDK_FC_INI_DI_LIST_FOREACH(di_bdev, tgt, bdev_list, link) {
				if (di_bdev->bdev.is_ready == true) {
					bdev_ready_cnt++;
				}
			}
		}

		if (bdev_ready_cnt > 0) {
			/* do "not ready" callback for all the port's ready luns */
			uint32_t bdev_update_cnt = 0;

			spdk_fc_ini_bdev_update_t *update_list =
				(spdk_fc_ini_bdev_update_t *)
				malloc(sizeof(spdk_fc_ini_bdev_update_t) * bdev_ready_cnt);
			SPDK_FC_INI_DI_LIST_NL_FOREACH(tgt, dwqe->ini_port, target_list, link) {
				spdk_fc_ini_di_bdev_t *di_bdev;
				SPDK_FC_INI_DI_LIST_NL_FOREACH(di_bdev, tgt, bdev_list, link) {
					if (bdev_update_cnt < bdev_ready_cnt &&
						di_bdev->bdev.is_ready == true) {
						di_bdev->bdev.is_ready = false;
						update_list[bdev_update_cnt].bdev = &di_bdev->bdev;
						update_list[bdev_update_cnt++].update_mask =
							SPDK_FC_INI_BLKDEV_UPDATE_DEVICE_NOT_READY;
					}
				}
				SPDK_FC_INI_DI_LIST_UNLOCK(tgt, bdev_list);
			}

			SPDK_FC_INI_DI_LIST_UNLOCK(dwqe->ini_port, target_list);
			if (g_spdk_fc_ini_di_dev_list_update_cb) {
				g_spdk_fc_ini_di_dev_list_update_cb(bdev_update_cnt, update_list);
			}
			free(update_list);
		} else {
			SPDK_FC_INI_DI_LIST_UNLOCK(dwqe->ini_port, target_list);
		}	
	}

	spdk_fc_ini_di_complete_dwqe(dwqe);
}

static
void spdk_fc_ini_di_process_target_update(spdk_fc_ini_di_wqe_t *dwqe)
{   /* process update to a target - send REPORT LUNS */
	if (dwqe->ini_target) {
		struct iovec iov[1];
		uint8_t scsi_cdb[12];
		struct spdk_fc_ini_bdev *tmp_bdev = NULL;

		/* get a report luns buffer */
		int rc = rte_mempool_get(dwqe->ini_port->mp, (void **)&dwqe->cmd_buf);
		if (rc < 0 || dwqe->cmd_buf == NULL) {
			SPDK_ERRLOG(
				"Can't get report luns buf for port: %lx, tgt: %lx, err %d\n",
				DI_DWQE_PORT_WWPN(dwqe), DI_DWQE_TGT_WWPN(dwqe), rc);
		} else {
			iov[0].iov_base	= (void *)dwqe->cmd_buf->phys_addr;
			iov[0].iov_len = SPDK_FC_INI_DI_SCSI_BUFFER_SIZE;

			tmp_bdev = (struct spdk_fc_ini_bdev *)malloc(
				sizeof(struct spdk_fc_ini_bdev));

			memset(tmp_bdev, 0, sizeof(struct spdk_fc_ini_bdev));
			tmp_bdev->target_id = (void *)dwqe->ini_target->target_node;

			/* create an REPORT LUNS CDB */
			memset(scsi_cdb, 0, sizeof(scsi_cdb));
			scsi_cdb[0] = SCSI_REPORT_LUNS;
			to_be32(&scsi_cdb[6], SPDK_FC_INI_DI_SCSI_BUFFER_SIZE);

			/* send it to target */
			rc = spdk_fc_ini_scsi_pass_thru(tmp_bdev, scsi_cdb,
				(uint32_t)sizeof(scsi_cdb), iov, 1,
				SPDK_FC_INI_DI_SCSI_BUFFER_SIZE,
				SPDK_SCSI_DIR_FROM_DEV,
				spdk_fc_ini_di_report_luns_complete,
				dwqe, 0);
			if (rc < 0) {
				free(tmp_bdev);
				DI_RETURN_IF_NOT_RUNNING;
				if (spdk_fc_ini_di_log_scsi_send_cmd_error("REPORT LUNS",
					rc, dwqe)) { /* start next dwqe */
					spdk_fc_ini_di_complete_dwqe(dwqe);
				}
			}
		}
	}
}

static
void spdk_fc_ini_di_report_luns_complete(
	struct spdk_fc_ini_cmd_complete_info *cci)
{   /* process SCSI REPORT LUNS complete for a target */
	spdk_fc_ini_di_wqe_t *dwqe;

	DI_RETURN_IF_NOT_RUNNING;

	free(cci->bdev);

	dwqe = (spdk_fc_ini_di_wqe_t *)cci->cb_data;
	dwqe->num_reported_luns = 0;

	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY, "port %lx, target %lx\n",
		DI_DWQE_PORT_WWPN(dwqe),  DI_DWQE_TGT_WWPN(dwqe));


	if (cci->cmd_result.host_result == 0 && cci->cmd_result.tgt_result == 0) {
		spdk_fc_ini_di_bdev_t *bdev;

		dwqe->scsi_retry_in_progress = false;

		if (cci->cmd_result.data_transferred > 0) {
			memcpy(dwqe->report_luns_buf, dwqe->cmd_buf->buf,
				cci->cmd_result.data_transferred);
			dwqe->num_reported_luns = from_be32(dwqe->report_luns_buf) / 8;
			if (dwqe->num_reported_luns > SPDK_FC_INI_DI_MAX_LUNS) {
				dwqe->num_reported_luns = SPDK_FC_INI_DI_MAX_LUNS;
			}
		}

		/* scan existing lun list matching with luns in REPORT LUNS buffer */
		SPDK_FC_INI_DI_LIST_FOREACH(bdev, dwqe->ini_target, bdev_list, link) {
			uint32_t lun_id = 1;
			for (; lun_id <= dwqe->num_reported_luns &&
				*((uint64_t*)&(dwqe->report_luns_buf[lun_id * 8])) != 
					bdev->bdev.fc_dev.lun_no; lun_id++);

			if (lun_id > dwqe->num_reported_luns) {
				/* existing lun not found - not ready */
				bdev->bdev.is_ready = false;
				if (g_spdk_fc_ini_di_dev_list_update_cb) {
					/* callback about not ready lun */
					spdk_fc_ini_bdev_update_t update_list[1];
					update_list[0].update_mask =
						SPDK_FC_INI_BLKDEV_UPDATE_DEVICE_NOT_READY;
					update_list[0].bdev = &bdev->bdev;
					g_spdk_fc_ini_di_dev_list_update_cb(1, update_list);
				}
			}
		}
		SPDK_FC_INI_DI_LIST_UNLOCK(dwqe->ini_target, bdev_list);

		if (dwqe->num_reported_luns == 0) {   /* no luns */
			SPDK_WARNLOG(
				"No report luns for port: %lx tgt: %lx\n",
				DI_DWQE_PORT_WWPN(dwqe),  DI_DWQE_TGT_WWPN(dwqe));
			spdk_fc_ini_di_complete_dwqe(dwqe);
		} else {   /* start processing luns */
			SPDK_DEBUGLOG(
				SPDK_LOG_FC_DISCOVERY, "num_luns=%d\n",
				dwqe->num_reported_luns);
			dwqe->curr_lun_index = 0;
			spdk_fc_ini_di_process_next_lun(dwqe);
		}
	} else {   /* error with REPORT LUNS */
		struct sense_info sense_info = { 0 };
		/* check for unit attention and immediately re-send command */
		if (cci->cmd_result.tgt_result == SAM_STAT_CHECK_CONDITION) {
			if (ocs_get_sense_info(cci->cmd_result.scsi_resp,
					cci->cmd_result.scsi_resp_len, &sense_info) == 0) {
				if (sense_info.sense_key  == SPDK_SCSI_SENSE_UNIT_ATTENTION
					&& dwqe->scsi_retry_in_progress == false) {
					/* try report luns command again */
					dwqe->scsi_retry_in_progress = true;
					spdk_fc_ini_di_process_target_update(dwqe);
					return;
				}
			} 
		}
		if (spdk_fc_ini_di_log_scsi_cmd_resp_error("REPORT LUNS", cci)) {
			spdk_fc_ini_di_complete_dwqe(dwqe);
		}
	}
}

static
void spdk_fc_ini_di_process_next_lun(spdk_fc_ini_di_wqe_t *dwqe)
{   /* process next lun in the REPORT LUNS buffer */
	DI_RETURN_IF_NOT_RUNNING;

	dwqe->curr_lun_index++;
	if (dwqe->curr_lun_index > dwqe->num_reported_luns) {
		/* done with this target, start the next one */
		SPDK_DEBUGLOG(SPDK_LOG_FC_DISCOVERY, "no more luns\n");
		spdk_fc_ini_di_complete_dwqe(dwqe);
	} else { /* send inquiry command to next LUN */
		int rc;
		struct iovec iov[1];
		uint8_t scsi_cdb[6];
		struct spdk_fc_ini_bdev *tmp_bdev = NULL;

		SPDK_DEBUGLOG(
			SPDK_LOG_FC_DISCOVERY, "port %lx, target %lx, lun %016lx\n",
			DI_DWQE_PORT_WWPN(dwqe), DI_DWQE_TGT_WWPN(dwqe),
			*((uint64_t*)&dwqe->report_luns_buf[dwqe->curr_lun_index * 8]));


		iov[0].iov_base	= (void *)dwqe->cmd_buf->phys_addr;
		iov[0].iov_len = 0xff;

		tmp_bdev = (struct spdk_fc_ini_bdev *)malloc(
			sizeof(struct spdk_fc_ini_bdev));
		memset(tmp_bdev, 0, sizeof(struct spdk_fc_ini_bdev));
		tmp_bdev->target_id = (void *)dwqe->ini_target->target_node;
		*((uint64_t*)&(tmp_bdev->fc_dev.lun_no)) =  
			*((uint64_t*)&(dwqe->report_luns_buf[dwqe->curr_lun_index * 8]));

		/* create an INQUIRY CDB */
		memset(scsi_cdb, 0, sizeof(scsi_cdb));
		scsi_cdb[0] = SCSI_INQUIRY;
		scsi_cdb[4] = 0xff; /* 255 max. return length */

		SPDK_DEBUGLOG(SPDK_LOG_FC_DISCOVERY, "sending inquiry\n");
		rc = spdk_fc_ini_scsi_pass_thru(tmp_bdev, scsi_cdb,
			(uint32_t)sizeof(scsi_cdb), iov, 1,
			0xff, SPDK_SCSI_DIR_FROM_DEV,
			spdk_fc_ini_di_inquiry_complete, dwqe, 0);

		if (rc < 0) {   /* error sending inquiry cmd */
			free(tmp_bdev);
			DI_RETURN_IF_NOT_RUNNING;
			if (spdk_fc_ini_di_log_scsi_send_cmd_error("INQUIRY", rc, dwqe)) {
				spdk_fc_ini_di_process_next_lun(dwqe); /* move on to next lun */
			}
		}
	}
}

static
void spdk_fc_ini_di_inquiry_complete(struct spdk_fc_ini_cmd_complete_info *cci)
{   /* process SCSI INQUIRY complete on a LUN */
	spdk_fc_ini_di_wqe_t *dwqe;

	DI_RETURN_IF_NOT_RUNNING;

	dwqe = (spdk_fc_ini_di_wqe_t *)cci->cb_data;

	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY,
		"port %lx, target %lx, lun %lx, (hr=%d, tr=%d, size=%ld)\n",
		DI_DWQE_PORT_WWPN(dwqe),  DI_DWQE_TGT_WWPN(dwqe),
		cci->bdev->fc_dev.lun_no,
		cci->cmd_result.host_result,
		cci->cmd_result.tgt_result,
		cci->cmd_result.data_transferred);

	if (cci->cmd_result.host_result == 0
		&& cci->cmd_result.tgt_result == 0
		&& cci->cmd_result.data_transferred >= 36) {
		spdk_fc_ini_di_process_inquiry(cci->bdev->fc_dev.lun_no, dwqe);
	}
	else if (spdk_fc_ini_di_log_scsi_cmd_resp_error("INQUIRY", cci)) {
			spdk_fc_ini_di_process_next_lun(dwqe);
	}

	free(cci->bdev);
}

static
void spdk_fc_ini_di_process_inquiry(uint64_t lun_no,
	spdk_fc_ini_di_wqe_t *dwqe)
{   /* process LUN's inquiry data */
	uint8_t *inq_buf = (uint8_t *)dwqe->cmd_buf->buf;
	spdk_fc_ini_di_bdev_t *bdev;

	if (inq_buf[0] == 0) {   /* disk device - see if already in lun_list */
		SPDK_FC_INI_DI_LIST_FOREACH(bdev, dwqe->ini_target, bdev_list, link) {
			if (lun_no == bdev->bdev.fc_dev.lun_no) {
				memcpy(bdev->bdev.fc_dev.vendor_id, &inq_buf[8], 16);
				memcpy(bdev->bdev.fc_dev.product_id, &inq_buf[16], 16);
				break;
			}
		}
		SPDK_FC_INI_DI_LIST_UNLOCK(dwqe->ini_target, bdev_list);

		if (bdev == NULL) {   /* add new lun (bdev) */
			bdev = (spdk_fc_ini_di_bdev_t *)malloc(
				sizeof(spdk_fc_ini_di_bdev_t));
			memset(bdev, 0, sizeof(spdk_fc_ini_di_bdev_t));
			bdev->new_lun = true;
			bdev->bdev.is_ready = true;
			bdev->bdev.lcore_mask = dwqe->ini_port->ini_ocs_port->lcore_mask;
			bdev->bdev.target_id = dwqe->ini_target->target_node;
			bdev->bdev.fc_dev.di_tgt = (void*) dwqe->ini_target;
			bdev->bdev.fc_dev.lun_no = lun_no;
			memcpy(bdev->bdev.fc_dev.vendor_id, &inq_buf[8],
				sizeof(bdev->bdev.fc_dev.vendor_id));
			memcpy(bdev->bdev.fc_dev.product_id, &inq_buf[16],
				sizeof(bdev->bdev.fc_dev.product_id));
			bdev->bdev.fc_dev.di_tgt = dwqe->ini_target;
			bdev->bdev.ini_address.wwnn =
				spdk_fc_ini_di_port_node_wwnn(dwqe->ini_port->ini_ocs_port);
			bdev->bdev.ini_address.wwpn = dwqe->ini_port->wwpn;
			bdev->bdev.tgt_address.wwnn =
				spdk_fc_ini_di_tgt_node_wwnn(dwqe->ini_target->target_node);
			bdev->bdev.tgt_address.wwpn = dwqe->ini_target->wwpn;
			SPDK_FC_INI_DI_LIST_INSERT(dwqe->ini_target, bdev_list, bdev, link);
		}

		spdk_fc_ini_di_send_read_capacity(lun_no, dwqe);
	} else {
		/* not a disk, see if lun with same number in lun_list and remove it */
		SPDK_DEBUGLOG(
			SPDK_LOG_FC_DISCOVERY,
			"non-disk lun: port %lx, target %lx, lun %lx, type=%d\n",
			DI_DWQE_PORT_WWPN(dwqe),  DI_DWQE_TGT_WWPN(dwqe),
			lun_no, inq_buf[0]);
		SPDK_FC_INI_DI_LIST_FOREACH(bdev, dwqe->ini_target, bdev_list,
			link) {
			if (lun_no == bdev->bdev.fc_dev.lun_no) {
				/* found it - remove it */
				SPDK_FC_INI_DI_LIST_REMOVE(dwqe->ini_target, bdev_list, bdev,
					link);
				if (g_spdk_fc_ini_di_dev_list_update_cb) {
					/* callback about removed lun */
					spdk_fc_ini_bdev_update_t update_list[1];
					update_list[0].update_mask =
						SPDK_FC_INI_BLKDEV_UPDATE_DEVICE_REMOVED;
					update_list[0].bdev = &bdev->bdev;
					g_spdk_fc_ini_di_dev_list_update_cb(1, update_list);
				}
				free(bdev);
				break;
			}
		}
		SPDK_FC_INI_DI_LIST_UNLOCK(dwqe->ini_target, bdev_list);

		DI_RETURN_IF_NOT_RUNNING;
		spdk_fc_ini_di_process_next_lun(dwqe);
	}
}

static
void spdk_fc_ini_di_send_read_capacity(uint64_t lun_no,
	spdk_fc_ini_di_wqe_t *dwqe)
{   /* send SCSI READ CAPACITY command to a LUN */
	int rc;
	struct iovec iov[1];
	uint8_t scsi_cdb[16];
	struct spdk_fc_ini_bdev *tmp_bdev = NULL;

	DI_RETURN_IF_NOT_RUNNING;

	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY, "port %lx, target %lx, lun %lx\n",
		DI_DWQE_PORT_WWPN(dwqe),  DI_DWQE_TGT_WWPN(dwqe), lun_no);

	iov[0].iov_base	= (void *)dwqe->cmd_buf->phys_addr;
	iov[0].iov_len = 32;

	tmp_bdev = (struct spdk_fc_ini_bdev *)malloc(
		sizeof(struct spdk_fc_ini_bdev));
	memset(tmp_bdev, 0, sizeof(struct spdk_fc_ini_bdev));
	tmp_bdev->target_id = (void *)dwqe->ini_target->target_node;

	tmp_bdev->fc_dev.lun_no = lun_no;

	/* create a READ CAPACITY CDB */
	memset(scsi_cdb, 0, sizeof(scsi_cdb));
	scsi_cdb[0] = SCSI_SERVICE_ACTION_IN16;
	scsi_cdb[1] = SCSI_SVC_ACTION16_READ_CAPACITY;
	scsi_cdb[13] = 32; /* 32 max. return length */

	rc = spdk_fc_ini_scsi_pass_thru(tmp_bdev, scsi_cdb,
		(uint32_t)sizeof(scsi_cdb), iov, 1,
		32, SPDK_SCSI_DIR_FROM_DEV,
		spdk_fc_ini_di_read_capacity_complete, dwqe, 0);

	if (rc < 0) {
		DI_RETURN_IF_NOT_RUNNING;
		if (spdk_fc_ini_di_log_scsi_send_cmd_error("READ CAPACITY",
			rc, dwqe)) {
			spdk_fc_ini_di_process_next_lun(dwqe); /* try next lun */
		}
	}
}

static
void spdk_fc_ini_di_read_capacity_complete(
	struct spdk_fc_ini_cmd_complete_info *cci)
{   /* process SCSI READ CAPACITY complete */
	spdk_fc_ini_di_wqe_t *dwqe;

	DI_RETURN_IF_NOT_RUNNING;

	dwqe = (spdk_fc_ini_di_wqe_t *)cci->cb_data;

	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY, "port %lx, target %lx, lun %lx\n",
		DI_DWQE_PORT_WWPN(dwqe),  DI_DWQE_TGT_WWPN(dwqe),
		cci->bdev->fc_dev.lun_no);

	if (cci->cmd_result.host_result == 0
		&& cci->cmd_result.tgt_result == 0
		&& cci->cmd_result.data_transferred >= 32) {
		uint8_t *rc_buf = (uint8_t *)dwqe->cmd_buf->buf;
		spdk_fc_ini_di_bdev_t *bdev;

		SPDK_FC_INI_DI_LIST_FOREACH(bdev, dwqe->ini_target, bdev_list, link) {
			if (cci->bdev->fc_dev.lun_no == bdev->bdev.fc_dev.lun_no) {
				break;
			}
		}
		SPDK_FC_INI_DI_LIST_UNLOCK(dwqe->ini_target, bdev_list);

		if (bdev == NULL) {
			/* could not find lun in target's lun_list - should not happen */
			SPDK_ERRLOG(
				"ERROR - lun list corrupted - port: %lx tgt: %lx\n",
				DI_DWQE_PORT_WWPN(dwqe),  DI_DWQE_TGT_WWPN(dwqe));
		} else {
			uint64_t blk_cnt = from_be64(&rc_buf[0]);
			uint32_t blk_len = from_be32(&rc_buf[8]);
			uint32_t update_mask = 0;

			if (!bdev->new_lun &&
				((bdev->bdev.fc_dev.blk_cnt != blk_cnt) ||
				 (bdev->bdev.fc_dev.blk_len != blk_len))) {
				update_mask |= SPDK_FC_INI_BLKDEV_UPDATE_DEVICE_SIZE_CHANGE;
			}

			bdev->bdev.fc_dev.blk_cnt = blk_cnt;
			bdev->bdev.fc_dev.blk_len = blk_len;

			if (bdev->new_lun) {
				bdev->new_lun = false;
				update_mask |= SPDK_FC_INI_BLKDEV_UPDATE_NEW_DEVICE;
			}

			if (bdev->bdev.is_ready == false) {
				bdev->bdev.is_ready = true;
				update_mask |= SPDK_FC_INI_BLKDEV_UPDATE_DEVICE_READY;
			}

			if (update_mask != 0 && g_spdk_fc_ini_di_dev_list_update_cb) {
				/* callback for updated lun */
				spdk_fc_ini_bdev_update_t update_list[1];
				update_list[0].update_mask = update_mask;
				update_list[0].bdev = &bdev->bdev;
				SPDK_DEBUGLOG(SPDK_LOG_FC_DISCOVERY,
					"doing dev_list_update_callback to client\n");
				g_spdk_fc_ini_di_dev_list_update_cb(1, update_list);
			}
		}
	} else {   // error with READ CAPACITY command
		if (spdk_fc_ini_di_log_scsi_cmd_resp_error("READ CAPACITY", cci) == false)
		   /* Recovery will start from beginning, don't go for next lun */
			goto free_bdev;
	}

	spdk_fc_ini_di_process_next_lun(dwqe);
free_bdev:
	free(cci->bdev);
}

static void spdk_fc_ini_queue_on_recovery_q(
	spdk_fc_ini_di_cmd_recovery_q_t *rq,
	spdk_fc_ini_di_wqe_t *dwqe)
{
	bool start_q = false;

	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY,
		"queueing work request on recovery queue for port: %lx\n",
		DI_DWQE_PORT_WWPN(dwqe));

	SPDK_FC_INI_DI_LIST_LOCK(rq, cmd_recovery_work_q);
	start_q = SPDK_FC_INI_DI_LIST_EMPTY(rq, cmd_recovery_work_q);
	SPDK_FC_INI_DI_LIST_NL_INSERT(rq, cmd_recovery_work_q, dwqe, link);
	SPDK_FC_INI_DI_LIST_UNLOCK(rq, cmd_recovery_work_q);

	if (start_q) {
		g_cmd_recovery_timer = spdk_poller_register(spdk_fc_ini_di_command_recovery_startup,
							    (void *) rq,
							    SPDK_FC_INI_DI_RECOVERY_TIMER_INTERVAL);
		SPDK_DEBUGLOG(
			SPDK_LOG_FC_DISCOVERY, "reset recovery q timer\n");
	}
}

static int
spdk_fc_ini_di_command_recovery_startup(void *arg)
{
	spdk_fc_ini_di_cmd_recovery_q_t *rq =
		(spdk_fc_ini_di_cmd_recovery_q_t*)arg;

	/* Unregister poller */
	spdk_poller_unregister(&g_cmd_recovery_timer);
		
	if (!g_spdk_fc_ini_di_running) {
		return 0;
	}

	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY, "\n");		
		
	if (rq)
	{
		spdk_fc_ini_di_wqe_t *dwqe;

		SPDK_FC_INI_DI_LIST_LOCK(rq, cmd_recovery_work_q);
		if (!g_spdk_fc_ini_di_running) {
			return 0;
		}
		dwqe = SPDK_FC_INI_DI_LIST_FIRST(rq, cmd_recovery_work_q);
		while (dwqe) {
			SPDK_FC_INI_DI_LIST_NL_REMOVE(rq, cmd_recovery_work_q, dwqe,
				link);
			spdk_fc_ini_di_queue_dwqe(dwqe); /* queue on port's work q */
			if (!g_spdk_fc_ini_di_running) {
				return 0;
			}
			dwqe = SPDK_FC_INI_DI_LIST_FIRST(rq, cmd_recovery_work_q);
		}
		SPDK_FC_INI_DI_LIST_UNLOCK(rq, cmd_recovery_work_q);
	}

	return 0;
}


/***********************/
/* External functions  */
/***********************/

#ifdef DEBUG_TEST_DI_NOTIFICATIONS
static struct spdk_poller *g_dbg_timer;
static void spdk_fc_ini_di_dbg_tst_event_call(void *arg);
#endif

void spdk_fc_ini_di_init(spdk_fc_ini_bdev_list_update_cb cb)
{
#ifdef DEBUG
	/* set discovery trace logging */
	extern struct spdk_log_flag SPDK_LOG_FC_DISCOVERY;
	SPDK_LOG_FC_DISCOVERY.enabled = true;
#endif

	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY, "Initializing FC LUN Discovery\n");

	/* LUN discovery initialization */
	if (!g_spdk_fc_ini_di_running) {
		ocs_t *ports[SPDK_FC_INI_DI_MAX_PORTS];
		uint32_t num_ports = 0;

		g_spdk_fc_ini_di_running = true;

#ifdef DEBUG_TEST_DI_NOTIFICATIONS
		spdk_poller_register(&g_dbg_timer, spdk_fc_ini_di_dbg_tst_event_call,
				0, rte_lcore_id(), NULL, 10000);
#endif

		g_spdk_fc_ini_di_dev_list_update_cb = cb;
		
		num_ports = ocsu_ini_get_initiators(ports, SPDK_FC_INI_DI_MAX_PORTS);
		if (num_ports == 0) {
			SPDK_WARNLOG("WARNING!!! No initiator ports!\n");
		} else if (num_ports > SPDK_FC_INI_DI_MAX_PORTS) {
			SPDK_ERRLOG("Too many initiator ports (%d)!\n", num_ports);
		} else {
			uint32_t pind;
			spdk_fc_ini_di_cmd_recovery_q_t *rq = 
				&g_spdk_fc_ini_di_cmd_recovery_q;

			SPDK_DEBUGLOG(
				SPDK_LOG_FC_DISCOVERY, "%d ports\n", num_ports);

			SPDK_FC_INI_DI_LIST_INIT(rq, cmd_recovery_work_q);

			for (pind = 0; pind < num_ports; pind++) {
				spdk_fc_ini_di_port_t* port = &g_fc_ini_port_list[pind];  
				int mp_obj_size = SPDK_FC_INI_DI_SCSI_BUFFER_SIZE +
					sizeof(struct spdk_fc_ini_di_mobj) +
					512;
				char mempool_name[32];

				port->ini_ocs_port = ports[pind];
				port->wwpn = spdk_fc_ini_di_port_node_wwpn(ports[pind]);

				SPDK_FC_INI_DI_LIST_INIT(port, target_list);
				SPDK_FC_INI_DI_LIST_INIT(port, discovery_work_q);

				snprintf(mempool_name, sizeof(mempool_name),
					"spdk_fc_ini_di_port_mp_%d", pind);

				g_fc_ini_port_list[pind].mp = rte_mempool_create(
					mempool_name,
					SPDK_FC_INI_DI_NUM_SCSI_BUFFERS,
					mp_obj_size, 0, 0,
					NULL, NULL,
					spdk_fc_ini_di_mp_ctor,
					NULL, SOCKET_ID_ANY, 0);
			}

			g_fc_ini_num_ports = num_ports;
			/* let OCS driver know it can start sending update callbacks */
			ocsu_ini_init();
		}
	}
}

void spdk_fc_ini_di_update_devs_notify(
	SPDK_FC_INI_PORT_TGT_EVENT_TYPE event_type,
	void *update_node)
{   /* callback from FC driver indicating discovery needs to be done */
	if (g_spdk_fc_ini_di_running) {
		switch (event_type) {
		case INI_NODE_EVENT_TYPE_NEW_TARGET:
			SPDK_DEBUGLOG(
				SPDK_LOG_FC_DISCOVERY, "event_type=NEW TARGET\n");
			spdk_fc_ini_di_new_target((ocs_node_t *)update_node);
			break;

		case INI_NODE_EVENT_TYPE_TARGET_DOWN:
			SPDK_DEBUGLOG(
				SPDK_LOG_FC_DISCOVERY, "event_type=TARGET DOWN\n");
			spdk_fc_ini_di_target_update((ocs_node_t *)update_node,
				event_type);
			break;

		case INI_NODE_EVENT_TYPE_TARGET_UP:
			SPDK_DEBUGLOG(
				SPDK_LOG_FC_DISCOVERY, "event_type=TARGET UP\n");
			spdk_fc_ini_di_target_update((ocs_node_t *)update_node,
				event_type);
			break;

		case INI_NODE_EVENT_TYPE_TGT_LUNS:
			SPDK_DEBUGLOG(
				SPDK_LOG_FC_DISCOVERY, "event_type=TARGET LUN CHANGE\n");
			spdk_fc_ini_di_target_update((ocs_node_t *)update_node,
				event_type);
			break;

		case INI_NODE_EVENT_TYPE_PORT_DOWN:
			SPDK_DEBUGLOG(
				SPDK_LOG_FC_DISCOVERY, "event_type=PORT DOWN\n");
			spdk_fc_ini_di_port_down((ocs_t *)update_node);
			break;
	
		default:
			SPDK_WARNLOG(
				"Unknown event type (%d) received from OCS driver\n",
				(int)event_type);
			break;
		}
	}
}

int spdk_fc_ini_di_shutdown(void)
{
	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY, "\n");

	/* clean up discovery for shutdown */
	if (g_spdk_fc_ini_di_running && g_fc_ini_num_ports > 0) {
		uint32_t pind;
		spdk_fc_ini_di_cmd_recovery_q_t *rq = &g_spdk_fc_ini_di_cmd_recovery_q;

		g_spdk_fc_ini_di_running = false;

		SPDK_DEBUGLOG(
			SPDK_LOG_FC_DISCOVERY, "clean port recovery work queue\n");
		/* no need to lock queues because we turned off running flag */
		spdk_fc_ini_di_wqe_t *dwqe = SPDK_FC_INI_DI_LIST_FIRST(rq, 
			cmd_recovery_work_q);
		while (dwqe) {
			SPDK_FC_INI_DI_LIST_NL_REMOVE(rq, cmd_recovery_work_q, dwqe, link);
			free(dwqe);
			dwqe = SPDK_FC_INI_DI_LIST_FIRST(rq, cmd_recovery_work_q);
		}

		for (pind = 0; pind < g_fc_ini_num_ports; pind++) {
			spdk_fc_ini_di_target_t *tgt;
			spdk_fc_ini_di_port_t *port = &g_fc_ini_port_list[pind];
			spdk_fc_ini_di_wqe_t *dwqe = SPDK_FC_INI_DI_LIST_FIRST(port, 
				discovery_work_q);
			SPDK_DEBUGLOG(
				SPDK_LOG_FC_DISCOVERY, "clean port work queue\n");
			while (dwqe) {
				SPDK_FC_INI_DI_LIST_NL_REMOVE(port, discovery_work_q, dwqe, link);
				free(dwqe);
				dwqe = SPDK_FC_INI_DI_LIST_FIRST(port, discovery_work_q);
			}


			SPDK_DEBUGLOG(
				SPDK_LOG_FC_DISCOVERY, "clean port's targets\n");
			tgt = SPDK_FC_INI_DI_LIST_FIRST(port, target_list);
			while (tgt) {
				spdk_fc_ini_di_bdev_t *di_bdev = SPDK_FC_INI_DI_LIST_FIRST(
					tgt, bdev_list);
				while (di_bdev) {
					SPDK_FC_INI_DI_LIST_NL_REMOVE(tgt, bdev_list, di_bdev, link);
					free(di_bdev);
					di_bdev = SPDK_FC_INI_DI_LIST_FIRST(tgt, bdev_list);
				}
				SPDK_FC_INI_DI_LIST_NL_REMOVE(port, target_list, tgt, link);
				free(tgt);
				tgt = SPDK_FC_INI_DI_LIST_FIRST(port, target_list);
			}

			SPDK_DEBUGLOG(
				SPDK_LOG_FC_DISCOVERY, "free port's mempool\n");
			rte_mempool_free(port->mp);
		}

		g_fc_ini_num_ports = 0;
	}

	spdk_fc_ini_blkreq_free(); // free the block layer mempool
	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY, "done\n");
	return 0;
}

SPDK_LOG_REGISTER_COMPONENT("fc_ini_discovery", SPDK_LOG_FC_DISCOVERY)

/* Debug Testing - Push Driver Events through discovery code */
#ifdef DEBUG_TEST_DI_NOTIFICATIONS
static uint32_t dbg_wwpn_byte6 = 1;
static uint32_t dbg_event_index = 0;
SPDK_FC_INI_PORT_TGT_EVENT_TYPE dbg_events[] = {
	INI_NODE_EVENT_TYPE_PORT_DOWN,
	INI_NODE_EVENT_TYPE_TARGET_UP,
	//INI_NODE_EVENT_TYPE_NEW_TARGET,
	INI_NODE_EVENT_TYPE_TARGET_DOWN,
	INI_NODE_EVENT_TYPE_TARGET_UP,
	INI_NODE_EVENT_TYPE_TGT_LUNS,
};

static void spdk_fc_ini_di_dbg_tst_new_tgt(void);
static void spdk_fc_ini_di_dbg_tst_tgt_down(void);
static void spdk_fc_ini_di_dbg_tst_tgt_up(void);
static void spdk_fc_ini_di_dbg_tst_tgt_luns(void);
static void spdk_fc_ini_di_dbg_tst_tgt_port_down(void);

static void
spdk_fc_ini_di_dbg_tst_new_tgt(void)
{
	spdk_fc_ini_di_target_t *tgt;

	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY, "\n");

	if (g_fc_ini_num_ports > 0 && dbg_wwpn_byte6 <= 10) {
		spdk_fc_ini_di_port_t *port = &g_fc_ini_port_list[0];
		tgt = SPDK_FC_INI_DI_LIST_FIRST(port, target_list);
		if (tgt) {
			ocs_node_t 	*node = (ocs_node_t*) malloc (sizeof(ocs_node_t));
			memcpy(node, tgt->target_node, sizeof(ocs_node_t));
			node->display_name[0] = (char)dbg_wwpn_byte6++;
			spdk_fc_ini_di_update_devs_notify(
				INI_NODE_EVENT_TYPE_NEW_TARGET, (void*)node);
		}
	}
}

static void
spdk_fc_ini_di_dbg_tst_tgt_down(void)
{
	spdk_fc_ini_di_target_t *tgt;

	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY, "\n");

	if (g_fc_ini_num_ports > 0) {
		spdk_fc_ini_di_port_t *port = &g_fc_ini_port_list[0];
		tgt = SPDK_FC_INI_DI_LIST_FIRST(port, target_list);
		if (tgt) {
			spdk_fc_ini_di_update_devs_notify(
				INI_NODE_EVENT_TYPE_TARGET_DOWN, (void*)tgt->target_node);
		}
	}
}

static void
spdk_fc_ini_di_dbg_tst_tgt_up(void)
{
	spdk_fc_ini_di_target_t *tgt;

	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY, "\n");

	if (g_fc_ini_num_ports > 0) {
		spdk_fc_ini_di_port_t *port = &g_fc_ini_port_list[0];
		SPDK_FC_INI_DI_LIST_NL_FOREACH(tgt, port, target_list, link) {
			spdk_fc_ini_di_update_devs_notify(
				INI_NODE_EVENT_TYPE_NEW_TARGET, (void*)tgt->target_node);
		}
	}
}

static void
spdk_fc_ini_di_dbg_tst_tgt_luns(void)
{
	spdk_fc_ini_di_target_t *tgt;

	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY, "\n");

	if (g_fc_ini_num_ports > 0) {
		spdk_fc_ini_di_port_t *port = &g_fc_ini_port_list[0];
		tgt = SPDK_FC_INI_DI_LIST_FIRST(port, target_list);
		if (tgt) {
			spdk_fc_ini_di_update_devs_notify(
				INI_NODE_EVENT_TYPE_TGT_LUNS, (void*)tgt->target_node);
		}
	}
}

static void
spdk_fc_ini_di_dbg_tst_tgt_port_down(void)
{
	spdk_fc_ini_di_target_t *tgt;

	SPDK_DEBUGLOG(
		SPDK_LOG_FC_DISCOVERY, "\n");

	if (g_fc_ini_num_ports > 0) {
		spdk_fc_ini_di_port_t *port = &g_fc_ini_port_list[0];
		tgt = SPDK_FC_INI_DI_LIST_FIRST(port, target_list);
		if (tgt) {
			spdk_fc_ini_di_update_devs_notify(
				INI_NODE_EVENT_TYPE_PORT_DOWN, (void*)port->ini_ocs_port);
		}
	}
}

static void
spdk_fc_ini_di_dbg_tst_event_call(void *arg)
{
	switch (dbg_events[dbg_event_index])
	{
	case INI_NODE_EVENT_TYPE_NEW_TARGET:
		spdk_fc_ini_di_dbg_tst_new_tgt();
		break;
	case INI_NODE_EVENT_TYPE_TARGET_DOWN:
		spdk_fc_ini_di_dbg_tst_tgt_down();
		break;
	case INI_NODE_EVENT_TYPE_TARGET_UP:
		spdk_fc_ini_di_dbg_tst_tgt_up();
		break;
	case INI_NODE_EVENT_TYPE_TGT_LUNS:
		spdk_fc_ini_di_dbg_tst_tgt_luns();
		break;
	case INI_NODE_EVENT_TYPE_PORT_DOWN:
		spdk_fc_ini_di_dbg_tst_tgt_port_down();
		break;
	}

	dbg_event_index++;
	dbg_event_index %= (sizeof(dbg_events)/sizeof(SPDK_FC_INI_PORT_TGT_EVENT_TYPE));

	/* start  new timer */
	spdk_poller_register(&g_dbg_timer, spdk_fc_ini_di_dbg_tst_event_call, 0,
			rte_lcore_id(), NULL, 5000);
}
#endif /* DEBUG_TEST_DI_NOTIFICATIONS */


