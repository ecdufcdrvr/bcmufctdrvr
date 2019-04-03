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

#include <inttypes.h>
#include <unistd.h>
#include "spdk_internal/log.h"
#include "spdk_internal/event.h"
#include "spdk/env.h"
#include "task.h"
#include "fc.h"
#include "fc_subsystem.h"
#include "ocs.h"
#include "ocs_spdk_conf.h"

#define DEFAULT_TASK_POOL_SIZE 16384

extern uint32_t ocs_spdk_master_core;

extern int spdk_fc_ini_di_shutdown(void);
extern void spdk_fc_free_all_pools(void);
struct spdk_fc_globals g_spdk_fc;

static void
spdk_mobj_ctor(struct rte_mempool *mp, __attribute__((unused))void *arg,
	void *_m, __attribute__((unused))unsigned in)
{
	struct spdk_mobj *m = _m;
	off_t off;

	m->mp = mp;
	m->buf = (uint8_t *)m + sizeof(struct spdk_mobj);
	m->buf = (void *)((unsigned long)((uint8_t *)m->buf + 512) & ~511UL);
	off = (uint64_t)(uint8_t *)m->buf - (uint64_t)(uint8_t *)m;

	m->phys_addr = spdk_vtophys(m, NULL) + off;
}


static int
spdk_fc_initialize_all_pools(void)
{
	struct spdk_fc_globals *fc = &g_spdk_fc;
	int mobj_size = SPDK_FC_LARGE_RBUF_MAX_SIZE + sizeof(struct spdk_mobj) + 512;


	/* create scsi_task pool */
	fc->task_pool = rte_mempool_create("SCSI_TASK_Pool",
		DEFAULT_TASK_POOL_SIZE,
		sizeof(struct spdk_fc_task),
		128, 0,
		NULL, NULL, NULL, NULL,
		SOCKET_ID_ANY, 0);
	if (!fc->task_pool) {
		SPDK_ERRLOG("create task pool failed\n");
		return -1;
	}

	/* Buffer pool for writes */
	fc->buff_pool = rte_mempool_create("write_buff_Pool",
		4096,
		mobj_size,
		0, 0, NULL, NULL,
		spdk_mobj_ctor, NULL,
		SOCKET_ID_ANY, 0);
	if (!fc->buff_pool) {
		SPDK_ERRLOG("create buff pool failed\n");
		return -1;
	}

	return 0;
}

void
spdk_fc_free_all_pools(void)
{
	struct spdk_fc_globals *fc = &g_spdk_fc;
	if (fc->task_pool) {
		rte_mempool_free(fc->task_pool);
	}

	if (fc->buff_pool) {
		rte_mempool_free(fc->buff_pool);
	}
}

static int
spdk_fc_app_read_parameters(void)
{
	struct spdk_fc_hba_port *tmp 	= NULL;
	struct spdk_fc_ig *tmp1 	= NULL;
	struct spdk_fc_lun_map *tmp2 	= NULL;
	
	if (spdk_fc_cf_init_hba_ports() != 0) {
		goto error;
	}

	if (spdk_fc_cf_init_igs() != 0) {
		goto error;
	}

	if (spdk_fc_cf_init_lun_maps() != 0) {
		goto error;
	} 
 
	return 0;
error:
	CLEANUP_NODE_LIST(g_spdk_fc_hba_ports, tmp);
	CLEANUP_NODE_LIST(g_spdk_fc_igs, tmp1);
	CLEANUP_NODE_LIST(g_spdk_fc_lun_maps, tmp2);
	return -1;
}

int
spdk_fc_subsystem_init(void)
{
	int 	 rc;

	/* save master core value */
	ocs_spdk_master_core = spdk_env_get_current_core();

	rc = spdk_fc_app_read_parameters();
	if (rc < 0) {
		SPDK_ERRLOG("spdk_fc_app_read_parameters() failed\n");
		return -1;
	}

	rc = spdk_fc_initialize_all_pools();
	if (rc != 0) {
		SPDK_ERRLOG("spdk_initialize_all_pools() failed\n");
		return -1;
	}

	rc = ocsu_init();
	if (rc < 0) {
		SPDK_ERRLOG("ocsu_init() failed\n");
		goto fail;
	}

	return 0;

fail:
	spdk_fc_free_all_pools();

	return rc;
}

int
spdk_fc_subsystem_fini(void)
{

	spdk_fc_shutdown();

	spdk_fc_free_all_pools();

	return(spdk_fc_ini_di_shutdown());

}

void
spdk_fc_config_text(FILE *fp)
{
	/* Display running configuration. */
}

//SPDK_SUBSYSTEM_REGISTER(fc, spdk_fc_subsystem_init, spdk_fc_subsystem_fini,
//	spdk_fc_config_text)
//SPDK_SUBSYSTEM_DEPEND(fc, scsi)
//SPDK_SUBSYSTEM_DEPEND(fc, bdev)
