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

#if !defined(__OCSU_SCSI_IF_H__)
#define __OCSU_SCSI_IF_H__

#include "ocs_os.h"

#include "spdk/scsi.h"
#include "scsi_internal.h"

#undef __USE_SPDK_SCSI_LIB__

#ifdef __USE_SPDK_SCSI_LIB__
#define spdk_scsi_task_put			spdk_scsi_task_put
#define spdk_scsi_dev_destruct			spdk_scsi_dev_destruct
#define spdk_scsi_lun_construct			spdk_scsi_lun_construct
#define spdk_scsi_dev_add_port			spdk_scsi_dev_add_port
#define spdk_scsi_dev_delete_lun		spdk_scsi_dev_delete_lun
#define spdk_scsi_port_create			spdk_scsi_port_create
#define spdk_scsi_dev_queue_mgmt_task		spdk_scsi_dev_queue_mgmt_task
#define spdk_scsi_dev_get_lun			spdk_scsi_dev_get_lun
#define spdk_scsi_dev_allocate_io_channels	spdk_scsi_dev_allocate_io_channels
#define spdk_scsi_dev_free_io_channels		spdk_scsi_dev_free_io_channels
#define spdk_scsi_dev_construct			spdk_scsi_dev_construct
#define spdk_scsi_task_process_null_lun		spdk_scsi_task_process_null_lun
#define spdk_scsi_dev_find_port_by_id		spdk_scsi_dev_find_port_by_id
#define spdk_scsi_task_construct		spdk_scsi_task_construct
#define spdk_scsi_dev_queue_task		spdk_scsi_dev_queue_task

#else

#define spdk_scsi_task_put			ocs_spdk_scsi_task_put
#define spdk_scsi_dev_destruct			ocs_spdk_scsi_dev_destruct
#define spdk_scsi_lun_construct			ocs_spdk_scsi_lun_construct
#define spdk_scsi_dev_add_port			ocs_spdk_scsi_dev_add_port
#define spdk_scsi_dev_delete_lun		ocs_spdk_scsi_dev_delete_lun
#define spdk_scsi_port_create			ocs_spdk_scsi_port_create
#define spdk_scsi_dev_queue_mgmt_task		ocs_spdk_scsi_dev_queue_mgmt_task
#define spdk_scsi_dev_get_lun			ocs_spdk_scsi_dev_get_lun
#define spdk_scsi_dev_allocate_io_channels	ocs_spdk_scsi_dev_allocate_io_channels
#define spdk_scsi_dev_free_io_channels		ocs_spdk_scsi_dev_free_io_channels
#define spdk_scsi_dev_construct			ocs_spdk_scsi_dev_construct
#define spdk_scsi_task_process_null_lun		ocs_spdk_scsi_task_process_null_lun
#define spdk_scsi_dev_find_port_by_id		ocs_spdk_scsi_dev_find_port_by_id
#define spdk_scsi_task_construct		ocs_spdk_scsi_task_construct
#define spdk_scsi_dev_queue_task		ocs_spdk_scsi_dev_queue_task

static inline void ocs_spdk_log_unexpected_call(const char *fname)
{
	ocs_log_err(NULL, "invalid spdk scsi api called, %s\n", fname);
}

static inline void ocs_spdk_scsi_task_put(struct spdk_scsi_task *task)
{
	ocs_spdk_log_unexpected_call(__func__);
}

static inline void ocs_spdk_scsi_dev_destruct(struct spdk_scsi_dev *dev,
				spdk_scsi_dev_destruct_cb_t cb_fn, void *cb_arg)
{
	ocs_spdk_log_unexpected_call(__func__);
}

static inline struct spdk_scsi_lun *ocs_spdk_scsi_lun_construct(
				struct spdk_bdev *bdev,
				void (*hotremove_cb)(const struct spdk_scsi_lun *, void *),
				void *hotremove_ctx)
{
	ocs_spdk_log_unexpected_call(__func__);

	return NULL;
}

static inline int ocs_spdk_scsi_dev_add_port(struct spdk_scsi_dev *dev, uint64_t id, const char *name)
{
	ocs_spdk_log_unexpected_call(__func__);

	return 0;
}

static inline void ocs_spdk_scsi_dev_delete_lun(struct spdk_scsi_dev *dev, struct spdk_scsi_lun *lun)
{
}

static inline struct spdk_scsi_port *ocs_spdk_scsi_port_create(uint64_t id, uint16_t index, const char *name)
{
	ocs_spdk_log_unexpected_call(__func__);

	return NULL;
}

static inline void ocs_spdk_scsi_dev_queue_mgmt_task(struct spdk_scsi_dev *dev, struct spdk_scsi_task *task)
{
	ocs_spdk_log_unexpected_call(__func__);
}

static inline struct spdk_scsi_lun *ocs_spdk_scsi_dev_get_lun(struct spdk_scsi_dev *dev, int lun_id)
{
	ocs_spdk_log_unexpected_call(__func__);

	return NULL;
}

static inline int ocs_spdk_scsi_dev_allocate_io_channels(struct spdk_scsi_dev *dev)
{
	ocs_spdk_log_unexpected_call(__func__);

	return 0;
}

static inline void ocs_spdk_scsi_dev_free_io_channels(struct spdk_scsi_dev *dev)
{
	ocs_spdk_log_unexpected_call(__func__);
}

static inline struct spdk_scsi_dev *ocs_spdk_scsi_dev_construct(const char *name,
				const char *bdev_name_list[], int *lun_id_list,
				int num_luns, uint8_t protocol_id,
				void (*hotremove_cb)(const struct spdk_scsi_lun *, void *),
				void *hotremove_ctx)
{
	ocs_spdk_log_unexpected_call(__func__);

	return NULL;
}

static inline void ocs_spdk_scsi_task_process_null_lun(struct spdk_scsi_task *task)
{
	ocs_spdk_log_unexpected_call(__func__);
}

static inline struct spdk_scsi_port *ocs_spdk_scsi_dev_find_port_by_id(struct spdk_scsi_dev *dev, uint64_t id)
{
	ocs_spdk_log_unexpected_call(__func__);

	return NULL;
}

static inline void ocs_spdk_scsi_task_construct(struct spdk_scsi_task *task,
                              spdk_scsi_task_cpl cpl_fn,
                              spdk_scsi_task_free free_fn)
{
	ocs_spdk_log_unexpected_call(__func__);
}

static inline void ocs_spdk_scsi_dev_queue_task(struct spdk_scsi_dev *dev, struct spdk_scsi_task *task)
{
	ocs_spdk_log_unexpected_call(__func__);
}
#endif

#endif
