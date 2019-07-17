#include "ocs.h"
#include "ocs_os.h"
#include "ocs_scsi.h"
#include "scsi_cmds.h"
#include "fc.h"

int32_t
ocs_scsi_tgt_driver_init(void)
{
	return 0;
}

int32_t
ocs_scsi_tgt_driver_exit(void)
{
	return 0;
}

int32_t
ocs_scsi_validate_initiator(ocs_node_t *node)
{
	return 0;
}

int32_t
ocs_scsi_new_initiator(ocs_node_t *node)
{
	return 0;
}

int32_t
ocs_scsi_del_initiator(ocs_node_t *node, ocs_scsi_del_initiator_reason_e reason)
{
	return 0;
}

int32_t
ocs_scsi_recv_cmd(ocs_io_t *io, uint32_t lun, uint8_t *cdb, uint32_t cdb_len, uint32_t flags)
{
	return 0;
}

int32_t
ocs_scsi_recv_cmd_first_burst(ocs_io_t *io, uint32_t lun, uint8_t *cdb, uint32_t cdb_len, uint32_t flags,
	ocs_dma_t first_burst_buffers[], uint32_t first_burst_bytes)
{
	return 0;
}

int32_t
ocs_scsi_recv_tmf(ocs_io_t *tmfio, uint64_t lun, ocs_scsi_tmf_cmd_e cmd, ocs_io_t *abortio,
	uint32_t flags)
{
	return 0;
}

void
ocs_scsi_tgt_ddump(ocs_textbuf_t *textbuf, ocs_scsi_ddump_type_e type, void *obj)
{
}

int32_t
ocs_scsi_tgt_new_domain(ocs_domain_t *domain)
{
	return 0;
}

void
ocs_scsi_tgt_del_domain(ocs_domain_t *domain)
{
	return;
}
int32_t
ocs_scsi_tgt_new_device(ocs_t *ocs)
{
	return 0;
}

int32_t
ocs_scsi_tgt_del_device(ocs_t *ocs)
{
	return 0;
}

int32_t
ocs_scsi_tgt_new_sport(ocs_sport_t *sport)
{
	return 0;
}

void
ocs_scsi_tgt_del_sport(ocs_sport_t *sport)
{
	/* No SCSI SPDK call to delete port. */
}	

int32_t
ocs_scsi_alloc_task_pool(struct rte_mempool **task_pool)
{
	/* create scsi_task pool */
	*task_pool = NULL;
	return 0;
}

void
spdk_fc_cf_cleanup_cfg(void)
{
}

struct spdk_fc_hba_port *
ocs_spdk_tgt_find_hba_port(int instance)
{
	return NULL;
}
