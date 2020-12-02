/*
 * Copyright (C) 2020 Broadcom. All Rights Reserved.
 * The term “Broadcom” refers to Broadcom Inc. and/or its subsidiaries.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 * Fabric Emulation
 *
 */

#include "ocs.h"
#include "ocs_femul.h"
#include "ocs_fabric.h"
#include "ocs_els.h"
#include "ocs_scsi_fc.h"
#include "ocs_device.h"

#if defined(ENABLE_FABRIC_EMULATION)

/* Name Services Database API */
struct ocs_ns_record_s {
	uint32_t active:1,
		 scr_requested:1;
	uint32_t port_id;
	uint32_t fc4_types;
	uint8_t	fc4_features;
	uint8_t type_code;
	uint64_t node_name;
	uint64_t port_name;
	char *sym_node_name;
	uint32_t sym_node_name_len;
	uint32_t class_of_srvc;
	ocs_node_t *node;
};

void *__ocs_sport_femul_fabric_idle(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
void *__ocs_node_femul_fabric_idle(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

static int32_t ocs_femul_process_rft_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
static int32_t ocs_femul_process_gnn_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
static int32_t ocs_femul_process_gpn_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
static int32_t ocs_femul_process_gfpn_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
static int32_t ocs_femul_process_gff_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
static int32_t ocs_femul_process_gid_ft(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
static int32_t ocs_femul_process_gid_pt(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
static int32_t ocs_femul_process_ga_nxt(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
static int32_t ocs_femul_process_rff_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
static int32_t ocs_femul_process_rpn_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
static int32_t ocs_femul_process_rnn_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
static int32_t ocs_femul_process_rcs_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
static int32_t ocs_femul_process_rsnn_nn(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
static int32_t ocs_femul_process_rspn_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
static int32_t ocs_femul_process_rhba(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
static int32_t ocs_femul_process_rpa(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);

static int32_t ocs_ns_event_add(ocs_ns_t *ns, uint32_t port_id);
static void ocs_ns_event_clear(ocs_ns_t *ns);

static void ocs_ns_rscn_timeout(void *arg);

#define OCS_FEMUL_PORTID_BASE		0x20100
#define OCS_FEMUL_NUM_PORTID		256

struct ocs_ns_s {
	ocs_domain_t *domain;
	uint32_t ns_record_count;
	ocs_ns_record_t *ns_records;
	ocs_lock_t ns_event_lock;
	uint32_t *ns_event_list;
	uint32_t ns_event_list_len;
	uint32_t ns_event_list_count;
	ocs_timer_t rscn_timer;
};


/**
 * @brief Allocate a port ID from the port_id pool
 *
 * A port ID is allocated from the domain's port_id pool
 *
 * @param domain Pointer to the domain object
 *
 * @return returns positive port ID value, or negative value for an error
 */
int32_t
ocs_femul_portid_alloc(ocs_domain_t *domain)
{
	if (domain->portid_pool == NULL) {
		return -1;
	}

	return ocs_bitmap_find(domain->portid_pool, OCS_FEMUL_NUM_PORTID) + OCS_FEMUL_PORTID_BASE;
}

/**
 * @brief Free a port ID
 *
 * Free the previously allocated port ID
 *
 * @param domain Pointer to domain object
 * @param portid Port ID to free
 *
 * @return none
 */
void
ocs_femul_portid_free(ocs_domain_t *domain, int32_t portid)
{
	int32_t bitnum = portid - OCS_FEMUL_PORTID_BASE;

	ocs_assert((bitnum >= 0) && (bitnum < OCS_FEMUL_NUM_PORTID));

	if (domain->portid_pool != NULL) {
		ocs_bitmap_clear(domain->portid_pool, portid - OCS_FEMUL_PORTID_BASE);
	}
}

/**
 * @brief Initialize Fabric Emulation subsystem
 *
 * The Fabric Emulation subsystem is initialized, which includes allocating of the
 * port ID pool, as well as instantiating ports for the Fabric Port controller,
 * the Fabric Controller, and the Name services controller,
 *
 * @param domain Pointer to domain object
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocs_femul_init(ocs_domain_t *domain)
{
	/* Initialize Port ID allocation pool */
	domain->portid_pool = ocs_bitmap_alloc(OCS_FEMUL_NUM_PORTID);

	ocs_atomic_init(&domain->femul_count, 0);

	return 0;
}


/**
 * @brief Shut down Fabric Emulation
 *
 * The Fabric Emulation is shut down.   This includes freeing port ID allocation
 * bitmap
 *
 * @param domain Pointer to domain object
 *
 * @return returns 0
 */
int32_t
ocs_femul_shutdown(ocs_domain_t *domain)
{
	ocs_bitmap_free(domain->portid_pool);
	return 0;
}

/**
 * @brief Process FLOGI while in Fabric Emulation mode
 *
 * An inbound FLOGI request is processed while in Fabric Emulation mode.   In this
 * case a port ID is allocated and returned to the requestor, with service
 * parameters indicating that this node is an F_port.
 *
 * @param io Pointer to an IO object
 * @param hdr Pointer to the FC header
 * @param payload Pointer to payload buffer (service parameters)
 * @param payload_len Length of payload buffer in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocs_femul_process_flogi(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	ocs_t *ocs = io->ocs;
	ocs_domain_t *domain = io->node->sport->domain;
	fc_plogi_payload_t *flogi = payload;
	uint64_t port_name = ocs_be64toh(((long)flogi->port_name_hi << 32 |
					  flogi->port_name_lo));
	int32_t portid;
	ocs_ns_record_t *nsrec;

	/*
	 * handle the case where we may receive another FLOGI. Check the name
	 * server DB for the port name (from the FLOGI). If it exists, then
	 * just send the FLOGI accept.
	 */
	nsrec = ocs_ns_find_port_name(domain->ocs_ns, port_name);
	if (nsrec != NULL) {
		ocs_log_debug(ocs, "Found existing NS record, send FLOGI acc\n");
		ocs_send_flogi_acc(io, ocs_be16toh(hdr->ox_id), TRUE, NULL, NULL);
		return 0;
	}

	/* Allocate the physical port id */
	portid = ocs_femul_portid_alloc(domain);
	if (portid < 0) {
		ocs_log_err(ocs, "ocs_femul_portid_alloc() failed: %d\n", portid);
		return -1;
	}

	/* Save OX_ID for FLOGI accecpt */
	domain->femul_oxid = ocs_be16toh(hdr->ox_id);

	/* Save port_name for NS database accecpt */
	domain->femul_port_name = port_name;

	/*
	 * During cable pull events, we may get a second flogi request. So see if the
	 * fabric vports have been created, to prevent creating duplicates.
	 */
	if (!domain->femul_vports_started) {
		domain->femul_vports_started = TRUE;
		/*
		 * Attach the domain with the local physical port
		 *
		 * Note: Use the fabric controller address for the physical port
		 *       so that any traffic to the fabric controller is not discarded
		 *       before the vport is registered.
		 */
		__ocs_domain_attach_internal(domain, portid);

		/* Create F_Port controller, Fabric controller, and Directory server (name server) */
		ocs_atomic_add_return(&domain->femul_count, 4);
		ocs_sport_vport_new(domain, 0ll, 0ll, FC_ADDR_FABRIC, FALSE, FALSE, NULL, NULL, FALSE);
		ocs_sport_vport_new(domain, 0ll, 0ll, FC_ADDR_NAMESERVER, FALSE, FALSE, NULL, NULL, FALSE);
		ocs_sport_vport_new(domain, 0ll, 0ll, FC_ADDR_CONTROLLER, FALSE, FALSE, NULL, NULL, FALSE);
	}

	/* Free the IO that was passed in */
	ocs_els_io_free(io);

	return 0;
}

/**
 * @brief Process FDISC while in Fabric Emulation mode
 *
 * An inbound FDISC request is processed while in Fabric Emulation mode.   In this
 * case a port ID is allocated and returned to the requestor, with service
 * parameters indicating that this node is an F_port.
 *
 * @param io Pointer to an IO object
 * @param hdr Pointer to the FC header
 * @param payload Pointer to payload buffer (service parameters)
 * @param payload_len Length of payload buffer in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocs_femul_process_fdisc(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
        ocs_t *ocs = io->ocs;
        int32_t portid;
        ocs_node_t *rspnode;
        ocs_io_t *rspio;

	/* Allocate the physical port id */
	portid = ocs_femul_portid_alloc(io->node->sport->domain);
	if (portid < 0) {
		ocs_log_err(ocs, "ocs_femul_portid_alloc() failed: %d\n", portid);
		return -1;
	}

        /* Allocate a node */
        rspnode = ocs_node_alloc(io->node->sport, portid, FALSE, FALSE);
        if (rspnode == NULL) {
                ocs_log_err(ocs, "ocs_node_alloc failed\n");
                return -1;
        }
        ocs_node_transition(rspnode, __ocs_d_init, NULL);

	/* Allocate an IO for the response */
        rspio = ocs_els_io_alloc(rspnode, sizeof(fc_plogi_payload_t), OCS_ELS_ROLE_RESPONDER);
        if (rspio == NULL) {
                ocs_log_err(ocs, "OCS_EVT_FDISC_RCVD: ocs_els_io_alloc failed\n");
                ocs_node_free(rspnode);
                return -1;
        }

	/* Send response */
        ocs_send_flogi_acc(rspio, ocs_be16toh(hdr->ox_id), TRUE, NULL, NULL);

        /* Free the IO that was passed in */
        ocs_els_io_free(io);

	return 0;
}

/**
 * @brief Process sport attached event
 *
 * Called when a sport is attached. This function will create a name server
 * entry for the sport.
 *
 * @param sport Pointer to the sport object
 */
void
ocs_femul_sport_attach(ocs_sport_t *sport)
{
        ocs_t *ocs = sport->ocs;
	ocs_domain_t *domain = sport->domain;
	ocs_ns_record_t *nsrec;

	if (sport->fc_id == FC_ADDR_FABRIC ||
	    sport->fc_id == FC_ADDR_NAMESERVER ||
	    sport->fc_id == FC_ADDR_CONTROLLER) {
		return;
	}

	nsrec = ocs_ns_alloc(domain->ocs_ns, sport->fc_id);
	if (nsrec == NULL) {
		ocs_log_err(ocs, "failed to allocate nsrec for fc_id 0x%x\n", sport->fc_id);
		return;
	}

	nsrec->fc4_types = (1 << FC_GS_TYPE_BIT(FC_TYPE_FCP));
	nsrec->fc4_features = ((sport->enable_ini ? FC4_FEATURE_INITIATOR : 0) |
			       (sport->enable_tgt ? FC4_FEATURE_TARGET : 0));
	nsrec->type_code = FC_TYPE_FCP;
	nsrec->node_name = sport->wwnn;
	nsrec->port_name = sport->wwpn;
	nsrec->class_of_srvc = FCCT_CLASS_OF_SERVICE_2;
}

/**
 * @brief Process fabric emulation ports attached event
 *
 * Called with all of the well known port attaches have completed.  At this point
 * the FLOGI accept is sent to the direct connect port.
 *
 * @param domain Pointer to the domain object
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t ocs_femul_ports_attached(ocs_domain_t *domain)
{
	ocs_t *ocs;
	ocs_sport_t *fabric;
	int32_t fc_id;
	ocs_node_t *node;
	ocs_io_t *rspio;
	ocs_ns_record_t *nsrec;

	ocs_assert(domain != NULL, -1);

	ocs = domain->ocs;
	ocs_assert(ocs != NULL, -1);
	ocs_log_debug(ocs, "all sports attached\n");

	fabric = ocs_sport_find(domain, FC_ADDR_FABRIC);
	if (fabric == NULL) {
		ocs_log_warn(ocs, "ocs_sport_find(FC_ADD_FABRIC) failed\n");
		return -1;
	}

	/* Assign an FC_ID for the FLOGI node */
	fc_id = ocs_femul_portid_alloc(domain);

	/* Allocate a node */
	node = ocs_node_alloc(fabric, fc_id, 0, 0);
	if (node == NULL) {
		ocs_log_err(ocs, "ocs_node_alloc() failed\n");
		ocs_femul_portid_free(domain, fc_id);
		return -1;
	}
	ocs_node_transition(node, __ocs_d_init, NULL);

	/* Allocate an IO */
	rspio = ocs_els_io_alloc(node, sizeof(fc_plogi_payload_t), OCS_ELS_ROLE_RESPONDER);
	if (rspio == NULL) {
		ocs_log_err(ocs, "ocs_els_io_alloc() failed\n");
		ocs_node_free(node);
		ocs_femul_portid_free(domain, fc_id);
		return -1;
	}

	/*
	 * Set the port name that was sent of the flogi in the name server db.
	 *
	 * Note: This must be done before sending the FLOGI accept in case the
	 *       initiator sends another FLOGI. We need to find the name server
	 *       entry so we can just send FLOGI-ACC and not create any more
	 *       vports
	 */
	nsrec = ocs_ns_alloc(domain->ocs_ns, fc_id);
	if (nsrec != NULL) {
		nsrec->port_name = domain->femul_port_name;
	}

	/* send FLOGI response */
	ocs_send_flogi_acc(rspio, domain->femul_oxid, TRUE, NULL, NULL);

	return 0;
}

/**
 * @brief Process and RFT_ID FCGS request
 *
 * An RFT_ID request is processed while in Fabric Emulation mode.   In this case, the
 * payload contains the Port ID and fc4_typess of the requester which is added
 * to the directory services database.
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocs_femul_process_rft_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	fcgs_rft_id_t *rftid = payload;
	ocs_ns_record_t *nsrec;
	uint32_t port_id = ocs_be32toh(rftid->port_id);

	node_printf(io->node, "port_id x%x fc4_types x%x\n", ocs_be32toh(rftid->port_id), ocs_be32toh(rftid->fc4_types));

	/* Allocate a name services record */
	nsrec = ocs_ns_alloc(io->node->sport->domain->ocs_ns, port_id);
	if (nsrec == NULL) {
		ocs_log_err(io->ocs, "ocs_ns_alloc failed\n");
		return -1;
	}

	/* Update the fc4_types */
	nsrec->fc4_types = ocs_be32toh(rftid->fc4_types);

	return ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_ACCEPT, 0, 0);
}

/**
 * @brief Process RFF_ID request
 *
 * The RFF_ID request is processed.
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
ocs_femul_process_rff_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	fcgs_rff_id_t *rffid = payload;
	uint32_t port_id = ocs_be32toh(rffid->port_id);
	ocs_ns_t *ns = io->node->sport->domain->ocs_ns;
	ocs_ns_record_t *nsrec;

	nsrec = ocs_ns_alloc(ns, port_id);
	if (nsrec == NULL) {
		ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_REJECT, FCCT_UNABLE_TO_PERFORM,
			FCCT_DATA_BASE_FULL);
		return -1;
	}
	/* Update fc4_features and type_code */
	nsrec->fc4_features = rffid->fc4_features;
	nsrec->type_code = rffid->type_code;

	return ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_ACCEPT, 0, 0);
}

/**
 * @brief Process RPN_ID request
 *
 * The RPN_ID request is processed.
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
ocs_femul_process_rpn_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	fcgs_rpn_id_t *rpnid = payload;
	uint32_t port_id = ocs_be32toh(rpnid->port_id);
	ocs_ns_t *ns = io->node->sport->domain->ocs_ns;
	ocs_ns_record_t *nsrec;

	nsrec = ocs_ns_alloc(ns, port_id);
	if (nsrec == NULL) {
		ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_REJECT, FCCT_UNABLE_TO_PERFORM,
			FCCT_DATA_BASE_FULL);
		return -1;
	}

	/* Update port name */
	nsrec->port_name = ocs_be64toh(rpnid->port_name);

	return ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_ACCEPT, 0, 0);
}

/**
 * @brief Process RNN_ID request
 *
 * The RNN_ID request is processed.
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
ocs_femul_process_rnn_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	fcgs_rnn_id_t *rnnid = payload;
	uint32_t port_id = ocs_be32toh(rnnid->port_id);
	ocs_ns_t *ns = io->node->sport->domain->ocs_ns;
	ocs_ns_record_t *nsrec;

	nsrec = ocs_ns_alloc(ns, port_id);
	if (nsrec == NULL) {
		ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_REJECT, FCCT_UNABLE_TO_PERFORM,
			FCCT_DATA_BASE_FULL);
		return -1;
	}

	/* Update node name */
	nsrec->node_name = ocs_be64toh(rnnid->node_name);

	return ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_ACCEPT, 0, 0);
}

/**
 * @brief Process RCS_ID request
 *
 * The RCS_ID request is processed.
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
ocs_femul_process_rcs_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	fcgs_rcs_id_t *rcsid = payload;
	uint32_t port_id = ocs_be32toh(rcsid->port_id);
	ocs_ns_t *ns = io->node->sport->domain->ocs_ns;
	ocs_ns_record_t *nsrec;

	nsrec = ocs_ns_alloc(ns, port_id);
	if (nsrec == NULL) {
		ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_REJECT, FCCT_UNABLE_TO_PERFORM,
			FCCT_DATA_BASE_FULL);
		return -1;
	}

	/* Update node name */
	nsrec->class_of_srvc = ocs_be32toh(rcsid->class_of_srvc);

	return ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_ACCEPT, 0, 0);
}

/**
 * @brief Process RSNN_NN request
 *
 * The register symbolic node name (RSNN_NN) request is processed.
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
ocs_femul_process_rsnn_nn(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	fcgs_rsnn_nn_t *rsnnnn = payload;
	uint64_t node_name = ocs_be64toh(rsnnnn->node_name);
	ocs_ns_t *ns = io->node->sport->domain->ocs_ns;
	ocs_ns_record_t *nsrec;
	ocs_t *ocs = io->ocs;

	for (nsrec = NULL; (nsrec = ocs_ns_enumerate(ns, nsrec)) != NULL; ) {
		if (nsrec->node_name == node_name) {
			break;
		}
	}

	if (nsrec == NULL) {
		ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_REJECT, FCCT_UNABLE_TO_PERFORM,
			FCCT_DATA_BASE_FULL);
		return -1;
	}

	nsrec->sym_node_name = ocs_malloc(ocs, rsnnnn->name_len + 1, OCS_M_ZERO);
	if (nsrec->sym_node_name == NULL) {
		ocs_log_err(io->ocs, "ocs_malloc sym_node_name failed\n");
		ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_REJECT, FCCT_UNABLE_TO_PERFORM,
			FCCT_NO_ADDITIONAL_EXPLANATION);
		return -1;
	}
	nsrec->sym_node_name_len = rsnnnn->name_len + 1;
	ocs_memcpy(nsrec->sym_node_name, rsnnnn->sym_node_name, rsnnnn->name_len);

	/* Extract symbolic node name */

	return ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_ACCEPT, 0, 0);
}

/**
 * @brief Process RSPN_ID request
 *
 * The RSPN_ID request is processed.
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
ocs_femul_process_rspn_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	return ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_ACCEPT, 0, 0);
}

/**
 * @brief Process RHBA request
 *
 * The RHBA request is processed.
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
ocs_femul_process_rhba(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	/* Just send an accept - data is not stored */
	return ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_ACCEPT, 0, 0);
}

/**
 * @brief Process RPA request
 *
 * The RPA request is processed.
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
ocs_femul_process_rpa(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	/* Just send an accept - data is not stored */
	return ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_ACCEPT, 0, 0);
}

/**
 * @brief Process GNN_ID command
 *
 * Process the GNN_ID request
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
ocs_femul_process_gnn_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	fcct_gnnid_req_t *gnnid = payload;
	uint32_t port_id = fc_be24toh(gnnid->port_id);
	fcct_gnnid_acc_t *acc = io->els_info->els_rsp.virt;
	ocs_ns_record_t *nsrec = NULL;
	uint32_t cmd_rsp_code = FCCT_HDR_CMDRSP_ACCEPT;
	uint32_t reason_code = 0;
	uint32_t reason_code_explanation = 0;

	/* Find the record for the port */
	nsrec = ocs_ns_find_port_id(io->node->sport->domain->ocs_ns, port_id);
	if (nsrec != NULL) {
		acc->node_name = ocs_htobe64(nsrec->node_name);
		io->wire_len += sizeof(acc->node_name);
	} else {
		ocs_log_test(io->ocs, "port_id 0x%x not found\n", port_id);
		cmd_rsp_code = FCCT_HDR_CMDRSP_REJECT;
		reason_code = FCCT_UNABLE_TO_PERFORM;
		reason_code_explanation = FCCT_DEVICES_NOT_IN_COMMON_ZONE;
	}

	return ocs_send_ct_rsp(io, hdr->ox_id, payload, cmd_rsp_code,
			       reason_code, reason_code_explanation);
}

/**
 * @brief Process GPN_ID command
 *
 * Process the GPN_ID request
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
ocs_femul_process_gpn_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	fcct_gpnid_req_t *gpnid = payload;
	uint32_t port_id = fc_be24toh(gpnid->port_id);
	fcct_gpnid_acc_t *acc = io->els_info->els_rsp.virt;
	ocs_ns_record_t *nsrec = NULL;
	uint32_t cmd_rsp_code = FCCT_HDR_CMDRSP_ACCEPT;
	uint32_t reason_code = 0;
	uint32_t reason_code_explanation = 0;

	/* Find the record for the port */
	nsrec = ocs_ns_find_port_id(io->node->sport->domain->ocs_ns, port_id);
	if (nsrec != NULL) {
		acc->port_name = ocs_htobe64(nsrec->port_name);
		io->wire_len += sizeof(acc->port_name);
	} else {
		ocs_log_test(io->ocs, "port_id 0x%x not found\n", port_id);
		cmd_rsp_code = FCCT_HDR_CMDRSP_REJECT;
		reason_code = FCCT_UNABLE_TO_PERFORM;
		reason_code_explanation = FCCT_DEVICES_NOT_IN_COMMON_ZONE;
	}

	return ocs_send_ct_rsp(io, hdr->ox_id, payload, cmd_rsp_code,
			       reason_code, reason_code_explanation);
}

/**
 * @brief Process GFPN_ID command
 *
 * Process the GFPN_ID request
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
ocs_femul_process_gfpn_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	return ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_REJECT,
			       FCCT_FABRIC_PORT_NAME_NOT_REGISTERED, 0);
}

/**
 * @brief Process GFF_ID command
 *
 * Process the GFF_ID request
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
ocs_femul_process_gff_id(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	fcct_gffid_req_t *gffid = payload;
	uint32_t port_id = fc_be24toh(gffid->port_id);
	fcct_gffid_acc_t *acc = io->els_info->els_rsp.virt;
	ocs_ns_record_t *nsrec = NULL;
	uint32_t cmd_rsp_code = FCCT_HDR_CMDRSP_ACCEPT;
	uint32_t reason_code = 0;
	uint32_t reason_code_explanation = 0;

	/* Find the record for the port */
	nsrec = ocs_ns_find_port_id(io->node->sport->domain->ocs_ns, port_id);
	if (nsrec != NULL) {
		acc->fc4_feature_bits = nsrec->fc4_features;
		io->wire_len += sizeof(acc->fc4_feature_bits);
	} else {
		ocs_log_test(io->ocs, "port_id 0x%x not found\n", port_id);
		cmd_rsp_code = FCCT_HDR_CMDRSP_REJECT;
		reason_code = FCCT_UNABLE_TO_PERFORM;
		reason_code_explanation = FCCT_DEVICES_NOT_IN_COMMON_ZONE;
	}

	return ocs_send_ct_rsp(io, hdr->ox_id, payload, cmd_rsp_code,
			       reason_code, reason_code_explanation);
}

/**
 * @brief Process GID_FT command
 *
 * Process the GID_FT request
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
ocs_femul_process_gid_ft(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	fcct_gidft_acc_t *gidft = io->els_info->els_rsp.virt;
	uint32_t reqid = fc_be24toh(hdr->s_id);
	uint32_t idx;
	uint32_t max_entries;
	ocs_ns_record_t *nsrec = NULL;

	/* Compute maximum entries, given the response payload size */
	max_entries = (io->els_info->els_rsp.size - sizeof(gidft->hdr)) / sizeof(uint32_t);

	/* enumerate all nodes that are in the domain to which the requestor belongs */
	idx = 0;
	for (nsrec = ocs_ns_enumerate(io->node->sport->domain->ocs_ns, nsrec); nsrec != NULL;
		nsrec = ocs_ns_enumerate(io->node->sport->domain->ocs_ns, nsrec)) {

		if (idx >= max_entries) {
			ocs_log_test(io->ocs, "overflowed GID_FT response buffer\n");
			break;
		}
		/* Skip this entry if it blocks to the requestor */
		if (nsrec->port_id == reqid) {
			continue;
		}
		gidft->port_list[idx].ctl = 0;
		gidft->port_list[idx].port_id = fc_htobe24(nsrec->port_id);
		io->wire_len += sizeof(uint32_t);
		idx++;
	}

	gidft->port_list[idx > 0 ? idx-1 : idx].ctl = 0x80;

	return ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_ACCEPT, 0, 0);
}

/**
 * @brief Process GID_PT command
 *
 * Process the GID_PT request
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
ocs_femul_process_gid_pt(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	fcct_gidpt_acc_t *gidpt = io->els_info->els_rsp.virt;
	uint32_t reqid = fc_be24toh(hdr->s_id);
	uint32_t idx;
	uint32_t max_entries;
	ocs_ns_record_t *nsrec = NULL;

	/* Compute maximum entries, given the response payload size */
	max_entries = (io->els_info->els_rsp.size - sizeof(gidpt->hdr)) / sizeof(uint32_t);

	/* enumerate all nodes that are in the domain to which the requestor belongs */
	idx = 0;
	for (nsrec = ocs_ns_enumerate(io->node->sport->domain->ocs_ns, nsrec); nsrec != NULL;
		nsrec = ocs_ns_enumerate(io->node->sport->domain->ocs_ns, nsrec)) {

		if (idx >= max_entries) {
			ocs_log_test(io->ocs, "overflowed GID_PT response buffer\n");
			break;
		}
		/* Skip this entry if it blocks to the requestor */
		if (nsrec->port_id == reqid) {
			continue;
		}
		gidpt->port_list[idx].ctl = 0;
		gidpt->port_list[idx].port_id = fc_htobe24(nsrec->port_id);
		io->wire_len += sizeof(uint32_t);
		idx++;
	}

	gidpt->port_list[idx > 0 ? idx-1 : idx].ctl = 0x80;

	return ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_ACCEPT, 0, 0);
}

/**
 * @brief Process GA_NXT command
 *
 * Process the GA_NXT request
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
ocs_femul_process_ga_nxt(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	fcct_ganxt_req_t *ganxt_req = payload;
	fcct_ganxt_acc_t *ganxt_resp = io->els_info->els_rsp.virt;
	uint32_t port_id = ganxt_req->port_id;
	ocs_ns_record_t *nsrec = NULL, *tmp_nsrec = NULL;

	ocs_memset(io->els_info->els_rsp.virt, 0, sizeof(ganxt_resp));
	while ((nsrec = ocs_ns_enumerate(io->node->sport->domain->ocs_ns, nsrec)) != NULL) {
		if (nsrec->port_id > port_id) {
			if (!tmp_nsrec || (tmp_nsrec->port_id > nsrec->port_id))
				tmp_nsrec = nsrec;
		}
	}

	if (tmp_nsrec) {
		ganxt_resp->port_id 		= tmp_nsrec->port_id;
		ganxt_resp->port_name	 	= tmp_nsrec->port_name;
		ganxt_resp->sym_node_name_len 	= tmp_nsrec->sym_node_name_len;

		ocs_memcpy(ganxt_resp->sym_node_name, tmp_nsrec->sym_node_name, MIN(tmp_nsrec->sym_node_name_len, 255));

		ganxt_resp->class_of_serv	= tmp_nsrec->class_of_srvc;
		ganxt_resp->node_name	 	= tmp_nsrec->node_name;

		return ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_ACCEPT, 0, 0);
	}

	ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_REJECT, FCCT_UNABLE_TO_PERFORM,
			FCCT_NO_ADDITIONAL_EXPLANATION);
	return -1;
}

/**
 * @brief Process SCR request
 *
 * Process the SCR request
 *
 * @param io Pointer to IO object
 * @param hdr Pointer to FC header
 * @param payload Address of payload
 * @param payload_len Length of payload in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocs_femul_process_scr(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	ocs_ns_t *ns = io->node->sport->domain->ocs_ns;
	uint32_t port_id = fc_be24toh(hdr->s_id);
	ocs_ns_record_t *nsrec;

	nsrec = ocs_ns_alloc(io->node->sport->domain->ocs_ns, port_id);

	if (nsrec == NULL) {
		ocs_log_test(io->ocs, "port_id %x not found\n", port_id);
		/* caller will send failure response */
		return -1;
	}
	nsrec->scr_requested = TRUE;
	nsrec->node = io->node;
	ocs_log_debug(io->ocs, "SCR accepted from %x\n", port_id);

	/* Clear the ns_event_list */
	ocs_ns_event_clear(ns);

	ocs_send_ls_acc(io, ocs_htobe16(hdr->ox_id), NULL, NULL);
	return 0;
}

/**
 * @brief Process Fabric Emulation FCGS request
 *
 * The requested FCGS command is processed
 *
 * @param funcname Name of function
 * @param io Pointer to IO object
 * @param evt Event to process
 * @param hdr Pointer to FC header
 * @param payload Address of payload buffer
 * @param payload_len Length of payload buffer in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

int32_t
ocs_femul_process_fc_gs(const char *funcname, ocs_io_t *io, ocs_sm_event_t evt, fc_header_t *hdr, void *payload, uint32_t payload_len)
{
	int32_t rc = -1;
	ocs_node_t *node = io->node;
	ocs_t *ocs = io->ocs;

	/*
	 * ocs_send_ct_rsp uses this to append the header size. Clear to 0
	 * and the individual functions can add whatever they need to the
	 * payload size.
	 */
	io->wire_len = 0;

	switch(evt) {
	case OCS_EVT_RFF_ID_RCVD:
		rc = ocs_femul_process_rff_id(io, hdr, payload, payload_len);
		break;
	case OCS_EVT_GNN_ID_RCVD:
		rc = ocs_femul_process_gnn_id(io, hdr, payload, payload_len);
		break;
	case OCS_EVT_GPN_ID_RCVD:
		rc = ocs_femul_process_gpn_id(io, hdr, payload, payload_len);
		break;
	case OCS_EVT_GFPN_ID_RCVD:
		rc = ocs_femul_process_gfpn_id(io, hdr, payload, payload_len);
		break;
	case OCS_EVT_GFF_ID_RCVD:
		rc = ocs_femul_process_gff_id(io, hdr, payload, payload_len);
		break;
	case OCS_EVT_GID_FT_RCVD:
		rc = ocs_femul_process_gid_ft(io, hdr, payload, payload_len);
		break;
	case OCS_EVT_GID_PT_RCVD:
		rc = ocs_femul_process_gid_pt(io, hdr, payload, payload_len);
		break;
	case OCS_EVT_GA_NXT_RCVD:
		rc = ocs_femul_process_ga_nxt(io, hdr, payload, payload_len);
		break;
	case OCS_EVT_RPN_ID_RCVD:
		rc = ocs_femul_process_rpn_id(io, hdr, payload, payload_len);
		break;
	case OCS_EVT_RNN_ID_RCVD:
		rc = ocs_femul_process_rnn_id(io, hdr, payload, payload_len);
		break;
	case OCS_EVT_RCS_ID_RCVD:
		rc = ocs_femul_process_rcs_id(io, hdr, payload, payload_len);
		break;
	case OCS_EVT_RSNN_NN_RCVD:
		rc = ocs_femul_process_rsnn_nn(io, hdr, payload, payload_len);
		break;
	case OCS_EVT_RSPN_ID_RCVD:
		rc = ocs_femul_process_rspn_id(io, hdr, payload, payload_len);
		break;
	case OCS_EVT_RFT_ID_RCVD:
		rc = ocs_femul_process_rft_id(io, hdr, payload, payload_len);
		break;
	case OCS_EVT_RHBA_RCVD:
		rc = ocs_femul_process_rhba(io, hdr, payload, payload_len);
		break;
	case OCS_EVT_RPA_RCVD:
		rc = ocs_femul_process_rpa(io, hdr, payload, payload_len);
		break;
	default: {
		ocs_log_test(ocs, "[%s] (%s) %s sending CT_REJECT\n",
			     node->display_name, funcname, ocs_sm_event_name(evt));
		ocs_send_ct_rsp(io, hdr->ox_id, payload, FCCT_HDR_CMDRSP_REJECT, FCCT_COMMAND_NOT_SUPPORTED, 0);
		break;
	}
	}
	return rc;

}

/**
 * @brief Attach the Name Services database
 *
 * Allocate the name services database
 *
 * @param domain Pointer to the domain object
 * @param max_ports Maximum number of ports to be supported by this name services database
 *
 * @return Pointer to name services datatbase object, or NULL if there is an error
 */
ocs_ns_t *
ocs_ns_attach(ocs_domain_t *domain, uint32_t max_ports)
{
	ocs_ns_t *ns;
	ocs_t *ocs = domain->ocs;

	ns = ocs_malloc(ocs, sizeof(*ns), OCS_M_ZERO);
	if (ns == NULL) {
		ocs_log_err(ocs, "ocs_malloc ns failed\n");
		return NULL;
	}
	ns->ns_records = ocs_malloc(ocs, sizeof(*ns->ns_records)*max_ports, OCS_M_ZERO);
	ns->ns_record_count = max_ports;
	if (ns->ns_records == NULL) {
		ocs_log_err(ocs, "ocs_malloc ns records failed\n");
		ocs_free(ocs, ns, sizeof(*ns));
		return NULL;
	}
	ocs_lock_init(ocs, &ns->ns_event_lock, "ns_event_list lock[%d]", domain->instance_index);
	ns->ns_event_list_len = sizeof(*ns->ns_event_list)*max_ports;
	ns->ns_event_list = ocs_malloc(ocs, sizeof(*ns->ns_event_list)*max_ports, OCS_M_ZERO);
	if (ns->ns_event_list == NULL) {
		ocs_log_err(ocs, "ocs_malloc ns_event_list failed\n");
		ocs_free(ocs, ns->ns_records, sizeof(*ns->ns_records)*max_ports);
		ocs_free(ocs, ns, sizeof(*ns));
		return NULL;
	}
	ns->ns_event_list_count = 0;

	ns->domain = domain;

	return ns;
}

/**
 * @brief Detach name services database
 *
 * The name services database is freed
 *
 * @param ns Pointer to name services database object
 *
 * @return none
 */
void
ocs_ns_detach(ocs_ns_t *ns)
{
	ocs_t *ocs = NULL; 
	ocs_ns_record_t *nsrec = NULL;

	if (!ns) {
		ocs_log_err(NULL, "invalid argument\n");
		return;
	}

	ocs = ns->domain->ocs;

	if (ocs_timer_pending(&ns->rscn_timer)) {
		ocs_del_timer(&ns->rscn_timer);
	}

	for (nsrec = NULL; (nsrec = ocs_ns_enumerate(ns, nsrec)) != NULL; ) {
		if (nsrec->sym_node_name != NULL) {
			ocs_free(ocs, nsrec->sym_node_name, nsrec->sym_node_name_len);
			nsrec->sym_node_name = NULL;
		}
	}

	if (ns->ns_records != NULL) {
		ocs_free(ocs, ns->ns_records, sizeof(*ns->ns_records) * ns->ns_record_count);
	}
	if (ns->ns_event_list != NULL) {
		ocs_free(ocs, ns->ns_event_list, ns->ns_event_list_len);
	}
	ocs_lock_free(&ns->ns_event_lock);
	ocs_free(ocs, ns, sizeof(*ns));
}

/**
 * @brief Find or allocate a port ID record
 *
 * A port ID record is found or allocated, and returned.
 *
 * @param ns Pointer to name services database
 * @param port_id Port ID to find/allocate
 *
 * @return Pointer to NS record, or NULL if failure
 */
ocs_ns_record_t *
ocs_ns_alloc(ocs_ns_t *ns, uint32_t port_id)
{
	uint32_t i;
	ocs_ns_record_t *nsrec;
	ocs_t *ocs = ns->domain->ocs;

	/* Don't accept registratoin requests from the well known switch ports */
	switch(port_id) {
	case FC_ADDR_FABRIC:
	case FC_ADDR_NAMESERVER:
	case FC_ADDR_CONTROLLER:
		return NULL;
	}

	nsrec = ocs_ns_find_port_id(ns, port_id);
	if (nsrec != NULL) {
		return nsrec;
	}

	/* Find an empty slot */
	for (i = 0, nsrec = ns->ns_records; i < ns->ns_record_count; i++, nsrec++) {
		if (!nsrec->active) {
			nsrec->active = 1;
			nsrec->port_id = port_id;
			break;
		}
	}

	if (nsrec != NULL) {
		/* Add this port_id to the event list */
		ocs_ns_event_add(ns, nsrec->port_id);

		/* Start delay and then send rscn's */
		if (ocs_timer_pending(&ns->rscn_timer)) {
			ocs_del_timer(&ns->rscn_timer);
		}
		ocs_setup_timer(ocs, &ns->rscn_timer, ocs_ns_rscn_timeout, ns, 1000, false);
	}
	return nsrec;
}

/**
 * @brief Free name services database record
 *
 * A name services database record is freed
 *
 * @param ns Pointer to name services database object
 * @param port_id Port ID to free
 *
 * @return none
 */
void
ocs_ns_free(ocs_ns_t *ns, uint32_t port_id)
{
	ocs_ns_record_t *nsrec;

	nsrec = ocs_ns_find_port_id(ns, port_id);
	if (nsrec != NULL) {
		ocs_memset(nsrec, 0, sizeof(*ns));
	}
}

/**
 * @brief Find port ID in name services database
 *
 * Find a name services record given a port ID
 *
 * @param ns Pointer to name services database
 * @param port_id Port ID to find
 *
 * @return Pointer to NS record, or NULL if not found
 */
ocs_ns_record_t *
ocs_ns_find_port_id(ocs_ns_t *ns, uint32_t port_id)
{
	uint32_t i;
	ocs_ns_record_t *nsrec;

	for (i = 0, nsrec = ns->ns_records; i < ns->ns_record_count; i++, nsrec++) {
		if (nsrec->active && (nsrec->port_id == port_id)) {
			return nsrec;
		}
	}
	return NULL;
}

/**
 * @brief Find port name in name services database
 *
 * Find a name services record given a port name
 *
 * @param ns Pointer to name services database
 * @param port_name Port WWPN to find
 *
 * @return Pointer to NS record, or NULL if not found
 */
ocs_ns_record_t *
ocs_ns_find_port_name(ocs_ns_t *ns, uint64_t port_name)
{
	uint32_t i;
	ocs_ns_record_t *nsrec;

	for (i = 0, nsrec = ns->ns_records; i < ns->ns_record_count; i++, nsrec++) {
		if (nsrec->active && (nsrec->port_name == port_name)) {
			return nsrec;
		}
	}
	return NULL;
}

/**
 * @brief Enumerate NS records
 *
 * Name services database records are enumerated.   The pattern for using this function is:
 *
 * ocs_ns_record_t *nsrec = NULL;
 * while ((nsrec = ocs_ns_enumerate(ns, nsrec)) != NULL) {
 *	action(nsrec);
 * }
 *
 * @param ns Pointer to name services database record
 * @param nsrec Pointer to NS record to iterate
 *
 * @return Pointer to next NS record or NULL
 */
ocs_ns_record_t *
ocs_ns_enumerate(ocs_ns_t *ns, ocs_ns_record_t *nsrec)
{
	uint32_t idx;

	if (nsrec == NULL) {
		nsrec = ns->ns_records;
	} else {
		nsrec++;
	}
	idx = nsrec - ns->ns_records;
	while (idx < ns->ns_record_count) {
		if (nsrec->active) {
			return nsrec;
		}
		idx--;
		nsrec++;
	}
	return NULL;
}


/**
 * @brief Name services RSCN timeout handler
 *
 * When an RSCN event is detected (registering or removing a port ID), all remote nodes that
 * have registered for state change notifications (using SCR) will be notified.   In order
 * to keep from thrashing with a blast of changes, the RSCN's are accumultated for a timeout period
 * then sent.
 *
 * @param arg Pointer to name services database
 *
 * @return none
 */
static void
ocs_ns_rscn_timeout(void *arg)
{
	ocs_ns_t *ns = arg;
	ocs_t *ocs = ns->domain->ocs;
	ocs_ns_record_t *nsrec;
	uint32_t port_ids_buf_len;
	fc_rscn_affected_port_id_page_t *port_ids_buf;
	fc_rscn_affected_port_id_page_t *pid;
	uint32_t i;

	ocs_del_timer(&ns->rscn_timer);

	/* If event list is empty, then don't send out any RSCN's */
	ocs_lock(&ns->ns_event_lock);
		if (ns->ns_event_list_count == 0) {
			ocs_unlock(&ns->ns_event_lock);
			return;
		}

		/* Build the payload */
		port_ids_buf_len = sizeof(*port_ids_buf) * ns->ns_event_list_count;
		port_ids_buf = ocs_malloc(ocs, port_ids_buf_len, OCS_M_ZERO);
		if (port_ids_buf == NULL) {
			ocs_log_err(ocs, "ocs_malloc port_ids failed\n");
			ocs_unlock(&ns->ns_event_lock);
			return;
		}
		for (i = 0, pid = port_ids_buf; i < ns->ns_event_list_count; i++, pid++) {
			pid->port_id = fc_htobe24(ns->ns_event_list[i]);
			pid->address_format = 0;
			pid->rscn_event_qualifier = 0;
		}
	ocs_unlock(&ns->ns_event_lock);

	/* Find all of the registered ports, and send an RSCN to them  */
	nsrec = NULL;
	while ((nsrec = ocs_ns_enumerate(ns, nsrec)) != NULL) {
		if (nsrec->scr_requested) {
			ocs_send_rscn(nsrec->node, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT, OCS_FC_ELS_DEFAULT_RETRIES,
				port_ids_buf, port_ids_buf_len / sizeof(*port_ids_buf), NULL, NULL);
		}
	}

	/* Clear the event list */
	ocs_ns_event_clear(ns);

	ocs_free(ocs, port_ids_buf, port_ids_buf_len);
}

/**
 * @brief Add a changed event
 *
 * Add a port ID to the changed event list
 *
 * @param ns Pointer to name services database
 * @param port_id Port ID to add to the event list
 *
 * @return returns 0
 */
static int32_t
ocs_ns_event_add(ocs_ns_t *ns, uint32_t port_id)
{
	uint32_t i;
	ocs_t *ocs = ns->domain->ocs;

	ocs_lock(&ns->ns_event_lock);
		/* See if this event has already been posted */
		for (i = 0; i < ns->ns_event_list_count; i++) {
			if (ns->ns_event_list[i] == port_id) {
				ocs_unlock(&ns->ns_event_lock);
				return 0;
			}
		}

		if (ns->ns_event_list_count >= ns->ns_record_count) {
			ocs_log_test(ocs, "ns_event_list is full\n");
			ocs_unlock(&ns->ns_event_lock);
			return -1;
		}

		ns->ns_event_list[ns->ns_event_list_count++] = port_id;
	ocs_unlock(&ns->ns_event_lock);

	return 0;
}

/**
 * @brief Clear name services event list
 *
 * The name services changed event list is cleared
 *
 * @param ns Pointer to name services database
 *
 * @return none
 */
static void
ocs_ns_event_clear(ocs_ns_t *ns)
{
	ocs_lock(&ns->ns_event_lock);
		ns->ns_event_list_count = 0;
	ocs_unlock(&ns->ns_event_lock);
}

/**
 * @brief Process driver dump request
 *
 * Process driver dump request by providing name services database information
 *
 * @param textbuf Pointer to text buffer
 * @param ns Pointer to name services database object
 *
 * @return returns 0
 */
int32_t
ocs_ddump_ns(ocs_textbuf_t *textbuf, ocs_ns_t *ns)
{
	ocs_ns_record_t *nsrec = NULL;

	if (!textbuf)
		return 0;

	if (textbuf->ocs->domain && textbuf->ocs->domain->femul_enable) {
		ocs_ddump_section(textbuf, "nameserver", 0);
		ocs_lock(&ns->ns_event_lock);
			while ((nsrec = ocs_ns_enumerate(ns, nsrec)) != NULL) {
				ocs_ddump_value(textbuf, "portid", "%06x", nsrec->port_id);
				ocs_ddump_value(textbuf, "scr_requested", "%d", nsrec->scr_requested);
				ocs_ddump_value(textbuf, "fc4_types", "%08x", nsrec->fc4_types);
				ocs_ddump_value(textbuf, "fc4_featurces", "%02x", nsrec->fc4_features);
				ocs_ddump_value(textbuf, "port_name", "%" PRIx64 , nsrec->port_name);
				ocs_ddump_value(textbuf, "node_name", "%" PRIx64 , nsrec->node_name);
				ocs_ddump_value(textbuf, "sym_node_name", "%s", nsrec->sym_node_name);
			}
		ocs_unlock(&ns->ns_event_lock);
		ocs_ddump_endsection(textbuf, "nameserver", 0);
	}
	return 0;
}
#endif
