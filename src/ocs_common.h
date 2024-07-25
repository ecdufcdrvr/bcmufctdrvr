/*
 * BSD LICENSE
 *
 * Copyright (C) 2024 Broadcom. All Rights Reserved.
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
 * Contains declarations shared between the alex layer and HAL/SLI4
 */


#if !defined(__OCS_COMMON_H__)
#define __OCS_COMMON_H__

#include "ocs_common_shared.h"
#include "ocs_sm.h"
#include "spv.h"
#include "ocs_auth.h"
#include "ocs_mgmt.h"

typedef enum {OCS_XPORT_FC, OCS_XPORT_ISCSI} ocs_xport_e;

#define OCS_SERVICE_PARMS_LENGTH		0x74
#define OCS_DISPLAY_NAME_LENGTH			64
#define OCS_DISPLAY_BUS_INFO_LENGTH		16

#define OCS_WWN_LENGTH				32

#define OCS_INITIATOR_TYPE_FCP			0x1
#define OCS_INITIATOR_TYPE_NVME			0x2

#define OCS_TARGET_TYPE_FCP			0x1
#define OCS_TARGET_TYPE_NVME			0x2

#define OCS_NODE_MAX_EXPLICIT_LOGO_RETRIES	32

typedef struct ocs_hal_s ocs_hal_t;
typedef struct ocs_domain_s ocs_domain_t;
typedef struct ocs_sli_port_s ocs_sli_port_t;
typedef struct ocs_sli_port_s ocs_sport_t;
typedef struct ocs_remote_node_s ocs_remote_node_t;
typedef struct ocs_remote_node_group_s ocs_remote_node_group_t;
typedef struct ocs_node_s ocs_node_t;
typedef struct ocs_io_s ocs_io_t;
typedef struct ocs_xport_s ocs_xport_t;
typedef struct ocs_node_cb_s ocs_node_cb_t;
typedef struct ocs_ns_s ocs_ns_t;

/* Node group data structure */
typedef struct ocs_node_group_dir_s ocs_node_group_dir_t;

#include "ocs_scsi_ini.h"
#include "ocs_scsi_tgt.h"

/*--------------------------------------------------
 * Shared HAL/SLI objects
 *
 * Several objects used by the HAL/SLI layers are communal; part of the
 * object is for the sole use of the lower layers, but implementations
 * are free to add their own fields if desired.
 */

/**
 * @brief Description of discovered Fabric Domain
 *
 * @note Not all fields are valid for all mediums (FC/ethernet).
 */
typedef struct ocs_domain_record_s {
	uint32_t	index;		/**< FCF table index (used in REG_FCFI) */
	uint32_t	priority;	/**< FCF reported priority */
	uint8_t		address[6];	/**< Switch MAC/FC address */
	uint8_t		wwn[8];		/**< Switch WWN */
	union {
		uint8_t	vlan[512];	/**< bitmap of valid VLAN IDs */
		uint8_t	loop[128];	/**< FC-AL position map */
	} map;
	uint32_t	speed;		/**< link speed */
	uint32_t	fc_id;		/**< our ports fc_id */
	bool		is_fc;		/**< Connection medium is native FC */
	bool		is_ethernet;	/**< Connection medium is ethernet (FCoE) */
	bool		is_loop;	/**< Topology is FC-AL */
	bool		is_nport;	/**< Topology is N-PORT */
} ocs_domain_record_t;

/**
 * @brief Node group directory entry
 */
struct ocs_node_group_dir_s {
	uint32_t instance_index;		/*<< instance index */
	ocs_sport_t *sport;			/*<< pointer to sport */
	uint8_t service_params[OCS_SERVICE_PARMS_LENGTH];	/**< Login parameters */
	ocs_list_link_t link;			/**< linked list link */
	ocs_list_t node_group_list;		/**< linked list of node groups */
	uint32_t node_group_list_count;		/**< current number of elements on the node group list */
	uint32_t next_idx;			/*<< index of the next node group in list */
};

typedef enum {
	OCS_SPORT_TOPOLOGY_UNKNOWN=0,
	OCS_SPORT_TOPOLOGY_FABRIC,
	OCS_SPORT_TOPOLOGY_P2P,
	OCS_SPORT_TOPOLOGY_LOOP,
} ocs_sport_topology_e;

/**
 * @brief SLI Port object
 *
 * The SLI Port object represents the connection between the driver and the
 * FC/FCoE domain. In some topologies / hardware, it is possible to have
 * multiple connections to the domain via different WWN. Each would require
 * a separate SLI port object.
 */
struct ocs_sli_port_s {

	ocs_t *ocs;				/**< pointer to ocs */
	uint32_t tgt_id;			/**< target id */
	uint32_t index;				/**< ??? */
	uint32_t instance_index;
	char display_name[OCS_DISPLAY_NAME_LENGTH]; /**< sport display name */
	ocs_domain_t *domain;			/**< current fabric domain */
	bool		is_vport;		/**< this SPORT is a virtual port */
	uint64_t	wwpn;			/**< WWPN from HAL (host endian) */
	uint64_t	wwnn;			/**< WWNN from HAL (host endian) */
	ocs_list_t node_list;			/**< list of nodes */
	ocs_lock_t	node_shutdown_lock;
	ocs_list_t node_shutdown_list;		/**< list of nodes in shutdown stage */
	ocs_scsi_ini_sport_t ini_sport;		/**< initiator backend private sport data */
	ocs_scsi_tgt_sport_t tgt_sport;		/**< target backend private sport data */
	void	*tgt_data;			/**< target backend private pointer */
	void	*ini_data;			/**< initiator backend private pointer */
	ocs_mgmt_functions_t *mgmt_functions;

	/*
	 * Members private to HAL/SLI
	 */
	ocs_sm_ctx_t	ctx;		/**< state machine context */
	ocs_hal_t	*hal;		/**< pointer to HAL */
	uint32_t	indicator;	/**< VPI */
	uint32_t	fc_id;		/**< FC address */
	ocs_dma_t	dma;		/**< memory for Service Parameters */

	uint8_t		wwnn_str[OCS_WWN_LENGTH];	/**< WWN (ASCII) */
	uint64_t	sli_wwpn;	/**< WWPN (wire endian) */
	uint64_t	sli_wwnn;	/**< WWNN (wire endian) */
	bool		sm_free_req_pending;	/**< Free request received while waiting for attach response */

	/*
	 * Implementation specific fields allowed here
	 */
	ocs_sm_ctx_t	sm;			/**< sport context state machine */
	sparse_vector_t lookup;			/**< fc_id to node lookup object */
	ocs_list_link_t link;
	ocs_ref_t	ref;			/**< refcount object */
	bool		enable_ini;		/**< SCSI initiator enabled for this node */
	bool		enable_tgt;		/**< SCSI target enabled for this node */
	bool		enable_rscn;		/**< This SPORT will be expecting RSCN */
	bool		shutting_down;		/**< sport in process of shutting down: locking: device lock */
	bool		p2p_winner;		/**< TRUE if we're the point-to-point winner: locking: device lock */
	bool		unreg_rpi_all;		/**< unreg all RPIs for a given VPI: locking: device lock */
	bool		re_enable;		/**< TRUE if sport has to be re-enabled: locking: device lock */
	uint8_t		async_flush_state;      /**< Track Flush request on RQs */

	uint8_t		ini_fc_types;		/**< NVME/SCSI initiator */
	uint8_t		tgt_fc_types;		/**< NVME/SCSI target */

	ocs_sport_topology_e topology;		/**< topology: fabric/p2p/unknown */
	uint8_t		service_params[OCS_SERVICE_PARMS_LENGTH]; /**< Login parameters */
	uint32_t	p2p_remote_port_id;	/**< Remote node's port id for p2p */
	uint32_t	p2p_port_id;		/**< our port's id */

	/* List of remote node group directory entries (used by high login mode) */
	ocs_lock_t	node_group_lock;
	uint32_t	node_group_dir_next_instance; /**< HLM next node group directory instance value */
	uint32_t	node_group_next_instance; /**< HLM next node group instance value */
	ocs_list_t	node_group_dir_list;
	uint32_t	fdmi_hba_mask;
	uint32_t	fdmi_port_mask;
	uint32_t	num_disc_ports;
	uint32_t	fabric_rpi;
	uint32_t	fabric_rpi_index;
};

#define OCS_ASYNC_FLUSH_START		1
#define OCS_ASYNC_FLUSH_COMPLETE	2

/**
 * @brief Fibre Channel domain object
 *
 * This object is a container for the various SLI components needed
 * to connect to the domain of a FC or FCoE switch
 */
struct ocs_domain_s {

	ocs_t *ocs;				/**< pointer back to ocs */
	uint32_t instance_index;		/**< unique instance index value */
	char display_name[OCS_DISPLAY_NAME_LENGTH]; /**< Node display name */
	ocs_list_t sport_list;			/**< linked list of SLI ports */
	ocs_scsi_ini_domain_t ini_domain;	/**< initiator backend private domain data */
	ocs_scsi_tgt_domain_t tgt_domain;	/**< target backend private domain data */
	ocs_mgmt_functions_t *mgmt_functions;
	uint32_t	evtdepth; 	/**< current event posting nesting depth. Free doing only when 0 */

	/* Declarations private to HAL/SLI */
	ocs_hal_t	*hal;		/**< pointer to HAL */
	ocs_sm_ctx_t	sm;		/**< state machine context */
	uint32_t	fcf;		/**< FC Forwarder table index */
	uint32_t	fcf_indicator;	/**< FCFI */
	uint32_t	vlan_id;	/**< VLAN tag for this domain */
	uint32_t	indicator;	/**< VFI */
	ocs_dma_t	dma;		/**< memory for Service Parameters */
	bool		req_rediscover_fcf;	/**< TRUE if fcf rediscover is needed (in response
						 * to Vlink Clear async event: device lock */

	/* Declarations private to FC transport */
	uint64_t	fcf_wwn;	/**< WWN for FCF/switch */
	ocs_list_link_t link;
	ocs_sm_ctx_t	drvsm;		/**< driver domain sm context */
	ocs_rlock_t	drvsm_lock;	/**< driver domain sm lock */
	bool		attached;	/**< set true after attach completes: device lock */
	bool		is_fc;		/**< is FC: device lock */
	bool		is_loop;	/**< is loop topology: device lock */
	bool		is_nlport;	/**< is public loop: device lock */
	bool		domain_found_pending;	/**< A domain found is pending, drec is updated: device lock */
	bool		req_domain_free;	/**< True if domain object should be free'd: device lock */
	bool		req_accept_frames;	/**< set in domain state machine to enable frames: device lock */
	bool		domain_notify_pend;  /** Set in domain SM to avoid duplicate node event post: device lock */
	uint8_t		async_flush_state;	/**< Flush request on all RQs complete */
	ocs_domain_record_t pending_drec; /**< Pending drec if a domain found is pending */
	uint8_t		service_params[OCS_SERVICE_PARMS_LENGTH]; /**< any sports service parameters */
	uint8_t		flogi_service_params[OCS_SERVICE_PARMS_LENGTH]; /**< Fabric/P2p service parameters from FLOGI */
	uint8_t		femul_enable;	/**< TRUE if Fabric Emulation mode is enabled */
#if defined(ENABLE_FABRIC_EMULATION)
	uint8_t		femul_vports_started;	/**< TRUE if fabric vports are started */
	ocs_atomic_t	femul_count;	/**< count of fabric emulation nodes in the process of being created */
	uint16_t	femul_oxid;	/**< ox_id for FLOGI response */
	uint64_t	femul_port_name;/**< used to populate nameserver db */
#endif

	/* Declarations shared with back-ends */
	sparse_vector_t lookup;		/**< d_id to node lookup object */
	ocs_lock_t	lookup_lock;

	ocs_sli_port_t	*sport;		/**< Pointer to first (physical) SLI port (also at the head of sport_list) */
	uint32_t	sport_instance_count; /**< count of sport instances */

	/* Fabric Emulation */
	ocs_bitmap_t *portid_pool;
	ocs_ns_t *ocs_ns;			/*>> Directory(Name) services data */
};

struct ocs_domain_exec_arg_s;

typedef struct ocs_sport_exec_arg_s {
	uint32_t arg_in_length;
	void *arg_in;
	uint32_t arg_out_length;
	void *arg_out;
	int32_t retval;
	struct ocs_domain_exec_arg_s *domain_exec_args;
	void (*exec_cb)(ocs_sport_t *sport, void *arg);
} ocs_sport_exec_arg_t;

typedef struct ocs_domain_exec_arg_s {
	uint32_t arg_in_length;
	void *arg_in;
	uint32_t arg_out_length;
	void *arg_out;
	int32_t retval;
	ocs_sem_t sem;
	ocs_atomic_t ref_cnt;
	ocs_sport_exec_arg_t sport_exec_args;
	void (*exec_cb)(ocs_domain_t *, void *arg);
} ocs_domain_exec_arg_t;

/**
 * @brief Remote Node object
 *
 * This object represents a connection between the SLI port and another
 * Nx_Port on the fabric. Note this can be either a well known port such
 * as a F_Port (i.e. ff:ff:fe) or another N_Port.
 */
struct ocs_remote_node_s {
	/*
	 * Members private to HAL/SLI
	 */
	uint32_t	indicator;	/**< RPI */
	uint32_t	index;
	uint32_t	fc_id;		/**< FC address */

	bool	attached;	/**< true if attached: node lock */
	bool	node_group;	/**< true if in node group: node lock */
	bool	free_group;	/**< true if the node group should be free'd: node lock */

	ocs_sli_port_t	*sport;		/**< associated SLI port */

	/*
	 * Implementation specific fields allowed here
	 */
	void *node;			/**< associated node */
};

struct ocs_remote_node_group_s {
	/*
	 * Members private to HAL/SLI
	 */
	uint32_t	indicator;	/**< RPI */
	uint32_t	index;

	/*
	 * Implementation specific fields allowed here
	 */
	uint32_t instance_index;		/*<< instance index */
	ocs_node_group_dir_t *node_group_dir;	/*<< pointer to the node group directory */
	ocs_list_link_t link;			/*<< linked list link */
};

typedef enum {
	OCS_LS_RSP_TYPE_PLOGI = 0,
	OCS_LS_RSP_TYPE_FCP_PRLI,
	OCS_LS_RSP_TYPE_NVME_PRLI,
	OCS_LS_RSP_TYPE_LOGO,
	OCS_LS_RSP_TYPE_FCP_PRLO,
	OCS_LS_RSP_TYPE_NVME_PRLO,
	OCS_LS_RSP_TYPE_MAX,
} ocs_ls_rsp_type_e;

typedef enum {
	OCS_NODE_SHUTDOWN_DEFAULT = 0,
	OCS_NODE_SHUTDOWN_EXPLICIT_LOGO,
	OCS_NODE_SHUTDOWN_IMPLICIT_LOGO,
} ocs_node_shutd_rsn_e;

#define NODE_MAGIC 0xcafecafe

/**
 * @brief FC Node object
 *
 */
struct ocs_node_s {

	ocs_t *ocs;				/**< pointer back to ocs structure */
	uint32_t instance_index;		/**< unique instance index value */
	uint32_t magic;				/**< for sanity checks */
	char display_name[OCS_DISPLAY_NAME_LENGTH]; /**< Node display name */
	ocs_sport_t *sport;
	ocs_rlock_t lock;			/**< node wide lock */
	ocs_lock_t active_ios_lock;		/**< active SCSI and XPORT I/O's for this node */
	ocs_list_t active_ios;			/**< active I/O's for this node */
	uint64_t max_wr_xfer_size;		/**< Max write IO size per phase for the transport */
	ocs_scsi_ini_node_t ini_node;		/**< backend initiator private node data */
	ocs_scsi_tgt_node_t tgt_node;		/**< backend target private node data */
	ocs_mgmt_functions_t *mgmt_functions;

	/* Declarations private to HAL/SLI */
	ocs_remote_node_t	rnode;		/**< Remote node */

	/* Declarations private to FC transport */
	ocs_sm_ctx_t		sm;		/**< state machine context */
	uint32_t		evtdepth;	/**< current event posting nesting depth */
	bool			req_free;	/**< this node is to be free'd: node lock */
	bool			mark_for_deletion; /**< this node is marked for deletion: node lock */
	bool			hold_frames;	/**< hold incoming frames if true */
	bool			attached;	/**< node is attached (REGLOGIN complete): node lock */
	bool			wait_attached;  /**< node waiting for node attach to complete */
	bool			fcp_enabled;	/**< node is enabled to handle FCP: node lock */
	bool			rscn_pending;	/**< for name server node RSCN is pending: node lock */
	bool			send_plogi;	/**< if initiator, send PLOGI at node initialization: node lock*/
	bool			send_plogi_acc; /**< send PLOGI accept, upon completion of node attach */
	bool			io_alloc_enabled; /**< TRUE if ocs_scsi_io_alloc() and ocs_els_io_alloc() are enabled: active_ios_lock */
	uint8_t			async_flush_state; /**< Flush request on all RQs complete */

	uint16_t		ini_fct_prli_success; /** initiator mode SCSI/NVME PRLI success */
	uint16_t		tgt_fct_prli_success; /** target mode SCSI/NVME PRLI success */
	uint16_t		prli_rsp_pend;	/** prli sent and waiting for response */
	uint16_t		sess_attach_pend; /** backend session registration in progress */
	bool			remote_nvme_prli_rcvd; /** NVMe PRLI request received from remote node */
	bool			remote_fcp_prli_rcvd; /** FCP PRLI request received from remote node */

	/**
	 * Array of ELS IO pointers to handle the incoming PLOGI/FCP PRLI/NVMe PRLI
	 * requests before the node state machine moves to port_logged_in state.
	 */
	ocs_io_t		*ls_rsp_io[OCS_LS_RSP_TYPE_MAX];
	uint32_t		ls_rjt_reason_code; /** < Reason code for LS RJT */
	uint32_t		ls_rjt_reason_expl_code; /** < Reason explanation code for LS RJT */
	ocs_node_shutd_rsn_e	shutdown_reason;/**< reason for node shutdown */
	ocs_dma_t		sparm_dma_buf;	/**< service parameters buffer */
	uint8_t			service_params[OCS_SERVICE_PARMS_LENGTH]; /**< plogi/acc frame from remote device */
	int32_t			suppress_rsp;	/**< Suppress Response requested */
	ocs_lock_t		pend_frames_lock; /**< lock for inbound pending frames list */
	ocs_list_t		pend_frames;	/**< inbound pending frames list */
	uint32_t		pend_frames_processed; /**< count of frames processed in hold frames interval */
	uint32_t		ox_id_in_use;	/**< used to verify one at a time us of ox_id */
	uint32_t		els_retries_remaining; /**< for ELS, number of retries remaining */
	uint32_t		els_req_cnt;	/**< number of outstanding ELS requests */
	uint32_t		els_cmpl_cnt;	/**< number of outstanding ELS completions */
	uint32_t		abort_cnt;	/**< Abort counter for debugging purpose */
	uint64_t		shutdown_start_time; /**< node shutdown start ticks */

	char current_state_name[OCS_DISPLAY_NAME_LENGTH]; /**< current node state */
	char prev_state_name[OCS_DISPLAY_NAME_LENGTH]; /**< previous node state */
	ocs_sm_event_t		current_evt;	/**< current event */
	ocs_sm_event_t		prev_evt;	/**< current event */
	bool			targ;		/**< node is scsi target capable: node lock */
	bool			init;		/**< node is scsi init capable */
	bool			nvme_init;	/**< node is NVME init capable: node lock*/
	bool			nvme_tgt;	/**< node is NVME tgt capable: node lock */
	bool			refound;	/**< Handle node refound case when node is being deleted */
	bool			unreg_rpi;	/**< Used for synchronizing unreg_rpi / unreg_rpi_all: node lock */
	bool			first_burst;	/* lock: node lock */
	bool			nvme_sler;	/* lock: node lock */
	bool			nvme_conf;	/* lock: node lock */
	uint32_t		nvme_prli_service_params;
	ocs_list_t		els_io_pend_list; /**< list of pending (not yet processed) ELS IOs */
	ocs_list_t		els_io_active_list; /**< list of active (processed) ELS IOs */

	ocs_sm_function_t	nodedb_state;	/**< Node debugging, saved state */

	ocs_timer_t		gidpt_delay_timer; /**< GIDPT delay timer */
	time_t			time_last_gidpt_msec; /**< Start time of last target RSCN GIDPT */

	/* WWN */
	char wwnn[OCS_WWN_LENGTH];		/**< remote port WWN (uses iSCSI naming) */
	char wwpn[OCS_WWN_LENGTH];		/**< remote port WWN (uses iSCSI naming) */

	/* Statistics */
	uint32_t		chained_io_count; /**< count of IOs with chained SGL's */

	ocs_list_link_t		link;		/**< node list link */
	ocs_list_link_t		shutdown_link;		/**< node list link */

	ocs_remote_node_group_t	*node_group;	/**< pointer to node group (if HLM enabled) */

	/* runtime auth negotiation info with remote node */
	union ocs_auth_info	auth;
	/* copy of the auth cfg info for the local/remote node pair */
	struct ocs_auth_cfg_info auth_cfg;
	struct ocs_auth_cfg_pass auth_pass;

	/* Outstanding plogi request */
	ocs_io_t *plogi_els_io;

	/* Outstanding duplicate prli context */
	ocs_node_cb_t *dup_prli_cbdata;

	void (*prep_notify_it_nexus_loss)(ocs_t *ocs, struct ocs_node_s *node);

	/* Retried counter due to explicit logout */
	uint32_t explicit_logout_retries;
	bool detach_notify_pending;
};

/**
 * @brief Virtual port specification
 *
 * Collection of the information required to restore a virtual port across
 * link events
 */

typedef struct ocs_vport_spec_s ocs_vport_spec_t;
struct ocs_vport_spec_s {
	uint32_t domain_instance;		/*>> instance index of this domain for the sport */
	uint64_t wwnn;				/*>> node name */
	uint64_t wwpn;				/*>> port name */
	uint32_t fc_id;				/*>> port id */
	bool	enable_tgt;			/*>> port is a target */
	bool	enable_ini;			/*>> port is an initiator */
	uint32_t ini_fc_types;			/*>> initiator mode FCP/NVME */
	uint32_t tgt_fc_types;			/*>> target mode FCP/NVME */
	ocs_list_link_t link;			/*>> link */
	void *tgt_data;				/**< target backend pointer */
	void *ini_data;				/**< initiator backend pointer */
	ocs_sport_t *sport;			/**< Used to match record after attaching for update */
};

extern void ocs_mgmt_domain_exec_cb(ocs_domain_t *domain, void *);
extern void ocs_mgmt_sport_exec_cb(ocs_sport_t *sport, void *);

#endif // __OCS_COMMON_H__
