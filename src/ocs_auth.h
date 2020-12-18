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
 * OCS linux driver authentication protocol definitions
 */

#if !defined(__OCS_AUTH_H__)
#define __OCS_AUTH_H__

#define OCS_AUTH_ELS_RETRIES	2
#define AUTH_TOV_MS		(45 * 1000)

/* AUTH_ELS Message header */
struct ocs_auth_els_hdr {
#define AUTH_ELS_CODE		0x90
	u8		els_code;
	u8		els_flags;
/* auth_msg_code defines */
#define	AUTH_REJECT		0x0A
#define	AUTH_NEGOTIATE		0x0B
#define	AUTH_DONE		0x0C
#define	DHCHAP_CHALLENGE	0x10
#define	DHCHAP_REPLY		0x11
#define	DHCHAP_SUCCESS		0x12
	u8		msg_code;
#define	AUTH_PROTOCOL_VER_1	0x1
	u8		protocol_ver;
	__be32		msg_len;	/* msg payload len */
	__be32		trans_id;	/* transaction id */
	/* ... msg payload */
};

struct ocs_auth_node_name {
#define	AUTH_NAME_TAG		0x1
	__be16		tag;
#define	AUTH_NAME_LEN		8
	__be16		len;
	u8		val[AUTH_NAME_LEN];	/* WWPN */
};

/* AUTH_Negotiate msg */
struct ocs_auth_neg_hdr {
	struct ocs_auth_node_name name;
	__be32		num_proto;	/* only dhchap for now */
	/* ... proto params */
};

struct ocs_auth_proto_params_hdr {
	__be32		len;
#define	PROTOCOL_DHCHAP		0x1
#define	PROTOCOL_FCAP		0x2
#define	PROTOCOL_FCPAP		0x3
	__be32		proto_id;
	/* ... proto param variable len data */
};

/* Auth_Negotiate DH-CHAP param format */
struct ocs_auth_neg_dhchap_param {
#define	DHCHAP_TAG_HASHLIST	0x1
#define	DHCHAP_TAG_DHGRPLIST	0x2
	__be16		tag;
	__be16		word_cnt;
	/* ... variable size value */
};

/* DHCHAP Group ID defines */
#define DH_GROUP_NULL		0x00
#define DH_GROUP_1024		0x01
#define DH_GROUP_1280		0x02
#define DH_GROUP_1536		0x03
#define DH_GROUP_2048		0x04
struct ocs_dh_group {
	u32		groupid;
	u32		length;
#define	MAX_DH_VAL_LEN			256
	u8		value[MAX_DH_VAL_LEN];
};

/* hash function defines */
#define HASH_MD5		0x05
#define HASH_SHA1		0x06

#define	MD5_HASH_LEN		16
#define	SHA1_HASH_LEN		20
#define MAX_HASH_LEN		(MD5_HASH_LEN > SHA1_HASH_LEN ? \
				 MD5_HASH_LEN : SHA1_HASH_LEN)

/* AUTH_Reject payload */
struct ocs_auth_rej_msg {
#define	AUTH_RJT_CODE_FAILURE		1
#define	AUTH_RJT_CODE_LOGICAL_ERR	2
	u8		code;
#define	AUTH_RJT_EXPL_MECH_UNUSABLE	1
#define	AUTH_RJT_EXPL_DHGRP_UNUSABLE	2
#define	AUTH_RJT_EXPL_HASH_UNUSABLE	3
#define	AUTH_RJT_EXPL_AUTH_STARTED	4
#define	AUTH_RJT_EXPL_AUTH_FAILED	5
#define	AUTH_RJT_EXPL_BAD_PAYLOAD	6
#define	AUTH_RJT_EXPL_BAD_PROTOCOL	7
#define	AUTH_RJT_EXPL_RESTART_AUTH	8
#define	AUTH_RJT_EXPL_CONCAT_NOSUPP	9
#define	AUTH_RJT_EXPL_PROTO_VER_NOSUPP	0xA
	u8		expl;
	u8		rsvd[2];
};

/* DHCHAP_Challenge msg payload */
struct ocs_dhchap_chal_hdr {
	struct ocs_auth_node_name name;
	__be32		hash_id;	/* Hash function id */
	__be32		dh_grp_id;	/* DH_Group id */
	/* ... challenge, dh val */
};

/* AUTH state machine related defines */

/* AUTH events */
enum ocs_auth_evt {
	OCS_AUTH_EVT_START,
	OCS_AUTH_EVT_RESTART,
	OCS_AUTH_EVT_STOP,
	OCS_AUTH_EVT_MSG_SEND_FAIL,
	OCS_AUTH_EVT_NEXT_MSG_TIMEOUT,

	/* DHCHAP related evts */
	OCS_AUTH_EVT_NEG_RECV,
	OCS_AUTH_EVT_REJ_RECV,
	OCS_AUTH_EVT_DHCHAP_CHAL_RECV,
	OCS_AUTH_EVT_DHCHAP_REPLY_RECV,
	OCS_AUTH_EVT_DHCHAP_SUCCESS_RECV,
};

/*
 * DHCHAP state machine
 * We'll need a separate set of states for each Auth protocol
 * This enum is used as an index in the state table array. So, it must
 * start with zero and increment without gaps.
 */
enum ocs_dhchap_state {
	OCS_DHCHAP_STATE_INIT,
	OCS_DHCHAP_STATE_NEG_SENT,
	OCS_DHCHAP_STATE_CHAL_SENT,
	OCS_DHCHAP_STATE_REPLY_SENT,
	OCS_DHCHAP_STATE_SUCCESS_SENT,
	OCS_DHCHAP_STATE_DONE,
	OCS_DHCHAP_STATE_LAST
};

/* Auth config data store
 * ======================
 * The driver maintains auth configuration information (including passwords) for
 * all remote ports globally in coherent DMA memory. This config data is added
 * and updated by the user and persistently stored in the adapter via FW.
 *
 * The config data is:
 * - loaded from NVRAM via FW at boot-time
 * - updated by user (add/del) at run-time
 * - commited to NVRAM via FW at unload-time
 *
 *   The config data object (at a pre-defined path) is read from FW using the
 *   OBJECT_READ SLI4cmd. The object file is common for all functions of the
 *   adapter.
 *
 *   The config data need not be DH-CHAP specific; the same data store will be
 *   used to maintain entries applicable to other auth protocols too.
 */
#define OCS_AUTH_CFG_NVRAM_PATH		"/driver/auth.cfg"
#define OCS_AUTH_PSSWD_MAX_LEN 128
struct ocs_auth_cfg_pass {
	u8		local_len;
	u8		local_mode;
	u8		remote_len;
	u8		remote_mode;
	u8		local[OCS_AUTH_PSSWD_MAX_LEN];
	u8		remote[OCS_AUTH_PSSWD_MAX_LEN];
};

struct ocs_auth_cfg_hdr {
#define OCS_AUTH_CFG_SIGNATURE		0x61757468
	u32		signature;
#define OCS_AUTH_CFG_VERSION		1
	u8		version;
	u8		size;
	u16		num_entries;
};

struct ocs_auth_cfg_info {
	u16			tmo;
#define OCS_AUTH_MODE_DISABLE 1
#define OCS_AUTH_MODE_ACTIVE  2
#define OCS_AUTH_MODE_PASSIVE 3
	u8			mode;
#define OCS_AUTH_FLAGS_BIDI	BIT(0)
#define OCS_AUTH_FLAGS_VALID	BIT(7)
	u8			flags;
	/* NA as only DHCHAP supported currently */
	u8			proto_prio[4];
#define OCS_HASH_MD5		1
#define OCS_HASH_SHA1		2

#define OCS_MAX_HASH		2
#define OCS_MAX_CFG_HASH	4 /* cfg has 4 slots though we support only 2 */
	/* in the order of priority */
	u8			hash_list[OCS_MAX_CFG_HASH];
#define OCS_DH_GROUP_NULL	1
#define OCS_DH_GROUP_1024	2
#define OCS_DH_GROUP_1280	3
#define OCS_DH_GROUP_1536	4
#define OCS_DH_GROUP_2048	5

#define OCS_MAX_DH_GROUP	5
#define OCS_MAX_CFG_DH_GROUP	8
	/* in the order of priority */
	u8			dh_grp_list[OCS_MAX_CFG_DH_GROUP];
	u32			reauth_interval;
};

struct ocs_auth_cfg_id {
#define FABRIC_WWPN		0xFFFFFFFFFFFFFFFFLL
	u8			local_wwpn[8];
	u8			remote_wwpn[8];
};

struct ocs_auth_cfg_entry {
	struct ocs_auth_cfg_id		id;
	struct ocs_auth_cfg_info	info;
	struct ocs_auth_cfg_pass	pass;
};

/* Size of the auth cfg object shared by all functions */
#define OCS_AUTH_CFG_SIZE_GLOBAL	0x8000

/* Size of the per function share of the auth cfg object,
 * assuming a max of 4 functions on a the adapter
 */
#define OCS_AUTH_CFG_SIZE		(OCS_AUTH_CFG_SIZE_GLOBAL / 4)
#define OCS_AUTH_CFG_ENTRIES_MAX					\
	(int)((OCS_AUTH_CFG_SIZE - sizeof(struct ocs_auth_cfg_hdr)) /	\
			sizeof(struct ocs_auth_cfg_entry))

struct ocs_auth_cfg_db {
	struct ocs_auth_cfg_hdr		hdr;
	struct ocs_auth_cfg_entry	entries[OCS_AUTH_CFG_ENTRIES_MAX];
};

/* ... remaining API later ...*/

/****** End: Auth config data store *******/

/* Auth runtime info related defines */
enum ocs_auth_role {
	OCS_AUTH_ROLE_INITIATOR,
	OCS_AUTH_ROLE_RESPONDER
};

enum ocs_auth_status {
	OCS_AUTH_SUCCESS,
	OCS_AUTH_ERROR,
	OCS_AUTH_FAILURE,
	OCS_AUTH_STOPPED
};

/* Temporary structs needed to pass info to the FSM */
struct ocs_auth_msg_info {
	u8	code;
	u8	*msg;	/* msg payload */
	u32	len;
	__be32	trans_id;
};

struct ocs_auth_err {
	u8	code;
	u8	expl;
};

/* The runtime DH-CHAP negotiation info with the remote node is stored in
 * the sturct below. A separate instance of this struct is maintained for
 * each remote port (Fx_Port or Nx_Port), irrespective of whether the sport
 * is a vport or not.
 */
struct ocs_dhchap_info {
	enum ocs_dhchap_state		state;
	enum ocs_auth_role		role;
	enum ocs_auth_status		status;
	bool				flogi_acc_fcsp;
	__be32				trans_id;
	u32				hash_id;
	u32				hash_len;
	u32				dh_grp_id; /* points to g and p */
	u32				dh_len;

	u8				chal[MAX_HASH_LEN];
#define	NONCE_LEN			16
	u8				nonce[NONCE_LEN];
	u8				eph_dh[MAX_DH_VAL_LEN];
	bool				eph_dh_computed;

	u32				reauth_interval_s;

	/* expect to receive next msg before this timer expires */
	ocs_timer_t			next_msg_timer;
};

union ocs_auth_info {
	struct ocs_dhchap_info dhchap;
	/* ... defns of other auth protocols supported in future */
};


/* forward declarations */
struct ocs_node_s;
typedef struct ocs_node_s ocs_node_t;
struct ocs_node_cb_s;
typedef struct ocs_node_cb_s ocs_node_cb_t;

/* externs */
int ocs_auth_cfg_setup(ocs_t *ocs);
void ocs_auth_cfg_clear(ocs_t *ocs);
void ocs_node_auth_init(ocs_node_t *node);
bool ocs_node_auth_enabled(ocs_node_t *node);
void ocs_node_auth_stop(ocs_node_t *node);
int ocs_node_auth_start(ocs_node_t *node, enum ocs_auth_role role,
			enum ocs_auth_evt evt, void *arg);
void ocs_auth_els_recv(ocs_node_t *node, ocs_node_cb_t *cbdata);
void ocs_auth_els_resp(ocs_node_t *node, ocs_node_cb_t *cbdata);
int ocs_auth_cfg_info_update(ocs_t *ocs, struct ocs_auth_cfg_info *cfg_info);
int ocs_auth_cfg_info_get(ocs_t *ocs, struct ocs_auth_cfg_info *cfg_info);
int ocs_auth_cfg_entry_update(ocs_t *ocs, u8 *local_wwpn, u8 *remote_wwpn,
			      struct ocs_auth_cfg_pass *pass);
int ocs_auth_cfg_commit(ocs_t *ocs);
int ocs_auth_cfg_entries_get(ocs_t *ocs, struct ocs_auth_cfg_id *ids, u32 num);
int ocs_auth_cfg_num_entries_get(ocs_t *ocs, u32 *num);

#endif /* __OCS_AUTH_H__ */
