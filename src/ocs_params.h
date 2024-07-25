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
 * Driver parameters
 */

#if !defined(__OCS_PARAMS_H__)
#define __OCS_PARAMS_H__

/**
 * @brief Module parameter defines
 *
 * The module/program parameters are defined here as a two level macro. The
 * intent is to encapsulate the relevent data in a general way such that the
 * C preprocess can emit C code for required declarations and code.
 *
 * The define consists of the PARAM_LIST macro, that in turn is defined as a set
 * of P() macros.  By defining the P() macro a particular way the PARAM_LIST
 * macro expands to the desired set of C code.
 *
 * For the module parameters, the information encoded in the P() macro are:
 *
 *	type		This is the type of the paramters, int or charp are currently supported
 *	name		Name of the variable
 *	value		This is the default value of the variable
 *	desc		Description of the variable
 *
 * An example of this design pattern is given in the following:
 *
 *	\#define PARAM_LIST
 *		P(int, initiator, 0, "enable initiator functionality (default is 0)")
 *		P(charp, progname, "a.out", "name of program (default is a.out)")
 *
 * By defining P()
 *
 *	\#define P(type, name, value, desc)	type name = value;
 *	PARAM_LIST
 *	\#undef P
 *
 * PARAM_LIST will expand to:
 *
 *	int initiator = 0;
 *	charp progname = "a.out";
 */

typedef char * charp;

#define OCS_COMMON_PARAM_LIST \
	P(int,		initiator,		0,	"enable initiator functionality(default is 0)") \
	P(int,		initiator_flags,	0,	"initiator type bit[0]-SCSI, bit[1]-NVME (default is 1)") \
	P(int,		target,			1,	"enable target functionality(default is 1)") \
	P(int,		logdest,		5,	"logging destination (default is 0)\n" \
							"bit[0] = system log (default is 1)\n" \
							"bit[1] = ram log (default is 0)\n" \
							"bit[2] = log timestamp") \
	P(int,		ramlog_size,		1*1024*1024,	"size of ram logging buffer (default is 1M)") \
	P(int,		ddump_saved_size,	0,	"size of saved ddump (default is 0)") \
	P(int,		uspacemask,		0,	"bit[0] 0:thread per port(default), 1:poll from single thread\n" \
							"bit[1] 0:threads block on events(default), 1:threads poll") \
	P(charp,	hal_war_version,	NULL,	"Use this as f/w version for HAL workaround matching") \
	P(int,		num_scsi_ios,		8192,	"Number of SCSI IOs to allocate")

#if !defined(OCS_USPACE_SPDK_UPSTREAM)
#define OCS_PARAM_LOGLEVEL \
	P(int,		loglevel,		5,	"logging level 0=CRIT, 1=ERR, 2=WARN, 3=INFO, 4=TEST, 5=DEBUG")
#else
#define OCS_PARAM_LOGLEVEL \
	P(int,		loglevel,		7,	"logging level defined based on syslog level. 7=DEBUG")
#endif

#define OCS_PARAM_CTRLMASK \
	P(int,		ctrlmask,		0xA,	"control bitmask (default is 0xA)\n" \
							"bit[0] disable autorsp on target reads\n" \
							"bit[1] disable autorsp on target writes\n" \
							"bit[3] enable target RSCN procesing\n" \
							"bit[4] Tgt Always verify DIF (clear AT/ATRT)")

#define OCS_FC_PARAM_LIST \
	P(int,		logmask,		0,	"logging bitmask (default is 0)") \
	P(int,		speed,			0,	"link speed in Mbps (0 - auto)") \
	P(int,		topology,		0,	"topology, 0 - auto, 1 - N_PORT, 2 - LOOP (default is 0)") \
	P(charp,	wwn_bump,		"0",	"xor'd to 64 bits of WWN's (default is 0)") \
	P(int,		ethernet_license,	0,	"ethernet license") \
	P(int,		tow_feature,		0,	"Target Optimized Write feature. 1 - Auto Xfer Rdy (default is 0)") \
	P(int,		tow_io_size,		65536,	"The maximum sized write command to use for Target Optimized write feature (default is 65536)") \
	P(int,		tow_xri_cnt,		512,	"Number of XRIs posted") \
	P(int,		esoc,			0,	"To control the start offset computation functionality for auto transfer ready")\
	P(int,		enable_hlm,		0,	"enable high login mode") \
	P(int,		hlm_group_size,		8,	"high login mode group size") \
	P(int,		explicit_buffer_list,	0,	"Enable Explict Buffer Lists 0 - false, 1 - true (default is false)") \
	P(int,		dif_separate,		0,	"1 - DIF block is separate, 0 - DIF block is interleaved") \
	P(int,		num_vports,		0,	"Number of NPIV ports (default is 0)") \
	P(int,		external_loopback,	0,	"External loopback mode (default is 0)") \
	P(charp,	queue_topology,		"eq cq rq cq mq $nulp($nwq(cq wq:ulp=$rpt1)) cq wq:len=256:class=1", "Queue topology string " \
						"(default \"eq cq rq cq mq $nulp($nwq(cq wq:ulp=$rpt1)) cq wq:class=1\")") \
	P(int,		target_io_timer,	0,	"Timeout value, in seconds, for target commands (default 0 - timer disabled)") \
	P(int,		target_wqe_timer,	30,	"Timeout value, in seconds, for target commands (default 30)") \
	P(int,		hal_bounce,		0,	"HAL bounce, 0 - no bounce, 1 - bounce (default 0)") \
	P(int,		rq_threads,		0,	"The number of RQ threads to create (default 0)") \
	P(int,		cq_process_limit,	64,	"cq process limit") \
	P(int,		rr_quanta,		1,	"MultiRQ round robin quanta (default 1)") \
	P(charp,	sliport_pause_errors,	"disabled",	"enable sliport pause mechanism (default disabled)") \
	P(int,		enable_bbcr,		1,	"Enable BB credit recovery (default is 1)") \
	P(int,		enable_fw_ag_rsp,	0,	"Enable firmware path for auto-good response (reset parameter field to 0) (default is 0)") \
	P(charp,	fw_diag_log_size,	0,	"Enables FW diagnostic logging to host memory (range: 0 or 128K - 1M, default: 0(disabled))") \
	P(charp,	fw_diag_log_level,	0,	"FW diagnostic logging level (range: 0-4, default: 0)") \
	P(int,		enable_dual_dump,	0,	"Enable/disable firmware dual dump (valid values: 0[disable] or 1[enable], default=0)") \
	P(int,		fw_dump_type,		1,	"Valid fw dump types: 1 - Dump to host, 2 - Dump to Flash, Default=1") \
	P(int,		enable_rq_no_buff_warns, 0,	"Enable RQ no buffer warnings generation (Default 0)") \
	P(int,		enable_auto_recovery,	1,	"Enable auto recovery support (Default 1)") \
	P(int,		max_remote_nodes,	2048,	"Max number of remote node objects to allocate") \
	P(int,		enable_dpp,		0,	"Enable direct packet push (default is 0)") \
	P(int,		hbs_bufsize,		OCS_HBS_DEFAULT_BUFFER_SIZE,	"Host Backing Store buffers size. Buffers are configured only when HBS is enabled.") \
	P(int,		io_pool_cache_num,	4,	"Number of caches to hold IOs freed/alloced from an io pool") \
	P(int,		io_pool_cache_thresh,	1000,	"Cache threshold while relasing IO to io pool") \
	P(int,		enable_tdz,		1,	"enable target peer zoning")
#if defined(OCS_USPACE_SPDK)

#if defined(OCS_USPACE_SPDK_UPSTREAM)
#define FC_PARAM_DEF_FILTER_DEF			"0x28ff30f0,0x08ff06ff,0,0,0,0,0,0"
#define FC_PARAM_DEF_TARGET_FLAGS		0x2
#else
#define FC_PARAM_DEF_FILTER_DEF			"0x28ff30f0,0x08ff06ff,0x08ff06ff,0x08ff09ff,0,0,0,0"
#define FC_PARAM_DEF_TARGET_FLAGS		0x3
#endif

#define OCS_FC_PARAM_LIST_SPDK \
	P(int,		target_flags,		FC_PARAM_DEF_TARGET_FLAGS,	"target type bit[0]-SCSI, bit[1]-NVME (default is 1)") \
	P(int,		enable_nsler,		0,				"Enables Sequence Level Error Recovery (default: 0)") \
	P(charp,	filter_def,		FC_PARAM_DEF_FILTER_DEF,	"REG_FCFI routing filter definitions (default \"" FC_PARAM_DEF_FILTER_DEF "\")")
#define OCS_FC_PARAM_LIST_NON_SPDK
#else
#define FC_PARAM_DEF_FILTER_DEF			"0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0"
#define OCS_FC_PARAM_LIST_NON_SPDK \
	P(int,		target_flags,		0x1,				"target type bit[0]-SCSI, bit[1]-NVME (default is 1)") \
	P(int,		enable_nsler,		0,				"Enables Sequence Level Error Recovery (default: 0)") \
	P(charp,	filter_def,		FC_PARAM_DEF_FILTER_DEF,	"REG_FCFI routing filter definitions (default \"" FC_PARAM_DEF_FILTER_DEF "\")")
#define OCS_FC_PARAM_LIST_SPDK
#endif

#if defined(OCS_INCLUDE_RAMD)
#define OCS_RAMD_PARAM_LIST \
	P(int,		global_ramdisc,		1,	"use global ramdiscs (default is 1)") \
	P(charp,	ramdisc_size,		"50m",	"ramdisc size: value as <number>[k|K|m|M|g|G] (default is 50M)") \
	P(int,		ramdisc_blocksize,	512,	"ramdisc block size (default is 512)") \
	P(int,		num_luns,		1,	"number of LUNs within a ramdisc (default is 1)") \
	P(int,		p_type,			0,	"protection type for T10 DIF") \
	P(int,		stub_res6_rel6,		0,	"stub for Reserve and Release 6 SCSI commands (default is 0") \
	P(int,		ramd_threading,		0,	"1 - start ramd threads, 0 - don't start ramd threads (default is 0)") \
	P(int,		thread_cmds,		0,	"1 - thread ramd commands, 0 don't thread ramd commands (default is 0)" ) \
	P(charp,	external_dif,		"",	"Enable proprietary end-to-end protection by specifying T10 Vendor ID (default is \"\")") \
	P(charp,	ramd_delay_range_us,	"0,0",	"ramd read/write delay range in micro-secs specified as min,max") \

#else
#define OCS_RAMD_PARAM_LIST
#endif

#define PARAM_LIST \
	OCS_PARAM_LOGLEVEL \
	OCS_PARAM_CTRLMASK \
	OCS_COMMON_PARAM_LIST \
	OCS_FC_PARAM_LIST \
	OCS_RAMD_PARAM_LIST \
	OCS_FC_PARAM_LIST_SPDK \
	OCS_FC_PARAM_LIST_NON_SPDK

/* PARAM_LIST: generate extern declarations for module parameters */
#define P(type, name, value, desc) extern type name;
PARAM_LIST
#undef P
#endif // __OCS_PARAMS_H__
