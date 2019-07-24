/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2018 Broadcom.  All Rights Reserved.
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
	P(int,		target,			1,	"enable target functionality(default is 1)") \
	P(int,		logdest,		1,	"logging destination (default is 0)\n" \
							"bit[0] = system log (default is 1)\n" \
							"bit[1] = ram log (default is 0)") \
	P(int,		loglevel,		7,	"logging level 0=CRIT, 1=ERR, 2=WARN, 3=INFO, 4=TEST, 5=DEBUG") \
	P(int,		ramlog_size,		1*1024*1024,	"size of ram logging buffer (default is 1M)") \
	P(int,		ddump_saved_size,	0,	"size of saved ddump (default is 0)") \
	P(int,		uspacemask,		0,	"bit[0] 0:thread per port(default), 1:poll from single thread\n" \
							"bit[1] 0:threads block on events(default), 1:threads poll") \
	P(charp,	hal_war_version,	NULL,	"Use this as f/w version for HAL workaround matching") \
	P(int,		num_scsi_ios,		8192,	"Number of SCSI IOs to allocate")

#if defined(OCS_INCLUDE_FC) && !defined(OCS_INCLUDE_ISCSI)
#define OCS_PARAM_CTRLMASK \
	P(int,		ctrlmask,		10,	"control bitmask (default is 2)\n" \
							"bit[0] disable autorsp on target reads\n" \
							"bit[1] disable autorsp on target writes\n" \
							"bit[3] enable target RSCN procesing\n" \
							"bit[4] Tgt Always verify DIF (clear AT/ATRT)")
#endif
#if !defined(OCS_INCLUDE_FC) && defined(OCS_INCLUDE_ISCSI)
#define OCS_PARAM_CTRLMASK \
	P(int,		ctrlmask,		0,	"control bitmask (default is 0)\n" \
							"bit[4] Tgt Always verify DIF (clear AT/ATRT)\n" \
							"bit[5] Tgt set ref tag and CRC on format")
#endif
#if defined(OCS_INCLUDE_FC)
#define OCS_FC_PARAM_LIST \
	P(int,		logmask,		0,	"logging bitmask (default is 0)") \
	P(int,		speed,			0,	"link speed in Mbps (0 - auto)") \
	P(int,		topology,		0,	"topology, 0 - auto, 1 - N_PORT, 2 - LOOP (default is 0)") \
	P(int,		holdoff_link_online,	0,	"hold off link online (default is 0)\n" \
							"0 - immediately\n" \
							"1 - until all PCI devices have been enumerated\n" \
							"2 - until brought up by mgmt request") \
	P(charp,	wwn_bump,		"0",	"xor'd to 64 bits of WWN's (default is 0)") \
	P(int,		ethernet_license,	0,	"ethernet license") \
	P(int,		auto_xfer_rdy_size,	0,	"The maximum sized write command to use a auto generated\n" \
							"transfer ready. (default is 0)") \
        P(int,		esoc,			0,	"To control the start offset computation functionality for auto transfer ready")\
	P(int,		auto_xfer_rdy_xri_cnt,   1500,	"Number of XRIs posted")\
	P(int,		enable_hlm,		0,	"enable high login mode") \
	P(int,		hlm_group_size,		8,	"high login mode group size") \
	P(int,		explicit_buffer_list,	0,	"Enable Explict Buffer Lists 0 - false, 1 - true (default is false)") \
	P(int,		dif_separate,		0,	"1 - DIF block is separate, 0 - DIF block is interleaved") \
	P(int,		num_vports,		0,	"Number of NPIV ports (default is 0)") \
	P(int,		external_loopback,	0,	"External loopback mode (default is 0)") \
	P(charp,	queue_topology,		"eq cq rq cq mq $nulp($nwq(cq wq:ulp=$rpt1)) cq wq:len=256:class=1", "Queue topology string " \
						"(default \"eq cq rq cq mq $nulp($nwq(cq wq:ulp=$rpt1)) cq wq:class=1\")") \
	P(int,		target_io_timer,	0,	"Timeout value, in seconds, for target commands (default 0 - timer disabled)") \
	P(int,		hal_bounce,		0,	"HAL bounce, 0 - no bounce, 1 - bounce (default 0)") \
	P(int,		rq_threads,		0,	"The number of RQ threads to create (default 0)") \
	P(charp,	filter_def,		"0x28ff30f0,0x08ff06ff,0,0,0,0,0,0", "REG_FCFI routing filter definitions (default \"0,0,0,0\")") \
	P(int,		watchdog_timeout,	0,	"Watchdog timeout") \
	P(int,		sliport_healthcheck,	1,	"enable sliport health check (0 - disabled, 1 - enabled)")
#else
#define OCS_FC_PARAM_LIST
#endif

#if defined(OCS_INCLUDE_ISCSI)
#define OCS_ISCSI_PARAM_LIST \
	P(int,		num_cxns,		1,		"number of connections (default is 1)") \
	P(int,		enable_sgl_chaining,	1,		"enable sgl chaining functionality(default is 1)") \
	P(int,		enable_immediate_data,	1,		"enable immediate data support (default is 1)") \
	P(int,		enable_hdr_digest,	1,		"enable header digest support (default is 1)") \
	P(int,		enable_data_digest,	1,		"enable data digest support (default is 1)") \
	P(int,		enable_jumbo_frame,	1,		"enable jumbo frame support (default is 1)") \
	P(int,		max_sge_size,		65535,		"Maximum SGE size (default is 65535)") \
	P(int,		post_sgl_config,	0,		"Post SGL configuration\n" \
								"0 - chip default\n" \
								"1 - v0_32 SGEs\n" \
								"2 - v0_256 SGEs\n" \
								"3 - v1_32 SGEs\n" \
								"4 - v1_256 SGEs") \
	P(charp,	local_ip,		"192.168.0.106",	"Local IP address (default is 192.168.0.106)") \
	P(charp,	local_subnet,		"255.255.255.0","Local subnet (default is 255.255.255.0)") \
	P(charp,	gateway,		"",		"gateway IP address (default is no gateway)") \
	P(int,		tcp_window_size,	65536,		"TCP window size (default is 65536)") \
	P(int,		tcp_window_scale,	4,		"TCP window scaling shift (default is 4)") \
	P(int,		num_tgts,		1,		"number of targets per port (default is 1)") \

#else

#define OCS_ISCSI_PARAM_LIST
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


#else
#define OCS_RAMD_PARAM_LIST
#endif

#define PARAM_LIST \
	OCS_PARAM_CTRLMASK \
	OCS_COMMON_PARAM_LIST \
	OCS_FC_PARAM_LIST \
	OCS_ISCSI_PARAM_LIST \
	OCS_RAMD_PARAM_LIST

/* PARAM_LIST: generate extern declarations for module parameters */
#define P(type, name, value, desc) extern type name;
PARAM_LIST
#undef P
#endif // __OCS_PARAMS_H__
