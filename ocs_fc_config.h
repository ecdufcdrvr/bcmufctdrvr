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
 * OCS FC/FCoE driver tuneable parameters
 */

#if !defined(__OCS_FC_CONFIG_H__)
#define __OCS_FC_CONFIG_H__

/***************************************************************************
 * Receive queue configuration
 */

#ifndef OCS_FC_RQ_SIZE_DEFAULT
#define OCS_FC_RQ_SIZE_DEFAULT			1024
#endif


/***************************************************************************
 * IO Configuration
 */

/**
 * @brief Defines the number of SGLs allocated on each IO object
 */
#ifndef OCS_FC_MAX_SGL
#define OCS_FC_MAX_SGL		64
#endif


/***************************************************************************
 * DIF Configuration
 */

/**
 * @brief Defines the DIF seed value used for the CRC calculation.
 */
#ifndef OCS_FC_DIF_SEED
#define OCS_FC_DIF_SEED		0
#endif

/***************************************************************************
 * Timeouts
 */
#ifndef OCS_FC_ELS_SEND_DEFAULT_TIMEOUT
#define OCS_FC_ELS_SEND_DEFAULT_TIMEOUT		0
#endif

#ifndef OCS_FC_ELS_DEFAULT_RETRIES
#define OCS_FC_ELS_DEFAULT_RETRIES		3
#endif

#ifndef OCS_FC_FLOGI_TIMEOUT_SEC
#define OCS_FC_FLOGI_TIMEOUT_SEC		5 /* shorter than default */
#endif

#ifndef OCS_FC_DOMAIN_SHUTDOWN_TIMEOUT_USEC
#define OCS_FC_DOMAIN_SHUTDOWN_TIMEOUT_USEC	30000000 /* 30 seconds */
#endif

/***************************************************************************
 * Watermark
 */
#ifndef OCS_WATERMARK_HIGH_PCT
#define OCS_WATERMARK_HIGH_PCT			90
#endif
#ifndef OCS_WATERMARK_LOW_PCT
#define OCS_WATERMARK_LOW_PCT			80
#endif
#ifndef OCS_IO_WATERMARK_PER_INITIATOR
#define OCS_IO_WATERMARK_PER_INITIATOR		8
#endif

#endif // __OCS_FC_CONFIG_H__

/* vim: set noexpandtab textwidth=120: */
