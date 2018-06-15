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
 *
 */

#if !defined(__OCS_HAL_CONFIG_H__)
#define __OCS_HAL_CONFIG_H__

/**
 * @brief Defines the number of the RQ buffers for each RQ
 */

#ifndef OCS_HAL_RQ_NUM_HDR
#define OCS_HAL_RQ_NUM_HDR		1024
#endif

#ifndef OCS_HAL_RQ_NUM_PAYLOAD
#define OCS_HAL_RQ_NUM_PAYLOAD			1024
#endif

/**
 * @brief Defines the size of the RQ buffers used for each RQ
 */
#ifndef OCS_HAL_RQ_SIZE_HDR
#define OCS_HAL_RQ_SIZE_HDR		128
#endif

#ifndef OCS_HAL_RQ_SIZE_PAYLOAD
#define OCS_HAL_RQ_SIZE_PAYLOAD		2048
#endif

/*
 * @brief Define the maximum number of multi-receive queues
 */
#ifndef OCS_HAL_MAX_MRQS
#define OCS_HAL_MAX_MRQS			8
#endif

/*
 * @brief Define count of when to set the WQEC bit in a submitted
 * WQE, causing a consummed/released completion to be posted.
 */
#ifndef OCS_HAL_WQEC_SET_COUNT
#define OCS_HAL_WQEC_SET_COUNT			32
#endif

/*
 * @brief Send frame timeout in seconds
 */
#ifndef OCS_HAL_SEND_FRAME_TIMEOUT
#define OCS_HAL_SEND_FRAME_TIMEOUT		10
#endif

/*
 * @brief FDT Transfer Hint value, reads greater than this value
 * will be segmented to implement fairness.   A value of zero disables
 * the feature.
 */
#ifndef OCS_HAL_FDT_XFER_HINT
#define OCS_HAL_FDT_XFER_HINT			8192
#endif

#endif // __OCS_HAL_CONFIG_H__
