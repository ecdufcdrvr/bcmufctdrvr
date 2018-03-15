/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2018, Broadcom.  All Rights Reserved.
 *  The term "Broadcom" refers to Broadcom Limited and/or its subsidiaries.
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
 * Stubs for functions, so that not every driver needs to have these.
 */

#include "ocs.h"

/**
 * @brief Notification from base driver that domain is in force-free path.
 *
 * @par Description Domain is forcefully going away.  Cleanup any resources associated with it.
 *
 * @param domain Pointer to domain being free'd.
 *
 * @return None.
 */

static inline void
ocs_scsi_notify_domain_force_free(ocs_domain_t *domain)
{
	/* Nothing to do */
	return;
}

/**
 * @brief Notification from base driver that sport is in force-free path.
 *
 * @par Description Sport is forcefully going away.  Cleanup any resources associated with it.
 *
 * @param sport Pointer to sport being free'd.
 *
 * @return None.
 */

static inline void
ocs_scsi_notify_sport_force_free(ocs_sport_t *sport)
{
	/* Nothing to do */
	return;
}


/**
 * @brief Notification from base driver that node is in force-free path.
 *
 * @par Description Node is forcefully going away.  Cleanup any resources associated with it.
 *
 * @param node Pointer to node being free'd.
 *
 * @return None.
 */

static inline void
ocs_scsi_notify_node_force_free(ocs_node_t *node)
{
	/* Nothing to do */
	return;
}
