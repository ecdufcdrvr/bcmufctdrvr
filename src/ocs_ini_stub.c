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

#include "ocs.h"

/*!
 * @defgroup scsi_api_initiator SCSI Initiator
 */

/**
 * @brief Initialize initiator client.
 *
 * @par Description
 * Called by OS initialization code to initialize initiator
 * client. This is called prior to any PCI device enumeration.
 *
 * @param initiator TRUE for initiator mode
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */
int32_t
ocs_scsi_ini_driver_init(int32_t initiator)
{
	return 0;
}

/**
 * @brief Initialize SCSI IO.
 *
 * @par Description
 * Initialize SCSI IO, this function is called once per IO during IO pool
 * allocation so that the initiator client may initialize any of its own private
 * data.
 *
 * @param io Pointer to SCSI IO object.
 *
 * @return Returns 0 on success, a negative error code value on failure.
 */
int32_t
ocs_scsi_ini_io_init(ocs_io_t *io)
{
	return 0;
}

/**
 * @brief Uninitialize SCSI IO.
 *
 * @par Description
 * Uninitialize initiator client private data in a SCSI io object.
 *
 * @param io Pointer to SCSI IO object.
 *
 * @return Returns 0 on success, a negative error code value on failure.
 */
int32_t
ocs_scsi_ini_io_exit(ocs_io_t *io)
{
	return 0;
}

/**
 * @brief Check Scsi_Host midlayer registered
 *
 * @param Pointer to OCS context
 *
 * @return returns TRUE if Scsi_Host ml is registered, otherwise FALSE.
 */
int32_t ocs_scsi_host_ml_registered(ocs_t *ocs)
{
	return FALSE;
}

/**
 * @brief initiator client exit
 *
 * @par Description
 * Called by OS driver-wide cleanup/exit code to cleanup
 * initiator client.
 *
 * @param initiator TRUE for initiator mode
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocs_scsi_ini_driver_exit(int32_t initiator)
{
	return 0;
}

/**
 * @ingroup scsi_api_initiator
 * @brief Initializes any initiator fields on the ocs structure.
 *
 * @par Description
 * Called by OS initialization code when a new device is discovered.
 *
 * @param ocs pointer to ocs
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

int32_t
ocs_scsi_ini_new_device(ocs_t *ocs)
{
	return 0;
}

/**
 * @ingroup scsi_api_initiator
 * @brief Tears down target members of ocs structure.
 *
 * @par Description
 * Called by OS code when device is removed.
 *
 * @param ocs pointer to ocs
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

int32_t
ocs_scsi_ini_del_device(ocs_t *ocs)
{
	return 0;
}

/**
 * @ingroup scsi_api_initiator
 * @brief accept new domain notification
 *
 * @par Description
 * Called by base drive when new domain is discovered.  An initiator-client
 * will accept this call to prepare for new remote node notifications
 * arising from ocs_scsi_new_target().
 *
 * The domain context has the element <b>ocs_scsi_ini_domain_t ini_domain</b> which is
 * declared by the initiator-client code and is used for initiator-client private data.
 *
 * This function will only be called if the base-driver has been enabled for initiator
 * capability.
 *
 * Note that this call is made to initiator-client backends, the ocs_scsi_tgt_new_domain()
 * function is called to target-server backends.
 *
 * @param domain pointer to domain
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t ocs_scsi_ini_new_domain(ocs_domain_t *domain)
{
	ocs_assert(domain, -1);
	ocs_assert(domain->ocs, -1);
	ocs_log_debug(domain->ocs, "ocs_scsi_ini_new_domain\n");
	return 0;
}


int32_t ocs_scsi_host_update_supported_speed(ocs_t *ocs)
{
	return 0;
}
/**
 * @ingroup scsi_api_initiator
 * @brief accept domain lost notification
 *
 * @par Description
 * Called by base-driver when a domain goes away.  An initiator-client will
 * use this call to clean up all domain scoped resources.
 *
 * This function will only be called if the base-driver has been enabled for initiator
 * capability.
 *
 * Note that this call is made to initiator-client backends, the ocs_scsi_tgt_del_domain()
 * function is called to target-server backends.
 *
 * @param domain pointer to domain
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
void ocs_scsi_ini_del_domain(ocs_domain_t *domain)
{
	ocs_assert(domain);
	ocs_assert(domain->ocs);
}

int32_t ocs_scsi_ini_new_sport(ocs_sport_t *sport)
{
	ocs_log_debug(sport->domain->ocs, "New SPORT: %s\n", sport->display_name);
	return 0;
}

void ocs_scsi_ini_del_sport(ocs_sport_t *sport)
{
	ocs_log_debug(sport->domain->ocs, "Del SPORT: %s\n", sport->display_name);
}

/**
 * @ingroup scsi_api_initiator
 * @brief receive notification of a new SCSI target node
 *
 * @par Description
 * Sent by base driver to notify an initiator-client of the presense of a new
 * remote target.   The initiator-server may use this call to prepare for
 * inbound IO from this node.
 *
 * This function is only called if the base driver is enabled for initiator capability.
 *
 * @param node pointer to new remote initiator node
 *
 * @return none
 *
 * @note
 */

int32_t ocs_scsi_new_target(ocs_node_t *node)
{
	return 0;
}

/**
 * @ingroup scsi_api_initiator
 * @brief Delete a SCSI target node
 *
 * @par Description
 * Sent by base driver to notify a initiator-client that a remote target is now gone.
 * The initiator-client should terminate and cleanup all I/O's associated with the node.
 *
 * The ocs_node_t structure has and elment of type ocs_scsi_ini_node_t named
 * ni_node that is declared and used by a target-server for private
 * information.
 *
 * This function is only called if the base driver is enabled for target capability.
 *
 * @param node pointer node being deleted
 *
 * @return Returns OCS_SCSI_CALL_ASYNC if target delete is queued for async completion
 * and OCS_SCSI_CALL_COMPLETE if call completed or error.
 *
 */
int32_t ocs_scsi_del_target(ocs_node_t *node)
{
	ocs_assert(node, OCS_SCSI_CALL_COMPLETE);
	ocs_assert(node->ocs, OCS_SCSI_CALL_COMPLETE);
	ocs_log_debug(node->ocs, "ocs_scsi_del_target\n");
	return OCS_SCSI_CALL_COMPLETE;
}

void ocs_scsi_ini_ddump(ocs_textbuf_t *textbuf, ocs_scsi_ddump_type_e type, void *obj)
{
}
