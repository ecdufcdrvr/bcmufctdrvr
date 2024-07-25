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
 *
 */

#if !defined(__OCS_NVME_STUB_H__)
#define __OCS_NVME_STUB_H__

int ocs_nvme_validate_initiator(ocs_node_t *node);
int ocs_nvme_process_abts(ocs_node_t *node, uint16_t oxid, uint16_t rxid);
int ocs_nvme_process_flush_bls(ocs_node_t *node, uint16_t oxid, uint16_t rxid,
			       uint32_t sler_qual, bool ht, uint16_t flush_count);
int ocs_nvme_hw_port_quiesce(ocs_t *ocs);
int ocs_nvme_hw_port_free(ocs_t *ocs);
int ocs_nvme_hw_port_reinit(ocs_t *ocs);

int ocs_nvme_new_target(ocs_node_t *node);
int ocs_nvme_del_target(ocs_node_t *node);
int ocs_nvme_new_initiator(ocs_node_t *node, void *cbdata);
int ocs_nvme_del_initiator(ocs_node_t *node, void *cbdata);

int ocs_nvme_tgt_new_domain(ocs_domain_t *domain);
int ocs_nvme_tgt_del_domain(ocs_domain_t *domain);
int ocs_nvme_tgt_new_sport(ocs_sport_t *sport);
int ocs_nvme_tgt_del_sport(ocs_sport_t *sport);
int ocs_nvme_tgt_new_device(ocs_t *ocs);
int ocs_nvme_tgt_del_device(ocs_t *ocs);

int ocs_nvme_ini_new_domain(ocs_domain_t *domain);
int ocs_nvme_ini_del_domain(ocs_domain_t *domain);
int ocs_nvme_ini_new_sport(ocs_sport_t *sport);
int ocs_nvme_ini_del_sport(ocs_sport_t *sport);
int ocs_nvme_ini_new_device(ocs_t *ocs);
int ocs_nvme_ini_del_device(ocs_t *ocs);

bool ocs_tgt_nvme_backend_enabled(ocs_t *ocs, ocs_sport_t *sport);
bool ocs_ini_nvme_backend_enabled(ocs_t *ocs, ocs_sport_t *sport);
bool ocs_nvme_backend_enabled(ocs_t *ocs, ocs_sport_t *sport);

int ocs_nvme_tgt_driver_init(void);
int ocs_nvme_tgt_driver_exit(void);

#endif // __OCS_NVME_STUB_H__
