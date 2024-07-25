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

#if !defined(__OCS_FEMUL_H__)
#define __OCS_FEMUL_H__

#if defined(ENABLE_FABRIC_EMULATION)

#define OCS_NS_MAX_RECORDS		100				/*<< Max number of directory services records */

typedef struct ocs_ns_record_s ocs_ns_record_t;

/* Fabric Emulation API */
extern int32_t ocs_femul_init(ocs_domain_t *domain);
extern int32_t ocs_femul_shutdown(ocs_domain_t *domain);
extern int32_t ocs_femul_portid_alloc(ocs_domain_t *domain);
extern void ocs_femul_portid_free(ocs_domain_t *domain, int32_t portid);
extern int32_t ocs_femul_process_flogi(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
extern int32_t ocs_femul_process_fdisc(ocs_io_t *io, fc_header_t *hdr, void *payload, uint32_t payload_len);
extern int32_t ocs_femul_process_fc_gs(const char *funcname, ocs_io_t *io, ocs_sm_event_t evt, fc_header_t *hdr, void *payload, uint32_t payload_len);
extern int32_t ocs_femul_process_scr(ocs_io_t *node, fc_header_t *hdr, void *payload, uint32_t payload_len);
extern void ocs_femul_sport_attach(ocs_sport_t *sport);
extern int32_t ocs_femul_ports_attached(ocs_domain_t *domain);

extern ocs_ns_t *ocs_ns_attach(ocs_domain_t *domain, uint32_t max_ports);
extern void ocs_ns_detach(ocs_ns_t *ns);
extern ocs_ns_record_t *ocs_ns_alloc(ocs_ns_t *ns, uint32_t port_id);
extern void ocs_ns_free(ocs_ns_t *ns, uint32_t port_id);
extern ocs_ns_record_t *ocs_ns_find_port_id(ocs_ns_t *ns, uint32_t port_id);
extern ocs_ns_record_t *ocs_ns_find_port_name(ocs_ns_t *ns, uint64_t port_name);
extern ocs_ns_record_t *ocs_ns_enumerate(ocs_ns_t *ns, ocs_ns_record_t *nsrec);
extern int32_t ocs_ddump_ns(ocs_textbuf_t *textbuf, ocs_ns_t *ns);

#endif
#endif // __OCS_FEMUL_H__


