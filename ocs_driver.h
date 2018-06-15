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
 * Common OS to OCS driver interface API
 *
 */

#if !defined(__OCS_DRIVER_H__)
#define __OCS_DRIVER_H__

#define for_each_ocs(i, ocs) \
        for (i = 0; (ocs = ocs_get_instance(i)), i < MAX_OCS_DEVICES; i++)

extern void *ocs_device_alloc(uint32_t nid);
extern int32_t ocs_device_interrupts_required(ocs_t *ocs);
extern int32_t ocs_device_attach(ocs_t *ocs);
extern int32_t ocs_device_detach(ocs_t *ocs);
extern void ocs_device_free(ocs_t *ocs);
extern int ocs_device_ioctl(ocs_t *ocs, unsigned int cmd, unsigned long arg);
extern int32_t ocs_device_init(void);
extern void ocs_device_init_complete(void);
extern void ocs_device_shutdown(void);
extern void ocs_device_shutdown_complete(void);
extern uint32_t ocsu_ini_get_initiators(ocs_t *ini_ocs_port[],
					uint32_t max_ports);

#endif // __OCS_DRIVER_H__
