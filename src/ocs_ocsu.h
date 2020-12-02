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
 * Common OCS userspace driver API exports
 *
 */

#if !defined(__OCS_OCSU_H__)
#define __OCS_OCSU_H__

extern int ocsu_init(void);
extern int ocsu_wait(void);
extern int32_t ocsu_terminate_event_threads(ocs_t *ocs);
extern int32_t ocsu_wait_event(ocs_t *ocs, uint32_t num_interrupts);
extern int32_t ocsu_wait_event_list(ocs_t *devlist[], uint8_t *pending, uint32_t devcount);
extern int32_t ocsu_process_events(ocs_t *ocs);
extern int ocs_spdk_poller_stop(ocs_t *ocs);
extern int ocs_spdk_poller_start(ocs_t *ocs);
extern bool ocsu_device_remove(struct spdk_pci_addr *pci_addr);
extern bool ocsu_device_add(struct spdk_pci_addr *pci_addr);
extern int32_t ocs_ioctl_server(ocs_thread_t *mythread);
extern int32_t ocsu_spdk_worker_thread(ocs_thread_t *mythread);
extern void ocs_device_delete_worker_thread(ocs_t *ocs);
extern int ocs_send_msg_to_worker(ocs_t *ocs, ocs_spdk_worker_msg_t msg, bool sync,
		       ocs_spdk_worker_func func, void *func_arg1);
extern int32_t ocsu_spdk_shutdown_thread(ocs_thread_t *mythread);
extern void ocsu_shutdown(void);

#endif // __OCS_OCSU_H__
