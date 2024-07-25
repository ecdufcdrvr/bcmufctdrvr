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
 * Declarations for OCS socket API used by user space driver for inter-communication
 * between the IOCTL RPC server and client applications
 */


#if !defined(__OCS_SOCKET_H__)
#define __OCS_SOCKET_H__

#include <stdint.h>
#include <netinet/in.h>
#define OCS_IPC_FRAME_HEADER_MAGIC 0xba5eba11
#define OCS_IPC_FLAGS_MORE	(1U << 0)
typedef struct {
	uint32_t magic;
	uint32_t len;
	uint32_t flags;
	uint32_t payload_check;
	uint32_t header_check;
} ocs_skt_frame_header_t;

typedef struct {
	int s;
	uint32_t eof:1;
	char ipaddr[INET_ADDRSTRLEN];
} ocs_skt_node_t;

typedef struct {
	void *buffer;
	uint32_t buffer_len;
	uint32_t flags;
} ocs_skt_frame_t;

void _errprintf(char *fmt, ...);

extern int ocs_skt_client(ocs_skt_node_t *node, const char *hostport, int devidx);
extern int ocs_skt_server(ocs_skt_node_t *node, uint32_t port);
extern int ocs_skt_close(ocs_skt_node_t *node);
extern int ocs_skt_wait(ocs_skt_node_t *node, ocs_skt_node_t *client);
extern int ocs_skt_write(ocs_skt_node_t *node, void *buffer, uint32_t buffer_len, uint32_t flags);
extern int ocs_skt_read(ocs_skt_node_t *node, void *buffer, uint32_t buffer_len, uint32_t *pflags);
extern ocs_skt_frame_t *ocs_skt_frame_read(ocs_skt_node_t *node);
extern void ocs_skt_frame_free(ocs_skt_frame_t *frame);
extern int ocs_skt_str_write(ocs_skt_node_t *node, uint32_t flags, char *fmt, ...);
extern char *ocs_skt_str_read(ocs_skt_node_t *node, uint32_t *pflags);

#endif // __OCS_SOCKET_H__
