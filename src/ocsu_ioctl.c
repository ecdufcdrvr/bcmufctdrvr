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

/* 
 * File related to handling ioctls for userspace implementations
 */

#include "ocs.h"
#include "ocs_ocsu.h"
#include "ocs_socket.h"
#include "ocs_driver.h"
#include "ocsu_ioctl.h"

/**
 * @ingroup os
 * @brief  Copy user arguments in to kernel space for an ioctl
 * @par Description
 * This function is called at the beginning of an ioctl function
 * to copy the ioctl argument from user space to kernel space.
 *
 * Because the user space driver doesn't need to copy data between
 * address spaces this function just returns the argument.
 *
 * @param arg The argument passed to the ioctl function
 * @param size The size of the structure pointed to by arg
 *
 * @return A pointer to a kernel space copy of the argument on
 *         success; NULL on failure
 */
void *ocs_ioctl_preprocess(ocs_t *ocs, void *arg, size_t size)
{
	 return arg;
}

/**
 * @ingroup os
 * @brief  Copy results of an ioctl back to user space
 * @par Description
 * This function is called at the end of ioctl processing to
 * copy the argument back to user space.
 *
 * The user space driver doesn't need to copy data between
 * address spaces so nothing is needed here.
 *
 * @param arg The argument passed to the ioctl function
 * @param kern_ptr A pointer to the kernel space copy of the
 *		   argument
 * @param size The size of the structure pointed to by arg.
 *
 * @return Returns 0.
 */
int32_t ocs_ioctl_postprocess(ocs_t *ocs, void *arg, void *kern_ptr, size_t size)
{
	return 0;
}

/**
 * @ingroup os
 * @brief  Free memory allocated by ocs_ioctl_preprocess
 * @par Description
 * This function is called in the event of an error in ioctl
 * processing.  For operating environments where ocs_ioctlpreprocess
 * allocates memory, this call frees the memory without copying
 * results back to user space.
 *
 * For the user space driver, because no memory was allocated in
 * ocs_ioctl_preprocess, nothing needs to be done here.
 *
 * @param kern_ptr A pointer to the kernel space copy of the
 *                 argument
 * @param size The size of the structure pointed to by arg.
 *
 * @return Returns nothing.
 */
void ocs_ioctl_free(ocs_t *ocs, void *kern_ptr, size_t size)
{
        return;
}

void
ioctl_sig_handler(int signo)
{
}

int32_t
ocs_ioctl_server(ocs_thread_t *mythread)
{
	ocs_t *ocs = mythread->arg;
	ocs_skt_node_t node[1];
	ocs_skt_node_t client[1];
	int rc;
	uint32_t flags;
	int port = 9000 + ocs->instance_index;
	uint8_t unload = FALSE;
	struct sigaction act;

	ocs_log_debug(ocs, " %s started\n", mythread->name);

	rc = ocs_skt_server(node, port);
	if (rc) {
		ocs_log_err(ocs, "Error: ocs_skt_server() port %d failed: %d\n", port, rc);
		return -1;
	}

	/* Register signal handler for this threads quit request */
	signal(SIGRTMIN, ioctl_sig_handler);

	/* Make sure that system calls may be interrupted */
	if (sigaction(SIGRTMIN, NULL, &act) == -1) {
		return -1;
	}

	act.sa_flags &= ~SA_RESTART;
	if (sigaction(SIGRTMIN, &act, NULL) == -1) {
		return -1;
	}

	while (!unload && !ocs_thread_terminate_requested(mythread)) {
		uint32_t i;
		uint32_t cmd = 0;
		uint32_t c2scount = 0;
		ocs_skt_frame_t **c2sbufs = NULL;

		rc = ocs_skt_wait(node, client);
		if (rc) {
			break;
		}
		ocs_log_debug(ocs, "port %d: opening connection from: %s\n", port, client->ipaddr);

		while (!ocs_thread_terminate_requested(mythread)) {
			char *p = ocs_skt_str_read(client, &flags);
			if (p == NULL) {
				break;
			}

			i = ocs_sscanf(p, "ocs:ioctl:%x:%d", &cmd, &c2scount);
			if (i != 2) {
				ocs_log_err(ocs, "ocs_sscanf scanned %d argument instead of 2 !\n", i);
				free(p);
				break;
			}
			free(p);

			/* Allocate client to server buffers */
			c2sbufs = malloc(sizeof(*c2sbufs) * c2scount);
			if (c2sbufs == NULL) {
				ocs_log_err(ocs, "s: Error: malloc failed\n");
				break;
			}
			memset(c2sbufs, 0, sizeof(*c2sbufs) * c2scount);

			for (i = 0; i < c2scount; i ++) {
				c2sbufs[i] = ocs_skt_frame_read(client);
			}

			/* Expect terminator */
			p = ocs_skt_str_read(client, &flags);
			if (strcmp(p, "ocs:ioctl:end")) {
				ocs_log_test(ocs, "s: missing end of request got %s\n", p);
				free(p);
				for (i = 0; i < c2scount; i ++) {
					ocs_skt_frame_free(c2sbufs[i]);
				}
				free(c2sbufs);
				continue;
			}
			free(p);

			/* Execute the ioctl request */
			switch(cmd) {
			case OCS_IOCTL_CMD_TEST: {
				ocs_ioctl_test_t *req = c2sbufs[0]->buffer;

				rc = ocs_device_ioctl(ocs, cmd, (unsigned long) req);
				ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, 1);
				ocs_skt_write(client, req, sizeof(*req), 0);
				break;
			}

			case OCS_IOCTL_CMD_ELXU_MBOX: {
				ocs_ioctl_elxu_mbox_t *r_req = c2sbufs[0]->buffer;
				ocs_ioctl_elxu_mbox_t l_req;
				uint32_t outbufcount = 1;

				l_req = *r_req;
				l_req.in_addr = (uint64_t) c2sbufs[1]->buffer;
				l_req.out_addr = 0ll;
				if (r_req->out_bytes) {
					l_req.out_addr = (uint64_t) malloc(r_req->out_bytes);
					if ((void*)l_req.out_addr == NULL) {
						ocs_log_err(ocs, "ioctl(%d): malloc failed\n", __LINE__);
						ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", -1, 0);
						break;
					}
					outbufcount++;
				}
				rc = ocs_device_ioctl(ocs, cmd, (unsigned long) &l_req);
				ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, rc ? 1 : outbufcount);
				ocs_skt_write(client, &l_req, sizeof(l_req), 0);
				if ((rc == 0) && (l_req.out_addr != 0ll)) {
					ocs_skt_write(client, (void*) l_req.out_addr, l_req.out_bytes, 0);
				}
				free((void*)l_req.out_addr);
				break;
			}

			case OCS_IOCTL_CMD_DRIVER_INFO: {
				ocs_ioctl_driver_info_t req;

				rc = ocs_device_ioctl(ocs, cmd, (unsigned long) &req);
				ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, 1);
				ocs_skt_write(client, &req, sizeof(req), 0);
				break;
			}

			case OCS_IOCTL_CMD_ECD_HELPER: {
				ocs_ioctl_ecd_helper_t *req = c2sbufs[0]->buffer;

				rc = ocs_device_ioctl(ocs, cmd, (unsigned long) &req);
				ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, 1);
				ocs_skt_write(client, &req, sizeof(req), 0);
				break;
			}

			case OCS_IOCTL_CMD_VPORT: {
				ocs_ioctl_vport_t *req;

				req = c2sbufs[0]->buffer;
				rc = ocs_device_ioctl(ocs, cmd, (unsigned long) req);
				ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, 1);
				ocs_skt_write(client, req, sizeof(*req), 0);
				break;
			}

			case OCS_IOCTL_CMD_GET_DDUMP: {
				ocs_ioctl_ddump_t *r_req;
				ocs_ioctl_ddump_t l_req;

				r_req = c2sbufs[0]->buffer;

				/* Allocate a buffer for the response */
				l_req.user_buffer = malloc(r_req->user_buffer_len);
				if (l_req.user_buffer == NULL) {
					ocs_log_err(ocs, "GET_DDUMP: malloc failed\n");
					ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", -1, 0);
					break;
				}
				l_req.user_buffer_len = r_req->user_buffer_len;
				l_req.args = r_req->args;

				rc = ocs_device_ioctl(ocs, cmd, (unsigned long) &l_req);

				/* start message */
				ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, rc ? 1 : 2);

				r_req->bytes_written = l_req.bytes_written;
				ocs_skt_write(client, r_req, sizeof(*r_req), 0);
				if (rc == 0) {
					ocs_skt_write(client, l_req.user_buffer, l_req.bytes_written, 0);
				}
				free(l_req.user_buffer);

				break;
			}

			case OCS_IOCTL_CMD_MGMT_LIST:
			case OCS_IOCTL_CMD_MGMT_GET_ALL:
			{
				ocs_ioctl_mgmt_buffer_t *r_req = c2sbufs[0]->buffer;
				ocs_ioctl_mgmt_buffer_t l_req;

				l_req.user_buffer_len = r_req->user_buffer_len;
				l_req.user_buffer = malloc(r_req->user_buffer_len);
				l_req.get_all = r_req->get_all;
				if (l_req.user_buffer == NULL) {
					ocs_log_err(ocs, "ioctl(%d): malloc failed\n", __LINE__);
					ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, 0);
					break;
				}

				rc = ocs_device_ioctl(ocs, cmd, (unsigned long) &l_req);

				ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, rc ? 1 : 2);
				r_req->bytes_written = l_req.bytes_written;
				ocs_skt_write(client, r_req, sizeof(*r_req), 0);

				if (rc == 0) {
					ocs_skt_write(client, l_req.user_buffer, l_req.bytes_written, 0);
				}
				free(l_req.user_buffer);
				break;
			}

			case OCS_IOCTL_CMD_MGMT_GET:
			{
				ocs_ioctl_cmd_get_t *r_req = c2sbufs[0]->buffer;
				ocs_ioctl_cmd_get_t l_req;

				l_req.name = c2sbufs[1]->buffer;
				l_req.name_len = ocs_strlen(c2sbufs[1]->buffer);
				l_req.value = malloc(r_req->value_length);
				if (l_req.value == NULL) {
					ocs_log_err(ocs, "ioctl/GET_STATUS: malloc failed\n");
					ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, 0);
					break;
				}
				l_req.value_length = r_req->value_length;

				rc = ocs_device_ioctl(ocs, cmd, (unsigned long) &l_req);

				ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, rc ? 1 : 2);
				r_req->value_length = l_req.value_length;
				ocs_skt_write(client, r_req, sizeof(*r_req), 0);

				if (rc == 0) {
					ocs_skt_write(client, l_req.value, l_req.value_length, 0);
				}

				free(l_req.value);
				break;
			}

			case OCS_IOCTL_CMD_MGMT_SET:
			{
				ocs_ioctl_cmd_set_t *r_req = c2sbufs[0]->buffer;
				ocs_ioctl_cmd_set_t l_req;

				l_req.name = c2sbufs[1]->buffer;
				l_req.name_len = ocs_strlen(c2sbufs[1]->buffer);
				l_req.value = c2sbufs[2]->buffer;
				rc = ocs_device_ioctl(ocs, cmd, (unsigned long) &l_req);
				ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, 1);
				r_req->result = l_req.result;
				ocs_skt_write(client, r_req, sizeof(*r_req), 0);
				break;
			}

			case OCS_IOCTL_CMD_MGMT_EXEC:
			{
				ocs_ioctl_action_t *r_req = c2sbufs[0]->buffer;
				ocs_ioctl_action_t l_req;

				l_req.name =  c2sbufs[1]->buffer;
				l_req.name_len = ocs_strlen(c2sbufs[1]->buffer);
				l_req.arg_in = c2sbufs[2]->buffer;
				l_req.arg_in_length = c2sbufs[2]->buffer_len;
				l_req.arg_out = malloc(r_req->arg_out_length);
				if (l_req.arg_out == NULL) {
					ocs_log_err(ocs, "ioctl(%d) malloc failed\n", __LINE__);
					ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, 0);
					break;
				}

				l_req.arg_out_length = r_req->arg_out_length;

				rc = ocs_device_ioctl(ocs, cmd, (unsigned long) &l_req);
				ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, rc ? 1 : 2);
				r_req->arg_out_length = l_req.arg_out_length;
				r_req->result = l_req.result;
				ocs_skt_write(client, r_req, sizeof(*r_req), 0);
				if (rc == 0) {
					ocs_skt_write(client, l_req.arg_out, l_req.arg_out_length, 0);
				}
				free(l_req.arg_out);

				break;
			}

			case OCS_IOCTL_CMD_CONNECTION_INFO: {
				ocs_ioctl_connections_t *r_req = c2sbufs[0]->buffer;
				ocs_ioctl_connections_t l_req;
				int32_t connections_mem_size;

				l_req.max_connections = r_req->max_connections;
				connections_mem_size = l_req.max_connections * sizeof(ocs_ioctl_connection_info_t);
				l_req.connections = malloc(connections_mem_size);
				if (l_req.connections == NULL) {
					ocs_log_err(ocs, "ioctl/GET_STATUS: malloc failed\n");
					ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, 0);
					break;
				}

				rc = ocs_device_ioctl(ocs, cmd, (unsigned long) &l_req);

				ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, rc ? 1 : 2);
				r_req->num_connections = l_req.num_connections;
				ocs_skt_write(client, r_req, sizeof(*r_req), 0);

				if (rc == 0) {
					ocs_skt_write(client, l_req.connections, connections_mem_size, 0);
				}

				free(l_req.connections);
				break;
			}

			case OCS_IOCTL_CMD_LINK_ONLINE:
			case OCS_IOCTL_CMD_GEN_DUMP:
				rc = ocs_device_ioctl(ocs, cmd, (unsigned long) NULL);
				ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, rc ? 1 : 2);
				break;

			case OCS_IOCTL_CMD_UNLOAD:
				unload = TRUE;
				ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", rc, 0);
				rc = 0;
				break;

			default:
				ocs_log_test(ocs, "Error: unsupported IOCTL command: %08x\n", cmd);
				ocs_skt_str_write(client, 0, "ocs:ioctl:resp:%d:%d", -1, 0);
				break;
			}

			ocs_skt_str_write(client, 0, "ocs:ioctl:resp:done");

			for (i = 0; i < c2scount; i ++) {
				ocs_skt_frame_free(c2sbufs[i]);
			}
			free(c2sbufs);
		}
		ocs_log_debug(ocs, "port %d, closing connection from: %s\n", port, client->ipaddr);
		ocs_skt_close(client);
	}

	ocs_skt_close(node);
	ocs_log_debug(ocs, "%s terminatied\n", mythread->name);
	return 0;
}
