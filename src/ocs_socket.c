/*
 * Copyright (c) 2011-2015, Emulex
 * All rights reserved.
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
 * OCS socket API used by user space driver for inter-communication
 * between the IOCTL RPC server and client applications
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>

#include <sys/types.h>
#include <netdb.h>

#include "ocs_socket.h"
#include <crc32.h>

#if !defined(MIN)
#define MIN(x,y)	((x) < (y) ? (x) : (y))
#endif

#define ENABLE_TRACE		0
#if ENABLE_TRACE
#define trace(fmt, ...)		do {printf("%s: " fmt, __func__ , ##__VA_ARGS__); fflush(stdout); } while (0)
#else
#define trace(...)
#endif

static int readuntil(int fd, void *buffer, uint32_t count);

void _errprintf(char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
}
#define ENABLE_ERROR_PRINTF	1
#if ENABLE_ERROR_PRINTF
#define errprintf(fmt, ...) _errprintf("Error: %s:" fmt, __func__, ##__VA_ARGS__)
#else
#define errprintf(fmt, ...)
#endif

/**
 * @brief Initialize a client node
 *
 * A socket is opened to the host:port given by 'path'
 *
 * @param node pointer to node to initialize
 * @param path host:port path
 * @param devidx device index
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

int ocs_skt_client(ocs_skt_node_t *node, const char *hostport, int devidx)
{
	char *s;
	int rc = 0;
	char *p;
	struct hostent *hostinfo;
	struct sockaddr_in servername;
	const char *hostname = "localhost";
	uint32_t port = 9000;

	s = strdup(hostport);
	/* If there's a ':', then the string is host:port */
	p = strchr(s, ':');
	if (p) {
		port = atoi(p+1);
		*p = 0;
	}
	hostname = s;

	port += devidx;

	trace("hostname %s port %d\n", hostname, port);

	memset(node, 0, sizeof(*node));
	memset(&servername, 0, sizeof(struct sockaddr_in));

	/* Create the socket. */
	node->s = socket(PF_INET, SOCK_STREAM, 0);
	if (node->s < 0)
	{
		errprintf("socket(client) %s\n", strerror(errno));
		rc = errno > 0 ? -errno : errno;
		goto done;
	}

	servername.sin_family = AF_INET;
	servername.sin_port = htons(port);
	hostinfo = gethostbyname(hostname);
	if (hostinfo == NULL)
	{
		errprintf("Unknown host %s\n", hostname);
		rc = errno > 0 ? -errno : errno;
		goto done;
	}
	servername.sin_addr = *(struct in_addr *) hostinfo->h_addr;

	/* Connect to the server. */
	if (connect(node->s, (struct sockaddr *) &servername, sizeof (servername)) < 0) {
		/* errprintf(": connect(client) %s\n", strerror(errno)); */
		rc = errno > 0 ? -errno : errno;
		goto done;
	}

done:
	free(s);
	return rc;
}

/**
 * @brief Initialize a server node
 *
 * A server socket is initialized on 'port'
 *
 * @param node pointer to node to initialize
 * @param port port to listen on
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

int ocs_skt_server(ocs_skt_node_t *node, uint32_t port)
{
	int rc = 0;
	struct sockaddr_in name;
	int flag;

	/* Ignore broken pipe signals ... happens if the client closes before we've completed
	 * a write
	 */

	signal(SIGPIPE, SIG_IGN);
	memset(node, 0, sizeof(*node));
	memset(&name, 0, sizeof(struct sockaddr_in));

	/* Create the socket.  */
	node->s = socket(PF_INET, SOCK_STREAM, 0);
	if (node->s < 0)
	{
		errprintf("socket %s\n", strerror(errno));
		return errno > 0 ? -errno : errno;
	}

	flag = 1;
	rc = setsockopt(node->s, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag));
	if (rc) {
		errprintf("setsockopt %s\n", strerror(errno));
		return errno > 0 ? -errno : errno;
	}

	/* Give the socket a name.  */
	name.sin_family = AF_INET;
	name.sin_port = htons(port);
	name.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(node->s, (struct sockaddr *) &name, sizeof (name)) < 0)
	{
		errprintf("bind %s\n", strerror(errno));
		return errno > 0 ? -errno : errno;
	}

	rc = listen(node->s, 1);
	if (rc != 0) {
		errprintf("listen %s\n", strerror(errno));
		return -1;
	}

	return rc;
}

/**
 * @brief wait for connection request
 *
 * server node waits and accepts a connection request
 *
 * @param node pointer to server node
 * @param client pointer to client node that is inialized following connection
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

int ocs_skt_wait(ocs_skt_node_t *node, ocs_skt_node_t *client)
{
	int rc = 0;
	struct sockaddr ss_addr;
	socklen_t ss_addr_len;

	memset(client, 0, sizeof(*client));

	ss_addr_len = sizeof(ss_addr);
	client->s = accept(node->s, &ss_addr, &ss_addr_len);
	if (client->s < 0) {
		if (errno == EINTR)
			printf("Socket accept system call Interrupted\n");
		else
			errprintf("accept failed: %s\n", strerror(errno));

		return -1;
	}
	strcpy(client->ipaddr, "<unknown>");
	if (ss_addr.sa_family == AF_INET) {
		struct sockaddr_in *in = (struct sockaddr_in*) &ss_addr;

		inet_ntop(in->sin_family, &in->sin_addr, client->ipaddr, sizeof(client->ipaddr));
	}
	return rc;
}

/**
 * @brief close a server or client socket
 *
 * The socket is closed
 *
 * @param node pointer to node
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

int ocs_skt_close(ocs_skt_node_t *node)
{
	int rc = 0;
	close(node->s);
	return rc;
}

/**
 * @brief send a frame
 *
 * Send a data frame
 *
 * @param node pointer to node
 * @param buffer pointer to data buffer to send
 * @param buffer_len length of send buffer in bytes
 * @param flags flags value to accomany data
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

int ocs_skt_write(ocs_skt_node_t *node, void *buffer, uint32_t buffer_len, uint32_t flags)
{
	int rc = 0;
	ocs_skt_frame_header_t header;
	struct iovec iov[2];
	uint32_t crc = crc32(0L, NULL, 0);

	trace("buf %p len %d\n", buffer, buffer_len);

	memset(&header, 0, sizeof(header));
	header.magic = OCS_IPC_FRAME_HEADER_MAGIC;
	header.len = buffer_len;
	header.flags = flags;
	header.payload_check = crc32(crc, buffer, buffer_len);
	header.header_check = crc32(crc, (void*)&header, sizeof(header) - sizeof(header.header_check));

	iov[0].iov_base = &header;
	iov[0].iov_len = sizeof(header);
	iov[1].iov_base = buffer;
	iov[1].iov_len = buffer_len;

	rc = writev(node->s, iov, 2);

	if (rc >= 0) {
		rc -= sizeof(header);
		if (rc < 0) {
			rc = -1;
		}
	}

	return rc;
}

/**
 * @brief flush inbound frame data
 *
 * Inbound data is received and discarded
 *
 * @param node pointer to node
 * @param len number of bytes to flush
 *
 * @return none
 */

static void ocs_skt_flush(ocs_skt_node_t *node, uint32_t len)
{
	int rc = 0;
	uint8_t buffer[256];
	int n;

	while (len > 0) {
		n = MIN(sizeof(buffer), len);
		rc = read(node->s, buffer, n);
		if (n <= 0) {
			errprintf("read failed: %d\n", rc);
			break;
		}
		len -= n;
	}
}

/**
 * @brief receive a frame header
 *
 * The header portion of a frame is read and checked
 *
 * @param node node to read from
 * @param header pointer to header to rad
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

static int ocs_skt_header_read(ocs_skt_node_t *node, ocs_skt_frame_header_t *header)
{
	int rc = 0;
	uint32_t crc = crc32(0L, NULL, 0);

	rc = readuntil(node->s, header, sizeof(*header));

	if (rc == 0) {
		node->eof = 1;
		return 0;
	}

	if (rc <= 0) {
		errprintf("read failed: %d\n", rc);
		return -1;
	}

	if (rc != sizeof(*header)) {
		errprintf("header size mismatch\n");
		return -1;
	}

	if (header->magic != OCS_IPC_FRAME_HEADER_MAGIC) {
		errprintf("header magic mismatch\n");
		return -1;
	}

	/* Check header CRC */
	crc = crc32(0L, NULL, 0);
	if (crc32(crc, (void*) header, sizeof(*header) - sizeof(header->header_check)) != header->header_check) {
		errprintf("header check bits compare failed\n");
		return -1;
	}

	return sizeof(ocs_skt_frame_header_t);
}

/**
 * @brief receive data buffer
 *
 * read and check a header and data buffer
 *
 * @param node pointer to node
 * @param buffer pointer to buffer to read into
 * @param buffer_len length of buffer
 * @param pflags pointer to location to store received flags
 *
 * @return returns number of bytes read, or a negative error code value for failure.
 */

int ocs_skt_read(ocs_skt_node_t *node, void *buffer, uint32_t buffer_len, uint32_t *pflags)
{
	int rc = 0;
	ocs_skt_frame_header_t header;
	uint32_t crc = crc32(0L, NULL, 0);

	rc = ocs_skt_header_read(node, &header);
	if (rc == 0) { // eof
		return 0;
	}
	if (rc < 0) {
		errprintf("ocs_skt_header_read failed: %d\n", rc);
		return rc;
	}

	/* Read in the buffer */
	if (header.len > buffer_len) {
		errprintf("buffer of %d bytes is too small for %d bytes of data\n", buffer_len, header.len);
		ocs_skt_flush(node, header.len);
		return -1;
	}

	rc = readuntil(node->s, buffer, header.len);
	if (rc <= 0) {
		errprintf("read failed: expected %d got %d\n", header.len, rc);
		return -1;
	}

	if (crc32(crc, buffer, header.len) != header.payload_check) {
		errprintf("payload check bits compare failed\n");
		return -1;
	}

	if (pflags) {
		*pflags = header.flags;
	}

	trace("%d read\n", rc);
	/* Return bytes read */
	return rc;
}

/**
 * @brief receive data frame
 *
 * Read and check a data buffer, allocating and returning a frame object.  The
 * frame must be free'd by the caller using ocs_skt_frame_free()
 *
 * @param node pointer to node
 *
 * @return returns allocated frame or NULL
 */

ocs_skt_frame_t *ocs_skt_frame_read(ocs_skt_node_t *node)
{
	int rc = 0;
	ocs_skt_frame_t *frame;
	ocs_skt_frame_header_t header;
	uint32_t crc = crc32(0L, NULL, 0);

	/* Read header */
	rc = ocs_skt_header_read(node, &header);

	if (rc == 0) { // eof
		trace("eof\n");
		return NULL;
	}

	if (rc < 0) {
		errprintf("ocs_skt_header_read failed: %d\n", rc);
		return NULL;
	}

	frame = malloc(sizeof(*frame));
	if (frame == NULL) {
		errprintf("malloc failed\n");
		return NULL;
	}
	memset(frame, 0, sizeof(*frame));

	/* allocate buffer */
	frame->flags = header.flags;
	frame->buffer_len = header.len;
	frame->buffer = malloc(frame->buffer_len);
	if (frame->buffer == NULL) {
		errprintf("malloc payload failed\n");
		ocs_skt_frame_free(frame);
		return NULL;
	}

	trace("buffer_len %d\n", frame->buffer_len);

	/* read buffer */
	rc = readuntil(node->s, frame->buffer, frame->buffer_len);
	if (rc < 0) {
		errprintf("error on read: %d\n", rc);
		ocs_skt_frame_free(frame);
		return NULL;
	}
	if (rc != (int)frame->buffer_len) {
		errprintf("shorted read, %d remaining\n", frame->buffer_len - rc);
	}

	/* Check CRC */
	if (crc32(crc, frame->buffer, frame->buffer_len) != header.payload_check) {
		errprintf("payload crc check failed\n");
		ocs_skt_frame_free(frame);
		return NULL;
	}

	trace("%d bytes read\n", frame->buffer_len);

	return frame;
}

/**
 * @brief free a data frame
 *
 * Free a data frame
 *
 * @param frame pointer to frame
 *
 * @return none
 */

void ocs_skt_frame_free(ocs_skt_frame_t *frame)
{
	if (frame) {
		if (frame->buffer) {
			free(frame->buffer);
		}
		free(frame);
	}
}

/**
 * @brief send a string
 *
 * A null terminated string is formatted and sent to the remote node.   The null
 * character is sent.
 *
 * @param node pointer to node
 * @param flags flags to send
 * @param fmt printf style format
 * @param ... optional arguments
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

int ocs_skt_str_write(ocs_skt_node_t *node, uint32_t flags, char *fmt, ...)
{
	char buf[256];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	trace("%s\n", buf);
	return ocs_skt_write(node, buf, strlen(buf) + 1, flags);
}

/**
 * @brief receive a string
 *
 * A null terminated string is received and returned.   The caller must free the string
 * using free()
 *
 * @param node pointer to node
 * @param pflags pointer to location to store flags
 *
 * @return returns pointer to string or null
 */

char *ocs_skt_str_read(ocs_skt_node_t *node, uint32_t *pflags)
{
	ocs_skt_frame_t *f;
	char *s = NULL;

	f = ocs_skt_frame_read(node);
	if (f) {
		s = strdup(f->buffer);
		if (pflags != NULL) {
			*pflags = f->flags;
		}
		ocs_skt_frame_free(f);
	}
	trace("%s\n", s);
	return s;
}

static int
readuntil(int fd, void *buffer, uint32_t count)
{
	int rc;
	int nread = 0;

	while(count > 0) {
		rc = read(fd, buffer, count);
		if (rc == 0) {
			break;
		}
		if (rc < 0) {
			return errno > 0 ? -errno : errno;
		}
		nread += rc;
		count -= rc;
		buffer += rc;
	}
	return nread;
}

