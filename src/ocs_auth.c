/*
 * Copyright (C) 2020 Broadcom. All Rights Reserved.
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
 * Functions for authentication protocol support - DH-CHAP for now
 */

#include "ocs.h"
#include "ocs_els.h"
#include "ocs_scsi_fc.h"
#include "ocs_device.h"
#include "ocs_fabric.h"
#include "ocs_auth.h"

/* Enum strings used for logging/tracing */
#define NAME(x)       case x:         return #x;
static const char *auth_msg_name(u8 msg_code)
{
        switch (msg_code) {
	NAME(AUTH_REJECT)
	NAME(AUTH_NEGOTIATE)
	NAME(AUTH_DONE)
	NAME(DHCHAP_CHALLENGE)
	NAME(DHCHAP_REPLY)
	NAME(DHCHAP_SUCCESS)
	default:
		return "unknown";
	}
}

static const char *auth_evt_name(enum ocs_auth_evt evt)
{
        switch (evt) {
	NAME(OCS_AUTH_EVT_START)
	NAME(OCS_AUTH_EVT_RESTART)
	NAME(OCS_AUTH_EVT_STOP)
	NAME(OCS_AUTH_EVT_MSG_SEND_FAIL)
	NAME(OCS_AUTH_EVT_NEXT_MSG_TIMEOUT)
	NAME(OCS_AUTH_EVT_NEG_RECV)
	NAME(OCS_AUTH_EVT_REJ_RECV)
	NAME(OCS_AUTH_EVT_DHCHAP_CHAL_RECV)
	NAME(OCS_AUTH_EVT_DHCHAP_REPLY_RECV)
	NAME(OCS_AUTH_EVT_DHCHAP_SUCCESS_RECV)
	default:
		return "unknown";
	}
};

static const char *dhchap_state_name(enum ocs_dhchap_state state)
{
        switch (state) {
	NAME(OCS_DHCHAP_STATE_INIT)
	NAME(OCS_DHCHAP_STATE_NEG_SENT)
	NAME(OCS_DHCHAP_STATE_CHAL_SENT)
	NAME(OCS_DHCHAP_STATE_REPLY_SENT)
	NAME(OCS_DHCHAP_STATE_SUCCESS_SENT)
	NAME(OCS_DHCHAP_STATE_DONE)
	default:
		return "unknown";
	}
}

static const char *auth_err_name(u8 code)
{
        switch (code) {
	NAME(AUTH_RJT_CODE_FAILURE)
	NAME(AUTH_RJT_CODE_LOGICAL_ERR)
	default:
		return "unknown";
	}
}

static const char *auth_expl_name(u8 expl)
{
        switch (expl) {
	NAME(AUTH_RJT_EXPL_MECH_UNUSABLE)
	NAME(AUTH_RJT_EXPL_DHGRP_UNUSABLE)
	NAME(AUTH_RJT_EXPL_HASH_UNUSABLE)
	NAME(AUTH_RJT_EXPL_AUTH_STARTED)
	NAME(AUTH_RJT_EXPL_AUTH_FAILED)
	NAME(AUTH_RJT_EXPL_BAD_PAYLOAD)
	NAME(AUTH_RJT_EXPL_BAD_PROTOCOL)
	NAME(AUTH_RJT_EXPL_RESTART_AUTH)
	NAME(AUTH_RJT_EXPL_CONCAT_NOSUPP)
	NAME(AUTH_RJT_EXPL_PROTO_VER_NOSUPP)
	default:
		return "unknown";
	}
}

/* DH Groups from FC-SP-2 Section 5.3.5 */
#define	DH_BASE			2
struct ocs_dh_group dh_group_array[5] = {
	{
		DH_GROUP_NULL, 0,
		{
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
		}
	},
	{
		DH_GROUP_1024, 128,
		{
			0xEE, 0xAF, 0x0A, 0xB9, 0xAD, 0xB3, 0x8D, 0xD6,
			0x9C, 0x33, 0xF8, 0x0A, 0xFA, 0x8F, 0xC5, 0xE8,
			0x60, 0x72, 0x61, 0x87, 0x75, 0xFF, 0x3C, 0x0B,
			0x9E, 0xA2, 0x31, 0x4C, 0x9C, 0x25, 0x65, 0x76,
			0xD6, 0x74, 0xDF, 0x74, 0x96, 0xEA, 0x81, 0xD3,
			0x38, 0x3B, 0x48, 0x13, 0xD6, 0x92, 0xC6, 0xE0,
			0xE0, 0xD5, 0xD8, 0xE2, 0x50, 0xB9, 0x8B, 0xE4,
			0x8E, 0x49, 0x5C, 0x1D, 0x60, 0x89, 0xDA, 0xD1,
			0x5D, 0xC7, 0xD7, 0xB4, 0x61, 0x54, 0xD6, 0xB6,
			0xCE, 0x8E, 0xF4, 0xAD, 0x69, 0xB1, 0x5D, 0x49,
			0x82, 0x55, 0x9B, 0x29, 0x7B, 0xCF, 0x18, 0x85,
			0xC5, 0x29, 0xF5, 0x66, 0x66, 0x0E, 0x57, 0xEC,
			0x68, 0xED, 0xBC, 0x3C, 0x05, 0x72, 0x6C, 0xC0,
			0x2F, 0xD4, 0xCB, 0xF4, 0x97, 0x6E, 0xAA, 0x9A,
			0xFD, 0x51, 0x38, 0xFE, 0x83, 0x76, 0x43, 0x5B,
			0x9F, 0xC6, 0x1D, 0x2F, 0xC0, 0xEB, 0x06, 0xE3,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
		}
	},
	{
		DH_GROUP_1280, 160,
		{
			0xD7, 0x79, 0x46, 0x82, 0x6E, 0x81, 0x19, 0x14,
			0xB3, 0x94, 0x01, 0xD5, 0x6A, 0x0A, 0x78, 0x43,
			0xA8, 0xE7, 0x57, 0x5D, 0x73, 0x8C, 0x67, 0x2A,
			0x09, 0x0A, 0xB1, 0x18, 0x7D, 0x69, 0x0D, 0xC4,
			0x38, 0x72, 0xFC, 0x06, 0xA7, 0xB6, 0xA4, 0x3F,
			0x3B, 0x95, 0xBE, 0xAE, 0xC7, 0xDF, 0x04, 0xB9,
			0xD2, 0x42, 0xEB, 0xDC, 0x48, 0x11, 0x11, 0x28,
			0x32, 0x16, 0xCE, 0x81, 0x6E, 0x00, 0x4B, 0x78,
			0x6C, 0x5F, 0xCE, 0x85, 0x67, 0x80, 0xD4, 0x18,
			0x37, 0xD9, 0x5A, 0xD7, 0x87, 0xA5, 0x0B, 0xBE,
			0x90, 0xBD, 0x3A, 0x9C, 0x98, 0xAC, 0x0F, 0x5F,
			0xC0, 0xDE, 0x74, 0x4B, 0x1C, 0xDE, 0x18, 0x91,
			0x69, 0x08, 0x94, 0xBC, 0x1F, 0x65, 0xE0, 0x0D,
			0xE1, 0x5B, 0x4B, 0x2A, 0xA6, 0xD8, 0x71, 0x00,
			0xC9, 0xEC, 0xC2, 0x52, 0x7E, 0x45, 0xEB, 0x84,
			0x9D, 0xEB, 0x14, 0xBB, 0x20, 0x49, 0xB1, 0x63,
			0xEA, 0x04, 0x18, 0x7F, 0xD2, 0x7C, 0x1B, 0xD9,
			0xC7, 0x95, 0x8C, 0xD4, 0x0C, 0xE7, 0x06, 0x7A,
			0x9C, 0x02, 0x4F, 0x9B, 0x7C, 0x5A, 0x0B, 0x4F,
			0x50, 0x03, 0x68, 0x61, 0x61, 0xF0, 0x60, 0x5B,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
		}
	},
	{
		DH_GROUP_1536, 192,
		{
			0x9D, 0xEF, 0x3C, 0xAF, 0xB9, 0x39, 0x27, 0x7A,
			0xB1, 0xF1, 0x2A, 0x86, 0x17, 0xA4, 0x7B, 0xBB,
			0xDB, 0xA5, 0x1D, 0xF4, 0x99, 0xAC, 0x4C, 0x80,
			0xBE, 0xEE, 0xA9, 0x61, 0x4B, 0x19, 0xCC, 0x4D,
			0x5F, 0x4F, 0x5F, 0x55, 0x6E, 0x27, 0xCB, 0xDE,
			0x51, 0xC6, 0xA9, 0x4B, 0xE4, 0x60, 0x7A, 0x29,
			0x15, 0x58, 0x90, 0x3B, 0xA0, 0xD0, 0xF8, 0x43,
			0x80, 0xB6, 0x55, 0xBB, 0x9A, 0x22, 0xE8, 0xDC,
			0xDF, 0x02, 0x8A, 0x7C, 0xEC, 0x67, 0xF0, 0xD0,
			0x81, 0x34, 0xB1, 0xC8, 0xB9, 0x79, 0x89, 0x14,
			0x9B, 0x60, 0x9E, 0x0B, 0xE3, 0xBA, 0xB6, 0x3D,
			0x47, 0x54, 0x83, 0x81, 0xDB, 0xC5, 0xB1, 0xFC,
			0x76, 0x4E, 0x3F, 0x4B, 0x53, 0xDD, 0x9D, 0xA1,
			0x15, 0x8B, 0xFD, 0x3E, 0x2B, 0x9C, 0x8C, 0xF5,
			0x6E, 0xDF, 0x01, 0x95, 0x39, 0x34, 0x96, 0x27,
			0xDB, 0x2F, 0xD5, 0x3D, 0x24, 0xB7, 0xC4, 0x86,
			0x65, 0x77, 0x2E, 0x43, 0x7D, 0x6C, 0x7F, 0x8C,
			0xE4, 0x42, 0x73, 0x4A, 0xF7, 0xCC, 0xB7, 0xAE,
			0x83, 0x7C, 0x26, 0x4A, 0xE3, 0xA9, 0xBE, 0xB8,
			0x7F, 0x8A, 0x2F, 0xE9, 0xB8, 0xB5, 0x29, 0x2E,
			0x5A, 0x02, 0x1F, 0xFF, 0x5E, 0x91, 0x47, 0x9E,
			0x8C, 0xE7, 0xA2, 0x8C, 0x24, 0x42, 0xC6, 0xF3,
			0x15, 0x18, 0x0F, 0x93, 0x49, 0x9A, 0x23, 0x4D,
			0xCF, 0x76, 0xE3, 0xFE, 0xD1, 0x35, 0xF9, 0xBB,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
		}
	},
	{
		DH_GROUP_2048, 256,
		{
			0xAC, 0x6B, 0xDB, 0x41, 0x32, 0x4A, 0x9A, 0x9B,
			0xF1, 0x66, 0xDE, 0x5E, 0x13, 0x89, 0x58, 0x2F,
			0xAF, 0x72, 0xB6, 0x65, 0x19, 0x87, 0xEE, 0x07,
			0xFC, 0x31, 0x92, 0x94, 0x3D, 0xB5, 0x60, 0x50,
			0xA3, 0x73, 0x29, 0xCB, 0xB4, 0xA0, 0x99, 0xED,
			0x81, 0x93, 0xE0, 0x75, 0x77, 0x67, 0xA1, 0x3D,
			0xD5, 0x23, 0x12, 0xAB, 0x4B, 0x03, 0x31, 0x0D,
			0xCD, 0x7F, 0x48, 0xA9, 0xDA, 0x04, 0xFD, 0x50,
			0xE8, 0x08, 0x39, 0x69, 0xED, 0xB7, 0x67, 0xB0,
			0xCF, 0x60, 0x95, 0x17, 0x9A, 0x16, 0x3A, 0xB3,
			0x66, 0x1A, 0x05, 0xFB, 0xD5, 0xFA, 0xAA, 0xE8,
			0x29, 0x18, 0xA9, 0x96, 0x2F, 0x0B, 0x93, 0xB8,
			0x55, 0xF9, 0x79, 0x93, 0xEC, 0x97, 0x5E, 0xEA,
			0xA8, 0x0D, 0x74, 0x0A, 0xDB, 0xF4, 0xFF, 0x74,
			0x73, 0x59, 0xD0, 0x41, 0xD5, 0xC3, 0x3E, 0xA7,
			0x1D, 0x28, 0x1E, 0x44, 0x6B, 0x14, 0x77, 0x3B,
			0xCA, 0x97, 0xB4, 0x3A, 0x23, 0xFB, 0x80, 0x16,
			0x76, 0xBD, 0x20, 0x7A, 0x43, 0x6C, 0x64, 0x81,
			0xF1, 0xD2, 0xB9, 0x07, 0x87, 0x17, 0x46, 0x1A,
			0x5B, 0x9D, 0x32, 0xE6, 0x88, 0xF8, 0x77, 0x48,
			0x54, 0x45, 0x23, 0xB5, 0x24, 0xB0, 0xD5, 0x7D,
			0x5E, 0xA7, 0x7A, 0x27, 0x75, 0xD2, 0xEC, 0xFA,
			0x03, 0x2C, 0xFB, 0xDB, 0xF5, 0x2F, 0xB3, 0x78,
			0x61, 0x60, 0x27, 0x90, 0x04, 0xE5, 0x7A, 0xE6,
			0xAF, 0x87, 0x4E, 0x73, 0x03, 0xCE, 0x53, 0x29,
			0x9C, 0xCC, 0x04, 0x1C, 0x7B, 0xC3, 0x08, 0xD8,
			0x2A, 0x56, 0x98, 0xF3, 0xA8, 0xD0, 0xC3, 0x82,
			0x71, 0xAE, 0x35, 0xF8, 0xE9, 0xDB, 0xFB, 0xB6,
			0x94, 0xB5, 0xC8, 0x03, 0xD8, 0x9F, 0x7A, 0xE4,
			0x35, 0xDE, 0x23, 0x6D, 0x52, 0x5F, 0x54, 0x75,
			0x9B, 0x65, 0xE3, 0x72, 0xFC, 0xD6, 0x8E, 0xF2,
			0x0F, 0xA7, 0x11, 0x1F, 0x9E, 0x4A, 0xFF, 0x73,
		}
	},
};

/* Auth CFG data store routines */

/* Currently limiting to only one page */
#define for_each_cfg_entry(cfg_db, i, entry)				\
		for (i = 0, entry = &cfg_db->entries[0];		\
		     i < OCS_AUTH_CFG_ENTRIES_MAX;			\
		     i++, entry++)

static int
ocs_auth_cfg_obj_read(ocs_t *ocs, u32 offset, ocs_dma_t *dma, u32 *read_len)
{
	u8 status, addl_status;
	ocs_hal_rtn_e hal_rc;

	hal_rc = ocs_hal_read_common_obj_wait(&ocs->hal,
					      OCS_AUTH_CFG_NVRAM_PATH, offset,
					      dma, read_len,
					      &status, &addl_status);
	/* INVALID_OBJECT/OFFSET errors may be returned if the object
	 * was never commited to the NVRAM; they are not fatal errors
	 */
	if (OCS_HAL_RTN_IS_ERROR(hal_rc) &&
	    (addl_status == SLI4_MGMT_ADD_STATUS_INVALID_OBJECT ||
	     addl_status == SLI4_MGMT_ADD_STATUS_INVALID_OFFSET))
		hal_rc = OCS_HAL_RTN_SUCCESS;

	return OCS_HAL_RTN_IS_ERROR(hal_rc) ? -EIO : 0;
}

int
ocs_auth_cfg_setup(ocs_t *ocs)
{
	struct ocs_auth_cfg_entry *entry;
	struct ocs_auth_cfg_db *cfg_db;
	struct ocs_auth_cfg_hdr *hdr;
	u32 read_len, offset;
	int i, rc;

	if (!ocs_gpl_api_enabled()) {
		ocs_log_err(ocs, "DHCHAP authentication cannot be enabled\n");
		return -EOPNOTSUPP;
	}

	ocs_lock_init(ocs, &ocs->auth_cfg_lock, "auth cfg lock");

	rc = ocs_dma_alloc(ocs, &ocs->auth_cfg_mem, OCS_AUTH_CFG_SIZE, 4);
	if (rc)
		goto err;
	ocs_memset(ocs->auth_cfg_mem.virt, 0, OCS_AUTH_CFG_SIZE);

	offset = ocs->hal.sli.physical_port * OCS_AUTH_CFG_SIZE;
	rc = ocs_auth_cfg_obj_read(ocs, offset, &ocs->auth_cfg_mem, &read_len);
	if (rc)
		goto err;

	cfg_db = ocs->auth_cfg_mem.virt;
	hdr = &cfg_db->hdr;
	if (read_len == 0 || hdr->signature == 0) {
		/* there's no stored config */
		ocs_log_debug(ocs, "no stored config\n");
		hdr->signature = OCS_AUTH_CFG_SIGNATURE;
		hdr->size = sizeof(*hdr);
		hdr->version = OCS_AUTH_CFG_VERSION;
	} else {
		/* ensure that the stored config is valid */
		if (hdr->signature != OCS_AUTH_CFG_SIGNATURE) {
			ocs_log_err(ocs, "Incorrect signature in file %s\n",
				    OCS_AUTH_CFG_NVRAM_PATH);
			rc = -EIO;
			goto err;
		}
		ocs_log_debug(ocs, "Read %d entries from %s\n",
			      hdr->num_entries, OCS_AUTH_CFG_NVRAM_PATH);

		/* copy info form a valid entry to common info */
		for_each_cfg_entry(cfg_db, i, entry) {
			if (entry->info.flags & OCS_AUTH_FLAGS_VALID) {
				ocs_memcpy(&ocs->auth_cfg_info,
					   &entry->info,
					   sizeof(ocs->auth_cfg_info));
				break;
			}
		}
	}
	return 0;
err:
	ocs_log_err(ocs, "Error rc=%d\n", rc);
	if (ocs->auth_cfg_mem.virt)
		ocs_dma_free(ocs, &ocs->auth_cfg_mem);
	return rc;
}

static bool
ocs_auth_cfg_enabled(ocs_t *ocs)
{
	return ocs->auth_cfg_mem.virt != NULL;
}

/*
 * Must be called from inside the auth_cfg_lock
 * Sync the common info struct with all the entries
 */
static void
ocs_auth_cfg_info_sync(ocs_t *ocs)
{
	struct ocs_auth_cfg_db *cfg_db = ocs->auth_cfg_mem.virt;
	struct ocs_auth_cfg_info *info = &ocs->auth_cfg_info;
	struct ocs_auth_cfg_hdr *hdr = &cfg_db->hdr;
	struct ocs_auth_cfg_entry *entry;
	int i, j = 0;

	if (!(info->flags & OCS_AUTH_FLAGS_VALID))
		return;

	/* If no entries are present create a dummy entry
	 * to store a valid info
	 */
	if (hdr->num_entries == 0) {
		hdr->num_entries++;
		entry = &cfg_db->entries[0];
		ocs_memcpy(&entry->info, info, sizeof(*info));
	} else { /* update all valid entries */
		for_each_cfg_entry(cfg_db, i, entry) {
			if (entry->info.flags & OCS_AUTH_FLAGS_VALID) {
				ocs_memcpy(&entry->info, info,
					   sizeof(*info));
				j++;
			}
			if (j == hdr->num_entries)
				break;
		}
		ocs_log_debug(ocs, "updated %d entries\n", j);
	}
}

int
ocs_auth_cfg_commit(ocs_t *ocs)
{
	ocs_dma_t dma = { 0 };
	u32 offset, read_len;
	ocs_hal_rtn_e hrc;
	int rc = 0;

	ocs_lock(&ocs->auth_cfg_lock);

	if (!ocs->auth_cfg_updated)
		goto done;

	ocs_auth_cfg_info_sync(ocs);

	/* alloc temp dma memory for reading global auth cfg object */
	rc = ocs_dma_alloc(ocs, &dma, OCS_AUTH_CFG_SIZE_GLOBAL, 4);
	if (rc)
		goto done;
	ocs_memset(dma.virt, 0, OCS_AUTH_CFG_SIZE_GLOBAL);

	/* read the global auth cfg object */
	rc = ocs_auth_cfg_obj_read(ocs, 0, &dma, &read_len);
	if (rc)
		goto done;

	/* update the global auth cfg with my data at my offset */
	offset = ocs->hal.sli.physical_port * OCS_AUTH_CFG_SIZE;
	ocs_memcpy((u8 *)dma.virt + offset, ocs->auth_cfg_mem.virt,
		   OCS_AUTH_CFG_SIZE);

	/* write the whole thing back to nvram */
	hrc = ocs_hal_common_write_obj_wait(&ocs->hal, OCS_AUTH_CFG_NVRAM_PATH,
					    &dma, OCS_AUTH_CFG_SIZE_GLOBAL);
	if (OCS_HAL_RTN_IS_ERROR(hrc))
		rc = -EIO;
	else
		ocs->auth_cfg_updated = false;
done:
	ocs_unlock(&ocs->auth_cfg_lock);
	ocs_dma_free(ocs, &dma);
	if (rc)
		ocs_log_err(ocs, "Error: rc=%d\n", rc);
	return rc;
}

void
ocs_auth_cfg_clear(ocs_t *ocs)
{
	ocs_dma_free(ocs, &ocs->auth_cfg_mem);
	return;
}

static struct ocs_auth_cfg_entry *
__ocs_auth_cfg_entry_unused(ocs_t *ocs)
{
	struct ocs_auth_cfg_db *cfg_db = ocs->auth_cfg_mem.virt;
	struct ocs_auth_cfg_entry *entry;
	int i;

	for_each_cfg_entry(cfg_db, i, entry) {
		if (!(entry->info.flags & OCS_AUTH_FLAGS_VALID))
			return entry;
	}
	return NULL;
}

static struct ocs_auth_cfg_entry *
__ocs_auth_cfg_entry_lkup(ocs_t *ocs, u8 *local_wwpn, u8 *remote_wwpn)
{
	struct ocs_auth_cfg_db *cfg_db = ocs->auth_cfg_mem.virt;
	struct ocs_auth_cfg_entry *entry;
	int i;

	for_each_cfg_entry(cfg_db, i, entry) {
		if (entry->info.flags & OCS_AUTH_FLAGS_VALID &&
		    ocs_memcmp(entry->id.local_wwpn, local_wwpn, 8) == 0 &&
		    ocs_memcmp(entry->id.remote_wwpn, remote_wwpn, 8) == 0)
			return entry;
	}
	return NULL;
}

/*
 * Returns true if entry found; if found fills *info and *pass
 */
static bool
ocs_auth_cfg_lkup(ocs_node_t *node)
{
	ocs_t *ocs = node->ocs;
	__be64 local_wwpn = ocs_htobe64(node->sport->wwpn);
	__be64 remote_wwpn = ocs_htobe64(FABRIC_WWPN);
	struct ocs_auth_cfg_entry *entry;

	if (!ocs_auth_cfg_enabled(ocs))
		return false;

	ocs_lock(&ocs->auth_cfg_lock);
	entry = __ocs_auth_cfg_entry_lkup(ocs, (u8 *)&local_wwpn,
					  (u8 *)&remote_wwpn);
	if (entry) {
		ocs_memcpy(&node->auth_cfg, &ocs->auth_cfg_info,
			   sizeof(node->auth_cfg));
		ocs_memcpy(&node->auth_pass, &entry->pass,
			   sizeof(node->auth_pass));
	}
	ocs_unlock(&ocs->auth_cfg_lock);

	return entry ? true : false;
}

int
ocs_auth_cfg_entry_update(ocs_t *ocs, u8 *local_wwpn, u8 *remote_wwpn,
			  struct ocs_auth_cfg_pass *pass)
{
	struct ocs_auth_cfg_db *cfg_db = ocs->auth_cfg_mem.virt;
	struct ocs_auth_cfg_entry *entry;
	int rc = 0;

	if (!ocs_auth_cfg_enabled(ocs))
		return -EOPNOTSUPP;

	ocs_lock(&ocs->auth_cfg_lock);

	/* if the entry already exists - update or delete */
	entry = __ocs_auth_cfg_entry_lkup(ocs, local_wwpn, remote_wwpn);

	/* delete request */
	if (!pass->local_len) {
		if (entry) {
			ocs_memset(entry, 0, sizeof(*entry));
			cfg_db->hdr.num_entries--;
		} else {
			rc = -ENOKEY;
		}
		goto done;
	}

	if (entry) { /* update request */
		ocs_memcpy(&entry->pass, pass, sizeof(*pass));
	} else { /* add request */
		entry = __ocs_auth_cfg_entry_unused(ocs);
		if (!entry) {
			rc = -ENOSPC;
			goto done;
		}
		ocs_memcpy(entry->id.local_wwpn, local_wwpn, 8);
		ocs_memcpy(entry->id.remote_wwpn, remote_wwpn, 8);
		ocs_memcpy(&entry->pass, pass, sizeof(*pass));
		entry->info.flags |= OCS_AUTH_FLAGS_VALID;
		cfg_db->hdr.num_entries++;
	}

done:
	if (!rc)
		ocs->auth_cfg_updated = true;
	ocs_unlock(&ocs->auth_cfg_lock);
	if (rc)
		ocs_log_err(ocs, "Error: rc=%d\n", rc);
	return rc;
}

int
ocs_auth_cfg_num_entries_get(ocs_t *ocs, u32 *num)
{
	struct ocs_auth_cfg_db *cfg_db = ocs->auth_cfg_mem.virt;

	if (!ocs_auth_cfg_enabled(ocs))
		return -EOPNOTSUPP;

	*num = cfg_db->hdr.num_entries;
	return 0;
}

int
ocs_auth_cfg_entries_get(ocs_t *ocs, struct ocs_auth_cfg_id *ids, u32 num)
{
	struct ocs_auth_cfg_db *cfg_db = ocs->auth_cfg_mem.virt;
	struct ocs_auth_cfg_hdr *hdr = &cfg_db->hdr;
	struct ocs_auth_cfg_entry *entry;
	int rc = 0, i;

	if (!ocs_auth_cfg_enabled(ocs))
		return -EOPNOTSUPP;

	ocs_lock(&ocs->auth_cfg_lock);
	if (num < hdr->num_entries) {
		rc = -ERANGE;
		goto done;
	}

	for_each_cfg_entry(cfg_db, i, entry) {
		if (entry->info.flags & OCS_AUTH_FLAGS_VALID) {
			ocs_memcpy(ids, &entry->id, sizeof(*ids));
			ids++;
		}
	}
done:
	ocs_unlock(&ocs->auth_cfg_lock);
	return rc;
}

int
ocs_auth_cfg_info_get(ocs_t *ocs, struct ocs_auth_cfg_info *cfg_info)
{
	struct ocs_auth_cfg_info *info = &ocs->auth_cfg_info;
	int rc = 0;

	if (!ocs_auth_cfg_enabled(ocs))
		return -EOPNOTSUPP;

	ocs_lock(&ocs->auth_cfg_lock);
	if (info->flags & OCS_AUTH_FLAGS_VALID)
		ocs_memcpy(cfg_info, info, sizeof(*cfg_info));
	else
		rc = -ENODATA;
	ocs_unlock(&ocs->auth_cfg_lock);

	return rc;
}

int
ocs_auth_cfg_info_update(ocs_t *ocs, struct ocs_auth_cfg_info *new_cfg_info)
{
	if (!ocs_auth_cfg_enabled(ocs))
		return -EOPNOTSUPP;

	ocs_lock(&ocs->auth_cfg_lock);
	ocs_memcpy(&ocs->auth_cfg_info, new_cfg_info, sizeof(*new_cfg_info));
	ocs->auth_cfg_updated = true;
	ocs_unlock(&ocs->auth_cfg_lock);

	return 0;
}

/* Functions for sending Auth ELS messages */
void
ocs_auth_els_resp_cb(ocs_node_t *node, ocs_node_cb_t *cbdata, void *cb_arg);

static ocs_io_t *
ocs_auth_els_prep(ocs_node_t *node, u8 els_flags, u8 msg_code,
		  u32 msg_len, __be32 trans_id)
{
	struct ocs_auth_els_hdr *hdr;
	ocs_t *ocs = node->ocs;
	ocs_io_t *els = els;

	ocs_log_debug(node->ocs, "msg_code=%d, msg_len=%d, trans_id=0x%x\n",
		      msg_code, msg_len, trans_id);

	els = ocs_els_io_alloc(node, msg_len + sizeof(*hdr),
			       OCS_ELS_ROLE_ORIGINATOR);
	if (!els) {
		ocs_log_err(ocs, "AUTH ELS msg_code=0x%x alloc failure\n",
			    msg_code);
		return NULL;
	}

	els->els_info->els_timeout_sec = OCS_FC_ELS_SEND_DEFAULT_TIMEOUT;
	els->iparam.els.timeout = OCS_FC_ELS_SEND_DEFAULT_TIMEOUT;
	els->els_info->els_retries_remaining = OCS_AUTH_ELS_RETRIES;
	els->els_info->els_callback = ocs_auth_els_resp_cb;
	els->els_info->els_callback_arg = NULL;
	els->display_name = "auth";
	els->hio_type = OCS_HAL_ELS_REQ;

	hdr = els->els_info->els_req.virt;
	/* TODO: must not be done here */
	ocs_memset(hdr, 0, msg_len + sizeof(*hdr));
	hdr->els_code = AUTH_ELS_CODE;
	hdr->els_flags = els_flags;
	hdr->msg_code = msg_code;
	hdr->protocol_ver = AUTH_PROTOCOL_VER_1;
	hdr->msg_len = ocs_htobe32(msg_len);
	hdr->trans_id = trans_id;
	return els;
}

static void
ocs_auth_msg_name_fill(struct ocs_auth_node_name *name, u8 *wwpn)
{
	name->tag = ocs_htobe16(AUTH_NAME_TAG);
	name->len = ocs_htobe16(AUTH_NAME_LEN);
	ocs_memcpy(name->val, wwpn, AUTH_NAME_LEN);
}

static u8
sli_to_auth_hash_id(u8 sli_hash_id)
{
	switch(sli_hash_id) {
	case OCS_HASH_MD5:
		return HASH_MD5;
	case OCS_HASH_SHA1:
		return HASH_SHA1;
	default:
		return 0;
	}
}

static u8
sli_to_auth_dh_grp_id(u8 sli_dh_grp_id)
{
	switch(sli_dh_grp_id) {
	case OCS_DH_GROUP_NULL:
		return DH_GROUP_NULL;
	case OCS_DH_GROUP_1024:
		return DH_GROUP_1024;
	case OCS_DH_GROUP_1280:
		return DH_GROUP_1280;
	case OCS_DH_GROUP_1536:
		return DH_GROUP_1536;
	case OCS_DH_GROUP_2048:
		return DH_GROUP_2048;
	default:
		return 0xff;
	}
}

static int
ocs_auth_neg_send(ocs_node_t *node)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	__be64 local_wwpn = ocs_htobe64(node->sport->wwpn);
	struct ocs_auth_cfg_info *cfg = &node->auth_cfg;
	struct ocs_auth_proto_params_hdr *params_hdr;
	struct ocs_auth_neg_dhchap_param *param_hdr;
	u32 dhchap_params_len, *param_val, msg_len;
	struct ocs_auth_neg_hdr *hdr;
	ocs_io_t *els;
	u8 *msg;
	int i;

	/* Currently only DH-CHAP is supported
	 * In DH-CHAP we support max 2 hash functions and max 5 DH-Groups
	 * Count how many hash and dh_grp we support for the remote node
	 */
	for (i = 0; i < OCS_MAX_HASH && cfg->hash_list[i]; i++);
	dhchap_params_len =
		sizeof(struct ocs_auth_neg_dhchap_param) + i * sizeof(u32);

	for (i = 0; i < OCS_MAX_DH_GROUP && cfg->dh_grp_list[i]; i++);
	dhchap_params_len +=
		sizeof(struct ocs_auth_neg_dhchap_param) + i * sizeof(u32);

	msg_len = sizeof(struct ocs_auth_neg_hdr) +
		  sizeof(struct ocs_auth_proto_params_hdr) + dhchap_params_len;

	els = ocs_auth_els_prep(node, 0, AUTH_NEGOTIATE, msg_len,
				dhchap->trans_id);
	if (!els)
		return -ENOMEM;
	msg = els->els_info->els_req.virt + sizeof(struct ocs_auth_els_hdr);

	/* neg payload hdr */
	hdr = (struct ocs_auth_neg_hdr *)msg;
	ocs_auth_msg_name_fill(&hdr->name, (u8 *)&local_wwpn);
	hdr->num_proto = ocs_htobe32(1); /* only DH-CHAP */
	msg += sizeof(*hdr);

	/* neg params_hdr hdr */
	params_hdr = (struct ocs_auth_proto_params_hdr *)msg;
	params_hdr->len = ocs_htobe32(4 /* for proto id */ + dhchap_params_len);
	params_hdr->proto_id = ocs_htobe32(PROTOCOL_DHCHAP);
	msg += sizeof(*params_hdr);

	/* hash list */
	param_hdr = (struct ocs_auth_neg_dhchap_param *)msg;
	param_hdr->tag = ocs_htobe16(DHCHAP_TAG_HASHLIST);
	msg += sizeof(*param_hdr);
	param_val = (u32 *) msg;
	for (i = 0; i < OCS_MAX_HASH && cfg->hash_list[i]; i++) {
		*param_val =
			ocs_htobe32(sli_to_auth_hash_id(cfg->hash_list[i]));
		param_val++;
	}
	param_hdr->word_cnt = ocs_htobe16(i);
	msg += i * sizeof(*param_val);

	/* dh grp list */
	param_hdr = (struct ocs_auth_neg_dhchap_param *)msg;
	param_hdr->tag = ocs_htobe16(DHCHAP_TAG_DHGRPLIST);
	msg += sizeof(*param_hdr);
	param_val = (u32 *) msg;
	for (i = 0; i < OCS_MAX_DH_GROUP && cfg->dh_grp_list[i]; i++) {
		*param_val =
			ocs_htobe32(sli_to_auth_dh_grp_id(cfg->dh_grp_list[i]));
		param_val++;
	}
	param_hdr->word_cnt = ocs_htobe16(i);

	ocs_els_io_send(els);
	return 0;
}

static int
ocs_auth_rej_send(ocs_node_t *node, __be32 trans_id, u8 code, u8 expl)
{
	struct ocs_auth_rej_msg *rej;
	ocs_io_t *els;

	ocs_log_debug(node->ocs, "trans_id=0x%x, code=%s, expl=%s",
		      trans_id, auth_err_name(code), auth_expl_name(expl));

	els = ocs_auth_els_prep(node, 0, AUTH_REJECT, sizeof(*rej),
				trans_id);
	if (!els)
		return -ENOMEM;
	rej = els->els_info->els_req.virt + sizeof(struct ocs_auth_els_hdr);
	rej->code = code;
	rej->expl = expl;

	ocs_els_io_send(els);
	return 0;
}

static int
ocs_dhchap_chal_send(ocs_node_t *node, u8 *dh)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	__be64 local_wwpn = ocs_htobe64(node->sport->wwpn);
	struct ocs_dhchap_chal_hdr *hdr;
	ocs_io_t *els;
	u32 msg_len;
	u8 *msg;

	msg_len = sizeof(*hdr) + sizeof(u32) + dhchap->hash_len +
		  sizeof(u32) + dhchap->dh_len;
	els = ocs_auth_els_prep(node, 0, DHCHAP_CHALLENGE, msg_len,
				dhchap->trans_id);
	if (!els)
		return -ENOMEM;
	msg = els->els_info->els_req.virt + sizeof(struct ocs_auth_els_hdr);

	/* challenge msg hdr */
	hdr = (struct ocs_dhchap_chal_hdr *)msg;
	ocs_auth_msg_name_fill(&hdr->name, (u8 *)&local_wwpn);
	hdr->hash_id = ocs_htobe32(dhchap->hash_id);
	hdr->dh_grp_id = ocs_htobe32(dhchap->dh_grp_id);
	msg += sizeof(*hdr);

	/* challenge */
	*(u32 *)msg = ocs_htobe32(dhchap->hash_len);
	msg += sizeof(u32);
	ocs_memcpy(msg, dhchap->chal, dhchap->hash_len);
	msg += dhchap->hash_len;

	/* DH val */
	*(u32 *)msg = ocs_htobe32(dhchap->dh_len);
	if (dhchap->dh_len) {
		msg += sizeof(u32);
		ocs_memcpy(msg, dh, dhchap->dh_len);
	}

	ocs_els_io_send(els);
	return 0;
}

/*
 * Skip including DH-val if dh is NULL.
 * Skip including Challenge if chal is NULL.
 */
static int
ocs_dhchap_reply_send(ocs_node_t *node, u8 *resp, u8 *dh, u8 *chal)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	ocs_io_t *els;
	u32 msg_len;
	u8 *msg;

	msg_len = sizeof(u32) + dhchap->hash_len + /* resp */
		  sizeof(u32) + dhchap->dh_len + /* dh-val */
		  sizeof(u32) + dhchap->hash_len; /* chal */

	els = ocs_auth_els_prep(node, 0, DHCHAP_REPLY, msg_len,
				dhchap->trans_id);
	if (!els)
		return -ENOMEM;
	msg = els->els_info->els_req.virt + sizeof(struct ocs_auth_els_hdr);

	/* response */
	*(u32 *)msg = ocs_htobe32(dhchap->hash_len);
	msg += sizeof(u32);
	ocs_memcpy(msg, resp, dhchap->hash_len);
	msg += dhchap->hash_len;

	/* DH val */
	*(u32 *)msg = ocs_htobe32(dhchap->dh_len);
	msg += sizeof(u32);
	if (dh) {
		ocs_memcpy(msg, dh, dhchap->dh_len);
		msg += dhchap->dh_len;
	}

	/* challenge */
	*(u32 *)msg = chal != NULL ? ocs_htobe32(dhchap->hash_len) : 0;
	msg += sizeof(u32);
	if (chal)
		ocs_memcpy(msg, chal, dhchap->hash_len);

	ocs_els_io_send(els);
	return 0;
}

static int
ocs_dhchap_success_send(ocs_node_t *node, u8 *resp)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	ocs_io_t *els;
	u32 resp_len;
	u8 *msg;

	resp_len = resp ? dhchap->hash_len : 0;
	els = ocs_auth_els_prep(node, 0, DHCHAP_SUCCESS, resp_len + sizeof(u32),
				dhchap->trans_id);
	if (!els)
		return -ENOMEM;
	msg = els->els_info->els_req.virt + sizeof(struct ocs_auth_els_hdr);

	/* response */
	*(u32 *)msg = ocs_htobe32(resp_len);
	if (resp_len) {
		msg += sizeof(u32);
		ocs_memcpy(msg, resp, resp_len);
	}

	ocs_els_io_send(els);
	return 0;
}

static u32
ocs_hash_len(u32 hash_id)
{
	switch(hash_id) {
	case HASH_MD5:
		return MD5_HASH_LEN;
	case HASH_SHA1:
		return SHA1_HASH_LEN;
	default:
		return 0;
	}
}

static int
ocs_auth_hash_compute(u32 hash_id, u8 *result, u8 *val1, u32 val1_len,
		      u8 *val2, u32 val2_len, u8 *val3, u32 val3_len)
{
	char *hash_alg;
	u32 hash_len;

	switch (hash_id) {
	case HASH_MD5:
		hash_alg = "md5";
		hash_len = MD5_HASH_LEN;
		break;
	case HASH_SHA1:
		hash_alg = "sha1";
		hash_len = SHA1_HASH_LEN;
		break;
	default:
		return -EINVAL;
	}

	return ocs_hash(hash_alg, val1, val1_len, val2, val2_len,
			val3, val3_len, result, hash_len);
}

/*
 * dh = (base ^ exp) % mod
 * If dh_len has more space, fill dh with leading zeros
 */
static int
ocs_auth_dh_compute(u32 dh_grp_id, u8 *base, u32 base_len,
		    u8 *exp, u32 exp_len, u8 *dh)
{
	u32 mod_len, dh_len;
	u8 *mod;

	switch(dh_grp_id) {
	case DH_GROUP_1024:
	case DH_GROUP_1280:
	case DH_GROUP_1536:
	case DH_GROUP_2048:
		dh_len = dh_group_array[dh_grp_id].length;
		mod_len = dh_len;
		mod = dh_group_array[dh_grp_id].value;
		break;
	default:
		return -EINVAL;
	}

	return ocs_mpi_powm(base, base_len, exp, exp_len, mod, mod_len,
			    dh, dh_len);
}

static void
ocs_auth_nonce_compute(u8 *nonce, u32 len)
{
	ocs_get_random_bytes(nonce, len);
}

static void
ocs_dhchap_chal_compute(u8 *chal, u32 chal_len)
{
	ocs_get_random_bytes(chal, chal_len);
}

/*
 * DHCHAP Computations
 * *******************
 *             Auth_Initiator(n)               Auth_Responder(m)
 *
 *                              +                  +
 *           *DHCHAP_STATE_INIT |                  | *DHCHAP_STATE_INIT
 *                              |        AUTH_Neg  |
 *                              +----------------->+
 *                              |                  | Challenge = C1
 *       *DHCHAP_STATE_NEG_SENT |                  | Nonce = x
 *                              | C1, g^x mod p    | DH-val = g^x mod p
 *	                        +<-----------------+
 * Nonce = y			|		   |
 * DH-val = g^y mod p           |                  | *DHCHAP_STATE_CHAL_SENT
 * Eph-DH = (g^x mod p)^y mod p |                  |
 * Ca1 = H(C1 || Eph-DH)        |                  |
 * R1 = H(Ca1 || Kn || T)       |                  |
 * Challenge = C2		| R1, g^y mod p,C2 |
 *                        	+----------------->+
 *     *DHCHAP_STATE_REPLY_SENT |                  | Eph-DH=(g^y mod p)^x mod p
 *				|		   | Ca1 = H(C1 || Eph-DH)
 *				|		   | R1' = H(Ca1 || Kn || T)
 *				|		   | Validate (R1' == R)
 *				|		   |
 *				|		   | Ca2 = H(C2 || Eph-DH)
 *				|		   | R2 = H(Ca2 || Km || T)
 *				|		   |
 *                        	| Success, R2	   |
 *                        	+<-----------------+
 *  Ca2 = H(C2 || Eph-DH)	|                  | *DHCHAP_STATE_SUCCESS_SENT
 *  R2' = H(Ca2 || Km || T)	|		   |
 *  Validate (R2' == R2)       	|          Success |
 *                        	+----------------->+
 *   	     *DHCHAP_STATE_DONE |                  | *DHCHAP_STATE_DONE
 *                        	|                  |
 */
static int
__ocs_dhchap_resp_compute(ocs_node_t *node, u8* resp, u8 *chal,
			  u8 *eph_dh, u8 *pass, u32 pass_len)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	u8 trans_id_lsb, temp[MAX_HASH_LEN], *aug_chal;
	u32 dh_grp_id = dhchap->dh_grp_id;
	int rc;

	/* Compute the augmented challenge Ca */
	if (dh_grp_id != DH_GROUP_NULL) {
		rc = ocs_auth_hash_compute(dhchap->hash_id, temp, chal,
					   dhchap->hash_len, eph_dh,
					   dhchap->dh_len, NULL, 0);
		if (rc)
			goto err;
		aug_chal = temp;
	} else {
		/* No need to compute aug_chal for NULL DH */
		aug_chal = chal;
	}


	/* Compute response */
	trans_id_lsb = ocs_be32toh(dhchap->trans_id) & 0xff;
	rc = ocs_auth_hash_compute(dhchap->hash_id, resp, &trans_id_lsb,
				   sizeof(trans_id_lsb), pass, pass_len,
				   aug_chal, dhchap->hash_len);
	if (rc)
		goto err;
	return 0;
err:
	ocs_log_err(node->ocs, "Error rc=%d\n", rc);
	return rc;
}

/*
 * Used for:
 * a) computing a response to be sent for a rcvd challenge (local_pw is used)
 * b) validating a rcvd response for a sent challenge (remote_pw is used)
 */
static int
ocs_dhchap_resp_compute(ocs_node_t *node, u8* resp, u8 *chal, u8 *rcvd_dh,
			u8 *pass, u32 pass_len)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	int rc;

	/* Compute ephemeral DH-val: (g^x mod p)^y mod p)
	 * Store the result in dhchap->eph_dh so that it can be
	 * used for both sending resp and validating rcvd resp (bidi)
	 */
	if (dhchap->dh_grp_id != DH_GROUP_NULL && !dhchap->eph_dh_computed) {
		rc = ocs_auth_dh_compute(dhchap->dh_grp_id,
					 rcvd_dh, dhchap->dh_len,
					 dhchap->nonce, sizeof(dhchap->nonce),
					 dhchap->eph_dh);
		if (rc)
			goto err;
		dhchap->eph_dh_computed = true;
	}

	rc = __ocs_dhchap_resp_compute(node, resp, chal, dhchap->eph_dh,
				       pass, pass_len);
	if (rc)
		goto err;

	return 0;
err:
	ocs_log_err(node->ocs, "Error rc=%d\n", rc);
	return rc;
}

/*
 * Returns <  0 on error while validating
 * 	   == 0 on auth success
 * 	   >  0 on auth failure
 */
static int
ocs_dhchap_resp_validate(ocs_node_t *node, u8* rcvd_resp, u8 *rcvd_dh)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	u8 exp_resp[MAX_HASH_LEN];
	int rc;

	/* compute expected response */
	rc = ocs_dhchap_resp_compute(node, exp_resp, dhchap->chal, rcvd_dh,
				     node->auth_pass.local,
				     node->auth_pass.local_len);
	if (rc)
		goto err;

	rc = ocs_memcmp(rcvd_resp, exp_resp, dhchap->hash_len);
	if (rc) {
		ocs_log_err(node->ocs, "Auth failure\n");
		rc = 1;
	}

	return rc;
err:
	ocs_log_err(node->ocs, "Error rc=%d\n", rc);
	return rc;
}

static void
ocs_dhchap_params_save(struct ocs_dhchap_info *dhchap,
		       u32 hash_id, u32 dh_grp_id)
{
	dhchap->hash_id = hash_id;
	dhchap->hash_len = ocs_hash_len(hash_id);
	dhchap->dh_grp_id = dh_grp_id;
	dhchap->dh_len = dh_group_array[dh_grp_id].length;
}

/*
 * Move the msg pointer forward by the offset value.
 * Check if there's enough lenght in the msg to allow moving forward
 */
static u8 *
ptr_adv_check(u8 *msg, u8 *msg_start, u32 msg_len, u32 offset)
{
	msg += offset;
	return (msg - msg_start) > msg_len ? NULL : msg;
}

/*
 * Move the msg pointer forward by the offset value with out any check
 */
static inline u8 *
ptr_adv(u8 *msg, u32 offset)
{
	msg += offset;
	return msg;
}

/*
 * Return RJT code associated with expl value.
 */
static u8
auth_rjt_code(u8 expl)
{
	switch (expl) {
	case AUTH_RJT_EXPL_MECH_UNUSABLE:
	case AUTH_RJT_EXPL_RESTART_AUTH:
	case AUTH_RJT_EXPL_CONCAT_NOSUPP:
	case AUTH_RJT_EXPL_PROTO_VER_NOSUPP:
	case AUTH_RJT_EXPL_DHGRP_UNUSABLE:
	case AUTH_RJT_EXPL_HASH_UNUSABLE:
	case AUTH_RJT_EXPL_AUTH_STARTED:
		return AUTH_RJT_CODE_LOGICAL_ERR;

	case AUTH_RJT_EXPL_AUTH_FAILED:
	case AUTH_RJT_EXPL_BAD_PAYLOAD:
	case AUTH_RJT_EXPL_BAD_PROTOCOL:
		return AUTH_RJT_CODE_FAILURE;
	default:
		ocs_assert(FALSE, 0);
	}
}

static enum ocs_auth_role
ocs_auth_neg_collision_resolve(ocs_node_t *node, u8 *remote_wwn)
{
	__be64 local_wwpn;

	/* If there's a collision between Nx_Port and F_Port controller
	 * the Nx_Port always gets to be the Auth Initiator.
	 */
	if (node->rnode.fc_id == FC_ADDR_FABRIC)
		return OCS_AUTH_ROLE_INITIATOR;

	/* For a collision between Nx_Port and Nx_Port, the numberically
	 * higher name shall remain the Initiator.
	 */
	local_wwpn = ocs_htobe64(node->sport->wwpn);
	return memcmp((u8 *)&local_wwpn, remote_wwn, AUTH_NAME_LEN) > 0 ?
		OCS_AUTH_ROLE_INITIATOR : OCS_AUTH_ROLE_RESPONDER;
}

/*
 * Parse the AUTH_Negotiate msg.
 * Check for possible negotiate collision
 * Parse DHCHAP params like hash_id and dh_grp_id and store them
 * Returns AUTH_RJT_EXP_XXX error or 0 for success
 */
static u8
ocs_auth_neg_parse(ocs_node_t *node, u8 *msg, u32 msg_len, __be32 trans_id)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	struct ocs_auth_cfg_info *cfg = &node->auth_cfg;
	struct ocs_auth_neg_dhchap_param *dhchap_param;
	struct ocs_auth_proto_params_hdr *params_hdr;
	struct ocs_auth_node_name *remote_name;
	u32 hash_id, dh_grp_id = DH_GROUP_NULL;
	struct ocs_auth_neg_hdr *neg_hdr;
	enum ocs_auth_role new_role;
	u8 *msg_start = msg;
	u16 i, j, cnt;

	neg_hdr = (struct ocs_auth_neg_hdr *)msg;

	/* validate remote name */
	remote_name = &neg_hdr->name;
	if (remote_name->tag != ocs_htobe16(AUTH_NAME_TAG) ||
	    remote_name->len != ocs_htobe16(AUTH_NAME_LEN))
		return AUTH_RJT_EXPL_BAD_PAYLOAD;

	/* check if there's a negotiate collision */
	if (dhchap->role == OCS_AUTH_ROLE_INITIATOR) {
		new_role = ocs_auth_neg_collision_resolve(node,
							  remote_name->val);
		ocs_log_debug(node->ocs, "Neg collision: new-role=%d",
			      new_role);
		if (new_role == OCS_AUTH_ROLE_INITIATOR)
			return AUTH_RJT_EXPL_AUTH_STARTED;
		else /* role switch to responder */
			dhchap->role = new_role;
	}

	/* remote should advertise atlest one proto */
	if (neg_hdr->num_proto == ocs_htobe32(0))
		return AUTH_RJT_EXPL_BAD_PAYLOAD;

	msg = ptr_adv_check(msg, msg_start, msg_len, sizeof(*neg_hdr));
	if (!msg)
		return AUTH_RJT_EXPL_BAD_PAYLOAD;

	params_hdr = (struct ocs_auth_proto_params_hdr *)msg;
	cnt = ocs_be32toh(neg_hdr->num_proto);
	for (i = 0; i < cnt; i++) {
		if (params_hdr->proto_id == ocs_htobe32(PROTOCOL_DHCHAP))
			break;

		msg = ptr_adv_check(msg, msg_start, msg_len,
				    sizeof(u32) + ocs_be32toh(params_hdr->len));
		if (!msg)
			return AUTH_RJT_EXPL_BAD_PAYLOAD;

		params_hdr = (struct ocs_auth_proto_params_hdr *)msg;
	}
	/* DHCHAP not found */
	if (i == cnt)
		return AUTH_RJT_EXPL_MECH_UNUSABLE;

	/* DHCHAP proto params */
	msg = ptr_adv_check(msg, msg_start, msg_len, 2 * sizeof(u32));
	if (!msg)
		return AUTH_RJT_EXPL_BAD_PAYLOAD;

	/* Hash ID list */
	dhchap_param = (struct ocs_auth_neg_dhchap_param *)msg;
	if (dhchap_param->tag != ocs_htobe16(DHCHAP_TAG_HASHLIST))
		return AUTH_RJT_EXPL_BAD_PAYLOAD;

	msg += sizeof(u16) + sizeof(u16); /* move msg-ptr to hash-list val */
	cnt = ocs_be16toh(dhchap_param->word_cnt);
	for (i = 0; i < cnt; i++) {
		hash_id = ocs_be32toh(*((u32 *)msg + i));

		for (j = 0; j < OCS_MAX_HASH; j++) {
			if (hash_id == sli_to_auth_hash_id(cfg->hash_list[j]))
				break;
		}
		if (j < OCS_MAX_HASH) /* found hash */
			break;
	}
	/* supported hash not found */
	if (i == cnt)
		return AUTH_RJT_EXPL_HASH_UNUSABLE;

	msg = ptr_adv_check(msg, msg_start, msg_len, sizeof(u32) * cnt);
	if (!msg)
		return AUTH_RJT_EXPL_BAD_PAYLOAD;

	/* DH group ID list */
	dhchap_param = (struct ocs_auth_neg_dhchap_param *)msg;
	if (dhchap_param->tag != ocs_htobe16(DHCHAP_TAG_DHGRPLIST))
		return AUTH_RJT_EXPL_BAD_PAYLOAD;

	msg += sizeof(u16) + sizeof(u16); /* move msg-ptr to dh-grp-list val */
	cnt = ocs_be16toh(dhchap_param->word_cnt);
	for (i = 0; i < cnt; i++) {
		dh_grp_id = ocs_be32toh(*((u32 *)msg + i));

		for (j = 0; j < OCS_MAX_DH_GROUP; j++) {
			if (dh_grp_id ==
				sli_to_auth_dh_grp_id(cfg->dh_grp_list[j]))
				break;
		}
		if (j < OCS_MAX_DH_GROUP) /* found dh_grp */
			break;
	}
	/* supported dh-grp not found */
	if (i == cnt)
		return AUTH_RJT_EXPL_DHGRP_UNUSABLE;

	ocs_dhchap_params_save(dhchap, hash_id, dh_grp_id);

	/* as a responder use the trans-id of the initiator */
	dhchap->trans_id = trans_id;

	return 0;
}

/*
 * Parse the challenge msg.
 * Store dhchap params like hash_id and dh_grp_id.
 * Returns rcvd DH-val and Challenge
 * Returns AUTH_RJT_EXP_XXX error or 0 for success
 */
static u8
ocs_dhchap_chal_parse(ocs_node_t *node, u8 *msg, u32 msg_len,
		      u8 **rcvd_chal, u8 **rcvd_dh)
{
	u32 offset, hash_id, dh_grp_id, rcvd_chal_len, dh_len;
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	struct ocs_auth_cfg_info *cfg = &node->auth_cfg;
	struct ocs_dhchap_chal_hdr *hdr;
	int i;

	/* check if the msg_len is as expected */
	hdr = (struct ocs_dhchap_chal_hdr *)msg;
	offset = sizeof(*hdr);
	rcvd_chal_len = ocs_be32toh(*(u32 *)(msg + offset));
	offset += sizeof(u32) + rcvd_chal_len;
	dh_len = ocs_be32toh(*(u32 *)(msg + offset));
	offset += sizeof(u32) + dh_len;
	if (msg_len != offset)
		return AUTH_RJT_EXPL_BAD_PAYLOAD;

	msg = ptr_adv(msg, sizeof(*hdr));

	/* check if we support this hash */
	hash_id = ocs_be32toh(hdr->hash_id);
	for (i = 0; i < OCS_MAX_HASH; i++) {
		if (hash_id == sli_to_auth_hash_id(cfg->hash_list[i]))
			break;
	}
	/* hash unusable */
	if (i == OCS_MAX_HASH)
		return AUTH_RJT_EXPL_HASH_UNUSABLE;

	/* check if we support this dh-grp*/
	dh_grp_id = ocs_be32toh(hdr->dh_grp_id);
	for (i = 0; i < OCS_MAX_DH_GROUP; i++) {
		if (dh_grp_id == sli_to_auth_dh_grp_id(cfg->dh_grp_list[i]))
			break;
	}
	/* dh unusable */
	if (i == OCS_MAX_DH_GROUP)
		return AUTH_RJT_EXPL_DHGRP_UNUSABLE;

	/* challenge */
	if (rcvd_chal_len != ocs_hash_len(hash_id))
		return AUTH_RJT_EXPL_BAD_PAYLOAD;
	msg = ptr_adv(msg, sizeof(u32));
	*rcvd_chal = msg;
	msg = ptr_adv(msg, rcvd_chal_len);

	/* dh-val */
	if (dh_len != dh_group_array[dh_grp_id].length)
		return AUTH_RJT_EXPL_BAD_PAYLOAD;
	msg = ptr_adv(msg, sizeof(u32));
	*rcvd_dh = dh_len ? msg : NULL;

	/* All checks done: save the auth params */
	ocs_dhchap_params_save(dhchap, hash_id, dh_grp_id);
	return 0;
}

/*
 * Parse the AUTH_reply msg.
 * Return parsed response, DH val and Challenge if present
 * Returns AUTH_RJT_EXP_XXX error or 0 for success
 */
static u8
ocs_dhchap_reply_parse(ocs_node_t *node, u8 *msg, u32 msg_len,
		      u8 **rcvd_resp, u8 **rcvd_dh, u8 **rcvd_chal)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	u32 offset, resp_len, rcvd_chal_len, dh_len;

	*rcvd_resp = NULL;
	*rcvd_dh = NULL;
	*rcvd_chal = NULL;

	/* check if the msg_len is as expected */
	offset = sizeof(u32) + dhchap->hash_len /* resp */ +
		 sizeof(u32) + dhchap->dh_len /* dh val */;
	rcvd_chal_len = ocs_be32toh(*(u32 *)(msg + offset));
	offset += sizeof(u32) + rcvd_chal_len;
	if (msg_len != offset)
		return AUTH_RJT_EXPL_BAD_PAYLOAD;

	/* resp len */
	resp_len = ocs_be32toh(*(u32 *)msg);
	if (resp_len != dhchap->hash_len)
		return AUTH_RJT_EXPL_BAD_PAYLOAD;

	/* move to response val */
	msg = ptr_adv(msg, sizeof(u32));
	*rcvd_resp = msg;

	/* move to dh val length */
	msg = ptr_adv(msg, resp_len);
	dh_len = ocs_be32toh(*(u32 *)msg);
	if (dh_len != dhchap->dh_len)
		return AUTH_RJT_EXPL_BAD_PAYLOAD;

	/* move to dh val */
	msg = ptr_adv(msg, sizeof(u32));
	*rcvd_dh = msg;

	/* move to chal val */
	msg = ptr_adv(msg, dh_len + sizeof(u32));
	*rcvd_chal = msg;

	return 0;
}

/*
 * Parse DHCHAP_Scucess msg:
 * Returns resp if present
 * Returns AUTH_RJT_EXP_XXX error or 0 for success
 */
static u8
ocs_dhchap_success_parse(ocs_node_t *node, u8 *msg, u32 msg_len, u8 **resp)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	struct ocs_auth_cfg_info *cfg = &node->auth_cfg;
	u32 resp_len;

	resp_len = ocs_be32toh(*(u32 *)msg);

	/* Check if the msg has enough room for resp */
	if (msg_len != sizeof(u32) + resp_len)
		return AUTH_RJT_EXPL_BAD_PAYLOAD;

	if (!resp_len) {
		/* No response: if as an Auth_Initiator we sent a challenge,
		 * then we expect a response in the DHCHAP_Success msg
		 */
		if (dhchap->role == OCS_AUTH_ROLE_INITIATOR &&
		    cfg->flags & OCS_AUTH_FLAGS_BIDI)
			return AUTH_RJT_EXPL_BAD_PROTOCOL;

		*resp = NULL;
	} else {
		if (resp_len != dhchap->hash_len)
			return AUTH_RJT_EXPL_BAD_PAYLOAD;
		*resp = msg + sizeof(u32);
	}

	return 0;
}

/* Fwd decls */
static void
ocs_auth_evt_post(ocs_node_t *node, enum ocs_auth_evt evt, void *arg);
static void
ocs_dhchap_state_change(ocs_node_t *node, enum ocs_dhchap_state new_state);

/* Timer routines */
static int
ocs_auth_next_msg_timer_stop(ocs_node_t *node)
{
	return ocs_del_timer(&node->auth.dhchap.next_msg_timer);
}

static void
ocs_dhchap_next_msg_timer_cb(void *arg)
{
	ocs_node_t *node = arg;

	/* As the timer event is triggered from outside the parent node
	 * FSM, it needs to be processed under the node lock.
	 */
	ocs_node_lock(node);
	ocs_auth_evt_post(node, OCS_AUTH_EVT_NEXT_MSG_TIMEOUT, NULL);
	ocs_node_unlock(node);
}

static int
ocs_auth_next_msg_timer_start(ocs_node_t *node)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;

	return ocs_setup_timer(node->ocs, &dhchap->next_msg_timer,
			       ocs_dhchap_next_msg_timer_cb, node, AUTH_TOV_MS, false);
}

static void
ocs_auth_cleanup(ocs_node_t *node, enum ocs_auth_status status)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;

	dhchap->eph_dh_computed = false;
	ocs_auth_next_msg_timer_stop(node);
	dhchap->status = status;
	ocs_dhchap_state_change(node, OCS_DHCHAP_STATE_DONE);
}

static void
ocs_auth_done(ocs_node_t *node, enum ocs_auth_status status)
{
	ocs_sm_event_t node_evt = OCS_EVT_NODE_AUTH_FAIL;

	ocs_log_info(node->ocs, "DH-CHAP authentication Done: status=%d\n",
		     status);

	ocs_auth_cleanup(node, status);

	if (status == OCS_AUTH_SUCCESS) {
		node_evt = OCS_EVT_NODE_AUTH_OK;
		/* TODO restart re-auth timer */
	}
	ocs_node_post_event(node, node_evt, NULL);
}

/*
 * Move the FSM to a new state when a msg is sent out
 * expect_reply indicates if a reply to the sent msg is expected.
 * A vanilla success msg (without an embedded dhchap resp) and a Reject
 * msg don't expect a reply
 */
static int
ocs_auth_msg_sent(ocs_node_t *node, u8 msg_type, bool expect_reply)
{
	int rc = 0;

	switch(msg_type) {
	case AUTH_REJECT:
		break;
	case AUTH_NEGOTIATE:
		ocs_dhchap_state_change(node, OCS_DHCHAP_STATE_NEG_SENT);
		break;
	case DHCHAP_CHALLENGE:
		ocs_dhchap_state_change(node, OCS_DHCHAP_STATE_CHAL_SENT);
		break;
	case DHCHAP_REPLY:
		ocs_dhchap_state_change(node, OCS_DHCHAP_STATE_REPLY_SENT);
		break;
	case DHCHAP_SUCCESS:
		if (expect_reply)
			ocs_dhchap_state_change(node,
						OCS_DHCHAP_STATE_SUCCESS_SENT);
		else
			ocs_auth_done(node, OCS_AUTH_SUCCESS);
		break;
	default:
		break;
	}

	if (expect_reply)
		rc = ocs_auth_next_msg_timer_start(node);
	return rc;
}

static int
ocs_auth_rej(ocs_node_t *node, __be32 trans_id, u8 code, u8 expl)
{
	int rc;

	ocs_log_debug(node->ocs, "trans_id=%x, code/expl=%s/%s\n",
		      trans_id, auth_err_name(code), auth_expl_name(expl));
	rc = ocs_auth_rej_send(node, trans_id, code, expl);
	if (!rc)
		ocs_auth_msg_sent(node, AUTH_REJECT, false);

	switch (expl) {
	case AUTH_RJT_EXPL_AUTH_STARTED:
	case AUTH_RJT_EXPL_RESTART_AUTH:
		break;
	case AUTH_RJT_EXPL_AUTH_FAILED:
		ocs_auth_done(node, OCS_AUTH_FAILURE);
		break;
	default:
		ocs_auth_done(node, OCS_AUTH_ERROR);
		break;
	}
	return rc;
}

/* Auth msg receive routines */
/*
 * Process the AUTH_Reject msg:
 * a) Parse the error code
 * b) If fatal error, end the auth FSM and notify Node FSM
 */
static int
ocs_auth_rej_recv(ocs_node_t *node, u8 *msg, u32 msg_len, __be32 trans_id)
{
	struct ocs_auth_rej_msg *rej = (struct ocs_auth_rej_msg *)msg;

	ocs_log_debug(node->ocs, "code/expl=%s/%s\n",
		      auth_err_name(rej->code), auth_expl_name(rej->expl));

	switch (rej->expl) {
	case AUTH_RJT_EXPL_AUTH_STARTED:
		/* Ignore this reject, the collision is resolved when
		 * a Auth_Negotiate msg is rcvd from the peer
		 */
		break;
	case AUTH_RJT_EXPL_RESTART_AUTH:
		/* Restart authenticaion */
		ocs_auth_evt_post(node, OCS_AUTH_EVT_RESTART, NULL);
		break;
	case AUTH_RJT_EXPL_AUTH_FAILED:
		ocs_auth_done(node, OCS_AUTH_FAILURE);
		break;
	default:
		ocs_auth_done(node, OCS_AUTH_ERROR);
		break;
	}
	return 0;
}

/*
 * Process the AUTH_Negotiate msg:
 * a) Parse the rcvd Negotiate msg, store DHCHAP params like hash_id, dh_grp_id
 * b) Compute Challenge, nonce and DH-val
 * c) Send DHCHAP_Challenge msg
 */
static int
ocs_auth_neg_recv(ocs_node_t *node, u8 *msg, u32 msg_len, __be32 trans_id)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	u8 send_dh[MAX_DH_VAL_LEN], dh_base = DH_BASE, expl;
	int rc;

	expl = ocs_auth_neg_parse(node, msg, msg_len, trans_id);
	if (expl)
		return ocs_auth_rej(node, trans_id, expl, auth_rjt_code(expl));

	/* compute and store challenge */
	ocs_dhchap_chal_compute(dhchap->chal, dhchap->hash_len);

	if (dhchap->dh_len) {
		/* Compute and store nonce */
		ocs_auth_nonce_compute(dhchap->nonce, sizeof(dhchap->nonce));

		/* Compute dh val to be sent in the DHCHAP_Challenge */
		rc = ocs_auth_dh_compute(dhchap->dh_grp_id, &dh_base,
					 sizeof(dh_base), dhchap->nonce,
					 sizeof(dhchap->nonce), send_dh);
		if (rc)
			return rc;
	}

	/* send challenge */
	rc = ocs_dhchap_chal_send(node, send_dh);
	if (rc)
		return rc;

	return ocs_auth_msg_sent(node, DHCHAP_CHALLENGE, true);
}

/*
 * Process the DHCHAP_Challenge msg:
 * a) Parse the rcvd Challenge and DH-val
 * b) Compute response
 * c) Send DHCHAP_Reply msg
 */
static int
ocs_dhchap_chal_recv(ocs_node_t *node, u8 *msg, u32 msg_len)
{
	u8 resp[MAX_HASH_LEN], dh[MAX_DH_VAL_LEN], dh_base = DH_BASE;
	u8 *rcvd_chal, *rcvd_dh, *send_dh, *send_chal, expl;
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	struct ocs_auth_cfg_info *cfg = &node->auth_cfg;
	int rc;

	/* parse chal and dh, save auth params */
	expl = ocs_dhchap_chal_parse(node, msg, msg_len, &rcvd_chal, &rcvd_dh);
	if (expl)
		return ocs_auth_rej(node, dhchap->trans_id,
				    expl, auth_rjt_code(expl));

	/* dh_len already stored by now */
	if (dhchap->dh_len) {
		/* Compute and store the nonce */
		ocs_auth_nonce_compute(dhchap->nonce, sizeof(dhchap->nonce));

		/* Compute dh val to be sent in the DHCHAP_Reply */
		rc = ocs_auth_dh_compute(dhchap->dh_grp_id, &dh_base,
					 sizeof(dh_base), dhchap->nonce,
					 sizeof(dhchap->nonce), dh);
		if (rc)
			return rc;
		send_dh = dh;
	} else {
		send_dh = NULL;
	}

	/* Compute response */
	rc = ocs_dhchap_resp_compute(node, resp, rcvd_chal, rcvd_dh,
				     node->auth_pass.remote,
				     node->auth_pass.remote_len);
	if (rc)
		return rc;

	/* If bidi compute and store challenge */
	if (cfg->flags & OCS_AUTH_FLAGS_BIDI) {
		ocs_dhchap_chal_compute(dhchap->chal, dhchap->hash_len);
		send_chal = dhchap->chal;
	} else {
		send_chal = NULL;
	}

	/* Send DHCHAP_Reply msg */
	rc = ocs_dhchap_reply_send(node, resp, send_dh, send_chal);
	if (rc)
		return rc;

	return ocs_auth_msg_sent(node, DHCHAP_REPLY, true);
}

/*
 * Process the DHCHAP_Reply msg:
 * a) Parse the rcvd Reply msg
 * b) Validate received resp
 * c) Compute response if Challenge is rcvd
 * d) Send DHCHAP_Success msg if validatition(b) successful
 */
static int
ocs_dhchap_reply_recv(ocs_node_t *node, u8 *msg, u32 msg_len)
{
	u8 resp[MAX_HASH_LEN], *rcvd_resp, *rcvd_dh, *rcvd_chal, *send_resp;
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	u8 expl;
	int rc;

	/* Parse reply msg; this computes and stores the eph_dh_val */
	expl = ocs_dhchap_reply_parse(node, msg, msg_len, &rcvd_resp,
				      &rcvd_dh, &rcvd_chal);
	if (expl)
		return ocs_auth_rej(node, dhchap->trans_id,
				    expl, auth_rjt_code(expl));

	/* validate response */
	rc = ocs_dhchap_resp_validate(node, rcvd_resp, rcvd_dh);
	if (rc < 0)
		return rc;
	else if (rc > 0) /* Auth failed */
		return ocs_auth_rej(node, dhchap->trans_id,
				    AUTH_RJT_CODE_FAILURE,
				    AUTH_RJT_EXPL_AUTH_FAILED);

	/* Compute response if Challenge is rcvd */
	if (rcvd_chal) {
		rc = ocs_dhchap_resp_compute(node, resp, rcvd_chal, rcvd_dh,
					     node->auth_pass.remote,
					     node->auth_pass.remote_len);
		if (rc)
			return rc;
		send_resp = resp;
	} else {
		send_resp = NULL;
	}

	/* Send AUTH_Success along with response */
	rc = ocs_dhchap_success_send(node, send_resp);
	if (rc)
		return rc;

	return ocs_auth_msg_sent(node, DHCHAP_SUCCESS, send_resp != NULL);
}

/*
 * Process the DHCHAP_Success msg:
 * a) Parse the rcvd Success msg
 * b) Validate response if present
 * c) Send DHCHAP_Success msg if validatition successful
 */
static int
ocs_dhchap_success_recv(ocs_node_t *node, u8 *msg, u32 msg_len)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	u8 *resp, expl;
	int rc;

	expl = ocs_dhchap_success_parse(node, msg, msg_len, &resp);
	if (expl)
		return ocs_auth_rej(node, dhchap->trans_id,
				    expl, auth_rjt_code(expl));

	if (resp)
		rc = ocs_dhchap_resp_validate(node, resp, NULL);
	else
		rc = 0;

	if (rc > 0) { /* Auth failed */
		rc = ocs_auth_rej(node, dhchap->trans_id, AUTH_RJT_CODE_FAILURE,
				  AUTH_RJT_EXPL_AUTH_FAILED);
	} else if (rc == 0) {
		/* Send DHCHAP_Success msg if resp was present */
		if (resp) {
			rc = ocs_dhchap_success_send(node, NULL);
			if (rc)
				return rc;

			rc = ocs_auth_msg_sent(node, DHCHAP_SUCCESS, false);
		} else {
			ocs_auth_done(node, OCS_AUTH_SUCCESS);
		}
	}

	return rc;
}

/*
 * DHCHAP FSM
 * **********
 *             Auth_Initiator               Auth_Responder
 *
 *                        +                  +
 *     DHCHAP_STATE_INIT  |                  | DHCHAP_STATE_INIT
 *                        |        AUTH_Neg  |
 *                        +----------------->+
 * DHCHAP_STATE_NEG_SENT  |                  |
 *                        | DHCHAP_Chal      |
 *                        +<-----------------+
 *                        |                  | DHCHAP_STATE_CHAL_SENT
 *                        |     DHCHAP_Reply |
 *                        +----------------->+
 *DHCHAP_STATE_REPLY_SENT |                  |
 *                        | DHCHAP_Success   |
 *                        <------------------+
 *                        |                  | DHCHAP_STATE_SUCCESS_SENT
 *                        |   DHCHAP_Success |
 *                        +----------------->+
 *   DHCHAP_STATE_DONE    |                  | DHCHAP_STATE_DONE
 *                        |                  |
 */

static void
ocs_auth_unexp_evt(ocs_node_t *node, enum ocs_auth_evt evt)
{
	ocs_log_err(node->ocs, "Error: unexpected evt=%s in state=%sn",
		    auth_evt_name(evt),
		    dhchap_state_name(node->auth.dhchap.state));
}

static void
ocs_dhchap_state_init(ocs_node_t *node, enum ocs_auth_evt evt, void *arg)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	struct ocs_auth_msg_info *msg_info;
	int rc = 0;

	switch(evt) {
	case OCS_AUTH_EVT_START:
		if (dhchap->role == OCS_AUTH_ROLE_INITIATOR) {
			rc = ocs_auth_neg_send(node);
			if  (rc)
				break;
			rc = ocs_auth_msg_sent(node, AUTH_NEGOTIATE, true);
		} else {
			/* Expect to receive Neg msg in AUTH_TOVs */
			rc = ocs_auth_next_msg_timer_start(node);
		}
		break;

	case OCS_AUTH_EVT_NEG_RECV:
		msg_info = arg;
		rc = ocs_auth_neg_recv(node, msg_info->msg, msg_info->len,
				       msg_info->trans_id);
		break;

	case OCS_AUTH_EVT_DHCHAP_CHAL_RECV:
	case OCS_AUTH_EVT_DHCHAP_REPLY_RECV:
	case OCS_AUTH_EVT_DHCHAP_SUCCESS_RECV:
		msg_info = arg;
		rc = ocs_auth_rej(node, msg_info->trans_id,
				  AUTH_RJT_CODE_FAILURE,
				  AUTH_RJT_EXPL_BAD_PROTOCOL);
		break;

	case OCS_AUTH_EVT_REJ_RECV:
		msg_info = arg;
		rc = ocs_auth_rej_recv(node, msg_info->msg, msg_info->len,
				       msg_info->trans_id);
		break;

	case OCS_AUTH_EVT_MSG_SEND_FAIL:
	case OCS_AUTH_EVT_NEXT_MSG_TIMEOUT:
	default:
		rc = -1;
		break;
	}

	if (rc)
		ocs_auth_done(node, OCS_AUTH_ERROR);
}

static void
ocs_dhchap_state_neg_sent(ocs_node_t *node,enum ocs_auth_evt evt, void *arg)
{
	struct ocs_auth_msg_info *msg_info;
	int rc = 0;

	switch(evt) {
	case OCS_AUTH_EVT_NEG_RECV:
		msg_info = arg;
		rc = ocs_auth_neg_recv(node, msg_info->msg, msg_info->len,
				       msg_info->trans_id);
		break;

	case OCS_AUTH_EVT_DHCHAP_CHAL_RECV:
		msg_info = arg;
		rc = ocs_dhchap_chal_recv(node, msg_info->msg, msg_info->len);
		break;

	case OCS_AUTH_EVT_DHCHAP_REPLY_RECV:
	case OCS_AUTH_EVT_DHCHAP_SUCCESS_RECV:
		msg_info = arg;
		rc = ocs_auth_rej(node, msg_info->trans_id,
				  AUTH_RJT_CODE_FAILURE,
				  AUTH_RJT_EXPL_BAD_PROTOCOL);
		break;

	case OCS_AUTH_EVT_REJ_RECV:
		msg_info = arg;
		rc = ocs_auth_rej_recv(node, msg_info->msg, msg_info->len,
				       msg_info->trans_id);
		break;

	case OCS_AUTH_EVT_MSG_SEND_FAIL:
	case OCS_AUTH_EVT_NEXT_MSG_TIMEOUT:
	default:
		rc = -1;
		break;
	}

	if (rc)
		ocs_auth_done(node, OCS_AUTH_ERROR);
}

static void
ocs_dhchap_state_chal_sent(ocs_node_t *node, enum ocs_auth_evt evt, void *arg)
{
	struct ocs_auth_msg_info *msg_info;
	int rc = 0;

	switch(evt) {
	case OCS_AUTH_EVT_DHCHAP_REPLY_RECV:
		msg_info = arg;
		rc = ocs_dhchap_reply_recv(node, msg_info->msg, msg_info->len);
		break;

	case OCS_AUTH_EVT_REJ_RECV:
		msg_info = arg;
		rc = ocs_auth_rej_recv(node, msg_info->msg, msg_info->len,
				       msg_info->trans_id);
		break;

	case OCS_AUTH_EVT_MSG_SEND_FAIL:
	case OCS_AUTH_EVT_NEXT_MSG_TIMEOUT:
	default:
		rc = -1;
		break;
	}

	if (rc)
		ocs_auth_done(node, OCS_AUTH_ERROR);
}

static void
ocs_dhchap_state_reply_sent(ocs_node_t *node, enum ocs_auth_evt evt, void *arg)
{
	struct ocs_auth_msg_info *msg_info;
	int rc = 0;

	switch(evt) {
	case OCS_AUTH_EVT_DHCHAP_SUCCESS_RECV:
		msg_info = arg;
		rc = ocs_dhchap_success_recv(node, msg_info->msg,
					     msg_info->len);
		break;

	case OCS_AUTH_EVT_DHCHAP_CHAL_RECV:
	case OCS_AUTH_EVT_NEG_RECV:
	case OCS_AUTH_EVT_DHCHAP_REPLY_RECV:
		msg_info = arg;
		rc = ocs_auth_rej(node, msg_info->trans_id,
				  AUTH_RJT_CODE_FAILURE,
				  AUTH_RJT_EXPL_BAD_PROTOCOL);
		break;

	case OCS_AUTH_EVT_REJ_RECV:
		msg_info = arg;
		rc = ocs_auth_rej_recv(node, msg_info->msg, msg_info->len,
				       msg_info->trans_id);
		break;

	case OCS_AUTH_EVT_MSG_SEND_FAIL:
	case OCS_AUTH_EVT_NEXT_MSG_TIMEOUT:
	default:
		rc = -1;
		break;
	}

	if (rc)
		ocs_auth_done(node, OCS_AUTH_ERROR);
}

static void
ocs_dhchap_state_success_sent(ocs_node_t *node, enum ocs_auth_evt evt,
			      void *arg)
{
	struct ocs_auth_msg_info *msg_info;
	int rc = 0;

	switch(evt) {
	case OCS_AUTH_EVT_DHCHAP_SUCCESS_RECV:
		msg_info = arg;
		rc = ocs_dhchap_success_recv(node, msg_info->msg,
					     msg_info->len);
		break;

	case OCS_AUTH_EVT_DHCHAP_CHAL_RECV:
	case OCS_AUTH_EVT_NEG_RECV:
	case OCS_AUTH_EVT_DHCHAP_REPLY_RECV:
		msg_info = arg;
		rc = ocs_auth_rej(node, msg_info->trans_id,
				  AUTH_RJT_CODE_FAILURE,
				  AUTH_RJT_EXPL_BAD_PROTOCOL);
		break;

	case OCS_AUTH_EVT_REJ_RECV:
		msg_info = arg;
		rc = ocs_auth_rej_recv(node, msg_info->msg, msg_info->len,
				       msg_info->trans_id);
		break;

	default:
		ocs_auth_unexp_evt(node, evt);
		break;
	}

	if (rc)
		ocs_auth_done(node, OCS_AUTH_ERROR);
}

static void
ocs_dhchap_state_done(ocs_node_t *node, enum ocs_auth_evt evt, void *arg)
{
	struct ocs_auth_msg_info *msg_info;
	int rc = 0;

	switch(evt) {
	case OCS_AUTH_EVT_RESTART:
		rc = ocs_node_auth_start(node, OCS_AUTH_ROLE_INITIATOR,
					 OCS_AUTH_EVT_START, NULL);
		break;

	case OCS_AUTH_EVT_NEG_RECV:
		/* looks like fabric wants to re-authentate us:
		 * reset the FSM and repost evt
		 */
		rc = ocs_node_auth_start(node, OCS_AUTH_ROLE_RESPONDER,
					 OCS_AUTH_EVT_NEG_RECV, arg);
		break;

	case OCS_AUTH_EVT_DHCHAP_CHAL_RECV:
	case OCS_AUTH_EVT_DHCHAP_REPLY_RECV:
	case OCS_AUTH_EVT_DHCHAP_SUCCESS_RECV:
		/* After auth is completed don't expect to recv any
		 * of these msgs
		 */
		msg_info = arg;
		rc = ocs_auth_rej(node, msg_info->trans_id,
				  AUTH_RJT_CODE_FAILURE,
				  AUTH_RJT_EXPL_BAD_PROTOCOL);
		break;

	case OCS_AUTH_EVT_REJ_RECV:
		msg_info = arg;
		rc = ocs_auth_rej_recv(node, msg_info->msg, msg_info->len,
				       msg_info->trans_id);
		break;

	default:
		ocs_auth_unexp_evt(node, evt);
		break;
	}

	if (rc)
		ocs_log_err(node->ocs, "Error rc=%d\n", rc);
}

struct ocs_dhchap_sm_table {
	enum ocs_dhchap_state state;
	void (*ocs_auth_sm_handler)(ocs_node_t *, enum ocs_auth_evt, void *arg);
} dhchap_sm_table[OCS_DHCHAP_STATE_LAST] =
	{
	 {OCS_DHCHAP_STATE_INIT, ocs_dhchap_state_init},
	 {OCS_DHCHAP_STATE_NEG_SENT, ocs_dhchap_state_neg_sent},
	 {OCS_DHCHAP_STATE_CHAL_SENT, ocs_dhchap_state_chal_sent},
	 {OCS_DHCHAP_STATE_REPLY_SENT, ocs_dhchap_state_reply_sent},
	 {OCS_DHCHAP_STATE_SUCCESS_SENT, ocs_dhchap_state_success_sent},
	 {OCS_DHCHAP_STATE_DONE, ocs_dhchap_state_done}
	};

static void
ocs_auth_evt_post(ocs_node_t *node, enum ocs_auth_evt evt, void *arg)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;

	ocs_log_debug(node->ocs, "DHCHAP SM: state=%s, evt=%s\n",
		      dhchap_state_name(dhchap->state), auth_evt_name(evt));

	ocs_assert(dhchap_sm_table[dhchap->state].state == dhchap->state);
	dhchap_sm_table[dhchap->state].ocs_auth_sm_handler(node, evt, arg);
}

static void
ocs_dhchap_state_change(ocs_node_t *node, enum ocs_dhchap_state new_state)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;

	ocs_log_debug(node->ocs, "DHCHAP SM: state=%s, new_state=%s\n",
		      dhchap_state_name(dhchap->state),
		      dhchap_state_name(new_state));
	dhchap->state = new_state;
}

/* Response to an Auth ELS send */
void
ocs_auth_els_resp_cb(ocs_node_t *node, ocs_node_cb_t *cbdata, void *cb_arg)
{
	struct ocs_auth_els_hdr *hdr = cbdata->els->els_info->els_req.virt;

	if (cbdata->status == SLI4_FC_WCQE_STATUS_SUCCESS) {
		ocs_log_debug(node->ocs, "Auth ELS send OK: msg_code=%s\n",
			      auth_msg_name(hdr->msg_code));
	} else {
		/* TODO parse LS_RJT details */
		ocs_log_debug(node->ocs, "Auth ELS send Fail: msg_code=%s\n",
			      auth_msg_name(hdr->msg_code));
		ocs_auth_evt_post(node, OCS_AUTH_EVT_MSG_SEND_FAIL, NULL);
	}
}

/* Rcvd an Auth ELS */
void
ocs_auth_els_recv(ocs_node_t *node, ocs_node_cb_t *cbdata)
{
	fc_header_t *fc_hdr = cbdata->header;
	struct ocs_auth_els_hdr *hdr = cbdata->payload;
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	u32 msg_len = ocs_be32toh(hdr->msg_len);
	u32 els_len = cbdata->payload_len;
	struct ocs_auth_msg_info msg_info;
	enum ocs_auth_evt evt;

	ocs_log_debug(node->ocs, "auth msg_code=%s\n",
		      auth_msg_name(hdr->msg_code));

	if (!ocs_node_auth_enabled(node))
		goto ls_rjt;

	switch (hdr->msg_code) {
	case DHCHAP_CHALLENGE:
		evt = OCS_AUTH_EVT_DHCHAP_CHAL_RECV;
		break;
	case DHCHAP_REPLY:
		evt = OCS_AUTH_EVT_DHCHAP_REPLY_RECV;
		break;
	case DHCHAP_SUCCESS:
		evt = OCS_AUTH_EVT_DHCHAP_SUCCESS_RECV;
		break;
	case AUTH_REJECT:
		evt = OCS_AUTH_EVT_REJ_RECV;
		break;
	case AUTH_NEGOTIATE:
		evt = OCS_AUTH_EVT_NEG_RECV;
		break;
	default:
		/* only the above msgs supported now */
		goto ls_rjt;
	}

	ocs_auth_next_msg_timer_stop(node);

	/* send LS_ACC before checking other fields of Auth msg */
	ocs_send_ls_acc(cbdata->io, ocs_be16toh(fc_hdr->ox_id), NULL, NULL);

	/* validate trans_id */
	if (hdr->msg_code != AUTH_NEGOTIATE && hdr->msg_code != AUTH_REJECT) {
		if (hdr->trans_id != dhchap->trans_id) {
			ocs_auth_rej(node, hdr->trans_id, AUTH_RJT_CODE_FAILURE,
				     AUTH_RJT_EXPL_BAD_PROTOCOL);
			return;
		}
	}

	/* check msg_len, version is atleast VER_1 */
	if (msg_len != (els_len - sizeof(*hdr)) ||
	    hdr->protocol_ver < AUTH_PROTOCOL_VER_1) {
		ocs_auth_rej(node, hdr->trans_id, AUTH_RJT_CODE_FAILURE,
			     AUTH_RJT_EXPL_BAD_PAYLOAD);
		return;
	}

	/* we support only VER_1 */
	if (hdr->protocol_ver > AUTH_PROTOCOL_VER_1) {
		ocs_auth_rej(node, hdr->trans_id, AUTH_RJT_CODE_LOGICAL_ERR,
			     AUTH_RJT_EXPL_PROTO_VER_NOSUPP);
		return;
	}

	msg_info.code = hdr->msg_code;
	msg_info.msg = (u8 *)hdr + sizeof(*hdr);
	msg_info.len = msg_len;
	msg_info.trans_id = hdr->trans_id;

	ocs_auth_evt_post(node, evt, &msg_info);
	return;

ls_rjt:
	ocs_send_ls_rjt(cbdata->io, ocs_be16toh(fc_hdr->ox_id),
			FC_REASON_COMMAND_NOT_SUPPORTED,
			FC_EXPL_REQUEST_NOT_SUPPORTED, 0, NULL, NULL);
	return;
}

/*
 * Start the auth FSM by initing role, state and posting the event
 * to kick start it.
 */
int
ocs_node_auth_start(ocs_node_t *node, enum ocs_auth_role role,
		    enum ocs_auth_evt evt, void *arg)
{
	struct ocs_dhchap_info *dhchap = &node->auth.dhchap;
	bool rc;

	dhchap->role = role;
	if (role == OCS_AUTH_ROLE_INITIATOR)
		dhchap->trans_id = ocs_rand(); /* first time seeding */

	/* lkup cfg for any changes */
	rc = ocs_auth_cfg_lkup(node);
	if (!rc) {
		ocs_log_err(node->ocs, "Error: auth cfg missing\n");
		return -ENOENT;
	}

	ocs_dhchap_state_change(node, OCS_DHCHAP_STATE_INIT);
	ocs_auth_evt_post(node, evt, arg);
	return 0;
}

void
ocs_node_auth_stop(ocs_node_t *node)
{
	if (ocs_node_auth_enabled(node))
		ocs_auth_cleanup(node, OCS_AUTH_STOPPED);
}

bool
ocs_node_auth_enabled(ocs_node_t *node)
{
	struct ocs_auth_cfg_info *info = &node->auth_cfg;

	return info->flags & OCS_AUTH_FLAGS_VALID &&
		(info->mode == OCS_AUTH_MODE_ACTIVE ||
		 info->mode == OCS_AUTH_MODE_PASSIVE);
}

void
ocs_node_auth_init(ocs_node_t *node)
{
	bool rc;

	node->auth.dhchap.state = OCS_DHCHAP_STATE_INIT;

	rc = ocs_auth_cfg_lkup(node);
	ocs_log_debug(node->ocs, "auth cfg %s\n", rc ? "found" : "not found");
}
