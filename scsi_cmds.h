/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2018 Broadcom.  All Rights Reserved.
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
 * Define all SCSI command byte values
 */


// scsi_cmds.h - borrowed from bridge f/w
#if !defined(__SCSI_CMDS_H__)
#define __SCSI_CMDS_H__
#define SCSI_ATA_PASSTHRU12             0xA1
#define SCSI_ATA_PASSTHRU16             0x85
#define SCSI_E6_SMART_DUMP              0xE6
#define SCSI_F7_DUMP_CLEAR_RET_TRIGR    0xF7
#define SCSI_CHANGE_DEFINITION          0x40
#define SCSI_FORMAT_UNIT                0x04
#define SCSI_INQUIRY                    0x12
#define SCSI_LOCK_UNLOCK_CACHE10        0x36
#define SCSI_LOCK_UNLOCK_CACHE16        0x92
#define SCSI_LOG_SELECT                 0x4C
#define SCSI_LOG_SENSE                  0x4D
#define SCSI_MODE_SELECT10              0x55
#define SCSI_MODE_SELECT6               0x15
#define SCSI_MODE_SENSE10               0x5A
#define SCSI_MODE_SENSE6                0x1A
#define SCSI_MOVE_MEDIUM                0xA7
#define SCSI_PERSISTENT_RESERVE_IN      0x5E
#define SCSI_PERSISTENT_RESERVE_OUT     0x5F
#define SCSI_PRE_FETCH10                0x34
#define SCSI_PRE_FETCH16                0x90
#define SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL   0x1E
#define SCSI_READ_ATTRIBUTE             0x8C
#define SCSI_READ_BUFFER                0x3C
#define SCSI_READ_CAPACITY10            0x25
#define SCSI_READ_DEFECT_DATA10         0x37
#define SCSI_READ_DEFECT_DATA12         0xB7
#define SCSI_READ_ELEMENT_STATUS        0xB4
#define SCSI_READ_LONG10                0x3E
#define SCSI_READ10                     0x28
#define SCSI_READ12                     0xA8
#define SCSI_READ16                     0x88
#define SCSI_READ6                      0x08
#define SCSI_REASSIGN_BLOCKS            0x07
#define SCSI_REBUILD16                  0x81
#define SCSI_RECV_DIAGNOSTIC_RESULTS    0x1C
#define SCSI_REGENERATE16               0x82
#define SCSI_RELEASE10                  0x57
#define SCSI_RELEASE6                   0x17
#define SCSI_REPORT_LUNS                0xA0
#define SCSI_MAINTENANCE_IN             0xA3
#define SCSI_MAINTENANCE_OUT            0xA4
#define SCSI_READ_MEDIA_SERIAL_NUMBER   0xAB
#define SCSI_REQUEST_SENSE              0x03
#define SCSI_RESERVE10                  0x56
#define SCSI_RESERVE6                   0x16
#define SCSI_REZERO_UNIT                0x01
#define SCSI_SECURITY_PROTOCOL_IN       0xA2
#define SCSI_SECURITY_PROTOCOL_OUT      0xB5
#define SCSI_SEEK6                      0x0B
#define SCSI_SEEK10                     0x2B
#define SCSI_SEND_DIAGNOSTIC            0x1D
#define SCSI_SERVICE_ACTION_IN16        0x9E
#define SCSI_SERVICE_ACTION_OUT16       0x9F
#define SCSI_SET_LIMITS10               0x33
#define SCSI_SET_LIMITS12               0xB3
#define SCSI_START_STOP_UNIT            0x1B
#define SCSI_SYNCHRONIZE_CACHE10        0x35
#define SCSI_SYNCHRONIZE_CACHE16        0x91
#define SCSI_TEST_UNIT_READY            0x00
#define SCSI_UNMAP                      0x42
#define SCSI_VARIABLE_LEN_CDB           0x7F
#define SCSI_VERIFY10                   0x2F
#define SCSI_VERIFY12                   0xAF
#define SCSI_VERIFY16                   0x8F
#define SCSI_WRITE_AND_VERIFY10         0x2E
#define SCSI_WRITE_AND_VERIFY12         0xAE
#define SCSI_WRITE_AND_VERIFY16         0x8E
#define SCSI_WRITE_BUFFER               0x3B
#define SCSI_WRITE_LONG10               0x3F
#define SCSI_WRITE_SAME10               0x41
#define SCSI_WRITE_SAME16               0x93
#define SCSI_WRITE10                    0x2A
#define SCSI_WRITE12                    0xAA
#define SCSI_WRITE16                    0x8A
#define SCSI_WRITE6                     0x0A
#define SCSI_XDREAD10                   0x52
#define SCSI_XDWRITE_EXTENDED16         0x80
#define SCSI_XDWRITE10                  0x50
#define SCSI_XDWRITEREAD10              0x53
#define SCSI_XPWRITE10                  0x51

#ifdef EMC_SKIP_MASK_LONG_ENABLE
#define SCSI_EMC_SKIP_MASK_LONG         0xCA
#endif

#ifdef ENABLE_SCSI_PASSTHRU_FRAMEWORK
#define SCSI_SAMSUNG_VU_CMD_D0          0xD0
#define SCSI_SAMSUNG_VU_CMD_DA          0xDA
#endif

#ifdef E7_ADD_DEFECT_LIST_ENABLE
#define SCSI_E7_ADD_DEFECT_LIST         0xE7
#endif

#ifdef EMC_SKIP_MASK_ENABLE
#define SCSI_EMC_SKIP_MASK              0xEA
#endif

#ifdef IBM_SKIP_MASK_ENABLE
#define SCSI_IBM_SKIP_MASK_READ         0xE8
#define SCSI_IBM_SKIP_MASK_WRITE        0xEA
#endif

#ifdef IBM_READ10_STATE_SAVE
#define READ10_STATE_SAVE_MASK          0xFFFFFF00
#endif

#define SCSI_INVALID_CDB                0xFF

#define SCSI_READ6_WRITE6_MAX_XFER_LEN  256

/* Service Action definitions op code: 0x7F */
#define SCSI_VAR_CMD_SVC_ACT_READ32          0x09
#define SCSI_VAR_CMD_SVC_ACT_VERIFY32        0x0A
#define SCSI_VAR_CMD_SVC_ACT_WRITE32         0x0B
#define SCSI_VAR_CMD_SVC_ACT_WRITE_VERIFY32  0x0C
#define SCSI_VAR_CMD_SVC_ACT_WRITE_SAME32    0x0D

/* Service Action definitions op code: 0x9E */
#define SCSI_SVC_ACTION16_READ_CAPACITY      0x10
#define SCSI_SVC_ACTION16_READ_LONG          0x11
#define SCSI_SVC_ACTION16_GET_LBA_STATUS     0x12
#define SCSI_SVC_ACTION16_REPORT_REFERRALS   0x13

/* Service Action definitions op code: 0x9F */
#define SCSI_SVC_ACTION16_WRITE_LONG    0x11


/* MAINTENANCE IN op code 0xA3 service actions */
#define SCSI_MAINTENANCE_IN_SERVICE_ACTION_REPORT_IDENTIFYING_INFORMATION             0x05
#define SCSI_MAINTENANCE_IN_SERVICE_ACTION_REPORT_TARGET_PORT_GROUPS                  0x0A
#define SCSI_MAINTENANCE_IN_SERVICE_ACTION_REPORT_ALIASES                             0x0B
#define SCSI_MAINTENANCE_IN_SERVICE_ACTION_REPORT_SUPPORTED_OPERATION_CODES           0x0C
#define SCSI_MAINTENANCE_IN_SERVICE_ACTION_REPORT_SUPPORTED_TASK_MANAGEMENT_FUNCTIONS 0x0D
#define SCSI_MAINTENANCE_IN_SERVICE_ACTION_REPORT_PRIORITY                            0x0E
#define SCSI_MAINTENANCE_IN_SERVICE_ACTION_REPORT_TIMESTAMP                           0x0F
#define SCSI_MAINTENANCE_IN_SERVICE_ACTION_MANAGEMENT_PROTOCOL_IN                     0x10

/* MAINTENANCE OUT op code 0xA4 service actions */
#define SCSI_MAINTENANCE_OUT_SERVICE_ACTION_SET_IDENTIFYING_INFORMATION               0x06
#define SCSI_MAINTENANCE_OUT_SERVICE_ACTION_SET_TARGET_PORT_GROUPS                    0x0A
#define SCSI_MAINTENANCE_OUT_SERVICE_ACTION_CHANGE_ALIASES                            0x0B
#define SCSI_MAINTENANCE_OUT_SERVICE_ACTION_SET_PRIORITY                              0x0E
#define SCSI_MAINTENANCE_OUT_SERVICE_ACTION_SET_TIMESTAMP                             0x0F
#define SCSI_MAINTENANCE_OUT_SERVICE_ACTION_MANAGEMENT_PROTOCOL_OUT                   0x10


/* CDB command/sizes bit positions                                         */
/* ubit16                                                                  */
/* |     CDB SIZE                      |       IO CMD TYPE               | */
/* ======================================================================| */
/* | 15  14  13  12   11   10    9   8 |  7   6   5   4    3   2   1   0 | */
/* | RV  RV  RV  32   16   12   10   6 |  RV  UN  SM  WS  WV   V   W   R | */
/* ======================================================================| */
/* RV=reserved, SM=Skip Mask, WS=Write Same, WV=Write/Verify, V=Verify, W=Write, R=Read */
/* UN = Write Same Unmap                                                                */

#define SCSI_CDB_CMD_READ              (1 << 0)
#define SCSI_CDB_CMD_WRITE             (1 << 1)
#define SCSI_CDB_CMD_VERIFY            (1 << 2)
#define SCSI_CDB_CMD_WRITE_VERIFY      (1 << 3)
#define SCSI_CDB_CMD_WRITE_SAME        (1 << 4)
#define SCSI_CDB_CMD_SKIP_MASK         (1 << 5)
#define SCSI_CDB_CMD_WRITE_SAME_UNMAP  (1 << 6)
#define SCSI_6_BYTE_CDB                (1 << 8)
#define SCSI_10_BYTE_CDB               (1 << 9)
#define SCSI_12_BYTE_CDB               (1 << 10)
#define SCSI_16_BYTE_CDB               (1 << 11)
#define SCSI_32_BYTE_CDB               (1 << 12)

#define SCSI_CDB_READ_WRITE_MASK       (SCSI_CDB_CMD_READ | SCSI_CDB_CMD_WRITE)
#define SCSI_CDB_WRVERIFY_VERIFY_MASK  (SCSI_CDB_CMD_WRITE_VERIFY | SCSI_CDB_CMD_VERIFY)
#define SCSI_CDB_10_16_32_BYTE_MASK    (SCSI_10_BYTE_CDB | SCSI_16_BYTE_CDB | SCSI_32_BYTE_CDB)
#define SCSI_CDB_WRITE_CMDS_MASK       (SCSI_CDB_CMD_WRITE | SCSI_CDB_CMD_WRITE_VERIFY | SCSI_CDB_CMD_WRITE_SAME)




/* control definitions */
#define SCSI_LINK               0x01
#define SCSI_NACA               0x04
#define VS_CONTROL_MASK         0xC0
#define SCSI_CONTROL_BYTE_MASK  0xC7

/* flag definitions */
#define SCSI_DPO                0x10
#define SCSI_FUA                0x08
#define SCSI_CORRCT             0x02
#define SCSI_EBP                0x04
#define SCSI_BLKVFY             0x04
#define SCSI_BYTCHK             0x02
#define SCSI_FUA_NV             0x02

/* EDC definitions */
#define SCSI_PROTECT_SHIFT      5
#define SCSI_PROTECT_MASK       (0x7<<SCSI_PROTECT_SHIFT)
#define SCSI_PROTECT0           0
#define SCSI_PROTECT1           1
#define SCSI_PROTECT2           2
#define SCSI_PROTECT3           3
#define SCSI_PROTECT4           4
#define SCSI_PROTECT5           5

// EMC defines CDB byte 1, bit 7 as the "low priority" bit for Read (10), Write (10),
// Write Verify (10), Verify (10), and EMC Skip Mask.  NOTE: this conflicts with the
// PROTECT fields for most of those commands.
#define EMC_SCSI_LOW_PRIORITY   0x80


// Report Support Opcodes
#define  SCSI_RPT_SUPT_OP_CODES_CMD_TO_DESC_LEN  0x000A
#define  SCSI_RPT_SUPT_OP_CODES_DATA_HDR_LEN     4
#define  SCSI_RPT_SUPT_OP_CODES_OPTION_00        0
#define  SCSI_RPT_SUPT_OP_CODES_OPTION_01        1
#define  SCSI_RPT_SUPT_OP_CODES_OPTION_02        2
#define  SCSI_RPT_SUPT_OP_CODES_NO_INFO          0
#define  SCSI_RPT_SUPT_OP_CODES_NO_SUPPORT       1
#define  SCSI_RPT_SUPT_OP_CODES_STD_SUPPORT      3
#define  SCSI_RPT_SUPT_OP_CODES_VS_SUPPORT       5
#define  SCSI_RPT_SUPT_OP_CODES_TIMEOUT_DESC_LEN 12

/* SCSI command status */
#define SCSI_STATUS_GOOD				0x00
#define SCSI_STATUS_CHECK_CONDITION			0x02
#define SCSI_STATUS_CONDITION_MET			0x04
#define SCSI_STATUS_BUSY				0x08
#define SCSI_STATUS_RESERVATION_CONFLICT		0x18
#define SCSI_STATUS_TASK_SET_FULL			0x28
#define SCSI_STATUS_ACA_ACTIVE				0x30
#define SCSI_STATUS_TASK_ABORTED			0x40


#endif // __SCSI_CMDS_H__
