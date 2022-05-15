/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2012-2019, The Linux Foundation. All rights reserved.
 */

#ifndef _DRIVERS_CRYPTO_MSM_QCRYPTOHW_50_H_
#define _DRIVERS_CRYPTO_MSM_QCRYPTOHW_50_H_


#define CRYPTO_BAM_CNFG_BITS_REG		0x0007C
#define CRYPTO_BAM_CD_ENABLE			27
#define CRYPTO_BAM_CD_ENABLE_MASK		(1 << CRYPTO_BAM_CD_ENABLE)

#define QCE_AUTH_REG_BYTE_COUNT 4
#define CRYPTO_VERSION_REG			0x1A000

#define CRYPTO_DATA_IN0_REG			0x1A010
#define CRYPTO_DATA_IN1_REG			0x1A014
#define CRYPTO_DATA_IN2_REG			0x1A018
#define CRYPTO_DATA_IN3_REG			0x1A01C

#define CRYPTO_DATA_OUT0_REG			0x1A020
#define CRYPTO_DATA_OUT1_REG			0x1A024
#define CRYPTO_DATA_OUT2_REG			0x1A028
#define CRYPTO_DATA_OUT3_REG			0x1A02C

#define CRYPTO_STATUS_REG			0x1A100
#define CRYPTO_STATUS2_REG			0x1A104
#define CRYPTO_ENGINES_AVAIL			0x1A108
#define CRYPTO_FIFO_SIZES_REG			0x1A10C

#define CRYPTO_SEG_SIZE_REG			0x1A110
#define CRYPTO_GOPROC_REG			0x1A120
#define CRYPTO_GOPROC_QC_KEY_REG		0x1B000
#define CRYPTO_GOPROC_OEM_KEY_REG		0x1C000

#define CRYPTO_ENCR_SEG_CFG_REG			0x1A200
#define CRYPTO_ENCR_SEG_SIZE_REG		0x1A204
#define CRYPTO_ENCR_SEG_START_REG		0x1A208

#define CRYPTO_ENCR_KEY0_REG			0x1D000
#define CRYPTO_ENCR_KEY1_REG			0x1D004
#define CRYPTO_ENCR_KEY2_REG			0x1D008
#define CRYPTO_ENCR_KEY3_REG			0x1D00C
#define CRYPTO_ENCR_KEY4_REG			0x1D010
#define CRYPTO_ENCR_KEY5_REG			0x1D014
#define CRYPTO_ENCR_KEY6_REG			0x1D018
#define CRYPTO_ENCR_KEY7_REG			0x1D01C

#define CRYPTO_ENCR_XTS_KEY0_REG		0x1D020
#define CRYPTO_ENCR_XTS_KEY1_REG		0x1D024
#define CRYPTO_ENCR_XTS_KEY2_REG		0x1D028
#define CRYPTO_ENCR_XTS_KEY3_REG		0x1D02C
#define CRYPTO_ENCR_XTS_KEY4_REG		0x1D030
#define CRYPTO_ENCR_XTS_KEY5_REG		0x1D034
#define CRYPTO_ENCR_XTS_KEY6_REG		0x1D038
#define CRYPTO_ENCR_XTS_KEY7_REG		0x1D03C

#define CRYPTO_ENCR_PIPE0_KEY0_REG		0x1E000
#define CRYPTO_ENCR_PIPE0_KEY1_REG		0x1E004
#define CRYPTO_ENCR_PIPE0_KEY2_REG		0x1E008
#define CRYPTO_ENCR_PIPE0_KEY3_REG		0x1E00C
#define CRYPTO_ENCR_PIPE0_KEY4_REG		0x1E010
#define CRYPTO_ENCR_PIPE0_KEY5_REG		0x1E014
#define CRYPTO_ENCR_PIPE0_KEY6_REG		0x1E018
#define CRYPTO_ENCR_PIPE0_KEY7_REG		0x1E01C

#define CRYPTO_ENCR_PIPE1_KEY0_REG		0x1E020
#define CRYPTO_ENCR_PIPE1_KEY1_REG		0x1E024
#define CRYPTO_ENCR_PIPE1_KEY2_REG		0x1E028
#define CRYPTO_ENCR_PIPE1_KEY3_REG		0x1E02C
#define CRYPTO_ENCR_PIPE1_KEY4_REG		0x1E030
#define CRYPTO_ENCR_PIPE1_KEY5_REG		0x1E034
#define CRYPTO_ENCR_PIPE1_KEY6_REG		0x1E038
#define CRYPTO_ENCR_PIPE1_KEY7_REG		0x1E03C

#define CRYPTO_ENCR_PIPE2_KEY0_REG		0x1E040
#define CRYPTO_ENCR_PIPE2_KEY1_REG		0x1E044
#define CRYPTO_ENCR_PIPE2_KEY2_REG		0x1E048
#define CRYPTO_ENCR_PIPE2_KEY3_REG		0x1E04C
#define CRYPTO_ENCR_PIPE2_KEY4_REG		0x1E050
#define CRYPTO_ENCR_PIPE2_KEY5_REG		0x1E054
#define CRYPTO_ENCR_PIPE2_KEY6_REG		0x1E058
#define CRYPTO_ENCR_PIPE2_KEY7_REG		0x1E05C

#define CRYPTO_ENCR_PIPE3_KEY0_REG		0x1E060
#define CRYPTO_ENCR_PIPE3_KEY1_REG		0x1E064
#define CRYPTO_ENCR_PIPE3_KEY2_REG		0x1E068
#define CRYPTO_ENCR_PIPE3_KEY3_REG		0x1E06C
#define CRYPTO_ENCR_PIPE3_KEY4_REG		0x1E070
#define CRYPTO_ENCR_PIPE3_KEY5_REG		0x1E074
#define CRYPTO_ENCR_PIPE3_KEY6_REG		0x1E078
#define CRYPTO_ENCR_PIPE3_KEY7_REG		0x1E07C


#define CRYPTO_ENCR_PIPE0_XTS_KEY0_REG		0x1E200
#define CRYPTO_ENCR_PIPE0_XTS_KEY1_REG		0x1E204
#define CRYPTO_ENCR_PIPE0_XTS_KEY2_REG		0x1E208
#define CRYPTO_ENCR_PIPE0_XTS_KEY3_REG		0x1E20C
#define CRYPTO_ENCR_PIPE0_XTS_KEY4_REG		0x1E210
#define CRYPTO_ENCR_PIPE0_XTS_KEY5_REG		0x1E214
#define CRYPTO_ENCR_PIPE0_XTS_KEY6_REG		0x1E218
#define CRYPTO_ENCR_PIPE0_XTS_KEY7_REG		0x1E21C

#define CRYPTO_ENCR_PIPE1_XTS_KEY0_REG		0x1E220
#define CRYPTO_ENCR_PIPE1_XTS_KEY1_REG		0x1E224
#define CRYPTO_ENCR_PIPE1_XTS_KEY2_REG		0x1E228
#define CRYPTO_ENCR_PIPE1_XTS_KEY3_REG		0x1E22C
#define CRYPTO_ENCR_PIPE1_XTS_KEY4_REG		0x1E230
#define CRYPTO_ENCR_PIPE1_XTS_KEY5_REG		0x1E234
#define CRYPTO_ENCR_PIPE1_XTS_KEY6_REG		0x1E238
#define CRYPTO_ENCR_PIPE1_XTS_KEY7_REG		0x1E23C

#define CRYPTO_ENCR_PIPE2_XTS_KEY0_REG		0x1E240
#define CRYPTO_ENCR_PIPE2_XTS_KEY1_REG		0x1E244
#define CRYPTO_ENCR_PIPE2_XTS_KEY2_REG		0x1E248
#define CRYPTO_ENCR_PIPE2_XTS_KEY3_REG		0x1E24C
#define CRYPTO_ENCR_PIPE2_XTS_KEY4_REG		0x1E250
#define CRYPTO_ENCR_PIPE2_XTS_KEY5_REG		0x1E254
#define CRYPTO_ENCR_PIPE2_XTS_KEY6_REG		0x1E258
#define CRYPTO_ENCR_PIPE2_XTS_KEY7_REG		0x1E25C

#define CRYPTO_ENCR_PIPE3_XTS_KEY0_REG		0x1E260
#define CRYPTO_ENCR_PIPE3_XTS_KEY1_REG		0x1E264
#define CRYPTO_ENCR_PIPE3_XTS_KEY2_REG		0x1E268
#define CRYPTO_ENCR_PIPE3_XTS_KEY3_REG		0x1E26C
#define CRYPTO_ENCR_PIPE3_XTS_KEY4_REG		0x1E270
#define CRYPTO_ENCR_PIPE3_XTS_KEY5_REG		0x1E274
#define CRYPTO_ENCR_PIPE3_XTS_KEY6_REG		0x1E278
#define CRYPTO_ENCR_PIPE3_XTS_KEY7_REG		0x1E27C


#define CRYPTO_CNTR0_IV0_REG			0x1A20C
#define CRYPTO_CNTR1_IV1_REG			0x1A210
#define CRYPTO_CNTR2_IV2_REG			0x1A214
#define CRYPTO_CNTR3_IV3_REG			0x1A218

#define CRYPTO_CNTR_MASK_REG0			0x1A23C
#define CRYPTO_CNTR_MASK_REG1			0x1A238
#define CRYPTO_CNTR_MASK_REG2			0x1A234
#define CRYPTO_CNTR_MASK_REG			0x1A21C

#define CRYPTO_ENCR_CCM_INT_CNTR0_REG		0x1A220
#define CRYPTO_ENCR_CCM_INT_CNTR1_REG		0x1A224
#define CRYPTO_ENCR_CCM_INT_CNTR2_REG		0x1A228
#define CRYPTO_ENCR_CCM_INT_CNTR3_REG		0x1A22C

#define CRYPTO_ENCR_XTS_DU_SIZE_REG		0x1A230

#define CRYPTO_AUTH_SEG_CFG_REG			0x1A300
#define CRYPTO_AUTH_SEG_SIZE_REG		0x1A304
#define CRYPTO_AUTH_SEG_START_REG		0x1A308

#define CRYPTO_AUTH_KEY0_REG			0x1D040
#define CRYPTO_AUTH_KEY1_REG			0x1D044
#define CRYPTO_AUTH_KEY2_REG			0x1D048
#define CRYPTO_AUTH_KEY3_REG			0x1D04C
#define CRYPTO_AUTH_KEY4_REG			0x1D050
#define CRYPTO_AUTH_KEY5_REG			0x1D054
#define CRYPTO_AUTH_KEY6_REG			0x1D058
#define CRYPTO_AUTH_KEY7_REG			0x1D05C
#define CRYPTO_AUTH_KEY8_REG			0x1D060
#define CRYPTO_AUTH_KEY9_REG			0x1D064
#define CRYPTO_AUTH_KEY10_REG			0x1D068
#define CRYPTO_AUTH_KEY11_REG			0x1D06C
#define CRYPTO_AUTH_KEY12_REG			0x1D070
#define CRYPTO_AUTH_KEY13_REG			0x1D074
#define CRYPTO_AUTH_KEY14_REG			0x1D078
#define CRYPTO_AUTH_KEY15_REG			0x1D07C

#define CRYPTO_AUTH_PIPE0_KEY0_REG		0x1E800
#define CRYPTO_AUTH_PIPE0_KEY1_REG		0x1E804
#define CRYPTO_AUTH_PIPE0_KEY2_REG		0x1E808
#define CRYPTO_AUTH_PIPE0_KEY3_REG		0x1E80C
#define CRYPTO_AUTH_PIPE0_KEY4_REG		0x1E810
#define CRYPTO_AUTH_PIPE0_KEY5_REG		0x1E814
#define CRYPTO_AUTH_PIPE0_KEY6_REG		0x1E818
#define CRYPTO_AUTH_PIPE0_KEY7_REG		0x1E81C
#define CRYPTO_AUTH_PIPE0_KEY8_REG		0x1E820
#define CRYPTO_AUTH_PIPE0_KEY9_REG		0x1E824
#define CRYPTO_AUTH_PIPE0_KEY10_REG		0x1E828
#define CRYPTO_AUTH_PIPE0_KEY11_REG		0x1E82C
#define CRYPTO_AUTH_PIPE0_KEY12_REG		0x1E830
#define CRYPTO_AUTH_PIPE0_KEY13_REG		0x1E834
#define CRYPTO_AUTH_PIPE0_KEY14_REG		0x1E838
#define CRYPTO_AUTH_PIPE0_KEY15_REG		0x1E83C

#define CRYPTO_AUTH_PIPE1_KEY0_REG		0x1E880
#define CRYPTO_AUTH_PIPE1_KEY1_REG		0x1E884
#define CRYPTO_AUTH_PIPE1_KEY2_REG		0x1E888
#define CRYPTO_AUTH_PIPE1_KEY3_REG		0x1E88C
#define CRYPTO_AUTH_PIPE1_KEY4_REG		0x1E890
#define CRYPTO_AUTH_PIPE1_KEY5_REG		0x1E894
#define CRYPTO_AUTH_PIPE1_KEY6_REG		0x1E898
#define CRYPTO_AUTH_PIPE1_KEY7_REG		0x1E89C
#define CRYPTO_AUTH_PIPE1_KEY8_REG		0x1E8A0
#define CRYPTO_AUTH_PIPE1_KEY9_REG		0x1E8A4
#define CRYPTO_AUTH_PIPE1_KEY10_REG		0x1E8A8
#define CRYPTO_AUTH_PIPE1_KEY11_REG		0x1E8AC
#define CRYPTO_AUTH_PIPE1_KEY12_REG		0x1E8B0
#define CRYPTO_AUTH_PIPE1_KEY13_REG		0x1E8B4
#define CRYPTO_AUTH_PIPE1_KEY14_REG		0x1E8B8
#define CRYPTO_AUTH_PIPE1_KEY15_REG		0x1E8BC

#define CRYPTO_AUTH_PIPE2_KEY0_REG		0x1E900
#define CRYPTO_AUTH_PIPE2_KEY1_REG		0x1E904
#define CRYPTO_AUTH_PIPE2_KEY2_REG		0x1E908
#define CRYPTO_AUTH_PIPE2_KEY3_REG		0x1E90C
#define CRYPTO_AUTH_PIPE2_KEY4_REG		0x1E910
#define CRYPTO_AUTH_PIPE2_KEY5_REG		0x1E914
#define CRYPTO_AUTH_PIPE2_KEY6_REG		0x1E918
#define CRYPTO_AUTH_PIPE2_KEY7_REG		0x1E91C
#define CRYPTO_AUTH_PIPE2_KEY8_REG		0x1E920
#define CRYPTO_AUTH_PIPE2_KEY9_REG		0x1E924
#define CRYPTO_AUTH_PIPE2_KEY10_REG		0x1E928
#define CRYPTO_AUTH_PIPE2_KEY11_REG		0x1E92C
#define CRYPTO_AUTH_PIPE2_KEY12_REG		0x1E930
#define CRYPTO_AUTH_PIPE2_KEY13_REG		0x1E934
#define CRYPTO_AUTH_PIPE2_KEY14_REG		0x1E938
#define CRYPTO_AUTH_PIPE2_KEY15_REG		0x1E93C

#define CRYPTO_AUTH_PIPE3_KEY0_REG		0x1E980
#define CRYPTO_AUTH_PIPE3_KEY1_REG		0x1E984
#define CRYPTO_AUTH_PIPE3_KEY2_REG		0x1E988
#define CRYPTO_AUTH_PIPE3_KEY3_REG		0x1E98C
#define CRYPTO_AUTH_PIPE3_KEY4_REG		0x1E990
#define CRYPTO_AUTH_PIPE3_KEY5_REG		0x1E994
#define CRYPTO_AUTH_PIPE3_KEY6_REG		0x1E998
#define CRYPTO_AUTH_PIPE3_KEY7_REG		0x1E99C
#define CRYPTO_AUTH_PIPE3_KEY8_REG		0x1E9A0
#define CRYPTO_AUTH_PIPE3_KEY9_REG		0x1E9A4
#define CRYPTO_AUTH_PIPE3_KEY10_REG		0x1E9A8
#define CRYPTO_AUTH_PIPE3_KEY11_REG		0x1E9AC
#define CRYPTO_AUTH_PIPE3_KEY12_REG		0x1E9B0
#define CRYPTO_AUTH_PIPE3_KEY13_REG		0x1E9B4
#define CRYPTO_AUTH_PIPE3_KEY14_REG		0x1E9B8
#define CRYPTO_AUTH_PIPE3_KEY15_REG		0x1E9BC


#define CRYPTO_AUTH_IV0_REG			0x1A310
#define CRYPTO_AUTH_IV1_REG			0x1A314
#define CRYPTO_AUTH_IV2_REG			0x1A318
#define CRYPTO_AUTH_IV3_REG			0x1A31C
#define CRYPTO_AUTH_IV4_REG			0x1A320
#define CRYPTO_AUTH_IV5_REG			0x1A324
#define CRYPTO_AUTH_IV6_REG			0x1A328
#define CRYPTO_AUTH_IV7_REG			0x1A32C
#define CRYPTO_AUTH_IV8_REG			0x1A330
#define CRYPTO_AUTH_IV9_REG			0x1A334
#define CRYPTO_AUTH_IV10_REG			0x1A338
#define CRYPTO_AUTH_IV11_REG			0x1A33C
#define CRYPTO_AUTH_IV12_REG			0x1A340
#define CRYPTO_AUTH_IV13_REG			0x1A344
#define CRYPTO_AUTH_IV14_REG			0x1A348
#define CRYPTO_AUTH_IV15_REG			0x1A34C

#define CRYPTO_AUTH_INFO_NONCE0_REG		0x1A350
#define CRYPTO_AUTH_INFO_NONCE1_REG		0x1A354
#define CRYPTO_AUTH_INFO_NONCE2_REG		0x1A358
#define CRYPTO_AUTH_INFO_NONCE3_REG		0x1A35C

#define CRYPTO_AUTH_BYTECNT0_REG		0x1A390
#define CRYPTO_AUTH_BYTECNT1_REG		0x1A394
#define CRYPTO_AUTH_BYTECNT2_REG		0x1A398
#define CRYPTO_AUTH_BYTECNT3_REG		0x1A39C

#define CRYPTO_AUTH_EXP_MAC0_REG		0x1A3A0
#define CRYPTO_AUTH_EXP_MAC1_REG		0x1A3A4
#define CRYPTO_AUTH_EXP_MAC2_REG		0x1A3A8
#define CRYPTO_AUTH_EXP_MAC3_REG		0x1A3AC
#define CRYPTO_AUTH_EXP_MAC4_REG		0x1A3B0
#define CRYPTO_AUTH_EXP_MAC5_REG		0x1A3B4
#define CRYPTO_AUTH_EXP_MAC6_REG		0x1A3B8
#define CRYPTO_AUTH_EXP_MAC7_REG		0x1A3BC

#define CRYPTO_CONFIG_REG			0x1A400
#define CRYPTO_DEBUG_ENABLE_REG			0x1AF00
#define CRYPTO_DEBUG_REG			0x1AF04



/* Register bits */
#define CRYPTO_CORE_STEP_REV_MASK		0xFFFF
#define CRYPTO_CORE_STEP_REV			0 /* bit 15-0 */
#define CRYPTO_CORE_MAJOR_REV_MASK		0xFF000000
#define CRYPTO_CORE_MAJOR_REV			24 /* bit 31-24 */
#define CRYPTO_CORE_MINOR_REV_MASK		0xFF0000
#define CRYPTO_CORE_MINOR_REV			16 /* bit 23-16 */

/* status reg  */
#define CRYPTO_MAC_FAILED			31
#define CRYPTO_DOUT_SIZE_AVAIL			26 /* bit 30-26 */
#define CRYPTO_DOUT_SIZE_AVAIL_MASK		(0x1F << CRYPTO_DOUT_SIZE_AVAIL)
#define CRYPTO_DIN_SIZE_AVAIL			21 /* bit 21-25 */
#define CRYPTO_DIN_SIZE_AVAIL_MASK		(0x1F << CRYPTO_DIN_SIZE_AVAIL)
#define CRYPTO_HSD_ERR				20
#define CRYPTO_ACCESS_VIOL			19
#define CRYPTO_PIPE_ACTIVE_ERR			18
#define CRYPTO_CFG_CHNG_ERR			17
#define CRYPTO_DOUT_ERR				16
#define CRYPTO_DIN_ERR				15
#define CRYPTO_AXI_ERR				14
#define CRYPTO_CRYPTO_STATE			10 /* bit 13-10 */
#define CRYPTO_CRYPTO_STATE_MASK		(0xF << CRYPTO_CRYPTO_STATE)
#define CRYPTO_ENCR_BUSY			9
#define CRYPTO_AUTH_BUSY			8
#define CRYPTO_DOUT_INTR			7
#define CRYPTO_DIN_INTR				6
#define CRYPTO_OP_DONE_INTR			5
#define CRYPTO_ERR_INTR				4
#define CRYPTO_DOUT_RDY				3
#define CRYPTO_DIN_RDY				2
#define CRYPTO_OPERATION_DONE			1
#define CRYPTO_SW_ERR				0

/* status2 reg  */
#define CRYPTO_AXI_EXTRA			1
#define CRYPTO_LOCKED				2

/* config reg */
#define CRYPTO_REQ_SIZE				17 /* bit 20-17 */
#define CRYPTO_REQ_SIZE_MASK			(0xF << CRYPTO_REQ_SIZE)
#define CRYPTO_REQ_SIZE_ENUM_1_BEAT	0
#define CRYPTO_REQ_SIZE_ENUM_2_BEAT	1
#define CRYPTO_REQ_SIZE_ENUM_3_BEAT	2
#define CRYPTO_REQ_SIZE_ENUM_4_BEAT	3
#define CRYPTO_REQ_SIZE_ENUM_5_BEAT	4
#define CRYPTO_REQ_SIZE_ENUM_6_BEAT	5
#define CRYPTO_REQ_SIZE_ENUM_7_BEAT	6
#define CRYPTO_REQ_SIZE_ENUM_8_BEAT	7
#define CRYPTO_REQ_SIZE_ENUM_9_BEAT	8
#define CRYPTO_REQ_SIZE_ENUM_10_BEAT	9
#define CRYPTO_REQ_SIZE_ENUM_11_BEAT	10
#define CRYPTO_REQ_SIZE_ENUM_12_BEAT	11
#define CRYPTO_REQ_SIZE_ENUM_13_BEAT	12
#define CRYPTO_REQ_SIZE_ENUM_14_BEAT	13
#define CRYPTO_REQ_SIZE_ENUM_15_BEAT	14
#define CRYPTO_REQ_SIZE_ENUM_16_BEAT	15

#define CRYPTO_MAX_QUEUED_REQ			14 /* bit 16-14 */
#define CRYPTO_MAX_QUEUED_REQ_MASK		(0x7 << CRYPTO_MAX_QUEUED_REQ)
#define CRYPTO_ENUM_1_QUEUED_REQS	0
#define CRYPTO_ENUM_2_QUEUED_REQS	1
#define CRYPTO_ENUM_3_QUEUED_REQS	2

#define CRYPTO_IRQ_ENABLES			10	/* bit 13-10 */
#define CRYPTO_IRQ_ENABLES_MASK			(0xF << CRYPTO_IRQ_ENABLES)

#define CRYPTO_LITTLE_ENDIAN_MODE		9
#define CRYPTO_LITTLE_ENDIAN_MASK		(1 << CRYPTO_LITTLE_ENDIAN_MODE)
#define CRYPTO_PIPE_SET_SELECT			5 /* bit 8-5 */
#define CRYPTO_PIPE_SET_SELECT_MASK		(0xF << CRYPTO_PIPE_SET_SELECT)

#define CRYPTO_HIGH_SPD_EN_N			4

#define CRYPTO_MASK_DOUT_INTR			3
#define CRYPTO_MASK_DIN_INTR			2
#define CRYPTO_MASK_OP_DONE_INTR		1
#define CRYPTO_MASK_ERR_INTR			0

/* auth_seg_cfg reg */
#define CRYPTO_COMP_EXP_MAC			24
#define CRYPTO_COMP_EXP_MAC_DISABLED		0
#define CRYPTO_COMP_EXP_MAC_ENABLED		1

#define CRYPTO_F9_DIRECTION			23
#define CRYPTO_F9_DIRECTION_UPLINK		0
#define CRYPTO_F9_DIRECTION_DOWNLINK		1

#define CRYPTO_AUTH_NONCE_NUM_WORDS		20 /* bit 22-20 */
#define CRYPTO_AUTH_NONCE_NUM_WORDS_MASK \
				(0x7 << CRYPTO_AUTH_NONCE_NUM_WORDS)

#define CRYPTO_USE_PIPE_KEY_AUTH		19
#define CRYPTO_USE_HW_KEY_AUTH			18
#define CRYPTO_FIRST				17
#define CRYPTO_LAST				16

#define CRYPTO_AUTH_POS				14 /* bit 15 .. 14*/
#define CRYPTO_AUTH_POS_MASK			(0x3 << CRYPTO_AUTH_POS)
#define CRYPTO_AUTH_POS_BEFORE			0
#define CRYPTO_AUTH_POS_AFTER			1

#define CRYPTO_AUTH_SIZE			9 /* bits 13 .. 9*/
#define CRYPTO_AUTH_SIZE_MASK			(0x1F << CRYPTO_AUTH_SIZE)
#define CRYPTO_AUTH_SIZE_SHA1		0
#define CRYPTO_AUTH_SIZE_SHA256		1
#define CRYPTO_AUTH_SIZE_ENUM_1_BYTES	0
#define CRYPTO_AUTH_SIZE_ENUM_2_BYTES	1
#define CRYPTO_AUTH_SIZE_ENUM_3_BYTES	2
#define CRYPTO_AUTH_SIZE_ENUM_4_BYTES	3
#define CRYPTO_AUTH_SIZE_ENUM_5_BYTES	4
#define CRYPTO_AUTH_SIZE_ENUM_6_BYTES	5
#define CRYPTO_AUTH_SIZE_ENUM_7_BYTES	6
#define CRYPTO_AUTH_SIZE_ENUM_8_BYTES	7
#define CRYPTO_AUTH_SIZE_ENUM_9_BYTES	8
#define CRYPTO_AUTH_SIZE_ENUM_10_BYTES	9
#define CRYPTO_AUTH_SIZE_ENUM_11_BYTES	10
#define CRYPTO_AUTH_SIZE_ENUM_12_BYTES	11
#define CRYPTO_AUTH_SIZE_ENUM_13_BYTES	12
#define CRYPTO_AUTH_SIZE_ENUM_14_BYTES	13
#define CRYPTO_AUTH_SIZE_ENUM_15_BYTES	14
#define CRYPTO_AUTH_SIZE_ENUM_16_BYTES	15


#define CRYPTO_AUTH_MODE			6 /* bit 8 .. 6*/
#define CRYPTO_AUTH_MODE_MASK			(0x7 << CRYPTO_AUTH_MODE)
#define CRYPTO_AUTH_MODE_HASH	0
#define CRYPTO_AUTH_MODE_HMAC	1
#define CRYPTO_AUTH_MODE_CCM	0
#define CRYPTO_AUTH_MODE_CMAC	1

#define CRYPTO_AUTH_KEY_SIZE			3  /* bit 5 .. 3*/
#define CRYPTO_AUTH_KEY_SIZE_MASK		(0x7 << CRYPTO_AUTH_KEY_SIZE)
#define CRYPTO_AUTH_KEY_SZ_AES128	0
#define CRYPTO_AUTH_KEY_SZ_AES256	2

#define CRYPTO_AUTH_ALG				0 /* bit 2 .. 0*/
#define CRYPTO_AUTH_ALG_MASK			7
#define CRYPTO_AUTH_ALG_NONE	0
#define CRYPTO_AUTH_ALG_SHA	1
#define CRYPTO_AUTH_ALG_AES	2
#define CRYPTO_AUTH_ALG_KASUMI	3
#define CRYPTO_AUTH_ALG_SNOW3G	4
#define CRYPTO_AUTH_ALG_ZUC	5

/* encr_xts_du_size reg */
#define CRYPTO_ENCR_XTS_DU_SIZE			0 /* bit 19-0  */
#define CRYPTO_ENCR_XTS_DU_SIZE_MASK		0xfffff

/* encr_seg_cfg reg */
#define CRYPTO_F8_KEYSTREAM_ENABLE		17/* bit */
#define CRYPTO_F8_KEYSTREAM_DISABLED	0
#define CRYPTO_F8_KEYSTREAM_ENABLED	1

#define CRYPTO_F8_DIRECTION			16 /* bit */
#define CRYPTO_F8_DIRECTION_UPLINK	0
#define CRYPTO_F8_DIRECTION_DOWNLINK	1


#define CRYPTO_USE_PIPE_KEY_ENCR		15 /* bit */
#define CRYPTO_USE_PIPE_KEY_ENCR_ENABLED	1
#define CRYPTO_USE_KEY_REGISTERS		0


#define CRYPTO_USE_HW_KEY_ENCR			14
#define CRYPTO_USE_KEY_REG	0
#define CRYPTO_USE_HW_KEY	1

#define CRYPTO_LAST_CCM				13
#define CRYPTO_LAST_CCM_XFR	1
#define CRYPTO_INTERM_CCM_XFR	0


#define CRYPTO_CNTR_ALG				11 /* bit 12-11 */
#define CRYPTO_CNTR_ALG_MASK			(3 << CRYPTO_CNTR_ALG)
#define CRYPTO_CNTR_ALG_NIST	0

#define CRYPTO_ENCODE				10

#define CRYPTO_ENCR_MODE			6 /* bit 9-6 */
#define CRYPTO_ENCR_MODE_MASK			(0xF << CRYPTO_ENCR_MODE)
/* only valid when AES */
#define CRYPTO_ENCR_MODE_ECB	0
#define CRYPTO_ENCR_MODE_CBC	1
#define CRYPTO_ENCR_MODE_CTR	2
#define CRYPTO_ENCR_MODE_XTS	3
#define CRYPTO_ENCR_MODE_CCM	4

#define CRYPTO_ENCR_KEY_SZ			3 /* bit 5-3 */
#define CRYPTO_ENCR_KEY_SZ_MASK			(7 << CRYPTO_ENCR_KEY_SZ)
#define CRYPTO_ENCR_KEY_SZ_DES		0
#define CRYPTO_ENCR_KEY_SZ_3DES		1
#define CRYPTO_ENCR_KEY_SZ_AES128	0
#define CRYPTO_ENCR_KEY_SZ_AES256	2

#define CRYPTO_ENCR_ALG				0 /* bit 2-0 */
#define CRYPTO_ENCR_ALG_MASK			(7 << CRYPTO_ENCR_ALG)
#define CRYPTO_ENCR_ALG_NONE		0
#define CRYPTO_ENCR_ALG_DES		1
#define CRYPTO_ENCR_ALG_AES		2
#define CRYPTO_ENCR_ALG_KASUMI		4
#define CRYPTO_ENCR_ALG_SNOW_3G		5
#define CRYPTO_ENCR_ALG_ZUC		6

/* goproc reg */
#define CRYPTO_GO				0
#define CRYPTO_CLR_CNTXT			1
#define CRYPTO_RESULTS_DUMP			2

/*  F8 definition of CRYPTO_ENCR_CNTR1_IV1 REG  */
#define CRYPTO_CNTR1_IV1_REG_F8_PKT_CNT		16	/* bit 31 - 16 */
#define CRYPTO_CNTR1_IV1_REG_F8_PKT_CNT_MASK \
		(0xffff << CRYPTO_CNTR1_IV1_REG_F8_PKT_CNT)

#define CRYPTO_CNTR1_IV1_REG_F8_BEARER		0	/* bit 4 - 0 */
#define CRYPTO_CNTR1_IV1_REG_F8_BEARER_MASK \
		(0x1f << CRYPTO_CNTR1_IV1_REG_F8_BEARER)

/* F9 definition of CRYPTO_AUTH_IV4 REG */
#define CRYPTO_AUTH_IV4_REG_F9_VALID_BIS	0	/* bit 2 - 0 */
#define CRYPTO_AUTH_IV4_REG_F9_VALID_BIS_MASK \
		(0x7  << CRYPTO_AUTH_IV4_REG_F9_VALID_BIS)

/* engines_avail */
#define CRYPTO_ENCR_AES_SEL			0
#define CRYPTO_DES_SEL				1
#define CRYPTO_ENCR_SNOW3G_SEL			2
#define CRYPTO_ENCR_KASUMI_SEL			3
#define CRYPTO_SHA_SEL				4
#define CRYPTO_SHA512_SEL			5
#define CRYPTO_AUTH_AES_SEL			6
#define CRYPTO_AUTH_SNOW3G_SEL			7
#define CRYPTO_AUTH_KASUMI_SEL			8
#define CRYPTO_BAM_PIPE_SETS			9	/* bit 12 - 9 */
#define CRYPTO_AXI_WR_BEATS			13	/* bit 18 - 13 */
#define CRYPTO_AXI_RD_BEATS			19	/* bit 24 - 19 */
#define CRYPTO_ENCR_ZUC_SEL			26
#define CRYPTO_AUTH_ZUC_SEL			27
#define CRYPTO_ZUC_ENABLE			28
#endif /* _DRIVERS_CRYPTO_MSM_QCRYPTOHW_50_H_ */
