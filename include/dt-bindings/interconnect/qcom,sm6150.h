/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __DT_BINDINGS_INTERCONNECT_QCOM_SM6150_H
#define __DT_BINDINGS_INTERCONNECT_QCOM_SM6150_H

#define MASTER_APPSS_PROC				0
#define MASTER_GPU_TCU				1
#define MASTER_SYS_TCU				2
#define MASTER_IPA_CORE				3
#define MASTER_LLCC				4
#define MASTER_A1NOC_CFG				5
#define MASTER_CNOC_DC_NOC				6
#define MASTER_GEM_NOC_CFG				7
#define MASTER_CNOC_MNOC_CFG				8
#define MASTER_QDSS_BAM				9
#define MASTER_QSPI				10
#define MASTER_QUP_0				11
#define MASTER_BLSP_1				12
#define MASTER_SNOC_CFG				13
#define MASTER_SPDM				14
#define MASTER_A1NOC_SNOC				15
#define MASTER_CNOC_A2NOC				16
#define MASTER_GEM_NOC_SNOC				17
#define MASTER_GEM_NOC_PCIE_SNOC				18
#define MASTER_GFX3D				19
#define MASTER_LPASS_ANOC				20
#define MASTER_MNOC_HF_MEM_NOC				21
#define MASTER_MNOC_SF_MEM_NOC				22
#define MASTER_ANOC_PCIE_SNOC				23
#define MASTER_SNOC_CNOC				24
#define MASTER_SNOC_GC_MEM_NOC				25
#define MASTER_SNOC_SF_MEM_NOC				26
#define MASTER_CAMNOC_HF0				27
#define MASTER_CAMNOC_HF0_UNCOMP				28
#define MASTER_CAMNOC_HF1				29
#define MASTER_CAMNOC_HF1_UNCOMP				30
#define MASTER_CAMNOC_SF				31
#define MASTER_CAMNOC_SF_UNCOMP				32
#define MASTER_CRYPTO				33
#define MASTER_IPA				34
#define MASTER_MDP0				35
#define MASTER_PIMEM				36
#define MASTER_ROTATOR				37
#define MASTER_VIDEO_P0				38
#define MASTER_VIDEO_PROC				39
#define MASTER_EMAC_EVB				40
#define MASTER_GIC				41
#define MASTER_PCIE				42
#define MASTER_QDSS_DAP				43
#define MASTER_QDSS_ETR				44
#define MASTER_SDCC_1				45
#define MASTER_SDCC_2				46
#define MASTER_UFS_MEM				47
#define MASTER_USB2				48
#define MASTER_USB3_0				49
#define SLAVE_EBI1				512
#define SLAVE_IPA_CORE				513
#define SLAVE_A1NOC_CFG				514
#define SLAVE_AHB2PHY_EAST				515
#define SLAVE_AHB2PHY_WEST				516
#define SLAVE_AOP				517
#define SLAVE_AOSS				518
#define SLAVE_APPSS				519
#define SLAVE_CAMERA_CFG				520
#define SLAVE_CLK_CTL				521
#define SLAVE_RBCPR_CX_CFG				522
#define SLAVE_RBCPR_MX_CFG				523
#define SLAVE_CRYPTO_0_CFG				524
#define SLAVE_DC_NOC_GEMNOC				525
#define SLAVE_CNOC_DDRSS				526
#define SLAVE_DISPLAY_CFG				527
#define SLAVE_EMAC_AVB_CFG				528
#define SLAVE_GLM				529
#define SLAVE_GFX3D_CFG				530
#define SLAVE_IMEM_CFG				531
#define SLAVE_IPA_CFG				532
#define SLAVE_LLCC_CFG				533
#define SLAVE_MSS_PROC_MS_MPU_CFG				534
#define SLAVE_CNOC_MNOC_CFG				535
#define SLAVE_PCIE_CFG				536
#define SLAVE_PIMEM_CFG				537
#define SLAVE_PRNG				538
#define SLAVE_QDSS_CFG				539
#define SLAVE_QSPI				540
#define SLAVE_QUP_0				541
#define SLAVE_QUP_1				542
#define SLAVE_SDCC_1				543
#define SLAVE_SDCC_2				544
#define SLAVE_SNOC_CFG				545
#define SLAVE_SPDM_WRAPPER				546
#define SLAVE_TCSR				547
#define SLAVE_TLMM_EAST				548
#define SLAVE_TLMM_SOUTH				549
#define SLAVE_TLMM_WEST				550
#define SLAVE_UFS_MEM_CFG				551
#define SLAVE_USB2				552
#define SLAVE_USB3				553
#define SLAVE_VENUS_CFG				554
#define SLAVE_VSENSE_CTRL_CFG				555
#define SLAVE_MNOC_SF_MEM_NOC				556
#define SLAVE_A1NOC_SNOC				557
#define SLAVE_CAMNOC_UNCOMP				558
#define SLAVE_SNOC_CNOC				559
#define SLAVE_CNOC_A2NOC				560
#define SLAVE_GEM_NOC_SNOC				561
#define SLAVE_SNOC_GEM_NOC_SF				562
#define SLAVE_LLCC				563
#define SLAVE_LPASS_SNOC				564
#define SLAVE_MNOC_HF_MEM_NOC				565
#define SLAVE_SNOC_MEM_NOC_GC				566
#define SLAVE_ANOC_PCIE_SNOC				567
#define SLAVE_MEM_NOC_PCIE_SNOC				568
#define SLAVE_IMEM				569
#define SLAVE_PIMEM				570
#define SLAVE_SERVICE_A2NOC				571
#define SLAVE_SERVICE_CNOC				572
#define SLAVE_SERVICE_GEM_NOC				573
#define SLAVE_SERVICE_MNOC				574
#define SLAVE_SERVICE_SNOC				575
#define SLAVE_PCIE_0				576
#define SLAVE_QDSS_STM				577
#define SLAVE_TCU				578
#define MASTER_LLCC_DISP				1000
#define MASTER_MNOC_HF_MEM_NOC_DISP				1001
#define MASTER_MNOC_SF_MEM_NOC_DISP				1002
#define MASTER_MDP0_DISP				1003
#define MASTER_ROTATOR_DISP				1004
#define SLAVE_EBI1_DISP				1512
#define SLAVE_MNOC_SF_MEM_NOC_DISP				1513
#define SLAVE_LLCC_DISP				1514
#define SLAVE_MNOC_HF_MEM_NOC_DISP				1515

#endif
