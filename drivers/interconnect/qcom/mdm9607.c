// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Konrad Dybcio <konrad.dybcio@somainline.org>
 * Copyright (c) 2021, AngeloGioacchino Del Regno
 *                     <angelogioacchino.delregno@somainline.org>
 */

#include <linux/device.h>
#include <linux/interconnect-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <dt-bindings/interconnect/qcom,mdm9607.h>

#include "icc-rpm.h"

enum {
	MDM9607_BIMC_MASTER_APPS_PROC = 1,
	MDM9607_BIMC_MASTER_PCNOC_BIMC_1,
	MDM9607_BIMC_MASTER_TCU_0,
	MDM9607_BIMC_SLAVE_EBI,
	MDM9607_BIMC_SLAVE_BIMC_PCNOC,
	MDM9607_PCNOC_MASTER_QDSS_BAM,
	MDM9607_PCNOC_MASTER_BIMC_PCNOC,
	MDM9607_PCNOC_MASTER_QDSS_ETR,
	MDM9607_PCNOC_MASTER_AUDIO,
	MDM9607_PCNOC_MASTER_QPIC,
	MDM9607_PCNOC_MASTER_HSIC,
	MDM9607_PCNOC_MASTER_BLSP_1,
	MDM9607_PCNOC_MASTER_USB_HS1,
	MDM9607_PCNOC_MASTER_MASTER_CRYPTO,
	MDM9607_PCNOC_MASTER_SDCC_1,
	MDM9607_PCNOC_MASTER_SDCC_2,
	MDM9607_PCNOC_MASTER_XI_USB_HS1,
	MDM9607_PCNOC_MASTER_XI_HSIC,
	MDM9607_PCNOC_MASTER_SGMII,
	MDM9607_PCNOC_MASTER_PCNOC_M_0,
	MDM9607_PCNOC_SLAVE_PCNOC_M_0,
	MDM9607_PCNOC_MASTER_PCNOC_M_1,
	MDM9607_PCNOC_SLAVE_PCNOC_M_1,
	MDM9607_PCNOC_MASTER_QDSS_INT,
	MDM9607_PCNOC_SLAVE_QDSS_INT,
	MDM9607_PCNOC_MASTER_PCNOC_INT_0,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_0,
	MDM9607_PCNOC_MASTER_PCNOC_INT_2,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_2,
	MDM9607_PCNOC_MASTER_PCNOC_INT_3,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_3,
	MDM9607_PCNOC_MASTER_PCNOC_S_0,
	MDM9607_PCNOC_SLAVE_PCNOC_S_0,
	MDM9607_PCNOC_MASTER_PCNOC_S_1,
	MDM9607_PCNOC_SLAVE_PCNOC_S_1,
	MDM9607_PCNOC_MASTER_PCNOC_S_2,
	MDM9607_PCNOC_SLAVE_PCNOC_S_2,
	MDM9607_PCNOC_MASTER_PCNOC_S_3,
	MDM9607_PCNOC_SLAVE_PCNOC_S_3,
	MDM9607_PCNOC_MASTER_PCNOC_S_4,
	MDM9607_PCNOC_SLAVE_PCNOC_S_4,
	MDM9607_PCNOC_MASTER_PCNOC_S_5,
	MDM9607_PCNOC_SLAVE_PCNOC_S_5,
	MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1,
	MDM9607_PCNOC_SLAVE_QDSS_STM,
	MDM9607_PCNOC_SLAVE_CATS_0,
	MDM9607_PCNOC_SLAVE_IMEM,
	MDM9607_PCNOC_SLAVE_TCSR,
	MDM9607_PCNOC_SLAVE_SDCC_1,
	MDM9607_PCNOC_SLAVE_BLSP_1,
	MDM9607_PCNOC_SLAVE_SGMII,
	MDM9607_PCNOC_SLAVE_CRYPTO_0_CFG,
	MDM9607_PCNOC_SLAVE_MESSAGE_RAM,
	MDM9607_PCNOC_SLAVE_PDM,
	MDM9607_PCNOC_SLAVE_PRNG,
	MDM9607_PCNOC_SLAVE_USB2,
	MDM9607_PCNOC_SLAVE_SDCC_2,
	MDM9607_PCNOC_SLAVE_AUDIO,
	MDM9607_PCNOC_SLAVE_HSIC,
	MDM9607_PCNOC_SLAVE_USB_PHY,
	MDM9607_PCNOC_SLAVE_TLMM,
	MDM9607_PCNOC_SLAVE_IMEM_CFG,
	MDM9607_PCNOC_SLAVE_PMIC_ARB,
	MDM9607_PCNOC_SLAVE_TCU,
	MDM9607_PCNOC_SLAVE_QPIC,
};

#define NO_CONNECTION	0

/* TODO: verify qcom,blacklist downstream */

/* BIMC nodes */
static const u16 mas_apps_proc_links[] = {
	MDM9607_BIMC_SLAVE_BIMC_PCNOC,
	MDM9607_BIMC_SLAVE_EBI
};

static struct qcom_icc_node mas_apps_proc = {
	.name = "mas_apps_proc",
	.id = MDM9607_BIMC_MASTER_APPS_PROC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = 0,
	.num_links = ARRAY_SIZE(mas_apps_proc_links),
	.links = mas_apps_proc_links,
};

static const u16 mas_pcnoc_bimc_1_links[] = {
	MDM9607_BIMC_SLAVE_EBI
};

static struct qcom_icc_node mas_pcnoc_bimc_1 = {
	.name = "mas_pcnoc_bimc_1",
	.id = MDM9607_BIMC_MASTER_PCNOC_BIMC_1,
	.buswidth = 8,
	.mas_rpm_id = 139,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_pcnoc_bimc_1_links),
	.links = mas_pcnoc_bimc_1_links,
};

static const u16 mas_tcu_0_links[] = {
	MDM9607_BIMC_SLAVE_EBI
};

static struct qcom_icc_node mas_tcu_0 = {
	.name = "mas_tcu_0",
	.id = MDM9607_BIMC_MASTER_TCU_0,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 2,
	.qos.prio_level = 2,
	.qos.qos_port = 5,
	.num_links = ARRAY_SIZE(mas_tcu_0_links),
	.links = mas_tcu_0_links,
};

static const u16 slv_ebi_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_ebi = {
	.name = "slv_ebi",
	.id = MDM9607_BIMC_SLAVE_EBI,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 0,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_ebi_links),
	.links = slv_ebi_links,
};

static const u16 slv_bimc_pcnoc_links[] = {
	MDM9607_PCNOC_MASTER_BIMC_PCNOC
};

static struct qcom_icc_node slv_bimc_pcnoc = {
	.name = "slv_bimc_pcnoc",
	.id = MDM9607_BIMC_SLAVE_BIMC_PCNOC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 202,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_bimc_pcnoc_links),
	.links = slv_bimc_pcnoc_links,
};

static struct qcom_icc_node *mdm9607_bimc_nodes[] = {
	[MASTER_APPS_PROC] = &mas_apps_proc,
	[MASTER_PCNOC_BIMC_1] = &mas_pcnoc_bimc_1,
	[MASTER_TCU_0] = &mas_tcu_0,
	[SLAVE_EBI] = &slv_ebi,
	[SLAVE_BIMC_PCNOC] = &slv_bimc_pcnoc,
};

static const struct regmap_config mdm9607_bimc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x58000,
	.fast_io	= true,
};

static struct qcom_icc_desc mdm9607_bimc = {
	.nodes = mdm9607_bimc_nodes,
	.num_nodes = ARRAY_SIZE(mdm9607_bimc_nodes),
	.regmap_cfg = &mdm9607_bimc_regmap_config,
};

/* PCNoC nodes */
static const u16 mas_qdss_bam_links[] = {
	MDM9607_PCNOC_MASTER_QDSS_INT,
	MDM9607_PCNOC_SLAVE_QDSS_INT
};

static struct qcom_icc_node mas_qdss_bam = {
	.name = "mas_qdss_bam",
	.id = MDM9607_PCNOC_MASTER_QDSS_BAM,
	.buswidth = 4,
	.mas_rpm_id = 19,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_qdss_bam_links),
	.links = mas_qdss_bam_links,
};

static const u16 mas_bimc_pcnoc_links[] = {
	MDM9607_PCNOC_MASTER_PCNOC_INT_0,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_0,
	MDM9607_PCNOC_MASTER_PCNOC_INT_2,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_2,
	MDM9607_PCNOC_SLAVE_CATS_0
};

static struct qcom_icc_node mas_bimc_pcnoc = {
	.name = "mas_bimc_pcnoc",
	.id = MDM9607_PCNOC_MASTER_BIMC_PCNOC,
	.buswidth = 8,
	.mas_rpm_id = 140,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_bimc_pcnoc_links),
	.links = mas_bimc_pcnoc_links,
};

static const u16 mas_qdss_etr_links[] = {
	MDM9607_PCNOC_MASTER_QDSS_INT,
	MDM9607_PCNOC_SLAVE_QDSS_INT
};

static struct qcom_icc_node mas_qdss_etr = {
	.name = "mas_qdss_etr",
	.id = MDM9607_PCNOC_MASTER_QDSS_ETR,
	.buswidth = 8,
	.mas_rpm_id = 31,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_qdss_etr_links),
	.links = mas_qdss_etr_links,
};

static const u16 mas_audio_links[] = {
	MDM9607_PCNOC_MASTER_PCNOC_M_0,
	MDM9607_PCNOC_SLAVE_PCNOC_M_0
};

static struct qcom_icc_node mas_audio = {
	.name = "mas_audio",
	.id = MDM9607_PCNOC_MASTER_AUDIO,
	.buswidth = 4,
	.mas_rpm_id = 78,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_audio_links),
	.links = mas_audio_links,
};

static const u16 mas_qpic_links[] = {
	MDM9607_PCNOC_MASTER_PCNOC_M_0,
	MDM9607_PCNOC_SLAVE_PCNOC_M_0
};

static struct qcom_icc_node mas_qpic = {
	.name = "mas_qpic",
	.id = MDM9607_PCNOC_MASTER_QPIC,
	.buswidth = 4,
	.mas_rpm_id = 58,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_qpic_links),
	.links = mas_qpic_links,
};

static const u16 mas_hsic_links[] = {
	MDM9607_PCNOC_MASTER_PCNOC_M_0,
	MDM9607_PCNOC_SLAVE_PCNOC_M_0
};

static struct qcom_icc_node mas_hsic = {
	.name = "mas_hsic",
	.id = MDM9607_PCNOC_MASTER_HSIC,
	.buswidth = 4,
	.mas_rpm_id = 40,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_hsic_links),
	.links = mas_hsic_links,
};

static const u16 mas_blsp_1_links[] = {
	MDM9607_PCNOC_MASTER_PCNOC_M_1,
	MDM9607_PCNOC_SLAVE_PCNOC_M_1
};

static struct qcom_icc_node mas_blsp_1 = {
	.name = "mas_blsp_1",
	.id = MDM9607_PCNOC_MASTER_BLSP_1,
	.buswidth = 4,
	.mas_rpm_id = 41,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_blsp_1_links),
	.links = mas_blsp_1_links,
};

static const u16 mas_usb_hs1_links[] = {
	MDM9607_PCNOC_MASTER_PCNOC_M_1,
	MDM9607_PCNOC_SLAVE_PCNOC_M_1
};

static struct qcom_icc_node mas_usb_hs1 = {
	.name = "mas_usb_hs1",
	.id = MDM9607_PCNOC_MASTER_USB_HS1,
	.buswidth = 4,
	.mas_rpm_id = 42,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_usb_hs1_links),
	.links = mas_usb_hs1_links,
};

static const u16 mas_crypto_links[] = {
	MDM9607_PCNOC_MASTER_PCNOC_INT_3,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_3
};

static struct qcom_icc_node mas_crypto = {
	.name = "mas_crypto",
	.id = MDM9607_PCNOC_MASTER_MASTER_CRYPTO,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = NOC_QOS_MODE_FIXED,
	.qos.areq_prio = 2,
	.qos.prio_level = 2,
	.qos.qos_port = 0,
	.num_links = ARRAY_SIZE(mas_crypto_links),
	.links = mas_crypto_links,
};

static const u16 mas_sdcc_1_links[] = {
	MDM9607_PCNOC_MASTER_PCNOC_INT_3,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_3
};

static struct qcom_icc_node mas_sdcc_1 = {
	.name = "mas_sdcc_1",
	.id = MDM9607_PCNOC_MASTER_SDCC_1,
	.buswidth = 8,
	.mas_rpm_id = 33,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_sdcc_1_links),
	.links = mas_sdcc_1_links,
};

static const u16 mas_sdcc_2_links[] = {
	MDM9607_PCNOC_MASTER_PCNOC_INT_3,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_3
};

static struct qcom_icc_node mas_sdcc_2 = {
	.name = "mas_sdcc_2",
	.id = MDM9607_PCNOC_MASTER_SDCC_2,
	.buswidth = 8,
	.mas_rpm_id = 35,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_sdcc_2_links),
	.links = mas_sdcc_2_links,
};

static const u16 mas_xi_usb_hs1_links[] = {
	MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1,
	MDM9607_PCNOC_MASTER_PCNOC_INT_2,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_2
};

static struct qcom_icc_node mas_xi_usb_hs1 = {
	.name = "mas_xi_usb_hs1",
	.id = MDM9607_PCNOC_MASTER_XI_USB_HS1,
	.buswidth = 8,
	.mas_rpm_id = 138,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_xi_usb_hs1_links),
	.links = mas_xi_usb_hs1_links,
};

static const u16 mas_xi_hsic_links[] = {
	MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1,
	MDM9607_PCNOC_MASTER_PCNOC_INT_2,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_2
};

static struct qcom_icc_node mas_xi_hsic = {
	.name = "mas_xi_hsic",
	.id = MDM9607_PCNOC_MASTER_XI_HSIC,
	.buswidth = 8,
	.mas_rpm_id = 141,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_xi_hsic_links),
	.links = mas_xi_hsic_links,
};

/* TODO: fix QoS */
static const u16 mas_sgmii_links[] = {
	MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1,
	MDM9607_PCNOC_MASTER_PCNOC_INT_2,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_2
};

static struct qcom_icc_node mas_sgmii = {
	.name = "mas_sgmii",
	.id = MDM9607_PCNOC_MASTER_SGMII,
	.buswidth = 8,
	.mas_rpm_id = 142,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_sgmii_links),
	.links = mas_sgmii_links,
};

static const u16 slv_pcnoc_bimc_1_links[] = {
	MDM9607_BIMC_MASTER_PCNOC_BIMC_1
};

static struct qcom_icc_node slv_pcnoc_bimc_1 = {
	.name = "slv_pcnoc_bimc_1",
	.id = MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 203,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_pcnoc_bimc_1_links),
	.links = slv_pcnoc_bimc_1_links,
};

static const u16 slv_qdss_stm_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_qdss_stm = {
	.name = "slv_qdss_stm",
	.id = MDM9607_PCNOC_SLAVE_QDSS_STM,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_qdss_stm_links),
	.links = slv_qdss_stm_links,
};

static const u16 slv_cats_0_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_cats_0 = {
	.name = "slv_cats_0",
	.id = MDM9607_PCNOC_SLAVE_CATS_0,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_cats_0_links),
	.links = slv_cats_0_links,
};

static const u16 slv_imem_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_imem = {
	.name = "slv_imem",
	.id = MDM9607_PCNOC_SLAVE_IMEM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_imem_links),
	.links = slv_imem_links,
};

static const u16 slv_tcsr_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_tcsr = {
	.name = "slv_tcsr",
	.id = MDM9607_PCNOC_SLAVE_TCSR,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 50,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_tcsr_links),
	.links = slv_tcsr_links,
};

static const u16 slv_sdcc_1_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_sdcc_1 = {
	.name = "slv_sdcc_1",
	.id = MDM9607_PCNOC_SLAVE_SDCC_1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 31,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_sdcc_1_links),
	.links = slv_sdcc_1_links,
};

static const u16 slv_blsp_1_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_blsp_1 = {
	.name = "slv_blsp_1",
	.id = MDM9607_PCNOC_SLAVE_BLSP_1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 39,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_blsp_1_links),
	.links = slv_blsp_1_links,
};

static const u16 slv_sgmii_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_sgmii = {
	.name = "slv_sgmii",
	.id = MDM9607_PCNOC_SLAVE_SGMII,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_sgmii_links),
	.links = slv_sgmii_links,
};

static const u16 slv_crypto_0_cfg_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_crypto_0_cfg = {
	.name = "slv_crypto_0_cfg",
	.id = MDM9607_PCNOC_SLAVE_CRYPTO_0_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_crypto_0_cfg_links),
	.links = slv_crypto_0_cfg_links,
};

static const u16 slv_message_ram_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_message_ram = {
	.name = "slv_message_ram",
	.id = MDM9607_PCNOC_SLAVE_MESSAGE_RAM,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 55,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_message_ram_links),
	.links = slv_message_ram_links,
};

static const u16 slv_pdm_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_pdm = {
	.name = "slv_pdm",
	.id = MDM9607_PCNOC_SLAVE_PDM,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 41,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_pdm_links),
	.links = slv_pdm_links,
};

static const u16 slv_prng_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_prng = {
	.name = "slv_prng",
	.id = MDM9607_PCNOC_SLAVE_PRNG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 44,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_prng_links),
	.links = slv_prng_links,
};

static const u16 slv_usb2_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_usb2 = {
	.name = "slv_usb2",
	.id = MDM9607_PCNOC_SLAVE_USB2,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_usb2_links),
	.links = slv_usb2_links,
};

static const u16 slv_sdcc_2_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_sdcc_2 = {
	.name = "slv_sdcc_2",
	.id = MDM9607_PCNOC_SLAVE_SDCC_2,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 33,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_sdcc_2_links),
	.links = slv_sdcc_2_links,
};

static const u16 slv_audio_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_audio = {
	.name = "slv_audio",
	.id = MDM9607_PCNOC_SLAVE_AUDIO,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 105,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_audio_links),
	.links = slv_audio_links,
};

static const u16 slv_hsic_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_hsic = {
	.name = "slv_hsic",
	.id = MDM9607_PCNOC_SLAVE_HSIC,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 38,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_hsic_links),
	.links = slv_hsic_links,
};

static const u16 slv_usb_phy_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_usb_phy = {
	.name = "slv_usb_phy",
	.id = MDM9607_PCNOC_SLAVE_USB_PHY,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_usb_phy_links),
	.links = slv_usb_phy_links,
};

static const u16 slv_tlmm_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_tlmm = {
	.name = "slv_tlmm",
	.id = MDM9607_PCNOC_SLAVE_TLMM,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 51,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_tlmm_links),
	.links = slv_tlmm_links,
};

static const u16 slv_imem_cfg_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_imem_cfg = {
	.name = "slv_imem_cfg",
	.id = MDM9607_PCNOC_SLAVE_IMEM_CFG,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_imem_cfg_links),
	.links = slv_imem_cfg_links,
};

static const u16 slv_pmic_arb_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_pmic_arb = {
	.name = "slv_pmic_arb",
	.id = MDM9607_PCNOC_SLAVE_PMIC_ARB,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 59,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_pmic_arb_links),
	.links = slv_pmic_arb_links,
};

static const u16 slv_tcu_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_tcu = {
	.name = "slv_tcu",
	.id = MDM9607_PCNOC_SLAVE_TCU,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_tcu_links),
	.links = slv_tcu_links,
};

static const u16 slv_qpic_links[] = {
	NO_CONNECTION
};

static struct qcom_icc_node slv_qpic = {
	.name = "slv_qpic",
	.id = MDM9607_PCNOC_SLAVE_QPIC,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 80,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_qpic_links),
	.links = slv_qpic_links,
};

/* Internal nodes */
static const u16 mas_pcnoc_m_0_links[] = {
	MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1,
	MDM9607_PCNOC_MASTER_PCNOC_INT_2,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_2
};

static struct qcom_icc_node mas_pcnoc_m_0 = {
	.name = "mas_pcnoc_m_0",
	.id = MDM9607_PCNOC_MASTER_PCNOC_M_0,
	.buswidth = 4,
	.mas_rpm_id = 87,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_pcnoc_m_0_links),
	.links = mas_pcnoc_m_0_links,
};

static const u16 mas_pcnoc_m_1_links[] = {
	MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1,
	MDM9607_PCNOC_MASTER_PCNOC_INT_2,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_2
};

static struct qcom_icc_node mas_pcnoc_m_1 = {
	.name = "mas_pcnoc_m_1",
	.id = MDM9607_PCNOC_MASTER_PCNOC_M_1,
	.buswidth = 4,
	.mas_rpm_id = 88,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_pcnoc_m_1_links),
	.links = mas_pcnoc_m_1_links,
};

static const u16 mas_qdss_int_links[] = {
	MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1,
	MDM9607_PCNOC_MASTER_PCNOC_INT_0,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_0
};

static struct qcom_icc_node mas_qdss_int = {
	.name = "mas_qdss_int",
	.id = MDM9607_PCNOC_MASTER_QDSS_INT,
	.buswidth = 8,
	.mas_rpm_id = 98,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_qdss_int_links),
	.links = mas_qdss_int_links,
};

static const u16 mas_pcnoc_int_0_links[] = {
	MDM9607_PCNOC_SLAVE_IMEM,
	MDM9607_PCNOC_SLAVE_QDSS_STM
};

static struct qcom_icc_node mas_pcnoc_int_0 = {
	.name = "mas_pcnoc_int_0",
	.id = MDM9607_PCNOC_MASTER_PCNOC_INT_0,
	.buswidth = 8,
	.mas_rpm_id = 85,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_pcnoc_int_0_links),
	.links = mas_pcnoc_int_0_links,
};

static const u16 mas_pcnoc_int_2_links[] = {
	MDM9607_PCNOC_MASTER_PCNOC_S_1,
	MDM9607_PCNOC_SLAVE_PCNOC_S_1,
	MDM9607_PCNOC_MASTER_PCNOC_S_0,
	MDM9607_PCNOC_SLAVE_PCNOC_S_0,
	MDM9607_PCNOC_MASTER_PCNOC_S_4,
	MDM9607_PCNOC_SLAVE_PCNOC_S_4,
	MDM9607_PCNOC_MASTER_PCNOC_S_5,
	MDM9607_PCNOC_SLAVE_PCNOC_S_5,
	MDM9607_PCNOC_MASTER_PCNOC_S_3,
	MDM9607_PCNOC_SLAVE_PCNOC_S_3,
	MDM9607_PCNOC_SLAVE_TCU
};

static struct qcom_icc_node mas_pcnoc_int_2 = {
	.name = "mas_pcnoc_int_2",
	.id = MDM9607_PCNOC_MASTER_PCNOC_INT_2,
	.buswidth = 8,
	.mas_rpm_id = 124,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_pcnoc_int_2_links),
	.links = mas_pcnoc_int_2_links,
};

static const u16 mas_pcnoc_int_3_links[] = {
	MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1,
	MDM9607_PCNOC_MASTER_PCNOC_INT_2,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_2
};

static struct qcom_icc_node mas_pcnoc_int_3 = {
	.name = "mas_pcnoc_int_3",
	.id = MDM9607_PCNOC_MASTER_PCNOC_INT_3,
	.buswidth = 8,
	.mas_rpm_id = 125,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_pcnoc_int_3_links),
	.links = mas_pcnoc_int_3_links,
};

static const u16 mas_pcnoc_s_0_links[] = {
	MDM9607_PCNOC_SLAVE_TCSR,
	MDM9607_PCNOC_SLAVE_SDCC_1,
	MDM9607_PCNOC_SLAVE_BLSP_1,
	MDM9607_PCNOC_SLAVE_SGMII
};

static struct qcom_icc_node mas_pcnoc_s_0 = {
	.name = "mas_pcnoc_s_0",
	.id = MDM9607_PCNOC_MASTER_PCNOC_S_0,
	.buswidth = 4,
	.mas_rpm_id = 89,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_pcnoc_s_0_links),
	.links = mas_pcnoc_s_0_links,
};

static const u16 mas_pcnoc_s_1_links[] = {
	MDM9607_PCNOC_SLAVE_USB2,
	MDM9607_PCNOC_SLAVE_CRYPTO_0_CFG,
	MDM9607_PCNOC_SLAVE_PRNG,
	MDM9607_PCNOC_SLAVE_PDM,
	MDM9607_PCNOC_SLAVE_MESSAGE_RAM
};

static struct qcom_icc_node mas_pcnoc_s_1 = {
	.name = "mas_pcnoc_s_1",
	.id = MDM9607_PCNOC_MASTER_PCNOC_S_1,
	.buswidth = 4,
	.mas_rpm_id = 90,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_pcnoc_s_1_links),
	.links = mas_pcnoc_s_1_links,
};

static const u16 mas_pcnoc_s_2_links[] = {
	MDM9607_PCNOC_SLAVE_HSIC,
	MDM9607_PCNOC_SLAVE_SDCC_2,
	MDM9607_PCNOC_SLAVE_AUDIO
};

static struct qcom_icc_node mas_pcnoc_s_2 = {
	.name = "mas_pcnoc_s_2",
	.id = MDM9607_PCNOC_MASTER_PCNOC_S_2,
	.buswidth = 4,
	.mas_rpm_id = 91,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_pcnoc_s_2_links),
	.links = mas_pcnoc_s_2_links,
};

static const u16 mas_pcnoc_s_3_links[] = {
	MDM9607_PCNOC_SLAVE_USB_PHY
};

static struct qcom_icc_node mas_pcnoc_s_3 = {
	.name = "mas_pcnoc_s_3",
	.id = MDM9607_PCNOC_MASTER_PCNOC_S_3,
	.buswidth = 4,
	.mas_rpm_id = 92,
	.slv_rpm_id = -1,
	.qos.ap_owned = true,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_pcnoc_s_3_links),
	.links = mas_pcnoc_s_3_links,
};

static const u16 mas_pcnoc_s_4_links[] = {
	MDM9607_PCNOC_SLAVE_IMEM_CFG,
	MDM9607_PCNOC_SLAVE_PMIC_ARB
};

static struct qcom_icc_node mas_pcnoc_s_4 = {
	.name = "mas_pcnoc_s_4",
	.id = MDM9607_PCNOC_MASTER_PCNOC_S_4,
	.buswidth = 4,
	.mas_rpm_id = 93,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_pcnoc_s_4_links),
	.links = mas_pcnoc_s_4_links,
};

static const u16 mas_pcnoc_s_5_links[] = {
	MDM9607_PCNOC_SLAVE_TLMM
};

static struct qcom_icc_node mas_pcnoc_s_5 = {
	.name = "mas_pcnoc_s_5",
	.id = MDM9607_PCNOC_MASTER_PCNOC_S_5,
	.buswidth = 4,
	.mas_rpm_id = 129,
	.slv_rpm_id = -1,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(mas_pcnoc_s_5_links),
	.links = mas_pcnoc_s_5_links,
};

static const u16 slv_pcnoc_m_0_links[] = {
	MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1,
	MDM9607_PCNOC_MASTER_PCNOC_INT_2,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_2
};

static struct qcom_icc_node slv_pcnoc_m_0 = {
	.name = "slv_pcnoc_m_0",
	.id = MDM9607_PCNOC_SLAVE_PCNOC_M_0,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 116,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_pcnoc_m_0_links),
	.links = slv_pcnoc_m_0_links,
};

static const u16 slv_pcnoc_m_1_links[] = {
	MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1,
	MDM9607_PCNOC_MASTER_PCNOC_INT_2,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_2
};

static struct qcom_icc_node slv_pcnoc_m_1 = {
	.name = "slv_pcnoc_m_1",
	.id = MDM9607_PCNOC_SLAVE_PCNOC_M_1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 117,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_pcnoc_m_1_links),
	.links = slv_pcnoc_m_1_links,
};

static const u16 slv_qdss_int_links[] = {
	MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1,
	MDM9607_PCNOC_MASTER_PCNOC_INT_0,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_0
};

static struct qcom_icc_node slv_qdss_int = {
	.name = "slv_qdss_int",
	.id = MDM9607_PCNOC_SLAVE_QDSS_INT,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 128,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_qdss_int_links),
	.links = slv_qdss_int_links,
};

static const u16 slv_pcnoc_int_0_links[] = {
	MDM9607_PCNOC_SLAVE_IMEM,
	MDM9607_PCNOC_SLAVE_QDSS_STM
};

static struct qcom_icc_node slv_pcnoc_int_0 = {
	.name = "slv_pcnoc_int_0",
	.id = MDM9607_PCNOC_SLAVE_PCNOC_INT_0,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 114,
	.qos.ap_owned = true,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_pcnoc_int_0_links),
	.links = slv_pcnoc_int_0_links,
};

static const u16 slv_pcnoc_int_2_links[] = {
	MDM9607_PCNOC_MASTER_PCNOC_S_1,
	MDM9607_PCNOC_SLAVE_PCNOC_S_1,
	MDM9607_PCNOC_MASTER_PCNOC_S_0,
	MDM9607_PCNOC_SLAVE_PCNOC_S_0,
	MDM9607_PCNOC_MASTER_PCNOC_S_4,
	MDM9607_PCNOC_SLAVE_PCNOC_S_4,
	MDM9607_PCNOC_MASTER_PCNOC_S_5,
	MDM9607_PCNOC_SLAVE_PCNOC_S_5,
	MDM9607_PCNOC_MASTER_PCNOC_S_3,
	MDM9607_PCNOC_SLAVE_PCNOC_S_3,
	MDM9607_PCNOC_SLAVE_TCU
};

static struct qcom_icc_node slv_pcnoc_int_2 = {
	.name = "slv_pcnoc_int_2",
	.id = MDM9607_PCNOC_SLAVE_PCNOC_INT_2,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 184,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_pcnoc_int_2_links),
	.links = slv_pcnoc_int_2_links,
};

static const u16 slv_pcnoc_int_3_links[] = {
	MDM9607_PCNOC_SLAVE_PCNOC_BIMC_1,
	MDM9607_PCNOC_MASTER_PCNOC_INT_2,
	MDM9607_PCNOC_SLAVE_PCNOC_INT_2
};

static struct qcom_icc_node slv_pcnoc_int_3 = {
	.name = "slv_pcnoc_int_3",
	.id = MDM9607_PCNOC_SLAVE_PCNOC_INT_3,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 185,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_pcnoc_int_3_links),
	.links = slv_pcnoc_int_3_links,
};

static const u16 slv_pcnoc_s_0_links[] = {
	MDM9607_PCNOC_SLAVE_TCSR,
	MDM9607_PCNOC_SLAVE_SDCC_1,
	MDM9607_PCNOC_SLAVE_BLSP_1,
	MDM9607_PCNOC_SLAVE_SGMII
};

static struct qcom_icc_node slv_pcnoc_s_0 = {
	.name = "slv_pcnoc_s_0",
	.id = MDM9607_PCNOC_SLAVE_PCNOC_S_0,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 118,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_pcnoc_s_0_links),
	.links = slv_pcnoc_s_0_links,
};

static const u16 slv_pcnoc_s_1_links[] = {
	MDM9607_PCNOC_SLAVE_USB2,
	MDM9607_PCNOC_SLAVE_CRYPTO_0_CFG,
	MDM9607_PCNOC_SLAVE_PRNG,
	MDM9607_PCNOC_SLAVE_PDM,
	MDM9607_PCNOC_SLAVE_MESSAGE_RAM
};

static struct qcom_icc_node slv_pcnoc_s_1 = {
	.name = "slv_pcnoc_s_1",
	.id = MDM9607_PCNOC_SLAVE_PCNOC_S_1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 119,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_pcnoc_s_1_links),
	.links = slv_pcnoc_s_1_links,
};

static const u16 slv_pcnoc_s_2_links[] = {
	MDM9607_PCNOC_SLAVE_HSIC,
	MDM9607_PCNOC_SLAVE_SDCC_2,
	MDM9607_PCNOC_SLAVE_AUDIO
};

static struct qcom_icc_node slv_pcnoc_s_2 = {
	.name = "slv_pcnoc_s_2",
	.id = MDM9607_PCNOC_SLAVE_PCNOC_S_2,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 120,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_pcnoc_s_2_links),
	.links = slv_pcnoc_s_2_links,
};

static const u16 slv_pcnoc_s_3_links[] = {
	MDM9607_PCNOC_SLAVE_USB_PHY
};

static struct qcom_icc_node slv_pcnoc_s_3 = {
	.name = "slv_pcnoc_s_3",
	.id = MDM9607_PCNOC_SLAVE_PCNOC_S_3,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 121,
	.qos.ap_owned = true,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_pcnoc_s_3_links),
	.links = slv_pcnoc_s_3_links,
};

static const u16 slv_pcnoc_s_4_links[] = {
	MDM9607_PCNOC_SLAVE_IMEM_CFG,
	MDM9607_PCNOC_SLAVE_PMIC_ARB
};

static struct qcom_icc_node slv_pcnoc_s_4 = {
	.name = "slv_pcnoc_s_4",
	.id = MDM9607_PCNOC_SLAVE_PCNOC_S_4,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 122,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_pcnoc_s_4_links),
	.links = slv_pcnoc_s_4_links,
};

static const u16 slv_pcnoc_s_5_links[] = {
	MDM9607_PCNOC_SLAVE_TLMM
};

static struct qcom_icc_node slv_pcnoc_s_5 = {
	.name = "slv_pcnoc_s_5",
	.id = MDM9607_PCNOC_SLAVE_PCNOC_S_5,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = 189,
	.qos.ap_owned = false,
	.qos.qos_mode = -1,
	.qos.areq_prio = 0,
	.qos.prio_level = 0,
	.qos.qos_port = -1,
	.num_links = ARRAY_SIZE(slv_pcnoc_s_5_links),
	.links = slv_pcnoc_s_5_links,
};

static struct qcom_icc_node *mdm9607_pcnoc_nodes[] = {
	[MASTER_QDSS_BAM] = &mas_qdss_bam,
	[MASTER_BIMC_PCNOC] = &mas_bimc_pcnoc,
	[MASTER_QDSS_ETR] = &mas_qdss_etr,
	[MASTER_AUDIO] = &mas_audio,
	[MASTER_QPIC] = &mas_qpic,
	[MASTER_HSIC] = &mas_hsic,
	[MASTER_BLSP_1] = &mas_blsp_1,
	[MASTER_USB_HS1] = &mas_usb_hs1,
	[MASTER_CRYPTO] = &mas_crypto,
	[MASTER_SDCC_1] = &mas_sdcc_1,
	[MASTER_SDCC_2] = &mas_sdcc_2,
	[MASTER_XI_USB_HS1] = &mas_xi_usb_hs1,
	[MASTER_XI_HSIC] = &mas_xi_hsic,
	[MASTER_SGMII] = &mas_sgmii,
	[SLAVE_PCNOC_BIMC_1] = &slv_pcnoc_bimc_1,
	[SLAVE_QDSS_STM] = &slv_qdss_stm,
	[SLAVE_CATS_0] = &slv_cats_0,
	[SLAVE_IMEM] = &slv_imem,
	[SLAVE_TCSR] = &slv_tcsr,
	[SLAVE_SDCC_1] = &slv_sdcc_1,
	[SLAVE_BLSP_1] = &slv_blsp_1,
	[SLAVE_SGMII] = &slv_sgmii,
	[SLAVE_CRYPTO_0_CFG] = &slv_crypto_0_cfg,
	[SLAVE_MESSAGE_RAM] = &slv_message_ram,
	[SLAVE_PDM] = &slv_pdm,
	[SLAVE_PRNG] = &slv_prng,
	[SLAVE_USB2] = &slv_usb2,
	[SLAVE_SDCC_2] = &slv_sdcc_2,
	[SLAVE_AUDIO] = &slv_audio,
	[SLAVE_HSIC] = &slv_hsic,
	[SLAVE_USB_PHY] = &slv_usb_phy,
	[SLAVE_TLMM] = &slv_tlmm,
	[SLAVE_IMEM_CFG] = &slv_imem_cfg,
	[SLAVE_PMIC_ARB] = &slv_pmic_arb,
	[SLAVE_TCU] = &slv_tcu,
	[SLAVE_QPIC] = &slv_qpic,

	/* Internal nodes */
	[MASTER_PCNOC_M_0] = &mas_pcnoc_m_0,
	[MASTER_PCNOC_M_1] = &mas_pcnoc_m_1,
	[MASTER_QDSS_INT] = &mas_qdss_int,
	[MASTER_PCNOC_INT_0] = &mas_pcnoc_int_0,
	[MASTER_PCNOC_INT_2] = &mas_pcnoc_int_2,
	[MASTER_PCNOC_INT_3] = &mas_pcnoc_int_3,
	[MASTER_PCNOC_S_0] = &mas_pcnoc_s_0,
	[MASTER_PCNOC_S_1] = &mas_pcnoc_s_1,
	[MASTER_PCNOC_S_2] = &mas_pcnoc_s_2,
	[MASTER_PCNOC_S_3] = &mas_pcnoc_s_3,
	[MASTER_PCNOC_S_4] = &mas_pcnoc_s_4,
	[MASTER_PCNOC_S_5] = &mas_pcnoc_s_5,

	[SLAVE_PCNOC_M_0] = &slv_pcnoc_m_0,
	[SLAVE_PCNOC_M_1] = &slv_pcnoc_m_1,
	[SLAVE_QDSS_INT] = &slv_qdss_int,
	[SLAVE_PCNOC_INT_0] = &slv_pcnoc_int_0,
	[SLAVE_PCNOC_INT_2] = &slv_pcnoc_int_2,
	[SLAVE_PCNOC_INT_3] = &slv_pcnoc_int_3,
	[SLAVE_PCNOC_S_0] = &slv_pcnoc_s_0,
	[SLAVE_PCNOC_S_1] = &slv_pcnoc_s_1,
	[SLAVE_PCNOC_S_2] = &slv_pcnoc_s_2,
	[SLAVE_PCNOC_S_3] = &slv_pcnoc_s_3,
	[SLAVE_PCNOC_S_4] = &slv_pcnoc_s_4,
	[SLAVE_PCNOC_S_5] = &slv_pcnoc_s_5,
};

static const struct regmap_config mdm9607_pcnoc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x15080,
	.fast_io	= true,
};

static struct qcom_icc_desc mdm9607_pcnoc = {
	.nodes = mdm9607_pcnoc_nodes,
	.num_nodes = ARRAY_SIZE(mdm9607_pcnoc_nodes),
	.regmap_cfg = &mdm9607_pcnoc_regmap_config,
};

static const struct of_device_id mdm9607_noc_of_match[] = {
	{ .compatible = "qcom,mdm9607-bimc", .data = &mdm9607_bimc },
	{ .compatible = "qcom,mdm9607-pcnoc", .data = &mdm9607_pcnoc },
	{ }
};
MODULE_DEVICE_TABLE(of, mdm9607_noc_of_match);

static struct platform_driver mdm9607_noc_driver = {
	.probe = qnoc_probe,
	.remove = qnoc_remove,
	.driver = {
		.name = "qnoc-mdm9607",
		.of_match_table = mdm9607_noc_of_match,
	},
};
module_platform_driver(mdm9607_noc_driver);
MODULE_AUTHOR("Konrad Dybcio <konrad.dybcio@somainline.org>");
MODULE_DESCRIPTION("Qualcomm MDM9607 NoC driver");
MODULE_LICENSE("GPL v2");
