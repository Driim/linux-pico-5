// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <linux/arm-smccc.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#define REV_A0				0x10
#define REV_B0				0x20
#define REV_B1				0x21

#define IMX8MQ_SW_INFO_A0		0x800
#define IMX8MQ_SW_INFO_B0		0x83C
#define IMX8MQ_SW_INFO_B1		0x40
#define IMX8MQ_SW_MAGIC_B1		0xff0055aa

/* Same as ANADIG_DIGPROG_IMX7D */
#define ANADIG_DIGPROG_IMX8MM	0x800
#define FSL_SIP_GET_SOC_INFO		0xc2000006

struct imx8_soc_data {
	char *name;
	u32 (*soc_revision)(void);
};

static u32 __init imx8mq_soc_revision_atf(void)
{
	struct arm_smccc_res res = { 0 };

	arm_smccc_smc(FSL_SIP_GET_SOC_INFO, 0, 0, 0, 0, 0, 0, 0, &res);
	/*
	 * Bit [23:16] is the silicon ID
	 * Bit[7:4] is the base layer revision,
	 * Bit[3:0] is the metal layer revision
	 * e.g. 0x10 stands for Tapeout 1.0
	 */
	return res.a0 & 0xff;
}

static u32 __init imx8mq_soc_magic_node(const char *node, u32 offset)
{
	struct device_node *np;
	void __iomem *base;
	u32 magic;

	np = of_find_compatible_node(NULL, NULL, node);
	if (!np)
		return 0;
	base = of_iomap(np, 0);
	WARN_ON(!base);

	magic = readl_relaxed(base + offset);
	iounmap(base);
	of_node_put(np);

	return magic;
}

static u32 __init imx8mq_soc_revision(void)
{
	u32 magic;

	/* B1 revision identified by ocotop */
	magic = imx8mq_soc_magic_node("fsl,imx8mq-ocotp", IMX8MQ_SW_INFO_B1);
	if (magic == IMX8MQ_SW_MAGIC_B1)
		return REV_B1;

	/* B0 identified by bootrom */
	magic = imx8mq_soc_magic_node("fsl,imx8mq-bootrom", IMX8MQ_SW_INFO_B0);
	if ((magic & 0xff) == REV_B0)
		return REV_B0;

	/* A0 identified by anatop */
	magic = imx8mq_soc_magic_node("fsl,imx8mq-anatop", IMX8MQ_SW_INFO_A0);
	if ((magic & 0xff) == REV_A0)
		return REV_A0;

	/* Read revision from ATF as fallback */
	magic = imx8mq_soc_revision_atf();
	if (magic != 0xff)
		return magic;

	return 0;
}

static const struct imx8_soc_data imx8mq_soc_data = {
	.name = "i.MX8MQ",
	.soc_revision = imx8mq_soc_revision,
};

static const struct of_device_id imx8_soc_match[] = {
	{ .compatible = "fsl,imx8mq", .data = &imx8mq_soc_data, },
	{ }
};

#define imx8_revision(soc_rev) \
	soc_rev ? \
	kasprintf(GFP_KERNEL, "%d.%d", (soc_rev >> 4) & 0xf,  soc_rev & 0xf) : \
	"unknown"

static int __init imx8_soc_init(void)
{
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;
	struct device_node *root;
	const struct of_device_id *id;
	u32 soc_rev = 0;
	const struct imx8_soc_data *data;
	int ret;

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return -ENODEV;

	soc_dev_attr->family = "Freescale i.MX";

	root = of_find_node_by_path("/");
	ret = of_property_read_string(root, "model", &soc_dev_attr->machine);
	if (ret)
		goto free_soc;

	id = of_match_node(imx8_soc_match, root);
	if (!id)
		goto free_soc;

	of_node_put(root);

	data = id->data;
	if (data) {
		soc_dev_attr->soc_id = data->name;
		if (data->soc_revision)
			soc_rev = data->soc_revision();
	}

	soc_dev_attr->revision = imx8_revision(soc_rev);
	if (!soc_dev_attr->revision)
		goto free_soc;

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev))
		goto free_rev;

	return 0;

free_rev:
	kfree(soc_dev_attr->revision);
free_soc:
	kfree(soc_dev_attr);
	of_node_put(root);
	return -ENODEV;
}
device_initcall(imx8_soc_init);
