// SPDX-License-Identifier: GPL-2.0+
/*
 * i.MX8 NWL MIPI DSI host driver
 *
 * Copyright (C) 2017 NXP
 * Copyright (C) 2019 Purism SPC
 */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/gpio/consumer.h>
#include <linux/irq.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <linux/sys_soc.h>
#include <video/videomode.h>

#include "nwl-drv.h"
#include "nwl-dsi.h"

#define DRV_NAME "imx-nwl-dsi"

/* 8MQ SRC specific registers */
#define SRC_MIPIPHY_RCR	0x28
#define RESET_BYTE_N	BIT(1)
#define RESET_N		BIT(2)
#define DPI_RESET_N	BIT(3)
#define ESC_RESET_N	BIT(4)
#define PCLK_RESET_N	BIT(5)

#define IOMUXC_GPR13	0x34
#define IMX8MQ_GPR13_MIPI_MUX_SEL BIT(2)

/* Possible clocks */
#define CLK_PIXEL	"pixel"
#define CLK_CORE	"core"
#define CLK_BYPASS	"bypass"

enum imx_ext_regs {
	IMX_REG_CSR = BIT(1),
	IMX_REG_SRC = BIT(2),
	IMX_REG_GPR = BIT(3),
};

static const struct regmap_config nwl_dsi_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = IRQ_MASK2,
	.name = DRV_NAME,
};

struct imx_nwl_platform_data {
	int (*poweron)(struct imx_nwl_dsi *dsi);
	int (*poweroff)(struct imx_nwl_dsi *dsi);
	u32 ext_regs; /* required external registers */
	struct imx_nwl_clk_config clk_config[NWL_MAX_PLATFORM_CLOCKS];
};


static inline struct imx_nwl_dsi *encoder_to_dsi(struct drm_encoder *encoder)
{
	return container_of(encoder, struct imx_nwl_dsi, encoder);
}

static inline struct imx_nwl_dsi *connector_to_dsi(struct drm_connector *con)
{
	return container_of(con, struct imx_nwl_dsi, connector);
}

static void imx_nwl_dsi_set_clocks(struct imx_nwl_dsi *dsi, bool enable)
{
	struct device *dev = dsi->dev;
	const char *id;
	struct clk *clk;
	unsigned long new_rate, cur_rate;
	bool enabled;
	size_t i;
	int ret;

	DRM_DEV_DEBUG_DRIVER(dev, "%sabling platform clocks",
			     enable ? "en" : "dis");
	for (i = 0; i < ARRAY_SIZE(dsi->pdata->clk_config); i++) {
		if (!dsi->clk_config[i].present)
			continue;
		id = dsi->clk_config[i].id;
		clk = dsi->clk_config[i].clk;
		new_rate = dsi->clk_config[i].rate;
		cur_rate = clk_get_rate(clk);
		enabled = dsi->clk_config[i].enabled;

		/* BYPASS clk must have the same rate as PHY_REF clk */
		if (!strcmp(id, CLK_BYPASS))
			new_rate = clk_get_rate(dsi->phy_ref_clk);

		if (enable) {
			if (enabled && new_rate != cur_rate)
				clk_disable_unprepare(clk);
			else if (enabled && new_rate == cur_rate)
				continue;
			if (new_rate > 0)
				clk_set_rate(clk, new_rate);
			ret = clk_prepare_enable(clk);
			if (ret < 0) {
				DRM_DEV_ERROR(dev, "Failed to enable clock %s",
					      id);
			}
			dsi->clk_config[i].enabled = true;
			cur_rate = clk_get_rate(clk);
			DRM_DEV_DEBUG_DRIVER(dev,
				"Enabled %s clk (rate: req=%lu act=%lu)\n",
				id, new_rate, cur_rate);
		} else if (enabled) {
			clk_disable_unprepare(clk);
			dsi->clk_config[i].enabled = false;
			DRM_DEV_DEBUG_DRIVER(dev, "Disabled %s clk\n", id);
		}
	}
}

static void imx_nwl_dsi_enable(struct imx_nwl_dsi *dsi)
{
	struct device *dev = dsi->dev;
	int ret;

	imx_nwl_dsi_set_clocks(dsi, true);

	ret = dsi->pdata->poweron(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "Failed to power on DSI (%d)\n", ret);
}

static void imx_nwl_dsi_disable(struct imx_nwl_dsi *dsi)
{
	struct device *dev = dsi->dev;

	if (dsi->dpms_mode != DRM_MODE_DPMS_ON)
		return;

	DRM_DEV_DEBUG_DRIVER(dev, "Disabling encoder");
	dsi->pdata->poweroff(dsi);
	imx_nwl_dsi_set_clocks(dsi, false);
}

static void imx_nwl_set_input_source(struct imx_nwl_dsi *dsi, bool dcss)
{
	u32 mux_val;

	mux_val = dcss ? IMX8MQ_GPR13_MIPI_MUX_SEL : 0;
	DRM_DEV_INFO(dsi->dev, "Using %s as input source\n",
		     (mux_val) ? "DCSS" : "LCDIF");
	regmap_update_bits(dsi->mux_sel,
			   IOMUXC_GPR13,
			   IMX8MQ_GPR13_MIPI_MUX_SEL,
			   mux_val);
}

static void imx_nwl_dsi_encoder_enable(struct drm_encoder *encoder)
{
	struct imx_nwl_dsi *dsi = encoder_to_dsi(encoder);

	if (dsi->dpms_mode == DRM_MODE_DPMS_ON)
		return;

	/* Use DCSS, for LCDIF we'll act as a bridge (tbd) */
	imx_nwl_set_input_source(dsi, true);

	pm_runtime_get_sync(dsi->dev);
	imx_nwl_dsi_enable(dsi);
	nwl_dsi_enable(dsi);
	dsi->dpms_mode = DRM_MODE_DPMS_ON;
}

static void imx_nwl_dsi_encoder_disable(struct drm_encoder *encoder)
{
	struct imx_nwl_dsi *dsi = encoder_to_dsi(encoder);

	if (dsi->dpms_mode != DRM_MODE_DPMS_ON)
		return;

	nwl_dsi_disable(dsi);
	imx_nwl_dsi_disable(dsi);
	pm_runtime_put_sync(dsi->dev);
	dsi->dpms_mode = DRM_MODE_DPMS_OFF;
}

static int imx_nwl_dsi_encoder_atomic_check(struct drm_encoder *encoder,
					struct drm_crtc_state *crtc_state,
					struct drm_connector_state *conn_state)
{
	struct imx_nwl_dsi *dsi = encoder_to_dsi(encoder);
	struct device *dev = dsi->dev;
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	union phy_configure_opts new_cfg;
	unsigned long phy_ref_rate;
	int ret;

	ret = nwl_dsi_get_dphy_params(dsi, mode, &new_cfg);
	if (ret < 0)
		return ret;

	/*
	 * If hs clock is unchanged, we're all good - all parameters are
	 * derived from it atm.
	 */
	if (new_cfg.mipi_dphy.hs_clk_rate == dsi->phy_cfg.mipi_dphy.hs_clk_rate)
		return 0;

	phy_ref_rate = clk_get_rate(dsi->phy_ref_clk);
	DRM_DEV_DEBUG_DRIVER(dev, "PHY at ref rate: %lu\n", phy_ref_rate);
	if (ret < 0) {
		DRM_DEV_ERROR(dsi->dev,
			      "Cannot setup PHY for mode: %ux%u @%d Hz\n",
			      mode->hdisplay, mode->vdisplay, mode->clock);
		DRM_DEV_ERROR(dsi->dev, "PHY ref clk: %lu, bit clk: %lu\n",
			      phy_ref_rate, new_cfg.mipi_dphy.hs_clk_rate);
	} else {
		/* Save the new desired phy config */
		memcpy(&dsi->phy_cfg, &new_cfg, sizeof(new_cfg));
	}
	return ret;
}

static void imx_nwl_dsi_encoder_mode_set(struct drm_encoder *encoder,
					 struct drm_display_mode *mode,
					 struct drm_display_mode *adjusted)
{
	struct imx_nwl_dsi *dsi = encoder_to_dsi(encoder);

	drm_display_mode_to_videomode(adjusted, &dsi->vm);
	drm_mode_debug_printmodeline(adjusted);
}

static const struct drm_encoder_helper_funcs
imx_nwl_dsi_encoder_helper_funcs = {
	.enable = imx_nwl_dsi_encoder_enable,
	.disable = imx_nwl_dsi_encoder_disable,
	.atomic_check = imx_nwl_dsi_encoder_atomic_check,
	.mode_set = imx_nwl_dsi_encoder_mode_set,
};

static const struct drm_encoder_funcs imx_nwl_dsi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int imx_nwl_dsi_connector_get_modes(struct drm_connector *connector)
{
	struct imx_nwl_dsi *dsi = connector_to_dsi(connector);

	return drm_panel_get_modes(dsi->panel);
}

static struct drm_connector_helper_funcs imx_nwl_dsi_connector_helper_funcs = {
	.get_modes = imx_nwl_dsi_connector_get_modes,
};

static void imx_nwl_dsi_drm_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs imx_nwl_dsi_atomic_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = imx_nwl_dsi_drm_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int imx_nwl_mipi_dsi_register(struct drm_device *drm,
				     struct imx_nwl_dsi *dsi)
{
	struct drm_encoder *encoder = &dsi->encoder;
	struct drm_connector *connector = &dsi->connector;
	struct device *dev = dsi->dev;
	int ret;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm,
							     dev->of_node);
	if (encoder->possible_crtcs == 0) {
		DRM_DEV_DEBUG_DRIVER(dev, "No CRTČs found yet\n");
		return -EPROBE_DEFER;
	}

	drm_encoder_helper_add(&dsi->encoder,
			       &imx_nwl_dsi_encoder_helper_funcs);
	ret = drm_encoder_init(drm, &dsi->encoder, &imx_nwl_dsi_encoder_funcs,
			       DRM_MODE_ENCODER_DSI, NULL);
	if (ret)
		return ret;

	drm_connector_helper_add(connector,
				 &imx_nwl_dsi_connector_helper_funcs);

	ret = drm_connector_init(drm, &dsi->connector,
				 &imx_nwl_dsi_atomic_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret)
		return ret;

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret)
		return ret;

	return 0;
}

static int imx_nwl_dsi_parse_dt(struct imx_nwl_dsi *dsi)
{
	struct device_node *np = dsi->dev->of_node;
	struct platform_device *pdev = to_platform_device(dsi->dev);
	struct resource *res;
	struct clk *clk;
	const char *clk_id;
	void __iomem *regs;
	int i, ret;

	dsi->phy = devm_phy_get(dsi->dev, "dphy");
	if (IS_ERR(dsi->phy)) {
		ret = PTR_ERR(dsi->phy);
		dev_err(dsi->dev, "Could not get PHY (%d)\n", ret);
		return ret;
	}

	/* Platform dependent clocks */
	memcpy(dsi->clk_config, dsi->pdata->clk_config,
	       sizeof(dsi->pdata->clk_config));

	for (i = 0; i < ARRAY_SIZE(dsi->pdata->clk_config); i++) {
		if (!dsi->clk_config[i].present)
			continue;

		clk_id = dsi->clk_config[i].id;
		clk = devm_clk_get(dsi->dev, clk_id);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			dev_err(dsi->dev, "Failed to get %s clock (%d)\n",
				clk_id, ret);
			return ret;
		}
		DRM_DEV_DEBUG_DRIVER(dsi->dev, "Setup clk %s (rate: %lu)\n",
				     clk_id, clk_get_rate(clk));
		dsi->clk_config[i].clk = clk;
	}

	/* DSI clocks */
	clk = devm_clk_get(dsi->dev, "phy_ref");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(dsi->dev, "Failed to get phy_ref clock: %d\n", ret);
		return ret;
	}
	dsi->phy_ref_clk = clk;

	clk = devm_clk_get(dsi->dev, "rx_esc");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(dsi->dev, "Failed to get rx_esc clock: %d\n", ret);
		return ret;
	}
	dsi->rx_esc_clk = clk;

	clk = devm_clk_get(dsi->dev, "tx_esc");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(dsi->dev, "Failed to get tx_esc clock: %d\n", ret);
		return ret;
	}
	dsi->tx_esc_clk = clk;

	/* Look for required regmaps */
	dsi->csr = syscon_regmap_lookup_by_phandle(np, "csr");
	if (IS_ERR(dsi->csr) && dsi->pdata->ext_regs & IMX_REG_CSR) {
		ret = PTR_ERR(dsi->csr);
		DRM_DEV_ERROR(dsi->dev, "Failed to get CSR regmap (%d).\n",
			      ret);
		return ret;
	}
	dsi->reset = syscon_regmap_lookup_by_phandle(np, "src");
	if (IS_ERR(dsi->reset) && (dsi->pdata->ext_regs & IMX_REG_SRC)) {
		ret = PTR_ERR(dsi->reset);
		DRM_DEV_ERROR(dsi->dev, "Failed to get SRC regmap (%d).\n",
			      ret);
		return ret;
	}
	dsi->mux_sel = syscon_regmap_lookup_by_phandle(np, "mux-sel");
	if (IS_ERR(dsi->mux_sel) && (dsi->pdata->ext_regs & IMX_REG_GPR)) {
		ret = PTR_ERR(dsi->mux_sel);
		DRM_DEV_ERROR(dsi->dev, "Failed to get GPR regmap (%d).\n",
			      ret);
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(dsi->dev, res);
	if (IS_ERR(regs)) {
		DRM_DEV_ERROR(dsi->dev, "Failed to map NWL DSI registers.\n");
		return PTR_ERR(regs);
	}

	dsi->regs = devm_regmap_init_mmio(dsi->dev, regs,
					  &nwl_dsi_regmap_config);
	if (IS_ERR(dsi->regs)) {
		DRM_DEV_ERROR(dsi->dev, "Failed to create NWL DSI regmap.\n");
		return PTR_ERR(dsi->regs);
	}

	dsi->irq = platform_get_irq(pdev, 0);
	if (dsi->irq < 0) {
		DRM_DEV_ERROR(dsi->dev, "Failed to get device IRQ.\n");
		return -EINVAL;
	}

	return 0;
}

static int imx8mq_dsi_poweron(struct imx_nwl_dsi *dsi)
{
	/* otherwise the display stays blank */
	usleep_range(200, 300);

	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   PCLK_RESET_N, PCLK_RESET_N);
	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   ESC_RESET_N, ESC_RESET_N);
	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   RESET_BYTE_N, RESET_BYTE_N);
	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   DPI_RESET_N, DPI_RESET_N);

	return 0;
}

static int imx8mq_dsi_poweroff(struct imx_nwl_dsi *dsi)
{
	if (USE_SRC_RESET_QUIRK(dsi->quirks))
		return 0;

	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   PCLK_RESET_N, 0);
	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   ESC_RESET_N, 0);
	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   RESET_BYTE_N, 0);
	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   DPI_RESET_N, 0);
	return 0;
}

static struct imx_nwl_platform_data imx8mq_dev = {
	.poweron = &imx8mq_dsi_poweron,
	.poweroff = &imx8mq_dsi_poweroff,
	.clk_config = {
		{ .id = CLK_CORE,   .present = true },
		{ .id = CLK_PIXEL,  .present = false },
		{ .id = CLK_BYPASS, .present = false },
	},
	.ext_regs = IMX_REG_SRC | IMX_REG_GPR,
};

static const struct of_device_id imx_nwl_dsi_dt_ids[] = {
	{ .compatible = "fsl,imx8mq-nwl-dsi", .data = &imx8mq_dev, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_nwl_dsi_dt_ids);

static const struct soc_device_attribute imx_nwl_quirks_match[] = {
	{ .soc_id = "i.MX8MQ", .revision = "2.0",
	  .data = (void *)(E11418_HS_MODE_QUIRK | SRC_RESET_QUIRK) },
	{ /* sentinel. */ },
};

static int imx_nwl_dsi_bind(struct device *dev,	struct device *master,
			    void *data)
{
	struct drm_device *drm = data;
	const struct of_device_id *of_id =
		of_match_device(imx_nwl_dsi_dt_ids, dev);
	const struct imx_nwl_platform_data *pdata = of_id->data;
	const struct soc_device_attribute *attr;
	struct imx_nwl_dsi *dsi;
	int ret;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	dsi->dev = dev;
	dsi->pdata = pdata;
	dsi->dpms_mode = DRM_MODE_DPMS_OFF;

	ret = imx_nwl_dsi_parse_dt(dsi);
	if (ret)
		return ret;

	ret = devm_request_irq(dev, dsi->irq, nwl_dsi_irq_handler, 0,
			       dev_name(dev), dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to request IRQ: %d (%d)\n",
			      dsi->irq, ret);
		return ret;
	}

	ret = imx_nwl_mipi_dsi_register(drm, dsi);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to register MIPI host: %d\n", ret);
		goto err_cleanup;
	}

	dsi->dsi_host.ops = &nwl_dsi_host_ops;
	dsi->dsi_host.dev = dev;
	ret = mipi_dsi_host_register(&dsi->dsi_host);

	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to register MIPI host: %d\n", ret);
		goto err_cleanup;
	}

	attr = soc_device_match(imx_nwl_quirks_match);
	if (attr)
		dsi->quirks = (uintptr_t)attr->data;

	if (!dsi->panel) {
		ret = -EPROBE_DEFER;
		goto err_mipi_dsi_host;
	}

	dev_set_drvdata(dev, dsi);
	pm_runtime_enable(dev);
	return 0;

err_mipi_dsi_host:
	mipi_dsi_host_unregister(&dsi->dsi_host);
err_cleanup:
	devm_free_irq(dev, dsi->irq, dsi);
	dsi->connector.funcs->destroy(&dsi->connector);
	dsi->encoder.funcs->destroy(&dsi->encoder);
	return ret;
}

static void imx_nwl_dsi_unbind(struct device *dev,
			   struct device *master,
			   void *data)
{
	struct imx_nwl_dsi *dsi = dev_get_drvdata(dev);

	if (dsi->dpms_mode == DRM_MODE_DPMS_ON)
		imx_nwl_dsi_encoder_disable(&dsi->encoder);

	if (dsi->encoder.dev)
		drm_encoder_cleanup(&dsi->encoder);
}

static const struct component_ops imx_nwl_dsi_component_ops = {
	.bind	= imx_nwl_dsi_bind,
	.unbind	= imx_nwl_dsi_unbind,
};

static int imx_nwl_dsi_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &imx_nwl_dsi_component_ops);
}

static int imx_nwl_dsi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &imx_nwl_dsi_component_ops);
	return 0;
}

static struct platform_driver imx_nwl_dsi_driver = {
	.probe		= imx_nwl_dsi_probe,
	.remove		= imx_nwl_dsi_remove,
	.driver		= {
		.of_match_table = imx_nwl_dsi_dt_ids,
		.name	= DRV_NAME,
	},
};

module_platform_driver(imx_nwl_dsi_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_AUTHOR("Purism SPC");
MODULE_DESCRIPTION("i.MX8 Northwest Logic MIPI-DSI driver");
MODULE_LICENSE("GPL"); /* GPLv2 or later */
