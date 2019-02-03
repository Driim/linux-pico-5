// SPDX-License-Identifier: GPL-2.0+
/*
 * i.MX8 NWL MIPI DSI host driver
 *
 * Copyright (C) 2017 NXP
 * Copyright (C) 2019 Purism SPC
 */

#ifndef __NWL_DRV_H__
#define __NWL_DRV_H__

#include <linux/phy/phy.h>
#include <drm/drm_mipi_dsi.h>

struct imx_nwl_platform_data;

/* imx nwl quirks */
/* i.MX8MQ errata E11418 */
#define E11418_HS_MODE_QUIRK BIT(0)
/* Skip DSI bits in SRC on disable to avoid blank display on enable */
#define SRC_RESET_QUIRK BIT(1)

#define USE_E11418_HS_MODE_QUIRK(x) ((x) & E11418_HS_MODE_QUIRK)
#define USE_SRC_RESET_QUIRK(x) ((x) & SRC_RESET_QUIRK)

#define NWL_MAX_PLATFORM_CLOCKS 3
struct imx_nwl_clk_config {
	const char *id;
	struct clk *clk;
	bool present;
	bool enabled;
	u32 rate;
};

struct imx_nwl_dsi {
	struct drm_encoder encoder;
	struct drm_connector connector;
	struct mipi_dsi_host dsi_host;
	struct drm_panel *panel;
	struct device *dev;
	struct phy *phy;
	union phy_configure_opts phy_cfg;
	unsigned int quirks;

	struct regmap *regs;
	int irq;

	/* External registers */
	struct regmap *csr;
	struct regmap *mux_sel;
	struct regmap *reset;

	/* Platform dependent clocks */
	struct imx_nwl_clk_config clk_config[3];
	/* DSI clocks */
	struct clk *phy_ref_clk;
	struct clk *rx_esc_clk;
	struct clk *tx_esc_clk;

	/* dsi lanes */
	u32 lanes;
	enum mipi_dsi_pixel_format format;
	struct videomode vm;
	unsigned long dsi_mode_flags;

	int dpms_mode;

	struct mipi_dsi_transfer *xfer;

	const struct imx_nwl_platform_data *pdata;
};

#endif /* __NWL_DRV_H__ */
