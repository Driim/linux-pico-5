// SPDX-License-Identifier: GPL-2.0+
/*
 * NWL DSI host driver
 *
 * Copyright (C) 2017 NXP
 * Copyright (C) 2019 Purism SPC
 */

#include <asm/unaligned.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/regmap.h>
#include <video/mipi_display.h>
#include <video/videomode.h>

#include "nwl-drv.h"
#include "nwl-dsi.h"

#define MIPI_FIFO_TIMEOUT msecs_to_jiffies(500)

/* PKT reg bit manipulation */
#define REG_MASK(e, s) (((1 << ((e) - (s) + 1)) - 1) << (s))
#define REG_PUT(x, e, s) (((x) << (s)) & REG_MASK(e, s))
#define REG_GET(x, e, s) (((x) & REG_MASK(e, s)) >> (s))

/*
 * PKT_CONTROL format:
 * [15: 0] - word count
 * [17:16] - virtual channel
 * [23:18] - data type
 * [24]    - LP or HS select (0 - LP, 1 - HS)
 * [25]    - perform BTA after packet is sent
 * [26]    - perform BTA only, no packet tx
 */
#define WC(x)		REG_PUT((x), 15,  0)
#define TX_VC(x)	REG_PUT((x), 17, 16)
#define TX_DT(x)	REG_PUT((x), 23, 18)
#define HS_SEL(x)	REG_PUT((x), 24, 24)
#define BTA_TX(x)	REG_PUT((x), 25, 25)
#define BTA_NO_TX(x)	REG_PUT((x), 26, 26)

/*
 * RX_PKT_HEADER format:
 * [15: 0] - word count
 * [21:16] - data type
 * [23:22] - virtual channel
 */
#define RX_DT(x)	REG_GET((x), 21, 16)
#define RX_VC(x)	REG_GET((x), 23, 22)

/*
 * DSI Video mode
 */
#define VIDEO_MODE_BURST_MODE_WITH_SYNC_PULSES      0
#define VIDEO_MODE_NON_BURST_MODE_WITH_SYNC_EVENTS  BIT(0)
#define VIDEO_MODE_BURST_MODE                       BIT(1)

/*
 * DPI color coding
 */
#define	DPI_16_BIT_565_PACKED  0
#define	DPI_16_BIT_565_ALIGNED 1
#define	DPI_16_BIT_565_SHIFTED 2
#define	DPI_18_BIT_PACKED      3
#define	DPI_18_BIT_ALIGNED     4
#define	DPI_24_BIT	       5

/*
 * DPI Pixel format
 */
#define PIXEL_FORMAT_16                 0
#define PIXEL_FORMAT_18                 BIT(0)
#define PIXEL_FORMAT_18L                BIT(1)
#define PIXEL_FORMAT_24                 (BIT(0) | BIT(1))

enum transfer_direction {
	DSI_PACKET_SEND,
	DSI_PACKET_RECEIVE
};

struct mipi_dsi_transfer {
	const struct mipi_dsi_msg *msg;
	struct mipi_dsi_packet packet;
	struct completion completed;

	int status;    /* status of transmission */
	enum transfer_direction direction;
	bool need_bta;
	u8 cmd;
	u16 rx_word_count;
	size_t tx_len; /* bytes sent */
	size_t rx_len; /* bytes received */
};

static inline int nwl_dsi_write(struct imx_nwl_dsi *dsi, unsigned int reg,
				u32 val)
{
	int ret;

	ret = regmap_write(dsi->regs, reg, val);
	if (ret < 0)
		DRM_DEV_ERROR(dsi->dev, "Failed to write NWL DSI reg 0x%x: %d\n",
			      reg, ret);
	return ret;
}

static inline u32 nwl_dsi_read(struct imx_nwl_dsi *dsi, u32 reg)
{
	unsigned int val;
	int ret;

	ret = regmap_read(dsi->regs, reg, &val);
	if (ret < 0)
		DRM_DEV_ERROR(dsi->dev, "Failed to read NWL DSI reg 0x%x: %d\n",
			      reg, ret);

	return val;
}

static u32 nwl_dsi_get_dpi_pixel_format(enum mipi_dsi_pixel_format format)
{

	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return PIXEL_FORMAT_16;
	case MIPI_DSI_FMT_RGB666:
		return PIXEL_FORMAT_18L;
	case MIPI_DSI_FMT_RGB666_PACKED:
		return PIXEL_FORMAT_18;
	case MIPI_DSI_FMT_RGB888:
		return PIXEL_FORMAT_24;
	default:
		return -EINVAL;
	}
}

int nwl_dsi_get_dphy_params(struct imx_nwl_dsi *dsi,
			   struct drm_display_mode *mode,
			   union phy_configure_opts *phy_opts)
{
	unsigned long rate;

	if (dsi->lanes < 1 || dsi->lanes > 4)
		return -EINVAL;

	/*
	 * So far the DPHY spec minimal timings work for both mixel
	 * dphy and nwl dsi host
	 */
	phy_mipi_dphy_get_default_config(
		mode->crtc_clock * 1000,
		mipi_dsi_pixel_format_to_bpp(dsi->format),
		dsi->lanes, &phy_opts->mipi_dphy);
	rate = clk_get_rate(dsi->tx_esc_clk);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "LP clk is @%lu Hz\n", rate);
	phy_opts->mipi_dphy.lp_clk_rate = rate;

	return 0;
}
EXPORT_SYMBOL_GPL(nwl_dsi_get_dphy_params);

#define PSEC_PER_SEC	1000000000000LL
/*
 * ps2bc - Picoseconds to byte clock cycles
 */
static unsigned int ps2bc(struct imx_nwl_dsi *dsi, unsigned long long ps)
{
	int bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);

	return DIV_ROUND_UP(ps * dsi->vm.pixelclock * bpp,
			    dsi->lanes * 8 * PSEC_PER_SEC);
}

/**
 * ui2bc - UI time periods to byte clock cycles
 */
static unsigned int ui2bc(struct imx_nwl_dsi *dsi, unsigned long long ui)
{
	int bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);

	return DIV_ROUND_UP(ui * dsi->lanes,
			    dsi->vm.pixelclock * bpp);
}

#define USEC_PER_SEC	1000000L
/*
 * us2bc - micro seconds to lp clock cycles
 */
static unsigned int us2lp(unsigned int lp_clk_rate, unsigned long us)
{
	return DIV_ROUND_UP(us * lp_clk_rate, USEC_PER_SEC);
}

static int nwl_dsi_config_host(struct imx_nwl_dsi *dsi)
{
	u32 cycles;
	struct phy_configure_opts_mipi_dphy *cfg = &dsi->phy_cfg.mipi_dphy;

	if (dsi->lanes < 1 || dsi->lanes > 4)
		return -EINVAL;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "DSI Lanes %d\n", dsi->lanes);
	nwl_dsi_write(dsi, CFG_NUM_LANES, dsi->lanes - 1);

	if (dsi->dsi_mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) {
		nwl_dsi_write(dsi, CFG_NONCONTINUOUS_CLK, 0x01);
		nwl_dsi_write(dsi, CFG_AUTOINSERT_EOTP, 0x01);
	} else {
		nwl_dsi_write(dsi, CFG_NONCONTINUOUS_CLK, 0x00);
		nwl_dsi_write(dsi, CFG_AUTOINSERT_EOTP, 0x00);
	}

	/* values in byte clock cycles */
	cycles = ui2bc(dsi, cfg->clk_pre);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "cfg_t_pre: 0x%x\n", cycles);
	nwl_dsi_write(dsi, CFG_T_PRE, cycles);
	cycles = ps2bc(dsi, cfg->lpx + cfg->clk_prepare + cfg->clk_zero);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "cfg_tx_gap (pre): 0x%x\n", cycles);
	cycles += ui2bc(dsi, cfg->clk_pre);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "cfg_tx_gap: 0x%x\n", cycles);
	nwl_dsi_write(dsi, CFG_T_POST, cycles);
	cycles = ps2bc(dsi, cfg->hs_exit);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "cfg_tx_gap: 0x%x\n", cycles);
	nwl_dsi_write(dsi, CFG_TX_GAP, cycles);

	nwl_dsi_write(dsi, CFG_EXTRA_CMDS_AFTER_EOTP, 0x01);
	nwl_dsi_write(dsi, CFG_HTX_TO_COUNT, 0x00);
	nwl_dsi_write(dsi, CFG_LRX_H_TO_COUNT, 0x00);
	nwl_dsi_write(dsi, CFG_BTA_H_TO_COUNT, 0x00);
	/* In LP clock cycles */
	cycles = us2lp(cfg->lp_clk_rate, cfg->wakeup);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "cfg_twakeup: 0x%x\n", cycles);
	nwl_dsi_write(dsi, CFG_TWAKEUP, cycles);

	return 0;
}

static int nwl_dsi_config_dpi(struct imx_nwl_dsi *dsi)
{
	struct videomode *vm = &dsi->vm;
	u32 color_format, mode;
	bool burst_mode;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "hfront_porch = %d\n", vm->hfront_porch);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "hback_porch = %d\n", vm->hback_porch);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "hsync_len = %d\n", vm->hsync_len);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "hactive = %d\n", vm->hactive);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "vfront_porch = %d\n", vm->vfront_porch);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "vback_porch = %d\n", vm->vback_porch);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "vsync_len = %d\n", vm->vsync_len);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "vactive = %d\n", vm->vactive);
	DRM_DEV_DEBUG_DRIVER(dsi->dev, "clock = %lu kHz\n",
			     vm->pixelclock / 1000);

	color_format = nwl_dsi_get_dpi_pixel_format(dsi->format);
	if (color_format < 0) {
		DRM_DEV_ERROR(dsi->dev, "Invalid color format 0x%x\n",
			      dsi->format);
		return color_format;
	}

	nwl_dsi_write(dsi, INTERFACE_COLOR_CODING, DPI_24_BIT);
	nwl_dsi_write(dsi, PIXEL_FORMAT, color_format);
	nwl_dsi_write(dsi, VSYNC_POLARITY,
		      !!(vm->flags & DISPLAY_FLAGS_VSYNC_HIGH));
	nwl_dsi_write(dsi, HSYNC_POLARITY,
		      !!(vm->flags & DISPLAY_FLAGS_HSYNC_HIGH));

	burst_mode = (dsi->dsi_mode_flags & MIPI_DSI_MODE_VIDEO_BURST) &&
		!(dsi->dsi_mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE);

	if (burst_mode) {
		nwl_dsi_write(dsi, VIDEO_MODE, VIDEO_MODE_BURST_MODE);
		nwl_dsi_write(dsi, PIXEL_FIFO_SEND_LEVEL, 256);
	} else {
		mode = ((dsi->dsi_mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE) ?
			VIDEO_MODE_BURST_MODE_WITH_SYNC_PULSES :
			VIDEO_MODE_NON_BURST_MODE_WITH_SYNC_EVENTS);
		nwl_dsi_write(dsi, VIDEO_MODE, mode);
		nwl_dsi_write(dsi, PIXEL_FIFO_SEND_LEVEL, vm->hactive);
	}

	nwl_dsi_write(dsi, HFP, vm->hfront_porch);
	nwl_dsi_write(dsi, HBP, vm->hback_porch);
	nwl_dsi_write(dsi, HSA, vm->hsync_len);

	nwl_dsi_write(dsi, ENABLE_MULT_PKTS, 0x0);
	nwl_dsi_write(dsi, BLLP_MODE, 0x1);
	nwl_dsi_write(dsi, ENABLE_MULT_PKTS, 0x0);
	nwl_dsi_write(dsi, USE_NULL_PKT_BLLP, 0x0);
	nwl_dsi_write(dsi, VC, 0x0);

	nwl_dsi_write(dsi, PIXEL_PAYLOAD_SIZE, vm->hactive);
	nwl_dsi_write(dsi, VACTIVE, vm->vactive - 1);
	nwl_dsi_write(dsi, VBP, vm->vback_porch);
	nwl_dsi_write(dsi, VFP, vm->vfront_porch);

	return 0;
}

static int nwl_dsi_enable_tx_clock(struct imx_nwl_dsi *dsi)
{
	struct device *dev = dsi->dev;
	int ret;

	ret = clk_prepare_enable(dsi->tx_esc_clk);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to enable tx_esc clk: %d\n", ret);
		return ret;
	}

	DRM_DEV_DEBUG_DRIVER(dev, "Enabled tx_esc clk @%lu Hz\n",
			     clk_get_rate(dsi->tx_esc_clk));
	return 0;
}

static int nwl_dsi_enable_rx_clock(struct imx_nwl_dsi *dsi)
{
	struct device *dev = dsi->dev;
	int ret;

	ret = clk_prepare_enable(dsi->rx_esc_clk);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to enable rx_esc clk: %d\n", ret);
		return ret;
	}

	DRM_DEV_DEBUG_DRIVER(dev, "Enabled rx_esc clk @%lu Hz\n",
			     clk_get_rate(dsi->rx_esc_clk));
	return 0;
}

static void nwl_dsi_init_interrupts(struct imx_nwl_dsi *dsi)
{
	u32 irq_enable;

	nwl_dsi_write(dsi, IRQ_MASK, 0xffffffff);
	nwl_dsi_write(dsi, IRQ_MASK2, 0x7);

	irq_enable = ~(u32)(TX_PKT_DONE_MASK | RX_PKT_HDR_RCVD_MASK |
			    TX_FIFO_OVFLW_MASK | HS_TX_TIMEOUT_MASK);

	nwl_dsi_write(dsi, IRQ_MASK, irq_enable);
}

static int nwl_dsi_host_attach(struct mipi_dsi_host *dsi_host,
			       struct mipi_dsi_device *device)
{
	struct imx_nwl_dsi *dsi = container_of(dsi_host,
						struct imx_nwl_dsi,
						dsi_host);
	struct device *dev = dsi->dev;

	DRM_DEV_INFO(dev, "lanes=%u, format=0x%x flags=0x%lx\n",
		     device->lanes, device->format, device->mode_flags);

	if (device->lanes < 1 || device->lanes > 4)
		return -EINVAL;

	dsi->lanes = device->lanes;
	dsi->format = device->format;
	dsi->dsi_mode_flags = device->mode_flags;
	dsi->panel = of_drm_find_panel(device->dev.of_node);
	if (dsi->panel)
		return drm_panel_attach(dsi->panel, &dsi->connector);

	return -EINVAL;
}

static int nwl_dsi_host_detach(struct mipi_dsi_host *dsi_host,
			       struct mipi_dsi_device *device)
{
	struct imx_nwl_dsi *dsi = container_of(dsi_host,
						struct imx_nwl_dsi,
						dsi_host);
	drm_panel_detach(dsi->panel);
	return 0;
}

static bool nwl_dsi_read_packet(struct imx_nwl_dsi *dsi, u32 status)
{
	struct device *dev = dsi->dev;
	struct mipi_dsi_transfer *xfer = dsi->xfer;
	u8 *payload = xfer->msg->rx_buf;
	u32 val;
	u16 word_count;
	u8 channel;
	u8 data_type;

	xfer->status = 0;

	if (xfer->rx_word_count == 0) {
		if (!(status & RX_PKT_HDR_RCVD))
			return false;
		/* Get the RX header and parse it */
		val = nwl_dsi_read(dsi, RX_PKT_HEADER);
		word_count = WC(val);
		channel	= RX_VC(val);
		data_type = RX_DT(val);

		if (channel != xfer->msg->channel) {
			DRM_DEV_ERROR(dev,
				"[%02X] Channel mismatch (%u != %u)\n",
				 xfer->cmd, channel, xfer->msg->channel);
			return true;
		}

		switch (data_type) {
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
			/* Fall through */
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
			if (xfer->msg->rx_len > 1) {
				/* read second byte */
				payload[1] = word_count >> 8;
				++xfer->rx_len;
			}
			/* Fall through */
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
			/* Fall through */
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
			if (xfer->msg->rx_len > 0) {
				/* read first byte */
				payload[0] = word_count & 0xff;
				++xfer->rx_len;
			}
			xfer->status = xfer->rx_len;
			return true;
		case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
			word_count &= 0xff;
			DRM_DEV_ERROR(dev,
				"[%02X] DSI error report: 0x%02x\n",
				xfer->cmd, word_count);
			xfer->status = -EPROTO;
			return true;

		}

		if (word_count > xfer->msg->rx_len) {
			DRM_DEV_ERROR(dev,
				"[%02X] Receive buffer too small: %lu (< %u)\n",
				 xfer->cmd,
				 xfer->msg->rx_len,
				 word_count);
			return true;
		}

		xfer->rx_word_count = word_count;
	} else {
		/* Set word_count from previous header read */
		word_count = xfer->rx_word_count;
	}

	/* If RX payload is not yet received, wait for it */
	if (!(status & RX_PKT_PAYLOAD_DATA_RCVD))
		return false;

	/* Read the RX payload */
	while (word_count >= 4) {
		val = nwl_dsi_read(dsi, RX_PAYLOAD);
		payload[0] = (val >>  0) & 0xff;
		payload[1] = (val >>  8) & 0xff;
		payload[2] = (val >> 16) & 0xff;
		payload[3] = (val >> 24) & 0xff;
		payload += 4;
		xfer->rx_len += 4;
		word_count -= 4;
	}

	if (word_count > 0) {
		val = nwl_dsi_read(dsi, RX_PAYLOAD);
		switch (word_count) {
		case 3:
			payload[2] = (val >> 16) & 0xff;
			++xfer->rx_len;
			/* Fall through */
		case 2:
			payload[1] = (val >>  8) & 0xff;
			++xfer->rx_len;
			/* Fall through */
		case 1:
			payload[0] = (val >>  0) & 0xff;
			++xfer->rx_len;
			break;
		}
	}

	xfer->status = xfer->rx_len;

	return true;
}

static void nwl_dsi_finish_transmission(struct imx_nwl_dsi *dsi, u32 status)
{
	struct mipi_dsi_transfer *xfer = dsi->xfer;
	bool end_packet = false;

	if (!xfer)
		return;

	if (status & TX_FIFO_OVFLW) {
		DRM_DEV_ERROR_RATELIMITED(dsi->dev, "tx fifo overflow\n");
		return;
	}

	if (status & HS_TX_TIMEOUT) {
		DRM_DEV_ERROR_RATELIMITED(dsi->dev, "HS tx timeout\n");
		return;
	}

	if (xfer->direction == DSI_PACKET_SEND && status & TX_PKT_DONE) {
		xfer->status = xfer->tx_len;
		end_packet = true;
	} else if (status & DPHY_DIRECTION &&
		   ((status & (RX_PKT_HDR_RCVD | RX_PKT_PAYLOAD_DATA_RCVD)))) {
		end_packet = nwl_dsi_read_packet(dsi, status);
	}

	if (end_packet)
		complete(&xfer->completed);
}

static void nwl_dsi_begin_transmission(struct imx_nwl_dsi *dsi)
{
	struct mipi_dsi_transfer *xfer = dsi->xfer;
	struct mipi_dsi_packet *pkt = &xfer->packet;
	const u8 *payload;
	size_t length;
	u16 word_count;
	u8 hs_mode;
	u32 val;
	u32 hs_workaround = 0;

	/* Send the payload, if any */
	length = pkt->payload_length;
	payload = pkt->payload;

	while (length >= 4) {
		val = get_unaligned_le32(payload);
		hs_workaround |= !(val & 0xFFFF00);
		nwl_dsi_write(dsi, TX_PAYLOAD, val);
		payload += 4;
		length -= 4;
	}
	/* Send the rest of the payload */
	val = 0;
	switch (length) {
	case 3:
		val |= payload[2] << 16;
		/* Fall through */
	case 2:
		val |= payload[1] << 8;
		hs_workaround |= !(val & 0xFFFF00);
		/* Fall through */
	case 1:
		val |= payload[0];
		nwl_dsi_write(dsi, TX_PAYLOAD, val);
		break;
	}
	xfer->tx_len = pkt->payload_length;

	/*
	 * Send the header
	 * header[0] = Virtual Channel + Data Type
	 * header[1] = Word Count LSB (LP) or first param (SP)
	 * header[2] = Word Count MSB (LP) or second param (SP)
	 */
	word_count = pkt->header[1] | (pkt->header[2] << 8);
	if ((hs_workaround && USE_E11418_HS_MODE_QUIRK(dsi->quirks))) {
		DRM_DEV_DEBUG_DRIVER(dsi->dev,
				     "Using hs mode workaround for cmd 0x%x\n",
				     xfer->cmd);
		hs_mode = 1;
	} else {
		hs_mode = (xfer->msg->flags & MIPI_DSI_MSG_USE_LPM) ? 0 : 1;
	}
	val = WC(word_count) |
		TX_VC(xfer->msg->channel) |
		TX_DT(xfer->msg->type) |
		HS_SEL(hs_mode) |
		BTA_TX(xfer->need_bta);
	nwl_dsi_write(dsi, PKT_CONTROL, val);

	/* Send packet command */
	nwl_dsi_write(dsi, SEND_PACKET, 0x1);
}

static ssize_t nwl_dsi_host_transfer(struct mipi_dsi_host *dsi_host,
				     const struct mipi_dsi_msg *msg)
{
	struct imx_nwl_dsi *dsi = container_of(dsi_host,
					       struct imx_nwl_dsi,
					       dsi_host);
	struct mipi_dsi_transfer xfer;
	ssize_t ret = 0;

	/* Create packet to be sent */
	dsi->xfer = &xfer;
	ret = mipi_dsi_create_packet(&xfer.packet, msg);
	if (ret < 0) {
		dsi->xfer = NULL;
		return ret;
	}

	if ((msg->type & MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM ||
	    msg->type & MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM ||
	    msg->type & MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM ||
	    msg->type & MIPI_DSI_DCS_READ) &&
	    msg->rx_len > 0 &&
	    msg->rx_buf != NULL)
		xfer.direction = DSI_PACKET_RECEIVE;
	else
		xfer.direction = DSI_PACKET_SEND;

	xfer.need_bta = (xfer.direction == DSI_PACKET_RECEIVE);
	xfer.need_bta |= (msg->flags & MIPI_DSI_MSG_REQ_ACK)?1:0;
	xfer.msg = msg;
	xfer.status = -ETIMEDOUT;
	xfer.rx_word_count = 0;
	xfer.rx_len = 0;
	xfer.cmd = 0x00;
	if (msg->tx_len > 0)
		xfer.cmd = ((u8 *)(msg->tx_buf))[0];
	init_completion(&xfer.completed);

	nwl_dsi_enable_rx_clock(dsi);

	/* Initiate the DSI packet transmision */
	nwl_dsi_begin_transmission(dsi);

	if (!wait_for_completion_timeout(&xfer.completed, MIPI_FIFO_TIMEOUT)) {
		DRM_DEV_ERROR(dsi_host->dev,
			      "[%02X] DSI transfer timed out\n", xfer.cmd);
		ret = -ETIMEDOUT;
	} else {
		ret = xfer.status;
	}

	clk_disable_unprepare(dsi->rx_esc_clk);

	return ret;
}

const struct mipi_dsi_host_ops nwl_dsi_host_ops = {
	.attach = nwl_dsi_host_attach,
	.detach = nwl_dsi_host_detach,
	.transfer = nwl_dsi_host_transfer,
};


irqreturn_t nwl_dsi_irq_handler(int irq, void *data)
{
	u32 irq_status;
	struct imx_nwl_dsi *dsi = data;

	irq_status = nwl_dsi_read(dsi, IRQ_STATUS);

	if (irq_status & TX_PKT_DONE ||
	    irq_status & RX_PKT_HDR_RCVD ||
	    irq_status & RX_PKT_PAYLOAD_DATA_RCVD)
		nwl_dsi_finish_transmission(dsi, irq_status);

	return IRQ_HANDLED;
}
EXPORT_SYMBOL_GPL(nwl_dsi_irq_handler);

int nwl_dsi_enable(struct imx_nwl_dsi *dsi)
{
	struct device *dev = dsi->dev;
	union phy_configure_opts *phy_cfg = &dsi->phy_cfg;
	int ret;

	if (!dsi->lanes) {
		DRM_DEV_ERROR(dev, "Need DSI lanes: %d\n", dsi->lanes);
		return -EINVAL;
	}

	ret = phy_init(dsi->phy);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to init DSI phy: %d\n", ret);
		return ret;
	}

	ret = phy_configure(dsi->phy, phy_cfg);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to configure DSI phy: %d\n", ret);
		return ret;
	}

	ret = nwl_dsi_enable_tx_clock(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to enable tx clock: %d\n", ret);
		return ret;
	}

	ret = nwl_dsi_config_host(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set up DSI: %d", ret);
		return ret;
	}

	ret = nwl_dsi_config_dpi(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set up DPI: %d", ret);
		return ret;
	}

	ret = phy_power_on(dsi->phy);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to power on DPHY (%d)\n", ret);
		return ret;
	}

	nwl_dsi_init_interrupts(dsi);

	ret = drm_panel_prepare(dsi->panel);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to setup panel\n");
		return ret;
	}

	nwl_dsi_write(dsi, CFG_NONCONTINUOUS_CLK,
		      !!(dsi->dsi_mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS));

	DRM_DEV_DEBUG_DRIVER(dev, "Enabling the panel\n");
	ret = drm_panel_enable(dsi->panel);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to enable panel\n");
		drm_panel_unprepare(dsi->panel);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(nwl_dsi_enable);

int nwl_dsi_disable(struct imx_nwl_dsi *dsi)
{
	struct device *dev = dsi->dev;

	DRM_DEV_DEBUG_DRIVER(dev, "Disabling clocks and phy\n");

	phy_power_off(dsi->phy);
	phy_exit(dsi->phy);

	/* Disabling the clock before the phy breaks enabling dsi again */
	clk_disable_unprepare(dsi->tx_esc_clk);

	return 0;
}
EXPORT_SYMBOL_GPL(nwl_dsi_disable);
