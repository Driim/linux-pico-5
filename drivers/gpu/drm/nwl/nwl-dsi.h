// SPDX-License-Identifier: GPL-2.0+
/*
 * i.MX8 NWL MIPI DSI host driver
 *
 * Copyright (C) 2017 NXP
 * Copyright (C) 2019 Purism SPC
 */
#ifndef __NWL_DSI_H__
#define __NWL_DSI_H__

#include <drm/drm_mipi_dsi.h>

/* DSI HOST registers */
#define CFG_NUM_LANES			0x0
#define CFG_NONCONTINUOUS_CLK		0x4
#define CFG_T_PRE			0x8
#define CFG_T_POST			0xc
#define CFG_TX_GAP			0x10
#define CFG_AUTOINSERT_EOTP		0x14
#define CFG_EXTRA_CMDS_AFTER_EOTP	0x18
#define CFG_HTX_TO_COUNT		0x1c
#define CFG_LRX_H_TO_COUNT		0x20
#define CFG_BTA_H_TO_COUNT		0x24
#define CFG_TWAKEUP			0x28
#define CFG_STATUS_OUT			0x2c
#define RX_ERROR_STATUS			0x30

/* DSI DPI registers */
#define PIXEL_PAYLOAD_SIZE		0x200
#define PIXEL_FIFO_SEND_LEVEL		0x204
#define INTERFACE_COLOR_CODING		0x208
#define PIXEL_FORMAT			0x20c
#define VSYNC_POLARITY			0x210
#define HSYNC_POLARITY			0x214
#define VIDEO_MODE			0x218
#define HFP				0x21c
#define HBP				0x220
#define HSA				0x224
#define ENABLE_MULT_PKTS		0x228
#define VBP				0x22c
#define VFP				0x230
#define BLLP_MODE			0x234
#define USE_NULL_PKT_BLLP		0x238
#define VACTIVE				0x23c
#define VC				0x240

/* DSI APB PKT control */
#define TX_PAYLOAD			0x280
#define PKT_CONTROL			0x284
#define SEND_PACKET			0x288
#define PKT_STATUS			0x28c
#define PKT_FIFO_WR_LEVEL		0x290
#define PKT_FIFO_RD_LEVEL		0x294
#define RX_PAYLOAD			0x298
#define RX_PKT_HEADER			0x29c

/* DSI IRQ handling */
#define IRQ_STATUS			0x2a0
#define SM_NOT_IDLE			BIT(0)
#define TX_PKT_DONE			BIT(1)
#define DPHY_DIRECTION			BIT(2)
#define TX_FIFO_OVFLW			BIT(3)
#define TX_FIFO_UDFLW			BIT(4)
#define RX_FIFO_OVFLW			BIT(5)
#define RX_FIFO_UDFLW			BIT(6)
#define RX_PKT_HDR_RCVD			BIT(7)
#define RX_PKT_PAYLOAD_DATA_RCVD	BIT(8)
#define BTA_TIMEOUT			BIT(29)
#define LP_RX_TIMEOUT			BIT(30)
#define HS_TX_TIMEOUT			BIT(31)

#define IRQ_STATUS2			0x2a4
#define SINGLE_BIT_ECC_ERR		BIT(0)
#define MULTI_BIT_ECC_ERR		BIT(1)
#define CRC_ERR				BIT(2)

#define IRQ_MASK			0x2a8
#define SM_NOT_IDLE_MASK		BIT(0)
#define TX_PKT_DONE_MASK		BIT(1)
#define DPHY_DIRECTION_MASK		BIT(2)
#define TX_FIFO_OVFLW_MASK		BIT(3)
#define TX_FIFO_UDFLW_MASK		BIT(4)
#define RX_FIFO_OVFLW_MASK		BIT(5)
#define RX_FIFO_UDFLW_MASK		BIT(6)
#define RX_PKT_HDR_RCVD_MASK		BIT(7)
#define RX_PKT_PAYLOAD_DATA_RCVD_MASK	BIT(8)
#define BTA_TIMEOUT_MASK		BIT(29)
#define LP_RX_TIMEOUT_MASK		BIT(30)
#define HS_TX_TIMEOUT_MASK		BIT(31)

#define IRQ_MASK2			0x2ac
#define SINGLE_BIT_ECC_ERR_MASK		BIT(0)
#define MULTI_BIT_ECC_ERR_MASK		BIT(1)
#define CRC_ERR_MASK			BIT(2)

extern const struct mipi_dsi_host_ops nwl_dsi_host_ops;

irqreturn_t nwl_dsi_irq_handler(int irq, void *data);
int nwl_dsi_enable(struct imx_nwl_dsi *dsi);
int nwl_dsi_disable(struct imx_nwl_dsi *dsi);
int nwl_dsi_get_dphy_params(struct imx_nwl_dsi *dsi,
			    struct drm_display_mode *mode,
			    union phy_configure_opts *phy_opts);

#endif /* __NWL_DSI_H__ */
