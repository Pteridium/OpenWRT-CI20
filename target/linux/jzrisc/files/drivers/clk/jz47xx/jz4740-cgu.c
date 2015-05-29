/*
 * Ingenic jz4740 SoC CGU driver
 *
 * Copyright (c) 2015 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <dt-bindings/clock/jz4740-cgu.h>
#include <asm/mach-jz4740/clock.h>
#include "jz47xx-cgu.h"

/* CGU register offsets */
#define CGU_REG_CPCCR		0x00
#define CGU_REG_LCR		0x04
#define CGU_REG_CPPCR		0x10
#define CGU_REG_CLKGR		0x20
#define CGU_REG_I2SCDR		0x60
#define CGU_REG_LPCDR		0x64
#define CGU_REG_MSCCDR		0x68
#define CGU_REG_UHCCDR		0x6c
#define CGU_REG_SSICDR		0x74

/* bits within a PLL control register */
#define PLLCTL_M_SHIFT		23
#define PLLCTL_M_MASK		(0x1ff << PLLCTL_M_SHIFT)
#define PLLCTL_N_SHIFT		18
#define PLLCTL_N_MASK		(0x1f << PLLCTL_N_SHIFT)
#define PLLCTL_OD_SHIFT		16
#define PLLCTL_OD_MASK		(0x3 << PLLCTL_OD_SHIFT)
#define PLLCTL_STABLE		(1 << 10)
#define PLLCTL_BYPASS		(1 << 9)
#define PLLCTL_ENABLE		(1 << 8)

/* bits within the LCR register */
#define LCR_SLEEP		(1 << 0)

/* bits within the CLKGR register */
#define CLKGR_UDC		(1 << 11)

static struct jz47xx_cgu *cgu;

static const s8 pll_od_encoding[4] = {
	0x0, 0x1, -1, 0x3,
};

static const struct jz47xx_cgu_clk_info jz4740_cgu_clocks[] = {

	/* External clocks */

	[JZ4740_CLK_EXT] = { "ext", CGU_CLK_EXT },
	[JZ4740_CLK_RTC] = { "rtc", CGU_CLK_EXT },

	[JZ4740_CLK_PLL] = {
		"pll", CGU_CLK_PLL,
		.parents = { JZ4740_CLK_EXT, -1 },
		.pll = {
			.reg = CGU_REG_CPPCR,
			.m_shift = 23,
			.m_bits = 9,
			.m_offset = 2,
			.n_shift = 18,
			.n_bits = 5,
			.n_offset = 2,
			.od_shift = 16,
			.od_bits = 2,
			.od_max = 4,
			.od_encoding = pll_od_encoding,
			.stable_bit = 10,
			.bypass_bit = 9,
			.enable_bit = 8,
		},
	},

	/* Muxes & dividers */

	[JZ4740_CLK_PLL_HALF] = {
		"pll half", CGU_CLK_DIV,
		.parents = { JZ4740_CLK_PLL, -1 },
		.div = { CGU_REG_CPCCR, 21, 1, -1, -1, -1 },
	},

	[JZ4740_CLK_CCLK] = {
		"cclk", CGU_CLK_DIV,
		.parents = { JZ4740_CLK_PLL, -1 },
		.div = { CGU_REG_CPCCR, 0, 4, 22, -1, -1 },
	},

	[JZ4740_CLK_HCLK] = {
		"hclk", CGU_CLK_DIV,
		.parents = { JZ4740_CLK_PLL, -1 },
		.div = { CGU_REG_CPCCR, 4, 4, 22, -1, -1 },
	},

	[JZ4740_CLK_PCLK] = {
		"pclk", CGU_CLK_DIV,
		.parents = { JZ4740_CLK_PLL, -1 },
		.div = { CGU_REG_CPCCR, 8, 4, 22, -1, -1 },
	},

	[JZ4740_CLK_MCLK] = {
		"mclk", CGU_CLK_DIV,
		.parents = { JZ4740_CLK_PLL, -1 },
		.div = { CGU_REG_CPCCR, 12, 4, 22, -1, -1 },
	},

	[JZ4740_CLK_LCD] = {
		"lcd", CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { JZ4740_CLK_PLL_HALF, -1 },
		.div = { CGU_REG_CPCCR, 16, 5, 22, -1, -1 },
		.gate_bit = 10,
	},

	[JZ4740_CLK_LCD_PCLK] = {
		"lcd_pclk", CGU_CLK_DIV,
		.parents = { JZ4740_CLK_PLL_HALF, -1 },
		.div = { CGU_REG_LPCDR, 0, 11, -1, -1, -1 },
	},

	[JZ4740_CLK_I2S] = {
		"i2s", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { JZ4740_CLK_EXT, JZ4740_CLK_PLL_HALF, -1 },
		.mux = { CGU_REG_CPCCR, 31, 1 },
		.div = { CGU_REG_I2SCDR, 0, 8, -1, -1, -1 },
		.gate_bit = 6,
	},

	[JZ4740_CLK_SPI] = {
		"spi", CGU_CLK_MUX | CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { JZ4740_CLK_EXT, JZ4740_CLK_PLL, -1 },
		.mux = { CGU_REG_SSICDR, 31, 1 },
		.div = { CGU_REG_SSICDR, 0, 4, -1, -1, -1 },
		.gate_bit = 4,
	},

	[JZ4740_CLK_MMC] = {
		"mmc", CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { JZ4740_CLK_PLL_HALF, -1 },
		.div = { CGU_REG_MSCCDR, 0, 5, -1, -1, -1 },
		.gate_bit = 7,
	},

	[JZ4740_CLK_UHC] = {
		"uhc", CGU_CLK_DIV | CGU_CLK_GATE,
		.parents = { JZ4740_CLK_PLL_HALF, -1 },
		.div = { CGU_REG_UHCCDR, 0, 4, -1, -1, -1 },
		.gate_bit = 14,
	},

	[JZ4740_CLK_UDC] = {
		"udc", CGU_CLK_MUX | CGU_CLK_DIV,
		.parents = { JZ4740_CLK_EXT, JZ4740_CLK_PLL_HALF, -1 },
		.mux = { CGU_REG_CPCCR, 29, 1 },
		.div = { CGU_REG_CPCCR, 23, 6, -1, -1, -1 },
		/* TODO: gate via SCR */
	},

	/* Gate-only clocks */

	[JZ4740_CLK_UART0] = {
		"uart0", CGU_CLK_GATE,
		.parents = { JZ4740_CLK_EXT, -1 },
		.gate_bit = 0,
	},

	[JZ4740_CLK_UART1] = {
		"uart1", CGU_CLK_GATE,
		.parents = { JZ4740_CLK_EXT, -1 },
		.gate_bit = 15,
	},

	[JZ4740_CLK_DMA] = {
		"dma", CGU_CLK_GATE,
		.parents = { JZ4740_CLK_PCLK, -1 },
		.gate_bit = 12,
	},

	[JZ4740_CLK_IPU] = {
		"ipu", CGU_CLK_GATE,
		.parents = { JZ4740_CLK_PCLK, -1 },
		.gate_bit = 13,
	},

	[JZ4740_CLK_ADC] = {
		"adc", CGU_CLK_GATE,
		.parents = { JZ4740_CLK_EXT, -1 },
		.gate_bit = 8,
	},

	[JZ4740_CLK_I2C] = {
		"i2c", CGU_CLK_GATE,
		.parents = { JZ4740_CLK_EXT, -1 },
		.gate_bit = 3,
	},

	[JZ4740_CLK_AIC] = {
		"aic", CGU_CLK_GATE,
		.parents = { JZ4740_CLK_EXT, -1 },
		.gate_bit = 5,
	},
};

static void __init jz4740_cgu_init(struct device_node *np)
{
	int retval;
	cgu = jz47xx_cgu_new(jz4740_cgu_clocks, ARRAY_SIZE(jz4740_cgu_clocks),
			     np);
	if (!cgu)
		pr_err("%s: failed to initialise CGU\n", __func__);

	retval = jz47xx_cgu_register_clocks(cgu);
	if (retval)
		pr_err("%s: failed to register CGU Clocks\n", __func__);
}
CLK_OF_DECLARE(jz4740_cgu, "ingenic,jz4740-cgu", jz4740_cgu_init);

void jz4740_clock_set_wait_mode(enum jz4740_wait_mode mode)
{
	uint32_t lcr = readl(cgu->base + CGU_REG_LCR);

	switch (mode) {
	case JZ4740_WAIT_MODE_IDLE:
		lcr &= ~LCR_SLEEP;
		break;

	case JZ4740_WAIT_MODE_SLEEP:
		lcr |= LCR_SLEEP;
		break;
	}

	writel(lcr, cgu->base + CGU_REG_LCR);
}

void jz4740_clock_udc_disable_auto_suspend(void)
{
	uint32_t clkgr = readl(cgu->base + CGU_REG_CLKGR);
	clkgr &= ~CLKGR_UDC;
	writel(clkgr, cgu->base + CGU_REG_CLKGR);
}
EXPORT_SYMBOL_GPL(jz4740_clock_udc_disable_auto_suspend);

void jz4740_clock_udc_enable_auto_suspend(void)
{
	uint32_t clkgr = readl(cgu->base + CGU_REG_CLKGR);
	clkgr |= CLKGR_UDC;
	writel(clkgr, cgu->base + CGU_REG_CLKGR);
}
EXPORT_SYMBOL_GPL(jz4740_clock_udc_enable_auto_suspend);

#define JZ_CLOCK_GATE_UART0	BIT(0)
#define JZ_CLOCK_GATE_TCU	BIT(1)
#define JZ_CLOCK_GATE_DMAC	BIT(12)

void jz4740_clock_suspend(void)
{
	uint32_t clkgr, cppcr;

	clkgr = readl(cgu->base + CGU_REG_CLKGR);
	clkgr |= JZ_CLOCK_GATE_TCU | JZ_CLOCK_GATE_DMAC | JZ_CLOCK_GATE_UART0;
	writel(clkgr, cgu->base + CGU_REG_CLKGR);

	cppcr = readl(cgu->base + CGU_REG_CPPCR);
	cppcr &= ~BIT(jz4740_cgu_clocks[JZ4740_CLK_PLL].pll.enable_bit);
	writel(cppcr, cgu->base + CGU_REG_CPPCR);
}

void jz4740_clock_resume(void)
{
	uint32_t clkgr, cppcr;

	cppcr = readl(cgu->base + CGU_REG_CPPCR);
	cppcr |= BIT(jz4740_cgu_clocks[JZ4740_CLK_PLL].pll.enable_bit);
	writel(cppcr, cgu->base + CGU_REG_CPPCR);

	do {
		cppcr = readl(cgu->base + CGU_REG_CPPCR);
	} while (!(cppcr & BIT(jz4740_cgu_clocks[JZ4740_CLK_PLL].pll.stable_bit)));

	clkgr = readl(cgu->base + CGU_REG_CLKGR);
	clkgr &= ~(JZ_CLOCK_GATE_TCU | JZ_CLOCK_GATE_DMAC | JZ_CLOCK_GATE_UART0);
	writel(clkgr, cgu->base + CGU_REG_CLKGR);
}