/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2022 Julien Cassette <julien.cassette@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/bus.h>

#include <dev/fdt/simplebus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk_div.h>
#include <dev/extres/clk/clk_fixed.h>
#include <dev/extres/clk/clk_mux.h>

#include <arm/allwinner/clkng/aw_ccung.h>

#include <dt-bindings/clock/sun20i-d1-ccu.h>
#include <dt-bindings/reset/sun20i-d1-ccu.h>

/* Non exported clocks */
#define CLK_PLL_PERI	145
#define CLK_PLL_VIDEO0	146
#define CLK_PLL_VIDEO1	147
#define CLK_PLL_AUDIO0	148
#define CLK_PLL_CPU_P	149

/* Some names in the device tree are not consistent */
#define HOSC		"osc24M"
#define CLK16M_RC	"iosc"
#define CLK32K		"osc32k"

static struct aw_ccung_reset d1_ccu_resets[] = {
	CCU_RESET(RST_MBUS,		0x540,	30)
	CCU_RESET(RST_BUS_DE,		0x60C,	16)
	CCU_RESET(RST_BUS_DI,		0x62C,	16)
	CCU_RESET(RST_BUS_G2D,		0x63C,	16)
	CCU_RESET(RST_BUS_CE,		0x68C,	16)
	CCU_RESET(RST_BUS_VE,		0x69C,	16)
	CCU_RESET(RST_BUS_DMA,		0x70C,	16)
	CCU_RESET(RST_BUS_MSGBOX2,	0x71C,	18)
	CCU_RESET(RST_BUS_MSGBOX1,	0x71C,	17)
	CCU_RESET(RST_BUS_MSGBOX0,	0x71C,	16)
	CCU_RESET(RST_BUS_SPINLOCK,	0x72C,	16)
	CCU_RESET(RST_BUS_HSTIMER,	0x73C,	16)
	CCU_RESET(RST_BUS_DBGSYS,	0x78C,	16)
	CCU_RESET(RST_BUS_PWM,		0x7AC,	16)
	CCU_RESET(RST_BUS_DRAM,		0x80C,	16)
	CCU_RESET(RST_BUS_MMC2,		0x84C,	18)
	CCU_RESET(RST_BUS_MMC1,		0x84C,	17)
	CCU_RESET(RST_BUS_MMC0,		0x84C,	16)
	CCU_RESET(RST_BUS_UART5,	0x90C,	21)
	CCU_RESET(RST_BUS_UART4,	0x90C,	20)
	CCU_RESET(RST_BUS_UART3,	0x90C,	19)
	CCU_RESET(RST_BUS_UART2,	0x90C,	18)
	CCU_RESET(RST_BUS_UART1,	0x90C,	17)
	CCU_RESET(RST_BUS_UART0,	0x90C,	16)
	CCU_RESET(RST_BUS_I2C3,		0x91C,	19)
	CCU_RESET(RST_BUS_I2C2,		0x91C,	18)
	CCU_RESET(RST_BUS_I2C1,		0x91C,	17)
	CCU_RESET(RST_BUS_I2C0,		0x91C,	16)
	CCU_RESET(RST_BUS_SPI1,		0x96C,	17)
	CCU_RESET(RST_BUS_SPI0,		0x96C,	16)
	CCU_RESET(RST_BUS_EMAC,		0x97C,	16)
	CCU_RESET(RST_BUS_IRTX,		0x9CC,	16)
	CCU_RESET(RST_BUS_GPADC,	0x9EC,	16)
	CCU_RESET(RST_BUS_THS,		0x9FC,	16)
	CCU_RESET(RST_BUS_I2S2,		0xA20,	18)
	CCU_RESET(RST_BUS_I2S1,		0xA20,	17)
	CCU_RESET(RST_BUS_I2S0,		0xA20,	16)
	CCU_RESET(RST_BUS_SPDIF,	0xA2C,	16)
	CCU_RESET(RST_BUS_DMIC,		0xA4C,	16)
	CCU_RESET(RST_BUS_AUDIO,	0xA5C,	16)
	CCU_RESET(RST_USBPHY0,		0xA70,	30)
	CCU_RESET(RST_USBPHY1,		0xA74,	30)
	CCU_RESET(RST_BUS_OTG0,		0xA8C,	24)
	CCU_RESET(RST_BUS_EHCI1,	0xA8C,	21)
	CCU_RESET(RST_BUS_EHCI0,	0xA8C,	20)
	CCU_RESET(RST_BUS_OHCI1,	0xA8C,	17)
	CCU_RESET(RST_BUS_OHCI0,	0xA8C,	16)
	CCU_RESET(RST_BUS_LRADC,	0xA9C,	16)
	CCU_RESET(RST_BUS_DPSS_TOP,	0xABC,	16)
	CCU_RESET(RST_BUS_DSI,		0xB4C,	16)
	CCU_RESET(RST_BUS_TCONLCD,	0xB7C,	16)
	CCU_RESET(RST_BUS_TCONTV,	0xB9C,	16)
	CCU_RESET(RST_BUS_LVDS0,	0xBAC,	16)
	CCU_RESET(RST_BUS_TVE,		0xBBC,	17)
	CCU_RESET(RST_BUS_TVE_TOP,	0xBBC,	16)
	CCU_RESET(RST_BUS_TVD,		0xBDC,	17)
	CCU_RESET(RST_BUS_TVD_TOP,	0xBDC,	16)
	CCU_RESET(RST_BUS_LEDC,		0xBFC,	16)
	CCU_RESET(RST_BUS_CSI,		0xC1C,	16)
	CCU_RESET(RST_BUS_TPADC,	0xC5C,	16)
	CCU_RESET(RST_BUS_DSP_DBG,	0xC7C,	18)
	CCU_RESET(RST_BUS_DSP_CFG,	0xC7C,	17)
	CCU_RESET(RST_BUS_DSP,		0xC7C,	16)
	CCU_RESET(RST_BUS_RISCV_CFG,	0xD0C,	16)
};

static struct aw_ccung_gate d1_ccu_gates[] = {
	CCU_GATE(CLK_BUS_DE,		"bus-de",	"psi",	0x60C,	0)
	CCU_GATE(CLK_BUS_DI,		"bus-di",	"psi",	0x62C,	0)
	CCU_GATE(CLK_BUS_G2D,		"bus-g2d",	"psi",	0x63C,	0)
	CCU_GATE(CLK_BUS_CE,		"bus-ce",	"psi",	0x68C,	0)
	CCU_GATE(CLK_BUS_VE,		"bus-ve",	"psi",	0x690,	0)
	CCU_GATE(CLK_BUS_DMA,		"bus-dma",	"psi",	0x70C,	0)
	CCU_GATE(CLK_BUS_MSGBOX2,	"bus-msgbox2",	"psi",	0x71C,	2)
	CCU_GATE(CLK_BUS_MSGBOX1,	"bus-msgbox1",	"psi",	0x71C,	1)
	CCU_GATE(CLK_BUS_MSGBOX0,	"bus-msgbox0",	"psi",	0x71C,	0)
	CCU_GATE(CLK_BUS_SPINLOCK,	"bus-spinlock",	"psi",	0x72C,	0)
	CCU_GATE(CLK_BUS_HSTIMER,	"bus-hstimer",	"psi",	0x73C,	0)
	CCU_GATE(CLK_AVS,		"avs",		HOSC,	0x740,	31)
	CCU_GATE(CLK_BUS_DBGSYS,	"bus-dbgsys",	"psi",	0x78C,	0)
	CCU_GATE(CLK_BUS_PWM,		"bus-pwm",	"psi",	0x7AC,	0)
	CCU_GATE(CLK_BUS_IOMMU,		"bus-iommu",	"apb0",	0x7BC,	0)
	CCU_GATE(CLK_MBUS_RISCV,	"mbus-riscv",	"mbus",	0x804,	11)
	CCU_GATE(CLK_MBUS_G2D,		"mbus-g2d",	"mbus",	0x804,	10)
	CCU_GATE(CLK_MBUS_CSI,		"mbus-csi",	"mbus",	0x804,	8)
	CCU_GATE(CLK_MBUS_TVIN,		"mbus-tvin",	"mbus",	0x804,	7)
	CCU_GATE(CLK_MBUS_CE,		"mbus-ce",	"mbus",	0x804,	2)
	CCU_GATE(CLK_MBUS_VE,		"mbus-ve",	"mbus",	0x804,	1)
	CCU_GATE(CLK_MBUS_DMA,		"mbus-dma",	"mbus",	0x804,	0)
	CCU_GATE(CLK_BUS_DRAM,		"bus-dram",	"psi",	0x80C,	0)
	CCU_GATE(CLK_BUS_MMC2,		"bus-mmc2",	"psi",	0x84C,	2)
	CCU_GATE(CLK_BUS_MMC1,		"bus-mmc1",	"psi",	0x84C,	1)
	CCU_GATE(CLK_BUS_MMC0,		"bus-mmc0",	"psi",	0x84C,	0)
	CCU_GATE(CLK_BUS_UART5,		"bus-uart5",	"apb1",	0x90C,	5)
	CCU_GATE(CLK_BUS_UART4,		"bus-uart4",	"apb1",	0x90C,	4)
	CCU_GATE(CLK_BUS_UART3,		"bus-uart3",	"apb1",	0x90C,	3)
	CCU_GATE(CLK_BUS_UART2,		"bus-uart2",	"apb1",	0x90C,	2)
	CCU_GATE(CLK_BUS_UART1,		"bus-uart1",	"apb1",	0x90C,	1)
	CCU_GATE(CLK_BUS_UART0,		"bus-uart0",	"apb1",	0x90C,	0)
	CCU_GATE(CLK_BUS_I2C3,		"bus-i2c3",	"apb1",	0x91C,	3)
	CCU_GATE(CLK_BUS_I2C2,		"bus-i2c2",	"apb1",	0x91C,	2)
	CCU_GATE(CLK_BUS_I2C1,		"bus-i2c1",	"apb1",	0x91C,	1)
	CCU_GATE(CLK_BUS_I2C0,		"bus-i2c0",	"apb1",	0x91C,	0)
	CCU_GATE(CLK_BUS_SPI1,		"bus-spi1",	"psi",	0x96C,	1)
	CCU_GATE(CLK_BUS_SPI0,		"bus-spi0",	"psi",	0x96C,	0)
	CCU_GATE(CLK_BUS_EMAC,		"bus-emac",	"psi",	0x97C,	0)
	CCU_GATE(CLK_BUS_IRTX,		"bus-irtx",	"apb0",	0x9CC,	0)
	CCU_GATE(CLK_BUS_GPADC,		"bus-gpadc",	"apb0",	0x9EC,	0)
	CCU_GATE(CLK_BUS_THS,		"bus-ths",	"apb0",	0x9FC,	0)
	CCU_GATE(CLK_BUS_I2S2,		"bus-i2s2",	"apb0",	0xA10,	2)
	CCU_GATE(CLK_BUS_I2S1,		"bus-i2s1",	"apb0",	0xA10,	1)
	CCU_GATE(CLK_BUS_I2S0,		"bus-i2s0",	"apb0",	0xA10,	0)
	CCU_GATE(CLK_BUS_SPDIF,		"bus-spdif",	"apb0",	0xA2C,	0)
	CCU_GATE(CLK_BUS_DMIC,		"bus-dmic",	"apb0",	0xA4C,	0)
	CCU_GATE(CLK_BUS_AUDIO,		"bus-audio",	"apb0",	0xA5C,	0)
	CCU_GATE(CLK_BUS_OTG0,		"bus-otg",	"psi",	0xA8C,	8)
	CCU_GATE(CLK_BUS_EHCI1,		"bus-ehci1",	"psi",	0xA8C,	5)
	CCU_GATE(CLK_BUS_EHCI0,		"bus-ehci0",	"psi",	0xA8C,	4)
	CCU_GATE(CLK_BUS_OHCI1,		"bus-ohci1",	"psi",	0xA8C,	1)
	CCU_GATE(CLK_BUS_OHCI0,		"bus-ohci0",	"psi",	0xA8C,	0)
	CCU_GATE(CLK_BUS_LRADC,		"bus-lradc",	"apb0",	0xA9C,	0)
	CCU_GATE(CLK_BUS_DPSS_TOP,	"bus-dpss-top",	"psi",	0xABC,	0)
	CCU_GATE(CLK_BUS_DSI,		"bus-dsi",	"psi",	0xB4C,	0)
	CCU_GATE(CLK_BUS_TCONLCD,	"bus-tconlcd",	"psi",	0xB7C,	0)
	CCU_GATE(CLK_BUS_TCONTV,	"bus-tcontv",	"psi",	0xB9C,	0)
	CCU_GATE(CLK_BUS_TVE,		"bus-tve",	"psi",	0xBBC,	1)
	CCU_GATE(CLK_BUS_TVE_TOP,	"bus-tve-top",	"psi",	0xBBC,	0)
	CCU_GATE(CLK_BUS_TVD,		"bus-tvd",	"psi",	0xBDC,	1)
	CCU_GATE(CLK_BUS_TVD_TOP,	"bus-tvd-top",	"psi",	0xBDC,	0)
	CCU_GATE(CLK_BUS_LEDC,		"bus-ledc",	"psi",	0xBFC,	0)
	CCU_GATE(CLK_BUS_CSI,		"bus-csi",	"psi",	0xC1C,	0)
	CCU_GATE(CLK_BUS_TPADC,		"bus-tpadc",	"apb0",	0xC5C,	0)
	CCU_GATE(CLK_BUS_DSP_CFG,	"bus-dsp-cfg",	"psi",	0xC7C,	1)
	CCU_GATE(CLK_BUS_RISCV_CFG,	"bus-riscv-cfg", "psi",	0xD0C,	0)
};

static const char *pll_cpu_parents[] = { HOSC };
NP_CLK(pll_cpu_clk,
    CLK_PLL_CPU,				/* id */
    "pll_cpu",					/* name */
    pll_cpu_parents,				/* parents */
    0x0,					/* offset */
    8, 7, 0, 0,					/* n factor */
    0, 2, 0, 0,					/* p factor */
    27,						/* gate */
    28, 1000,					/* lock */
    AW_CLK_HAS_GATE | AW_CLK_HAS_LOCK);		/* flags */

static const char *pll_ddr_parents[] = { HOSC };
NMM_CLK(pll_ddr_clk,
    CLK_PLL_DDR,				/* id */
    "pll_ddr",					/* name */
     pll_ddr_parents,				/* parents */
    0x10,					/* offset */
    8, 7, 0, 0,					/* n factor */
    0, 1, 0, 0,					/* m0 factor */
    1, 1, 0, 0,					/* m1 factor */
    27,						/* gate */
    28, 1000,					/* lock */
    AW_CLK_HAS_GATE | AW_CLK_HAS_LOCK);		/* flags */

/* For child clocks: 24 MHz * N / M */
static const char *pll_peri_parents[] = { HOSC };
NP_CLK(pll_peri_clk,
    CLK_PLL_PERI,				/* id */
    "pll_peri",					/* name */
    pll_peri_parents,				/* parents */
    0x20,					/* offset */
    8, 7, 0, 0,					/* n factor */
    1, 1, 0, 0,					/* p factor */
    27,						/* gate */
    28, 1000,					/* lock */
    AW_CLK_HAS_GATE | AW_CLK_HAS_LOCK);		/* flags */

/* PLL_PERI(2X) = 24 MHz * N / M / P0 */
static const char *pll_peri_2x_parents[] = { "pll_peri" };
M_CLK(pll_peri_2x_clk,
    CLK_PLL_PERI_2X,				/* id */
    "pll_peri_2x",				/* name */
    pll_peri_2x_parents,			/* parents */
    0x20,					/* offset */
    16, 3, 0, 0,				/* m factor */
    0, 0,					/* mux */
    0,						/* gate */
    0);						/* flags */

/* PLL_PERI(1X) = 24 MHz * N / M / P0 / 2 */
static const char *pll_peri_1x_parents[] = { "pll_peri_2x" };
FIXED_CLK(pll_peri_1x_clk,
    CLK_PLL_PERI_1X,				/* id */
    "pll_peri_1x",				/* name */
    pll_peri_1x_parents,			/* parents */
    0,						/* freq */
    1,						/* mult */
    2,						/* div */
    0);						/* flags */

/* PLL_PERI(800M) = 24 MHz * N / M / P1 */
static const char *pll_peri_800m_parents[] = { "pll_peri" };
M_CLK(pll_peri_800m_clk,
    CLK_PLL_PERI_800M,				/* id */
    "pll_peri_800m",				/* name */
    pll_peri_800m_parents,			/* parents */
    0x20,					/* offset */
    20, 3, 0, 0,				/* m factor */
    0, 0,					/* mux */
    0,						/* gate */
    0);						/* flags */

/* For child clocks: InputFreq * N / M */
static const char *pll_video0_parents[] = { HOSC };
NP_CLK(pll_video0_clk,
    CLK_PLL_VIDEO0,				/* id */
    "pll_video0",				/* name */
    pll_video0_parents,				/* parents */
    0x40,					/* offset */
    8, 7, 0, 0,					/* n factor */
    1, 1, 0, 0,					/* p factor */
    27,						/* gate */
    28, 1000,					/* lock */
    AW_CLK_HAS_GATE | AW_CLK_HAS_LOCK);		/* flags */

/* PLL_VIDEO0(4X) = InputFreq * N / M / D */
/* D is only for testing */
static const char *pll_video0_4x_parents[] = { "pll_video0" };
M_CLK(pll_video0_4x_clk,
    CLK_PLL_VIDEO0_4X,				/* id */
    "pll_video0_4x",				/* name */
    pll_video0_4x_parents,			/* parents */
    0x40,					/* offset */
    0, 1, 0, 0,					/* m factor */
    0, 0,					/* mux */
    0,						/* gate */
    0);						/* flags */

/* PLL_VIDEO0(2X) = InputFreq * N / M / 2 */
static const char *pll_video0_2x_parents[] = { "pll_video0" };
FIXED_CLK(pll_video0_2x_clk,
    CLK_PLL_VIDEO0_2X,				/* id */
    "pll_video0_2x",				/* name */
    pll_video0_2x_parents,			/* parents */
    0,						/* freq */
    1,						/* mult */
    2,						/* div */
    0);						/* flags */

/* PLL_VIDEO0(1X) = InputFreq * N / M / 4 */
static const char *pll_video0_1x_parents[] = { "pll_video0" };
FIXED_CLK(pll_video0_1x_clk,
    CLK_PLL_VIDEO0_1X,				/* id */
    "pll_video0_1x",				/* name */
    pll_video0_1x_parents,			/* parents */
    0,						/* freq */
    1,						/* mult */
    4,						/* div */
    0);						/* flags */

/* For child clocks: InputFreq * N / M */
static const char *pll_video1_parents[] = { HOSC };
NP_CLK(pll_video1_clk,
    CLK_PLL_VIDEO1,				/* id */
    "pll_video1", 				/* name */
    pll_video1_parents,				/* parents */
    0x48,					/* offset */
    8, 7, 0, 0,					/* n factor */
    1, 1, 0, 0,					/* p factor */
    27,						/* gate */
    28, 1000,					/* lock */
    AW_CLK_HAS_GATE | AW_CLK_HAS_LOCK);		/* flags */

/* PLL_VIDEO1(4X) = InputFreq * N / M / D */
/* D is only for testing */
static const char *pll_video1_4x_parents[] = { "pll_video1" };
M_CLK(pll_video1_4x_clk,
    CLK_PLL_VIDEO1_4X,				/* id */
    "pll_video1_4x",				/* name */
    pll_video1_4x_parents,			/* parents */
    0x48,					/* offset */
    0, 1, 0, 0,					/* m factor */
    0, 0,					/* mux */
    0,						/* gate */
    0);						/* flags */

/* PLL_VIDEO1(2X) = InputFreq * N / M / 2 */
static const char *pll_video1_2x_parents[] = { "pll_video1" };
FIXED_CLK(pll_video1_2x_clk,
    CLK_PLL_VIDEO1_2X,				/* id */
    "pll_video1_2x",				/* name */
    pll_video1_2x_parents,			/* parents */
    0,						/* freq */
    1,						/* mult */
    2,						/* div */
    0);						/* flags */

/* PLL_VIDEO1(1X) = InputFreq * N / M / 4 */
static const char *pll_video1_1x_parents[] = { "pll_video1" };
FIXED_CLK(pll_video1_1x_clk,
    CLK_PLL_VIDEO1_1X,				/* id */
    "pll_video1_1x",				/* name */
    pll_video1_1x_parents,			/* parents */
    0,						/* freq */
    1,						/* mult */
    4,						/* div */
    0);						/* flags */

static const char *pll_ve_parents[] = { HOSC };
NMM_CLK(pll_ve_clk,
    CLK_PLL_VE,					/* id */
    "pll_ve",					/* name */
    pll_ve_parents,				/* parents */
    0x58,					/* offset */
    8, 7, 0, 0,					/* n factor */
    0, 1, 0, 0,					/* m0 factor */
    1, 1, 0, 0,					/* m1 factor */
    27,						/* gate */
    28, 1000,					/* lock */
    AW_CLK_HAS_GATE | AW_CLK_HAS_LOCK);		/* flags */

/* For child clocks: 24MHz * N / M1 / M0 */
static const char *pll_audio0_parents[] = { HOSC };
NMM_CLK(pll_audio0_clk,
    CLK_PLL_AUDIO0,				/* id */
    "pll_audio0",				/* name */
    pll_audio0_parents,				/* parents */
    0x78,					/* offset */
    8, 7, 0, 0,					/* n factor */
    0, 1, 0, 0,					/* m0 factor */
    1, 1, 0, 0,					/* m1 factor */
    27,						/* gate */
    28, 1000,					/* lock */
    AW_CLK_HAS_GATE | AW_CLK_HAS_LOCK);		/* flags */

/* PLL_AUDIO0(4X) = 24MHz * N / M1 / M0 / P */
static const char *pll_audio0_4x_parents[] = { "pll_audio0" };
M_CLK(pll_audio0_4x_clk,
    CLK_PLL_AUDIO0_4X,				/* id */
    "pll_audio0_4x",				/* name */
    pll_audio0_4x_parents,			/* parents */
    0x78,					/* offset */
    16, 5, 0, 0,				/* m factor */
    0, 0,					/* mux */
    0,						/* gate */
    0);						/* flags */

/* PLL_AUDIO0(2X) = (24MHz * N / M1 / M0) / P / 2 */
static const char *pll_audio0_2x_parents[] = { "pll_audio0_4x" };
FIXED_CLK(pll_audio0_2x_clk,
    CLK_PLL_AUDIO0_2X,				/* id */
    "pll_audio0_2x",				/* name */
    pll_audio0_2x_parents,			/* parents */
    0,						/* freq */
    1,						/* mult */
    2,						/* div */
    0);						/* flags */

/* PLL_AUDIO0(1X) = (24MHz * N / M1 / M0) / P / 4 */
static const char *pll_audio0_1x_parents[] = { "pll_audio0_4x" };
FIXED_CLK(pll_audio0_1x_clk,
    CLK_PLL_AUDIO0_1X,				/* id */
    "pll_audio0_1x",				/* name */
    pll_audio0_1x_parents,			/* parents */
    0,						/* freq */
    1,						/* mult */
    4,						/* div */
    0);						/* flags */

/* For child clocks: 24MHz * N / M */
static const char *pll_audio1_parents[] = { HOSC };
NP_CLK(pll_audio1_clk,
    CLK_PLL_AUDIO1,				/* id */
    "pll_audio1", 				/* name */
    pll_audio1_parents,				/* parents */
    0x80,					/* offset */
    8, 7, 0, 0,					/* n factor */
    1, 1, 0, 0,					/* p factor */
    27,						/* gate */
    28, 1000,					/* lock */
    AW_CLK_HAS_GATE | AW_CLK_HAS_LOCK);		/* flags */

/* PLL_AUDIO1(DIV2) = 24MHz * N / M / P0 */
static const char *pll_audio1_div2_parents[] = { "pll_audio1" };
M_CLK(pll_audio1_div2_clk,
    CLK_PLL_AUDIO1_DIV2,			/* id */
    "pll_audio1_div2",				/* name */
    pll_audio1_div2_parents,			/* parents */
    0x80,					/* offset */
    16, 3, 0, 0,				/* m factor */
    0, 0,					/* mux */
    0,						/* gate */
    0);						/* flags */

/* PLL_AUDIO1(DIV5) = 24MHz * N / M / P1 */
static const char *pll_audio1_div5_parents[] = { "pll_audio1" };
M_CLK(pll_audio1_div5_clk,
    CLK_PLL_AUDIO1_DIV5,			/* id */
    "pll_audio1_div5",				/* name */
    pll_audio1_div5_parents,			/* parents */
    0x80,					/* offset */
    20, 3, 0, 0,				/* m factor */
    0, 0,					/* mux */
    0,						/* gate */
    0);						/* flags */

/* For CPU_AXI: PLL_CPU / P */
static const char *pll_cpu_p_parents[] = { "pll_cpu" };
M_CLK(pll_cpu_p_clk,
    CLK_PLL_CPU_P,				/* id */
    "pll_cpu_p",				/* name */
    pll_cpu_p_parents,				/* parents */
    0x500,					/* offset */
    16, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* m factor */
    0, 0,					/* mux */
    0,						/* gate */
    0);						/* flags */

static const char *cpu_axi_parents[] = { HOSC, CLK32K, CLK16M_RC,
    "pll_cpu_p", "pll_peri_1x", "pll_peri_2x", "pll_peri_800m" };
M_CLK(cpu_axi_clk,
    CLK_CPU_AXI,				/* id */
    "cpu_axi",					/* name */
    cpu_axi_parents,				/* parents */
    0x500,					/* offset */
    0, 2, 0, 0,					/* m factor */
    0, 0,					/* mux */
    0,						/* gate */
    0);						/* flags */

static const char *psi_parents[] = { HOSC, CLK32K, CLK16M_RC,
    "pll_peri_1x" };
NM_CLK(psi_clk,
    CLK_PSI, "psi", psi_parents,		/* id, name, parents */
    0x510,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 2, 0, 0,					/* m factor */
    24, 2,					/* mux */
    0,						/* gate */
    AW_CLK_HAS_MUX | AW_CLK_REPARENT);		/* flags */

static const char *apb0_parents[] = { HOSC, CLK32K, "psi", "pll_peri_1x" };
NM_CLK(apb0_clk,
    CLK_APB0, "apb0", apb0_parents,		/* id, name, parents */
    0x520,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 2, 0, 0,					/* m factor */
    24, 2,					/* mux */
    0,						/* gate */
    AW_CLK_HAS_MUX | AW_CLK_REPARENT);		/* flags */

static const char *apb1_parents[] = { HOSC, CLK32K, "psi", "pll_peri_1x" };
NM_CLK(apb1_clk,
    CLK_APB1, "apb1", apb1_parents,		/* id, name, parents */
    0x524,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 2, 0, 0,					/* m factor */
    24, 2,					/* mux */
    0,						/* gate */
    AW_CLK_HAS_MUX | AW_CLK_REPARENT);		/* flags */

static const char *mbus_parents[] = { "dram" };
FIXED_CLK(mbus_clk,
    CLK_MBUS, "mbus", mbus_parents,		/* id, name, parents */
    0,						/* freq */
    1,						/* mult */
    4,						/* div */
    0);						/* flags */

static const char *de_parents[] = { "pll_peri_2x", "pll_video0_4x",
    "pll_video1_4x", "pll_audio1_div2" };
M_CLK(de_clk,
    CLK_DE, "de", de_parents,			/* id, name, parents */
    0x600,					/* offset */
    0, 5, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *di_parents[] = { "pll_peri_2x", "pll_video0_4x",
    "pll_video1_4x", "pll_audio1_div2" };
M_CLK(di_clk,
    CLK_DI, "di", di_parents,			/* id, name, parents */
    0x620,					/* offset */
    0, 5, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *g2d_parents[] = { "pll_peri_2x", "pll_video0_4x",
    "pll_video1_4x", "pll_audio1_div2" };
M_CLK(g2d_clk,
    CLK_G2D, "g2d", g2d_parents,		/* id, name, parents */
    0x630,					/* offset */
    0, 5, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *ce_parents[] = { HOSC, "pll_peri_2x", "pll_peri_1x" };
NM_CLK(ce_clk,
    CLK_CE, "ce", ce_parents,			/* id, name, parents */
    0x680,					/* offset */
    8, 2, 0, 0,					/* n factor */
    0, 4, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *ve_parents[] = { "pll_ve", "pll_peri_2x" };
M_CLK(ve_clk,
    CLK_VE, "ve", ve_parents,			/* id, name, parents */
    0x690,					/* offset */
    0, 5, 0, 0,					/* m factor */
    24, 1,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |		/* flags */
    AW_CLK_REPARENT);

static const char *dram_parents[] = { "pll_ddr", "pll_audio1_div2",
    "pll_peri_2x", "pll_peri_800m" };
NM_CLK(dram_clk,
    CLK_DRAM, "dram", dram_parents,		/* id, name, parents */
    0x800,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 2, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |		/* flags */
    AW_CLK_REPARENT);

/* SMHC0 */
static const char *mmc0_parents[] = { HOSC, "pll_peri_1x", "pll_peri_2x",
    "pll_audio1_div2" };
NM_CLK(mmc0_clk,
    CLK_MMC0, "mmc0", mmc0_parents,		/* id, name, parents */
    0x830,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 4, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

/* SMHC1 */
static const char *mmc1_parents[] = { HOSC, "pll_peri_1x", "pll_peri_2x",
    "pll_audio1_div2" };
NM_CLK(mmc1_clk,
    CLK_MMC1, "mmc1", mmc1_parents,		/* id, name, parents */
    0x834,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 4, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

/* SMHC2 */
static const char *mmc2_parents[] = { HOSC, "pll_peri_1x", "pll_peri_2x",
    "pll_peri_800m", "pll_audio1_div2" };
NM_CLK(mmc2_clk,
    CLK_MMC2, "mmc2", mmc2_parents,		/* id, name, parents */
    0x838,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 4, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *spi0_parents[] = { HOSC, "pll_peri_1x", "pll_peri_2x",
    "pll_audio1_div2", "pll_audio1_div5" };
NM_CLK(spi0_clk,
    CLK_SPI0, "spi0", spi0_parents,		/* id, name, parents */
    0x940,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 4, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *spi1_parents[] = { HOSC, "pll_peri_1x", "pll_peri_2x",
    "pll_audio1_div2", "pll_audio1_div5" };
NM_CLK(spi1_clk,
    CLK_SPI1, "spi1", spi1_parents,		/* id, name, parents */
    0x944,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 4, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

/* Use M_CLK to have gate */
static const char *emac_25m_parents[] = { "pll_peri_1x" };
M_CLK(emac_25m_clk,
    CLK_EMAC_25M,				/* id */
    "emac_25m",					/* name */
    emac_25m_parents,				/* parents */
    0x970,					/* offset */
    0, 0, 24, AW_CLK_FACTOR_FIXED,		/* m factor */
    0, 0,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_REPARENT);		/* flags */

static const char *irtx_parents[] = { HOSC, "pll_peri_1x" };
NM_CLK(irtx_clk,
    CLK_IRTX, "irtx", irtx_parents,		/* id, name, parents */
    0x9C0,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 4, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *i2s0_parents[] = { "pll_audio0_1x", "pll_audio0_4x",
    "pll_audio1_div2", "pll_audio1_div5" };
NM_CLK(i2s0_clk,
    CLK_I2S0, "i2s0", i2s0_parents,		/* id, name, parents */
    0xA10,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 5, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *i2s1_parents[] = { "pll_audio0_1x", "pll_audio0_4x",
    "pll_audio1_div2", "pll_audio1_div5" };
NM_CLK(i2s1_clk,
    CLK_I2S1, "i2s1", i2s1_parents,		/* id, name, parents */
    0xA14,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 5, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *i2s2_parents[] = { "pll_audio0_1x", "pll_audio0_4x",
    "pll_audio1_div2", "pll_audio1_div5" };
NM_CLK(i2s2_clk,
    CLK_I2S2, "i2s2", i2s2_parents,		/* id, name, parents */
    0xA18,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 5, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *i2s2_asrc_parents[] = { "pll_audio0_4x", "pll_peri_1x",
    "pll_audio1_div2", "pll_audio1_div5" };
NM_CLK(i2s2_asrc_clk,
    CLK_I2S2_ASRC,				/* id */
    "i2s2_asrc",				/* name */
    i2s2_asrc_parents,				/* parents */
    0xA1C,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 5, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

/* OWA_TX */
static const char *spdif_tx_parents[] = { "pll_audio0_1x", "pll_audio0_4x",
    "pll_audio1_div2", "pll_audio1_div5" };
NM_CLK(spdif_tx_clk,
    CLK_SPDIF_TX, "spdif_tx", spdif_tx_parents,	/* id, name, parents */
    0xA24,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 5, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

/* OWA_RX */
static const char *spdif_rx_parents[] = { "pll_peri_1x", "pll_audio1_div2",
    "pll_audio1_div5" };
NM_CLK(spdif_rx_clk,
    CLK_SPDIF_RX, "spdif_rx", spdif_rx_parents,	/* id, name, parents */
    0xA28,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 5, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *dmic_parents[] = { "pll_audio0_1x", "pll_audio1_div2",
    "pll_audio1_div5" };
NM_CLK(dmic_clk,
    CLK_DMIC, "dmic", dmic_parents,		/* id, name, parents */
    0xA40,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 5, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *audio_dac_parents[] = { "pll_audio0_1x", "pll_audio1_div2",
    "pll_audio1_div5" };
NM_CLK(audio_dac_clk,
    CLK_AUDIO_DAC,				/* id */
    "audio_dac",				/* name */
    audio_dac_parents,				/* parents */
    0xA50,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 5, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *audio_adc_parents[] = { "pll_audio0_1x", "pll_audio1_div2",
    "pll_audio1_div5" };
NM_CLK(audio_adc_clk,
    CLK_AUDIO_ADC,				/* id */
    "audio_adc",				/* name */
    audio_adc_parents,				/* parents */
    0xA54,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 5, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

/* TODO USB0 */

/* TODO USB1 */

static const char *dsi_parents[] = { HOSC, "pll_peri_1x", "pll_video0_2x",
    "pll_video1_2x", "pll_audio1_div2" };
M_CLK(dsi_clk,
    CLK_DSI, "dsi", dsi_parents,		/* id, name, parents */
    0xB24,					/* offset */
    0, 4, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *tconlcd_parents[] = { "pll_video0_1x", "pll_video0_4x",
    "pll_video1_1x", "pll_video1_4x", "pll_peri_2x", "pll_audio1_div2" };
NM_CLK(tconlcd_clk,
    CLK_TCONLCD, "tconlcd", tconlcd_parents,	/* id, name, parents */
    0xB60,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 4, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *tcontv_parents[] = { "pll_video0_1x", "pll_video0_4x",
    "pll_video1_1x", "pll_video1_4x", "pll_peri_2x", "pll_audio1_div2" };
NM_CLK(tcontv_clk,
    CLK_TCONTV, "tcontv", tcontv_parents,	/* id, name, parents */
    0xB80,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 4, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *tve_parents[] = { "pll_video0_1x", "pll_video0_4x",
    "pll_video1_1x", "pll_video1_4x", "pll_peri_2x", "pll_audio1_div2" };
NM_CLK(tve_clk,
    CLK_TVE, "tve", tve_parents,		/* id, name, parents */
    0xBB0,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 4, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *tvd_parents[] = { HOSC, "pll_video0_1x", "pll_video1_1x",
    "pll_peri_1x" };
M_CLK(tvd_clk,
    CLK_TVD, "tvd", tvd_parents,		/* id, name, parents */
    0xBC0,					/* offset */
    0, 5, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *ledc_parents[] = { HOSC, "pll_peri_1x" };
NM_CLK(ledc_clk,
    CLK_LEDC, "ledc", ledc_parents,		/* id, name, parents */
    0xBF0,					/* offset */
    8, 2, 0, AW_CLK_FACTOR_POWER_OF_TWO,	/* n factor */
    0, 4, 0, 0,					/* m factor */
    24, 1,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *csi_parents[] = { "pll_peri_2x", "pll_video0_2x",
    "pll_video1_2x" };
M_CLK(csi_clk,
    CLK_CSI, "csi", csi_parents,		/* id, name, parents */
    0xC04,					/* offset */
    0, 4, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *csi_master_parents[] = { HOSC, "pll_peri_1x",
    "pll_video0_1x", "pll_video1_1x", "pll_audio1_div2", "pll_audio1_div5" };
M_CLK(csi_master_clk,
    CLK_CSI_MASTER,				/* id */
    "csi_master",				/* name */
    csi_master_parents,				/* parents */
    0xC08,					/* offset */
    0, 5, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

/* Use M_CLK to have mux and gate */
static const char *tpadc_parents[] = { HOSC, "pll_audio0_1x" };
M_CLK(tpadc_clk,
    CLK_TPADC, "tpadc", tpadc_parents,		/* id, name, parents */
    0xC50,					/* offset */
    0, 0, 1, AW_CLK_FACTOR_FIXED,		/* m factor */
    24, 2,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

static const char *dsp_parents[] = { HOSC, CLK32K, CLK16M_RC,
    "pll_peri_2x", "pll_audio1_div2" };
M_CLK(dsp_clk,
    CLK_DSP, "dsp", dsp_parents,		/* id, name, parents */
    0xC70,					/* offset */
    0, 5, 0, 0,					/* m factor */
    24, 3,					/* mux */
    31,						/* gate */
    AW_CLK_HAS_GATE | AW_CLK_HAS_MUX |
    AW_CLK_REPARENT);				/* flags */

/* TODO RISCV and RISCV_AXI */

/* TODO FANOUT */

static struct aw_ccung_clk d1_ccu_clks[] = {
	{ .type = AW_CLK_NP,	.clk.np		= &pll_cpu_clk },
	{ .type = AW_CLK_NMM,	.clk.nmm	= &pll_ddr_clk },
	{ .type = AW_CLK_NP,	.clk.np		= &pll_peri_clk },
	{ .type = AW_CLK_M,	.clk.m		= &pll_peri_2x_clk },
	{ .type = AW_CLK_FIXED,	.clk.fixed	= &pll_peri_1x_clk },
	{ .type = AW_CLK_M,	.clk.m		= &pll_peri_800m_clk },
	{ .type = AW_CLK_NP,	.clk.np		= &pll_video0_clk },
	{ .type = AW_CLK_M,	.clk.m		= &pll_video0_4x_clk },
	{ .type = AW_CLK_FIXED,	.clk.fixed	= &pll_video0_2x_clk },
	{ .type = AW_CLK_FIXED,	.clk.fixed	= &pll_video0_1x_clk },
	{ .type = AW_CLK_NP,	.clk.np		= &pll_video1_clk },
	{ .type = AW_CLK_M,	.clk.m		= &pll_video1_4x_clk },
	{ .type = AW_CLK_FIXED,	.clk.fixed	= &pll_video1_2x_clk },
	{ .type = AW_CLK_FIXED,	.clk.fixed	= &pll_video1_1x_clk },
	{ .type = AW_CLK_NMM,	.clk.nmm	= &pll_ve_clk },
	{ .type = AW_CLK_NMM,	.clk.nmm	= &pll_audio0_clk },
	{ .type = AW_CLK_M,	.clk.m		= &pll_audio0_4x_clk },
	{ .type = AW_CLK_FIXED,	.clk.fixed	= &pll_audio0_2x_clk },
	{ .type = AW_CLK_FIXED,	.clk.fixed	= &pll_audio0_1x_clk },
	{ .type = AW_CLK_NP,	.clk.np		= &pll_audio1_clk },
	{ .type = AW_CLK_M,	.clk.m		= &pll_audio1_div2_clk },
	{ .type = AW_CLK_M,	.clk.m		= &pll_audio1_div5_clk },
	{ .type = AW_CLK_M,	.clk.m		= &pll_cpu_p_clk },
	{ .type = AW_CLK_M,	.clk.m		= &cpu_axi_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &psi_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &apb0_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &apb1_clk },
	{ .type = AW_CLK_FIXED,	.clk.fixed	= &mbus_clk },
	{ .type = AW_CLK_M,	.clk.m		= &de_clk },
	{ .type = AW_CLK_M,	.clk.m		= &di_clk },
	{ .type = AW_CLK_M,	.clk.m		= &g2d_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &ce_clk },
	{ .type = AW_CLK_M,	.clk.m		= &ve_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &dram_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &mmc0_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &mmc1_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &mmc2_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &spi0_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &spi1_clk },
	{ .type = AW_CLK_M,	.clk.m		= &emac_25m_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &irtx_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &i2s0_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &i2s1_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &i2s2_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &i2s2_asrc_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &spdif_tx_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &spdif_rx_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &dmic_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &audio_dac_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &audio_adc_clk },
	{ .type = AW_CLK_M,	.clk.m		= &dsi_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &tconlcd_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &tcontv_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &tve_clk },
	{ .type = AW_CLK_M,	.clk.m		= &tvd_clk },
	{ .type = AW_CLK_NM,	.clk.nm		= &ledc_clk },
	{ .type = AW_CLK_M,	.clk.m		= &csi_clk },
	{ .type = AW_CLK_M,	.clk.m		= &csi_master_clk },
	{ .type = AW_CLK_M,	.clk.m		= &tpadc_clk },
	{ .type = AW_CLK_M,	.clk.m		= &dsp_clk },
};

static int
d1_ccu_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "allwinner,sun20i-d1-ccu"))
		return (ENXIO);

	device_set_desc(dev, "Allwinner D1 Clock Controller Unit");
	return (BUS_PROBE_DEFAULT);
}

static int
d1_ccu_attach(device_t dev)
{
	struct aw_ccung_softc *sc;

	sc = device_get_softc(dev);

	sc->resets = d1_ccu_resets;
	sc->nresets = nitems(d1_ccu_resets);
	sc->gates = d1_ccu_gates;
	sc->ngates = nitems(d1_ccu_gates);
	sc->clks = d1_ccu_clks;
	sc->nclks = nitems(d1_ccu_clks);

	return (aw_ccung_attach(dev));
}

static device_method_t d1_ccu_methods[] = {
	DEVMETHOD(device_probe,		d1_ccu_probe),
	DEVMETHOD(device_attach,	d1_ccu_attach),

	DEVMETHOD_END
};

static devclass_t d1_ccu_devclass;

DEFINE_CLASS_1(d1_ccu, d1_ccu_driver, d1_ccu_methods,
  sizeof(struct aw_ccung_softc), aw_ccung_driver);

EARLY_DRIVER_MODULE(d1_ccu, simplebus, d1_ccu_driver,
    d1_ccu_devclass, 0, 0, BUS_PASS_RESOURCE + BUS_PASS_ORDER_MIDDLE);
