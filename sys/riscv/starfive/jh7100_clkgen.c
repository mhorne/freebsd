/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright 2016 Michal Meloun <mmel@FreeBSD.org>
 * Copyright (c) 2020 Oskar Holmlund <oskar.holmlund@ohdata.se>
 * Copyright (c) 2022 Mitchell Horne <mhorne@FreeBSD.org>
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
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/fbio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/resource.h>

#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/clk/clk_div.h>
#include <dev/extres/clk/clk_gate.h>
#include <dev/extres/clk/clk_fixed.h>
#include <dev/extres/clk/clk_mux.h>

#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dt-bindings/clock/starfive-jh7100.h>

#include <riscv/starfive/jh7100_clk.h>

#include "clkdev_if.h"

#if 0
#define DPRINTF(dev, msg...) device_printf(dev, msg)
#else
#define DPRINTF(dev, msg...)
#endif

/* Documentation/devicetree/bindings/clock/ti-clkctrl.txt */

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7100-clkgen",	1 },
	{ NULL,				0 }
};

struct jh7100_clkgen_softc {
	struct resource		*res;
	int			*rid;
	struct clkdom		*clkdom;
	struct mtx		mtx;
};

static struct resource_spec res_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	RESOURCE_SPEC_END
};

#define	RD4(sc, reg)		bus_read_4(&(sc)->res[0], (reg))
#define	WR4(sc, reg, val)	bus_write_4(&(sc)->res[0], (reg), (val))

static const char *pll2_refclk_p[] = { "osc_sys", "osc_aud" };
static const char *perh0_root_p[] = { "osc_sys", "pll0_out" };
static const char *perh1_root_p[] = { "osc_sys", "pll2_out" };
static const char *cpundbus_root_p[] = { "osc_sys", "pll0_out", "pll1_out", "pll2_out" };
static const char *gmacusb_root_p[] = { "osc_sys", "pll0_out", "pll2_out" };
//static const char *gmac_tx_p[] = { "gmac_gtx", "gmac_tx_inv", "gmac_rmii_tx" };
//static const char *gmac_rx_pre_p[] = { "gmac_gr_mii_rx", "gmac_rmii_rx" };
#if 0
static struct clk_mux_def mux_clks[] = {
	JH7100_MUX(JH7100_CLK_PLL2_REF, "pll2_refclk", pll2_refclk_p),
	JH7100_MUX(JH7100_CLK_PERH0_ROOT, "perh0_root", perh0_root_p),
	JH7100_MUX(JH7100_CLK_PERH1_ROOT, "perh1_root", perh1_root_p),
	JH7100_MUX(JH7100_CLK_CPUNDBUS_ROOT, "cpundbus_root", cpundbus_root_p),
	JH7100_MUX(JH7100_CLK_GMACUSB_ROOT, "gmacusb_root", gmacusb_root_p),
};
#endif

static const char *perh0_src_p[] = { "perh0_root" };
static const char *perh1_src_p[] = { "perh1_root" };
static const char *uart0_core_p[] = { "perh1_src" };
static const char *uart1_core_p[] = { "perh1_src" };
static const char *uart2_core_p[] = { "perh0_src" };
static const char *uart3_core_p[] = { "perh0_src" };
static const char *cpunbus_root_div_p[] = { "cpundbus_root" };
static const char *ahb_bus_p[] = { "cpunbus_root_div" };
static const char *apb1_bus_p[] = { "ahb_bus" };
static const char *apb2_bus_p[] = { "ahb_bus" };
static const char *gmac_root_div_p[] = { "gmacusb_root" };
static const char *gmac_ptp_refclk_p[] = { "gmac_root_div" };
#if 0
static struct clk_div_def div_clks[] = {
	JH7100_DIV(JH7100_CLK_PERH0_SRC, "perh0_src", perh0_src_p),
	JH7100_DIV(JH7100_CLK_PERH1_SRC, "perh1_src", perh1_src_p),
	JH7100_DIV(JH7100_CLK_UART0_CORE, "uart0_core", uart0_core_p),
	JH7100_DIV(JH7100_CLK_UART1_CORE, "uart1_core", uart1_core_p),
	JH7100_DIV(JH7100_CLK_CPUNBUS_ROOT_DIV, "cpunbus_root_div", cpunbus_root_div_p),
	JH7100_DIV(JH7100_CLK_AHB_BUS, "ahb_bus", ahb_bus_p),
	JH7100_DIV(JH7100_CLK_APB1_BUS, "apb1_bus", apb1_bus_p),
	JH7100_DIV(JH7100_CLK_APB2_BUS, "apb2_bus", apb2_bus_p),
	JH7100_DIV(JH7100_CLK_GMAC_ROOT_DIV, "gmac_root_div", gmac_root_div_p),
	JH7100_DIV(JH7100_CLK_GMAC_PTP_REF, "gmac_ptp_refclk", gmac_ptp_refclk_p),
};
#endif

static const char *uartN_apb_p[] = { "apb1_bus" };
static const char *sdio0_ahb_p[] = { "ahb_bus" };
static const char *sdio0_cclkint_p[] = { "perh0_src" };
static const char *sdio1_ahb_p[] = { "ahb_bus" };
static const char *sdio1_cclkint_p[] = { "perh1_src" };
static const char *gmac_ahb_p[] = { "ahb_bus" };
#if 0
static struct clk_gate_def gate_clks[] = {
	JH7100_GATE(JH7100_CLK_UART0_APB, "uart0_apb", uart0_apb_p),
	JH7100_GATE(JH7100_CLK_UART1_APB, "uart1_apb", uart1_apb_p),
	JH7100_GATE(JH7100_CLK_SDIO0_AHB, "sdio0_ahb", sdio0_ahb_p),
	JH7100_GATE(JH7100_CLK_SDIO0_CCLKINT, "sdio0_cclkint", sdio0_cclkint_p),
	JH7100_GATE(JH7100_CLK_SDIO1_AHB, "sdio1_ahb", sdio1_ahb_p),
	JH7100_GATE(JH7100_CLK_SDIO1_CCLKINT, "sdio1_cclkint", sdio1_cclkint_p),
	JH7100_GATE(JH7100_CLK_GMAC_AHB, "gmac_ahb", gmac_ahb_p),
};
#endif
static const char *gmac_gtx_p[] = { "gmac_root_div" };
//static const char *gmac_rmii_tx_p[] = { "gmac_rmii_ref" };
//static const char *gmac_rmii_rx_p[] = { "gmac_rmii_ref" };

static const struct jh7100_clk_def jh7100_clks[] = {
	JH7100_MUX(JH7100_CLK_PLL2_REF, "pll2_refclk", pll2_refclk_p),
	JH7100_MUX(JH7100_CLK_PERH0_ROOT, "perh0_root", perh0_root_p),
	JH7100_MUX(JH7100_CLK_PERH1_ROOT, "perh1_root", perh1_root_p),
	JH7100_MUX(JH7100_CLK_CPUNDBUS_ROOT, "cpundbus_root", cpundbus_root_p),
	JH7100_MUX(JH7100_CLK_GMACUSB_ROOT, "gmacusb_root", gmacusb_root_p),
	//JH7100_MUX(JH7100_CLK_GMAC_TX, "gmac_tx", gmac_tx_p),
	//JH7100_MUX(JH7100_CLK_GMAC_RX_PRE, "gmac_rx_pre", gmac_rx_pre_p),

	JH7100_GATE(JH7100_CLK_UART0_APB, "uart0_apb", uartN_apb_p),
	JH7100_GATE(JH7100_CLK_UART1_APB, "uart1_apb", uartN_apb_p),
	JH7100_GATE(JH7100_CLK_UART2_APB, "uart2_apb", uartN_apb_p),
	JH7100_GATE(JH7100_CLK_UART3_APB, "uart3_apb", uartN_apb_p),
	JH7100_GATE(JH7100_CLK_SDIO0_AHB, "sdio0_ahb", sdio0_ahb_p),
	JH7100_GATE(JH7100_CLK_SDIO1_AHB, "sdio1_ahb", sdio1_ahb_p),
	JH7100_GATE(JH7100_CLK_GMAC_AHB, "gmac_ahb", gmac_ahb_p),

	JH7100_DIV(JH7100_CLK_PERH0_SRC, "perh0_src", perh0_src_p),
	JH7100_DIV(JH7100_CLK_PERH1_SRC, "perh1_src", perh1_src_p),
	JH7100_DIV(JH7100_CLK_UART0_CORE, "uart0_core", uart0_core_p),
	JH7100_DIV(JH7100_CLK_UART1_CORE, "uart1_core", uart1_core_p),
	JH7100_DIV(JH7100_CLK_UART2_CORE, "uart2_core", uart2_core_p),
	JH7100_DIV(JH7100_CLK_UART3_CORE, "uart3_core", uart3_core_p),
	JH7100_DIV(JH7100_CLK_CPUNBUS_ROOT_DIV, "cpunbus_root_div", cpunbus_root_div_p),
	JH7100_DIV(JH7100_CLK_AHB_BUS, "ahb_bus", ahb_bus_p),
	JH7100_DIV(JH7100_CLK_APB1_BUS, "apb1_bus", apb1_bus_p),
	JH7100_DIV(JH7100_CLK_APB2_BUS, "apb2_bus", apb2_bus_p),
	JH7100_DIV(JH7100_CLK_GMAC_ROOT_DIV, "gmac_root_div", gmac_root_div_p),
	JH7100_DIV(JH7100_CLK_GMAC_PTP_REF, "gmac_ptp_refclk", gmac_ptp_refclk_p),

	JH7100_GATEDIV(JH7100_CLK_GMAC_GTX, "gmac_gtx", gmac_gtx_p),
	JH7100_GATEDIV(JH7100_CLK_SDIO0_CCLKINT, "sdio0_cclkint", sdio0_cclkint_p),
	JH7100_GATEDIV(JH7100_CLK_SDIO1_CCLKINT, "sdio1_cclkint", sdio1_cclkint_p),
	//JH7100_GATEDIV(JH7100_CLK_GMAC_RMII_TX, "gmac_rmii_tx", gmac_rmii_tx_p),
	//JH7100_GATEDIV(JH7100_CLK_GMAC_RMII_RX, "gmac_rmii_rx", gmac_rmii_rx_p),
};

static const char *pll_parents[] = { "osc_sys" };
static const char *pll2_parents[] = { "pll2_refclk" };
static const char *sdio0_cclkint_inv_p[] = { "sdio0_cclkint" };
static const char *sdio1_cclkint_inv_p[] = { "sdio1_cclkint" };
//static const char *gmac_tx_inv_p[] = { "gmac_tx" };
//static const char *gmac_rx_inv_p[] = { "gmac_rx_pre" };
static struct clk_fixed_def pll_clks[] = {
	{
		.clkdef.id = JH7100_CLK_PLL0_OUT,
		.clkdef.name = "pll0_out",
		.clkdef.parent_names = pll_parents,
		.clkdef.parent_cnt = nitems(pll_parents),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
		.mult = 40,
		.div = 1,
	},
	{
		.clkdef.id = JH7100_CLK_PLL1_OUT,
		.clkdef.name = "pll1_out",
		.clkdef.parent_names = pll_parents,
		.clkdef.parent_cnt = nitems(pll_parents),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
		.mult = 64,
		.div = 1,
	},
	{
		.clkdef.id = JH7100_CLK_PLL2_OUT,
		.clkdef.name = "pll2_out",
		.clkdef.parent_names = pll2_parents,
		.clkdef.parent_cnt = nitems(pll2_parents),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
		.mult = 55,
		.div = 1,
	},
	{
		.clkdef.id = JH7100_CLK_SDIO0_CCLKINT_INV,
		.clkdef.name = "sdio0_cclkint_inv",
		.clkdef.parent_names = sdio0_cclkint_inv_p,
		.clkdef.parent_cnt = nitems(sdio0_cclkint_inv_p),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
		.mult = 1,
		.div = 1,
	},
	{
		.clkdef.id = JH7100_CLK_SDIO1_CCLKINT_INV,
		.clkdef.name = "sdio1_cclkint_inv",
		.clkdef.parent_names = sdio1_cclkint_inv_p,
		.clkdef.parent_cnt = nitems(sdio1_cclkint_inv_p),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
		.mult = 1,
		.div = 1,
	},
#if 0
	{
		.clkdef.id = JH7100_CLK_GMAC_TX_INV,
		.clkdef.name = "gmac_tx_inv",
		.clkdef.parent_names = gmac_tx_inv_p,
		.clkdef.parent_cnt = nitems(gmac_tx_inv_p),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
		.mult = 1,
		.div = 1,
	},
	{
		.clkdef.id = JH7100_CLK_GMAC_RX_INV,
		.clkdef.name = "gmac_rx_inv",
		.clkdef.parent_names = gmac_rx_inv_p,
		.clkdef.parent_cnt = nitems(gmac_rx_inv_p),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
		.mult = 1,
		.div = 1,
	},
#endif
};

/* Default DT mapper. */
static int
jh7100_ofw_map(struct clkdom *clkdom, uint32_t ncells,
    phandle_t *cells, struct clknode **clk)
{

	if (ncells == 1)
		/* TODO: Adjust for u-boot device tree discrepancy. */
		*clk = clknode_find_by_id(clkdom, cells[0] - 5);
	else
		return  (ERANGE);

	if (*clk == NULL)
		return (ENXIO);
	return (0);
}

static int
jh7100_clkgen_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive Clock Controller");

	return (BUS_PROBE_DEFAULT);
}

static void
jh7100_print_clkreg(const char *name, uint32_t reg)
{
#define	JH7100_DIV_MASK		0xff
#define	JH7100_DIV_SHIFT	0
#define	JH7100_FRAC_MASK	0xff
#define	JH7100_FRAC_SHIFT	8
#define	JH7100_MUX_SHIFT	24
#define	JH7100_MUX_MASK		0xf
#define	JH7100_ENABLE_SHIFT	31

	int enable = reg >> JH7100_ENABLE_SHIFT;
	uint32_t mux = (reg >> 24) & 0xf;
	uint32_t divider = reg & JH7100_DIV_MASK;
	uint32_t frac = (reg >> 8) & 0xff;

	printf("Register clock %s:\n", name);
	printf("\treg: 0x%0x\n", reg);
	printf("\tenable: %d\n", enable);
	printf("\tmux:  %d\n", mux);
	printf("\tfrac: %u\n", frac);
	printf("\tdiv:  %u\n", divider);
}

static int
jh7100_clkgen_attach(device_t dev)
{
	struct jh7100_clkgen_softc *sc;
	phandle_t node;
	cell_t reg;
	int i;
	int error;

	sc = device_get_softc(dev);

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	error = bus_alloc_resources(dev, res_spec, &sc->res);
	if (error) {
		device_printf(dev, "Couldn't allocate resources\n");
		return (ENXIO);
	}

	node = ofw_bus_get_node(dev);

	/* Sanity check */
	error = OF_searchencprop(node, "#clock-cells", &reg, sizeof(reg));
	if (error == -1) {
		device_printf(dev, "Failed to get #clock-cells\n");
		return (ENXIO);
	}
	if (reg != 1) {
		device_printf(dev, "clock cells(%d) != 1\n", reg);
		return (ENXIO);
	}

	/* Create clock domain */
	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL) {
		//free(reg, M_DEVBUF);
		DPRINTF(dev, "Failed to create clkdom\n");
		return (ENXIO);
	}
	clkdom_set_ofw_mapper(sc->clkdom, jh7100_ofw_map);

	for (i = 0; i < nitems(pll_clks); i++) {
		error = clknode_fixed_register(sc->clkdom, &pll_clks[i]);
		if (error != 0)
			device_printf(dev, "Failed to register clock %s: %d\n",
			    pll_clks[i].clkdef.name, error);
		else
			printf("%s reg\t%x\n", pll_clks[i].clkdef.name, RD4(sc, pll_clks[i].clkdef.id * sizeof(uint32_t)));
	}

#if 0
	for (i = 0; i < nitems(mux_clks); i++) {
		error = clknode_mux_register(sc->clkdom, &mux_clks[i]);
		if (error != 0)
			device_printf(dev, "Failed to register clock %s: %d\n",
			    mux_clks[i].clkdef.name, error);
		else
			printf("%s reg\t%x\n", mux_clks[i].clkdef.name, RD4(sc, mux_clks[i].offset));
	}

	for (i = 0; i < nitems(div_clks); i++) {
		error = clknode_div_register(sc->clkdom, &div_clks[i]);
		if (error != 0)
			device_printf(dev, "Failed to register clock %s: %d\n",
			    div_clks[i].clkdef.name, error);
		else
			printf("%s reg\t%x\n", div_clks[i].clkdef.name, RD4(sc, div_clks[i].offset));
	}

	for (i = 0; i < nitems(gate_clks); i++) {
		error = clknode_gate_register(sc->clkdom, &gate_clks[i]);
		if (error != 0)
			device_printf(dev, "Failed to register clock %s: %d\n",
			    gate_clks[i].clkdef.name, error);
		else
			printf("%s reg\t%x\n", gate_clks[i].clkdef.name, RD4(sc, gate_clks[i].offset));
	}
#endif

	for (i = 0; i < nitems(jh7100_clks); i++) {
		error = jh7100_clk_register(sc->clkdom, &jh7100_clks[i]);
		if (error != 0)
			device_printf(dev, "Failed to register clock %s: %d\n",
			    jh7100_clks[i].clkdef.name, error);
		else
			//printf("%s reg\t0x%0x\n", jh7100_clks[i].clkdef.name, RD4(sc, jh7100_clks[i].offset));
			jh7100_print_clkreg(jh7100_clks[i].clkdef.name, RD4(sc, jh7100_clks[i].offset));

	}

	error = clkdom_finit(sc->clkdom);
	if (error) {
		DPRINTF(dev, "Clk domain finit fails %x.\n", error);
		return (ENXIO);
		//err = ENXIO;
		//goto cleanup;
	}

	//if (bootverbose)
		clkdom_dump(sc->clkdom);

	breakpoint();

	return (0);
}

static int
jh7100_clkgen_detach(device_t dev)
{
	return (EBUSY);
}

static int
jh7100_clkgen_read_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
	struct jh7100_clkgen_softc *sc;

	sc = device_get_softc(dev);

	*val = RD4(sc, addr);
	return (0);
}

static int
jh7100_clkgen_write_4(device_t dev, bus_addr_t addr, uint32_t val)
{
	struct jh7100_clkgen_softc *sc;

	sc = device_get_softc(dev);
	WR4(sc, addr, val);
	return (0);
}

static int
jh7100_clkgen_modify_4(device_t dev, bus_addr_t addr, uint32_t clr, uint32_t set)
{
	struct jh7100_clkgen_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);

	reg = RD4(sc, addr);
	reg &= ~clr;
	reg |= set;
	WR4(sc, addr, reg);

	return (0);
}

static void
jh7100_clkgen_device_lock(device_t dev)
{
	struct jh7100_clkgen_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
jh7100_clkgen_device_unlock(device_t dev)
{
	struct jh7100_clkgen_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static device_method_t jh7100_clkgen_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		jh7100_clkgen_probe),
	DEVMETHOD(device_attach,	jh7100_clkgen_attach),
	DEVMETHOD(device_detach,	jh7100_clkgen_detach),

	/* clkdev interface */
	DEVMETHOD(clkdev_read_4,	jh7100_clkgen_read_4),
	DEVMETHOD(clkdev_write_4,	jh7100_clkgen_write_4),
	DEVMETHOD(clkdev_modify_4,	jh7100_clkgen_modify_4),
	DEVMETHOD(clkdev_device_lock,	jh7100_clkgen_device_lock),
	DEVMETHOD(clkdev_device_unlock,	jh7100_clkgen_device_unlock),

	DEVMETHOD_END
};

DEFINE_CLASS_0(jh7100_clkgen, jh7100_clkgen_driver, jh7100_clkgen_methods,
    sizeof(struct jh7100_clkgen_softc));

EARLY_DRIVER_MODULE(jh7100_clkgen, simplebus, jh7100_clkgen_driver,
0, 0, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);

/* TODO: MODULE_DEPEND? */
MODULE_VERSION(jh7100_clkgen, 1);
