/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2024 Jari Sihvola <jsihv@gmx.com>
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
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/fbio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/resource.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dt-bindings/clock/starfive,jh7110-crg.h>

#include <dev/clk/starfive/jh7110_clk_pll.h>
#include <dev/clk/starfive/jh7110_clk.h>
#include <dev/clk/clk.h>

#include "clkdev_if.h"

#define PLL_OFFSET_0_			0x18
#define PLL_OFFSET_1_			0x1c
#define PLL_OFFSET_2_			0x20
#define PLL_OFFSET_3_			0x24
#define PLL_OFFSET_4_			0x28
#define PLL_OFFSET_5_			0x2c
#define PLL_OFFSET_6_			0x30
#define PLL_OFFSET_7_			0x34

#define	DEVICE_LOCK(_clk)				\
	CLKDEV_DEVICE_LOCK(clknode_get_device(_clk))
#define	DEVICE_UNLOCK(_clk)				\
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(_clk))

#define	SYSCON_READ_4(reg)                              \
	bus_read_4(sc->syscon_mem_res, (reg))
#define	SYSCON_MODIFY_4(reg, cbits, sbits)		\
	jh7110_syscon_modify_4(sc->syscon_mem_res, (reg), (cbits), (sbits))

#define PLL_MASK_FILL(sc, id)					\
do {								\
	sc->dacpd_mask = PLL## id ##_DACPD_MASK;		\
	sc->dsmpd_mask = PLL## id ##_DSMPD_MASK;		\
	sc->fbdiv_mask = PLL## id ##_FBDIV_MASK;		\
	sc->frac_mask = PLL## id ##_FRAC_MASK;			\
	sc->prediv_mask = PLL## id ##_PREDIV_MASK;		\
	sc->postdiv1_mask = PLL## id ##_POSTDIV1_MASK;		\
} while (0)

#define PLL_SHIFT_FILL(sc, id)					\
do {								\
	sc->dacpd_shift = PLL## id ##_DACPD_SHIFT;		\
	sc->dsmpd_shift = PLL## id ##_DSMPD_SHIFT;		\
	sc->fbdiv_shift = PLL## id ##_FBDIV_SHIFT;		\
	sc->frac_shift = PLL## id ##_FRAC_SHIFT;		\
	sc->prediv_shift = PLL## id ##_PREDIV_SHIFT;		\
	sc->postdiv1_shift = PLL## id ##_POSTDIV1_SHIFT;	\
} while (0)

#define PLL_OFFSET_FILL(sc, val1, val2, val3, val4, val5, val6)	\
do {								\
	sc->dacpd_offset = PLL_OFFSET_## val1 ##_ ;		\
	sc->dsmpd_offset = PLL_OFFSET_## val2 ##_ ;		\
	sc->fbdiv_offset = PLL_OFFSET_## val3 ##_ ;		\
	sc->frac_offset = PLL_OFFSET_## val4 ##_ ;		\
	sc->prediv_offset = PLL_OFFSET_## val5 ##_ ;		\
	sc->postdiv1_offset = PLL_OFFSET_## val6 ##_ ;		\
} while (0)

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-sys-syscon", 1 },
	{ NULL,				0 }
};

static struct resource_spec res_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE | RF_SHAREABLE },
	RESOURCE_SPEC_END
};

struct jh7110_clk_pll_softc {
	struct mtx		mtx;
	struct clkdom		*clkdom;
	struct resource		*syscon_mem_res;
};

struct jh7110_pll_clknode_softc {
	uint32_t	dacpd_offset;
	uint32_t	dsmpd_offset;
	uint32_t	fbdiv_offset;
	uint32_t	frac_offset;
	uint32_t	prediv_offset;
	uint32_t	postdiv1_offset;

	uint32_t	dacpd_mask;
	uint32_t	dsmpd_mask;
	uint32_t	fbdiv_mask;
	uint32_t	frac_mask;
	uint32_t	prediv_mask;
	uint32_t	postdiv1_mask;

	uint32_t	dacpd_shift;
	uint32_t	dsmpd_shift;
	uint32_t	fbdiv_shift;
	uint32_t	frac_shift;
	uint32_t	prediv_shift;
	uint32_t	postdiv1_shift;

	const struct jh7110_pll_syscon_value *syscon_arr;
	int		syscon_nitems;
};

static const char *pll_parents[] = { "osc" };

static struct jh7110_clk_def pll_out_clks[] = {
	{
		.clkdef.id = JH7110_PLLCLK_PLL0_OUT,
		.clkdef.name = "pll0_out",
		.clkdef.parent_names = pll_parents,
		.clkdef.parent_cnt = nitems(pll_parents),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
	},
	{
		.clkdef.id = JH7110_PLLCLK_PLL1_OUT,
		.clkdef.name = "pll1_out",
		.clkdef.parent_names = pll_parents,
		.clkdef.parent_cnt = nitems(pll_parents),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
	},
	{
		.clkdef.id = JH7110_PLLCLK_PLL2_OUT,
		.clkdef.name = "pll2_out",
		.clkdef.parent_names = pll_parents,
		.clkdef.parent_cnt = nitems(pll_parents),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
	},
};

static int jh7110_clk_pll_register(struct clkdom *clkdom,
				   struct jh7110_clk_def *clkdef);

static int
jh7110_syscon_modify_4(struct resource *res, bus_size_t offset,
		       uint32_t clear_bits, uint32_t set_bits)
{
	uint32_t val;
	uint32_t new_val;

	val = bus_read_4(res, offset);

	new_val = (val & ~clear_bits) | set_bits;

	if (new_val != val)
		bus_write_4(res, offset, new_val);

	return (0);
}

static int
jh7110_clk_pll_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	struct jh7110_clk_pll_softc *sc;
	struct jh7110_pll_clknode_softc *clk_sc;
	uint32_t dacpd, dsmpd, fbdiv, prediv, postdiv1;
	uint64_t frac, fcal = 0;

	sc = device_get_softc(clknode_get_device(clk));
	clk_sc = clknode_get_softc(clk);

	DEVICE_LOCK(clk);

	dacpd = (SYSCON_READ_4(clk_sc->dacpd_offset) & clk_sc->dacpd_mask) >>
		clk_sc->dacpd_shift;
	dsmpd = (SYSCON_READ_4(clk_sc->dsmpd_offset) & clk_sc->dsmpd_mask) >>
		clk_sc->dsmpd_shift;
	fbdiv = (SYSCON_READ_4(clk_sc->fbdiv_offset) & clk_sc->fbdiv_mask) >>
		clk_sc->fbdiv_shift;
	prediv = (SYSCON_READ_4(clk_sc->prediv_offset) & clk_sc->prediv_mask) >>
		clk_sc->prediv_shift;
	postdiv1 = (SYSCON_READ_4(clk_sc->postdiv1_offset) &
		    clk_sc->postdiv1_mask) >> clk_sc->postdiv1_shift;
	frac = (SYSCON_READ_4(clk_sc->frac_offset) & clk_sc->frac_mask) >>
		clk_sc->frac_shift;

	DEVICE_UNLOCK(clk);

	/* dacpd and dsmpd both being 0 entails Fraction Multiple Mode */
	if (dacpd == 0 && dsmpd == 0)
		fcal = frac * FRAC_PATR_SIZE / (1 << 24);

	*freq = *freq / FRAC_PATR_SIZE * (fbdiv * FRAC_PATR_SIZE +
					  fcal) / prediv / (1 << postdiv1);

	return (0);
}

static int
jh7110_clk_pll_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *done)
{
	struct jh7110_clk_pll_softc *sc;
	struct jh7110_pll_clknode_softc *clk_sc;
	const struct jh7110_pll_syscon_value *syscon_val = NULL;

	sc = device_get_softc(clknode_get_device(clk));
	clk_sc = clknode_get_softc(clk);

	for (int i = 0; i != clk_sc->syscon_nitems; i++) {
		if (*fout == clk_sc->syscon_arr[i].freq) {
			syscon_val = &clk_sc->syscon_arr[i];
		}
	}

	if (syscon_val == NULL) {
		printf("%s: tried to set an unknown frequency %ju for %s\n",
		       __func__, *fout, clknode_get_name(clk));
		return (EINVAL);
	}

	if (flags & CLK_SET_DRYRUN) {
		*done = 1;
		return (0);
	}

	DEVICE_LOCK(clk);

	SYSCON_MODIFY_4(clk_sc->dacpd_offset, clk_sc->dacpd_mask,
		syscon_val->dacpd << clk_sc->dacpd_shift & clk_sc->dacpd_mask);
	SYSCON_MODIFY_4(clk_sc->dsmpd_offset, clk_sc->dsmpd_mask,
		syscon_val->dsmpd << clk_sc->dsmpd_shift & clk_sc->dsmpd_mask);
	SYSCON_MODIFY_4(clk_sc->prediv_offset, clk_sc->prediv_mask,
		syscon_val->prediv << clk_sc->prediv_shift & clk_sc->prediv_mask);
	SYSCON_MODIFY_4(clk_sc->fbdiv_offset, clk_sc->fbdiv_mask,
		syscon_val->fbdiv << clk_sc->fbdiv_shift & clk_sc->fbdiv_mask);
	SYSCON_MODIFY_4(clk_sc->postdiv1_offset,
			clk_sc->postdiv1_mask, (syscon_val->postdiv1 >> 1)
			<< clk_sc->postdiv1_shift & clk_sc->postdiv1_mask);

	if (!syscon_val->dacpd && !syscon_val->dsmpd) {
		SYSCON_MODIFY_4(clk_sc->frac_offset, clk_sc->frac_mask,
			syscon_val->frac << clk_sc->frac_shift & clk_sc->frac_mask);
	}

	DEVICE_UNLOCK(clk);

	*done = 1;

	return (0);
}

static int
jh7110_clk_pll_init(struct clknode *clk, device_t dev)
{
	clknode_init_parent_idx(clk, 0);

	return (0);
}

static int
jh7110_clk_pll_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 pll clock generator");

	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_clk_pll_attach(device_t dev)
{
	struct jh7110_clk_pll_softc *sc;
	int i, error;

	sc = device_get_softc(dev);

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	error = bus_alloc_resources(dev, res_spec, &sc->syscon_mem_res);
	if (error) {
		device_printf(dev, "Couldn't allocate resources\n");
		return (ENXIO);
	}

	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL) {
		device_printf(dev, "Couldn't create clkdom\n");
		return (ENXIO);
	}

	for (i = 0; i < nitems(pll_out_clks); i++) {
		error = jh7110_clk_pll_register(sc->clkdom, &pll_out_clks[i]);
		if (error != 0)
			device_printf(dev, "Couldn't register clock %s: %d\n",
			    pll_out_clks[i].clkdef.name, error);
	}

	error = clkdom_finit(sc->clkdom);
	if (error) {
		device_printf(dev, "Clkdom_finit() returns error %d\n", error);
	}

	if (bootverbose)
		clkdom_dump(sc->clkdom);

	return (0);
}

static void
jh7110_clk_pll_device_lock(device_t dev)
{
	struct jh7110_clk_pll_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
jh7110_clk_pll_device_unlock(device_t dev)
{
	struct jh7110_clk_pll_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static clknode_method_t jh7110_pllnode_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		jh7110_clk_pll_init),
	CLKNODEMETHOD(clknode_recalc_freq,	jh7110_clk_pll_recalc_freq),
	CLKNODEMETHOD(clknode_set_freq,		jh7110_clk_pll_set_freq),

	CLKNODEMETHOD_END
};

static device_method_t jh7110_clk_pll_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			jh7110_clk_pll_probe),
	DEVMETHOD(device_attach,		jh7110_clk_pll_attach),

	/* clkdev interface */
	DEVMETHOD(clkdev_device_lock,		jh7110_clk_pll_device_lock),
	DEVMETHOD(clkdev_device_unlock,		jh7110_clk_pll_device_unlock),

	DEVMETHOD_END
};

DEFINE_CLASS_1(jh7110_pllnode, jh7110_pllnode_class, jh7110_pllnode_methods,
    sizeof(struct jh7110_pll_clknode_softc), clknode_class);
DEFINE_CLASS_0(jh7110_clk_pll, jh7110_clk_pll_driver, jh7110_clk_pll_methods,
    sizeof(struct jh7110_clk_pll_softc));
EARLY_DRIVER_MODULE(jh7110_clk_pll, simplebus, jh7110_clk_pll_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_EARLY);
MODULE_VERSION(jh7110_clk_pll, 1);

int
jh7110_clk_pll_register(struct clkdom *clkdom, struct jh7110_clk_def *clkdef)
{
	struct clknode *clk = NULL;
	struct jh7110_pll_clknode_softc *sc;

	clk = clknode_create(clkdom, &jh7110_pllnode_class, &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);

	switch (clkdef->clkdef.id) {
	case JH7110_PLLCLK_PLL0_OUT:
		sc->syscon_arr = jh7110_pll0_syscon_freq;
		sc->syscon_nitems = nitems(jh7110_pll0_syscon_freq);
		PLL_MASK_FILL(sc, 0);
		PLL_SHIFT_FILL(sc, 0);
		PLL_OFFSET_FILL(sc, 0, 0, 1, 2, 3, 2);
		break;
	case JH7110_PLLCLK_PLL1_OUT:
		sc->syscon_arr = jh7110_pll1_syscon_freq;
		sc->syscon_nitems = nitems(jh7110_pll1_syscon_freq);
		PLL_MASK_FILL(sc, 1);
		PLL_SHIFT_FILL(sc, 1);
		PLL_OFFSET_FILL(sc, 3, 3, 3, 4, 5, 4);
		break;
	case JH7110_PLLCLK_PLL2_OUT:
		sc->syscon_arr = jh7110_pll2_syscon_freq;
		sc->syscon_nitems = nitems(jh7110_pll2_syscon_freq);
		PLL_MASK_FILL(sc, 2);
		PLL_SHIFT_FILL(sc, 2);
		PLL_OFFSET_FILL(sc, 5, 5, 5, 6, 7, 6);
		break;
	default:
		return (EINVAL);
	}

	clknode_register(clkdom, clk);

	return (0);
}
