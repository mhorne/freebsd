/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Emmanuel Vadot <manu@freebsd.org>
 * Copyright (c) 2022 Mitchell Horne <mhorne@FreeBSD.org>
 * Copyright (c) 2023 Jari Sihvola <jsihv@gmx.com>
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
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/resource.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

#include <dt-bindings/clock/starfive,jh7110-crg.h>

#include <dev/clk/starfive/jh7110_clk.h>

#include "clkdev_if.h"
#include "hwreset_if.h"

#define	JH7110_DIV_MASK		0xffffff
#define	JH7110_MUX_SHIFT	24
#define	JH7110_MUX_MASK		0x3f000000
#define	JH7110_ENABLE_SHIFT	31
#define REG_SIZE		4

/* offsets for reset registers */
#define AONCRG_RESET_SELECTOR	0x38
#define STGCRG_RESET_SELECTOR	0x74
#define AONCRG_RESET_STATUS	0x3c
#define STGCRG_RESET_STATUS	0x78
#define SYSCRG_RESET_SELECTOR0	0x2f8
#define SYSCRG_RESET_SELECTOR1	0x2fc
#define SYSCRG_RESET_SELECTOR2	0x300
#define SYSCRG_RESET_SELECTOR3	0x304
#define SYSCRG_RESET_STATUS0	0x308
#define SYSCRG_RESET_STATUS1	0x30c
#define SYSCRG_RESET_STATUS2	0x310
#define SYSCRG_RESET_STATUS3	0x314

enum jh7110_reset_crg {
	SYS0,
	SYS1,
	SYS2,
	SYS3,
	STG,
	AON,
};

struct jh7110_clk_sc {
	uint32_t		offset;
	uint32_t		flags;
        uint64_t		d_max;
	int			id;
};

struct jh7110_rstdata {
	int			crg;
	uint64_t		status_offset;
	uint64_t		selector_offset;
};

#define DIV_ROUND_CLOSEST(n, d)  (((n) + (d) / 2) / (d))

#define	READ4(_sc, _off)				\
	bus_read_4(_sc->mem_res, _off)
#define WRITE4(_sc, _off, _val)				\
	bus_write_4(_sc->mem_res, _off, _val)

#define	DEVICE_LOCK(_clk)				\
	CLKDEV_DEVICE_LOCK(clknode_get_device(_clk))
#define	DEVICE_UNLOCK(_clk)				\
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(_clk))

/* Reset functions */

static void
jh7110_reset_io_assign(device_t dev, intptr_t id, struct jh7110_rstdata *iodata)
{
	struct jh7110_clkgen_softc *sc;
	uint32_t crg;

	sc = device_get_softc(dev);

	if (sc->dev_flag == JH7110_CLK_AON) {
		iodata->crg = AON;
		iodata->status_offset = AONCRG_RESET_STATUS;
		iodata->selector_offset = AONCRG_RESET_SELECTOR;

		return;
	}

	crg = id / 32;

	if (crg == SYS0) {
		iodata->crg = SYS0;
		iodata->status_offset = SYSCRG_RESET_STATUS0;
		iodata->selector_offset = SYSCRG_RESET_SELECTOR0;
	} else if (crg == SYS1) {
		iodata->crg = SYS1;
		iodata->status_offset = SYSCRG_RESET_STATUS1;
		iodata->selector_offset = SYSCRG_RESET_SELECTOR1;
	} else if (crg == SYS2) {
		iodata->crg = SYS2;
		iodata->status_offset = SYSCRG_RESET_STATUS2;
		iodata->selector_offset = SYSCRG_RESET_SELECTOR2;
	} else if (crg == SYS3) {
		iodata->crg = SYS3;
		iodata->status_offset = SYSCRG_RESET_STATUS3;
		iodata->selector_offset = SYSCRG_RESET_SELECTOR3;
	} else
		panic("Reset with id %lu, crg %d has no group\n", id, crg);

	return;
}

int
jh7110_reset_assert(device_t dev, intptr_t id, bool assert)
{
	struct jh7110_clkgen_softc *sc;
	struct jh7110_rstdata iodata;
	uint32_t regvalue, bitmask = 1UL << id % 32;

	sc = device_get_softc(dev);

	jh7110_reset_io_assign(dev, id, &iodata);

	mtx_lock(&sc->mtx);

	regvalue = READ4(sc, iodata.selector_offset);

	if (assert)
		regvalue |= bitmask;
	else
		regvalue &= ~bitmask;
	WRITE4(sc, iodata.selector_offset, regvalue);

	mtx_unlock(&sc->mtx);

	return (0);
}

int
jh7110_reset_is_asserted(device_t dev, intptr_t id, bool *reset)
{
	struct jh7110_clkgen_softc *sc;
	struct jh7110_rstdata iodata;
	uint32_t regvalue;
	uint32_t bitmask;

	sc = device_get_softc(dev);

	jh7110_reset_io_assign(dev, id, &iodata);

	mtx_lock(&sc->mtx);

	regvalue = READ4(sc, iodata.status_offset);
	bitmask = 1UL << id % 32;

	mtx_unlock(&sc->mtx);

	*reset = (regvalue & bitmask) == 0;

	return (0);
}

/* Clock functions */

static int
jh7110_clk_init(struct clknode *clk, device_t dev)
{
	struct jh7110_clkgen_softc *sc;
	struct jh7110_clk_sc *sc_clk;
	uint32_t reg;
	int idx = 0;

	sc = device_get_softc(clknode_get_device(clk));
	sc_clk = clknode_get_softc(clk);

	if (sc_clk->flags & JH7110_CLK_HAS_MUX) {
		DEVICE_LOCK(clk);
		reg = READ4(sc, sc_clk->offset);
		DEVICE_UNLOCK(clk);
		idx = (reg & JH7110_MUX_MASK) >> JH7110_MUX_SHIFT;
	}

	clknode_init_parent_idx(clk, idx);

	return (0);
}

static int
jh7110_clk_set_gate(struct clknode *clk, bool enable)
{
	struct jh7110_clkgen_softc *sc;
	struct jh7110_clk_sc *sc_clk;
	uint32_t reg;

	sc = device_get_softc(clknode_get_device(clk));
	sc_clk = clknode_get_softc(clk);

	if ((sc_clk->flags & JH7110_CLK_HAS_GATE) == 0)
		return (0);

	DEVICE_LOCK(clk);

	reg = READ4(sc, sc_clk->offset);
	if (enable)
		reg |= (1 << JH7110_ENABLE_SHIFT);
	else
		reg &= ~(1 << JH7110_ENABLE_SHIFT);
	WRITE4(sc, sc_clk->offset, reg);

	DEVICE_UNLOCK(clk);

	return (0);
}

static int
jh7110_clk_set_mux(struct clknode *clk, int idx)
{
	struct jh7110_clkgen_softc *sc;
	struct jh7110_clk_sc *sc_clk;
	uint32_t reg;

	sc = device_get_softc(clknode_get_device(clk));
	sc_clk = clknode_get_softc(clk);

	if ((sc_clk->flags & JH7110_CLK_HAS_MUX) == 0)
		return (ENXIO);

	/* Checking index size */
	if ((idx & (JH7110_MUX_MASK >> JH7110_MUX_SHIFT)) != idx)
		return (EINVAL);

	DEVICE_LOCK(clk);

	reg = READ4(sc, sc_clk->offset) & ~JH7110_MUX_MASK;
	reg |= idx << JH7110_MUX_SHIFT;
	WRITE4(sc, sc_clk->offset, reg);

	DEVICE_UNLOCK(clk);

	return (0);
}

static int
jh7110_clk_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	struct jh7110_clkgen_softc *sc;
	struct jh7110_clk_sc *sc_clk;
	uint32_t divisor;

	sc = device_get_softc(clknode_get_device(clk));
	sc_clk = clknode_get_softc(clk);

	/* Returning error here causes panic */
	if ((sc_clk->flags & JH7110_CLK_HAS_DIV) == 0)
		return (0);

	DEVICE_LOCK(clk);

	divisor = READ4(sc, sc_clk->offset) & JH7110_DIV_MASK;

	DEVICE_UNLOCK(clk);

	if (divisor)
		*freq = *freq / divisor;
	else
		*freq = 0;

	return (0);
}

static int
jh7110_clk_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *done)
{
	struct jh7110_clkgen_softc *sc;
	struct jh7110_clk_sc *sc_clk;
	uint32_t divisor;

	sc = device_get_softc(clknode_get_device(clk));
	sc_clk = clknode_get_softc(clk);

	if ((sc_clk->flags & JH7110_CLK_HAS_DIV) == 0)
		return (0);

	divisor = MIN(MAX(DIV_ROUND_CLOSEST(fin, *fout), 1UL), sc_clk->d_max);

	if (flags & CLK_SET_DRYRUN)
		goto done;

	DEVICE_LOCK(clk);

	divisor |= READ4(sc, sc_clk->offset) & ~JH7110_DIV_MASK;
	WRITE4(sc, sc_clk->offset, divisor);

	DEVICE_UNLOCK(clk);

done:
	*fout = divisor;
	*done = 1;

	return (0);
}

static clknode_method_t jh7110_clknode_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		jh7110_clk_init),
	CLKNODEMETHOD(clknode_set_gate,		jh7110_clk_set_gate),
	CLKNODEMETHOD(clknode_set_mux,		jh7110_clk_set_mux),
	CLKNODEMETHOD(clknode_recalc_freq,	jh7110_clk_recalc_freq),
	CLKNODEMETHOD(clknode_set_freq,		jh7110_clk_set_freq),
	CLKNODEMETHOD_END
};

DEFINE_CLASS_1(jh7110_clknode, jh7110_clknode_class, jh7110_clknode_methods,
    sizeof(struct jh7110_clk_sc), clknode_class);

int
jh7110_clk_register(struct clkdom *clkdom, const struct jh7110_clk_def *clkdef)
{
	struct clknode *clk;
	struct jh7110_clk_sc *sc;

	clk = clknode_create(clkdom, &jh7110_clknode_class, &clkdef->clkdef);
	if (clk == NULL)
		return (-1);

	sc = clknode_get_softc(clk);

	sc->offset = clkdef->clkdef.id * REG_SIZE;

	sc->flags = clkdef->flags;
	sc->id = clkdef->clkdef.id;
	sc->d_max = clkdef->d_max;

	clknode_register(clkdom, clk);

	return (0);
}
