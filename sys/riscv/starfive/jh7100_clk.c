/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Emmanuel Vadot <manu@freebsd.org>
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

#include <dev/extres/clk/clk.h>

#include <riscv/starfive/jh7100_clk.h>

#include "clkdev_if.h"

/*
 * Layout of the clock registers:
 *
 * +--------------------------------------------------------+
 * | 31 | 30  | 29 - 28  |27 - 24| 23 - 16  | 15 - 8 | 7 - 0|
 * | EN | INV | reserved | MUX   | reserved | FRAC   | DIV  |
 * +--------------------------------------------------------+
 */
#define	JH7100_DIV_MASK		0xff
#define	JH7100_DIV_SHIFT	0
#define	JH7100_FRAC_MASK	0xff
#define	JH7100_FRAC_SHIFT	8
#define	JH7100_MUX_SHIFT	24
#define	JH7100_MUX_MASK		0xf
#define	JH7100_ENABLE_SHIFT	31

struct jh7100_clk_sc {
	uint32_t	offset;
	uint32_t	flags;

	uint64_t	divider;
#if 0
	struct aw_clk_factor	m;
	struct aw_clk_factor	n;
	struct aw_clk_frac	frac;

	uint64_t		min_freq;
	uint64_t		max_freq;

	uint32_t	mux_shift;
	uint32_t	mux_mask;
	uint32_t	gate_shift;
	uint32_t	lock_shift;
	uint32_t	lock_retries;
#endif
};

#define	WRITE4(_clk, off, val)					\
	CLKDEV_WRITE_4(clknode_get_device(_clk), off, val)
#define	READ4(_clk, off, val)					\
	CLKDEV_READ_4(clknode_get_device(_clk), off, val)
#define	DEVICE_LOCK(_clk)					\
	CLKDEV_DEVICE_LOCK(clknode_get_device(_clk))
#define	DEVICE_UNLOCK(_clk)					\
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(_clk))

static int
jh7100_clk_init(struct clknode *clk, device_t dev)
{
	struct jh7100_clk_sc *sc;
	uint32_t reg;
	int idx;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(clk);
	READ4(clk, sc->offset, &reg);
	DEVICE_UNLOCK(clk);

	/* Safe to read the mux field even for non-mux types. */
	idx = (reg >> JH7100_MUX_SHIFT) & JH7100_MUX_MASK;
	clknode_init_parent_idx(clk, idx);

	return (0);
}

static int
jh7100_clk_set_gate(struct clknode *clk, bool enable)
{
	struct jh7100_clk_sc *sc;
	uint32_t reg;

	sc = clknode_get_softc(clk);

	if ((sc->flags & JH7100_CLK_HAS_GATE) == 0)
		return (0);

	DEVICE_LOCK(clk);
	READ4(clk, sc->offset, &reg);
	if (enable)
		reg |= (1 << JH7100_ENABLE_SHIFT);
	else
		reg &= ~(1 << JH7100_ENABLE_SHIFT);
	WRITE4(clk, sc->offset, reg);
	DEVICE_UNLOCK(clk);

	return (0);
}

static int
jh7100_clk_set_mux(struct clknode *clk, int idx)
{
	struct jh7100_clk_sc *sc;
	uint32_t reg;

	sc = clknode_get_softc(clk);

	if ((sc->flags & JH7100_CLK_HAS_MUX) == 0)
		return (ENXIO);

	/* Index too big? */
	if ((idx & JH7100_MUX_MASK) != idx)
		return (EINVAL);

	DEVICE_LOCK(clk);
	READ4(clk, sc->offset, &reg);
	reg &= JH7100_MUX_MASK << JH7100_MUX_SHIFT;
	reg |= idx << JH7100_MUX_SHIFT;
	WRITE4(clk, sc->offset, reg);
	DEVICE_UNLOCK(clk);

	return (0);
}

static int
jh7100_clk_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	struct jh7100_clk_sc *sc;
	uint32_t reg;

	sc = clknode_get_softc(clk);

	if ((sc->flags & (JH7100_CLK_HAS_DIV | JH7100_CLK_HAS_FRAC)) == 0)
		return (0); /* TODO error? */

	//MPASS(sc->divider != 0);

	DEVICE_LOCK(clk);
	READ4(clk, sc->offset, &reg);
	if ((sc->flags & JH7100_CLK_HAS_FRAC) == 0) {
		/* Easy case, clock has div only. */
		if ((sc->flags & JH7100_CLK_HAS_DIV) != 0) {
			u_int divider = (reg >> JH7100_DIV_SHIFT) &
			    JH7100_DIV_MASK;
			*freq = *freq / divider;
		}
	} else {
		/* TODO */
		*freq = *freq;
	
	}
	DEVICE_UNLOCK(clk);

	return (0);
}

static int
jh7100_clk_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *done)
{
	//struct jh7100_clk_sc *sc;
	//uint32_t reg;

	//sc = clknode_get_softc(clk);
	panic("%s: tried to set freq for %s\n", __func__, clknode_get_name(clk));

	/* TODO */
	return (0);
}

static clknode_method_t jh7100_clknode_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		jh7100_clk_init),
	CLKNODEMETHOD(clknode_set_gate,		jh7100_clk_set_gate),
	CLKNODEMETHOD(clknode_set_mux,		jh7100_clk_set_mux),
	CLKNODEMETHOD(clknode_recalc_freq,	jh7100_clk_recalc_freq),
	CLKNODEMETHOD(clknode_set_freq,		jh7100_clk_set_freq),
	CLKNODEMETHOD_END
};

DEFINE_CLASS_1(jh7100_clknode, jh7100_clknode_class, jh7100_clknode_methods,
    sizeof(struct jh7100_clk_sc), clknode_class);

int
jh7100_clk_register(struct clkdom *clkdom, const struct jh7100_clk_def *clkdef)
{
	struct clknode *clk;
	struct jh7100_clk_sc *sc;

	clk = clknode_create(clkdom, &jh7100_clknode_class, &clkdef->clkdef);
	if (clk == NULL)
		return (-1);

	sc = clknode_get_softc(clk);

	sc->offset = clkdef->offset;
	sc->flags  = clkdef->flags;

	clknode_register(clkdom, clk);

	return (0);
}
