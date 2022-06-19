/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright 2022 Mitchell Horne <mhorne@FreeBSD.org>
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

#ifndef _CLK_JH7100_CLK_H_
#define	_CLK_JH7100_CLK_H_

#define	JH7100_CLK_HAS_GATE	0x01
#define	JH7100_CLK_HAS_MUX	0x02
#define	JH7100_CLK_HAS_DIV	0x04
#define	JH7100_CLK_HAS_FRAC	0x08

struct jh7100_clk_def {
	struct clknode_init_def	clkdef;
	uint32_t		offset;
	uint32_t		flags;
};

#define	JH7100_CLK(_idx, _name, _pn, _flags)			\
{								\
	.clkdef.id =	_idx,					\
	.clkdef.name =	_name,					\
	.clkdef.parent_names = _pn,				\
	.clkdef.parent_cnt = nitems(_pn),			\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,		\
	.offset = ((_idx) * sizeof(uint32_t)),			\
	.flags = _flags,					\
}

#define	JH7100_GATE(_idx, _name, _pn)				\
    JH7100_CLK(_idx, _name, _pn, JH7100_CLK_HAS_GATE)
#define	JH7100_MUX(_idx, _name, _pn)				\
    JH7100_CLK(_idx, _name, _pn, JH7100_CLK_HAS_MUX)
#define	JH7100_GATEMUX(_idx, _name, _pn)			\
    JH7100_CLK(_idx, _name, _pn, JH7100_CLK_HAS_GATE | JH7100_CLK_HAS_MUX)
#define	JH7100_DIV(_idx, _name, _pn)				\
    JH7100_CLK(_idx, _name, _pn, JH7100_CLK_HAS_DIV)
#define	JH7100_FRACDIV(_idx, _name, _pn)			\
    JH7100_CLK(_idx, _name, _pn, JH7100_CLK_HAS_DIV | JH7100_CLK_HAS_FRAC)
#define	JH7100_GATEDIV(_idx, _name, _pn)			\
    JH7100_CLK(_idx, _name, _pn, JH7100_CLK_HAS_GATE | JH7100_CLK_HAS_DIV)

int jh7100_clk_register(struct clkdom *clkdom,
    const struct jh7100_clk_def *clkdef);

#endif	/* _CLK_JH7100_CLK_H_ */
