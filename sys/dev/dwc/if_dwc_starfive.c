/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2021 Mitchell Horne <mhorne@FreeBSD.org>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
*/

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/socket.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/pmap.h>

#include <net/if.h>
#include <net/if_media.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/hwreset/hwreset.h>

#include <dev/dwc/if_dwcvar.h>
#include <dev/dwc/dwc1000_reg.h>

#include "if_dwc_if.h"

static struct ofw_compat_data compat_data[] = {
        {"snps,dwmac",		 1},
	{NULL,			 0}
};

static int
if_dwc_starfive_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "StarFive Gigabit Ethernet Controller");

	return (BUS_PROBE_VENDOR);
}

static int
if_dwc_starfive_init(device_t dev)
{

	return (0);
}

static int
if_dwc_starfive_mii_clk(device_t dev)
{
	clk_t miiclk;
	uint64_t freq = 0;
	int rv;

	rv = clk_get_by_ofw_name(dev, 0, "stmmaceth", &miiclk);
	if (rv != 0)
		device_printf(dev, "Cannot get smmmaceth!\n");
	clk_get_freq(miiclk, &freq);
	printf("smmaceth freq: %lu\n", freq);

	return (GMAC_MII_CLK_100_150M_DIV62);
}

static int
if_dwc_starfive_set_speed(device_t dev, int speed)
{
	uint32_t *addr, val;

	device_printf(dev, "%s\n", __func__);

	/* TODO which clock is this? set the speed more naturally. */
	addr = pmap_mapdev(0x118001EC, 4);
	val = *addr;
	printf("val: %x\n", val);

	switch(speed) {
	case IFM_1000_T:
	case IFM_1000_SX:
		printf("speed 1000\n");
		val |= 0x4;
		*addr = val;
		fence();
		break;
	case IFM_100_TX:
		printf("speed 100\n");
		break;
	case IFM_10_T:
		printf("speed 10\n");
		break;
	default:
		printf("unknown speed %d\n", speed);
	}
	pmap_unmapdev(addr, 4);

	return (0);
}

static device_method_t if_dwc_starfive_methods[] = {
	DEVMETHOD(device_probe,		if_dwc_starfive_probe),

	DEVMETHOD(if_dwc_init,		if_dwc_starfive_init),
	DEVMETHOD(if_dwc_mii_clk,	if_dwc_starfive_mii_clk),
	DEVMETHOD(if_dwc_set_speed,	if_dwc_starfive_set_speed),

	DEVMETHOD_END
};

extern driver_t dwc_driver;

DEFINE_CLASS_1(dwc, dwc_starfive_driver, if_dwc_starfive_methods,
    sizeof(struct dwc_softc), dwc_driver);
DRIVER_MODULE(dwc_starfive, simplebus, dwc_starfive_driver, 0, 0);
MODULE_DEPEND(dwc_starfive, dwc, 1, 1, 1);
