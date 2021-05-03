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
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#if 1 /* StarFive JH7100 WAR */
#include <machine/atomic.h>
#include <machine/cpufunc.h>
#include <vm/pmap.h>
#endif

#define	REG_CONTROL_OFFSET	0x0
#define	REG_WAYENAB_OFFSET	0x8

#define	REG_FLUSH64_OFFSET	0x200
#define	REG_FLUSH32_OFFSET	0x240

static struct ofw_compat_data compat_data[] = {
	{ "sifive,fu540-c000-ccache",	0 },
	{ "starfive,ccache0",		0 },
	{ NULL, 0 }
};

struct sifive_l2_cache_softc {
	struct resource *cc_res;
	struct resource *intr_res;
	void *ih_cookie;
};

#define RD1(sc, reg) \
    bus_read_1(sc->cc_res, (reg))
#define RD4(sc, reg) \
    bus_read_4(sc->cc_res, (reg))

static struct resource *bus_res = NULL;

static int
sifive_l2_cache_probe(device_t dev)
{
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_str == NULL)
		return (ENXIO);

	device_set_desc(dev, "SiFive L2 cache controller");
	return (BUS_PROBE_DEFAULT);
}

static void
cache_intr_handler(void *arg)
{
	printf("cache interrupt!\n");
}

static int
sifive_l2_cache_attach(device_t dev)
{
	struct sifive_l2_cache_softc *sc;
	uint32_t reg_config;
	uint32_t reg_wayenable;
	int rid;
	int error;

	sc = device_get_softc(dev);

	/* Request memory resources */
	rid = 0;
	sc->cc_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->cc_res == NULL) {
		device_printf(dev,
		    "Error: could not allocate memory resources\n");
		return (ENXIO);
	}

	sc->intr_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &rid, RF_ACTIVE);
	if (sc->intr_res == NULL) {
		device_printf(dev, "Unable to allocate event INTR resource.\n");
		return (ENOMEM);
	}

	
	error = bus_setup_intr(dev, sc->intr_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, cache_intr_handler, sc, &sc->ih_cookie);
	if (error) {
		device_printf(dev, "Fail to setup event intr\n");
		return (error);
	}

	/* Get config register */
	reg_config = RD4(sc, REG_CONTROL_OFFSET);
	device_printf(dev, "control:\t0x%x\n", reg_config);
	reg_wayenable = RD1(sc, REG_WAYENAB_OFFSET);
	device_printf(dev, "WayEnable:\t0x%x\n", reg_wayenable);

	/* For JH7100 WAR */
	bus_res = sc->cc_res;

	return (0);
}

#if 1
void
sifive_l2_cache_flush_dcache(vm_offset_t va, vm_size_t size)
{
	vm_paddr_t line, end;

	line = rounddown(pmap_kextract(va), 64);
	end = line + size ;

	if (bus_res == NULL)
		return;
	//MPASS(bus_res != NULL);
	MPASS((line & 0x3f) == 0);

	//printf("flush dcache: %lx-%lx\n", line, end);

	while (line < end) {
		bus_write_8(bus_res, REG_FLUSH64_OFFSET, line);
		line += 64;
	}
	mb();
}
#endif

static device_method_t sifive_l2_cache_methods[] = {
	/* device methods */
	DEVMETHOD(device_probe, 	sifive_l2_cache_probe),
	DEVMETHOD(device_attach, 	sifive_l2_cache_attach),

	DEVMETHOD_END
};

static driver_t sifive_l2_cache_driver = {
	"cache",
	sifive_l2_cache_methods,
	sizeof(struct sifive_l2_cache_softc),
};
static devclass_t sifive_l2_cache_devclass;

EARLY_DRIVER_MODULE(sifive_l2_cache, simplebus, sifive_l2_cache_driver,
    sifive_l2_cache_devclass, NULL, NULL,
    BUS_PASS_RESOURCE + BUS_PASS_ORDER_MIDDLE);
