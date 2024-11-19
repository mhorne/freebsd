/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 The FreeBSD Foundation
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

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/condvar.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>

#include <machine/bus.h>

#include <dev/fdt/simplebus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

struct cdns3_starfive_softc {
	struct resource *sc_mem_res;
	clk_t		clk_lpm;
	clk_t		clk_stb;
	clk_t		clk_apb;
	clk_t		clk_axi;
	clk_t		clk_utmi_apb;
	hwreset_t	rst_pwrup;
	hwreset_t	rst_apb;
	hwreset_t	rst_axi;
	hwreset_t	rst_utmi_apb;
};
static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-usb",	1 },
	{ NULL,				0 }
};

static int
cdns3_starfive_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "cdns3 USB Starfive (Glue)");
	return (BUS_PROBE_DEFAULT);
}

static int
cdns3_starfive_attach(device_t dev)
{
	struct cdns3_starfive_softc *sc;
	phandle_t node;
	int rid;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);

	/* Fetch and enable clocks. */

	if (clk_get_by_ofw_name(dev, node, "lpm", &sc->clk_lpm) != 0)
		device_printf(dev, "Cannot get lpm clock\n");
	if (clk_get_by_ofw_name(dev, node, "stb", &sc->clk_stb) != 0)
		device_printf(dev, "Cannot get stb clock\n");
	if (clk_get_by_ofw_name(dev, node, "apb", &sc->clk_apb) != 0)
		device_printf(dev, "Cannot get apb clock\n");
	if (clk_get_by_ofw_name(dev, node, "axi", &sc->clk_axi) != 0)
		device_printf(dev, "Cannot get axi clock\n");
	if (clk_get_by_ofw_name(dev, node, "utmi_apb", &sc->clk_utmi_apb) != 0)
		device_printf(dev, "Cannot get utmi_apb clock\n");

	if (sc->clk_lpm != NULL) {
		if (clk_enable(sc->clk_lpm) != 0)
			device_printf(dev, "Cannot enable lpm clock\n");
	}
	if (sc->clk_stb != NULL) {
		if (clk_enable(sc->clk_stb) != 0)
			device_printf(dev, "Cannot enable stb clock\n");
	}
	if (sc->clk_apb != NULL) {
		if (clk_enable(sc->clk_apb) != 0)
			device_printf(dev, "Cannot enable apb clock\n");
	}
	if (sc->clk_axi != NULL) {
		if (clk_enable(sc->clk_axi) != 0)
			device_printf(dev, "Cannot enable axi clock\n");
	}
	if (sc->clk_utmi_apb != NULL) {
		if (clk_enable(sc->clk_utmi_apb) != 0)
			device_printf(dev, "Cannot enable utmi_apb clock\n");
	}

	/* Fetch and deassert resets. */
	if (hwreset_get_by_ofw_name(dev, node, "pwrup", &sc->rst_pwrup) != 0)
		device_printf(dev, "Cannot get pwrup reset\n");
	if (hwreset_get_by_ofw_name(dev, node, "apb", &sc->rst_apb) != 0)
		device_printf(dev, "Cannot get apb reset\n");
	if (hwreset_get_by_ofw_name(dev, node, "axi", &sc->rst_axi) != 0)
		device_printf(dev, "Cannot get axi reset\n");
	if (hwreset_get_by_ofw_name(dev, node, "utmi_apb", &sc->rst_utmi_apb) != 0)
		device_printf(dev, "Cannot get utmi_apb reset\n");

	if (sc->rst_pwrup != NULL) {
		if (hwreset_deassert(sc->rst_pwrup) != 0)
			device_printf(dev, "Cannot deassert pwrup reset\n");
	}
	if (sc->rst_apb != NULL) {
		if (hwreset_deassert(sc->rst_apb) != 0)
			device_printf(dev, "Cannot deassert apb reset\n");
	}
	if (sc->rst_axi != NULL) {
		if (hwreset_deassert(sc->rst_axi) != 0)
			device_printf(dev, "Cannot deassert axi reset\n");
	}
	if (sc->rst_utmi_apb != NULL) {
		if (hwreset_deassert(sc->rst_utmi_apb) != 0)
			device_printf(dev, "Cannot deassert utmi_apb reset\n");
	}

	/* Probe ofw children. */

	return (0);
}

static int
cdns3_starfive_detach(device_t dev)
{
	struct cdns3_starfive_softc *sc;

	sc = device_get_softc(dev);

	if (sc->clk_lpm != NULL)
		clk_release(sc->clk_lpm);
	if (sc->clk_stb != NULL)
		clk_release(sc->clk_stb);
	if (sc->clk_apb != NULL)
		clk_release(sc->clk_apb);
	if (sc->clk_axi != NULL)
		clk_release(sc->clk_axi);
	if (sc->clk_utmi_apb != NULL)
		clk_release(sc->clk_utmi_apb);
	if (sc->rst_pwrup != NULL)
		hwreset_assert(sc->rst_pwrup);
	if (sc->rst_apb != NULL)
		hwreset_assert(sc->rst_apb);
	if (sc->rst_axi != NULL)
		hwreset_assert(sc->rst_axi);
	if (sc->rst_utmi_apb != NULL)
		hwreset_assert(sc->rst_utmi_apb);

	if (sc->sc_mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem_res);

	return (0);

}

static device_method_t cdns3_starfive_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		cdns3_starfive_probe),
	DEVMETHOD(device_attach,	cdns3_starfive_attach),
	DEVMETHOD(device_detach,	cdns3_starfive_detach),

	DEVMETHOD_END
};

DEFINE_CLASS_0(cdns3_starfive, cdns3_starfive_driver, cdns3_starfive_methods,
    sizeof(struct cdns3_starfive_softc));

DRIVER_MODULE(cdns3_starfive, simplebus, cdns3_starfive_driver, 0, 0);
