/*
 * Copyright 2017 Emmanuel Vadot <manu@freebsd.org>
 * Copyright 2021 Mitchell Horne <mhorne@FreeBSD.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/queue.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmc_fdt_helpers.h>

#include <dev/mmc/host/dwmmc_var.h>

#include <dev/ofw/ofw_bus_subr.h>

#include "opt_mmccam.h"

static struct ofw_compat_data compat_data[] = {
	{"snps,dw-mshc",	0},
	{NULL,			0},
};

static int dwmmc_starfive_update_ios(struct dwmmc_softc *sc, struct mmc_ios *ios);

static int
starfive_dwmmc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_str == NULL)
		return (ENXIO);

	device_set_desc(dev, "Synopsys DesignWare Mobile "
	    "Storage Host Controller (StarFive)");

	return (BUS_PROBE_VENDOR);
}

static int
starfive_dwmmc_attach(device_t dev)
{
	struct dwmmc_softc *sc;

	sc = device_get_softc(dev);
	sc->hwtype = HWTYPE_STARFIVE;

	return (dwmmc_attach(dev));
}

static device_method_t starfive_dwmmc_methods[] = {
	/* bus interface */
	DEVMETHOD(device_probe, starfive_dwmmc_probe),
	DEVMETHOD(device_attach, starfive_dwmmc_attach),
	DEVMETHOD(device_detach, dwmmc_detach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(starfive_dwmmc, starfive_dwmmc_driver, starfive_dwmmc_methods,
    sizeof(struct dwmmc_softc), dwmmc_driver);

DRIVER_MODULE(starfive_dwmmc, simplebus, starfive_dwmmc_driver, 0, 0);
#ifndef MMCCAM
MMC_DECLARE_BRIDGE(starfive_dwmmc);
#endif
