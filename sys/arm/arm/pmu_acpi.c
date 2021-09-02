/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2020 Greg V <greg@unrelenting.technology>
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

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/smp.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <arm/arm/gic_common.h>

#include "acpi_bus_if.h"
#include "pmu.h"

struct madt_ctx {
	struct pmu_softc *sc;
	int i;
};

static void
madt_handler(ACPI_SUBTABLE_HEADER *entry, void *arg)
{
	ACPI_MADT_GENERIC_INTERRUPT *intr;
	struct madt_ctx *ctx = arg;
	struct pmu_softc *sc = ctx->sc;
	//struct acpi_device *ad;
	struct pcpu *pcpu;
	rman_res_t irq;
	int rid = ctx->i;
	int cpuid = -1;
	int i;

	if (entry->Type != ACPI_MADT_TYPE_GENERIC_INTERRUPT)
		return;

	intr = (ACPI_MADT_GENERIC_INTERRUPT *)entry;

	//for (i = 0; i < MAXCPU; i++) {
	CPU_FOREACH(i) {
		pcpu = pcpu_find(i);
		if (pcpu != NULL && pcpu->pc_mpidr == intr->ArmMpidr) {
			cpuid = i;
			break;
		}
	}
	if (cpuid == -1) {
		device_printf(sc->dev, "Could not find matching mpidr! %lx\n",
		    intr->ArmMpidr);
		return;
	}

	if (bootverbose)
		device_printf(sc->dev,
		    "MADT: cpu %d (mpidr %lu) irq %d %s-triggered\n",
		    cpuid, intr->ArmMpidr, intr->PerformanceInterrupt,
		    (intr->Flags & ACPI_MADT_PERFORMANCE_IRQ_MODE) ?
		    "edge" : "level");

	bus_set_resource(sc->dev, SYS_RES_IRQ, rid,
	    intr->PerformanceInterrupt, 1);
	sc->irq[ctx->i].res = bus_alloc_resource_any(sc->dev, SYS_RES_IRQ,
	    &rid, RF_ACTIVE | RF_SHAREABLE);
	if (sc->irq[ctx->i].res == NULL)
		device_printf(sc->dev, "failed to allocate IRQ %d\n", ctx->i);


	/*
	 * BUS_CONFIG_INTR does nothing on arm64, so we manually register an IRQ
	 * resource with correct trigger mode instead of doing BUS_SET_RESOURCE.
	 */
	irq = 0;
	//irq = (rman_res_t)ACPI_BUS_MAP_INTR(device_get_parent(sc->dev), sc->dev,
	//    intr->PerformanceInterrupt,
	//    (intr->Flags & ACPI_MADT_PERFORMANCE_IRQ_MODE) ?
	//        INTR_TRIGGER_EDGE : INTR_TRIGGER_LEVEL,
	//    INTR_POLARITY_HIGH);
	//ad = device_get_ivars(sc->dev);
	//resource_list_add(&ad->ad_rl, SYS_RES_IRQ,
	//    ctx->i, irq, irq, 1);


	/* Set cpuid for SPI interrupts. */
	if (!intr_is_per_cpu(sc->irq[ctx->i].res))
		sc->irq[ctx->i].cpuid = cpuid;

	ctx->i++;
}

static void
pmu_acpi_identify(driver_t *driver, device_t parent)
{
	device_t dev;

	if (acpi_find_table(ACPI_SIG_MADT) == 0)
		return;

	dev = BUS_ADD_CHILD(parent, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LAST,
	    "pmu", -1);

	if (dev == NULL)
		device_printf(parent, "pmu: Unable to add pmu child\n");
}

static int
pmu_acpi_probe(device_t dev)
{
	device_set_desc(dev, "Performance Monitoring Unit");
	return (BUS_PROBE_NOWILDCARD);
}

static int
pmu_acpi_attach(device_t dev)
{
	struct pmu_softc *sc;
	struct madt_ctx ctx;
	ACPI_TABLE_MADT *madt;

	sc = device_get_softc(dev);
	sc->dev = dev;

	madt = acpi_map_table(acpi_find_table(ACPI_SIG_MADT), ACPI_SIG_MADT);
	if (madt == NULL) {
		device_printf(dev, "Unable to map the MADT table\n");
		return (ENXIO);
	}

	ctx.sc = sc;
	ctx.i = 0;
	acpi_walk_subtables(madt + 1, (char *)madt + madt->Header.Length,
	    madt_handler, &ctx);

	acpi_unmap_table(madt);
	return (pmu_attach(dev));
}

static device_method_t pmu_acpi_methods[] = {
	DEVMETHOD(device_identify,		pmu_acpi_identify),
	DEVMETHOD(device_probe,		pmu_acpi_probe),
	DEVMETHOD(device_attach,	pmu_acpi_attach),
	DEVMETHOD_END,
};

static driver_t pmu_acpi_driver = {
	"pmu",
	pmu_acpi_methods,
	sizeof(struct pmu_softc),
};

static devclass_t pmu_acpi_devclass;

DRIVER_MODULE(pmu, acpi, pmu_acpi_driver, pmu_acpi_devclass, 0, 0);
