/*-
 * Copyright (c) 2021 Emmanuel Vadot <manu@freebsd.org>
 * Copyright (c) 2022 Julien Cassette <julien.cassette@gmail.com>
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
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/intr.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "pic_if.h"

struct aw_d1_intc_softc {
	device_t		dev;
	device_t		parent;

	struct intr_map_data_fdt *parent_map_data;
};

static struct ofw_compat_data compat_data[] = {
	{"allwinner,sun20i-d1-intc",	1},
	{NULL,				0}
};

static int
aw_d1_intc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Allwinner D1 INTC");
	return (BUS_PROBE_DEFAULT);
}

static int
aw_d1_intc_attach(device_t dev)
{
	struct aw_d1_intc_softc *sc;
	phandle_t node, xref, intr_parent;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	/* Look for our parent */
	if ((intr_parent = ofw_bus_find_iparent(node)) == 0) {
		device_printf(dev,
		     "cannot find parent interrupt controller\n");
		return (ENXIO);
	}
	if ((sc->parent = OF_device_from_xref(intr_parent)) == NULL) {
		device_printf(dev,
		     "cannot find parent interrupt controller device\n");
		return (ENXIO);
	}

	/* Register ourself as a interrupt controller */
	xref = OF_xref_from_node(node);
	if (intr_pic_register(dev, xref) == NULL) {
		device_printf(dev, "cannot register as interrupt controller\n");
		return (ENXIO);
	}

	/* Allocate PLIC compatible mapping */
	sc->parent_map_data = (struct intr_map_data_fdt *)intr_alloc_map_data(
	    INTR_MAP_DATA_FDT, sizeof(struct intr_map_data_fdt) +
	    + 1 * sizeof(phandle_t), M_WAITOK | M_ZERO);

	/* Register ourself to device can find us */
	OF_device_register_xref(xref, dev);

	return (0);
}

static int
aw_d1_intc_detach(device_t dev)
{

	return (EBUSY);
}

static struct intr_map_data *
aw_d1_intc_convert_map_data(struct aw_d1_intc_softc *sc,
    struct intr_map_data *data)
{
	struct intr_map_data_fdt *daf;

	daf = (struct intr_map_data_fdt *)data;

	if (daf->ncells != 2)
		return (NULL);

	if (daf->cells[0] < 16)
		return (NULL);

	sc->parent_map_data->ncells = 1;
	sc->parent_map_data->cells[0] = daf->cells[0];

	return ((struct intr_map_data *)sc->parent_map_data);
}

static int
aw_d1_intc_activate_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct aw_d1_intc_softc *sc;

	sc = device_get_softc(dev);
	data = aw_d1_intc_convert_map_data(sc, data);
	if (data == NULL)
		return (EINVAL);

	return (PIC_ACTIVATE_INTR(sc->parent, isrc, res, data));
}

static void
aw_d1_intc_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct aw_d1_intc_softc *sc;

	sc = device_get_softc(dev);

	PIC_ENABLE_INTR(sc->parent, isrc);
}

static void
aw_d1_intc_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct aw_d1_intc_softc *sc;

	sc = device_get_softc(dev);

	PIC_DISABLE_INTR(sc->parent, isrc);
}

static int
aw_d1_intc_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	struct aw_d1_intc_softc *sc;
	int ret;

	sc = device_get_softc(dev);

	if (data->type != INTR_MAP_DATA_FDT)
		return (ENOTSUP);

	data = aw_d1_intc_convert_map_data(sc, data);
	if (data == NULL)
		return (EINVAL);

	ret = PIC_MAP_INTR(sc->parent, data, isrcp);
	(*isrcp)->isrc_dev = sc->dev;

	return (ret);
}

static int
aw_d1_intc_deactivate_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct aw_d1_intc_softc *sc;

	sc = device_get_softc(dev);

	data = aw_d1_intc_convert_map_data(sc, data);
	if (data == NULL)
		return (EINVAL);

	return (PIC_DEACTIVATE_INTR(sc->parent, isrc, res, data));
}

static int
aw_d1_intc_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct aw_d1_intc_softc *sc;

	sc = device_get_softc(dev);
	data = aw_d1_intc_convert_map_data(sc, data);
	if (data == NULL)
		return (EINVAL);

	return (PIC_SETUP_INTR(sc->parent, isrc, res, data));
}

static int
aw_d1_intc_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct aw_d1_intc_softc *sc;

	sc = device_get_softc(dev);
	data = aw_d1_intc_convert_map_data(sc, data);
	if (data == NULL)
		return (EINVAL);

	return (PIC_TEARDOWN_INTR(sc->parent, isrc, res, data));
}

static void
aw_d1_intc_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	struct aw_d1_intc_softc *sc;

	sc = device_get_softc(dev);

	PIC_PRE_ITHREAD(sc->parent, isrc);
}

static void
aw_d1_intc_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	struct aw_d1_intc_softc *sc;

	sc = device_get_softc(dev);

	PIC_POST_ITHREAD(sc->parent, isrc);
}

static void
aw_d1_intc_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
	struct aw_d1_intc_softc *sc;

	sc = device_get_softc(dev);

	PIC_POST_FILTER(sc->parent, isrc);
}

static device_method_t aw_d1_intc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		aw_d1_intc_probe),
	DEVMETHOD(device_attach,	aw_d1_intc_attach),
	DEVMETHOD(device_detach,	aw_d1_intc_detach),

	/* Interrupt controller interface */
	DEVMETHOD(pic_activate_intr,	aw_d1_intc_activate_intr),
	DEVMETHOD(pic_disable_intr,	aw_d1_intc_disable_intr),
	DEVMETHOD(pic_enable_intr,	aw_d1_intc_enable_intr),
	DEVMETHOD(pic_map_intr,		aw_d1_intc_map_intr),
	DEVMETHOD(pic_deactivate_intr,	aw_d1_intc_deactivate_intr),
	DEVMETHOD(pic_setup_intr,	aw_d1_intc_setup_intr),
	DEVMETHOD(pic_teardown_intr,	aw_d1_intc_teardown_intr),
	DEVMETHOD(pic_post_filter,	aw_d1_intc_post_filter),
	DEVMETHOD(pic_post_ithread,	aw_d1_intc_post_ithread),
	DEVMETHOD(pic_pre_ithread,	aw_d1_intc_pre_ithread),

	DEVMETHOD_END
};

static devclass_t aw_d1_intc_devclass;

static driver_t aw_d1_intc_driver = {
	"aw_d1_intc",
	aw_d1_intc_methods,
	sizeof(struct aw_d1_intc_softc),
};

EARLY_DRIVER_MODULE(aw_d1_intc, simplebus, aw_d1_intc_driver,
    aw_d1_intc_devclass, 0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
