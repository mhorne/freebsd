/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Jari Sihvola <jsihv@gmx.com>
 * Copyright (c) 2025 The FreeBSD Foundation
 *
 * Portions of this software was developed by Mitchell Horne
 * <mhorne@FreeBSD.org> under sponsorship from the FreeBSD Foundation.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/resource.h>

#include <dev/clk/clk.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "fdt_pinctrl_if.h"
#include "gpio_if.h"

#define	GPIO_PINS		64
#define	GPIO_REGS		2

#define	JH7110_GPIO_DEFAULT_CAPS \
    (GPIO_PIN_INPUT | GPIO_PIN_OUTPUT | GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN)

#define	GP0_DOEN_CFG		0x0
#define	GP0_DOUT_CFG		0x40

#define	GPIOEN			0xdc
#define	GPIOE_0			0x100
#define	GPIOE_1			0x104
#define	GPIOIN_LOW		0x118
#define	GPIOIN_HIGH		0x11c
#define	IOMUX_SYSCFG_288	0x120

#define	PAD_INPUT_EN		(1 << 0)
#define	PAD_PULLUP		(1 << 3)
#define	PAD_PULLDOWN		(1 << 4)
#define	PAD_BIAS_MASK		(PAD_PULLUP | PAD_PULLDOWN)
#define	PAD_HYST		(1 << 6)

#define	ENABLE_MASK		0x3f
#define	DATA_OUT_MASK		0x7f

#define	GPOEN_ENABLE		0
#define	DIROUT_DISABLE		1

struct jh7110_gpio_softc {
	device_t		dev;
	device_t		busdev;
	struct mtx		mtx;
	struct resource		*res;
	clk_t			clk;
	struct gpio_pin		gpio_pins[GPIO_PINS];
};

static struct ofw_compat_data compat_data[] = {
	{"starfive,jh7110-sys-pinctrl", 1},
	{NULL,				0}
};

static struct resource_spec jh7110_gpio_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ -1, 0 }
};

#define	GPIO_RW_OFFSET(_val)		(_val & ~3)
#define	GPIO_SHIFT(_val)		((_val & 3) * 8)
#define	PAD_OFFSET(_val)		(_val * 4)

#define	JH7110_GPIO_LOCK(_sc)		mtx_lock(&(_sc)->mtx)
#define	JH7110_GPIO_UNLOCK(_sc)		mtx_unlock(&(_sc)->mtx)
#define	JH7110_GPIO_ASSERT_LOCKED(_sc)	mtx_assert(&(_sc)->mtx, MA_OWNED);

#define	READ4(sc, reg)			bus_read_4((sc)->res, (reg))
#define	WRITE4(sc, reg, val)		bus_write_4((sc)->res, (reg), (val))

#define	JH7110_DEFAULT_CAPS		(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT)

static device_t
jh7110_gpio_get_bus(device_t dev)
{
	struct jh7110_gpio_softc *sc;

	sc = device_get_softc(dev);

	return (sc->busdev);
}

static int
jh7110_gpio_pin_max(device_t dev, int *maxpin)
{
	*maxpin = GPIO_PINS - 1;

	return (0);
}

static int
jh7110_gpio_pin_get(device_t dev, uint32_t pin, uint32_t *val)
{
	struct jh7110_gpio_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);

	if (pin >= GPIO_PINS)
		return (EINVAL);

	JH7110_GPIO_LOCK(sc);
	if (pin < GPIO_PINS / GPIO_REGS) {
		reg = READ4(sc, GPIOIN_LOW);
		*val = (reg >> pin) & 0x1;
	} else {
		reg = READ4(sc, GPIOIN_HIGH);
		*val = (reg >> (pin - GPIO_PINS / GPIO_REGS)) & 0x1;
	}
	JH7110_GPIO_UNLOCK(sc);

	return (0);
}

static void
jh7110_gpio_pin_set_locked(struct jh7110_gpio_softc *sc, uint32_t pin,
    uint32_t val)
{
	uint32_t reg;

	JH7110_GPIO_ASSERT_LOCKED(sc);
	MPASS(pin < GPIO_PINS);

	reg = READ4(sc, GP0_DOUT_CFG + GPIO_RW_OFFSET(pin));
	reg &= ~(DATA_OUT_MASK << GPIO_SHIFT(pin));
	if (val != 0)
		reg |= 0x1 << GPIO_SHIFT(pin);
	WRITE4(sc, GP0_DOUT_CFG + GPIO_RW_OFFSET(pin), reg);
}

static int
jh7110_gpio_pin_set(device_t dev, uint32_t pin, uint32_t val)
{
	struct jh7110_gpio_softc *sc;

	sc = device_get_softc(dev);

	if (pin >= GPIO_PINS)
		return (EINVAL);

	JH7110_GPIO_LOCK(sc);
	jh7110_gpio_pin_set_locked(sc, pin, val);
	JH7110_GPIO_UNLOCK(sc);

	return (0);
}

static int
jh7110_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct jh7110_gpio_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);

	if (pin >= GPIO_PINS)
		return (EINVAL);

	JH7110_GPIO_LOCK(sc);
	reg = READ4(sc, GP0_DOUT_CFG + GPIO_RW_OFFSET(pin));
	if ((reg & 0x1 << GPIO_SHIFT(pin)) != 0) {
		reg &= ~(DATA_OUT_MASK << GPIO_SHIFT(pin));
	} else {
		reg &= ~(DATA_OUT_MASK << GPIO_SHIFT(pin));
		reg |= 0x1 << GPIO_SHIFT(pin);
	}
	WRITE4(sc, GP0_DOUT_CFG + GPIO_RW_OFFSET(pin), reg);
	JH7110_GPIO_UNLOCK(sc);

	return (0);
}

static int
jh7110_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct jh7110_gpio_softc *sc;

	sc = device_get_softc(dev);

	if (pin >= GPIO_PINS)
		return (EINVAL);

	JH7110_GPIO_LOCK(sc);
	*caps = sc->gpio_pins[pin].gp_caps;
	JH7110_GPIO_UNLOCK(sc);

	return (0);
}

static int
jh7110_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct jh7110_gpio_softc *sc;

	sc = device_get_softc(dev);

	if (pin >= GPIO_PINS)
		return (EINVAL);

	JH7110_GPIO_LOCK(sc);
	*flags = sc->gpio_pins[pin].gp_flags;
	JH7110_GPIO_UNLOCK(sc);

	return (0);
}

static int
jh7110_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct jh7110_gpio_softc *sc;

	sc = device_get_softc(dev);

	if (pin >= GPIO_PINS)
		return (EINVAL);

	JH7110_GPIO_LOCK(sc);
	memcpy(name, sc->gpio_pins[pin].gp_name, GPIOMAXNAME);
	JH7110_GPIO_UNLOCK(sc);

	return (0);
}

static __inline void
jh7110_gpio_pin_setup_input(struct jh7110_gpio_softc *sc, uint32_t pin,
    uint32_t flags)
{
	uint32_t reg;

	/* Configure PAD first: enable input, SMT trigger, clear bias. */
	reg = READ4(sc, IOMUX_SYSCFG_288 + PAD_OFFSET(pin));
	reg &= ~(PAD_BIAS_MASK | PAD_INPUT_EN | PAD_HYST);
	reg |= (PAD_INPUT_EN | PAD_HYST);
	WRITE4(sc, IOMUX_SYSCFG_288 + PAD_OFFSET(pin), reg);

	/* Then update DOEN register. */
	reg = READ4(sc, GP0_DOEN_CFG + GPIO_RW_OFFSET(pin));
	reg &= ~(ENABLE_MASK << GPIO_SHIFT(pin));
	reg |= DIROUT_DISABLE << GPIO_SHIFT(pin);
	WRITE4(sc, GP0_DOEN_CFG + GPIO_RW_OFFSET(pin), reg);

	/* Update software state. */
	sc->gpio_pins[pin].gp_flags &= GPIO_PIN_OUTPUT;
	sc->gpio_pins[pin].gp_flags |= GPIO_PIN_INPUT;
}

static __inline void
jh7110_gpio_pin_setup_output(struct jh7110_gpio_softc *sc, uint32_t pin,
    uint32_t flags)
{
	uint32_t reg;

	/* First update DOEN register. */
	reg = READ4(sc, GP0_DOEN_CFG + GPIO_RW_OFFSET(pin));
	reg &= ~(ENABLE_MASK << GPIO_SHIFT(pin));
	if ((flags & GPIO_PIN_INPUT) != 0) {
		reg |= DIROUT_DISABLE << GPIO_SHIFT(pin);
	}
	WRITE4(sc, GP0_DOEN_CFG + GPIO_RW_OFFSET(pin), reg);

	/* Next, DOUT register. */
	reg = READ4(sc, GP0_DOUT_CFG + GPIO_RW_OFFSET(pin));
	reg &= ~(ENABLE_MASK << GPIO_SHIFT(pin));
	reg |= 0x1 << GPIO_SHIFT(pin);
	WRITE4(sc, GP0_DOUT_CFG + GPIO_RW_OFFSET(pin), reg);

	/* Finally PAD register */
	reg = READ4(sc, IOMUX_SYSCFG_288 + PAD_OFFSET(pin));
	reg &= ~(PAD_INPUT_EN | PAD_PULLUP | PAD_PULLDOWN | PAD_HYST);
	WRITE4(sc, IOMUX_SYSCFG_288 + PAD_OFFSET(pin), reg);

	/* Handle preset values. */
	if ((flags & GPIO_PIN_PRESET_LOW) != 0) {
		jh7110_gpio_pin_set_locked(sc, pin, 0);
	} else if ((flags & GPIO_PIN_PRESET_HIGH) != 0) {
		jh7110_gpio_pin_set_locked(sc, pin, 1);
	}

	/* Update software state. */
	sc->gpio_pins[pin].gp_flags &= GPIO_PIN_INPUT;
	sc->gpio_pins[pin].gp_flags |= GPIO_PIN_OUTPUT;
}

static int
jh7110_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct jh7110_gpio_softc *sc;

	sc = device_get_softc(dev);

	if (pin >= GPIO_PINS)
		return (EINVAL);

	JH7110_GPIO_LOCK(sc);

	if ((flags & GPIO_PIN_INPUT) != 0) {
		jh7110_gpio_pin_setup_input(sc, pin, flags);
	} else if ((flags & GPIO_PIN_OUTPUT) != 0) {
		jh7110_gpio_pin_setup_output(sc, pin, flags);
	}

	JH7110_GPIO_UNLOCK(sc);

	return (0);
}

static int
jh7110_gpio_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 GPIO controller");

	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_gpio_detach(device_t dev)
{
	struct jh7110_gpio_softc *sc;

	sc = device_get_softc(dev);

	bus_release_resources(dev, jh7110_gpio_spec, &sc->res);
	if (sc->busdev != NULL)
		gpiobus_detach_bus(dev);
	if (sc->clk != NULL)
		clk_release(sc->clk);
	mtx_destroy(&sc->mtx);

	return (0);
}

static int
jh7110_gpio_attach(device_t dev)
{
	struct jh7110_gpio_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(sc->dev), NULL, MTX_DEF);

	if (bus_alloc_resources(dev, jh7110_gpio_spec, &sc->res) != 0) {
		device_printf(dev, "Could not allocate resources\n");
		mtx_destroy(&sc->mtx);
		return (ENXIO);
	}

	if (clk_get_by_ofw_index(dev, 0, 0, &sc->clk) != 0) {
		device_printf(dev, "Cannot get clock\n");
		jh7110_gpio_detach(dev);
		return (ENXIO);
	}

	if (clk_enable(sc->clk) != 0) {
		device_printf(dev, "Could not enable clock %s\n",
		    clk_get_name(sc->clk));
		jh7110_gpio_detach(dev);
		return (ENXIO);
	}

	/* Set initial pin software state. */
	for (int i = 0; i < GPIO_PINS; i++) {
		reg = READ4(sc, GP0_DOEN_CFG + GPIO_RW_OFFSET(i));

		sc->gpio_pins[i].gp_pin = i;
		sc->gpio_pins[i].gp_caps = JH7110_GPIO_DEFAULT_CAPS;
		sc->gpio_pins[i].gp_flags =
		    ((reg & (ENABLE_MASK << GPIO_SHIFT(i))) == 0) ?
		    GPIO_PIN_OUTPUT : GPIO_PIN_INPUT;

		snprintf(sc->gpio_pins[i].gp_name, GPIOMAXNAME, "GPIO %d", i);
	}

	/* Disable all GPIO interrupts. */
	WRITE4(sc, GPIOE_0, 0);
	WRITE4(sc, GPIOE_1, 0);
	WRITE4(sc, GPIOEN, 1);

	/*
	 * Register as a pinctrl device
	 */
	fdt_pinctrl_register(dev, NULL);
	fdt_pinctrl_configure_tree(dev);

	sc->busdev = gpiobus_add_bus(dev);
	if (sc->busdev == NULL) {
		device_printf(dev, "Cannot attach gpiobus\n");
		jh7110_gpio_detach(dev);
		return (ENXIO);
	}

	bus_attach_children(dev);
	return (0);
}

static phandle_t
jh7110_gpio_get_node(device_t bus, device_t dev)
{
	return (ofw_bus_get_node(bus));
}

/* fdt_pinctrl configuration */

static struct jh7110_gpio_pincfg {
	uint32_t pinmux;
	uint32_t flags;
}

static void
jh7110_gpio_configure_single_pin(struct jh7110_gpio_softc *sc,
    uint32_t pinmux, uint32_t padcfg, uint32_t padmask)
{
	uint32_t din, dout, doen;
	uint32_t pad;
	u_int pin;

	pin  = (cfg->pinmux[i] >> & 0x3f;
	doen = (cfg->pinmux[i] >> 10) & 0x3f;
	dout = (cfg->pinmux[i] >> 16) & 0xff;
	din  = (cfg->pinmux[i] >> 24) & 0xff;

	/* Not supported yet. */
	if (pin >= GPIO_PINS)
		return;

	padcfg = 0;
	if ((cfg->flags & PINCFG_FLAG_PULLUP) != 0)
		padcfg |= PAD_PULLUP;
	else if ((cfg->flags & PINCFG_FLAG_PULLDOWN) != 0)
		padcfg |= PAD_PULLDOWN;

	if ((cfg->flags & PINCFG_FLAG_SMT) != 0)
		padcfg |= PAD_HYST;

	/* Set output enable state. */
	WRITE4(sc, GP0_DOEN_CFG + GPIO_RW_OFFSET(pin), doen);


	/* Update PAD configuration for pin. */
	pad = READ4(sc, IOMUX_SYSCFG_288 + PAD_OFFSET(pin));
	pad &= ~padmask;
	pad |= padcfg;
	WRITE4(sc, IOMUX_SYSCFG_288 + PAD_OFFSET(pin), pad);

	/* TODO: write registers */
}

static int
jh7110_gpio_configure_pins(device_t dev, phandle_t cfgxref)
{
	struct jh7110_gpio_softc *sc;
	struct jh7110_gpio_pincfg cfg;
	phandle_t node, child;
	pcell_t *pinmux;
	uint32_t flags;
	int ncells;
	u_int pin;

	sc = device_get_softc(dev);
	node = OF_node_from_xref(cfgxref);

	/* Process any and all children. */
	for (child = OF_child(node); child != 0; child = OF_peer(child)) {
		printf("process child node %u\n", child);

		/* Get pinmux */
		ncells = OF_getencprop_alloc_multi(child, "pinmux", sizeof(pcell_t),
		    (void **)&pinmux);
		if (ncells == -1) {
			device_printf(dev, "couldn't find 'pinmux' property\n");
			continue;
		}

		flags = 0;
		padcfg = 0;
		padmask = PAD_INPUT_EN;

		/* Parse input/output */
		if (OF_hasprop(child, "input-enable")) {
			flags |= GPIO_PIN_INPUT;
			padcfg |= PAD_INPUT_EN;
		} else if (OF_hasprop(child, "input-disable"))
			flags |= GPIO_PIN_OUTPUT;

		/* Parse bias */
		if (!OF_hasprop(child, "bias-disable")) {
			if (OF_hasprop(child, "bias-pull-up")) {
				flags |= GPIO_PIN_PULLUP;
				padcfg |= PAD_PULLUP;
			} else if (OF_hasprop(child, "bias-pull-down")) {
				flags |= GPIO_PIN_PULLDOWN;
				padcfg |= PAD_PULLDOWN;
			}
		}

		if (OF_hasprop(child

		/* TODO: parse driver strength */
		/* TODO: parse schmitt (?) */

		for (int i = 0; i < ncells; i++) {



		}
		OF_prop_free(pinmux);
	}

	return (0);
}

static device_method_t jh7110_gpio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		jh7110_gpio_probe),
	DEVMETHOD(device_attach,	jh7110_gpio_attach),
	DEVMETHOD(device_detach,	jh7110_gpio_detach),

	/* GPIO protocol */
	DEVMETHOD(gpio_get_bus,		jh7110_gpio_get_bus),
	DEVMETHOD(gpio_pin_max,		jh7110_gpio_pin_max),
	DEVMETHOD(gpio_pin_get,		jh7110_gpio_pin_get),
	DEVMETHOD(gpio_pin_set,		jh7110_gpio_pin_set),
	DEVMETHOD(gpio_pin_toggle,	jh7110_gpio_pin_toggle),
	DEVMETHOD(gpio_pin_getflags,	jh7110_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_setflags,	jh7110_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_getcaps,	jh7110_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_getname,	jh7110_gpio_pin_getname),

	/* ofw_bus interface */
	DEVMETHOD(ofw_bus_get_node,	jh7110_gpio_get_node),

        /* fdt_pinctrl interface */
	DEVMETHOD(fdt_pinctrl_configure, jh7110_gpio_configure_pins),

	DEVMETHOD_END
};

DEFINE_CLASS_0(gpio, jh7110_gpio_driver, jh7110_gpio_methods,
    sizeof(struct jh7110_gpio_softc));
EARLY_DRIVER_MODULE(jh7110_gpio, simplebus, jh7110_gpio_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
MODULE_DEPEND(jh7110_gpio, gpiobus, 1, 1, 1);
