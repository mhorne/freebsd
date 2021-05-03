#-
# Copyright (c) 2015 Luiz Otavio O Souza <loos@FreeBSD.org>
# Copyright (c) 2014 Ruslan Bukin <br@bsdpad.com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
# $FreeBSD$
#

INTERFACE if_dwc;

#include <dev/dwc/if_dwc.h>
#include <sys/bus.h>
#include <sys/socket.h>
#include <vm/vm.h>
#include <vm/pmap.h>

#include <net/if.h>
#include <net/if_media.h>

#include <dev/dwc/if_dwcvar.h>
#include <machine/pmap.h>

CODE {
	static int
	if_dwc_default_init(device_t dev)
	{
		device_printf(dev, "%s\n", __func__);
		return (0);
	}

	static int
	if_dwc_default_mac_type(device_t dev)
	{
		device_printf(dev, "%s\n", __func__);
		return (DWC_GMAC_EXT_DESC);
	}

	static int
	if_dwc_default_mii_clk(device_t dev)
	{
		device_printf(dev, "%s\n", __func__);
		return (GMAC_MII_CLK_25_35M_DIV16);
	}

	static int
	if_dwc_default_set_speed(device_t dev, int speed)
	{
		struct dwc_softc *sc;
		uint32_t *addr;
		uint32_t val;

		device_printf(dev, "%s\n", __func__);
		sc = device_get_softc(dev);

		addr = pmap_mapdev(0x118001EC, 4);
		val = *addr;
		printf("val: %x\n", val);

		switch(speed) {
		case IFM_1000_T:
		case IFM_1000_SX:
			printf("speed 1000\n");
			val |= 0x4;
			*addr = val;
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

		return (0);
	}
};

HEADER {
};

#
# Initialize the SoC specific registers.
#
METHOD int init {
	device_t dev;
} DEFAULT if_dwc_default_init;

#
# Return the DWC MAC type (descriptor type).
#
METHOD int mac_type {
	device_t dev;
} DEFAULT if_dwc_default_mac_type;

#
# Return the DWC MII clock for a specific hardware.
#
METHOD int mii_clk {
	device_t dev;
} DEFAULT if_dwc_default_mii_clk;

#
# Signal media change to a specific hardware
#
METHOD int set_speed {
	device_t dev;
	int speed;
} DEFAULT if_dwc_default_set_speed;
