/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2010-2022 Hans Petter Selasky
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
#include <sys/cdefs.h>
#include <sys/stdint.h>
#include <sys/stddef.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>

#define	PCI_XHCI_VENDORID_AMD		0x1022
#define	PCI_XHCI_VENDORID_INTEL		0x8086
#define	PCI_XHCI_VENDORID_VMWARE	0x15ad
#define	PCI_XHCI_VENDORID_ZHAOXIN	0x1d17

static inline const char *
xhci_pci_match(device_t self)
{
	uint32_t device_id = pci_get_devid(self);

	switch (device_id) {
	case 0x145c1022:
		return ("AMD KERNCZ USB 3.0 controller");
	case 0x148c1022:
		return ("AMD Starship USB 3.0 controller");
	case 0x149c1022:
		return ("AMD Matisse USB 3.0 controller");
	case 0x15e01022:
	case 0x15e11022:
		return ("AMD Raven USB 3.1 controller");
	case 0x43ba1022:
		return ("AMD X399 USB 3.0 controller");
	case 0x43b91022: /* X370 */
	case 0x43bb1022: /* B350 */
		return ("AMD 300 Series USB 3.1 controller");
	case 0x43d51022:
		return ("AMD 400 Series USB 3.1 controller");
	case 0x78121022:
	case 0x78141022:
	case 0x79141022:
		return ("AMD FCH USB 3.0 controller");

	case 0x077815ad:
	case 0x077915ad:
		return ("VMware USB 3.0 controller");

	case 0x145f1d94:
		return ("Hygon USB 3.0 controller");

	case 0x01941033:
		return ("NEC uPD720200 USB 3.0 controller");
	case 0x00151912:
		return ("NEC uPD720202 USB 3.0 controller");

	case 0x10001b73:
		return ("Fresco Logic FL1000G USB 3.0 controller");
	case 0x10091b73:
		return ("Fresco Logic FL1009 USB 3.0 controller");
	case 0x11001b73:
		return ("Fresco Logic FL1100 USB 3.0 controller");

	case 0x10421b21:
		return ("ASMedia ASM1042 USB 3.0 controller");
	case 0x11421b21:
		return ("ASMedia ASM1042A USB 3.0 controller");
	case 0x13431b21:
		return ("ASMedia ASM1143 USB 3.1 controller");
	case 0x32421b21:
		return ("ASMedia ASM3242 USB 3.2 controller");

	case 0x0b278086:
		return ("Intel Goshen Ridge Thunderbolt 4 USB controller");
	case 0x0f358086:
		return ("Intel BayTrail USB 3.0 controller");
	case 0x11388086:
		return ("Intel Maple Ridge Thunderbolt 4 USB controller");
	case 0x15c18086:
	case 0x15d48086:
	case 0x15db8086:
		return ("Intel Alpine Ridge Thunderbolt 3 USB controller");
	case 0x15e98086:
	case 0x15ec8086:
	case 0x15f08086:
		return ("Intel Titan Ridge Thunderbolt 3 USB controller");
	case 0x19d08086:
		return ("Intel Denverton USB 3.0 controller");
	case 0x9c318086:
	case 0x1e318086:
		return ("Intel Panther Point USB 3.0 controller");
	case 0x22b58086:
		return ("Intel Braswell USB 3.0 controller");
	case 0x31a88086:
		return ("Intel Gemini Lake USB 3.0 controller");
	case 0x34ed8086:
		return ("Intel Ice Lake-LP USB 3.1 controller");
	case 0x43ed8086:
		return ("Intel Tiger Lake-H USB 3.2 controller");
	case 0x461e8086:
		return ("Intel Alder Lake-P Thunderbolt 4 USB controller");
	case 0x51ed8086:
		return ("Intel Alder Lake PCH USB 3.2 controller");
	case 0x5aa88086:
		return ("Intel Apollo Lake USB 3.0 controller");
	case 0x7ae08086:
		return ("Intel Alder Lake USB 3.2 controller");
	case 0x8a138086:
		return ("Intel Ice Lake Thunderbolt 3 USB controller");
	case 0x8c318086:
		return ("Intel Lynx Point USB 3.0 controller");
	case 0x8cb18086:
		return ("Intel Wildcat Point USB 3.0 controller");
	case 0x8d318086:
		return ("Intel Wellsburg USB 3.0 controller");
	case 0x9a138086:
		return ("Intel Tiger Lake-LP Thunderbolt 4 USB controller");
	case 0x9a178086:
		return ("Intel Tiger Lake-H Thunderbolt 4 USB controller");
	case 0x9cb18086:
		return ("Broadwell Integrated PCH-LP chipset USB 3.0 controller");
	case 0x9d2f8086:
		return ("Intel Sunrise Point-LP USB 3.0 controller");
	case 0xa0ed8086:
		return ("Intel Tiger Lake-LP USB 3.2 controller");
	case 0xa12f8086:
		return ("Intel Sunrise Point USB 3.0 controller");
	case 0xa1af8086:
		return ("Intel Lewisburg USB 3.0 controller");
	case 0xa2af8086:
		return ("Intel Union Point USB 3.0 controller");
	case 0xa36d8086:
		return ("Intel Cannon Lake USB 3.1 controller");
	case 0xa71e8086:
		return ("Intel Raptor Lake-P Thunderbolt 4 USB Controller");

	case 0xa01b177d:
		return ("Cavium ThunderX USB 3.0 controller");

	case 0x1ada10de:
		return ("NVIDIA TU106 USB 3.1 controller");

	case 0x92021d17:
		return ("Zhaoxin ZX-100 USB 3.0 controller");
	case 0x92031d17:
		return ("Zhaoxin ZX-200 USB 3.0 controller");
	case 0x92041d17:
		return ("Zhaoxin ZX-E USB 3.0 controller");

	default:
		break;
	}

	if ((pci_get_class(self) == PCIC_SERIALBUS)
	    && (pci_get_subclass(self) == PCIS_SERIALBUS_USB)
	    && (pci_get_progif(self) == PCIP_SERIALBUS_USB_XHCI)) {
		return ("XHCI (generic) USB 3.0 controller");
	}
	return (NULL);			/* dunno */
}
