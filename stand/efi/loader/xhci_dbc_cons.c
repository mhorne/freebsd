/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019-2024 Hiroki Sato <hrs@FreeBSD.org>
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
 *
 */
#include <sys/param.h>
#include <dev/pci/pcireg.h>
#include <dev/usb/controller/xhcireg.h>

#include <bootstrap.h>
#include <efi.h>
#include <efilib.h>

#include "xhci_dbc_cons.h"
#include "xhci_dbc_pci.h"
#include "xhci_dbc_dma.h"
#include <dev/usb/controller/xhci_pci.h>
#include <dev/usb/controller/xhci_private.h>
#include <dev/usb/controller/xhci_dbc.h>
#include <dev/usb/controller/xhci_dbc_private.h>

static void udbc_probe(struct console *);
static bool udbc_alloc(struct console *, EFI_PCI_IO_PROTOCOL *, EFI_HANDLE *);

static int udbc_getc(void);
static void udbc_putc(int);
static int udbc_ischar(void);
static int udbc_init(int);
struct console udb_console = {
	.c_name = "udbc",
	.c_desc = "USB DbC serial port",
	.c_flags = 0,
	.c_probe = udbc_probe,
	.c_init = udbc_init,
	.c_out = udbc_putc,
	.c_in = udbc_getc,
	.c_ready = udbc_ischar
};

struct xhci_debug_softc *udb_sc0;	/* linked-list */
struct xhci_debug_softc *udb_sc;	/* active instance */
const char *udb_hostname;
const char *udb_serial;

static void
udbc_probe(struct console *cons)
{
	EFI_STATUS status;
	EFI_PCI_IO_PROTOCOL *pciio;
	EFI_HANDLE *h0, *h;
	UINTN hlen;
	int i, error;
	device_t dev;

	h0 = h = NULL;
	if (udb_sc != NULL)
		return;

	/* Get PCI I/O handle. */
	hlen = 0;
	status = BS->LocateHandleBuffer(
	    ByProtocol,
	    &pciio_guid,
	    NULL,
	    &hlen,
	    &h0);
	if (EFI_ERROR(status))
		goto error;

	for (UINTN i = 0; i < hlen; i++) {
		h = h0[i];
		status = BS->HandleProtocol(
		    h,
		    &pciio_guid,
		    (VOID **)&pciio);
		if (EFI_ERROR(status))
			continue;
		status = pciio->Pci.Read(
		    pciio,
		    EfiPciIoWidthUint8,
		    0,
		    sizeof(dev),
		    &dev);
		if (EFI_ERROR(status))
			continue;
		if (pci_get_headertype(dev) != PCIM_HDRTYPE_NORMAL &&
		    pci_get_headertype(dev) != PCIM_MFDEV)
			continue;
		if (xhci_pci_match(dev) != NULL)
			(void) udbc_alloc(cons, pciio, h);
	}

	if (udb_sc0 != NULL) {
		cons->c_flags = C_PRESENTIN | C_PRESENTOUT;
		xhci_debug_export_softc(udb_sc0);

		return;
	}
error:
	DEBUG_PRINTF(1, "%s: Compatible xHC not found.\n", __func__);
	(void) BS->FreePool(h0);
	return;
}

static bool
udbc_alloc(struct console *cons, EFI_PCI_IO_PROTOCOL *pciio, EFI_HANDLE *h)
{
	struct xhci_debug_softc *sc;
	UINTN SegmentNumber, BusNumber, DeviceNumber, FunctionNumber;
	EFI_STATUS status;
	int error;

	if (pciio == NULL || h == NULL)
		return (false);
	status = pciio->GetLocation(
	    pciio,
	    &SegmentNumber,
	    &BusNumber,
	    &DeviceNumber,
	    &FunctionNumber);
	if (EFI_ERROR(status))
		return (false);
	sc = udb_sc_malloc(sizeof(*sc), h, pciio);
	if (sc == NULL)
		return (false);
	sc->sc_pci_rid = PCI_RID(BusNumber, DeviceNumber, FunctionNumber);
	sc->sc_efi_pciio = pciio;
	sc->sc_efi_hand = h;

	sc->sc_dbc_off = xhci_debug_probe(sc);
	if (sc->sc_dbc_off == 0)
		goto error;

	xhci_debug_disable(sc);
	xhci_debug_update_state(sc);

	DEBUG_PRINTF(2, "%s: before init\n", __func__);
	if (sc->sc_init == false) {
		error = xhci_debug_init_dma(sc);
		if (error) {
			DEBUG_PRINTF(1, "USB DbC DMA configuration error\n");
			goto error;
		}
		error = xhci_debug_init_ring(sc);
		if (error) {
			DEBUG_PRINTF(1, "USB DbC TRB ring configuration error\n");
			goto error;
		}
		sc->sc_init = true;
		sc->sc_cookie = XHCI_DC_COOKIE;
	}
	xhci_debug_enable(sc);

	/* Add a new entry */
	if (udb_sc0 != NULL)
		sc->sc_next = udb_sc0;
	udb_sc0 = sc;

	return (true);
error:
	/* XXX: free softc */
	return (false);
}

static int
udbc_init(int arg)
{
	struct xhci_debug_softc *sc;
	uint32_t rid;
	char *p, *endp;
	int error;

	p = getenv("hw.usb.xhci.dbc.enable");
	if (p != NULL && p[0] == '0')
		return (CMD_OK);

	p = getenv("hw.usb.xhci.dbc.debug");
	if (p != NULL) {
		dbc_debug = strtol(p, &endp, 0);
		if (*endp != '\0')
			dbc_debug = 0;
	}

	rid = 0;
	p = getenv("hw.usb.xhci.dbc.pci_rid");
	if (p != NULL) {
		rid = strtol(p, &endp, 0);
		if (*endp != '\0')
			rid = 0;
	}

	/* XXX: too late */
	udb_hostname = getenv("hw.usb.xhci.dbc.hostname");
	udb_serial = getenv("hw.usb.xhci.dbc.serial");

	for (sc = udb_sc0; sc != NULL; sc->sc_next) {
		/* If RID is not specified, use the first one. */
		if ((rid == 0 || rid == sc->sc_pci_rid) &&
		    sc->sc_dbc_off != 0)
			break;
	}
	if (sc == NULL) {		/* allocated in c_probe */
		DEBUG_PRINTF(1, "USB DbC not found\n");
		return (CMD_ERROR);
	}

	p = getenv("hw.usb.xhci.dbc.gdb");
	if (p != NULL && p[0] != '0')
		sc->sc_flags |= XHCI_DEBUG_FLAGS_GDB;

	xhci_debug_enable(sc);
	udb_sc = sc;
	xhci_debug_event_dequeue(sc);

	return (CMD_OK);
}

static void
udbc_putc(int c0)
{
	struct xhci_debug_softc *sc = udb_sc;
	u_char c = (0xff & c0);

	if (sc == NULL)
		return;
	if (work_enqueue(&sc->udb_oring, &c, sizeof(c)) != sizeof(c))
		return;

	xhci_debug_bulk_transfer(sc);
	/*
	 * This dequeue just after the transfer is important.
	 * Do not remove this.
	 */
	xhci_debug_event_dequeue(sc);
}

/* udbc_getc() is called periodically. */
static int
udbc_getc(void)
{
	struct xhci_debug_softc *sc = udb_sc;

	if (sc == NULL)
		return (-1);

	/* Use udbc_ischar() to receive data */
	return (udbc_ischar() ? work_dequeue(&sc->udb_iring) : -1);
}

static int
udbc_ischar(void)
{
	struct xhci_debug_softc *sc = udb_sc;
	struct xhci_debug_ring *iring;

	if (sc == NULL)
		return (0);
	iring = &sc->udb_iring;
	if (!DC_WORK_RING_EMPTY(iring))
		return (1);

	xhci_debug_bulk_transfer(sc);
	xhci_debug_event_dequeue(sc);

	return (!DC_WORK_RING_EMPTY(iring));
}
