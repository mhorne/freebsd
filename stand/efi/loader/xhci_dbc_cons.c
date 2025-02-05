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

#include "pci.h"
#include "xhci_dbc_cons.h"
#include "xhci_dbc_pci.h"
#include "xhci_dbc_dma.h"	/* udb_init_dma() */
#include <dev/usb/controller/xhci_private.h>
#include <dev/usb/controller/xhci_dbc.h>
#include <dev/usb/controller/xhci_dbc_private.h>

static void udbc_probe(struct console *);
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

struct xhci_debug_softc *udb_sc;

static void
udbc_probe(struct console *cons)
{
	struct xhci_debug_softc *sc;
	EFI_STATUS status;
	EFI_HANDLE *h0, *h;
	EFI_PCI_IO_PROTOCOL *pciio;
	UINTN hlen;
	uint32_t devfn;
	char buf[32];
	int i, error;
	int found;

	h0 = h = NULL;
	if (udb_sc != NULL)
		return;

	/* Get PCI I/O handle. */
	hlen = 0;
	status = BS->LocateHandle(ByProtocol, &pciio_guid, NULL, &hlen, NULL);
	DEBUG_PRINTF(1, "%s: LocateHandle1 = %lx\n", __func__, status);
	if (status != EFI_BUFFER_TOO_SMALL)
		goto error;
	h0 = malloc(hlen);
	status = BS->LocateHandle(ByProtocol, &pciio_guid, NULL, &hlen, h0);
	DEBUG_PRINTF(1, "%s: LocateHandle2 = %lu\n", __func__, status);
	if (status != EFI_SUCCESS)
		goto error;
	hlen /= sizeof(EFI_HANDLE);

	/* Look up vendor IDs. */
	found = 0;
	for (h = h0; h < h0 + hlen; h++) {
		UINT32 dev_ven;

		status = OpenProtocolByHandle(*h, &pciio_guid,
		    (void **)&pciio);
		if (status != EFI_SUCCESS)
			continue;
		status = pciio->Pci.Read(pciio, EfiPciIoWidthUint32,
		    PCIR_DEVVENDOR, 1, &dev_ven);
		if (status != EFI_SUCCESS) {
			BS->CloseProtocol(h, &pciio_guid, IH, NULL);
			continue;
		}
		DEBUG_PRINTF(1, "%s: dev_ven = %08x\n", __func__, dev_ven);
		for (i = 0; i < nitems(ids); i++) {
			if (dev_ven ==
			    ((ids[i].device << 16) | ids[i].vendor)) {
				found = 1;
				break;
			}
		}
	}
	if (found == 0)
		goto error;

	sc = udb_sc_malloc(sizeof(*sc), h, pciio);
	if (sc == NULL)
		return;
	udb_sc = sc;
	sc->sc_cons = cons;

	h0 = NULL;
	for (devfn = 0; devfn < 256; devfn++) {
		uint32_t sc_ca;
		uint32_t header_type;
		uint32_t class;

		sc_ca = 0;
		sc_ca |= PCIR_IO_CA_ENABLE;
		sc_ca |= PCIR_IO_CA_DEV_SET(devfn >> 3);
		sc_ca |= PCIR_IO_CA_FUN_SET(devfn);

		header_type =
		    (pci_read_config(sc_ca, PCIR_CA_HEADER) >> 16) & 0xff;

		if (header_type != PCIM_HDRTYPE_NORMAL &&
		    header_type != PCIM_MFDEV)
			continue;
		class = pci_read_config(sc_ca, PCIR_CA_CLASS) >> 8;
		if (class == XHC_CLASSC) {
			sc->sc_ca = sc_ca;
			break;
		}
	}
	if (sc->sc_ca == 0) {
		DEBUG_PRINTF(1, "%s: Compatible xHC not found.\n", __func__);
		goto error;
	}
	if (!xhci_debug_probe(sc))
		goto error;

	xhci_debug_disable(sc);
	cons->c_flags = C_PRESENTIN | C_PRESENTOUT;

	xhci_debug_update_state(sc);
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
	}
	xhci_debug_enable(sc);
	xhci_debug_update_state(sc);

	return;
error:
	free(h0);
	free(sc);
	sc = NULL;
	return;
}

static int
udbc_init(int arg)
{
	struct xhci_debug_softc *sc = udb_sc;
	int error;

	if (sc == NULL) {		/* allocated in c_probe */
		DEBUG_PRINTF(1, "USB DbC not found\n");
		return (CMD_ERROR);
	}
	if (sc->sc_dbc_off == 0) {	/* set in c_probe */
		DEBUG_PRINTF(1, "USB DbC register not found\n");
		return (CMD_ERROR);
	}
	xhci_debug_enable(sc);
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
	if (!work_enqueue(&sc->udb_oring, &c, sizeof(c)))
		return;

	xhci_debug_bulk_transfer(sc);
}

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
	if (!UDB_WORK_RING_EMPTY(iring))
		return (1);
	
	iring = &sc->udb_iring;

	xhci_debug_bulk_transfer(sc);
	xhci_debug_event_dequeue(sc);

	return (!UDB_WORK_RING_EMPTY(iring));
}
