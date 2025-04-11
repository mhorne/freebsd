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
#include <machine/atomic.h>

#include <assert.h>

#include <bootstrap.h>
#include <efi.h>
#include <efilib.h>
#include <dev/usb/controller/xhci_private.h>
#include "xhci_dbc_pci.h"
#include <dev/usb/controller/xhci_dbc.h>
#include <dev/usb/controller/xhci_dbc_private.h>
#include "xhci_dbc_cons.h"
#include "xhci_dbc_dma.h"

EFI_GUID pciio_guid = EFI_PCI_IO_PROTOCOL_GUID;

static void *udb_alloc_dma(struct xhci_debug_softc *, uint64_t);
static void udb_free_dma(struct xhci_debug_softc *, VOID *);
static uint64_t v2p(struct xhci_debug_softc *, void *);

uint64_t
v2p(struct xhci_debug_softc *sc, void *virt)
{
	UINTN i;
	UINTN p;
	UINTN offset;
	UINTN needed;
	UINTN mapped;
	struct dma *dma;
	EFI_STATUS status;
	EFI_PHYSICAL_ADDRESS paddr;
	VOID *mapping;

	dma = NULL;
	for (i = 0; i < nitems(sc->dma_desc); i++) {
		dma = &sc->dma_desc[i];

		DEBUG_PRINTF(1, "%s: dma_desc[%lu].cpu_addr = %p\n", __func__,
		    i, dma->cpu_addr);
		for (p = 0; p < dma->pages; p++) {
			UINTN addr = (UINTN)dma->cpu_addr + (p * PAGE_SIZE);

			DEBUG_PRINTF(1, "%s: check addr: %lu = %lu + %lu * %u\n",
			    __func__, addr, (UINTN)dma->cpu_addr, p, PAGE_SIZE);
			if ((UINTN)virt == addr) {
				offset = addr - (UINTN)dma->cpu_addr;
				goto found;
			}
		}
		dma = NULL;
	}
	if (dma == NULL) {
		DEBUG_PRINTF(1, "CPU addr %p not found in DMA descriptor\n", virt);
		return 0;
	}

found:
#if 0
	DEBUG_PRINTF(1, "%s: found, i = %lu, dma->paddr = %lu, offset = %lu\n",
	    __func__, i, dma->paddr, offset);
#endif
	if (dma->paddr && dma->mapping)
		return (dma->paddr + offset);

	needed = dma->pages << EFI_PAGE_SHIFT;
	mapped = needed;
	status = sc->sc_efi_pciio->Map(sc->sc_efi_pciio,
	    EfiPciIoOperationBusMasterCommonBuffer,
	    (void *)virt, &mapped, &paddr, &mapping);
	if (EFI_ERROR(status) || mapped != needed) {
		DEBUG_PRINTF(1, "pciio->Map failed: rc: 0x%lx, mapped: %lu, needed: %lu\n",
		    status, mapped, needed);
		return (0);
	}
	dma->paddr = paddr;
	dma->mapping = mapping;

	if ((const void *)paddr != virt) {
		DEBUG_PRINTF(1, "Non-identity DMA mapping: dma: 0x%lx cpu: %p\n",
		    paddr, virt);
	}

	return (paddr);
}

struct xhci_debug_softc *
udb_alloc_softc(size_t len, EFI_HANDLE *h, EFI_PCI_IO_PROTOCOL *pciio)
{
	struct xhci_debug_softc *sc;
	EFI_STATUS status;
	VOID *addr;
	int pages;

	DEBUG_PRINTF(2, "%s: called\n", __func__);
	sc = NULL;
	pages = sizeof(*sc) / PAGE_SIZE + 1;
	status = pciio->AllocateBuffer(pciio,
	    AllocateAnyPages,
	    EfiRuntimeServicesData,
	    pages,
	    &addr,
	    EFI_PCI_IO_ATTRIBUTE_MEMORY_CACHED);
	if (EFI_ERROR(status)) {
		DEBUG_PRINTF(1, "%s: AllocateBuffer failed: 0x%lx\n",
		    __func__, status);
		return (NULL);
	}
	memset(addr, 0, sizeof(*sc));
	sc = (struct xhci_debug_softc *)addr;

	sc->dma_desc[0].pages = pages;
	sc->dma_desc[0].cpu_addr = addr;
	sc->sc_efi_pciio = pciio;
	sc->sc_efi_hand = h;
	sc->sc_init_dma = false;
	sc->sc_init = false;
	if (udb_hostname != NULL)
		strlcpy(sc->sc_hostname, udb_hostname, sizeof(sc->sc_hostname));
	if (udb_serial != NULL)
		strlcpy(sc->sc_serial, udb_serial, sizeof(sc->sc_serial));

	return (sc);
}

void
xhci_debug_export_softc(struct xhci_debug_softc *sc0)
{
	struct xhci_debug_softc *sc;
	uint64_t len;
	char buf[256];

	if (sc0 == NULL)
		return;

	len = 0;
	for (sc = sc0; sc != NULL; sc = sc->sc_next)
		len += sizeof(*sc);

	snprintf(buf, sizeof(buf), "%p", (void *)(uintptr_t)v2p(sc0, sc0));
	buf[sizeof(buf) - 1] = '\0';
	setenv("hw.usb.xhci.dbc.softc.paddr", buf, 1);

	snprintf(buf, sizeof(buf), "%zu", len);
	buf[sizeof(buf) - 1] = '\0';
	setenv("hw.usb.xhci.dbc.softc.len", buf, 1);
}

int
xhci_debug_init_dma(struct xhci_debug_softc *sc)
{
	struct xhci_debug_softc *sc_udb;
#define	ALLOC_DMA(p, type, order)				\
	do { 							\
		p = (type)udb_alloc_dma(sc, order); 		\
		if (p == NULL) { 				\
			xhci_debug_uninit_dma(sc);		\
			return (1);				\
		}						\
	} while (0)

	if (sc->sc_init_dma)
		return (0);

	ALLOC_DMA(sc->udb_ctx, struct xhci_debug_ctx *, 0);
	ALLOC_DMA(sc->udb_erst, struct xhci_event_ring_seg *, 0);
	ALLOC_DMA(sc->udb_str, char *, 0);
	ALLOC_DMA(sc->udb_ering.trb, struct xhci_trb *, DC_TRB_RING_ORDER);
	ALLOC_DMA(sc->udb_iring.trb, struct xhci_trb *, DC_TRB_RING_ORDER);
	ALLOC_DMA(sc->udb_oring.trb, struct xhci_trb *, DC_TRB_RING_ORDER);
	ALLOC_DMA(sc->udb_ering.work.buf, uint8_t *, DC_WORK_RING_ORDER);
	ALLOC_DMA(sc->udb_iring.work.buf, uint8_t *, DC_WORK_RING_ORDER);
	ALLOC_DMA(sc->udb_oring.work.buf, uint8_t *, DC_WORK_RING_ORDER);

	/* Export CTX, ERST, and STR */
#define	SETADDR(n, len) do { \
		sc->udb_##n##_paddr = (uintptr_t)v2p(sc, sc->udb_##n); \
		sc->udb_##n##_len = len; \
	} while (0)
	SETADDR(ctx, PAGE_SIZE);
	SETADDR(erst, PAGE_SIZE);
	SETADDR(str, PAGE_SIZE);

	/* Export TRB rings. */
#define	SETADDR_RING(n, paddr, lenp, buf, len) do { \
		sc->udb_##paddr = (uintptr_t)v2p(sc, sc->udb_##buf); \
		sc->udb_##lenp = len; \
	} while (0)
	
	SETADDR_RING(ering, ering.paddr, ering.len, ering.trb,
	    PAGE_SIZE * (1UL << DC_TRB_RING_ORDER));
	SETADDR_RING(iring, iring.paddr, iring.len, iring.trb,
	    PAGE_SIZE * (1UL << DC_TRB_RING_ORDER));
	SETADDR_RING(oring, oring.paddr, oring.len, oring.trb,
	    PAGE_SIZE * (1UL << DC_TRB_RING_ORDER));
	SETADDR_RING(ering.work, ering.work.paddr, ering.work.len,
	    ering.work.buf, PAGE_SIZE * (1UL << DC_WORK_RING_ORDER));
	SETADDR_RING(iring.work, iring.work.paddr, iring.work.len,
	    iring.work.buf, PAGE_SIZE * (1UL << DC_WORK_RING_ORDER));
	SETADDR_RING(oring.work, oring.work.paddr, oring.work.len,
	    oring.work.buf, PAGE_SIZE * (1UL << DC_WORK_RING_ORDER));

	sc->sc_init_dma = true;

	return (0);
#undef	ALLOC_DMA
}

static void *
udb_alloc_dma(struct xhci_debug_softc *sc, uint64_t order)
{
	struct dma *dma;
	EFI_STATUS status;
	VOID *addr;
	UINTN pages = 1UL << order;
	UINTN i;

	for (i = 0; i < nitems(sc->dma_desc); i++) {
		dma = &sc->dma_desc[i];
		if (dma->cpu_addr == NULL)
			break;
	}
	if (i == nitems(sc->dma_desc)) {
		DEBUG_PRINTF(1, "%s: Out of DMA descriptor\n", __func__);
		return (NULL);
	}
	status = sc->sc_efi_pciio->AllocateBuffer(sc->sc_efi_pciio,
	    AllocateAnyPages,
	    EfiRuntimeServicesData,
	    pages,
	    &addr,
	    EFI_PCI_IO_ATTRIBUTE_MEMORY_CACHED);
	if (EFI_ERROR(status)) {
		DEBUG_PRINTF(1, "%s: AllocateBuffer failed: 0x%lx\n",
		    __func__, status);
		return (NULL);
	}

	dma->pages = pages;
	dma->cpu_addr = addr;
	DEBUG_PRINTF(1, "%s: i = %lu, cpu_addr = %p, pages = %lu\n", __func__,
	    i, dma->cpu_addr, pages);

	return (addr);
}

static void
udb_free_dma(struct xhci_debug_softc *sc, VOID *addr)
{
	struct dma *dma;
	EFI_STATUS status;
	UINTN i;

	for (i = 0; i < nitems(sc->dma_desc); i++) {
		dma = &sc->dma_desc[i];
		if (dma->cpu_addr == addr)
			break;
	}
	if (i == nitems(sc->dma_desc))
		return;
	if (dma->mapping) {
		status = sc->sc_efi_pciio->Unmap(sc->sc_efi_pciio,
		    dma->mapping);
		if (EFI_ERROR(status)) {
			DEBUG_PRINTF(1, "%s: Unmap failed: 0x%lx\n",
			    __func__, status);
			return;
		}
	}
	status = sc->sc_efi_pciio->FreeBuffer(sc->sc_efi_pciio, dma->pages,
	    addr);
	if (EFI_ERROR(status)) {
		DEBUG_PRINTF(1, "%s: FreeBuffer failed: 0x%lx\n",
		    __func__, status);
		return;
	}
	memset(&dma, 0, sizeof(*dma));
}

void
xhci_debug_uninit_dma(struct xhci_debug_softc *sc)
{
	udb_free_dma(sc, sc->udb_ctx);
	udb_free_dma(sc, sc->udb_erst);
	udb_free_dma(sc, sc->udb_ering.trb);
	udb_free_dma(sc, sc->udb_oring.trb);
	udb_free_dma(sc, sc->udb_iring.trb);
	udb_free_dma(sc, sc->udb_ering.work.buf);
	udb_free_dma(sc, sc->udb_oring.work.buf);
	udb_free_dma(sc, sc->udb_iring.work.buf);

	sc->sc_init_dma = false;
}
