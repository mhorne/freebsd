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
#ifndef _KERNEL
/* for struct mtx in xhci_debug_softc */
#include <sys/lock.h>
#include <sys/mutex.h>

void mtx_lock_spin(struct mtx *);
int mtx_trylock_spin(struct mtx *);
void mtx_unlock_spin(struct mtx *);
#endif

/* DbC idVendor and idProduct */
#define DC_VENDOR	0x1d6b	/* Linux Foundation */
#define DC_PRODUCT	0x0011	/* Linux */ 
#define DC_PROTOCOL	0x0000	/* GNU GDB = 1 */
#define DC_REVISION	0x0010	/* 1.0 */

#define	DC_STRING_MANUFACTURER	"The FreeBSD Foundation"
#define	DC_STRING_PRODUCT	"FreeBSD USB DbC Serial"
#define	DC_STRING_SERIAL	"12345678"

#define XHCI_DCPORTSC_ACK_MASK	\
	(XHCI_DCPORTSC_PED | \
	 XHCI_DCPORTSC_CSC | XHCI_DCPORTSC_PRC | \
	 XHCI_DCPORTSC_PLC | XHCI_DCPORTSC_CEC)

/* DbC endpoint types */
enum {
	EP_BULK_OUT = 2,
	EP_BULK_IN = 6
};

#define UDB_TRB_PER_PAGE (XHCI_PAGE_SIZE / sizeof(struct xhci_trb))

/* Defines the size in bytes of TRB rings as 2^UDB_TRB_RING_ORDER * 4096 */
#define UDB_TRB_RING_ORDER	4
#define UDB_TRB_RING_LEN (UDB_TRB_PER_PAGE * (1ULL << UDB_TRB_RING_ORDER))
#define UDB_TRB_RING_OFFSET_MASK (UDB_TRB_RING_LEN - 1U)
#define UDB_TRB_RING_LAST	(UDB_TRB_RING_LEN - 1U)
#define UDB_TRB_RING_BYTES	(UDB_TRB_RING_LEN * sizeof(struct xhci_trb))
#define UDB_TRB_RING_MASK	(UDB_TRB_RING_BYTES - 1U)

/* Defines the size in bytes of work rings as 2^WORK_RING_ORDER * 4096 */
#define	UDB_WORK_RING_ORDER	3
#define UDB_WORK_RING_LEN	(XHCI_PAGE_SIZE * (1ULL << UDB_WORK_RING_ORDER))
#define UDB_WORK_RING_OFFSET_MASK	(UDB_WORK_RING_LEN - 1U)
#define UDB_WORK_RING_LAST	(UDB_WORK_RING_LEN - 1U)
#define	UDB_WORK_RING_BYTES	UDB_WORK_RING_LEN

#define	UDB_WORK_RING_FULL(r) \
	((((r)->work.enq + 1) & UDB_WORK_RING_OFFSET_MASK) == (r)->work.deq)
#define	UDB_WORK_RING_EMPTY(r) \
	((r)->work.enq == (r)->work.deq)
#define	XHCI_DEBUG_RING_FULL(r) \
	((((r)->enq + 1) & UDB_TRB_RING_OFFSET_MASK) == (r)->deq)
#define	XHCI_DEBUG_RING_EMPTY(r) \
	((r)->enq == (r)->deq)
#define	XHCI_DEBUG_RING_PENDING(r) \
	((r)->deq <= (r)->enq \
	    ? (r)->enq - (r)->deq \
	    : UDB_TRB_RING_LEN - (r)->deq - 1 + (r)->enq)

struct xhci_debug_reg {
	uint32_t id;
	uint32_t doorbell;
	uint32_t erstsz;
	uint32_t rsvdz;
	uint64_t erstba;
	uint64_t erdp;
	uint32_t ctrl;
	uint32_t st;
	uint32_t portsc;
	uint32_t rsvdp;
	uint64_t cp;
	uint32_t ddi1;
	uint32_t ddi2;
};

struct xhci_debug_ctx {
	uint32_t info[16];
	struct xhci_endp_ctx64 ep_out;
	struct xhci_endp_ctx64 ep_in;
};

struct xhci_debug_work_ring {
	uint8_t *buf;
	uint32_t enq;
	uint32_t deq;
	uint64_t paddr;
};

struct xhci_debug_ring {
	struct xhci_trb	*trb;	/* Array of TRBs */
	uint64_t	paddr;	/* base address of TRBs */
	uint32_t	enq;	/* The offset of the enqueue ptr */
	uint32_t	deq;	/* The offset of the dequeue ptr */
	uint8_t		cyc;	/* Cycle state toggled on each wrap-around */
	uint32_t	doorbell;	/* Doorbell target */
#define	XHCI_DEBUG_RING_IN(ring)	((ring)->doorbell == XHCI_DCDB_IN)
#define	XHCI_DEBUG_RING_OUT(ring)	((ring)->doorbell == XHCI_DCDB_OUT)
#define	XHCI_DEBUG_RING_EV(ring)	((ring)->doorbell == XHCI_DCDB_INVALID)
	struct xhci_debug_work_ring work;

	struct mtx	mtx;
	uint32_t	flags;
#define	XHCI_DEBUG_RING_F_IN_PROGRESS	0x0001
	uint32_t lasterror;
	uint32_t lastenq;
	uint32_t lastdeq;
/*
 * Bitmap for completion report.
 *
 * Set when enqueuing a normal TRB, and
 * reset when receiving the corrensponding transfer event TRB.
 */
	uint8_t		iocbitmap[UDB_TRB_RING_LEN / 8 + 1];
#define	IOCBITMAP_GET(ring, n)	\
	(((ring)->iocbitmap[(n) / 8] >> ((uint8_t)(n) % 8)) & (uint8_t)1)
#define	IOCBITMAP_SET(ring, n)	\
	((ring)->iocbitmap[(n) / 8] |=  ((uint8_t)1 << (uint8_t)((n) % 8)))
#define	IOCBITMAP_CLEAR(ring, n)	\
	((ring)->iocbitmap[(n) / 8] &= ~((uint8_t)1 << (uint8_t)((n) % 8)))

	uint64_t ec[256];	/* error counter */
#define	XHCI_TRB_ERROR_EVLOCKED	0x30
#define	XHCI_TRB_ERROR_CALLED	0x31
#define	XHCI_TRB_ERROR_UNKNOWN	0xff
};

#ifndef _KERNEL
#define DMA_DESC_CAP 16
struct dma {
	UINTN pages;
	EFI_PHYSICAL_ADDRESS paddr;
	VOID *cpu_addr;
	VOID *mapping;
};
#endif

/*
 * This softc is used in kernel and loader.
 */
struct xhci_debug_softc {
	struct xhci_debug_reg	reg;
	struct xhci_debug_ctx	*udb_ctx;
	uint64_t		udb_ctx_paddr;
	struct xhci_event_ring_seg *udb_erst;
	uint64_t		udb_erst_paddr;
	struct xhci_debug_ring	udb_ering;
	struct xhci_debug_ring	udb_oring;
	struct xhci_debug_ring	udb_iring;
	char			*udb_str;
	uint64_t		udb_str_paddr;
	struct xhci_debug_softc	*sc_udb;

	uint32_t		sc_state;
	uint64_t		sc_polling_count;
	bool			sc_init;
	bool			sc_init_dma;
	bool			sc_fixup_done;
	bool			sc_open;

	struct mtx		sc_mtx;

	/* in kernel, sc_*_off in struct xhci_softc is used */
	uint32_t		sc_dbc_off;
#ifdef _KERNEL
	struct xhci_softc	*sc_xhci;
#else
	uint32_t		sc_capa_off;
	/*
	 * XXX: X{READ,WRITE}4 macro in a loader will use sc_mmio_phys
	 * instead of bus_dma(9) at this moment.
	 */
	volatile uintptr_t	sc_mmio_phys;
	uint64_t		sc_mmio_size;
	void			*sc_mmio;

	uint32_t		sc_ca;
	EFI_HANDLE		sc_efi_hand;
	EFI_PCI_IO_PROTOCOL	*sc_efi_pciio;
	struct dma		dma_desc[DMA_DESC_CAP];

	/* loader console */
	struct console		*sc_cons;
#endif
};

int xhci_debug_probe(struct xhci_debug_softc *);
bool xhci_debug_enable(struct xhci_debug_softc *);
void xhci_debug_disable(struct xhci_debug_softc *);

int xhci_debug_init_ring(struct xhci_debug_softc *);
void xhci_debug_update_state(struct xhci_debug_softc *);

uint64_t xhci_debug_bulk_transfer(struct xhci_debug_softc *);
void xhci_debug_event_dequeue(struct xhci_debug_softc *);

int work_dequeue(struct xhci_debug_ring *);
int64_t work_enqueue(struct xhci_debug_ring *, const char *, int64_t);

void udb_dump_info(struct xhci_debug_softc *);

#ifdef _KERNEL
int udbcons_init(struct xhci_softc *, device_t);
#endif
