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
#include <sys/condvar.h>
#include <sys/systm.h>	/* getenv_quad */
#include <sys/lock.h>
#include <sys/mutex.h>

#ifdef _KERNEL
# include <sys/bus.h>
# include <sys/cons.h>
# include <sys/kdb.h>
# include <sys/tty.h>
# include <sys/reboot.h>
# include <sys/sbuf.h>
# include <sys/sysctl.h>
# include <vm/vm.h>
# include <vm/vm_param.h>
# include <vm/pmap.h>
# include <dev/usb/usb.h>
# include <dev/usb/usbdi.h>
# include <dev/usb/usb_core.h>
# include <dev/usb/usb_busdma.h>
# include <dev/usb/usb_process.h>
# include <dev/usb/usb_transfer.h>
# include <dev/usb/usb_device.h>
# include <dev/usb/usb_controller.h>
# include <dev/usb/usb_bus.h>
# include <dev/usb/controller/xhci.h>
# ifdef __aarch64__
# include <machine/cpufunc.h>
# endif
#else
# include <bootstrap.h>
# include <efi.h>
# include <efilib.h>
# include <dev/usb/controller/xhci_private.h>
#endif

#include <dev/usb/controller/xhcireg.h>
#include <dev/usb/controller/xhci_dbc.h>
#include <dev/usb/controller/xhci_dbc_private.h>

#ifndef _KERNEL
#include "xhci_dbc_cons.h"
#include "xhci_dbc_dma.h"
#endif

#ifdef	_KERNEL
#define	SBUF_PRINTF(sb, ...)	sbuf_printf(sb, __VA_ARGS__)
#define	SBUF_FINISH(sb)		sbuf_finish(sb)
#define	SBUF_DELETE(sb)		sbuf_delete(sb)
#define	SBUF_NEW_AUTO()		sbuf_new_auto()
#define	SBUF_DATA(sb)		sbuf_data(sb)
#define	SBUF_LEN(sb)		sbuf_len(sb)
#else
#define	SBUF_PRINTF(sb, ...)	printf(__VA_ARGS__)
#define	SBUF_FINISH(sb)		(0)
#define	SBUF_DELETE(sb)
#define	SBUF_NEW_AUTO()		(NULL)
#define	SBUF_DATA(sb)		(NULL)
#define	SBUF_LEN(sb)		(0)
#endif

#define	DUMP_RING(ring) do { 					\
	int _i, _j, _lsb;					\
	uint32_t enq;						\
	const char *_label;					\
								\
	if (XHCI_DEBUG_RING_IN(ring))				\
		_label = "IR";					\
	else if (XHCI_DEBUG_RING_OUT(ring))			\
		_label = "OR";					\
	else							\
		_label = "ER";					\
	enq = xhci_debug_ring_boundary(ring, false);		\
	SBUF_PRINTF(sb, "%s:", _label); 			\
	SBUF_PRINTF(sb, ", e/E/d=%d/%d/%d", (ring)->enq, enq, (ring)->deq);\
	SBUF_PRINTF(sb, "(%llu)", UDB_TRB_RING_LAST);		\
	for (_i = 0; _i < (int)sizeof((ring)->iocbitmap); _i++)	\
		if ((ring)->iocbitmap[_i] != 0)			\
			break;					\
	if (_i == (int)sizeof((ring)->iocbitmap))		\
		_lsb = -1;					\
	else {							\
		_lsb = 8 * _i;					\
		for (_j = 0; _j < 8; _j++)			\
			if ((ring)->iocbitmap[_i] & (1 << _j)) { \
				_lsb += _j;			\
				break;				\
			}					\
	}							\
	SBUF_PRINTF(sb, " iocbitmap=%d", _lsb);			\
	SBUF_PRINTF(sb, ", flags=%d", (ring)->flags);		\
	SBUF_PRINTF(sb, ", w.e/d=%u/%u", (ring)->work.enq, (ring)->work.deq); \
	SBUF_PRINTF(sb, "(%llu)", UDB_WORK_RING_LAST);		\
	SBUF_PRINTF(sb, ", paddr(/w)=%p/%p", (void *)(ring)->paddr, \
	    (void *)(ring)->work.paddr);			\
	SBUF_PRINTF(sb, ", doorbell=%04x", (ring)->doorbell);	\
	SBUF_PRINTF(sb, ", cyc=%02x", (ring)->cyc);		\
	SBUF_PRINTF(sb, ", lasterror=%u", (ring)->lasterror);	\
	SBUF_PRINTF(sb, ", lastenq=%u", (ring)->lastenq);	\
	SBUF_PRINTF(sb, ", lastdeq=%u", (ring)->lastdeq);	\
	SBUF_PRINTF(sb, ", ec_STALL=%lu",			\
	    (ring)->ec[XHCI_TRB_ERROR_STALL]);			\
	SBUF_PRINTF(sb, ", ec_TRB=%lu",				\
	    (ring)->ec[XHCI_TRB_ERROR_TRB]);			\
	SBUF_PRINTF(sb, ", ec_EVLOCKED=%lu",			\
	    (ring)->ec[XHCI_TRB_ERROR_EVLOCKED]);		\
	SBUF_PRINTF(sb, ", ec_CALLED=%lu",			\
	    (ring)->ec[XHCI_TRB_ERROR_CALLED]);			\
	SBUF_PRINTF(sb, ", ec_UNKNOWN=%lu",			\
	    (ring)->ec[XHCI_TRB_ERROR_UNKNOWN]);		\
	SBUF_PRINTF(sb, "\n");					\
	} while (0)

/* XXX: this should be removed */
static void flush_range(void *, uint32_t);

#if 0
static bool xhci_debug_running(struct xhci_debug_softc *);
#endif
static bool xhci_debug_update_regbit(struct xhci_debug_softc *, int, int, int,
    bool);
static bool xhci_debug_set_regbit(struct xhci_debug_softc *, int, int, int);
static bool xhci_debug_reset_regbit(struct xhci_debug_softc *, int, int, int);
static bool xhci_debug_wait_connection(struct xhci_debug_softc *, uint32_t);

static void xhci_debug_ring_init(struct xhci_debug_softc *,
    struct xhci_debug_ring *, int, uint32_t);
static void xhci_debug_init_ep(struct xhci_endp_ctx64 *, uint64_t, uint32_t,
    uint64_t);

static void xhci_debug_softc_fixup(struct xhci_debug_softc *);
static void xhci_debug_init_strings(struct xhci_debug_softc *, uint32_t *);

static void xhci_debug_ring_doorbell(struct xhci_debug_softc *, uint32_t);
static void xhci_debug_show_ring(struct xhci_debug_ring *);

static uint64_t xhci_debug_bulk_transfer0(struct xhci_debug_softc *,
    struct xhci_debug_ring *);
static bool xhci_debug_event_ready(struct xhci_debug_softc *);
static uint32_t xhci_debug_ring_boundary(struct xhci_debug_ring *, bool);

static uint64_t trb_tx_enqueue_locked(struct xhci_debug_softc *sc,
    struct xhci_debug_ring *);
static uint64_t trb_rx_enqueue_locked(struct xhci_debug_softc *sc,
    struct xhci_debug_ring *);
static uint64_t trb_push_locked(struct xhci_debug_softc *sc,
    struct xhci_debug_ring *, uint64_t, uint64_t);

static void xhci_debug_event_tx(struct xhci_debug_softc *, uint64_t,
    uint32_t, uint32_t);

/* sc_state */

#define	STRSTATE(x)	[x] = #x
static const char *strstate[] = {
	STRSTATE(XHCI_DCPORT_ST_OFF),
	STRSTATE(XHCI_DCPORT_ST_DISCONNECTED_RUNNING),
	STRSTATE(XHCI_DCPORT_ST_DISCONNECTED),
	STRSTATE(XHCI_DCPORT_ST_DISABLED),
	STRSTATE(XHCI_DCPORT_ST_RESETTING),
	STRSTATE(XHCI_DCPORT_ST_ENABLED),
	STRSTATE(XHCI_DCPORT_ST_CONFIGURED)
};

/* sysctls */

int udb_debug;
int udb_reset;
int udb_enable = 1;

#ifdef _KERNEL
static SYSCTL_NODE(_hw_usb_xhci, OID_AUTO, dbc, CTLFLAG_RW | CTLFLAG_MPSAFE, 0,
    "USB XHCI DbC");
SYSCTL_INT(_hw_usb_xhci_dbc, OID_AUTO, enable, CTLFLAG_RWTUN, &udb_enable, 1,
    "Set to enable XHCI DbC support");
SYSCTL_INT(_hw_usb_xhci_dbc, OID_AUTO, debug, CTLFLAG_RWTUN, &udb_debug, 0,
    "Debug level");
SYSCTL_INT(_hw_usb_xhci_dbc, OID_AUTO, reset, CTLFLAG_RWTUN, &udb_reset, 0,
    "Reset DbC");
#endif

/* XXX */
static void
flush_range(void *ptr, uint32_t bytes)
{
#ifdef _KERNEL
# ifdef __amd64__
	pmap_flush_cache_range((vm_offset_t)(uintptr_t)ptr,
	    (vm_offset_t)((uintptr_t)ptr + bytes));
# elif __i386__
	pmap_invalidate_cache_range((vm_offset_t)(uintptr_t)ptr,
	    (vm_offset_t)((uintptr_t)ptr + bytes));
# elif __aarch64__
	cpu_dcache_inv_range((vm_offset_t)(uintptr_t)ptr,
	    (vm_offset_t)((uintptr_t)ptr + bytes));
# endif
#else
	uint32_t i;
	const uint32_t clshft = 6;
	const uint32_t clsize = (1UL << clshft);
	const uint32_t clmask = clsize - 1;
	uint32_t lines = (bytes >> clshft);
	lines += (bytes & clmask) != 0;

	if (bytes == 0)
		return;

# if defined(__amd64__) || defined(__i386__)
#define	CLFLUSH(ptr) \
	__asm volatile("clflush %0" : "+m"(*(volatile char *)ptr))

	for (i = 0; i < lines; i++)
		CLFLUSH((void *)((uint64_t)ptr + (i * clsize)));
#undef	CLFLUSH
#endif
#endif
}

void
udb_dump_info(struct xhci_debug_softc *sc)
{
	uint32_t enq;

	xhci_debug_event_dequeue(sc);

	xhci_debug_update_state(sc);
	printf("=============\n");
	printf("    ctrl: 0x%x stat: 0x%x psc: 0x%x\n",
	    _XREAD4(sc, dbc, XHCI_DCCTRL),
	    _XREAD4(sc, dbc, XHCI_DCST),
	    _XREAD4(sc, dbc, XHCI_DCPORTSC));
	printf("      DbC Enable = %s\t",
	    _XREAD4(sc, dbc, XHCI_DCCTRL) & XHCI_DCCTRL_DCE ? "YES" : "NO");
	printf("      DbC Running = %s\n",
	    _XREAD4(sc, dbc, XHCI_DCCTRL) & XHCI_DCCTRL_DCR ? "YES" : "NO");

	printf("      Link State Event Enable = %s\t",
	    _XREAD4(sc, dbc, XHCI_DCCTRL) & XHCI_DCCTRL_LSE ? "YES" : "NO");
	printf("      Halt OUT TR = %s\n",
	    _XREAD4(sc, dbc, XHCI_DCCTRL) & XHCI_DCCTRL_HOT ? "YES" : "NO");

	printf("      Halt IN TR = %s\t",
	    _XREAD4(sc, dbc, XHCI_DCCTRL) & XHCI_DCCTRL_HIT ? "YES" : "NO");
	printf("      DbC Run Change = %s\n",
	    _XREAD4(sc, dbc, XHCI_DCCTRL) & XHCI_DCCTRL_DRC ? "YES" : "NO");

	printf("      DbC CCS = %s\t",
	    _XREAD4(sc, dbc, XHCI_DCPORTSC) & XHCI_DCPORTSC_CCS ? "YES" : "NO");
	printf("      DbC PED = %s\n",
	    _XREAD4(sc, dbc, XHCI_DCPORTSC) & XHCI_DCPORTSC_PED ? "YES" : "NO");

	printf("      DbC Port = %d\n",
	    (uint8_t)XHCI_DCST_PORT_GET(_XREAD4(sc, dbc, XHCI_DCST)));

	printf("    id: 0x%x\t", _XREAD4(sc, dbc, XHCI_DCID));
	printf("    erstsz: %u, erstba: 0x%lx\n",
	    _XREAD4(sc, dbc, XHCI_DCERSTSZ),
	    _XREAD44LH(sc, dbc, XHCI_DCERSTBA));

	printf("    erdp: 0x%lx, cp: 0x%lx\t",
	    _XREAD44LH(sc, dbc, XHCI_DCERDP),
	    _XREAD44LH(sc, dbc, XHCI_DCCP));
	printf("    ddi1: 0x%x, ddi2: 0x%x\n",
	    _XREAD4(sc, dbc, XHCI_DCDDI1),
	    _XREAD4(sc, dbc, XHCI_DCDDI2));

	printf("    erstba == sc->udb_erst_paddr: %d",
	    _XREAD44LH(sc, dbc, XHCI_DCERSTBA) == sc->udb_erst_paddr);
	printf("    erdp == erst[0].qwEvrsTablePtr: %d",
	    _XREAD44LH(sc, dbc, XHCI_DCERDP) == sc->udb_erst[0].qwEvrsTablePtr);
	printf("    cp == sc->udb_ctx_paddr: %d\n",
	    _XREAD44LH(sc, dbc, XHCI_DCCP) == sc->udb_ctx_paddr);

#ifndef _KERNEL
	/* Loader console structure */
	printf("=============\n");
	printf("cons->c_flags = %x\n", sc->sc_cons->c_flags);
#endif
	printf("=============\n");
	/* Update er->enq */
	enq = xhci_debug_ring_boundary(&sc->udb_ering, true);
	printf("ering ready: enq=%u, ering.enq=%u\n", enq, sc->udb_ering.enq);
	xhci_debug_show_ring(&sc->udb_ering);
	xhci_debug_show_ring(&sc->udb_iring);
	xhci_debug_show_ring(&sc->udb_oring);
	printf("=============\n");
	printf("sc->sc_init_dma = %d, ", sc->sc_init_dma);
	printf("sc->sc_init = %d\n", sc->sc_init);

	printf("sc->sc_state = %s(%02x)\n", strstate[sc->sc_state],
	    sc->sc_state);
	printf("=============\n");
	printf("DbC Commands:\n");
	printf(" udb_send xxx: Send 'xxx'\n");
	printf(" udb_recv: Recv'\n");
	printf(" udb_probe: Probe and initialize DbC\n");
	printf("=============\n");
}

#if 0
static bool
xhci_debug_running(struct xhci_debug_softc *sc)
{
	if (sc == NULL)
		goto error;
	if (sc->sc_init == false)
		goto error;

	xhci_debug_update_state(sc);
	/* Reinitialize if possible */
	if (sc->sc_state != XHCI_DCPORT_ST_CONFIGURED) {
		xhci_debug_enable(sc);
		xhci_debug_event_dequeue(sc);
		xhci_debug_update_state(sc);
	}
	return (true);
error:
	DEBUG_PRINTF(2, "%s: false\n", __func__);
	return (false);
}
#endif

void
xhci_debug_update_state(struct xhci_debug_softc *sc)
{
	uint32_t ctrl;
	uint32_t portsc;
	uint32_t sc_state_old;

	ctrl = _XREAD4(sc, dbc, XHCI_DCCTRL);
	portsc = _XREAD4(sc, dbc, XHCI_DCPORTSC);
	sc_state_old = sc->sc_state;
	sc->sc_state = XHCI_DCSTATUS(ctrl, portsc);
	if (sc->sc_state != sc_state_old)
		DEBUG_PRINTF(1, "state: %s -> %s(%d)\n",
		    strstate[sc_state_old], strstate[sc->sc_state],
		    sc->sc_state);
}

static bool
xhci_debug_update_regbit(struct xhci_debug_softc *sc, int reg, int bit,
    int timeout, bool reset)
{
	uint32_t temp;
	int expected;

	timeout *= 10;	/* ms */
	temp = le32toh(_XREAD4(sc, dbc, reg));
	if (reset)
		temp &= ~bit;
	else
		temp |= bit;
	_XWRITE4(sc, dbc, reg, htole32(temp));
	expected = (reset) ? 0 : bit;
	while ((le32toh(_XREAD4(sc, dbc, reg)) & bit) != expected &&
	    timeout != 0) {
		delay(100);
		timeout--;
	}
	return (timeout > 0);
}
static bool
xhci_debug_set_regbit(struct xhci_debug_softc *sc, int reg, int bit,
    int timeout)
{

	return (xhci_debug_update_regbit(sc, reg, bit, timeout, false));
}
static bool
xhci_debug_reset_regbit(struct xhci_debug_softc *sc, int reg, int bit,
    int timeout)
{

	return (xhci_debug_update_regbit(sc, reg, bit, timeout, true));
}

bool
xhci_debug_enable(struct xhci_debug_softc *sc)
{
	bool ret;

	xhci_debug_softc_fixup(sc);
	xhci_debug_update_state(sc);
	ret = true;
	if (sc->sc_state == XHCI_DCPORT_ST_OFF) {
		ret = xhci_debug_set_regbit(sc, XHCI_DCCTRL, XHCI_DCCTRL_DCE,
		    1);
		xhci_debug_update_state(sc);
		if (ret == false)
			DEBUG_PRINTF(1, "%s: setting XHCI_DCCTRL_DCE failed\n",
			    __func__);
	}
	/* If success, check the cable. */
	return ((ret == true) ? (xhci_debug_wait_connection(sc, 1)) : ret);
}

static bool
xhci_debug_wait_connection(struct xhci_debug_softc *sc, uint32_t timeout)
{
	bool ret;

	xhci_debug_update_state(sc);
	if (sc->sc_state == XHCI_DCPORT_ST_DISCONNECTED) {
		ret = xhci_debug_set_regbit(sc, XHCI_DCPORTSC,
		    XHCI_DCPORTSC_PED, 1000);	/* XXX: 1 was not enough */
		xhci_debug_update_state(sc);
		if (ret == false) {
			DEBUG_PRINTF(1,
			   "%s: setting XHCI_DCPORTSC_PED failed\n",
			    __func__);
			return (ret); 
		}
	}
	DEBUG_PRINTF(1, "%s: Wait DCR\n", __func__);
	timeout = 10000;
	xhci_debug_update_state(sc);
	while (sc->sc_state != XHCI_DCPORT_ST_CONFIGURED) {
		DEBUG_PRINTF(1, "Insert a debug cable: PR=%d, PLS=%d, DCR=0\n",
		    ((_XREAD4(sc, dbc, XHCI_DCPORTSC) & XHCI_PS_PR) != 0),
		    XHCI_PS_PLS_GET(_XREAD4(sc, dbc, XHCI_DCPORTSC)));
		delay(500);
		if (timeout-- == 0) {
			DEBUG_PRINTF(1, "Insert a debug cable: timed out\n");
			break;
		}
		xhci_debug_update_state(sc);
	}
	if (timeout == 0)
		return (false);

	DEBUG_PRINTF(2, "Debug Cable Inserted: PR=%d, PLS=%d, DCR=1\n",
	    ((_XREAD4(sc, dbc, XHCI_DCPORTSC) & XHCI_PS_PR) != 0),
	    XHCI_PS_PLS_GET(_XREAD4(sc, dbc, XHCI_DCPORTSC)));
	return (true);
}

void
xhci_debug_disable(struct xhci_debug_softc *sc)
{
	bool ret;

	xhci_debug_reset_regbit(sc, XHCI_DCPORTSC, XHCI_DCPORTSC_PED, 0);
	ret = xhci_debug_reset_regbit(sc, XHCI_DCCTRL, XHCI_DCCTRL_DCE, 1000);
	if (ret == false)
		DEBUG_PRINTF(1, "%s: clearing XHCI_DCCTRL_DCE failed\n",
		    __func__);
	xhci_debug_update_state(sc);
}

int
xhci_debug_init_ring(struct xhci_debug_softc *sc)
{
	uint64_t erdp, out, in;
	uint32_t temp;

	if (sc->sc_dbc_off == 0) {
		DEBUG_PRINTF(1, "%s: register is empty\n", __func__);
		return (-1);
	}
	/* Create an event ring. */
	xhci_debug_ring_init(sc, &sc->udb_ering, 0, XHCI_DCDB_INVAL);
	erdp = sc->udb_ering.paddr;
	memset(sc->udb_erst, 0, sizeof(*sc->udb_erst));
	sc->udb_erst->qwEvrsTablePtr = htole64(erdp);
	sc->udb_erst->dwEvrsTableSize = htole32(UDB_TRB_RING_LEN);
	DEBUG_PRINTF(2, "%s: ERDP=0x%016llx\n", __func__,
	    (unsigned long long)erdp);

	/* Create an output ring. */
	xhci_debug_ring_init(sc, &sc->udb_oring, 1, XHCI_DCDB_OUT);
	out = sc->udb_oring.paddr;

	/* Create an input ring. */
	xhci_debug_ring_init(sc, &sc->udb_iring, 1, XHCI_DCDB_IN);
	in = sc->udb_iring.paddr;

	/* Initialize DbC context. */
	memset(sc->udb_ctx, 0, sizeof(*sc->udb_ctx));
	xhci_debug_init_strings(sc, sc->udb_ctx->info);

	/* Initialize endpoints. */ 
	temp = _XREAD4(sc, dbc, XHCI_DCCTRL);
	xhci_debug_init_ep(&sc->udb_ctx->ep_out,
	    (uint64_t)XHCI_DCCTRL_MBS_GET(le32toh(temp)), EP_BULK_OUT, out);
	xhci_debug_init_ep(&sc->udb_ctx->ep_in,
	    (uint64_t)XHCI_DCCTRL_MBS_GET(le32toh(temp)), EP_BULK_IN, in);

	/* Configure register values. */
	_XWRITE4(sc, dbc, XHCI_DCERSTSZ, htole32(1));
	_XWRITE44LH(sc, dbc, XHCI_DCERSTBA, htole64(sc->udb_erst_paddr));
	_XWRITE44LH(sc, dbc, XHCI_DCERDP, htole64(erdp));
	_XWRITE44LH(sc, dbc, XHCI_DCCP, htole64(sc->udb_ctx_paddr));
	_XWRITE4(sc, dbc, XHCI_DCDDI1, htole32((DC_VENDOR   << 16) | DC_PROTOCOL));
	_XWRITE4(sc, dbc, XHCI_DCDDI2, htole32((DC_REVISION << 16) | DC_PRODUCT));
	flush_range(sc->udb_ctx, sizeof(*sc->udb_ctx));
	flush_range(sc->udb_erst, sizeof(*sc->udb_erst));
	flush_range(sc->udb_ering.trb, UDB_TRB_RING_BYTES);
	flush_range(sc->udb_oring.trb, UDB_TRB_RING_BYTES);
	flush_range(sc->udb_iring.trb, UDB_TRB_RING_BYTES);
	flush_range(sc->udb_oring.work.buf, UDB_WORK_RING_BYTES);
	flush_range(sc->udb_iring.work.buf, UDB_WORK_RING_BYTES);

	return (0);
}

int
xhci_debug_probe(struct xhci_debug_softc *sc)
{
	uint32_t hccp1;
	uint32_t eec;
	uint32_t eecp;

	if (sc == NULL)
		return (0);
#ifndef	_KERNEL
	xhci_debug_pci_mmio_init(sc);
#endif
	hccp1 = _XREAD4(sc, capa, XHCI_HCSPARAMS0);

	if (XHCI_HCS0_XECP(hccp1) == 0)
		goto notfound;

	eec = -1;	/* XXX */
	for (eecp = XHCI_HCS0_XECP(hccp1) << 2;
	     eecp != 0 && XHCI_XECP_NEXT(eec) != 0;
	     eecp += XHCI_XECP_NEXT(eec) << 2) {
		eec = _XREAD4(sc, capa, eecp);

		if (XHCI_XECP_ID(eec) != XHCI_ID_USB_DEBUG) {
			DEBUG_PRINTF(3, "%s: looking for %02x\n",
			    __func__, XHCI_XECP_ID(eec));
			continue;
		}
	}
	if (XHCI_XECP_ID(eec) != XHCI_ID_USB_DEBUG)
		goto notfound;
	DEBUG_PRINTF(1, "%s: DBC_ID was found at %08x\n", __func__, eecp);
	sc->sc_dbc_off = eecp;
	return (1);

notfound:
	DEBUG_PRINTF(1, "%s: DBC_ID was not found\n", __func__);
	return (0);
}

/*
 * The hw.usb.xhci.dbc mibs are used to pass physical addresses
 * from the loader to the kernel.  This function simply relies on
 * the direct mapping at this moment and works only on the platforms
 * supporting it.
 */
static void
xhci_debug_softc_fixup(struct xhci_debug_softc *sc)
{
	if (sc->sc_fixup_done)
		return;
#if _KERNEL
	struct trb_addrs {
		const char *key;
		struct xhci_debug_ring *ringp;
	} parray[] = {
		{ "hw.usb.xhci.dbc.ering", &sc->udb_ering },
		{ "hw.usb.xhci.dbc.oring", &sc->udb_oring },
		{ "hw.usb.xhci.dbc.iring", &sc->udb_iring },
		{ NULL, NULL }
	};
	struct trb_addrs *p;
	quad_t addr, len;
	char buf[256];

	KASSERT(PMAP_HAS_DMAP, ("direct-map required"));

	mtx_init(&sc->sc_mtx, "xhci_dbc_sc_mtx", NULL, MTX_SPIN);
	mtx_init(&sc->udb_ering.mtx, "xhci_dbc_ering_mtx", NULL, MTX_SPIN);
	mtx_init(&sc->udb_iring.mtx, "xhci_dbc_iring_mtx", NULL, MTX_SPIN);
	mtx_init(&sc->udb_oring.mtx, "xhci_dbc_oring_mtx", NULL, MTX_SPIN);

	mtx_lock_spin(&sc->sc_mtx);

	getenv_quad("hw.usb.xhci.dbc.softc.paddr", &addr);
	getenv_quad("hw.usb.xhci.dbc.softc.len", &len);
	sc->sc_udb = (struct xhci_debug_softc *)
	    pmap_map(NULL, (vm_paddr_t)addr, (vm_paddr_t)addr + len, 0);
	if (pmap_change_attr((vm_offset_t)sc->sc_udb,
	    (vm_offset_t)len, VM_MEMATTR_DEVICE) != 0)
		printf("%s: pmap_change_attr() failed: %p",
		    __func__, (void *)sc->sc_udb);

	getenv_quad("hw.usb.xhci.dbc.ctx.paddr", &addr);
	getenv_quad("hw.usb.xhci.dbc.ctx.len", &len);
	sc->udb_ctx = (struct xhci_debug_ctx *)
	    pmap_map(NULL, (vm_paddr_t)addr, (vm_paddr_t)addr + len, 0);
	if (pmap_change_attr((vm_offset_t)sc->udb_ctx,
	    (vm_offset_t)len, VM_MEMATTR_DEVICE) != 0)
		printf("%s: pmap_change_attr() failed: %p",
		    __func__, (void *)sc->udb_ctx);

	getenv_quad("hw.usb.xhci.dbc.erst.paddr", &addr);
	getenv_quad("hw.usb.xhci.dbc.erst.len", &len);
	sc->udb_erst = (struct xhci_event_ring_seg *)
	    pmap_map(NULL, (vm_paddr_t)addr, (vm_paddr_t)addr + len, 0);
	if (pmap_change_attr((vm_offset_t)sc->udb_erst,
	    (vm_offset_t)len, VM_MEMATTR_DEVICE) != 0)
		printf("%s: pmap_change_attr() failed: %p",
		    __func__, (void *)sc->udb_erst);

	getenv_quad("hw.usb.xhci.dbc.str.paddr", &addr);
	getenv_quad("hw.usb.xhci.dbc.str.len", &len);
	sc->udb_str = (char *)
	    pmap_map(NULL, (vm_paddr_t)addr, (vm_paddr_t)addr + len, 0);
	if (pmap_change_attr((vm_offset_t)sc->udb_str,
	    (vm_offset_t)len, VM_MEMATTR_DEVICE) != 0)
		printf("%s: pmap_change_attr() failed: %p",
		    __func__, (void *)sc->udb_str);

	for (p = &parray[0]; p->key != NULL; p++) {
		snprintf(buf, sizeof(buf), "%s.paddr", p->key);
		buf[sizeof(buf) - 1] = '\0';
		getenv_quad(buf, &addr);
		snprintf(buf, sizeof(buf), "%s.len", p->key);
		buf[sizeof(buf) - 1] = '\0';
		getenv_quad(buf, &len);

		p->ringp->paddr = (uintptr_t)addr;
		p->ringp->trb =
		    (struct xhci_trb *)pmap_map(NULL, (vm_paddr_t)addr,
			(vm_paddr_t)addr + len, 0);
		if (pmap_change_attr((vm_offset_t)p->ringp->trb,
		    (vm_offset_t)len, VM_MEMATTR_DEVICE) != 0)
			printf("%s: pmap_change_attr() failed: %p",
			    __func__, (void *)p->ringp->trb);
	}
	getenv_quad("hw.usb.xhci.dbc.ering.work.paddr", &addr);
	getenv_quad("hw.usb.xhci.dbc.ering.work.len", &len);
	sc->udb_ering.work.paddr = (uintptr_t)addr;
	sc->udb_ering.work.buf = (char *)
	    pmap_map(NULL, (vm_paddr_t)addr, (vm_paddr_t)addr + len, 0);
	if (pmap_change_attr((vm_offset_t)sc->udb_ering.work.buf,
	    (vm_offset_t)len, VM_MEMATTR_DEVICE) != 0)
		printf("%s: pmap_change_attr() failed: %p",
		    __func__, (void *)sc->udb_ering.work.buf);

	getenv_quad("hw.usb.xhci.dbc.iring.work.paddr", &addr);
	getenv_quad("hw.usb.xhci.dbc.iring.work.len", &len);
	sc->udb_iring.work.paddr = (uintptr_t)addr;
	sc->udb_iring.work.buf = (char *)
	    pmap_map(NULL, (vm_paddr_t)addr, (vm_paddr_t)addr + len, 0);
	if (pmap_change_attr((vm_offset_t)sc->udb_iring.work.buf,
	    (vm_offset_t)len, VM_MEMATTR_DEVICE) != 0)
		printf("%s: pmap_change_attr() failed: %p",
		    __func__, (void *)sc->udb_iring.work.buf);

	getenv_quad("hw.usb.xhci.dbc.oring.work.paddr", &addr);
	getenv_quad("hw.usb.xhci.dbc.oring.work.len", &len);
	sc->udb_oring.work.paddr = (uintptr_t)addr;
	sc->udb_oring.work.buf = (char *)
	    pmap_map(NULL, (vm_paddr_t)addr, (vm_paddr_t)addr + len, 0);
	if (pmap_change_attr((vm_offset_t)sc->udb_oring.work.buf,
	    (vm_offset_t)len, VM_MEMATTR_DEVICE) != 0)
		printf("%s: pmap_change_attr() failed: %p",
		    __func__, (void *)sc->udb_oring.work.buf);

	/* Reset the pointers and TRBs after reset. */
	/* XXX: This should be revisited to support suspend/resume. */
	xhci_debug_ring_init(sc, &sc->udb_ering, 0, XHCI_DCDB_INVAL);
	memcpy(sc->udb_ering.ec, sc->sc_udb->udb_ering.ec,
	    sizeof(sc->udb_ering.ec));
	xhci_debug_ring_init(sc, &sc->udb_iring, 1, XHCI_DCDB_IN);
	memcpy(sc->udb_iring.ec, sc->sc_udb->udb_iring.ec,
	    sizeof(sc->udb_iring.ec));
	xhci_debug_ring_init(sc, &sc->udb_oring, 1, XHCI_DCDB_OUT);
	memcpy(sc->udb_oring.ec, sc->sc_udb->udb_oring.ec,
	    sizeof(sc->udb_oring.ec));

	sc->sc_init_dma = sc->sc_udb->sc_init_dma;
	sc->sc_init = sc->sc_udb->sc_init;

	mtx_unlock_spin(&sc->sc_mtx);
#endif
	sc->sc_fixup_done = true;
}

static void
xhci_debug_ring_init(struct xhci_debug_softc *sc,
    struct xhci_debug_ring *ring, int producer, uint32_t doorbell)
{
	memset(ring->trb, 0, UDB_TRB_RING_BYTES);
	memset(ring->work.buf, 0, UDB_WORK_RING_BYTES);

	ring->enq = 0;
	ring->deq = 0;
	ring->cyc = 1;
	ring->doorbell = doorbell;
	ring->work.enq = 0;
	ring->work.deq = 0;

	/* Place a link TRB at the tail if producer == 1. */
	if (producer) {
		/* Fields for link TRBs (section 6.4.4.1) */

		ring->trb[UDB_TRB_RING_LAST] = (struct xhci_trb){
		    .qwTrb0 = htole64(ring->paddr),
		    .dwTrb3 = htole32(
			XHCI_TRB_3_TYPE_SET(XHCI_TRB_TYPE_LINK) |
			XHCI_TRB_3_TC_BIT
		    )
		};
	}
}

struct usb_strdesc {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t chars[62];	/* UTF-16LE */
};
#define USB_DESC_TYPE_STRING	3

/* Initialize the DbC info with USB string descriptor addresses. */
void
xhci_debug_init_strings(struct xhci_debug_softc *sc, uint32_t *info)
{
	struct usb_strdesc udb_strdesc[] = {
		{
		  .bLength = 2 + 4,
		  .bDescriptorType = USB_DESC_TYPE_STRING,
		  .chars = { 9, 0, 4, 0 },	/* 0x0409 = English */
		},
		{
		  .bLength = 2 + 36,
		  .bDescriptorType = USB_DESC_TYPE_STRING,
		  .chars = {
		    'F', 0, 'r', 0, 'e', 0, 'e', 0, 'B', 0, 'S', 0, 'D', 0,
		    ' ', 0,
		    'F', 0, 'o', 0, 'u', 0, 'n', 0, 'd', 0, 'a', 0, 't', 0,
		    'i', 0, 'o', 0, 'n', 0,
		  }
		},
		{
		  .bLength = 2 + 50,
		  .bDescriptorType = USB_DESC_TYPE_STRING,
		  .chars = {
		    'F', 0, 'r', 0, 'e', 0, 'e', 0, 'B', 0, 'S', 0, 'D', 0,
		    ' ', 0,
		    'U', 0, 'S', 0, 'B', 0, ' ', 0, 'D', 0, 'b', 0, 'C', 0,
		    '.', 0, 'G', 0, 'P', 0, ' ', 0, 'D', 0, 'e', 0, 'v', 0,
		    'i', 0, 'c', 0, 'e', 0,
		  }
		},
		{
		  .bLength = 2 + 2,
		  .bDescriptorType = USB_DESC_TYPE_STRING,
		  .chars = { '0', 0 }
		}
	};
	uint64_t *sda;

	/* udb_str is 1 page buffer longer than sizeof(udb_strdesc) */
	memcpy(sc->udb_str, udb_strdesc, sizeof(udb_strdesc));

	sda = (uint64_t *)(void *)&info[0];
	sda[0] = sc->udb_str_paddr;
	sda[1] = sda[0] + 64;
	sda[2] = sda[0] + 64 + 64;
	sda[3] = sda[0] + 64 + 64 + 64;
	info[8] = (4 << 24) | (52 << 16) | (38 << 8) | 6;
}

static void
xhci_debug_ring_doorbell(struct xhci_debug_softc *sc, uint32_t target)
{
	uint32_t doorbell;

	/* The other bits in DCDB are RsvdP. */
	/* XXX: the spec has contradiction between Figure 7-9 and Table 7-18. */
	doorbell = _XREAD4(sc, dbc, XHCI_DCDB) & ~XHCI_DCDB_MASK;
	doorbell |= target;
	_XWRITE4(sc, dbc, XHCI_DCDB, doorbell);

	DEBUG_PRINTF(2, "%s: doorbell = %04x\n", __func__, doorbell);
}

uint64_t
xhci_debug_bulk_transfer(struct xhci_debug_softc *sc)
{
	return (
	    xhci_debug_bulk_transfer0(sc, &sc->udb_iring) +
	    xhci_debug_bulk_transfer0(sc, &sc->udb_oring));
}

uint64_t
xhci_debug_bulk_transfer0(struct xhci_debug_softc *sc,
    struct xhci_debug_ring *ring)
{
	uint32_t temp;
	uint64_t len;

	/* XXX: running status check? */
	xhci_debug_update_state(sc);

	/* configure-exit */
	temp = _XREAD4(sc, dbc, XHCI_DCCTRL);
	if (XHCI_DCCTRL_DRC_GET(le32toh(temp)) != 0)
		_XWRITE4(sc, dbc, XHCI_DCCTRL, temp);

	xhci_debug_event_dequeue(sc);
	xhci_debug_update_state(sc);

	len = 0;
	if (mtx_trylock_spin(&sc->udb_ering.mtx) == 0)
		goto end0;
	if (mtx_trylock_spin(&ring->mtx) == 0)
		goto end1;
	if (XHCI_DEBUG_RING_FULL(ring)) /* TRB queue is full */
		goto end2;

	len = (ring->doorbell == XHCI_DCDB_IN)
	    ? trb_rx_enqueue_locked(sc, ring)
	    : trb_tx_enqueue_locked(sc, ring);

	xhci_debug_update_state(sc);
end2:
	mtx_unlock_spin(&ring->mtx);
end1:
	mtx_unlock_spin(&sc->udb_ering.mtx);
end0:
	return (len);
}

static void
xhci_debug_show_ring(struct xhci_debug_ring *ring)
{
#ifdef _KERNEL
	struct sbuf *sb;
#else
	char *sb;
#endif
	sb = SBUF_NEW_AUTO();

	DUMP_RING(ring);

#ifdef	_KERNEL
	if (SBUF_FINISH(sb) != 0)
		return;
	printf("%s", SBUF_DATA(sb));

	SBUF_DELETE(sb);
#endif
}

/*
 * Initializes the endpoint as specified in sections 7.6.3.2 and 7.6.9.2.
 * Each endpoint is Bulk, so the MaxPStreams, LSA, HID, CErr, FE,
 * Interval, Mult, and Max ESIT Payload fields are all 0.   
 *
 * Max packet size: 1024
 * Max burst size: mbs
 * EP type: 2 for OUT bulk, 6 for IN bulk   
 * TR dequeue ptr: physical base address of transfer ring
 * Avg TRB length: software defined (see 4.14.1.1 for suggested defaults, 3kB)
 */
static void
xhci_debug_init_ep(struct xhci_endp_ctx64 *ep64, uint64_t mbs,
    uint32_t type, uint64_t ring_paddr)
{
	struct xhci_endp_ctx *ep;

	if (ep64 == NULL)
		return;
	ep = &ep64->ctx;
	memset(ep64, 0, sizeof(*ep64));

	/* Do we need XHCI_EPCTX_1_CERR_SET(3) in Sec 6.2.3 ? */
	ep->dwEpCtx1 = htole32(
	    XHCI_EPCTX_1_MAXP_SIZE_SET(1024) |
	    XHCI_EPCTX_1_MAXB_SET((uint32_t)mbs) |
	    XHCI_EPCTX_1_EPTYPE_SET(type));
	ep->qwEpCtx2 = htole64(
	    (ring_paddr & XHCI_EPCTX_2_TR_DQ_PTR_MASK) |
	    XHCI_EPCTX_2_DCS_SET(1));
	ep->dwEpCtx4 = htole32(
	    XHCI_EPCTX_4_AVG_TRB_LEN_SET(1024 * 3));
}

static bool
xhci_debug_event_ready(struct xhci_debug_softc *sc)
{
	uint32_t temp;

	/* Seems inaccurate */
	temp = _XREAD4(sc, dbc, XHCI_DCST);
	return ((le32toh(temp) & XHCI_DCST_ER) == XHCI_DCST_ER);
}

static uint32_t
xhci_debug_ring_boundary(struct xhci_debug_ring *ring, bool update_enq)
{
	struct xhci_trb *trb = &ring->trb[ring->deq];
	uint32_t dwTrb3;
	uint32_t cyc;
	uint32_t deq;

	cyc = ring->cyc;
	deq = ring->deq;
	LFENCE();
	dwTrb3 = le32toh(trb->dwTrb3);

	while ((dwTrb3 & XHCI_TRB_3_CYCLE_BIT) == cyc) {
		cyc = (deq == UDB_TRB_RING_LAST)
		    ? cyc ^ 1
		    : cyc;
		deq = (deq + 1) & UDB_TRB_RING_OFFSET_MASK;
		trb = &ring->trb[deq];
		LFENCE();
		dwTrb3 = le32toh(trb->dwTrb3);
	}
	if (update_enq)
		ring->enq = deq;

	return (deq);
}

static void
xhci_debug_event_tx(struct xhci_debug_softc *sc,
    uint64_t qwTrb0, uint32_t dwTrb2, uint32_t dwTrb3)
{
	struct xhci_trb *trb;
	struct xhci_debug_ring *ring;
#if 0
	uint32_t dwEpCtx0;
#endif
	uint64_t qwEpCtx2;
	uint32_t slot, epid;
	uint32_t error, len;
	uint32_t deq_old, deq_on_er;
	const char *deq_mark;

	/* EP ID is an index of context. */
	epid = XHCI_TRB_3_EP_GET(dwTrb3);
	slot = XHCI_TRB_3_SLOT_GET(dwTrb3);
	error = XHCI_TRB_2_ERROR_GET(dwTrb2);

	DEBUG_PRINTF(3, "%s: event transfer TRB: "
	    "index=%d, epno=%d\n", __func__, slot, epid);

	switch (epid) {
	case XHCI_DC_EPID_OUT:
	case XHCI_DC_EPID_OUT_INTEL:
		ring = &sc->udb_oring;
		break;
	case XHCI_DC_EPID_IN:
	case XHCI_DC_EPID_IN_INTEL:
		ring = &sc->udb_iring;
		break;
	default:
		ring = &sc->udb_ering;
	}
	ring->lasterror = error;

	if (error != XHCI_TRB_ERROR_SUCCESS &&
	    error != XHCI_TRB_ERROR_SHORT_PKT) {
		DEBUG_PRINTF(1, "%s: transfer error (%u)", __func__, error);
		ring->ec[error]++;

		switch (error) {
		case XHCI_TRB_ERROR_RING_UNDERRUN:
			DEBUG_PRINTF(1, ": RING_UNDERRUN\n");
			break;
		case XHCI_TRB_ERROR_RING_OVERRUN:
			DEBUG_PRINTF(1, ": RING_OVERRUN\n");
			break;
		case XHCI_TRB_ERROR_TRB:
			DEBUG_PRINTF(1, ": TRB\n");
			break;
		case XHCI_TRB_ERROR_STALL:
			DEBUG_PRINTF(1, ": STALL\n");
			break;
		default:
			DEBUG_PRINTF(1, "\n");
		}
       	}
	if (ring != &sc->udb_ering)
		mtx_lock_spin(&ring->mtx);
	if (epid == XHCI_DC_EPID_OUT || epid == XHCI_DC_EPID_OUT_INTEL ||
	    epid == XHCI_DC_EPID_IN || epid == XHCI_DC_EPID_IN_INTEL) {
		if ((dwTrb3 & XHCI_TRB_3_ISP_BIT) != 0) {
			/* Event Data */
			goto end;
		}
		deq_old = ring->deq;
		deq_on_er = 0;
		deq_mark = " ";

		LFENCE();
#if 0
		dwEpCtx0 = XHCI_EPCTX_0_EPSTATE_GET(
		    le32toh(sc->udb_ctx->ep_out.ctx.dwEpCtx0));
#endif
		if (error == XHCI_TRB_ERROR_STALL) {
			/* Halted state (sec 7.6.4.3) */
			/* Update the deq pointer using ctx. */

			/* Wait for retry */

			LFENCE();
			qwEpCtx2 = (ring == &sc->udb_oring)
			    ? le64toh(sc->udb_ctx->
				ep_out.ctx.qwEpCtx2) &
				XHCI_EPCTX_2_TR_DQ_PTR_MASK
			    : le64toh(sc->udb_ctx->
				ep_in.ctx.qwEpCtx2) &
				XHCI_EPCTX_2_TR_DQ_PTR_MASK;

			ring->deq = qwEpCtx2 - (uint64_t)ring->paddr;
			ring->deq /= sizeof(struct xhci_trb);
			ring->deq &= UDB_TRB_RING_OFFSET_MASK;

			/*
			 * XXXHRS: Roll back the enqueued TRB.
			 * I am unsure of whether this is correct or not, but
			 * the enqueued TRB will not generate another event TRB
			 * with XHCI_TRB_ERROR_SUCCESS after this event.
			 */
			ring->enq = ring->deq;
			IOCBITMAP_CLEAR(ring, ring->enq);

			trb = &ring->trb[ring->deq];
		} else if (error == XHCI_TRB_ERROR_INVALID) { 

			/* do nothing */

			trb = NULL;
		} else if (error == XHCI_TRB_ERROR_SUCCESS &&
		    XHCI_TRB_2_BYTES_GET(dwTrb2) == 0 && qwTrb0 == 0) {
			/*
			 * Use the current deq value
			 * because Transfer Event TRBs with
			 * zero length have no valid TRB
			 * pointer.
			 * XXXHRS: where is this in the spec?
			 */

			/*
			 * Use the next of deq_old because the request TRB
			 * seems be ignored in this case.
			 */ 
			ring->deq = deq_old + 1;
			ring->deq &= UDB_TRB_RING_OFFSET_MASK;
			IOCBITMAP_CLEAR(ring, ring->deq);
			deq_mark = "!";
			ring->lastdeq = ring->deq;
			trb = NULL;
		} else {
			/* Update the deq pointer using ER. */
			/* XXX: should be optimized. */
			/* (qwTrb0 & MASK) >> 4 (16 bytes) ? */

			ring->deq = qwTrb0 - (uint64_t)ring->paddr;
			ring->deq /= sizeof(struct xhci_trb);
			ring->deq &= UDB_TRB_RING_OFFSET_MASK;

			/* Clear the obtained Enq bit */
			if (IOCBITMAP_GET(ring, ring->deq) == 0)
				deq_mark = "*";
			IOCBITMAP_CLEAR(ring, ring->deq);
			ring->lastdeq = ring->deq;
			trb = &ring->trb[ring->deq];

			/*
			 * XXX: Workaround for Forward jump.  This is
			 * the biggest problem.  Dequeue pointers reported
			 * in event TRBs are not sometimes monotonic.
			 * If a jump is detected, use the +1 address instead.
			 */
			if (ring->deq < deq_old &&
			    1 < (ring->deq + UDB_TRB_RING_LEN) - deq_old) {
				deq_on_er = ring->deq;
				ring->deq = deq_old + 1;
				ring->deq &= UDB_TRB_RING_OFFSET_MASK;
			} else if (deq_old < ring->deq && 
			    1 < ring->deq - deq_old) {
				deq_on_er = ring->deq;
				ring->deq = deq_old + 1;
				ring->deq &= UDB_TRB_RING_OFFSET_MASK;
			}
		}
	}
	/* debug printf() */
	if (epid == XHCI_DC_EPID_OUT || epid == XHCI_DC_EPID_OUT_INTEL ||
	    epid == XHCI_DC_EPID_IN || epid == XHCI_DC_EPID_IN_INTEL) {
		if ((error == XHCI_TRB_ERROR_SUCCESS ||
		     error == XHCI_TRB_ERROR_SHORT_PKT) &&
		    trb != NULL) {
			len = XHCI_TRB_2_BYTES_GET(le32toh(trb->dwTrb2))
			    - XHCI_TRB_2_BYTES_GET(dwTrb2);

			DEBUG_PRINTF(2, "%s: er->deq=%u, transfer length = "
			    "%u - %u = %u\n",
			    __func__, sc->udb_ering.deq,
			    XHCI_TRB_2_BYTES_GET(le32toh(trb->dwTrb2)),
			    XHCI_TRB_2_BYTES_GET(dwTrb2),
			    len);
		}
	}
	if (epid == XHCI_DC_EPID_IN || epid == XHCI_DC_EPID_IN_INTEL) {
		if (error == XHCI_TRB_ERROR_SUCCESS ||
		    error == XHCI_TRB_ERROR_SHORT_PKT) {

			/*
			 * The new dequeue pointer is the next of
			 * the reported one.
			 */
			if (!XHCI_DEBUG_RING_EMPTY(ring))
				ring->deq = (ring->deq + 1) &
				    UDB_TRB_RING_OFFSET_MASK;

			if (trb != NULL) {
				len = XHCI_TRB_2_BYTES_GET(le32toh(trb->dwTrb2))
				    - XHCI_TRB_2_BYTES_GET(dwTrb2);

				/* Advance the enq of work q */
				flush_range(&ring->work.buf[ring->work.enq],
				    len);
				ring->work.enq = (ring->work.enq + len)
				    & UDB_WORK_RING_OFFSET_MASK;
			}
		}
	}
	if (epid == XHCI_DC_EPID_OUT || epid == XHCI_DC_EPID_OUT_INTEL ||
	    epid == XHCI_DC_EPID_IN || epid == XHCI_DC_EPID_IN_INTEL) {
		/* printf() debug */
		DEBUG_PRINTF(1,
		    "%s: er->deq=%u"
		    ", er: qwTrb0=%lx"
		    ", dwTrb2=%x"
		    ", dwTrb3=%x"
		    ", type=%d"
		    ", error=%d"
		    ", TRDP=%lx"
		    ", RING e/d=%u/%u -> %u (%u)%s, ",
		    __func__,
		    sc->udb_ering.deq,
		    qwTrb0,
		    dwTrb2,
		    dwTrb3,
		    XHCI_TRB_3_TYPE_GET(dwTrb3),
		    error,
		    (error == XHCI_TRB_ERROR_STALL)
		    ? qwEpCtx2 & XHCI_EPCTX_2_TR_DQ_PTR_MASK
		    : ring->paddr,
		    ring->enq, deq_old, ring->deq, deq_on_er, deq_mark);
#ifdef _KERNEL
		if (1 <= udb_debug) {
			struct sbuf *sb;

			sb = SBUF_NEW_AUTO();
			
			DUMP_RING(ring);

			if (SBUF_FINISH(sb) == 0)
				printf("%s", SBUF_DATA(sb));

			SBUF_DELETE(sb);
		}
#endif
		DEBUG_PRINTF(3, "addr: paddr=%lu, qwTrb0=%lu, "
		    "masked addr=%lu, "
		    "calculated index=%lu\n",
		    (uint64_t)ring->paddr,
		    qwTrb0,
		    (qwTrb0 & (uint64_t)UDB_TRB_RING_MASK) >> 4,
		    ((qwTrb0 - (uint64_t)ring->paddr) / sizeof(struct xhci_trb))
		);
	}
	if (epid != XHCI_DC_EPID_OUT && epid != XHCI_DC_EPID_OUT_INTEL &&
	    epid != XHCI_DC_EPID_IN && epid != XHCI_DC_EPID_IN_INTEL) {
		DEBUG_PRINTF(1, "%s: unknown EP ID=%d\n",
		    __func__, XHCI_TRB_3_EP_GET(dwTrb3));
		ring->ec[XHCI_TRB_ERROR_UNKNOWN]++;
		DEBUG_PRINTF(1,
		    "%s: er->deq=%u"
		    ", er: qwTrb0=%lx"
		    ", dwTrb2=%x"
		    ", dwTrb3=%x"
		    ", type=%d"
		    ", error=%d\n",
		    __func__,
		    sc->udb_ering.deq,
		    qwTrb0,
		    dwTrb2,
		    dwTrb3,
		    XHCI_TRB_3_TYPE_GET(dwTrb3),
		    error);
	}
end:
	if (ring != &sc->udb_ering)
		mtx_unlock_spin(&ring->mtx);
}

void
xhci_debug_event_dequeue(struct xhci_debug_softc *sc)
{
	struct xhci_debug_ring *er;
	struct xhci_trb *ev;
	uint64_t qwTrb0;
	uint32_t dwTrb3, dwTrb2;
	uint64_t erdp;
	bool erdp_update;
	uint32_t temp;

	er = &sc->udb_ering;
	er->ec[XHCI_TRB_ERROR_CALLED]++;
	if (mtx_trylock_spin(&er->mtx) == 0) {
		er->ec[XHCI_TRB_ERROR_EVLOCKED]++;
		return;
	}

 	ev = &er->trb[er->deq];
	sc->sc_polling_count++;
	erdp_update = false;

	LFENCE();
	qwTrb0 = le64toh(ev->qwTrb0);
	dwTrb2 = le32toh(ev->dwTrb2);
	dwTrb3 = le32toh(ev->dwTrb3);
	while ((dwTrb3 & XHCI_TRB_3_CYCLE_BIT) == er->cyc) {
		switch (XHCI_TRB_3_TYPE_GET(dwTrb3)) {
		case XHCI_TRB_EVENT_TRANSFER:
			xhci_debug_event_tx(sc, qwTrb0, dwTrb2, dwTrb3);
			break;
		case XHCI_TRB_EVENT_PORT_STS_CHANGE:
			temp = _XREAD4(sc, dbc, XHCI_DCPORTSC);
			if (XHCI_DCPORTSC_CSC_GET(le32toh(temp))) {
				DEBUG_PRINTF(1, "cable status changed\n");

				if (XHCI_DCPORTSC_CCS_GET(le32toh(temp)) == 0)
					DEBUG_PRINTF(1, "cable unplugged\n");
			}
			_XWRITE4(sc, dbc, XHCI_DCPORTSC,
			    temp & htole32(XHCI_DCPORTSC_ACK_MASK));
			break;
		default:
			er->ec[XHCI_TRB_ERROR_UNKNOWN]++;

			DEBUG_PRINTF(1,
			    "%s: er->deq=%u"
			    ", er: qwTrb0=%lx"
			    ", dwTrb2=%x"
			    ", dwTrb3=%x"
			    ", type=%d (unknown)"
			    ", ec_unknown=%lu\n",
			    __func__,
			    er->deq,
			    qwTrb0,
			    dwTrb2,
			    dwTrb3,
			    XHCI_TRB_3_TYPE_GET(dwTrb3),
			    er->ec[XHCI_TRB_ERROR_UNKNOWN]);
			break;
		}
		if (er->deq == UDB_TRB_RING_LAST)
			er->cyc ^= 1;
		er->deq = (er->deq + 1) & UDB_TRB_RING_OFFSET_MASK;
		erdp_update = true;

		ev = &er->trb[er->deq];
		LFENCE();
		qwTrb0 = le64toh(ev->qwTrb0);
		dwTrb2 = le32toh(ev->dwTrb2);
		dwTrb3 = le32toh(ev->dwTrb3);
	}
	if (erdp_update) {
		erdp = _XREAD44LH(sc, dbc, XHCI_DCERDP);
		erdp &= ~UDB_TRB_RING_MASK;
		erdp |= er->deq * sizeof(struct xhci_trb);
		_XWRITE44LH(sc, dbc, XHCI_DCERDP, erdp);
	}
	mtx_unlock_spin(&er->mtx);
}

static uint64_t
trb_rx_enqueue_locked(struct xhci_debug_softc *sc, struct xhci_debug_ring *ring)
{
	struct xhci_debug_work_ring *work;
	uint64_t addr;
	uint64_t len;

	if (ring == NULL)
		return (0);
	if ((work = &ring->work) == NULL)
		return (0);
	/* Skip if in-progress */
	if (ring->flags != 0)
		return (0);
	/* Skip if there are pending TRB */
	if (!XHCI_DEBUG_RING_EMPTY(ring))
		return (0);
	/* Skip if there are received data */
	if (!UDB_WORK_RING_EMPTY(ring))
		return (0);

	addr = work->paddr + work->enq;
	len = (work->enq < work->deq) ? work->deq : UDB_WORK_RING_LEN;
	len -= work->enq;

	return (trb_push_locked(sc, ring, addr, len));
}

static uint64_t
trb_tx_enqueue_locked(struct xhci_debug_softc *sc, struct xhci_debug_ring *ring)
{
	struct xhci_debug_work_ring *work;
	uint64_t addr;
	uint64_t len0, len1;

	if (ring == NULL)
		return (0);
	if ((work = &ring->work) == NULL)
		return (0);
	if (ring->flags != 0)
		return (0);
	if (UDB_WORK_RING_EMPTY(ring))
		return (0);

	/* XXX: pending check? */

	len0 = len1 = 0;
	if (work->deq < work->enq) {
		addr = work->paddr + work->deq;
		len0 = trb_push_locked(sc, ring, addr, work->enq - work->deq);

		work->deq += len0;
	} else {
		addr = work->paddr + work->deq;
		len0 = trb_push_locked(sc, ring, addr,
		    UDB_WORK_RING_LEN - work->deq);

		work->deq += len0;
		work->deq &= UDB_WORK_RING_OFFSET_MASK;

		if (work->deq == 0 &&
		    !UDB_WORK_RING_EMPTY(ring) &&
		    !XHCI_DEBUG_RING_FULL(ring)) {
			addr = work->paddr;
			len1 = trb_push_locked(sc, ring, addr, work->enq);

			work->deq += len1;
		}
	}
	return (len0 + len1);
}

static uint64_t
trb_push_locked(struct xhci_debug_softc *sc, struct xhci_debug_ring *ring,
    uint64_t addr, uint64_t len)
{
	struct xhci_trb trb;
	const char *label;

	if (ring->enq == UDB_TRB_RING_LAST) {
		/*
		 * If it is the last entry, whose TRB type is LINK,
		 * cyc is set and ring->enq is advanced.
		 * The next enq should always be zero.
		 */
		ring->trb[ring->enq].dwTrb3 &= ~htole32(XHCI_TRB_3_CYCLE_BIT);
		ring->trb[ring->enq].dwTrb3 |= htole32(ring->cyc);
		flush_range(&ring->trb[ring->enq], sizeof(ring->trb[0]));

		/* Must be zero */
		ring->enq = (ring->enq + 1) & UDB_TRB_RING_OFFSET_MASK;
		ring->cyc ^= 1;
	}
	trb = (struct xhci_trb){
	    .qwTrb0 = htole64(addr),
	    .dwTrb2 = htole32(XHCI_TRB_2_BYTES_SET((uint32_t)len)),
	    .dwTrb3 = htole32(
		XHCI_TRB_3_TYPE_SET(XHCI_TRB_TYPE_NORMAL)
		| ring->cyc | XHCI_TRB_3_IOC_BIT
	    )
	};

	/* XXX: overwrite check of iocbitmap? */

	/* Note that the last entry never have a bit in the iocbitmap. */
	IOCBITMAP_SET(ring, ring->enq);
	ring->lastenq = ring->enq;

	if (XHCI_DEBUG_RING_IN(ring))
		label = "IR";
	else if (XHCI_DEBUG_RING_OUT(ring))
		label = "OR";
	else
		label = "ER";
	DEBUG_PRINTF(1, "%s: enqueue: %s: enq=%d, len=%lu, deq=%d\n", __func__,
	    label, ring->enq, len, ring->deq);
	memcpy(&ring->trb[ring->enq], &trb, sizeof(ring->trb[0]));
	flush_range(&ring->trb[ring->enq], sizeof(ring->trb[0]));
	ring->enq = (ring->enq + 1) & UDB_TRB_RING_OFFSET_MASK;

	xhci_debug_ring_doorbell(sc, ring->doorbell);

	return (len);
}

int
work_dequeue(struct xhci_debug_ring *ring)
{
	struct xhci_debug_work_ring *work;
	int c;

	if (ring == NULL)
		return (-1);
	if ((work = &ring->work) == NULL)
		return (-1);

	c = -1;
	mtx_lock_spin(&ring->mtx);
#ifdef _KERNEL
	if (1 <= udb_debug)
		xhci_debug_show_ring(ring);
#endif
	if (!UDB_WORK_RING_EMPTY(ring)) {
		c = (int)work->buf[work->deq];
		work->deq = (work->deq + 1) & UDB_WORK_RING_OFFSET_MASK;
		DEBUG_PRINTF(2, "%s: [%c(%02x)]\n", __func__,
		    (char)c, (char)c);
	}
#ifdef _KERNEL
	if (1 <= udb_debug)
		xhci_debug_show_ring(ring);
#endif
	mtx_unlock_spin(&ring->mtx);
	return (c);
}

int64_t
work_enqueue(struct xhci_debug_ring *ring, const char *buf, int64_t len)
{
	struct xhci_debug_work_ring *work;
	int64_t i = 0;
	uint32_t start;
	uint32_t end = 0;

	if (ring == NULL)
		return (0);
	if ((work = &ring->work) == NULL)
		return (0);
	if (UDB_WORK_RING_FULL(ring))
		return (0);

	mtx_lock_spin(&ring->mtx);
	start = work->enq;
	DEBUG_PRINTF(1, "%s: %s: len=%lu: ", __func__,
	    XHCI_DEBUG_RING_IN(ring) ? "IR" : "OR", len);
	while (!UDB_WORK_RING_FULL(ring) && i < len) {
		DEBUG_PRINTF(2, "[%c(%02x)]", buf[i], buf[i]);
		work->buf[work->enq++] = buf[i++];
		work->enq &= UDB_WORK_RING_OFFSET_MASK;
	}
	end = work->enq;
	DEBUG_PRINTF(1, ": start=%u, end=%u\n", start, end);

	if (start < end)
		flush_range(&work->buf[start], end - start);
	else if (0 < i) {
		flush_range(&work->buf[start], UDB_WORK_RING_LEN - start);
		flush_range(&work->buf[0], end);
	}
	mtx_unlock_spin(&ring->mtx);
	return (i);
}

#ifdef _KERNEL
/* udbcons is the device node for USB DbC. */

#define	UDBCONS_NAME	"udbcons"

struct udbcons_priv;
static int udbcons_ischar(struct udbcons_priv *);
typedef void udbcons_putc_t(struct udbcons_priv *, int);
typedef int udbcons_getc_t(struct udbcons_priv *);
struct udbcons_ops {
	udbcons_putc_t	*putc;
	udbcons_getc_t	*getc;
};
struct udbcons_priv {
	device_t			dev;	/* XHCI */
	struct xhci_softc		*xhci;
	const struct udbcons_ops	*ops;
	struct callout			callout;
	struct tty			*tp;
	bool				opened;
	int				polltime;
	int				alt_break_state;
};

static struct udbcons_priv udbcons0;

static int
sysctl_sc_polling_count(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc;
	int error;
	uint64_t value;

	if (udbcons0.xhci == NULL)
		return (-1);
	if ((sc = udbcons0.xhci->sc_udbc) == NULL)
		return (-1);

	value = sc->sc_polling_count;

	error = sysctl_handle_64(oidp, &value, 0, req);

	return (error);
}
SYSCTL_PROC(_hw_usb_xhci_dbc, OID_AUTO, polling_count,
    CTLTYPE_U64 | CTLFLAG_RD | CTLFLAG_MPSAFE,
    NULL, 0, sysctl_sc_polling_count, "LU", "DbC polling count"); 

static int
sysctl_ering(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc;
	struct sbuf *sb;
	int error;

	if (udbcons0.xhci == NULL)
		return (-1);
	if ((sc = udbcons0.xhci->sc_udbc) == NULL)
		return (-1);

	sb = SBUF_NEW_AUTO();
	DUMP_RING(&sc->udb_ering);
	if (SBUF_FINISH(sb) != 0)
		return (-1);
	error = sysctl_handle_opaque(oidp, SBUF_DATA(sb), SBUF_LEN(sb), req);
	SBUF_DELETE(sb);

	return (error);
}
static SYSCTL_NODE(_hw_usb_xhci_dbc, OID_AUTO, ering,
    CTLFLAG_RW | CTLFLAG_MPSAFE, 0, "Event Ring");
SYSCTL_PROC(_hw_usb_xhci_dbc_ering, OID_AUTO, info,
    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
    NULL, 0, sysctl_ering, "A", "Event Ring Info"); 

static int
sysctl_oring(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc;
	struct sbuf *sb;
	int error;

	if (udbcons0.xhci == NULL)
		return (-1);
	if ((sc = udbcons0.xhci->sc_udbc) == NULL)
		return (-1);

	sb = SBUF_NEW_AUTO();
	DUMP_RING(&sc->udb_oring);
	if (SBUF_FINISH(sb) != 0)
		return (-1);
	error = sysctl_handle_opaque(oidp, SBUF_DATA(sb), SBUF_LEN(sb), req);
	SBUF_DELETE(sb);

	return (error);
}
static SYSCTL_NODE(_hw_usb_xhci_dbc, OID_AUTO, oring,
    CTLFLAG_RW | CTLFLAG_MPSAFE, 0, "Output Ring");
SYSCTL_PROC(_hw_usb_xhci_dbc_oring, OID_AUTO, info,
    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
    NULL, 0, sysctl_oring, "A", "Output Ring Info"); 

static int
sysctl_iring(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc;
	struct sbuf *sb;
	int error;

	if (udbcons0.xhci == NULL)
		return (-1);
	if ((sc = udbcons0.xhci->sc_udbc) == NULL)
		return (-1);

	sb = SBUF_NEW_AUTO();
	DUMP_RING(&sc->udb_iring);
	if (SBUF_FINISH(sb) != 0)
		return (-1);
	error = sysctl_handle_opaque(oidp, SBUF_DATA(sb), SBUF_LEN(sb), req);
	SBUF_DELETE(sb);

	return (error);
}
static SYSCTL_NODE(_hw_usb_xhci_dbc, OID_AUTO, iring,
    CTLFLAG_RW | CTLFLAG_MPSAFE, 0, "Input Ring");
SYSCTL_PROC(_hw_usb_xhci_dbc_iring, OID_AUTO, info,
    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
    NULL, 0, sysctl_iring, "A", "Input Ring Info"); 

static int
sysctl_sc_state(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc;
	char buf[64];
	int error;

	if (udbcons0.xhci == NULL)
		return (-1);
	if ((sc = udbcons0.xhci->sc_udbc) == NULL)
		return (-1);
	snprintf(buf, sizeof(buf), "%s(%02x)", strstate[sc->sc_state],
	    sc->sc_state);
	buf[sizeof(buf) - 1] = '\0';
	error = sysctl_handle_opaque(oidp, buf, sizeof(buf), req);

	return (error);
}
SYSCTL_PROC(_hw_usb_xhci_dbc, OID_AUTO, state,
    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
    NULL, 0, sysctl_sc_state, "A", "DbC state"); 

static int
sysctl_dcst_portnum(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc;
	uint32_t temp;
	int error, value;

	if (udbcons0.xhci == NULL)
		return (-1);
	if ((sc = udbcons0.xhci->sc_udbc) == NULL)
		return (-1);
	temp = _XREAD4(sc, dbc, XHCI_DCST);
	value = XHCI_DCST_PORT_GET(le32toh(temp));

	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error)
		return (error);
	return (0);
}
SYSCTL_PROC(_hw_usb_xhci_dbc, OID_AUTO, portnum,
    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
    NULL, 0, sysctl_dcst_portnum, "I", "Debug Port Number"); 

static int
sysctl_dcst_er(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc;
	uint32_t temp;
	int error, value;

	if (udbcons0.xhci == NULL)
		return (-1);
	if ((sc = udbcons0.xhci->sc_udbc) == NULL)
		return (-1);
	temp = _XREAD4(sc, dbc, XHCI_DCST);
	value = XHCI_DCST_ER_GET(le32toh(temp));

	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error)
		return (error);
	return (0);
}
SYSCTL_PROC(_hw_usb_xhci_dbc, OID_AUTO, dcst_er,
    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
    NULL, 0, sysctl_dcst_er, "I", "DCST Event Ring Not Empty"); 

static int
sysctl_dcst_sbr(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc;
	uint32_t temp;
	int error, value;

	if (udbcons0.xhci == NULL)
		return (-1);
	if ((sc = udbcons0.xhci->sc_udbc) == NULL)
		return (-1);
	temp = _XREAD4(sc, dbc, XHCI_DCST);
	value = XHCI_DCST_SBR_GET(le32toh(temp));

	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error)
		return (error);
	return (0);
	
}
SYSCTL_PROC(_hw_usb_xhci_dbc, OID_AUTO, dcst_sbr,
    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
    NULL, 0, sysctl_dcst_sbr, "I", "DCST DbC System Bus Reset"); 

static int
udbcons_getc(struct udbcons_priv *cons)
{
	struct xhci_softc *sc;
	struct xhci_debug_softc *sc_udbc;
	struct xhci_debug_ring *iring;

	if (cons == NULL)
		 return (-1);
	if ((sc = cons->xhci) == NULL)
		 return (-1);
	if (sc->sc_dbc_off == 0)
		 return (-1);
	if ((sc_udbc = sc->sc_udbc) == NULL)
		return (-1);

	iring = &sc_udbc->udb_iring;

	/* Use udbcons_ischar() to receive data */
	return (udbcons_ischar(cons) ? work_dequeue(iring) : -1);
}
static int
udbcons_ischar(struct udbcons_priv *cons)
{
	struct xhci_softc *sc;
	struct xhci_debug_softc *sc_udbc;
	struct xhci_debug_ring *iring;

	if (cons == NULL)
		 return (0);
	if ((sc = cons->xhci) == NULL)
		 return (0);
	if (sc->sc_dbc_off == 0)
		return (0);
	if ((sc_udbc = sc->sc_udbc) == NULL)
		return (0);

	iring = &sc_udbc->udb_iring;
	if (!UDB_WORK_RING_EMPTY(iring))
		return (1);

	xhci_debug_bulk_transfer(sc_udbc);
	xhci_debug_event_dequeue(sc_udbc);

	return (!UDB_WORK_RING_EMPTY(iring));
}
static void
udbcons_putc(struct udbcons_priv *cons, int ch)
{
	struct xhci_softc *sc;
	struct xhci_debug_softc *sc_udbc;
	struct xhci_debug_ring *oring;
	char c;

	if (cons == NULL)
		 return;
	if ((sc = cons->xhci) == NULL)
		 return;
	if (sc->sc_dbc_off == 0)
		return;
	if ((sc_udbc = sc->sc_udbc) == NULL)
		return;

	oring = &sc_udbc->udb_oring;
	c = (char)(ch & 0xff);
	if (work_enqueue(oring, &c, sizeof(c)) != sizeof(c))
		return;

	xhci_debug_bulk_transfer(sc_udbc);
	xhci_debug_event_dequeue(sc_udbc);
}
static struct udbcons_ops udbcons0_ops = {
	.putc = udbcons_putc,
	.getc = udbcons_getc,
};

static tsw_outwakeup_t xhci_debug_tty_outwakeup;

static struct ttydevsw xhci_debug_ttydevsw = {
	.tsw_flags	= TF_NOPREFIX,
	.tsw_outwakeup	= xhci_debug_tty_outwakeup,
};

static void xhci_debug_timeout(void *);

int
udbcons_init(struct xhci_softc *sc, device_t dev)
{
	struct udbcons_priv *cons;

	cons = &udbcons0;
	cons->dev = dev;
	cons->xhci = sc;
	cons->ops = &udbcons0_ops;

	cons->tp = tty_alloc(&xhci_debug_ttydevsw, cons);
	cons->polltime = 10;

	tty_makedev(cons->tp, NULL, "%s", UDBCONS_NAME);
	tty_init_console(cons->tp, 0);

	callout_init(&cons->callout, 1);
	callout_reset(&cons->callout, cons->polltime,
	    xhci_debug_timeout, cons->tp);

	return (0);
}

/* xhci_debug_cn* are methods for the UDBCONS console instance */

static cn_probe_t xhci_debug_cnprobe;
static cn_init_t xhci_debug_cninit;
static cn_term_t xhci_debug_cnterm;
static cn_getc_t xhci_debug_cngetc;
static cn_putc_t xhci_debug_cnputc;
static cn_grab_t xhci_debug_cngrab;
static cn_ungrab_t xhci_debug_cnungrab;

const struct consdev_ops xhci_debug_cnops = {
	.cn_probe	= xhci_debug_cnprobe,
	.cn_init	= xhci_debug_cninit,
	.cn_term	= xhci_debug_cnterm,
	.cn_getc	= xhci_debug_cngetc,
	.cn_putc	= xhci_debug_cnputc,
	.cn_grab	= xhci_debug_cngrab,
	.cn_ungrab	= xhci_debug_cnungrab,
};

CONSOLE_DRIVER(xhci_debug);

#define MAX_BURST_LEN	1
static void
xhci_debug_tty_outwakeup(struct tty *tp)
{
	struct udbcons_priv *cons;
	u_char buf[MAX_BURST_LEN];
	int len;
	int i;

	cons = tty_softc(tp);
	for (;;) {
		len = ttydisc_getc(tp, buf, sizeof(buf));
		if (len == 0)
			break;
		KASSERT(len == 1, ("tty error"));

		for (i = 0; i < len; i++)
			udbcons_putc(cons, buf[i]);
	}
}

static void
xhci_debug_timeout(void *v)
{
	struct udbcons_priv *cons;
	struct xhci_debug_softc *sc;
	struct tty *tp;
	int ch;

	tp = v;
	cons = tty_softc(tp);

	if (cons->xhci == NULL)
		goto end;
	if (cons->xhci->sc_udbc == NULL)
		goto end;
	sc = cons->xhci->sc_udbc;

	if (udb_reset > 0) {
		/* XXX: still does not work. */
		int error;

		mtx_lock_spin(&sc->udb_ering.mtx);
		mtx_lock_spin(&sc->udb_iring.mtx);
		mtx_lock_spin(&sc->udb_oring.mtx);

		udb_reset = 0;
		xhci_debug_disable(sc);
		xhci_debug_update_state(sc);
		error = xhci_debug_init_ring(sc);
		if (error)
			goto end;
		sc->sc_init = true;
		xhci_debug_enable(sc);
		xhci_debug_update_state(sc);

		mtx_unlock_spin(&sc->udb_oring.mtx);
		mtx_unlock_spin(&sc->udb_iring.mtx);
		mtx_unlock_spin(&sc->udb_ering.mtx);
	} else {
		tty_lock(tp);
		while ((ch = xhci_debug_cngetc(NULL)) != -1)
			ttydisc_rint(tp, ch, 0);
		ttydisc_rint_done(tp);
		tty_unlock(tp);
	}
end:
	callout_reset(&cons->callout, cons->polltime, xhci_debug_timeout, tp);
}

static void
xhci_debug_cnprobe(struct consdev *cp)
{

	cp->cn_pri = (boothowto & RB_SERIAL) ? CN_REMOTE : CN_NORMAL;
	sprintf(cp->cn_name, "%s", UDBCONS_NAME);
}

static void
xhci_debug_cninit(struct consdev *cp)
{
}

static void
xhci_debug_cnterm(struct consdev *cp)
{
}

/* empty */
static void
xhci_debug_cngrab(struct consdev *cp)
{
}

/* empty */
static void
xhci_debug_cnungrab(struct consdev *cp)
{
}

static int
xhci_debug_cngetc(struct consdev *cp)
{
	int ch;

	ch = udbcons_getc(&udbcons0);
	if (ch > 0 && ch < 0xff) {
#if defined(KDB)
		kdb_alt_break(ch, &udbcons0.alt_break_state);
#endif
		return (ch);
	}
	return (-1);
}

static void
xhci_debug_cnputc(struct consdev *cp, int ch)
{

	udbcons_putc(&udbcons0, ch);
}

#if 0
#if defined(GDB)
/* GDB methods */

static gdb_probe_f	xhci_debug_dbg_probe;
static gdb_init_f	xhci_debug_dbg_init;
static gdb_term_f	xhci_debug_dbg_term;
static gdb_getc_f	xhci_debug_dbg_getc;
static gdb_putc_f	xhci_debug_dbg_putc;

GDB_DBGPORT(udbcons, xhci_debug_dbg_probe, xhci_debug_dbg_init,
    xhci_debug_dbg_term, xhci_debug_dbg_getc, xhci_debug_dbg_putc);

extern struct gdb_dbgport *gdb_cur;
#endif
#endif
#endif
