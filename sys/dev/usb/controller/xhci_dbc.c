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
#include <machine/atomic.h>

#ifdef _KERNEL
# include <sys/bus.h>
# include <sys/cons.h>
# include <sys/kdb.h>
# include <sys/module.h>
# include <sys/tty.h>
# include <sys/reboot.h>
# include <sys/sbuf.h>
# include <sys/sysctl.h>
# include <vm/vm.h>
# include <vm/vm_param.h>
# include <vm/pmap.h>
# include <gdb/gdb.h>
# include <ddb/ddb.h>
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
# include <dev/pci/pcireg.h>
# include <dev/pci/pcivar.h>
# ifdef __aarch64__
# include <machine/cpufunc.h>
# endif
#else
# include <bootstrap.h>
# include <efi.h>
# include <efilib.h>
# include <xhci_dbc_pci.h>	/* device_t */
# include <dev/usb/controller/xhci_private.h>
#endif

#include <dev/usb/controller/xhcireg.h>
#include <dev/usb/controller/xhci_dbc.h>
#include <dev/usb/controller/xhci_dbc_private.h>

#ifdef _KERNEL
#include "opt_kdb.h"
#include "opt_gdb.h"
#include "opt_ddb.h"
#endif

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

#define	DUMP_RING(ring, cr) do {				\
	int _i, _j, _lsb;					\
	uint32_t enq;						\
	bool _update_enq;					\
	const char *_label;					\
								\
	_update_enq = false;					\
	if (XHCI_DC_RING_IN(ring))				\
		_label = "IR";					\
	else if (XHCI_DC_RING_OUT(ring))			\
		_label = "OR";					\
	else {							\
		_update_enq = true;				\
		_label = "ER";					\
	}							\
	enq = xhci_debug_ring_boundary(ring, _update_enq);	\
	SBUF_PRINTF(sb, "%s:", _label); 			\
	SBUF_PRINTF(sb, ", e/E/d=%d/%d/%d", (ring)->enq, enq, (ring)->deq);\
	SBUF_PRINTF(sb, "(%u)", (uint32_t)XHCI_DC_RING_SLOTS(ring));	\
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
	SBUF_PRINTF(sb, "(%u)", (uint32_t)DC_WORK_RING_SLOTS(&(ring)->work));\
	SBUF_PRINTF(sb, ", paddr(/w)=%p/%p", (void *)(ring)->paddr, \
	    (void *)(ring)->work.paddr);			\
	SBUF_PRINTF(sb, ", doorbell=%04x", (ring)->doorbell);	\
	SBUF_PRINTF(sb, ", cyc=%02x", (ring)->cyc);		\
	SBUF_PRINTF(sb, ", lasterror=%u", (ring)->lasterror);	\
	SBUF_PRINTF(sb, ", lastenq=%u", (ring)->lastenq);	\
	SBUF_PRINTF(sb, ", lastdeq=%u", (ring)->lastdeq);	\
	SBUF_PRINTF(sb, ", ec[STALL/TRB/EVLOCKED/CALLED/WORKQFULL/UNKNOWN]=" \
	    "%lu/%lu/%lu/%lu/%lu/%lu",				\
	    (ring)->ec[XHCI_TRB_ERROR_STALL],			\
	    (ring)->ec[XHCI_TRB_ERROR_TRB],			\
	    (ring)->ec[XHCI_TRB_ERROR_EVLOCKED],		\
	    (ring)->ec[XHCI_TRB_ERROR_CALLED],			\
	    (ring)->ec[XHCI_TRB_ERROR_WORKQ_FULL],		\
	    (ring)->ec[XHCI_TRB_ERROR_UNKNOWN]);		\
	if (cr)							\
		SBUF_PRINTF(sb, "\n");				\
	} while (0)

static void flush_range(void *, uint32_t);

static bool xhci_debug_update_regbit(struct xhci_debug_softc *, int, int, int,
    bool);
static bool xhci_debug_set_regbit(struct xhci_debug_softc *, int, int, int);
static bool xhci_debug_reset_regbit(struct xhci_debug_softc *, int, int, int);

static bool xhci_debug_wait_connection(struct xhci_debug_softc *, uint32_t);

static struct xhci_debug_softc * xhci_debug_alloc_softc(device_t);
static bool xhci_debug_softc_fixup(struct xhci_debug_softc *);
static void xhci_debug_ring_init(struct xhci_debug_softc *,
    struct xhci_debug_ring *, int, uint32_t);
struct usb_strdesc;
static void atou16cpy(struct usb_strdesc *, const char *);
static void xhci_debug_init_strings(struct xhci_debug_softc *, uint32_t [16]);

static void xhci_debug_ring_doorbell(struct xhci_debug_softc *, uint32_t);
static uint64_t xhci_debug_bulk_transfer0(struct xhci_debug_softc *,
    struct xhci_debug_ring *);

static void xhci_debug_show_ring(struct xhci_debug_ring *, int);
static void xhci_debug_init_ep(struct xhci_endp_ctx64 *, uint64_t, uint32_t,
    uint64_t);

#if 0
static bool xhci_debug_event_ready(struct xhci_debug_softc *);
#endif
static uint32_t xhci_debug_ring_boundary(struct xhci_debug_ring *, bool);

static void xhci_debug_transfer_event_handler(struct xhci_debug_softc *,
    uint64_t, uint32_t, uint32_t);
static uint64_t trb_tx_enqueue_locked(struct xhci_debug_softc *sc,
    struct xhci_debug_ring *);
static uint64_t trb_rx_enqueue_locked(struct xhci_debug_softc *sc,
    struct xhci_debug_ring *);
static uint64_t trb_push_locked(struct xhci_debug_softc *sc,
    struct xhci_debug_ring *, uint64_t, uint64_t);

#ifdef _KERNEL
static int udbcons_attach(device_t dev);
#endif

/* sc_state */

#define	STRSTATE(x)	[XHCI_DCPORT_ST_##x] = #x
static const char *strstate[] = {
	STRSTATE(OFF),
	STRSTATE(DISCONNECTED_RUNNING),	/* not in spec */
	STRSTATE(DISCONNECTED),
	STRSTATE(DISABLED),
	STRSTATE(RESETTING),
	STRSTATE(ENABLED),
	STRSTATE(CONFIGURED)
};

/* sysctls */

int dbc_debug;
int dbc_reset;
int dbc_enable = 1;
int dbc_baud = 115200;
int dbc_gdb;
uint32_t dbc_pci_rid;

#ifdef _KERNEL
static SYSCTL_NODE(_hw_usb_xhci, OID_AUTO, dbc, CTLFLAG_RW | CTLFLAG_MPSAFE, 0,
    "USB XHCI DbC");
SYSCTL_INT(_hw_usb_xhci_dbc, OID_AUTO, enable, CTLFLAG_RWTUN, &dbc_enable, 1,
    "Set to enable XHCI DbC support");
SYSCTL_INT(_hw_usb_xhci_dbc, OID_AUTO, debug, CTLFLAG_RWTUN, &dbc_debug, 0,
    "Debug level");
SYSCTL_INT(_hw_usb_xhci_dbc, OID_AUTO, reset, CTLFLAG_RWTUN, &dbc_reset, 0,
    "Set to reset DbC");
SYSCTL_INT(_hw_usb_xhci_dbc, OID_AUTO, gdb, CTLFLAG_RWTUN, &dbc_gdb, 0,
    "Set to enable GDB");
SYSCTL_INT(_hw_usb_xhci_dbc, OID_AUTO, pci_rid, CTLFLAG_RWTUN,
    &dbc_pci_rid, 0, "Set PCI RID for console");
#endif

static struct xhci_debug_reg {
	uint32_t	erstsz;
	uint64_t	erstba;
	uint64_t	erdp;
	uint64_t	cp;
	uint32_t	ddi1;
	uint32_t	ddi2;
} udbc_reg;

void
xhci_debug_reg_read(struct xhci_debug_softc *sc)
{

	if (sc == NULL)
		return;

	udbc_reg.erstsz = _XREAD4(sc, dbc, XHCI_DCERSTSZ);
	udbc_reg.erstba = _XREAD44LH(sc, dbc, XHCI_DCERSTBA);
	udbc_reg.erdp = _XREAD44LH(sc, dbc, XHCI_DCERDP);
	udbc_reg.cp = _XREAD44LH(sc, dbc, XHCI_DCCP);
	udbc_reg.ddi1 = _XREAD4(sc, dbc, XHCI_DCDDI1);
	udbc_reg.ddi2 = _XREAD4(sc, dbc, XHCI_DCDDI2);
}
void
xhci_debug_reg_restore(struct xhci_debug_softc *sc)
{

	if (sc == NULL)
		return;

	_XWRITE4(sc, dbc, XHCI_DCERSTSZ, udbc_reg.erstsz);
	_XWRITE44LH(sc, dbc, XHCI_DCERSTBA, udbc_reg.erstba);
	_XWRITE44LH(sc, dbc, XHCI_DCERDP, udbc_reg.erdp);
	_XWRITE44LH(sc, dbc, XHCI_DCCP, udbc_reg.cp);
	_XWRITE4(sc, dbc, XHCI_DCDDI1, udbc_reg.ddi1);
	_XWRITE4(sc, dbc, XHCI_DCDDI2, udbc_reg.ddi2);
}

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

	xhci_debug_event_dequeue(sc);

	xhci_debug_update_state(sc);
	printf("=============\n");
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

	printf("    erstsz: %u, erstba: 0x%lx\n",
	    _XREAD4(sc, dbc, XHCI_DCERSTSZ),
	    _XREAD44LH(sc, dbc, XHCI_DCERSTBA));

	printf("    erdp: 0x%lx, cp: 0x%lx\t",
	    _XREAD44LH(sc, dbc, XHCI_DCERDP),
	    _XREAD44LH(sc, dbc, XHCI_DCCP));
	printf("    ddi1: 0x%x, ddi2: 0x%x\n",
	    _XREAD4(sc, dbc, XHCI_DCDDI1),
	    _XREAD4(sc, dbc, XHCI_DCDDI2));
	printf("=============\n");
	xhci_debug_ring_boundary(&sc->udb_ering, true); /* Update er->enq */
	xhci_debug_show_ring(&sc->udb_ering, 0);
	xhci_debug_show_ring(&sc->udb_iring, 0);
	xhci_debug_show_ring(&sc->udb_oring, 0);
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

uint32_t
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
		device_printf(sc->sc_dev, "state: %s(0x%02x) -> %s(0x%02x)\n",
		    strstate[sc_state_old], sc_state_old,
		    strstate[sc->sc_state], sc->sc_state);
	return (sc->sc_state);
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

	if (sc == NULL)
		return (false);
	ret = true;
	if (xhci_debug_update_state(sc) == XHCI_DCPORT_ST_OFF) {
		ret = xhci_debug_set_regbit(sc, XHCI_DCCTRL, XHCI_DCCTRL_DCE,
		    1);
		if (ret == false)
			device_printf(sc->sc_dev, "initialization failed\n");
	}
	/* If success, check the cable. */
	return ((ret == true) ? (xhci_debug_wait_connection(sc, 1)) : ret);
}

static bool
xhci_debug_wait_connection(struct xhci_debug_softc *sc, uint32_t timeout)
{
	bool ret;

	if (xhci_debug_update_state(sc) == XHCI_DCPORT_ST_DISCONNECTED) {
		ret = xhci_debug_set_regbit(sc, XHCI_DCPORTSC,
		    XHCI_DCPORTSC_PED, 1000);	/* XXX: 1 was not enough */
		if (ret == false) {
			device_printf(sc->sc_dev, "No DbC cable detected\n");
			return (ret); 
		}
	}
	timeout = 10000;
	device_printf(sc->sc_dev, "waiting for a cable\n");
	while (xhci_debug_update_state(sc) != XHCI_DCPORT_ST_CONFIGURED) {
		delay(500);
		if (timeout-- == 0) {
			device_printf(sc->sc_dev,
			    "DbC cable detection timed out\n");
			break;
		}
	}
	if (timeout == 0)
		return (false);

	device_printf(sc->sc_dev, "DbC cable detected\n");

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
	sc->udb_erst->dwEvrsTableSize = htole32(DC_TRB_RING_LEN);
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
	_XWRITE4(sc, dbc, XHCI_DCDDI1,
	    htole32((DC_VENDOR   << 16) | DC_PROTOCOL));
	_XWRITE4(sc, dbc, XHCI_DCDDI2,
	    htole32((DC_REVISION << 16) | DC_PRODUCT));
	flush_range(sc->udb_ctx, sizeof(*sc->udb_ctx));
	flush_range(sc->udb_erst, sizeof(*sc->udb_erst));
	flush_range(sc->udb_ering.trb, DC_TRB_RING_BYTES);
	flush_range(sc->udb_oring.trb, DC_TRB_RING_BYTES);
	flush_range(sc->udb_iring.trb, DC_TRB_RING_BYTES);
	flush_range(sc->udb_oring.work.buf, DC_WORK_RING_BYTES);
	flush_range(sc->udb_iring.work.buf, DC_WORK_RING_BYTES);

	return (0);
}

uint32_t
xhci_debug_get_xecp(struct xhci_debug_softc *sc)
{
	uint32_t hccp1;
	uint32_t eec;
	uint32_t eecp;

	if (sc == NULL)
		return (0);

	hccp1 = _XREAD4(sc, capa, XHCI_HCSPARAMS0);

	if (XHCI_HCS0_XECP(hccp1) == 0)
		goto notfound;

	eec = -1;	/* XXX */
	for (eecp = XHCI_HCS0_XECP(hccp1) << 2;
	     eecp != 0 && XHCI_XECP_NEXT(eec) != 0;
	     eecp += XHCI_XECP_NEXT(eec) << 2) {
		eec = _XREAD4(sc, capa, eecp);

		DEBUG_PRINTF(1, "%s: Looking for xECP: "
		    "expected=%02x, found=%02x, next=%04x\n", __func__,
		    XHCI_ID_USB_DEBUG, XHCI_XECP_ID(eec),
		    XHCI_XECP_NEXT(eec) << 2);

		if (XHCI_XECP_ID(eec) == XHCI_ID_USB_DEBUG)
			break;
	}
	if (eecp == 0 || XHCI_XECP_ID(eec) == 0)
		goto notfound;

	DEBUG_PRINTF(1, "%s: DbC was found at %08x\n", __func__, eecp);

	return (eecp);
notfound:
	DEBUG_PRINTF(1, "%s: DbC was not found\n", __func__); 

	return (0);
}

/*
 * The hw.usb.xhci.dbc.softc.* variable is used to pass physical addresses
 * from the loader to the kernel.  This function simply relies on
 * the direct mapping at this moment and works only on the platforms
 * supporting it.
 */
static struct xhci_debug_softc * 
xhci_debug_alloc_softc(device_t dev)
{
	struct xhci_debug_softc *sc = NULL;
#if _KERNEL
	struct xhci_softc *sc_xhci;
	struct xhci_debug_softc *sc0;
	quad_t addr, len;
	uintptr_t rid;

	KASSERT(PMAP_HAS_DMAP, ("direct-map required"));

	getenv_quad("hw.usb.xhci.dbc.softc.paddr", &addr);
	getenv_quad("hw.usb.xhci.dbc.softc.len", &len);

	if (addr == 0 || len == 0)
		return (NULL);

	if ((sc_xhci = device_get_ivars(dev)) == NULL)
		return (NULL);
	
	/* softc array */
	sc0 = (struct xhci_debug_softc *)
	    pmap_map(NULL, (vm_paddr_t)addr, (vm_paddr_t)addr + len, 0);
	if (pmap_change_attr((vm_offset_t)sc0,
	    (vm_offset_t)len, VM_MEMATTR_DEVICE) != 0)
		printf("%s: pmap_change_attr() failed: %p\n",
		    __func__, (void *)sc0);

	/* Fixing up the link pointers */
	for (sc = sc0; sc != NULL; sc = sc->sc_next) {
		device_printf(dev, "Fixing up RID: 0x%04x\n", sc->sc_pci_rid);
		if (sc->sc_next == NULL)
			break;
		if (sc->sc_next_fixup == true)
			continue;

		addr = (vm_paddr_t)sc->sc_next;
		len = sizeof(*sc);
		sc->sc_next = (struct xhci_debug_softc *)
		    pmap_map(NULL, (vm_paddr_t)addr,
			(vm_paddr_t)addr + len, 0);
		if (pmap_change_attr((vm_offset_t)sc->sc_next,
		    (vm_offset_t)len, VM_MEMATTR_DEVICE) != 0) {
			printf("%s: pmap_change_attr() failed: %p\n",
			    __func__, (void *)sc->sc_next);
			return (NULL);
		}
		sc->sc_next_fixup = true;
	}

	/* Check if PCI RID matching */
	if (pci_get_id(sc_xhci->sc_bus.parent, PCI_ID_RID, &rid) != 0) {
		device_printf(dev, "pci_get_id() failed\n");
		return (NULL);
	}
	for (sc = sc0; sc != NULL; sc = sc->sc_next)
		if (sc->sc_pci_rid == (uint32_t)rid)
			break;
	if (sc == NULL) {
		device_printf(dev, "RID not found: expected=0x%04x\n",
		    (uint32_t)rid);
		return (NULL);
	}
	if (sc->sc_dbc_off == 0) {
		device_printf(dev, "DbC offset was not initialized\n");
		return (NULL);
	}
	/* Fixup and initialization */
	sc->sc_xhci = sc_xhci;
	sc->sc_dev = dev;
	sc->sc_polling_time = XHCI_DC_POLLING_TIME;
	if ((sc->sc_fixup_done = xhci_debug_softc_fixup(sc)) == false)
		sc = NULL;
#endif	/* _KERNEL */
	return (sc);
}

static bool
xhci_debug_softc_fixup(struct xhci_debug_softc *sc)
{
#if _KERNEL
	quad_t addr, len;

	/* sc->sc_{dev,xhci} must be initialized before entering. */
	KASSERT(sc != NULL, ("null sc"));

	if (sc->sc_fixup_done == true)
		return (false);
	if (sc->sc_cookie != XHCI_DC_COOKIE) {
		device_printf(sc->sc_dev,
		    "COOKIE mismatch: expected=0x%04x, found=0x%04x\n",
		    XHCI_DC_COOKIE, sc->sc_cookie);
		return (false);
	}

#define	MAP(mib, type)	do {						\
		addr = mib##_paddr;					\
		len = mib##_len;					\
		(mib) = (type)						\
		    pmap_map(NULL, (vm_paddr_t)addr, (vm_paddr_t)addr + len, \
			0); \
		if (pmap_change_attr((vm_offset_t)(mib),		\
		    (vm_offset_t)len, VM_MEMATTR_DEVICE) != 0)		\
			device_printf(sc->sc_dev,			\
			    "%s: pmap_change_attr() failed: %p\n",	\
			    __func__, (void *)(mib));			\
	} while (0)
#define	MAP_RING(mib)	do {						\
		addr = (mib).paddr;					\
		len = (mib).len;					\
		(mib).trb = (struct xhci_trb *)				\
		    pmap_map(NULL, (vm_paddr_t)addr, (vm_paddr_t)addr + len, \
			0); \
		if (pmap_change_attr((vm_offset_t)(mib).trb,		\
		    (vm_offset_t)len, VM_MEMATTR_DEVICE) != 0)		\
			device_printf(sc->sc_dev,			\
			    "%s: pmap_change_attr() failed: %p\n",	\
			    __func__, (void *)(mib).trb);		\
		addr = (mib).work.paddr;				\
		len = (mib).work.len;					\
		(mib).work.buf = (char *)				\
		    pmap_map(NULL, (vm_paddr_t)addr, (vm_paddr_t)addr + len, \
			0); \
		if (pmap_change_attr((vm_offset_t)(mib).trb,		\
		    (vm_offset_t)len, VM_MEMATTR_DEVICE) != 0)		\
			device_printf(sc->sc_dev,			\
			    "%s: pmap_change_attr() failed: %p\n",	\
			    __func__, (void *)(mib).work.buf);		\
	} while (0)

	MAP(sc->udb_ctx, struct xhci_debug_ctx *);
	MAP(sc->udb_erst, struct xhci_event_ring_seg *);
	MAP(sc->udb_str, char *);
	MAP_RING(sc->udb_ering);
	MAP_RING(sc->udb_iring);
	MAP_RING(sc->udb_oring);

	mtx_init(&sc->sc_mtx, "xhci_dbc_sc_mtx", NULL, MTX_SPIN);
	mtx_init(&sc->udb_ering.mtx, "xhci_dbc_ering_mtx", NULL, MTX_SPIN);
	mtx_init(&sc->udb_iring.mtx, "xhci_dbc_iring_mtx", NULL, MTX_SPIN);
	mtx_init(&sc->udb_oring.mtx, "xhci_dbc_oring_mtx", NULL, MTX_SPIN);

	/* Reset the pointers and TRBs. */
	/* XXX: This should be revisited to support suspend/resume. */
	xhci_debug_ring_init(sc, &sc->udb_ering, 0, XHCI_DCDB_INVAL);
	xhci_debug_ring_init(sc, &sc->udb_iring, 1, XHCI_DCDB_IN);
	xhci_debug_ring_init(sc, &sc->udb_oring, 1, XHCI_DCDB_OUT);

#endif	/* _KERNEL */
	return (true);
}

static void
xhci_debug_ring_init(struct xhci_debug_softc *sc,
    struct xhci_debug_ring *ring, int producer, uint32_t doorbell)
{
	memset(ring->trb, 0, DC_TRB_RING_BYTES);
	memset(ring->work.buf, 0, DC_WORK_RING_BYTES);

	ring->enq = 0;
	ring->deq = 0;
	ring->cyc = 1;
	ring->doorbell = doorbell;
	ring->work.enq = 0;
	ring->work.deq = 0;

	/* Place a link TRB at the tail if producer == 1. */
	if (producer) {
		/* Fields for link TRBs (section 6.4.4.1) */

		ring->trb[DC_TRB_RING_LAST] = (struct xhci_trb){
		    .qwTrb0 = htole64(ring->paddr),
		    .dwTrb3 = htole32(
			XHCI_TRB_3_TYPE_SET(XHCI_TRB_TYPE_LINK) |
			XHCI_TRB_3_TC_BIT
		    )
		};
	}
}

#define	DC_STRDESC_STRING0	0
#define	DC_STRDESC_MANUFACTURER	1
#define	DC_STRDESC_PRODUCT	2
#define	DC_STRDESC_SERIAL	3
#define USB_DESC_TYPE_STRING	3
static struct usb_strdesc {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t chars[62];	/* UTF-16LE */
} strdesc[] = {
	[DC_STRDESC_STRING0] = {
		.bLength = 2 + 4,
		.bDescriptorType = USB_DESC_TYPE_STRING,
		.chars = { 9, 0, 4, 0 },	/* 0x0409 = English */
	},
	[DC_STRDESC_MANUFACTURER] = {
		.bLength = 2,
		.bDescriptorType = USB_DESC_TYPE_STRING,
	},
	[DC_STRDESC_PRODUCT] = {
		.bLength = 2,
		.bDescriptorType = USB_DESC_TYPE_STRING,
	},
	[DC_STRDESC_SERIAL] = {
		.bLength = 2,
		.bDescriptorType = USB_DESC_TYPE_STRING,
	}
};
static void
atou16cpy(struct usb_strdesc *desc, const char *str)
{
	int i;

	for (i = 0; i < strlen(str) && i * 2 < sizeof(desc->chars) - 1; i++) {
		desc->chars[i * 2] = str[i];
		desc->chars[i * 2 + 1] = '\0';
	}
	desc->bLength += i * 2;
	desc->chars[sizeof(desc->chars) - 1] = '\0';
}

/* Initialize the DbC info with USB string descriptor addresses. */
static void
xhci_debug_init_strings(struct xhci_debug_softc *sc, uint32_t info[16])
{
	const char *serial;
	char hostname[256];
	char *p;

	/*
	 * The product string will be "DC_STRING_PRODUCT (hostname)".
	 */
	p = hostname;
	strlcpy(hostname, DC_STRING_PRODUCT, sizeof(hostname));
	if (sc->sc_hostname[0] != '\0' &&
	    strlen(sc->sc_hostname)
		< sizeof(hostname) - sizeof(DC_STRING_PRODUCT) - 4) {
		p = hostname + strlen(hostname);
		*p++ = ' ';
		*p++ = '(';
		strlcpy(p, sc->sc_hostname, sizeof(hostname) - (p - hostname));
		p = hostname + strlen(hostname);
		*p++ = ')';
		*p++ = '\0';
	}
	serial = (sc->sc_serial[0] != '\0')
	    ? sc->sc_serial : DC_STRING_SERIAL;
	atou16cpy(&strdesc[DC_STRDESC_MANUFACTURER], DC_STRING_MANUFACTURER);
	atou16cpy(&strdesc[DC_STRDESC_PRODUCT], hostname);
	atou16cpy(&strdesc[DC_STRDESC_SERIAL], serial);

	/* udb_str is 1 page buffer longer than sizeof(strdesc) */
	memcpy(sc->udb_str, strdesc, sizeof(strdesc));

#define	STRADDR0(index) \
	((uint64_t)sc->udb_str_paddr + index * sizeof(struct usb_strdesc))
#define	STRADDR_LO(index) \
	((uint32_t)(STRADDR0(index) & 0xffffffff))
#define	STRADDR_HI(index) \
	((uint32_t)((STRADDR0(index) >> 32) & 0xffffffff))

	info[XHCI_DCDBCIC_STR0DESC_LO] = STRADDR_LO(DC_STRDESC_STRING0);
	info[XHCI_DCDBCIC_STR0DESC_HI] = STRADDR_HI(DC_STRDESC_STRING0);
	info[XHCI_DCDBCIC_MANUDESC_LO] = STRADDR_LO(DC_STRDESC_MANUFACTURER);
	info[XHCI_DCDBCIC_MANUDESC_HI] = STRADDR_HI(DC_STRDESC_MANUFACTURER);
	info[XHCI_DCDBCIC_PRODDESC_LO] = STRADDR_LO(DC_STRDESC_PRODUCT);
	info[XHCI_DCDBCIC_PRODDESC_HI] = STRADDR_HI(DC_STRDESC_PRODUCT);
	info[XHCI_DCDBCIC_SERIALDESC_LO] = STRADDR_LO(DC_STRDESC_SERIAL);
	info[XHCI_DCDBCIC_SERIALDESC_HI] = STRADDR_HI(DC_STRDESC_SERIAL);
	info[XHCI_DCDBCIC_DESCLEN] =
	    XHCI_DCDBCIC_STR0DESC_LEN_SET(strdesc[DC_STRDESC_STRING0].bLength) |
	    XHCI_DCDBCIC_MANUDESC_LEN_SET(strdesc[DC_STRDESC_MANUFACTURER].bLength) |
	    XHCI_DCDBCIC_PRODDESC_LEN_SET(strdesc[DC_STRDESC_PRODUCT].bLength) |
	    XHCI_DCDBCIC_SERIALDESC_LEN_SET(strdesc[DC_STRDESC_SERIAL].bLength);

#undef	STRADDR0
#undef	STRADDR_LO
#undef	STRADDR_HI
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

static uint64_t
xhci_debug_bulk_transfer0(struct xhci_debug_softc *sc,
    struct xhci_debug_ring *ring)
{
	uint64_t len;
	uint32_t temp;

	xhci_debug_event_dequeue(sc);

	len = 0;
	if (mtx_trylock_spin(&sc->udb_ering.mtx) == 0)
		goto end0;
	if (mtx_trylock_spin(&ring->mtx) == 0)
		goto end1;
	if (XHCI_DC_RING_FULL(ring)) /* TRB queue is full */
		goto end2;
	/*
	 * DRC bit is asserted when it exits the CONFIGURED state.
	 * Clear the bit because DCDB is disabled until clearing the DRC bit.
	 * (sec 7.6.8.4)
	 * The timeout is ignored because this is a best-effort attempt.
	 */
	temp = _XREAD4(sc, dbc, XHCI_DCCTRL);
	if ((le32toh(temp) & XHCI_DCCTRL_DRC) != 0)
		xhci_debug_reset_regbit(sc, XHCI_DCCTRL, XHCI_DCCTRL_DRC, 1);
	/*
	 * Updating the state in bulk_transfer() is critical in loader.
	 * Do not remove this.
	 */
	xhci_debug_update_state(sc);

	len = (XHCI_DC_RING_IN(ring))
	    ? trb_rx_enqueue_locked(sc, ring)
	    : trb_tx_enqueue_locked(sc, ring);
end2:
	mtx_unlock_spin(&ring->mtx);
end1:
	mtx_unlock_spin(&sc->udb_ering.mtx);
end0:
	return (len);
}

static void
xhci_debug_show_ring(struct xhci_debug_ring *ring, int level)
{
	if (dbc_debug < level)
		return;
#ifdef _KERNEL
	struct sbuf *sb;
#else
	char *sb;
#endif
	sb = SBUF_NEW_AUTO();

	DUMP_RING(ring, true);

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

#if 0
static bool
xhci_debug_event_ready(struct xhci_debug_softc *sc)
{
	uint32_t temp;

	/* Seems inaccurate */
	temp = _XREAD4(sc, dbc, XHCI_DCST);
	return ((le32toh(temp) & XHCI_DCST_ER) == XHCI_DCST_ER);
}
#endif

static uint32_t
xhci_debug_ring_boundary(struct xhci_debug_ring *ring, bool update_enq)
{
	struct xhci_trb *trb = &ring->trb[ring->deq];
	uint32_t dwTrb3;
	uint32_t cyc;
	uint32_t deq;

	cyc = ring->cyc;
	deq = ring->deq;
	rmb();
	dwTrb3 = le32toh(trb->dwTrb3);

	while ((dwTrb3 & XHCI_TRB_3_CYCLE_BIT) == cyc) {
		cyc = (deq == DC_TRB_RING_LAST)
		    ? cyc ^ 1
		    : cyc;
		deq = (deq + 1) & DC_TRB_RING_OFFSET_MASK;
		trb = &ring->trb[deq];
		rmb();
		dwTrb3 = le32toh(trb->dwTrb3);
	}
	if (update_enq)
		ring->enq = deq;

	return (deq);
}

static void
xhci_debug_transfer_event_handler(struct xhci_debug_softc *sc,
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

		rmb();
#if 0
		dwEpCtx0 = XHCI_EPCTX_0_EPSTATE_GET(
		    le32toh(sc->udb_ctx->ep_out.ctx.dwEpCtx0));
#endif
		if (error == XHCI_TRB_ERROR_STALL) {
			/* Halted state (sec 7.6.4.3) */
			/* Update the deq pointer using ctx. */

			rmb();
			qwEpCtx2 = (ring == &sc->udb_oring)
			    ? le64toh(sc->udb_ctx->
				ep_out.ctx.qwEpCtx2) &
				XHCI_EPCTX_2_TR_DQ_PTR_MASK
			    : le64toh(sc->udb_ctx->
				ep_in.ctx.qwEpCtx2) &
				XHCI_EPCTX_2_TR_DQ_PTR_MASK;

			ring->deq = qwEpCtx2 - (uint64_t)ring->paddr;
			ring->deq /= sizeof(struct xhci_trb);
			ring->deq &= DC_TRB_RING_OFFSET_MASK;
			/*
			 * XXXHRS: Rollback the enqueued TRB.
			 *
			 * I am unsure of whether this is correct or not, but
			 * the enqueued old TRB will not be processed or
			 * generate another event TRB with
			 * XHCI_TRB_ERROR_SUCCESS after this event,
			 * while the internal dequeue pointer seems to remain.
			 * Enqueuing another TRB into the same address
			 * and ringing the doorbell seems to work.
			 */
			ring->enq = ring->deq;
			deq_mark = "$";
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
			ring->deq &= DC_TRB_RING_OFFSET_MASK;
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
			ring->deq &= DC_TRB_RING_OFFSET_MASK;

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
			    1 < (ring->deq + DC_TRB_RING_LEN) - deq_old) {
				deq_on_er = ring->deq;
				ring->deq = deq_old + 1;
				ring->deq &= DC_TRB_RING_OFFSET_MASK;
			} else if (deq_old < ring->deq && 
			    1 < ring->deq - deq_old) {
				deq_on_er = ring->deq;
				ring->deq = deq_old + 1;
				ring->deq &= DC_TRB_RING_OFFSET_MASK;
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
			if (!XHCI_DC_RING_EMPTY(ring))
				ring->deq = (ring->deq + 1) &
				    DC_TRB_RING_OFFSET_MASK;

			if (trb != NULL) {
				len = XHCI_TRB_2_BYTES_GET(le32toh(trb->dwTrb2))
				    - XHCI_TRB_2_BYTES_GET(dwTrb2);

				/* Advance the enq of work q */
				flush_range(&ring->work.buf[ring->work.enq],
				    len);
				ring->work.enq = (ring->work.enq + len)
				    & DC_WORK_RING_OFFSET_MASK;
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
		xhci_debug_show_ring(ring, 1);
		DEBUG_PRINTF(3, "addr: paddr=%lu, qwTrb0=%lu, "
		    "masked addr=%lu, "
		    "calculated index=%lu\n",
		    (uint64_t)ring->paddr,
		    qwTrb0,
		    (qwTrb0 & (uint64_t)DC_TRB_RING_MASK) >> 4,
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
	sc->sc_dequeue_count++;
	erdp_update = false;

	rmb();
	qwTrb0 = le64toh(ev->qwTrb0);
	dwTrb2 = le32toh(ev->dwTrb2);
	dwTrb3 = le32toh(ev->dwTrb3);
	while ((dwTrb3 & XHCI_TRB_3_CYCLE_BIT) == er->cyc) {
		switch (XHCI_TRB_3_TYPE_GET(dwTrb3)) {
		case XHCI_TRB_EVENT_TRANSFER:
			xhci_debug_transfer_event_handler(sc, qwTrb0, dwTrb2,
			    dwTrb3);
			break;
		case XHCI_TRB_EVENT_PORT_STS_CHANGE:
			temp = _XREAD4(sc, dbc, XHCI_DCPORTSC);
			if (XHCI_DCPORTSC_CSC_GET(le32toh(temp))) {
				device_printf(sc->sc_dev,
				    "DbC cable status changed%s\n",
				    (XHCI_DCPORTSC_CCS_GET(le32toh(temp)))
					? ""
					: ": unplugged");
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
		if (er->deq == DC_TRB_RING_LAST)
			er->cyc ^= 1;
		er->deq = (er->deq + 1) & DC_TRB_RING_OFFSET_MASK;
		erdp_update = true;

		ev = &er->trb[er->deq];
		rmb();
		qwTrb0 = le64toh(ev->qwTrb0);
		dwTrb2 = le32toh(ev->dwTrb2);
		dwTrb3 = le32toh(ev->dwTrb3);
	}
	if (erdp_update) {
		erdp = _XREAD44LH(sc, dbc, XHCI_DCERDP);
		erdp &= ~DC_TRB_RING_MASK;
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
	if (!XHCI_DC_RING_EMPTY(ring))
		return (0);
	/* Skip if there are received data */
	if (!DC_WORK_RING_EMPTY(ring))
		return (0);

	addr = work->paddr + work->enq;
	len = (work->enq < work->deq) ? work->deq : DC_WORK_RING_LEN;
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
	if (DC_WORK_RING_EMPTY(ring))
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
		    DC_WORK_RING_LEN - work->deq);

		work->deq += len0;
		work->deq &= DC_WORK_RING_OFFSET_MASK;

		if (work->deq == 0 &&
		    !DC_WORK_RING_EMPTY(ring) &&
		    !XHCI_DC_RING_FULL(ring)) {
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

	if (ring->enq == DC_TRB_RING_LAST) {
		/*
		 * If it is the last entry, whose TRB type is LINK,
		 * cyc is set and ring->enq is advanced.
		 * The next enq should always be zero.
		 */
		ring->trb[ring->enq].dwTrb3 &= ~htole32(XHCI_TRB_3_CYCLE_BIT);
		ring->trb[ring->enq].dwTrb3 |= htole32(ring->cyc);
		flush_range(&ring->trb[ring->enq], sizeof(ring->trb[0]));

		/* Must be zero */
		ring->enq = (ring->enq + 1) & DC_TRB_RING_OFFSET_MASK;
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

	if (XHCI_DC_RING_IN(ring))
		label = "IR";
	else if (XHCI_DC_RING_OUT(ring))
		label = "OR";
	else
		label = "ER";
	DEBUG_PRINTF(1, "%s: enqueue: %s: enq=%d, len=%lu, deq=%d\n", __func__,
	    label, ring->enq, len, ring->deq);
	memcpy(&ring->trb[ring->enq], &trb, sizeof(ring->trb[0]));
	flush_range(&ring->trb[ring->enq], sizeof(ring->trb[0]));
	ring->enq = (ring->enq + 1) & DC_TRB_RING_OFFSET_MASK;

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
	xhci_debug_show_ring(ring, 1);
	if (!DC_WORK_RING_EMPTY(ring)) {
		c = (int)work->buf[work->deq];
		work->deq = (work->deq + 1) & DC_WORK_RING_OFFSET_MASK;
		DEBUG_PRINTF(2, "%s: [%c(%02x)]\n", __func__,
		    (char)c, (char)c);
	}
	xhci_debug_show_ring(ring, 1);
	mtx_unlock_spin(&ring->mtx);
	return (c);
}

int64_t
work_enqueue(struct xhci_debug_ring *ring, const char *buf, int64_t len)
{
	struct xhci_debug_work_ring *work;
	uint32_t start, end;
	int64_t i;

	if (ring == NULL)
		return (0);
	if ((work = &ring->work) == NULL)
		return (0);

	i = 0;
	end = 0;
	mtx_lock_spin(&ring->mtx);
	if (DC_WORK_RING_SLOTS(ring) < len) {
		ring->ec[XHCI_TRB_ERROR_WORKQ_FULL]++;
		goto end;
	}
	start = work->enq;
	DEBUG_PRINTF(1, "%s: %s: len=%lu: ", __func__,
	    XHCI_DC_RING_IN(ring) ? "IR" : "OR", len);
	while (!DC_WORK_RING_FULL(ring) && i < len) {
		DEBUG_PRINTF(2, "[%c(%02x)]", buf[i], buf[i]);
		work->buf[work->enq++] = buf[i++];
		work->enq &= DC_WORK_RING_OFFSET_MASK;
	}
	end = work->enq;
	DEBUG_PRINTF(1, ": start=%u, end=%u\n", start, end);

	if (start < end)
		flush_range(&work->buf[start], end - start);
	else if (0 < i) {
		flush_range(&work->buf[start], DC_WORK_RING_LEN - start);
		flush_range(&work->buf[0], end);
	}
end:
	mtx_unlock_spin(&ring->mtx);
	return (i);
}

#ifdef _KERNEL
/* These methods are called from xhci_pci.c */
static int
xhci_debug_probe(device_t dev)
{
	struct xhci_softc *sc_xhci;
	uint32_t sc_dbc_off;

	if (dbc_enable == false)
		return (ENXIO);
	if ((sc_xhci = device_get_ivars(dev))== NULL) {
		device_printf(dev, "no xhci\n");
		return (ENXIO);
	}
	sc_dbc_off = xhci_debug_get_xecp(&(struct xhci_debug_softc){
	    .sc_xhci = sc_xhci
	});

	if (sc_dbc_off != 0) {
		device_set_desc(dev, "USB xHCI DbC Console");
		return (BUS_PROBE_DEFAULT);
	} else
		return (ENXIO);
}
static int
xhci_debug_attach(device_t dev)
{
	struct xhci_softc *sc_xhci = device_get_ivars(dev);
	struct xhci_debug_softc *sc;

	sc = xhci_debug_alloc_softc(dev);
	if (sc == NULL)
		return (ENXIO);
	if (sc_xhci->sc_udbc != NULL) {
		device_printf(dev, "DbC is already allocated\n");
		return (ENXIO);
	}
	device_set_softc(dev, sc);
	sc_xhci->sc_udbc = sc;

	/* sc->sc_dbc_off must be initialized in loader. */
	/* XXX: multiple DbC xECP must be supported */
	sc_xhci->sc_dbc_off = sc->sc_dbc_off;
	device_printf(dev, "DbC xECP found at 0x%x\n", sc->sc_dbc_off);

	return (udbcons_attach(dev));
}
static int
xhci_debug_detach(device_t dev)
{
	struct xhci_softc *sc_xhci = device_get_ivars(dev);
#if 0
	struct xhci_debug_softc *sc;
#endif

	if (sc_xhci == NULL)
		return (0);
	sc_xhci->sc_udbc = NULL;
	sc_xhci->sc_dbc_off = 0;

#if 0
	sc = device_get_softc(dev);
	/* do nothing */
#endif

	return (0);
}
static device_probe_t xhci_debug_probe;
static device_attach_t xhci_debug_attach;
static device_detach_t xhci_debug_detach;

static device_method_t xhci_debug_methods[] = {
	DEVMETHOD(device_probe, xhci_debug_probe),
	DEVMETHOD(device_attach, xhci_debug_attach),
	DEVMETHOD(device_detach, xhci_debug_detach),
	DEVMETHOD_END
};
static driver_t udbcons_driver = {
	.name = "udbcons",
	.methods = xhci_debug_methods,
	.size = sizeof(struct xhci_debug_softc)
};
DRIVER_MODULE(udbcons, xhci, udbcons_driver, NULL, NULL);

/* udbcons is the device node for USB DbC. */

#define	UDBCONS_NAME	"udbcons"

struct udbcons_priv;
static int udbcons_ischar(struct udbcons_priv *);
typedef void udbcons_putc_t(struct udbcons_priv *, int);
typedef int udbcons_getc_t(struct udbcons_priv *);
typedef bool udbcons_busy_t(struct udbcons_priv *);
typedef uint32_t udbcons_slots_t(struct udbcons_priv *);
struct udbcons_ops {
	udbcons_putc_t	*putc;
	udbcons_getc_t	*getc;
	udbcons_busy_t	*busy;
	udbcons_slots_t	*slots;
};
struct udbcons_priv {
	struct udbcons_priv		*next;
	device_t			dev;
	const struct udbcons_ops	*ops;
	struct callout			callout;
	struct tty			*tp;
	bool				opened;
	int				alt_break_state;
};

static struct udbcons_priv *udbcons0;		/* linked-list */
static struct udbcons_priv *udbcons_curr;	/* kernel console */

#if 0
/*
 * The GDB initialization is performed before xhci probing.
 * udb_gdb global variable is used for now.
 */
static int
sysctl_sc_flags_gdb(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc;
	unsigned int val;
	int error;

	val = 0;
	if (udbcons_curr == NULL || udbcons_curr.xhci == NULL)
		goto error;
	if ((sc = udbcons_curr->xhci->sc_udbc) == NULL)
		goto error;
	val = (sc->sc_flags & XHCI_DEBUG_FLAGS_GDB) == XHCI_DEBUG_FLAGS_GDB;

error:
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 ||
	    req->newptr == NULL ||
	    val == ((sc->sc_flags & XHCI_DEBUG_FLAGS_GDB)
		== XHCI_DEBUG_FLAGS_GDB))
		return (error);

	if (sc == NULL)
		return (0);

	if (val == 0)
		sc->sc_flags &= ~XHCI_DEBUG_FLAGS_GDB;
	else
		sc->sc_flags |= XHCI_DEBUG_FLAGS_GDB;

	return (0);
}
SYSCTL_PROC(_hw_usb_xhci_dbc, OID_AUTO, gdb,
    CTLTYPE_INT | CTLFLAG_RWTUN | CTLFLAG_MPSAFE,
    NULL, 0, sysctl_sc_flags_gdb, "I", "Set to enable DbC GDB support"); 
#endif

static int
sysctl_udbcons_curr(SYSCTL_HANDLER_ARGS)
{
	struct sbuf *sb;
	int error;

	sb = SBUF_NEW_AUTO();
	if (udbcons_curr != NULL)
		sbuf_printf(sb, "%s%d",
		    device_get_name(udbcons_curr->dev),
		    device_get_unit(udbcons_curr->dev));
	else
		sbuf_printf(sb, "none");
	if (SBUF_FINISH(sb) != 0)
		return (-1);
	error = sysctl_handle_opaque(oidp, SBUF_DATA(sb), SBUF_LEN(sb), req);
	SBUF_DELETE(sb);

	return (error);
}
SYSCTL_PROC(_hw_usb_xhci_dbc, OID_AUTO, udbcons,
    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
    NULL, 0, sysctl_udbcons_curr, "A", "udbcons for console"); 

static int
sysctl_sc_polling_count(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc = arg1;
	uint64_t val;
	int error;

	val = sc->sc_polling_count;
	error = sysctl_handle_64(oidp, &val, 0, req);

	return (error);
}

static int
sysctl_sc_dequeue_count(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc = arg1;
	uint64_t val;
	int error;

	val = sc->sc_dequeue_count;
	error = sysctl_handle_64(oidp, &val, 0, req);

	return (error);
}

static int
sysctl_sc_polling_time(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc = arg1;
	uint32_t val;
	int error;

	val = sc->sc_polling_time;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	if (sc == NULL)
		return (0);
	sc->sc_polling_time = val;

	return (0);
}

#define	SYSCTL_FUNC_DEFINE_SHOW_RING(ring)		\
	static int					\
	sysctl_##ring(SYSCTL_HANDLER_ARGS)		\
	{						\
		struct xhci_debug_softc *sc = arg1;	\
		struct sbuf *sb;			\
		int error;				\
							\
		sb = SBUF_NEW_AUTO();			\
		DUMP_RING(&sc->udb_##ring, false);	\
		if (SBUF_FINISH(sb) != 0)		\
			return (-1);			\
		error = sysctl_handle_opaque(oidp,	\
		    SBUF_DATA(sb), SBUF_LEN(sb), req);	\
		SBUF_DELETE(sb);			\
							\
		return (error);				\
	}
SYSCTL_FUNC_DEFINE_SHOW_RING(ering);
SYSCTL_FUNC_DEFINE_SHOW_RING(oring);
SYSCTL_FUNC_DEFINE_SHOW_RING(iring);

#define	SYSCTL_FUNC_DEFINE_SC_UINT32_STRING(mib)	\
	static int					\
	sysctl_sc_##mib(SYSCTL_HANDLER_ARGS)		\
	{						\
		struct xhci_debug_softc *sc = arg1;	\
		char buf[64];				\
		int error;				\
							\
		snprintf(buf, sizeof(buf), "0x%x", sc->sc_##mib);	\
		buf[sizeof(buf) - 1] = '\0';			\
		error = sysctl_handle_opaque(oidp, buf, sizeof(buf), req); \
							\
		return (error);				\
	}

SYSCTL_FUNC_DEFINE_SC_UINT32_STRING(dbc_off);

static int
sysctl_sc_pci_rid(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc = arg1;
	char buf[64];
	int error;

	snprintf(buf, sizeof(buf), "0x%04x(%d:%d:%d)",
	    sc->sc_pci_rid,
	    PCI_RID2BUS(sc->sc_pci_rid),
	    PCI_RID2SLOT(sc->sc_pci_rid),
	    PCI_RID2FUNC(sc->sc_pci_rid));
	buf[sizeof(buf) - 1] = '\0';
	error = sysctl_handle_opaque(oidp, buf, sizeof(buf), req);

	return (error);
}

static int
sysctl_sc_softc(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc = arg1;
	char buf[64];
	int error;
	snprintf(buf, sizeof(buf), "%p", sc);
	buf[sizeof(buf) - 1] = '\0';
	error = sysctl_handle_opaque(oidp, buf, sizeof(buf), req);

	return (error);
}

static int
sysctl_sc_state(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc = arg1;
	char buf[64];
	int error;

	snprintf(buf, sizeof(buf), "%s(%02x)", strstate[sc->sc_state],
	    sc->sc_state);
	buf[sizeof(buf) - 1] = '\0';
	error = sysctl_handle_opaque(oidp, buf, sizeof(buf), req);

	return (error);
}

static int
sysctl_dcst_portnum(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc = arg1;
	uint32_t temp;
	int error, val;

	temp = _XREAD4(sc, dbc, XHCI_DCST);
	val = XHCI_DCST_PORT_GET(le32toh(temp));
	error = sysctl_handle_int(oidp, &val, 0, req);

	return (error);
}

static int
sysctl_dcst_er(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc = arg1;
	uint32_t temp;
	int error, val;

	temp = _XREAD4(sc, dbc, XHCI_DCST);
	val = XHCI_DCST_ER_GET(le32toh(temp));
	error = sysctl_handle_int(oidp, &val, 0, req);

	return (error);
}

static int
sysctl_dcst_sbr(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc = arg1;
	uint32_t temp;
	int error, val;

	temp = _XREAD4(sc, dbc, XHCI_DCST);
	val = XHCI_DCST_SBR_GET(le32toh(temp));
	error = sysctl_handle_int(oidp, &val, 0, req);

	return (error);
	
}

static int
sysctl_ddi1(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc = arg1;
	uint32_t temp;
	uint32_t val;
	char buf[64];
	int error;

	temp = _XREAD4(sc, dbc, XHCI_DCDDI1);
	val = le32toh(temp);

	snprintf(buf, sizeof(buf), "0x%x", val);
	buf[sizeof(buf) - 1] = '\0';
	error = sysctl_handle_opaque(oidp, buf, sizeof(buf), req);

	return (error);
}

static int
sysctl_ddi2(SYSCTL_HANDLER_ARGS)
{
	struct xhci_debug_softc *sc = arg1;
	uint32_t temp;
	uint32_t val;
	char buf[64];
	int error;

	temp = _XREAD4(sc, dbc, XHCI_DCDDI2);
	val = le32toh(temp);

	snprintf(buf, sizeof(buf), "0x%x", val);
	buf[sizeof(buf) - 1] = '\0';
	error = sysctl_handle_opaque(oidp, buf, sizeof(buf), req);

	return (error);
}

#define	SYSCTL_FUNC_DEFINE_UINT64_STRING(mib, offset)	\
	static int					\
	sysctl_##mib(SYSCTL_HANDLER_ARGS)		\
	{						\
		struct xhci_debug_softc *sc = arg1;	\
		uint64_t temp;				\
		uint64_t val;				\
		char buf[64];				\
		int error;				\
							\
		temp = _XREAD44LH(sc, dbc, offset);	\
		val = le64toh(temp);			\
		snprintf(buf, sizeof(buf), "0x%lx", val);	\
		buf[sizeof(buf) - 1] = '\0';			\
		error = sysctl_handle_opaque(oidp, buf, sizeof(buf), req); \
							\
		return (error);				\
	}

SYSCTL_FUNC_DEFINE_UINT64_STRING(erstba, XHCI_DCERSTBA);
SYSCTL_FUNC_DEFINE_UINT64_STRING(erdp, XHCI_DCERDP);
SYSCTL_FUNC_DEFINE_UINT64_STRING(cp, XHCI_DCCP);

#define	SYSCTL_FUNC_DEFINE_PADDR_STRING(mib)		\
	static int					\
	sysctl_##mib(SYSCTL_HANDLER_ARGS)		\
	{						\
		struct xhci_debug_softc *sc = arg1;	\
		char buf[64];				\
		int error;				\
							\
		snprintf(buf, sizeof(buf), "addr=%p, paddr=0x%lx, len=%lu", \
		    sc->udb_##mib, sc->udb_##mib##_paddr,	\
		    sc->udb_##mib##_len);		\
		buf[sizeof(buf) - 1] = '\0';		\
		error = sysctl_handle_opaque(oidp, buf, sizeof(buf), req); \
							\
		return (error);				\
	}
SYSCTL_FUNC_DEFINE_PADDR_STRING(ctx);
SYSCTL_FUNC_DEFINE_PADDR_STRING(erst);
SYSCTL_FUNC_DEFINE_PADDR_STRING(str);

/*
 * udbcons_getc() is called periodically.  It calls ischar() that
 * calls bulk_transfer() and dequeue().
 */
static int
udbcons_getc(struct udbcons_priv *cons)
{
	struct xhci_debug_softc *sc_udbc;
	struct xhci_debug_ring *iring;

	if (cons == NULL)
		 return (-1);
 	if ((sc_udbc = device_get_softc(cons->dev)) == NULL)
		 return (-1);
	if (sc_udbc->sc_dbc_off == 0)
		return (-1);

	iring = &sc_udbc->udb_iring;

	/* Use udbcons_ischar() to receive data */
	return (udbcons_ischar(cons) ? work_dequeue(iring) : -1);
}
static int
udbcons_ischar(struct udbcons_priv *cons)
{
	struct xhci_debug_softc *sc_udbc;
	struct xhci_debug_ring *iring;

	if (cons == NULL)
		 return (0);
 	if ((sc_udbc = device_get_softc(cons->dev)) == NULL)
		 return (0);
	if (sc_udbc->sc_dbc_off == 0)
		return (0);

	iring = &sc_udbc->udb_iring;
	if (!DC_WORK_RING_EMPTY(iring))
		return (1);

	xhci_debug_bulk_transfer(sc_udbc);
	xhci_debug_event_dequeue(sc_udbc);

	return (!DC_WORK_RING_EMPTY(iring));
}
static void
udbcons_putc(struct udbcons_priv *cons, int ch)
{
	struct xhci_debug_softc *sc_udbc;
	struct xhci_debug_ring *oring;
	char c;

	if (cons == NULL)
		 return;
 	if ((sc_udbc = device_get_softc(cons->dev)) == NULL)
		 return;
	if (sc_udbc->sc_dbc_off == 0)
		return;

	oring = &sc_udbc->udb_oring;
	c = (char)(ch & 0xff);
	if (work_enqueue(oring, &c, sizeof(c)) != sizeof(c)) {
		oring->ec[XHCI_TRB_ERROR_WORKQ_FULL]++;
		return;
	}
	/* bulk_transfer() is called periodically in getc(). */
	xhci_debug_event_dequeue(sc_udbc);
}
static bool
udbcons_busy(struct udbcons_priv *cons)
{
	struct xhci_debug_softc *sc_udbc;

	if (cons == NULL)
		 return (true);
 	if ((sc_udbc = device_get_softc(cons->dev)) == NULL)
		 return (true);
	if (sc_udbc->sc_dbc_off == 0)
		return (true);

	return (DC_WORK_RING_FULL(&sc_udbc->udb_oring) != 0 ||
	    sc_udbc->sc_state != XHCI_DCPORT_ST_CONFIGURED);
}
static uint32_t
udbcons_slots(struct udbcons_priv *cons)
{
	struct xhci_debug_softc *sc_udbc;

	if (cons == NULL)
		 return (0);
 	if ((sc_udbc = device_get_softc(cons->dev)) == NULL)
		 return (0);
	if (sc_udbc->sc_dbc_off == 0)
		return (0);

	return (DC_WORK_RING_SLOTS(&sc_udbc->udb_oring));
}

static struct udbcons_ops udbcons_ops = {
	.putc = udbcons_putc,
	.getc = udbcons_getc,
	.busy = udbcons_busy,
	.slots = udbcons_slots,
};

static tsw_outwakeup_t udbcons_tty_outwakeup;
static tsw_busy_t udbcons_tty_busy;

static struct ttydevsw udbcons_ttydevsw = {
	.tsw_flags	= TF_NOPREFIX,
	.tsw_outwakeup	= udbcons_tty_outwakeup,
	.tsw_busy	= udbcons_tty_busy
};

static void xhci_debug_timeout(void *);

int
udbcons_attach(device_t dev)
{
	struct udbcons_priv *cons;
	struct xhci_debug_softc *sc;
	struct sysctl_oid_list *child;
	struct sysctl_oid *ring;
	struct sysctl_ctx_list *ctx;

	sc = device_get_softc(dev);
	if (sc == NULL)
		return (0);
	if (sc->sc_dbc_off == 0)
		return (0);

	/* If hw.usb.xhci.dbc.pci_rid is specified, use it */
	if (dbc_pci_rid != 0 && dbc_pci_rid != sc->sc_pci_rid) {
		device_printf(sc->sc_dev,
		    "console was not activated because RID mismatch: "
		    "expected=%u, found=%u\n", dbc_pci_rid,
		    sc->sc_pci_rid);
		return (0);
	}

	cons = malloc(sizeof(*cons), M_DEVBUF, M_WAITOK);
	*cons = (struct udbcons_priv){
	    .dev = dev,
	    .ops = &udbcons_ops
	};
	cons->tp = tty_alloc(&udbcons_ttydevsw, cons);
	sc->sc_cons = cons;	/* back pointer */

	device_printf(sc->sc_dev, "Creating /dev/" UDBCONS_NAME "%d\n",
	    device_get_unit(dev));
	tty_makedev(cons->tp, NULL, UDBCONS_NAME "%d", device_get_unit(dev));
	tty_init_console(cons->tp, dbc_baud);

	ctx = device_get_sysctl_ctx(dev);
	child = SYSCTL_CHILDREN(device_get_sysctl_tree(dev));

        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "state",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_sc_state, "A", "DbC State");
        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "portnum",
            CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_dcst_portnum, "I", "Debug Port Number");
        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "softc",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_sc_softc, "A", "softc");
        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "dbc_off",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_sc_dbc_off, "A", "DbC xECP");
        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "pci_rid",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_sc_pci_rid, "A", "DbC PCI RID");
        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "polling_count",
            CTLTYPE_U64 | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_sc_polling_count, "LU", "Polling Count");
        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "polling_time",
            CTLTYPE_U32 | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_sc_polling_time, "I", "Polling Interval");
        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "dequeue_count",
            CTLTYPE_U64 | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_sc_dequeue_count, "LU", "Dequeue Count");

	ring = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "ering",
	    CTLFLAG_RD | CTLFLAG_MPSAFE, 0, "Event Ring");
        SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(ring), OID_AUTO, "info",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_ering, "A", "Queue Info");

	ring = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "iring",
	    CTLFLAG_RD | CTLFLAG_MPSAFE, 0, "Input Ring");
        SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(ring), OID_AUTO, "info",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_iring, "A", "Queue Info");

	ring = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "oring",
	    CTLFLAG_RD | CTLFLAG_MPSAFE, 0, "Output Ring");
        SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(ring), OID_AUTO, "info",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_oring, "A", "Queue Info");

        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "dcst_er",
            CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_dcst_er, "I", "DCST Event Ring Not Empty");
        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "dcst_sbr",
            CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_dcst_sbr, "I", "DCST System Bus Reset");

        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "ddi1",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_ddi1, "A", "DCDDI1");
        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "ddi2",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_ddi2, "A", "DCDDI2");
        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "erstba",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_erstba, "A", "DCERSTBA");
        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "erdp",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_erdp, "A", "DCERDP");
        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "cp",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_cp, "A", "DCCP");

        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "erst",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_erst, "A", "ERST");
        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "ctx",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_ctx, "A", "CTX");
        SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "str",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            sysctl_str, "A", "STR");

	callout_init(&cons->callout, 1);
	callout_reset(&cons->callout, sc->sc_polling_time,
	    xhci_debug_timeout, cons->tp);

	/* Add new entry to udbcons0 */
	if (udbcons0 != NULL)
		cons->next = udbcons0;
	udbcons0 = cons;

	/* XXX: the first one is used at this moment */
	if (udbcons_curr == NULL)
		udbcons_curr = cons;

	return (0);
}

/* udbcons_cn* are methods for the UDBCONS console instance */

static cn_probe_t udbcons_cnprobe;
static cn_init_t udbcons_cninit;
static cn_term_t udbcons_cnterm;
static cn_getc_t udbcons_cngetc;
static cn_putc_t udbcons_cnputc;
static cn_grab_t udbcons_cngrab;
static cn_ungrab_t udbcons_cnungrab;

const struct consdev_ops udbcons_cnops = {
	.cn_probe	= udbcons_cnprobe,
	.cn_init	= udbcons_cninit,
	.cn_term	= udbcons_cnterm,
	.cn_getc	= udbcons_cngetc,
	.cn_putc	= udbcons_cnputc,
	.cn_grab	= udbcons_cngrab,
	.cn_ungrab	= udbcons_cnungrab,
};

CONSOLE_DRIVER(udbcons);

#define MAX_BURST_LEN	1
static void
udbcons_tty_outwakeup(struct tty *tp)
{
	struct udbcons_priv *cons;
	u_char buf[MAX_BURST_LEN];
	int len;
	int i;

	cons = tty_softc(tp);
	for (;;) {
		/*
		 * If the remaining work queue slots are smaller than
		 * sizeof(buf), draining tty is skipped. 
		 */
		if (cons->ops->slots(cons) < sizeof(buf))
			break;
		len = ttydisc_getc(tp, buf, sizeof(buf));
		if (len == 0)
			break;
		for (i = 0; i < len; i++)
			cons->ops->putc(cons, buf[i]);
	}
}
static bool
udbcons_tty_busy(struct tty *tp)
{
	struct udbcons_priv *cons;

	cons = tty_softc(tp);

	return (cons->ops->busy(cons));
}

static void
xhci_debug_timeout(void *v)
{
	struct udbcons_priv *cons;
	struct xhci_debug_softc *sc;
	struct tty *tp;
	int polling_factor;
	int ch;

	tp = v;
	polling_factor = 1;
	cons = tty_softc(tp);
	sc = device_get_softc(cons->dev);

	if (sc == NULL)
		goto end;
	if (sc->sc_dbc_off == 0)
		goto end;

	sc->sc_polling_count++;

	if (dbc_reset > 0) {
		/* XXX: still does not work. */
		int error;

		mtx_lock_spin(&sc->udb_ering.mtx);
		mtx_lock_spin(&sc->udb_iring.mtx);
		mtx_lock_spin(&sc->udb_oring.mtx);

		dbc_reset = 0;
		xhci_debug_disable(sc);
		error = xhci_debug_init_ring(sc);
		if (error)
			goto end;
		sc->sc_init = true;
		xhci_debug_enable(sc);

		mtx_unlock_spin(&sc->udb_oring.mtx);
		mtx_unlock_spin(&sc->udb_iring.mtx);
		mtx_unlock_spin(&sc->udb_ering.mtx);
	} else {
		/*
		 * If the DC port is not ready, the state is checked with
		 * a reduced frequency.  If ready, cngetc() is called.
		 */
		if (sc->sc_state != XHCI_DCPORT_ST_CONFIGURED) {
			polling_factor = 5;

			xhci_debug_update_state(sc);
		} else {
			polling_factor = 1;

			tty_lock(tp);
			if (cons == udbcons_curr) {
				while ((ch = udbcons_cngetc(NULL)) != -1)
					ttydisc_rint(tp, ch, 0);
			} else {
				while ((ch = udbcons_getc(cons)) != -1)
					ttydisc_rint(tp, ch, 0);
			}
			ttydisc_rint_done(tp);
			tty_unlock(tp);
		}
	}
end:
	callout_reset(&cons->callout, sc->sc_polling_time * polling_factor,
	    xhci_debug_timeout, tp);
}

static void
udbcons_cnprobe(struct consdev *cp)
{

	cp->cn_pri = (boothowto & RB_SERIAL) ? CN_REMOTE : CN_NORMAL;
	strlcpy(cp->cn_name, UDBCONS_NAME, sizeof(cp->cn_name));
}

static void
udbcons_cninit(struct consdev *cp)
{
}

static void
udbcons_cnterm(struct consdev *cp)
{
}

/* empty */
static void
udbcons_cngrab(struct consdev *cp)
{
}

/* empty */
static void
udbcons_cnungrab(struct consdev *cp)
{
}

#if defined(GDB)
static bool udbcons_dbg_enabled(struct consdev *cp);
#endif

static int
udbcons_cngetc(struct consdev *cp)
{
	int ch;

	ch = udbcons_getc(udbcons_curr);
	if (ch > 0 && ch < 0xff) {
		/* when udbcons_curr == NULL, ch == -1 */
#if defined(KDB)
#if defined(GDB)
		if (udbcons_dbg_enabled(cp))
			kdb_alt_break_gdb(ch, &udbcons_curr->alt_break_state);
		else
#endif
			kdb_alt_break(ch, &udbcons_curr->alt_break_state);
#endif
		return (ch);
	}
	return (-1);
}
static void
udbcons_cnputc(struct consdev *cp, int ch)
{

	udbcons_putc(udbcons_curr, ch);
}

#if defined(GDB)
extern struct gdb_dbgport *gdb_cur;

/* GDB methods */

static gdb_probe_f	udbcons_dbg_probe;
static gdb_init_f	udbcons_dbg_init;
static gdb_term_f	udbcons_dbg_term;
static gdb_getc_f	udbcons_dbg_getc;
static gdb_putc_f	udbcons_dbg_putc;

GDB_DBGPORT(udbcons, udbcons_dbg_probe, udbcons_dbg_init,
    udbcons_dbg_term, udbcons_dbg_getc, udbcons_dbg_putc);

static bool
udbcons_dbg_enabled(struct consdev *cp __unused)
{

	return (gdb_cur == &udbcons_gdb_dbgport);
}
static int
udbcons_dbg_probe(void)
{
#if 0
	struct xhci_debug_softc *sc;

	if (udbcons_curr == NULL || udbcons_curr->xhci == NULL)
		return (-1);
	if ((sc = udbcons_curr->xhci->sc_udbc) == NULL)
		return (-1);
	if ((sc->sc_flags & XHCI_DEBUG_FLAGS_GDB) == XHCI_DEBUG_FLAGS_GDB)
		return (1);
#endif
	if (dbc_gdb != 0)
		return (1);

	return (-1);
}
static void
udbcons_dbg_term(void)
{
	/* empty */
}
static void
udbcons_dbg_init(void)
{
	/* empty */
}
static void
udbcons_dbg_putc(int ch)
{

	udbcons_putc(udbcons_curr, ch);
}
static int
udbcons_dbg_getc(void)
{

	return (udbcons_getc(udbcons_curr));
}
#endif	/* GDB */
#endif	/* _KERNEL */
