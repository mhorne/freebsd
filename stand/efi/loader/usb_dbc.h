#ifndef	_USB_DBC_H_
#define	_USB_DBC_H_

#define	PCIR_DEVVENDOR	0

/* Supported xHC PCI configurations */
#define XHC_CLASSC		0x0c0330ULL
#define XHC_VEN_INTEL		0x8086ULL

#define	XHC_DEV_COMET_LAKE_LP	0x02edULL
#define	XHC_DEV_COMET_LAKE	0x06edULL
#define	XHC_DEV_N2000		0x0f35ULL
#define	XHC_DEV_C3000		0x19d0ULL
#define	XHC_DEV_C210		0x1e31ULL
#define	XHC_DEV_E8000		0x22b5ULL
#define	XHC_DEV_PUMA		0x2bb5ULL
#define	XHC_DEV_CELERON		0x31a8ULL
#define	XHC_DEV_ICE_LAKE	0x34edULL
#define	XHC_DEV_TIGER_LAKE_H	0x43edULL
#define	XHC_DEV_N3350		0x5aa8ULL
#define	XHC_DEV_C220		0x8c31ULL
#define	XHC_DEV_NINE		0x8cb1ULL
#define	XHC_DEV_C610		0x8d31ULL
#define	XHC_DEV_EIGHT		0x9c31ULL
#define	XHC_DEV_WILDCAT_POINT	0x9cb1ULL
#define	XHC_DEV_SUNRISE_POINT	0x9d2fULL
#define	XHC_DEV_CANNON_POINT	0x9dedULL
#define	XHC_DEV_SILVER		0x31a8ULL
#define	XHC_DEV_TIGER_LAKE	0xa0edULL
#define	XHC_DEV_C230		0xa12fULL
#define	XHC_DEV_C620		0xa1afULL
#define	XHC_DEV_Z370		0xa2afULL
#define	XHC_DEV_CANNON_LAKE	0xa36dULL

#define XHC_VEN_AMD		0x1022ULL

#define	XHC_DEV_KERNCZ		0x145cULL
#define	XHC_DEV_STARSHIP	0x148cULL
#define	XHC_DEV_MATISSE		0x149cULL
#define	XHC_DEV_X399		0x43baULL
#define	XHC_DEV_X370		0x43b9ULL
#define	XHC_DEV_B350		0x43bbULL
#define	XHC_DEV_400		0x43d5ULL
#define	XHC_DEV_FCH_1		0x7812ULL
#define	XHC_DEV_FCH_2		0x7814ULL
#define	XHC_DEV_FCH_3		0x7914ULL

#define XHC_VEN_VMWARE		0x15adULL
#define	XHC_DEV_VMWARE_1	0x0778ULL
#define	XHC_DEV_VMWARE_2	0x0779ULL

#define XHC_VEN_ASMEDIA		0x1b21ULL
#define XHC_DEV_ASM1042		0x1042ULL
#define XHC_DEV_ASM1042A	0x1142ULL
#define XHC_DEV_ASM1143		0x1343ULL
#define XHC_DEV_ASM3242		0x3242ULL

/* DbC idVendor and idProduct */
#define DC_VENDOR	0x1d6b	/* Linux Foundation */
#define DC_PRODUCT	0x0011	/* Linux */ 
#define DC_PROTOCOL	0x0000	/* GNU GDB = 1 */
#define DC_REVISION	0x0010	/* 1.0 */

#define	DC_STRING_MANUFACTURER	"The FreeBSD Foundation"
#define	DC_STRING_PRODUCT	"FreeBSD USB DbC Serial"
#define	DC_STRING_SERIAL	"12345678"

#define XHCI_DCPORTSC_ACK_MASK	\
	(XHCI_DCPORTSC_CSC | XHCI_DCPORTSC_PRC | \
	 XHCI_DCPORTSC_PLC | XHCI_DCPORTSC_CEC)

/* DbC endpoint types */
enum {
	EP_BULK_OUT = 2,
	EP_BULK_IN = 6
};

#define UDB_TRB_MAX_TRANSFER (XHCI_PAGE_SIZE << 4)
#define UDB_TRB_PER_PAGE (XHCI_PAGE_SIZE / sizeof(struct xhci_trb))

/* Defines the size in bytes of TRB rings as 2^UDB_TRB_RING_ORDER * 4096 */
#define UDB_TRB_RING_ORDER 4
#define UDB_TRB_RING_CAP (UDB_TRB_PER_PAGE * (1ULL << UDB_TRB_RING_ORDER))
#define UDB_TRB_RING_BYTES (UDB_TRB_RING_CAP * sizeof(struct xhci_trb))
#define UDB_TRB_RING_MASK (UDB_TRB_RING_BYTES - 1U)

/* Defines the size in bytes of work rings as 2^WORK_RING_ORDER * 4096 */
#define	UDB_WORK_RING_ORDER 3
#define UDB_WORK_RING_CAP (XHCI_PAGE_SIZE * (1ULL << UDB_WORK_RING_ORDER))
#define	UDB_WORK_RING_BYTES UDB_WORK_RING_CAP

#if UDB_WORK_RING_CAP > UDB_TRB_MAX_TRANSFER
#error "UDB_WORK_RING_ORDER must be at most 4"
#endif

struct xhci_debug_ring {
	struct xhci_trb *trb;	/* Array of TRBs */
	uint32_t enq;		/* The offset of the enqueue ptr */
	uint32_t deq;		/* The offset of the dequeue ptr */
	uint8_t cyc;		/* Cycle state toggled on each wrap-around */
#define XHCI_DB_OUT	0x0
#define XHCI_DB_IN	0x1
#define XHCI_DB_INVAL	0xFF
	uint8_t doorbell;	/* Doorbell target */
};

struct xhci_debug_work_ring {
	uint8_t *buf;
	uint32_t enq;
	uint32_t deq;
	uint64_t paddr;
};

extern struct console udb_console;
void udb_info(void);

#endif
