#include <sys/param.h>
#include <bootstrap.h>
#include <efi.h>
#include <efilib.h>
#include <xhcireg.h>

#include "usb_xhci.h"
#include "usb_dbc.h"

#define	SFENCE(x)	__asm volatile("sfence" ::: "memory")
#define	PAUSE(x)	__asm volatile("pause" ::: "memory")

#define DMA_DESC_CAP 7
struct dma {
	UINTN pages;
	EFI_PHYSICAL_ADDRESS paddr;
	VOID *cpu_addr;
	VOID *mapping;
};

struct udb_ctx {
	uint32_t info[16];
	uint32_t ep_out[16];
	uint32_t ep_in[16];
};

struct udb_efi {
	EFI_HANDLE img_hand;
	EFI_HANDLE pci_hand;
	EFI_PCI_IO_PROTOCOL *pciio;
};
static EFI_GUID pciio_guid = EFI_PCI_IO_PROTOCOL_GUID;

/* Must be aligned with xhci_debug_ring_ctx in xhci.h */
struct xhci_debug_softc {
	int open;
	uint64_t xhci_xecp_offset;

	struct xhci_event_ring_seg *udb_erst;
	struct xhci_debug_ring udb_ering;
	struct xhci_debug_ring udb_oring;
	struct xhci_debug_ring udb_iring;
	struct xhci_debug_work_ring udb_owork;

/* private */
	struct xhci_debug_reg *udb_reg;
	struct udb_ctx *udb_ctx;

	uint32_t xhc_cf8;
	uint64_t xhc_mmio_phys;
	uint64_t xhc_mmio_size;
	void *xhc_mmio;

	struct udb_efi efi;
	struct dma dma_desc[DMA_DESC_CAP];

	int dma_allocated;
	int init;

	char *udb_str;
};
static struct xhci_debug_softc *udb_sc;

static uint32_t ind(uint32_t);
static void outd(uint32_t, uint32_t);
static uint32_t pci_read_config(uint32_t, uint32_t);
static void pci_write_config(uint32_t, uint32_t, uint32_t);
static void udb_enable(struct xhci_debug_softc *);
static void udb_disable(struct xhci_debug_softc *);
static int udb_init_ring(struct xhci_debug_softc *);

static void udb_probe(struct console *);
static int udb_init(int);
static int udb_find(struct xhci_debug_softc *);
static int udb_setup(struct xhci_debug_softc *);
static void *udb_alloc_dma(struct xhci_debug_softc *, uint64_t);
static void udb_free_dma(struct xhci_debug_softc *, void *);
static void udb_uninit_dma(struct xhci_debug_softc *);
static int udb_init_dma(struct xhci_debug_softc *);
static void udb_xhci_debug_ring_init(struct xhci_debug_softc *, struct xhci_debug_ring *,
    int, int);
static void udb_init_ep(uint32_t *, size_t, uint64_t, uint32_t, uint64_t);

static struct xhci_debug_softc *udb_open(void);
static void udb_close(struct xhci_debug_softc *);
static void udb_pop_events(struct xhci_debug_softc *);
static void udb_init_strings(struct xhci_debug_softc *, uint32_t *);
static uint64_t v2p(struct xhci_debug_softc *, void *);
static void udb_flush(struct xhci_debug_softc *);

static int udb_getc(void);
static void udb_putc(int);
static int udb_ischar(void);

static void push_trb(struct xhci_debug_ring *, uint64_t, uint64_t);
static int xhci_debug_work_ring_full(const struct xhci_debug_work_ring *);
static uint64_t xhci_debug_work_ring_size(const struct xhci_debug_work_ring *);
static int64_t push_work(struct xhci_debug_work_ring *, const char *, int64_t);

static int xhci_debug_ring_full(const struct xhci_debug_ring *);
static void flush_range(void *, uint32_t);

struct console udb_console = {
	.c_name = "udb",
	.c_desc = "USB DbC serial port",
	.c_flags = 0,
	.c_probe = udb_probe,
	.c_init = udb_init,
	.c_out = udb_putc,
	.c_in = udb_getc,
	.c_ready = udb_ischar
};

static struct {
	uint16_t vendor;
	uint16_t device;
} ids[] = {
	{ XHC_VEN_INTEL, XHC_DEV_COMET_LAKE_LP },
	{ XHC_VEN_INTEL, XHC_DEV_COMET_LAKE },
	{ XHC_VEN_INTEL, XHC_DEV_N2000 },
	{ XHC_VEN_INTEL, XHC_DEV_C3000 },
	{ XHC_VEN_INTEL, XHC_DEV_C210 },
	{ XHC_VEN_INTEL, XHC_DEV_E8000 },
	{ XHC_VEN_INTEL, XHC_DEV_PUMA },
	{ XHC_VEN_INTEL, XHC_DEV_CELERON },
	{ XHC_VEN_INTEL, XHC_DEV_ICE_LAKE },
	{ XHC_VEN_INTEL, XHC_DEV_TIGER_LAKE_H },
	{ XHC_VEN_INTEL, XHC_DEV_N3350 },
	{ XHC_VEN_INTEL, XHC_DEV_C220 },
	{ XHC_VEN_INTEL, XHC_DEV_NINE },
	{ XHC_VEN_INTEL, XHC_DEV_C610 },
	{ XHC_VEN_INTEL, XHC_DEV_EIGHT },
	{ XHC_VEN_INTEL, XHC_DEV_WILDCAT_POINT },
	{ XHC_VEN_INTEL, XHC_DEV_SUNRISE_POINT },
	{ XHC_VEN_INTEL, XHC_DEV_CANNON_POINT },
	{ XHC_VEN_INTEL, XHC_DEV_SILVER },
	{ XHC_VEN_INTEL, XHC_DEV_TIGER_LAKE },
	{ XHC_VEN_INTEL, XHC_DEV_C230 },
	{ XHC_VEN_INTEL, XHC_DEV_C620 },
	{ XHC_VEN_INTEL, XHC_DEV_Z370 },
	{ XHC_VEN_INTEL, XHC_DEV_CANNON_LAKE },

	{ XHC_VEN_AMD, XHC_DEV_KERNCZ },
	{ XHC_VEN_AMD, XHC_DEV_STARSHIP },
	{ XHC_VEN_AMD, XHC_DEV_MATISSE },
	{ XHC_VEN_AMD, XHC_DEV_X399 },
	{ XHC_VEN_AMD, XHC_DEV_X370 },
	{ XHC_VEN_AMD, XHC_DEV_B350 },
	{ XHC_VEN_AMD, XHC_DEV_400 },
	{ XHC_VEN_AMD, XHC_DEV_FCH_1 },
	{ XHC_VEN_AMD, XHC_DEV_FCH_2 },
	{ XHC_VEN_AMD, XHC_DEV_FCH_3 },

	{ XHC_VEN_VMWARE, XHC_DEV_VMWARE_1 },
	{ XHC_VEN_VMWARE, XHC_DEV_VMWARE_2 },

	{ XHC_VEN_ASMEDIA, XHC_DEV_ASM1042 },
	{ XHC_VEN_ASMEDIA, XHC_DEV_ASM1042A },
	{ XHC_VEN_ASMEDIA, XHC_DEV_ASM1143 },
	{ XHC_VEN_ASMEDIA, XHC_DEV_ASM3242 },
	{ 0, 0 }
};

void
udb_probe(struct console *cons)
{
	struct xhci_debug_softc *sc;
	EFI_STATUS status;
	EFI_HANDLE *h0, *h;
	EFI_PCI_IO_PROTOCOL *pciio;
	UINTN hlen;
	uint64_t devfn;
	char buf[32];
	int i;

	h0 = h = NULL;
	if (udb_sc != NULL)
		return;
	sc = calloc(1, sizeof(*sc));
	if (sc == NULL)
		return;

	/* Get PCI I/O handle. */
	hlen = 0;
	status = BS->LocateHandle(ByProtocol, &pciio_guid, NULL, &hlen, NULL);
	printf("%s: LocateHandle1 = %lx\n", __func__, status);
	if (status != EFI_BUFFER_TOO_SMALL)
		goto error;

	h0 = malloc(hlen);
	status = BS->LocateHandle(ByProtocol, &pciio_guid, NULL, &hlen, h0);
	printf("%s: LocateHandle2 = %lu\n", __func__, status);
	if (status != EFI_SUCCESS)
		goto error;
	hlen /= sizeof(EFI_HANDLE);

	/* Look up vendor IDs. */
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
		printf("%s: dev_ven = %08x\n", __func__, dev_ven);
		for (i = 0; i < sizeof(ids)/sizeof(ids[0]); i++) {
			if (dev_ven != ((ids[i].device << 16) | ids[i].vendor))
				continue;
			sc->efi.pci_hand = h;
			sc->efi.pciio = pciio;
			udb_sc = sc;
			goto done;
		}
	}
done:
	snprintf(buf, sizeof(buf), "0x%016llx", (unsigned long long)udb_sc);
	setenv("usb_dbc_sc", buf, 1);
	free(h0);

	for (devfn = 0; devfn < 256; devfn++) {
		uint32_t dev = (devfn & 0xf8) >> 3;
		uint32_t fun = devfn & 0x07;
		uint32_t cf8 = (1UL << 31) | (dev << 11) | (fun << 8);
		uint32_t hdr = (pci_read_config(cf8, 3) & 0xff0000U) >> 16;

		if (hdr == 0 || hdr == 0x80) {
			if ((pci_read_config(cf8, 2) >> 8) == XHC_CLASSC) {
				sc->xhc_cf8 = cf8;
				break;
			}
		}
	}
	if (sc->xhc_cf8 == 0) {
		printf("%s: Compatible xHC not found.\n", __func__);
		goto error;
	}
	if (udb_setup(sc) != 0) {
		cons->c_flags = C_PRESENTIN | C_PRESENTOUT;

		udb_disable(sc);
		if (udb_init_dma(sc) != 0) {
			printf("USB DbC DMA configuration error\n");
			return;
		}
		udb_init_ring(sc);
		udb_enable(sc);
	}
	return;
error:
	free(h0);
	free(sc);
	sc = NULL;
	return;
}

static uint32_t
ind(uint32_t port)
{
	uint32_t val;

	__asm volatile(
	    "xorq %%rax, %%rax\n\t"
	    "movq %1, %%rdx\n\t"
	    "inl %%dx, %%eax\n\t"
	    : "=a"(val)
	    : "g"((uint64_t)port));
	return (val);
}

static void
outd(uint32_t port, uint32_t val)
{

	__asm volatile(
	    "movq %0, %%rdx\n\t"
	    "movq %1, %%rax\n\t"
	    "outl %%eax, %%dx\n\t"
	    :
	    : "g"((uint64_t)port), "g"((uint64_t)val));
}

static uint32_t
pci_read_config(uint32_t cf8, uint32_t reg)
{
	uint32_t addr = (cf8 & 0xffffff03UL) | (reg << 2);

	outd(0xcf8, addr);
	return (ind(0xcfc));
}

static void
pci_write_config(uint32_t cf8, uint32_t reg, uint32_t val)
{
	uint32_t addr = (cf8 & 0xffffff03UL) | (reg << 2);

	outd(0xcf8, addr);
	outd(0xcfc, val);
}

static void
udb_enable(struct xhci_debug_softc *sc)
{

	sc->udb_reg->ctrl |= XHCI_DCCTRL_DCE;
	SFENCE();
	while ((sc->udb_reg->ctrl & XHCI_DCCTRL_DCE) == 0)
		PAUSE();

	sc->udb_reg->portsc |= XHCI_PS_PED;
}

static int
udb_init(int arg)
{
	struct xhci_debug_softc *sc = udb_sc;

	if (sc == NULL) {		/* allocated in c_probe */
		printf("USB DbC not found\n");
		return (CMD_ERROR);
	}

	if (sc->udb_reg == NULL) {	/* set in c_probe */
		printf("USB DbC register not found\n");
		return (CMD_ERROR);
	}

	while ((sc->udb_reg->ctrl & XHCI_DCCTRL_DCR) == 0) {
		printf("Insert a debug cable: PR=%d, PLS=%d, DCR=0\n",
		    ((sc->udb_reg->portsc & XHCI_PS_PR) != 0),
		    XHCI_PS_PLS_GET(sc->udb_reg->portsc));
		PAUSE();
	}
	printf("Debug Cable Inserted: PR=%d, PLS=%d, DCR=1\n",
		    ((sc->udb_reg->portsc & XHCI_PS_PR) != 0),
		    XHCI_PS_PLS_GET(sc->udb_reg->portsc));

	sc->open = 1;
	sc->init++;

	return (CMD_OK);
}

static void
udb_disable(struct xhci_debug_softc *sc)
{
	sc->udb_reg->portsc &= ~XHCI_PS_PED;
	SFENCE();
	sc->udb_reg->ctrl &= ~XHCI_DCCTRL_DCE;

	while (sc->udb_reg->ctrl & XHCI_DCCTRL_DCE)
		PAUSE();
}

static int
udb_init_ring(struct xhci_debug_softc *sc)
{
	uint64_t erdp, out, in;

	if (sc->udb_reg == NULL) {
		printf("%s: register is empty\n", __func__);
		return (0);
	}
	/* Create an event ring. */
	udb_xhci_debug_ring_init(sc, &sc->udb_ering, 0, XHCI_DB_INVAL);
	if ((erdp = v2p(sc, sc->udb_ering.trb)) == 0)
		return (0);
	memset(sc->udb_erst, 0, sizeof(*sc->udb_erst));
	/* htole64? */
	sc->udb_erst->qwEvrsTablePtr = htole64(erdp);
	sc->udb_erst->dwEvrsTableSize = htole32(UDB_TRB_RING_CAP);
	printf("%s: ERDP=0x%016llx\n", __func__, (unsigned long long)erdp);

	/* Create an output ring. */
	udb_xhci_debug_ring_init(sc, &sc->udb_oring, 1, XHCI_DB_OUT);
	if ((out = v2p(sc, sc->udb_oring.trb)) == 0)
		return (0);

	/* Create an input ring. */
	udb_xhci_debug_ring_init(sc, &sc->udb_iring, 1, XHCI_DB_IN);
	if ((in = v2p(sc, sc->udb_iring.trb)) == 0)
		return (0);

	/* Initialize DbC context. */
	memset(sc->udb_ctx, 0, sizeof(*sc->udb_ctx));
	udb_init_strings(sc, sc->udb_ctx->info);

	/* Initialize endppints. */ 
	udb_init_ep(sc->udb_ctx->ep_out, sizeof(*sc->udb_ctx->ep_out),
	    (uint64_t)XHCI_DCCTRL_MBS_GET(sc->udb_reg->ctrl), EP_BULK_OUT, out);
	udb_init_ep(sc->udb_ctx->ep_in, sizeof(*sc->udb_ctx->ep_in),
	    (uint64_t)XHCI_DCCTRL_MBS_GET(sc->udb_reg->ctrl), EP_BULK_IN, in);

	/* Configure register values. */
	sc->udb_reg->erstsz = 1;
	sc->udb_reg->erstba = v2p(sc, sc->udb_erst);
	sc->udb_reg->erdp = erdp;
	sc->udb_reg->cp = v2p(sc, sc->udb_ctx);
	sc->udb_reg->ddi1 = (DC_VENDOR   << 16) | DC_PROTOCOL;
	sc->udb_reg->ddi2 = (DC_REVISION << 16) | DC_PRODUCT;

	flush_range(sc->udb_ctx, sizeof(*sc->udb_ctx));
	flush_range(sc->udb_erst, sizeof(*sc->udb_erst));
	flush_range(sc->udb_ering.trb, UDB_TRB_RING_BYTES);
	flush_range(sc->udb_oring.trb, UDB_TRB_RING_BYTES);
	flush_range(sc->udb_iring.trb, UDB_TRB_RING_BYTES);
	flush_range(sc->udb_owork.buf, UDB_WORK_RING_BYTES);

	sc->udb_owork.enq = 0;
	sc->udb_owork.deq = 0;
	sc->udb_owork.paddr = v2p(sc, sc->udb_owork.buf);

	return (1);
}

static int
udb_find(struct xhci_debug_softc *sc)
{
	uint32_t *xcap;
	uint8_t *mmio;
	uint32_t *hccp1;

	mmio = (uint8_t *)sc->xhc_mmio;
	hccp1 = (uint32_t *)(void *)(mmio + 0x10);

	/* Extended Capability Pointer in HCCP1 must be non-zero */
	if ((*hccp1 &0xffff0000) == 0)
		return (0);

	/*
	 * Look up ID=0x0a in the capability list
	 * located at mmio + (HCCPARAMS1[31:16] << 2).
	 */
	xcap = (uint32_t *)(void *)
	    (mmio + (((*hccp1 & 0xffff0000) >> 16) << 2));

#if 0
	while (XHCI_XECP_ID(*xcap) != XHCI_ID_USB_DEBUG && XHCI_XECP_NEXT(*xcap)) {
		printf("%s: looking for %02x: %02x\n",
		    __func__, XHCI_ID_USB_DEBUG, XHCI_XECP_ID(*xcap));
		xcap += XHCI_XECP_NEXT(*xcap);
	}
	if (XHCI_XECP_ID(*xcap) != XHCI_ID_USB_DEBUG) {
		printf("%s: XHCI_ID_USB_DEBUG is not found\n", __func__);
		return (0);
	}
#else
#define        ID(x) (*(x) & 0xff)
#define        NEXT(x) ((*(x) & 0xff00) >> 8)
	while (ID(xcap) != XHCI_ID_USB_DEBUG && NEXT(xcap)) {
		printf("%s: looking for DBC_ID(0x0a): %02x\n",
		    __func__, ID(xcap));
		xcap += NEXT(xcap);
	}
	if (ID(xcap) != XHCI_ID_USB_DEBUG) {
		printf("%s: DBC_ID is not found\n", __func__);
		return (0);
	}
#endif
	sc->xhci_xecp_offset = (uint64_t)xcap - (uint64_t)mmio;
	sc->udb_reg = (struct xhci_debug_reg *)xcap;
	return (1);
}

static int
udb_setup(struct xhci_debug_softc *sc)
{
	uint32_t bar0;
	uint64_t bar1;

	/* Parse the BAR and map the registers */
	bar0 = pci_read_config(sc->xhc_cf8, 4);
	bar1 = pci_read_config(sc->xhc_cf8, 5);

	if ((bar0 & 0x1) != 0 || ((bar0 & 0x6) >> 1) != 2) {
		printf("%s: BAR must be 64-bit.\n", __func__);
		return (0);
	}

	pci_write_config(sc->xhc_cf8, 4, 0xffffffff);
	sc->xhc_mmio_size = ~(pci_read_config(sc->xhc_cf8, 4) & 0xfffffff0) + 1;
	pci_write_config(sc->xhc_cf8, 4, bar0);

	sc->xhc_mmio_phys = (bar0 & 0xfffffff0) | (bar1 << 32);
	sc->xhc_mmio = (void *)sc->xhc_mmio_phys;

	if (sc->xhc_mmio == NULL) {
		printf("%s: mmio_phys is NULL.\n", __func__);
		return (0);
	}

	return (udb_find(sc));
}

static void *
udb_alloc_dma(struct xhci_debug_softc *sc, uint64_t order)
{
	struct dma *dma;
	EFI_STATUS status;
	VOID *addr;
	UINTN pages = 1UL << order;
	UINTN i;

	for (i = 0; i < DMA_DESC_CAP; i++) {
		dma = &sc->dma_desc[i];
		if (dma->cpu_addr == NULL)
			break;
	}
	if (i == DMA_DESC_CAP) {
		printf("%s: Out of DMA descriptor\n", __func__);
		return (NULL);
	}
	status = sc->efi.pciio->AllocateBuffer(sc->efi.pciio,
	    AllocateAnyPages,
	    EfiRuntimeServicesData,
	    pages,
	    &addr,
	    EFI_PCI_IO_ATTRIBUTE_MEMORY_CACHED);
	if (EFI_ERROR(status)) {
		printf("%s: AllocateBuffer failed: 0x%lx\n",
		    __func__, status);
		return (NULL);
	}

	dma->pages = pages;
	dma->cpu_addr = addr;
	printf("%s: i = %lu, cpu_addr = %p, pages = %lu\n", __func__,
	    i, dma->cpu_addr, pages);

	return (addr);
}

static void
udb_free_dma(struct xhci_debug_softc *sc, VOID *addr)
{
	struct dma *dma;
	EFI_STATUS status;
	UINTN i;

	for (i = 0; i < DMA_DESC_CAP; i++) {
		dma = &sc->dma_desc[i];
		if (dma->cpu_addr == addr)
			break;
	}
	if (i == DMA_DESC_CAP)
		return;
	if (dma->mapping) {
		status = sc->efi.pciio->Unmap(sc->efi.pciio, dma->mapping);
		if (EFI_ERROR(status)) {
			printf("%s: Unmap failed: 0x%lx\n",
			    __func__, status);
			return;
		}
	}
	status = sc->efi.pciio->FreeBuffer(sc->efi.pciio, dma->pages, addr);
	if (EFI_ERROR(status)) {
		printf("%s: FreeBuffer failed: 0x%lx\n",
		    __func__, status);
		return;
	}
	memset(&dma, 0, sizeof(*dma));
}

static void
udb_uninit_dma(struct xhci_debug_softc *sc)
{
	udb_free_dma(sc, sc->udb_ctx);
	udb_free_dma(sc, sc->udb_erst);
	udb_free_dma(sc, sc->udb_ering.trb);
	udb_free_dma(sc, sc->udb_oring.trb);
	udb_free_dma(sc, sc->udb_iring.trb);
	udb_free_dma(sc, sc->udb_owork.buf);

	sc->dma_allocated = 0;
}

static int
udb_init_dma(struct xhci_debug_softc *sc)
{
#define	ALLOC_DMA(p, type, order)				\
	do { 							\
		p = (type)udb_alloc_dma(sc, order); 	\
		if (p == NULL) { 				\
			udb_uninit_dma(sc);			\
			return (1);				\
		}						\
	} while (0)

	if (sc->dma_allocated)
		return (0);
	ALLOC_DMA(sc->udb_ctx, struct udb_ctx *, 0);
	ALLOC_DMA(sc->udb_erst, struct xhci_event_ring_seg *, 0);
	ALLOC_DMA(sc->udb_ering.trb, struct xhci_trb *, UDB_TRB_RING_ORDER);
	ALLOC_DMA(sc->udb_iring.trb, struct xhci_trb *, UDB_TRB_RING_ORDER);
	ALLOC_DMA(sc->udb_oring.trb, struct xhci_trb *, UDB_TRB_RING_ORDER);
	ALLOC_DMA(sc->udb_owork.buf, uint8_t *, UDB_WORK_RING_ORDER);
	ALLOC_DMA(sc->udb_str, char *, 0);
	sc->dma_allocated = 1;

	return (0);
#undef	ALLOC_DMA
}

uint64_t
v2p(struct xhci_debug_softc *sc, void *virt)
{
	UINTN i = 0;
	UINTN offset = 0;
	UINTN needed = 0;
	UINTN mapped = 0;
	struct dma *dma = NULL;
	EFI_PHYSICAL_ADDRESS paddr = 0;
	EFI_PCI_IO_PROTOCOL *pci = sc->efi.pciio;
	EFI_STATUS status;
	VOID *mapping = NULL;

	for (; i < DMA_DESC_CAP; i++) {
		dma = &sc->dma_desc[i];
		UINTN p = 0;

#if 0
		printf("%s: dma_desc[%lu].cpu_addr = %p\n", __func__,
		    i, dma->cpu_addr);
#endif
		for (; p < dma->pages; p++) {
			UINTN addr = (UINTN)dma->cpu_addr + (p * PAGE_SIZE);
#if 0
			printf("%s: check addr: %lu = %lu + %lu * %llu\n",
			    __func__, addr, (UINTN)dma->cpu_addr, p, PAGE_SIZE);
#endif
			if ((UINTN)virt == addr) {
				offset = addr - (UINTN)dma->cpu_addr;
				goto found;
			}
		}
		dma = NULL;
	}

	if (!dma) {
		printf("CPU addr %p not found in DMA descriptor\n", virt);
		return 0;
	}

found:
	printf("%s: found, i = %lu, dma->paddr = %lu, offset = %lu\n",
	    __func__, i, dma->paddr, offset);
	if (dma->paddr && dma->mapping)
		return dma->paddr + offset;

	needed = dma->pages << EFI_PAGE_SHIFT;
	mapped = needed;
	status = pci->Map(pci, EfiPciIoOperationBusMasterCommonBuffer,
	    (void *)virt, &mapped, &paddr, &mapping);
	if (EFI_ERROR(status) || mapped != needed) {
		printf("pci->Map failed: rc: 0x%lx, mapped: %lu, needed: %lu\n",
		    status, mapped, needed);
		return (0);
	}
	dma->paddr = paddr;
	dma->mapping = mapping;

	if ((const void *)paddr != virt) {
		printf("Non-identity DMA mapping: dma: 0x%lx cpu: %p\n",
		    paddr, virt);
	}

	return (paddr);
}

static void
udb_xhci_debug_ring_init(struct xhci_debug_softc *sc, struct xhci_debug_ring *ring,
    int producer, int doorbell)
{
	memset(ring->trb, 0, UDB_TRB_RING_CAP * sizeof(ring->trb[0]));

	ring->enq = 0;
	ring->deq = 0;
	ring->cyc = 1;
	ring->doorbell = (uint8_t)doorbell;

	/* Place a link TRB at the tail if producer == 1. */
	if (producer) {
		struct xhci_trb *trb = &ring->trb[UDB_TRB_RING_CAP - 1];

		trb->dwTrb3 |= XHCI_TRB_3_TYPE_SET(XHCI_TRB_TYPE_LINK);
		trb->dwTrb3 |= XHCI_TRB_3_TC_BIT;
		/* Fields for link TRBs (section 6.4.4.1) */
		trb->qwTrb0 = v2p(sc, ring->trb);
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
udb_init_strings(struct xhci_debug_softc *sc, uint32_t *info)
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
	sda[0] = v2p(sc, sc->udb_str);
	sda[1] = sda[0] + 64;
	sda[2] = sda[0] + 64 + 64;
	sda[3] = sda[0] + 64 + 64 + 64;
	info[8] = (4 << 24) | (52 << 16) | (38 << 8) | 6;
}

void
udb_info(void)
{
	struct xhci_debug_softc *sc = udb_sc;
	struct xhci_debug_reg *r;
	if (sc == NULL)
		return;
	printf("USB DbC INFO (softc = %p):\n", sc);

	r = sc->udb_reg;
	if (r == NULL)
		return;

	printf("USB DbC INFO (reg = %p):\n", r);
	printf("=============\n");
	printf("sc->dma_allocated = %d\n", sc->dma_allocated);
	printf("sc->open = %d\n", sc->open);
	printf("sc->init = %d\n", sc->init);
	printf("=============\n");
	printf("    ctrl: 0x%x stat: 0x%x psc: 0x%x\n", r->ctrl, r->st,
	    r->portsc);
	printf("      DbC Enable = %s\n",
	    (r->ctrl & XHCI_DCCTRL_DCE) ? "YES" : "NO");
	printf("      DbC Running = %s\n",
	    (r->ctrl & XHCI_DCCTRL_DCR) ? "YES" : "NO");
	printf("      Link State Event Enable = %s\n",
	    (r->ctrl & XHCI_DCCTRL_LSE) ? "YES" : "NO");
	printf("      Halt OUT TR = %s\n",
	    (r->ctrl & XHCI_DCCTRL_HOT) ? "YES" : "NO");
	printf("      Halt IN TR = %s\n",
	    (r->ctrl & XHCI_DCCTRL_HIT) ? "YES" : "NO");
	printf("      DbC Run Change = %s\n",
	    (r->ctrl & XHCI_DCCTRL_DRC) ? "YES" : "NO");
	printf("      DbC Port = %u\n", (uint32_t)XHCI_DCST_PORT_GET(r->st));
	printf("    id: 0x%x, doorbell: 0x%x\n", r->id, r->doorbell);
	printf("    erstsz: %u, erstba: 0x%lx\n", r->erstsz, r->erstba);
	printf("    erdp: 0x%lx, cp: 0x%lx\n", r->erdp, r->cp);
	printf("    ddi1: 0x%x, ddi2: 0x%x\n", r->ddi1, r->ddi2);
	printf("    erstba == v2p(sc, erst): %d\n",
	    r->erstba == v2p(sc, sc->udb_erst));
	printf("    erdp == erst[0].qwEvrsTablePtr: %d\n",
	    r->erdp == sc->udb_erst[0].qwEvrsTablePtr);
	printf("    cp == v2p(ctx): %d\n",
	    r->cp == v2p(sc, sc->udb_ctx));
	printf("=============\n");
	printf("DbC Commands:\n");
	printf(" udb_send xxx: Send 'xxx'\n");
	printf(" udb_probe: Probe and initialize DbC\n");
	printf(" udb_init: Enable DbC\n");
	printf(" udb_recon: Reconnect DbC Cable\n");
	printf("=============\n");
}

/*
 * Commit the pending transfer TRBs to the DbC.  This notifies
 * the DbC of any previously-queued data on the work ring and
 * rings the doorbell.
 */
void
udb_flush(struct xhci_debug_softc *sc)
{
	struct xhci_debug_reg *reg = sc->udb_reg;
	struct xhci_debug_ring *oring = &sc->udb_oring;
	struct xhci_debug_work_ring *work = &sc->udb_owork;
	uint32_t doorbell = (reg->doorbell & 0xffff00ff) | (oring->doorbell << 8);

	/* Check if DbC is running */
	if (sc->open && (reg->ctrl & XHCI_DCCTRL_DCE) == 0) {
		/* Try to reinitialize */
		if (udb_init(0) != CMD_OK)
			return;
	}

	udb_pop_events(sc);

	if ((reg->ctrl & XHCI_DCCTRL_DCR) == 0) {
		printf("%s: DbC not configured\n", __func__);
		return;
	}
	if (reg->ctrl & XHCI_DCCTRL_DRC) {
		reg->ctrl |= XHCI_DCCTRL_DRC;
		reg->portsc |= XHCI_PS_PED;
		SFENCE();
	}
	if (xhci_debug_ring_full(oring))	/* TRB queue is full */
		return;
	if (work->enq == work->deq)	/* Work queue is empty */
		return;

	if (work->enq > work->deq) {
		/* Push TRBs on work queue to TRB queue */
		push_trb(oring, work->paddr + work->deq, work->enq - work->deq);
		work->deq = work->enq;
	} else {
		push_trb(oring, work->paddr + work->deq, UDB_WORK_RING_CAP - work->deq);
		work->deq = 0;
		if (work->enq > 0 && !xhci_debug_ring_full(oring)) {
			push_trb(oring, work->paddr, work->enq);
			work->deq = work->enq;
		}
	}
	/* Ring the doorbell */
	SFENCE();
	reg->doorbell = doorbell;
}

static void
udb_putc(int c0)
{
	struct xhci_debug_softc *sc = udb_sc;
	u_char c = (0xff & c0);

	if (sc == NULL)
		return;
	if (!push_work(&sc->udb_owork, &c, 1))
		return;
	if (c == '\n')
		udb_flush(sc);
}

static int
udb_getc(void)
{

	return (-1);
}

static int
udb_ischar(void)
{

	return (0);	/* XXX: never arrive a new char */
}

static struct xhci_debug_softc *
udb_open(void)
{
	return (NULL);
}

static void
udb_free(struct xhci_debug_softc *sc)
{
	struct xhci_debug_softc *sc0;

	sc0 = sc;
}

static void
udb_close(struct xhci_debug_softc *sc)
{
	udb_disable(sc);
	udb_free(sc);
	sc->open = 0;
}

/*
 * Initializes the endpoint as specified in sections 7.6.3.2 and 7.6.9.2.
 * Each endpoint is Bulk, so the MaxPStreams, LSA, HID, CErr, FE,
 * Interval, Mult, and Max ESIT Payload fields are all 0.   
 *
 * Max packet size: 1024
 * Max burst size: debug mbs (from db_reg->ctrl register)
 * EP type: 2 for OUT bulk, 6 for IN bulk   
 * TR dequeue ptr: physical base address of transfer ring
 * Avg TRB length: software defined (see 4.14.1.1 for suggested defaults)
 */
static void
udb_init_ep(uint32_t *ep, size_t len, uint64_t mbs, uint32_t type,
    uint64_t ring_paddr)
{
	memset(ep, 0, len);

	ep[1] = (1024 << 16) | ((uint32_t)mbs << 8) | (type << 3);
	ep[2] = (ring_paddr & 0xFFFFFFFF) | 1;
	ep[3] = ring_paddr >> 32;
	ep[4] = 3 * 1024;
}

/* XXX: OUT transfer only */
static void
udb_pop_events(struct xhci_debug_softc *sc)
{
	const int trb_shift = 4;
	struct xhci_debug_reg *reg = sc->udb_reg;
	struct xhci_debug_ring *er = &sc->udb_ering;
	struct xhci_debug_ring *tr = &sc->udb_oring;
	struct xhci_trb *event = &er->trb[er->deq];
	uint64_t erdp = reg->erdp;

#define LFENCE(x)	__asm volatile("lfence" ::: "memory")
	LFENCE();
#undef	LFENCE
	while ((event->dwTrb3 & XHCI_TRB_3_CYCLE_BIT) == er->cyc) {
		switch (XHCI_TRB_3_TYPE_GET(event->dwTrb3)) {
		case XHCI_TRB_EVENT_TRANSFER:
			if (XHCI_TRB_2_ERROR_GET(event->dwTrb2) != XHCI_TRB_ERROR_SUCCESS) {
				printf("transfer error: %u\n",
				    XHCI_TRB_2_ERROR_GET(event->dwTrb2));
            			break;
        		}
			tr->deq =
			    (event->qwTrb0 & UDB_TRB_RING_MASK) >> trb_shift;
			break;
		case XHCI_TRB_EVENT_PORT_STS_CHANGE:
			reg->portsc |= (XHCI_DCPORTSC_ACK_MASK & reg->portsc);
			break;
		default:
			break;
		}
		er->cyc = (er->deq == UDB_TRB_RING_CAP - 1) ? er->cyc ^ 1 : er->cyc;
		er->deq = (er->deq + 1) & (UDB_TRB_RING_CAP - 1);
		event = &er->trb[er->deq];
	}

	erdp &= ~UDB_TRB_RING_MASK;
	erdp |= (er->deq << trb_shift);
	SFENCE();
	reg->erdp = erdp;
}

static void
push_trb(struct xhci_debug_ring *ring, uint64_t addr, uint64_t len)
{
	struct xhci_trb trb;

	if (ring->enq == UDB_TRB_RING_CAP - 1) {
		/*
		 * Make sure the xHC processes the link TRB in order
		 * for wrap-around to work properly by setting the TRB's
		 * cycle bit, just like with normal TRBs.
		 */
		struct xhci_trb *link = &ring->trb[ring->enq];

		link->dwTrb3 &= ~XHCI_TRB_3_CYCLE_BIT;
		link->dwTrb3 |= ring->cyc;
		ring->enq = 0;
		ring->cyc ^= 1;
	}
	trb.qwTrb0 = 0;
	trb.dwTrb2 = 0;
	trb.dwTrb3 = 0;

	trb.dwTrb3 |= XHCI_TRB_3_TYPE_SET(XHCI_TRB_TYPE_NORMAL);
	trb.dwTrb3 &= ~XHCI_TRB_3_CYCLE_BIT;
	trb.dwTrb3 |= ring->cyc;

	trb.qwTrb0 = addr;
	trb.dwTrb2 = XHCI_TRB_2_BYTES_SET(trb.dwTrb2 | (uint32_t)len);
	trb.dwTrb3 |= XHCI_TRB_3_IOC_BIT;

	ring->trb[ring->enq++] = trb;
	flush_range(&ring->trb[ring->enq - 1], sizeof(trb));
}

static int
xhci_debug_work_ring_full(const struct xhci_debug_work_ring *ring)
{

	return ((ring->enq + 1) & (UDB_WORK_RING_CAP - 1)) == ring->deq;
}

static uint64_t
xhci_debug_work_ring_size(const struct xhci_debug_work_ring *ring)
{

	return (ring->enq >= ring->deq)
	    ? (ring->enq - ring->deq)
	    : (UDB_WORK_RING_CAP - ring->deq + ring->enq);
}

static int64_t
push_work(struct xhci_debug_work_ring *ring, const char *buf, int64_t len)
{
	int64_t i = 0;
	uint32_t start = ring->enq;
	uint32_t end = 0;

	while (!xhci_debug_work_ring_full(ring) && i < len) {
		ring->buf[ring->enq] = buf[i++];
		ring->enq = (ring->enq + 1) & (UDB_WORK_RING_CAP - 1);
	}

	end = ring->enq;

	if (end > start)
		flush_range(&ring->buf[start], end - start);
	else if (i > 0) {
		flush_range(&ring->buf[start], UDB_WORK_RING_CAP - start);
		flush_range(&ring->buf[0], end);
	}
	return (i);
}

static int
xhci_debug_ring_full(const struct xhci_debug_ring *ring)
{

	return ((ring->enq + 1) & (UDB_TRB_RING_CAP - 1)) == ring->deq;
}

static void
flush_range(void *ptr, uint32_t bytes)
{
	uint32_t i;

	const uint32_t clshft = 6;
	const uint32_t clsize = (1UL << clshft);
	const uint32_t clmask = clsize - 1;

	uint32_t lines = (bytes >> clshft);
	lines += (bytes & clmask) != 0;

#define	CLFLUSH(ptr) \
	__asm volatile("clflush %0" : "+m"(*(volatile char *)ptr))

	for (i = 0; i < lines; i++)
		CLFLUSH((void *)((uint64_t)ptr + (i * clsize)));
#undef	CLFLUSH
}
