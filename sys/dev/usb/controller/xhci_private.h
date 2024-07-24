/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2010-2022 Hans Petter Selasky
 * Copyright (c) 2020-2024 Hiroki Sato <hrs@FreeBSD.org>
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

#ifndef _USB_XHCI_PRIVATE_H_
#define	_USB_XHCI_PRIVATE_H_

#define	XHCI_DEV_CTX_ADDR_ALIGN		64	/* bytes */
#define	XHCI_DEV_CTX_ALIGN		64	/* bytes */
#define	XHCI_INPUT_CTX_ALIGN		64	/* bytes */
#define	XHCI_SLOT_CTX_ALIGN		32	/* bytes */
#define	XHCI_ENDP_CTX_ALIGN		32	/* bytes */
#define	XHCI_STREAM_CTX_ALIGN		16	/* bytes */
#define	XHCI_TRANS_RING_SEG_ALIGN	16	/* bytes */
#define	XHCI_CMD_RING_SEG_ALIGN		64	/* bytes */
#define	XHCI_EVENT_RING_SEG_ALIGN	64	/* bytes */
#define	XHCI_SCRATCH_BUF_ARRAY_ALIGN	64	/* bytes */
#define	XHCI_SCRATCH_BUFFER_ALIGN	USB_PAGE_SIZE
#define	XHCI_TRB_ALIGN			16	/* bytes */
#define	XHCI_TD_ALIGN			64	/* bytes */
#define	XHCI_PAGE_SIZE			4096	/* bytes */

struct xhci_endp_ctx {
	volatile uint32_t	dwEpCtx0;
#define	XHCI_EPCTX_0_EPSTATE_SET(x)		((x) & 0x7)
#define	XHCI_EPCTX_0_EPSTATE_GET(x)		((x) & 0x7)
#define	XHCI_EPCTX_0_EPSTATE_DISABLED		0
#define	XHCI_EPCTX_0_EPSTATE_RUNNING		1
#define	XHCI_EPCTX_0_EPSTATE_HALTED		2
#define	XHCI_EPCTX_0_EPSTATE_STOPPED		3
#define	XHCI_EPCTX_0_EPSTATE_ERROR		4
#define	XHCI_EPCTX_0_EPSTATE_RESERVED_5		5
#define	XHCI_EPCTX_0_EPSTATE_RESERVED_6		6
#define	XHCI_EPCTX_0_EPSTATE_RESERVED_7		7
#define	XHCI_EPCTX_0_MULT_SET(x)		(((x) & 0x3) << 8)
#define	XHCI_EPCTX_0_MULT_GET(x)		(((x) >> 8) & 0x3)
#define	XHCI_EPCTX_0_MAXP_STREAMS_SET(x)	(((x) & 0x1F) << 10)
#define	XHCI_EPCTX_0_MAXP_STREAMS_GET(x)	(((x) >> 10) & 0x1F)
#define	XHCI_EPCTX_0_LSA_SET(x)			(((x) & 0x1) << 15)
#define	XHCI_EPCTX_0_LSA_GET(x)			(((x) >> 15) & 0x1)
#define	XHCI_EPCTX_0_IVAL_SET(x)		(((x) & 0xFF) << 16)
#define	XHCI_EPCTX_0_IVAL_GET(x)		(((x) >> 16) & 0xFF)
	volatile uint32_t	dwEpCtx1;
#define	XHCI_EPCTX_1_CERR_SET(x)		(((x) & 0x3) << 1)
#define	XHCI_EPCTX_1_CERR_GET(x)		(((x) >> 1) & 0x3)
#define	XHCI_EPCTX_1_EPTYPE_SET(x)		(((x) & 0x7) << 3)
#define	XHCI_EPCTX_1_EPTYPE_GET(x)		(((x) >> 3) & 0x7)
#define	XHCI_EPCTX_1_HID_SET(x)			(((x) & 0x1) << 7)
#define	XHCI_EPCTX_1_HID_GET(x)			(((x) >> 7) & 0x1)
#define	XHCI_EPCTX_1_MAXB_SET(x)		(((x) & 0xFF) << 8)
#define	XHCI_EPCTX_1_MAXB_GET(x)		(((x) >> 8) & 0xFF)
#define	XHCI_EPCTX_1_MAXP_SIZE_SET(x)		(((x) & 0xFFFF) << 16)
#define	XHCI_EPCTX_1_MAXP_SIZE_GET(x)		(((x) >> 16) & 0xFFFF)
	volatile uint64_t	qwEpCtx2;
#define	XHCI_EPCTX_2_DCS_SET(x)			((x) & 0x1)
#define	XHCI_EPCTX_2_DCS_GET(x)			((x) & 0x1)
#define	XHCI_EPCTX_2_TR_DQ_PTR_MASK		0xFFFFFFFFFFFFFFF0U
	volatile uint32_t	dwEpCtx4;
#define	XHCI_EPCTX_4_AVG_TRB_LEN_SET(x)		((x) & 0xFFFF)
#define	XHCI_EPCTX_4_AVG_TRB_LEN_GET(x)		((x) & 0xFFFF)
#define	XHCI_EPCTX_4_MAX_ESIT_PAYLOAD_SET(x)	(((x) & 0xFFFF) << 16)
#define	XHCI_EPCTX_4_MAX_ESIT_PAYLOAD_GET(x)	(((x) >> 16) & 0xFFFF)
	volatile uint32_t	dwEpCtx5;
	volatile uint32_t	dwEpCtx6;
	volatile uint32_t	dwEpCtx7;
};

struct xhci_endp_ctx64 {
	struct xhci_endp_ctx	ctx;
	volatile uint8_t	padding[32];
};

struct xhci_trb {
	volatile uint64_t	qwTrb0;
#define	XHCI_TRB_0_DIR_IN_MASK		(0x80ULL << 0)
#define	XHCI_TRB_0_WLENGTH_MASK		(0xFFFFULL << 48)
	volatile uint32_t	dwTrb2;
#define	XHCI_TRB_2_ERROR_GET(x)		(((x) >> 24) & 0xFF)
#define	XHCI_TRB_2_ERROR_SET(x)		(((x) & 0xFF) << 24)
#define	XHCI_TRB_2_TDSZ_GET(x)		(((x) >> 17) & 0x1F)
#define	XHCI_TRB_2_TDSZ_SET(x)		(((x) & 0x1F) << 17)
#define	XHCI_TRB_2_REM_GET(x)		((x) & 0xFFFFFF)
#define	XHCI_TRB_2_REM_SET(x)		((x) & 0xFFFFFF)
#define	XHCI_TRB_2_BYTES_GET(x)		((x) & 0x1FFFF)
#define	XHCI_TRB_2_BYTES_SET(x)		((x) & 0x1FFFF)
#define	XHCI_TRB_2_IRQ_GET(x)		(((x) >> 22) & 0x3FF)
#define	XHCI_TRB_2_IRQ_SET(x)		(((x) & 0x3FF) << 22)
#define	XHCI_TRB_2_STREAM_GET(x)	(((x) >> 16) & 0xFFFF)
#define	XHCI_TRB_2_STREAM_SET(x)	(((x) & 0xFFFF) << 16)

	volatile uint32_t	dwTrb3;
#define	XHCI_TRB_3_TYPE_GET(x)		(((x) >> 10) & 0x3F)
#define	XHCI_TRB_3_TYPE_SET(x)		(((x) & 0x3F) << 10)
#define	XHCI_TRB_3_CYCLE_BIT		(1U << 0)
#define	XHCI_TRB_3_TC_BIT		(1U << 1)	/* command ring only */
#define	XHCI_TRB_3_ENT_BIT		(1U << 1)	/* transfer ring only */
#define	XHCI_TRB_3_ISP_BIT		(1U << 2)
#define	XHCI_TRB_3_NSNOOP_BIT		(1U << 3)
#define	XHCI_TRB_3_CHAIN_BIT		(1U << 4)
#define	XHCI_TRB_3_IOC_BIT		(1U << 5)
#define	XHCI_TRB_3_IDT_BIT		(1U << 6)
#define	XHCI_TRB_3_TBC_GET(x)		(((x) >> 7) & 3)
#define	XHCI_TRB_3_TBC_SET(x)		(((x) & 3) << 7)
#define	XHCI_TRB_3_BEI_BIT		(1U << 9)
#define	XHCI_TRB_3_DCEP_BIT		(1U << 9)
#define	XHCI_TRB_3_PRSV_BIT		(1U << 9)
#define	XHCI_TRB_3_BSR_BIT		(1U << 9)
#define	XHCI_TRB_3_TRT_MASK		(3U << 16)
#define	XHCI_TRB_3_TRT_NONE		(0U << 16)
#define	XHCI_TRB_3_TRT_OUT		(2U << 16)
#define	XHCI_TRB_3_TRT_IN		(3U << 16)
#define	XHCI_TRB_3_DIR_IN		(1U << 16)
#define	XHCI_TRB_3_TLBPC_GET(x)		(((x) >> 16) & 0xF)
#define	XHCI_TRB_3_TLBPC_SET(x)		(((x) & 0xF) << 16)
#define	XHCI_TRB_3_EP_GET(x)		(((x) >> 16) & 0x1F)
#define	XHCI_TRB_3_EP_SET(x)		(((x) & 0x1F) << 16)
#define	XHCI_TRB_3_FRID_GET(x)		(((x) >> 20) & 0x7FF)
#define	XHCI_TRB_3_FRID_SET(x)		(((x) & 0x7FF) << 20)
#define	XHCI_TRB_3_ISO_SIA_BIT		(1U << 31)
#define	XHCI_TRB_3_SUSP_EP_BIT		(1U << 23)
#define	XHCI_TRB_3_SLOT_GET(x)		(((x) >> 24) & 0xFF)
#define	XHCI_TRB_3_SLOT_SET(x)		(((x) & 0xFF) << 24)

/* Commands */
#define	XHCI_TRB_TYPE_RESERVED		0x00
#define	XHCI_TRB_TYPE_NORMAL		0x01
#define	XHCI_TRB_TYPE_SETUP_STAGE	0x02
#define	XHCI_TRB_TYPE_DATA_STAGE	0x03
#define	XHCI_TRB_TYPE_STATUS_STAGE	0x04
#define	XHCI_TRB_TYPE_ISOCH		0x05
#define	XHCI_TRB_TYPE_LINK		0x06
#define	XHCI_TRB_TYPE_EVENT_DATA	0x07
#define	XHCI_TRB_TYPE_NOOP		0x08
#define	XHCI_TRB_TYPE_ENABLE_SLOT	0x09
#define	XHCI_TRB_TYPE_DISABLE_SLOT	0x0A
#define	XHCI_TRB_TYPE_ADDRESS_DEVICE	0x0B
#define	XHCI_TRB_TYPE_CONFIGURE_EP	0x0C
#define	XHCI_TRB_TYPE_EVALUATE_CTX	0x0D
#define	XHCI_TRB_TYPE_RESET_EP		0x0E
#define	XHCI_TRB_TYPE_STOP_EP		0x0F
#define	XHCI_TRB_TYPE_SET_TR_DEQUEUE	0x10
#define	XHCI_TRB_TYPE_RESET_DEVICE	0x11
#define	XHCI_TRB_TYPE_FORCE_EVENT	0x12
#define	XHCI_TRB_TYPE_NEGOTIATE_BW	0x13
#define	XHCI_TRB_TYPE_SET_LATENCY_TOL  	0x14
#define	XHCI_TRB_TYPE_GET_PORT_BW	0x15
#define	XHCI_TRB_TYPE_FORCE_HEADER	0x16
#define	XHCI_TRB_TYPE_NOOP_CMD		0x17

/* Events */
#define	XHCI_TRB_EVENT_TRANSFER		0x20
#define	XHCI_TRB_EVENT_CMD_COMPLETE	0x21
#define	XHCI_TRB_EVENT_PORT_STS_CHANGE  0x22
#define	XHCI_TRB_EVENT_BW_REQUEST      	0x23
#define	XHCI_TRB_EVENT_DOORBELL		0x24
#define	XHCI_TRB_EVENT_HOST_CTRL	0x25
#define	XHCI_TRB_EVENT_DEVICE_NOTIFY	0x26
#define	XHCI_TRB_EVENT_MFINDEX_WRAP	0x27

/* Error codes */
#define	XHCI_TRB_ERROR_INVALID		0x00
#define	XHCI_TRB_ERROR_SUCCESS		0x01
#define	XHCI_TRB_ERROR_DATA_BUF		0x02
#define	XHCI_TRB_ERROR_BABBLE		0x03
#define	XHCI_TRB_ERROR_XACT		0x04
#define	XHCI_TRB_ERROR_TRB		0x05
#define	XHCI_TRB_ERROR_STALL		0x06
#define	XHCI_TRB_ERROR_RESOURCE		0x07
#define	XHCI_TRB_ERROR_BANDWIDTH	0x08
#define	XHCI_TRB_ERROR_NO_SLOTS		0x09
#define	XHCI_TRB_ERROR_STREAM_TYPE	0x0A
#define	XHCI_TRB_ERROR_SLOT_NOT_ON	0x0B
#define	XHCI_TRB_ERROR_ENDP_NOT_ON	0x0C
#define	XHCI_TRB_ERROR_SHORT_PKT	0x0D
#define	XHCI_TRB_ERROR_RING_UNDERRUN	0x0E
#define	XHCI_TRB_ERROR_RING_OVERRUN	0x0F
#define	XHCI_TRB_ERROR_VF_RING_FULL	0x10
#define	XHCI_TRB_ERROR_PARAMETER	0x11
#define	XHCI_TRB_ERROR_BW_OVERRUN	0x12
#define	XHCI_TRB_ERROR_CONTEXT_STATE	0x13
#define	XHCI_TRB_ERROR_NO_PING_RESP	0x14
#define	XHCI_TRB_ERROR_EV_RING_FULL	0x15
#define	XHCI_TRB_ERROR_INCOMPAT_DEV	0x16
#define	XHCI_TRB_ERROR_MISSED_SERVICE	0x17
#define	XHCI_TRB_ERROR_CMD_RING_STOP	0x18
#define	XHCI_TRB_ERROR_CMD_ABORTED	0x19
#define	XHCI_TRB_ERROR_STOPPED		0x1A
#define	XHCI_TRB_ERROR_LENGTH		0x1B
#define	XHCI_TRB_ERROR_BAD_MELAT	0x1D
#define	XHCI_TRB_ERROR_ISOC_OVERRUN	0x1F
#define	XHCI_TRB_ERROR_EVENT_LOST	0x20
#define	XHCI_TRB_ERROR_UNDEFINED	0x21
#define	XHCI_TRB_ERROR_INVALID_SID	0x22
#define	XHCI_TRB_ERROR_SEC_BW		0x23
#define	XHCI_TRB_ERROR_SPLIT_XACT	0x24
} __aligned(4);

struct xhci_event_ring_seg {
	volatile uint64_t	qwEvrsTablePtr;
	volatile uint32_t	dwEvrsTableSize;
	volatile uint32_t	dwEvrsReserved;
};

#endif					/* _USB_XHCI_PRIVATE_H_ */
