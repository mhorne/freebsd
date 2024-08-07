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

extern EFI_GUID pciio_guid;

/*
 * Dummy functions to use xhci_pci_match().  Note that device_t is defined
 * as PCI Type 00 header.
 */
typedef struct {  
	UINT16	VendorId;
	UINT16	DeviceId;
	UINT16	Command;
	UINT16	Status;
	UINT8	RevisionID;
	UINT8	ClassCode[3];
	UINT8	CacheLineSize;
	UINT8	LatencyTimer;
	UINT8	HeaderType;
	UINT8	BIST;
} PCI_DEVICE_INDEPENDENT_REGION;

typedef struct {
	UINT32	Bar[6];
	UINT32	CISPtr;
	UINT16	SubsystemVendorID;
	UINT16	SubsystemID;
	UINT32	ExpansionRomBar;
	UINT8	CapabilityPtr;
	UINT8	Reserved1[3];
	UINT32	Reserved2;
	UINT8	InterruptLine;
	UINT8	InterruptPin;
	UINT8	MinGnt;
	UINT8	MaxLat;
} PCI_DEVICE_HEADER_TYPE_REGION;

typedef struct {
        PCI_DEVICE_INDEPENDENT_REGION   Hdr;
        PCI_DEVICE_HEADER_TYPE_REGION   Device;
} PCI_TYPE00;

typedef	PCI_TYPE00 device_t;

static inline uint32_t
pci_get_devid(device_t dev)
{
	return ((dev.Hdr.DeviceId << 16) | dev.Hdr.VendorId);
}
static inline uint8_t
pci_get_class(device_t dev)
{
	return (dev.Hdr.ClassCode[0]);
}
static inline uint8_t
pci_get_subclass(device_t dev)
{
	return (dev.Hdr.ClassCode[1]);
}
static inline uint8_t
pci_get_progif(device_t dev)
{
	return (dev.Hdr.ClassCode[2]);
}
static inline uint8_t
pci_get_headertype(device_t dev)
{
	return (dev.Hdr.HeaderType);
}
