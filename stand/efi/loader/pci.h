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

#define	PCIR_IO_COMMAND_ADDRESS	0x0cf8
#define	PCIR_IO_CA_ENABLE	(1UL << 31)
#define	PCIR_IO_CA_BUS_GET(x)	(((x) >> 16) & 0xff)
#define	PCIR_IO_CA_BUS_SET(x)	(((x) & 0xff) << 16)
#define	PCIR_IO_CA_DEV_GET(x)	(((x) >> 11) & 0x1f)
#define	PCIR_IO_CA_DEV_SET(x)	(((x) & 0x1f) << 11)
#define	PCIR_IO_CA_FUN_GET(x)	(((x) >> 8)  & 0x07)
#define	PCIR_IO_CA_FUN_SET(x)	(((x) & 0x07) << 8)
#define	PCIR_IO_COMMAND_DATA	0x0cfc
#define	PCIR_CA_CLASS	0x02	/* 0x04h in octect */
#define	PCIR_CA_HEADER	0x03	/* 0x08h in octect */
#define	PCIR_CA_BAR_LO	0x04	/* 0x10h in octect */
#define	PCIR_CA_BAR_HI	0x05	/* 0x14h in octect */

struct pciid {
	uint16_t vendor;
	uint16_t device;
};

volatile uint32_t pci_read_config(volatile uint32_t, volatile uint32_t);
void pci_write_config(volatile uint32_t, volatile uint32_t, volatile uint32_t);
