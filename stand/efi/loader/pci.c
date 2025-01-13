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

#include "pci.h"

static inline volatile uint32_t ind(volatile uint32_t);
static inline void outd(volatile uint32_t, volatile uint32_t);

static inline volatile uint32_t
ind(volatile uint32_t port)
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

static inline void
outd(volatile uint32_t port, volatile uint32_t val)
{

	__asm volatile(
	    "movq %0, %%rdx\n\t"
	    "movq %1, %%rax\n\t"
	    "outl %%eax, %%dx\n\t"
	    :
	    : "g"((uint64_t)port), "g"((uint64_t)val));
}

volatile uint32_t
pci_read_config(volatile uint32_t sc_ca, volatile uint32_t reg)
{

	outd(PCIR_IO_COMMAND_ADDRESS, sc_ca | (reg << 2));
	return (ind(PCIR_IO_COMMAND_DATA));
}

void
pci_write_config(volatile uint32_t sc_ca, volatile uint32_t reg,
    volatile uint32_t val)
{

	outd(PCIR_IO_COMMAND_ADDRESS, sc_ca | (reg << 2));
	outd(PCIR_IO_COMMAND_DATA, val);
}
