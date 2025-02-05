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

#ifdef _KERNEL
extern int udb_debug;
#define	DEBUG_PRINTF(level, ...)	do { 		\
	if ((level) <= udb_debug) printf(__VA_ARGS__);	\
	} while (0)

#define	_XREAD4(sc, what, a)		XREAD4((sc)->sc_xhci, what, a)
#define	_XWRITE4(sc, what, a, x)	XWRITE4((sc)->sc_xhci, what, a, (x))
#define	_XREAD44LH(sc, what, a)		XREAD44LH((sc)->sc_xhci, what, a)
#define	_XWRITE44LH(sc, what, a, x) XWRITE44LH((sc)->sc_xhci, what, a, (x))
/* sys/systm.h */
#define	delay(x)	DELAY(x)
#define LFENCE(x)	__asm volatile("lfence" ::: "memory")

#else

#define	DEBUG_PRINTF(level, ...)
#define LFENCE(x)	__asm volatile("lfence" ::: "memory")

/* Read/write macros for MMIO space */
#define	_XREAD4(sc, what, a)	\
	(*((volatile uint32_t *) \
	 ((sc)->sc_mmio_phys + (a) + (sc)->sc_##what##_off)))
#define	_XREAD44LH(sc, what, a)	\
	((uint64_t)_XREAD4(sc, what, a##_LO) | \
	     ((uint64_t)_XREAD4(sc, what, a##_HI) << 32)) 
#define	_XWRITE4(sc, what, a, x) \
	do { \
		__asm volatile("sfence" ::: "memory"); \
		*((volatile uint32_t *) \
		 ((sc)->sc_mmio_phys + (a) + (sc)->sc_##what##_off)) = (x); \
		__asm volatile("sfence" ::: "memory"); \
	} while (0)
#define	_XWRITE44LH(sc, what, a, x) \
	do { \
		_XWRITE4(sc, what, a##_LO, (uint32_t)((x) & 0xffffffff)); \
		_XWRITE4(sc, what, a##_HI, (uint32_t)((x) >> 32)); \
	} while (0)
#endif
