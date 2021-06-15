/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2020, Scott Phillips <scottph@freebsd.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef	_SYS_DUMPSET_H_
#define	_SYS_DUMPSET_H_

#include <sys/_bitset.h>
#include <sys/bitset.h>
#include <vm/vm_phys.h>

extern struct bitset *vm_page_dump;
extern long vm_page_dump_pages;
extern vm_paddr_t dump_avail[PHYS_AVAIL_COUNT];

static inline bool
dump_page_is_dumpable(vm_paddr_t *availp, vm_paddr_t pa)
{
	vm_page_t m;
	int i;

	if ((m = vm_phys_paddr_to_vm_page(pa)) != NULL)
		return ((m->flags & PG_NODUMP) == 0);
	for (i = 0; availp[i] != 0 || availp[i + 1] != 0; i += 2) {
		if (pa >= availp[i] && pa < availp[i + 1])
			return (true);
	}
	return (false);
}

static inline void
dump_add_page(vm_paddr_t *availp, struct bitset *bitset, vm_paddr_t pa)
{
	vm_pindex_t adj;
	int i;

	adj = 0;
	for (i = 0; availp[i + 1] != 0; i += 2) {
		if (pa >= availp[i] && pa < availp[i + 1]) {
			BIT_SET_ATOMIC(vm_page_dump_pages, (pa >> PAGE_SHIFT) -
			    (availp[i] >> PAGE_SHIFT) + adj, bitset);
			return;
		}
		adj += howmany(availp[i + 1], PAGE_SIZE) -
		    availp[i] / PAGE_SIZE;
	}
}

static inline void
dump_drop_page(vm_paddr_t *availp, struct bitset *bitset, vm_paddr_t pa)
{
	vm_pindex_t adj;
	int i;

	adj = 0;
	for (i = 0; availp[i + 1] != 0; i += 2) {
		if (pa >= availp[i] && pa < availp[i + 1]) {
			BIT_CLR_ATOMIC(vm_page_dump_pages, (pa >> PAGE_SHIFT) -
			    (availp[i] >> PAGE_SHIFT) + adj, bitset);
			return;
		}
		adj += howmany(availp[i + 1], PAGE_SIZE) -
		    availp[i] / PAGE_SIZE;
	}
}

static inline vm_paddr_t
vm_page_dump_index_to_pa(vm_paddr_t *availp, int bit)
{
	int i, tot;

	for (i = 0; availp[i + 1] != 0; i += 2) {
		tot = howmany(availp[i + 1], PAGE_SIZE) -
		    availp[i] / PAGE_SIZE;
		if (bit < tot)
			return ((vm_paddr_t)bit * PAGE_SIZE +
			    (availp[i] & ~PAGE_MASK));
		bit -= tot;
	}
	return ((vm_paddr_t)NULL);
}

#define VM_PAGE_DUMP_FOREACH(availp, bitset, pa)				\
	for (vm_pindex_t __b = BIT_FFS(vm_page_dump_pages, bitset);		\
	    (pa) = vm_page_dump_index_to_pa(availp, __b - 1), __b != 0;		\
	    __b = BIT_FFS_AT(vm_page_dump_pages, bitset, __b))

#endif	/* _SYS_DUMPSET_H_ */
