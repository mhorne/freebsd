/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2020 Mitchell Horne
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
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

#ifndef _MACHINE_GDB_MACHDEP_H_
#define	_MACHINE_GDB_MACHDEP_H_

#define	GDB_BUFSZ	4096
#define	GDB_NREGS	33
#define	GDB_REG_RA	1
#define	GDB_REG_SP	2
#define	GDB_REG_GP	3
#define	GDB_REG_TP	4
#define	GDB_REG_T0	5
#define	GDB_REG_FP	8
#define	GDB_REG_S1	9
#define	GDB_REG_A0	10
#define	GDB_REG_S2	18
#define	GDB_REG_T3	28
#define	GDB_REG_PC	32
#define	GDB_REG_STATUS	33
#define	GDB_REG_TVAL	34
#define	GDB_REG_CAUSE	35
_Static_assert(GDB_BUFSZ >= (GDB_NREGS * 16), "buffer fits 'g' regs");

static __inline size_t
gdb_cpu_regsz(int regnum __unused)
{

	return (8);
}

static __inline int
gdb_cpu_query(void)
{
	return (0);
}

static __inline void *
gdb_begin_write(void)
{

	return (NULL);
}

static __inline void
gdb_end_write(void *arg __unused)
{

}

static __inline void
gdb_cpu_stop_reason(int type __unused, int code __unused)
{

}

void	*gdb_cpu_getreg(int, size_t *);
void	 gdb_cpu_setreg(int, void *);
int	 gdb_cpu_signal(int, int);

#endif /* !_MACHINE_GDB_MACHDEP_H_ */
