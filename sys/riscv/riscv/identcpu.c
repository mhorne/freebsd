/*-
 * Copyright (c) 2015-2016 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * Portions of this software were developed by SRI International and the
 * University of Cambridge Computer Laboratory under DARPA/AFRL contract
 * FA8750-10-C-0237 ("CTSRD"), as part of the DARPA CRASH research programme.
 *
 * Portions of this software were developed by the University of Cambridge
 * Computer Laboratory as part of the CTSRD Project, with support from the
 * UK Higher Education Innovation Fund (HEIF).
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

#include "opt_platform.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/ctype.h>
#include <sys/kernel.h>
#include <sys/pcpu.h>
#include <sys/sysctl.h>

#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/elf.h>
#include <machine/md_var.h>
#include <machine/trap.h>

#ifdef FDT
#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

char machine[] = "riscv";

SYSCTL_STRING(_hw, HW_MACHINE, machine, CTLFLAG_RD, machine, 0,
    "Machine class");

/* Hardware implementation info. These values may be empty. */
register_t mvendorid;	/* The CPU's JEDEC vendor ID */
register_t marchid;	/* The architecture ID */
register_t mimpid;	/* The implementation ID */

struct cpu_desc {
	u_int		cpu_marchid;
	u_int		cpu_part_num;
	u_int		cpu_core_impl;
	const char	*cpu_march_name;
	const char	*cpu_part_name;
	const char	*cpu_vendor_name;
};

struct cpu_desc cpu_desc[MAXCPU];

struct cpu_parts {
	u_int		part_id;
	const char	*part_name;
};
#define	CPU_PART_NONE	{ -1, "Unknown Processor" }

/*
 * CPU base
 */
static const struct cpu_parts cpu_parts_std[] = {
	{ CPU_PART_RV32,	"RV32" },
	{ CPU_PART_RV64,	"RV64" },
	{ CPU_PART_RV128,	"RV128" },
	CPU_PART_NONE,
};

/*
 * Architecture implementers table.
 */
#define	MARCHID_IMPLEMENTER_NONE	{ 0, "Unknown Architecture Implementer" }

struct marchid_implementers {
	register_t	march_id;
	const char	*march_name;
};

const struct marchid_implementers marchid_implementers[] = {
	{ MARCHID_UCB_ROCKET,	"UC Berkeley Rocket" },
	{ MARCHID_UCB_BOOM,	"UC Berkeley Boom" },
	{ MARCHID_OHWG_CVA6,	"OpenHW Group CVA6" },
	{ MARCHID_OHWG_CV32E40P, "OpenHW Group CV32E40P" },
	{ MARCHID_UCB_SPIKE,	"UC Berkeley Spike" },
	{ MARCHID_UC_RVBS,	"University of Cambridge RVBS" },
	MARCHID_IMPLEMENTER_NONE,
};

/*
 * Core implementers table.
*/
#define	MVENDORID_NONE		{ 0, "Unknown Core Implementer" }

struct mvendorid_implementers {
	register_t	mvendor_id;
	const char	*mvendor_name;
};

const struct mvendorid_implementers mvendorid_implementers[] = {
	{ MVENDORID_WDC,	"Western Digital Corporation" },
	{ MVENDORID_SIFIVE,	"SiFive" },
	MVENDORID_NONE,
};

#ifdef FDT
/*
 * The ISA string is made up of a small prefix (e.g. rv64) and up to 26 letters
 * indicating the presence of the 26 possible standard extensions. Therefore 32
 * characters will be sufficient.
 */
#define	ISA_PREFIX		("rv" __XSTRING(__riscv_xlen))
#define	ISA_PREFIX_LEN		(sizeof(ISA_PREFIX) - 1)

static __inline int
parse_ext_s(char *isa, int idx, int len)
{
	/*
	 * Proceed to the next multi-letter extension or the end of the
	 * string.
	 *
	 * TODO: parse these once we gain support
	 */
	printf("multi-letter ext: ");
	while (isa[idx] != '_' && idx < len) {
		cnputc(isa[idx]);
		idx++;
	}
	cnputc('\n');

	return (idx);
}

static __inline int
parse_ext_x(char *isa, int idx, int len)
{
	/*
	 * Proceed to the next multi-letter extension or the end of the
	 * string.
	 */
	printf("multi-letter ext: ");
	while (isa[idx] != '_' && idx < len) {
		cnputc(isa[idx]);
		idx++;
	}
	cnputc('\n');

	return (idx);
}

static __inline int
parse_ext_z(char *isa, int idx, int len)
{
	/*
	 * Proceed to the next multi-letter extension or the end of the
	 * string.
	 *
	 * TODO: parse some of these.
	 */
	printf("multi-letter ext: ");
	while (isa[idx] != '_' && idx < len) {
		cnputc(isa[idx]);
		idx++;
	}
	cnputc('\n');

	return (idx);
}

static __inline int
parse_ext_version(char *isa, int idx, u_int *majorp __unused,
    u_int *minorp __unused)
{
	/* Major version. */
	while (isdigit(isa[idx]))
		idx++;

	if (isa[idx] != 'p')
		return (idx);
	else
		idx++;

	/* Minor version. */
	while (isdigit(isa[idx]))
		idx++;

	return (idx);
}

/*
 * Parse the ISA string
 *
 * The st
 */
static void
parse_riscv_isa(char *isa, int len, u_long *hwcapp)
{
	u_long hwcap;
	char c;
	int i;

	hwcap = 0;
	i = ISA_PREFIX_LEN;
	while (i < len) {
		switch(isa[i]) {
		case 'a':
		case 'c':
#ifdef FPE
		case 'd':
		case 'f':
#endif
		case 'i':
		case 'm':
			hwcap |= HWCAP_ISA_BIT(isa[i]);
			i++;
			break;
		case 'g':
			hwcap |= HWCAP_ISA_G;
			i++;
			break;
		case 's':
			/*
			 * XXX: older versions of this string erroneously
			 * indicated supervisor and user mode support as
			 * single-letter extensions. Detect and skip both 's'
			 * and 'u'.
			 */
			if (isa[i - 1] != '_' && isa[i + 1] == 'u') {
				i += 2;
				continue;
			}

			/*
			 * Supervisor-level extension namespace.
			 */
			i = parse_ext_s(isa, i, len);
			break;
		case 'x':
			/*
			 * Custom extension namespace. For now, we ignore
			 * these.
			 */
			i = parse_ext_x(isa, i, len);
			break;
		case 'z':
			/*
			 * Multi-letter standard extension namespace.
			 */
			i = parse_ext_z(isa, i, len);
			break;
		case '_':
			i++;
			continue;
		default:
			/* Unrecognized/unsupported. */
			i++;
			break;
		}

		i = parse_ext_version(isa, i, NULL, NULL);
	}

	if (hwcapp != NULL)
		*hwcapp = hwcap;
}

static void
fill_elf_hwcap(void *dummy __unused)
{
	char isa[2048];
	u_long hwcap;
	phandle_t node;
	ssize_t len;

	node = OF_finddevice("/cpus");
	if (node == -1) {
		if (bootverbose)
			printf("fill_elf_hwcap: Can't find cpus node\n");
		return;
	}

	/*
	 * Iterate through the CPUs and examine their ISA string. While we
	 * could assign elf_hwcap to be whatever the boot CPU supports, to
	 * handle the (unusual) case of running a system with hetergeneous
	 * ISAs, keep only the extension bits that are common to all harts.
	 */
	for (node = OF_child(node); node > 0; node = OF_peer(node)) {
		/* Skip any non-CPU nodes, such as cpu-map. */
		if (!ofw_bus_node_is_compatible(node, "riscv"))
			continue;

		len = OF_getprop(node, "riscv,isa", isa, sizeof(isa));
		KASSERT(len <= sizeof(isa), ("ISA string truncated"));
		if (len == -1) {
			if (bootverbose)
				printf("fill_elf_hwcap: "
				    "Can't find riscv,isa property\n");
			return;
		} else if (strncmp(isa, ISA_PREFIX, ISA_PREFIX_LEN) != 0) {
			if (bootverbose)
				printf("fill_elf_hwcap: "
				    "Unsupported ISA string: %s\n", isa);
			return;
		}

		parse_riscv_isa(isa, len, &hwcap);
		if (elf_hwcap != 0)
			elf_hwcap &= hwcap;
		else
			elf_hwcap = hwcap;

		OF_prop_free(isa);
	}
}

SYSINIT(identcpu, SI_SUB_CPU, SI_ORDER_ANY, fill_elf_hwcap, NULL);
#endif

void
identify_cpu(void)
{
	const struct cpu_parts *cpu_partsp;
	struct cpu_desc *desc;
	uint32_t part_id;
	uint64_t misa;
	u_int cpu;
	size_t i;

	cpu_partsp = NULL;

	/* TODO: can we get misa somewhere ? */
	misa = 0;

	cpu = PCPU_GET(cpuid);
	desc = &cpu_desc[cpu];

	for (i = 0; i < nitems(marchid_implementers); i++) {
		if (marchid == marchid_implementers[i].march_id ||
		    marchid_implementers[i].march_id == 0) {
			desc->cpu_marchid = marchid;
			desc->cpu_march_name = marchid_implementers[i].march_name;
			cpu_partsp = cpu_parts_std;
			break;
		}
	}

	for (i = 0; i < nitems(mvendorid_implementers); i++) {
		if (mvendorid == mvendorid_implementers[i].mvendor_id ||
		    mvendorid_implementers[i].mvendor_id == 0) {
			desc->cpu_core_impl = mvendorid;
			desc->cpu_vendor_name =
			    mvendorid_implementers[i].mvendor_name;
			break;
		}
	}

	part_id = CPU_PART(misa);
	for (i = 0; &cpu_partsp[i] != NULL; i++) {
		if (part_id == cpu_partsp[i].part_id ||
		    cpu_partsp[i].part_id == -1) {
			desc->cpu_part_num = part_id;
			desc->cpu_part_name = cpu_partsp[i].part_name;
			break;
		}
	}

	/* Print details for boot CPU or if we want verbose output */
	if (cpu == 0 || bootverbose) {
		if (desc->cpu_core_impl) {
			printf("CPU(%d): %s,"
			    " Core Manufacturer: %s"
			    " Processor Implementation Version: %lu\n",
			    cpu, desc->cpu_march_name, desc->cpu_vendor_name,
			    mimpid);
		} else {
			printf("CPU(%d): %s,"
			" %s\n",
			cpu,
			desc->cpu_march_name,
			desc->cpu_vendor_name);
		}
	}
}
