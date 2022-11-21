/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022,2023 The FreeBSD Foundation
 *
 * This software was developed by Mitchell Horne <mhorne@FreeBSD.org>
 * under sponsorship from the FreeBSD Foundation.
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

#include <sys/param.h>
#include <sys/pmc.h>
#include <sys/pmckern.h>

#include <machine/pmc_mdep.h>
#include <machine/sbi.h>
#include <machine/sbi_pmu.h>

static u_int riscv_sbi_npmcs;

/* PCPU structures. */
struct riscv_sbi_cpu {
	struct pmc_hw *pc_phws;
};
static struct riscv_sbi_cpu **riscv_sbi_pcpu;

static int riscv_sbi_allocate_pmc(int cpu, int ri, struct pmc *pm,
    const struct pmc_op_pmcallocate *a);
static int riscv_sbi_release_pmc(int cpu, int ri, struct pmc *pm);
static int riscv_sbi_describe(int cpu, int ri, struct pmc_info *pi,
    struct pmc **ppm);
static int riscv_sbi_get_config(int cpu, int ri, struct pmc **ppm);
static int riscv_sbi_config_pmc(int cpu, int ri, struct pmc *pm);
static int riscv_sbi_read_pmc(int cpu, int ri, struct pmc *pm, pmc_value_t *v);
static int riscv_sbi_write_pmc(int cpu, int ri, struct pmc *pm, pmc_value_t v);
static int riscv_sbi_start_pmc(int cpu, int ri, struct pmc *pm);
static int riscv_sbi_stop_pmc(int cpu, int ri, struct pmc *pm);
static int riscv_sbi_pcpu_init(struct pmc_mdep *md, int cpu);
static int riscv_sbi_pcpu_fini(struct pmc_mdep *md, int cpu);

static uint64_t
read_hpmcounter(int idx)
{
	uint64_t val = 0;

	/*
	 * This is not ideal. But the csrr instruction requires the full name
	 * of the CSR register, not an expression held in a register. So the
	 * switch statement is the best we can do.
	 */
	switch (idx) {
	case 0: val = rdcycle(); break;
	case 1: val = rdtime(); break;
	case 2: val = rdinstret(); break;
	case 3: val = rdhpmcounter(3); break;
	case 4: val = rdhpmcounter(4); break;
	case 5: val = rdhpmcounter(5); break;
	case 6: val = rdhpmcounter(6); break;
	case 7: val = rdhpmcounter(7); break;
	case 8: val = rdhpmcounter(8); break;
	case 9: val = rdhpmcounter(9); break;
	case 10: val = rdhpmcounter(10); break;
	case 11: val = rdhpmcounter(11); break;
	case 12: val = rdhpmcounter(12); break;
	case 13: val = rdhpmcounter(13); break;
	case 14: val = rdhpmcounter(14); break;
	case 15: val = rdhpmcounter(15); break;
	case 16: val = rdhpmcounter(16); break;
	case 17: val = rdhpmcounter(17); break;
	case 18: val = rdhpmcounter(18); break;
	case 19: val = rdhpmcounter(19); break;
	case 20: val = rdhpmcounter(20); break;
	case 21: val = rdhpmcounter(21); break;
	case 22: val = rdhpmcounter(22); break;
	case 23: val = rdhpmcounter(23); break;
	case 24: val = rdhpmcounter(24); break;
	case 25: val = rdhpmcounter(25); break;
	case 26: val = rdhpmcounter(26); break;
	case 27: val = rdhpmcounter(27); break;
	case 28: val = rdhpmcounter(28); break;
	case 29: val = rdhpmcounter(29); break;
	case 30: val = rdhpmcounter(30); break;
	case 31: val = rdhpmcounter(31); break;
	}
	return (val);
}

static int
riscv_sbi_intr(struct trapframe *tf)
{
	struct pmc *pm;
	u_int cpu, ri;
	int error;
	bool found_interrupt = false;

	cpu = curcpu;
	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[riscv-sbi,%d] CPU %d out of range", __LINE__, cpu));

	PMCDBG3(MDP, INT, 1, "cpu=%d tf=%p um=%d", cpu, tf, TRAPF_USERMODE(tf));

	/* TODO: check Sscomfpf support and do proper overflow detection, etc. */
	if (PMC_PROFCLOCK_SAMPLING()) {
		/* Process all sampling PMCs at this time. */
		for (ri = 0; ri < riscv_sbi_npmcs; ri++) {
			pm = riscv_sbi_pcpu[cpu]->pc_phws[ri].phw_pmc;
			if (pm == NULL)
				continue;

			if (!PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pm)))
				continue;

			found_interrupt = true;
			error = pmc_process_interrupt(PMC_HR, pm, tf);
			if (error != 0)
				riscv_sbi_stop_pmc(cpu, ri, pm);

			/* No reloading required. */
		}
	}

	if (found_interrupt)
		counter_u64_add(pmc_stats.pm_intr_processed, 1);
	else
		counter_u64_add(pmc_stats.pm_intr_ignored, 1);

	return (0);
}

static int
riscv_sbi_allocate_pmc(int cpu, int ri, struct pmc *pm,
    const struct pmc_op_pmcallocate *a)
{
	uint64_t config, data;
	int error;

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[riscv-sbi,%d] illegal CPU value %d", __LINE__, cpu));
	KASSERT(ri >= 0 && ri < riscv_sbi_npmcs,
	    ("[riscv-sbi,%d] illegal row index %d", __LINE__, ri));

	if (a->pm_class != PMC_CLASS_RISCV_SBI)
		return (EINVAL);
	/* TODO: */
	if (PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pm)))
		return (EINVAL);

	PMCDBG2(MDP, ALL, 1, "riscv-sbi-allocate ri=%d caps=0x%x", ri,
	    pm->pm_caps);

	config = data = 0;
	error = 0;

	/* Fixed CSR events. Nothing to configure. */
	if (a->pm_ev == PMC_EV_RISCV_SBI_HW_CPU_CYCLES) {
		error = ri == 0 ? 0 : EINVAL;
		goto out;
	} else if (a->pm_ev == PMC_EV_RISCV_SBI_HW_INSTR) {
		error = ri == 2 ? 0 : EINVAL;
		goto out;
	}

	/* Get the event code based on sub-category. */
	switch (a->pm_ev) {
	case PMC_EV_RISCV_SBI_HW_FIRST ... PMC_EV_RISCV_SBI_HW_LAST:
		/* General HW event */
		config = a->pm_ev - PMC_EV_RISCV_SBI_HW_FIRST;
		config |= (SBI_PMU_EVENT_TYPE_HW_GENERAL << 16);
		break;
	case PMC_EV_RISCV_SBI_FW_FIRST ... PMC_EV_RISCV_SBI_FW_LAST:
		/* Firmware event */
		config = a->pm_ev - PMC_EV_RISCV_SBI_FW_FIRST;
		config |= (SBI_PMU_EVENT_TYPE_FW << 16);
		break;
	case PMC_EV_RISCV_SBI_HW_RAW_EVENT:
		/* Raw/user-specified event. The 64-bit CSR value should be in
		 * $fieldname */
		config = 0; /* event_code */
		config |= (SBI_PMU_EVENT_TYPE_HW_RAW << 16); /* event_type */
		data = a->pm_md.pm_riscv.pm_riscv_evsel; /* raw event value */

		/* data has 48-bits only. */
		if (data > 0xffffffffffff)
			return (EINVAL);
	default:
		return (EINVAL);
	}

	pm->pm_md.pm_riscv.pm_riscv_startval = 0;
	if (sbi_pmu_counter_configure(ri, config, data) != 0) {
		return (EINVAL);
	}

out:
	if (error != 0) {
		PMCDBG2(MDP, ALL, 2,
		    "riscv-sbi-allocate ri=%d -> config=0x%lx, data=0x%lx",
		    ri, config);
	}

	return (error);
}

static int
riscv_sbi_release_pmc(int cpu, int ri, struct pmc *pm)
{
	struct pmc_hw *phw __diagused;

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[riscv-sbi,%d] illegal CPU value %d", __LINE__, cpu));
	KASSERT(ri >= 0 && ri < riscv_sbi_npmcs,
	    ("[riscv-sbi,%d] illegal row-index %d", __LINE__, ri));

	/* TODO */
	phw = &riscv_sbi_pcpu[cpu]->pc_phws[ri];
	KASSERT(phw->phw_pmc == NULL,
	    ("[riscv-sbi,%d] PHW pmc %p non-NULL", __LINE__, phw->phw_pmc));

	return (0);
}

static int
riscv_sbi_describe(int cpu, int ri, struct pmc_info *pi, struct pmc **ppm)
{
	struct pmc_hw *phw;

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[riscv-sbi,%d], illegal CPU %d", __LINE__, cpu));
	KASSERT(ri >= 0 && ri < riscv_sbi_npmcs,
	    ("[riscv-sbi,%d] row-index %d out of range", __LINE__, ri));

	snprintf(pi->pm_name, PMC_NAME_MAX, "RISCV-SBI-%d", ri);
	pi->pm_class = PMC_CLASS_RISCV_SBI;

	phw = &riscv_sbi_pcpu[cpu]->pc_phws[ri];
	if (phw->phw_state & PMC_PHW_FLAG_IS_ENABLED) {
		pi->pm_enabled = true;
		*ppm = phw->phw_pmc;
	} else {
		pi->pm_enabled = false;
		*ppm = NULL;
	}

	return (0);
}

static int
riscv_sbi_get_config(int cpu, int ri, struct pmc **ppm)
{

	*ppm = riscv_sbi_pcpu[cpu]->pc_phws[ri].phw_pmc;
	return (0);
}

static int
riscv_sbi_config_pmc(int cpu, int ri, struct pmc *pm)
{
	struct pmc_hw *phw;
	u_long info;
	int error = 0;

	PMCDBG3(MDP, CFG, 1, "cpu=%d ri=%d pm=%p", cpu, ri, pm);

	phw = &riscv_sbi_pcpu[cpu]->pc_phws[ri];

	KASSERT(pm == NULL || phw->phw_pmc == NULL,
	    ("[riscv-sbi,%d] pm=%p phw->pm=%p hwpmc not unconfigured",
	    __LINE__, pm, phw->phw_pmc));

	if (pm != NULL) {
		error = sbi_pmu_counter_get_info(ri, &info);
		if ((info & (0x1ul << 63)) == 0)
			pm->pm_md.pm_riscv.pm_csr = (info & 0xfff) - 0xc00;
		else
			pm->pm_md.pm_riscv.pm_csr = -1;
	}
	if (error == 0)
		phw->phw_pmc = pm;

	return (error);
}

static int
riscv_sbi_read_pmc(int cpu, int ri, struct pmc *pm, pmc_value_t *v)
{
	int csr;
	int error = 0;
	uint64_t val = 0;

	PMCDBG2(MDP, REA, 1, "riscv-sbi-read cpu=%u, id=%d", cpu, ri);

	/* If hw counter, read CSR. */
	csr = pm->pm_md.pm_riscv.pm_csr;
	if (csr != -1) {
		printf("CSR: %d\n", csr);
		*v = read_hpmcounter(csr);
	} else {
		error = sbi_pmu_counter_fw_read(ri, &val);
		*v = val;
	}
	PMCDBG1(MDP, REA, 1, "riscv-sbi-read value=%lx", *v);

	return (error);
}

static int
riscv_sbi_write_pmc(int cpu __unused, int ri __unused,
    struct pmc *pm, pmc_value_t v)
{

	/* Write to software context */
	pm->pm_md.pm_riscv.pm_riscv_startval = v;
	return (0);
}

static int
riscv_sbi_start_pmc(int cpu __diagused, int ri, struct pmc *pm)
{
	int error;

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[riscv-sbi,%d] illegal CPU value %d", __LINE__, cpu));
	KASSERT(ri >= 0 && ri < riscv_sbi_npmcs,
	    ("[riscv-sbi,%d] illegal row-index %d", __LINE__, ri));

	/* TODO: check/return error? */
	error = sbi_pmu_counter_start(ri, pm->pm_md.pm_riscv.pm_riscv_startval);
	if (error == 0 && PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pm))) {
		atomic_add_acq_int(&pmc_profclock_sampling, 1);
	}

	return (error);
}

static int
riscv_sbi_stop_pmc(int cpu __diagused, int ri, struct pmc *pm)
{

	KASSERT(cpu >= 0 && cpu < pmc_cpu_max(),
	    ("[riscv-sbi,%d] illegal CPU value %d", __LINE__, cpu));
	KASSERT(ri >= 0 && ri < riscv_sbi_npmcs,
	    ("[riscv-sbi,%d] illegal row-index %d", __LINE__, ri));

	if (PMC_IS_SAMPLING_MODE(PMC_TO_MODE(pm))) {
		atomic_subtract_rel_int(&pmc_profclock_sampling, 1);
	}
	/* TODO: check/return error? */
	sbi_pmu_counter_stop(ri);

	return (0);
}

static int
riscv_sbi_pcpu_init(struct pmc_mdep *md, int cpu)
{
	struct pmc_cpu *pc;
	struct pmc_hw *phw;
	struct riscv_sbi_cpu *pac;
	u_int first_ri;
	int i;

	PMCDBG1(MDP, INI, 1, "riscv-sbi-pcpu-init cpu=%d", cpu);

	riscv_sbi_pcpu[cpu] = pac = malloc(sizeof(struct riscv_sbi_cpu), M_PMC,
	    M_WAITOK | M_ZERO);

	pac->pc_phws = malloc(sizeof(struct pmc_hw) * riscv_sbi_npmcs,
	    M_PMC, M_WAITOK | M_ZERO);
	pc = pmc_pcpu[cpu];
	KASSERT(pc != NULL, ("[riscv-sbi,%d] NULL per-cpu pointer", __LINE__));

	first_ri = md->pmd_classdep[PMC_MDEP_CLASS_INDEX_RISCV_SBI].pcd_ri;

	for (i = 0, phw = pac->pc_phws; i < riscv_sbi_npmcs; i++, phw++) {
		phw->phw_state = PMC_PHW_FLAG_IS_ENABLED |
		    PMC_PHW_CPU_TO_STATE(cpu) | PMC_PHW_INDEX_TO_STATE(i);
		phw->phw_pmc = NULL;
		pc->pc_hwpmcs[i + first_ri] = phw;
	}

	return (0);
}

static int
riscv_sbi_pcpu_fini(struct pmc_mdep *md, int cpu)
{
	PMCDBG1(MDP,INI,1,"riscv-sbi-pcpu-fini cpu=%d", cpu);

	free(riscv_sbi_pcpu[cpu]->pc_phws, M_PMC);
	free(riscv_sbi_pcpu[cpu], M_PMC);
	riscv_sbi_pcpu[cpu] = NULL;

	return (0);
}

struct pmc_mdep *
pmc_riscv_sbi_initialize(void)
{
	struct pmc_mdep *pmc_mdep;
	struct pmc_classdep *pcd;
	u_int npmcs;

	if (!sbi_has_extension(SBI_EXT_ID_PMU))
		return (NULL);

	/* Query SBI for how many counters we have */
	npmcs = riscv_sbi_npmcs = sbi_pmu_num_counters();
	PMCDBG1(MDP, INI, 1, "riscv-sbi-init npmcs=%d", npmcs);

	/* TODO: alloc pcpu structures */
	riscv_sbi_pcpu = malloc(sizeof(struct riscv_sbi_cpu *) * npmcs, M_PMC,
	    M_WAITOK | M_ZERO);

	/* Just one class */
	pmc_mdep = pmc_mdep_alloc(1);

	/* Set up class descriptor */
	pcd = &pmc_mdep->pmd_classdep[PMC_MDEP_CLASS_INDEX_RISCV_SBI];
	pcd->pcd_caps  = RISCV_PMC_CAPS;
	pcd->pcd_class = PMC_CLASS_RISCV_SBI;
	pcd->pcd_num   = npmcs;
	pcd->pcd_ri    = pmc_mdep->pmd_npmc;
	pcd->pcd_width = 64; /* Uh oh */

	pcd->pcd_allocate_pmc	= riscv_sbi_allocate_pmc;
	pcd->pcd_config_pmc	= riscv_sbi_config_pmc;
	pcd->pcd_pcpu_fini	= riscv_sbi_pcpu_fini;
	pcd->pcd_pcpu_init	= riscv_sbi_pcpu_init;
	pcd->pcd_describe	= riscv_sbi_describe;
	pcd->pcd_get_config	= riscv_sbi_get_config;
	pcd->pcd_read_pmc	= riscv_sbi_read_pmc;
	pcd->pcd_release_pmc	= riscv_sbi_release_pmc;
	pcd->pcd_start_pmc	= riscv_sbi_start_pmc;
	pcd->pcd_stop_pmc	= riscv_sbi_stop_pmc;
	pcd->pcd_write_pmc	= riscv_sbi_write_pmc;

	pmc_mdep->pmd_cputype	= PMC_CPU_RISCV_SBI;
	pmc_mdep->pmd_intr	= NULL;

	pmc_mdep->pmd_npmc += riscv_sbi_npmcs;

	return (pmc_mdep);
}

void
pmc_riscv_sbi_finalize(struct pmc_mdep *md)
{

}
