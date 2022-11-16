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

#ifndef _MACHINE_SBI_PMU_H_
#define	_MACHINE_SBI_PMU_H_

#include <sys/types.h>

#include <machine/sbi.h>

/*
 * SBI PMU event types.
 */
enum sbi_pmu_event_type {
	SBI_PMU_EVENT_TYPE_HW_GENERAL	= 0,
	SBI_PMU_EVENT_TYPE_HW_CACHE	= 1,
	SBI_PMU_EVENT_TYPE_HW_RAW	= 2,
	SBI_PMU_EVENT_TYPE_FW		= 15,
};

static __inline void
sbi_pmu_counter_idx_to_bitmask(u_int idx, u_long *base, u_long *mask)
{

	*base = idx / (sizeof(u_long) * NBBY);
	*mask = 0x1ul << (idx % (sizeof(u_long) * NBBY));
}

/*
 * SBI function impls
 */

/* Returns the total number of counters (hw and fw). */
static __inline u_int
sbi_pmu_num_counters(void)
{
	/* Always returns SBI_SUCCESS */
	return ((u_int)SBI_CALL0(SBI_EXT_ID_PMU, SBI_PMU_NUM_COUNTERS).value);
}

#define	COUNTER_INFO_CSR(info)		((info) & 0xfff)
#define	COUNTER_INFO_WIDTH(info)	((((info) & 0x3f000) >> 12) + 1)
#define	COUNTER_INFO_TYPE(info)		\
    (((info) & (0x1ul << (XLEN - 1))) >> (XLEN - 1))

static __inline int
sbi_pmu_counter_get_info(u_int counter_idx, u_long *info)
{
	struct sbi_ret ret;

	ret = SBI_CALL1(SBI_EXT_ID_PMU, SBI_PMU_COUNTER_GET_INFO, counter_idx);
	*info = ret.value;

	return (ret.error);
}

/*
 * Flags
 */
#define	SBI_PMU_CFG_FLAG_SKIP_MATCH	0x01	/* Skip the counter matching */
#define	SBI_PMU_CFG_FLAG_CLEAR_VALUE	0x02	/* Zero the counter value */
#define	SBI_PMU_CFG_FLAG_AUTO_START	0x04	/* Start the counter after configure */
#define	SBI_PMU_CFG_FLAG_SET_VUINH	0x08	/* Inhibit counters in VU-mode */
#define	SBI_PMU_CFG_FLAG_SET_VSINH	0x10	/* Inhibit counters in VS-mode */
#define	SBI_PMU_CFG_FLAG_SET_UINH	0x20	/* Inhibit counters in U-mode */
#define	SBI_PMU_CFG_FLAG_SET_SINH	0x40	/* Inhibit counters in S-mode */
#define	SBI_PMU_CFG_FLAG_SET_MINH	0x80	/* Inhibit counters in M-mode */

static __inline int
sbi_pmu_counter_configure(u_int counter_idx, u_long evsel, u_long data)
{
	struct sbi_ret ret;
	u_long base, mask;
	u_long flags = SBI_PMU_CFG_FLAG_CLEAR_VALUE;

	sbi_pmu_counter_idx_to_bitmask(counter_idx, &base, &mask);
	printf("%s: idx=%u, evsel=%lx, base=%lx, mask=%lx\n",
	    __func__, counter_idx, evsel, base, mask);
	ret = SBI_CALL5(SBI_EXT_ID_PMU, SBI_PMU_COUNTER_CONFIG, base, mask,
	    flags, evsel, data);

	if (ret.error == 0)
		printf("idx=%lu\n", ret.value);
	else
		printf("%s: error=%ld\n", __func__, ret.error);
	return (ret.error);
}

/*
 * Flags
 */
#define	SBI_PMU_START_FLAG_SET_VALUE	0x1

static __inline int
sbi_pmu_counter_start(u_int counter_idx, uint64_t value)
{
	struct sbi_ret ret;
	u_long base, mask;
	u_long flags = SBI_PMU_START_FLAG_SET_VALUE;

	sbi_pmu_counter_idx_to_bitmask(counter_idx, &base, &mask);
	ret = SBI_CALL4(SBI_EXT_ID_PMU, SBI_PMU_COUNTER_START, base, mask,
	    flags, value);

	/* TODO convert to errno values? */
	return (ret.error);
}

#define	SBI_PMU_STOP_FLAG_RESET		0x1

static __inline int
sbi_pmu_counter_stop(u_int counter_idx)
{
	struct sbi_ret ret;
	u_long base, mask;

	sbi_pmu_counter_idx_to_bitmask(counter_idx, &base, &mask);
	ret = SBI_CALL3(SBI_EXT_ID_PMU, SBI_PMU_COUNTER_STOP, base, mask,
	    /* stop_flags */ 0UL);

	/* TODO convert to errno values? */
	return (ret.error);
}

static __inline int
sbi_pmu_counter_fw_read(u_int counter_idx, uint64_t *value)
{
	struct sbi_ret ret;

	ret = SBI_CALL1(SBI_EXT_ID_PMU, SBI_PMU_COUNTER_FW_READ, counter_idx);
	if (ret.error == 0)
		*value = ret.value;

	/* TODO convert to errno values? */
	return (ret.error);
}

#endif /* !_MACHINE_SBI_PMU_H_ */
