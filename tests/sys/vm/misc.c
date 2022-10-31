/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) The FreeBSD Foundation
 *
 * Portions of this software were developed by Mitchell Horne under sponsorship
 * from the FreeBSD Foundation.
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

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/sysctl.h>
#include <sys/wait.h>

#include <atf-c.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define PAGE_SIZE	4096

/* Larger than one superpage: 2MiB + 7 * 4KiB */
#define	MAP_LENGTH	((512 + 7) * PAGE_SIZE)

/* Keep this global so the memcpy is not optimized away. */
extern char smallbuf[16];
char smallbuf[16];

static void
child_run(int fd)
{
	char *addr, *cpyaddr;

	/* mmap the first MAP_LENGTH bytes of the file into the process. */
	addr = mmap(NULL, MAP_LENGTH, PROT_READ, MAP_PRIVATE, fd, 0);
	if (addr == MAP_FAILED)
		err(1, "%s: mmap failed", __func__);

	/* Now madvise() with WILLNEED, so that the mappings are (re-)created. */
	if (madvise(addr, MAP_LENGTH, MADV_WILLNEED) == -1)
		err(1, "%s: madvise(MADV_WILLNEED) failed", __func__);

	close(fd);
	char vec = 0;
	if (mincore(addr, PAGE_SIZE, &vec) == -1)
		err(1, "%s: mincore(2) failed", __func__);

	if ((vec & MINCORE_INCORE) == 0) {
		/* Read a few bytes from every page in the mapped region, so that they
		 * are paged in. */
		for (cpyaddr = addr; cpyaddr < (addr + MAP_LENGTH);
		    cpyaddr += PAGE_SIZE) {
			memcpy(smallbuf, cpyaddr, sizeof(smallbuf));
		}
		exit(1);
	} else if ((vec & MINCORE_PSIND(1)) == 0) {
		/* Didn't get a superpage mapping. Retry. */
		exit(1);
	}

	if (munmap(addr, MAP_LENGTH) == -1) {
		err(1, "%s: munmap failed", __func__);
	}

	/*
	 * End process. The panic should manifest during the
	 * pmap_remove_pages() cleanup.
	 */
	exit(0);
}

#define	MAPFILE_NAME	"/tmp/mapfile"
#define	MAPFILE_SIZE	(8 * (1 << 20)) /* 8MB */
#define NRETRIES	5

ATF_TC_WITHOUT_HEAD(misc__madvise_willneed_munmap_regression);
ATF_TC_BODY(misc__madvise_willneed_munmap_regression, tc)
{
	pid_t pid;
	int mapfd;
	int status;
	char *buf;
	size_t len;

	len = sizeof(int);
	if (getpagesizes(NULL, 0) < 2) {
		atf_tc_skip("superpages unsupported or disabled\n");
		return;
	}

	/* Preamble: create the zero-filled temp file. Use write(2) rather than
	 * ftruncate(2) so it is truly zero-filled. */
	buf = malloc(PAGE_SIZE);
	ATF_REQUIRE_MSG(buf != NULL, "malloc() failed");

	mapfd = open(MAPFILE_NAME, O_RDWR | O_CREAT | O_TRUNC,
	    S_IWUSR | S_IRUSR);
	ATF_REQUIRE_MSG(mapfd != -1, "failed to open mapfile: %s", strerror(errno));

	bzero(buf, PAGE_SIZE);
	for (len = 0; len < MAPFILE_SIZE;) {
		len += write(mapfd, buf, PAGE_SIZE);
	}

	free(buf);
	close(mapfd);

	/* Now open it again, read-only. */
	mapfd = open(MAPFILE_NAME, O_RDONLY);
	ATF_REQUIRE_MSG(mapfd != -1, "failed to open mapfile: %s",
	    strerror(errno));

	for (int i = 0; i < NRETRIES; i++) {
		pid = fork();
		ATF_REQUIRE_MSG(pid != -1, "fork() failed: %s", strerror(errno));

		if (pid == 0) {
			child_run(mapfd);
		} else {
			waitpid(pid, &status, 0);
			printf("child finished: pid=%d, status=%d\n", pid, WEXITSTATUS(status));
			if (WIFEXITED(status) && WEXITSTATUS(status) == 0)
				break;
		}
	}

	unlink(MAPFILE_NAME);
}

ATF_TP_ADD_TCS(tp)
{

	ATF_TP_ADD_TC(tp, misc__madvise_willneed_munmap_regression);

	return (atf_no_error());
}
