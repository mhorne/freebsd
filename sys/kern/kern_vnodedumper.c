/*-
 * Copyright (c) 2021 Juniper Networks
 *
 * This software was developed by Mitchell Horne <mhorne@FreeBSD.org>
 * under sponsorship from Juniper Networks and Klara Systems.
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
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/disk.h>
#include <sys/fcntl.h>
#include <sys/kerneldump.h>
#include <sys/malloc.h>
#include <sys/namei.h>
#include <sys/proc.h>
#include <sys/stat.h>
#include <sys/sysctl.h>
#include <sys/vnode.h>

#include <machine/vmparam.h>

static dumper_t vnode_dump;
static dumper_start_t vnode_dumper_start;
static dumper_hdr_t vnode_write_headers;

static int livedump_running;

/*
 * Invoke a live minidump on the system.
 */
static int
sysctl_live_dump(SYSCTL_HANDLER_ARGS)
{
#if MINIDUMP_PAGE_TRACKING == 1
	struct dumperinfo di;
	char path[PATH_MAX];
	int error;

	if (req->newptr == NULL)
		return (EINVAL);
	if (req->newlen == 0)
		return (EINVAL);
	if (req->newlen > sizeof(path))
		return (ENAMETOOLONG);

	error = SYSCTL_IN(req, path, req->newlen);
	if (error != 0)
		return (error);

	/* Set up dumper */
	bzero(&di, sizeof(di));
	di.dumper_start = vnode_dumper_start;
	di.dumper = vnode_dump;
	di.dumper_hdr = vnode_write_headers;
	di.blocksize = PAGE_SIZE; /* Arbitrary. */
	di.maxiosize = MAXDUMPPGS * PAGE_SIZE;
	di.mediasize = 0;
	di.mediaoffset = 0;
	di.priv = path; /* pass path via priv */

	/* Only allow one livedump to proceed at a time. */
	while (!atomic_cmpset_int(&livedump_running, 0, 1))
		pause("livedump", hz / 2);

	dump_savectx();
	error = minidumpsys(&di, true);

	/* Unlock the vnode before departing. */
	if (di.priv != NULL)
		VOP_UNLOCK((struct vnode *)di.priv);
	livedump_running = 0;

	return (error);
#else
	return (EOPNOTSUPP);
#endif /* MINIDUMP_PAGE_TRACKING == 1 */
}

SYSCTL_PROC(_kern, OID_AUTO, livedump,
    CTLTYPE_STRING | CTLFLAG_WR | CTLFLAG_MPSAFE | CTLFLAG_CAPWR,
    NULL, 0, sysctl_live_dump, "A",
    "Start live dump of system");

/*
 * Perform any initialization needed prior to transmitting the kernel core.
 */
int
vnode_dumper_start(struct dumperinfo *di, void *key, uint32_t keysize)
{
	struct nameidata nd;
	char *path;
	int flags;
	int error;

	path = di->priv;
	MPASS(path != NULL);
	di->priv = NULL;

	if (keysize > 0) {
		/* TODO */
		printf("warning: keysize > 0 unhandled\n");
		return (EINVAL);
	}

	/* Instantiate a vnode for provided path */
	flags = FWRITE | O_NOFOLLOW | O_CREAT;
	NDINIT(&nd, LOOKUP, NOFOLLOW | LOCKLEAF, UIO_SYSSPACE, path);

	error = vn_open_cred(&nd, &flags, S_IRUSR | S_IWUSR,
	    VN_OPEN_NOAUDIT | VN_OPEN_NAMECACHE, curthread->td_ucred, NULL);
	if (error != 0) {
		NDFREE(&nd, NDF_NO_DVP_UNLOCK);
		return (error);
	}

	/* Replace priv with the vnode pointer, now that we've obtained it. */
	di->priv = (void *)nd.ni_vp;

	/* Start at offset 0. */
	di->dumpoff = 0;

	return (0);
}

/*
 * Callback from dumpsys() to dump a chunk of memory.
 * Copies it out to our static buffer then sends it across the network.
 * Detects the initial KDH and makes sure it is given a special packet type.
 *
 * Parameters:
 *	arg	 Opaque private pointer to vnode
 *	virtual  Virtual address (where to read the data from)
 *	physical Physical memory address (unused)
 *	offset	 Offset from start of core file
 *	length	 Data length
 *
 * Return value:
 *	0 on success
 *	errno on error
 */
int
vnode_dump(void *arg, void *virtual, vm_offset_t physical, off_t offset, size_t length)
{
	struct vnode *vp;
	int error = 0;

	vp = arg;
	MPASS(vp != NULL);
	ASSERT_VOP_LOCKED(vp, __func__);

	error = vn_rdwr(UIO_WRITE, vp, virtual, length, offset, UIO_SYSSPACE, IO_NODELOCKED,
	    curthread->td_ucred, NOCRED, NULL, curthread);
	if (error != 0)
		printf("vnode_dump error writing to vnode %p\n", vp);

	return (error);
}

int
vnode_write_headers(struct dumperinfo *di, struct kerneldumpheader *kdh)
{
	struct vnode *vp;
	int error;

	vp = di->priv;
	MPASS(vp != NULL);
	ASSERT_VOP_LOCKED(vp, __func__);

	/* Write the kernel dump header to the end of the file. */
	error = vn_rdwr(UIO_WRITE, vp, kdh, sizeof(*kdh), di->dumpoff, UIO_SYSSPACE, IO_NODELOCKED,
	    curthread->td_ucred, NOCRED, NULL, curthread);
	if (error != 0)
		printf("vnode_write_headers error writing to vnode %p: %d\n", vp, error);

	return (0);
}
