.include <bsd.own.mk>

ATF_TESTS_C+=		arc4random_test
ATF_TESTS_C+=		dir2_test
ATF_TESTS_C+=		dlopen_empty_test
ATF_TESTS_C+=		fmtcheck2_test
ATF_TESTS_C+=		fmtmsg_test
ATF_TESTS_C+=		fnmatch2_test
ATF_TESTS_C+=		fpclassify2_test
.if ${COMPILER_FEATURES:Mblocks}
ATF_TESTS_C+=		fts_blocks_test
.endif
ATF_TESTS_C+=		fts_misc_test
ATF_TESTS_C+=		fts_options_test
ATF_TESTS_C+=		ftw_test
ATF_TESTS_C+=		getentropy_test
ATF_TESTS_C+=		getmntinfo_test
ATF_TESTS_C+=		glob2_test
.if ${COMPILER_FEATURES:Mblocks}
ATF_TESTS_C+=		glob_blocks_test
.endif
ATF_TESTS_C+=		makecontext_test
ATF_TESTS_C+=		popen_test
ATF_TESTS_C+=		posix_spawn_test
ATF_TESTS_C+=		realpath2_test
ATF_TESTS_C+=		scandir_test
.if ${COMPILER_FEATURES:Mblocks}
ATF_TESTS_C+=		scandir_blocks_test
.endif
ATF_TESTS_C+=		sig2str_test
ATF_TESTS_C+=		sigsetops_test
ATF_TESTS_C+=		wordexp_test

# TODO: t_closefrom, t_fmtcheck, t_randomid,
# TODO: t_siginfo (fixes require further inspection)
# TODO: t_sethostname_test (consistently screws up the hostname)

FILESGROUPS+=		posix_spawn_test_FILES

posix_spawn_test_FILES=	spawnp_enoexec.sh
posix_spawn_test_FILESDIR=	${TESTSDIR}
posix_spawn_test_FILESMODE= 0755
posix_spawn_test_FILESOWN= root
posix_spawn_test_FILESGRP= wheel
posix_spawn_test_FILESPACKAGE=	${PACKAGE}

CFLAGS+=	-DTEST_LONG_DOUBLE

# Define __HAVE_LONG_DOUBLE for architectures whose long double has greater
# precision than their double.
.if ${MACHINE_CPUARCH} == "aarch64" || \
    ${MACHINE_CPUARCH} == "amd64" || \
    ${MACHINE_CPUARCH} == "i386" || \
    ${MACHINE_CPUARCH} == "riscv"
CFLAGS+=	-D__HAVE_LONG_DOUBLE
.endif

NETBSD_ATF_TESTS_C=	alarm_test
NETBSD_ATF_TESTS_C+=	assert_test
NETBSD_ATF_TESTS_C+=	basedirname_test
NETBSD_ATF_TESTS_C+=	cpuset_test
NETBSD_ATF_TESTS_C+=	dir_test
NETBSD_ATF_TESTS_C+=	floatunditf_test
NETBSD_ATF_TESTS_C+=	fnmatch_test
NETBSD_ATF_TESTS_C+=	fpclassify_test
NETBSD_ATF_TESTS_C+=	fpsetmask_test
NETBSD_ATF_TESTS_C+=	fpsetround_test
NETBSD_ATF_TESTS_C+=	ftok_test
NETBSD_ATF_TESTS_C+=	getcwd_test
NETBSD_ATF_TESTS_C+=	getgrent_test
NETBSD_ATF_TESTS_C+=	glob_test
NETBSD_ATF_TESTS_C+=	humanize_number_test
NETBSD_ATF_TESTS_C+=	isnan_test
NETBSD_ATF_TESTS_C+=	nice_test
NETBSD_ATF_TESTS_C+=	pause_test
NETBSD_ATF_TESTS_C+=	raise_test
NETBSD_ATF_TESTS_C+=	realpath_test
NETBSD_ATF_TESTS_C+=	setdomainname_test
NETBSD_ATF_TESTS_C+=	sethostname_test
NETBSD_ATF_TESTS_C+=	sleep_test
NETBSD_ATF_TESTS_C+=	syslog_test
NETBSD_ATF_TESTS_C+=	time_test
NETBSD_ATF_TESTS_C+=	ttyname_test
NETBSD_ATF_TESTS_C+=	vis_test

.include "../Makefile.netbsd-tests"

CFLAGS.getentropy_test+=	-I${SRCTOP}/include
LIBADD.getentropy_test+=	c
LIBADD.humanize_number_test+=	util

LIBADD.fpclassify_test+=m
LIBADD.fpsetround_test+=m
LIBADD.siginfo_test+=	m

LIBADD.nice_test+=	pthread
LIBADD.syslog_test+=	pthread

CFLAGS+=		-I${.CURDIR}

SRCS.fmtcheck2_test=	fmtcheck_test.c
SRCS.fnmatch2_test=	fnmatch_test.c

TEST_METADATA.setdomainname_test+=	is_exclusive=true
TESTS_SUBDIRS=	execve
TESTS_SUBDIRS+=	posix_spawn

# Tests that require address sanitizer
.if ${COMPILER_FEATURES:Masan}
.for t in scandir_test realpath2_test
CFLAGS.${t}.c+=		-fsanitize=address
LDFLAGS.${t}+=		-fsanitize=address
.endfor
.endif

# Tests that require blocks support
.for t in fts_blocks_test glob_blocks_test scandir_blocks_test
CFLAGS.${t}.c+=		-fblocks
LIBADD.${t}+=		BlocksRuntime
.endfor

# The old testcase name
TEST_FNMATCH=	test-fnmatch
CLEANFILES+=		${GEN_SH_CASE_TESTCASES}
sh-tests: .PHONY
.for target in clean obj depend all
	@cd ${.CURDIR} && ${MAKE} PROG=${TEST_FNMATCH} \
	    -DNO_SUBDIR ${target}
.endfor
	@cd ${.OBJDIR} && ./${TEST_FNMATCH} -s 1 > \
	    ${SRCTOP}/bin/sh/tests/builtins/case2.0
	@cd ${.OBJDIR} && ./${TEST_FNMATCH} -s 2 > \
	    ${SRCTOP}/bin/sh/tests/builtins/case3.0

.include <bsd.test.mk>
