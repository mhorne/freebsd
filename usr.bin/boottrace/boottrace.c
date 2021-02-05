
#include <sys/types.h>
#include <sys/sysctl.h>
#include <sys/wait.h>

#include <err.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define	BOOTTRACE_SYSCTL_RUNTRACE	"kern.boottrace.runtimes"

static void
usage(void)
{
	fprintf(stderr, "usage: boottrace utility [argument ...]\n");
	exit(1);
}

int
main(int argc, char **argv)
{
	pid_t pid;
	char msg[40];
	size_t n;
	int status;

	for (int i = 0; i < argc; i++) {
		printf("argv[%d]: %s\n", i, argv[i]);
	}

	if (argc < 2)
		usage();
	argv++;

	n = strlcpy(msg, *argv, sizeof(msg));
	if (n >= sizeof(msg))
		msg[39] = '\0';
	else
		n = strlcat(msg + n, " start", sizeof(msg) - n);
	if (sysctlbyname(BOOTTRACE_SYSCTL_RUNTRACE, NULL, 0, msg, sizeof(msg)) == -1) {
		fprintf(stderr, "sysctlbyname failed: %s\n", strerror(errno));
	}
	pid = fork();
	if (pid == -1) {
		exit(1);
	} else if (pid == 0) {
		execvp(*argv, argv);
		err(1, "execvp %s", *argv);
	}
	waitpid(pid, &status, 0);
	if ( ! WIFEXITED(status))
		warnx("command terminated abnormally");
	n = strlcpy(msg, *argv, sizeof(msg));
	n = strlcat(msg + n, " done", sizeof(msg) - n);
	if (sysctlbyname(BOOTTRACE_SYSCTL_RUNTRACE, NULL, 0, msg, sizeof(msg)) == -1) {
		fprintf(stderr, "sysctlbyname failed: %s\n", strerror(errno));
	}

	return (0);
}
