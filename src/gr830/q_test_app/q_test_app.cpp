/*************************************************************************
 * Copyright (c) 2015 Group 830 Aalborg University. All rights reserved.
 * Author:   Group 830 <15gr830@es.aau.dk>
 *************************************************************************
 *
 * A small test app to figure out quaternion class
 *
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <poll.h>
#include <lib/mathlib/mathlib.h>

extern "C" __EXPORT int q_test_app_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

int q_test_app_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

int q_test_app_main(int argc, char *argv[]) {
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("q_test_app",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 5,
					 2048,
					 q_test_app_thread_main,
					 (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("running");
		} else {
			warnx("stopped");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int q_test_app_thread_main(int argc, char *argv[]) {
        printf("Hello from test_app\n");
        math::Quaternion q, p, r, t;
        math::Vector<3> qim;
        //q.from_euler(0.f, 90.f, 0.f);
        q = {7,6,9,2};
        p = {0.0412, -0.0353, -0.0529, -0.0118};
        //q.conjugate();
        // q.inv();
        r = p * q;
        t = q * p;
//	while (!thread_should_exit) {
        // qim = q.imag();
        //printf("%f %f %f %f\n", (double)q(0), (double)q(1), (double)q(2), (double)q(3));
        printf("%4.4f %4.4f %4.4f %4.4f\n", (double)r(0), (double)r(1), (double)r(2), (double)r(3));
        printf("%4.4f %4.4f %4.4f %4.4f\n", (double)t(0), (double)t(1), (double)t(2), (double)t(3));
        double lr = r.norm();
        double lt = r.norm();
        printf("%4.4f\n", (double)lr);
        printf("%4.4f\n", (double)lt);
//        }
       
	warnx("exiting");
	thread_running = false;

	return 0;
}

static void usage(const char *reason) {

        if (reason)
		fprintf(stderr, "%s\n", reason);

	errx(1, "usage: test_app {start|stop|status}");
	exit(1);
}
