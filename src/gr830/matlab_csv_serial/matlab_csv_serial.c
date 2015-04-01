/****************************************************************************
 *
 *   Copyright (c) 2014-2015 PX4 Development Team. All rights reserved.		 
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

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
#include <uORB/topics/sensor_combined.h>

__EXPORT int matlab_csv_serial_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

int matlab_csv_serial_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

int matlab_csv_serial_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start"))
	{
		if (thread_running)
		{
			warnx("already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("matlab_csv_serial",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 5,
					 2000,
					 matlab_csv_serial_thread_main,
					 (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop"))
	{
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status"))
	{
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

int matlab_csv_serial_thread_main(int argc, char *argv[])
{

	if (argc < 2) {
		errx(1, "need a serial port name as argument");
	}

	const char* uart_name = argv[1];

	warnx("opening port %s", uart_name);

	int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

	unsigned speed = 921600;

	if (serial_fd < 0) {
		err(1, "failed to open port: %s", uart_name);
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(serial_fd, &uart_config)) < 0) {
		warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
		close(serial_fd);
		return -1;
	}

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* USB serial is indicated by /dev/ttyACM0*/
	if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			warnx("ERR SET BAUD %s: %d\n", uart_name, termios_state);
			close(serial_fd);
			return -1;
		}

	}

	if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR SET CONF %s\n", uart_name);
		close(serial_fd);
		return -1;
	}

	int sub_raw = orb_subscribe(ORB_ID(sensor_combined));

        struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));

        /*This runs at the rate of the sensors */
        struct pollfd fds[] = {
                { .fd = sub_raw, .events = POLLIN }
        };

	thread_running = true;

	while (!thread_should_exit)
	{

		/* wait for a sensor update, check for exit condition every 500 ms */
		int ret = poll(fds, 1, 500);

		if (ret < 0)
		{
			/* poll error, ignore */

		}
		else if (ret == 0)
		{
			/* no return value, ignore */
			warnx("no sensor data");
		}
		else
		{

			/* accel0 update available? */
			if (fds[0].revents & POLLIN)
			{

                                orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);

                                /* Accelerometer data */
                                dprintf(serial_fd, "%llu,%d,%d,%d,%f,%f,%f,%llu,%d,%d,%d,%f,%f,%f,", raw.accelerometer_timestamp, (int)raw.accelerometer_raw[0], (int)raw.accelerometer_raw[1], (int)raw.accelerometer_raw[2], (double)raw.accelerometer_m_s2[0], (double)raw.accelerometer_m_s2[1], (double)raw.accelerometer_m_s2[2], raw.accelerometer1_timestamp, (int)raw.accelerometer1_raw[0], (int)raw.accelerometer1_raw[1], (int)raw.accelerometer1_raw[2], (double)raw.accelerometer1_m_s2[0], (double)raw.accelerometer1_m_s2[1], (double)raw.accelerometer1_m_s2[2]);

                                dprintf(serial_fd, "%llu,%d,%d,%d,%f,%f,%f,", raw.accelerometer2_timestamp, (int)raw.accelerometer2_raw[0], (int)raw.accelerometer2_raw[1], (int)raw.accelerometer2_raw[2], (double)raw.accelerometer2_m_s2[0], (double)raw.accelerometer2_m_s2[1], (double)raw.accelerometer2_m_s2[2]);

                                /* Gyro data */
                                dprintf(serial_fd, "%llu,%d,%d,%d,%f,%f,%f,%llu,%d,%d,%d,%f,%f,%f,", raw.timestamp, (int)raw.gyro_raw[0], (int)raw.gyro_raw[1], (int)raw.gyro_raw[2], (double)raw.gyro_rad_s[0], (double)raw.gyro_rad_s[1], (double)raw.gyro_rad_s[2], raw.gyro1_timestamp, (int)raw.gyro1_raw[0], (int)raw.gyro1_raw[1], (int)raw.gyro1_raw[2], (double)raw.gyro1_rad_s[0], (double)raw.gyro1_rad_s[1], (double)raw.gyro1_rad_s[2]);

                                dprintf(serial_fd, "%llu,%d,%d,%d,%f,%f,%f,%f\n", raw.gyro2_timestamp, (int)raw.gyro2_raw[0], (int)raw.gyro2_raw[1], (int)raw.gyro2_raw[2], (double)raw.gyro2_rad_s[0], (double)raw.gyro2_rad_s[1], (double)raw.gyro2_rad_s[2], (double)raw.baro_temp_celcius);

			}

		}
	}

	warnx("exiting");
	thread_running = false;

	fflush(stdout);
	return 0;
}


