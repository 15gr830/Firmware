/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************
 *
 * Implementation of an quadrotor attitude control app for use in the
 * semester project.
 *
 */

#include <stdio.h>
#include <poll.h>
#include <mavlink/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/quad_mode.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/quad_velocity_sp.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <lib/mathlib/mathlib.h>

#include "quad_att_control_main.h"
#include "params.h"

/* Function prototypes */
__EXPORT int q_att_control_main(int argc, char *argv[]);
int att_control_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

/**
 * Globals
 */
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;


int q_att_control_thread_main(int argc, char *argv[]) {
        warnx("[q_att_control] has begun");

        static int mavlink_fd;
        mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
        mavlink_log_info(mavlink_fd, "[q_att_control] started");

        struct output_s out;
        memset(&out, 0, sizeof(out));
        struct PID_object_s roll;
        memset(&roll, 0, sizeof(roll));
        struct PID_object_s pitch;
        memset(&pitch, 0, sizeof(pitch));
        struct PID_object_s yaw;
        memset(&yaw, 0, sizeof(yaw));

        /**
         * Subscriptions
         */
        struct quad_velocity_sp_s sp;
        memset(&sp, 0, sizeof(sp));
        struct vehicle_attitude_s v_att;
        memset(&v_att, 0, sizeof(v_att));
        struct vehicle_status_s v_status;
        memset(&v_status, 0, sizeof(v_status));

        int sp_sub = orb_subscribe(ORB_ID(quad_velocity_sp));
        int v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
        int v_status_sub = orb_subscribe(ORB_ID(vehicle_status));

        /**
         * Topics to be published on
         */
        struct actuator_controls_s actuators;
        memset(&actuators, 0, sizeof(actuators));

        orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);

        struct pollfd fd_v_att[1];
        fd_v_att[0].fd = v_att_sub;
        fd_v_att[0].events = POLLIN;

        bool error = false;

        while ( !thread_should_exit ) {
                bool v_status_updated;
                orb_check(v_status_sub, &v_status_updated);
                if ( v_status_updated )
                        orb_copy(ORB_ID(vehicle_status), v_status_sub, &v_status);

                int ret_v_att = poll(fd_v_att, 1, 1);
                if (ret_v_att < 0) {
                        warnx("poll sp error");
                } else if (ret_v_att == 0) {
                        /* no return value - nothing has happened */
                } else if (fd_v_att[0].revents & POLLIN) {
                        orb_copy(ORB_ID(vehicle_attitude), v_att_sub, &v_att);

                        bool sp_updated;
                	orb_check(sp_sub, &sp_updated);

	                if ( sp_updated ) {
	                        orb_copy(ORB_ID(quad_velocity_sp), sp_sub, &sp);
	                }

                        time = (hrt_absolute_time() / (float)1000000);
                        dt = time - time_old;
                        time_old = time;



                        actuators.control[0] = (float)out.roll;
                        actuators.control[1] = (float)out.pitch;
                        actuators.control[2] = (float)out.yaw;
                        actuators.control[3] = (float)out.thrust;

                        orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

                } else {
                        /* nothing happened */
                }
        }
}

static void usage(const char *reason) {
        if (reason)
                fprintf(stderr, "%s\n", reason);

        fprintf(stderr, "usage: q_att_control {start|stop|status}\n\n");
        exit(1);
}

int q_att_control_main(int argc, char *argv[]) {
        if (argc < 1)
                usage("missing argument");

        if (!strcmp(argv[1], "start")) {

                if (thread_running) {
                        printf("q_att_control already running\n");

                        exit(0);
                }

                thread_should_exit = false;
                daemon_task = task_spawn_cmd("q_att_control",
                                             SCHED_DEFAULT,
                                             SCHED_PRIORITY_MAX - 5,
                                             2048,
                                             q_att_control_thread_main,
                                             (argv) ? (const char **)&argv[2] : (const char **)NULL);
                thread_running = true;
                exit(0);
        }

        if (!strcmp(argv[1], "stop")) {
                thread_should_exit = true;
                exit(0);
        }

        if (!strcmp(argv[1], "status")) {
                if (thread_running) {
                        printf("\tatt_control is running\n");

                } else {
                        printf("\tatt_control not started\n");
                }

                exit(0);
        }

        usage("unrecognized command");
        exit(1);
}
