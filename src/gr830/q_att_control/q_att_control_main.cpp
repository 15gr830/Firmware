/*************************************************************************
 * Copyright (c) 2015 Group 830 Aalborg University. All rights reserved.
 * Author:   Group 830 <15gr830@es.aau.dk>
 *************************************************************************
 *
 * Implementation of an quadrotor attitude control app for use in the
 * semester project. 
 *
 */

#include <stdio.h>
#include <poll.h>
#include <math.h>
#include <stdlib.h>
#include <mavlink/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <lib/mathlib/mathlib.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "q_att_control_main.hpp"
#include "params.hpp"
#include "lqr.hpp"

/* Function prototypes */
extern "C" __EXPORT int q_att_control_main(int argc, char *argv[]);
int q_att_control_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

/**
 * Globals
 */
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;


int q_att_control_thread_main(int argc, char *argv[]) {
        warnx("[q_att_control] has begun");

        struct output_s out;
        memset(&out, 0, sizeof(out));

        /**
         * Subscriptions
         */
        struct vehicle_attitude_s v_att; // FIXME: Indeholder ikke positione og hastigheder
        memset(&v_att, 0, sizeof(v_att));
        struct vehicle_status_s v_status;
        memset(&v_status, 0, sizeof(v_status));
        struct vehicle_control_mode_s control_mode;
        memset(&control_mode, 0, sizeof(control_mode));
        struct position_setpoint_s pos_sp;
        memset(&pos_sp, 0, sizeof(pos_sp));

        int v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
        int v_status_sub = orb_subscribe(ORB_ID(vehicle_status));
        int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
        int pos_sp_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));

        /**
         * Topics to be published on
         */
        struct actuator_controls_s actuators;
        memset(&actuators, 0, sizeof(actuators));

        orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);

        struct pollfd fd_v_att[1];
        fd_v_att[0].fd = v_att_sub;
        fd_v_att[0].events = POLLIN;

        Lqr *lqr = new Lqr;
        float x_est[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        float x_ref[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

        // ekstra references to LQR
        lqr->r = {0,0,0,0};
        lqr->q_ref->data[0] = 1;
        lqr->q_ref->data[1] = 0;
        lqr->q_ref->data[2] = 0;
        lqr->q_ref->data[3] = 0;

        math::Vector<4> u,id;
        math::Matrix<4,4> *act_scale = new math::Matrix<4,4>;
        act_scale->identity();

        bool  error   = false;
        float rp_max  = 0.8, // roll and pitch maximum actuator output
              yaw_max = 0.8, // yaw maximum actuator output
              rp_safe = 0.8;

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
                        // TODO: orb_copy positioner og hastigheder (hvilket topic?)

                        bool pos_sp_updated; // Position setpoint from gnd
                	orb_check(pos_sp_sub, &pos_sp_updated);
	                if ( pos_sp_updated ) {
	                        orb_copy(ORB_ID(position_setpoint_triplet), pos_sp_sub, &pos_sp);
	                }

                        for (int i = 0; i < 4; i++)
                                lqr->q_est->data[i] = v_att.q[i];

                        // x_est[0] to x_est[2] is calculated in the lqr class
                        x_est[3]  = v_att.rollspeed;
                        x_est[4]  = v_att.pitchspeed;
                        x_est[5]  = v_att.yawspeed;
                        x_est[6]  = 0; // FIXME: xyz positioner
                        x_est[7]  = 0;
                        x_est[8]  = 0;
                        x_est[9]  = 0; // FIXME: xyz hastigheder
                        x_est[10] = 0;
                        x_est[12] = 0;

                        lqr->x_est = x_est; // state vector = [q1 q2 q3 w1 w2 w3 x y z vx vy vz rpm1 rpm2 rpm3 rpm4]^T

                        // x_ref[0] to x_ref[2] is calculated in the lqr class
                        x_ref[3]  = 0;
                        x_ref[4]  = 0;
                        x_ref[5]  = 0;
                        x_ref[6]  = pos_sp.x;
                        x_ref[7]  = pos_sp.y;
                        x_ref[8]  = pos_sp.z;
                        x_ref[9]  = 0;
                        x_ref[10] = 0;
                        x_ref[12] = 0;

                        lqr->x_ref = x_ref;

                        u = lqr->run(); 

                        out.roll   = act_scale->data[0][0]*u.data[0] + act_scale->data[0][1]*u.data[1] + act_scale->data[0][2]*u.data[2] + act_scale->data[0][3]*u.data[3];
                        out.pitch  = act_scale->data[1][0]*u.data[0] + act_scale->data[1][1]*u.data[1] + act_scale->data[1][2]*u.data[2] + act_scale->data[1][3]*u.data[3];
                        out.yaw    = act_scale->data[2][0]*u.data[0] + act_scale->data[2][1]*u.data[1] + act_scale->data[2][2]*u.data[2] + act_scale->data[2][3]*u.data[3];
                        out.thrust = act_scale->data[3][0]*u.data[0] + act_scale->data[3][1]*u.data[1] + act_scale->data[3][2]*u.data[2] + act_scale->data[3][3]*u.data[3];
                        
                        // TODO: De rigtige værdier skal findes til sikkerhed
                        /* Limiting attitude controllers output */
                        if ( (float)fabs(out.roll) > rp_max )
                                out.roll = rp_max * (out.roll / (float)fabs(out.roll));

                        if ( (float)fabs(out.pitch) > rp_max )
                                out.pitch = rp_max * (out.pitch / (float)fabs(out.pitch));

                        if ( (float)fabs(out.yaw) > yaw_max )
                                out.yaw = yaw_max * (out.yaw / (float)fabs(out.yaw));

                        if ( ( ((float)fabs(v_att.roll) > rp_safe) || ((float)fabs(v_att.pitch) > rp_safe) || error ) && (v_status.arming_state == ARMING_STATE_ARMED) ) {
                                out.roll = 0;
                                out.pitch = 0;
                                out.yaw = 0;
                                out.thrust = 0;

                                error = true;
                        }
                        
                        actuators.control[0] = (float)out.roll;
                        actuators.control[1] = (float)out.pitch;
                        actuators.control[2] = (float)out.yaw;
                        actuators.control[3] = (float)out.thrust;

                        bool control_mode_updated;
                	orb_check(control_mode_sub, &control_mode_updated);
	                if ( control_mode_updated ) {
	                        orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);
	                }

                        if (control_mode.flag_control_altitude_enabled) {
                                orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
                        }

                } else {
                        /* nothing happened */
                }
        }
        delete lqr;
        return -1;
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
                                             (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
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
