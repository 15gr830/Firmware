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
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_local_position.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <lib/mathlib/mathlib.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "q_control_main.hpp"
#include "params.hpp"
#include "lqr.hpp"

/* Function prototypes */
extern "C" __EXPORT int q_control_main(int argc, char *argv[]);
int q_control_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
math::Matrix<4,4> init_act_map(void);
struct output_s act_map_run( math::Matrix<4,4> act_map, math::Vector<4> u );
void out_safety_check( struct output_s *out );

/**
 * Globals
 */
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;


int q_control_thread_main(int argc, char *argv[]) {
        warnx("[q_control] has begun");

        struct output_s out;
        memset(&out, 0, sizeof(out));

        /**
         * Subscriptions
         */
        struct vehicle_attitude_s v_att;
        memset(&v_att, 0, sizeof(v_att));
        struct vehicle_local_position_s v_local_pos;
        memset(&v_local_pos, 0, sizeof(v_local_pos));
        struct vehicle_status_s v_status;
        memset(&v_status, 0, sizeof(v_status));
        struct position_setpoint_s pos_sp;
        memset(&pos_sp, 0, sizeof(pos_sp));
        struct vehicle_command_s cmd;
        memset(&cmd, 0, sizeof(cmd));

        int v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
        int v_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
        int v_status_sub = orb_subscribe(ORB_ID(vehicle_status));
        int pos_sp_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
        int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));

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
        math::Matrix<4,4> act_map;
        act_map = init_act_map();
        // act_map.print();

        bool error = false;
        bool once  = false;
        bool output_on = false;
        bool first = false;

        while ( !thread_should_exit ) {

                bool cmd_updated;
                orb_check(cmd_sub, &cmd_updated);
                if ( cmd_updated ) {
                        orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

                        if ( (cmd.command == (enum VEHICLE_CMD)VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY) && (cmd.param1 <= 0) ) {
                                printf("Output off modtaget\n");
                                output_on = false;
                                first = true;
                        } else if ( (cmd.command == (enum VEHICLE_CMD)VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY) && (cmd.param1 > 0) ) {
                                printf("Output on modtaget\n");
                                output_on = true;
                                first = true;
                        }
                }

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

                        bool v_local_pos_updated; // Position estimates from EKF
                	orb_check(v_local_pos_sub, &v_local_pos_updated);
	                if ( v_local_pos_updated ) {
	                        orb_copy(ORB_ID(vehicle_local_position), v_local_pos_sub, &v_local_pos);
	                }

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
                        x_est[6]  = v_local_pos.x;
                        x_est[7]  = v_local_pos.y;
                        x_est[8]  = v_local_pos.z;
                        x_est[9]  = v_local_pos.vx;
                        x_est[10] = v_local_pos.vy;
                        x_est[11] = v_local_pos.vz;

                        lqr->x_est = x_est; // state vector = [q1 q2 q3 w1 w2 w3 x y z vx vy vz rpm1 rpm2 rpm3 rpm4]^T

                        // Reference state vector
                        x_ref[3]  = 0;        // q1
                        x_ref[4]  = 0;        // q2
                        x_ref[5]  = 0;        // q3
                        x_ref[6]  = pos_sp.x; // setpoint position
                        x_ref[7]  = pos_sp.y; // setpoint position
                        x_ref[8]  = pos_sp.z; // setpoint position
                        x_ref[9]  = 0;        // x velocity
                        x_ref[10] = 0;        // y velocity
                        x_ref[11] = 0;        // z velocity

                        lqr->x_ref = x_ref;
                        
                        u = lqr->run();

                        if (once == false) {
                                lqr->print();
                                once = true;
                        }

                        out = act_map_run(act_map, u);
                        out_safety_check(&out);
                        
                        if ( ( (fabs(v_att.roll) > (double)RP_SAFE) || (fabs(v_att.pitch) > (double)RP_SAFE) || error ) && (v_status.arming_state == ARMING_STATE_ARMED) ) {
                                out.roll = 0;
                                out.pitch = 0;
                                out.yaw = 0;
                                out.thrust = 0;

                                error = true;
                        }

                        // printf("[ %4.4f %4.4f %4.4f %4.4f ]\n", (double)out.thrust, (double)out.roll, (double)out.pitch, (double)out.yaw);

                        actuators.control[0] = (float)out.roll;
                        actuators.control[1] = (float)out.pitch;
                        actuators.control[2] = (float)out.yaw;
                        actuators.control[3] = (float)out.thrust;

                        if ( output_on ) {
                                orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
                                if ( first ) {
                                        printf("Motor on\n");
                                        first = false;
                                }

                        } else if ( !output_on ) {
                                if ( first ) {
                                        printf("Motor off\n");
                                        first = false;
                                }

                                for (int i = 0; i < 4; i++)
                                        actuators.control[i] = (double)0;

                                orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
                        }
                } else {
                        /* nothing happened */
                }
        }
        delete lqr;
        return -1;
}


// TODO: De rigtige værdier skal findes til sikkerhed
/* Limiting attitude controllers output */
void out_safety_check( struct output_s *out ) {
        if ( (double)fabs(out->roll) > (double)RP_MAX )
                out->roll = (float)RP_MAX * (out->roll / (float)fabs(out->roll));

        if ( (double)fabs(out->pitch) > (double)RP_MAX )
                out->pitch = (float)RP_MAX * (out->pitch / (float)fabs(out->pitch));

        if ( (double)fabs(out->yaw) > (double)YAW_MAX )
                out->yaw = (float)YAW_MAX * (out->yaw / (float)fabs(out->yaw));

}


struct output_s act_map_run( math::Matrix<4,4> act_map, math::Vector<4> u ) {
        struct output_s out_scaled;

        out_scaled.thrust = act_map.data[0][0]*u.data[0] + act_map.data[0][1]*u.data[1] + act_map.data[0][2]*u.data[2] + act_map.data[0][3]*u.data[3];
        out_scaled.roll   = act_map.data[1][0]*u.data[0] + act_map.data[1][1]*u.data[1] + act_map.data[1][2]*u.data[2] + act_map.data[1][3]*u.data[3];
        out_scaled.pitch  = act_map.data[2][0]*u.data[0] + act_map.data[2][1]*u.data[1] + act_map.data[2][2]*u.data[2] + act_map.data[2][3]*u.data[3];
        out_scaled.yaw    = act_map.data[3][0]*u.data[0] + act_map.data[3][1]*u.data[1] + act_map.data[3][2]*u.data[2] + act_map.data[3][3]*u.data[3];

        return out_scaled;
}


math::Matrix<4,4> init_act_map(void) {
        math::Matrix<4,4> act_map_temp;
        act_map_temp.data[0][0] = (double)ACT_MAP_1_1;
        act_map_temp.data[0][1] = (double)ACT_MAP_1_2;
        act_map_temp.data[0][2] = (double)ACT_MAP_1_3;
        act_map_temp.data[0][3] = (double)ACT_MAP_1_4;

        act_map_temp.data[1][0] = (double)ACT_MAP_2_1;
        act_map_temp.data[1][1] = (double)ACT_MAP_2_2;
        act_map_temp.data[1][2] = (double)ACT_MAP_2_3;
        act_map_temp.data[1][3] = (double)ACT_MAP_2_4;

        act_map_temp.data[2][0] = (double)ACT_MAP_3_1;
        act_map_temp.data[2][1] = (double)ACT_MAP_3_2;
        act_map_temp.data[2][2] = (double)ACT_MAP_3_3;
        act_map_temp.data[2][3] = (double)ACT_MAP_3_4;

        act_map_temp.data[3][0] = (double)ACT_MAP_4_1;
        act_map_temp.data[3][1] = (double)ACT_MAP_4_2;
        act_map_temp.data[3][2] = (double)ACT_MAP_4_3;
        act_map_temp.data[3][3] = (double)ACT_MAP_4_4;

        return act_map_temp;
}


static void usage(const char *reason) {
        if (reason)
                fprintf(stderr, "%s\n", reason);

        fprintf(stderr, "usage: q_control {start|stop|status}\n\n");
        exit(1);
}


int q_control_main(int argc, char *argv[]) {
        if (argc < 1)
                usage("missing argument");

        if (!strcmp(argv[1], "start")) {

                if (thread_running) {
                        printf("q_control already running\n");

                        exit(0);
                }

                thread_should_exit = false;
                daemon_task = task_spawn_cmd("q_control",
                                             SCHED_DEFAULT,
                                             SCHED_PRIORITY_MAX - 5,
                                             2048,
                                             q_control_thread_main,
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
                        printf("\tq_control is running\n");

                } else {
                        printf("\tq_control not started\n");
                }

                exit(0);
        }

        usage("unrecognized command");
        exit(1);
}
