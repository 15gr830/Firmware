/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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

/*
 * Implementation of Extended Kalman Filters for both position and
 * attitude, for use in the semester project
 * Adapted by 15gr830 Aalborg university
 *
 * Originally written by:
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

/* Ved nygenereret Kalman kode:
 *
 * memcpy er ændret til loops i de autogenererede kalman filtre
 * dataen bliver ødelagt når der benyttes memcpy?
 *
 * tjek i den autogenerede kode om der er husket alle paranteser ved 
 * generering af quaternion
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/att_pos_mocap.h>
#include <drivers/drv_hrt.h>

#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "att_codegen/AttitudeEKF2grav.h"
#include "pos_codegen/posEKF.h"
#include "q_ekf_params.h"
#ifdef __cplusplus
}
#endif

// #define Q_EKF_ATT_DEBUG
// #define Q_EKF_POS_DEBUG

extern "C" __EXPORT int q_ekf_main(int argc, char *argv[]);

static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int attitude_estimator_ekf_task;				/**< Handle of deamon task / thread */

/**
 * Mainloop of attitude_estimator_ekf.
 */
int q_ekf_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);
math::Matrix<3,3> quat2Rot(float q[4]);
math::Vector<3> matVectMult(math::Matrix<3,3> mat, math::Vector<3> v);
math::Vector<4> qmult( float *q, float *p );

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: q_ekf {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The attitude_estimator_ekf app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_spawn_cmd().
 */
int q_ekf_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("q_ekf already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		attitude_estimator_ekf_task = task_spawn_cmd("q_ekf",
                                                             SCHED_DEFAULT,
                                                             SCHED_PRIORITY_MAX - 5,
                                                             7700,
                                                             q_ekf_thread_main,
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
			exit(0);

		} else {
			warnx("not started");
			exit(1);
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

/*
 * [Rot_matrix,x_aposteriori,P_aposteriori] = attitudeKalmanfilter(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst)
 */

/*
 * EKF Attitude Estimator main function.
 *
 * Estimates the attitude recursively once started.
 *
 * @param argc number of commandline arguments (plus command name)
 * @param argv strings containing the arguments
 */
int q_ekf_thread_main(int argc, char *argv[])
{

        // const unsigned int loop_interval_alarm = 6500;	// loop interval in microseconds

	float dt = 0.005f;
/* state vector x has the following entries [ax,ay,az||mx,my,mz||wox,woy,woz||wx,wy,wz]' */
	float z_k[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -9.81f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; /**< Measurement vector */
	float x_aposteriori_k[12];		/**< states */
	float P_aposteriori_k[144] = {100.f,  0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                                      0,      100.f,  0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                                      0,      0,      100.f,  0,      0,      0,      0,      0,      0,      0,      0,      0,
                                      0,      0,      0,      100.f,  0,      0,      0,      0,      0,      0,      0,      0,
                                      0,      0,      0,      0,      100.f,  0,      0,      0,      0,      0,      0,      0,
                                      0,      0,      0,      0,      0,      100.f,  0,      0,      0,      0,      0,      0,
                                      0,      0,      0,      0,      0,      0,      100.f,  0,      0,      0,      0,      0,
                                      0,      0,      0,      0,      0,      0,      0,      100.f,  0,      0,      0,      0,
                                      0,      0,      0,      0,      0,      0,      0,      0,      100.f,  0,      0,      0,
                                      0,      0,      0,      0,      0,      0,      0,      0,      0.0f,   100.0f, 0,      0,
                                      0,      0,      0,      0,      0,      0,      0,      0,      0.0f,   0,      100.0f, 0,
                                      0,      0,      0,      0,      0,      0,      0,      0,      0.0f,   0,      0,      100.0f,
        }; /**< init: diagonal matrix with big values */
        float x_aposteriori[12];
	float P_aposteriori[144];

        double z_pos_k[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        float x_pos_apo_k[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        float P_pos_apo_k[81] = {100.f, 0,     0,     0,     0,     0,     0,     0,     0, 
                                 0,     100.f, 0,     0,     0,     0,     0,     0,     0,
                                 0,     0,     100.f, 0,     0,     0,     0,     0,     0,
                                 0,     0,     0,     100.f, 0,     0,     0,     0,     0,
                                 0,     0,     0,     0,     100.f, 0,     0,     0,     0,
                                 0,     0,     0,     0,     0,     100.f, 0,     0,     0,
                                 0,     0,     0,     0,     0,     0,     100.f, 0,     0,
                                 0,     0,     0,     0,     0,     0,     0,     100.f, 0,
                                 0,     0,     0,     0,     0,     0,     0,     0,     100.f
        };
        float x_pos_aposteriori[9];
	float P_pos_aposteriori[81];

	/* output euler angles */
	float euler[3] = {0.0f, 0.0f, 0.0f};

	float Rot_matrix[9] = {1.f,  0,  0,
                               0,  1.f,  0,
                               0,  0,  1.f
        };		/**< init: identity matrix */

	float debugOutput[4] = { 0.0f };
        float q_att[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	// int overloadcounter = 19;

        // printf("1\n");

	/* Initialize filters */
	AttitudeEKF2grav_initialize();
        posEKF_initialize();

        // printf("2\n");

	/* store start time to guard against too slow update rates */
	// uint64_t last_run = hrt_absolute_time();

	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));
        struct att_pos_mocap_s got_pos; //  GOT position floats x, y and z
        memset(&got_pos, 0, sizeof(got_pos));
        struct vision_position_estimate ptam; // data from ptam (xyz and quaternion); floats x, y,z and q[4]
        memset(&ptam, 0, sizeof(ptam));
        struct vehicle_local_position_s pos;
        memset(&pos, 0, sizeof(pos));

	uint64_t last_data = 0;
	uint64_t last_measurement = 0;
	// uint64_t last_vel_t = 0;

	/* current velocity */
	math::Vector<3> vel;
	vel.zero();
	/* previous velocity */
	math::Vector<3> vel_prev;
	vel_prev.zero();
	/* actual acceleration (by GPS velocity) in body frame */
	math::Vector<3> acc;
	acc.zero();
	/* rotation matrix */
	math::Matrix<3, 3> R;
	R.identity();

	/* subscribe to raw data */
	int sub_raw = orb_subscribe(ORB_ID(sensor_combined));
	/* rate-limit raw data updates to 333 Hz (sensors app publishes at 200, so this is just paranoid) */
	orb_set_interval(sub_raw, 3);

	/* subscribe to param changes */
	int sub_params = orb_subscribe(ORB_ID(parameter_update));

	/* subscribe to control mode */
	int sub_control_mode = orb_subscribe(ORB_ID(vehicle_control_mode));

        /* subscribe to ptam data */
        int ptam_sub = orb_subscribe(ORB_ID(vision_position_estimate));

        /* subscribe to data from GOT */
        int got_pos_sub = orb_subscribe(ORB_ID(att_pos_mocap));

	/* advertise attitude */
	orb_advert_t pub_att = orb_advertise(ORB_ID(vehicle_attitude), &att);
        
        /* advertise vehicle position */
        orb_advert_t pub_pos = orb_advertise(ORB_ID(vehicle_local_position), &pos);

	int loopcounter = 0;

	thread_running = true;

	/* keep track of sensor updates */
	uint64_t sensor_last_timestamp[4] = {0, 0, 0, 0};//[gyro, acc, ptam, got]

	struct attitude_estimator_ekf_params ekf_params;
	memset(&ekf_params, 0, sizeof(ekf_params));

	struct attitude_estimator_ekf_param_handles ekf_param_handles = { 0 };

	/* initialize parameter handles */
	parameters_init(&ekf_param_handles);

	bool initialized = false;

	float gyro_offsets[3] = { 0.0f, 0.0f, 0.0f };

	/* magnetic declination, in radians */
	// float mag_decl = 0.0f;

	/* rotation matrix for magnetic declination */
	// math::Matrix<3, 3> R_decl;
	// R_decl.identity();

        double q_pos_acc     = 0.0001,
               q_pos_speed   = 0.08,
               q_pos         = 0.009,
               r_pos_acc     = 10.f,
               r_pos_ptam[3] = {0.1f, 0.1f, 0.1f},
                r_pos_got     = 0.0001;//,
                //rad2deg       = 57.2957914331;
        
        float debug_pos[4]  = {0, 0, 0, 0};

        math::Matrix<3,3> R_n;
        R.identity();
        float q_h2c[4] = {0.6124, -0.3536, -0.3536, -0.6124};
        float q_b2c_conj[4] = {0.6124, 0.3536, 0.3536, 0.6124};
        float q_h2ptam[4] = {1, 0, 0, 0};
        float q_h2b[4] = {1, 0, 0, 0};
        float q_ptam2b[4] = {1, 0, 0, 0};
        float q_ptam_conj[4] = {1, 0, 0, 0};
        math::Vector<4> q_temp;
        math::Vector<3> mag_in_h = {1, 0, 0}, grav_in_h = {0, 0, -9.82f}, gm, gg;
        bool ptam_initialized = false;
        math::Matrix<3,3> rot_b2c_pos;
        math::Vector<3> ptam_pos_vect;
        math::Vector<3> ptam_pos_vect_rotated;

	// struct vision_position_estimate vision {};

	/* register the perf counter */
	perf_counter_t ekf_loop_perf = perf_alloc(PC_ELAPSED, "attitude_estimator_ekf");

        // printf("3\n");

        // int debug = 0;

	/* Main loop*/
	while (!thread_should_exit) {

		struct pollfd fds[2];
		fds[0].fd = sub_raw;
		fds[0].events = POLLIN;
		fds[1].fd = sub_params;
		fds[1].events = POLLIN;
		int ret = poll(fds, 2, 1000);

                // if (debug < 5)
                //         printf("4\n");

		if (ret < 0) {
			/* XXX this is seriously bad - should be an emergency */
		} else if (ret == 0) {
			/* check if we're in HIL - not getting sensor data is fine then */
			orb_copy(ORB_ID(vehicle_control_mode), sub_control_mode, &control_mode);

			if (!control_mode.flag_system_hil_enabled) {
				warnx("WARNING: Not getting sensors - sensor app running?");
			}

		} else {
                        // if (debug < 5)
                        //         printf("5\n");

			/* only update parameters if they changed */
			if (fds[1].revents & POLLIN) {
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), sub_params, &update);

				/* update parameters */
				parameters_update(&ekf_param_handles, &ekf_params);
			}

			/* only run filter if sensor values changed */
			if (fds[0].revents & POLLIN) {

				/* get latest measurements */
				orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);

				if (!initialized) {
					// XXX disabling init for now
					initialized = true;

				} else {

					perf_begin(ekf_loop_perf);

					/* Calculate data time difference in seconds */
					dt = (raw.timestamp - last_measurement) / 1000000.0f;
					last_measurement = raw.timestamp;
					uint8_t update_vect[4] = {0, 0, 0, 0};
                                        uint8_t update_pos_vect[3] = {0, 0, 0};

					/* Fill in gyro measurements */
					if (sensor_last_timestamp[0] != raw.timestamp) {
						update_vect[0] = 1;
						// sensor_update_hz[0] = 1e6f / (raw.timestamp - sensor_last_timestamp[0]);
						sensor_last_timestamp[0] = raw.timestamp;
					}

					z_k[0] =  raw.gyro_rad_s[0] - gyro_offsets[0];
					z_k[1] =  raw.gyro_rad_s[1] - gyro_offsets[1];
					z_k[2] =  raw.gyro_rad_s[2] - gyro_offsets[2];

					/* update accelerometer measurements */
					if (sensor_last_timestamp[1] != raw.accelerometer_timestamp) {
						update_vect[1] = 1;
                                                update_pos_vect[0] = 1;
						// sensor_update_hz[1] = 1e6f / (raw.timestamp - sensor_last_timestamp[1]);
						sensor_last_timestamp[1] = raw.accelerometer_timestamp;
					}

					z_k[3] = raw.accelerometer_m_s2[0]; //  - acc(0);
					z_k[4] = raw.accelerometer_m_s2[1]; //  - acc(1);
					z_k[5] = raw.accelerometer_m_s2[2]; //  - acc(2);

                                        bool ptam_updated = false;
					orb_check(ptam_sub, &ptam_updated);

					if (ptam_updated) {
						orb_copy(ORB_ID(vision_position_estimate), ptam_sub, &ptam);
                                                // printf("\n\nPTAM modtaget\n\n");
					}

                                        // Use PTAM data
                                        if (sensor_last_timestamp[2] != ptam.timestamp_boot) {
						update_vect[2]     = 1;
                                                update_vect[3]     = 1;
                                                update_pos_vect[1] = 1;
                                                // TODO: Måske skal der vendes på quaternionen
						
						sensor_last_timestamp[2] = ptam.timestamp_boot;
					}

                                        bool got_updated = false;
					orb_check(got_pos_sub, &got_updated);

					if (got_updated) {
						orb_copy(ORB_ID(att_pos_mocap), got_pos_sub, &got_pos);
                                                // printf("\n\nGOT modtaget\n\n");
					}

                                        // Use GOT data
                                        if (sensor_last_timestamp[3] != got_pos.timestamp) {
                                                update_pos_vect[2] = 1;
						
						sensor_last_timestamp[3] = got_pos.timestamp;
					}

                                        /* runs only the first time we get ptam data */
                                        if ( !ptam_initialized && ((update_vect[2] == 1) || (update_pos_vect[1] == 1)) ) {
                                                math::Vector<4> res;
                                                res = qmult( q_h2c, ptam.q );
                                                for (int i = 0; i < 4; i++)
                                                        q_h2ptam[i] = res.data[i]; // FIXME: Fusk kan gøres bedre

                                                ptam_initialized = true;
                                        }

                                        /* runs everytime we get ptam data */
                                        if ( ptam_initialized && (update_vect[2] == 1) ) {
                                                q_ptam_conj[0] = ptam.q[0];
                                                for (int i = 1; i < 4; i++)
                                                        q_ptam_conj[i] = -ptam.q[i];

                                                q_temp = qmult( q_ptam_conj, q_b2c_conj );
                                                for (int i = 0; i < 4; i++)
                                                        q_ptam2b[i] = q_temp.data[i];

                                                q_temp = qmult( q_h2ptam, q_ptam2b );
                                                for (int i = 0; i < 4; i++)
                                                        q_h2b[i] = q_temp.data[i];

                                                q_h2b[3] *= -1.0f;
                                        }

                                        R_n = quat2Rot(q_h2b);
                                        gm = matVectMult(R_n, mag_in_h);
                                        gg = matVectMult(R_n, grav_in_h);

                                        z_k[6] = gg.data[0];
                                        z_k[7] = gg.data[1];
                                        z_k[8] = gg.data[2];

                                        z_k[9] = gm.data[0];
                                        z_k[10] = gm.data[1];
                                        z_k[11] = gm.data[2];

                                        // gg.print();
                                        // gm.print();

                                        // POS data prep
                                        ptam_pos_vect.data[0] = ptam.x;
                                        ptam_pos_vect.data[1] = ptam.y;
                                        ptam_pos_vect.data[2] = ptam.z;

                                        rot_b2c_pos = quat2Rot(q_h2c);
                                        ptam_pos_vect_rotated = matVectMult(rot_b2c_pos, ptam_pos_vect);

                                        z_pos_k[3] = ptam_pos_vect_rotated.data[0] * -1.0f; // FIXME: mulig fejl
                                        z_pos_k[4] = ptam_pos_vect_rotated.data[1];
                                        z_pos_k[5] = ptam_pos_vect_rotated.data[2];

                                        z_pos_k[6] = got_pos.x;
                                        z_pos_k[7] = got_pos.y;
                                        z_pos_k[8] = got_pos.z;

					// uint64_t now = hrt_absolute_time();
					// unsigned int time_elapsed = now - last_run;
					// last_run = now;

					// if (time_elapsed > loop_interval_alarm) {
					// 	//TODO: add warning, cpu overload here
					// 	// if (overloadcounter == 20) {
					// 	// 	printf("CPU OVERLOAD DETECTED IN ATTITUDE ESTIMATOR EKF (%lu > %lu)\n", time_elapsed, loop_interval_alarm);
					// 	// 	overloadcounter = 0;
					// 	// }

					// 	overloadcounter++;
					// }

					static bool const_initialized = false;

					/* initialize with good values once we have a reasonable dt estimate */
					if (!const_initialized && dt < 0.1f && dt > 0.001f) {
						dt = 0.005f;
						parameters_update(&ekf_param_handles, &ekf_params);

                                                ekf_params.moment_inertia_J[0] = 0.0018f; 
                                                ekf_params.moment_inertia_J[1] = 0.0f; 
                                                ekf_params.moment_inertia_J[2] = 0.0f;
                                                ekf_params.moment_inertia_J[3] = 0.0f;
                                                ekf_params.moment_inertia_J[4] = 0.0018f;
                                                ekf_params.moment_inertia_J[5] = 0.0f;
                                                ekf_params.moment_inertia_J[6] = 0.0f;
                                                ekf_params.moment_inertia_J[7] = 0.0f;
                                                ekf_params.moment_inertia_J[8] = 0.0018f; // FIXME: init fusk

                                                // if (debug < 10) {
                                                //         printf("J_initialized = \n");
                                                //         for (int i = 0; i < 9; i++){
                                                //                 if ( !(i%3) )
                                                //                         printf("[ ");
                                                //                 printf("%4.10f ", ekf_params.moment_inertia_J[i]);
                                                //                 if ( !((i + 1)%3) )
                                                //                         printf("]\n");
                                                //         }
                                                // }

						x_aposteriori_k[0] = z_k[0];
						x_aposteriori_k[1] = z_k[1];
						x_aposteriori_k[2] = z_k[2];
						x_aposteriori_k[3] = 0.0f;
						x_aposteriori_k[4] = 0.0f;
						x_aposteriori_k[5] = 0.0f;
						x_aposteriori_k[6] = z_k[6];
						x_aposteriori_k[7] = z_k[7];
						x_aposteriori_k[8] = z_k[8];
						x_aposteriori_k[9] = z_k[9];
						x_aposteriori_k[10] = z_k[10];
						x_aposteriori_k[11] = z_k[11];

						const_initialized = true;
					}

					/* do not execute the filter if not initialized */
					if (!const_initialized) {
						continue;
					}

                                        // if (debug < 5)
                                        //         printf("6\n");

#ifdef Q_EKF_ATT_DEBUG
                                        if (debug < 100) {
                                                printf("\n");
                                                for (int i = 0; i < 4; i++){
                                                        printf("ekf_param.q[%i] = ",i);
                                                        printf("%4.4f", (double)ekf_params.q[i]);
                                                        printf("\n");
                                                }

                                                for (int i = 0; i < 2; i++){
                                                        printf("ekf_param.r[%i] = ",i);
                                                        printf("%4.4f", (double)ekf_params.r[i]);
                                                        printf("\n");
                                                }

                                                printf("ekf_params.r_ptam = %4.4f\n", (double)ekf_params.r_ptam);

                                                printf("z_k = [ ");
                                                for (int i = 0; i < 12; i++)
                                                        printf("%4.4f ", (double)z_k[i]);
                                                printf("]\n");

                                                printf("update_vector = [ ");
                                                for (int i = 0; i < 4; i++)
                                                        printf("%f ", (double)update_vect[i]);
                                                printf("]\n");

                                                printf("dt = %4.4f\n", (double)dt);

                                                printf("J = \n");
                                                for (int i = 0; i < 9; i++){
                                                        if ( !(i%3) )
                                                                printf("[ ");
                                                        printf("%4.5f ", ekf_params.moment_inertia_J[i]);
                                                        if ( !((i + 1)%3) )
                                                                printf("]\n");
                                                }

                                                printf("x_apo = [ ");
                                                for (int i = 0; i < 12; i++)
                                                        printf("%4.4f ", (double)x_aposteriori_k[i]);
                                                printf("]\n");

                                                printf("Rot_matrix =\n");
                                                for (int i = 0; i < 9; i++){
                                                        if ( !(i%3) )
                                                                printf("[ ");
                                                        printf("%4.4f ", (double)Rot_matrix[i]);
                                                        if ( !((i + 1)%3) )
                                                                printf("]\n");
                                                }

                                                printf("P_apo =\n");
                                                for (int i = 0; i < 144; i++){
                                                        if ( !(i%12) )
                                                                printf("[ ");
                                                        printf("%4.4f ", (double)P_aposteriori_k[i]);
                                                        if ( !((i + 1)%12) )
                                                                printf("]\n");
                                                }

                                                debug++;
                                        }
#endif //Q_EKF_ATT_DEBUG

#ifdef Q_EKF_POS_DEBUG
                                        if (debug < 1000) {
                                                printf("\n");

                                                printf("z_pos_k = [ ");
                                                for (int i = 0; i < 9; i++)
                                                        printf("%4.4f ", (double)z_pos_k[i]);
                                                printf("]\n");

                                                printf("update_vector = [ ");
                                                for (int i = 0; i < 3; i++)
                                                        printf("%f ", (double)update_pos_vect[i]);
                                                printf("]\n");

                                                // printf("dt = %4.4f\n", (double)dt);

                                                printf("x_pos_apo = [ ");
                                                for (int i = 0; i < 9; i++)
                                                        printf("%4.4f ", (double)x_pos_apo_k[i]);
                                                printf("]\n");

                                                // printf("P_apo =\n");
                                                // for (int i = 0; i < 81; i++){
                                                //         if ( !(i%9) )
                                                //                 printf("[ ");
                                                //         printf("%4.2f ", (double)P_pos_apo_k[i]);
                                                //         if ( !((i + 1)%9) )
                                                //                 printf("]\n");
                                                // }
                                                printf("x = %4.4f, y = %4.4f, z = %4.4f\n", (double)pos.x, (double)pos.y, (double)pos.z);
                                                if ( update_pos_vect[2] == 1 )
                                                        printf("GOT: x = %4.1f, y = %4.1f, z = %4.1f\n", (double)got_pos.x, (double)got_pos.y, (double)got_pos.z);

                                                if ( update_pos_vect[1] == 1 )
                                                        printf("PTAM: x = %4.1f, y = %4.1f, z = %4.1f\n", (double)ptam.x, (double)ptam.y, (double)ptam.z);

                                                debug++;
                                        }
#endif //Q_EKF_POS_DEBUG
					/* Call the estimators */
					AttitudeEKF2grav(false, // approx_prediction
                                                         (unsigned char)ekf_params.use_moment_inertia,
                                                         update_vect,
                                                         dt,
                                                         z_k,
                                                         ekf_params.q[0], // q_rotSpeed,
                                                         ekf_params.q[1], // q_rotAcc
                                                         ekf_params.q[2], // q_acc
                                                         ekf_params.q[3], // q_mag
                                                         ekf_params.r[0], // r_gyro
                                                         ekf_params.r[1], // r_accel
                                                         ekf_params.r_ptam, // r_ptam
                                                         ekf_params.moment_inertia_J,
                                                         x_aposteriori_k,
                                                         P_aposteriori,
                                                         Rot_matrix,
                                                         euler,
                                                         debugOutput,
                                                         q_att);

                                        acc(0) = x_aposteriori_k[6];
                                        acc(1) = x_aposteriori_k[7];
                                        acc(2) = x_aposteriori_k[8];

                                        // if ( debug < 10000 )
                                        //         acc.print();

                                        z_pos_k[0] = raw.accelerometer_m_s2[0] - acc(0);
					z_pos_k[1] = raw.accelerometer_m_s2[1] - acc(1);
					z_pos_k[2] = raw.accelerometer_m_s2[2] - acc(2);

                                        posEKF(update_pos_vect,
                                               dt,
                                               z_pos_k,
                                               q_pos_acc,
                                               q_pos_speed,
                                               q_pos,
                                               r_pos_acc,
                                               r_pos_ptam,
                                               r_pos_got,
                                               x_pos_apo_k,
                                               P_pos_apo_k,
                                               debug_pos);

                                        // for (int i = 0; i < 3; i++)
                                        //         printf("%f", (double)x_pos_apo_k[i]);
                                        // printf("\n");

                                        // if (debug < 5) {
                                        //         printf("euler = [%4.4f %4.4f %4.4f]\n", (double)euler[0], (double)euler[1], (double)euler[2]);
                                        //         printf("q_att = [%4.4f %4.4f %4.4f %4.4f]\n", (double)q_att[0], (double)q_att[1], (double)q_att[2], (double)q_att[3]);
                                        //         printf("7\n");
                                        // }

					/* swap values for next iteration, check for fatal inputs */
					if (isfinite(euler[0]) && isfinite(euler[1]) && isfinite(euler[2]) && isfinite(x_pos_apo_k[6]) && isfinite(x_pos_apo_k[7]) && isfinite(x_pos_apo_k[8])) {
                                                for (int i = 0; i < 144; i++)
                                                        P_aposteriori[i] = P_aposteriori_k[i];

                                                for (int i = 0; i < 12; i++)
                                                        x_aposteriori[i] = x_aposteriori_k[i];

                                                for (int i = 0; i < 81; i++)
                                                        P_pos_aposteriori[i] = P_pos_apo_k[i];

                                                P_pos_aposteriori[1] = P_pos_aposteriori[1]; // just to satisfy the compiler

                                                for (int i = 0; i < 9; i++)
                                                        x_pos_aposteriori[i] = x_pos_apo_k[i];

                                                // if (debug < 5)
                                                //         printf("finite\n");

					} else {
						/* due to inputs or numerical failure the output is invalid, skip it */
                                                // if (debug < 5)
                                                //         printf("continue\n");

						continue;
					}

					if (last_data > 0 && raw.timestamp - last_data > 30000)
						printf("[q ekf] sensor data missed! (%llu)\n", raw.timestamp - last_data);

					last_data = raw.timestamp;

					/* send out */
					att.timestamp = raw.timestamp;

					att.roll = euler[0];
					att.pitch = euler[1];
					att.yaw = euler[2];// + mag_decl;

					att.rollspeed = x_aposteriori[0];
					att.pitchspeed = x_aposteriori[1];
					att.yawspeed = x_aposteriori[2];
					att.rollacc = x_aposteriori[3];
					att.pitchacc = x_aposteriori[4];
					att.yawacc = x_aposteriori[5];
                                        
                                        att.q[0] = q_att[0];
                                        att.q[1] = q_att[1];
                                        att.q[2] = q_att[2];
                                        att.q[3] = q_att[3];

                                        att.g_comp[0] = raw.accelerometer_m_s2[0] - acc(0);
                                        att.g_comp[1] = raw.accelerometer_m_s2[1] - acc(1);
                                        att.g_comp[2] = raw.accelerometer_m_s2[2] - acc(2);

                                        pos.x = x_pos_aposteriori[6]; 
                                        pos.y = x_pos_aposteriori[7];
                                        pos.z = x_pos_aposteriori[8];

                                        //if ( debug < 5 )
                                        // printf("x = %4.4f, y = %4.4f, z = %4.4f\n", (double)pos.x, (double)pos.y, (double)pos.z);

                                        // if (debug < 100) {
                                        //         printf("q0 = %4.4f q1 = %4.4f q2 = %4.4f q3 = %4.4f\n", (double)att.q[0], (double)att.q[1], (double)att.q[2], (double)att.q[3]);
                                        //         printf("roll = %4.4f, pitch = %4.4f, yaw = %4.4f\n", (double)att.roll*(double)rad2deg, (double)att.pitch*(double)rad2deg, (double)att.yaw*(double)rad2deg);
                                        // }

					/* copy offsets */
					memcpy(&att.rate_offsets, &(x_aposteriori[3]), sizeof(att.rate_offsets));

					if (isfinite(pos.x) && isfinite(pos.y) && isfinite(pos.z)) {
						// Broadcast
						orb_publish(ORB_ID(vehicle_local_position), pub_pos, &pos);

                                                // if (debug < 5)
                                                //         printf("Publishing\n");

					} else {
                                                warnx("NaN in x/y/z estimate!");
					}

					if (isfinite(att.roll) && isfinite(att.pitch) && isfinite(att.yaw)) {
						// Broadcast
						orb_publish(ORB_ID(vehicle_attitude), pub_att, &att);
                                                // printf("roll = %4.4f, pitch = %4.4f, yaw = %4.4f\n", (double)att.roll*(double)rad2deg, (double)att.pitch*(double)rad2deg, (double)att.yaw*(double)rad2deg);
                                                // printf("q0 = %4.4f, q1 = %4.4f, q2 = %4.4f, q3 = %4.4f\n", (double)att.q[0], (double)att.q[1], (double)att.q[2], (double)att.q[3]);
                                                // if (debug < 5)
                                                //         printf("Publishing\n");

					} else {
						warnx("NaN in roll/pitch/yaw estimate!");
					}

					perf_end(ekf_loop_perf);
				}
			}
		}

		loopcounter++;
	}

	thread_running = false;

	return 0;
}

math::Matrix<3,3> quat2Rot(float q[4]) {
        math::Matrix<3,3> R;

        R.data[0][0] = 2*q[0]*q[0] - 1 + 2*q[1]*q[1];
        R.data[0][1] = 2*q[1]*q[2] + 2*q[0]*q[3];
        R.data[0][2] = 2*q[1]*q[3] - 2*q[0]*q[2];

        R.data[1][0] = 2*q[1]*q[2] - 2*q[0]*q[3];
        R.data[1][1] = 2*q[0]*q[0] - 1 + 2*q[2]*q[2];
        R.data[1][2] = 2*q[2]*q[3] + 2*q[0]*q[1];

        R.data[2][0] = 2*q[1]*q[3] + 2*q[0]*q[2];
        R.data[2][1] = 2*q[2]*q[3] - 2*q[0]*q[1];
        R.data[2][2] = 2*q[0]*q[0] - 1 + 2*q[3]*q[3];

        return R;
}

math::Vector<3> matVectMult(math::Matrix<3,3> mat, math::Vector<3> v) {
        math::Vector<3> vr;

        vr.data[0] = mat.data[0][0]*v.data[0] + mat.data[0][1]*v.data[1] + mat.data[0][2]*v.data[2];
        vr.data[1] = mat.data[1][0]*v.data[0] + mat.data[1][1]*v.data[1] + mat.data[1][2]*v.data[2];
        vr.data[2] = mat.data[2][0]*v.data[0] + mat.data[2][1]*v.data[1] + mat.data[2][2]*v.data[2];

        return vr;
}

math::Vector<4> qmult( float *q, float *p ) {
        math::Vector<4> res;

        res.data[0] = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3];
        res.data[1] = q[1]*p[0] + q[0]*p[1] - q[3]*p[2] + q[2]*p[3];
        res.data[2] = q[2]*p[0] + q[3]*p[1] + q[0]*p[2] - q[1]*p[3];
        res.data[3] = q[3]*p[0] - q[2]*p[1] + q[1]*p[2] + q[0]*p[3];

        return res; 
}


