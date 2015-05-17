/*
 * File: AttitudeEKF2grav.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 17-May-2015 11:49:33
 */

#ifndef __ATTITUDEEKF2GRAV_H__
#define __ATTITUDEEKF2GRAV_H__

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "AttitudeEKF2grav_types.h"

/* Function Declarations */
extern void AttitudeEKF2grav(unsigned char approx_prediction, unsigned char
                             use_inertia_matrix, const unsigned char zFlag[4], double dt, float z[12],
                             double q_rotSpeed, double q_rotAcc, double q_acc, double q_mag, double r_gyro,
                             double r_accel, double r_ptam, const double J[9], float xa_apo[12], float
                             Pa_apo[144], float Rot_matrix[9], float eulerAngles[3], float debugOutput[4],
                             float q[4]);
extern void AttitudeEKF2grav_initialize(void);
extern void AttitudeEKF2grav_terminate(void);

#endif

/*
 * File trailer for AttitudeEKF2grav.h
 *
 * [EOF]
 */
