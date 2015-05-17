/* 
 * File: _coder_AttitudeEKF2grav_api.h 
 *  
 * MATLAB Coder version            : 2.6 
 * C/C++ source code generated on  : 17-May-2015 11:49:33 
 */

#ifndef ___CODER_ATTITUDEEKF2GRAV_API_H__
#define ___CODER_ATTITUDEEKF2GRAV_API_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */
extern void AttitudeEKF2grav_initialize(emlrtContext *aContext);
extern void AttitudeEKF2grav_terminate(void);
extern void AttitudeEKF2grav_atexit(void);
extern void AttitudeEKF2grav_api(const mxArray *prhs[13], const mxArray *plhs[6]);
extern void AttitudeEKF2grav(unsigned char approx_prediction, unsigned char use_inertia_matrix, unsigned char zFlag[4], double dt, double z[12], double q_rotSpeed, double q_rotAcc, double q_acc, double q_mag, double r_gyro, double r_accel, double r_ptam, double J[9], float xa_apo[12], float Pa_apo[144], float Rot_matrix[9], float eulerAngles[3], float debugOutput[4], float q[4]);
extern void AttitudeEKF2grav_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_AttitudeEKF2grav_api.h 
 *  
 * [EOF] 
 */
