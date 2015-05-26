/* 
 * File: _coder_posEKF_api.h 
 *  
 * MATLAB Coder version            : 2.6 
 * C/C++ source code generated on  : 26-May-2015 11:27:41 
 */

#ifndef ___CODER_POSEKF_API_H__
#define ___CODER_POSEKF_API_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */
extern void posEKF_initialize(emlrtContext *aContext);
extern void posEKF_terminate(void);
extern void posEKF_atexit(void);
extern void posEKF_api(const mxArray *prhs[9], const mxArray *plhs[3]);
extern void posEKF(unsigned char zFlag[3], double dt, double z[9], double q_acc, double q_speed, double q_pos, double r_acc, double r_ptam[3], double r_got, float xa_apo[9], float Pa_apo[81], float debugOutput[4]);
extern void posEKF_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_posEKF_api.h 
 *  
 * [EOF] 
 */
