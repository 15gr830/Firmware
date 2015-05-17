/*
 * File: posEKF.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 17-May-2015 11:18:31
 */

#ifndef __POSEKF_H__
#define __POSEKF_H__

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "posEKF_types.h"

/* Function Declarations */
extern void posEKF(const unsigned char zFlag[3], double dt, const double z[9],
                   double q_acc, double q_speed, double q_pos, double r_acc,
                   const double r_ptam[3], double r_got, float xa_apo[9], float
                   Pa_apo[81], float debugOutput[4]);
extern void posEKF_initialize(void);
extern void posEKF_terminate(void);

#endif

/*
 * File trailer for posEKF.h
 *
 * [EOF]
 */
