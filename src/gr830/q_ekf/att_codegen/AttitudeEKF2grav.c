/*
 * File: AttitudeEKF2grav.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 17-May-2015 11:49:33
 */

/* Include files */
#include "AttitudeEKF2grav.h"

/* Variable Definitions */
static float Ji[9];
static boolean_T Ji_not_empty;
static float x_apo[12];
static float P_apo[144];
static double Q[144];
static boolean_T Q_not_empty;

/* Function Declarations */
static void AttitudeEKF2grav_init(void);
static void b_mrdivide(float A[144], const float B[144]);
static void b_sign(float *x);
static void c_mrdivide(float A[108], const float B[81]);
static void cross(const float a[3], const float b[3], float c[3]);
static void d_mrdivide(float A[72], const float B[36]);
static void diag(const double v[12], double d[144]);
static void eye(double I[144]);
static void inv(const double x[9], double y[9]);
static void mrdivide(const float A[36], const float B[9], float y[36]);
static float norm(const float x[3]);
static void rdivide(const float x[3], float y, float z[3]);

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
static void AttitudeEKF2grav_init(void)
{
        int i;
        static const float fv5[12] = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
                                       -9.81F, 1.0F, 0.0F, 0.0F };

        for (i = 0; i < 12; i++) {
                x_apo[i] = fv5[i];
        }

        for (i = 0; i < 144; i++) {
                P_apo[i] = 200.0F;
        }
}

/*
 * Arguments    : float A[144]
 *                const float B[144]
 * Return Type  : void
 */
static void b_mrdivide(float A[144], const float B[144])
{
        float b_A[144];
        signed char ipiv[12];
        int k;
        int j;
        int c;
        int jAcol;
        int ix;
        float temp;
        float s;
        int i;
        int jBcol;
        int kBcol;
        memcpy(&b_A[0], &B[0], 144U * sizeof(float));
        for (k = 0; k < 12; k++) {
                ipiv[k] = (signed char)(1 + k);
        }

        for (j = 0; j < 11; j++) {
                c = j * 13;
                jAcol = 0;
                ix = c;
                temp = (real32_T)fabs(b_A[c]);
                for (k = 2; k <= 12 - j; k++) {
                        ix++;
                        s = (real32_T)fabs(b_A[ix]);
                        if (s > temp) {
                                jAcol = k - 1;
                                temp = s;
                        }
                }

                if (b_A[c + jAcol] != 0.0F) {
                        if (jAcol != 0) {
                                ipiv[j] = (signed char)((j + jAcol) + 1);
                                ix = j;
                                jAcol += j;
                                for (k = 0; k < 12; k++) {
                                        temp = b_A[ix];
                                        b_A[ix] = b_A[jAcol];
                                        b_A[jAcol] = temp;
                                        ix += 12;
                                        jAcol += 12;
                                }
                        }

                        k = (c - j) + 12;
                        for (i = c + 1; i + 1 <= k; i++) {
                                b_A[i] /= b_A[c];
                        }
                }

                jBcol = c;
                jAcol = c + 12;
                for (kBcol = 1; kBcol <= 11 - j; kBcol++) {
                        temp = b_A[jAcol];
                        if (b_A[jAcol] != 0.0F) {
                                ix = c + 1;
                                k = (jBcol - j) + 24;
                                for (i = 13 + jBcol; i + 1 <= k; i++) {
                                        b_A[i] += b_A[ix] * -temp;
                                        ix++;
                                }
                        }

                        jAcol += 12;
                        jBcol += 12;
                }
        }

        for (j = 0; j < 12; j++) {
                jBcol = 12 * j;
                jAcol = 12 * j;
                for (k = 1; k <= j; k++) {
                        kBcol = 12 * (k - 1);
                        if (b_A[(k + jAcol) - 1] != 0.0F) {
                                for (i = 0; i < 12; i++) {
                                        A[i + jBcol] -= b_A[(k + jAcol) - 1] * A[i + kBcol];
                                }
                        }
                }

                temp = 1.0F / b_A[j + jAcol];
                for (i = 0; i < 12; i++) {
                        A[i + jBcol] *= temp;
                }
        }

        for (j = 11; j > -1; j += -1) {
                jBcol = 12 * j;
                jAcol = 12 * j - 1;
                for (k = j + 2; k < 13; k++) {
                        kBcol = 12 * (k - 1);
                        if (b_A[k + jAcol] != 0.0F) {
                                for (i = 0; i < 12; i++) {
                                        A[i + jBcol] -= b_A[k + jAcol] * A[i + kBcol];
                                }
                        }
                }
        }

        for (jAcol = 10; jAcol > -1; jAcol += -1) {
                if (ipiv[jAcol] != jAcol + 1) {
                        for (jBcol = 0; jBcol < 12; jBcol++) {
                                temp = A[jBcol + 12 * jAcol];
                                A[jBcol + 12 * jAcol] = A[jBcol + 12 * (ipiv[jAcol] - 1)];
                                A[jBcol + 12 * (ipiv[jAcol] - 1)] = temp;
                        }
                }
        }
}

/*
 * Arguments    : float *x
 * Return Type  : void
 */
static void b_sign(float *x)
{
        if (*x < 0.0F) {
                *x = -1.0F;
        } else {
                if (*x > 0.0F) {
                        *x = 1.0F;
                }
        }
}

/*
 * Arguments    : float A[108]
 *                const float B[81]
 * Return Type  : void
 */
static void c_mrdivide(float A[108], const float B[81])
{
        float b_A[81];
        signed char ipiv[9];
        int k;
        int j;
        int c;
        int jAcol;
        int ix;
        float temp;
        float s;
        int i;
        int jBcol;
        int kBcol;
        memcpy(&b_A[0], &B[0], 81U * sizeof(float));
        for (k = 0; k < 9; k++) {
                ipiv[k] = (signed char)(1 + k);
        }

        for (j = 0; j < 8; j++) {
                c = j * 10;
                jAcol = 0;
                ix = c;
                temp = (real32_T)fabs(b_A[c]);
                for (k = 2; k <= 9 - j; k++) {
                        ix++;
                        s = (real32_T)fabs(b_A[ix]);
                        if (s > temp) {
                                jAcol = k - 1;
                                temp = s;
                        }
                }

                if (b_A[c + jAcol] != 0.0F) {
                        if (jAcol != 0) {
                                ipiv[j] = (signed char)((j + jAcol) + 1);
                                ix = j;
                                jAcol += j;
                                for (k = 0; k < 9; k++) {
                                        temp = b_A[ix];
                                        b_A[ix] = b_A[jAcol];
                                        b_A[jAcol] = temp;
                                        ix += 9;
                                        jAcol += 9;
                                }
                        }

                        k = (c - j) + 9;
                        for (i = c + 1; i + 1 <= k; i++) {
                                b_A[i] /= b_A[c];
                        }
                }

                jBcol = c;
                jAcol = c + 9;
                for (kBcol = 1; kBcol <= 8 - j; kBcol++) {
                        temp = b_A[jAcol];
                        if (b_A[jAcol] != 0.0F) {
                                ix = c + 1;
                                k = (jBcol - j) + 18;
                                for (i = 10 + jBcol; i + 1 <= k; i++) {
                                        b_A[i] += b_A[ix] * -temp;
                                        ix++;
                                }
                        }

                        jAcol += 9;
                        jBcol += 9;
                }
        }

        for (j = 0; j < 9; j++) {
                jBcol = 12 * j;
                jAcol = 9 * j;
                for (k = 1; k <= j; k++) {
                        kBcol = 12 * (k - 1);
                        if (b_A[(k + jAcol) - 1] != 0.0F) {
                                for (i = 0; i < 12; i++) {
                                        A[i + jBcol] -= b_A[(k + jAcol) - 1] * A[i + kBcol];
                                }
                        }
                }

                temp = 1.0F / b_A[j + jAcol];
                for (i = 0; i < 12; i++) {
                        A[i + jBcol] *= temp;
                }
        }

        for (j = 8; j > -1; j += -1) {
                jBcol = 12 * j;
                jAcol = 9 * j - 1;
                for (k = j + 2; k < 10; k++) {
                        kBcol = 12 * (k - 1);
                        if (b_A[k + jAcol] != 0.0F) {
                                for (i = 0; i < 12; i++) {
                                        A[i + jBcol] -= b_A[k + jAcol] * A[i + kBcol];
                                }
                        }
                }
        }

        for (jAcol = 7; jAcol > -1; jAcol += -1) {
                if (ipiv[jAcol] != jAcol + 1) {
                        for (jBcol = 0; jBcol < 12; jBcol++) {
                                temp = A[jBcol + 12 * jAcol];
                                A[jBcol + 12 * jAcol] = A[jBcol + 12 * (ipiv[jAcol] - 1)];
                                A[jBcol + 12 * (ipiv[jAcol] - 1)] = temp;
                        }
                }
        }
}

/*
 * Arguments    : const float a[3]
 *                const float b[3]
 *                float c[3]
 * Return Type  : void
 */
static void cross(const float a[3], const float b[3], float c[3])
{
        c[0] = a[1] * b[2] - a[2] * b[1];
        c[1] = a[2] * b[0] - a[0] * b[2];
        c[2] = a[0] * b[1] - a[1] * b[0];
}

/*
 * Arguments    : float A[72]
 *                const float B[36]
 * Return Type  : void
 */
static void d_mrdivide(float A[72], const float B[36])
{
        float b_A[36];
        signed char ipiv[6];
        int k;
        int j;
        int c;
        int jAcol;
        int ix;
        float temp;
        float s;
        int i;
        int jBcol;
        int kBcol;
        memcpy(&b_A[0], &B[0], 36U * sizeof(float));
        for (k = 0; k < 6; k++) {
                ipiv[k] = (signed char)(1 + k);
        }

        for (j = 0; j < 5; j++) {
                c = j * 7;
                jAcol = 0;
                ix = c;
                temp = (real32_T)fabs(b_A[c]);
                for (k = 2; k <= 6 - j; k++) {
                        ix++;
                        s = (real32_T)fabs(b_A[ix]);
                        if (s > temp) {
                                jAcol = k - 1;
                                temp = s;
                        }
                }

                if (b_A[c + jAcol] != 0.0F) {
                        if (jAcol != 0) {
                                ipiv[j] = (signed char)((j + jAcol) + 1);
                                ix = j;
                                jAcol += j;
                                for (k = 0; k < 6; k++) {
                                        temp = b_A[ix];
                                        b_A[ix] = b_A[jAcol];
                                        b_A[jAcol] = temp;
                                        ix += 6;
                                        jAcol += 6;
                                }
                        }

                        k = (c - j) + 6;
                        for (i = c + 1; i + 1 <= k; i++) {
                                b_A[i] /= b_A[c];
                        }
                }

                jBcol = c;
                jAcol = c + 6;
                for (kBcol = 1; kBcol <= 5 - j; kBcol++) {
                        temp = b_A[jAcol];
                        if (b_A[jAcol] != 0.0F) {
                                ix = c + 1;
                                k = (jBcol - j) + 12;
                                for (i = 7 + jBcol; i + 1 <= k; i++) {
                                        b_A[i] += b_A[ix] * -temp;
                                        ix++;
                                }
                        }

                        jAcol += 6;
                        jBcol += 6;
                }
        }

        for (j = 0; j < 6; j++) {
                jBcol = 12 * j;
                jAcol = 6 * j;
                for (k = 1; k <= j; k++) {
                        kBcol = 12 * (k - 1);
                        if (b_A[(k + jAcol) - 1] != 0.0F) {
                                for (i = 0; i < 12; i++) {
                                        A[i + jBcol] -= b_A[(k + jAcol) - 1] * A[i + kBcol];
                                }
                        }
                }

                temp = 1.0F / b_A[j + jAcol];
                for (i = 0; i < 12; i++) {
                        A[i + jBcol] *= temp;
                }
        }

        for (j = 5; j > -1; j += -1) {
                jBcol = 12 * j;
                jAcol = 6 * j - 1;
                for (k = j + 2; k < 7; k++) {
                        kBcol = 12 * (k - 1);
                        if (b_A[k + jAcol] != 0.0F) {
                                for (i = 0; i < 12; i++) {
                                        A[i + jBcol] -= b_A[k + jAcol] * A[i + kBcol];
                                }
                        }
                }
        }

        for (jAcol = 4; jAcol > -1; jAcol += -1) {
                if (ipiv[jAcol] != jAcol + 1) {
                        for (jBcol = 0; jBcol < 12; jBcol++) {
                                temp = A[jBcol + 12 * jAcol];
                                A[jBcol + 12 * jAcol] = A[jBcol + 12 * (ipiv[jAcol] - 1)];
                                A[jBcol + 12 * (ipiv[jAcol] - 1)] = temp;
                        }
                }
        }
}

/*
 * Arguments    : const double v[12]
 *                double d[144]
 * Return Type  : void
 */
static void diag(const double v[12], double d[144])
{
        int j;
        memset(&d[0], 0, 144U * sizeof(double));
        for (j = 0; j < 12; j++) {
                d[j + 12 * j] = v[j];
        }
}

/*
 * Arguments    : double I[144]
 * Return Type  : void
 */
static void eye(double I[144])
{
        int k;
        memset(&I[0], 0, 144U * sizeof(double));
        for (k = 0; k < 12; k++) {
                I[k + 12 * k] = 1.0;
        }
}

/*
 * Arguments    : const double x[9]
 *                double y[9]
 * Return Type  : void
 */
static void inv(const double x[9], double y[9])
{
        double b_x[9];
        int p1;
        int p2;
        int p3;
        double absx11;
        double absx21;
        double absx31;
        int itmp;
        double b_y;
        memcpy(&b_x[0], &x[0], 9U * sizeof(double));
        p1 = 0;
        p2 = 3;
        p3 = 6;
        absx11 = fabs(x[0]);
        absx21 = fabs(x[1]);
        absx31 = fabs(x[2]);
        if ((absx21 > absx11) && (absx21 > absx31)) {
                p1 = 3;
                p2 = 0;
                b_x[0] = x[1];
                b_x[1] = x[0];
                b_x[3] = x[4];
                b_x[4] = x[3];
                b_x[6] = x[7];
                b_x[7] = x[6];
        } else {
                if (absx31 > absx11) {
                        p1 = 6;
                        p3 = 0;
                        b_x[0] = x[2];
                        b_x[2] = x[0];
                        b_x[3] = x[5];
                        b_x[5] = x[3];
                        b_x[6] = x[8];
                        b_x[8] = x[6];
                }
        }

        absx21 = b_x[1] / b_x[0];
        b_x[1] /= b_x[0];
        absx11 = b_x[2] / b_x[0];
        b_x[2] /= b_x[0];
        b_x[4] -= absx21 * b_x[3];
        b_x[5] -= absx11 * b_x[3];
        b_x[7] -= absx21 * b_x[6];
        b_x[8] -= absx11 * b_x[6];
        if (fabs(b_x[5]) > fabs(b_x[4])) {
                itmp = p2;
                p2 = p3;
                p3 = itmp;
                b_x[1] = absx11;
                b_x[2] = absx21;
                absx11 = b_x[4];
                b_x[4] = b_x[5];
                b_x[5] = absx11;
                absx11 = b_x[7];
                b_x[7] = b_x[8];
                b_x[8] = absx11;
        }

        absx31 = b_x[5];
        b_y = b_x[4];
        absx21 = b_x[5] / b_x[4];
        b_x[8] -= absx21 * b_x[7];
        absx11 = (absx21 * b_x[1] - b_x[2]) / b_x[8];
        absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
        y[p1] = ((1.0 - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
        y[p1 + 1] = absx21;
        y[p1 + 2] = absx11;
        absx11 = -(absx31 / b_y) / b_x[8];
        absx21 = (1.0 - b_x[7] * absx11) / b_x[4];
        y[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
        y[p2 + 1] = absx21;
        y[p2 + 2] = absx11;
        absx11 = 1.0 / b_x[8];
        absx21 = -b_x[7] * absx11 / b_x[4];
        y[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
        y[p3 + 1] = absx21;
        y[p3 + 2] = absx11;
}

/*
 * Arguments    : const float A[36]
 *                const float B[9]
 *                float y[36]
 * Return Type  : void
 */
static void mrdivide(const float A[36], const float B[9], float y[36])
{
        float b_A[9];
        int rtemp;
        int r1;
        int r2;
        int r3;
        float maxval;
        float a21;
        for (rtemp = 0; rtemp < 9; rtemp++) {
                b_A[rtemp] = B[rtemp];
        }

        r1 = 0;
        r2 = 1;
        r3 = 2;
        maxval = (real32_T)fabs(B[0]);
        a21 = (real32_T)fabs(B[1]);
        if (a21 > maxval) {
                maxval = a21;
                r1 = 1;
                r2 = 0;
        }

        if ((real32_T)fabs(B[2]) > maxval) {
                r1 = 2;
                r2 = 1;
                r3 = 0;
        }

        b_A[r2] = B[r2] / B[r1];
        b_A[r3] /= b_A[r1];
        b_A[3 + r2] -= b_A[r2] * b_A[3 + r1];
        b_A[3 + r3] -= b_A[r3] * b_A[3 + r1];
        b_A[6 + r2] -= b_A[r2] * b_A[6 + r1];
        b_A[6 + r3] -= b_A[r3] * b_A[6 + r1];
        if ((real32_T)fabs(b_A[3 + r3]) > (real32_T)fabs(b_A[3 + r2])) {
                rtemp = r2;
                r2 = r3;
                r3 = rtemp;
        }

        b_A[3 + r3] /= b_A[3 + r2];
        b_A[6 + r3] -= b_A[3 + r3] * b_A[6 + r2];
        for (rtemp = 0; rtemp < 12; rtemp++) {
                y[rtemp + 12 * r1] = A[rtemp] / b_A[r1];
                y[rtemp + 12 * r2] = A[12 + rtemp] - y[rtemp + 12 * r1] * b_A[3 + r1];
                y[rtemp + 12 * r3] = A[24 + rtemp] - y[rtemp + 12 * r1] * b_A[6 + r1];
                y[rtemp + 12 * r2] /= b_A[3 + r2];
                y[rtemp + 12 * r3] -= y[rtemp + 12 * r2] * b_A[6 + r2];
                y[rtemp + 12 * r3] /= b_A[6 + r3];
                y[rtemp + 12 * r2] -= y[rtemp + 12 * r3] * b_A[3 + r3];
                y[rtemp + 12 * r1] -= y[rtemp + 12 * r3] * b_A[r3];
                y[rtemp + 12 * r1] -= y[rtemp + 12 * r2] * b_A[r2];
        }
}

/*
 * Arguments    : const float x[3]
 * Return Type  : float
 */
static float norm(const float x[3])
{
        float y;
        float scale;
        int k;
        float absxk;
        float t;
        y = 0.0F;
        scale = 1.17549435E-38F;
        for (k = 0; k < 3; k++) {
                absxk = (real32_T)fabs(x[k]);
                if (absxk > scale) {
                        t = scale / absxk;
                        y = 1.0F + y * t * t;
                        scale = absxk;
                } else {
                        t = absxk / scale;
                        y += t * t;
                }
        }

        return scale * (real32_T)sqrt(y);
}

/*
 * Arguments    : const float x[3]
 *                float y
 *                float z[3]
 * Return Type  : void
 */
static void rdivide(const float x[3], float y, float z[3])
{
        int i;
        for (i = 0; i < 3; i++) {
                z[i] = x[i] / y;
        }
}

/*
 * LQG Postion Estimator and Controller
 *  Observer:
 *         x[n|n]   = x[n|n-1] + M(y[n] - Cx[n|n-1] - Du[n])
 *         x[n+1|n] = Ax[n|n] + Bu[n]
 *
 *  $Author: Tobias Naegeli $    $Date: 2014 $    $Revision: 3 $
 *
 *
 *  Arguments:
 *  approx_prediction: if 1 then the exponential map is approximated with a
 *  first order taylor approximation. has at the moment not a big influence
 *  (just 1st or 2nd order approximation) we should change it to rodriquez
 *  approximation.
 *  use_inertia_matrix: set to true if you have the inertia matrix J for your
 *  quadrotor
 *  xa_apo_k: old state vectotr
 *  zFlag: if sensor measurement is available [gyro, acc, mag]
 *  dt: dt in s
 *  z: measurements [gyro, acc, mag]
 *  q_rotSpeed: process noise gyro
 *  q_rotAcc: process noise gyro acceleration
 *  q_acc: process noise acceleration
 *  q_mag: process noise magnetometer
 *  r_gyro: measurement noise gyro
 *  r_accel: measurement noise accel
 *  r_mag: measurement noise mag
 * r_ptam: PTAM noise
 * r_got: Games of Track noise
 *  J: moment of inertia matrix
 * Arguments    : unsigned char approx_prediction
 *                unsigned char use_inertia_matrix
 *                const unsigned char zFlag[4]
 *                double dt
 *                const double z[12]
 *                double q_rotSpeed
 *                double q_rotAcc
 *                double q_acc
 *                double q_mag
 *                double r_gyro
 *                double r_accel
 *                double r_ptam
 *                const double J[9]
 *                float xa_apo[12]
 *                float Pa_apo[144]
 *                float Rot_matrix[9]
 *                float eulerAngles[3]
 *                float debugOutput[4]
 *                float q[4]
 * Return Type  : void
 */
void AttitudeEKF2grav(unsigned char approx_prediction, 
                      unsigned char use_inertia_matrix, 
                      const unsigned char zFlag[4], 
                      double dt, 
                      float z[12], 
                      double q_rotSpeed, 
                      double q_rotAcc,
                      double q_acc, 
                      double q_mag, 
                      double r_gyro, 
                      double r_accel,
                      double r_ptam, 
                      const double J[9], 
                      float xa_apo[12], 
                      float Pa_apo[144], 
                      float Rot_matrix[9], 
                      float eulerAngles[3],
                      float debugOutput[4], 
                      float q[4])
{
        double dv0[9];
        int i;
        float fv0[3];
        float wak[3];
        float zek[3];
        int i0;
        float b_zek[3];
        float fv1[3];
        float O[9];
        float b_O[9];
        static const signed char iv0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

        float fv2[3];
        double y;
        int i1;
        float fv3[9];
        float fv4[3];
        float muk[3];
        float x_apr[12];
        static double dv1[144];
        static float A_lin[144];
        static const signed char iv1[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1,
                                             0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        static float b_A_lin[144];
        double b_q_rotSpeed[12];
        static float P_apr[144];
        float f0;
        static const signed char a[144] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
        };

        static const signed char b[144] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                            0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
        };

        float b_r_gyro[12];
        float c_A_lin[12];
        float K_k[144];
        float b_a[108];
        static const signed char c_a[108] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                                              0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                              0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

        float S_k[81];
        static const signed char b_b[108] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                              0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                              0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                              0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

        float c_r_gyro[9];
        float b_K_k[108];
        float b_S_k[36];
        static const signed char d_a[36] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        static const signed char c_b[36] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        float d_r_gyro[3];
        float c_K_k[36];
        float e_a[72];
        static const signed char f_a[72] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                                             1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                             0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0 };

        static const signed char d_b[72] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                             0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                             0, 0, 1, 0, 0, 0 };

        float e_r_gyro[6];
        float c_S_k[6];
        float d_K_k[72];
        static const signed char g_a[72] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                                             1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
                                             0, 0, 0, 0, 0, 1 };

        static const signed char e_b[72] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                             0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                                             0, 0, 0, 0, 0, 1 };

        float b_z[6];
        float q0;
        float f1;
        float f2;

        /*  Output: */
        /*  xa_apo: updated state vectotr */
        /*  Pa_apo: updated state covariance matrix */
        /*  Rot_matrix: rotation matrix */
        /*  eulerAngles: euler angles */
        /*  debugOutput: not used */
        /* % model specific parameters */
        /*  compute once the inverse of the Inertia */
        /* 'AttitudeEKF2grav:51' if isempty(Ji) */
        if (!Ji_not_empty) {
                /* 'AttitudeEKF2grav:52' Ji=single(inv(J)); */
                inv(J, dv0);
                for (i = 0; i < 9; i++) {
                        Ji[i] = (float)dv0[i];
                }

                Ji_not_empty = true;
        }

        /* % init */
        /* 'AttitudeEKF2grav:57' if(isempty(x_apo)) */
        /* 'AttitudeEKF2grav:67' if(isempty(P_apo)) */
        /* 'AttitudeEKF2grav:72' debugOutput = single(zeros(4,1)); */
        for (i = 0; i < 4; i++) {
                debugOutput[i] = 0.0F;
        }

        /* % copy the states */
        /* 'AttitudeEKF2grav:75' wx=  x_apo(1); */
        /*  x  body angular rate */
        /* 'AttitudeEKF2grav:76' wy=  x_apo(2); */
        /*  y  body angular rate */
        /* 'AttitudeEKF2grav:77' wz=  x_apo(3); */
        /*  z  body angular rate */
        /* 'AttitudeEKF2grav:79' wax=  x_apo(4); */
        /*  x  body angular acceleration */
        /* 'AttitudeEKF2grav:80' way=  x_apo(5); */
        /*  y  body angular acceleration */
        /* 'AttitudeEKF2grav:81' waz=  x_apo(6); */
        /*  z  body angular acceleration */
        /* 'AttitudeEKF2grav:83' zex=  x_apo(7); */
        /*  x  component gravity vector */
        /* 'AttitudeEKF2grav:84' zey=  x_apo(8); */
        /*  y  component gravity vector */
        /* 'AttitudeEKF2grav:85' zez=  x_apo(9); */
        /*  z  component gravity vector */
        /* 'AttitudeEKF2grav:87' mux=  x_apo(10); */
        /*  x  component magnetic field vector */
        /* 'AttitudeEKF2grav:88' muy=  x_apo(11); */
        /*  y  component magnetic field vector */
        /* 'AttitudeEKF2grav:89' muz=  x_apo(12); */
        /*  z  component magnetic field vector */
        /* anx = x_apo(13); % x component of angle vector */
        /* any = x_apo(14); % y component of angle vector */
        /* anz = x_apo(15); % z component of angle vector */
        /* % prediction section */
        /*  compute the apriori state estimate from the previous aposteriori estimate */
        /* body angular accelerations */
        /* 'AttitudeEKF2grav:99' if (use_inertia_matrix==1) */
        if (use_inertia_matrix == 1) {
                /* 'AttitudeEKF2grav:100' wak =[wax;way;waz]+Ji*(-cross([wax;way;waz],J*[wax;way;waz]))*dt; */
                fv0[0] = x_apo[3];
                fv0[1] = x_apo[4];
                fv0[2] = x_apo[5];
                zek[0] = x_apo[3];
                zek[1] = x_apo[4];
                zek[2] = x_apo[5];
                for (i = 0; i < 3; i++) {
                        wak[i] = 0.0F;
                        for (i0 = 0; i0 < 3; i0++) {
                                wak[i] += (float)J[i + 3 * i0] * zek[i0];
                        }
                }

                cross(fv0, wak, b_zek);
                for (i = 0; i < 3; i++) {
                        zek[i] = -b_zek[i];
                }

                fv1[0] = x_apo[3];
                fv1[1] = x_apo[4];
                fv1[2] = x_apo[5];
                for (i = 0; i < 3; i++) {
                        fv0[i] = 0.0F;
                        for (i0 = 0; i0 < 3; i0++) {
                                fv0[i] += Ji[i + 3 * i0] * zek[i0];
                        }

                        wak[i] = fv1[i] + fv0[i] * (float)dt;
                }
        } else {
                /* 'AttitudeEKF2grav:101' else */
                /* 'AttitudeEKF2grav:102' wak =[wax;way;waz]; */
                wak[0] = x_apo[3];
                wak[1] = x_apo[4];
                wak[2] = x_apo[5];
        }

        /* body angular rates */
        /* 'AttitudeEKF2grav:106' wk =[wx;  wy; wz] + dt*wak; */
        /* derivative of the prediction rotation matrix */
        /* 'AttitudeEKF2grav:109' O=[0,-wz,wy;wz,0,-wx;-wy,wx,0]'; */
        O[0] = 0.0F;
        O[1] = -x_apo[2];
        O[2] = x_apo[1];
        O[3] = x_apo[2];
        O[4] = 0.0F;
        O[5] = -x_apo[0];
        O[6] = -x_apo[1];
        O[7] = x_apo[0];
        O[8] = 0.0F;

        /* prediction of the earth z vector */
        /* 'AttitudeEKF2grav:112' if (approx_prediction==1) */
        if (approx_prediction == 1) {
                /* e^(Odt)=I+dt*O+dt^2/2!O^2 */
                /*  so we do a first order approximation of the exponential map */
                /* 'AttitudeEKF2grav:115' zek =(O*dt+single(eye(3)))*[zex;zey;zez]; */
                for (i = 0; i < 3; i++) {
                        for (i0 = 0; i0 < 3; i0++) {
                                b_O[i0 + 3 * i] = O[i0 + 3 * i] * (float)dt + (float)iv0[i0 + 3 * i];
                        }
                }

                fv2[0] = x_apo[6];
                fv2[1] = x_apo[7];
                fv2[2] = x_apo[8];
                for (i = 0; i < 3; i++) {
                        b_zek[i] = 0.0F;
                        for (i0 = 0; i0 < 3; i0++) {
                                b_zek[i] += b_O[i + 3 * i0] * fv2[i0];
                        }
                }
        } else {
                /* 'AttitudeEKF2grav:117' else */
                /* 'AttitudeEKF2grav:118' zek =(single(eye(3))+O*dt+dt^2/2*O^2)*[zex;zey;zez]; */
                y = dt * dt / 2.0;
                for (i = 0; i < 3; i++) {
                        for (i0 = 0; i0 < 3; i0++) {
                                b_O[i + 3 * i0] = 0.0F;
                                for (i1 = 0; i1 < 3; i1++) {
                                        b_O[i + 3 * i0] += O[i + 3 * i1] * O[i1 + 3 * i0];
                                }
                        }
                }

                for (i = 0; i < 3; i++) {
                        for (i0 = 0; i0 < 3; i0++) {
                                fv3[i0 + 3 * i] = ((float)iv0[i0 + 3 * i] + O[i0 + 3 * i] * (float)dt) +
                                        (float)y * b_O[i0 + 3 * i];
                        }
                }

                fv2[0] = x_apo[6];
                fv2[1] = x_apo[7];
                fv2[2] = x_apo[8];
                for (i = 0; i < 3; i++) {
                        b_zek[i] = 0.0F;
                        for (i0 = 0; i0 < 3; i0++) {
                                b_zek[i] += fv3[i + 3 * i0] * fv2[i0];
                        }
                }

                /* zek =expm2(O*dt)*[zex;zey;zez]; not working because use double */
                /* precision */
        }

        /* prediction of the magnetic vector */
        /* 'AttitudeEKF2grav:125' if (approx_prediction==1) */
        if (approx_prediction == 1) {
                /* e^(Odt)=I+dt*O+dt^2/2!O^2 */
                /*  so we do a first order approximation of the exponential map */
                /* 'AttitudeEKF2grav:128' muk =(O*dt+single(eye(3)))*[mux;muy;muz]; */
                for (i = 0; i < 3; i++) {
                        for (i0 = 0; i0 < 3; i0++) {
                                b_O[i0 + 3 * i] = O[i0 + 3 * i] * (float)dt + (float)iv0[i0 + 3 * i];
                        }
                }

                fv4[0] = x_apo[9];
                fv4[1] = x_apo[10];
                fv4[2] = x_apo[11];
                for (i = 0; i < 3; i++) {
                        muk[i] = 0.0F;
                        for (i0 = 0; i0 < 3; i0++) {
                                muk[i] += b_O[i + 3 * i0] * fv4[i0];
                        }
                }
        } else {
                /* 'AttitudeEKF2grav:129' else */
                /* 'AttitudeEKF2grav:130' muk =(single(eye(3))+O*dt+dt^2/2*O^2)*[mux;muy;muz]; */
                y = dt * dt / 2.0;
                for (i = 0; i < 3; i++) {
                        for (i0 = 0; i0 < 3; i0++) {
                                b_O[i + 3 * i0] = 0.0F;
                                for (i1 = 0; i1 < 3; i1++) {
                                        b_O[i + 3 * i0] += O[i + 3 * i1] * O[i1 + 3 * i0];
                                }
                        }
                }

                for (i = 0; i < 3; i++) {
                        for (i0 = 0; i0 < 3; i0++) {
                                fv3[i0 + 3 * i] = ((float)iv0[i0 + 3 * i] + O[i0 + 3 * i] * (float)dt) +
                                        (float)y * b_O[i0 + 3 * i];
                        }
                }

                fv4[0] = x_apo[9];
                fv4[1] = x_apo[10];
                fv4[2] = x_apo[11];
                for (i = 0; i < 3; i++) {
                        muk[i] = 0.0F;
                        for (i0 = 0; i0 < 3; i0++) {
                                muk[i] += fv3[i + 3 * i0] * fv4[i0];
                        }
                }

                /* muk =expm2(O*dt)*[mux;muy;muz]; not working because use double */
                /* precision */
        }

        /* Prediction of angles */
        /* ank = [anx;any;anz] + wk*dt + 0.5*wak*dt^2; */
        /* x_apr=[wk;wak;zek;muk; ank]; */
        /* 'AttitudeEKF2grav:140' x_apr=[wk;wak;zek;muk]; */
        x_apr[0] = x_apo[0] + (float)dt * wak[0];
        x_apr[1] = x_apo[1] + (float)dt * wak[1];
        x_apr[2] = x_apo[2] + (float)dt * wak[2];
        for (i = 0; i < 3; i++) {
                x_apr[i + 3] = wak[i];
        }

        for (i = 0; i < 3; i++) {
                x_apr[i + 6] = b_zek[i];
        }

        for (i = 0; i < 3; i++) {
                x_apr[i + 9] = muk[i];
        }

        /*  compute the apriori error covariance estimate from the previous */
        /* aposteriori estimate */
        /* 'AttitudeEKF2grav:146' EZ=[0,zez,-zey; */
        /* 'AttitudeEKF2grav:147'     -zez,0,zex; */
        /* 'AttitudeEKF2grav:148'     zey,-zex,0]'; */
        /* 'AttitudeEKF2grav:149' MA=[0,muz,-muy; */
        /* 'AttitudeEKF2grav:150'     -muz,0,mux; */
        /* 'AttitudeEKF2grav:151'     muy,-mux,0]'; */
        /* 'AttitudeEKF2grav:153' E=single(eye(3)); */
        /* 'AttitudeEKF2grav:154' Z=single(zeros(3)); */
        /* 'AttitudeEKF2grav:156' A_lin=[ Z,  E,  Z, Z;... */
        /* 'AttitudeEKF2grav:157'     Z,  Z,  Z,  Z;... */
        /* 'AttitudeEKF2grav:158'     EZ, Z,  O,  Z;... */
        /* 'AttitudeEKF2grav:159'     MA, Z,  Z,  O]; */
        /* A_lin=eye(12)+A_lin*dt; */
        /* 'AttitudeEKF2grav:163' A_lin=eye(12)+A_lin*dt; */
        eye(dv1);
        for (i = 0; i < 12; i++) {
                for (i0 = 0; i0 < 3; i0++) {
                        A_lin[i0 + 12 * i] = iv1[i0 + 3 * i];
                }

                for (i0 = 0; i0 < 3; i0++) {
                        A_lin[(i0 + 12 * i) + 3] = 0.0F;
                }
        }

        A_lin[6] = 0.0F;
        A_lin[7] = x_apo[8];
        A_lin[8] = -x_apo[7];
        A_lin[18] = -x_apo[8];
        A_lin[19] = 0.0F;
        A_lin[20] = x_apo[6];
        A_lin[30] = x_apo[7];
        A_lin[31] = -x_apo[6];
        A_lin[32] = 0.0F;
        for (i = 0; i < 3; i++) {
                for (i0 = 0; i0 < 3; i0++) {
                        A_lin[(i0 + 12 * (i + 3)) + 6] = 0.0F;
                }
        }

        for (i = 0; i < 3; i++) {
                for (i0 = 0; i0 < 3; i0++) {
                        A_lin[(i0 + 12 * (i + 6)) + 6] = O[i0 + 3 * i];
                }
        }

        for (i = 0; i < 3; i++) {
                for (i0 = 0; i0 < 3; i0++) {
                        A_lin[(i0 + 12 * (i + 9)) + 6] = 0.0F;
                }
        }

        A_lin[9] = 0.0F;
        A_lin[10] = x_apo[11];
        A_lin[11] = -x_apo[10];
        A_lin[21] = -x_apo[11];
        A_lin[22] = 0.0F;
        A_lin[23] = x_apo[9];
        A_lin[33] = x_apo[10];
        A_lin[34] = -x_apo[9];
        A_lin[35] = 0.0F;
        for (i = 0; i < 3; i++) {
                for (i0 = 0; i0 < 3; i0++) {
                        A_lin[(i0 + 12 * (i + 3)) + 9] = 0.0F;
                }
        }

        for (i = 0; i < 3; i++) {
                for (i0 = 0; i0 < 3; i0++) {
                        A_lin[(i0 + 12 * (i + 6)) + 9] = 0.0F;
                }
        }

        for (i = 0; i < 3; i++) {
                for (i0 = 0; i0 < 3; i0++) {
                        A_lin[(i0 + 12 * (i + 9)) + 9] = O[i0 + 3 * i];
                }
        }

        for (i = 0; i < 12; i++) {
                for (i0 = 0; i0 < 12; i0++) {
                        b_A_lin[i0 + 12 * i] = (float)dv1[i0 + 12 * i] + A_lin[i0 + 12 * i] *
                                (float)dt;
                }
        }

        /* process covariance matrix */
        /* 'AttitudeEKF2grav:168' if (isempty(Q)) */
        if (!Q_not_empty) {
                /* 'AttitudeEKF2grav:169' Q=diag([ q_rotSpeed,q_rotSpeed,q_rotSpeed,... */
                /* 'AttitudeEKF2grav:170'         q_rotAcc,q_rotAcc,q_rotAcc,... */
                /* 'AttitudeEKF2grav:171'         q_acc,q_acc,q_acc,... */
                /* 'AttitudeEKF2grav:172'         q_mag,q_mag,q_mag]); */
                b_q_rotSpeed[0] = q_rotSpeed;
                b_q_rotSpeed[1] = q_rotSpeed;
                b_q_rotSpeed[2] = q_rotSpeed;
                b_q_rotSpeed[3] = q_rotAcc;
                b_q_rotSpeed[4] = q_rotAcc;
                b_q_rotSpeed[5] = q_rotAcc;
                b_q_rotSpeed[6] = q_acc;
                b_q_rotSpeed[7] = q_acc;
                b_q_rotSpeed[8] = q_acc;
                b_q_rotSpeed[9] = q_mag;
                b_q_rotSpeed[10] = q_mag;
                b_q_rotSpeed[11] = q_mag;
                diag(b_q_rotSpeed, Q);
                Q_not_empty = true;
        }

        /* 'AttitudeEKF2grav:175' P_apr=A_lin*P_apo*A_lin'+Q; */
        for (i = 0; i < 12; i++) {
                for (i0 = 0; i0 < 12; i0++) {
                        A_lin[i + 12 * i0] = 0.0F;
                        for (i1 = 0; i1 < 12; i1++) {
                                A_lin[i + 12 * i0] += b_A_lin[i + 12 * i1] * P_apo[i1 + 12 * i0];
                        }
                }
        }

        for (i = 0; i < 12; i++) {
                for (i0 = 0; i0 < 12; i0++) {
                        f0 = 0.0F;
                        for (i1 = 0; i1 < 12; i1++) {
                                f0 += A_lin[i + 12 * i1] * b_A_lin[i0 + 12 * i1];
                        }

                        P_apr[i + 12 * i0] = f0 + (float)Q[i + 12 * i0];
                }
        }

        /* % update */
        /* % update */
        /* 'AttitudeEKF2grav:179' if zFlag(1)==1&&zFlag(2)==1&&zFlag(3)==1&&zFlag(4)==1 */
        if ((zFlag[0] == 1) && (zFlag[1] == 1) && (zFlag[2] == 1) && (zFlag[3] == 1))
        {
                /*      R=[ r_gyro,0,0,0,0,0,0,0,0,0,0,0,0,0,0; */
                /*          0,r_gyro,0,0,0,0,0,0,0,0,0,0,0,0,0; */
                /*          0,0,r_gyro,0,0,0,0,0,0,0,0,0,0,0,0; */
                /*          0,0,0,r_accel,0,0,0,0,0,0,0,0,0,0,0; */
                /*          0,0,0,0,r_accel,0,0,0,0,0,0,0,0,0,0; */
                /*          0,0,0,0,0,r_accel,0,0,0,0,0,0,0,0,0; */
                /*          0,0,0,0,0,0,r_mag,0,0,0,0,0,0,0,0; */
                /*          0,0,0,0,0,0,0,r_mag,0,0,0,0,0,0,0; */
                /*          0,0,0,0,0,0,0,0,r_mag,0,0,0,0,0,0; */
                /*          0,0,0,0,0,0,0,0,0,         ,0,0,0;     */
                /*          0,0,0,0,0,0,0,0,0,  r_ptam ,0,0,0; */
                /*          0,0,0,0,0,0,0,0,0,    3x3  ,0,0,0; */
                /* 'AttitudeEKF2grav:193' R_v=[r_gyro,r_gyro,r_gyro,r_accel,r_accel,r_accel,r_ptam(1),r_ptam(1),r_ptam(1),r_ptam(1),r_ptam(1),r_ptam(1)]; */
                /* observation matrix */
                /* [zw;ze;zmk]; */
                /* 'AttitudeEKF2grav:196' H_k=[  E,     Z,      Z,    Z;  */
                /* 'AttitudeEKF2grav:197'            Z,     Z,      E,    Z; */
                /* 'AttitudeEKF2grav:198'            Z,     Z,      E,    Z; */
                /* 'AttitudeEKF2grav:199'            Z,     Z,      Z,    E]; */
                /* Fix H_k to use one more measurement */
                /* 'AttitudeEKF2grav:202' y_k=z(1:12)-H_k*x_apr; */
                /* S_k=H_k*P_apr*H_k'+R; */
                /* 'AttitudeEKF2grav:206' S_k=H_k*P_apr*H_k'; */
                for (i = 0; i < 12; i++) {
                        for (i0 = 0; i0 < 12; i0++) {
                                A_lin[i + 12 * i0] = 0.0F;
                                for (i1 = 0; i1 < 12; i1++) {
                                        A_lin[i + 12 * i0] += (float)a[i + 12 * i1] * P_apr[i1 + 12 * i0];
                                }
                        }

                        for (i0 = 0; i0 < 12; i0++) {
                                b_A_lin[i + 12 * i0] = 0.0F;
                                for (i1 = 0; i1 < 12; i1++) {
                                        b_A_lin[i + 12 * i0] += A_lin[i + 12 * i1] * (float)b[i1 + 12 * i0];
                                }
                        }
                }

                /* 'AttitudeEKF2grav:207' S_k(1:12+1:end) = S_k(1:12+1:end) + R_v; */
                b_r_gyro[0] = (float)r_gyro;
                b_r_gyro[1] = (float)r_gyro;
                b_r_gyro[2] = (float)r_gyro;
                b_r_gyro[3] = (float)r_accel;
                b_r_gyro[4] = (float)r_accel;
                b_r_gyro[5] = (float)r_accel;
                b_r_gyro[6] = (float)r_ptam;
                b_r_gyro[7] = (float)r_ptam;
                b_r_gyro[8] = (float)r_ptam;
                b_r_gyro[9] = (float)r_ptam;
                b_r_gyro[10] = (float)r_ptam;
                b_r_gyro[11] = (float)r_ptam;
                for (i = 0; i < 12; i++) {
                        c_A_lin[i] = b_A_lin[13 * i] + b_r_gyro[i];
                }

                for (i = 0; i < 12; i++) {
                        b_A_lin[13 * i] = c_A_lin[i];
                }

                /* 'AttitudeEKF2grav:208' K_k=(P_apr*H_k'/(S_k)); */
                for (i = 0; i < 12; i++) {
                        for (i0 = 0; i0 < 12; i0++) {
                                K_k[i + 12 * i0] = 0.0F;
                                for (i1 = 0; i1 < 12; i1++) {
                                        K_k[i + 12 * i0] += P_apr[i + 12 * i1] * (float)b[i1 + 12 * i0];
                                }
                        }
                }

                b_mrdivide(K_k, b_A_lin);

                /* 'AttitudeEKF2grav:211' x_apo=x_apr+K_k*y_k; */
                for (i = 0; i < 12; i++) {
                        f0 = 0.0F;
                        for (i0 = 0; i0 < 12; i0++) {
                                f0 += (float)a[i + 12 * i0] * x_apr[i0];
                        }

                        b_r_gyro[i] = (float)z[i] - f0;
                }

                for (i = 0; i < 12; i++) {
                        f0 = 0.0F;
                        for (i0 = 0; i0 < 12; i0++) {
                                f0 += K_k[i + 12 * i0] * b_r_gyro[i0];
                        }

                        x_apo[i] = x_apr[i] + f0;
                }

                /* 'AttitudeEKF2grav:212' P_apo=(eye(12)-K_k*H_k)*P_apr; */
                eye(dv1);
                for (i = 0; i < 12; i++) {
                        for (i0 = 0; i0 < 12; i0++) {
                                f0 = 0.0F;
                                for (i1 = 0; i1 < 12; i1++) {
                                        f0 += K_k[i + 12 * i1] * (float)a[i1 + 12 * i0];
                                }

                                b_A_lin[i + 12 * i0] = (float)dv1[i + 12 * i0] - f0;
                        }
                }

                for (i = 0; i < 12; i++) {
                        for (i0 = 0; i0 < 12; i0++) {
                                P_apo[i + 12 * i0] = 0.0F;
                                for (i1 = 0; i1 < 12; i1++) {
                                        P_apo[i + 12 * i0] += b_A_lin[i + 12 * i1] * P_apr[i1 + 12 * i0];
                                }
                        }
                }
        } else {
                /* 'AttitudeEKF2grav:213' else */
                /* 'AttitudeEKF2grav:214' if zFlag(1)==1&&zFlag(2)==1&&zFlag(3)==1 */
                if ((zFlag[0] == 1) && (zFlag[1] == 1) && (zFlag[2] == 1)) {
                        /*      R=[r_gyro,0,0,0,0,0,0,0,0; */
                        /*          0,r_gyro,0,0,0,0,0,0,0; */
                        /*          0,0,r_gyro,0,0,0,0,0,0; */
                        /*          0,0,0,r_accel,0,0,0,0,0; */
                        /*          0,0,0,0,r_accel,0,0,0,0; */
                        /*          0,0,0,0,0,r_accel,0,0,0; */
                        /*          0,0,0,0,0,0,r_mag,0,0; */
                        /*          0,0,0,0,0,0,0,r_mag,0; */
                        /*          0,0,0,0,0,0,0,0,r_mag]; */
                        /* 'AttitudeEKF2grav:225' R_v=[r_gyro,r_gyro,r_gyro,r_accel,r_accel,r_accel,r_ptam(1),r_ptam(1),r_ptam(1)]; */
                        /* observation matrix */
                        /* [zw;ze;zmk]; */
                        /* 'AttitudeEKF2grav:228' H_k=[  E,     Z,      Z,    Z; */
                        /* 'AttitudeEKF2grav:229'             Z,     Z,      E,    Z; */
                        /* 'AttitudeEKF2grav:230'             Z,     Z,      Z,    E]; */
                        /* 'AttitudeEKF2grav:231' y_k=z(1:9)-H_k*x_apr; */
                        /* S_k=H_k*P_apr*H_k'+R; */
                        /* 'AttitudeEKF2grav:235' S_k=H_k*P_apr*H_k'; */
                        for (i = 0; i < 9; i++) {
                                for (i0 = 0; i0 < 12; i0++) {
                                        b_a[i + 9 * i0] = 0.0F;
                                        for (i1 = 0; i1 < 12; i1++) {
                                                b_a[i + 9 * i0] += (float)c_a[i + 9 * i1] * P_apr[i1 + 12 * i0];
                                        }
                                }

                                for (i0 = 0; i0 < 9; i0++) {
                                        S_k[i + 9 * i0] = 0.0F;
                                        for (i1 = 0; i1 < 12; i1++) {
                                                S_k[i + 9 * i0] += b_a[i + 9 * i1] * (float)b_b[i1 + 12 * i0];
                                        }
                                }
                        }

                        /* 'AttitudeEKF2grav:236' S_k(1:9+1:end) = S_k(1:9+1:end) + R_v; */
                        c_r_gyro[0] = (float)r_gyro;
                        c_r_gyro[1] = (float)r_gyro;
                        c_r_gyro[2] = (float)r_gyro;
                        c_r_gyro[3] = (float)r_accel;
                        c_r_gyro[4] = (float)r_accel;
                        c_r_gyro[5] = (float)r_accel;
                        c_r_gyro[6] = (float)r_ptam;
                        c_r_gyro[7] = (float)r_ptam;
                        c_r_gyro[8] = (float)r_ptam;
                        for (i = 0; i < 9; i++) {
                                O[i] = S_k[10 * i] + c_r_gyro[i];
                        }

                        for (i = 0; i < 9; i++) {
                                S_k[10 * i] = O[i];
                        }

                        /* 'AttitudeEKF2grav:237' K_k=(P_apr*H_k'/(S_k)); */
                        for (i = 0; i < 12; i++) {
                                for (i0 = 0; i0 < 9; i0++) {
                                        b_K_k[i + 12 * i0] = 0.0F;
                                        for (i1 = 0; i1 < 12; i1++) {
                                                b_K_k[i + 12 * i0] += P_apr[i + 12 * i1] * (float)b_b[i1 + 12 * i0];
                                        }
                                }
                        }

                        c_mrdivide(b_K_k, S_k);

                        /* 'AttitudeEKF2grav:240' x_apo=x_apr+K_k*y_k; */
                        for (i = 0; i < 9; i++) {
                                f0 = 0.0F;
                                for (i0 = 0; i0 < 12; i0++) {
                                        f0 += (float)c_a[i + 9 * i0] * x_apr[i0];
                                }

                                O[i] = (float)z[i] - f0;
                        }

                        for (i = 0; i < 12; i++) {
                                f0 = 0.0F;
                                for (i0 = 0; i0 < 9; i0++) {
                                        f0 += b_K_k[i + 12 * i0] * O[i0];
                                }

                                x_apo[i] = x_apr[i] + f0;
                        }

                        /* 'AttitudeEKF2grav:241' P_apo=(eye(12)-K_k*H_k)*P_apr; */
                        eye(dv1);
                        for (i = 0; i < 12; i++) {
                                for (i0 = 0; i0 < 12; i0++) {
                                        f0 = 0.0F;
                                        for (i1 = 0; i1 < 9; i1++) {
                                                f0 += b_K_k[i + 12 * i1] * (float)c_a[i1 + 9 * i0];
                                        }

                                        b_A_lin[i + 12 * i0] = (float)dv1[i + 12 * i0] - f0;
                                }
                        }

                        for (i = 0; i < 12; i++) {
                                for (i0 = 0; i0 < 12; i0++) {
                                        P_apo[i + 12 * i0] = 0.0F;
                                        for (i1 = 0; i1 < 12; i1++) {
                                                P_apo[i + 12 * i0] += b_A_lin[i + 12 * i1] * P_apr[i1 + 12 * i0];
                                        }
                                }
                        }
                } else {
                        /* 'AttitudeEKF2grav:242' else */
                        /* 'AttitudeEKF2grav:243' if zFlag(1)==1&&zFlag(2)==0&&zFlag(3)==0 */
                        if ((zFlag[0] == 1) && (zFlag[1] == 0) && (zFlag[2] == 0)) {
                                /* R=[r_gyro,0,0; */
                                /*     0,r_gyro,0; */
                                /*     0,0,r_gyro]; */
                                /* 'AttitudeEKF2grav:248' R_v=[r_gyro,r_gyro,r_gyro]; */
                                /* observation matrix */
                                /* 'AttitudeEKF2grav:251' H_k=[  E,     Z,      Z,    Z]; */
                                /* 'AttitudeEKF2grav:253' y_k=z(1:3)-H_k(1:3,1:12)*x_apr; */
                                /*  S_k=H_k(1:3,1:12)*P_apr*H_k(1:3,1:12)'+R(1:3,1:3); */
                                /* 'AttitudeEKF2grav:256' S_k=H_k(1:3,1:12)*P_apr*H_k(1:3,1:12)'; */
                                for (i = 0; i < 3; i++) {
                                        for (i0 = 0; i0 < 12; i0++) {
                                                b_S_k[i + 3 * i0] = 0.0F;
                                                for (i1 = 0; i1 < 12; i1++) {
                                                        b_S_k[i + 3 * i0] += (float)d_a[i + 3 * i1] * P_apr[i1 + 12 * i0];
                                                }
                                        }

                                        for (i0 = 0; i0 < 3; i0++) {
                                                O[i + 3 * i0] = 0.0F;
                                                for (i1 = 0; i1 < 12; i1++) {
                                                        O[i + 3 * i0] += b_S_k[i + 3 * i1] * (float)c_b[i1 + 12 * i0];
                                                }
                                        }
                                }

                                /* 'AttitudeEKF2grav:257' S_k(1:3+1:end) = S_k(1:3+1:end) + R_v; */
                                d_r_gyro[0] = (float)r_gyro;
                                d_r_gyro[1] = (float)r_gyro;
                                d_r_gyro[2] = (float)r_gyro;
                                for (i = 0; i < 3; i++) {
                                        zek[i] = O[i << 2] + d_r_gyro[i];
                                }

                                for (i = 0; i < 3; i++) {
                                        O[i << 2] = zek[i];
                                }

                                /* 'AttitudeEKF2grav:258' K_k=(P_apr*H_k(1:3,1:12)'/(S_k)); */
                                for (i = 0; i < 12; i++) {
                                        for (i0 = 0; i0 < 3; i0++) {
                                                b_S_k[i + 12 * i0] = 0.0F;
                                                for (i1 = 0; i1 < 12; i1++) {
                                                        b_S_k[i + 12 * i0] += P_apr[i + 12 * i1] * (float)c_b[i1 + 12 * i0];
                                                }
                                        }
                                }

                                mrdivide(b_S_k, O, c_K_k);

                                /* 'AttitudeEKF2grav:261' x_apo=x_apr+K_k*y_k; */
                                for (i = 0; i < 3; i++) {
                                        f0 = 0.0F;
                                        for (i0 = 0; i0 < 12; i0++) {
                                                f0 += (float)d_a[i + 3 * i0] * x_apr[i0];
                                        }

                                        zek[i] = (float)z[i] - f0;
                                }

                                for (i = 0; i < 12; i++) {
                                        f0 = 0.0F;
                                        for (i0 = 0; i0 < 3; i0++) {
                                                f0 += c_K_k[i + 12 * i0] * zek[i0];
                                        }

                                        x_apo[i] = x_apr[i] + f0;
                                }

                                /* 'AttitudeEKF2grav:262' P_apo=(eye(12)-K_k*H_k(1:3,1:12))*P_apr; */
                                eye(dv1);
                                for (i = 0; i < 12; i++) {
                                        for (i0 = 0; i0 < 12; i0++) {
                                                f0 = 0.0F;
                                                for (i1 = 0; i1 < 3; i1++) {
                                                        f0 += c_K_k[i + 12 * i1] * (float)d_a[i1 + 3 * i0];
                                                }

                                                b_A_lin[i + 12 * i0] = (float)dv1[i + 12 * i0] - f0;
                                        }
                                }

                                for (i = 0; i < 12; i++) {
                                        for (i0 = 0; i0 < 12; i0++) {
                                                P_apo[i + 12 * i0] = 0.0F;
                                                for (i1 = 0; i1 < 12; i1++) {
                                                        P_apo[i + 12 * i0] += b_A_lin[i + 12 * i1] * P_apr[i1 + 12 * i0];
                                                }
                                        }
                                }
                        } else {
                                /* 'AttitudeEKF2grav:263' else */
                                /* 'AttitudeEKF2grav:264' if  zFlag(1)==1&&zFlag(2)==1&&zFlag(3)==0 */
                                if ((zFlag[0] == 1) && (zFlag[1] == 1) && (zFlag[2] == 0)) {
                                        /*              R=[r_gyro,0,0,0,0,0; */
                                        /*                  0,r_gyro,0,0,0,0; */
                                        /*                  0,0,r_gyro,0,0,0; */
                                        /*                  0,0,0,r_accel,0,0; */
                                        /*                  0,0,0,0,r_accel,0; */
                                        /*                  0,0,0,0,0,r_accel]; */
                                        /* 'AttitudeEKF2grav:273' R_v=[r_gyro,r_gyro,r_gyro,r_accel,r_accel,r_accel]; */
                                        /* observation matrix */
                                        /* 'AttitudeEKF2grav:276' H_k=[  E,     Z,      Z,    Z; */
                                        /* 'AttitudeEKF2grav:277'                     Z,     Z,      E,    Z]; */
                                        /* 'AttitudeEKF2grav:279' y_k=z(1:6)-H_k(1:6,1:12)*x_apr; */
                                        /*  S_k=H_k(1:6,1:12)*P_apr*H_k(1:6,1:12)'+R(1:6,1:6); */
                                        /* 'AttitudeEKF2grav:282' S_k=H_k(1:6,1:12)*P_apr*H_k(1:6,1:12)'; */
                                        for (i = 0; i < 6; i++) {
                                                for (i0 = 0; i0 < 12; i0++) {
                                                        e_a[i + 6 * i0] = 0.0F;
                                                        for (i1 = 0; i1 < 12; i1++) {
                                                                e_a[i + 6 * i0] += (float)f_a[i + 6 * i1] * P_apr[i1 + 12 * i0];
                                                        }
                                                }

                                                for (i0 = 0; i0 < 6; i0++) {
                                                        b_S_k[i + 6 * i0] = 0.0F;
                                                        for (i1 = 0; i1 < 12; i1++) {
                                                                b_S_k[i + 6 * i0] += e_a[i + 6 * i1] * (float)d_b[i1 + 12 * i0];
                                                        }
                                                }
                                        }

                                        /* 'AttitudeEKF2grav:283' S_k(1:6+1:end) = S_k(1:6+1:end) + R_v; */
                                        e_r_gyro[0] = (float)r_gyro;
                                        e_r_gyro[1] = (float)r_gyro;
                                        e_r_gyro[2] = (float)r_gyro;
                                        e_r_gyro[3] = (float)r_accel;
                                        e_r_gyro[4] = (float)r_accel;
                                        e_r_gyro[5] = (float)r_accel;
                                        for (i = 0; i < 6; i++) {
                                                c_S_k[i] = b_S_k[7 * i] + e_r_gyro[i];
                                        }

                                        for (i = 0; i < 6; i++) {
                                                b_S_k[7 * i] = c_S_k[i];
                                        }

                                        /* 'AttitudeEKF2grav:284' K_k=(P_apr*H_k(1:6,1:12)'/(S_k)); */
                                        for (i = 0; i < 12; i++) {
                                                for (i0 = 0; i0 < 6; i0++) {
                                                        d_K_k[i + 12 * i0] = 0.0F;
                                                        for (i1 = 0; i1 < 12; i1++) {
                                                                d_K_k[i + 12 * i0] += P_apr[i + 12 * i1] * (float)d_b[i1 + 12 *
                                                                                                                      i0];
                                                        }
                                                }
                                        }

                                        d_mrdivide(d_K_k, b_S_k);

                                        /* 'AttitudeEKF2grav:287' x_apo=x_apr+K_k*y_k; */
                                        for (i = 0; i < 6; i++) {
                                                f0 = 0.0F;
                                                for (i0 = 0; i0 < 12; i0++) {
                                                        f0 += (float)f_a[i + 6 * i0] * x_apr[i0];
                                                }

                                                e_r_gyro[i] = (float)z[i] - f0;
                                        }

                                        for (i = 0; i < 12; i++) {
                                                f0 = 0.0F;
                                                for (i0 = 0; i0 < 6; i0++) {
                                                        f0 += d_K_k[i + 12 * i0] * e_r_gyro[i0];
                                                }

                                                x_apo[i] = x_apr[i] + f0;
                                        }

                                        /* 'AttitudeEKF2grav:288' P_apo=(eye(12)-K_k*H_k(1:6,1:12))*P_apr; */
                                        eye(dv1);
                                        for (i = 0; i < 12; i++) {
                                                for (i0 = 0; i0 < 12; i0++) {
                                                        f0 = 0.0F;
                                                        for (i1 = 0; i1 < 6; i1++) {
                                                                f0 += d_K_k[i + 12 * i1] * (float)f_a[i1 + 6 * i0];
                                                        }

                                                        b_A_lin[i + 12 * i0] = (float)dv1[i + 12 * i0] - f0;
                                                }
                                        }

                                        for (i = 0; i < 12; i++) {
                                                for (i0 = 0; i0 < 12; i0++) {
                                                        P_apo[i + 12 * i0] = 0.0F;
                                                        for (i1 = 0; i1 < 12; i1++) {
                                                                P_apo[i + 12 * i0] += b_A_lin[i + 12 * i1] * P_apr[i1 + 12 * i0];
                                                        }
                                                }
                                        }
                                } else {
                                        /* 'AttitudeEKF2grav:289' else */
                                        /* 'AttitudeEKF2grav:290' if  zFlag(1)==1&&zFlag(2)==0&&zFlag(3)==1 */
                                        if ((zFlag[0] == 1) && (zFlag[1] == 0) && (zFlag[2] == 1)) {
                                                /*                  R=[r_gyro,0,0,0,0,0; */
                                                /*                      0,r_gyro,0,0,0,0; */
                                                /*                      0,0,r_gyro,0,0,0; */
                                                /*                      0,0,0,r_mag,0,0; */
                                                /*                      0,0,0,0,r_mag,0; */
                                                /*                      0,0,0,0,0,r_mag]; */
                                                /* 'AttitudeEKF2grav:297' R_v=[r_gyro,r_gyro,r_gyro,r_ptam(1),r_ptam(1),r_ptam(1)]; */
                                                /* observation matrix */
                                                /* 'AttitudeEKF2grav:300' H_k=[  E,     Z,      Z,    Z; */
                                                /* 'AttitudeEKF2grav:301'                         Z,     Z,      Z,    E]; */
                                                /* 'AttitudeEKF2grav:303' y_k=[z(1:3);z(7:9)]-H_k(1:6,1:12)*x_apr; */
                                                /* S_k=H_k(1:6,1:12)*P_apr*H_k(1:6,1:12)'+R(1:6,1:6); */
                                                /* 'AttitudeEKF2grav:306' S_k=H_k(1:6,1:12)*P_apr*H_k(1:6,1:12)' */
                                                for (i = 0; i < 6; i++) {
                                                        for (i0 = 0; i0 < 12; i0++) {
                                                                e_a[i + 6 * i0] = 0.0F;
                                                                for (i1 = 0; i1 < 12; i1++) {
                                                                        e_a[i + 6 * i0] += (float)g_a[i + 6 * i1] * P_apr[i1 + 12 * i0];
                                                                }
                                                        }

                                                        for (i0 = 0; i0 < 6; i0++) {
                                                                b_S_k[i + 6 * i0] = 0.0F;
                                                                for (i1 = 0; i1 < 12; i1++) {
                                                                        b_S_k[i + 6 * i0] += e_a[i + 6 * i1] * (float)e_b[i1 + 12 * i0];
                                                                }
                                                        }
                                                }

                                                /* 'AttitudeEKF2grav:307' S_k(1:6+1:end) = S_k(1:6+1:end) + R_v; */
                                                e_r_gyro[0] = (float)r_gyro;
                                                e_r_gyro[1] = (float)r_gyro;
                                                e_r_gyro[2] = (float)r_gyro;
                                                e_r_gyro[3] = (float)r_ptam;
                                                e_r_gyro[4] = (float)r_ptam;
                                                e_r_gyro[5] = (float)r_ptam;
                                                for (i = 0; i < 6; i++) {
                                                        c_S_k[i] = b_S_k[7 * i] + e_r_gyro[i];
                                                }

                                                for (i = 0; i < 6; i++) {
                                                        b_S_k[7 * i] = c_S_k[i];
                                                }

                                                /* 'AttitudeEKF2grav:308' K_k=(P_apr*H_k(1:6,1:12)'/(S_k)); */
                                                for (i = 0; i < 12; i++) {
                                                        for (i0 = 0; i0 < 6; i0++) {
                                                                d_K_k[i + 12 * i0] = 0.0F;
                                                                for (i1 = 0; i1 < 12; i1++) {
                                                                        d_K_k[i + 12 * i0] += P_apr[i + 12 * i1] * (float)e_b[i1 + 12 *
                                                                                                                              i0];
                                                                }
                                                        }
                                                }

                                                d_mrdivide(d_K_k, b_S_k);

                                                /* 'AttitudeEKF2grav:311' x_apo=x_apr+K_k*y_k; */
                                                for (i = 0; i < 3; i++) {
                                                        e_r_gyro[i] = (float)z[i];
                                                }

                                                for (i = 0; i < 3; i++) {
                                                        e_r_gyro[i + 3] = (float)z[6 + i];
                                                }

                                                for (i = 0; i < 6; i++) {
                                                        c_S_k[i] = 0.0F;
                                                        for (i0 = 0; i0 < 12; i0++) {
                                                                c_S_k[i] += (float)g_a[i + 6 * i0] * x_apr[i0];
                                                        }

                                                        b_z[i] = e_r_gyro[i] - c_S_k[i];
                                                }

                                                for (i = 0; i < 12; i++) {
                                                        f0 = 0.0F;
                                                        for (i0 = 0; i0 < 6; i0++) {
                                                                f0 += d_K_k[i + 12 * i0] * b_z[i0];
                                                        }

                                                        x_apo[i] = x_apr[i] + f0;
                                                }

                                                /* 'AttitudeEKF2grav:312' P_apo=(eye(12)-K_k*H_k(1:6,1:12))*P_apr; */
                                                eye(dv1);
                                                for (i = 0; i < 12; i++) {
                                                        for (i0 = 0; i0 < 12; i0++) {
                                                                f0 = 0.0F;
                                                                for (i1 = 0; i1 < 6; i1++) {
                                                                        f0 += d_K_k[i + 12 * i1] * (float)g_a[i1 + 6 * i0];
                                                                }

                                                                b_A_lin[i + 12 * i0] = (float)dv1[i + 12 * i0] - f0;
                                                        }
                                                }

                                                for (i = 0; i < 12; i++) {
                                                        for (i0 = 0; i0 < 12; i0++) {
                                                                P_apo[i + 12 * i0] = 0.0F;
                                                                for (i1 = 0; i1 < 12; i1++) {
                                                                        P_apo[i + 12 * i0] += b_A_lin[i + 12 * i1] * P_apr[i1 + 12 *
                                                                                                                           i0];
                                                                }
                                                        }
                                                }
                                        } else {
                                                /* 'AttitudeEKF2grav:313' else */
                                                /* 'AttitudeEKF2grav:314' x_apo=x_apr; */
                                                for (i = 0; i < 12; i++) {
                                                        x_apo[i] = x_apr[i];
                                                }

                                                /* 'AttitudeEKF2grav:315' P_apo=P_apr; */
                                                memcpy(&P_apo[0], &P_apr[0], 144U * sizeof(float));
                                        }
                                }
                        }
                }
        }

        /* % euler anglels extraction */
        /* 'AttitudeEKF2grav:324' z_n_b = -x_apo(7:9)./norm(x_apo(7:9)); */
        for (i = 0; i < 3; i++) {
                fv0[i] = -x_apo[i + 6];
        }

        rdivide(fv0, norm(*(float (*)[3])&x_apo[6]), wak);

        /* 'AttitudeEKF2grav:325' m_n_b = x_apo(10:12)./norm(x_apo(10:12)); */
        rdivide(*(float (*)[3])&x_apo[9], norm(*(float (*)[3])&x_apo[9]), b_zek);

        /* 'AttitudeEKF2grav:327' y_n_b=cross(z_n_b,m_n_b); */
        for (i = 0; i < 3; i++) {
                zek[i] = b_zek[i];
        }

        cross(wak, zek, b_zek);

        /* 'AttitudeEKF2grav:328' y_n_b=y_n_b./norm(y_n_b); */
        for (i = 0; i < 3; i++) {
                zek[i] = b_zek[i];
        }

        rdivide(zek, norm(b_zek), b_zek);

        /* 'AttitudeEKF2grav:330' x_n_b=(cross(y_n_b,z_n_b)); */
        cross(b_zek, wak, muk);

        /* 'AttitudeEKF2grav:331' x_n_b=x_n_b./norm(x_n_b); */
        for (i = 0; i < 3; i++) {
                zek[i] = muk[i];
        }

        rdivide(zek, norm(muk), muk);

        /* 'AttitudeEKF2grav:334' xa_apo=x_apo; */
        for (i = 0; i < 12; i++) {
                xa_apo[i] = x_apo[i];
        }

        /* 'AttitudeEKF2grav:335' Pa_apo=P_apo; */
        memcpy(&Pa_apo[0], &P_apo[0], 144U * sizeof(float));

        /*  rotation matrix from earth to body system */
        /* 'AttitudeEKF2grav:337' Rot_matrix=[x_n_b,y_n_b,z_n_b]; */
        for (i = 0; i < 3; i++) {
                Rot_matrix[i] = muk[i];
                Rot_matrix[3 + i] = b_zek[i];
                Rot_matrix[6 + i] = wak[i];
        }

        /* 'AttitudeEKF2grav:340' phi=atan2(Rot_matrix(2,3),Rot_matrix(3,3)); */
        /* 'AttitudeEKF2grav:341' theta=-asin(Rot_matrix(1,3)); */
        /* 'AttitudeEKF2grav:342' psi=atan2(Rot_matrix(1,2),Rot_matrix(1,1)); */
        /* 'AttitudeEKF2grav:343' eulerAngles=[phi;theta;psi]; */
        eulerAngles[0] = (real32_T)atan2(Rot_matrix[7], Rot_matrix[8]);
        eulerAngles[1] = -(real32_T)asin(Rot_matrix[6]);
        eulerAngles[2] = (real32_T)atan2(Rot_matrix[3], Rot_matrix[0]);

        /* making a quaternion */
        /* 'AttitudeEKF2grav:346' q0 = [0.5*sqrt(Rot_matrix(1,1)+Rot_matrix(2,2)+Rot_matrix(3,3)+1)]; */
        q0 = 0.5F * (real32_T)sqrt(((Rot_matrix[0] + Rot_matrix[4]) + Rot_matrix[8]) +
                                   1.0F);

        /* 'AttitudeEKF2grav:347' q = [q0;... */
        /* 'AttitudeEKF2grav:348'     sign(q0)*((Rot_matrix(2,3)-Rot_matrix(3,2))/4*q0);... */
        /* 'AttitudeEKF2grav:349'      sign(q0)*((Rot_matrix(3,1)-Rot_matrix(1,3))/4*q0);... */
        /* 'AttitudeEKF2grav:350'      sign(q0)*((Rot_matrix(1,2)-Rot_matrix(2,1))/4*q0)]; */
        f0 = q0;
        b_sign(&f0);
        f1 = q0;
        b_sign(&f1);
        f2 = q0;
        b_sign(&f2);
        q[0] = q0;
        q[1] = f0 * ((Rot_matrix[7] - Rot_matrix[5]) / 4.0F * q0);
        q[2] = f1 * ((Rot_matrix[2] - Rot_matrix[6]) / 4.0F * q0);
        q[3] = f2 * ((Rot_matrix[3] - Rot_matrix[1]) / 4.0F * q0);

        /* radtodeg([atan2(2*(q(1)*q(2)+q(3)*q(4)),(q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2)) ;... */
        /* -asin(2*(q(2)*q(4)-q(1)*q(3)));... */
        /* atan2(2*(q(1)*q(4)+q(2)*q(3)),(q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2))]) */
        /* q = [cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2);... */
        /*      sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2);... */
        /*      cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2);... */
        /*      cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2)]; */
        /* radtodeg([atan2(2*(q2(1)*q2(2)+q2(3)*q2(4)),(q2(1)^2 - q2(2)^2 - q2(3)^2 + q2(4)^2)) ;... */
        /* -asin(2*(q2(2)*q2(4)-q2(1)*q2(3)));... */
        /* atan2(2*(q2(1)*q2(4)+q2(2)*q2(3)),(q2(1)^2 + q2(2)^2 - q2(3)^2 - q2(4)^2))]); */
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void AttitudeEKF2grav_initialize(void)
{
        Q_not_empty = false;
        Ji_not_empty = false;
        AttitudeEKF2grav_init();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void AttitudeEKF2grav_terminate(void)
{
        /* (no terminate code required) */
}

/*
 * File trailer for AttitudeEKF2grav.c
 *
 * [EOF]
 */
