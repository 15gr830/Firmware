/*
 * File: posEKF.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 17-May-2015 11:18:31
 */

/* Include files */
#include "posEKF.h"

/* Variable Definitions */
static float x_apo[9];
static float P_apo[81];
static double Q[81];
static boolean_T Q_not_empty;

/* Function Declarations */
static void b_mrdivide(float A[54], const float B[36]);
static void mrdivide(float A[81], const float B[81]);
static void posEKF_init(void);

/* Function Definitions */

/*
 * Arguments    : float A[54]
 *                const float B[36]
 * Return Type  : void
 */
static void b_mrdivide(float A[54], const float B[36])
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
                jBcol = 9 * j;
                jAcol = 6 * j;
                for (k = 1; k <= j; k++) {
                        kBcol = 9 * (k - 1);
                        if (b_A[(k + jAcol) - 1] != 0.0F) {
                                for (i = 0; i < 9; i++) {
                                        A[i + jBcol] -= b_A[(k + jAcol) - 1] * A[i + kBcol];
                                }
                        }
                }

                temp = 1.0F / b_A[j + jAcol];
                for (i = 0; i < 9; i++) {
                        A[i + jBcol] *= temp;
                }
        }

        for (j = 5; j > -1; j += -1) {
                jBcol = 9 * j;
                jAcol = 6 * j - 1;
                for (k = j + 2; k < 7; k++) {
                        kBcol = 9 * (k - 1);
                        if (b_A[k + jAcol] != 0.0F) {
                                for (i = 0; i < 9; i++) {
                                        A[i + jBcol] -= b_A[k + jAcol] * A[i + kBcol];
                                }
                        }
                }
        }

        for (jAcol = 4; jAcol > -1; jAcol += -1) {
                if (ipiv[jAcol] != jAcol + 1) {
                        for (jBcol = 0; jBcol < 9; jBcol++) {
                                temp = A[jBcol + 9 * jAcol];
                                A[jBcol + 9 * jAcol] = A[jBcol + 9 * (ipiv[jAcol] - 1)];
                                A[jBcol + 9 * (ipiv[jAcol] - 1)] = temp;
                        }
                }
        }
}

/*
 * Arguments    : float A[81]
 *                const float B[81]
 * Return Type  : void
 */
static void mrdivide(float A[81], const float B[81])
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
                jBcol = 9 * j;
                jAcol = 9 * j;
                for (k = 1; k <= j; k++) {
                        kBcol = 9 * (k - 1);
                        if (b_A[(k + jAcol) - 1] != 0.0F) {
                                for (i = 0; i < 9; i++) {
                                        A[i + jBcol] -= b_A[(k + jAcol) - 1] * A[i + kBcol];
                                }
                        }
                }

                temp = 1.0F / b_A[j + jAcol];
                for (i = 0; i < 9; i++) {
                        A[i + jBcol] *= temp;
                }
        }

        for (j = 8; j > -1; j += -1) {
                jBcol = 9 * j;
                jAcol = 9 * j - 1;
                for (k = j + 2; k < 10; k++) {
                        kBcol = 9 * (k - 1);
                        if (b_A[k + jAcol] != 0.0F) {
                                for (i = 0; i < 9; i++) {
                                        A[i + jBcol] -= b_A[k + jAcol] * A[i + kBcol];
                                }
                        }
                }
        }

        for (jAcol = 7; jAcol > -1; jAcol += -1) {
                if (ipiv[jAcol] != jAcol + 1) {
                        for (jBcol = 0; jBcol < 9; jBcol++) {
                                temp = A[jBcol + 9 * jAcol];
                                A[jBcol + 9 * jAcol] = A[jBcol + 9 * (ipiv[jAcol] - 1)];
                                A[jBcol + 9 * (ipiv[jAcol] - 1)] = temp;
                        }
                }
        }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void posEKF_init(void)
{
        int i;
        for (i = 0; i < 9; i++) {
                x_apo[i] = 0.0F;
        }

        for (i = 0; i < 81; i++) {
                P_apo[i] = 200.0F;
        }
}

/*
 * % model specific parameters
 * Arguments    : const unsigned char zFlag[3]
 *                double dt
 *                const double z[9]
 *                double q_acc
 *                double q_speed
 *                double q_pos
 *                double r_acc
 *                const double r_ptam[3]
 *                double r_got
 *                float xa_apo[9]
 *                float Pa_apo[81]
 *                float debugOutput[4]
 * Return Type  : void
 */
void posEKF(const unsigned char zFlag[3], double dt, const double z[9], double
            q_acc, double q_speed, double q_pos, double r_acc, const double
            r_ptam[3], double r_got, float xa_apo[9], float Pa_apo[81], float
            debugOutput[4])
{
        int i;
        float acck[3];
        float velk[3];
        double a;
        float x_apr[9];
        signed char I[81];
        float A_lin[81];
        int r2;
        static const signed char b_a[27] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        static const signed char E[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

        float b_A_lin[81];
        double v[9];
        int r1;
        float P_apr[81];
        float maxval;
        static const signed char c_a[81] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0,
                                             0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1 };

        static const signed char b[81] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                                           0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

        float S_k[9];
        float c_A_lin[9];
        float K_k[81];
        float b_K_k[27];
        static const signed char b_b[27] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 };

        float b_r_acc[3];
        float y[27];
        int r3;
        float a21;
        float d_a[54];
        static const signed char e_a[54] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                                             1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                             0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

        float b_S_k[36];
        static const signed char c_b[54] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                                             0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                                             0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

        float c_r_acc[6];
        float c_S_k[6];
        float c_K_k[54];
        float b_z[6];

        /* % init */
        /* 'posEKF:9' if(isempty(x_apo)) */
        /* 'posEKF:17' if(isempty(posx)) */
        /* 'posEKF:30' if(isempty(P_apo)) */
        /* 'posEKF:34' debugOutput = single(zeros(4,1)); */
        for (i = 0; i < 4; i++) {
                debugOutput[i] = 0.0F;
        }

        /* % copy the states */
        /* 'posEKF:37' accx=  x_apo(1); */
        /*  x  body acceleration */
        /* 'posEKF:38' accy=  x_apo(2); */
        /*  y  body acceleration */
        /* 'posEKF:39' accz=  x_apo(3); */
        /*  z  body acceleration */
        /* 'posEKF:41' velx=  x_apo(4); */
        /*  x  body velocity */
        /* 'posEKF:42' vely=  x_apo(5); */
        /*  y  body velocity */
        /* 'posEKF:43' velz=  x_apo(6); */
        /*  z  body velocity */
        /* 'posEKF:45' posx =  x_apo(7); */
        /*  x  body position */
        /* 'posEKF:46' posy=  x_apo(8); */
        /*  y  body position */
        /* 'posEKF:47' posz=  x_apo(9); */
        /*  z  body positiony */
        /* % prediction section */
        /*  compute the apriori state estimate from the previous aposteriori estimate */
        /* body accelerations */
        /* 'posEKF:54' acck =[accx;accy;accz]; */
        acck[0] = x_apo[0];
        acck[1] = x_apo[1];
        acck[2] = x_apo[2];

        /* body velocity */
        /* 'posEKF:57' velk =[velx;  vely; velz] + dt*acck; */
        velk[0] = x_apo[3] + (float)dt * x_apo[0];
        velk[1] = x_apo[4] + (float)dt * x_apo[1];
        velk[2] = x_apo[5] + (float)dt * x_apo[2];

        /* body position */
        /* 'posEKF:60' posk = [posx; posy; posz] + dt*velk + 0.5*dt*acck; */
        a = 0.5 * dt;

        /* 'posEKF:63' x_apr=[acck;velk;posk]; */
        for (i = 0; i < 3; i++) {
                x_apr[i] = acck[i];
        }

        for (i = 0; i < 3; i++) {
                x_apr[i + 3] = velk[i];
        }

        x_apr[6] = (x_apo[6] + (float)dt * velk[0]) + (float)a * x_apo[0];
        x_apr[7] = (x_apo[7] + (float)dt * velk[1]) + (float)a * x_apo[1];
        x_apr[8] = (x_apo[8] + (float)dt * velk[2]) + (float)a * x_apo[2];

        /*  compute the apriori error covariance estimate from the previous */
        /* aposteriori estimate */
        /* 'posEKF:68' E=single(eye(3)); */
        /* 'posEKF:69' Z=single(zeros(3)); */
        /* 'posEKF:71' A_lin=[ Z,  Z, Z;... */
        /* 'posEKF:72'         E,  Z,  Z;... */
        /* 'posEKF:73'         0.5*dt*E, E,  Z]; */
        a = 0.5 * dt;

        /* 'posEKF:76' A_lin=eye(9)+A_lin*dt; */
        memset(&I[0], 0, 81U * sizeof(signed char));
        for (i = 0; i < 9; i++) {
                I[i + 9 * i] = 1;
                for (r2 = 0; r2 < 3; r2++) {
                        A_lin[r2 + 9 * i] = 0.0F;
                }

                for (r2 = 0; r2 < 3; r2++) {
                        A_lin[(r2 + 9 * i) + 3] = b_a[r2 + 3 * i];
                }
        }

        for (r2 = 0; r2 < 3; r2++) {
                for (i = 0; i < 3; i++) {
                        A_lin[(i + 9 * r2) + 6] = (float)a * (float)E[i + 3 * r2];
                }
        }

        for (r2 = 0; r2 < 3; r2++) {
                for (i = 0; i < 3; i++) {
                        A_lin[(i + 9 * (r2 + 3)) + 6] = E[i + 3 * r2];
                }
        }

        for (r2 = 0; r2 < 3; r2++) {
                for (i = 0; i < 3; i++) {
                        A_lin[(i + 9 * (r2 + 6)) + 6] = 0.0F;
                }
        }

        for (r2 = 0; r2 < 9; r2++) {
                for (i = 0; i < 9; i++) {
                        b_A_lin[i + 9 * r2] = (float)I[i + 9 * r2] + A_lin[i + 9 * r2] * (float)dt;
                }
        }

        /* process covariance matrix */
        /* 'posEKF:80' if (isempty(Q)) */
        if (!Q_not_empty) {
                /* 'posEKF:81' Q=diag([q_acc,q_acc,q_acc,... */
                /* 'posEKF:82'         q_speed,q_speed,q_speed,... */
                /* 'posEKF:83'         q_pos,q_pos,q_pos]); */
                v[0] = q_acc;
                v[1] = q_acc;
                v[2] = q_acc;
                v[3] = q_speed;
                v[4] = q_speed;
                v[5] = q_speed;
                v[6] = q_pos;
                v[7] = q_pos;
                v[8] = q_pos;
                memset(&Q[0], 0, 81U * sizeof(double));
                for (i = 0; i < 9; i++) {
                        Q[i + 9 * i] = v[i];
                }

                Q_not_empty = true;
        }

        /* 'posEKF:86' P_apr=A_lin*P_apo*A_lin'+Q; */
        for (r2 = 0; r2 < 9; r2++) {
                for (i = 0; i < 9; i++) {
                        A_lin[r2 + 9 * i] = 0.0F;
                        for (r1 = 0; r1 < 9; r1++) {
                                A_lin[r2 + 9 * i] += b_A_lin[r2 + 9 * r1] * P_apo[r1 + 9 * i];
                        }
                }
        }

        for (r2 = 0; r2 < 9; r2++) {
                for (i = 0; i < 9; i++) {
                        maxval = 0.0F;
                        for (r1 = 0; r1 < 9; r1++) {
                                maxval += A_lin[r2 + 9 * r1] * b_A_lin[i + 9 * r1];
                        }

                        P_apr[r2 + 9 * i] = maxval + (float)Q[r2 + 9 * i];
                }
        }

        /* % update */
        /* % update */
        /* 'posEKF:90' if zFlag(1)==1&&zFlag(2)==1&&zFlag(3)==1 */
        if ((zFlag[0] == 1) && (zFlag[1] == 1) && (zFlag[2] == 1)) {
                /*      R=[ r_acc,0,0,0,0,0,0,0,0,0,0,0,0,0,0; */
                /*          0,r_acc,0,0,0,0,0,0,0,0,0,0,0,0,0; */
                /*          0,0,r_acc,0,0,0,0,0,0,0,0,0,0,0,0; */
                /*          0,0,0,r_ptam,0,0,0,0,0,0,0,0,0,0,0; */
                /*          0,0,0,0,r_ptam,0,0,0,0,0,0,0,0,0,0; */
                /*          0,0,0,0,0,r_ptam,0,0,0,0,0,0,0,0,0; */
                /*          0,0,0,0,0,0,r_got,0,0,0,0,0,0,0,0; */
                /*          0,0,0,0,0,0,0,r_got,0,0,0,0,0,0,0; */
                /*          0,0,0,0,0,0,0,0,r_got,0,0,0,0,0,0; */
                /* 'posEKF:101' R_v=[r_acc,r_acc,r_acc,r_ptam(1),r_ptam(2),r_ptam(3),r_got,r_got,r_got]; */
                /* 'posEKF:103' H_k=[  E,     Z,      Z;  */
                /* 'posEKF:104'            Z,     Z,      E; */
                /* 'posEKF:105'            Z,     Z,      E]; */
                /* 'posEKF:107' y_k=z(1:9)-H_k*x_apr; */
                /* S_k=H_k*P_apr*H_k'+R; */
                /* 'posEKF:111' S_k=H_k*P_apr*H_k'; */
                for (r2 = 0; r2 < 9; r2++) {
                        for (i = 0; i < 9; i++) {
                                A_lin[r2 + 9 * i] = 0.0F;
                                for (r1 = 0; r1 < 9; r1++) {
                                        A_lin[r2 + 9 * i] += (float)c_a[r2 + 9 * r1] * P_apr[r1 + 9 * i];
                                }
                        }

                        for (i = 0; i < 9; i++) {
                                b_A_lin[r2 + 9 * i] = 0.0F;
                                for (r1 = 0; r1 < 9; r1++) {
                                        b_A_lin[r2 + 9 * i] += A_lin[r2 + 9 * r1] * (float)b[r1 + 9 * i];
                                }
                        }
                }

                /* 'posEKF:112' S_k(1:9+1:end) = S_k(1:9+1:end) + R_v; */
                S_k[0] = (float)r_acc;
                S_k[1] = (float)r_acc;
                S_k[2] = (float)r_acc;
                S_k[3] = (float)r_ptam[0];
                S_k[4] = (float)r_ptam[1];
                S_k[5] = (float)r_ptam[2];
                S_k[6] = (float)r_got;
                S_k[7] = (float)r_got;
                S_k[8] = (float)r_got;
                for (r2 = 0; r2 < 9; r2++) {
                        c_A_lin[r2] = b_A_lin[10 * r2] + S_k[r2];
                }

                for (r2 = 0; r2 < 9; r2++) {
                        b_A_lin[10 * r2] = c_A_lin[r2];
                }

                /* 'posEKF:113' K_k=(P_apr*H_k'/(S_k)); */
                for (r2 = 0; r2 < 9; r2++) {
                        for (i = 0; i < 9; i++) {
                                K_k[r2 + 9 * i] = 0.0F;
                                for (r1 = 0; r1 < 9; r1++) {
                                        K_k[r2 + 9 * i] += P_apr[r2 + 9 * r1] * (float)b[r1 + 9 * i];
                                }
                        }
                }

                mrdivide(K_k, b_A_lin);

                /* 'posEKF:116' x_apo=x_apr+K_k*y_k; */
                for (r2 = 0; r2 < 9; r2++) {
                        maxval = 0.0F;
                        for (i = 0; i < 9; i++) {
                                maxval += (float)c_a[r2 + 9 * i] * x_apr[i];
                        }

                        S_k[r2] = (float)z[r2] - maxval;
                }

                for (r2 = 0; r2 < 9; r2++) {
                        maxval = 0.0F;
                        for (i = 0; i < 9; i++) {
                                maxval += K_k[r2 + 9 * i] * S_k[i];
                        }

                        x_apo[r2] = x_apr[r2] + maxval;
                }

                /* 'posEKF:117' P_apo=(eye(9)-K_k*H_k)*P_apr; */
                memset(&I[0], 0, 81U * sizeof(signed char));
                for (i = 0; i < 9; i++) {
                        I[i + 9 * i] = 1;
                }

                for (r2 = 0; r2 < 9; r2++) {
                        for (i = 0; i < 9; i++) {
                                maxval = 0.0F;
                                for (r1 = 0; r1 < 9; r1++) {
                                        maxval += K_k[r2 + 9 * r1] * (float)c_a[r1 + 9 * i];
                                }

                                A_lin[r2 + 9 * i] = (float)I[r2 + 9 * i] - maxval;
                        }
                }

                for (r2 = 0; r2 < 9; r2++) {
                        for (i = 0; i < 9; i++) {
                                P_apo[r2 + 9 * i] = 0.0F;
                                for (r1 = 0; r1 < 9; r1++) {
                                        P_apo[r2 + 9 * i] += A_lin[r2 + 9 * r1] * P_apr[r1 + 9 * i];
                                }
                        }
                }
        } else {
                /* 'posEKF:119' else */
                /* 'posEKF:120' if zFlag(1)==1&&zFlag(2)==0&&zFlag(3)==0 */
                if ((zFlag[0] == 1) && (zFlag[1] == 0) && (zFlag[2] == 0)) {
                        /*      R=[ r_acc,0,0; */
                        /*          0,r_acc,0; */
                        /*          0,0,r_acc; */
                        /* 'posEKF:126' R_v=[r_acc,r_acc,r_acc]; */
                        /* observation matrix */
                        /* 'posEKF:129' H_k=[  E,     Z,      Z;]; */
                        /* 'posEKF:131' y_k=z(1:3)-H_k(1:3,1:9)*x_apr; */
                        /* 'posEKF:133' S_k=H_k(1:3,1:9)*P_apr*H_k(1:3,1:9)'; */
                        for (r2 = 0; r2 < 3; r2++) {
                                for (i = 0; i < 9; i++) {
                                        b_K_k[r2 + 3 * i] = 0.0F;
                                        for (r1 = 0; r1 < 9; r1++) {
                                                b_K_k[r2 + 3 * i] += (float)b_a[r2 + 3 * r1] * P_apr[r1 + 9 * i];
                                        }
                                }

                                for (i = 0; i < 3; i++) {
                                        S_k[r2 + 3 * i] = 0.0F;
                                        for (r1 = 0; r1 < 9; r1++) {
                                                S_k[r2 + 3 * i] += b_K_k[r2 + 3 * r1] * (float)b_b[r1 + 9 * i];
                                        }
                                }
                        }

                        /* 'posEKF:134' S_k(1:3+1:end) = S_k(1:3+1:end) + R_v; */
                        b_r_acc[0] = (float)r_acc;
                        b_r_acc[1] = (float)r_acc;
                        b_r_acc[2] = (float)r_acc;
                        for (r2 = 0; r2 < 3; r2++) {
                                acck[r2] = S_k[r2 << 2] + b_r_acc[r2];
                        }

                        for (r2 = 0; r2 < 3; r2++) {
                                S_k[r2 << 2] = acck[r2];
                        }

                        /* 'posEKF:135' K_k=(P_apr*H_k(1:3,1:9)'/(S_k)); */
                        for (r2 = 0; r2 < 9; r2++) {
                                for (i = 0; i < 3; i++) {
                                        y[r2 + 9 * i] = 0.0F;
                                        for (r1 = 0; r1 < 9; r1++) {
                                                y[r2 + 9 * i] += P_apr[r2 + 9 * r1] * (float)b_b[r1 + 9 * i];
                                        }
                                }
                        }

                        r1 = 0;
                        r2 = 1;
                        r3 = 2;
                        maxval = (real32_T)fabs(S_k[0]);
                        a21 = (real32_T)fabs(S_k[1]);
                        if (a21 > maxval) {
                                maxval = a21;
                                r1 = 1;
                                r2 = 0;
                        }

                        if ((real32_T)fabs(S_k[2]) > maxval) {
                                r1 = 2;
                                r2 = 1;
                                r3 = 0;
                        }

                        S_k[r2] /= S_k[r1];
                        S_k[r3] /= S_k[r1];
                        S_k[3 + r2] -= S_k[r2] * S_k[3 + r1];
                        S_k[3 + r3] -= S_k[r3] * S_k[3 + r1];
                        S_k[6 + r2] -= S_k[r2] * S_k[6 + r1];
                        S_k[6 + r3] -= S_k[r3] * S_k[6 + r1];
                        if ((real32_T)fabs(S_k[3 + r3]) > (real32_T)fabs(S_k[3 + r2])) {
                                i = r2;
                                r2 = r3;
                                r3 = i;
                        }

                        S_k[3 + r3] /= S_k[3 + r2];
                        S_k[6 + r3] -= S_k[3 + r3] * S_k[6 + r2];
                        for (i = 0; i < 9; i++) {
                                b_K_k[i + 9 * r1] = y[i] / S_k[r1];
                                b_K_k[i + 9 * r2] = y[9 + i] - b_K_k[i + 9 * r1] * S_k[3 + r1];
                                b_K_k[i + 9 * r3] = y[18 + i] - b_K_k[i + 9 * r1] * S_k[6 + r1];
                                b_K_k[i + 9 * r2] /= S_k[3 + r2];
                                b_K_k[i + 9 * r3] -= b_K_k[i + 9 * r2] * S_k[6 + r2];
                                b_K_k[i + 9 * r3] /= S_k[6 + r3];
                                b_K_k[i + 9 * r2] -= b_K_k[i + 9 * r3] * S_k[3 + r3];
                                b_K_k[i + 9 * r1] -= b_K_k[i + 9 * r3] * S_k[r3];
                                b_K_k[i + 9 * r1] -= b_K_k[i + 9 * r2] * S_k[r2];
                        }

                        /* 'posEKF:138' x_apo=x_apr+K_k*y_k; */
                        for (r2 = 0; r2 < 3; r2++) {
                                maxval = 0.0F;
                                for (i = 0; i < 9; i++) {
                                        maxval += (float)b_a[r2 + 3 * i] * x_apr[i];
                                }

                                acck[r2] = (float)z[r2] - maxval;
                        }

                        for (r2 = 0; r2 < 9; r2++) {
                                maxval = 0.0F;
                                for (i = 0; i < 3; i++) {
                                        maxval += b_K_k[r2 + 9 * i] * acck[i];
                                }

                                x_apo[r2] = x_apr[r2] + maxval;
                        }

                        /* 'posEKF:139' P_apo=(eye(9)-K_k*H_k(1:3,1:9))*P_apr; */
                        memset(&I[0], 0, 81U * sizeof(signed char));
                        for (i = 0; i < 9; i++) {
                                I[i + 9 * i] = 1;
                        }

                        for (r2 = 0; r2 < 9; r2++) {
                                for (i = 0; i < 9; i++) {
                                        maxval = 0.0F;
                                        for (r1 = 0; r1 < 3; r1++) {
                                                maxval += b_K_k[r2 + 9 * r1] * (float)b_a[r1 + 3 * i];
                                        }

                                        A_lin[r2 + 9 * i] = (float)I[r2 + 9 * i] - maxval;
                                }
                        }

                        for (r2 = 0; r2 < 9; r2++) {
                                for (i = 0; i < 9; i++) {
                                        P_apo[r2 + 9 * i] = 0.0F;
                                        for (r1 = 0; r1 < 9; r1++) {
                                                P_apo[r2 + 9 * i] += A_lin[r2 + 9 * r1] * P_apr[r1 + 9 * i];
                                        }
                                }
                        }
                } else {
                        /* 'posEKF:140' else */
                        /* 'posEKF:141' if  zFlag(1)==1&&zFlag(2)==1&&zFlag(3)==0 */
                        if ((zFlag[0] == 1) && (zFlag[1] == 1) && (zFlag[2] == 0)) {
                                /*      R=[ r_acc,0,0,0,0,0; */
                                /*          0,r_acc,0,0,0,0; */
                                /*          0,0,r_acc,0,0,0; */
                                /*          0,0,0,r_ptam,0,0; */
                                /*          0,0,0,0,r_ptam,0; */
                                /*          0,0,0,0,0,r_ptam; */
                                /* 'posEKF:150' R_v=[r_acc,r_acc,r_acc,r_ptam(1),r_ptam(2),r_ptam(3)]; */
                                /* observation matrix */
                                /* 'posEKF:153' H_k=[  E,     Z,      Z; */
                                /* 'posEKF:154'                        Z,     Z,      E]; */
                                /* 'posEKF:156' y_k=z(1:6)-H_k(1:6,1:9)*x_apr; */
                                /* 'posEKF:158' S_k=H_k(1:6,1:9)*P_apr*H_k(1:6,1:9)'; */
                                for (r2 = 0; r2 < 6; r2++) {
                                        for (i = 0; i < 9; i++) {
                                                d_a[r2 + 6 * i] = 0.0F;
                                                for (r1 = 0; r1 < 9; r1++) {
                                                        d_a[r2 + 6 * i] += (float)e_a[r2 + 6 * r1] * P_apr[r1 + 9 * i];
                                                }
                                        }

                                        for (i = 0; i < 6; i++) {
                                                b_S_k[r2 + 6 * i] = 0.0F;
                                                for (r1 = 0; r1 < 9; r1++) {
                                                        b_S_k[r2 + 6 * i] += d_a[r2 + 6 * r1] * (float)c_b[r1 + 9 * i];
                                                }
                                        }
                                }

                                /* 'posEKF:159' S_k(1:6+1:end) = S_k(1:6+1:end) + R_v; */
                                c_r_acc[0] = (float)r_acc;
                                c_r_acc[1] = (float)r_acc;
                                c_r_acc[2] = (float)r_acc;
                                c_r_acc[3] = (float)r_ptam[0];
                                c_r_acc[4] = (float)r_ptam[1];
                                c_r_acc[5] = (float)r_ptam[2];
                                for (r2 = 0; r2 < 6; r2++) {
                                        c_S_k[r2] = b_S_k[7 * r2] + c_r_acc[r2];
                                }

                                for (r2 = 0; r2 < 6; r2++) {
                                        b_S_k[7 * r2] = c_S_k[r2];
                                }

                                /* 'posEKF:160' K_k=(P_apr*H_k(1:6,1:9)'/(S_k)); */
                                for (r2 = 0; r2 < 9; r2++) {
                                        for (i = 0; i < 6; i++) {
                                                c_K_k[r2 + 9 * i] = 0.0F;
                                                for (r1 = 0; r1 < 9; r1++) {
                                                        c_K_k[r2 + 9 * i] += P_apr[r2 + 9 * r1] * (float)c_b[r1 + 9 * i];
                                                }
                                        }
                                }

                                b_mrdivide(c_K_k, b_S_k);

                                /* 'posEKF:163' x_apo=x_apr+K_k*y_k; */
                                for (r2 = 0; r2 < 6; r2++) {
                                        maxval = 0.0F;
                                        for (i = 0; i < 9; i++) {
                                                maxval += (float)e_a[r2 + 6 * i] * x_apr[i];
                                        }

                                        c_r_acc[r2] = (float)z[r2] - maxval;
                                }

                                for (r2 = 0; r2 < 9; r2++) {
                                        maxval = 0.0F;
                                        for (i = 0; i < 6; i++) {
                                                maxval += c_K_k[r2 + 9 * i] * c_r_acc[i];
                                        }

                                        x_apo[r2] = x_apr[r2] + maxval;
                                }

                                /* 'posEKF:164' P_apo=(eye(9)-K_k*H_k(1:6,1:9))*P_apr; */
                                memset(&I[0], 0, 81U * sizeof(signed char));
                                for (i = 0; i < 9; i++) {
                                        I[i + 9 * i] = 1;
                                }

                                for (r2 = 0; r2 < 9; r2++) {
                                        for (i = 0; i < 9; i++) {
                                                maxval = 0.0F;
                                                for (r1 = 0; r1 < 6; r1++) {
                                                        maxval += c_K_k[r2 + 9 * r1] * (float)e_a[r1 + 6 * i];
                                                }

                                                A_lin[r2 + 9 * i] = (float)I[r2 + 9 * i] - maxval;
                                        }
                                }

                                for (r2 = 0; r2 < 9; r2++) {
                                        for (i = 0; i < 9; i++) {
                                                P_apo[r2 + 9 * i] = 0.0F;
                                                for (r1 = 0; r1 < 9; r1++) {
                                                        P_apo[r2 + 9 * i] += A_lin[r2 + 9 * r1] * P_apr[r1 + 9 * i];
                                                }
                                        }
                                }
                        } else {
                                /* 'posEKF:165' else */
                                /* 'posEKF:166' if  zFlag(1)==1&&zFlag(2)==0&&zFlag(3)==1 */
                                if ((zFlag[0] == 1) && (zFlag[1] == 0) && (zFlag[2] == 1)) {
                                        /*         R=[ r_acc,0,0,0,0,0; */
                                        /*          0,r_acc,0,0,0,0; */
                                        /*          0,0,r_acc,0,0,0; */
                                        /*          0,0,0,r_got,0,0; */
                                        /*          0,0,0,0,r_got,0; */
                                        /*          0,0,0,0,0,r_got; */
                                        /* 'posEKF:173' R_v=[r_acc,r_acc,r_acc,r_got,r_got,r_got]; */
                                        /* observation matrix */
                                        /* 'posEKF:176' H_k=[  E,     Z,      Z; */
                                        /* 'posEKF:177'                         Z,     Z,        E]; */
                                        /* 'posEKF:179' y_k=[z(1:3);z(7:9)]-H_k(1:6,1:9)*x_apr; */
                                        /* 'posEKF:181' S_k=H_k(1:6,1:9)*P_apr*H_k(1:6,1:9)'; */
                                        for (r2 = 0; r2 < 6; r2++) {
                                                for (i = 0; i < 9; i++) {
                                                        d_a[r2 + 6 * i] = 0.0F;
                                                        for (r1 = 0; r1 < 9; r1++) {
                                                                d_a[r2 + 6 * i] += (float)e_a[r2 + 6 * r1] * P_apr[r1 + 9 * i];
                                                        }
                                                }

                                                for (i = 0; i < 6; i++) {
                                                        b_S_k[r2 + 6 * i] = 0.0F;
                                                        for (r1 = 0; r1 < 9; r1++) {
                                                                b_S_k[r2 + 6 * i] += d_a[r2 + 6 * r1] * (float)c_b[r1 + 9 * i];
                                                        }
                                                }
                                        }

                                        /* 'posEKF:182' S_k(1:6+1:end) = S_k(1:6+1:end) + R_v; */
                                        c_r_acc[0] = (float)r_acc;
                                        c_r_acc[1] = (float)r_acc;
                                        c_r_acc[2] = (float)r_acc;
                                        c_r_acc[3] = (float)r_got;
                                        c_r_acc[4] = (float)r_got;
                                        c_r_acc[5] = (float)r_got;
                                        for (r2 = 0; r2 < 6; r2++) {
                                                c_S_k[r2] = b_S_k[7 * r2] + c_r_acc[r2];
                                        }

                                        for (r2 = 0; r2 < 6; r2++) {
                                                b_S_k[7 * r2] = c_S_k[r2];
                                        }

                                        /* 'posEKF:183' K_k=(P_apr*H_k(1:6,1:9)'/(S_k)); */
                                        for (r2 = 0; r2 < 9; r2++) {
                                                for (i = 0; i < 6; i++) {
                                                        c_K_k[r2 + 9 * i] = 0.0F;
                                                        for (r1 = 0; r1 < 9; r1++) {
                                                                c_K_k[r2 + 9 * i] += P_apr[r2 + 9 * r1] * (float)c_b[r1 + 9 * i];
                                                        }
                                                }
                                        }

                                        b_mrdivide(c_K_k, b_S_k);

                                        /* 'posEKF:186' x_apo=x_apr+K_k*y_k; */
                                        for (r2 = 0; r2 < 3; r2++) {
                                                c_r_acc[r2] = (float)z[r2];
                                        }

                                        for (r2 = 0; r2 < 3; r2++) {
                                                c_r_acc[r2 + 3] = (float)z[6 + r2];
                                        }

                                        for (r2 = 0; r2 < 6; r2++) {
                                                c_S_k[r2] = 0.0F;
                                                for (i = 0; i < 9; i++) {
                                                        c_S_k[r2] += (float)e_a[r2 + 6 * i] * x_apr[i];
                                                }

                                                b_z[r2] = c_r_acc[r2] - c_S_k[r2];
                                        }

                                        for (r2 = 0; r2 < 9; r2++) {
                                                maxval = 0.0F;
                                                for (i = 0; i < 6; i++) {
                                                        maxval += c_K_k[r2 + 9 * i] * b_z[i];
                                                }

                                                x_apo[r2] = x_apr[r2] + maxval;
                                        }

                                        /* 'posEKF:187' P_apo=(eye(9)-K_k*H_k(1:6,1:9))*P_apr; */
                                        memset(&I[0], 0, 81U * sizeof(signed char));
                                        for (i = 0; i < 9; i++) {
                                                I[i + 9 * i] = 1;
                                        }

                                        for (r2 = 0; r2 < 9; r2++) {
                                                for (i = 0; i < 9; i++) {
                                                        maxval = 0.0F;
                                                        for (r1 = 0; r1 < 6; r1++) {
                                                                maxval += c_K_k[r2 + 9 * r1] * (float)e_a[r1 + 6 * i];
                                                        }

                                                        A_lin[r2 + 9 * i] = (float)I[r2 + 9 * i] - maxval;
                                                }
                                        }

                                        for (r2 = 0; r2 < 9; r2++) {
                                                for (i = 0; i < 9; i++) {
                                                        P_apo[r2 + 9 * i] = 0.0F;
                                                        for (r1 = 0; r1 < 9; r1++) {
                                                                P_apo[r2 + 9 * i] += A_lin[r2 + 9 * r1] * P_apr[r1 + 9 * i];
                                                        }
                                                }
                                        }
                                } else {
                                        /* 'posEKF:188' else */
                                        /* 'posEKF:189' x_apo=x_apr; */
                                        for (i = 0; i < 9; i++) {
                                                x_apo[i] = x_apr[i];
                                        }

                                        /* 'posEKF:190' P_apo=P_apr; */
                                        memcpy(&P_apo[0], &P_apr[0], 81U * sizeof(float));
                                }
                        }
                }
        }

        /* 'posEKF:195' xa_apo =x_apo; */
        for (i = 0; i < 9; i++) {
                xa_apo[i] = x_apo[i];
        }

        /* 'posEKF:196' Pa_apo =P_apo; */
        memcpy(&Pa_apo[0], &P_apo[0], 81U * sizeof(float));
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void posEKF_initialize(void)
{
        Q_not_empty = false;
        posEKF_init();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void posEKF_terminate(void)
{
        /* (no terminate code required) */
}

/*
 * File trailer for posEKF.c
 *
 * [EOF]
 */
