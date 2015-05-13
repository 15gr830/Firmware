/*************************************************************************
 * Copyright (c) 2015 Group 830 Aalborg University. All rights reserved.
 * Author:   Group 830 <15gr830@es.aau.dk>
 *************************************************************************
 *
 * LQR class
 *
 * Implements the following blockdiagram. To this class collect the
 * estimated state vector (x_est), set the desired state vector
 * (x_ref), set the r to the desired steady state values and call
 * the run() method, this will return a control vector u.
 *
 * state vector = [q1 q2 q3 w1 w2 w3 x y z vx vy vz rpm1 rpm2 rpm3 rpm4]^T
 *
 *   r -------O---- u
 *            |
 *         -------
 *         | -K  |
 *         -------
 *            |
 *  -x_ref--- O---- x_est (e.g. from a Kalman filter)
 *
 * This results in:
 * u = r + K*(x_ref - x_est)
 *
 */

#ifndef LQR_HPP
#define LQR_HPP

#include <math.h>
#include <lib/mathlib/mathlib.h>
#include "params.hpp"

class __EXPORT Lqr : public math::Matrix<4,16> {
public:
        math::Vector<16>  x_est, x_ref, x_err;
        math::Vector<4>   r;
        math::Quaternion *q_ref = new math::Quaternion(1,0,0,0);
        math::Quaternion *q_est = new math::Quaternion();
        math::Quaternion *q_err = new math::Quaternion();

        /*
         * Constructor initialises from params.hpp
         */
        Lqr() : Matrix<4, 16>() {
                data[0][0] = LQR_CONTROLLER_1_1;
                data[0][1] = LQR_CONTROLLER_1_2;
                data[0][2] = LQR_CONTROLLER_1_3;
                data[0][3] = LQR_CONTROLLER_1_4;
                data[0][4] = LQR_CONTROLLER_1_5;
                data[0][5] = LQR_CONTROLLER_1_6;
                data[0][6] = LQR_CONTROLLER_1_7;
                data[0][7] = LQR_CONTROLLER_1_8;
                data[0][8] = LQR_CONTROLLER_1_9;
                data[0][9] = LQR_CONTROLLER_1_10;
                data[0][10] = LQR_CONTROLLER_1_11;
                data[0][11] = LQR_CONTROLLER_1_12;
                data[0][12] = LQR_CONTROLLER_1_13;
                data[0][13] = LQR_CONTROLLER_1_14;
                data[0][14] = LQR_CONTROLLER_1_15;
                data[0][15] = LQR_CONTROLLER_1_16;

                data[1][0] = LQR_CONTROLLER_2_1;
                data[1][1] = LQR_CONTROLLER_2_2;
                data[1][2] = LQR_CONTROLLER_2_3;
                data[1][3] = LQR_CONTROLLER_2_4;
                data[1][4] = LQR_CONTROLLER_2_5;
                data[1][5] = LQR_CONTROLLER_2_6;
                data[1][6] = LQR_CONTROLLER_2_7;
                data[1][7] = LQR_CONTROLLER_2_8;
                data[1][8] = LQR_CONTROLLER_2_9;
                data[1][9] = LQR_CONTROLLER_2_10;
                data[1][10] = LQR_CONTROLLER_2_11;
                data[1][11] = LQR_CONTROLLER_2_12;
                data[1][12] = LQR_CONTROLLER_2_13;
                data[1][13] = LQR_CONTROLLER_2_14;
                data[1][14] = LQR_CONTROLLER_2_15;
                data[1][15] = LQR_CONTROLLER_2_16;

                data[2][0] = LQR_CONTROLLER_3_1;
                data[2][1] = LQR_CONTROLLER_3_2;
                data[2][2] = LQR_CONTROLLER_3_3;
                data[2][3] = LQR_CONTROLLER_3_4;
                data[2][4] = LQR_CONTROLLER_3_5;
                data[2][5] = LQR_CONTROLLER_3_6;
                data[2][6] = LQR_CONTROLLER_3_7;
                data[2][7] = LQR_CONTROLLER_3_8;
                data[2][8] = LQR_CONTROLLER_3_9;
                data[2][9] = LQR_CONTROLLER_3_10;
                data[2][10] = LQR_CONTROLLER_3_11;
                data[2][11] = LQR_CONTROLLER_3_12;
                data[2][12] = LQR_CONTROLLER_3_13;
                data[2][13] = LQR_CONTROLLER_3_14;
                data[2][14] = LQR_CONTROLLER_3_15;
                data[2][15] = LQR_CONTROLLER_3_16;

                data[3][0] = LQR_CONTROLLER_4_1;
                data[3][1] = LQR_CONTROLLER_4_2;
                data[3][2] = LQR_CONTROLLER_4_3;
                data[3][3] = LQR_CONTROLLER_4_4;
                data[3][4] = LQR_CONTROLLER_4_5;
                data[3][5] = LQR_CONTROLLER_4_6;
                data[3][6] = LQR_CONTROLLER_4_7;
                data[3][7] = LQR_CONTROLLER_4_8;
                data[3][8] = LQR_CONTROLLER_4_9;
                data[3][9] = LQR_CONTROLLER_4_10;
                data[3][10] = LQR_CONTROLLER_4_11;
                data[3][11] = LQR_CONTROLLER_4_12;
                data[3][12] = LQR_CONTROLLER_4_13;
                data[3][13] = LQR_CONTROLLER_4_14;
                data[3][14] = LQR_CONTROLLER_4_15;
                data[3][15] = LQR_CONTROLLER_4_16;
        }

        Lqr(const Matrix<4, 16> &m) : Matrix<4, 16>(m) {}        

        /*
         * Initialises from matrix (virker ikke - BGT)
         */
        void set(const float &d) {
//                memcpy(data, d, sizeof(data));
        }

        virtual ~Lqr() {
                delete q_ref;
                delete q_est;
                delete q_err;
        };

        /*
         * Run the controller
         */
        math::Vector<4> run(void) {
                math::Vector<4>  u, i;
                math::Vector<16> x_e;
                double           q0_sign = 0;

                // state vector = [q1 q2 q3 w1 w2 w3 x y z vx vy vz rpm1 rpm2 rpm3 rpm4]^T

                // Calculates quaternion error
                q_est->conjugate();
                // q_err = q_est^* qmult q_ref
                q_err->data[0] = q_est->data[0] * q_ref->data[0] - q_est->data[1] * q_ref->data[1] - q_est->data[2] * q_ref->data[2] - q_est->data[3] * q_ref->data[3];
                q_err->data[1] = q_est->data[0] * q_ref->data[1] + q_est->data[1] * q_ref->data[0] + q_est->data[2] * q_ref->data[3] - q_est->data[3] * q_ref->data[2];
                q_err->data[2] = q_est->data[0] * q_ref->data[2] - q_est->data[1] * q_ref->data[3] + q_est->data[2] * q_ref->data[0] + q_est->data[3] * q_ref->data[1];
                q_err->data[3] = q_est->data[0] * q_ref->data[3] + q_est->data[1] * q_ref->data[2] - q_est->data[2] * q_ref->data[1] + q_est->data[3] * q_ref->data[0];

                q0_sign = (double)q_err->data[0]/fabs(q_err->data[0]);

                // Collecting the error state vector x_e
                x_e.data[0]  = (double)q0_sign*(double)q_err->data[1];
                x_e.data[1]  = (double)q0_sign*(double)q_err->data[2];
                x_e.data[2]  = (double)q0_sign*(double)q_err->data[3];
                x_e.data[3]  = x_ref.data[3] - x_est.data[3];
                x_e.data[4]  = x_ref.data[4] - x_est.data[4];
                x_e.data[5]  = x_ref.data[5] - x_est.data[5];
                x_e.data[6]  = x_ref.data[6] - x_est.data[6];
                x_e.data[7]  = x_ref.data[7] - x_est.data[7];
                x_e.data[8]  = x_ref.data[8] - x_est.data[8];
                x_e.data[9]  = x_ref.data[9] - x_est.data[9];
                x_e.data[10] = x_ref.data[10] - x_est.data[10];
                x_e.data[11] = x_ref.data[11] - x_est.data[11];
                x_e.data[12] = 0;
                x_e.data[13] = 0;
                x_e.data[14] = 0;
                x_e.data[15] = 0;

                // printf("x_e = ");
                // x_e.print();

                // Calculating K*x_e
                i.data[0] = data[0][0]*x_e.data[0] + data[0][1]*x_e.data[1] + data[0][2]*x_e.data[2] + data[0][3]*x_e.data[3] + data[0][4]*x_e.data[4] + data[0][5]*x_e.data[5] + data[0][6]*x_e.data[6] + data[0][7]*x_e.data[7] + data[0][8]*x_e.data[8] + data[0][9]*x_e.data[9] + data[0][10]*x_e.data[10] + data[0][11]*x_e.data[11] + data[0][12]*x_e.data[12] + data[0][13]*x_e.data[13] + data[0][14]*x_e.data[14] + data[0][15]*x_e.data[15];
                i.data[1] = data[1][0]*x_e.data[0] + data[1][1]*x_e.data[1] + data[1][2]*x_e.data[2] + data[1][3]*x_e.data[3] + data[1][4]*x_e.data[4] + data[1][5]*x_e.data[5] + data[1][6]*x_e.data[6] + data[1][7]*x_e.data[7] + data[1][8]*x_e.data[8] + data[1][9]*x_e.data[9] + data[1][10]*x_e.data[10] + data[1][11]*x_e.data[11] + data[1][12]*x_e.data[12] + data[1][13]*x_e.data[13] + data[1][14]*x_e.data[14] + data[1][15]*x_e.data[15];
                i.data[2] = data[1][0]*x_e.data[0] + data[2][1]*x_e.data[1] + data[2][2]*x_e.data[2] + data[2][3]*x_e.data[3] + data[2][4]*x_e.data[4] + data[2][5]*x_e.data[5] + data[2][6]*x_e.data[6] + data[2][7]*x_e.data[7] + data[2][8]*x_e.data[8] + data[2][9]*x_e.data[9] + data[2][10]*x_e.data[10] + data[2][11]*x_e.data[11] + data[2][12]*x_e.data[12] + data[2][13]*x_e.data[13] + data[2][14]*x_e.data[14] + data[3][15]*x_e.data[15];
                i.data[3] = data[1][0]*x_e.data[0] + data[3][1]*x_e.data[1] + data[3][2]*x_e.data[2] + data[3][3]*x_e.data[3] + data[3][4]*x_e.data[4] + data[3][5]*x_e.data[5] + data[3][6]*x_e.data[6] + data[3][7]*x_e.data[7] + data[3][8]*x_e.data[8] + data[3][9]*x_e.data[9] + data[3][10]*x_e.data[10] + data[3][11]*x_e.data[11] + data[3][12]*x_e.data[12] + data[3][13]*x_e.data[13] + data[3][14]*x_e.data[14] + data[3][15]*x_e.data[15];

                // Calculating r + K*x_e
                u.data[0] = r.data[0] + i.data[0];
                u.data[1] = r.data[1] + i.data[1];
                u.data[2] = r.data[2] + i.data[2];
                u.data[3] = r.data[3] + i.data[3];

                return u;
        }
};

#endif //LQR_HPP
