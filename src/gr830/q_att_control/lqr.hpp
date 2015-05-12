/*************************************************************************
 * Copyright (c) 2015 Group 830 Aalborg University. All rights reserved.
 * Author:   Group 830 <15gr830@es.aau.dk>
 *************************************************************************
 *
 * LQR class
 *
 */

#ifndef LQR_HPP
#define LQR_HPP

#include <math.h>
#include <lib/mathlib/mathlib.h>
#include "params.hpp"

class __EXPORT Lqr : public math::Matrix<4,16> {
public:
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

        virtual ~Lqr() {};

        /*
         * Run the controller
         */
        math::Vector<4> run(const math::Vector<16> &e) {
                math::Vector<4> i = {0, 0, 0, 0};

                i.data[0] = data[0][0]*e.data[0] + data[0][1]*e.data[1] + data[0][2]*e.data[2] + data[0][3]*e.data[3] + data[0][4]*e.data[4] + data[0][5]*e.data[5] + data[0][6]*e.data[6] + data[0][7]*e.data[7] + data[0][8]*e.data[8] + data[0][9]*e.data[9] + data[0][10]*e.data[10] + data[0][11]*e.data[11] + data[0][12]*e.data[12] + data[0][13]*e.data[13] + data[0][14]*e.data[14] + data[0][15]*e.data[15];
                i.data[1] = data[1][0]*e.data[0] + data[1][1]*e.data[1] + data[1][2]*e.data[2] + data[1][3]*e.data[3] + data[1][4]*e.data[4] + data[1][5]*e.data[5] + data[1][6]*e.data[6] + data[1][7]*e.data[7] + data[1][8]*e.data[8] + data[1][9]*e.data[9] + data[1][10]*e.data[10] + data[1][11]*e.data[11] + data[1][12]*e.data[12] + data[1][13]*e.data[13] + data[1][14]*e.data[14] + data[1][15]*e.data[15];
                i.data[2] = data[1][0]*e.data[0] + data[2][1]*e.data[1] + data[2][2]*e.data[2] + data[2][3]*e.data[3] + data[2][4]*e.data[4] + data[2][5]*e.data[5] + data[2][6]*e.data[6] + data[2][7]*e.data[7] + data[2][8]*e.data[8] + data[2][9]*e.data[9] + data[2][10]*e.data[10] + data[2][11]*e.data[11] + data[2][12]*e.data[12] + data[2][13]*e.data[13] + data[2][14]*e.data[14] + data[3][15]*e.data[15];
                i.data[3] = data[1][0]*e.data[0] + data[3][1]*e.data[1] + data[3][2]*e.data[2] + data[3][3]*e.data[3] + data[3][4]*e.data[4] + data[3][5]*e.data[5] + data[3][6]*e.data[6] + data[3][7]*e.data[7] + data[3][8]*e.data[8] + data[3][9]*e.data[9] + data[3][10]*e.data[10] + data[3][11]*e.data[11] + data[3][12]*e.data[12] + data[3][13]*e.data[13] + data[3][14]*e.data[14] + data[3][15]*e.data[15];

                return i;
        }
};

#endif //LQR_HPP
