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
#include "../CMSIS/Include/arm_math.h"
#include "Vector.hpp"
#include "Matrix.hpp"

namespace math {

        class __EXPORT Lqr : public Matrix<4, 12> {
        public:
                Lqr() : Matrix<4, 12>() {}
                virtual ~Lqr();

                init_LQR();
        };

}

#endif //LQR_HPP
