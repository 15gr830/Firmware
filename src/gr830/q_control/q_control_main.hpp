/*************************************************************************
 * Copyright (c) 2015 Group 830 Aalborg University. All rights reserved.
 * Author:   Group 830 <15gr830@es.aau.dk>
 *************************************************************************
 *
 * Data types for the quadrotor controller application
 *
 */

#ifndef Q_CONTROL_MAIN_H
#define Q_CONTROL_MAIN_H


struct output_s {
        float roll;
        float pitch;
        float yaw;
        float thrust;
};


#endif  /* Q_CONTROL_MAIN_H */
