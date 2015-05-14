/*************************************************************************
 * Copyright (c) 2015 Group 830 Aalborg University. All rights reserved.
 * Author:   Group 830 <15gr830@es.aau.dk>
 *************************************************************************
 *
 * Parameters for the attitude controller.
 *
 */

#ifndef ATTITUDE_CONTROL_PARAMS_H
#define ATTITUDE_CONTROL_PARAMS_H

/* Entrys for the 4x12 LQR controller matrix; default identity matrix */
#define LQR_CONTROLLER_1_1     1
#define LQR_CONTROLLER_1_2     0
#define LQR_CONTROLLER_1_3     0
#define LQR_CONTROLLER_1_4     0
#define LQR_CONTROLLER_1_5     0
#define LQR_CONTROLLER_1_6     0
#define LQR_CONTROLLER_1_7     0
#define LQR_CONTROLLER_1_8     0
#define LQR_CONTROLLER_1_9     0
#define LQR_CONTROLLER_1_10    0
#define LQR_CONTROLLER_1_11    0
#define LQR_CONTROLLER_1_12    0
#define LQR_CONTROLLER_1_13    0
#define LQR_CONTROLLER_1_14    0
#define LQR_CONTROLLER_1_15    0
#define LQR_CONTROLLER_1_16    0
                              
#define LQR_CONTROLLER_2_1     0
#define LQR_CONTROLLER_2_2     1
#define LQR_CONTROLLER_2_3     0
#define LQR_CONTROLLER_2_4     0
#define LQR_CONTROLLER_2_5     0
#define LQR_CONTROLLER_2_6     0
#define LQR_CONTROLLER_2_7     0
#define LQR_CONTROLLER_2_8     0
#define LQR_CONTROLLER_2_9     0
#define LQR_CONTROLLER_2_10    0
#define LQR_CONTROLLER_2_11    0
#define LQR_CONTROLLER_2_12    0
#define LQR_CONTROLLER_2_13    0
#define LQR_CONTROLLER_2_14    0
#define LQR_CONTROLLER_2_15    0
#define LQR_CONTROLLER_2_16    0
                              
#define LQR_CONTROLLER_3_1     0
#define LQR_CONTROLLER_3_2     0
#define LQR_CONTROLLER_3_3     1
#define LQR_CONTROLLER_3_4     0
#define LQR_CONTROLLER_3_5     0
#define LQR_CONTROLLER_3_6     0
#define LQR_CONTROLLER_3_7     0
#define LQR_CONTROLLER_3_8     0
#define LQR_CONTROLLER_3_9     0
#define LQR_CONTROLLER_3_10    0
#define LQR_CONTROLLER_3_11    0
#define LQR_CONTROLLER_3_12    0
#define LQR_CONTROLLER_3_13    0
#define LQR_CONTROLLER_3_14    0
#define LQR_CONTROLLER_3_15    0
#define LQR_CONTROLLER_3_16    0
                              
#define LQR_CONTROLLER_4_1     0
#define LQR_CONTROLLER_4_2     0
#define LQR_CONTROLLER_4_3     0
#define LQR_CONTROLLER_4_4     1
#define LQR_CONTROLLER_4_5     0
#define LQR_CONTROLLER_4_6     0
#define LQR_CONTROLLER_4_7     0
#define LQR_CONTROLLER_4_8     0
#define LQR_CONTROLLER_4_9     0
#define LQR_CONTROLLER_4_10    0
#define LQR_CONTROLLER_4_11    0
#define LQR_CONTROLLER_4_12    0
#define LQR_CONTROLLER_4_13    0
#define LQR_CONTROLLER_4_14    0
#define LQR_CONTROLLER_4_15    0
#define LQR_CONTROLLER_4_16    0

// Safety values
#define RP_MAX  0.8
#define YAW_MAX 0.8
#define RP_SAFE 0.8


#endif  /* ATTITUDE_CONTROL_PARAMS_H */
