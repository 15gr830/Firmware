/*************************************************************************
 * Copyright (c) 2015 Group 830 Aalborg University. All rights reserved.
 * Author:   Group 830 <15gr830@es.aau.dk>
 *************************************************************************
 *
 * Parameters for the controller application.
 *
 */

#ifndef Q_CONTROL_PARAMS_H
#define Q_CONTROL_PARAMS_H


/* 
 * Entrys for the 4x16 LQR controller matrix 
 *
 * LQR =
 * [  roll      pitch     yaw       w_roll    w_pitch   w_yaw     x         y        z         vx        vy        vz        rpm1      rpm2      rpm3      rpm4   ]
 * [ -1.3963   -1.3963   -0.2089   -0.2602   -0.2602   -0.3267    0.0206    0.0206   0.0209    0.0818    0.0818    0.1915    0.0041   -0.0004   -0.0000   -0.0000 ]
 * [  1.3963    1.3963   -0.2089    0.2602    0.2602   -0.3267   -0.0206   -0.0206   0.0209   -0.0818   -0.0818    0.1915   -0.0004    0.0041   -0.0000   -0.0000 ]
 * [  1.3963   -1.3963    0.2089    0.2602   -0.2602    0.3267    0.0206   -0.0206   0.0209    0.0818   -0.0818    0.1915   -0.0000   -0.0000    0.0041   -0.0004 ]
 * [ -1.3963    1.3963    0.2089   -0.2602    0.2602    0.3267   -0.0206    0.0206   0.0209   -0.0818    0.0818    0.1915   -0.0000   -0.0000   -0.0004    0.0041 ]
 *
 */


#define LQR_CONTROLLER_1_1     -1.3963f
#define LQR_CONTROLLER_1_2     -1.3963f
#define LQR_CONTROLLER_1_3     -0.2089f
#define LQR_CONTROLLER_1_4     -0.2602f
#define LQR_CONTROLLER_1_5     -0.2602f
#define LQR_CONTROLLER_1_6     -0.3267f
#define LQR_CONTROLLER_1_7     0.0206f
#define LQR_CONTROLLER_1_8     0.0206f
#define LQR_CONTROLLER_1_9     0.0209f
#define LQR_CONTROLLER_1_10    0.0818f
#define LQR_CONTROLLER_1_11    0.0818f
#define LQR_CONTROLLER_1_12    0.1915f
#define LQR_CONTROLLER_1_13    0.0041f
#define LQR_CONTROLLER_1_14    -0.0004f
#define LQR_CONTROLLER_1_15    -0.0000f
#define LQR_CONTROLLER_1_16    -0.0000f
                              
#define LQR_CONTROLLER_2_1     1.3963f
#define LQR_CONTROLLER_2_2     1.3963f
#define LQR_CONTROLLER_2_3     -0.2089f
#define LQR_CONTROLLER_2_4     0.2602f
#define LQR_CONTROLLER_2_5     0.2602f
#define LQR_CONTROLLER_2_6     -0.3267f
#define LQR_CONTROLLER_2_7     -0.0206f
#define LQR_CONTROLLER_2_8     -0.0206f
#define LQR_CONTROLLER_2_9     0.0209f
#define LQR_CONTROLLER_2_10    -0.0818f
#define LQR_CONTROLLER_2_11    -0.0818f
#define LQR_CONTROLLER_2_12    0.1915f
#define LQR_CONTROLLER_2_13    -0.0004f
#define LQR_CONTROLLER_2_14    0.0041f
#define LQR_CONTROLLER_2_15    -0.0000f
#define LQR_CONTROLLER_2_16    -0.0000f 
                              
#define LQR_CONTROLLER_3_1     1.3963f
#define LQR_CONTROLLER_3_2     -1.3963f
#define LQR_CONTROLLER_3_3     0.2089f
#define LQR_CONTROLLER_3_4     0.2602f
#define LQR_CONTROLLER_3_5     -0.2602f
#define LQR_CONTROLLER_3_6     0.3267f
#define LQR_CONTROLLER_3_7     0.0206f
#define LQR_CONTROLLER_3_8     -0.0206f
#define LQR_CONTROLLER_3_9     0.0209f
#define LQR_CONTROLLER_3_10    0.0818f
#define LQR_CONTROLLER_3_11    -0.0818f
#define LQR_CONTROLLER_3_12    0.1915f
#define LQR_CONTROLLER_3_13    -0.0000f
#define LQR_CONTROLLER_3_14    -0.0000f
#define LQR_CONTROLLER_3_15    0.0041f
#define LQR_CONTROLLER_3_16    -0.0004f

#define LQR_CONTROLLER_4_1     -1.3963f
#define LQR_CONTROLLER_4_2     1.3963f
#define LQR_CONTROLLER_4_3     0.2089f
#define LQR_CONTROLLER_4_4     -0.2602f
#define LQR_CONTROLLER_4_5     0.2602f
#define LQR_CONTROLLER_4_6     0.3267f
#define LQR_CONTROLLER_4_7     -0.0206f
#define LQR_CONTROLLER_4_8     0.0206f
#define LQR_CONTROLLER_4_9     0.0209f
#define LQR_CONTROLLER_4_10    -0.0818f
#define LQR_CONTROLLER_4_11    0.0818f
#define LQR_CONTROLLER_4_12    0.1915f
#define LQR_CONTROLLER_4_13    -0.0000f
#define LQR_CONTROLLER_4_14    -0.0000f
#define LQR_CONTROLLER_4_15    -0.0004f
#define LQR_CONTROLLER_4_16    0.0041f


/*
 * Linear matrix map between rpm values and motor control topic
 */
// Thrust
#define ACT_MAP_1_1  0.14f
#define ACT_MAP_1_2  0.14f
#define ACT_MAP_1_3  0.14f
#define ACT_MAP_1_4  0.14f

// Roll                     
#define ACT_MAP_2_1  -0.14f
#define ACT_MAP_2_2  0.14f
#define ACT_MAP_2_3  0.14f
#define ACT_MAP_2_4  -0.14f

// Pitch
#define ACT_MAP_3_1  -0.14f
#define ACT_MAP_3_2  0.14f
#define ACT_MAP_3_3  -0.14f
#define ACT_MAP_3_4  0.14f

// Yaw
#define ACT_MAP_4_1  -0.14f
#define ACT_MAP_4_2  -0.14f
#define ACT_MAP_4_3  0.14f
#define ACT_MAP_4_4  0.14f


/* Safety values */

/*
 * If the output of the lqr controller os larger than this level
 * for the roll, pitch and yaw actuator topic then the published
 * value will be the ones below.
 */
#define RP_MAX  0.8f
#define YAW_MAX 0.8f

/*
 * If the quad has a roll or a pitch of more than this level, 
 * then a safety shutdown is triggered
 */
#define RP_SAFE 1.f

/*
 * Thrust value to counter the gravity
 */
#define ANTI_GRAVITY 0.2f


#endif  /* Q_CONTROL_PARAMS_H */
