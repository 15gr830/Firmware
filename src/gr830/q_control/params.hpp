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
 */
#define LQR_CONTROLLER_1_1     -0.8846
#define LQR_CONTROLLER_1_2     -0.8846
#define LQR_CONTROLLER_1_3     -0.1477
#define LQR_CONTROLLER_1_4     -0.2037
#define LQR_CONTROLLER_1_5     -0.2037
#define LQR_CONTROLLER_1_6     -0.2749
#define LQR_CONTROLLER_1_7     0.0207
#define LQR_CONTROLLER_1_8     0.0207
#define LQR_CONTROLLER_1_9     0.0209
#define LQR_CONTROLLER_1_10    0.0445
#define LQR_CONTROLLER_1_11    0.0445
#define LQR_CONTROLLER_1_12    0.1808
#define LQR_CONTROLLER_1_13    0.0040
#define LQR_CONTROLLER_1_14    -0.0003
#define LQR_CONTROLLER_1_15    0
#define LQR_CONTROLLER_1_16    0
                              
#define LQR_CONTROLLER_2_1     0.8846
#define LQR_CONTROLLER_2_2     0.8846
#define LQR_CONTROLLER_2_3     -0.1477
#define LQR_CONTROLLER_2_4     0.2037
#define LQR_CONTROLLER_2_5     0.2037
#define LQR_CONTROLLER_2_6     -0.2749
#define LQR_CONTROLLER_2_7     -0.0207
#define LQR_CONTROLLER_2_8     -0.0207
#define LQR_CONTROLLER_2_9     0.0209
#define LQR_CONTROLLER_2_10    -0.0445
#define LQR_CONTROLLER_2_11    -0.0445
#define LQR_CONTROLLER_2_12    0.1808
#define LQR_CONTROLLER_2_13    -0.0003
#define LQR_CONTROLLER_2_14    0.0040
#define LQR_CONTROLLER_2_15    0
#define LQR_CONTROLLER_2_16    0       
                              
#define LQR_CONTROLLER_3_1     0.8846
#define LQR_CONTROLLER_3_2     -0.8846 
#define LQR_CONTROLLER_3_3     0.1477
#define LQR_CONTROLLER_3_4     0.2037 
#define LQR_CONTROLLER_3_5     -0.2037 
#define LQR_CONTROLLER_3_6     0.2749
#define LQR_CONTROLLER_3_7     0.0207
#define LQR_CONTROLLER_3_8     -0.0207
#define LQR_CONTROLLER_3_9     0.0209 
#define LQR_CONTROLLER_3_10    0.0445
#define LQR_CONTROLLER_3_11    -0.0445
#define LQR_CONTROLLER_3_12    0.1808 
#define LQR_CONTROLLER_3_13    0.0000
#define LQR_CONTROLLER_3_14    0.0000
#define LQR_CONTROLLER_3_15    0.0040
#define LQR_CONTROLLER_3_16    -0.0003
                              
#define LQR_CONTROLLER_4_1     -0.8846 
#define LQR_CONTROLLER_4_2     0.8846
#define LQR_CONTROLLER_4_3     0.1477 
#define LQR_CONTROLLER_4_4     -0.2037 
#define LQR_CONTROLLER_4_5     0.2037
#define LQR_CONTROLLER_4_6     0.2749 
#define LQR_CONTROLLER_4_7     -0.0207 
#define LQR_CONTROLLER_4_8     0.0207
#define LQR_CONTROLLER_4_9     0.0209 
#define LQR_CONTROLLER_4_10    -0.0445 
#define LQR_CONTROLLER_4_11    0.0445
#define LQR_CONTROLLER_4_12    0.1808 
#define LQR_CONTROLLER_4_13    0.0000 
#define LQR_CONTROLLER_4_14    0.0000 
#define LQR_CONTROLLER_4_15    -0.0003
#define LQR_CONTROLLER_4_16    0.0040


/*
 * Linear matrix map between rpm values and motor control topic
 */
#define ACT_MAP_1_1  0.25f
#define ACT_MAP_1_2  0.25f
#define ACT_MAP_1_3  0.25f
#define ACT_MAP_1_4  0.25f
                     
#define ACT_MAP_2_1  -0.25f
#define ACT_MAP_2_2  0.25f
#define ACT_MAP_2_3  0.25f
#define ACT_MAP_2_4  -0.25f
                     
#define ACT_MAP_3_1  -0.25f
#define ACT_MAP_3_2  0.25f
#define ACT_MAP_3_3  -0.25f
#define ACT_MAP_3_4  0.25f
                     
#define ACT_MAP_4_1  0.25f
#define ACT_MAP_4_2  0.25f
#define ACT_MAP_4_3  -0.25f
#define ACT_MAP_4_4  -0.25f


/* Safety values */

/*
 * If the output of the lqr controller os larger than this level
 * for the roll, pitch and yaw actuator topic then the published
 * value will be the ones below.
 */
#define RP_MAX  0.4f
#define YAW_MAX 0.4f

/*
 * If the quad has a roll or a pitch of more than this level, 
 * then a safety shutdown is triggered
 */
#define RP_SAFE 0.5f

/*
 * Thrust value to counter the gravity
 */
#define ANTI_GRAVITY 0.f


#endif  /* Q_CONTROL_PARAMS_H */
