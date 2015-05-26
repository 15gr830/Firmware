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
 * Linear matrix map between rpm values and motor control topic
 */
// Thrust
#define ACT_MAP_1_1  0.14f
#define ACT_MAP_1_2  0.14f
#define ACT_MAP_1_3  0.14f
#define ACT_MAP_1_4  0.14f

// Roll                     
#define ACT_MAP_2_1  -0.045f
#define ACT_MAP_2_2  0.045f
#define ACT_MAP_2_3  0.045f
#define ACT_MAP_2_4  -0.045f

// Pitch
#define ACT_MAP_3_1  -0.045f
#define ACT_MAP_3_2  0.045f
#define ACT_MAP_3_3  -0.045f
#define ACT_MAP_3_4  0.045f

// Yaw
#define ACT_MAP_4_1  -0.14f
#define ACT_MAP_4_2  -0.14f
#define ACT_MAP_4_3  0.14f
#define ACT_MAP_4_4  0.14f

/*
 * Yaw Ki constant
 */
#define YAW_KI 0.0000

/*
 * Z axis Ki constant
 */
#define Z_AXIS_KI 0.00008

/*
 * Anti windup limits
 */
#define Z_INTEGRATION_LIMIT    0.1f
#define YAW_INTEGRATION_LIMIT  0.15f

/*
 * Thrust value to counter the gravity
 */
#define ANTI_GRAVITY 0.22

/*
 * Counter yaw constant offset
 */
#define ANTI_YAW -0.06f


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


#endif  /* Q_CONTROL_PARAMS_H */
