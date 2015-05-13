/****************************************************************************
15GR830
 ****************************************************************************/

/**
 * @file att_pos_mocap.h
 * Definition of the raw Motion Capture position
 */

#ifndef TOPIC_ATT_POS_MOCAP_H_
#define TOPIC_ATT_POS_MOCAP_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * Fused local position in NED.
 */
struct att_pos_mocap_s {
	uint64_t timestamp;			/**< time of this estimate, in microseconds since system start */

	float x;				/**< X positin in meters in NED earth-fixed frame */
	float y;				/**< X positin in meters in NED earth-fixed frame */
	float z;				/**< Z positin in meters in NED earth-fixed frame (negative altitude) */

	float q[4];				/**< Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)*/

	// TODO Add covariances here

};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(att_pos_mocap);

#endif
