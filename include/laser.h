#ifndef __LASER_H_INCLUDED__
#define __LASER_H_INCLUDED__

#include "contest1.h"

// ==== FORWARD DECLARATION FOR CALLBACK FUNCTION VARIABLES ====
extern float minDistanceDir; 

// ==== LASER HELPER FUNCTION DECLARATIONS ====== 
extern float minDistance(int32_t desiredAngleR, int32_t desiredAngleL=std::numeric_limits<int32_t>::infinity(), bool verbose=false);
extern void laserObstacleDirectionHandler(ros::Publisher *vel_pub, bool verbose=false);

#endif