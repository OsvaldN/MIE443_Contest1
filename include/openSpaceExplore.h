#ifndef __OPEN_SPACE_EXPLORE_H_INCLUDED__
#define __OPEN_SPACE_EXPLORE_H_INCLUDED__

#include "contest1.h"

// ==== OPEN SPACE EXPLORE FUNCTION DECLARATIONS ====== 
extern void spinAndStep(float stepSize, float speed, int Nbins, ros::Publisher *vel_pub, bool verbose=true); 
extern void spinToDist(float stepSize, float speed, float reqDist, float dir, float increment, ros::Publisher *vel_pub, bool verbose=true );

#endif