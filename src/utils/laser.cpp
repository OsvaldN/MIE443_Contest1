#include "laser.h"

/*
////////////////////////////
/ Laser helper functions /
////////////////////////////
*/

float minDistanceDir = 0.0; 

float minDistance(int32_t desiredAngleR, int32_t desiredAngleL, bool verbose){


    //returns the minimum distance in a view of the desired angle in DEGREES on either side
    
    //if nothing set for the range to the left default it to equal to the right side
    if (desiredAngleL == std::numeric_limits<int32_t>::infinity()){
        desiredAngleL = desiredAngleR;
    }

    //set the minimum distance to a very large value where a new smaller value can be found
    float minLaserDist = std::numeric_limits<float>::infinity();

    int32_t desiredNLasersR=0, desiredNLasersL=0;
    minDistanceDir = 0.0; // store the direction of minimum direction into this global variable

    //determine the number of laser needed to scan the desired angle.  Note: convert desired angle from degrees to radians
    desiredNLasersR = desiredAngleR*M_PI/(180*laser_ang_increment);
    desiredNLasersL = desiredAngleL*M_PI/(180*laser_ang_increment);

    //If it has not connected to the sensor yet, nLasers will be 0 which will cause a fault
    if (nLasers == 0 ){
        nLasers = 2;
    }

    //return minDistance = inf if the ranges were inputed incorrectly
    if (-desiredAngleR >= desiredAngleL ){

        if (verbose) {
            ROS_INFO("Search range is not applicable since left range is more to right than the right range");
        }
        return minLaserDist;

    }

    //fix if the values exceed the sensor abilities
    if (-desiredAngleR*M_PI/180 < laser_min_ang ){

        desiredNLasersR = nLasers/2;

        if (verbose) {
            ROS_INFO("Search range to the right %i exceeds the right limit of the sensor so it set to the max", desiredAngleR);
        }

        if (desiredAngleL*M_PI/180 < laser_min_ang ){

            desiredNLasersL = 0;

            if (verbose) {
                ROS_INFO("Search range to the left %i exceeds the right limit of the sensor so it is reset to 0 degrees", desiredAngleL);
            }
        }

    }
    if ( desiredAngleL*M_PI/ 180 > laser_max_ang ){

        desiredNLasersL = nLasers/2 - 1;
        if (verbose) {
            ROS_INFO("Search range to the left %i exceeds the left limit of the sensor so it is set to the max", desiredAngleL);
        }

        if (-desiredAngleR*M_PI/180 > laser_max_ang ){

            desiredNLasersR = 0;
            if (verbose) {
                ROS_INFO("Search range to the right %i exceeds the left limit of the sensor so it is reset to 0 degrees", desiredAngleR);
            }
        }

    }

    for (uint32_t laser_idx = nLasers/2-desiredNLasersR; laser_idx < nLasers/2+desiredNLasersL; ++laser_idx){

        if (LaserArray[laser_idx] < minLaserDist){
            minDistanceDir = LaserArrayAng[laser_idx];
        }
        minLaserDist = std::min(minLaserDist, LaserArray[laser_idx]);
        
    }

    int32_t totalNLasers = desiredNLasersL + desiredNLasersR;

    if (verbose){
        ROS_INFO("Search in range: %i degrees to the right to %i degrees to the left with %i total number of lasers", desiredAngleR, desiredAngleL, totalNLasers);
        ROS_INFO("minLaserDist measured is: %f and it direction relative to the robot is %f CCW", minLaserDist, minDistanceDir);
    }

    return minLaserDist;
}

void laserObstacleDirectionHandler(ros::Publisher *vel_pub, bool verbose) {
    float left_minLaserDist = minDistance(0, 15); 
    float right_minLaserDist = minDistance(15, 0); 

    // If the object is right infront of the robot, do spin, and choose direction of most space
    if ((left_minLaserDist < MIN_LASER_DIST && right_minLaserDist < MIN_LASER_DIST) || (std::isinf(left_minLaserDist) && std::isinf(right_minLaserDist)) || (std::isnan(left_minLaserDist) && std::isnan(right_minLaserDist))) {
        if (verbose) {
            ROS_INFO("DEBUG: obj detected FRONT, SPIN");
        }
        spinAndStep(1, SPEED_LIM, 5, vel_pub);

    } else if (left_minLaserDist < MIN_LASER_DIST || std::isinf(left_minLaserDist) || std::isnan(left_minLaserDist)) { // If the object is detected to be on the left, make a -CW rotation
        if (verbose) {
            ROS_INFO("DEBUG: obj detected LEFT, turn RIGHT"); 
        }
        rotByAngle(-M_PI/6, vel_pub);

    } else if (right_minLaserDist < MIN_LASER_DIST || std::isinf(right_minLaserDist) || std::isnan(right_minLaserDist)) { // If the object is detected to be on the right, make a -CW rotation
        if (verbose) {
            ROS_INFO("DEBUG: obj detected RIGHT, turn LEFT"); 
        }
        rotByAngle(M_PI/6, vel_pub);

    } else {
        if (verbose) {
            ROS_INFO("DEBUG: DEFAULT case, SPIN"); 
        }
        spinAndStep(1, SPEED_LIM, 5, vel_pub);
    }

    return;
}
