#include "wallFollow.h"

void wallFollow(ros::Publisher *vel_pub) {
    /* 
    If robot encouters a wall, make a rotation and keep wall on RHS. Otherwise proceed
    forward.
    */
    
    float minLaserDist = minDistance(15); 
    
    if (bumpersPressed()) { // If the bumper is depressed, determine which one and perform correct rotation.
        uint8_t b_index = specificBumperPressed(); // Determine the specific bumper that was depressed

        // Perform error check
        if (b_index < 0) {
            return;
        }
    
        // Reverse bot to avoid being in a blocked state during rotation
        stepDistance(0.01, SPEED_LIM, vel_pub, false, true); // Reverse = true

        // Determine which bumper made contact with the object
        if (b_index == 0) { // Left-most bumper depressed, perform -CW rotation
            rotByAngle(-M_PI/6, vel_pub);
        } else if (b_index == 1) { // Front depressed, perform random rotation (either +CCW/-CW)
            float rand_direction = randRange(0.0, 1.0); // Generate a random value to determine direction of rotation

            if (rand_direction < 0.5) {
                rotByAngle(M_PI/6, vel_pub);
            } else {
                rotByAngle(-M_PI/6, vel_pub);
            }
        } else { // Right-most bumper depressed, perform +CCW rotation
            rotByAngle(M_PI/6, vel_pub);

        }

    } else if (minLaserDist < MIN_LASER_DIST || std::isinf(minLaserDist) || std::isnan(minLaserDist)) { // If an obstacle is detected by the laser scan
        // DEBUG - turn off verbose (set to false)
        laserObstacleDirectionHandler(vel_pub, true); // Handle the detected object from laser scan based on direction of object.
    } else { // If there are no obstacles detected, proceed forward
        stepDistance(50, SPEED_LIM, vel_pub);
    }

    return;
}