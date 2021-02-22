#include "openSpaceExplore.h"

void spinToDist(float stepSize, float speed, float reqDist, float dir, float increment, ros::Publisher *vel_pub, bool verbose) {
    /*
    Rotates in direction (dir) until the laser callback finds an open distance of (reqDist) at velocity (speed)
    If no bin has reqDist space available the robot will step forward at the last bin.
    dir: 1 for CCW, -1 for CW rotation
    increment: degrees to rotate between each scan
    */

    float travelled = 0;
    float currDist;
    
    while (travelled < 360){
        // Check 30 degree fan in front of current position
        // TODO: try other fan distances
        currDist = minDistance(15, false);
        if (verbose) {
            ROS_INFO("%f degrees relative rotation, min dist: %f", dir*travelled, currDist);
        }

        if (currDist >= reqDist && !(std::isinf(currDist)) ) {
            break;
        }

        rotByAngle(dir * DEG2RAD(increment), vel_pub, false); 
        travelled += increment;
    }

    if (verbose) {
        ROS_INFO("Stepping %f at %f speed at relative angle %f", stepSize, speed, dir*travelled);
    }

    stepDistance(stepSize, speed, vel_pub, false);

    return;
}

void spinAndStep(float stepSize, float speed, int Nbins, ros::Publisher *vel_pub, bool verbose) {
    /*
    Spin 360 then step in direction of most open space by (stepSize) with a velocity of (speed)
    Divides 360 into Nbins equal sections and scans each section
    */

    float binAngle = 360 / Nbins;
    float binDistances[Nbins];
    int maxBin=0;

    for (int i=0; i < Nbins; i++){
        // TODO: play around with passing a smaller binAngle so it doesn't get minDist triggered from walls to the side of the robot
        binDistances[i] = minDistance(binAngle, false);
        if (verbose) {
            ROS_INFO("Bin %d/%d min dist: %f", i+1, Nbins, binDistances[i]);
        }
        // update only if binDist is larger and not NaN or inf
        if ( !(std::isnan(binDistances[i]) || std::isinf(binDistances[i])) ){
            if ( (binDistances[i] > binDistances[maxBin]) || std::isinf(binDistances[maxBin]) ){
                maxBin = i;
            }
        }
        // skip final rotation
        if (i < (Nbins-1)){
            rotByAngle(DEG2RAD(binAngle), vel_pub, false); 
        }
    }

    if (verbose) {
        ROS_INFO("Stepping %f at %f speed in bin %d/%d", stepSize, speed, maxBin+1, Nbins);
    }

    //rotate to proper bin
    bool CCW = ((maxBin+1) < (Nbins-1-maxBin));

    if ( (maxBin != (Nbins)) && CCW ){
        // faster to spin CCW
        rotByAngle( (maxBin+1) * DEG2RAD(binAngle), vel_pub, false);
    }
    else if ( (maxBin != (Nbins)) && !(CCW) ){
        // faster to spin CW
        rotByAngle( -(Nbins-1-maxBin) * DEG2RAD(binAngle), vel_pub, false);
    }
    
    stepDistance(stepSize, speed, vel_pub, false);

    return;
}