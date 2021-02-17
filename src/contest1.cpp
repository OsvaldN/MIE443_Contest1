#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <cmath>
#include <chrono>
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#define MAX_SPINRATE (M_PI/6)
#define SPEED_LIM (0.25)
#define OBS_SPEED_LIM (0.1)
#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) *180./M_PI)
#define DEG2RAD(deg) ((deg) *M_PI /180.)
#define MIN_LASER_THRESHOLD 100. // Default laser distance for minimum laser distance calculation

uint16_t DEBUG_COUNT = 0; //DEBUG REMOVE

//variables for rotational speed and linear speed
float angular = 0.0;
float linear = 0.0; 

//odom variables
float posX = 0.0, posY = 0.0, yaw = 0.0;

//bumper variables
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

//laser variables

int32_t nLasers=0;
float laser_min_ang = 0, laser_max_ang =0, laser_ang_increment =0;
float LaserArray[639]; // unfortunately we have to hard code this size, unless we try to use malloc on the array or use a vector instead
float LaserArrayAng[639];

float minDistanceDir = 0.0; // direction of the minimum distance

/*
//////////////////////
/ Callback functions /
//////////////////////
*/

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT,CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //obtain the size of the message array recieved from the laser (number of lasers)
    nLasers = (msg->angle_max - msg->angle_min)/msg->angle_increment;

    // copy data so it can be used in other sets of code
    laser_min_ang = msg->angle_min;
    laser_max_ang = msg->angle_max;
    laser_ang_increment = msg->angle_increment;

    //return an array with the angle of each laser with the same indexing as LaserArray

    if (LaserArrayAng[0] == 0){

        float angle_temp = laser_min_ang;

        for (uint32_t angle_idx = 0; angle_idx < nLasers; ++angle_idx){
            LaserArrayAng[angle_idx] = angle_temp;
            angle_temp = angle_temp + laser_ang_increment;
        }

    }

    //copy the ranges array to LaserArray
    //Note: the the laser scans from right to left so LaserArray[0] is the distance all the way to the right of FOV

    for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx){
        LaserArray[laser_idx] = msg->ranges[laser_idx];
        //ROS_INFO("LaserArray element %i is: %f", laser_idx, LaserArray[laser_idx]);

    }

    //msg->angle_max = max angle of the sensor (0.5242763 rad or 29.88 deg)
    //msg->angle_min = minimum angle of the sensor (-0.5215679 rad or 30.04 deg)
    //msg->angle_increment = the resolution of the sensor in radians (0.001636689 rad or 0.09434 deg)
    //msg->ranges[] = the array of distances
    
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}

/*
////////////////////////
/ Navigation functions /
////////////////////////
*/


// random number generator
float randRange(float low, float high){
    // Uniform random from interval [low, high]
    return low + ( (rand() / double(RAND_MAX)) * (high-low) );
}

float angleCorrect(float curr, float start){
    // corrects angle discontinuity in yaw
    float x = fabs(curr-start);
    float y = fabs(360+curr-start);
    float z = fabs(-360+curr-start);
    return x < y ? (x < z ? x : z) : (y < z ? y : z); // bootleg way to get min of the three
}

void VelPub(float angular, float linear, ros::Publisher *vel_pub){
    //Publish a velocity pair using using vel_pub
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub->publish(vel);
    return;
}

void rotByAngle(float angle, ros::Publisher *vel_pub, bool verbose=true){
    /*
    Rotates the robot by (angle) radians about the z-axis
     */

    ros::Rate loop_rate(10);
    // Rotate at maximum speed in direction of angle
    float dir = (angle > 0) - (angle < 0);
    float rotVel = dir * MAX_SPINRATE;

    ros::spinOnce(); //TODO: if spin stays in main loop maybe remove from here?
    float startYaw = yaw;

    if (verbose){
        ROS_INFO("Rotating by %f radians at vel: %f", angle, rotVel);
        ROS_INFO("Starting Yaw: %f rad / %f degrees.", yaw, RAD2DEG(yaw));
    }

    // this might have a problem at wrap-around if switching between -pi and pi
    while (angleCorrect(yaw, startYaw) < fabs(angle)) {    
        // publish to update velocity, spin to update yaw (clears velocity)
        VelPub(rotVel, 0.0, vel_pub);
        loop_rate.sleep();
        ros::spinOnce();
    }

    return;
}

float eucDist(float x1, float y1, float x2, float y2){
    // returns euclidean distance between (x1,y1) and (x2,y2)
    return sqrt(pow((x2-x1), 2) + pow((y2-y1), 2) );
}

bool bumpersPressed(){
    //return 1 if any bumpers are currently being pressed

    bool any_bumper_pressed = false;

    for(uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        any_bumper_pressed |= ( bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED );
    }

    return any_bumper_pressed;
}

float minDistance(int32_t desiredAngle, bool verbose=false){
    //returns the minimum distance in a view of the desired angle in DEGREES on either side
    
    //set the minimum distance to a very large value where a new smaller value can be found
    float minLaserDist = MIN_LASER_THRESHOLD;
    int32_t desiredNLasers=0;
    minDistanceDir = 0.0; // store the direction of minimum direction into this global variable
    //determine the number of laser needed to scan the desired angle.  Note: convert desired angle from degrees to radians
    desiredNLasers = desiredAngle*M_PI/(179*laser_ang_increment);

    //search the range from the negative and positve desired angle (converted to radians)
    if (desiredAngle*M_PI/179 < laser_max_ang && -desiredAngle*M_PI/ 180 > laser_min_ang){
   
        for (uint32_t laser_idx = nLasers/2-desiredNLasers; laser_idx < nLasers/2+desiredNLasers; ++laser_idx){

            if (LaserArray[laser_idx] < minLaserDist){
                minDistanceDir = LaserArrayAng[laser_idx];
            }
            minLaserDist = std::min(minLaserDist, LaserArray[laser_idx]);
            
        }

        if (verbose){
            ROS_INFO("Search in range: %i to %i degrees with %i number of lasers", -desiredAngle, desiredAngle, 1*desiredNLasers);
            ROS_INFO("minLaserDist measured is: %f and it direction relative to the robot is %f CCW", minLaserDist, minDistanceDir);
        }

    }
    //if the desired angle is larger than the sensor field, use the maximum angle
    else {
        for(uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx){

            if (LaserArray[laser_idx] < minLaserDist){
                minDistanceDir = LaserArrayAng[laser_idx];
            }
            minLaserDist = std::min(minLaserDist, LaserArray[laser_idx]);
            

        }

        if (verbose){
            ROS_INFO("Search max range of laser with max number of lasers which is %i", nLasers);
            ROS_INFO("minLaserDist measured is: %f and it direction relative to the robot is %f CCW", minLaserDist, minDistanceDir);
        }

    }

    return minLaserDist;
}


void stepDistance(float distance, float speed,ros::Publisher *vel_pub, bool verbose=true) {
    /*
    Moves the robot forward (distance) units at (speed)
        stops if there is a collision
     */
    
    ros::Rate loop_rate(10);
    // get start values
    ros::spinOnce();
    float startX = posX, startY = posY;
    float minLaserDistance; // Check the min distance from the laser reading

    if (verbose){
        ROS_INFO("Stepping %f units at speed: %f", distance, speed);
        ROS_INFO("Starting Yaw: %f rad / %f degrees.", yaw, RAD2DEG(yaw));
    }

    while ( (eucDist(startX, startY, posX, posY) < distance) && !(bumpersPressed()) ){
        // Check if obstacle infront of Robot before moving
        minLaserDistance = minDistance(10);
        if (minLaserDistance < 0.5 || minLaserDistance == MIN_LASER_THRESHOLD) {
            break;
        }

        // publish to update speed, spin to update pos (clears velocity)
        VelPub(0.0, speed, vel_pub);
        loop_rate.sleep();
        ros::spinOnce();
    }

    return;
}


float GetYawAngle(float input_angle, bool verbose=false){

    //input the angle relative to the robot (eg. LaserArrayAng[0] which is -0.52rad CCW offset (or all the way to the right) from directly in front of the robot)
    //and return the Yaw angle of that ranging from -pi to pi (in radians)

    float Output_YawAngle = input_angle + yaw;

    //ensure that yaw is within -pi to pi
    if (Output_YawAngle < -M_PI){
        Output_YawAngle = Output_YawAngle + (2*M_PI);
    }

    if (Output_YawAngle > M_PI){
        Output_YawAngle = Output_YawAngle - (2*M_PI);
    }

    if (verbose){
        ROS_INFO("angle relative to robot is: %f the corresponding yaw angle is: %f and the current robot yaw is: %f CCW", input_angle, Output_YawAngle, yaw);
    }

    return Output_YawAngle;
}



void wallFollow(ros::Publisher *vel_pub) {
    /* 
    If robot encouters a wall, make a +CCW rotation and keep wall on RHS. Otherwise proceed
    forward.
    */
    
    float minLaserDist = minDistance(10, true); // DEBUG REMOVE verbose
    
    if (bumpersPressed() || minLaserDist < 0.5 || minLaserDist == MIN_LASER_THRESHOLD) {
        
        float rand = randRange(0.0, 3.0);

        if (rand < 2.0) { // Randomize the rotation of the robot
            rotByAngle(M_PI/4, vel_pub);
        } else {
            rotByAngle(-M_PI/4, vel_pub);
        }

    } else {
        stepDistance(50, SPEED_LIM, vel_pub);
    }

    return;
}

void spinAndStep(float stepSize, float speed, int Nbins, ros::Publisher *vel_pub, bool verbose=true) {
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

void spinToDist(float stepSize, float speed, float reqDist, float dir, float increment, ros::Publisher *vel_pub, bool verbose=true ) {
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

/*
////////
/ main /
////////
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);

    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    
    // random seed
    //srand(0)

    while(ros::ok() && secondsElapsed <= 900) {
        ros::spinOnce();

        wallFollow(&vel_pub); 
        
	    // random spin followed by random step example
        //rotByAngle(randRange(-M_PI/2, M_PI/2), &vel_pub);
        //stepDistance(randRange(0.0, 100.0), SPEED_LIM, &vel_pub);

        spinAndStep(1, SPEED_LIM, 5, &vel_pub);
        spinToDist(2, SPEED_LIM, 1, 1, 30, &vel_pub);

        /*
        rotByAngle(-M_PI/2, &vel_pub);
        stepDistance(50, SPEED_LIM, &vel_pub);
        rotByAngle(-M_PI/2, &vel_pub);
        stepDistance(100, SPEED_LIM, &vel_pub);
        */


        // TODO: display type of motion taking place in current loop
        //          in high-level controller blocks make appropriate print statements

        // display motion info
        ROS_INFO("Position: (%f,%f) Orientation: %f degrees", posX, posY,RAD2DEG(yaw));
        
        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
