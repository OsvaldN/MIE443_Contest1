#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <cmath>
#include <chrono>
#include <math.h>
#include <queue>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#define MAX_SPINRATE (M_PI/6)
#define SPEED_LIM (0.25)
#define OBS_SPEED_LIM (0.1)
#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) *180./M_PI)
#define DEG2RAD(deg) ((deg) *M_PI /180.)
#define NO_BUMPER_DEPRESSED -1
#define MIN_LASER_DIST 0.6

void laserObstacleDirectionHandler(ros::Publisher *vel_pub, bool verbose);

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

int rotByAngle(float angle, ros::Publisher *vel_pub, bool verbose=true){
    //Rotates the robot by (angle) radians about the z-axis

    ros::Rate loop_rate(10);
    // Rotate at maximum speed in direction of angle
    float dir = (angle > 0) - (angle < 0);
    float rotVel = dir * MAX_SPINRATE;

    ros::spinOnce();
    float startYaw = yaw;
    float lastYaw = yaw;
    
    // initiate queue of past yaws with infs
    std::queue<float> yawQueue;
    for (int i=0;i<5;i++) { yawQueue.push(std::numeric_limits<float>::infinity());}

    if (verbose){
        ROS_INFO("Rotating by %f radians at vel: %f", angle, rotVel);
        ROS_INFO("Starting Yaw: %f rad / %f degrees.", yaw, RAD2DEG(yaw));
    }

    while (angleCorrect(yaw, startYaw) < fabs(angle)) {    
        // publish to update velocity, spin to update yaw (clears velocity)
        VelPub(rotVel, 0.0, vel_pub);
        loop_rate.sleep();
        ros::spinOnce();
	
        //update yawQueue and check distance travelled
        yawQueue.push(yaw);
        yawQueue.pop();
        // if last 5 spins don't meet 0.05rad threshold rotation then give up
        if (angleCorrect(yawQueue.front(), yawQueue.back()) < 0.05) {
            if (verbose) {
                ROS_INFO("Got stuck trying to turn after %f degrees", RAD2DEG(angleCorrect(yaw, startYaw)) );
            }
            return 1;
        }
    }

    return 0;
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

uint8_t specificBumperPressed() {
    /* This will the specific bumper that is depressed. If there are no bumpers depressed, a failure message 
    */

    for(uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        if (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED) {
            return b_idx;
        }
    }

    return NO_BUMPER_DEPRESSED;

}


float minDistance(int32_t desiredAngleR, int32_t desiredAngleL=std::numeric_limits<int32_t>::infinity(), bool verbose=false){


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

void stuck_check() {

    return;

}


int stepDistance(float distance, float speed,ros::Publisher *vel_pub, bool verbose=true, bool reverse=false) {
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
        minLaserDistance = minDistance(15);
        if (minLaserDistance < MIN_LASER_DIST || std::isinf(minLaserDistance) || std::isnan(minLaserDistance)) {
            laserObstacleDirectionHandler(vel_pub, true); // DEBUG: Change verbosity to false to limit console messages
        }

        // publish to update speed, spin to update pos (clears velocity)
        if (reverse) {
            VelPub(0.0, -speed, vel_pub);

        } else {
            VelPub(0.0, speed, vel_pub);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

    if (bumpersPressed()){
	return 1;
    }

    return 0;
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


void laserObstacleDirectionHandler(ros::Publisher *vel_pub, bool verbose=false) {
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
    
    // Implementation for Wall Follow algo
    // Step #1: Scan surroundings and determine direction of most open space
    spinAndStep(1, SPEED_LIM, 5, &vel_pub);

    while(ros::ok() && secondsElapsed <= 900) {
        ros::spinOnce();

        // Step #2: Perform wall follow operation
        wallFollow(&vel_pub); 

        // display motion info
        ROS_INFO("Position: (%f,%f) Orientation: %f degrees", posX, posY,RAD2DEG(yaw));
        
        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    ROS_INFO("TERMINATED: Turtlebot terminated after 15 min of navigation");

    return 0;
}
