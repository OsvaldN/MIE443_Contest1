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

#define MAX_SPINRATE (0.6)
#define SPEED_LIM (0.25)
#define OBS_SPEED_LIM (0.1)
#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) *180./M_PI)
#define DEG2RAD(deg) ((deg) *M_PI /180.)

//variables for rotational speed and linear speed
float angular = 0.0;
float linear = 0.0; 

//odom variables
float posX = 0.0, posY = 0.0, yaw = 0.0;

//bumper variables
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

//laser variables
float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredAngle=5;

//variable triggered if front bumper pressed to start sequence of events
int32_t F_bumper_idx =0;
//variable triggered if one of side bumpers is pressed to start sequence of events
int32_t L_bumper_idx =0;
int32_t R_bumper_idx =0;


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
	minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min)/msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI/(180*msg->angle_increment);
    // TODO: consider removing this, it gets printed a lot, is it informative?
    ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);

    if (desiredAngle*M_PI/180 < msg->angle_max && -desiredAngle*M_PI/ 180 > msg->angle_min){
        for (uint32_t laser_idx = nLasers/2-desiredNLasers; laser_idx < nLasers/2+desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist,msg->ranges[laser_idx]);
        }
    }
    else {
        for(uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist,msg->ranges[laser_idx]);

        }
    }
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

// TODO: test this function out, not sure if it works properly
int randRange(float low, float high){
    // Uniform random from interval [low, high]
    return low + ( (rand() / double(RAND_MAX)) * (high-low) );
}

void VelPub(float angular, float linear, ros::Publisher *vel_pub){
    /*
    Publish a velocity pair using using vel_pub
     */

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
    while (fabs(yaw - startYaw) < fabs(angle)) {    
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
    /*
    return 1 if any bumpers are currently being pressed
     */

    bool any_bumper_pressed = false;

    for(uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        any_bumper_pressed |= ( bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED );
    }

    return any_bumper_pressed;
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

    if (verbose){
        ROS_INFO("Stepping %f units at speed: %f", distance, speed);
        ROS_INFO("Starting Yaw: %f rad / %f degrees.", yaw, RAD2DEG(yaw));
    }

    while ( (eucDist(startX, startY, posX, posY) < distance) && !(bumpersPressed()) ){
        // publish to update speed, spin to update pos (clears velocity)
        VelPub(0.0, speed, vel_pub);
        loop_rate.sleep();
        ros::spinOnce();
    }

    return;
}

void spinAndStep(float step) {
    /*
    Spin 360 then step in direction of most open space
     */

    //code here
    
    return;
}

void spinToDist(float reqDist, float dir) {
    /*
    Rotates in direction (dir) until the laser callback finds an open distance of (reqDist)
     */

    //spin and scan and then stop once minDist > reqDist

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

	// random spin followed by random step
        //rotByAngle(randRange(-M_PI/2, M_PI/2), &vel_pub);
        //stepDistance(randRange(0.0, 100.0), SPEED_LIM, &vel_pub);
	

        rotByAngle(-M_PI/2, &vel_pub);
        stepDistance(50, SPEED_LIM, &vel_pub);
        rotByAngle(-M_PI/2, &vel_pub);
        stepDistance(100, SPEED_LIM, &vel_pub);


        // TODO: display type of motion taking place in current loop
        //          in high-level controller blocks make appropriate print statements

        // display motion info
        ROS_INFO("Position: (%f,%f) Orientation: %f degrees Range: %f", posX, posY,RAD2DEG(yaw),minLaserDist);
        
        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}