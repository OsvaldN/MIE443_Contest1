#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <cmath>
#include <chrono>

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
    //tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position: (%f,%f) Orientation: %f rad or %f degrees.", posX, posY,yaw,RAD2DEG(yaw));
}

/*
////////////////////////
/ Navigation functions /
////////////////////////
*/

int randRange(int low, int high){
    // Uniform random from interval [low, high]
    return low + ( rand() * (high-low) );
}

void VelPub(float angular, float linear, ros::Publisher vel_pub){
    /*
    Publish a velocity pair using using vel_pub
    TODO: make vel_pub global so it doesn't need to be passed?
     */

    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);

    return;
}

void rotByAngle(float angle, ros::Publisher vel_pub){
    /*
    Rotates the robot by (angle) degrees
	while this could be rad the abs() rounds the difference calculation
	and having a resolution of 1 rad would not be great
     */

    // Rotate at maximum speed in direction of angle
    float dir = (angle > 0) - (angle < 0);
    float rotVel = dir * MAX_SPINRATE;

    ros::spinOnce(); //TODO: if spin stays in main loop maybe remove from here?
    float startYaw = yaw;

    // start spinning
    VelPub(rotVel, 0.0, vel_pub);
    ROS_INFO("Set rotation velocity: %f", rotVel);
    ROS_INFO("Start Orientation: %f rad or %f degrees.",yaw,RAD2DEG(yaw));
    // this might have a problem at wrap-around if switching between -pi and pi
    while ((yaw - startYaw) < angle) {    
        VelPub(rotVel, 0.0, vel_pub);
	ros::spinOnce();
	//ROS_INFO("Mid-spin: %f rad, diff: %f, angle: %f, started at: %f",yaw,yaw- startYaw,angle, startYaw);
    	//exit(0);
    }

    // stop rotating
    VelPub(0.0, 0.0, vel_pub);

    return;
}

void spinAndStep() {
    /*
    Spin 360 then step in direction of most open space
     */

    //pseudo-code
    
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

        rotByAngle(0.1, vel_pub);
        rotByAngle(-0.1, vel_pub);

        // remove??
        //vel.angular.z = angular;
        //vel.linear.x = linear;
        //vel_pub.publish(vel);

        // TODO: display type of motion taking place in current loop

        // display motion info
        ROS_INFO("Linear  Velocity: %f", linear);
        ROS_INFO("Angular Velocity: %f", angular);
        ROS_INFO("Position: (%f,%f) Orientation: %f degrees Range: %f", posX, posY,RAD2DEG(yaw),minLaserDist);
        
        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
