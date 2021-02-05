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
    tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position: (%f,%f) Orientation: %f rad or %f degrees.", posX, posY,yaw,RAD2DEG(yaw));
}

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

    while(ros::ok() && secondsElapsed <= 900) {
        ros::spinOnce();
        //

        ROS_INFO("Position: (%f,%f) Orientation: %f degrees Range", posX, posY,RAD2DEG(yaw),minLaserDist);

        bool any_bumper_pressed=false;

        angular = 0.0;
        linear = 0.2;

        //check to see if any of the bumpers are being pressed
        for(uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            any_bumper_pressed|=(bumper[b_idx] ==kobuki_msgs::BumperEvent::PRESSED);
        }
        //
        // Control logic after bumpers are being pressed.

        //if the front bumper is pressed reverse and turn following sequence of events
    
        if (F_bumper_idx >= 1 && F_bumper_idx < 5){
            angular =0.0;
            linear =-0.2;
            F_bumper_idx += 1;

        }
        else if (F_bumper_idx >= 1 && F_bumper_idx < 40){
            angular = M_PI /6;
            linear = 0.0;
            F_bumper_idx += 1;

        }
        else if (F_bumper_idx >= 1){
            F_bumper_idx = 0;

        }
        // If side bumpers are pressed turn away from the wall

        if (R_bumper_idx >= 1 && R_bumper_idx < 12){
            angular = M_PI/6;
            linear = 0.0;
            R_bumper_idx += 1;

        }
        else if (R_bumper_idx >= 1){
            R_bumper_idx = 0;

        }

        if (L_bumper_idx >= 1 && L_bumper_idx < 12){
            angular = -M_PI/6;
            linear = 0.0;
            L_bumper_idx += 1;

        }
        else if (L_bumper_idx >= 1){
            L_bumper_idx = 0;

        }


        if (bumper[1] ==kobuki_msgs::BumperEvent::PRESSED) {
            F_bumper_idx = 1;
        }
        //if right bumper is pressed turn to the left
        else if (bumper[2] ==kobuki_msgs::BumperEvent::PRESSED) {
            R_bumper_idx = 1;
        }
        //if left bumper is pressed turn to the right
        else if (bumper[0] ==kobuki_msgs::BumperEvent::PRESSED) {
            L_bumper_idx = 1;
        }
        
        /*
        //if there is space in front and the bumpers are not pressed go forward
        else if (minLaserDist>1. && !any_bumper_pressed) {
            linear = 0.1;

            //correct robot's trajectory so that its x position is 0.5 andits orientation is 90 degrees
            if (yaw <17 / 36*M_PI || posX>0.6) {
                angular = -M_PI/12.;
            }
            else if (yaw <19 / 36*M_PI || posX<0.4) {
                angular = -M_PI/12.;
            }
            else {
                angular = 0;
            }
        }
        else {
            angular =0.0;
            linear =0.0;
            break;
        }
        */

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // "XXXXX you can print stuff here for debugging XXXXX"
        std::cout << "XXXXX you can print stuff here for debugging XXXXX" << std::endl;
        std::cout << "XXXXX you can print stuff here for debugging XXXXX" << std::endl;
        std::cout << "XXXXX you can print stuff here for debugging XXXXX" << std::endl;

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
