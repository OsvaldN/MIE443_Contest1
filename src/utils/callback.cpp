#include "callback.h"

/*
/////////////////////////////////
/ ROS callback helper functions /
/////////////////////////////////
*/


//variables for rotational speed and linear speed
float angular = 0.0;
float linear = 0.0; 

//odom variables
float posX = 0.0;
float posY = 0.0;
float yaw = 0.0;

//bumper variables
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

//laser variables
uint32_t nLasers=0;
float laser_min_ang = 0.0;
float laser_max_ang = 0.0; 
float laser_ang_increment = 0.0;
float LaserArray[639] = {0}; 
float LaserArrayAng[639] = {0};

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