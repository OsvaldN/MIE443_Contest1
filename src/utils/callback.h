#ifndef __CALLBACK_H_INCLUDED__
#define __CALLBACK_H_INCLUDED__

#include "contest1.h"

// ==== FORWARD DECLARATION FOR CALLBACK FUNCTION VARIABLES ====
extern float angular;
extern float linear; 
extern float posX;
extern float posY;
extern float yaw;
extern uint8_t bumper[3];
extern uint32_t nLasers;
extern float laser_min_ang; 
extern float laser_max_ang;
extern float laser_ang_increment;
extern float LaserArray[639];
extern float LaserArrayAng[639];

// ==== CALLBACK FUNCTION DECLARATIONS ====== 
extern void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
extern void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
extern void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

#endif