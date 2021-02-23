#ifndef __CONTEST1_H_INCLUDED__
#define __CONTEST1_H_INCLUDED__

// ======= CONTEST 1 HEADER FILES =======
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

#include "bumper.h"
#include "callback.h"
#include "control.h"
#include "laser.h"
#include "openSpaceExplore.h"
#include "wallFollow.h"
#include "gridExplore.h"

// ======= MACROS & CONSTANTS =======
#define MAX_SPINRATE (M_PI/6)
#define SPEED_LIM (0.25)
#define OBS_SPEED_LIM (0.1)
#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) *180./M_PI)
#define DEG2RAD(deg) ((deg) *M_PI /180.)
#define NO_BUMPER_DEPRESSED -1
#define MIN_LASER_DIST 0.6

#endif