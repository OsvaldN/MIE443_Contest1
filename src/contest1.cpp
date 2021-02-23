#include "contest1.h"

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
    
  

    while(ros::ok() && secondsElapsed <= 900) {
        ros::spinOnce();

        // Explore environment
        gridExplore(vel_pub);

        // display motion info
        ROS_INFO("Position: (%f,%f) Orientation: %f degrees", posX, posY,RAD2DEG(yaw));
        
        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    ROS_INFO("TERMINATED: Turtlebot terminated after 15 min of navigation");

    return 0;
}
