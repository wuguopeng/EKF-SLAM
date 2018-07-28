#include <ros/ros.h>

#include "ekfslam.h"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "myekf");
    ekfslam ekf_slam;

    ros::spin();

    return 0;
}
