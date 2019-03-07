//
// Created by pedro on 07-03-2019.
//

// System Includes
#include <ros/ros.h>
#include "velodyne_laserscan/VelodyneLaserScan.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_laserscan_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    // create VelodyneLaserScan class
    velodyne_laserscan::VelodyneLaserScan n(nh, nh_priv);

    // handle callbacks until shut down
    ros::spin();

    return 0;
}