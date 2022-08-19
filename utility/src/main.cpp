#include "ros/ros.h"


int main(int argc, char** argv)
{
    //  ##### ROS setup #####

    ros::init(argc, argv, "utilityNode");
    ros::NodeHandle n;
    ROS_INFO("Node constructed");
    // n.advertiseService("move_to_pose",movePose);
    auto spinner = ros::AsyncSpinner(1);
    spinner.start();

    while (ros::ok()) {}

    return 0;
}