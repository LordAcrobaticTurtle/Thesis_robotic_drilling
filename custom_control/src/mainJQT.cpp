#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include "custom_control/jointTrajectoryController.h"
#define NUMARGUMENTS 1


int main(int argc, char ** argv) {
    if (argc < NUMARGUMENTS) {
        ROS_INFO("Usage: ./exe < >");
        return -1;
    }
    // They should be on parameter server
    // float depth = atof(argv[1]);
    // if (depth <= 0) {
    //     ROS_INFO("Invalid depth");
    //     return -1;
    // }
    // init ros
    ros::init(argc,argv,"jtc_node");
    ros::NodeHandle nh;
    // Init spinners
    ros::AsyncSpinner spin(4);
    spin.start();

    thesis::jointTrajectoryController ctl(nh);
    ros::Rate rate(ctrl.get_mRateHz());

    while (ros::ok()) {
        ctl.get_mRateHz();
        rate.sleep();
    };

    ROS_INFO("Program exit");
    return 0;   
}