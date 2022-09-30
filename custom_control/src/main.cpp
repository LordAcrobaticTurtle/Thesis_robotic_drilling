#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <custom_control/complianceMovementController.h>

#define NUMARGUMENTS 2

int main(int argc, char ** argv) {
    if (argc < NUMARGUMENTS) {
        ROS_INFO("Usage: ./exe <mode (0/1)>");
        return -1;
    }
    ros::init(argc,argv,"cmc_node");
    ros::NodeHandle nh;
    thesis::cmc ctrl(nh);
    ctrl.setMode((thesis::cmc::state) atoi(argv[1]));
    ros::spin();
    ROS_INFO("Program exit");
    return 0;   
}