#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <yaml-cpp/yaml.h>
#include <custom_control/complianceMovementController.h>
#include "custom_control/jointTrajectoryController.h"
#include <tf/transform_listener.h>

int main(int argc, char ** argv) {
    // init ros
    ros::init(argc,argv,"cmc_node");
    ros::NodeHandle nh;
    // Init spinners
    ros::AsyncSpinner spin(4);
    spin.start();
    
    thesis::cmc ctrl(nh);
    // thesis::jointTrajectoryController ctl(nh);
    
    ros::Rate rate(ctrl.get_mRateHz());
    
    while (ros::ok()) {
        // ctrl.m_joyHandle.publish();
        rate.sleep();
    };

    ROS_INFO("Program exit");
    return 0;   
}

// try {
        //     listener.lookupTransform("base_link", "tool0", ros::Time(0), trf);
        //     std::cout << trf.getOrigin().getX() << std::endl;
        // } catch (tf::TransformException &ex) {
        //     ROS_ERROR("%s", ex.what());
        // }
        // ctrl.m_joyHandle.publish();
        // joyCtrl.publish();
        // ctl.get_mRateHz();