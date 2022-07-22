#include "ros/ros.h"

#include <sstream>
#include <fstream>
#include <cstdlib>

#include <move_it_api/pose_server.h>
// This is for interfacing with Moveit move group
#include <moveit/move_group_interface/move_group_interface.h>
// These are for the various ROS message formats we need
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
    //  ##### ROS setup #####
    if (argc < 8) {
        ROS_WARN("Usage: rosrun movement_controller moveToPose_client x y z ax ay az gripper");
        return 1;
    }
    ros::init(argc, argv, "move_to_pose_client");
    auto nh = ros::NodeHandle{};

    ros::ServiceClient mover = nh.serviceClient<move_it_api::pose_server>("move_to_pose");
    geometry_msgs::Pose target;
    target.position.x = atof(argv[1]);
    target.position.y = atof(argv[2]);
    target.position.z = atof(argv[3]);

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(atof(argv[4]), atof(argv[5]), atof(argv[6]));

    target.orientation = tf2::toMsg(myQuaternion);

    move_it_api::pose_server srv;
    srv.request.pose = target;
    if (mover.call(srv)) {
        ROS_INFO("Response: %d", srv.response.success.data);
    } else {
        ROS_ERROR("Failed to call service move_to_pose");
        return 1;
    }

    return 0;
}