#include "ros/ros.h"
// #include "std_srvs/Empty.h"
#include <sstream>
#include <fstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


geometry_msgs::Pose target;

static const std::string model_path = "/home/mtrn4230/lab_demo_repos/lab09_demo/lab09_gazebo/models/box/box.sdf";
static const std::string PLANNING_GROUP = "manipulator";

bool movePose(geometry_msgs::Pose &req, 
              geometry_msgs::Pose &res) 
            {
                ROS_INFO("%f, %f, %f", req.position.x, req.position.y, req.position.z);
                // ROS_INFO("Gripper %s", req.gripper ? "on" : "off");
                
                target = req;
                // gripper = req.gripper;
                return true;
            }


int main(int argc, char** argv)
{
    //  ##### ROS setup #####

    ros::init(argc, argv, "move_to_pose_server");
    ros::NodeHandle n;
    
    auto spinner = ros::AsyncSpinner(1);
    spinner.start();

    auto move_group = moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    auto const* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    auto my_plan = moveit::planning_interface::MoveGroupInterface::Plan{};

    ROS_INFO("Move to home configuration");

    // Options are currently, "zero", "home" and "up"
    auto group_state = "home";
    move_group.setNamedTarget(group_state);
    // Check if plan is possible
    auto success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success) {
        ROS_WARN("Unable to plan path. Ensure goal pose is valid or adjust tolerance");
        return 1;
    }

    // Execute motion to home position
    ROS_INFO("Found path to %s, moving robot...", group_state);
    move_group.move();



    ROS_INFO("Accepting pose coordinates");    
    
    ros::Rate loop_rate(0.5);
    target.position.x = 0;
    target.position.y = 0;
    target.position.z = 0;

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0,0,0);
    target.orientation = tf2::toMsg(myQuaternion);

    geometry_msgs::Pose oldTarget;
    oldTarget.position.x = 0;
    oldTarget.position.y = 0;
    oldTarget.position.z = 0;


    myQuaternion.setRPY(0,0,0);
    oldTarget.orientation = tf2::toMsg(myQuaternion);
    bool oldGripper = false;
    while (ros::ok()) {
        ROS_INFO("%f, %f %f", target.position.x, target.position.y, target.position.z);
        if (oldTarget != target) { 
            ROS_INFO("MOVE TO A POSE GOAL...");
            ROS_INFO("%f, %f, %f", target.position.x, target.position.y, target.position.z);
            // Subtract arm pose from target so card spawns and arm pose are aligned. 
            target.position.x -= 0.8;
            target.position.y -= 0;
            target.position.z -= 0.775001;
            move_group.setPoseTarget(target);
            success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            oldTarget = target;
            if(!success) {
                ROS_WARN("Unable to plan path. Ensure goal pose is valid or adjust tolerance");
            } else {
                ROS_INFO("Found a path to pose... moving");
                move_group.move();
            }
        }       
        loop_rate.sleep();

    }

    return 0;
}