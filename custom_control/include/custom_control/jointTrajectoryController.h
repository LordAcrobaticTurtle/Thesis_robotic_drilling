#pragma once
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <ros/timer.h>
#include "custom_control/joystick.h"
#include <dynamic_reconfigure/client.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <trajectory_msgs/JointTrajectory.h>



// /pos_joint_traj_controller/command

namespace thesis {
    class jointTrajectoryController {
        public:
            // Publisher to 
            jointTrajectoryController();
            jointTrajectoryController(ros::NodeHandle &p_nh);
            ~jointTrajectoryController();

            void main(const ros::TimerEvent &t);

            float get_mRateHz() {
                return m_rateHz;
            }


        private:
            ros::NodeHandle m_nh;

            void compute();
            void publish();

            float m_rateHz;
            // ROS related functions
            // Publishers
            ros::Publisher m_pubTargetTraj;
            
            // Subscribers
            ros::Subscriber m_subTargetWrench;
            ros::Subscriber m_subTargetPos;
            ros::Subscriber m_subSenseWrench;


            // Data
            trajectory_msgs::JointTrajectory m_targetTraj;
            geometry_msgs::WrenchStamped m_currWrench;

            

    } typedef JTC;
}