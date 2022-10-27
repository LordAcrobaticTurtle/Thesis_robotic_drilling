#pragma once
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <ros/timer.h>
#include <dynamic_reconfigure/client.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include "custom_control/joystick.h"

// Find a way to set dynamic reconfigure up inside this program. Or inside the launch file would be even better

#define TOLERANCE 0.01

namespace thesis {
    class complianceMovementController {
        public:
            enum state {
                ZERO,
                GENERAL   
            };
            complianceMovementController(ros::NodeHandle &p_nh);
            complianceMovementController(ros::NodeHandle &p_nh, float p_depth);
            ~complianceMovementController();
            void setMode(state p_state);
            void main(const ros::TimerEvent &t);
            joystick m_joyHandle;

            float get_mRateHz() {
                return m_rateHz;
            }
        private:
            // Nodehandle
            ros::NodeHandle m_nh;
            // ros::Rate m_rate;
            state m_state;
            float m_targetDepth;
            bool m_isAppRunning;
            // Publisher to target frame
            ros::Publisher m_pubTargetFrame;
            geometry_msgs::PoseStamped m_targetFrame;
            
            // Publisher to target wrench
            ros::Publisher m_pubTargetWrench;
            geometry_msgs::WrenchStamped m_targetWrench;

            geometry_msgs::PoseStamped m_home;
            geometry_msgs::PoseStamped m_posAfterHoming;
            geometry_msgs::PoseStamped m_currPose;
            geometry_msgs::WrenchStamped m_currWrench;
            tf::TransformListener  m_listener;

            // Wrench callback + data
            ros::Subscriber m_subWrench;
            void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& data);
            geometry_msgs::WrenchStamped::ConstPtr m_currWrenchMsg;

            double PID(double setpoint, double currentValue);
            double m_kp = 10;
            double m_kd = 1;
            double m_ki = 0.1;
            ros::Timer m_timer;
            
            // YAML config files that can be changed at runtime
            bool m_usePeaking;
            int m_stiffnessX;
            int m_stiffnessY;
            int m_stiffnessZ;
            // Feedrate currently not implemented
            float m_feedRate;
            float m_maxDrillForce;
            float m_maxHomeForce;
            std::string m_endEffectorLinkName;
            std::string m_baseLinkName;
            float m_drillBitWidth; 
            float m_tolerance;
            // Map to store 
            std::vector<std::tuple<float, float>> m_waypoints;

            // FIX FOR FRIDAY
            float m_rateHz = 50;
            
            void homing();
            void programmedHome();
            void computePecking(float targetDepth, float drillBitWidth );
            void updateWaypoints();
            void tfposCallback(const ros::TimerEvent &t);
            ros::Timer m_poseTimer;
            
            
    };
    typedef complianceMovementController cmc;
};


