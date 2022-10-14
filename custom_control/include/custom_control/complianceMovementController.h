#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <ros/timer.h>
#include "custom_control/joystick.h"
#include <dynamic_reconfigure/client.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
// Find a way to set dynamic reconfigure up inside this program. Or inside the launch file would be even better

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
            // Publisher to target frame
            ros::Publisher m_pubTargetFrame;
            geometry_msgs::PoseStamped m_targetFrame;
            
            // Publisher to target wrench
            ros::Publisher m_pubTargetWrench;
            geometry_msgs::WrenchStamped m_targetWrench;

            geometry_msgs::PoseStamped m_home;
            geometry_msgs::PoseStamped m_currPose;
            geometry_msgs::WrenchStamped m_currWrench;
            tf::TransformListener m_listener;
            tf::StampedTransform m_transform;
            // Wrench callback + data
            ros::Subscriber m_subWrench;
            void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& data);
            geometry_msgs::WrenchStamped::ConstPtr m_currWrenchMsg;

            double PID(double setpoint, double currentValue);
            double m_kp = 10;
            double m_kd = 1;
            double m_ki = 0.1;
            ros::Timer m_timer;
            
            // FIX FOR FRIDAY
            float m_rateHz = 50 ;
            
            void homing();
            
            
    };
    typedef complianceMovementController cmc;
};


