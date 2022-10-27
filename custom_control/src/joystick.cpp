#include "custom_control/joystick.h"

// m_subWrench = m_nh.subscribe<geometry_msgs::WrenchStamped>("/wrench",1000,&complianceMovementController::wrenchCallback,this);

namespace thesis {

    joystick::joystick() {}

    joystick::joystick(ros::NodeHandle &p_nh) {
        ROS_INFO("JOYSTICK CONSTRUCTED");
        m_nh = p_nh;
        m_subJoy = m_nh.subscribe<sensor_msgs::Joy>("/joy", 1000, &joystick::joyCallback,this);
        m_pubWrench = m_nh.advertise<geometry_msgs::WrenchStamped>("/wrench", 1000);
        cbTime = m_nh.createTimer(ros::Duration(1/50), &joystick::cbTimer, this);
    }
    
    joystick::~joystick() {
        ROS_INFO("JOYSTICK DESTROYED");
    }

    void joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& p_data) {
        m_joyState = *p_data;
        // ROS_INFO("Joystick msg received");
        // std::cout << m_joyState << std::endl;
    }

    void joystick::publish() {
        // joyToSensorWrench();
    }

    void joystick::joyToSensorWrench() {
        // ROS_INFO("Converting joy state to a wrench msg");
        geometry_msgs::WrenchStamped topic;
        
        if (m_joyState.axes.size() <= 0) {
            // ROS_DEBUG("joyToSensorWrench: Empty joy msg");
            return;
        }
        // ROS_INFO("Publishing sensor message");
        topic.header.frame_id = "base_link";
        topic.header.stamp = ros::Time::now();
        topic.wrench.force.z = 20*(m_joyState.axes[4]);
        m_pubWrench.publish(topic);        
    }

    void joystick::cbTimer(const ros::TimerEvent &t) {
        joyToSensorWrench();
    }

}