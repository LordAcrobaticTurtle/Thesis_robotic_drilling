#include <custom_control/complianceMovementController.h>
#include <cmath>
namespace thesis {
    complianceMovementController::complianceMovementController(ros::NodeHandle &p_nh) {
        ROS_INFO("CMC constructed");
        m_nh = p_nh;
        std::string controllerName = "/my_cartesian_compliance_controller/";
        m_pubTargetFrame = m_nh.advertise<geometry_msgs::PoseStamped>(controllerName + "target_frame",1000);
        m_pubTargetWrench = m_nh.advertise<geometry_msgs::WrenchStamped>(controllerName + "target_wrench",1000);
        m_subWrench = m_nh.subscribe<geometry_msgs::WrenchStamped>("/wrench",1000,&complianceMovementController::wrenchCallback,this);

        m_targetFrame.header.frame_id = "base_link";
        m_targetWrench.header.frame_id = "base_link";
        
        // ADJUST HOME POSITION
        m_home.pose.position.x = 0.459;
        m_home.pose.position.y = 0.200;
        m_home.pose.position.z = 0.200;

        m_currPose = m_home;

        m_currPose.header.frame_id = "base_link";
        m_currWrench.header.frame_id = "base_link";
        m_currWrench.wrench.force.x = 0;
        m_currWrench.wrench.force.y = 0;
        m_currWrench.wrench.force.z = 0;
        m_currWrench.wrench.torque.x = 0;
        m_currWrench.wrench.torque.y = 0;
        m_currWrench.wrench.torque.z = 0;

        m_timer = m_nh.createTimer(ros::Duration(1/m_rateHz), &cmc::main,this);
        // Jump into main loop
    };

    // complianceMovementController::complianceMovementController(ros::NodeHandle &p_nh, cmc::state p_state) {
    //     setMode(p_state);
    //     complianceMovementController(p_nh);
    // }

    complianceMovementController::~complianceMovementController() {
        ROS_INFO("CMC destroyed");
    };
    
    void complianceMovementController::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& data) {
        m_currWrenchMsg = data;
        m_currWrench.header = m_currWrenchMsg->header;
        m_currWrench.wrench = m_currWrenchMsg->wrench;
        // std::cout << m_currWrench << std::endl;
    }

    void cmc::setMode(state p_state) {
        m_state = p_state;
        ROS_INFO("New mode: %d", m_state);
    }



    void cmc::main(const ros::TimerEvent &t) {
        static double j = 0.0;
        static bool isAppRunning = true;
        // ROS_INFO("RUNNING - CMC main");
        
        if (!isAppRunning) {
            ROS_INFO("SHUTDOWN - CMC main");
            m_pubTargetFrame.publish(m_home);
            return;
        }

        // All commands must be passed in as metres
    
        // m_pubTargetWrench.publish(m_targetWrench);
        // We have a constant publisher here.
        m_targetFrame = m_currPose;
        m_targetFrame.pose.position.z = 0.25*sin(j)+0.3;
        // 2 N gate
        double response = PID(0,m_currWrench.wrench.force.z);
        m_targetWrench.wrench.force.z = response;
        
        
        if (abs(m_currWrench.wrench.force.z) >= 20.0) {
            isAppRunning = false;
            return;
        }

        ROS_INFO("CurrWrench Z: %f, PID response: %f", m_currWrench.wrench.force.z, response);

        std::cout << m_targetFrame << std::endl;
        
        m_targetWrench.header.frame_id = "base_link";
        m_pubTargetWrench.publish(m_targetWrench);
        // m_pubTargetFrame.publish(m_targetFrame);
        j += 0.00025;

        int i = 0;
        
        // std::cout << m_targetFrame << std::endl;
        // ROS_INFO("EXITING - CMC main");
    }


    double cmc::PID(double setpoint, double currValue) {
        static double prevError = 0;
        static double iSum = 0;
        static double iMax = 5.0;
        static double iMin = -iMax;
        static double PIDmax = 20;
        static double PIDmin = -PIDmax;
        double dt = 1.0/m_rateHz;

        // Calculate quantities
        double perror = setpoint - currValue;
        double derror = (perror-prevError) / dt;
        iSum += perror*dt;

        if (iSum > iMax)
            iSum = iMax;
        else if (iSum < iMin) 
            iSum = iMin;
        
        // Multipy by gains
        double pOut = m_kp*perror;
        double dOut = m_kd*derror;
        double iOut = m_ki*iSum;

        // Calculate result
        double result = pOut + iOut;

        // Prevent windup
        if (result > PIDmax) 
            result = PIDmax;
        else if (result < PIDmin)
            result = PIDmin;
        
        // Set error for next loop
        prevError = perror;

        return result;
    }


    
}