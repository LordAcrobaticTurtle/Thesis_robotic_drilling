#include <custom_control/complianceMovementController.h>
#include <cmath>
namespace thesis {
    complianceMovementController::complianceMovementController(ros::NodeHandle &p_nh) : m_joyHandle(p_nh) {
        ROS_INFO("CMC constructed");
        m_nh = p_nh;
        std::string controllerName = "/my_cartesian_compliance_controller/";
        m_pubTargetFrame = m_nh.advertise<geometry_msgs::PoseStamped>(controllerName + "target_frame",1000);
        m_pubTargetWrench = m_nh.advertise<geometry_msgs::WrenchStamped>(controllerName + "target_wrench",1000);
        m_subWrench = m_nh.subscribe<geometry_msgs::WrenchStamped>(controllerName + "ft_sensor_wrench",1000,&complianceMovementController::wrenchCallback,this);

        m_targetFrame.header.frame_id = "base_link";
        m_targetWrench.header.frame_id = "base_link";
        
        // ADJUST HOME POSITION
        m_home.pose.position.x = 0.-459;
        m_home.pose.position.y = 0.-200;
        m_home.pose.position.z = 0.300;
        m_home.pose.orientation.w = 0;
        m_home.pose.orientation.x = 1;
        m_home.pose.orientation.y = 0;
        m_home.pose.orientation.z = 0;
        
        m_currPose = m_home;
        m_home.header.frame_id = "base_link";
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
            ROS_INFO("SHUTDOWN - CMC main - Final pos:");
            std::cout << m_targetFrame << std::endl;
            m_pubTargetFrame.publish(m_home);
            return;
        }

        m_targetFrame = m_home;
        m_targetFrame.pose.position.z = -0.2*sin(j)+0.3;

        double target = 0;

        // Check if Joystick values exist for setpoint
        if (m_joyHandle.m_joyState.axes.size() > 0) {
            target = 20*m_joyHandle.m_joyState.axes[1];
        }
        
        // Calculate response and setup wrench data struct
        double response = PID(target,m_currWrench.wrench.force.z);
        m_targetWrench.wrench.force.z = response;
        m_targetWrench.header.stamp = ros::Time::now();
        m_targetWrench.header.frame_id = "base_link";

        // If force exceeds 25N exit the loop
        if (abs(m_currWrench.wrench.force.z) >= 25.0) {
            isAppRunning = false;
            return;
        }

        // ROS_INFO("CurrWrench Z: %f, PID response: %f", m_currWrench.wrench.force.z, response);

        
        m_pubTargetWrench.publish(m_targetWrench);
        m_pubTargetFrame.publish(m_targetFrame);
        j += 0.00025;

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