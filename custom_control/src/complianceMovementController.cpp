#include <cmath>

#include <custom_control/complianceMovementController.h>


namespace thesis {
    complianceMovementController::complianceMovementController(ros::NodeHandle &p_nh) : m_joyHandle(p_nh) {
        ROS_INFO("CMC constructed");
        m_nh = p_nh;
        std::string controllerName = "/my_cartesian_compliance_controller/";
        m_pubTargetFrame = m_nh.advertise<geometry_msgs::PoseStamped>(controllerName + "target_frame",1000);
        m_pubTargetWrench = m_nh.advertise<geometry_msgs::WrenchStamped>(controllerName + "target_wrench",1000);
        m_subWrench = m_nh.subscribe<geometry_msgs::WrenchStamped>
                    ("/wrench",1000,&complianceMovementController::wrenchCallback,this);

        m_targetFrame.header.frame_id = "base_link";
        m_targetWrench.header.frame_id = "base_link";
        
        // ADJUST HOME POSITION
        m_home.pose.position.x = 0.487;
        m_home.pose.position.y = 0.348;
        m_home.pose.position.z = 0.239;
        
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

        // Conduct homing sequence
        homing();
        // After homing sequence returns, start the main loop
        m_timer = m_nh.createTimer(ros::Duration(1/m_rateHz), &cmc::main,this);
    };

    complianceMovementController::complianceMovementController(ros::NodeHandle &p_nh, float p_depth) : 
    complianceMovementController(p_nh) 
    {
        m_targetDepth = p_depth;
    }

    complianceMovementController::~complianceMovementController() {
        ROS_INFO("CMC destroyed");
    };
    
    void complianceMovementController::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& data) {
        m_currWrenchMsg = data;
        m_currWrench.header = m_currWrenchMsg->header;
        m_currWrench.wrench = m_currWrenchMsg->wrench;
        // Convert TCP forces to base frame 
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
            ROS_INFO("SHUTDOWN - CMC main - Returning to home:");
            std::cout << m_targetFrame << std::endl;
            m_pubTargetFrame.publish(m_home);
            return;
        }
        m_targetFrame = m_home;
        double target = 0;
        // Calculate response and setup wrench data struct
        // double response = PID(target,m_currWrench.wrench.force.z);
        
        m_targetWrench.wrench.force.z = 0;
        m_targetWrench.header.stamp = ros::Time::now();
        m_targetWrench.header.frame_id = "base_link";
        
        m_targetFrame.pose.position.z = m_targetFrame.pose.position.z - m_targetDepth;
        
        // If force exceeds 25N exit the loop
        if (abs(m_currWrench.wrench.force.z) >= 25.0) {
            isAppRunning = false;
            return;
        }

        ROS_INFO("CurrWrench Z: %f, PID response: %f", m_currWrench.wrench.force.z, 0.0);
        // Can estimate currPose based on targetFrame and force + stiffness values
        
        m_pubTargetWrench.publish(m_targetWrench);
        m_pubTargetFrame.publish(m_targetFrame);
        j += 0.00025;

        // ROS_INFO("EXITING - CMC main");
    }

    void cmc::homing() {
        // Drive until spike in force.
        // Measure force in each direction and change setpoint to be the difference using spring relationship
        static bool isHomeFound = false;
        const float stiffnessX = 2500.0;
        const float stiffnessY = 2500.0;
        const float stiffnessZ = 500.0;
        ros::Rate rate(100);
        float j = 0;
        geometry_msgs::WrenchStamped spikeMeasurements;
        geometry_msgs::Pose deviation;
        while (ros::ok()) {
            // Assumption, starting at UR5e home position.

            // Slowly move EE down
            m_targetFrame = m_home;
            m_targetFrame.pose.position.z = -0.5*sin(j)+m_home.pose.position.z;
            ROS_INFO("m_targetFrame z: %f", m_targetFrame.pose.position.z);
            
            if (abs(m_currWrench.wrench.force.z) >= 10.0) {
                // Take measurements of each force + torque axis
                spikeMeasurements = m_currWrench;
                break;
            }
            // Publish pos target
            // m_pubTargetWrench.publish(m_targetWrench);
            m_pubTargetFrame.publish(m_targetFrame);
            
            // Sleep loop
            rate.sleep();
            j += 0.00025;
        }
        
        // Calculate deviation
        deviation.position.x = m_currWrench.wrench.force.x/stiffnessX; 
        deviation.position.y = m_currWrench.wrench.force.y/stiffnessY; 
        deviation.position.z = m_currWrench.wrench.force.z/stiffnessZ; 
        std::cout << deviation << std::endl;
        // 
        // Convert TCP frame to base frame
        
        // m_targetFrame.pose.position.x -= deviation.position.y;
        // m_targetFrame.pose.position.y -= deviation.position.x;
        m_targetFrame.pose.position.z -= deviation.position.z;
        m_pubTargetFrame.publish(m_targetFrame);
        std::cout << m_targetFrame << std::endl;

        // Publish deviation to arm 
        while(ros::ok()) {};
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