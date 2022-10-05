#include <custom_control/complianceMovementController.h>
#include <cmath>
namespace thesis {
    complianceMovementController::complianceMovementController(ros::NodeHandle &p_nh) :
        rate(100)
     {
        ROS_INFO("CMC constructed");
        m_nh = p_nh;
        std::string controllerName = "/my_cartesian_compliance_controller/";
        m_pubTargetFrame = m_nh.advertise<geometry_msgs::PoseStamped>(controllerName + "target_frame",1000);
        m_pubTargetWrench = m_nh.advertise<geometry_msgs::WrenchStamped>(controllerName + "target_wrench",1000);
        m_subWrench = m_nh.subscribe<geometry_msgs::WrenchStamped>("/wrench",1000,&cmc::wrenchCallback,this);

        m_targetFrame.header.frame_id = "base_link";
        m_targetWrench.header.frame_id = "base_link";
        
        m_home.pose.position.x = 0.459;
        m_home.pose.position.y = 0.200;
        m_home.pose.position.z = 0.300;

        m_currPose.header.frame_id = "base_link";
        m_currPose = m_home;

        m_currWrench.header.frame_id = "base_link";
        m_currWrench.wrench.force.x = 0;
        m_currWrench.wrench.force.y = 0;
        m_currWrench.wrench.force.z = 0;
        m_currWrench.wrench.torque.x = 0;
        m_currWrench.wrench.torque.y = 0;
        m_currWrench.wrench.torque.z = 0;

        // Jump into main loop
        main();
    };

    // complianceMovementController::complianceMovementController(ros::NodeHandle &p_nh, cmc::state p_state) {
    //     setMode(p_state);
    //     complianceMovementController(p_nh);
    // }

    complianceMovementController::~complianceMovementController() {
        ROS_INFO("CMC destroyed");
    };
    
    void cmc::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& data) {
        msgWrenchStamped = data;
    }

    void cmc::setMode(state p_state) {
        m_state = p_state;
        ROS_INFO("New mode: %d", m_state);
    }


    void cmc::main() {
        int i = 0;
        double j = 0.0;
        // All commands must be passed in as metres
        while (ros::ok()) {
            ROS_INFO("sin(j)= %f", sin(j));
            // m_pubTargetFrame.publish(m_targetFrame);
            // m_pubTargetWrench.publish(m_targetWrench);
            j += 0.05;
            rate.sleep();
        }
    }





    
}