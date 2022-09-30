#include <custom_control/complianceMovementController.h>

namespace thesis {
    complianceMovementController::complianceMovementController(ros::NodeHandle &p_nh) {
        ROS_INFO("CMC constructed");
        m_nh = p_nh;
        std::string controllerName = "/my_cartesian_compliance_controller/";
        m_pubTargetFrame = m_nh.advertise<geometry_msgs::PoseStamped>(controllerName + "target_frame",1000);
        m_pubTargetWrench = m_nh.advertise<geometry_msgs::WrenchStamped>(controllerName + "target_wrench",1000);
        m_subWrench = m_nh.subscribe<geometry_msgs::WrenchStamped>("/wrench",1000,&cmc::wrenchCallback,this);

        m_targetFrame.header.frame_id = "base_link";
        m_targetWrench.header.frame_id = "base_link";

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
        while (ros::ok()) {
            // Set position commands see what happens
            geometry_msgs::Point targetPoint;
            targetPoint.x = 0;
            targetPoint.y = 0;
            targetPoint.z = 0;
            
            m_targetFrame.pose.position = targetPoint;
            ROS_INFO("Publsihing target frame");
            m_pubTargetFrame.publish(m_targetFrame);

            m_targetWrench.wrench.force.z = 0;
            m_targetWrench.wrench.force.x = 1;
            m_targetWrench.wrench.force.y = 0;

            m_pubTargetWrench.publish(m_targetWrench);
        }
    }





    
}