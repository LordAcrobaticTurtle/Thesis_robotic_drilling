#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>

// Find a way to set dynamic reconfigure up inside this program. Or inside the launch file would be even better

namespace thesis {
    class complianceMovementController {
        public:
            enum state {
                ZERO,
                GENERAL   
            };
            complianceMovementController(ros::NodeHandle &p_nh);
            complianceMovementController(ros::NodeHandle &p_nh, state p_state);
            ~complianceMovementController();
            void setMode(state p_state);

        private:
            // Nodehandle
            ros::NodeHandle m_nh;
            state m_state;
            // Publisher to target frame
            ros::Publisher m_pubTargetFrame;
            // Publisher to target wrench
            ros::Publisher m_pubTargetWrench;
            
            // How to decouple 

            // Wrench callback + data
            ros::Subscriber m_subWrench;
            void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& data);
            geometry_msgs::WrenchStamped::ConstPtr msgWrenchStamped;

            void main();
            
    };
    typedef complianceMovementController cmc;
};


