#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>

// Find a way to set dynamic reconfigure up inside this program. Or inside the launch file would be even better

namespace thesis {
    class complianceMovementController {
        public:
            complianceMovementController();
            ~complianceMovementController();

            void setMode(/*ENUM INDICATING STATE*/)

        private:
            // Publisher to target frame
            ros::Publisher pubTargetFrame;
            // Publisher to target wrench
            ros::Publisher pubTargetWrench;
            // Subscriber to wrench
            ros::Subscriber subWrench;

            
    };
    typedef complianceMovementController cmc;
};


