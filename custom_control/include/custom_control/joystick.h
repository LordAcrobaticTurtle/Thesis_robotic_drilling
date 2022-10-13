#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/WrenchStamped.h>

namespace thesis {

    class joystick {
        public:
            joystick();
            joystick(ros::NodeHandle &p_nh);
            ~joystick();
            void publish();
            std::vector<float> getAxes();
            sensor_msgs::Joy m_joyState;
            
        private:

            ros::NodeHandle m_nh;
            ros::Publisher m_pubJoy;
            ros::Subscriber m_subJoy;
            void joyCallback(const sensor_msgs::Joy::ConstPtr& data);

            // Class functionality
            ros::Publisher m_pubWrench;
            void joyToSensorWrench();

    };

}