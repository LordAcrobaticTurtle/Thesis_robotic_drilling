#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/PoseStamped.h>
namespace controller_ns {


    class MyController : public controller_interface::Controller<hardware_interface::PositionJointInterface> 
    {
        public:
            bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &nh) 
            {
                // Get the name of the joint from the parameter server
                std::string myJoint;
                if (nh.getParam("joint", myJoint)) {
                    ROS_ERROR("Could not find specified joint");
                    return false;
                }
                // Get the joint to use in the real time loop
                m_joint = hw->getHandle(myJoint);
                if (nh.getParam("gain", gain)) {
                    ROS_ERROR("Could not find gain parameter");
                    return false;
                }

                m_setpointCommand = nh.subscribe<geometry_msgs::PoseStamped>("controller_target", )
                return true;
            }

            void commandCallback(geometry_msgs::PoseStamped &msg) {
                target = msg;
            }

            void update(const ros::Time &time , const ros::Duration &period) {
                double error = setpoint - m_joint.getPosition();
                m_joint.setCommand(error*gain);
            }

            void starting() {};
            void stopping() {};


        private:

            hardware_interface::JointHandle m_joint;
            geometry_msgs::PoseStamped target;
            double m_setpoint = 0;
            double m_gain = 2;
            ros::Subscriber m_setpointCommand;
    };

    PLUGINLIB_DECLARE_CLASS(package_name, MyController, controller_ns::MyController, controller_interface::ControllerBase);
}

