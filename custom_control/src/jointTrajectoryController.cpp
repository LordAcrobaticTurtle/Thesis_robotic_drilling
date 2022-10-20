#include "custom_control/jointTrajectoryController.h"


namespace thesis {
    JTC::jointTrajectoryController() {}

    JTC::~jointTrajectoryController() {}

    JTC::jointTrajectoryController(ros::NodeHandle &p_nh) {
        m_nh = p_nh;
    }

    void JTC::main(const ros::TimerEvent &t) {
        compute();
        publish();
    }

    void JTC::compute() {}

    void JTC::publish() {}

}