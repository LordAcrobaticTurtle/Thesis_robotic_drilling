#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <custom_control/complianceMovementController.h>
#include <yaml-cpp/yaml.h>
#include <cmath>

namespace thesis {
    complianceMovementController::complianceMovementController(ros::NodeHandle &p_nh) : m_joyHandle(p_nh) {
        ROS_INFO("CMC constructed");
        // Construct the transform and let it gather tf frames
        tf::StampedTransform transform;
        m_isAppRunning = true;
        m_nh = p_nh;
        std::string controllerName = "/my_cartesian_compliance_controller/";
        m_pubTargetFrame = m_nh.advertise<geometry_msgs::PoseStamped>(controllerName + "target_frame",1000);
        m_pubTargetWrench = m_nh.advertise<geometry_msgs::WrenchStamped>(controllerName + "target_wrench",1000);
        m_subWrench = m_nh.subscribe<geometry_msgs::WrenchStamped>
                    ("/wrench",1000,&complianceMovementController::wrenchCallback,this);

        m_targetFrame.header.frame_id = "base_link";
        m_targetWrench.header.frame_id = "base_link";


        // Fill parameters using config file
        YAML::Node config = YAML::LoadFile("/home/mtrn4230/catkin_ws/src/Thesis_robotic_drilling/custom_control/config/config.yaml");
    // They should be on parameter server

        m_targetDepth = config["targetDepth"].as<double>()/1000;
        m_depthStep = config["depthStep"].as<float>()/1000;
        if (m_depthStep > m_targetDepth) {
            ROS_ERROR("Depth step must be smaller than target depth. Halting");
            while (1);
        }
        m_usePeaking = config["usePeaking"].as<bool>();
        m_stiffnessX = config["stiffnessX"].as<int>();
        m_stiffnessY = config["stiffnessY"].as<int>();
        m_stiffnessZ = config["stiffnessZ"].as<int>();
        m_endEffectorLinkName = config["endEffectorLinkName"].as<std::string>();
        m_baseLinkName = config["baseLinkName"].as<std::string>();
        m_feedRate = config["feedRate"].as<float>() / 1000;
        m_maxDrillForce = config["maxDrillForce"].as<float>();
        m_maxHomeForce = config["maxHomeForce"].as<float>();
        m_drillBitWidth = config["drillBitWidth"].as<float>()/1000;
        m_tolerance = config["tolerance"].as<float>() / 1000;
        m_homeX = config["m_homeX"].as<float>();
        m_homeY = config["m_homeY"].as<float>();
        m_homeZ = config["m_homeZ"].as<float>();
        m_homeqW = config["m_homeqW"].as<float>();
        m_homeqX = config["m_homeqX"].as<float>();
        m_homeqY = config["m_homeqY"].as<float>();
        m_homeqZ = config["m_homeqZ"].as<float>();
        m_kp = config["m_kp"].as<double>();
        m_kd = config["m_kd"].as<double>();
        m_ki = config["m_ki"].as<double>();
        m_PIDmax = config["m_PIDmax"].as<double>();
        m_PIDmin = -1*m_PIDmax;
        m_iMax = config["m_iMax"].as<double>();
        m_iMin = -m_iMax;
        std::string wpMethod = config["wpMethod"].as<std::string>();
        bool useProgrammedHome = config["useProgrammedHome"].as<bool>();
        
        // Delay for a second so the tf tree can update.
        ros::Rate delay(0.5);
        // Setup m_currPose to listen to tf 
        m_poseTimer = m_nh.createTimer(ros::Duration(1/m_rateHz), &cmc::tfposCallback,this);
        delay.sleep();
        for (int i = 0; i < 20; i++) {
            try {
                m_listener.lookupTransform(m_baseLinkName.c_str(), m_endEffectorLinkName.c_str(), ros::Time(0), transform);
                m_home.pose.position.x = transform.getOrigin().x();
                m_home.pose.position.y = transform.getOrigin().y();
                m_home.pose.position.z = transform.getOrigin().z();
                m_home.pose.orientation.w = transform.getRotation().w();
                m_home.pose.orientation.x = transform.getRotation().x();
                m_home.pose.orientation.y = transform.getRotation().y();
                m_home.pose.orientation.z = transform.getRotation().z();
                ROS_INFO("Successful set from TF tree");
                break;
            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
                programmedHome();
            }
        }
        ROS_INFO("m_currPose");
        std::cout << m_currPose << std::endl;

        // If boolean is set to true in config file, overwrite m_home position
        if (useProgrammedHome) 
            programmedHome();
            

        ROS_INFO("m_home: ");
        std::cout << m_home << std::endl;
        // ADJUST HOME POSITION
        // Grab home position from end effector current pose in the TF tree.
        
        m_home.header.frame_id = m_baseLinkName;
        m_currPose.header.frame_id = m_baseLinkName;
        m_currWrench.header.frame_id = m_baseLinkName;
        m_currWrench.wrench.force.x = 0;
        m_currWrench.wrench.force.y = 0;
        m_currWrench.wrench.force.z = 0;
        m_currWrench.wrench.torque.x = 0;
        m_currWrench.wrench.torque.y = 0;
        m_currWrench.wrench.torque.z = 0;

        // Conduct homing sequence
        homing();
        if (wpMethod.compare("step") == 0) {
            ROS_WARN("STEP");
            computeStepWaypoints(m_targetDepth, m_drillBitWidth);
        } else if (wpMethod.compare("peck") == 0) {
            ROS_WARN("PECK");
            computePeckWaypoints(m_targetDepth, m_drillBitWidth);
        } else {
            ROS_ERROR("YOU MUST SPECIFY EITHER STEP OR PECK");
            while(ros::ok());
        }
        ROS_INFO("Activate drill now");
        sleep(5);
        
        // After homing sequence returns, start the main loop
        m_timer = m_nh.createTimer(ros::Duration(1/m_rateHz), &cmc::main,this);
    };

    complianceMovementController::complianceMovementController(ros::NodeHandle &p_nh, bool p_home) : 
    complianceMovementController(p_nh) 
    {
        if (p_home) {
            m_pubTargetFrame.publish(m_home);
            while(ros::ok());
        }
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
        // static double target = m_currPose.pose.position.z - m_targetDepth;
        
        if (!m_isAppRunning) {
            ROS_INFO("SHUTDOWN - CMC main - Returning to home:");
            std::cout << m_targetFrame << std::endl;
            m_pubTargetFrame.publish(m_home);
            return;
        }
    
        updateWaypoints();
        m_targetWrench.header.stamp = ros::Time::now();
        m_targetWrench.header.frame_id = "base_link";
        if (abs(m_currWrench.wrench.force.z) >= m_maxDrillForce) {
            m_isAppRunning = false;
            return;
        }
        
        // Change Force applied using PID. 
        double response = PID(m_targetFrame.pose.position.z, m_currPose.pose.position.z);
        m_targetWrench.wrench.force.z = -response;
        // ROS_INFO("CurrWrench Z: %f, PID response: %f, currpos: %f, targetpos: %f", m_currWrench.wrench.force.z, m_targetWrench.wrench.force.z, m_currPose.pose.position.z, m_targetFrame.pose.position.z);
        // Can estimate currPose based on targetFrame and force + stiffness values
        
        m_pubTargetWrench.publish(m_targetWrench);
        m_pubTargetFrame.publish(m_targetFrame);
        m_currPose = m_targetFrame;
        j += 0.00025;
    }

    void cmc::updateWaypoints() {
        // 
        static int waypoint_index = 0;
        static int timerCounter = 0;
        static float time = -1;
        static bool hasTimerReset = false;
        
        // Start timer when within tolerance of target position.
        // When updating waypoint, set m_targetFrame + increment i. 
        if (waypoint_index == m_waypoints.size()) {
            // ROS_WARN("WAYPOINTS COMPLETE");
            return;
        }
        ROS_INFO("Current position: %f", m_currPose.pose.position.z);
        ROS_INFO("Current target: %f", m_targetFrame.pose.position.z);
        // Check if we have reached the current waypoint within tolerance
        
        // Check if timer is finished. If true update waypoint
        if (timerCounter == (int) time) {
            ROS_WARN("depth target update: %f", std::get<0>(m_waypoints[waypoint_index]));
            m_targetFrame.pose.position.z = std::get<0>(m_waypoints[waypoint_index]);
            waypoint_index++;
            hasTimerReset = false;
        // Else Check if we are in position
        }
        // Need to do this once per waypoint.
        if ((m_targetFrame.pose.position.z - m_tolerance <= m_currPose.pose.position.z) && (m_currPose.pose.position.z <= m_targetFrame.pose.position.z + m_tolerance) && !hasTimerReset) {
            ROS_WARN("Timer restart");
            time = std::get<1>(m_waypoints[waypoint_index]);
            // If true start timer/counter up to the number supplied array
            timerCounter = 0;
            
            hasTimerReset = true;
        }
        // If false wait for system to move into position.
        timerCounter++;
    }

    void cmc::homing() {
        // Drive until spike in force.
        // Measure force in each direction and change setpoint to be the difference using spring relationship

        // Hacking the mainframe. Read forces in Y, command positions in Z.

        static bool isHomeFound = false;
        ros::Rate rate(100);
        float j = 0;
        geometry_msgs::WrenchStamped spikeMeasurements;
        geometry_msgs::Pose deviation;
        
        // Move to the home position and wait for the arm to get there.
        m_pubTargetFrame.publish(m_home);
        for (int i = 0; i < 100; i++) 
            rate.sleep();
        
        while (ros::ok()) {
            // Assumption, starting at UR5e home position.
            // Slowly move EE down
            m_targetFrame = m_home;
            m_targetFrame.pose.position.z = -0.5*sin(j)+m_home.pose.position.z;
            m_targetWrench.wrench.force.z = 0;
            m_targetWrench.wrench.force.y = 0;
            m_targetWrench.wrench.force.x = 0;

            ROS_INFO("m_targetFrame z: %f", m_targetFrame.pose.position.z);
            
            if (abs(m_currWrench.wrench.force.z) >= m_maxHomeForce) {
                // Take measurements of each force + torque axis
                spikeMeasurements = m_currWrench;
                break;
            }
            m_pubTargetFrame.publish(m_targetFrame);
            rate.sleep();
            j += 0.00025;
        }
        
        deviation.position.z = spikeMeasurements.wrench.force.z/m_stiffnessZ;
        // COMPARE currPose with deviation
        ROS_INFO("DEVIATION");
        std::cout << deviation << std::endl;
        ROS_INFO("currPose");
        std::cout << m_currPose << std::endl;
        ROS_INFO("TARGET_FRAME");
        std::cout << m_targetFrame << std::endl;
        // Convert TCP frame to base frame
         // Retrieve current EE pose
        m_posAfterHoming = m_currPose;
        m_pubTargetFrame.publish(m_currPose);
    }

// STATIC VALUES MEAN YOU CAN ONLY USE THIS ONE A SINGLE AXIS
    double cmc::PID(double setpoint, double currValue) {
        static double prevError = 0;
        static double iSum = 0;
        double dt = 1.0/m_rateHz;

        // Calculate quantities
        double perror = setpoint - currValue;
        double derror = (perror-prevError) / dt;
        iSum += perror*dt;

        if (iSum > m_iMax)
            iSum = m_iMax;
        else if (iSum < m_iMin) 
            iSum = m_iMin;
        
        // Multipy by gains
        double pOut = m_kp*perror;
        double dOut = m_kd*derror;
        double iOut = m_ki*iSum;

        // Calculate result
        double result = pOut + iOut;

        // Prevent windup
        if (result > m_PIDmax) 
            result = m_PIDmax;
        else if (result < m_PIDmin)
            result = m_PIDmin;
        
        // Set error for next loop
        prevError = perror;

        return result;
    }
    void cmc::computeStepWaypoints(float p_targetDepth, float p_drillBitWidth) {
        // Define a "feedrate" mm/s
        // constraints, no guarantee that thearm will reach the setpoint. 
        // Hopefully, the "updateWaypoints" function will handle.

        // Calculate depth of step as user supplied
        // How many steps can be completed in using depthStep?
        float numSteps = p_targetDepth/m_depthStep; 
        float positionAfterDrilling = m_posAfterHoming.pose.position.z - p_targetDepth;
        numSteps = floor(numSteps);
        // Add waypoints of numsteps
        for (int i = 1; i <= numSteps ; i++) 
            m_waypoints.push_back(std::tuple<float ,float>(m_posAfterHoming.pose.position.z - i*m_depthStep, 1.0));
        
        // Add the final point.
        m_waypoints.push_back(std::tuple<float, float>(positionAfterDrilling, 1));
        // Rise above material once complete.
        m_waypoints.push_back(std::tuple<float, float>(m_posAfterHoming.pose.position.z+0.02, 1));
        for (auto i = 0; i < m_waypoints.size(); i++) 
            std::cout << std::get<0>(m_waypoints[i]) << std::endl; 

    }

    // Written with parameters so that I could, in a years time lol, actually do some unit testing
    void cmc::computePeckWaypoints(float p_targetDepth, float p_drillBitWidth) {
        // Define a "feedrate" mm/s
        // constraints, no guarantee that thearm will reach the setpoint. 
        // Hopefully, the "updateWaypoints" function will handle.

        // Calculate depth of step as user supplied
        // How many steps can be completed in using depthStep?
        float numSteps = p_targetDepth/m_depthStep; 
        float positionAfterDrilling = m_posAfterHoming.pose.position.z - p_targetDepth;
        numSteps = floor(numSteps);
        // Add waypoints of numsteps
        for (int i = 1; i <= numSteps ; i++) {
            m_waypoints.push_back(std::tuple<float ,float>(m_posAfterHoming.pose.position.z - i*m_depthStep, 1.0));
            m_waypoints.push_back(std::tuple<float ,float>(m_posAfterHoming.pose.position.z +0.005, 1.0));
        }
        
        // Add the final point.
        m_waypoints.push_back(std::tuple<float, float>(positionAfterDrilling, 1));
        // Rise above material once complete.
        m_waypoints.push_back(std::tuple<float, float>(m_posAfterHoming.pose.position.z+0.02, 1));
        for (auto i = 0; i < m_waypoints.size(); i++) 
            std::cout << std::get<0>(m_waypoints[i]) << std::endl; 
    }

    void cmc::programmedHome() {
        m_home.pose.position.x = m_homeX;
        m_home.pose.position.y = m_homeY;
        m_home.pose.position.z = m_homeZ;
        m_home.pose.orientation.x = m_homeqX;
        m_home.pose.orientation.y = m_homeqY;
        m_home.pose.orientation.z = m_homeqZ;
        m_home.pose.orientation.w = m_homeqW;
    }

    void cmc::tfposCallback(const ros::TimerEvent &t) {
        // Test this function works
        tf::StampedTransform trf;
        try {
            m_listener.lookupTransform(m_baseLinkName.c_str(), m_endEffectorLinkName.c_str(),ros::Time(0), trf);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }

        m_currPose.pose.position.x = trf.getOrigin().x();
        m_currPose.pose.position.y = trf.getOrigin().y();
        m_currPose.pose.position.z = trf.getOrigin().z();
        m_currPose.pose.orientation.w = trf.getRotation().w();
        m_currPose.pose.orientation.x = trf.getRotation().x();
        m_currPose.pose.orientation.y = trf.getRotation().y();
        m_currPose.pose.orientation.z = trf.getRotation().z();
        // std::cout << m_currPose << std::endl;
        // ROS_INFO("m_currPose updated");
    }
    
}