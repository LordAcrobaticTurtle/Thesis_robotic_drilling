#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

ros::Publisher pub;
 ros::Subscriber sub;
// const std_msgs::String::ConstPtr&
void topicCallback(const geometry_msgs::WrenchStamped::ConstPtr& data) {
    geometry_msgs::WrenchStamped msg;
    msg.header = data->header;
    msg.wrench = data->wrench;
    pub.publish(msg);
    // ROS_INFO("SUBSCRIBER");
}



int main(int argc, char ** argv) {
    if (argc < 3) {
        std::cout << "usage: ./exe topicSub topicPub\n";
        return -1;
    }
    ros::init(argc, argv, "remapper");
    ros::NodeHandle nh;
    std::string topicSub = argv[1];
    std::string topicPub = argv[2];
    std::cout << "TopicSub: " << topicSub << ", topicPub: " << topicPub << std::endl;

    pub = nh.advertise<geometry_msgs::WrenchStamped>(topicPub, 1000);
    sub = nh.subscribe<geometry_msgs::WrenchStamped>(topicSub, 1000, topicCallback);

    ros::spin();


    return 0;



}