#!/usr/bin/env python
from asyncio.proactor_events import _ProactorSocketTransport
import rospy
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from tf.transformations import quaternion_from_euler
# Conversion for axes only
def convert_value(value):
        return 0.5*(-value+1)

class converter:
    """ Convert Twist messages to WrenchStamped """
        
    def __init__(self):
        rospy.init_node('converter', anonymous=False)
        self.joy_topic = rospy.get_param('~joy_topic', default="/joy")
        self.pose_topic = rospy.get_param('~pose_topic',default="/target_frame")
        self.wrench_topic = rospy.get_param('~wrench_topic',default="/target_wrench")
        self.frame_id = rospy.get_param('~frame_id',default="world")
        self.rate = rospy.Rate(rospy.get_param('~publishing_rate',default=100))
        self.wrenchBuffer = WrenchStamped()
        self.poseBuffer = PoseStamped()
        self.offset = PoseStamped()
        self.currPose = PoseStamped()
        self.currWrench = WrenchStamped()
        self.currPose.header.frame_id = "base_link"
        self.currWrench.header.frame_id = "base_link"
            
        # Home as set in the installation tab on the UR5e
        self.offset.pose.position.x = 0.459
        self.offset.pose.position.y = 0.200
        self.offset.pose.position.z = 0.300
        self.joyBuffer = Joy()
        self.wrenchPub = rospy.Publisher(self.wrench_topic, WrenchStamped, queue_size=10)
        self.posePub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        self.joySub = rospy.Subscriber(self.joy_topic, Joy, self.joy_cb)
        print("CONVERTER INIT")

    def joy_cb(self,data):
        # All axes work except 
        timeNow = rospy.Time.now()
        if (data.buttons[4]):
            self.wrenchBuffer.header.stamp     = timeNow
            self.wrenchBuffer.header.frame_id  = "base_link"
            self.wrenchBuffer.wrench.force.x   = 5*data.axes[0]
            self.wrenchBuffer.wrench.force.y   = 5*data.axes[1]
            self.wrenchBuffer.wrench.force.z   = 5*(convert_value(data.axes[2]) - convert_value(data.axes[5]))
            self.wrenchBuffer.wrench.torque.x  = 5*data.axes[3]
            self.wrenchBuffer.wrench.torque.y  = 5*data.axes[4]
            print(self.wrenchBuffer)
        else:
            self.poseBuffer.header.frame_id = "base_link"
            self.poseBuffer.header.stamp = timeNow
            self.poseBuffer.pose.position.x = self.offset.pose.position.x + 0.1*data.axes[0]
            self.poseBuffer.pose.position.y = self.offset.pose.position.y - 0.1*data.axes[1]
            self.poseBuffer.pose.position.z = self.offset.pose.position.z + 0.1*(convert_value(data.axes[2]) - convert_value(data.axes[5]))
            quat = quaternion_from_euler(data.axes[3], data.axes[4],0)
            print(quat)
            self.poseBuffer.pose.orientation.x = data.axes[3]
            self.poseBuffer.pose.orientation.y = data.axes[4]
            self.poseBuffer.pose.orientation.z = 0
            self.poseBuffer.pose.orientation.w = -1
            
            # print(self.poseBuffer)
        # print(self.buffer)
        # self.buffer.wrench.torque.z  = data.angular.z


    
    def publish(self):
        if not rospy.is_shutdown():
            try:
                self.wrenchPub.publish(self.wrenchBuffer)
                self.posePub.publish(self.poseBuffer)
            except rospy.ROSException:
                # Swallow 'publish() to closed topic' error.
                # This rarely happens on killing this node.
                pass
           
if __name__ == '__main__':
    conv = converter()
    try:
        while not rospy.is_shutdown():
            conv.publish()
            conv.rate.sleep()
    except rospy.ROSInterruptException:
        pass
