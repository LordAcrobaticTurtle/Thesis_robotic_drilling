#!/usr/bin/env python
from lib2to3.pytree import convert
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

joystick = Joy()
target = PoseStamped()

def joy_callback(data):
    global joystick
    joystick = data

def convert_value(value):
    return -0.5*(value-1)+1
    
def main():
    print("START")
    rospy.init_node('joy_to_geometry_node',anonymous=True)
    geometry_pub = rospy.Publisher("/my_cartesian_motion_controller/target_frame", PoseStamped, queue_size=10)
    joy_sub = rospy.Subscriber("/joy",Joy, joy_callback)
    rate = rospy.Rate(50)
    target.header.frame_id = "base_link"
    scale = 0.01    
    target.pose.position.x = 0.19
    target.pose.position.y = 0.518
    target.pose.position.z = 0.4
    

    pitch_angle = 0
    roll_angle = 0
    yaw_angle = 0
    while not rospy.is_shutdown():
        # print(scale*joystick.axes)
        if len(joystick.axes) > 0:
            controls = [scale*i for i in joystick.axes]
            print(controls)
            print(target)
            target.pose.position.x += controls[0]
            target.pose.position.y += controls[1]
            target.pose.position.z += convert_value(controls[2])
            target.pose.position.z -= convert_value(controls[5])
            pitch_angle += controls[3]
            roll_angle += controls[4]
            yaw_angle += controls[6]
            angle = quaternion_from_euler(roll_angle,pitch_angle,yaw_angle)
            target.pose.orientation.x = angle[0]
            target.pose.orientation.y = angle[1]
            target.pose.orientation.z = angle[2]
            target.pose.orientation.w = angle[3]
            geometry_pub.publish(target)
        rate.sleep()



if __name__ == '__main__':
    main()