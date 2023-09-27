#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

def callback(data):
    rospy.loginfo("Published: " + str(data.data))

def listener():
    rospy.init_node('start_listener', anonymous=True)
    rospy.Subscriber("start", Bool, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()    