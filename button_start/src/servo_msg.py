#!/usr/bin/python3
import rospy
from std_msgs.msg import UInt16MultiArray


def talker():
    pub = rospy.Publisher('servo', UInt16MultiArray, queue_size=10)
    rospy.init_node('servo_publisher_node', anonymous=True)
    rate = rospy.Rate(5)
    rospy.loginfo("Top level servo node is up")
    while not rospy.is_shutdown():
        msg = UInt16MultiArray()
        a = [50,160,15,135,20,165,612,612,612]
        msg.data = a
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
