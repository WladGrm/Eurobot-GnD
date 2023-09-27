#! /usr/bin/env python
import roslib
import rospy

from std_msgs.msg import Bool
from Button_msgs.msg import pushed_msg

def endf(data):
	if data.pushed_msg == True:
		rospy.loginfo('Pushed')
		#Kod HA Bblklu4eHie

def listener():
	#pushed_msg = Bool
	rospy.Subscriber('pushed', pushed_msg, endf)
	rospy.spin()

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('robot_button', anonymous = True)
    # Go to the main loop.
    rospy.loginfo('Hello')
    listener()