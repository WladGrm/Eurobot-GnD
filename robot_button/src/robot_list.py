#!/usr/bin/env python
# -*- coding: utf- 8 -*-

import rospy

from std_msgs.msg import Bool

def endf(data):
    	if data.data == False:
		rospy.loginfo('Кнопку нажал')
		#KOD HA 3AnyCK  

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("pushed", Bool, endf)
    rospy.spin()

if __name__ == '__main__':
    listener()