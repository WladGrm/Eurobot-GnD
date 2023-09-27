#!/usr/bin/env python
# -*- coding: utf- 8 -*-

import rospy
import time
import os
from threading import Timer

from std_msgs.msg import Bool

def start(data):
    	if data.data == False:
		rospy.loginfo('Чека выдернута - Поехали!')
		t = time.time()
		timer()
		delo()

def timer():
	t = Timer(95.0, stop)
	t.start()

def stop():
	rospy.loginfo('Время вышло, останавливаюсь!')
	#os.system("pkill -u odroid")

def delo():
	rospy.loginfo('Начинаю делать дело')
	time.sleep(10)
	rospy.loginfo('Сделал дело')

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("pushed", Bool, start)
    rospy.spin()

if __name__ == '__main__':
    listener()
