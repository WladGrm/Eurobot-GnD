#!/usr/bin/env python
# -*- coding: utf- 8 -*-
import rospy
import actionlib
import time
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from threading import Timer
from math import radians, degrees
from std_msgs.msg import Bool


def start(data):
	if data.data == False:
		rospy.loginfo("Чека выдернута - Поехали!")
		timer()
		moveCheck = poehali("pose2")
		if moveCheck == True:
			deloCheck = delo()
			if deloCheck == True:
				moveCheck = poehali("start")
				if moveCheck == True:
					moveCheck = poehali("pose2")
					if moveCheck == True:
						rospy.loginfo("Zaebis!")
						stop()

def timer():
	t = Timer(95.0, stop)
	t.start()

def stop():
	rospy.loginfo('Время вышло, останавливаюсь!')
	#rospy.sleep()
	#os.system("pkill -u odroid")

def moveToGoal(xGoal, yGoal):
	#Svaz s move_base серверom используя SimpleActionClient
	ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

	#Ожидание экшн сервера
	while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
		rospy.loginfo("Waiting for the move_base action server to come up")

	goal = MoveBaseGoal()

	#Устанавливаю параметры фреймов
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	#Отправляю приказ о езде к точке
	goal.target_pose.pose.position = Point(xGoal, yGoal, 0)
	goal.target_pose.pose.orientation.x = 0.0
	goal.target_pose.pose.orientation.y = 0.0
	goal.target_pose.pose.orientation.z = 0.0
	goal.target_pose.pose.orientation.w = 1.0

	rospy.loginfo('Sending goal location...')
	ac.send_goal(goal)

	ac.wait_for_result(rospy.Duration(200))

	if(ac.get_state() == GoalStatus.SUCCEEDED):
		rospy.loginfo('Приехали!')
		return True
	else:
		rospy.loginfo('Suka Blat!!!')
		return False

def poehali(kyda):
	#Объявляю точки
	xStart = 0.0
	yStart = 0.0
	xPose1 = 1.0
	yPose1 = 0.2
	xPose2 = 1.2
	yPose2 = 0.0
	gr = False
	if kyda == "start":
		gr = moveToGoal(xStart, yStart)
	elif kyda == "pose1":
		gr = moveToGoal(xPose1, yPose1)
	elif kyda == "pose2":
		gr = moveToGoal(xPose2, yPose2)

	return gr

def delo():
	rospy.loginfo('Начинаю делать дело')
	time.sleep(5)
	rospy.loginfo('Сделал дело')
	return True

#def emergency_cb(data):
#	if data == True:


	

def start_listener():
    rospy.init_node('start_listener', anonymous=True)
    rospy.Subscriber("start", Bool, start)
    rospy.spin()

#def emergency_listener():
#	rospy.init_node('emergency_listener',anonymous=True)
#	rospy.Subscriber("emergency", Bool, emergency_cb)
#	rospy.spin

if __name__ == '__main__':
    start_listener()
    
    
