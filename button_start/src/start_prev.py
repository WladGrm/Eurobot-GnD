#!/usr/bin/python3
# -*- coding: utf- 8 -*-
import rospy
import actionlib
import os
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from threading import Timer
from math import radians, degrees
from std_msgs.msg import Bool
from std_msgs.msg import UInt16MultiArray
from geometry_msgs.msg import Twist

def start(msg):
	if msg.data == False:
		rospy.loginfo("Чека выдернута - Поехали!")
		timer()
		delo0()
		time.sleep(2)
		delo1()
		time.sleep(2)
		delo2()
		time.sleep(2)
		delo3()
		time.sleep(2)
		delo4()
		time.sleep(2)
		delo5()
		rospy.loginfo("Кайфуем!")
		stop()


#def start(msg):
#	if msg.data == False:
#		rospy.loginfo("Чека выдернута - Поехали!")
#		timer()
#		moveCheck = poehali("pose2")
#		if moveCheck == True:
#			deloCheck = delo0()
#			if deloCheck == True:
#				moveCheck = poehali("start")
#				if moveCheck == True:
#					moveCheck = poehali("pose2")
#					if moveCheck == True:
#						rospy.loginfo("Zaebis!")
#						stop()

def timer():
	t = Timer(95.0, stop)
	t.start()

def stop():
	rospy.loginfo('Time is out! Killing all ros nodes!')
	os.system("rosnode kill -a")

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

#Схватываем Первые кейки
def delo0():
	rospy.loginfo('Начинаю делать дело')
	angle_0 = UInt16MultiArray()
	angle_0.data = [45, 160, 10, 140, 130, 65, 512, 512, 512]
	rospy.Publisher("servo", UInt16MultiArray,queue_size=1).publish(angle_0)
	time.sleep(5)
	rospy.loginfo('Сделал дело')
	return True
#angle_00 = [45, 160, 10, 140, 15, 170, 512, 512, 512]
#Схватываем Вторые кейки
def delo1():
	rospy.loginfo('Начинаю делать дело')
	angle_1 = [45, 40, 140, 140, 15, 170, 512, 512, 512]
	servo_pub.publish(angle_1)
	time.sleep(5)
	rospy.loginfo('Сделал дело')
	return True

#Схватываем Третьи кейки
def delo2():
	rospy.loginfo('Начинаю делать дело')
	angle_2 = [150, 160, 10, 20, 15, 170, 512, 512, 512]
	servo_pub.publish(angle_2)
	time.sleep(5)
	rospy.loginfo('Сделал дело')
	return True

#Выкладываем 1 вишни и отпускаем
def delo3():
	rospy.loginfo('Начинаю делать дело')
	angle_3 = [45, 160, 10, 140, 15, 170, 512, 512, 612]
	servo_pub.publish(angle_3)
	time.sleep(5)
	rospy.loginfo('Сделал дело')
	return True

#Выкладываем 2 вишни и отпускаем
def delo4():
	rospy.loginfo('Начинаю делать дело')
	angle_4 = [45, 160, 10, 140, 15, 170, 612, 512, 612]
	servo_pub.publish(angle_4)
	time.sleep(5)
	rospy.loginfo('Сделал дело')
	return True

#Выкладываем 3 вишни и отпускаем
def delo5():
	rospy.loginfo('Начинаю делать дело')
	angle_5 = [45, 160, 10, 140, 15, 170, 612, 612, 612]
	servo_pub.publish(angle_5)
	time.sleep(5)
	rospy.loginfo('Сделал дело')
	return True



	
def start_listener():
    rospy.init_node('start_listener', anonymous=True)
    rospy.Subscriber("start", Bool, start)
    rospy.spin()

if __name__ == '__main__':
    start_listener()
    #Публикуем в cmd_vel
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)
    servo_pub = rospy.Publisher("servo", UInt16MultiArray,queue_size=1)
    rate = rospy.Rate(5)
    move = Twist()
    #Публикуем углы для сервоприводов
    angle_0 = UInt16MultiArray()

