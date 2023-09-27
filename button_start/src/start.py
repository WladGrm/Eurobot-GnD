#!/usr/bin/python3
# -*- coding: utf- 8 -*-
import rospy
import actionlib
import os
import time
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from threading import Timer
from math import radians, degrees
from std_msgs.msg import Bool
from std_msgs.msg import UInt16MultiArray
from geometry_msgs.msg import Twist
#import numpy as np





def start_listener():
    servo_pub = rospy.Publisher('servo', UInt16MultiArray, queue_size=10)
    rospy.Subscriber("start", Bool, start)
    rospy.init_node('start_side1', anonymous=True)
    #rospy.init_node('servo_publisher_node', anonymous=True)
    rate = rospy.Rate(5)
    move = Twist()
    rospy.loginfo("Top level servo node is up")
    rospy.spin()

#def start(msg):
	#if msg.data == False:
		#rospy.loginfo("Чека выдернута - Поехали!")
		#timer()
		#delo0()
		#time.sleep(2)
		#delo1()
		#time.sleep(2)
		#delo2()
		#time.sleep(2)
		#delo3()
		#time.sleep(2)
		#delo4()
		#time.sleep(2)
		#delo5()
		#rospy.loginfo("Кайфуем!")
		#stop()


def start(msg):
	delo("start")
	if msg.data == False:
		rospy.loginfo("Чека выдернута - Поехали!")
		timer()
		moveCheck =  poehali_green("cake_b")
		if moveCheck == True:
			delo("cake_b")
			moveCheck = poehali_green("rotate_b")
			if moveCheck == True:
				moveCheck = poehali_green("dep_b")
				if moveCheck == True:
					deloCheck = delo("cherry_1")
					if deloCheck == True:
						moveCheck = poehali_green("cake_b_2")
						if moveCheck == True:
							moveCheck = poehali_green("prom")
							if moveCheck == True:
								moveCheck = poehali_green("parking")
								if moveCheck == True:
									rospy.loginfo("FINISHED!!!")
									stop()
		#if moveCheck == True:
			#delo("cake_1")
			#deloCheck = delo("cake_b")
			#if deloCheck == True:
				#moveCheck = poehali_green("start_off")
				#if moveCheck == True:
					#moveCheck = poehali_green("cake_y_preapproach")
					#if moveCheck == True:
						#moveCheck = poehali_green("cake_y")
						#if moveCheck ==  True:
							#delo("chery_1")
						#delo("cake_b_open")
						#moveCheck = poehali_green("cake_y")
						#if moveCheck == True:
							#deloCheck = delo("cake_y")
							#if deloCheck == True:
								#moveCheck == poehali_green("cake_r_preapproach")
								#if moveCheck == True:
									#moveCheck = poehali_green("cake_r")
									#if moveCheck == True:
										#deloCheck = delo("cake_r")
										#if deloCheck == True:
											#deloCheck = delo("test")
											#if deloCheck == True:
												#rospy.loginfo("Zaebis!")
												#stop()

def timer():
	t = Timer(95.0, stop)
	t.start()

def move_secs(x,y,z,secs):
	move = Twist()
	pub = rospy.Publisher("cmd_vel", Twist, queue_size = 100)
	rate = rospy.Rate(4)
	time.sleep(secs)
	move.linear.x = x
	move.linear.y = y
	move.angular.z = z
	pub.publish(move)
	time.sleep(secs)
	move.linear.x = 0
	move.linear.y = 0
	return True


def stop():
	rospy.loginfo('Time is out! Killing all ros nodes!')
	move = Twist()
	cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)
	rate = rospy.Rate(5)
	move.linear.x = 0
	move.linear.y = 0
	move.angular.z = 0.0
	while not rospy.is_shutdown():
		os.system("rosnode kill -a")        
		cmd_pub.publish(move)
		rate.sleep()

def moveToGoal(xGoal, yGoal, thetaGoal):
	#Svaz s move_base серверom используя SimpleActionClient
	ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

	#Ожидание экшн сервера
	while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
		rospy.loginfo("Waiting for the move_base action server to come up")

	goal = MoveBaseGoal()

	#Устанавливаю параметры фреймов
	goal.target_pose.header.frame_id = "odom"
	goal.target_pose.header.stamp = rospy.Time.now()

	#Отправляю приказ о езде к точке
	#goal.target_pose.pose.position = Point(xGoal, yGoal, thetaGoal)
	goal.target_pose.pose.position.x = xGoal
	goal.target_pose.pose.position.y = yGoal
	orient = tf.transformations.quaternion_from_euler(0,0,thetaGoal)
	#goal.target_pose.pose.orientation.x = 0.0
	#goal.target_pose.pose.orientation.y = 0.0
	goal.target_pose.pose.orientation.z = orient[2]
	goal.target_pose.pose.orientation.w = orient[3]

	rospy.loginfo('Sending goal location...')
	ac.send_goal(goal)

	ac.wait_for_result(rospy.Duration(200))

	if(ac.get_state() == GoalStatus.SUCCEEDED):
		rospy.loginfo('Приехали!')
		return True
	else:
		rospy.loginfo('Suka Blat!!!')
		return False

def poehali_green(kyda):
	#Объявляю точки
	Lb = 0.6
	Ly = 0.35
	Lr = 0.2
	theta0 = 0.0
	theta30 = 0.523599
	theta120 = 2.0944
	theta150 = 2.61799
	theta180 =  3.1415
	theta240 = 4.18879
	xStart = 0.0
	yStart = 0.0
	gr = False

	if kyda == "start":
		gr = moveToGoal(xStart, yStart, theta0)
	elif kyda == "cake_b":
		gr = moveToGoal(Lb, 0, theta0)
	elif kyda == "rotate_b":
		gr = moveToGoal(Lb, 0, theta180)
	elif kyda == "dep_b":
		gr = moveToGoal(0.2, 0, theta180)
	elif kyda == "cake_b_2":
		gr = moveToGoal(Lb+0.1, 0, theta180)
	elif kyda == "prom":
		gr = moveToGoal(Lb+0.1, -0.5, theta180)
	elif kyda =="parking":
		gr = moveToGoal(0.33,- 1.57, -2.150)
	elif kyda == "cake_y_preapproach":
		gr = moveToGoal(0.0, yStart, -theta30)
	elif kyda == "cake_y":
		rospy.loginfo("Еду за кейком желтым")
		gr = moveToGoal(0, Ly-0.1, -theta30)
	elif kyda == "cake_r_preapproach":
		gr = moveToGoal(0, Ly+0.05, -theta150)
	elif kyda == "cake_r":
		gr = moveToGoal(0, Ly+Lr+0.15, -theta150)
	elif kyda == "start_off":
		gr = moveToGoal(-0.15, 0, theta0)
	elif kyda == "cake_b_offset":
		gr = moveToGoal(0, 0.1, -theta30)
	return gr

def delo(type):
	pub = rospy.Publisher('servo', UInt16MultiArray, queue_size=10)
	angle = UInt16MultiArray()
	if type == "cake_b":
		rospy.loginfo('Gripping brown cake')
		a = [45, 160, 10, 140, 140, 55, 512, 512, 512]
		angle.data = a
		#rospy.Publisher("servo", UInt16MultiArray,queue_size=1).publish(angle_0)
		pub.publish(angle)
		time.sleep(1)
		rospy.loginfo('Сделал дело')
		return True

	elif type == "cake_r":
		rospy.loginfo('Gripping yellow cake')
		a = [45, 40, 140, 140, 15, 170, 512, 512, 512]
		angle.data = a
		pub.publish(angle)
		time.sleep(1)
		rospy.loginfo('Сделал дело')
		return True

	elif type == "cake_y":
		rospy.loginfo('Gripping red cake')
		a = [150, 160, 10, 20, 15, 170, 512, 512, 512]
		angle.data = a
		pub.publish(angle)
		time.sleep(1)
		rospy.loginfo('Сделал дело')
		return True

	elif type == "cherry_1":
		rospy.loginfo('Cherry 1 procedure')
		a = [45, 160, 10, 140, 15, 170, 512, 512, 612]
		angle.data = a
		pub.publish(angle)
		time.sleep(1)
		rospy.loginfo('Сделал дело')
		return True

	elif type == "cherry_2":
		rospy.loginfo('Cherry 2 procedure')
		a = [45, 160, 10, 140, 15, 170, 612, 512, 612]
		angle.data = a
		pub.publish(angle)
		time.sleep(1)
		rospy.loginfo('Сделал дело')
		return True

	elif type == "cherry_3":
		rospy.loginfo('Cherry 3 procedure')
		a = [45, 160, 10, 140, 15, 170, 612, 612, 612]
		angle.data = a
		pub.publish(angle)
		time.sleep(1)
		rospy.loginfo('Сделал дело')
		return True
		
	elif type == "test":
		rospy.loginfo('Taking off')
		a = [45, 160, 10, 140, 15, 170, 612, 612, 612]
		angle.data = a
		pub.publish(angle)
		time.sleep(1)
		rospy.loginfo('Сделал дело')
		return True
	elif type == "start":
		rospy.loginfo("Starting")
		a = [45, 160, 10, 140, 15, 170, 512, 512, 512]
		angle.data = a
		pub.publish(angle)
		time.sleep(1)
		rospy.loginfo("Done")
		return True
	elif type == "cake_b_open":
		rospy.loginfo("Opening brown")
		a = [45, 160, 10, 140, 15, 170, 512, 512, 612]
		angle.data = a
		pub.publish(angle)
		rospy.loginfo("Done")
		return True

if __name__ == '__main__':
    start_listener()
    start()


# #Схватываем Первые кейки
# def delo0():
# 	rospy.loginfo('Начинаю делать дело')
# 	angle_0 = UInt16MultiArray()
# 	a = [45, 160, 10, 140, 130, 65, 512, 512, 512]
# 	angle_0.data = a
# 	pub_0 = rospy.Publisher('servo', UInt16MultiArray, queue_size=10)
# 	#rospy.Publisher("servo", UInt16MultiArray,queue_size=1).publish(angle_0)
# 	pub.publish(angle_0)
# 	time.sleep(5)
# 	rospy.loginfo('Сделал дело')
# 	return True
# #angle_00 = [45, 160, 10, 140, 15, 170, 512, 512, 512]
# #Схватываем Вторые кейки
# def delo1():
# 	rospy.loginfo('Начинаю делать дело')
# 	angle_1 = [45, 40, 140, 140, 15, 170, 512, 512, 512]
# 	servo_pub.publish(angle_1)
# 	time.sleep(5)
# 	rospy.loginfo('Сделал дело')
# 	return True

# #Схватываем Третьи кейки
# def delo2():
# 	rospy.loginfo('Начинаю делать дело')
# 	angle_2 = [150, 160, 10, 20, 15, 170, 512, 512, 512]
# 	servo_pub.publish(angle_2)
# 	time.sleep(5)
# 	rospy.loginfo('Сделал дело')
# 	return True

# #Выкладываем 1 вишни и отпускаем
# def delo3():
# 	rospy.loginfo('Начинаю делать дело')
# 	angle_3 = [45, 160, 10, 140, 15, 170, 512, 512, 612]
# 	servo_pub.publish(angle_3)
# 	time.sleep(5)
# 	rospy.loginfo('Сделал дело')
# 	return True

# #Выкладываем 2 вишни и отпускаем
# def delo4():
# 	rospy.loginfo('Начинаю делать дело')
# 	angle_4 = [45, 160, 10, 140, 15, 170, 612, 512, 612]
# 	servo_pub.publish(angle_4)
# 	time.sleep(5)
# 	rospy.loginfo('Сделал дело')
# 	return True

# #Выкладываем 3 вишни и отпускаем
# def delo5():
# 	rospy.loginfo('Начинаю делать дело')
# 	angle_5 = [45, 160, 10, 140, 15, 170, 612, 612, 612]
# 	servo_pub.publish(angle_5)
# 	time.sleep(5)
# 	rospy.loginfo('Сделал дело')
# 	return True




