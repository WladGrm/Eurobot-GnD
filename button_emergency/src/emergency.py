#!/usr/bin/python3
import rospy
import os
from threading import Timer
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16MultiArray

def timer():
	t = Timer(2.0, stop)
	t.start()

def stop():
    rospy.loginfo('Time is out. Shutting down all nodes')  
    os.system("rosnode kill -a")
	
def callback(msg):
	if msg.data == True:
		servo_pub = rospy.Publisher("servo", UInt16MultiArray, queue_size = 10)
		angle = UInt16MultiArray()
		a = [45, 160, 10, 140, 15, 170, 512, 512, 512]
		angle.data = a
		rospy.loginfo("EMERGENCY!!! Sending 0's to /cmd_vel")
		move.linear.x = 0
		move.linear.y = 0
		move.angular.z = 0.0
		timer()
		while not rospy.is_shutdown():        
			pub.publish(move)
			servo_pub.publish(angle)
	 #rate.sleep()    


    
if __name__ == '__main__':
    try:
        rospy.init_node("Emergency_button_node", anonymous = True)
        rospy.Subscriber("emergency", Bool, callback)
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)
        rate = rospy.Rate(10)
        move = Twist()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
