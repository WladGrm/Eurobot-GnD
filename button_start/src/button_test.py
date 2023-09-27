#!/usr/bin/python3
import rospy
from std_msgs.msg import Bool

def listener():
    rospy.init_node("Start_button_node", anonymous = True)
    rospy.Subscriber("start", Bool, callback)
    rospy.spin()
    
    
def callback(msg):
    if msg.data == True:
        rospy.loginfo("TRUE")
    if msg.data == False:
        rospy.loginfo("FALSE")
        
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
