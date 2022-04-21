#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class JoyClass:
    def __init__(self, scale=1.0, offset=0.0, deadband=0.1):
        rospy.init_node("joy5_node")
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.wheel1_pub = rospy.Publisher("/wheel1_vel", Float32, queue_size=10)
        self.wheel2_pub = rospy.Publisher("/wheel2_vel", Float32, queue_size=10)
        self.wheel3_pub = rospy.Publisher("/wheel3_vel", Float32, queue_size=10)
        self.wheel4_pub = rospy.Publisher("/wheel4_vel", Float32, queue_size=10)
        self.yaxis_pub = rospy.Publisher("/yaxis_vel", Float32, queue_size=10)
        self.zaxis_pub = rospy.Publisher("/zaxis_vel", Float32, queue_size=10)
        self.test = rospy.Publisher("/test", Float32, queue_size=10)

    def joy_callback(self, msg):
        angular = 0.0
        rightTrig = (abs(msg.axes[5]-1.0)/2) * 0.1
        leftTrig = (-abs(msg.axes[2]-1.0)/2) * 0.1
        #self.test.publish(leftTrig)
        if(rightTrig!=0 and leftTrig!=0):
            angular = 0.0
        elif(rightTrig>0.0 and leftTrig == 0.0):
            angular = rightTrig
        elif(leftTrig<0.0 and rightTrig == 0.0):
            angular = leftTrig
        wheel1 = (msg.axes[1]+0.5*angular)*0.1
        wheel3 = (msg.axes[1]-0.5*angular)*0.1
        wheel2 = (msg.axes[0]+0.5*angular)*0.1
        wheel4 = (msg.axes[0]-0.5*angular)*0.1       
        self.wheel1_pub.publish(wheel1)
        self.wheel2_pub.publish(wheel2)
        self.wheel3_pub.publish(wheel3)
        self.wheel4_pub.publish(wheel4)
        self.yaxis_pub.publish(msg.axes[3]*0.1)
        self.zaxis_pub.publish(msg.axes[4]*0.1)

if __name__=="__main__":
    try:
        JoyClass()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
