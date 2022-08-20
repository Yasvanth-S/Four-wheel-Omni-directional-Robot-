#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))
class JoyClass:
    def __init__(self, scale=1.0, offset=0.0, deadband=0.1):
        rospy.init_node("quicksilver_node")
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.wheel1_pub = rospy.Publisher("/quicksilver/rim_wheel2_joint/command", Float64, queue_size=10)
        self.wheel2_pub = rospy.Publisher("/quicksilver/rim_wheel3_joint/command", Float64, queue_size=10)
        self.wheel3_pub = rospy.Publisher("/quicksilver/rim_wheel4_joint/command", Float64, queue_size=10)
        self.wheel4_pub = rospy.Publisher("/quicksilver/rim_wheel1_joint/command", Float64, queue_size=10)

        self.rate = rospy.Rate(10)

    def joy_callback(self, msg):
        wheel1, wheel2, wheel3, wheel4, joyY, joyZ, angular = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        rightTrig = (abs(msg.axes[5]-1.0)/2) 
        leftTrig = (-abs(msg.axes[2]-1.0)/2)
        if(rightTrig!=0 and leftTrig!=0):
            angular = 0.0
        elif(rightTrig>0.0 and leftTrig == 0.0):
            angular = rightTrig
        elif(leftTrig<0.0 and rightTrig == 0.0):
            angular = leftTrig     
        wheel1 = (msg.axes[1]*5.5) + (msg.axes[7]*5.5) + (angular * 3.6)
        wheel2 = (-msg.axes[0]*5.5) + (-msg.axes[6]*5.5) + (angular * 3.6)
        wheel3 = (-msg.axes[1]*5.5) + (-msg.axes[7]*5.5) + (angular * 3.6)
        wheel4 = (msg.axes[0]*5.5) + (msg.axes[6]*5.5) + (angular * 3.6)  #angular negative due to inward placement of wheels
        self.wheel1_pub.publish(constrain(wheel1,-5.5,5.5))
        self.wheel2_pub.publish(constrain(wheel2,-5.5,5.5))
        self.wheel3_pub.publish(constrain(wheel3,-5.5,5.5))
        self.wheel4_pub.publish(constrain(wheel4,-5.5,5.5))
        #self.test.publish(angular)

if __name__=="__main__":
    try:
        JoyClass()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
