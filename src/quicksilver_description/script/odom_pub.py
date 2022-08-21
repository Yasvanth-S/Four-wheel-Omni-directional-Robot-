#!/usr/bin/python3

import queue
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

rospy.init_node('odom_pub')
odom_pub = rospy.Publisher('/odom', Odometry,queue_size=100)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
br = tf.TransformBroadcaster()
odom = Odometry()
now = rospy.Time.now()
model = GetModelStateRequest()
model.model_name = 'quicksilver'

r = rospy.Rate(100)

while not rospy.is_shutdown():

    result = get_model_srv(model)
    msgx = result.pose.position.x
    msgy = result.pose.position.y
    msgth = result.pose.orientation.z
    br.sendTransform((msgx, msgy, 0),
                     (result.pose.orientation.x,result.pose.orientation.y,result.pose.orientation.z,result.pose.orientation.w),
                     rospy.Time.now(),
                     'origin_link',
                     "odom")

    odom.pose.pose = result.pose 
    odom.twist.twist = result.twist

    odom.header.stamp = now
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'origin_link'
    odom_pub.publish(odom)

    r.sleep()