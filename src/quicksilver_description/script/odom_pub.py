#!/usr/bin/python3
 
import rospy
from nav_msgs.msg import Odometry
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
    br.sendTransform((msgx, msgy, 0),
                     (0.0,0.0,result.pose.orientation.z,result.pose.orientation.w),
                     rospy.Time.now(),
                     'base_footprint',
                     "odom")

    odom.pose.pose.position.x = result.pose.position.x
    odom.pose.pose.position.y = result.pose.position.y
    odom.pose.pose.position.z = 0
    odom.pose.pose.orientation.x = 0.0
    odom.pose.pose.orientation.y = 0.0
    odom.pose.pose.orientation.z = result.pose.orientation.z
    odom.pose.pose.orientation.w = result.pose.orientation.w
    odom.twist.twist.linear.x = result.twist.linear.x
    odom.twist.twist.linear.y = result.twist.linear.y
    odom.twist.twist.angular.z = result.twist.angular.z

    odom.header.stamp = now
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_footprint'
    odom_pub.publish(odom)

    r.sleep()
    