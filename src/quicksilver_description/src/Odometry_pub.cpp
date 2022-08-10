#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>

#define PI 3.14159265

class OdometryPublisher
{
	public:
		OdometryPublisher();

	private:
		void encoderCallback(const std_msgs::Float32MultiArray::ConstPtr& encoders);

		ros::NodeHandle nh;
		ros::Subscriber enc_sub;
		ros::Publisher odom_pub;
		tf::TransformBroadcaster odom_broadcaster;
        ros::Time current_time;
        ros::Time last_time;
		double x, y, th,vx,vy,vth;
};


OdometryPublisher::OdometryPublisher()
{
	x = y = th = 0.0;
	vx = vy = vth = 0.0;
    ros::NodeHandle nh_priv("~");
    current_time = ros::Time::now();
    last_time = ros::Time::now();
	enc_sub = nh.subscribe<std_msgs::Float32MultiArray>("encoder", 10, &OdometryPublisher::encoderCallback, this);
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
}

void OdometryPublisher::encoderCallback(const std_msgs::Float32MultiArray::ConstPtr& encoders)
{	
    current_time = ros::Time::now();
    double s1 = (encoders->data[0]/60.0)*0.03141;
    double s2 = (encoders->data[1]/60.0)*0.03141;
	double s3 = (encoders->data[2]/60.0)*0.03141;
    double s4 = (encoders->data[3]/60.0)*0.03141;

    vx = ((-1)*s1) + ((1)*s3);
    vy = ((1)*s2) + ((-1)*s4);
    vth = s1+s2+s3+s4;

    double dt = (current_time - last_time).toSec();
    double deltaX = (vx * cos(th) - vy * sin(th)) * dt;
    double deltaY = (vx *sin(th) + vy*cos(th)) * dt;
    double deltaTh = vth * dt;
    x+=deltaX;
    y+=deltaY;
    th+=deltaTh;

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);


	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0; 
	odom_trans.transform.rotation = odom_quat;

	odom_broadcaster.sendTransform(odom_trans);

	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";


	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

   odom.child_frame_id = "base_link";
   odom.twist.twist.linear.x = vx;
   odom.twist.twist.linear.y = vy;
   odom.twist.twist.linear.z = 0.0;
   odom.twist.twist.angular.z = vth;

	odom_pub.publish(odom);
    last_time = current_time;
    vx = vy = vth = 0;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_publisher");
	OdometryPublisher odom;
	ros::spin();
}
