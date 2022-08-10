#include "ros/ros.h"
#include <bits/stdc++.h>
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Joy.h"

class Encoder_test
{
    public:
        Encoder_test();

    private:
        void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg);
        ros::NodeHandle nh;
        ros::Publisher enc_pub;
        ros::Subscriber joy_sub;
        std_msgs::Float32MultiArray msg_enc;
};

Encoder_test::Encoder_test()
{
    ros::NodeHandle nh_priv("~");
    nh_priv.setParam("joy_node/dev","/dev/input/js1");
    enc_pub = nh.advertise<std_msgs::Float32MultiArray>("encoder",1000);
    joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1000, &Encoder_test::joy_cb,this);
    msg_enc.data.clear();
    msg_enc.data.push_back(0.0);
    msg_enc.data.push_back(0.0);
    msg_enc.data.push_back(0.0);
    msg_enc.data.push_back(0.0);
}

void Encoder_test::joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    if(joy_msg->axes[7])
   {
       if(joy_msg->axes[7] > 0.1)
       {
           msg_enc.data[1] = 28.5;
           msg_enc.data[3] = -28.5;
           msg_enc.data[0] = 0.0;
           msg_enc.data[2] = 0.0;
           enc_pub.publish(msg_enc);
           msg_enc.data[0] =msg_enc.data[1] =msg_enc.data[2]=msg_enc.data[3]=0;
       }
       else if(joy_msg->axes[7] < 0.0)
       {
           msg_enc.data[1] = -28.5;
           msg_enc.data[3] = 28.5;
           msg_enc.data[0] = 0.0;
           msg_enc.data[2] = 0.0;
           enc_pub.publish(msg_enc);
           msg_enc.data[0] =msg_enc.data[1] =msg_enc.data[2]=msg_enc.data[3]=0;
       }
   }
   else if(joy_msg->axes[6])
   {
       if(joy_msg->axes[6] > 0.1)
       {
           msg_enc.data[0] = -28.5;
           msg_enc.data[2] = 28.5;
           msg_enc.data[1] = 0.0;
           msg_enc.data[3] = 0.0;
           enc_pub.publish(msg_enc);
           msg_enc.data[0] =msg_enc.data[1] =msg_enc.data[2]=msg_enc.data[3]=0;
       }
       else if(joy_msg->axes[6] < 0.0)
       {
           msg_enc.data[0] = 28.5;
           msg_enc.data[2] = -28.5;
           msg_enc.data[1] = 0.0;
           msg_enc.data[3] = 0.0;
           enc_pub.publish(msg_enc);
           msg_enc.data[0] =msg_enc.data[1] =msg_enc.data[2]=msg_enc.data[3]=0;
       }
    }
    else if(joy_msg->buttons[4])
    {
        msg_enc.data[1] = -11.4;
        msg_enc.data[3] = -11.4;
        msg_enc.data[0] = -11.4;
        msg_enc.data[2] = -11.4;
        enc_pub.publish(msg_enc);
        msg_enc.data[0] =msg_enc.data[1] =msg_enc.data[2]=msg_enc.data[3]=0;
    }
    else if(joy_msg->buttons[5])
    {
        msg_enc.data[1] = 11.4;
        msg_enc.data[3] = 11.4;
        msg_enc.data[0] = 11.4;
        msg_enc.data[2] = 11.4;
        enc_pub.publish(msg_enc);
        msg_enc.data[0] =msg_enc.data[1] =msg_enc.data[2]=msg_enc.data[3]=0;
    }
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv,"encoder_pub");
    Encoder_test enc;
    ros::spin();
    return 0;
}

