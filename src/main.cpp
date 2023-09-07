#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist vel;

void delta_vel_callback(const geometry_msgs::TwistConstPtr& msg){
    vel.linear.y += msg->linear.y;
    vel.angular.z += msg->angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "debug");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("delta_vel",10,delta_vel_callback);
    ros::Rate loop_rate(1);
    
    vel.linear.x = 1;
    vel.linear.y = 0;
    vel.angular.z = 0;
    
    while(ros::ok())
    {   
        ros::spinOnce();
        pub.publish(vel);
        loop_rate.sleep();
    }
    return 0;
}