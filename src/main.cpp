#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist vel;

void delta_vel_callback(const geometry_msgs::TwistConstPtr& msg){
    vel.linear.y = msg->linear.y;
    vel.angular.z = msg->angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    double span=1;
    nh.getParam("/span",span);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("delta_vel",10,delta_vel_callback);
    
    double Vx = 0,Vy = 0,w = 0;
    
    while(ros::ok())
    {   
        ros::spinOnce();

        nh.getParam("/Vx",Vx);
        nh.getParam("/Vy",Vy);
        nh.getParam("/w",w);
        vel.linear.x = Vx;
        vel.linear.y += Vy;
        vel.angular.z += w;
        
        pub.publish(vel);
        ros::Duration(span).sleep();
    }
    return 0;
}