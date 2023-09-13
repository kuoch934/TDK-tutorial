#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "math.h"

geometry_msgs::Twist vel,det_vel;
double V = 0,dV = 0,Vx = 0, Vy = 0, w = 0;
int dir = 0;
bool node = false;

void delta_vel_callback(const geometry_msgs::TwistConstPtr& msg){
    det_vel.linear.y = msg->linear.y;
    det_vel.angular.z = msg->angular.z;
}

void dir_callback(const std_msgs::Int8::ConstPtr& msg){
    dir = msg->data;
}

void detect_callback(const std_msgs::Bool::ConstPtr& msg){
    node = msg->data;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    double span=1;
    nh.getParam("/span",span);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("delta_vel",10,delta_vel_callback);
    ros::Subscriber dir_sub = nh.subscribe<std_msgs::Int8>("cmd_ori", 10, dir_callback);
    ros::Subscriber sub1 = nh.subscribe<std_msgs::Bool>("node_detect",10,detect_callback);
    
    
    
    

    while(ros::ok())
    {   
        ros::spinOnce();

        nh.getParam("/V",V);
        nh.getParam("/dV",dV);
        nh.getParam("/w",w);
        nh.getParam("/dir",dir);
        if(node){
            nh.setParam("/dir",2);
        }

        Vx = V*cos(((double)dir)*0.5*3.1415) - det_vel.linear.y*sin(((double)dir)*0.5*3.1415);
        Vy = V*sin(((double)dir)*0.5*3.1415) + det_vel.linear.y*cos(((double)dir)*0.5*3.1415);
        vel.linear.x = Vx;
        vel.linear.y = Vy;
        vel.angular.z = det_vel.angular.z;
        ROS_INFO("dir: %d ",dir);
        ROS_INFO("dv: %f  dtheta: %f",det_vel.linear.y,det_vel.angular.z);
        ROS_INFO("Vx: %f  Vy: %f  w: %f",vel.linear.x,vel.linear.y,vel.angular.z);

        pub.publish(vel);
        ros::Duration(span).sleep();
    }
    return 0;
}