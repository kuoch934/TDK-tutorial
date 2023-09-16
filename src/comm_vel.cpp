#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int8.h"

//to control which cmd_vel can be send

//main_function arguments --from main_func & script & navi
geometry_msgs::Twist vel;
int ori = 0;

void vel_callback(const geometry_msgs::TwistConstPtr& msg){
    vel.linear.x = msg->linear.x;
    vel.linear.y = msg->linear.y;
    vel.angular.z = msg->angular.z;
}

void ori_callback(const std_msgs::Int8::ConstPtr& msg){
    ori = msg->data;
}

//main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_vel");
    ros::NodeHandle nh;

    double span=1;
    nh.getParam("/span",span);
    ros::Subscriber sub_dir = nh.subscribe<std_msgs::Int8>("cmd_ori", 10, ori_callback);
    ros::Subscriber sub_vel = nh.subscribe<geometry_msgs::Twist>("vel", 10, vel_callback);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    
    
    std_msgs::Int8 script;
    
    

    while(ros::ok())
    {   
        ros::spinOnce();
            
        if(ori == -1){
            vel.linear.x = 0;
            vel.linear.y = 0;
            vel.angular.z = 0;
            pub.publish(vel);
        }
        else if(ori == 6){

        }
        else if(ori == 7){

        }
        else if(ori == 8){

        }
        else if(ori >= 0 && ori < 4 ){
            pub.publish(vel);
        }

        
        
        // ROS_INFO("dir: %d ",dir);
        // ROS_INFO("dv: %f  dtheta: %f",det_vel.linear.y,det_vel.angular.z);
        // ROS_INFO("Vx: %f  Vy: %f  w: %f",vel.linear.x,vel.linear.y,vel.angular.z);

        ros::Duration(span).sleep();
    }
    return 0;
}