#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

double Err_d,Err_theta;

void Err_callback(const geometry_msgs::TwistConstPtr& msg){
    Err_d = msg->linear.x;
    Err_theta = msg->angular.z;
}
double PID_ratio(double error, double kp, double ki, double kd){
    double u,differential;
    static double prev_error = 0, integral = 0;
    integral += error;
    if (ki * integral > 1) integral = 1/ki;
    else if (ki * integral < -1) integral = -1/ki;
    differential = error - prev_error;
    prev_error = error;

    u = kp * error + ki * integral + kd * differential;
    if (u > 1) {u = 1;}
    else if (u < -1) {u = -1;}

    return u;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PID_controller");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    ros::Subscriber Err_sub = nh.subscribe<geometry_msgs::Twist>("error",10,Err_callback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("delta_vel",10);
    ros::Rate LoopRate(1);


    geometry_msgs::Twist velocity;
    double vkp=0,vki=0,vkd=0,wkp=0,wki=0,wkd=0;
    double v_max,w_max;
    nh_local.param<double>("v_max", v_max, 0);
    nh_local.param<double>("vkp",vkp, 0);
    nh_local.param<double>("vki",vki, 0);
    nh_local.param<double>("vkd",vkd, 0);

    nh_local.param<double>("w_max", w_max, 0);
    nh_local.param<double>("wkp",wkp, 0);
    nh_local.param<double>("wki",wki, 0);
    nh_local.param<double>("wkd",wkd, 0);

    while(ros::ok())
    {   
        ros::spinOnce();
        velocity.linear.y = -v_max*PID_ratio(Err_d,vkp,vki,vkd);
        velocity.angular.z = w_max*PID_ratio(Err_theta,wkp,wki,wkd);

        // ROS_INFO("Err_d: %f  Err_theta: %f",Err_d,Err_theta);
        // ROS_INFO("PID_d: %f  PID_theta: %f",PID_ratio(Err_d,vkp,vki,vkd),PID_ratio(Err_theta,wkp,wki,wkd));
        // ROS_INFO("v_max: %f  vkp: %f  vki: %f  vkd: %f",v_max,vkp,vki,vkd);
        // ROS_INFO("w_max: %f  wkp: %f  wki: %f  wkd: %f",w_max,wkp,wki,wkd);
        ROS_INFO("linear.y: %f  angular.z: %f",velocity.linear.y,velocity.angular.z);

        vel_pub.publish(velocity);
        LoopRate.sleep();
    }
    return 0;
}