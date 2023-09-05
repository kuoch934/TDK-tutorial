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
    if (u > 1) u = 1;
    else if (u < -1) u = -1;
    return u;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PID_controller");
    ros::NodeHandle nh;
    ros::Subscriber Err_sub = nh.subscribe<geometry_msgs::Twist>("error",10,Err_callback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("delta_vel",10);
    geometry_msgs::Twist velocity;
    double v_max=1;

    while(ros::ok())
    {   
        ros::spinOnce();
        velocity.linear.y = -PID_ratio(Err_d,1,0,0);
        velocity.angular.z = PID_ratio(Err_theta,1,0,0);
        vel_pub.publish(velocity);
    }
    return 0;
}