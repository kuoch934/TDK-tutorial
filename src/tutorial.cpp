#include "ros/ros.h"
#include "std_msgs/Int64.h"

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    double span = 1;
    
    
    double v_max;
    nh_local.param<double>("v_max", v_max, 20);
    std::cout << v_max << std::endl;
    int number = 0;
    std_msgs::Int64 msg;
    
    while(ros::ok())
    {
        nh.getParam("/span",span);
        ros::Rate loop_rate(span);
        msg.data = number;
        //ROS_INFO("%ld", msg.data);
        ROS_INFO("hello");
        ROS_INFO("%f",span);
        number++;
        loop_rate.sleep();
    }
    return 0;
}