#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

// Sub : tracker_data, ori
// Pub : node_detect, vel

//base arguments
int black = 0,nonblack = 1;
size_t i;
//main_function arguments --in ROS
int ori=0, prev_ori;
std_msgs::Bool node_point;
double V = 0, vel_limit = 0;
double u_d = 0, u_theta = 0, u = 0;
geometry_msgs::Twist vel;
//tracker arguments --from Arduino
int black_count = 0;
double Err_d,Err_theta;
int8_t std_tracker_data[20],temp[20];
int8_t weight_array[20] = {2,1,0,-1,-2,-3,0,0,0,-3,-2,-1,0,1,2,3,0,0,0,3};


// 1. tracker data standardrize  
class Tracker {
public:
    Tracker(ros::NodeHandle& node_handle) : nh(node_handle) {
        sub_tracker = nh.subscribe<std_msgs::Int8MultiArray>("tracker_data", 10, &Tracker::tracker_callback, this);
        sub_ori = nh.subscribe<std_msgs::Int8>("cmd_ori", 10, &Tracker::ori_callback, this);
    }
    std::vector<int8_t> tracker_data;

    void tracker_callback(const std_msgs::Int8MultiArray::ConstPtr& msg) {
         tracker_data = msg->data;
    } 
    void ori_callback(const std_msgs::Int8::ConstPtr& msg)
    {
        ori = msg->data;
    }
    void tracker_data_std(){
        
        for (i = 0; i < tracker_data.size(); ++i) {
            temp[i] = tracker_data[i];}
        // 放temp就修好了 '_'
        for (i = 0; i < 4; ++i){for(size_t j= 0; j < 5; ++j){
            if(i-ori < 0){
                std_tracker_data[(i-ori+4)%4*5+j]=temp[i*5+j];
            }
            else{
                std_tracker_data[(i-ori)%4*5+j]=temp[i*5+j];
            }
        }}
    }
    
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_tracker;  //tracker data
    ros::Subscriber sub_ori;   //map
};

// 2. node detect  pub : node_point
std_msgs::Bool node_detect(){
    std_msgs::Bool result;
    if(std_tracker_data[4] == black  && std_tracker_data[10] == black && (std_tracker_data[5] == black || std_tracker_data[9] == black)){
        result.data = false;
        return result;
    }
    if(std_tracker_data[0] == black  && std_tracker_data[14] == black && (std_tracker_data[19] == black || std_tracker_data[15] == black)){
        result.data = false;
        return result;
    }
    if(std_tracker_data[16] == black && std_tracker_data[17] == black && std_tracker_data[12] == black){
        result.data = true;
        return result;
    }
    if(std_tracker_data[8] == black && std_tracker_data[7] == black && std_tracker_data[12] == black){
        result.data = true;
        return result;
    }
    result.data = false;
    return result;
}

// 3. PID control --vel
void error_cal(){
    int counter_F,counter_B,total_Err_F,total_Err_B;
    double Err_F,Err_B;
    size_t i;
    for(i = 0,black_count = 0,  counter_F = 0,counter_B = 0,total_Err_F = 0,total_Err_B = 0; i < 20; ++i){
            if(std_tracker_data[i] == black){
                black_count++;
                if(i <= 5 || i ==19){
                    total_Err_F += weight_array[i];
                    counter_F++;
                }
                if(i >= 9 && i <= 15){
                    total_Err_B += weight_array[i];
                    counter_B++;
        }}}
        if(counter_F == 0){
            Err_F = (double) total_Err_F;
        }
        else{
            Err_F = (double) total_Err_F / counter_F;
        }
        
        if(counter_B == 0){
            Err_B = (double) total_Err_B;
        }
        else{
            Err_B = (double) total_Err_B / counter_B;
        }
        Err_d = (Err_F + Err_B) / 2;
        Err_theta = atan((Err_F - Err_B) / 7);
}
double PID_control(double error, double kp, double ki, double kd){
    double u,differential;
    static double prev_error = 0, integral = 0;
    integral += error;
    if (ki * integral > 1) integral = 1/ki;
    else if (ki * integral < -1) integral = -1/ki;
    differential = error - prev_error;
    prev_error = error;

    u = kp * error + ki * integral + kd * differential;
    if (u > vel_limit) {u = vel_limit;}
    else if (u < -vel_limit) {u = -vel_limit;}

    return u;
}

//tools
bool detectfallingEdge(bool current) {
    static bool previous = false;
    bool temp = !current && previous;
    previous = current;
    return temp;
}

//main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "navi_tracking");
    ros::NodeHandle nh;
    Tracker tracker(nh);
    ros::Publisher pub_node = nh.advertise<std_msgs::Bool>("node_detect", 10);;   //map
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("vel",10);

    double span=1;
    nh.getParam("/span",span);
    nh.getParam("/black",black);
    nh.getParam("/nonblack",nonblack);

    
    double vkp = 0,vki = 0,vkd = 0,wkp = 0,wki = 0,wkd = 0;

    while (ros::ok()) {
        ros::spinOnce();
        
        nh.getParam("vkp",vkp);
        nh.getParam("vki",vki);
        nh.getParam("vkd",vkd);

        nh.getParam("wkp",wkp);
        nh.getParam("wki",wki);
        nh.getParam("wkd",wkd);

        nh.getParam("/V",V);
        nh.getParam("/vel_limit",vel_limit);

        if(ori >= 0 && ori < 4 ){
            tracker.tracker_data_std();
            node_point = node_detect();

            if(detectfallingEdge(node_point.data)){
                //turn left
                if(prev_ori == (ori+3)%4){
                    u = vel_limit;
                }
                //turn right
                else if(prev_ori == (ori+1)%4){
                    u = -vel_limit;
                }
                prev_ori = ori;
            }
            
            error_cal();
            if(black_count != 0){
                u_d = -PID_control(Err_d,vkp,vki,vkd);
                u_theta = -PID_control(Err_theta,wkp,wki,wkd);

                vel.linear.x = V*cos(((double)ori)*0.5*3.1415) - u_d*sin(((double)ori)*0.5*3.1415);
                vel.linear.y = V*sin(((double)ori)*0.5*3.1415) + u_d*cos(((double)ori)*0.5*3.1415);
                vel.angular.z = u_theta;
            }
            else{
                vel.linear.x = V*cos(((double)ori)*0.5*3.1415) - u*sin(((double)ori)*0.5*3.1415);
                vel.linear.y = V*sin(((double)ori)*0.5*3.1415) + u*cos(((double)ori)*0.5*3.1415);
                vel.angular.z = u_theta;
            }
            

            pub_node.publish(node_point);
            pub_vel.publish(vel);
        }

        //print for debug
        // ROS_INFO("node detect: %d",node_point.data);
        // ROS_INFO("black: %d  nonblack: %d",black,nonblack);
        //ROS_INFO("hello");
        //ROS_INFO("dir: %d",tracker.dir);
        // for (i = 0; i < 20; ++i){
        //     ROS_INFO("tracker_data[%zu]: %d",i,std_tracker_data[i]);
        // }
        //ROS_INFO("error_d: %f error_w: %f",error.linear.x,error.angular.z);
        
        ros::Duration(span).sleep();
    }

    return 0;
}

//tools

// bool detectRisingEdge(bool current) {
//     static bool previous = false;
//     bool temp = current && !previous;
//     previous = current;
//     return temp;
// }
