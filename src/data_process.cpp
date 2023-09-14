#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

int black = 0,nonblack = 1;
int dir=0;

int8_t std_tracker_data[20],temp[20];
int8_t weight_array[20] = {2,1,0,-1,-2,-3,0,0,0,-3,-2,-1,0,1,2,3,0,0,0,3};
int8_t corrdinate[20][2] = {{3,-2},{3,-1},{3,0},{3,1},{3,2}, {2,3},{1,3},{0,3},{-1,3},{-2,3}, {-3,2},{-3,1},{-3,0},{-3,-1},{-3,-2}, {-2,-3},{-1,3-},{0,-3},{1,-3},{2,-3}};
geometry_msgs::Twist error;
std_msgs::Bool node_point;
// 1. tracker data standardrize  sub : tracker_data, dir
class Tracker {
public:
    Tracker(ros::NodeHandle& node_handle) : nh(node_handle) {
        data_sub = nh.subscribe<std_msgs::Int8MultiArray>("tracker_data", 10, &Tracker::tracker_callback, this);
        dir_sub = nh.subscribe<std_msgs::Int8>("cmd_ori", 10, &Tracker::dir_callback, this);
    }
    std::vector<int8_t> tracker_data;
    

    void tracker_callback(const std_msgs::Int8MultiArray::ConstPtr& msg) {
         tracker_data = msg->data;
    } 
    void dir_callback(const std_msgs::Int8::ConstPtr& msg)
    {
        dir = msg->data;
    }
    void tracker_data_std(){
        size_t i;
        for (i = 0; i < tracker_data.size(); ++i) {
            temp[i] = tracker_data[i];}
        // 放temp就修好了 '_'
        for (i = 0; i < 4; ++i){for(size_t j= 0; j < 5; ++j){
            if(i-dir < 0){
                std_tracker_data[(i-dir+4)%4*5+j]=temp[i*5+j];
            }
            else{
                std_tracker_data[(i-dir)%4*5+j]=temp[i*5+j];
            }
        }}
    }
    
private:
    ros::NodeHandle nh;
    ros::Subscriber data_sub;  //tracker data
    ros::Subscriber dir_sub;   //map
};

// 2. node detect  pub : node_point
bool detectRisingEdge(bool current) {
    static bool previous = false;
    bool temp = current && !previous;
    previous = current;
    return temp;
}
bool detectfallingEdge(bool current) {
    static bool previous = false;
    bool temp = !current && previous;
    previous = current;
    return temp;
}

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
    // else if(detectfallingEdge((bool)std_tracker_data[8]) && std_tracker_data[7] == nonblack || detectfallingEdge((bool)std_tracker_data[16] && std_tracker_data[17] == nonblack)){
    //     result.data = true;
    //     return result;
    // }
    result.data = false;
    return result;
}

//3. error calculate  pub : error
void error_cal(){
    int counter_F,counter_B,total_Err_F,total_Err_B;
    double Err_d,Err_theta, Err_F,Err_B;
    size_t i;
    for(i = 0, counter_F = 0,counter_B = 0,total_Err_F = 0,total_Err_B = 0; i < 20; ++i){
            if(std_tracker_data[i] == black){
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
        error.linear.x = Err_d;
        error.angular.z = Err_theta;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_process");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    Tracker tracker(nh);

    double span=1;
    nh.getParam("/span",span);

    ros::Publisher node_pub;   //map
    ros::Publisher Err_pub;    //PID
    node_pub = nh.advertise<std_msgs::Bool>("node_detect", 10);
    Err_pub = nh.advertise<geometry_msgs::Twist>("error", 10);

    size_t i;
    error.linear.x=0;
    error.angular.z=0;
    
    nh.getParam("/black",black);
    nh.getParam("/nonblack",nonblack);

    while (ros::ok()) {
        ros::spinOnce();
        //nh.getParam("/dir",dir);
        
        if(dir >= 0 && dir < 4){
            tracker.tracker_data_std();
            node_point = node_detect();
            error_cal();
            ROS_INFO("node detect: %d",node_point.data);
            node_pub.publish(node_point);
            Err_pub.publish(error);
        }
        
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
