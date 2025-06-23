#include <vector>
#include <unordered_set>
#include <iostream>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>  
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <actionlib_msgs/GoalID.h>
#include <nav_msgs/Odometry.h>

#include <tutorial_vision/CircleDetectResult.h>
#include <tutorial_vision/StringStamped.h>

#define ALTITUDE  1.4
#define END_X 0.0
#define END_Y 0.0
#define END_Z 0.5
int fsm_state = 0;
int fly_task_state = 0;

int fly_task = 1;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg);
ros::Time last_request;
tf::Quaternion quat; 
double roll, pitch, yaw;
float init_position_x_take_off =0;
float init_position_y_take_off =0;
float init_position_z_take_off =0;
bool  flag_init_position = false;
nav_msgs::Odometry local_pos;
geometry_msgs::PoseStamped pose;
bool point_arrive_flag = false;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);
void fly_to_point(double x, double y, double z,double stop_time);   
void end_fly_task();

void run_fly_task1()
{ 
    switch (fly_task_state)
    {
    case 0:
        fly_to_point(0.5,0,ALTITUDE,0);
    break;
    case 1:
        fly_to_point(0.5,0.5,ALTITUDE,0);
    break;
    case 2:
        fly_to_point(0,0.5,ALTITUDE,0);
    break;
    case 3:
        end_fly_task();
    break;
    
    default:
        break;
    }
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


int8_t fly_target = 0x00;
void fly_target_cb(const std_msgs::UInt8::ConstPtr& msg) {
    ROS_INFO("接收到 fly_target: %d (十六进制: 0x%X)", msg->data, msg->data);
    fly_target = msg->data;
}
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg;
    if (flag_init_position==false && (local_pos.pose.pose.position.z!=0))
    {
		init_position_x_take_off = local_pos.pose.pose.position.x;
	    init_position_y_take_off = local_pos.pose.pose.position.y;
	    init_position_z_take_off = local_pos.pose.pose.position.z;
        flag_init_position = true;		    
    }
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);	
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

double getLengthBetweenPoints(double *out_err_x = nullptr, double *out_err_y = nullptr) {
    double err_x = pose.pose.position.x - local_pos.pose.pose.position.x;
    double err_y = pose.pose.position.y - local_pos.pose.pose.position.y;
    if (out_err_x != nullptr) *out_err_x = err_x;
    if (out_err_y != nullptr) *out_err_y = err_y;
    return sqrt(err_x * err_x + err_y * err_y);
}

double getHeightBetweenPoints(double *out_err_z = nullptr) {
    double err_z = pose.pose.position.z - local_pos.pose.pose.position.z;
    if (out_err_z != nullptr) *out_err_z = err_z;
    return fabs(err_z);
}

void end_fly_task()
{
    fsm_state = 100;
}

void fly_to_point(double x, double y, double z,double stop_time)
{
    if(getLengthBetweenPoints() < 0.1 && getHeightBetweenPoints() < 0.1 && ros::Time::now() - last_request > ros::Duration(1.0))
    {
        if(point_arrive_flag == false){
            point_arrive_flag = true;
            last_request = ros::Time::now();
        }else{
            if(ros::Time::now() - last_request > ros::Duration(stop_time))
            {
                point_arrive_flag = false;
                last_request = ros::Time::now();
                fly_task_state += 1;
                std::cout << "\033[32mComplete a fly task\033[0m" << std::endl;
            }
        }
    }else{

    }
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher fly_task_pub = nh.advertise<std_msgs::Int32>("fly_task", 1);

    ros::Subscriber fly_target_sub = nh.subscribe("fly_target", 10, fly_target_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::NodeHandle param_nh("~");
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    last_request = ros::Time::now();
    pose.pose.position.x =init_position_x_take_off + 0;
    pose.pose.position.y =init_position_y_take_off + 0;
    pose.pose.position.z =init_position_z_take_off + ALTITUDE;

    while (ros::ok()) {
        if(fsm_state != 0)
        {
            if(current_state.mode == "ALTCTL")
            {
                fly_task = 1;
            }else if (current_state.mode == "POINT")
            {
                fly_task = 2;
            }
            std_msgs::Int32 task_msg;
            task_msg.data = fly_task;
            fly_task_pub.publish(task_msg);
        }

        switch (fsm_state) {
            case 0:
                if(current_state.mode != "OFFBOARD"){
                    fsm_state = 1;
                    std::cout << "\033[32mReached Arm State.\033[0m" << std::endl;
                }
                break;
            case 1:
                if(current_state.armed){
                    fsm_state = 2;
                    std::cout << "\033[32mReached Takeoff State.\033[0m" << std::endl;
                }
                break;
            case 2:
                if(getHeightBetweenPoints() < 0.1 && ros::Time::now() - last_request > ros::Duration(1.0))
                {
                    fsm_state = 3;
                    std::cout << "\033[32mReached Fly State.\033[0m" << std::endl;
                }else{
                    pose.pose.position.x = init_position_x_take_off;
                    pose.pose.position.y = init_position_y_take_off;
                    pose.pose.position.z = init_position_z_take_off + ALTITUDE;
                }
                break;
            case 3:
                run_fly_task1();
                break;
            case 100:
                if(getLengthBetweenPoints() < 0.1 && getHeightBetweenPoints() < 0.1   && ros::Time::now() - last_request > ros::Duration(1.0))
                {
                    fsm_state = 101;
                    last_request = ros::Time::now();
                }else{
                    pose.pose.position.x = init_position_x_take_off + END_X;
                    pose.pose.position.y = init_position_y_take_off + END_Y;
                    pose.pose.position.z = init_position_z_take_off + END_Z;
                }
                break;
            case 101:
                if(current_state.mode == "AUTO.LAND"){
                    fsm_state = -1; 
                }else{
                    mavros_msgs::SetMode land_set_mode;
                    land_set_mode.request.custom_mode = "AUTO.LAND";
                    set_mode_client.call(land_set_mode);
                    last_request = ros::Time::now();
                }
                break;
            default:
                if (fsm_state != -1) {
                    ROS_FATAL("FATAL ERROR: FSM reaches an invalid state: %d, emergency landing.", fsm_state);
                    fsm_state = 101;
                }
                break;
            }

            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
    return 0;
}
