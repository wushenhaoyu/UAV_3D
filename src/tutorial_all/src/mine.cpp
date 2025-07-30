
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
#include <mavros_msgs/PositionTarget.h>
#include <tutorial_vision/CircleDetectResult.h>
#include <tutorial_vision/StringStamped.h>
#include <tutorial_serial/SerialData.h>
#include <cmath>
#include <std_msgs/Bool.h>
#include <tutorial_vision/ObjectError.h> 
#include <tutorial_serial/WayPoint.h> 
#define ALTITUDE  1.1
#define END_X 0
#define END_Y 0
#define END_Z 0.5

#define X_POS_P 0.5
#define X_POS_D 0.0001
#define Y_POS_P 0.5
#define Y_POS_D 0.0001

#define MAX_VELOCITY 0.25
#define PIXEL_TO_METER 0.0014

/*#define END_X 0
#define END_Y 0
#define END_Z 1.4*/

int fsm_state = 0;
int fly_task_state = 0;
int fly_ctrl_state = 0;
int fly_task = 1;
int8_t fly_target = 0x00;


mavros_msgs::State current_state;
ros::Time last_request;

tf::Quaternion quat; 
double roll, pitch, yaw;
double yaw_correct;
double yaw_target;
double initial_yaw = 0.0;
int yaw_symbol = 0;
bool flag_init_yaw = false;

float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
bool flag_init_position = false;
bool flag_vision_ready = false;
int vision_ready_count = 0;

nav_msgs::Odometry local_pos;
mavros_msgs::PositionTarget setpoint_raw;
bool point_arrive_flag = false;

ros::Publisher true_pos_pub;
ros::Publisher yaw_symbol_pub;
ros::Publisher local_pos_pub;
ros::Publisher camera_en_pub ;
ros::Publisher serial_pub;
void object_track_xy(int type);
void vision_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!flag_vision_ready) {
	vision_ready_count++;
	if(vision_ready_count >= 10){
		        flag_vision_ready = true;
        ROS_INFO("vision_pose received. Now ready to init position.");
	}
    }
}

double normalize_angle(double angle_rad) {
    while (angle_rad > M_PI) {
        angle_rad -= 2.0 * M_PI;
    }
    while (angle_rad < -M_PI) {
        angle_rad += 2.0 * M_PI;
    }
    return angle_rad;
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg) {
     if (!flag_vision_ready) return;
    local_pos = *msg;
    if (!flag_init_position && local_pos.pose.pose.position.z != 0) {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        flag_init_position = true;
	ROS_INFO("fly_point record: ENU(%.2f, %.2f, %.2f)", init_position_x_take_off, init_position_y_take_off, init_position_z_take_off);
    }
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);	
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (!flag_init_yaw) {
        initial_yaw = yaw;
        flag_init_yaw = true;
        ROS_INFO("Initial yaw recorded: %.2f deg", initial_yaw * 180.0 / M_PI);
    }
    yaw_correct = normalize_angle(yaw - initial_yaw);

    geometry_msgs::PoseStamped true_pos_msg;
    true_pos_msg.header = local_pos.header;
    true_pos_msg.pose.position.x = local_pos.pose.pose.position.x - init_position_x_take_off;
    true_pos_msg.pose.position.y = local_pos.pose.pose.position.y - init_position_y_take_off;
    true_pos_msg.pose.position.z = local_pos.pose.pose.position.z - init_position_z_take_off;

    // 将偏航角转换为四元数
    tf::Quaternion yaw_quat;
    yaw_quat.setRPY(0, 0, yaw_correct);
    true_pos_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_correct);

    true_pos_pub.publish(true_pos_msg);

//	ROS_INFO("yaw:%.3f",(yaw - initial_yaw) * 180 / M_PI);
    
   // ROS_INFO("Body position: (%.2f, %.2f, %.2f),ENU position: (%.2f, %.2f, %.2f),True_ENU position:(%.2f, %.2f, %.2f)",x_body,y_body,local_pos.pose.pose.position.z,x_enu,y_enu,local_pos.pose.pose.position.z,local_pos.pose.pose.position.x,local_pos.pose.pose.position.y,local_pos.pose.pose.position.z);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}


std::vector<tutorial_serial::WayPoint> waypoint_list;

void clear_waypoint_cb(const std_msgs::UInt8::ConstPtr& msg) {
    waypoint_list.clear();
    ROS_INFO("Waypoints cleared.");
}

void waypoint_request_cb(const tutorial_serial::WayPoint::ConstPtr& msg) {
    tutorial_serial::WayPoint waypoint = *msg;
    waypoint_list.push_back(waypoint);
    ROS_INFO("Waypoint requested: (%d, %d)", waypoint.x, waypoint.y);
}

tutorial_vision::ObjectError object_error;
ros::Time object_error_time;
void object_error_cb(const tutorial_vision::ObjectError::ConstPtr& msg)
{
    object_error = *msg;   // 直接拷贝整条消息
    object_error_time = ros::Time::now();
    /*ROS_INFO("""Object Error: type=%d, x=%.5f, y=%.5f", 
             object_error.type, object_error.x * PIXEL_TO_METER, object_error.y * PIXEL_TO_METER);*/
}
 
double getAngleBetweenPoints(double* out_err = nullptr) {
    double diff = yaw_target - yaw_correct;

    // 将 diff 归一化到 [-π, π]
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;

    double degree_diff = std::abs(diff) * 180.0 / M_PI;

    if (out_err) *out_err = degree_diff;
    return degree_diff;
}


double getLengthBetweenPoints(double *out_err_x = nullptr, double *out_err_y = nullptr) {
    double err_x = setpoint_raw.position.x - local_pos.pose.pose.position.x;
    double err_y = setpoint_raw.position.y - local_pos.pose.pose.position.y;
    if (out_err_x) *out_err_x = err_x;
    if (out_err_y) *out_err_y = err_y;
    return sqrt(err_x * err_x + err_y * err_y);
}

double getHeightBetweenPoints(double *out_err_z = nullptr) {
    double err_z = setpoint_raw.position.z - local_pos.pose.pose.position.z;
    if (out_err_z) *out_err_z = err_z;
    return fabs(err_z);
}

void camera_turn_forward(){ 
    tutorial_serial::SerialData serial_msg;
    serial_msg.func = 4;
    serial_msg.data = 0;
    serial_pub.publish(serial_msg);
}

void camera_turn_bottom(){
    tutorial_serial::SerialData serial_msg;
    serial_msg.func = 4;
    serial_msg.data = 1;
    serial_pub.publish(serial_msg);
}

void servo1_turn_forward(){ //一号舵机 1是投放 2是关
    tutorial_serial::SerialData serial_msg;
    serial_msg.func = 5;
    serial_msg.data = 1;
    serial_pub.publish(serial_msg);
}

void servo1_turn_bottom(){
    tutorial_serial::SerialData serial_msg;
    serial_msg.func = 5;
    serial_msg.data = 0;
    serial_pub.publish(serial_msg);
}

void servo2_turn_off(){ //二号舵机 1是投放 2是关
    tutorial_serial::SerialData serial_msg;
    serial_msg.func = 6;
    serial_msg.data = 0;
    serial_pub.publish(serial_msg);
}

void servo2_turn_on(){ //打开仓门投放
    tutorial_serial::SerialData serial_msg;
    serial_msg.func = 6;
    serial_msg.data = 1;
    serial_pub.publish(serial_msg);
}

void motors_turn_on(){//下放物品
    tutorial_serial::SerialData serial_msg;
    serial_msg.func = 7;
    serial_msg.data = 0;
    serial_pub.publish(serial_msg);;
}

void motors_turn_off(){ //上拉物品
    tutorial_serial::SerialData serial_msg;
    serial_msg.func = 7;
    serial_msg.data = 1;
    serial_pub.publish(serial_msg);
}


void laser_turn_forward(){//开激光
    tutorial_serial::SerialData serial_msg;
    serial_msg.func = 8;
    serial_msg.data = 1;
    serial_pub.publish(serial_msg);;
}

void laser_turn_bottom(){ //关激光
    tutorial_serial::SerialData serial_msg;
    serial_msg.func = 8;
    serial_msg.data = 0;
    serial_pub.publish(serial_msg);
}

void camera_enable(){
    std_msgs::Bool task_msg;
    task_msg.data = true;
    camera_en_pub.publish(task_msg);
    ROS_INFO("Camera enabled.");
}

void camera_disable(){
    std_msgs::Bool task_msg;
    task_msg.data = false;
    camera_en_pub.publish(task_msg);
    ROS_INFO("Camera disabled.");
}

void set_position_mode(){
    fly_ctrl_state = 0;
    setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 2048 ;
}

void set_velocity_xy_mode(){
    fly_ctrl_state = 1;
    setpoint_raw.type_mask = 1 + 2 + 32 + 64 + 128 + 256 + 512;
    setpoint_raw.velocity.x = 0; 
    setpoint_raw.velocity.y = 0; 
}

void set_velocity_yz_mode(){
    fly_ctrl_state = 2;
    setpoint_raw.type_mask = 8 + 16 + 4 + 64 + 128 + 256 + 512;
    setpoint_raw.velocity.z = 0; 
    setpoint_raw.velocity.y = 0; 
}

float limit_velocity(float velocity)
{
    if(velocity > MAX_VELOCITY)
    {
        velocity = MAX_VELOCITY;
    }else if(velocity < -MAX_VELOCITY)
    {
        velocity = -MAX_VELOCITY;
    }
    return velocity;
}


float error_x;
float error_y;
float last_error_x;
float last_error_y;
float distance_threshold = 0.05;
void pld_cal_xy()
{
    static float last_error_distance = 0;
    if(error_x < distance_threshold && error_x > -1 * distance_threshold)
    {
        error_x = 0;
    }
    if(error_y < distance_threshold && error_y > -1 *  distance_threshold)
    {
        error_y = 0;
    }

    last_error_x = error_x;
    last_error_y = error_y;

    setpoint_raw.velocity.x = limit_velocity(error_x * X_POS_P + (error_x - last_error_x) * X_POS_D);
    setpoint_raw.velocity.y = limit_velocity(error_y * Y_POS_P + (error_y - last_error_y) * Y_POS_D);
    //ROS_INFO("Velocity: x=%.5f, y=%.5f", error_x * X_POS_P + (error_x - last_error_x) * X_POS_D , error_y * Y_POS_P + (error_y - last_error_y) * Y_POS_D);
}

bool object_position_init_xy_flag = false;
bool set_position_flag = false;
float init_position_x_object_position_xy = 0;
float init_position_y_object_position_xy = 0;
void object_track_xy(int type)
{
    if(type == object_error.type && (ros::Time::now() - object_error_time).toSec() < 0.5){
        if(object_position_init_xy_flag  == false)
        {
            init_position_x_object_position_xy = local_pos.pose.pose.position.x;
            init_position_y_object_position_xy = local_pos.pose.pose.position.y;
            object_position_init_xy_flag = true;
        }
        error_x = object_error.x * PIXEL_TO_METER;
        error_y = object_error.y * PIXEL_TO_METER;
        set_velocity_xy_mode();
        pld_cal_xy();
    }else if((ros::Time::now() - object_error_time).toSec() > 2){
        fly_task_state += 1;
        object_position_init_xy_flag = false;
        set_position_flag = false;
        set_position_mode();
    }
    else{
        set_position_mode();
        if(set_position_flag == false){
            setpoint_raw.position.x = local_pos.pose.pose.position.x;
            setpoint_raw.position.y = local_pos.pose.pose.position.y;
            set_position_flag  = true;
        }
    }

}

/*
 * @brief : 转化对应 yaw，0 为起飞前方（+X），1 为后方（-X），2 为左方（+Y），3 为右方（-Y）
 * @param : direction 方向编号
 * @return: yaw 弧度 [−π, π]
 */
double trans_yaw(int direction)
{
    std_msgs::Int32 sybmol;
    sybmol.data = direction;
    yaw_symbol_pub.publish(sybmol);
    switch (direction) {
        case 0:  // 前方（+X）
            return 0.0;
        case 1:  // 后方（-X）
            return M_PI;
        case 2:  // 左方（+Y）
            return M_PI_2;  // M_PI / 2
        case 3:  // 右方（-Y）
            return -M_PI_2;  // -M_PI / 2
        default:
            ROS_WARN("Invalid direction: %d. Defaulting to 0 yaw.", direction);
            return 0.0;
    }
}

void change_yaw(int direction){
    if(getAngleBetweenPoints() < 4 &&  ros::Time::now() - last_request > ros::Duration(4.0) && getLengthBetweenPoints() < 0.1)
    {
        fly_task_state++;
        last_request = ros::Time::now();
        ROS_INFO("\033[32mComplete a yaw change\033[0m");
    }else{
        yaw_target = trans_yaw(direction);
	    double yaw_target_t = initial_yaw + yaw_target;
        setpoint_raw.yaw = yaw_target_t;
//	ROS_INFO("yaw_eror:%f,yaw_target:%f,yaw_correct:%f\n",getAngleBetweenPoints(),yaw_target,yaw_correct);
    }

}


void set_xy_velocity(double x, double y , double t) {
    if(fly_task_state != 1)
    {
        set_velocity_xy_mode();
    }
    setpoint_raw.velocity.x = x;
    setpoint_raw.velocity.y = y;
    if (ros::Time::now() - last_request > ros::Duration(t)) {
        fly_task_state++;
        last_request = ros::Time::now();
    }
}

void end_fly_task() {
    fsm_state = 100;
}

void fly_to_point(double x, double y, double z, double stop_time) {
    if(fly_task_state != 0)
    {
        set_position_mode();
    }
    setpoint_raw.position.x = init_position_x_take_off + x;
    setpoint_raw.position.y = init_position_y_take_off + y;
    setpoint_raw.position.z = init_position_z_take_off + z;
    if (getLengthBetweenPoints() < 0.1 && getHeightBetweenPoints() < 0.1 && ros::Time::now() - last_request > ros::Duration(1.0)) {
        if (!point_arrive_flag) {
            point_arrive_flag = true;
            last_request = ros::Time::now();
            camera_enable();
        } else if (ros::Time::now() - last_request > ros::Duration(stop_time)) {
            point_arrive_flag = false;
            last_request = ros::Time::now();
            fly_task_state++;
            camera_disable();
            ROS_INFO("\033[32mComplete a fly task\033[0m");
        }
    }
}

void run_fly_task()
{
    if (waypoint_list.empty()) {
        ROS_WARN("Waypoint list is empty, ending task.");
        end_fly_task();
        return;
    }

    // 已经全部飞完
    if (fly_task_state >= waypoint_list.size()) {
        end_fly_task();
        return;
    }

    // 继续飞往当前目标点
    const auto& wp = waypoint_list[fly_task_state];
    fly_to_point(wp.x, wp.y, ALTITUDE, 1.0);
}
void test(){
        switch (fly_task_state) {
        case 0: set_xy_velocity(0.4 , 0 , 3 );break;
        case 1: set_xy_velocity(0 , 0 , 3 );break;
        case 2: end_fly_task();break;
        default: break;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    ros::Publisher fly_task_pub = nh.advertise<std_msgs::Int32>("fly_task", 1);
    serial_pub = nh.advertise<tutorial_serial::SerialData>("serial_ctrl", 10);
    camera_en_pub = nh.advertise<std_msgs::Bool>("camera_en", 1);
    local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    yaw_symbol_pub = nh.advertise<std_msgs::Int32>("yaw_symbol", 1);
    true_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("true_position", 10);

    ros::Subscriber object_error_sub = nh.subscribe("/object_error",10,object_error_cb);
    ros::Subscriber state_sub = nh.subscribe("mavros/state", 10, state_cb);
    ros::Subscriber odom_sub = nh.subscribe("/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber vision_pose_sub = nh.subscribe("/mavros/vision_pose/pose", 10, vision_pos_cb);
    ros::Subscriber clear_waypoint_sub = nh.subscribe("clear_waypoint", 10,clear_waypoint_cb);
    ros::Subscriber waypoint_request_sub = nh.subscribe("waypoint_request", 10,waypoint_request_cb); 

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    while (ros::ok() && !current_state.connected) {
        ros::spinOnce(); rate.sleep();
    }

    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(setpoint_raw);
        ros::spinOnce(); 
        rate.sleep();
    }


    last_request = ros::Time::now();
    setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 2048 ;
	setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint_raw.position.x = init_position_x_take_off + 0;
    setpoint_raw.position.y = init_position_y_take_off + 0;
    setpoint_raw.position.z = init_position_z_take_off + ALTITUDE;

    while (ros::ok()) {
        /*if(fsm_state == 0)
        {
            if(current_state.mode == "ALTCTL")
            {
                fly_task = 1;
            }else if (current_state.mode == "POSCTL")
            {
                fly_task = 2;
            }
            std_msgs::Int32 task_msg;
            task_msg.data = fly_task;
            fly_task_pub.publish(task_msg);
            
        }*/

        switch (fsm_state) {
            case 0:
                if (current_state.mode == "OFFBOARD") {
                    fsm_state = 1; last_request = ros::Time::now();
                    ROS_INFO("\033[32mReached Arm State.\033[0m");
                }
                break;
            case 1:
                if (current_state.armed) {
                    fsm_state = 2; last_request = ros::Time::now();
                    ROS_INFO("\033[32mReached Takeoff State.\033[0m");
                }
                break;
            case 2:
                if (getHeightBetweenPoints() < 0.1 && ros::Time::now() - last_request > ros::Duration(1.0)) {
                    fsm_state = 3; last_request = ros::Time::now();
                    ROS_INFO("\033[32mReached Fly State.\033[0m");
		        if(fly_task == 1){
                ROS_INFO("\033[32mFly Task 1 starting!\033[0m");
                }else if(fly_task == 2){
                ROS_INFO("\033[32mFly Task 2 starting!\033[0m");
                }
                } else {
                    setpoint_raw.position.x = init_position_x_take_off + 0;
                    setpoint_raw.position.y = init_position_y_take_off + 0;
                    setpoint_raw.position.z = init_position_z_take_off + ALTITUDE;
                }
                break;
            case 3:
                    run_fly_task();
                break;
            case 100:
                set_position_mode();
                if(getLengthBetweenPoints() < 0.1 && getHeightBetweenPoints() < 0.1   && ros::Time::now() - last_request > ros::Duration(8.0))
                {
                    fsm_state = 101;
                    last_request = ros::Time::now();
                }else{
                    setpoint_raw.position.x = init_position_x_take_off + END_X;
                    setpoint_raw.position.y = init_position_y_take_off + END_Y;
                    setpoint_raw.position.z = init_position_z_take_off + ALTITUDE;
                }
                break;
	        case 101:
                if(getLengthBetweenPoints() < 0.1 && getHeightBetweenPoints() < 0.1   && ros::Time::now() - last_request > ros::Duration(4.0))
                {
                    fsm_state = 102;
                    last_request = ros::Time::now();
                }else{
                    setpoint_raw.position.x = init_position_x_take_off + END_X;
                    setpoint_raw.position.y = init_position_y_take_off + END_Y;
                    setpoint_raw.position.z = init_position_z_take_off + END_Z;
                }
                break;
            case 102:
                if(getLengthBetweenPoints() < 0.1 && fabs(init_position_z_take_off - local_pos.pose.pose.position.z) < 0.05 )
                {
                    fsm_state = 103;
                    last_request = ros::Time::now();
                }else{
                    setpoint_raw.position.x = init_position_x_take_off + END_X;
                    setpoint_raw.position.y = init_position_y_take_off + END_Y;
                    setpoint_raw.position.z = init_position_z_take_off - 0.15 ;
                }
                break;
            case 103:
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
                    ROS_FATAL("FSM in invalid state: %d, emergency landing.", fsm_state);
                    fsm_state = 101;
                }
                break;
        }

        if(fly_ctrl_state == 0){
            local_pos_pub.publish(setpoint_raw);
        }else if (fly_ctrl_state == 1)
        {
            local_pos_pub.publish(setpoint_raw);
        }
        
        ros::spinOnce(); 
        rate.sleep();
    }
    return 0;
}
