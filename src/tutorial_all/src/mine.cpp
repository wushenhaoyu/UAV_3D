
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
#include <cmath>

#define ALTITUDE  1.4
#define END_X 0.0
#define END_Y 0.0
#define END_Z 0.5

int fsm_state = 0;
int fly_task_state = 0;
int fly_task = 1;

mavros_msgs::State current_state;
ros::Time last_request;

tf::Quaternion quat; 
double roll, pitch, yaw;
double yaw_correct;
double yaw_target;
double initial_yaw = 0.0;
bool flag_init_yaw = false;

float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
bool flag_init_position = false;
bool flag_vision_ready = false;
int vision_ready_count = 0;

nav_msgs::Odometry local_pos;
geometry_msgs::PoseStamped pose;
bool point_arrive_flag = false;


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

//	ROS_INFO("yaw:%.3f",(yaw - initial_yaw) * 180 / M_PI);
    
   // ROS_INFO("Body position: (%.2f, %.2f, %.2f),ENU position: (%.2f, %.2f, %.2f),True_ENU position:(%.2f, %.2f, %.2f)",x_body,y_body,local_pos.pose.pose.position.z,x_enu,y_enu,local_pos.pose.pose.position.z,local_pos.pose.pose.position.x,local_pos.pose.pose.position.y,local_pos.pose.pose.position.z);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void fly_target_cb(const std_msgs::UInt8::ConstPtr& msg) {
    ROS_INFO("\033[34mRecieve fly_target: %d\033[0m", msg->data);
    fly_task = msg->data;
}

double getAngleBetweenPoints(double* out_err = nullptr) {
    double diff = normalize_angle(yaw_target - yaw_correct) * 180 / (2 * M_PI);
    if (out_err) *out_err = diff;
    return std::abs(diff);
}
double getLengthBetweenPoints(double *out_err_x = nullptr, double *out_err_y = nullptr) {
    double err_x = pose.pose.position.x - local_pos.pose.pose.position.x;
    double err_y = pose.pose.position.y - local_pos.pose.pose.position.y;
    if (out_err_x) *out_err_x = err_x;
    if (out_err_y) *out_err_y = err_y;
    return sqrt(err_x * err_x + err_y * err_y);
}

double getHeightBetweenPoints(double *out_err_z = nullptr) {
    double err_z = pose.pose.position.z - local_pos.pose.pose.position.z;
    if (out_err_z) *out_err_z = err_z;
    return fabs(err_z);
}

/*
 * @brief : 转化对应 yaw，0 为起飞前方（+X），1 为后方（-X），2 为左方（+Y），3 为右方（-Y）
 * @param : direction 方向编号
 * @return: yaw 弧度 [−π, π]
 */
double trans_yaw(int direction)
{
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
    if(getAngleBetweenPoints() < 4 &&  ros::Time::now() - last_request > ros::Duration(1.0))
    {
        fly_task_state++;
        last_request = ros::Time::now();
        ROS_INFO("\033[32mComplete a yaw change\033[0m");
    }else{
        yaw_target = trans_yaw(direction);
        tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, yaw_target);
        tf::quaternionTFToMsg(q, pose.pose.orientation);
    }
}


void end_fly_task() {
    fsm_state = 100;
}

void fly_to_point(double x, double y, double z, double stop_time) {
    pose.pose.position.x = init_position_x_take_off + x;
    pose.pose.position.y = init_position_y_take_off + y;
    pose.pose.position.z = init_position_z_take_off + z;

    if (getLengthBetweenPoints() < 0.1 && getHeightBetweenPoints() < 0.1 && ros::Time::now() - last_request > ros::Duration(1.0)) {
        if (!point_arrive_flag) {
            point_arrive_flag = true;
            last_request = ros::Time::now();
        } else if (ros::Time::now() - last_request > ros::Duration(stop_time)) {
            point_arrive_flag = false;
            last_request = ros::Time::now();
            fly_task_state++;
            ROS_INFO("\033[32mComplete a fly task\033[0m");
        }
    }
}

void run_fly_task1() {
    switch (fly_task_state) {
        case 0: fly_to_point(0.5, 0.0, ALTITUDE, 2); break;
        case 1: fly_to_point(0.5, 0.5, ALTITUDE, 2); break;
        case 2: fly_to_point(0.0, 0.5, ALTITUDE, 2); break;
        case 3: end_fly_task(); break;
        default: break;
    }
}

void run_fly_task2() {
    switch (fly_task_state) {
        case 0: change_yaw(2); break;
        case 1: change_yaw(1); break;
        case 2: change_yaw(3); break;
        case 3: change_yaw(0); break;
        case 4: end_fly_task();break;
        default: break;
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher fly_task_pub = nh.advertise<std_msgs::Int32>("fly_task", 1);

    ros::Subscriber state_sub = nh.subscribe("mavros/state", 10, state_cb);
    ros::Subscriber odom_sub = nh.subscribe("/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber fly_target_sub = nh.subscribe("fly_target", 10, fly_target_cb);
    ros::Subscriber vision_pose_sub = nh.subscribe("/mavros/vision_pose/pose", 10, vision_pos_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    while (ros::ok() && !current_state.connected) {
        ros::spinOnce(); rate.sleep();
    }

    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce(); rate.sleep();
    }

    last_request = ros::Time::now();
    pose.pose.position.x = init_position_x_take_off + 0;
    pose.pose.position.y = init_position_y_take_off + 0;
    pose.pose.position.z = init_position_z_take_off + ALTITUDE;

    while (ros::ok()) {
        if(fsm_state == 0)
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
                } else {
                    pose.pose.position.x = init_position_x_take_off + 0;
                    pose.pose.position.y = init_position_y_take_off + 0;
                    pose.pose.position.z = init_position_z_take_off + ALTITUDE;
                }
                break;
            case 3:
                run_fly_task2(); break;
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
                    ROS_FATAL("FSM in invalid state: %d, emergency landing.", fsm_state);
                    fsm_state = 101;
                }
                break;
        }


        local_pos_pub.publish(pose);
        ros::spinOnce(); rate.sleep();
    }
    return 0;
}
