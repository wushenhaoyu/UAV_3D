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

#include <tutorial_vision/CircleDetectResult.h>
#include <tutorial_vision/StringStamped.h>

#define POS_P 0.4
#define POS_I 0.00
#define POS_D 0.0
#define YAW_POS_P 1.2
#define YAW_POS_I 0.0
#define YAW_POS_D 0.000
#define PIX_POS_P 0.001
#define PIX_POS_I 0.0
#define PIX_POS_D 0.0

double getLengthBetweenPoints(geometry_msgs::Point a, double x, double y, double z,
                              double *out_err_x = nullptr, double *out_err_y = nullptr, double *out_err_z = nullptr) {
    double err_x = a.x - x;
    double err_y = a.y - y;
    double err_z = a.z - z;
    if (out_err_x != nullptr) *out_err_x = err_x;
    if (out_err_y != nullptr) *out_err_y = err_y;
    if (out_err_z != nullptr) *out_err_z = err_z;
    return sqrt(err_x * err_x + err_y * err_y + err_z * err_z);
}

double getLengthBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b,
                              double *out_err_x = nullptr, double *out_err_y = nullptr, double *out_err_z = nullptr) {
    return getLengthBetweenPoints(a, b.x, b.y, b.z, out_err_x, out_err_y, out_err_z);
}

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
geometry_msgs::Vector3 current_rpy;

bool yaw_initialized = false;
double initial_yaw = 0.0;

geometry_msgs::Twist get_pid_vel(geometry_msgs::Point target);

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // 提取四元数并计算当前 yaw
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(msg->pose.orientation, quaternion);
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    // 初始化起飞方向（只记录一次）
    if (!yaw_initialized) {
        initial_yaw = yaw;
        yaw_initialized = true;
        ROS_INFO("Yaw initialized at: %.2f degrees", initial_yaw * 180.0 / M_PI);
    }

    // === 坐标修正（ENU → 起飞系）===
    double x_enu = msg->pose.position.x;
    double y_enu = msg->pose.position.y;
    double x_corr = cos(-initial_yaw) * x_enu - sin(-initial_yaw) * y_enu;
    double y_corr = sin(-initial_yaw) * x_enu + cos(-initial_yaw) * y_enu;

    // === 姿态修正 ===
    double yaw_corr = yaw - initial_yaw;
    while (yaw_corr > M_PI) yaw_corr -= 2 * M_PI;
    while (yaw_corr < -M_PI) yaw_corr += 2 * M_PI;

    // === 更新 corrected_pose 和 current_rpy ===
    geometry_msgs::PoseStamped corrected_pose = *msg;
    corrected_pose.pose.position.x = x_corr;
    corrected_pose.pose.position.y = y_corr;

    tf::Quaternion corrected_quat;
    corrected_quat.setRPY(roll, pitch, yaw_corr);
    tf::quaternionTFToMsg(corrected_quat, corrected_pose.pose.orientation);

    current_pose = corrected_pose;
    current_rpy.x = roll;
    current_rpy.y = pitch;
    current_rpy.z = yaw_corr;

    // === 调用 PID 控制 ===
    geometry_msgs::Point target_point;
    target_point.x = 0;
    target_point.y = 0;
    target_point.z = 0;

 // geometry_msgs::Twist cmd_vel = get_pid_vel(target_point);
}

/*
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
geometry_msgs::Vector3 current_rpy;
geometry_msgs::Twist get_pid_vel(geometry_msgs::Point target);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_pose = *msg;
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(msg->pose.orientation, quaternion);
    tf::Matrix3x3(quaternion).getRPY(current_rpy.x, current_rpy.y, current_rpy.z);
        geometry_msgs::Point target_point;
    target_point.x = 0;
    target_point.y = 0;
    target_point.z = 0;

    // 定义变量接收get_pid_vel函数的返回值
    geometry_msgs::Twist a;
    a = get_pid_vel(target_point);
}*/
geometry_msgs::Twist move_base_twist;
void move_base_cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &msg) {
    move_base_twist = *msg;
}

int8_t fly_target = 0x00;
void fly_target_cb(const std_msgs::UInt8::ConstPtr& msg) {
    ROS_INFO("接收到 fly_target: %d (十六进制: 0x%X)", msg->data, msg->data);
    fly_target = msg->data;
}

geometry_msgs::Twist current_vel;

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    current_vel = msg->twist;  // 包含线速度（x, y, z）和角速度
}




geometry_msgs::Point last_err;
geometry_msgs::Point err_sum;
double last_yaw_err = 0.;
double yaw_err_sum = 0.;
ros::Time last_pid_control_time;
double target_yaw = 0;
ros::Duration dt;
geometry_msgs::Twist get_pid_vel(geometry_msgs::Point target) {
    ros::Time currentStamp = current_pose.header.stamp;
    if((currentStamp - last_pid_control_time).toSec() < 0.04)
    {
        dt = ros::Duration(0.1);
    }else{
        dt = currentStamp - last_pid_control_time;
    }
        if (dt.toSec() > 0.2) {
        err_sum.x = 0.;
        err_sum.y = 0.;
        err_sum.z = 0.;
        yaw_err_sum = 0.;
    }
	ROS_INFO("dt: %.6f", dt.toSec());

    geometry_msgs::Point err;
    double absErr = getLengthBetweenPoints(target, current_pose.pose.position, &err.x, &err.y, &err.z);
    double y_err = normalize_angle(target_yaw - current_rpy.z);
    double dy_err = (y_err - last_yaw_err) / dt.toSec();

    geometry_msgs::Twist ret;
    ret.angular.z = YAW_POS_P * y_err + YAW_POS_I * yaw_err_sum + YAW_POS_D * dy_err;
    ROS_INFO("dy_err:%f\n",dy_err);
    if (absErr > 0.8) {
        // 归一化直线接近目标
        ret.linear.x = err.x * 0.8 / absErr;
        ret.linear.y = err.y * 0.8 / absErr;
        ret.linear.z = err.z * 0.8 / absErr;

        err_sum.x = .0;
        err_sum.y = .0;
        err_sum.z = .0;
    } else {
        geometry_msgs::Point d_err;
        d_err.x = (err.x - last_err.x) / dt.toSec();
        d_err.y = (err.y - last_err.y) / dt.toSec();
        d_err.z = (err.z - last_err.z) / dt.toSec();

        ret.linear.x = POS_P * err.x + POS_I * err_sum.x + POS_D * d_err.x;
        ret.linear.y = POS_P * err.y + POS_I * err_sum.y + POS_D * d_err.y;
        ret.linear.z = POS_P * err.z + POS_I * err_sum.z + POS_D * d_err.z;

        err_sum.x += err.x * dt.toSec();
        err_sum.y += err.y * dt.toSec();
        err_sum.z += err.z * dt.toSec();
    }

    // === 将线速度从世界系旋转到机体坐标系 ===
    double yaw = current_rpy.z;
    double vx_world = ret.linear.x;
    double vy_world = ret.linear.y;
    ret.linear.x =  cos(-yaw) * vx_world - sin(-yaw) * vy_world;
    ret.linear.y =  sin(-yaw) * vx_world + cos(-yaw) * vy_world;

    // 限幅
    const double max_linear_velocity = 0.4; // 最大线速度
    const double max_angular_velocity = M_PI / 6.0; // 最大角速度（30°/s）

    ret.linear.x = std::max(-max_linear_velocity, std::min(max_linear_velocity, ret.linear.x));
    ret.linear.y = std::max(-max_linear_velocity, std::min(max_linear_velocity, ret.linear.y));
    ret.angular.z = std::max(-max_angular_velocity, std::min(max_angular_velocity, ret.angular.z));


    // 状态更新
    last_err = err;
    last_yaw_err = y_err;
    yaw_err_sum += y_err * dt.toSec();
    last_pid_control_time = currentStamp;
    ROS_INFO("pos[x=%.4f y=%.4f z=%.4f yaw=%.4f] err[x=%.4f y=%.4f z=%.4f yaw=%.4f] cmd[vx=%.4f vy=%.4f vz=%.4f wz=%.4f]",
         current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, current_rpy.z,
         err.x, err.y, err.z, y_err,
         ret.linear.x, ret.linear.y, ret.linear.z, ret.angular.z);
    	return ret;
}


/*geometry_msgs::Twist get_pid_vel(geometry_msgs::Point target) {
    ros::Time currentStamp = current_pose.header.stamp;
    ros::Duration dt = currentStamp - last_pid_control_time;
    if (dt.toSec() > 0.2) {
        err_sum.x = 0.;
        err_sum.y = 0.;
        err_sum.z = 0.;
        yaw_err_sum = 0.;
    }
    // 计算旋转矩阵
    double cos_theta = std::cos(current_rpy.z);
    double sin_theta = std::sin(current_rpy.z);
    // 将目标点坐标转换到局部坐标系
    double dx = target.x - current_pose.pose.position.x;
    double dy = target.y - current_pose.pose.position.y;
    double dz = target.z - current_pose.pose.position.z;
    double local_x = cos_theta * dx + sin_theta * dy;
    double local_y = -sin_theta * dx + cos_theta * dy;
    double local_z = dz;
    geometry_msgs::Point err;
    // 直接计算两点间距离和误差
    err.x = local_x - 0;
    err.y = local_y - 0;
    err.z = local_z - 0;
    double absErr = std::sqrt(err.x * err.x + err.y * err.y + err.z * err.z);
    double y_err = target_yaw - current_rpy.z;
    double dy_err = (y_err - last_yaw_err) / dt.toSec();
    geometry_msgs::Twist ret;
    ret.angular.z = YAW_VEL_P * y_err + YAW_VEL_I * yaw_err_sum + YAW_VEL_D * dy_err;
    if (absErr > 0.8) {
        ret.linear.x = err.x * 0.8 / absErr;
        ret.linear.y = err.y * 0.8 / absErr;
        ret.linear.z = err.z * 0.8 / absErr;
        err_sum.x = .0;
        err_sum.y = .0;
        err_sum.z = .0;
    } else {
        geometry_msgs::Point d_err;
        d_err.x = (err.x - last_err.x) / dt.toSec();
        d_err.y = (err.y - last_err.y) / dt.toSec();
        d_err.z = (err.z - last_err.z) / dt.toSec();
        ret.linear.x = VEL_P * err.x + VEL_I * err_sum.x + VEL_D * d_err.x;
        ret.linear.y = VEL_P * err.y + VEL_I * err_sum.y + VEL_D * d_err.y;
        ret.linear.z = VEL_P * err.z + VEL_I * err_sum.z + VEL_D * d_err.z;
        err_sum.x += err.x * dt.toSec();
        err_sum.y += err.y * dt.toSec();
        err_sum.z += err.z * dt.toSec();
    }
    last_err = err;
    last_yaw_err = y_err;
    yaw_err_sum += y_err * dt.toSec();
    last_pid_control_time = currentStamp;
    ROS_INFO("x:%f,y:%f,z:%f,yaw:%f\n", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,current_rpy.z);
    ROS_INFO("dx:%f,dy:%f,dz:%f,dyaw:%f\n", ret.linear.x, ret.linear.y, ret.linear.z , ret.angular.z);
    return ret;
}*/

geometry_msgs::Point pix_last_err;
geometry_msgs::Point pix_err_sum;
geometry_msgs::Twist get_pix_pid_vel(geometry_msgs::Point err) {
    ros::Time currentStamp = current_pose.header.stamp;
    ros::Duration dt = currentStamp - last_pid_control_time;
    if (dt.toSec() > 0.2) {
        pix_err_sum.x = 0.;
        pix_err_sum.y = 0.;
        yaw_err_sum = 0.;
    }

    double y_err = 0. - current_rpy.z;
    double dy_err = (y_err - last_yaw_err) / dt.toSec();

    geometry_msgs::Twist ret;
    ret.angular.z = YAW_POS_P * y_err + YAW_POS_I * yaw_err_sum + YAW_POS_D * dy_err;

    geometry_msgs::Point d_err;
    d_err.x = (err.x - pix_last_err.x) / dt.toSec();
    d_err.y = (err.y - pix_last_err.y) / dt.toSec();

    ret.linear.x = PIX_POS_P * err.y + PIX_POS_I * pix_err_sum.y + PIX_POS_D * d_err.y;
    ret.linear.y = PIX_POS_P * err.x + PIX_POS_I * pix_err_sum.x + PIX_POS_D * d_err.x;

    pix_err_sum.x += err.x * dt.toSec();
    pix_err_sum.y += err.y * dt.toSec();

    pix_last_err = err;
    last_yaw_err = y_err;
    yaw_err_sum += y_err * dt.toSec();
    last_pid_control_time = currentStamp;
    
    return ret;
}

ros::Time   last_srv_request;
void logTime()
{
    last_srv_request = ros::Time::now();
}

int fsm_state = 0;
int fly_task_state = 0;
bool point_arrive_flag = false;

/*
* @brief  飞到指定点
        ^ x
        |
        |
        |
Y<----------------
        |
        |
        |
        |
* @param  x,y,z 目标点
* @param  stop_time 停留时间
*/
geometry_msgs::Twist flyToPoint(double x, double y, double z, double stop_time)
{
    geometry_msgs::Point target_point;
    target_point.x = x;
    target_point.y = y;
    target_point.z = z;
    geometry_msgs::Twist twist;
    if(getLengthBetweenPoints(current_pose.pose.position, target_point) < 0.1) //ros::Time::now() - last_srv_request > ros::Duration(1.0) &&
    {
	ROS_INFO("stopping in the target point!");
        if(point_arrive_flag == false)
        {
            point_arrive_flag = true;
            logTime();
        }
        if (point_arrive_flag == true)
        {
            if(ros::Time::now() - last_srv_request > ros::Duration(stop_time))
            {
                fly_task_state += 1;
                point_arrive_flag = false;
                std::cout << "\033[32mComplete a fly task\033[0m" << std::endl;
                logTime();
            }
        }   
    }else{
//	ROS_INFO("Moving to target point!");
    }
    twist = get_pid_vel(target_point);
    return twist;
}

/*
 * @brief : 转化对应 yaw，0 为起飞前方（+X），1 为后方（-X），2 为左方（+Y），3 为右方（-Y）
 * @param : direction 方向编号
 * @return: yaw 弧度 [−π, π]
 */
double transYAW(int direction)
{
    switch (direction) {
        case 0:  // 前方（+Y）
            return 0.0;
        case 1:  // 后方（-Y）
            return M_PI;
        case 2:  // 左方（+X）
            return M_PI_2;  // M_PI / 2
        case 3:  // 右方（-X）
            return -M_PI_2;  // -M_PI / 2
        default:
            ROS_WARN("Invalid direction: %d. Defaulting to 0 yaw.", direction);
            return 0.0;
    }
}

/*
 * @brief : 改变目标 yaw，0 为起飞前方（+X），1 为后方（-X），2 为左方（+Y），3 为右方（-Y）
 * @param : direction 方向编号
 * @return: yaw 弧度 [−π, π]
 */

geometry_msgs::Twist changeYaw(double x, double y, double z, int target_direction)
{
    geometry_msgs::Twist twist;
    target_yaw = transYAW(target_direction); 
    geometry_msgs::Point target_point;
    target_point.x = x;
    target_point.y = y;
    target_point.z = z;
    if (getLengthBetweenPoints(current_pose.pose.position, target_point) < 0.1 && target_yaw - current_rpy.z < M_PI / 30 && target_yaw - current_rpy.z > -1 * M_PI / 30)
    {
        
                fly_task_state += 1;
                std::cout << "\033[32mComplete Yaw Change\033[0m" << std::endl;
                logTime(); 
    }else{
//	 std::cout << "\033[32mChanging Yaw~~~\033[0m" << std::endl;
    }
    twist = get_pid_vel(target_point);
    return twist;
}


geometry_msgs::Twist endFlyTask()
{
    fsm_state = 100;
    geometry_msgs::Twist twist;
    return twist;
}

geometry_msgs::Twist runFlyTask1()
/*测试飞行，一个方格子*/
{
    geometry_msgs::Twist twist;
    switch(fly_task_state)
    {
        case 0:
            twist = flyToPoint(0.5,0.0,1.0,1);
        break;
        case 1:
            twist = flyToPoint(0.5,0.5,1.0,1);
        break;
        case 2:
            twist = flyToPoint(0.0,0.5,1.0,1);
        break;
        case 3:
            twist = flyToPoint(0,0,1.0,1);
        break;
        case 4:
            endFlyTask();
        break;
    }
    return twist;
}

geometry_msgs::Twist runFlyTask2()
/*测试飞行，一个方格子*/
{
    geometry_msgs::Twist twist;
    switch(fly_task_state)
    {
        case 0:
            twist = changeYaw(0,0,1.0,2);
        break;
        case 1:
            twist = changeYaw(0,0,1.0,1);
        break;
        case 2:
            twist = changeYaw(0,0,1.0,3);
        break;
        case 3:
            twist = changeYaw(0,0,1.0,0);
        break;
        case 4:
            endFlyTask();
        break;
    }
    return twist;
}

geometry_msgs::Twist runFlyTask_2024_1()
{
    geometry_msgs::Twist twist;
    switch(fly_task_state)
    {
        case 0:
            twist = flyToPoint(2.0, 0.0, 1.4, 1); // 遍历前3、2、1点
        break;
        case 1:
            twist = flyToPoint(2.0, 0.0, 1.0, 1); // 下降高度
        break;
        case 2:
            twist = flyToPoint(-0.1, 0.0, 1.0, 1); // 遍历4、5、6点
        break;
        case 3:
            twist = flyToPoint(-0.1, 1.75, 1.0, 1); // 飞到中间
        break;
        case 4:
            twist = flyToPoint(2.0, 1.75, 1.0, 1); // 遍历18、17、16
        break;
        case 5:
            twist = flyToPoint(2.0, 1.75, 1.4, 1); // 上升高度
        break;
        case 6:
            twist = flyToPoint(0.0, 1.75, 1.4, 1); // 遍历13、14、15
        break;
        case 7:
            twist = changeYaw(0.0, 1.75, 1.4, 1); // 转180度
        break;
        case 8:
            twist = flyToPoint(2.0, 1.75, 1.4, 1); // 遍历7、8、9
        break;
        case 9:
            twist = flyToPoint(2.0, 1.75, 1.0, 1); // 下降高度
        break;
        case 10:
            twist = flyToPoint(-0.1, 1.75, 1.0, 1); // 遍历12、11、10
        break;
        case 11:
            twist = flyToPoint(-0.1, 3.5, 1.0, 1); // 到最后一个板子
        break;
        case 12:
            twist = flyToPoint(2.0, 3.5, 1.0, 1); // 遍历22、23、24
        break;
        case 13:
            twist = flyToPoint(2.0, 3.5, 1.4, 1); // 上升高度
        break;
        case 14:
            twist = flyToPoint(0.0, 3.5, 1.4, 1); // 遍历21、20、19
        break;
        case 15:
            endFlyTask();
        break;
    }
    return twist;
}

/*geometry_msgs::Twist runFlyTask_2024_2()
{
    geometry_msgs::Twist twist;
    
    if(fly_target > 0x00 && fly_target <= 0x06) // 1～6
    {
        if(fly_target <= 0x03) //1～3
        {
            switch (fly_task_state)
            {
            case 0:
                flyToPoint(0.0, 0.0, 1.4, 1);
            break;
            case 1:
                if(fly_target == 0x03){
                    flyToPoint(0.75, 0.0, 1.4, 4);
                }else if(fly_target == 0x02){
                    flyToPoint(1.25, 0.0, 1.4, 4);
                }else if (fly_target == 0x01){
                    flyToPoint(1.75, 0.0, 1.4, 4);
                }
            break;
            case 2:
                flyToPoint(-0.1, 0.0, 1.4, 1);
            break;
            case 3:
                flyToPoint(-0.1, 3.5, 1.4, 1);
            break;
            case 4:
                endFlyTask();
            break;  
            }
        }else if(fly_target > 0x03) //4～6 
        {
            switch (fly_task_state)
            {
            case 0:
                flyToPoint(0.0, 0.0, 1.0, 1);
            break;
            case 1:
                if(fly_target == 0x06){
                    flyToPoint(0.75, 0.0, 1.0, 4);
                }else if(fly_target == 0x05){
                    flyToPoint(1.25, 0.0, 1.0, 4);
                }else if (fly_target == 0x04){
                    flyToPoint(1.75, 0.0, 1.0, 4);
                }
            break;
            case 2:
                flyToPoint(-0.1, 0.0, 1.0, 1);
            break;
            case 3:
                flyToPoint(-0.1, 3.5, 1.0, 1);
            break;
            case 4:
                endFlyTask();
            break;  
            }
        }
    }else if(fly_target >= 0x07 && fly_target <= 0x18) //7～18
    {
        if(fly_target >= 0x07 && fly_target <= 0x12)
        {
            if(fly_target > 0x09)
            {
                switch (fly_task_state)
                {
                case 0:
                    flyToPoint(-0.1, 1.75, 1.0, 1);
                break;
                case 1:
                    changeYaw(-0.1, 1.75, 1.0, 1);
                break;
                case 2:
                    if(fly_target == 0x10){
                        flyToPoint(0.75, 1.75, 1.0, 4);
                    }else if(fly_target == 0x11){
                        flyToPoint(1.25, 1.75, 1.0, 4);
                    }else if (fly_target == 0x12){
                        flyToPoint(1.75, 1.75, 1.0, 4);
                    }
                break;
                case 3:
                    flyToPoint(-0.1, 1.75, 1.0, 1);
                break;
                case 4:
                    flyToPoint(-0.1, 3.5, 1.0, 1);
                break;
                case 5:
                    endFlyTask();
                break;  
                }
            }else if(fly_target <= 0x09)
            {
                switch (fly_task_state)
                {
                case 0:
                    flyToPoint(-0.1, 1.75, 1.4, 1);
                break;
                case 1:
                    changeYaw(-0.1, 1.75, 1.4, 1);
                break;
                case 2:
                    if(fly_target == 0x07){
                        flyToPoint(0.75, 1.75, 1.4, 4);
                    }else if(fly_target == 0x08){
                        flyToPoint(1.25, 1.75, 1.4, 4);
                    }else if (fly_target == 0x09){
                        flyToPoint(1.75, 1.75, 1.4, 4);
                    }
                break;
                case 3:
                    flyToPoint(-0.1, 1.75, 1.4, 1);
                break;
                case 4:
                    flyToPoint(-0.1, 3.5, 1.4, 1);
                break;
                case 5:
                    endFlyTask();
                break;  
                }
            }
        }else if(fly_target >= 0x13 && fly_target <= 0x18)
        {
            if(fly_target > 0x15)
            {
                switch (fly_task_state)
                {
                case 0:
                    flyToPoint(-0.1, 1.75, 1.0, 1);
                break;
                case 1:
                    if(fly_target == 0x16){
                        flyToPoint(0.75, 1.75, 1.0, 4);
                    }else if(fly_target == 0x17){
                        flyToPoint(1.25, 1.75, 1.0, 4);
                    }else if (fly_target == 0x18){
                        flyToPoint(1.75, 1.75, 1.0, 4);
                    }
                break;
                case 2:
                    flyToPoint(-0.1, 1.75, 1.0, 1);
                break;
                case 3:
                    flyToPoint(-0.1, 3.5, 1.0, 1);
                break;
                case 4:
                    endFlyTask();
                break;  
                }
            }else if(fly_target <= 0x15)
            {
                switch (fly_task_state)
                {
                case 0:
                    flyToPoint(-0.1, 1.75, 1.4, 1);
                break;
                case 1:
                    if(fly_target == 0x15){
                        flyToPoint(0.75, 1.75, 1.4, 4);
                    }else if(fly_target == 0x14){
                        flyToPoint(1.25, 1.75, 1.4, 4);
                    }else if (fly_target == 0x13){
                        flyToPoint(1.75, 1.75, 1.4, 4);
                    }
                break;
                case 2:
                    flyToPoint(-0.1, 1.75, 1.4, 1);
                break;
                case 3:
                    flyToPoint(-0.1, 3.5, 1.4, 1);
                break;
                case 4:
                    endFlyTask();
                break;  
                }
            }

        }

    }else if(fly_target >= 0x19 && fly_target <= 0x24){
            if(fly_target > 0x21)
            {
                switch (fly_task_state)
                {
                case 0:
                    flyToPoint(-0.1, 3.5, 1.0, 1);
                break;
                case 1:
                    changeYaw(-0.1, 3.5, 1.0, 1);
                break;
                case 2:
                    if(fly_target == 0x22){
                        flyToPoint(0.75, 3.5, 1.0, 4);
                    }else if(fly_target == 0x23){
                        flyToPoint(1.25, 3.5, 1.0, 4);
                    }else if (fly_target == 0x24){
                        flyToPoint(1.75, 3.5, 1.0, 4);
                    }
                break;
                case 3:
                    flyToPoint(-0.1, 3.5, 1.0, 1);
                break;
                case 4:
                    flyToPoint(-0.1, 3.5, 1.0, 1);
                break;
                case 5:
                    endFlyTask();
                break;  
                }
            }else if(fly_target <= 0x21)
            {
                switch (fly_task_state)
                {
                case 0:
                    flyToPoint(-0.1, 3.5, 1.4, 1);
                break;
                case 1:
                    changeYaw(-0.1, 3.5, 1.4, 1);
                break;
                case 2:
                    if(fly_target == 0x19){
                        flyToPoint(0.75, 3.5, 1.4, 4);
                    }else if(fly_target == 0x20){
                        flyToPoint(1.25, 3.5, 1.4, 4);
                    }else if (fly_target == 0x21){
                        flyToPoint(1.75, 3.5, 1.4, 4);
                    }
                break;
                case 3:
                    flyToPoint(-0.1, 3.5, 1.4, 1);
                break;
                case 4:
                    flyToPoint(-0.1, 3.5, 1.4, 1);
                break;
                case 5:
                    endFlyTask();
                break;  
                }
            }

    }
    return twist;
}*/





int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;
    ros::Subscriber fly_target_sub = nh.subscribe("fly_target", 10, fly_target_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    //ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, pose_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1, pose_cb);
    ros::Subscriber move_base_cmd_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, move_base_cmd_vel_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 1, vel_cb);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    ros::Publisher cancel_pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    ros::Publisher fly_task_pub = nh.advertise<std_msgs::Int32>("fly_task", 1);


    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    ros::NodeHandle param_nh("~");
    double working_altitude = param_nh.param("working_altitude", 1.0);
    int fly_task = 1;
    // Wait for FCU connection
    ros::Rate rate(20.0);
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    
    geometry_msgs::Point qr_position;
    qr_position.x = 1.8;
    qr_position.y = 0.0;
    qr_position.z = working_altitude;
    geometry_msgs::Point takeoff_position;
    takeoff_position.x = 0.0;
    takeoff_position.y = 0.0;
    takeoff_position.z = working_altitude;
    geometry_msgs::Point deliver_position[4];
    deliver_position[0].x = 0.5;
    deliver_position[0].y = 0;
    deliver_position[0].z = working_altitude;
    deliver_position[1].x = 0.5;
    deliver_position[1].y = 0.5;
    deliver_position[1].z = working_altitude;
    deliver_position[2].x = 0;
    deliver_position[2].y = 0.5;
    deliver_position[2].z = working_altitude;
    deliver_position[3].x = 0;
    deliver_position[3].y = 0;
    deliver_position[3].z = working_altitude;
    geometry_msgs::Point special_deliver_position;
    special_deliver_position.x = 6.0;
    special_deliver_position.y = 1.0;
    special_deliver_position.z = working_altitude;
    geometry_msgs::Point left_land_position, right_land_position;
    left_land_position.x = 0.0;
    left_land_position.y = 0.0;
    left_land_position.z = 0.4;
    right_land_position.x = 0.0;
    right_land_position.y = 0.0;
    right_land_position.z = working_altitude;
   int  checking_deliver_point = 0;
    std::unordered_set<std::string> post_target;
    std::string land_target;
    last_srv_request = ros::Time::now();
    std::cout << "\033[32mReached Offboard State.\033[0m" << std::endl;
    while (ros::ok()) {
    if(fsm_state != 0)
        {
            if(current_state.mode == "ALTCTL")
            {
                fly_task = 1;
            }else if (current_state.mode == "STABILIZED")
            {
                fly_task = 2;
            }
            std_msgs::Int32 task_msg;
            task_msg.data = fly_task;
            fly_task_pub.publish(task_msg);
        }

        geometry_msgs::TwistStamped twist;
        switch (fsm_state) {
            case 0:  // Offboard state
                if (current_state.mode == "OFFBOARD") {
                    fsm_state = 1;  // goto arm state
                    std::cout << "\033[32mReached Arm State.\033[0m" << std::endl;
                } else {
                   // ROS_INFO("Waitting for Offboard");
                }
                break;
            case 1:  // Arm state
                if (current_state.armed) {
                    fsm_state = 2;  // goto takeoff state
                    std::cout << "\033[32mReached Takeoff State.\033[0m" << std::endl;
                } else {
                   // ROS_INFO("Waitting for Arm");
                }
                break;
            case 2:  // Takeoff state
                if (current_pose.pose.position.z > working_altitude) {
                    fsm_state = 3;  // goto scan qr state
                    std::cout << "\033[32mComplete take off.\033[0m" << std::endl;
                    last_srv_request = ros::Time::now();
                } else {
                    twist.twist = get_pid_vel(takeoff_position);
                }
                break;
            case 3:  // Check deliver point state
              twist.twist = runFlyTask1();
               break;
            case 100:  // Wait for navigate to right land position state
                if (getLengthBetweenPoints(right_land_position, current_pose.pose.position) < 0.1) {
                    //actionlib_msgs::GoalID cancel_msg;
                    //cancel_pub.publish(cancel_msg);
                    fsm_state = 101;  // goto accurately land state
                    std::cout << "\033[32mReached Accurately Land State.\033[0m" << std::endl;
                    last_srv_request = ros::Time::now();
                } else {
                    twist.twist = get_pid_vel(right_land_position);
                    /*twist.twist = move_base_twist;
                    twist.twist.linear.z = std::max(-0.5, std::min(0.5, working_altitude - current_pose.pose.position.z));
                    twist.twist.angular.z = std::max(-1.57, std::min(1.57, -current_rpy.z));*/
                }
                break;
            case 101:  // Land state
                if (current_state.mode == "AUTO.LAND") {
                    fsm_state = -1;  // goto do nothing state
                } else if (current_pose.pose.position.z < 0.5 && getLengthBetweenPoints(left_land_position, current_pose.pose.position) < 0.1) {
                    if (ros::Time::now() - last_srv_request > ros::Duration(0.5)) {
                        mavros_msgs::SetMode land_set_mode;
                        land_set_mode.request.custom_mode = "AUTO.LAND";
                        set_mode_client.call(land_set_mode);
                        last_srv_request = ros::Time::now();
                    }
                } else {
		  twist.twist = get_pid_vel(left_land_position);
                    //twist.twist.linear.z = -0.5;
                }
                break;
            default:
                if (fsm_state != -1) {
                    ROS_FATAL("FATAL ERROR: FSM reaches an invalid state: %d, emergency landing.", fsm_state);
                    fsm_state = 101;
                }
                break;
        }
        vel_pub.publish(twist);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
