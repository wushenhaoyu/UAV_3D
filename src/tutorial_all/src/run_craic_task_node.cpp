#include <vector>
#include <unordered_set>
#include <iostream>
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
#include <std_msgs/UInt8.h>



#define VEL_P 1.0
#define VEL_I 0.0
#define VEL_D 0.0
#define YAW_VEL_P 1.0
#define YAW_VEL_I 0.0
#define YAW_VEL_D 0.0
#define PIX_VEL_P 0.001
#define PIX_VEL_I 0.0
#define PIX_VEL_D 0.0

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

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}


geometry_msgs::Vector3 current_rpy;
geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_pose = *msg;
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(msg->pose.orientation, quaternion);
    tf::Matrix3x3(quaternion).getRPY(current_rpy.x, current_rpy.y, current_rpy.z);
}

geometry_msgs::Twist move_base_twist;
void move_base_cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &msg) {
    move_base_twist = *msg;
}

int8_t fly_target = 0x00;
void flyTargetCallback(const std_msgs::UInt8::ConstPtr& msg) {
    ROS_INFO("接收到 fly_target: %d (十六进制: 0x%X)", msg->data, msg->data);
    fly_target = msg->data;
}

geometry_msgs::Point last_err;
geometry_msgs::Point err_sum;
double target_yaw = 0;
double last_yaw_err = 0.;
double yaw_err_sum = 0.;
ros::Time last_pid_control_time;

geometry_msgs::Twist get_pid_vel(geometry_msgs::Point target) {
    ros::Time currentStamp = current_pose.header.stamp;
    ros::Duration dt = currentStamp - last_pid_control_time;
    if (dt.toSec() > 0.2) {
        err_sum.x = 0.;
        err_sum.y = 0.;
        err_sum.z = 0.;
        yaw_err_sum = 0.;
    }

    geometry_msgs::Point err;
    double absErr = getLengthBetweenPoints(target, current_pose.pose.position, &err.x, &err.y, &err.z);

    // Yaw 控制部分：维持当前航向靠近目标航向角
    double yaw_err = target_yaw - current_rpy.z;
    // 将 yaw 误差限制在 [-π, π] 范围
    while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2 * M_PI;
    double dy_err = (yaw_err - last_yaw_err) / dt.toSec();

    geometry_msgs::Twist ret;
    ret.angular.z = YAW_VEL_P * yaw_err + YAW_VEL_I * yaw_err_sum + YAW_VEL_D * dy_err;

    // === 坐标变换：将误差从全局坐标系转到机体坐标系 ===
    double yaw = current_rpy.z;
    double cos_yaw = cos(-yaw);
    double sin_yaw = sin(-yaw);

    double body_x = err.x * cos_yaw - err.y * sin_yaw;
    double body_y = err.x * sin_yaw + err.y * cos_yaw;
    double body_z = err.z;

    if (absErr > 0.8) {
        double scale = 0.8 / absErr;
        ret.linear.x = body_x * scale;
        ret.linear.y = body_y * scale;
        ret.linear.z = body_z * scale;

        err_sum.x = .0;
        err_sum.y = .0;
        err_sum.z = .0;
    } else {
        geometry_msgs::Point d_err;
        d_err.x = (err.x - last_err.x) / dt.toSec();
        d_err.y = (err.y - last_err.y) / dt.toSec();
        d_err.z = (err.z - last_err.z) / dt.toSec();

        // 将 d_err 也转换到机体坐标系
        double d_body_x = d_err.x * cos_yaw - d_err.y * sin_yaw;
        double d_body_y = d_err.x * sin_yaw + d_err.y * cos_yaw;
        double d_body_z = d_err.z;

        // 同样对误差积分进行旋转
        double sum_body_x = err_sum.x * cos_yaw - err_sum.y * sin_yaw;
        double sum_body_y = err_sum.x * sin_yaw + err_sum.y * cos_yaw;
        double sum_body_z = err_sum.z;

        ret.linear.x = VEL_P * body_x + VEL_I * sum_body_x + VEL_D * d_body_x;
        ret.linear.y = VEL_P * body_y + VEL_I * sum_body_y + VEL_D * d_body_y;
        ret.linear.z = VEL_P * body_z + VEL_I * sum_body_z + VEL_D * d_body_z;

        err_sum.x += err.x * dt.toSec();
        err_sum.y += err.y * dt.toSec();
        err_sum.z += err.z * dt.toSec();
    }

    last_err = err;
    last_yaw_err = yaw_err;
    yaw_err_sum += yaw_err * dt.toSec();
    last_pid_control_time = currentStamp;

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

    geometry_msgs::Point err;
    double absErr = getLengthBetweenPoints(target, current_pose.pose.position, &err.x, &err.y, &err.z);

    double y_err = 0. - current_rpy.z;
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
    ret.angular.z = YAW_VEL_P * y_err + YAW_VEL_I * yaw_err_sum + YAW_VEL_D * dy_err;

    geometry_msgs::Point d_err;
    d_err.x = (err.x - pix_last_err.x) / dt.toSec();
    d_err.y = (err.y - pix_last_err.y) / dt.toSec();

    ret.linear.x = PIX_VEL_P * err.y + PIX_VEL_I * pix_err_sum.y + PIX_VEL_D * d_err.y;
    ret.linear.y = PIX_VEL_P * err.x + PIX_VEL_I * pix_err_sum.x + PIX_VEL_D * d_err.x;

    pix_err_sum.x += err.x * dt.toSec();
    pix_err_sum.y += err.y * dt.toSec();

    pix_last_err = err;
    last_yaw_err = y_err;
    yaw_err_sum += y_err * dt.toSec();
    last_pid_control_time = currentStamp;
    
    return ret;
}

#define ALTITUDE 1.4 //飞行高度
#define END_X 3.5
#define END_Y 2.5
#define END_Z 1.4
int fsm_state = 0;
int fly_task_state = 0;
bool point_arrive_flag = false;
ros::Time last_srv_request = ros::Time::now();


void logTime()
{
    last_srv_request = ros::Time::now();
}

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
                logTime();
            }
        }   
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
    if (getLengthBetweenPoints(current_pose.pose.position, target_point) < 0.1 && target_yaw - current_rpy.z < M_PI / 30)
    {
        if(point_arrive_flag == false)
        {
            point_arrive_flag = true;
            logTime();
        }
        if (point_arrive_flag == true)
        {
                fly_task_state += 1;
                point_arrive_flag = false;
                logTime();
        }  
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

geometry_msgs::Twist runFlyTask()
/*测试飞行，一个方格子*/
{
    geometry_msgs::Twist twist;
    switch(fly_task_state)
    {
        case 0:
            flyToPoint(0,0.5,ALTITUDE,1);
        break;
        case 1:
            flyToPoint(0.5,0.5,ALTITUDE,1);
        break;
        case 2:
            flyToPoint(0.5,0,ALTITUDE,1);
        break;
        case 3:
            flyToPoint(0,0,ALTITUDE,1);
        break;
        case 4:
            endFlyTask();
        break;
    }
    return twist;
}

geometry_msgs::Twist runFlyTask_2024_1()
/*
2024电赛题目,起飞方向朝板子
*/
{
    geometry_msgs::Twist twist;
    switch(fly_task_state)
    {
        case 0:
            flyToPoint(0.0,2.0,1.4,1); //遍历前3、2、1点
        break;
        case 1:
            flyToPoint(0.0,2.0,1.0,1); //下降高度
        break;
        case 2:
            flyToPoint(0.0,-0.1,1.0,1); //遍历4、5、6点
        break;
        case 3:
            flyToPoint(1.75,-0.1,1.0,1); //飞到中间
        break;
        case 4:
            flyToPoint(1.75,2.0,1.0,1);//遍历18、17、16
        break;
        case 5:
            flyToPoint(1.75,2.0,1.4,1); //上升高度
        break;
        case 6:
            flyToPoint(1.75,0.0,1.4,1);//遍历13、14、15
        break;
        case 7:
            changeYaw(1.75,0.0,1.4,1);//转180度
        break;
        case 8:
            flyToPoint(1.75,2.0,1.4,1);//遍历7、8、9
        break;
        case 9:
            flyToPoint(1.75,2.0,1.0,1);//下降高度
        break;
        case 10:
            flyToPoint(1.75,-0.1,1.0,1);//遍历12、11、10
        break;
        case 11:
            flyToPoint(3.5,-0.1,1.0,1);//到最后一个板子
        break;
        case 12:
            flyToPoint(3.5,2.0,1.0,1);//遍历22、23、24
        break;
        case 13:
            flyToPoint(3.5,2.0,1.4,1);//上升高度
        break;
        case 14:
            flyToPoint(3.5,0.0,1.4,1);//遍历21、20、19
        break;
        case 15:
            endFlyTask();
        break;
    }
    return twist;
}

geometry_msgs::Twist runFlyTask_2024_2()
{


}




int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, pose_cb);
    ros::Subscriber move_base_cmd_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, move_base_cmd_vel_cb);
    ros::Subscriber sub = nh.subscribe("fly_target", 10, fly_target_cb);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    ros::Publisher cancel_pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    ros::Publisher fly_task_pub = nh.advertise<std_msgs::Int32>("fly_task", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    


    // Wait for FCU connection
    ros::Rate rate(20.0);
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    geometry_msgs::Point land_point;
    land_point.x = END_X;
    land_point.y = END_Y;
    land_point.z = END_Z;

    int fly_task = 1;
    int checking_deliver_point = 0;
    //int posted_object = 0;
    //std::unordered_set<std::string> post_target;
    //std::string land_target;
    std::cout << "\033[32mReached Offboard State.\033[0m" << std::endl;
    while (ros::ok()) {
        //std::cout << posted_object << "/2 " << checking_deliver_point << "/4" << std::endl;
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
                }
                break;
            case 1:  // Arm state
                if (current_state.armed) {
                    fsm_state = 2;  // goto takeoff state
                    std::cout << "\033[32mReached Takeoff State.\033[0m" << std::endl;
                } 
                break;
            case 2:  // Takeoff state
                if (current_pose.pose.position.z > ALTITUDE) {
                    fsm_state = 4;  // goto task
                    std::cout << "\033[32mReached Scan QR State.\033[0m" << std::endl;
                    last_srv_request = ros::Time::now();
                } else {
                    twist.twist.linear.z = 0.4;
                }
                break;
            case 4:  // Check deliver point state
                runFlyTask();
                break;
            /*case 7:  // Navigate to special sign state
                {
                    geometry_msgs::PoseStamped move_base_msg;
                    move_base_msg.header.frame_id = "map";
                    move_base_msg.pose.position = special_deliver_position;
                    move_base_msg.pose.orientation.w = -1.0;
                    goal_pub.publish(move_base_msg);
                    fsm_state = 8;  // goto wait for navigation mission state
                    std::cout << "\033[32mReached Wait for Navigation Mission State.\033[0m" << std::endl;
                }
                break;
            case 8:  // Wait for navigation mission state
                if (getLengthBetweenPoints(special_deliver_position, current_pose.pose.position) < 0.3) {
                    actionlib_msgs::GoalID cancel_msg;
                    cancel_pub.publish(cancel_msg);
                    fsm_state = 9;  // goto special deliver state
                    std::cout << "\033[32mReached Special Deliver State.\033[0m" << std::endl;
                    last_srv_request = ros::Time::now();
                } else {
                    twist.twist = move_base_twist;
                    twist.twist.linear.z = std::max(-0.5, std::min(0.5, ALTITUDE - current_pose.pose.position.z));
                    twist.twist.angular.z = std::max(-1.57, std::min(1.57, -current_rpy.z));
                }
                break;
            case 9:  // Special deliver state
                if (ros::Time::now() - last_srv_request > ros::Duration(4.0)) {  // release catapult
                    std_msgs::Bool catapult_msg;
                    catapult_msg.data = true;
                    catapult_pubs[posted_object].publish(catapult_msg);
                    fsm_state = 10;  // goto special wait for object drop state
                    std::cout << "\033[32mReached Special Wait for Object Drop State.\033[0m" << std::endl;
                    last_srv_request = ros::Time::now();
                } else {
                    if (deliver_detect_result.header.stamp > last_srv_request - ros::Duration(1.0)) {
                        geometry_msgs::Point err;
                        err.x = deliver_detect_result.width / 2.0 - deliver_detect_result.circles[0].center_x;
                        err.y = deliver_detect_result.height / 2.0 - deliver_detect_result.circles[0].center_y;
                        twist.twist = get_pix_pid_vel(err);
                    }
                    twist.twist.linear.z = std::max(-0.5, std::min(0.5, ALTITUDE - current_pose.pose.position.z));
                    twist.twist.angular.z = std::max(-1.57, std::min(1.57, -current_rpy.z));
                }
                break;*/
            case 100:  // Wait for navigate to right land position state
                if (getLengthBetweenPoints(land_point, current_pose.pose.position) < 0.1) {
                    actionlib_msgs::GoalID cancel_msg;
                    cancel_pub.publish(cancel_msg);
                    fsm_state = 101;  // goto accurately land state
                    std::cout << "\033[32mReached Accurately Land State.\033[0m" << std::endl;
                    last_srv_request = ros::Time::now();
                } else {
                    twist.twist = move_base_twist;
                    twist.twist.linear.z = std::max(-0.5, std::min(0.5, ALTITUDE - current_pose.pose.position.z));
                    twist.twist.angular.z = std::max(-1.57, std::min(1.57, -current_rpy.z));
                }
                break;
            /*case 13:  // Accurately land state
                if (current_pose.pose.position.z < 0.1) {
                    fsm_state = 100;  // goto land state
                    std::cout << "\033[32mReached Land State.\033[0m" << std::endl;
                } else if (parking_detect_result.header.stamp > last_srv_request && parking_detect_result.circles.size() > 0) {
                    geometry_msgs::Point err;
                    err.x = parking_detect_result.width / 2.0 - parking_detect_result.circles[0].center_x;
                    err.y = parking_detect_result.height / 2.0 - parking_detect_result.circles[0].center_y;
                    twist.twist = get_pix_pid_vel(err);
                    twist.twist.linear.z = -0.1;
                }
                break;*/
            case 101:  // Land state
                if (current_state.mode == "AUTO.LAND") {
                    fsm_state = -1;  // goto do nothing state
                } else if (current_pose.pose.position.z < 0.1) {
                    if (ros::Time::now() - last_srv_request > ros::Duration(0.5)) {
                        mavros_msgs::SetMode land_set_mode;
                        land_set_mode.request.custom_mode = "AUTO.LAND";
                        set_mode_client.call(land_set_mode);
                    }
                } else {
                    twist.twist.linear.z = -0.2;
                }
                break;
            default:
                if (fsm_state != -1) {
                    ROS_FATAL("FATAL ERROR: FSM reaches an invalid state: %d, emergency landing.", fsm_state);
                    fsm_state = 101;
                }
                break;
        }
        if (fsm_state > 0 && current_state.mode != "OFFBOARD") {
            ROS_WARN("OFFBOARD mode lost! Triggering emergency landing.");
            fsm_state = 101;  // 或其他应急状态
            last_srv_request = ros::Time::now();
        }

        vel_pub.publish(twist);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
