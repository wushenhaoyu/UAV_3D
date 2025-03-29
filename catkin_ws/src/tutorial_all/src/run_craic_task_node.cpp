#include <vector>
#include <unordered_set>
#include <iostream>

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

geometry_msgs::PoseStamped current_pose;
geometry_msgs::Vector3 current_rpy;
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

tutorial_vision::CircleDetectResult parking_detect_result;
void parking_cb(const tutorial_vision::CircleDetectResult::ConstPtr &msg) {
    parking_detect_result = *msg;
    if (msg->header.stamp == ros::Time(0)) parking_detect_result.header.stamp = ros::Time::now();
}

tutorial_vision::CircleDetectResult deliver_detect_result;
void deliver_cb(const tutorial_vision::CircleDetectResult::ConstPtr &msg) {
    deliver_detect_result = *msg;
    if (msg->header.stamp == ros::Time(0)) deliver_detect_result.header.stamp = ros::Time::now();
}

tutorial_vision::StringStamped qr_result;
void qr_cb(const tutorial_vision::StringStamped::ConstPtr &msg) {
    qr_result = *msg;
    if (msg->header.stamp == ros::Time(0)) qr_result.header.stamp = ros::Time::now();
}

tutorial_vision::StringStamped sign_result;
void yolo_cb(const tutorial_vision::StringStamped::ConstPtr &msg) {
    sign_result = *msg;
    if (msg->header.stamp == ros::Time(0)) sign_result.header.stamp = ros::Time::now();
}

geometry_msgs::Point last_err;
geometry_msgs::Point err_sum;
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
}

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

void analyseQrMessage(const std::string &str, std::unordered_set<std::string> &post_target, std::string &land_target) {
    std::vector<std::string> split;
    boost::split(split, str, boost::is_any_of(","), boost::token_compress_on);
    post_target.insert(split[0]);
    post_target.insert(split[1]);
    land_target = split[2];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, pose_cb);
    ros::Subscriber move_base_cmd_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, move_base_cmd_vel_cb);
    ros::Subscriber parking_sub = nh.subscribe<tutorial_vision::CircleDetectResult>("parking_detect_result", 1, parking_cb);
    ros::Subscriber deliver_sub = nh.subscribe<tutorial_vision::CircleDetectResult>("deliver_detect_result", 1, deliver_cb);
    ros::Subscriber qr_sub = nh.subscribe<tutorial_vision::StringStamped>("qr_detect_result", 1, qr_cb);
    ros::Subscriber yolo_sub = nh.subscribe<tutorial_vision::StringStamped>("yolo_detect", 1, yolo_cb);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    ros::Publisher cancel_pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    std::vector<ros::Publisher> catapult_pubs = {
        nh.advertise<std_msgs::Bool>("servo/small_1", 1),
        nh.advertise<std_msgs::Bool>("servo/large", 1),
        nh.advertise<std_msgs::Bool>("servo/small_2", 1)
    };
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    ros::NodeHandle param_nh("~");
    double working_altitude = param_nh.param("working_altitude", 1.0);

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
    geometry_msgs::Point deliver_position[4];
    deliver_position[0].x = 1.8;
    deliver_position[0].y = 1.6;
    deliver_position[0].z = working_altitude;
    deliver_position[1].x = 3.6;
    deliver_position[1].y = 1.6;
    deliver_position[1].z = working_altitude;
    deliver_position[2].x = 3.6;
    deliver_position[2].y = -1.6;
    deliver_position[2].z = working_altitude;
    deliver_position[3].x = 1.8;
    deliver_position[3].y = -1.6;
    deliver_position[3].z = working_altitude;
    geometry_msgs::Point special_deliver_position;
    special_deliver_position.x = 6.0;
    special_deliver_position.y = 1.0;
    special_deliver_position.z = working_altitude;
    geometry_msgs::Point left_land_position, right_land_position;
    left_land_position.x = 0.0;
    left_land_position.y = 1.6;
    left_land_position.z = working_altitude;
    right_land_position.x = 0.0;
    right_land_position.y = -1.6;
    right_land_position.z = working_altitude;
    
    int fsm_state = 0;
    int checking_deliver_point = 0;
    int posted_object = 0;
    std::unordered_set<std::string> post_target;
    std::string land_target;
    ros::Time last_srv_request = ros::Time::now();
    std::cout << "\033[32mReached Offboard State.\033[0m" << std::endl;
    while (ros::ok()) {
        //std::cout << posted_object << "/2 " << checking_deliver_point << "/4" << std::endl;
        geometry_msgs::TwistStamped twist;
        switch (fsm_state) {
            case 0:  // Offboard state
                if (current_state.mode == "OFFBOARD") {
                    fsm_state = 1;  // goto arm state
                    std::cout << "\033[32mReached Arm State.\033[0m" << std::endl;
                } else {
                    if (ros::Time::now() - last_srv_request > ros::Duration(1.0)) {
                        mavros_msgs::SetMode offb_set_mode;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                            ROS_INFO("Offboard enabled");
                        } else {
                            ROS_WARN("Failed to enable offboard");
                        }
                        last_srv_request = ros::Time::now();
                    }
                }
                break;
            case 1:  // Arm state
                if (current_state.armed) {
                    fsm_state = 2;  // goto takeoff state
                    std::cout << "\033[32mReached Takeoff State.\033[0m" << std::endl;
                } else {
                    if (ros::Time::now() - last_srv_request > ros::Duration(1.0)) {
                        mavros_msgs::CommandBool arm_cmd;
                        arm_cmd.request.value = true;
                        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                            ROS_INFO("Vehicle armed");
                        } else {
                            ROS_WARN("Failed to arm vehicle");
                        }
                        last_srv_request = ros::Time::now();
                    }
                }
                break;
            case 2:  // Takeoff state
                if (current_pose.pose.position.z > working_altitude) {
                    fsm_state = 4;  // goto scan qr state
                    std::cout << "\033[32mReached Scan QR State.\033[0m" << std::endl;
                    last_srv_request = ros::Time::now();
                } else {
                    twist.twist.linear.z = 0.4;
                }
                break;
            case 4:  // Check deliver point state
                 if (ros::Time::now() - last_srv_request > ros::Duration(1.0) &&
                        sign_result.header.stamp > last_srv_request &&
                        getLengthBetweenPoints(current_pose.pose.position, deliver_position[checking_deliver_point]) < 0.2) {  // detect succeeded
                        checking_deliver_point++;
                        if (checking_deliver_point >= 4) {
                            fsm_state = 12;  // goto navigate to special sign state
                            std::cout << "\033[32mReached Navigate to Special Sign State.\033[0m" << std::endl;
                        }
                        last_srv_request = ros::Time::now();
                } else if (ros::Time::now() - last_srv_request > ros::Duration(5000.0)) {  // Timeout, detect failed
                    checking_deliver_point++;
                    if (checking_deliver_point >= 4) {
                        fsm_state = 12;  // goto navigate to special sign state
                        std::cout << "\033[32mReached Navigate to Special Sign State.\033[0m" << std::endl;
                    }
                    last_srv_request = ros::Time::now();
                } else {
                    if (getLengthBetweenPoints(current_pose.pose.position, deliver_position[checking_deliver_point]) > 0.3) {
                        last_srv_request = ros::Time::now();
                    }
                    twist.twist = get_pid_vel(deliver_position[checking_deliver_point]);
                }
                break;
            case 7:  // Navigate to special sign state
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
                    twist.twist.linear.z = std::max(-0.5, std::min(0.5, working_altitude - current_pose.pose.position.z));
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
                    twist.twist.linear.z = std::max(-0.5, std::min(0.5, working_altitude - current_pose.pose.position.z));
                    twist.twist.angular.z = std::max(-1.57, std::min(1.57, -current_rpy.z));
                }
                break;
            case 12:  // Wait for navigate to right land position state
                if (getLengthBetweenPoints(right_land_position, current_pose.pose.position) < 0.3) {
                    actionlib_msgs::GoalID cancel_msg;
                    cancel_pub.publish(cancel_msg);
                    fsm_state = 13;  // goto accurately land state
                    std::cout << "\033[32mReached Accurately Land State.\033[0m" << std::endl;
                    last_srv_request = ros::Time::now();
                } else {
                    twist.twist = move_base_twist;
                    twist.twist.linear.z = std::max(-0.5, std::min(0.5, working_altitude - current_pose.pose.position.z));
                    twist.twist.angular.z = std::max(-1.57, std::min(1.57, -current_rpy.z));
                }
                break;
            case 13:  // Accurately land state
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
                break;
            case 100:  // Land state
                if (current_state.mode == "AUTO.LAND") {
                    fsm_state = -1;  // goto do nothing state
                } else if (current_pose.pose.position.z < 0.1) {
                    if (ros::Time::now() - last_srv_request > ros::Duration(0.5)) {
                        mavros_msgs::SetMode land_set_mode;
                        land_set_mode.request.custom_mode = "AUTO.LAND";
                        set_mode_client.call(land_set_mode);
                        last_srv_request = ros::Time::now();
                    }
                } else {
                    twist.twist.linear.z = -0.2;
                }
                break;
            default:
                if (fsm_state != -1) {
                    ROS_FATAL("FATAL ERROR: FSM reaches an invalid state: %d, emergency landing.", fsm_state);
                    fsm_state = 100;
                }
                break;
        }
        vel_pub.publish(twist);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
