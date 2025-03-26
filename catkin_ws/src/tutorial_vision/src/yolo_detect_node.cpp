#include <iostream>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tutorial_vision/StringStamped.h>

#define VEL_P 1.0
#define VEL_I 0.0
#define VEL_D 0.0

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

tutorial_vision::StringStamped sign_result;
void yolo_cb(const tutorial_vision::StringStamped::ConstPtr &msg) {
    sign_result = *msg;
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
    ret.angular.z = VEL_P * y_err + VEL_I * yaw_err_sum + VEL_D * dy_err;

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "yolo_detect_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, pose_cb);
    ros::Subscriber yolo_sub = nh.subscribe<tutorial_vision::StringStamped>("yolo_detect", 1, yolo_cb);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // Get hovering location in parameters
    geometry_msgs::Point sign_target;
    ros::NodeHandle param_nh("~");
    sign_target.x = param_nh.param("sign_x", 0.);
    sign_target.y = param_nh.param("sign_y", 0.);
    sign_target.z = param_nh.param("sign_z", 1.);

    // Wait for FCU connection
    ros::Rate rate(20.0);
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    int fsm_state = 0;
    ros::Time last_srv_request = ros::Time::now();
    while (ros::ok()) {
        geometry_msgs::TwistStamped twist;
        switch (fsm_state) {
            case 0:  // Before offboard state
                if (current_state.mode == "OFFBOARD") {
                    fsm_state = 1;  // goto before armed state
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
            case 1:  // After offboard, before armed state
                if (current_state.armed) {
                    fsm_state = 2;  // goto takeoff state
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
                if (current_pose.pose.position.z > sign_target.z) {
                    fsm_state = 3;  // goto move state
                    last_srv_request = ros::Time::now();
                } else {
                    twist.twist.linear.z = 0.4;
                }
                break;
            case 3:  // Move state
                if (getLengthBetweenPoints(sign_target, current_pose.pose.position) > 0.3) {
                    last_srv_request = ros::Time::now();
                }
                if (ros::Time::now() - last_srv_request > ros::Duration(1.5)) {
                    fsm_state = 4;  // goto detect and print state
                } else {  // PID control
                    twist.twist = get_pid_vel(sign_target);
                }
                break;
            case 4:  // Detect and print state
                if (ros::Time::now() - sign_result.header.stamp < ros::Duration(3.0) && sign_result.data.size() > 0) {
                    std::cout << "Yolo Detected: " << sign_result.data[0] << std::endl;
                    fsm_state = 5;  // goto hover state
                    last_srv_request = ros::Time::now();
                } else {
                    twist.twist = get_pid_vel(sign_target);
                }
                break;
            case 5:  // Hover state
                if (ros::Time::now() - last_srv_request > ros::Duration(1.5)) {
                    fsm_state = 6;  // goto move back state
                    last_srv_request = ros::Time::now();
                } else {  // PID control
                    twist.twist = get_pid_vel(sign_target);
                }
                break;
            case 6:  // Move back state
                if (getLengthBetweenPoints(current_pose.pose.position, .0, .0, sign_target.z) > 0.3) {
                    last_srv_request = ros::Time::now();
                }
                if (ros::Time::now() - last_srv_request > ros::Duration(1.5)) {
                    fsm_state = 7;  // goto land state
                } else {  // PID control
                    geometry_msgs::Point origin_point;
                    origin_point.z = sign_target.z;
                    twist.twist = get_pid_vel(origin_point);
                }
                break;
            case 7:  // Land state
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
                fsm_state = -1;
                break;
        }
        vel_pub.publish(twist);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
