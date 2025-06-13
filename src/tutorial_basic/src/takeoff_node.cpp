#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_pose = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "takeoff_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, pose_cb);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // Wait for FCU connection
    ros::Rate rate(20.0);
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    int fsm_state = 0;
    ros::Time last_srv_request = ros::Time::now();
    while (ros::ok()) {
        geometry_msgs::TwistStamped twistStamped;
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
                if (current_pose.pose.position.z > 1.0) {
                    fsm_state = 3;  // goto hover state
                    last_srv_request = ros::Time::now();
                } else {
                    twistStamped.twist.linear.x = 0.0;
                    twistStamped.twist.linear.y = 0.0;
                    twistStamped.twist.linear.z = 0.4;
                }
                break;
            case 3:  // Hover state
                if (ros::Time::now() - last_srv_request > ros::Duration(3.0)) {
                    fsm_state = 4;  // goto land state
                } else {
                    twistStamped.twist.linear.x = 0.0;
                    twistStamped.twist.linear.y = 0.0;
                    twistStamped.twist.linear.z = 0.0;
                }
                break;
            case 4:  // Land state
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
                    twistStamped.twist.linear.z = -0.2;
                }
                break;
            default:
                fsm_state = -1;
                break;
        }
        vel_pub.publish(twistStamped);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
