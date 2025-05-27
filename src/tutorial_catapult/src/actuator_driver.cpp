#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/ActuatorControl.h>

void control_cb(const std_msgs::Bool::ConstPtr &msg,
                const ros::Publisher& mix_pub,
                int pin,
                int open_dir,
                mavros_msgs::ActuatorControl *control_msg) {
    control_msg->controls[pin] = msg->data ? open_dir : -open_dir;
    mix_pub.publish(*control_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "actuator_driver");
    ros::NodeHandle nh;

    ros::NodeHandle param_nh("~");
    int large_servo_pin = param_nh.param("large_servo_pin", 0);
    int small_servo_1_pin = param_nh.param("small_servo_1_pin", 1);
    int small_servo_2_pin = param_nh.param("small_servo_2_pin", 2);
    int large_servo_open_dir = param_nh.param("large_servo_open_dir", -1);
    int small_servo_1_open_dir = param_nh.param("small_servo_1_open_dir", -1);
    int small_servo_2_open_dir = param_nh.param("small_servo_2_open_dir", 1);

    ros::Publisher claw_pub_mix = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 1);
    mavros_msgs::ActuatorControl control_msg;
    control_msg.group_mix = 2;
    control_msg.controls[large_servo_pin] = -large_servo_open_dir;
    control_msg.controls[small_servo_1_pin] = -small_servo_1_open_dir;
    control_msg.controls[small_servo_2_pin] = -small_servo_2_open_dir;
    claw_pub_mix.publish(control_msg);
    
    ros::Subscriber control_sub_l = nh.subscribe<std_msgs::Bool>("servo/large", 1, boost::bind(&control_cb, _1, claw_pub_mix, large_servo_pin, large_servo_open_dir, &control_msg));
    ros::Subscriber control_sub_1 = nh.subscribe<std_msgs::Bool>("servo/small_1", 1, boost::bind(&control_cb, _1, claw_pub_mix, small_servo_1_pin, small_servo_1_open_dir, &control_msg));
    ros::Subscriber control_sub_2 = nh.subscribe<std_msgs::Bool>("servo/small_2", 1, boost::bind(&control_cb, _1, claw_pub_mix, small_servo_2_pin, small_servo_2_open_dir, &control_msg));
    ros::spin();
    return 0;
}

