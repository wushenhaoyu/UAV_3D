#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

void pose_cb(const nav_msgs::Odometry::ConstPtr& msg, const ros::Publisher& forward_target) {
    geometry_msgs::PoseStamped ret;
    ret.header = msg->header;
    ret.pose = msg->pose.pose;
    forward_target.publish(ret);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_forward_node");
    ros::NodeHandle nh;
    ros::Publisher vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    ros::Subscriber px4_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 1, boost::bind(&pose_cb, _1, vision_pose_pub));
    ros::spin();
    return 0;
}

