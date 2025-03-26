#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg, tf::TransformBroadcaster& br, const std::string &parent, const std::string &child) {
    tf::StampedTransform transform;
    transform.frame_id_ = parent;
    transform.child_frame_id_ = child;
    transform.stamp_ = msg->header.stamp;
    tf::Quaternion rotation;
    tf::quaternionMsgToTF(msg->pose.orientation, rotation);
    tf::Vector3 origin(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    transform.setData(tf::Transform(rotation, origin));
    br.sendTransform(transform);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_tf_forward_node");

    ros::NodeHandle param_nh("~");
    std::string parent = param_nh.param<std::string>("parent", "odom");
    std::string child = param_nh.param<std::string>("child", "base_link");

    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, boost::bind(&pose_cb, _1, br, parent, child));
    ros::spin();
    return 0;
}

