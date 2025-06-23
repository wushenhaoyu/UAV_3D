//包含ROS和MAVROS相关头文件 
#include <string> 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>   
#include <string>
#include <geometry_msgs/Twist.h>

#define ALTITUDE  1

int flag  = 1;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg);

//定义变量，用于接收无人机的里程计信息
tf::Quaternion quat; 
double roll, pitch, yaw;
float init_position_x_take_off =0;
float init_position_y_take_off =0;
float init_position_z_take_off =0;
bool  flag_init_position = false;
nav_msgs::Odometry local_pos;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

//回调函数接收无人机的里程计信息
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg;
    if (flag_init_position==false && (local_pos.pose.pose.position.z!=0))
    {
		init_position_x_take_off = local_pos.pose.pose.position.x;
	    init_position_y_take_off = local_pos.pose.pose.position.y;
	    init_position_z_take_off = local_pos.pose.pose.position.z;
        flag_init_position = true;		    
    }
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);	
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_multi_position");

    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x =init_position_x_take_off + 0;
    pose.pose.position.y =init_position_y_take_off + 0;
    pose.pose.position.z =init_position_z_take_off + ALTITUDE;
     

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
 
 	//此处满足一次请求进入offboard模式即可，官方例成循环切入offboard会导致无人机无法使用遥控器控制
    while(ros::ok())
    {
    	//请求进入OFFBOARD模式
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
           	last_request = ros::Time::now();
       	}
        else 
		{
			//请求解锁
			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
			{
		        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
		       	{
		            ROS_INFO("Vehicle armed");
		        }
		        	last_request = ros::Time::now();
			}
		}
	    
	    if(fabs(local_pos.pose.pose.position.z- init_position_z_take_off -ALTITUDE)<0.2)
		{	
			if(ros::Time::now() - last_request > ros::Duration(3.0))
			{
				break;
			}
		}
		//发布期望位置信息
		pose.pose.position.x =init_position_x_take_off + 0;
		pose.pose.position.y =init_position_y_take_off + 0;
		pose.pose.position.z =init_position_z_take_off + ALTITUDE;
		local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }   
    
    
    while(ros::ok())
    {
        if((flag == 1)  && (ros::Time::now() - last_request > ros::Duration(5.0)))
		{ 
			ROS_INFO("Position_1");
            pose.pose.position.x =init_position_x_take_off + 0;
			pose.pose.position.y =init_position_y_take_off + 0;
			pose.pose.position.z =init_position_z_take_off + ALTITUDE;                     
			last_request = ros::Time::now();
            flag=2;
        }

		if((flag ==2) && (ros::Time::now() - last_request > ros::Duration(8.0)))
		{
			ROS_INFO("Position_2 ");
		    pose.pose.position.x =init_position_x_take_off + 1.5;
			pose.pose.position.y =init_position_y_take_off + 0;
			pose.pose.position.z =init_position_z_take_off + ALTITUDE;     
			last_request = ros::Time::now();
			flag=3;
		}
		                   
		if((flag ==3) && (ros::Time::now() - last_request > ros::Duration(8.0)))
		{
		    ROS_INFO("Position_3 ");
		    pose.pose.position.x =init_position_x_take_off + 1.5;
			pose.pose.position.y =init_position_y_take_off + 1.5;
			pose.pose.position.z =init_position_z_take_off + ALTITUDE;      
		    last_request = ros::Time::now();
			flag=4;
		}

		if((flag ==4) && (ros::Time::now() - last_request > ros::Duration(8.0)))
		{
		    ROS_INFO("Position_4 ");
		    pose.pose.position.x =init_position_x_take_off + 0;
			pose.pose.position.y =init_position_y_take_off + 1.5;
			pose.pose.position.z =init_position_z_take_off + ALTITUDE;     
		    last_request = ros::Time::now();
			flag=5;
		}

		if((flag ==5) && (ros::Time::now() - last_request > ros::Duration(8.0)))
		{
            ROS_INFO("Position_1 ");
            pose.pose.position.x =init_position_x_take_off + 0;
			pose.pose.position.y =init_position_y_take_off + 0;
			pose.pose.position.z =init_position_z_take_off + ALTITUDE;     		
            last_request = ros::Time::now();
			flag=6;
        }
       if((flag ==6) && (ros::Time::now() - last_request > ros::Duration(8.0)))
        {
			ROS_INFO("LAND");
 			pose.pose.position.x =init_position_x_take_off + 0;
			pose.pose.position.y =init_position_y_take_off + 0;
			pose.pose.position.z =init_position_z_take_off - 0.5;    
        }
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}




