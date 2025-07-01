#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>   
#include <string>

#define MAX_ERROR 0.20
#define VEL_SET   0.10
#define ALTITUDE  0.40

using namespace std;


float target_x_angle = 0;
float target_distance = 1500;
float target_hgt = 0;
float linear_x_p = 0.6;
float linear_x_d = 0.22;
float yaw_rate_p = 0.8;
float yaw_rate_d = 0.5;
float x_angle_threshold = 0.1;
float distance_threshold = 300;
float linear_z_p = 0.5;
float linear_z_d = 0.33;
float hgt_threshold = 100;
float max_velocity_z = 0.3;
float max_raw_velocity_x = 2.0;
float max_raw_yaw_rate = 1.0;


geometry_msgs::PointStamped object_pos;

tf::Quaternion quat; 
double roll, pitch, yaw;
float init_position_x_take_off =0;
float init_position_y_take_off =0;
float init_position_z_take_off =0;
bool  flag_init_position = false; 
nav_msgs::Odometry local_pos;


mavros_msgs::State current_state;  
mavros_msgs::PositionTarget setpoint_raw;
//检测到的物体坐标值
string current_frame_id   = "no_object";
double current_position_x = 0;
double current_position_y = 0;
double current_distance   = 0;
double current_position_z = 0;

int tmp_flag_frame = 0;

//1、订阅无人机状态话题
ros::Subscriber state_sub;

//2、订阅无人机实时位置信息
ros::Subscriber local_pos_sub;

//3、订阅实时位置信息
ros::Subscriber object_pos_sub;

//4、发布无人机多维控制话题
ros::Publisher  mavros_setpoint_pos_pub;

//5、请求无人机解锁服务        
ros::ServiceClient arming_client;

//6、请求无人机设置飞行模式，本代码请求进入offboard
ros::ServiceClient set_mode_client;

bool  flag_no_object = false;
bool  temp_flag_no_object = true;
bool  temp_flag = true;
float current_x_record = 0; 
float current_y_record = 0; 
float current_z_record = 0; 

bool temp_flag_hgt = true;
bool temp_flag_distance = true;
bool temp_flag_hgt_distance = true;


float temp_distance_threshold = 0;
float temp_hgt_threshold      = 0;
void pid_control()
{
		static float last_error_x_angle = 0;
		static float last_error_distance = 0;
		static float last_error_hgt = 0;

		float x_angle;
		float distance;
		float hgt;

		if(current_position_x == 0 && current_position_y == 0 && current_distance == 0)
		{
			flag_no_object = true;
			x_angle  = target_x_angle;
			distance = target_distance;
			hgt = target_hgt;
		}
		else
		{
			flag_no_object = false;
			x_angle  = current_position_x / current_distance;
			distance = current_distance;
			hgt      = current_position_y;
		}
		
		float error_x_angle = x_angle - target_x_angle;
		float error_distance = distance - target_distance;
		float error_hgt = hgt - target_hgt;
		//角度控制
		if(error_x_angle > -x_angle_threshold && error_x_angle < x_angle_threshold)  
		{
			error_x_angle = 0;
		}
		//距离控制
		if(error_distance > -distance_threshold && error_distance < distance_threshold) 
		{
			error_distance = 0;
		}
		//高度控制
		if (error_hgt > -hgt_threshold && error_hgt < hgt_threshold)
		{
			error_hgt = 0;
		}
		printf("hgt = %f\r\n",hgt);
		printf("error_hgt = %f\r\n",error_hgt);
		printf("error_x_angle = %f\r\n",error_x_angle);
		
		
		//距离跟随控制
		setpoint_raw.velocity.x = error_distance*linear_x_p/1000 + (error_distance - last_error_distance)*linear_x_d/1000;
		if(setpoint_raw.velocity.x < -max_raw_velocity_x)  
		{
			setpoint_raw.velocity.x = -max_raw_velocity_x;
		}
		else if(setpoint_raw.velocity.x > max_raw_velocity_x) 
		{
			setpoint_raw.velocity.x = max_raw_velocity_x;	
		}
		
		//高度跟随控制
		setpoint_raw.velocity.z = error_hgt * linear_z_p/1000 + (error_hgt - last_error_hgt) * linear_z_d/1000;
		if (setpoint_raw.velocity.z < -max_velocity_z)
		{
			setpoint_raw.velocity.z = -max_velocity_z;
		}
		else if (setpoint_raw.velocity.z > max_velocity_z)
		{
			setpoint_raw.velocity.z = max_velocity_z;
		}
		
		//角度跟随控制
		setpoint_raw.yaw_rate = error_x_angle*yaw_rate_p + (error_x_angle - last_error_x_angle)*yaw_rate_d;
		if(setpoint_raw.yaw_rate < -max_raw_yaw_rate)  
		{
			setpoint_raw.yaw_rate = -max_raw_yaw_rate;
		}
		else if(setpoint_raw.yaw_rate > max_raw_yaw_rate) 
		{
			setpoint_raw.yaw_rate = max_raw_yaw_rate;
		}
		if(fabs(setpoint_raw.yaw_rate) < 0.1)  
		{
			setpoint_raw.yaw_rate = 0;
		}
		
						
	
		//没检测到目标的时候，直接保持原地不动即可，后续可能会改称旋转寻找目标		
		if(flag_no_object)
		{
			//hgt_threshold      = temp_hgt_threshold;
			//distance_threshold = temp_distance_threshold;
			setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 /*+ 2048*/;			
			if(temp_flag_no_object)
			{
				temp_flag_no_object = false;
				current_x_record = local_pos.pose.pose.position.x;
				current_y_record = local_pos.pose.pose.position.y;
				current_z_record = local_pos.pose.pose.position.z;
			}
			setpoint_raw.position.x = current_x_record;
			setpoint_raw.position.y = current_y_record;
			setpoint_raw.position.z = current_z_record;
			setpoint_raw.coordinate_frame = 1;
			
		}
		else
		{
			temp_flag_no_object = true;
			if(fabs(error_hgt)<=hgt_threshold)
			{
				//hgt_threshold = temp_hgt_threshold + 300;
				//distance_threshold = temp_distance_threshold;
				if(temp_flag_hgt)
				{
					temp_flag_hgt = false;					
					current_z_record = local_pos.pose.pose.position.z;
				}
				//error_hgt大于等于0，表明此时无人机在目标物的下方
				if(error_hgt>=0)
				{
					setpoint_raw.position.z = current_z_record + 0.4*hgt_threshold*0.001;
				}
				else
				{
					setpoint_raw.position.z = current_z_record - 0.4*hgt_threshold*0.001;
				}
				setpoint_raw.type_mask = 1 + 2 /*+ 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 /*+ 2048*/;
				setpoint_raw.coordinate_frame = 8;
			}
			else
			{
				//hgt_threshold      = temp_hgt_threshold;
				//distance_threshold = temp_distance_threshold;
				temp_flag_hgt = true;
				setpoint_raw.type_mask = 1 + 2 + 4 /*+ 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 /*+ 2048*/;
				setpoint_raw.coordinate_frame = 8;
			}
			//if(fabs(error_hgt)<=hgt_threshold)
			//if(error_x_angle)
			
		}		

		mavros_setpoint_pos_pub.publish(setpoint_raw);
		last_error_x_angle  = error_x_angle;
		last_error_distance = error_distance;
		last_error_hgt      = error_hgt;

}

 
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

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
}


double positon_x_table[5]={};
double positon_y_table[5]={};
double distance_table[5] ={};
int count_positon_x = 0;
int count_positon_y = 0;
int count_distance  = 0;

void object_pos_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	//ROS_INFO("current_position_x = %f");
	double temp_current_position_x;
	double temp_current_position_y;
	double temp_current_distance;
	int count_target_lost = 0;
	object_pos = *msg;
	current_frame_id   = object_pos.header.frame_id; 
	
	//获取最新的rpy值
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);	
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	printf("pitch = %f\r\n",pitch);
	
	//此处将距离由单位米改称毫米，方便提高控制精度
	temp_current_position_x = object_pos.point.x*(-1000);
    temp_current_position_y = object_pos.point.y*(-1000);
	temp_current_distance   = object_pos.point.z*(1000)/cos(pitch);

	printf("temp_current_distance = %f\r\n",temp_current_distance);

	//获取五次X方向数据
	positon_x_table[count_positon_x] = temp_current_position_x;
    count_positon_x++;
    if(count_positon_x>=5)
    {
    	count_positon_x = 0;
    } 
    //获取五次Y方向数据
    positon_y_table[count_positon_y] = temp_current_position_y;
    count_positon_y++;
    if(count_positon_y>=5)
    {
    	count_positon_y = 0;
    }
    //获取五次距离数据
    distance_table[count_distance] = temp_current_distance;
    count_distance++;
    if(count_distance>=5)
    {
    	count_distance = 0;
    } 
    
    double temp_positon_x_table[5]={};
	double temp_positon_y_table[5]={};
	double temp_distance_table[5] ={};
    //遍历数据查找是否有丢失目标的情况，每丢失一次，则计数器+1
    int temp_i = 0;
    for(int i=0;i<=4;i++)
    {
    	//所有数据为0，可以判定没有识别到目标
    	if(positon_x_table[i]==0 && positon_y_table[i]==0 && distance_table[i]==0)
    	{
    		count_target_lost++;
    		ROS_INFO("count_target_lost = %d",count_target_lost); 
    	}
    	else
    	{
    		temp_positon_x_table[temp_i] =  positon_x_table[i];
    		temp_positon_y_table[temp_i] =  positon_y_table[i];
    		temp_distance_table[temp_i]  =  distance_table[i];
    		temp_i++;
    	}
    }
    //如果5次里面丢失超过3次，直接判定为识别到目标，可能是其他干扰导致的误识别或者本身就是没有识别到目标
    if(count_target_lost>3)
    {
    	current_position_x = 0;
    	current_position_y = 0;
    	current_distance   = 0;
    }
    //如果认定数组里的数据有3个以上是有效的，那么应该除去最高与最低后采用均值滤波算法
    else
    {
    	current_position_x = (temp_positon_x_table[0]+temp_positon_x_table[1]+temp_positon_x_table[2])/3;
    	current_position_y = (temp_positon_y_table[0]+temp_positon_y_table[1]+temp_positon_y_table[2])/3;
    	current_distance   = (temp_distance_table[0] +temp_distance_table[1] +temp_distance_table[2])/3;
    }
    
	if(tmp_flag_frame == 1)
	{
		pid_control();	 
	}
}



int main(int argc, char *argv[])
{

    ros::init(argc, argv, "follow_pid");
    
    ros::NodeHandle nh;

	state_sub     = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);
		
    local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 100, local_pos_cb);
    
    object_pos_sub = nh.subscribe<geometry_msgs::PointStamped>("object_position", 100, object_pos_cb);
		
    mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);   
		               
	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
		
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0); 

	ros::param::get("linear_x_p",linear_x_p);
	ros::param::get("linear_x_d",linear_x_d);
	ros::param::get("yaw_rate_p",yaw_rate_p);
	ros::param::get("yaw_rate_d",yaw_rate_d);
	
	ros::param::get("target_x_angle", target_x_angle);
	ros::param::get("target_distance",target_distance);
	ros::param::get("x_angle_threshold", x_angle_threshold);
	ros::param::get("distance_threshold", distance_threshold);
	ros::param::get("linear_z_p", linear_z_d);
	ros::param::get("target_hgt", target_hgt);
	ros::param::get("hgt_threshold", hgt_threshold);
	
	ros::param::get("max_velocity_z",max_velocity_z);
	ros::param::get("max_raw_velocity_x",max_raw_velocity_x);
	ros::param::get("max_raw_yaw_rate",max_raw_yaw_rate);
	
	temp_distance_threshold = distance_threshold;
    temp_hgt_threshold      = hgt_threshold;
    printf("temp_distance_threshold = %f\r\n",temp_distance_threshold);
    printf("temp_hgt_threshold = %f\r\n",     temp_hgt_threshold);
	 //等待连接到PX4无人机
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = 0;
	setpoint_raw.position.y = 0;
	setpoint_raw.position.z = 0 + ALTITUDE;
	mavros_setpoint_pos_pub.publish(setpoint_raw);
 
    for(int i = 100; ros::ok() && i > 0; --i)
    {
		mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    
    //请求offboard模式变量
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    //请求解锁变量
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    //请求进入offboard模式并且解锁无人机，15秒后退出，防止重复请求       
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
		
		if(fabs(local_pos.pose.pose.position.z-ALTITUDE)<0.1)
		{
			if(ros::Time::now() - last_request > ros::Duration(1.0))
			{
				tmp_flag_frame= 1;
				break;
			}
		}
	

		mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }  
	while(ros::ok())
	{
		mavros_setpoint_pos_pub.publish(setpoint_raw);
		ros::spinOnce();
		rate.sleep();

	}

}

