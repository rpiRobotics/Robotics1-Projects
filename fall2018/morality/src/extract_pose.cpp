#include "ros/ros.h"
#include "duckietown_msgs/LanePose.h"
#include "duckietown_msgs/Twist2DStamped.h"
#include "std_msgs/Float32.h"

/* This node extracts the lane pose data into 2 separate topics to be
 * interpreted by the feedback controller.
 * It also takes in the resulting feedback output turn and publishes a car cmd
 */

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "extract_pose");
	
	ros::NodeHandle nh;
	
	constexpr auto main_velocity_param = "main_velocity";
	if(!nh.hasParam(main_velocity_param)) nh.setParam(main_velocity_param, 1.0);
	
	
	constexpr auto pub_q = 1000;
	auto lane_offset_pub = nh.advertise<std_msgs::Float32>("lane_offset", pub_q);
	auto angle_pub = nh.advertise<std_msgs::Float32>("lane_angle", pub_q);
	auto car_cmd_pub = nh.advertise<duckietown_msgs::Twist2DStamped>("car_cmd", pub_q);
	
	constexpr auto sub_q = 1000;
	auto pose_sub = nh.subscribe<duckietown_msgs::LanePose>(
		"lane_pose", sub_q, [lane_offset_pub, angle_pub](auto&& lane_pose)
		{
			static bool worked = false;
			if(!worked)
			{
				worked = true;
				ROS_INFO("Received our first lane pose");
			}
			
			std_msgs::Float32 offset, angle;
			offset.data = lane_pose->d;
			angle.data = lane_pose->phi;
			
			lane_offset_pub.publish(offset);
			angle_pub.publish(angle);
		});
	
	auto steer_sub = nh.subscribe<std_msgs::Float32>(
		"steer", sub_q, [car_cmd_pub, &nh](auto&& steer)
		{
			duckietown_msgs::Twist2DStamped msg;
			nh.getParamCached(main_velocity_param, msg.v);
			msg.omega = steer->data;
			
			car_cmd_pub.publish(msg);
		});
	
	ros::Rate loop_rate(100);
	
	ROS_INFO("Starting pose extractor!");
	ros::spin();
	/*while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}*/
}
