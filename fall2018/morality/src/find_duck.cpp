#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "find_duck");
	
	ros::NodeHandle n;
	
	auto hello_world = n.advertise<std_msgs::String>("hello_world", 1000);
	
	ros::Rate loop_rate(2);
	
	while(ros::ok())
	{
		std_msgs::String msg;
		
		msg.data = "hello, world";
		
		ROS_INFO("%s", msg.data.c_str());
		
		hello_world.publish(msg);
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}
	
	return 0;
}
