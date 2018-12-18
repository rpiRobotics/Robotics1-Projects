#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"

#include <iostream>
#include <string>

/* This node controls the steering wheel of the car.
 * Given a desired lane-offset (distance off-center), current lane-offset, and current angle-offset
 * it will generate the rotational velocity the motor output should attempt to
 * achieve in order to quickly get in that lane.
 * 
 * This is done with the feedback controller from the lecture 05 notes
 */

using distance_t = float;
using angle_t = float;


/* Implements a feedback controller for the lane-offset of the bot.
 * Steering output is proportional to the angle of the bot compared to straight,
 * as well as to the distance the bot is from centered about its desired lane
 */
class SteerController
{
	static constexpr auto kp_angle_param   = "kp_angle";
	static constexpr auto kp_laneoff_param = "kp_laneoff";
	
private:  // Data Members
	double kp_angle{1.0}, kp_laneoff{1.0};
	
public:
	// initializes param server with defaults
	explicit SteerController(ros::NodeHandle &nh)
	{
		kp_angle = nh.param(kp_angle_param, kp_angle);
		kp_laneoff = nh.param(kp_laneoff_param, kp_laneoff);
		ROS_INFO("kp_angle: %f, kp_laneoff: %f", kp_angle, kp_laneoff);
	}
	
	angle_t steer(distance_t lane_err, angle_t angle_err) 
	{
		return -(kp_laneoff * lane_err) - (kp_angle * angle_err);
	}
	
	void update_params(ros::NodeHandle &nh)
	{
		nh.getParamCached(kp_angle_param, kp_angle);
		nh.getParamCached(kp_laneoff_param, kp_laneoff);
	}
};

/* Maintains the state of the steer controller.
 * Calculates errors, and provides pub/sub interface.
 */
class SteerState
{
private:  // Data Members
	distance_t desired{0.0}, actual{0.0};
	angle_t angle{0.0};

public:
	SteerController controller;
	SteerState(const SteerController& cont): controller(cont) {}
	
private:
	angle_t next_steer() {return controller.steer(actual - desired, angle);}
	
public:
	template<typename UpdateFn>
	auto getOnDesired(UpdateFn&& fn)
	{
		return [this, update = std::forward<UpdateFn>(fn)](auto&& val)
		{
			desired = val->data;
			update(next_steer());
		};
	}
	
	template<typename UpdateFn>
	auto getOnActual(UpdateFn&& fn)
	{
		return [this, update = std::forward<UpdateFn>(fn)](auto&& val)
		{
			actual = val->data;
			update(next_steer());
		};
	}
	
	template<typename UpdateFn>
	auto getOnAngle(UpdateFn&& fn)
	{
		return [this, update = std::forward<UpdateFn>(fn)](auto&& val)
		{
			angle = val->data;
			//update(next_steer());
		};
	}
};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "path_plan");
	
	ros::NodeHandle nh("path_plan");
	
	auto steer_pub = nh.advertise<std_msgs::Float32>("steer", 1000);
	auto update_fn = [steer_pub](angle_t steer_omega)
	{
		std_msgs::Float32 msg;
		msg.data = steer_omega;
		steer_pub.publish(msg);
	};
	
	SteerState steer_state{SteerController{nh}};
	
	constexpr auto sub_q = 1000;
	auto desired_offset_sub = nh.subscribe<std_msgs::Float32>(
		"desired_lane_offset", sub_q, steer_state.getOnDesired(update_fn));
	
	auto actual_offset_sub = nh.subscribe<std_msgs::Float32>(
		"actual_lane_offset", sub_q, steer_state.getOnActual(update_fn));
	
	auto angle_sub = nh.subscribe<std_msgs::Float32>(
		"lane_angle", sub_q, steer_state.getOnAngle(update_fn));
	
	ros::Rate loop_rate(1000);
	
	ROS_INFO("Starting path feedback controller!");
	while(ros::ok())
	{
		steer_state.controller.update_params(nh);
		for(int i = 0; i < 100 && ros::ok(); i++)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
	}	
	
	return 0;
}
