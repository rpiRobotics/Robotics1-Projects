#include "ros/ros.h"
#include "duckietown_msgs/ObstacleProjectedDetection.h"
#include "duckietown_msgs/ObstacleProjectedDetectionList.h"
#include "morality/judge.h"

#include <vector>
#include <algorithm>

float get_lane_radius(ros::NodeHandle& nh)
{
	return nh.param("lane_radius", 10.0);
}

float get_duck_radius(ros::NodeHandle& nh)
{
	return nh.param("duck_radius", 1.0);
}

float get_car_width(ros::NodeHandle& nh)
{
	return nh.param("car_width", get_lane_radius(nh) - 2);
}

// Gets all the x-locations of the ducks and returns them sorted
std::vector<float> get_ducks(const duckietown_msgs::ObstacleProjectedDetectionList& obstacles)
{
	std::vector<float> ducks(obstacles.list.size());
	std::transform(obstacles.list.begin(), obstacles.list.end(), ducks.begin(),
				   [](const auto& obstacle) {return obstacle.location.x;});
	std::sort(ducks.begin(), ducks.end());
	
	return ducks;
}

/* Gets all the borders of ducks and returns them sorted, indicating whether
 * each border is a left or right border
 */
std::vector<std::pair<float, bool>> get_duck_borders(
	ros::NodeHandle& nh,
	const duckietown_msgs::ObstacleProjectedDetectionList& obstacles)
{
	const auto duck_radius = get_duck_radius(nh);
	const auto lane_radius = get_lane_radius(nh);
	
	std::vector<std::pair<float, bool>> ducks;
	ducks.push_back(std::make_pair(-2*lane_radius, true));  // far-left
	ducks.reserve(obstacles.list.size() * 2 + 1);
	
	std::transform(obstacles.list.begin(), obstacles.list.end(), std::back_inserter(ducks),
		[duck_radius](const auto& obstacle)
		{
			return std::make_pair(obstacle.location.x - duck_radius, true);
		});
	
	std::transform(obstacles.list.begin(), obstacles.list.end(), std::back_inserter(ducks),
	   [duck_radius](const auto& obstacle)
	   {
			return std::make_pair(obstacle.location.x + duck_radius, false);
	   });
	
	std::sort(ducks.begin(), ducks.end(),
			  [](const auto& lhs, const auto& rhs)
			  {
				  return lhs.first < rhs.first;
			  });
	return ducks;
}

/* Given the sorted spread of ducks, iterates between the start and end of
 * each duck from the left side of the lane to the right, searching for the
 * lane-offset which results in the fewest collisions with these ducks.
 * It does this by iterating on left and right bounds of the ducks.
 */ 
float fewest_deaths(
	ros::NodeHandle& nh,
	const duckietown_msgs::ObstacleProjectedDetectionList& obstacles)
{
	const auto lane_radius = get_lane_radius(nh);
	const auto car_width = get_car_width(nh);
	
	const auto borders = get_duck_borders(nh, obstacles);
	
	auto left_duck = borders.begin();
	
	float left_pos = -lane_radius;
	
	auto right_duck =
		std::lower_bound(borders.begin(), borders.end(), left_pos + car_width,
			[](const auto& lhs, const auto& rhs)
			{
				return lhs.first < rhs;
			}
		);
	
	// count left-borders
	int deaths = std::count_if(left_duck+1, right_duck, 
							   [](const auto& bord) {return bord.second;});
	
	if(deaths == 0)
		return left_pos;
	
	int best_deaths = deaths;
	float best_pos = left_pos;
	
	while(right_duck < borders.end())
	{
		// move right until before a death
		for(; right_duck < borders.end() && !right_duck->second; right_duck++);
		
		float right_pos = right_duck < borders.end()? right_duck->first:lane_radius;
		left_pos = right_pos - car_width;
		
		// progress left-edge, counting ducks-saved
		for(; left_duck < right_duck && left_duck->first < left_pos; left_duck++)
			if(!left_duck->second)
				deaths--;
		
		if(deaths <= best_deaths)
		{
			best_deaths = deaths;
			best_pos = left_pos;
		}
		
		// get the next death
		if(right_duck < borders.end())
		{
			right_duck++;
			deaths++;
		}
	}
	
	return best_pos;
}

/* This node hosts a service which determines the most optimal lane-offset for
 * the car to travel through a series of obstacles on.
 * The heuristics determine what the goal of the car is, and determine what function
 * is used to calculate the result.
 * 
 * The service input is a list of obstacles in robot-space. The obstacles are "ducks"
 * with a known width, and the car itself also has a known width.
 */
int main(int argc, char* argv[])
{
	using namespace morality;
	ros::init(argc, argv, "judgement");
	
	ros::NodeHandle nh("judgement");
	
	constexpr auto heuristic_param = "heuristic";
	if(!nh.hasParam(heuristic_param))
		nh.setParam(heuristic_param, judge::Request::DONT_MOVE);
	
	auto service = nh.advertiseService<judge::Request, judge::Response>(
		"judge", [&nh](judge::Request& req, judge::Response& resp)
		{
			ROS_INFO("Received judgement request");
			using Request = judge::Request;
			
			int heuristic = req.heuristic;
			
			if(heuristic == Request::DEFAULT)
				nh.getParamCached(heuristic_param, heuristic);
			
			switch(heuristic)
			{
			case Request::FEWEST_DEATHS:
				ROS_INFO("Fewest deaths heuristic selected");
				resp.lane = fewest_deaths(nh, req.obstacles);
				break;
			case Request::LEAST_INJURY:
				ROS_INFO("Least injury heuristic selected");
				break;
			case Request::LEAST_COST:
				ROS_INFO("Least cost heuristic selected");
				break;
			default:
			case Request::DONT_MOVE:
				ROS_INFO("Dont move heuristic selected");
				resp.lane = 0.0;
				resp.collided = 0;
				resp.score = 42.0;
				break;
			}
			
			return true;
		});
	
	ros::Rate loop_rate(100);
	
	ROS_INFO("Judgement node started!");
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
