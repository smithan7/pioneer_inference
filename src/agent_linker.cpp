#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d.h>

#include "Agent.h"

int main(int argc, char *argv[])
{
	// initialization
	ros::init(argc, argv, "agent0");

	ros::NodeHandle nHandle("~");

	float obsThresh = 50;
	float comThresh = 50;
	int myIndex = 1;
	int numAgents = 2;

	Agent *agent = new Agent(nHandle);
	agent->init(myIndex, obsThresh, comThresh, numAgents);

	// return the control to ROS
	ros::spin();

	return 0;
}