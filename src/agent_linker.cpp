#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d.h>

#include "Agent.h"

int main(int argc, char *argv[])
{
	// initialization
	ros::init(argc, argv, "agent2");

	ros::NodeHandle nHandle("~");

	int myIndex = 2;
	int numAgents = 3;
	Agent *agent = new Agent(nHandle, myIndex, numAgents);
	

	// return the control to ROS
	ros::spin();

	return 0;
}