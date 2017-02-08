/*
 * Agent.h
 *
 *  Created on: Jun 8, 2016
 *      Author: andy
 */
/*
 * Agent.h
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */

#ifndef SRC_Agent_H_
#define SRC_Agent_H_

#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
 #include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h> // action server
#include <visualization_msgs/Marker.h> // for making marks in rviz
#include <costmap_2d/costmap_2d.h>
#include <vector>
#include <utility>
#include <queue>
#include <fstream>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int8MultiArray.h"


#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include "CostmapCoordination.h" // includes - world.h, frontier.h
#include "CostmapPlanning.h"

#include "Graph.h" // includes - costmap.h, node.h
#include "GraphCoordination.h" // includes - frontier.h, TreeNode.h, Node.h
//#include "GraphPlanning.h"

#include "Inference.h" // includes - frontiers.h, contours.h
#include "Market.h" // for graphCoordination

using namespace std;
using namespace cv;

class Agent{
public:

	ros::Subscriber costSub, locSub, marketSub_A, mapUpdatesSub_A, marketSub_B, mapUpdatesSub_B;
	ros::Publisher markerPub, goalPub, marketPub, locPub, mapUpdatesPub;

	void costMapCallback(const nav_msgs::OccupancyGrid& cost_in );
	void locationCallback( const nav_msgs::Odometry& locIn);
	void marketCallback( const std_msgs::Float32MultiArray &marketOrders);
	void mapUpdatesCallback_A( const std_msgs::Int16MultiArray& transmission );
	void mapUpdatesCallback_B( const std_msgs::Int16MultiArray& transmission );

    double alignCostmap( Mat &set, Mat &sub, Mat &homography);
    Mat A_homography, B_homography, A_cells, B_cells;
    void getWallPts(Mat &mat, vector<Point2f> &pts);
    void getFreePts(Mat &mat, vector<Point2f> &pts);
	void plotMatches( vector<Point2f> &p_set, vector<Point2f> &res, vector<Point2f> &pair);
	Mat generateStartingConfig(Mat &set, Mat &sub_in);
	double linearDist(vector<Point2f> &p_sub, vector<Point2f> &p_set, vector<Point2f> &pair, vector<Point2f> &m_sub, float tol);
	void initMap( const std_msgs::Int16MultiArray& transmission, Point shift, float angle, Mat &free, Mat &wall );



	void publishNavGoalsToMoveBase();
	void publishRvizMarker(Point loc, float radius, int color, int id);
	void publishMarket();
	void publishMapUpdates();
	void publishLoc(const nav_msgs::Odometry& locIn);



	int iterCntr, iterPeriod;

	// agent stuff
	Agent(ros::NodeHandle nh);
	void init(int myIndex, float obsThresh, float comThresh, int numAgents);
	bool locationInitialized, costmapInitialized, marketInitialized;
	void pickMyColor();
	~Agent();
	void shareCostmap(Costmap &A, Costmap &B);
	void showCellsPlot();
	void shareGoals(vector<int> inG, int index);
	int myIndex;
	Scalar myColor;
	float comThresh;
	float obsThresh;
	bool explorationComplete;
	
	bool checkReportTime();
	bool lastReport(); // is my battery almost dead and I shouldn't report to do last bit of exploring
	Point reportToOperator();
	float getDistanceToReportToOperator(); // report means coms with
	float travelSpeed; // my travelling speed
	
	bool reportFlag; // am I reporting now?
	ros::Time timeOfLastReport;
	ros::Time timeOfNextReport;
	ros::Duration checkTimeReport;
	float reportInterval; // how often I am supposed to report

	void marketReturnInfo();
	bool checkReturnTime();
	Point returnToOperator();
	bool checkForExplorationFinished();
	float getDistanceToReturnToOperator(); // return means travel to
	
	bool returnFlag; // time to return
	ros::Duration checkTimeReturn;
	ros::Time missionTime; // what time is it currently
	ros::Time maxMissionTime; // when do I have to be back by?

	void marketRelaySacrifice();
	Point reportToRelayPt();
	Point returnToRelayPt();
	float getDistanceToReportToRelayPt(); // report means coms with
	float getDistanceToReturnToRelayPt(); // return means travel to
	float getDistanceToReportToOperatorFromRelayPt();
	float getDistanceToReturnToOperatorFromRelayPt();
	bool relayFlag, sacrificeFlag;
	vector<int> sacrificeList;
	int myRelay;

	Market market;
	std_msgs::Float32MultiArray marketOrders;

	// working
	void infer(string inferenceMethod, World &World);
	void plan(string planMethod);
	float lastPlan;
	void planExplore(string planMethod);
	void act();
	ros::Time actTimer;

	Point cLoc, gLoc, offset, oLoc, rLoc, gLocPrior; // for map
	vector<Point> myPath, agentLocs;

	// costmap class stuff
	Costmap costmap;
	CostmapCoordination costmapCoordination;
	CostmapPlanning costmapPlanning;

	// graph class stuff
	GraphCoordination graphCoordination;
	int cNode;
	int gNode;
	vector<int> nodePath;

	// inference stuff
	Inference inference;

	int marketNodeSelect(World &gMap);
	void greedyFrontiers();
};

#endif /* SRC_Agent_H_ */
