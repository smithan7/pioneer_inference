/*
 * GraphPlanning.h
 *
 *  Created on: Jul 13, 2016
 *      Author: andy
 */

#ifndef GRAPHPLANNING_H_
#define GRAPHPLANNING_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <time.h>

#include "Graph.h"
#include "TreeNode.h"
#include "ThinGraph.h"
#include "PRMGraph.h"

using namespace cv;
using namespace std;

class GraphPlanning {
public:
	GraphPlanning();
	virtual ~GraphPlanning();

	void init(float obsRadius );
	void marketPoses( Costmap &costmap, Point cLoc, Point &gLoc );
	float getPoseReward( Point in, Costmap &costmap);
	void getViews(Costmap &costmap, vector<int> cPoses, Mat &cView, float &cReward, vector<Point> &poses, vector<Mat> &views, vector<float> poseRewards, float &gReward);
	void plotPoses( Costmap &costmap, vector<int> &cPoses, vector<int> &oPoses, vector<Mat> &views, vector<Point> &poses);
	float getDiscountedRewards( Costmap &costmap, vector<Point> &locs, int i, float globalReward );
	void simulateNULLObservation(Point pose, Mat &resultingView, Costmap &costmap);

	// for finding the optimal poses
	Graph poseGraph, travelGraph, thinGraph;
	//ThinGraph thinGraph2;
	PRMGraph prmGraph;

	Mat lastView;
	vector<Point> hullBreaches;

	int nPullsEvolvePoseGraph;
	int minObsForPose;
	vector<Point> viewPerim;
	vector<Point> poseSet;
	vector<int> nodeIndices;

	bool gNodeValueCheck(Point cLoc, Point gLoc, Costmap &costmap);
	void checkTSPPosePathNodes(Costmap &costmap);

	void findPoseSet(Mat &costMat);
	void simulateObservation(Point pose, Mat &resultingView, Costmap &costmap);
	bool lineTraversabilityCheck(Mat &tSpace, Point sPt, Point fPt, int fValue);
	bool bresenhamLineCheck(Point cLoc, Point cPt);
	bool bisectionCheck(Point a, Point b);
	void findPosesEvolution(Costmap &costmap);
	vector<int> getWorkingSet(Costmap &costmap);
	int matCount(Mat &in);
	float observedReward(Mat &observed, Mat &reward);
	float getGraphObservations(Graph &graph, Costmap &costmap, Mat &gView, vector<int> &workingSet);
	float getCurrentPoseSetReward(Graph &graph, Costmap &costmap, Mat &cView, vector<int> &workingSet);
	void getOset(vector<int> &oStates, vector<int> &cStates, vector<int> &workingSet);

	// set up pose graph
	void mergePoseGraphIntoThinGraph();
	vector<int> getCSatesFromWorkingSet( vector<int> workingSet );
	void findPoseGraphTransitions(Costmap &costmap);
	void findGraphTravelGraphConnectors(Graph &graph, Costmap &costmap);
	void displayPoseGraph(Costmap &costmap);

	// MCTS over the optimal poses
	Point MCTSPosePathPlanning(float maxLength, Graph &graph, Costmap &costmap);

	// TSP over the optimal poses
	int nPullsTSP;
	vector<int> tspPosePath;
	vector<Point> tspPosePathLocations;

	Point TSPPosePathPlanning(Costmap &costmap);
	void tspPoseFullPathPlanner();
	vector<int> tspPoseEvolveFullPath(vector<int> path);


	void tspPosePathPlanner(float maxLength);
	float getNodeReward(vector<int> path, int node);
	vector<int> tspBuildRandomPath(float maxLength);
	void tspBuildGreedyPath(vector<int> &path);
	vector<int> tspEvolvePath(vector<int> inPath, float maxLength);
	float tspEstimatePathReward(vector<int> path);
	float tspEstimatePathLength(vector<int> path);
	void tspTrimPathToLength(vector<int> &path, float maxLength);
	void displayPosePath(Costmap &costmap);

	// planning the path general functions
	int getRandomNbr(int state, float remDist);

	// mcts planner
	int mctsPathPlanner(vector<int> &path, float maxLength, int maxPulls);
};

#endif /* GRAPHPLANNING_H_ */
