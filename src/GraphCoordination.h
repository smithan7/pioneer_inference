/*
 * GraphCoordination.h
 *
 *  Created on: Mar 14, 2016
 *      Author: andy
 */

#ifndef GraphCoordination_H_
#define GraphCoordination_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


#include "Frontier.h" // frontiers
#include "Graph.h"
#include "Market.h"

using namespace cv;
using namespace std;

class GraphCoordination {
public:
	GraphCoordination();
	virtual ~GraphCoordination();
	void init(float obsRadius, float comRadius);

	// for finding the optimal poses
	Graph thinGraph, poseGraph, travelGraph, tempGraph;

	float observedReward(Mat &observed, Mat &reward);
	void simulateObservation(Point pose, Mat &resultingView, Costmap &costmap);
	void findPosesEvolution(Costmap &costmap);
	void getOset(vector<int> &oStates, vector<int> &cStates, vector<int> &workingSet);

	void marketPoses( Costmap &costmap, Point cLoc, Point &gLoc, Market &market );
	float getPoseReward( Point in, Costmap &costmap);
	void getViews(Costmap &costmap, vector<int> cPoses, Mat &cView, float &cReward, vector<Point> &poses, vector<Mat> &views, vector<float> poseRewards, float &gReward);
	void plotPoses( Costmap &costmap, vector<int> &cPoses, vector<int> &oPoses, vector<Mat> &views, vector<Point> &poses);
	float getDiscountedRewards( Costmap &costmap, vector<Point> &locs, int i, float globalReward );
	void simulateNULLObservation(Point pose, Mat &resultingView, Costmap &costmap);



	Point posePathPlanningTSP(Graph &graph, Costmap &costmap, vector<Point> &agentLocs, int &myIndex);
	void displayPoseTours(Costmap &costmap);
	void tspPoseFullTourPlanner(vector<Point> &agentLocs, int &myIndex);
	float getGraphObservations(Graph &graph, Costmap &costmap, Mat &gView, vector<int> &workingSet);

	void findPoseGraphTransitions(Graph &graph, Costmap &costmap);
	void displayPoseGraph(Costmap &costmap);
	void findPosesEvolution(Graph &graph, Costmap &costmap, vector<Point> &agentLocs);
	int matReward(Mat &in);
	vector<int> getWorkingSet(Graph &graph, Costmap &costmap);
	float getCurrentPoseSetReward(Graph &graph, Costmap &costmap, Mat &cView, vector<int> &workingSet);
	void getOset(Graph &graph, Costmap &costmap, vector<int> &oStates, vector<int> &cStates, vector<int> &workingSet);

	int nPullsTSP;
	int nPullsEvolvePoseGraph;
	int minObsForPose;
	vector<Point> viewPerim, comPerim;
	void simulateCommunication(Point pose, Mat &resultingView, Costmap &costmap);

	vector<vector<int> > tspPoseTours; // [agent, pose along tour]
	vector<vector<Point > > tspPoseToursLocations;



	/*
	void createGraphCoordination(Mat &bw); // create GraphCoordination from explore image, change to do from explore graph, on my states not pixels
	void importInferenceMat(Mat &inferredMat);
	Mat inferredMat;

	void findPointOfIntereststates(); // use edge detector to find poi states for GraphCoordination, result is too sparse
	void breadthFirstSearchAssembleGraphCoordination(); // search through the graph looking for connections using Dijstras
	int grafSpacing;
	float grafConnectionSpacing;
	void findStatesCityBlockDistance(Mat &thinMat); // use city block distance to find states
	void findStateTransitionsCityBlockDistance(); // use city block distance to get dist graph
	void getUnobservedMat(Mat &inputMat);
	void getInferenceMat(Mat &inputMat);

	// inference tools
	void extractInferenceContour();
	void growObstacles();
	Mat breadthFirstSearchFindRoom(Mat &src, vector<int> pt); // search along walls, need to find way to exclude dominated points (do by searching next to wall in observed space!)
	void growFrontiers(vector<Frontier> frnt); // grow frontiers towards eachother
	void invertImageAroundPt(Mat &src, Mat &dst, vector<int> cLoc);

	// observation tools
	bool lineTraversabilityCheck(Mat &tSpace, vector<int> sPt, vector<int> fPt, int fValue);
	bool bisectionCheck(vector<int> a, vector<int> b); // for checking visibility
	bool bresenhamLineCheck(vector<int> cLoc, vector<int> cPt); // for checking visibility
	void simulateObservation(int state, Mat &viewMat);
	vector<vector<int> > corners;
	void cornerFinder(Mat &inputImage);
	Mat costMap;
	Mat obstacleMat;
	Mat freeMat;
	Mat rewardMat;
	Mat unknownMat;
	Mat inferenceMat;
	vector<vector<int> > viewPerim; // list of perimeter states on circle around the UAV that represent all points viewable, use for observation check

	void mctsPathPlanner(vector<int> &path, float maxDist, int maxPulls);
	vector<int> tspPathPlanner(float maxDist, int maxPulls);

	int masterGraphPathPlanning(float maxLength, int nPullsTSP, int nPullsEvolveMasterstates);
	vector<int> tspMasterGraphPathPlanner(int maxPulls, float maxDist);
	float tspMasterGraphEvaluatePath(vector<int> &path, float maxLength);
	vector<int> tspMasterGraphBuildPath();
	vector<int> tspMasterGraphEvolvePath(vector<int> inPath);

	//vector<Node> states;
	vector<vector<float> > transitions;
	//vector<Node> masterStates;
	vector<vector<float> > masterTransitions;

	void findMasterStatesEvolution(int nPulls);
	void findMasterTransitionMatrix();
	void mergeStatesBySharedNbr(); // merge states together based on shared nbrs

	int getRandomNbr(int state, float remDist);
	vector<int> buildPath(float maxDist);
	vector<int> modifyPath(vector<int> path, float maxdist);
	float getPathReward(vector<int> path, vector<Mat> &observations);

	float matReward(Mat &in); // calc how much of the Mat has been observed.

	void condenseGraph(); // find near states and bring them together
	void breadthFirstSearchstateConnections(vector<vector<int> > openstates);
	void floodForDist(); // create the distgraph
	void thinning(const Mat& src, Mat& dst); // used to create GraphCoordination
	void thinningIteration(Mat& img, int iter); // used to create GraphCoordination

	void displayCoordMap(); // draw the coordination map, locations and

	void findMystate(vector<int> cLoc);
	int findNeareststate(vector<int> in); // find the state closest to point
	//void frontierCosts(); // distance to each frontier cluster
	void importFrontiers(vector<vector<int> > frntList); // bring frontiers in from graph
	void getstateCosts(int cstate); // distance to travel to each state
	void getstateRewards(); // reward for each state, currently number of frontiers around it
	void getstateValues(); // value = reward - cost

	int getMaxIndex(vector<float> value); //
	vector<int> greedystateSelect(); // go to nearest state with frontiers
	vector<int> marketstateSelect(); // go to the highest value state

	vector<int> aStar(int strt, int gl);
	float aStarDist(int sLoc, int eLoc); // distance along travel graph from sLoc to eLoc
	float cityBlockDist(vector<int> a, vector<int> b); // fast dist calc
	float euclidianDist(vector<int> a, vector<int> b); // accurate dist calc

	void importUAVLocations(vector<vector<int> > cLocList);
	vector<vector<int> > cLocListMap;
	vector<int> cLocListstates;

	void myThinning(vector<graphstate> &space);
	void createGraphCoordination2(vector<graphstate> &graf);
	bool checkstateForThinning(int x, vector<graphstate> &graf); // should this state be pruned?
	bool checkPerimDerivative(int x, vector<graphstate> &graf);
	bool checkSumNbrs(int x, vector<graphstate> &graf);
	bool checkNbrs(int x, vector<graphstate> &graf, int condit);

	vector<graphstate> miniSpace;
	//void buildTree();

	Mat miniImage;
	int nmstates; // number of states on graph
	vector<vector<int> > graf; // list of all states on the GraphCoordination
	vector<vector<float> > distGraph; // distances between each state on the travel graph
	float scaling; // how much is my image scaled down from graph

	vector<vector<int> > cLocList; // list of UAV locations

	vector<vector<int> > frontiers; // list of frontier centers
	vector<float> frntCost; // cost to travel to each frontier along the graph
	vector<vector<int> > stateFrontiers; // frontiers at each state

	vector<float> stateCost; // cost to travel to each state
	vector<float> stateReward; // reward of each state
	vector<float> stateValue; // value of each state

	int cState; // which state am I located at
	*/
};

#endif /* GraphCoordination_H_ */