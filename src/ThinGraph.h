/*
 * ThinGraph.h
 *
 *  Created on: Sep 16, 2016
 *      Author: andy
 */

#ifndef THINGRAPH_H_
#define THINGRAPH_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

#include "Costmap.h" // includes - n/a

using namespace cv;
using namespace std;


class ThinGraph {
public:
	ThinGraph();
	virtual ~ThinGraph();

	// build graph
	int nodeSpacing;
	int nbrSpacing;

	int cNode; // my location
	vector<Point> nodeLocations;
	vector<vector<int> > nodeNbrs;
	vector<vector<float> > nodeTransitions;
	vector<Mat> nodeObservations;
	vector<int> nodeSubGraphIndices;
	vector<int> nodeUpdates;

	vector<int> cPath;

	void clearTransitions( int i );

	void createThinGraph(Costmap &costmap, int nodeSpacing, int nbrSpacing);
	void updateThinGraph(Costmap &costmap, int nodeSpacing, int nbrSpacing);

	void createPRMGraph(vector<int> cLoc, Costmap &costmap, int nodeSpacing, int nbrSpacing);
	void thinning(const Mat& src, Mat& dst, int limits[4]);
	void thinningIteration(Mat& img, int iter, int limits[4]);
	void findStatesByGrid(Mat &inMat);
	vector<Point> findStatesCityBlockDistance(Mat &thinMat, int limits[4]);
	void mergeTempNodesIntoGraph( vector<Point> tLocs, Costmap &costmap, Mat &obsMat, Mat &thinMat );
	void eraseNode(int index);

	void findCNodeTransitions(Costmap &costmap);
	void findStateTransitionsCityBlockDistance(Costmap &costmap);
	void findStateTransitionsByVisibility(Costmap &costmap);
	bool checkVisibility(Costmap &costmap, float dist, vector<int> a, vector<int> b);
	void mergeStatesBySharedNbr();

	// useful things
	float aStarDist(int sIndex, int gIndex, Costmap &costmap);
	int findNearestNode(Point in, Costmap &costmap);
	void displayCoordMap(Costmap &costmap, bool displayNodes);
	bool visibleLineCheck(Costmap &costmap, Point pose, Point pt);

};

#endif /* THINGRAPH_H_ */
