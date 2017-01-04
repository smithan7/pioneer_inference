/*
 * PRMGraph.h
 *
 *  Created on: Sep 16, 2016
 *      Author: andy
 */

#ifndef PRMGRAPH_H_
#define PRMGRAPH_H_

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


class PRMGraph {
public:
	PRMGraph();
	virtual ~PRMGraph();

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

	void updatePRMGraph(Costmap &costmap, int nodeSpacing, int nbrSpacing, int nPulls);
	void eraseNode(int index);

	bool checkVisibility(Costmap &costmap, float dist, vector<int> a, vector<int> b);

	// useful things
	float aStarDist(int sIndex, int gIndex, Costmap &costmap);
	int findNearestNode(Point in, Costmap &costmap);
	void displayCoordMap(Costmap &costmap, bool displayNodes);
	bool visibleLineCheck(Costmap &costmap, Point pose, Point pt);

};

#endif /* PRMGraph_H_ */
