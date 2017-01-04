/*
 * CostmapPlanning.h
 *
 *  Created on: Jul 13, 2016
 *      Author: andy
 */

#ifndef COSTMAPPLANNING_H_
#define COSTMAPPLANNING_H_

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

#include "Costmap.h"
#include "Contour.h"

using namespace std;

class CostmapPlanning {
public:
	CostmapPlanning();
	virtual ~CostmapPlanning();

	Point greedyFrontierPlanner(Costmap &costmap, Point cLoc);

	vector<Contour> getSearchContours(Costmap &costmap);
	vector<Contour> getExploreContours(Costmap &costmap);
	vector<Contour> getMappingContours(Costmap &costmap);

	Point explorePlanner(Costmap &costmap, Point cLoc);
	Point mappingPlanner(Costmap &costmap, Point cLoc);
	Point searchPlanner(Costmap &costmap, Point cLoc);

	/*

	// main coordination

	vector<Frontier> frontiers;

	// greedy frontier planning
	vector<int> findNearestFrontier(); // find the closest Frontier to me

	vector< vector<int> > kMeansClusteringTravel(vector<int> openFrnt, int numClusters, World &gMap); // cluster using travel dist
	vector< vector<int> > kMeansClusteringEuclid(vector<int> openFrnt, int numClusters, World &gMap); // cluster using euclidian

	*/

};

#endif /* COSTMAPPLANNING_H_ */
