/*
 * CostmapCoordination.h
 *
 *  Created on: Jul 12, 2016
 *      Author: andy
 */

#ifndef COSTMAPCOORDINATION_H_
#define COSTMAPCOORDINATION_H_

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

#include "Frontier.h"
#include "Market.h"

using namespace std;
using namespace cv;

class CostmapCoordination {
public:
	CostmapCoordination();
	virtual ~CostmapCoordination();

	// Frontiers
	vector<Frontier> frontiers; // graphs frontiers of item
	vector<Point> findFrontiers(Costmap &costmap); // search Graph and find Frontiers
	void clusterFrontiers(vector<Point> frntList, Costmap &costmap); // cluster Frontiers into groups

	// market frontiers
	Point marketFrontiers( Costmap &costmap, Point cLoc, Market &market);

	// useful functions
	void findClosestFrontier(Costmap &costmap, Point cLoc, int &goalIndex, float &goalDist);
	void plotFrontiers(Costmap &costmap, vector<Point> &frontierCells);

	/*
	vector< vector<int> > kMeansClusteringTravel(int numClusters, Costmap &costmap); // cluster using travel dist
	vector< vector<int> > kMeansClusteringEuclid(int numClusters, Costmap &costmap); // cluster using euclidian
	vector<vector<int> > centralMarket(Costmap &costmap, vector<vector<int> > cLoc);
	*/
};

#endif /* COSTMAPCOORDINATION_H_ */

