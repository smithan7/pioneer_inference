/*
 * CumulativePathPlanner.h
 *
 *  Created on: Nov 15, 2016
 *      Author: andy
 */

#ifndef CUMULATIVEPATHPLANNER_H_
#define CUMULATIVEPATHPLANNER_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

using namespace std;
using namespace cv;

#include "Costmap.h"

class CumulativePathPlanner {
public:
	CumulativePathPlanner(Costmap &costmap, Point gLoc);
	virtual ~CumulativePathPlanner();

	float cumulativeAStarDist(Costmap &costmap, Point sLoc);
	vector<Point> cumulativeAStarPath(Costmap &costmap, Point sLoc);

	Mat cSet, oSet, gScore, fScore;
	vector<Point> oVec;
	Point gLoc;
};

#endif /* CUMULATIVEPATHPLANNER_H_ */
