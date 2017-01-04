/*
 * Pose.h
 *
 *  Created on: Nov 2, 2016
 *      Author: andy
 */

#ifndef POSE_H_
#define POSE_H_

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include "math.h"

#include "Costmap.h"

using namespace std;
using namespace cv;

class Pose {
public:
	Pose(Point loc, Costmap &costmap);
	virtual ~Pose();
	void getPoseHistogram(Costmap &costmap);


	// location in mat
	Point2f loc;

	// for setting up the histogram
	float radius;
	int nSamples;

	// for histogram comparison
	float reward;
	vector<Point2f> obsLim;
	vector<float> obsLen;
	vector<int> obsVal;
	float mean;
	float stanDev;

	// for map merging
	int orient, direction;
	vector<Point2f> rotLim;

	// is it a library or one to be inferred over?
	bool needInference;

	// is a library, get mat
	vector<Point> obsWalls, obsFree;
	void getObservedCells(Costmap &costmap);
	Mat makeMat();

	void insertPoseInCostmap(Costmap &costmap, Point2f oLoc, Mat &mat);
	void rotateLimits();

	void getMean();
	void getStanDev();
	float getPDF( float x );
	float getCDF( float x );
	void drawHistogram(char* title);
};

#endif /* POSE_H_ */
