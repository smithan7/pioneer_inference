/*
 * Costmap.h
 *
 *  Created on: Jul 12, 2016
 *      Author: andy
 */

#ifndef COSTMAP_H_
#define COSTMAP_H_

#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/video/tracking.hpp"

#include <vector>

using namespace std;
using namespace cv;

class Costmap {
public:
	Costmap();
	virtual ~Costmap();

	bool init_flag;
	int map_height, map_width;
	int ros_unknown, ros_wall, ros_free;
	vector<int8_t> updateCells(vector<int8_t> &occupancy_grid_array);
	Point getCellIndex(int l);
    int getArrayIndex( Point p);
    void initCostmap(int h, int w);

	void prettyPrintCostmap();
	void simulateObservation(Point pose, float obsRadius);
	vector<Point> viewPerim;

	// useful stuff
	int obsFree, infFree, domFree, unknown, obsWall, infWall, inflatedWall;
	// 1 = free space // 2 = inferred free space // 3 = domFree
	// 101 = unknown
	// 201 = wall // 202 = inferred wall // 203 = inflated wall
	Vec3b cObsFree, cInfFree, cObsWall, cInfWall, cUnknown, cError;

	Mat cells;
	Mat euclidDist; // array of distances
	Mat occ; // floats, holds 0-1 of occupancy of a cell; 0-0.2 = obsFree, 0.2-0.35 = infFree, 0.35-0.65 = unknown, 0.65-0.8=infWall, 0.8-1.0 = obsWall
	// use inference to seed the occGrid and also for mapping, track by 0.1 per observation with p(0.01) of a misreading;
	Mat searchReward;
	Mat reward;

	vector<Point> cellUpdates;
	vector<Point> hullBreaches;

	void getRewardMat(float w[3], float e[2], float spread);
	void displayThermalMat(Mat &mat, Point cLoc);
	void buildOccPlot();
	void updateOccGrid();
	void spreadSearchArea(float growthRate);
	void displaySearchReward();

	float getEuclidianDistance(Point a, Point b);

	float cumulativeAStarDist(Point sLoc, Point gLoc, Mat &cSet, vector<Point> &oSet, Mat &fScore, Mat &gScore);
	float aStarDist(Point sLoc, Point gLoc);
	vector<Point> aStarPath(Point sLoc, Point gLoc);

	// TODO only update portion that needs it
	//void updateCostmap(vector<vector<int> > cells, vector<int> value);

	Mat displayPlot;
	void buildCellsPlot(); // build nice display plot
	void showCostmapPlot(int index); // show nice display plot and number it
	void addAgentToCostmapPlot(Scalar color, vector<Point> myPath, Point cLoc);

	Mat createCostMat(); // Largely used for inference
	vector<Point> getImagePointsAt(Mat &image, int intensity); // convert mat to vector of points

	/*retired functions

	float getPercentObserved(Costmap &globalCostmap, Costmap &workingCostmap);
	float getPercentDominated(Costmap &globalCostmap, Costmap &workingCostmap);
	float getPercentObservedAndInferred(Costmap &globalCostmap, Costmap &workingCostmap);
	float getPercentObservedAndInferredCorrectly(Costmap &globalCostmap, Costmap &workingCostmap);
	float getPercentInferredCorrectly(Costmap &globalCostmap, Costmap &workingCostmap);
	float getPercentInferredWrongly(Costmap &globalCostmap, Costmap &workingCostmap);

	 */


};

#endif /* COSTMAP_H_ */
