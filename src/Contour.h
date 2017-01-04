/*
 * Contour.h
 *
 *  Created on: Jul 29, 2016
 *      Author: andy
 */

#ifndef CONTOUR_H_
#define CONTOUR_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

#include "Costmap.h"

using namespace std;
using namespace cv;

class Contour{
public:

	Contour(Mat &matIn, vector<Point> pts);
	Contour();
	virtual ~Contour();

	// general stuff
	int area;
	int matRows;
	int matCols;
	void getCenter(Mat &cells);
	Point center;
	bool dominated; // am I dominated?

	float histogramLength;
	vector<float> histogram;
	vector<float> sequence;
	float meanLength;
	vector<float> length;

	// exploration stuff
	void getDominatedStatus( Costmap &costmap );
	void fillContourInCostmap( Costmap &costmap, int val );
	void getNbrsWithValue(Costmap &costmap, int value);
	vector<Point> nbrs; // [k][x,y,nbr value];
	vector<int> nbrValList;

	void getVals(Costmap &costmap);
	vector<int> vals; // value of points



	// structural inference stuff
	void getInvariantHistogram();
	void getSequenceHistogram();
	vector<Point> getContourExitPoints(int val);
	void drawHistogram(vector<float> histogram, char* title);


	vector<Point> points;

};

#endif /* CONTOUR_H_ */
