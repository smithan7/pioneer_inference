/*
 * World.h
 *
 *  Created on: Mar 28, 2016
 *      Author: andy
 */

#ifndef World_H_
#define World_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

#include "Costmap.h"

using namespace cv;
using namespace std;

class World {
public:
	World(string fName, int gSpace, float obsThresh, float comThresh);
	virtual ~World();

	// initialize world
	void initializeMaps(Mat &image, int gSpace); // initialize cost map, point map
	void getDistGraph();

	// for working in the world
	Costmap costmap; // what the agent uses to navigate
	vector<Point> viewPerim;
	void observe(Point cLoc, Costmap &costmap);
	vector<Point> getObservableCells(Point p);

	float obsThresh; // how far can I see? LOS
	float commThresh; // how far can I communicate, LOS

	/*
	Mat createMiniMapImg(); // for making perfect miniMapImg
	void saveWorldToYML(string fName); // save a world map to YML
	void pullWorldFromYML(string fName); // pull world from yml
	void getObsGraph(); // find nodes that can see eachother
	void getCommGraph(); // find nodes that can communicate with eachother
	 */


};

#endif /* World_H_ */
