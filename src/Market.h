/*
 * Market.h
 *
 *  Created on: Sep 27, 2016
 *      Author: andy
 */

#ifndef MARKET_H_
#define MARKET_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include "std_msgs/Float32MultiArray.h"

using namespace cv;
using namespace std;

class Market {
public:
	Market();
	void init(int nAgents, int myIndex, bool enableRelaySacrifice);
	void shareMarket( Market &in );
	void updatecLoc( Point cLoc );
	void updategLoc( Point gLoc );
	void updateTime( float t );
	void updateExploreCost( float cost );
	bool comCheck(int a);
	void iterateTime();
	void printMarket();
	virtual ~Market();

	void dissasembleTransmission(const std_msgs::Float32MultiArray &transmission);
	std_msgs::Float32MultiArray assembleTransmission();

	vector<Point> cLocs;
	vector<Point> gLocs;
	vector<float> exploreCosts; // distance to explore

	vector<Point> rLocs; // relay loc
	vector<int> roles; // -1 = n/a, 0 = sacrifice, 1 = relay
	vector<int> mates; // -1 = n/a, # = index of corresponding relay sacrifice
	vector<bool> returnFlags; // true means returning

	vector<int> reportTimes; // how long until each should report
	vector<float> reportCosts; // distance to report
	vector<int> reportRequests; // did someone clear my need to report?

	vector<float> times;
	vector<float> batteryLeft;

	// my general stuff
	int myIndex;
	int nAgents;
};

#endif /* MARKET_H_ */