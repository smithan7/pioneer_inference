/*
 * GraphNode.h
 *
 *  Created on: Sep 16, 2016
 *      Author: andy
 */

#ifndef GRAPHNODE_H_
#define GRAPHNODE_H_


#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

using namespace cv;
using namespace std;


class GraphNode {
public:
	GraphNode();
	virtual ~GraphNode();

	int location[2];
	float reward;
	float cost;
	Mat observation;
	vector<GraphNode> nbrs;
	vector<float> transitions;

};

#endif /* GRAPHNODE_H_ */
