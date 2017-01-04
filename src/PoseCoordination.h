/*
 * PoseCoordination.h
 *
 *  Created on: Jul 12, 2016
 *      Author: andy
 */

#ifndef POSECOORDINATION_H_
#define POSECOORDINATION_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

#include "World.h"
#include "Frontier.h"
#include "Agent.h"
#include "Contour.h"
#include "Inference.h"

class PoseCoordination {
public:
	PoseCoordination();
	virtual ~PoseCoordination();
};

#endif /* POSECOORDINATION_H_ */
