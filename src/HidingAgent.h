/*
 * HidingAgent.h
 *
 *  Created on: Sep 19, 2016
 *      Author: andy
 */

#ifndef HIDINGAGENT_H_
#define HIDINGAGENT_H_

#include "Costmap.h"

class HidingAgent {
public:
	HidingAgent();
	virtual ~HidingAgent();


	Point cLoc, gLoc;

	vector<Point> path;

	void plan(Costmap &costmap);
	void act();


};

#endif /* HIDINGAGENT_H_ */
