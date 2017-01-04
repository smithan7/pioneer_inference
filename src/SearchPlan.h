/*
 * SearchPlan.h
 *
 *  Created on: Sep 16, 2016
 *      Author: andy
 */

#ifndef SEARCHPLAN_H_
#define SEARCHPLAN_H_

#include "Graph.h"


class SearchPlan {
public:
	SearchPlan();
	virtual ~SearchPlan();

	Graph thinGraph;
	float targetSpeed;

	void spreadTarget();
};

#endif /* SEARCHPLAN_H_ */
