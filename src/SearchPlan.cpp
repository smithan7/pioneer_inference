/*
 * SearchPlan.cpp
 *
 *  Created on: Sep 16, 2016
 *      Author: andy
 */

#include "SearchPlan.h"

SearchPlan::SearchPlan() {
	targetSpeed = 1;
}

SearchPlan::~SearchPlan() {}

void SearchPlan::spreadTarget(){


	//for(size_t j=0; j<frontiers.size(); j++){
		// find nearest node on graph to each frontier and add uncertainty of +1

	//}


	for(size_t i= 0; i < thinGraph.nodeLocations.size(); i++){ // spread
		// check all nbrs and average their scores: myScore = nbrScore + myScore / 2; if i have 1 nbr but do for all nbrs


	}

}

