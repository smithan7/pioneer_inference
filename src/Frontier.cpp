/*
 * Frontier.cpp
 *
 *  Created on: Apr 9, 2016
 *      Author: andy
 */

#include "Frontier.h"

Frontier::Frontier(vector<Point> q) {
	projection.x = -1;
	projection.y = -1;
	projectionDistance = 2;

	center.x = -1;
	center.y = -1;

	orient[0] = -1;
	orient[1] = -1;

	reward = 0;
	cost = INFINITY;
	area = 0;

	editFlag = true;

	members = q;
}

Frontier::~Frontier() {

}

void Frontier::getCenter(){
	float tx = 0;
	float ty = 0;
	for(size_t k=0; k<this->members.size(); k++){ // and get cumulative distance to all other members
		tx += members[k].x;
		ty += members[k].y;
	}

	center.x = round(tx / (float)this->members.size());
	center.y = round(ty / (float)this->members.size());

}

void Frontier::getCentroid(Costmap &costmap){
	float minDist = INFINITY;
	int minDex;
	for(size_t j=0; j<this->members.size(); j++){ // go through each cluster member
		float tempDist = 0;
		for(size_t k=0; k<this->members.size(); k++){ // and get cumulative distance to all other members
			tempDist += costmap.getEuclidianDistance(members[k], members[j]);
		}
		if(tempDist < minDist){
			minDist = tempDist;
			minDex = j;
		}
	}
	this->center = members[minDex];
}

void Frontier::getProjection(){
	projection.x = center.x - round( projectionDistance*orient[1]);
	projection.y = center.y - round( projectionDistance*orient[0]);
}


void Frontier::getOrientation(Costmap &costmap){
	orient[0] = 0;
	orient[1] = 0;
	float count = 0;
	// check each member of each cluster
	for(size_t j=0; j<this->members.size(); j++){
		// check 4Nbr for being unobserved
		int xP = this->members[j].y;
		int yP = this->members[j].x;
		if(costmap.cells.at<short>(xP+1, yP) != costmap.obsFree && costmap.cells.at<short>(xP+1, yP) != costmap.obsWall){
			this->orient[0] += 1;
			count++;
		}
		if(costmap.cells.at<short>(xP-1, yP) != costmap.obsFree && costmap.cells.at<short>(xP-1, yP) != costmap.obsWall){
			this->orient[0] -= 1;
			count++;
		}
		if(costmap.cells.at<short>(xP, yP+1) != costmap.obsFree && costmap.cells.at<short>(xP, yP+1) != costmap.obsWall){
			this->orient[1] += 1;
			count++;
		}
		if(costmap.cells.at<short>(xP, yP-1) != costmap.obsFree && costmap.cells.at<short>(xP, yP-1) != costmap.obsWall){
			this->orient[1] -= 1;
			count++;
		}
	}
	if(count > 0){
		this->orient[0] /= count;
		this->orient[1] /= count;
	}
}