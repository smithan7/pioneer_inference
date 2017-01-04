/*
 * Observer.cpp
 *
 *  Created on: Jul 14, 2016
 *      Author: andy
 */

#include "Observer.h"

Observer::Observer(Point cLoc){
	this->cLoc = cLoc;
}

Observer::~Observer(){}

void Observer::showCellsPlot(vector<Agent> agents){
	this->costmap.buildCellsPlot();
	this->addAgentsToCostmapPlot(agents, this->costmap.displayPlot);
	namedWindow("Observer::costMat", WINDOW_NORMAL);
	imshow("Observer::costMat", this->costmap.displayPlot);
}

void Observer::addAgentsToCostmapPlot(vector<Agent> agents, Mat &displayPlot){
	for(size_t i=0; i<agents.size(); i++){
		Scalar black(0,0,0);
		circle(displayPlot,agents[i].cLoc,2, agents[i].myColor,-1, 8);
		circle(displayPlot,agents[i].gLoc,2, black,-1, 8);
		circle(displayPlot,agents[i].gLoc,1, agents[i].myColor,-1, 8);
	}
}
