/*
 * HidingAgent.cpp
 *
 *  Created on: Sep 19, 2016
 *      Author: andy
 */

#include "HidingAgent.h"

HidingAgent::HidingAgent() {
	// TODO Auto-generated constructor stub

}

HidingAgent::~HidingAgent() {
	// TODO Auto-generated destructor stub
}

void HidingAgent::plan(Costmap &costmap){

	if(cLoc == gLoc || cLoc.x == -1){

		while( true ){
			int x = rand() % costmap.cells.cols;
			int y = rand() % costmap.cells.rows;

			if(costmap.cells.at<uchar>(x,y) == costmap.obsFree){
				gLoc.x = x;
				gLoc.y = y;
				break;
			}
		}
		path = costmap.aStarPath(cLoc, gLoc);
	}
}

void HidingAgent::act(){
	cLoc = path[0];
	path.erase( path.begin() );
}

