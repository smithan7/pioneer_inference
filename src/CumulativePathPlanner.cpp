/*
 * CumulativePathPlanner.cpp
 *
 *  Created on: Nov 15, 2016
 *      Author: andy
 */

#include "CumulativePathPlanner.h"

bool pointOnMat(Point &a, Mat &b);
bool pointCompare(Point &a, Point &b);

CumulativePathPlanner::CumulativePathPlanner(Costmap &costmap, Point gLoc) {
	Mat cSet = Mat::zeros(costmap.cells.size(), CV_16SC1); // 1 means in closed set, 0 means not
	Mat oSet = Mat::zeros(costmap.cells.size(), CV_16SC1); // 1 means in open set, 0 means not

	Mat gScore = Mat::ones(costmap.cells.size(), CV_32FC1)*INFINITY; // known cost from initial node to n
	Mat fScore = Mat::ones(costmap.cells.size(), CV_32FC1)*INFINITY; // known cost from initial node to n

	this->gLoc = gLoc;

}

CumulativePathPlanner::~CumulativePathPlanner() {
	// TODO Auto-generated destructor stub
}

float CumulativePathPlanner::cumulativeAStarDist(Costmap &costmap, Point sLoc){

	//cerr << "Costmap::aStarDist::in" << endl;


	if(sLoc == gLoc){
		return 0;
	}
	oVec.push_back(sLoc);
	oSet.at<short>(sLoc) = 1; // starting node has score 0
	gScore.at<float>(sLoc)  = 0; // starting node in open set
	fScore.at<float>(sLoc) = sqrt(pow(sLoc.x-gLoc.x,2) + pow(sLoc.y-gLoc.y,2));
	fScore.at<float>(gLoc) = 1;

	// for nbrs
	int nx[8] = {-1,-1,-1,0,0,1,1,1};
	int ny[8] = {1,0,-1,1,-1,1,0,-1};

	while(oVec.size() > 0){
		/////////////////// this finds node with lowest fScore and makes current
		float min = INFINITY;
		int mindex = -1;

		for(size_t i=0; i<oVec.size(); i++){
			if(fScore.at<float>(oVec[i]) < min){
				min = fScore.at<float>(oVec[i]);
				mindex = i;
			}
		}

		if(mindex < 0){
			//cerr << "Costmap::aStarDist::out dirty" << endl;
			return INFINITY;
		}

		Point cLoc = oVec[mindex];
		oVec.erase(oVec.begin() + mindex);
		oSet.at<short>(cLoc) = 0;
		cSet.at<short>(cLoc) = 1;

		/////////////////////// end finding current node
		if(pointCompare(cLoc, gLoc) ){ // if the current node equals goal, construct path
			//cerr << "Costmap::aStarDist::out clean" << endl;
			return gScore.at<float>(cLoc);
		} ///////////////////////////////// end construct path

		for(int ni = 0; ni<9; ni++){
			Point nbr(cLoc.x + nx[ni], cLoc.y + ny[ni]);
			if(pointOnMat(nbr, costmap.cells) ){
				if(cSet.at<short>(nbr) == 1){ // has it already been eval? in cSet
					continue;
				}
				float ngScore = gScore.at<float>(cLoc) + sqrt(pow(cLoc.x-nbr.x,2) + pow(cLoc.y-nbr.y,2));//getEuclidianDistance(cLoc, nbr); // calc temporary gscore, estimate of total cost
				if(oSet.at<short>(nbr) == 0){
					oSet.at<short>(nbr) = 1;  // add nbr to open set
					oVec.push_back(nbr);
				}
				else if(ngScore >= gScore.at<float>(nbr) ){ // is temp gscore worse than stored g score of nbr
					continue;
				}

				gScore.at<float>(nbr) = ngScore;
				if(costmap.cells.at<short>(nbr) < costmap.obsWall){
					fScore.at<float>(nbr) = gScore.at<float>(nbr) + sqrt(pow(nbr.x-gLoc.x,2) + pow(nbr.y-gLoc.y,2));// +getEuclidianDistance(gLoc,nbr)
				}
				else{
					fScore.at<float>(nbr)= INFINITY;
				}
			}
		}
	}
	//cerr << "Costmap::aStarDist::out dirty" << endl;
	return INFINITY;
}
