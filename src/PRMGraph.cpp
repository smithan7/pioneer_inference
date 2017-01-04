/*
 * PRMGraph.cpp
 *
 *  Created on: Sep 16, 2016
 *      Author: andy
 */

#include "PRMGraph.h"

using namespace cv;
using namespace std;

PRMGraph::PRMGraph(){}

PRMGraph::~PRMGraph(){}

void PRMGraph::updatePRMGraph(Costmap &costmap, int nodeSpacing, int nbrSpacing, int nPulls){

	this->nodeSpacing = nodeSpacing;
	this->nbrSpacing = nbrSpacing;
	// 1 = observed free
	// 2 = inferred free
	// 3 = dominated free
	// 101 = observed obstacle
	// 102 = inferred obstacle
	// 201 unknown

	float i=0;
	while(i<nPulls){
		i++;

		Point p;
		if( rand() % 100 > 50){
			// select point from costmap
			while(true){
				Point r(rand() % costmap.cells.cols, rand() % costmap.cells.rows);
				if(costmap.cells.at<uchar>(r) <= costmap.domFree){
					p = r;
					break;
				}
			}
		}
		else{
			// select point from updates
			int r = rand() % costmap.cellUpdates.size();
			p = costmap.cellUpdates[r];
		}

		bool flag = true;
		vector<float> minDist(2,nbrSpacing);
		vector<int> mindex(2,-1);
		for( size_t iprime=0; i<nodeLocations.size(); iprime++){
			int index = iprime;
			float d = costmap.getEuclidianDistance( p, nodeLocations[index]);
			if( d < nodeSpacing){
				flag = false;
				break;
			}
			else if(d < minDist.back()){
				for(size_t j=0; j<minDist.size(); j++){
					if(d < minDist[j]){
						float td = minDist[0];
						int ti = mindex[0];

						minDist[0] = d;
						mindex[0] = index;

						d = td;
						index = ti;
					}
				}
			}
		}

		if(flag){
			nodeLocations.push_back(p);
			/*
			vector<int> nbrs;
			vector<float> trs;
			for(size_t j=0; j<minDist.size(); j++){
				if(mindex[j] != -1){
					trs.push_back( minDist[j] );
					nbrs.push_back( mindex[j] );

					nodeTransitions[mindex[j]].push_back(minDist[j]);
					nodeNbrs[mindex[j]].push_back(nodeLocations.size());
				}
			}
			nodeTransitions.push_back(trs);
			nodeNbrs.push_back(nbrs);
			*/
		}
		else{
			i-=0.5;
		}

	}
}

void PRMGraph::eraseNode(int index){

	nodeLocations.erase(nodeLocations.begin() + index);
	nodeUpdates.erase(nodeUpdates.begin() + index);
	nodeSubGraphIndices.erase(nodeSubGraphIndices.begin() + index);

	for(size_t i=0; i<nodeNbrs[index].size(); i++){ // go through index's nbrs and erase
		int nbr = nodeNbrs[index][i];

		for(size_t j =0; j<nodeNbrs[nbr].size(); j++){
			if(nodeNbrs[nbr][j] == index){
				nodeNbrs[nbr].erase(nodeNbrs[nbr].begin() + j);
				nodeTransitions[nbr].erase(nodeTransitions[nbr].begin() + j);
				break;
			}
		}
	}
	nodeTransitions[index].erase(nodeTransitions[index].begin() + index);
	nodeNbrs[index].erase(nodeNbrs[index].begin() + index);




}

float PRMGraph::aStarDist(int sIndex, int gIndex, Costmap &costmap){
	vector<bool> cSet(nodeLocations.size(), false); // 1 means in closed set, 0 means not
	vector<bool> oSet(nodeLocations.size(), false); // 1 means in open set, 0 means not
	vector<float> gScore(nodeLocations.size(), INFINITY); // known cost from initial state to n
	vector<float> fScore(nodeLocations.size(), INFINITY); // gScore + heuristic score (dist to goal + imposed cost)

	oSet[sIndex] = true; // starting state in open set
	gScore[sIndex] = 0; // starting state has score 0
	fScore[sIndex] = gScore[sIndex] + costmap.getEuclidianDistance(nodeLocations[sIndex],nodeLocations[gIndex]); // calc score of open set

	//cerr << "PRMGraph::aStarDist::sNode.loc: " << nodes[sIndex].loc[0] << ", " << nodes[sIndex].loc[1] << endl;
	//cerr << "PRMGraph::aStarDist::gNode.loc: " << nodes[gIndex].loc[0] << ", " << nodes[gIndex].loc[1] << endl;

	int foo = 1;
	int finishFlag = 0;
	while(foo>0 && finishFlag == 0){
		/////////////////// this finds state with lowest fScore and makes current
		float minScore = INFINITY;
		int current = -1;

		for(size_t i=0;i<nodeLocations.size();i++){
			//cerr << "PRMGraph::aStarDist::fScore: " << fScore[i] << endl;
			if(fScore[i] < minScore && oSet[i]){
				minScore = fScore[i];
				current = i;
			}
		}

		if(current == -1){
			cerr << "PRMGraph::A* Dist::No solution found, empty oSet" << endl;
			return INFINITY;
		}
		else{
			float cfScore = fScore[current];
			float cgScore = gScore[current];

			//cerr << "PRMGraph::aStarDist::current.loc: " << nodes[current].loc[0] << ", " << nodes[current].loc[1] << endl;
			oSet[current] = false;
			cSet[current] = true;
			/////////////////////// end finding current state
			if(current == gIndex){ // if the current state equals goal, then return the distance to the goal
				return cfScore;
			} ///////////////////////////////// end construct path
			for(size_t nbr=0; nbr<nodeLocations.size(); nbr++){

				if(nodeTransitions[current][nbr] != INFINITY && cSet[nbr] == false){

					float tgScore = cgScore + costmap.getEuclidianDistance(nodeLocations[current], nodeLocations[nbr]); // calc temporary gscore
					if(oSet[nbr]){
						if(cgScore < tgScore){
							gScore[nbr] = tgScore;
							fScore[nbr] = gScore[nbr] + costmap.getEuclidianDistance(nodeLocations[nbr], nodeLocations[gIndex]);
						}
					}
					else{
						oSet[nbr] = true;
						gScore[nbr] = tgScore;
						fScore[nbr] = cgScore + costmap.getEuclidianDistance(nodeLocations[nbr], nodeLocations[gIndex]);
					}
				}
			}
			/////////////// end condition for while loop, check if oSet is empty
			foo = oSet.size();
		}
	}
	cerr << "PRMGraph::A* Dist::No solution found, reached end of function" << endl;
	return INFINITY;
}

void PRMGraph::displayCoordMap(Costmap &costmap, bool displayNodes){
	Mat coordGraph = Mat::zeros(costmap.cells.size(), CV_8UC3);
	Scalar white;
	white[0] = 255; white[1] = 255; white[2] = 255;
	Scalar gray;
	gray[0] = 127; gray[1] = 127; gray[2] = 127;
	Scalar red;
	red[0] = 0; red[1] = 0; red[2] = 255;
	Scalar green;
	green[0] = 0; green[1] = 255; green[2] = 0;
	Scalar blue;
	blue[0] = 255; blue[1] = 0; blue[2] = 0;


	if(displayNodes){
		for(size_t i=0; i<nodeLocations.size(); i++){
			circle(coordGraph,Point(nodeLocations[i].y, nodeLocations[i].x),1,white,-1,8);
			//char str[50];
			//sprintf(str,"%d",i);
			//putText(coordGraph, str, temp, FONT_HERSHEY_PLAIN,0.5,green);
		}
	}


	if(nodeTransitions.size() > 0){
		for(size_t i=0; i<nodeNbrs.size(); i++){
			for(size_t j=0; j<nodeNbrs[i].size(); j++){
				line(coordGraph, nodeLocations[i], nodeLocations[nodeNbrs[i][j]], white, 1,8);
			}
		}
	}

	/*
	if(cNode >= 0){
		Point cloc;
		cloc.x = nodeLocations[cNode][1];
		cloc.y = nodeLocations[cNode][0];
		circle(coordGraph,cloc,2,blue,-1,8);
	}
	*/

	namedWindow("PRMGraph::coordGraph", WINDOW_NORMAL);
	imshow("PRMGraph::coordGraph", coordGraph);
}

bool PRMGraph::visibleLineCheck(Costmap &costmap, Point pose, Point pt){

	Mat t = Mat::zeros(costmap.cells.size(), CV_8UC1);

	LineIterator it(t, pose, pt, 4, false);

	for(int i=0; i<it.count; i++, ++it){

		Point pp  = it.pos();
		if(costmap.cells.at<uchar>(pp) > costmap.domFree){
			return false;
		}
	}

	return true;
}

int PRMGraph::findNearestNode(Point in, Costmap &costmap){
	float minDist = 5;
	int minIndex = -1;
	for(size_t i=0; i<nodeLocations.size(); i++){
		float a = costmap.getEuclidianDistance(in, nodeLocations[i]);
		if(a < minDist){
			 if( visibleLineCheck( costmap, in, nodeLocations[i]) ){
					minDist = a;
					minIndex = i;
			 }
		}
	}
	return minIndex;
}
