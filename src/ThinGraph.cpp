/*
 * ThinGraph.cpp
 *
 *  Created on: Sep 16, 2016
 *      Author: andy
 */

#include "ThinGraph.h"


void clearArea(Mat &thinMat, Point loc, int grafSpacing);

using namespace cv;
using namespace std;

ThinGraph::ThinGraph(){}

ThinGraph::~ThinGraph(){}

void ThinGraph::updateThinGraph(Costmap &costmap, int nodeSpacing, int nbrSpacing){

	this->nodeSpacing = nodeSpacing;
	this->nbrSpacing = nbrSpacing;
	// 1 = observed free
	// 2 = inferred free
	// 3 = dominated free
	// 101 = observed obstacle
	// 102 = inferred obstacle
	// 201 unknown

	// get Mat of all observed or inferred free cells to make travel graph
	Mat obsMat = Mat::zeros(costmap.cells.size(), CV_8UC1);
	Mat thinMat = Mat::zeros(costmap.cells.size(), CV_8UC1);
	Mat freeMat = Mat::zeros(costmap.cells.size(), CV_8UC1);

	// get all currently observed cells
	int limits[4] = {costmap.cells.cols, 0, costmap.cells.rows, 0};
	for(size_t i = 0; i<costmap.cellUpdates.size(); i++){
		obsMat.at<uchar>(costmap.cellUpdates[i]) = 255;
	}

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			if(costmap.cells.at<uchar>(i,j) <= costmap.domFree){
				freeMat.at<uchar>(i,j) = 255;
			}
		}
	}

	// get the thin free mat
	thinning(freeMat, thinMat, limits);

	// add nodes
	vector<Point> tLocs = findStatesCityBlockDistance(thinMat, limits); // add them to graf;
	mergeTempNodesIntoGraph( tLocs, costmap, obsMat, thinMat ); // TODO obsMat is to erase old nodes, find those observed that aren't on thinMat anymore



	findStateTransitionsCityBlockDistance(costmap); // find connections in graf; // TODO add line checking to this for intersections and increase distance
	//findStatesByGrid(freeMat);
	//findStateTransitionsByVisibility(costmap);
	//findCNodeTransitions(costmap);

}


void ThinGraph::mergeTempNodesIntoGraph( vector<Point> tLocs, Costmap &costmap, Mat &obsMat, Mat &thinMat ){
	for(size_t i=0; i<tLocs.size(); i++){

		int mindex = -1;
		float mindist = 2*nodeSpacing;

		for(size_t j=0; j<nodeLocations.size(); j++){

			float d = costmap.getEuclidianDistance(tLocs[i], nodeLocations[j]);
			if( d <= mindist ){
				mindist = d;
				mindex = j;
			}
		}


		if( mindex != -1){
			nodeLocations[mindex].x = 0.8*nodeLocations[mindex].x + 0.2*tLocs[i].x;
			nodeLocations[mindex].y = 0.8*nodeLocations[mindex].y + 0.2*tLocs[i].y;
			nodeUpdates[mindex] = 0;
		}
		else{
			nodeLocations.push_back(tLocs[i]);
			nodeUpdates.push_back(0);
		}
	}

	for(size_t i=0; i<nodeLocations.size(); i++){
		int x = nodeLocations[i].x;
		int y = nodeLocations[i].y;

		if(obsMat.at<uchar>(x,y) == 255 && thinMat.at<uchar>(x,y) != 255){
			if(nodeUpdates[i] > 5){
				//eraseNode(i);
			}
			else{
				nodeUpdates[i]++;
			}
		}
	}
}

void ThinGraph::eraseNode(int index){

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

vector<Point> ThinGraph::findStatesCityBlockDistance(Mat &thinMat, int limits[4]){

	vector<Point> tLocs;
	for(int i=limits[0]; i<limits[1]; i++){ // find new nodes
		for(int j=limits[2]; j<limits[3]; j++){
			if(thinMat.at<uchar>(i,j) == 255){
				Point loc;
				loc.x = j;
				loc.y = i;
				tLocs.push_back(loc);
				clearArea(thinMat, loc, nodeSpacing);
			}
		}
	}
	return tLocs;
}

float ThinGraph::aStarDist(int sIndex, int gIndex, Costmap &costmap){
	vector<bool> cSet(nodeLocations.size(), false); // 1 means in closed set, 0 means not
	vector<bool> oSet(nodeLocations.size(), false); // 1 means in open set, 0 means not
	vector<float> gScore(nodeLocations.size(), INFINITY); // known cost from initial state to n
	vector<float> fScore(nodeLocations.size(), INFINITY); // gScore + heuristic score (dist to goal + imposed cost)

	oSet[sIndex] = true; // starting state in open set
	gScore[sIndex] = 0; // starting state has score 0
	fScore[sIndex] = gScore[sIndex] + costmap.getEuclidianDistance(nodeLocations[sIndex],nodeLocations[gIndex]); // calc score of open set

	//cerr << "ThinGraph::aStarDist::sNode.loc: " << nodes[sIndex].loc[0] << ", " << nodes[sIndex].loc[1] << endl;
	//cerr << "ThinGraph::aStarDist::gNode.loc: " << nodes[gIndex].loc[0] << ", " << nodes[gIndex].loc[1] << endl;

	int foo = 1;
	int finishFlag = 0;
	while(foo>0 && finishFlag == 0){
		/////////////////// this finds state with lowest fScore and makes current
		float minScore = INFINITY;
		int current = -1;

		for(size_t i=0;i<nodeLocations.size();i++){
			//cerr << "ThinGraph::aStarDist::fScore: " << fScore[i] << endl;
			if(fScore[i] < minScore && oSet[i]){
				minScore = fScore[i];
				current = i;
			}
		}

		if(current == -1){
			cerr << "ThinGraph::A* Dist::No solution found, empty oSet" << endl;
			return INFINITY;
		}
		else{
			float cfScore = fScore[current];
			float cgScore = gScore[current];

			//cerr << "ThinGraph::aStarDist::current.loc: " << nodes[current].loc[0] << ", " << nodes[current].loc[1] << endl;
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
	cerr << "ThinGraph::A* Dist::No solution found, reached end of function" << endl;
	return INFINITY;
}

void ThinGraph::displayCoordMap(Costmap &costmap, bool displayNodes){
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
			circle(coordGraph,nodeLocations[i],1,white,-1,8);
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

	namedWindow("ThinGraph::coordGraph", WINDOW_NORMAL);
	imshow("ThinGraph::coordGraph", coordGraph);
}

bool ThinGraph::visibleLineCheck(Costmap &costmap, Point pose, Point pt){

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

void ThinGraph::findStatesByGrid(Mat &inMat){
	for(int i=0; i<inMat.rows; i += nodeSpacing){
		for(int j=0; j<inMat.cols; j += nodeSpacing){
			if(inMat.at<uchar>(i,j) == 255){
				Point loc(i,j);
				nodeLocations.push_back(loc);
			}
		}
	}
}



void ThinGraph::findStateTransitionsCityBlockDistance(Costmap &costmap){

	nodeTransitions.clear();

	for(size_t i=0; i<nodeLocations.size(); i++){
		for(size_t j=i+1; j<nodeLocations.size(); j++){
			float d = costmap.getEuclidianDistance(nodeLocations[i],nodeLocations[j]);
			if(d < nbrSpacing && i != j){
				nodeTransitions[i].push_back(d);
				nodeTransitions[j].push_back(d);

				nodeNbrs[i].push_back(j);
				nodeNbrs[j].push_back(i);
			}
		}
	}
}

void ThinGraph::findStateTransitionsByVisibility(Costmap &costmap){
	// make perimeter of viewing circle fit on image

	nodeTransitions.clear();

	for(size_t i=0; i<nodeLocations.size(); i++){
		for(size_t j=i+1; j<nodeLocations.size(); j++){
			float d = costmap.getEuclidianDistance(nodeLocations[i],nodeLocations[j]);
			if(d < nbrSpacing && i != j){
				if(visibleLineCheck(costmap, nodeLocations[i],nodeLocations[j])){
					nodeTransitions[i].push_back(d);
					nodeTransitions[j].push_back(d);

					nodeNbrs[i].push_back(j);
					nodeNbrs[j].push_back(i);
				}
			}
		}
	}
}

int ThinGraph::findNearestNode(Point in, Costmap &costmap){
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

void ThinGraph::thinning(const Mat& src, Mat& dst, int limits[4]){
 	//https://github.com/bsdnoobz/zhang-suen-thinning/blob/master/thinning.cpp
     dst = src.clone();
     dst /= 255;         // convert to binary image

     cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
     cv::Mat diff;

     do {
         thinningIteration(dst, 0, limits);
         thinningIteration(dst, 1, limits);
         cv::absdiff(dst, prev, diff);
         dst.copyTo(prev);
     }
     while (cv::countNonZero(diff) > 0);

     dst *= 255;
}

void ThinGraph::thinningIteration(Mat& img, int iter, int limits[4]){
 	//https://github.com/bsdnoobz/zhang-suen-thinning/blob/master/thinning.cpp
     CV_Assert(img.channels() == 1);
     CV_Assert(img.depth() != sizeof(uchar));
     CV_Assert(img.rows > 3 && img.cols > 3);

     cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

     int x, y;
     uchar *pAbove;
     uchar *pCurr;
     uchar *pBelow;
     uchar *nw, *no, *ne;    // north (pAbove)
     uchar *we, *me, *ea;
     uchar *sw, *so, *se;    // south (pBelow)

     uchar *pDst;

     // initialize row pointers
     pAbove = NULL;
     pCurr  = img.ptr<uchar>(0);
     pBelow = img.ptr<uchar>(1);

     for (y = limits[0]-2; y < limits[1]+2; ++y) {
         // shift the rows up by one
         pAbove = pCurr;
         pCurr  = pBelow;
         pBelow = img.ptr<uchar>(y+1);

         pDst = marker.ptr<uchar>(y);

         // initialize col pointers
         no = &(pAbove[0]);
         ne = &(pAbove[1]);
         me = &(pCurr[0]);
         ea = &(pCurr[1]);
         so = &(pBelow[0]);
         se = &(pBelow[1]);

         for (x = limits[2]-2; x < limits[3]+2; ++x) {
             // shift col pointers left by one (scan left to right)
             nw = no;
             no = ne;
             ne = &(pAbove[x+1]);
             we = me;
             me = ea;
             ea = &(pCurr[x+1]);
             sw = so;
             so = se;
             se = &(pBelow[x+1]);

             int A  = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) +
                      (*ea == 0 && *se == 1) + (*se == 0 && *so == 1) +
                      (*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
                      (*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
             int B  = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
             int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
             int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

             if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0){
                 pDst[x] = 1;
             }
         }
     }

     img &= ~marker;
}
