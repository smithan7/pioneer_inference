/*
 * Graph.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */


#include "Graph.h"

int getMinIndex(vector<float> value);
int getMaxIndex(vector<float> value);

Point findIntersection(Vec4i w1, Vec4i w2);
float distToLine(Vec4i w, Point a);
Point extendLine(Point a, Point m);
float distToLineSegment(Point p, Point v, Point w);
void clearArea(Mat &thinMat, Point loc, int grafSpacing);


using namespace cv;
using namespace std;

Graph::Graph(){
	nodeSpacing = 5;
	nbrSpacing = nodeSpacing;
}

void Graph::createPRMGraph(Point cLoc, Costmap &costmap, int nodeSpacing, int nbrSpacing){
	this->nodeSpacing = nodeSpacing;
	this->nbrSpacing = nbrSpacing;
	// 1 = observed free
	// 2 = inferred free
	// 101 = observed obstacle
	// 102 = inferred obstacle
	// 201 unknown

	// get Mat of all observed or inferred free cells to make travel graph
	Mat freeMat = Mat::zeros(costmap.cells.size() ,CV_8UC1);
	for(int i = 0; i<costmap.cells.cols; i++){
		for(int j =0; j<costmap.cells.rows; j++){
			if(costmap.cells.at<uchar>(i,j) <= costmap.domFree){ // observed free space or inferred free space - to make travel graph
				freeMat.at<uchar>(i,j,0) = 255;
			}
		}
	}

	threshold(freeMat,freeMat,10,255,CV_THRESH_BINARY);

	// get the thin free mat
	//Mat thinMat = Mat::zeros(freeMat.size(), CV_8UC1); // TODO bitwise or with prev thinMat and currently observed cells
	//thinning(freeMat, thinMat);

	// add nodes
	nodeLocations.clear();
	nodeLocations.push_back(cLoc);
	cNode = 0;

	//findStatesCityBlockDistance(freeMat); // add them to graf;
	//
	findStatesByGrid(freeMat);
	//findStateTransitionsByVisibility(costmap);
	findStateTransitionsCityBlockDistance(costmap); // find connections in graf; // TODO add line checking to this for intersections and increase distance
	//findCNodeTransitions(costmap);
	//mergeStatesBySharedNbr(); // TODO make this work, currently runs forever
}

void Graph::findStatesByGrid(Mat &inMat){
	for(int i=0; i<inMat.rows; i += nodeSpacing){
		for(int j=0; j<inMat.cols; j += nodeSpacing){
			if(inMat.at<uchar>(i,j) == 255){
				Point loc(i,j);
				nodeLocations.push_back(loc);
			}
		}
	}
}

void Graph::clearTransitions( int i ){
	for(size_t j = 0; j<nodeTransitions.size(); j++){
		nodeTransitions[j].erase( nodeTransitions[j].begin() + i);
	}
	nodeTransitions.erase( nodeTransitions.begin() + i);
}

void Graph::findStatesCityBlockDistance(Mat &thinMat){
	size_t i=0; // don't include cNode
	while(i<nodeLocations.size()){
		if(thinMat.at<uchar>(nodeLocations[i]) != 255){
			nodeLocations.erase(nodeLocations.begin() + i); // erase old nodes that shouldn't be nodes anymore
			clearTransitions(i);
		}
		else{
			clearArea(thinMat, nodeLocations[i], nodeSpacing); // clear area surrounding nodes that should be
		}
		i++;
	}

	/*
	namedWindow("Graph::thinGraph", WINDOW_NORMAL);
	imshow("Graph::thinGraph", thinMat);
	waitKey(0);
	*/

	for(int i=0; i<thinMat.cols; i++){ // find new nodes
		for(int j=0; j<thinMat.rows; j++){
			Point a(i,j);
			if(thinMat.at<uchar>(a) == 255){
				nodeLocations.push_back(a);
				clearArea(thinMat, a, nodeSpacing);
			}
		}
	}

}

void Graph::findStateTransitionsCityBlockDistance(Costmap &costmap){

	nodeTransitions.clear();
	for(size_t i=0; i<nodeLocations.size(); i++){
		vector<float> t;
		for(size_t j=0; j<nodeLocations.size(); j++){
			t.push_back(INFINITY);
		}
		nodeTransitions.push_back(t);
	}

	for(size_t i=0; i<nodeLocations.size(); i++){
		for(size_t j=i+1; j<nodeLocations.size(); j++){
			//cerr << i << "-i.loc: " << nodes[i].loc[0] << " , " << nodes[i].loc[1] << endl;
			//cerr << j << "-j.loc: " << nodes[j].loc[0] << " , " << nodes[j].loc[1] << endl;
			float d = costmap.getEuclidianDistance(nodeLocations[i],nodeLocations[j]);
			if(d < nbrSpacing && i != j){
				nodeTransitions[i][j] = d;
				nodeTransitions[j][i] = d;
			}
		}
	}
}

void Graph::findStateTransitionsByVisibility(Costmap &costmap){
	// make perimeter of viewing circle fit on image

	nodeTransitions.clear();
	for(size_t i=0; i<nodeLocations.size(); i++){
		vector<float> t;
		for(size_t j=0; j<nodeLocations.size(); j++){
			t.push_back(INFINITY);
		}
		nodeTransitions.push_back(t);
	}

	for(size_t i=0; i<nodeLocations.size(); i++){
		for(size_t j=i+1; j<nodeLocations.size(); j++){
			float d = costmap.getEuclidianDistance(nodeLocations[i],nodeLocations[j]);
			if(d < nbrSpacing && i != j){
				if(visibleLineCheck(costmap, nodeLocations[i],nodeLocations[j])){
					nodeTransitions[i][j] = d;
					nodeTransitions[j][i] = d;
				}
			}
		}
	}
}

void Graph::findCNodeTransitions(Costmap &costmap){

	vector<int> index;
	vector<float> dist;

	for(size_t i=0; i<nodeLocations.size(); i++){
		if( visibleLineCheck(costmap, nodeLocations[cNode], nodeLocations[i]) ){
			float d = costmap.getEuclidianDistance(nodeLocations[i],nodeLocations[cNode]);
			if(d < 2*this->nbrSpacing){
				if(visibleLineCheck(costmap, nodeLocations[i], nodeLocations[cNode])){
					nodeTransitions[i][cNode] = d;
					nodeTransitions[cNode][i] = d;
				}
			}
		}
	}

}

bool Graph::visibleLineCheck(Costmap &costmap, Point pose, Point pt){

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

void Graph::displayCoordMap(Costmap &costmap, bool displayNodes){
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
			circle(coordGraph,nodeLocations[i],3,white,-1,8);
			//char str[50];
			//sprintf(str,"%d",i);
			//putText(coordGraph, str, temp, FONT_HERSHEY_PLAIN,0.5,green);
		}
	}


	if(nodeTransitions.size() == nodeLocations.size() && nodeTransitions[0].size() == nodeLocations.size()){
		for(size_t i=0; i<nodeLocations.size()-1; i++){
			for(size_t j=i+1; j<nodeLocations.size(); j++){
				cerr << "b" << endl;
				if(nodeTransitions[i][j] < INFINITY){
					cerr << "a" << endl;
					line(coordGraph, nodeLocations[i], nodeLocations[j], white, 1,8);
				}
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

	namedWindow("Graph::coordGraph", WINDOW_NORMAL);
	imshow("Graph::coordGraph", coordGraph);
}

int Graph::findNearestNode(Point in, Costmap &costmap){
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

float Graph::aStarDist(int sIndex, int gIndex, Costmap &costmap){
	vector<bool> cSet(nodeLocations.size(), false); // 1 means in closed set, 0 means not
	vector<bool> oSet(nodeLocations.size(), false); // 1 means in open set, 0 means not
	vector<float> gScore(nodeLocations.size(), INFINITY); // known cost from initial state to n
	vector<float> fScore(nodeLocations.size(), INFINITY); // gScore + heuristic score (dist to goal + imposed cost)

	oSet[sIndex] = true; // starting state in open set
	gScore[sIndex] = 0; // starting state has score 0
	fScore[sIndex] = gScore[sIndex] + costmap.getEuclidianDistance(nodeLocations[sIndex],nodeLocations[gIndex]); // calc score of open set

	//cerr << "Graph::aStarDist::sNode.loc: " << nodes[sIndex].loc[0] << ", " << nodes[sIndex].loc[1] << endl;
	//cerr << "Graph::aStarDist::gNode.loc: " << nodes[gIndex].loc[0] << ", " << nodes[gIndex].loc[1] << endl;

	int foo = 1;
	int finishFlag = 0;
	while(foo>0 && finishFlag == 0){
		/////////////////// this finds state with lowest fScore and makes current
		float minScore = INFINITY;
		int current = -1;

		for(size_t i=0;i<nodeLocations.size();i++){
			//cerr << "Graph::aStarDist::fScore: " << fScore[i] << endl;
			if(fScore[i] < minScore && oSet[i]){
				minScore = fScore[i];
				current = i;
			}
		}

		if(current == -1){
			cerr << "Graph::A* Dist::No solution found, empty oSet" << endl;
			return INFINITY;
		}
		else{
			float cfScore = fScore[current];
			float cgScore = gScore[current];

			//cerr << "Graph::aStarDist::current.loc: " << nodes[current].loc[0] << ", " << nodes[current].loc[1] << endl;
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
	cerr << "Graph::A* Dist::No solution found, reached end of function" << endl;
	return INFINITY;
}

void Graph::mergeStatesBySharedNbr(){
	/*
	bool flag = true;
	while(flag){
		flag = false;
		for(size_t i=0; i<nodes.size(); i++){ // compare every node
			for(size_t j=0; j<nodes[i].nbrs.size(); j++){ // against all of its nbrs
				Node nbr = nodes[i].nbrs[j]; // pick out nbr

				for(size_t k=0; k<nodes[i].nbrs.size(); k++){ // to compare nbrs
					for(size_t l=0; l<nbr.nbrs.size(); l++){ // to compare nbrs
						if(nodes[i].nbrs[k].loc == nbr.nbrs[l].loc){
							flag = true;
							vector<int> p;
							int x = (nodes[i].nbrs[k].loc[0] + nbr.nbrs[l].loc[0])/2;
							int y = (nodes[i].nbrs[k].loc[1] + nbr.nbrs[l].loc[1])/2;
							p.push_back(x);
							p.push_back(y);
							Node a;
							a.loc = p;
							for(size_t m=0; m<nodes[i].nbrs.size(); m++){ // load nbrs to merged state
								if(nodes[i].nbrs[m].loc != nodes[i].nbrs[j].loc){ // dont load merged nbr
									a.nbrs.push_back(nodes[i].nbrs[m]);
								}
							}
							for(size_t m=0; m<nbr.nbrs.size(); m++){ // load nbrs to merged state
								if(nbr.nbrs[m].loc != nodes[i].loc){ // dont load merged nbr
									bool flag2 = true;
									for(size_t n=0; n<a.nbrs.size(); n++){ // not already in a.nbrs
										if(nbr.nbrs[m].loc == a.nbrs[n].loc){
											flag2 = false;
											break;
										}
									}
									if(flag2){
										a.nbrs.push_back(nbr.nbrs[m]); // not merged or already in a.nbrs, so add to a.nbrs
									}
								}
							}
							nodes.push_back(a);
						}
					}
				}

			}
		}
	}
	*/
}

void Graph::createThinGraph(Costmap &costmap, int nodeSpacing, int nbrSpacing){

	this->nodeSpacing = nodeSpacing;
	this->nbrSpacing = nbrSpacing;
	// 1 = observed free
	// 2 = inferred free
	// 3 = dominated free
	// 101 = observed obstacle
	// 102 = inferred obstacle
	// 201 unknown

	// get Mat of all observed or inferred free cells to make travel graph
	Mat freeMat = Mat::zeros(costmap.cells.size(), CV_8UC1);
	for(int i = 0; i<costmap.cells.cols; i++){
		for(int j =0; j<costmap.cells.rows; j++){
			Point a(i,j);
			if(costmap.cells.at<short>(a) <= costmap.domFree){ // observed free space or inferred free space - to make travel graph
				freeMat.at<uchar>(a) = 255;
			}
		}
	}

	if(true){
		Mat newFree = Mat::zeros(freeMat.rows/2, freeMat.cols/2, CV_8UC1);
		downSample(freeMat, newFree, costmap);

		threshold(newFree, newFree,10,255,CV_THRESH_BINARY);

		// get the thin free mat
		Mat tMat = Mat::zeros( newFree.size(), CV_8UC1);
		thinning(newFree, tMat);
		resize(tMat, thinMat, thinMat.size(), 2, 2, INTER_AREA);
	}
	else{


		threshold(freeMat,freeMat,10,255,CV_THRESH_BINARY);

		// get the thin free mat
		Mat thinMat = Mat::zeros(freeMat.size(), CV_8UC1); // TODO bitwise or with prev thinMat and currently observed cells
		thinning(freeMat, thinMat);
	}
	/*
	namedWindow("Graph::createThinGraph::thinMat", WINDOW_NORMAL);
	imshow("Graph::createThinGraph::thinMat", thinMat);
	waitKey(0);
	*/
	// add nodes
	nodeLocations.clear();
	findStatesCityBlockDistance(thinMat); // add them to graf;
	//findStateTransitionsCityBlockDistance(costmap); // find connections in graf; // TODO add line checking to this for intersections and increase distance
	//findStatesByGrid(freeMat);
	//findStateTransitionsByVisibility(costmap);
	//findCNodeTransitions(costmap);
	//mergeStatesBySharedNbr(); // TODO make this work, currently runs forever

}

void Graph::downSample( Mat &oMat, Mat &nMat, Costmap &costmap){

	int dx[4] = {0,1,0,1};
	int dy[4] = {0,0,1,1};

	for(int i=0; i<costmap.cells.cols/2-2; i++){
		for(int j=0; j<costmap.cells.rows/2-2; j++){
			Point n(i,j);
			bool flag = true;
			for(int k=0; k<4; k++){
				Point p(2*i+dx[k], 2*j+dy[k]);

				if(costmap.cells.at<short>(p) >= costmap.unknown){
					flag = false;
				}
			}
			if(flag){
				nMat.at<uchar>(n) = 255;
			}
			else{
				nMat.at<uchar>(n) = 0;
			}
		}
	}
}

void Graph::thinning(const Mat& src, Mat& dst){
 	//https://github.com/bsdnoobz/zhang-suen-thinning/blob/master/thinning.cpp
     dst = src.clone();
     dst /= 255;         // convert to binary image

     cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
     cv::Mat diff;

     do {
         thinningIteration(dst, 0);
         thinningIteration(dst, 1);
         cv::absdiff(dst, prev, diff);
         dst.copyTo(prev);
     }
     while (cv::countNonZero(diff) > 0);

     dst *= 255;
}

void Graph::thinningIteration(Mat& img, int iter){
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

     for (y = 1; y < img.rows-1; ++y) {
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

         for (x = 1; x < img.cols-1; ++x) {
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

void clearArea(cv::Mat &thinMat, Point loc, int grafSpacing){
	circle(thinMat, loc, grafSpacing, Scalar(0), -1, 8);
}

float cityBlockDist(vector<int> a, vector<int> b){
	float d = abs(a[0] - b[0]) + abs(a[1]+b[1]);
	return d;
}

float euclidianDist(vector<int> a, vector<int> b){
	float d = sqrt(pow(a[0]-b[0],2) + pow(a[1] - b[1],2));
	return d;
}

int getMaxIndex(vector<float> value){
	int maxdex = 0;
	float maxValue = -INFINITY;
	for(int i=0; i<(int)value.size(); i++){
		if(value[i] > maxValue){
			maxdex = i;
			maxValue  = value[i];
		}
	}
	return maxdex;
}

int getMinIndex(vector<float> value){
	int mindex = 0;
	float minValue = INFINITY;
	for(int i=0; i<(int)value.size(); i++){
		if(value[i] < minValue){
			mindex = i;
			minValue  = value[i];
		}
	}
	return mindex;
}

float distToLineSegment(Point p, Point v, Point w){
	float l = pow(v.x - w.x,2) + pow(v.y-w.y,2);
	if(l==0){ return sqrt(pow(v.x - p.x,2) + pow(v.y-p.y,2) ); } // v==w
	float t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l;
	if(t > 1){
		t = 1;
	}
	else if(t < 0){
		t = 0;
	}
	int xl = v.x + t * (w.x - v.x);
	int yl = v.y + t * (w.y - v.y);
	return sqrt( pow(p.x - xl,2) + pow(p.y-yl,2) );
}

Point extendLine(Point a, Point m){
	float dx = m.x - a.x;
	float dy = m.y - a.y;
	Point p;
	p.x = m.x + dx;
	p.y = m.y + dy;
	return(p);
}

float distToLine(Vec4i w, Point a){
	float x1 = w[0];
	float y1 = w[1];
	float x2 = w[2];
	float y2 = w[3];

	float x0 = a.x;
	float y0 = a.y;

	float denom = sqrt(pow(x2-x1,2) + pow(y2-y1,2));
	if(denom != 0){
		float dist = abs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1))/denom;
		return dist;
	}
	else{
		return(-1);
	}
}

Point findIntersection(Vec4i w1, Vec4i w2){
	float x1 = w1[0];
	float y1 = w1[1];
	float x2 = w1[2];
	float y2 = w1[3];

	float x3 = w2[0];
	float y3 = w2[1];
	float x4 = w2[2];
	float y4 = w2[3];

	float denom = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);

	if(denom != 0){
		Point p;
	    p.x = ((x1*y2-y1*x2)*(x3-x4) - (x3*y4-y3*x4)*(x1-x2))/denom;
	    p.y = ((x1*y2-y1*x2)*(y3-y4) - (x3*y4-y3*x4)*(y1-y2))/denom;
	    return(p);
	}
	else{
		Point p;
		p.x = -1;
		p.y = -1;
		return(p);
	}
}

Graph::~Graph(){}

void Graph::updateThinGraph(Costmap &costmap, int nodeSpacing, int nbrSpacing){
	this->nodeSpacing = nodeSpacing;
	this->nbrSpacing = nbrSpacing;
	// 1 = observed free
	// 2 = inferred free
	// 3 = dominated free
	// 101 = observed obstacle
	// 102 = inferred obstacle
	// 201 unknown

	// get Mat of all observed or inferred free cells to make travel graph
	Mat freeMat = Mat::zeros(costmap.cells.size(), CV_8UC1);

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			if(costmap.cells.at<uchar>(i,j) <= costmap.domFree){
				freeMat.at<uchar>(i,j) = 255;
			}
		}
	}
	//int limits2[4] = {2, costmap.cells.cols-3, 2, costmap.cells.rows-3};


	/*
	int limits[4] = {costmap.cells.size(), 0, costmap.cells[0].size(), 0};
	for(size_t i = 0; i<costmap.cellUpdates.size(); i++){
		int x = costmap.cellUpdates[i][0];
		int y = costmap.cellUpdates[i][1];
		freeMat.at<uchar>(x,y) = 255;

		if(x < limits[0]){
			limits[0] = x;
		}
		else if(x > limits[1]){
			limits[1] = x;
		}
		if(y < limits[2]){
			limits[2] = y;
		}
		else if(y > limits[3]){
			limits[3] = y;
		}
	}
	*/


	namedWindow("Graph::createThinGraph::freeMat", WINDOW_NORMAL);
	imshow("Graph::createThinGraph::freeMat", freeMat);
	waitKey(1);

	if(thinMat.empty()){
		thinMat = Mat::zeros(costmap.cells.size(), CV_8UC1);
	}

	bitwise_or(freeMat, thinMat, freeMat);

	namedWindow("Graph::createThinGraph::freeMat", WINDOW_NORMAL);
	imshow("Graph::createThinGraph::freeMat", freeMat);
	waitKey(1);

	// get the thin free mat
	thinning(freeMat, thinMat);

	/*
	namedWindow("Graph::createThinGraph::thinMat", WINDOW_NORMAL);
	imshow("Graph::createThinGraph::thinMat", thinMat);
	waitKey(1);
	*/

	Mat tThinMat = thinMat.clone();

	// add nodes
	nodeLocations.clear();
	findStatesCityBlockDistance(tThinMat); // add them to graf;
	//findStateTransitionsCityBlockDistance(costmap); // find connections in graf; // TODO add line checking to this for intersections and increase distance
	//findStatesByGrid(freeMat);
	//findStateTransitionsByVisibility(costmap);
	//findCNodeTransitions(costmap);
	//mergeStatesBySharedNbr(); // TODO make this work, currently runs forever

}

/*

bool Graph::checkVisibility(Costmap &costmap, float dist, vector<int> a, vector<int> b){
	float unitVecX = (b[0] - a[0]) / dist; // get unit vector in right direction
	float unitVecY = (b[1] - a[1]) / dist;
	int steps = ceil(dist); // steps to check

	for(int m=1; m<steps; m++){ // check all intermediate points between two cells
		int aX = a[0] + m*unitVecX;
		int aY = a[1] + m*unitVecY;
		if(costmap.cells[aX][aY] != costmap.obsFree &&  costmap.cells[aX][aY] != costmap.infFree){
			return false;
		}
	}
	return true;
}

void Graph::breadthFirstSearchAssembleGraph(){
	// breadth first search a random state to all other states to get travel distances
	// during search check if each location is a state and log distance then remove that branch from the search
	// start search again from next states, think of this as exploring all edges attached to one state to get dist;

	// create a list of states
	vector<vector<int> > nLoc;
	for(int i=0; i<nodes.size(); i++){
		vector<int> tempList;
		tempList.push_back(graf[i][0]);
		tempList.push_back(graf[i][1]);
		//cout << graf[i].x << " & " << graf[i].y << endl;
		nLoc.push_back(tempList);
	}

	for(int i=0; i<nodes.size(); i++){
		for(int j=0; j<nodes.size(); j++){
			;
		}
		distGraph.push_back(asdf);
	}

	int flag[miniImage.rows][miniImage.cols]; // is this state traversable
	for(int i=0; i<miniImage.rows; i++){
		vector<float> asdf;
		for(int j=0; j<miniImage.cols; j++){
			asdf.push_back(0);
			flag[i][j] = (miniImage.at<uchar>(j,i) !=0); // is it traverseable? 1-yes, 0-no
		}
		distGraph.push_back(asdf);
	}

	//for(int i=0; i<miniImage.rows; i++){
	//	for(int j=0; j<miniImage.cols; j++){
	//		for(int k=0; k<nLoc.size(); k++){
	//			if(i==nLoc[k][0] && j==nLoc[k][1]){
	//				cout << "*";
	//			}
	//			else{
	//				cout << flag[i][j];
	//			}
	//		}
	//	}
	//	cout << endl;
	//}


	//cout << "There are " << nodes.size() << endl;
	//Mat coordGraph(10*miniImage.rows, 10*miniImage.cols, CV_8UC1);
	for(int seed = 0; seed<(int)nodes.size(); seed++){
		vector<vector<int> > oSet; // stores locations
		vector<float> dist; // stores distance to all members in oSet
		vector<vector<int> > cSet; // stores locations of members in closed set
		vector<vector<int> > foundstate; // stores location of found states

		// initialize open set
		vector<int> o;
		o.push_back(nLoc[seed][0]);
		o.push_back(nLoc[seed][1]);
		oSet.push_back(o); //

		// initialize distance to first item in open set
		dist.push_back(0); // starting location has a distance of 0

		//cout << "BEGIN ITERATION WITH SEED " << seed << " /////// nLoc: " << nLoc[seed][0] << "< " << nLoc[seed][1] << endl;
		while((int)oSet.size() > 0){
			//cout << "   oSet: ";
			//for(int k=0; k<(int)oSet.size(); k++){
			//	cout << oSet[k][0] << "," << oSet[k][1] << "; ";
			//}
			//cout << endl;
			// find closeststate to seed in the open set; i.e. the minimum distance
			float minDist = INFINITY;
			int minLoc[2];
			int mindex;
			for(int i=0; i<(int)oSet.size(); i++){
				if(dist[i] < minDist){
					minDist = dist[i];
					mindex = i;
					minLoc[0]= oSet[i][0];
					minLoc[1] = oSet[i][1];
				}
			}
			//cout << "minDex: " << minLoc[0] << "," << minLoc[1] << endl;

			// is mindex at an undiscovered state?
			//cout << "checking if minDex is at an undiscovered state" << endl;
			for(int j = -2; j<3; j++){
				for(int k=-2; k<3; k++){

					for(size_t i=0; i<nodes.size(); i++){ // am I at a new state?
						if(nLoc[i][0] == minLoc[0]+j && nLoc[i][1] == minLoc[1]+k && i != seed){
				//			cout << "  Found state " << i << " from seed " << seed << " with a dist of " << dist[mindex]+sqrt(pow(j,2)+pow(k,2)) << endl;
							distGraph[seed][i] = dist[mindex]+sqrt(pow(j,2)+pow(k,2));
							distGraph[i][seed] = dist[mindex]+sqrt(pow(j,2)+pow(k,2));
							vector<int> t;
							t.push_back(nLoc[i][0]);
							t.push_back(nLoc[i][1]);
							foundstate.push_back(t);
						}
					}
				}
			}

			// is mindex near a discovered state? if so, don't expand
			//cout << "checking if minDex is near a discovered state" << endl;
			//for(int i=0; i<(int)foundstate.size(); i++){
			//	if(sqrt(pow(foundstate[i][0]-minLoc[0] && foundstate[i][1] == minLoc[1],2)) < 1){ // am I near a state I have found before?
			//		cout << "   near a discovered state" << endl;
			//		skip = true;
			//	}
			//}

			// check minDex's nbrs to see if they should be added to the open set
			//cout << "checking nbrs" << endl;

			//for(int i = -2; i<3; i++){
			//	for(int j=-2; j<3; j++){
			//		if(i == 0 && j == 0){
			//			cout << "*";
			//		}
			//		else{
			//			cout << flag[minLoc[0] + i][minLoc[1] + j];
			//		}
			//	}
			//	cout << endl;
			//}

			for(int i = -3; i<4; i++){
				for(int j=-3; j<4; j++){
					if(flag[minLoc[0] + i][minLoc[1] + j] == 1){ // traversable
						bool cFlag = true;
						for(int k=0; k<(int)cSet.size(); k++){ // not in closed set
							if(cSet[k][0] == minLoc[0] + i && cSet[k][1] == minLoc[1] + j){
								cFlag = false;
							}
						}
						for(int k=0; k<(int)oSet.size(); k++){ // not in open set
							if(oSet[k][0] == minLoc[0] + i && oSet[k][1] == minLoc[1] + j){
								cFlag = false;
							}
						}
						if(cFlag){ // add to openSet
							vector<int> o;
							o.push_back(minLoc[0] + i);
							o.push_back(minLoc[1] + j);
							oSet.push_back(o);

							dist.push_back(dist[mindex] + sqrt(pow(i,2)+pow(j,2))); // get distance
							//cout << "   found a nbr: " << minLoc[0] + i << "," << minLoc[1] + j << " at dist: " <<  dist[mindex] + sqrt(pow(i,2)+pow(j,2)) << endl;
						}
					}
				}
				//cout << "out" << endl;
			}
			// move the current state out of open set and into closed set
			//cout << "moving minDex to closed set" << endl;
			vector<int> ml;
			ml.push_back(minLoc[0]);
			ml.push_back(minLoc[1]);
			cSet.push_back(ml);

			//cout << "   cSet: ";
			//for(int k=0; k<(int)cSet.size(); k++){
			//	cout << cSet[k][0] << "," << cSet[k][1] << "; ";
			//}
			//cout << endl;

			//cout << "   oSet: ";
			//for(int k=0; k<(int)oSet.size(); k++){
			//	cout << oSet[k][0] << "," << oSet[k][1] << "; ";
			//}
			//cout << endl;

			oSet.erase(oSet.begin()+mindex,oSet.begin()+mindex+1);
			dist.erase(dist.begin()+mindex,dist.begin()+mindex+1);
		}
	}

	for(size_t i=0; i<nodes.size(); i++){
		distGraph[i][i] = 0;
	}

	//cout << "DISTGRAPH:" << endl;
	//for(int i=0; i<nodes.size(); i++){
	//	for(int j=0; j<nodes.size(); j++){
	//		cout << floor(100*distGraph[i][j])/100 << " , ";
	//	}
	//	cout << endl;
	//}


	//Mat coordGraph(10*miniImage.rows, 10*miniImage.cols, CV_8UC1);
	//for(int i=0; i<(int)graf.size(); i++){
	//	Point temp;
	//	temp.x = graf[i][0]*10;
	//	temp.y = graf[i][1]*10;
	//	circle(coordGraph,temp,2,white,-1,8);
	//	char str[50];
	//	sprintf(str,"%d",i);
	//	putText(coordGraph, str, temp, FONT_HERSHEY_PLAIN,2,white);
	//}
	//for(int i=0; i<nodes.size(); i++){
	//	for(int j=0; j<nodes.size(); j++){
	//		if(distGraph[i][j] < 1000){
	//			Point temp, temp2;
	//			temp.x = graf[i][0]*10;
	//			temp.y = graf[i][1]*10;
	//			temp2.x = graf[j][0]*10;
	//			temp2.y = graf[j][1]*10;
	//			line(coordGraph, temp, temp2, white, 1,8);
	//		}
	//	}
	//}
	//imshow("coordGraph", coordGraph);
	//waitKey(1);
}

void Graph::findPointOfInterestNodes(){

	int x, y;

	uchar *pAbove;
	uchar *pCurr;
	uchar *pBelow;

	uchar *bb, *bc, *bd;
	uchar *cb, *cc, *cd;
	uchar *db, *dc, *dd;

	// initialize row pointers
	pAbove = NULL;
	pCurr  = miniImage.ptr<uchar>(0);
	pBelow = miniImage.ptr<uchar>(1);

	graf.clear();
	for (y = 1; y < miniImage.rows-1; ++y) {
		// shift the rows up by one
		pAbove = pCurr;
		pCurr  = pBelow;
		pBelow = miniImage.ptr<uchar>(y+1);

		// initialize col pointers

		bb = &(pAbove[0]);
		bc = &(pAbove[1]);
		bd = &(pAbove[2]);

		cb = &(pCurr[0]);
		cc = &(pCurr[1]);
		cd = &(pCurr[2]);

		db = &(pBelow[0]);
		dc = &(pBelow[1]);
		dd = &(pBelow[2]);

		for (x = 1; x < miniImage.cols-1; ++x) {
			// shift col pointers left by one (scan left to right)
			bb = bc;
			bc = bd;
			bd = &(pAbove[x+2]);

			cb = cc;
			cc = cd;
			cd = &(pCurr[x+2]);

			db = dc;
			dc = dd;
			dd = &(pBelow[x+1]);

			int outerEdge[9];
			outerEdge[0] = (*bb != 0);
			outerEdge[1] = (*bc != 0);
			outerEdge[2] = (*bd != 0);
			outerEdge[3] = (*cd != 0);
			outerEdge[4] = (*dd != 0);
			outerEdge[5] = (*dc != 0);
			outerEdge[6] = (*db != 0);
			outerEdge[7] = (*cb != 0);
			outerEdge[8] = (*bb != 0);

			int edgeDetector = 0;
			for(int i=0; i<8; i++){
				if(outerEdge[i] != outerEdge[i+1]){
					edgeDetector++;
				}
			}

			if(*cc != 0 && edgeDetector != 4 && edgeDetector != 0){ // is the center pixel traversable && if 4 edges then there is one way into and one way out of the traversable path, not a state && is there a way to the state
				vector<int> t;
				t.push_back(x);
				t.push_back(y);
				graf.push_back(t);
				cout << "vec: ";
				for(int i=0; i<9; i++){
					cout << outerEdge[i] << ",";
				}

			    cout << endl;
			    cout << "edgeDetector: " << edgeDetector << endl;
			    cout << (*bb != 0) << "," << (*bc != 0) << "," << (*bd != 0) << endl;
			    cout << (*cb != 0) << "," << (*cc != 0) << "," << (*cd != 0) << endl;
			    cout << (*db != 0) << "," << (*dc != 0) << "," << (*dd != 0) << endl << endl;
			}
		}
	}
}

void Graph::breadthFirstSearchstateConnections(vector<vector<int> > openstates){

	graf.clear();
	graf.push_back(openstates[0]);
	cerr << "0" << endl;
	while(openstates.size() > 0){

		int mindex = 0;
		float mindist = INFINITY;

		size_t i =0;
		cerr << "on.size(): " << openstates.size() << endl;
		while(i < openstates.size()){ // while there are open states left to check
			bool flag = true;
			int myMindex = -1;
			float myMindist = INFINITY;

			for(size_t j=0; j<graf.size(); j++){ // check against all states already in graf
				float dist = euclidianDist(openstates[i], graf[j]);
				cerr << "   mindist / myMinDist , dist: " << myMindist << " , " << dist;
				if(dist < grafSpacing){
					cerr << ": state removed" << endl;
					openstates.erase(openstates.begin()+i);
					flag = false;
					break;
				}
				else if(dist < mindist){
					cerr << ": new closest state" << endl;
					myMindist = dist;
					myMindex = j;
				}
				cerr << "end for loop: " << myMindex << endl;
			}
			if(flag && myMindist < mindist){
				mindist = myMindist;
				mindex = i;
			}
			i++;
		}
		cerr << "mindex / on.size(): " << mindex << " / " << openstates.size() << endl;
		if(openstates.size() > 0){
			graf.push_back(openstates[mindex]);
		}
		cerr << "4" << endl;
	}
}

void Graph::condenseGraph(Costmap &costmap){
	vector<Node> keep;
	for(size_t i=0; i<nodes.size(); i++){
		bool flag = true;
		for(size_t j=0; j<nodes.size(); j++){
			if(costmap.getEuclidianDistance(nodes[i].loc,nodes[j].loc) < nodeSpacing && i!=j){
				vector<int> c;
				int a = (nodes[i].loc[0] + nodes[j].loc[0])/2;
				int b = (nodes[i].loc[1] + nodes[j].loc[1])/2;
				c.push_back(a);
				c.push_back(b);
				Node d(c);
				keep.push_back(d);
				flag = false;
			}
		}
		if(flag){
			keep.push_back(nodes[i]);
		}
	}
	nodes.clear();

	nodes = keep;
}
*/




