/*
 * Costmap.cpp
 *
 *  Created on: Jul 12, 2016
 *      Author: andy
 */




#include "Costmap.h"

bool pointCompare(Point &a, Point &b);
bool pointOnMat(Point &a, Mat &b);

Costmap::Costmap(){

	ros_unknown = -1;
	ros_wall = 100;
	ros_free = 0;

	Vec3b a(255,255,255);
	cObsFree = a;

	Vec3b b(200,200,200);
	cInfFree = b;

	Vec3b c(0,0,0);
	cObsWall = c;

	Vec3b d(50,50,50);
	cInfWall = d;

	Vec3b e(127,127,127);
	cUnknown = e;

	Vec3b f(255,0,0);
	cError = f;

	this->obsFree = 1;
	this->infFree = 2;
	this->domFree = 3;

	this->unknown = 101;

	this->obsWall = 201;
	this->infWall = 202;
	this->inflatedWall = 203;
}


Costmap::~Costmap() {}

void Costmap::simulateObservation(Point pose, float obsRadius){

	// make perimeter of viewing circle fit on image
	if( viewPerim.size() == 0 ){
		Mat temp =Mat::zeros(2*(obsRadius + 1), 2*(obsRadius + 1), CV_8UC1);
		Point cent;
		cent.x = obsRadius;
		cent.y = obsRadius;
		circle(temp,cent,obsRadius, Scalar(255));

		for(int i=0; i<temp.cols; i++){
			for(int j=0; j<temp.rows; j++){
				if(temp.at<uchar>(i,j,0) == 255){
					Point t(i-obsRadius, j-obsRadius);
					viewPerim.push_back(t);
				}
			}
		}
	}

	Mat temp =Mat::zeros(cells.size(), CV_8UC1);
	cellUpdates.clear();
	for(size_t i=0; i<viewPerim.size(); i++){
		Point v(viewPerim[i].x + pose.x, viewPerim[i].y + pose.y);
		LineIterator it(temp, pose, v, 8, false);
		for(int i=0; i<it.count; i++, ++it){
			Point pp  = it.pos();
			if(cells.at<short>(pp) > domFree){
				break;
			}
			else{
				cellUpdates.push_back(pp);
				//temp.at<uchar>(pp) = 255;
			}
		}
	}

	/*
	circle(temp, pose, 2, Scalar(127), -1, 8);
	namedWindow("GraphPlanning::simulateView::view", WINDOW_NORMAL);
	imshow("GraphPlanning::simulateView::view", temp);
	waitKey(0);
	*/
}

void Costmap::getRewardMat(float w[3], float e[2], float spread){
	// explore, search, map; dominated, breach; spread rate

	if(reward.empty()){
		reward = Mat::zeros(cells.size(), CV_32FC1);
	}

	if(w[1] > 0){ // spread if needed
		spreadSearchArea(spread);
		displaySearchReward();
	}

	for(int i=0; i<cells.cols; i++){
		for(int j=0; j<cells.rows; j++){
			Point a(i,j);
			float r[3] = {0,0,0};
			if(w[0] > 0){
				// explore reward
				if(cells.at<short>(a) == infFree){
					r[0] = 1;
				}
				else if(cells.at<short>(a) == domFree){
					r[0] = e[0];
				}
			}

			if(w[1] > 0){
				// search reward
				r[1] = searchReward.at<float>(a);
			}

			if(w[2] > 0){
				// map reward
				if(1-occ.at<float>(a) > occ.at<float>(a) ){
					r[2] = occ.at<float>(a);
				}
				else{
					r[2] = 1-occ.at<float>(a);
				}
			}
			reward.at<float>(a) = w[0]*r[0] + w[1]*r[1] + w[2]*r[2];
		}
	}

	if(w[0] > 0){ // add hull breaches reward
		for(size_t k=0; k<hullBreaches.size(); k++){
			reward.at<float>(hullBreaches[k]) = e[1];
		}
	}
}



void Costmap::initCostmap(int h, int w){
	map_height = h;
	map_width = w;

	Mat a = Mat::ones( map_height, map_width, CV_16S)*unknown;
	cells = a.clone();

	a = Mat::zeros( map_height, map_width, CV_32FC1);
	reward = a.clone();

	a = Mat::ones( map_height, map_width, CV_32FC1)*-1;
	euclidDist = a.clone();



	init_flag = false;

	ROS_ERROR("initialized costmap");
}

vector<int8_t> Costmap::updateCells( vector<int8_t> &occupancy_grid_array ){

	cout << cells.cols << " / " << cells.rows << endl;
	vector<int8_t> cmud;

	for(size_t i=0; i<occupancy_grid_array.size(); i++){
		 Point p = getCellIndex( i );

		 //ROS_INFO("p.x/y: %d/%d: %d", p.x, p.y,occupancy_grid_array[i]);
		 


		 if(occupancy_grid_array[i] == ros_unknown){
		 	if( cells.at<short>(p) != unknown){
		 		cells.at<short>(p) = unknown;
		 		cmud.push_back(p.x);
		 		cmud.push_back(p.y);
		 		cmud.push_back(unknown);
		 	}
		 }
		 else if(occupancy_grid_array[i] < ros_wall / 4){//ros_free){
		 	if( cells.at<short>(p) != obsFree){
		 		cells.at<short>(p) = obsFree;
		 		cmud.push_back(p.x);
		 		cmud.push_back(p.y);
		 		cmud.push_back(obsFree);
		 	}
		 }
		 else{//} if(occupancy_grid_array[i] == ros_wall){
		 	if( cells.at<short>(p) != obsWall){
		 		cells.at<short>(p) = obsWall;
		 		cmud.push_back(p.x);
		 		cmud.push_back(p.y);
		 		cmud.push_back(obsWall);
		 	}
		 }
	}
	return cmud;
}

 Point Costmap::getCellIndex(int l){ //TODO check these
	Point p;
	p.x = floor( l / map_width );
	p.y = l % map_height;

	return p;
}

int Costmap::getArrayIndex( Point p){ //TODO chek these
	int l = p.x * map_width + p.y;
	return l;
}

void Costmap::prettyPrintCostmap(){
	for(int i=0; i<cells.cols; i++){
		for(int j=0; j<cells.rows; j++){
			Point a(i,j);
			cout << cells.at<short>(a) << ",";
		}
		cout << endl;
	}
}

bool pointCompare(Point &a, Point &b){
	if(a.x == b.x && a.y == b.y){
		return true;
	}
	else{
		return false;
	}
}

bool pointOnMat(Point &a, Mat &b){
	if(a.x >= 0 && a.x < b.cols && a.y >= 0 && a.y < b.rows){
		return true;
	}
	else{
		return false;
	}
}

void Costmap::updateOccGrid(){

	if(occ.empty()){
		occ = Mat::ones(cells.size(), CV_32FC1)*0.5;
	}

	// set obstacles in costmap
	float pOcc = 0.65;
	float pFree = 0.35;

	int dx[4] = {-1,1,0,0};
	int dy[4] = {0,0,-1,1};

	for(size_t i=0; i<cellUpdates.size(); i++){
		Point c = cellUpdates[i];
		occ.at<float>(c) = occ.at<float>(c)*pFree / (occ.at<float>(c)*pFree + (1-occ.at<float>(c))*(1-pFree) ); // this cell is always free and being updated
		for(int k=0; k<4; k++){
			Point a(c.x+dx[k], c.y+dy[k]);
			if(cells.at<short>(a) == obsWall){ // are any of my nbrs obsWall?
				occ.at<float>(a) = occ.at<float>(a)*pOcc / (occ.at<float>(a)*pOcc + (1-occ.at<float>(a))*(1-pOcc) ); // nbr cell that is a wall
			}
		}
	}

	buildOccPlot();
}

void Costmap::buildOccPlot(){
	Mat temp= Mat::zeros(cells.size(),CV_8UC1);
	for(int i=0; i<cells.cols; i++){
		for(int j=0; j<cells.rows; j++){
			Point a(i,j);
			uchar shade;
			shade = round((1-occ.at<float>(a))*255);
			temp.at<uchar>(a) = shade;
		}
	}

	namedWindow("Occupancy Grid", WINDOW_NORMAL);
	imshow("Occupancy Grid", temp);
	waitKey(1);
}

void Costmap::spreadSearchArea(float growthRate){

	if(searchReward.empty()){
		searchReward = Mat::zeros(cells.size(), CV_32FC1);
	}

	Mat ts = Mat::zeros(cells.size(), CV_32FC1);

	int sx[5] = {-1,0,0,0,1};
	int sy[5] = {0,1,0,-1,0};
	for(int i=1; i<cells.cols-1; i++){
		for(int j=1; j<cells.rows-1; j++){
			Point loc(i,j);
			if(cells.at<short>(loc) == obsFree){
				float sReward = 0;
				for(int a = 0; a<5; a++){
					Point nLoc(i+sx[a], j+sy[a]);
					sReward += searchReward.at<float>(nLoc);
				}
				if(sReward > (1-growthRate) * 5){
					ts.at<float>(loc) = 1;
				}
				else if(sReward < 0.025){
					ts.at<float>(loc) = 0;
				}
				else{
					ts.at<float>(loc) = sReward / ( (1-growthRate) * 5);
				}
			}
			else if(cells.at<short>(loc) == domFree || cells.at<short>(loc) == infFree){
				ts.at<float>(loc) = 1;
			}
		}
	}

	searchReward = ts.clone();
	
	for(size_t i=0; i<cellUpdates.size(); i++){
		if(cells.at<short>(cellUpdates[i]) != infFree){
			searchReward.at<float>(cellUpdates[i]) = 0;
		}
	}
	

}

void Costmap::displayThermalMat(Mat &mat, Point cLoc){
	Mat tMat = Mat::zeros( cells.size(), CV_8UC3);

	float maxV = -1;

	for(int i=0; i<mat.cols; i++){
		for(int j=0; j<mat.rows; j++){
			Point a(i,j);
			if(mat.at<float>(a) > maxV){
				if( mat.at<float>(a) > 1 ){
					mat.at<float>(a) = 1;
					maxV = 1;
				}
				else{
					maxV = mat.at<float>(a);
				}
			}
		}
	}

	for(int i=0; i<mat.cols; i++){
		for(int j=0; j<mat.rows; j++){
			Point a(i,j);

			if(cells.at<short>(a) <= domFree){
				float r = mat.at<float>(a) / maxV;

				Vec3b color(0,0,0);
				if(r < 0.125){
					color[1] = 255;
					color[2] = 0;
				}

				else if( r > 0.875 ){
					color[1] = 0;
					color[2] = 255;
				}
				else{
					color[1] = round( 255 + (r+0.125)*-255/maxV );
					color[2] = round( (r-0.125)*255/(maxV-0.125) );
				}
				tMat.at<Vec3b>(a) = color;
			}
		}
	}

	if(cLoc.x >= 0){
		Scalar c(255,0,0);
		cv::circle( tMat, cLoc, 2, c, -1, 8);
	}

	Point2f src_center(tMat.cols/2.0F, tMat.rows/2.0F);
	Mat rot_mat = getRotationMatrix2D(src_center, 270, 1.0);
	Mat dst;
	warpAffine(tMat, dst, rot_mat, tMat.size());

	namedWindow("Thermal Mat", WINDOW_NORMAL);
	imshow("Thermal Mat", dst);
	waitKey(1);
}

void Costmap::showCostmapPlot(int index){ // show nice display plot and number it
	
	char buffer[50];
	sprintf(buffer, "Cells Mat %d", index);

	namedWindow(buffer, WINDOW_NORMAL);
	imshow(buffer, displayPlot);
	waitKey(100);
}

void Costmap::displaySearchReward(){

	Mat msr = Mat::zeros( cells.size(), CV_8UC3);
	for(int i=0; i<searchReward.cols; i++){
		for(int j=0; j<searchReward.rows; j++){
			Point loc(i,j);
			if(cells.at<short>(loc) <= domFree){
				float sr = searchReward.at<float>(loc);
				if(sr > 1){
					sr = 1;
				}

				Vec3b color(0,0,0);
				if(sr < 0.125){
					color[1] = 255;
					color[2] = 0;
				}

				else if( sr > 0.875 ){
					color[1] = 0;
					color[2] = 255;
				}
				else{
					color[1] = round( 255 + (sr+0.125)*-255 );
					color[2] = round( (sr - 0.125)*340 );
				}

				msr.at<Vec3b>(loc) = color;
			}
		}
	}

	for(size_t i=0; i<cellUpdates.size(); i++){
		Vec3b white(255, 255, 255);
		msr.at<Vec3b>(cellUpdates[i]) = white;
	}

	namedWindow("Search Reward", WINDOW_NORMAL);
	imshow("Search Reward", msr);
	waitKey(1);
}

void Costmap::buildCellsPlot(){
	this->displayPlot= Mat::zeros(cells.size(),CV_8UC3);
	for(int i=0; i<cells.cols; i++){
		for(int j=0; j<cells.rows; j++){
			Point a(i,j);
			if(this->cells.at<short>(a) == this->obsFree){
				this->displayPlot.at<Vec3b>(a) = this->cObsFree;
			}
			else if(this->cells.at<short>(a) == this->infFree){
				this->displayPlot.at<Vec3b>(a) = this->cInfFree;
			}
			else if(this->cells.at<short>(a) == this->obsWall){
				this->displayPlot.at<Vec3b>(a) = this->cObsWall;
			}
			else if(this->cells.at<short>(a) == this->infWall){
				this->displayPlot.at<Vec3b>(a) = this->cInfWall;
			}
			else if(this->cells.at<short>(a) == this->inflatedWall){
				this->displayPlot.at<Vec3b>(a) = this->cInfWall;
			}
			else if(this->cells.at<short>(a) == this->unknown){
				this->displayPlot.at<Vec3b>(a) = this->cUnknown;
			}
			else{ // anything else, should never happen
				this->displayPlot.at<Vec3b>(a) = this->cError;
			}
		}
	}
}

void Costmap::addAgentToCostmapPlot(Scalar color, vector<Point> myPath, Point cLoc){
	circle(displayPlot, cLoc, 2, color, -1);
	for(size_t i=1; i<myPath.size(); i++){
		Point a = myPath[i];
		Point b = myPath[i-1];
		line(displayPlot, a, b, color, 1);
	}
}

vector<Point> Costmap::getImagePointsAt(Mat &image, int intensity){
	vector<Point> temp;
	for(int i=0; i<image.cols; i++){
		for(int j=0; j<image.rows; j++){
			if(image.at<uchar>(i,j,0) == intensity){
				Point t(i,j);
				temp.push_back(t);
			}
		}
	}
	return temp;
}

float Costmap::getEuclidianDistance(Point a, Point b){
	int dx = a.x - b.x;
	int dy = a.x - b.y;

	if(euclidDist.at<float>(dx,dy) == -1){
		euclidDist.at<float>(dx,dy) = sqrt(pow(dx,2) + pow(dy,2));
	}

	return(euclidDist.at<float>(dx,dy) );
}


vector<Point> Costmap::aStarPath(Point sLoc, Point gLoc){

	if(sLoc == gLoc){
		vector<Point> totalPath;
		for(int i=0; i<4; i++){
			Point t = sLoc;
			totalPath.push_back(t);
		}
		return totalPath;
	}

	Mat cSet = Mat::zeros(cells.size(), CV_16S); // 1 means in closed set, 0 means not
	Mat oSet = Mat::zeros(cells.size(), CV_16S); // 1 means in open set, 0 means not
	Mat cameFromX = Mat::ones(cells.size(), CV_16S)*-1; // each square has a vector of the location it came from
	Mat cameFromY = Mat::ones(cells.size(), CV_16S)*-1; // each square has a vector of the location it came from

	Mat gScore = Mat::ones(cells.size(), CV_32F)*INFINITY; // known cost from initial node to n
	Mat fScore = Mat::ones(cells.size(), CV_32F)*INFINITY; // known cost from initial node to n

	vector<Point> oVec;
	oVec.push_back(sLoc);
	oSet.at<short>(sLoc) = 1; // starting node has score 0
	gScore.at<float>(sLoc)  = 0; // starting node in open set
	fScore.at<float>(sLoc) = sqrt(pow(sLoc.x-gLoc.x,2) + pow(sLoc.y-gLoc.y,2));
	fScore.at<float>(gLoc) = 1;

	bool foo = true;

	while(foo){
		/////////////////// this finds node with lowest fScore and makes current
		float min = INFINITY;
		int mindex = -1;

		for(size_t i=0; i<oVec.size(); i++){
			if(fScore.at<float>(oVec[i]) < min){
				min = fScore.at<float>(oVec[i]);
				mindex = i;
			}
		}

		Point cLoc = oVec[mindex];
		oVec.erase(oVec.begin() + mindex);
		oSet.at<short>(cLoc) = 0;
		cSet.at<short>(cLoc) = 1;

		//cout << "*** sLoc/cLoc/gLoc: " << sLoc << " / "<< cLoc << " / "<< gLoc << endl;

		/////////////////////// end finding current node
		if(pointCompare(cLoc, gLoc) ){ // if the current node equals goal, construct path
			vector<Point> totalPath;
			totalPath.push_back(gLoc);
			while( cLoc.x != sLoc.x || cLoc.y != sLoc.y ){ // work backwards to start
				Point tLoc(cameFromX.at<short>(cLoc), cameFromY.at<short>(cLoc));
				totalPath.push_back(tLoc); // append path
				cLoc.x = tLoc.x;
				cLoc.y = tLoc.y;
			}
			reverse(totalPath.begin(),totalPath.end());
			return totalPath;
		} ///////////////////////////////// end construct path

		// for nbrs
		int nx[8] = {-1,-1,-1,0,0,1,1,1};
		int ny[8] = {1,0,-1,1,-1,1,0,-1};

		for(int ni = 0; ni<8; ni++){
			Point nbr;
			nbr.x += cLoc.x + nx[ni];
			nbr.y += cLoc.y + ny[ni];
			if(pointOnMat(nbr, cells) ){
				//cout << "sLoc/cLoc/nbr/gLoc: " << sLoc << " / "<< cLoc << " / "<< nbr << " / "<< gLoc << endl;
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
				cameFromX.at<short>(nbr) = cLoc.x;
				cameFromY.at<short>(nbr) = cLoc.y;

				gScore.at<float>(nbr) = ngScore;
				if(cells.at<short>(nbr) < 102){
					fScore.at<float>(nbr) = gScore.at<float>(nbr) + sqrt(pow(nbr.x-gLoc.x,2) + pow(nbr.y-gLoc.y,2));// +getEuclidianDistance(gLoc,nbr)
				}
				else{
					fScore.at<float>(nbr)= INFINITY;
				}
			}
		}
		/////////////// end condition for while loop, check if oSet is empty
		if(oVec.size() == 0){
			foo = false;
		}
	}
	vector<Point> totalPath;
	for(int i=0; i<4; i++){
		Point t = sLoc;
		totalPath.push_back(t);
	}
	return totalPath;
}

float Costmap::aStarDist(Point sLoc, Point gLoc){

	//cerr << "Costmap::aStarDist::in" << endl;


	if(sLoc == gLoc){
		return 0;
	}

	Mat cSet = Mat::zeros(cells.size(), CV_16SC1); // 1 means in closed set, 0 means not
	Mat oSet = Mat::zeros(cells.size(), CV_16SC1); // 1 means in open set, 0 means not

	Mat gScore = Mat::ones(cells.size(), CV_32FC1)*INFINITY; // known cost from initial node to n
	Mat fScore = Mat::ones(cells.size(), CV_32FC1)*INFINITY; // known cost from initial node to n

	vector<Point> oVec;
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
			if(pointOnMat(nbr, cells) ){
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
				if(cells.at<short>(nbr) < obsWall){
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


float Costmap::cumulativeAStarDist(Point sLoc, Point gLoc, Mat &cSet, vector<Point> &oSet, Mat &fScore, Mat &gScore){

	if(cSet.empty() == 1){
		cSet = Mat::zeros(cells.size(), CV_16SC1);

		fScore = Mat::zeros(cells.size(), CV_32F);
		gScore = Mat::zeros(cSet.size(), CV_32F);
		oSet.push_back(sLoc);
	}
	else if(cSet.at<short>(gLoc) == 255){
		return gScore.at<float>(gLoc);
	}

	// for nbrs
	int nx[8] = {-1,-1,-1,0,0,1,1,1};
	int ny[8] = {1,0,-1,1,-1,1,0,-1};

	Point nbrLoc(-1,-1);

	if(cells.at<short>(sLoc) > domFree){
		return INFINITY;
	}
	fScore.at<float>(sLoc) = getEuclidianDistance(gLoc,sLoc);

	while(oSet.size()){
		/////////////////// this finds node with lowest fScore and makes current
		float minF = INFINITY;
		int cIndex = -1;
		for(size_t i=0; i<oSet.size(); i++){
			if(fScore.at<float>(oSet[i]) < minF){
				minF = fScore.at<float>(oSet[i]);
				cIndex = i;
			}
		}

		Point cLoc = oSet[cIndex];
		float cGScore = gScore.at<float>(oSet[cIndex]);
		float cFScore = minF;

		cSet.at<short>(cLoc) = 255;
		oSet.erase(oSet.begin() + cIndex);

		/*
		cMat.at<short>(gLoc[0], gLoc[1]) = 127;
		namedWindow("A star", WINDOW_NORMAL);
		imshow("A star", cMat);
		waitKey(1);
		*/
		/////////////////////// end finding current node
		if(pointCompare(cLoc, gLoc) ){ // if the current node equals goal, construct path
			return cFScore;
		} ///////////////////////////////// end construct path

		for(int nbr = 0; nbr<8; nbr++){

			Point nbrLoc(cLoc.x + nx[nbr], cLoc.y + ny[nbr]);

			if(nbrLoc.x >= 0 && nbrLoc.x < cells.cols && nbrLoc.y >= 0 && nbrLoc.y < cells.rows){
				bool cFlag = true;

				if(cSet.at<short>(nbrLoc) == 255){ // has it already been eval? in cSet
					cFlag = false;
				}

				if(cFlag){
					if(this->cells.at<short>(nbrLoc) <= unknown){
						float tGScore = cGScore + getEuclidianDistance(nbrLoc, cLoc); // calc temporary gscore
						cFlag = true;
						for(size_t i=0; i<oSet.size(); i++){
							if(oSet[i] == nbrLoc){
								cFlag = false; // already in oSet, update score if found cheaper
								if(tGScore < gScore.at<float>(oSet[i]) ){
									gScore.at<float>(oSet[i]) = tGScore;
									fScore.at<float>(oSet[i]) = tGScore + getEuclidianDistance(gLoc,nbrLoc);
								}
								break;
							}
						}

						if(cFlag){ // not in oSet, so add
							oSet.push_back(nbrLoc);  // add nbr to open set
							gScore.at<float>(nbrLoc) = tGScore;
							fScore.at<float>(nbrLoc) = tGScore + getEuclidianDistance(gLoc,nbrLoc);
						}
					}
					else{
						cSet.at<short>(nbrLoc) = 255;
					}
				}
			}
		}
	}

	return INFINITY;
}

/*


float Costmap::getPercentObserved(Costmap &globalCostmap, Costmap &workingCostmap){

	float globalFree = 0;
	float workingFree = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
				globalFree++;
			}
			if(workingCostmap.cells.at<short>(a) == workingCostmap.obsFree){
				workingFree++;
			}
		}
	}

	return workingFree / globalFree;
}

float Costmap::getPercentDominated(Costmap &globalCostmap, Costmap &workingCostmap){

	float globalFree = 0;
	float dominated = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
				globalFree++;
			}
			if(workingCostmap.cells.at<short>(a) == workingCostmap.domFree){
				dominated++;
			}
		}
	}

	return dominated / globalFree;
}

float Costmap::getPercentObservedAndInferredCorrectly(Costmap &globalCostmap, Costmap &workingCostmap){

	int globalFree = 0;
	int infCorrect = 0;
	int obs = 0;
	int infCount = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			// get global free
			if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
				globalFree++;
			}

			if(workingCostmap.cells.at<short>(a) == workingCostmap.infFree){ // inferred free
				infCount++;
				if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
					infCorrect++;
				}
			}
			else if(workingCostmap.cells.at<short>(a) == workingCostmap.obsFree){
				obs++;
			}

		}
	}
	return float(infCorrect + obs)/float(globalFree);
}

float Costmap::getPercentObservedAndInferred(Costmap &globalCostmap, Costmap &workingCostmap){

	int globalFree = 0;
	int infCount = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			// get global free
			if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
				globalFree++;
			}

			if(workingCostmap.cells.at<short>(a)== workingCostmap.infFree || globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){ // inferred free
				infCount++;
			}

		}
	}
	return float(infCount)/float(globalFree);
}

float Costmap::getPercentInferredCorrectly(Costmap &globalCostmap, Costmap &workingCostmap){

	int infCorrect = 0;
	int infCount = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(workingCostmap.cells.at<short>(a) == workingCostmap.infWall || workingCostmap.cells.at<short>(a) == workingCostmap.inflatedWall){ // inferred wall
				infCount++;
				if(globalCostmap.cells.at<short>(a) == globalCostmap.obsWall){
					infCorrect++;
				}
			}
			else if(workingCostmap.cells.at<short>(a) == workingCostmap.infFree){ // inferred free
				infCount++;
				if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
					infCorrect++;
				}
			}
		}
	}
	return float(infCorrect)/float(infCount);
}

float Costmap::getPercentInferredWrongly(Costmap &globalCostmap, Costmap &workingCostmap){

		int infWrong = 0;
		int infCount = 0;

		for(int i=0; i<globalCostmap.cells.cols; i++){
			for(int j=0; j<globalCostmap.cells.rows; j++){
				Point a(i,j);
				if(workingCostmap.cells.at<short>(a) == workingCostmap.infWall || workingCostmap.cells.at<short>(a) == workingCostmap.inflatedWall){ // inferred wall
					infCount++;
					if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
						infWrong++;
					}
				}
				else if(workingCostmap.cells.at<short>(a) == workingCostmap.infFree){ // inferred free
					infCount++;
					if(globalCostmap.cells.at<short>(a) == globalCostmap.obsWall){
						infWrong++;
					}
				}
			}
		}
		return float(infWrong)/float(infCount);
	}


float Costmap::aStarDist(vector<int> sLoc, vector<int> gLoc){
	float costMapPenalty = 0;
	vector<vector<int> > cSet; // 1 means in closed set, 0 means not
	vector<vector<int> > oSet; // 1 means in open set, 0 means not
	vector<vector<float> > gScore; // known cost from initial node to n
	vector<vector<float> > fScore; // gScore + heuristic score (dist to goal + imposed cost)
	for(int i=0;i<this->nCols; i++){
		vector<int> tC(this->nRows,INFINITY);
		vector<int> tO(this->nRows, INFINITY);
		vector<float> tG(this->nRows, INFINITY);
		vector<float> tF(this->nRows, INFINITY);
		cSet.push_back(tC);
		oSet.push_back(tC);
		gScore.push_back(tG); // init scores to inf
		fScore.push_back(tF); // init scores to inf
	}

	oSet[sLoc[0]][sLoc[1]] = 1; // starting node has score 0
	gScore[sLoc[0]][sLoc[1]] = 0; // starting node in open set
	if(this->cells[sLoc[0]][sLoc[1]] < 3){
		costMapPenalty = this->cells[sLoc[0]][sLoc[1]];
	}
	else{
		costMapPenalty = INFINITY;
	}
	fScore[sLoc[0]][sLoc[1]] = gScore[sLoc[0]][sLoc[1]] + this->getEuclidianDistance(sLoc,gLoc) + costMapPenalty;

	bool foo = true;
	int finishFlag = true;
	while(foo && finishFlag){
		/////////////////// this finds node with lowest fScore and makes current
		float min = INFINITY;
		vector<int> iMin;
		iMin.push_back(0);
		iMin.push_back(0);
		for(int i=0; i<this->nCols; i++){
			for(int j=0; j<this->nRows; j++){
				if(oSet[i][j] > 0 && fScore[i][j] < min){
					min = fScore[i][j];
					iMin[0] = i;
					iMin[1] = j;
				}
			}
		}
		vector<int> cLoc = iMin;

		vector<int> nbrLoc;
		nbrLoc.push_back(-1);
		nbrLoc.push_back(-1);
		/////////////////////// end finding current node
		if(cLoc == gLoc){ // if the current node equals goal, construct path
			finishFlag = false;
			return fScore[gLoc[0]][gLoc[1]];
		} ///////////////////////////////// end construct path
		oSet[cLoc[0]][cLoc[1]] = 0;
		cSet[cLoc[0]][cLoc[1]] = 1;
		for(int nbrRow=cLoc[1]-1;nbrRow<cLoc[1]+2;nbrRow++){
			if(nbrRow >= 0 && nbrRow < this->nRows){
				for(int nbrCol=cLoc[0]-1; nbrCol<cLoc[0]+2; nbrCol++){
					if(nbrCol >= 0 && nbrCol < this->nCols){
						if(cSet[nbrCol][nbrRow] == 1){ // has it already been eval? in cSet
							continue;
						}
						float tGScore;
						nbrLoc[0] = nbrCol;
						nbrLoc[1] = nbrRow;

						tGScore = gScore[cLoc[0]][cLoc[1]] + this->getEuclidianDistance(cLoc, nbrLoc); // calc temporary gscore
						if(oSet[nbrCol][nbrRow] == 0){
							oSet[nbrCol][nbrRow] = 1;  // add nbr to open set
						}
						else if(tGScore >= gScore[nbrCol][nbrRow]){ // is temp gscore better than stored g score of nbr
							continue;
						}
						gScore[nbrCol][nbrRow] = tGScore;
						if(this->cells[nbrCol][nbrRow] < 102){
							costMapPenalty = 0;//this->costMap[nbrCol][nbrRow];
						}
						else{
							costMapPenalty = INFINITY;
						}
						nbrLoc[0] = nbrCol;
						nbrLoc[1] = nbrRow;
						fScore[nbrCol][nbrRow] = gScore[nbrCol][nbrRow] + this->getEuclidianDistance(gLoc,nbrLoc) + costMapPenalty;
					}
				}
			}
		}
		/////////////// end condition for while loop, check if oSet is empty
		foo = true;
		for(int i=0; i<this->nCols; i++){
			for(int j=0; j<this->nRows; j++){
				if(oSet[i][j]){
					foo = false;
					break;
				}
				if(foo){
					break;
				}
			}
		}
	}
	return 0;
}
*/
