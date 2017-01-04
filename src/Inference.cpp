/* Inference.cpp
 *
 *  Created on: Jul 12, 2016
 *      Author: andy
 */

#include "Inference.h"

Point findIntersection(Vec4i w1, Vec4i w2);
float distToLine(Vec4i w, Point a);
Point extendLine(Point a, Point m);
float distToLineSegment(Point p, Point v, Point w);
void drawHistogram(vector<float> histogram, char* title);
vector<Point> findPointsAlongLine(Costmap &costmap, Point pose, Point pt);
void scanAlongEdge(Costmap &costmap, Point m, Point n1, Point n2, vector<float> &lengths, vector<Vec4i> &endPts, float searchDistance);
void lengthAlongLine(Costmap &costmap, Point pose, Point pt, Point &endPt, float &length);
vector<Vec4i> checkForDoors(vector<float> lengths, vector<Vec4i> endPts);
bool pointCompare(Point &a, Point &b);  // in costmap.cpp
bool pointOnMat(Point &a, Mat &b);  // in costmap.cpp
float absV( float num );
void printVector( vector<float> vec, string name );
float euclidianDistance( Point a, Point b );

Inference::Inference() {
	/*
	FileStorage fsR("/home/andy/git/fabmap2Test/InferenceLibrary/roomLibrary.yml", FileStorage::READ);
	fsR["names"] >> roomNameList;
	fsR["histogramList"] >> roomHistogramList;
	fsR["pointList"] >> roomPointList;
	fsR["centerList"] >> roomCenterList;
	fsR["meanLength"] >> roomMeanLengthList;
	fsR["wallsList"] >> roomWallsList;
	fsR.release();

	FileStorage fsB("/home/andy/git/fabmap2Test/InferenceLibrary/buildingLibrary.yml", FileStorage::READ);
	fsB["names"] >> buildingNameList;
	fsB["histogramList"] >> buildingHistogramList;
	fsB["sequenceList"] >> buildingSequenceList;
	fsB["pointList"] >> buildingPointList;
	fsB["centerList"] >> buildingCenterList;
	fsB["meanLength"] >> buildingMeanLengthList;
	fsB.release();
	*/

	minMatchStrength = 100;
	minContourToInfer = 100;
}

void Inference::resetInference(Costmap &costmap){
	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point a(i,j);
			if(costmap.cells.at<short>(a) != costmap.obsFree && costmap.cells.at<short>(a) != costmap.obsWall && costmap.cells.at<short>(a) != costmap.unknown){
				costmap.cells.at<short>(a) = costmap.unknown;
			}
		}
	}
}

void Inference::makeInference(string method, Costmap &costmap){
	if(method.compare("naive") == 0){
		makeNaiveInference( costmap );
	}
	else if(method.compare("geometric") == 0){
		resetInference( costmap );
		makeGeometricInference( costmap );
		makeNaiveInference( costmap );
	}
	else if(method.compare("visual") == 0){
		//resetInference( costmap );
		//makeVisualInference( costmap );
		makeNaiveInference( costmap );

	}
}

Inference::~Inference(){}

void Inference::makeNaiveInference( Costmap &costmap){
	// essentially findFrontiers and mark as inferred free
	for(int i=1; i<costmap.cells.cols-1; i++){
		for(int j=1; j<costmap.cells.rows-1; j++){
			bool newFrnt = false;
			if(costmap.cells.at<short>(j,i) != costmap.obsFree && costmap.cells.at<short>(j,i) != costmap.obsWall && costmap.cells.at<short>(j,i) != costmap.domFree){ // i'm unobserved
				if(costmap.cells.at<short>(j+1,i) == costmap.obsFree){ //  but one of my nbrs is observed
					newFrnt = true;
				}
				else if(costmap.cells.at<short>(j-1,i) == costmap.obsFree){ //  but one of my nbrs is observed
					newFrnt = true;
				}
				else if(costmap.cells.at<short>(j,i+1) == costmap.obsFree){ //  but one of my nbrs is observed
					newFrnt = true;
				}
				else if(costmap.cells.at<short>(j,i-1) == costmap.obsFree){ //  but one of my nbrs is observed
					newFrnt = true;
				}
			}
			if(newFrnt){
				costmap.cells.at<short>(j,i) = costmap.infFree;
			}
		}
	}
}

void Inference::init(float obsRadius){
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

void Inference::makeVisualInference(Costmap &costmap){

	resetInference( costmap );
	Mat geometricInferenceMat;
	geometricInference( costmap, geometricInferenceMat );

	
	//namedWindow("geometricInferenceMat", WINDOW_NORMAL);
	//imshow("geometricInferenceMat", geometricInferenceMat);
	//waitKey(1);
	

	Mat visualInferenceMat;
	visualInference( costmap, visualInferenceMat);

	//namedWindow("visInferenceMat", WINDOW_NORMAL);
	//imshow("visInferenceMat", visualInferenceMat);
	//waitKey(1);
	
	mergeVisualInferenceProbability( costmap, visualInferenceMat, geometricInferenceMat );

	//namedWindow("visInfMat", WINDOW_NORMAL);
	//imshow("visInfMat", visInfMat);
	//waitKey(1);

	//namedWindow("geoInfMat", WINDOW_NORMAL);
	//imshow("geoInfMat", geoInfMat);
	//waitKey(1);
	

	displayVisInfMat(costmap);

	inflateWalls(costmap, 3);
	inflateFree( costmap, 1);
	removeInaccessibleContoursFromCostmap(costmap);



	costmap.buildCellsPlot();
	namedWindow("costmap.displayPlot", WINDOW_NORMAL);
	imshow("costmap.displayPlot", costmap.displayPlot);
	waitKey(1);
}

void Inference::mergeVisualInferenceProbability( Costmap &costmap, Mat &visualInferenceMat, Mat &geometricInferenceMat ){

	if(visInfMat.empty() ){
		visInfMat = Mat::ones(costmap.cells.size(), CV_32FC1) * 0.5;
		geoInfMat = Mat::ones(costmap.cells.size(), CV_32FC1) * 0.5;
		for(int i=0; i<costmap.cells.cols; i++){
			for(int j=0; j<costmap.cells.rows; j++){
				Point t(i,j);
				if( costmap.cells.at<short>(t) == costmap.obsFree){
					visInfMat.at<float>(t) = 0;
					geoInfMat.at<float>(t) = 0;
				}
				else if( costmap.cells.at<short>(t) == costmap.obsWall){
					visInfMat.at<float>(t) = 1;
					geoInfMat.at<float>(t) = 1;
				}
			}
		}
	}


	for(int i=0; i<visualInferenceMat.cols; i++){
		for(int j=0; j<visualInferenceMat.rows; j++){
			Point t(i,j);

			if( costmap.cells.at<short>(t) == costmap.obsFree){
				visInfMat.at<float>(t) = 0;
				geoInfMat.at<float>(t) = 0;
				continue;
			}
			else if( costmap.cells.at<short>(t) == costmap.obsWall){
				visInfMat.at<float>(t) = 1;
				geoInfMat.at<float>(t) = 1;
				continue;
			}

			float pWall = 0.75;
			float pFree = 0.25;

			bool flag = true;
			if( visualInferenceMat.at<Vec3b>(t) == costmap.cInfFree){
				visInfMat.at<float>(t) = visInfMat.at<float>(t)*pFree / (visInfMat.at<float>(t)*pFree + (1-visInfMat.at<float>(t))*(1-pFree) );
				flag = false;
			}
			else if( visualInferenceMat.at<Vec3b>(t) == costmap.cInfWall){
				visInfMat.at<float>(t) = visInfMat.at<float>(t)*pWall / (visInfMat.at<float>(t)*pWall + (1-visInfMat.at<float>(t))*(1-pWall) );
				flag = false;
			}

			if(flag){
				if(visInfMat.at<float>(t) > 0.51){
					visInfMat.at<float>(t) -= 0.0075;
				}
				else if(visInfMat.at<float>(t) < 0.49){
					visInfMat.at<float>(t) += 0.0075;
				}
			}

			flag = true;

			pWall = 0.70;
			pFree = 0.05;

			if( geometricInferenceMat.at<short>(t) == costmap.infFree){
				geoInfMat.at<float>(t) = geoInfMat.at<float>(t)*pFree / (geoInfMat.at<float>(t)*pFree + (1-geoInfMat.at<float>(t))*(1-pFree) );
				flag = false;
			}
			else if( geometricInferenceMat.at<short>(t) == costmap.infWall){
				geoInfMat.at<float>(t) = geoInfMat.at<float>(t)*pWall / (geoInfMat.at<float>(t)*pWall + (1-geoInfMat.at<float>(t))*(1-pWall) );
				flag = false;
			}
			else if( geometricInferenceMat.at<short>(t) == costmap.unknown ){
				//geoInfMat.at<float>(t) = geoInfMat.at<float>(t)*pWall / (geoInfMat.at<float>(t)*pWall + (1-geoInfMat.at<float>(t))*(1-pWall) );
				//flag = false;
			}

			if(flag){
				if(geoInfMat.at<float>(t) > 0.6){
					geoInfMat.at<float>(t) -= 0.1;
				}
				else if(geoInfMat.at<float>(t) < 0.4){
					geoInfMat.at<float>(t) += 0.1;
				}
			}
		}
	}

	for(int i=0; i<1; i++){
		medianBlur(visInfMat, visInfMat, 5);
	}
	Mat infMat = geoInfMat + visInfMat;


	resetInference( costmap );


	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point t(i,j);
			if( costmap.cells.at<short>(t) != costmap.obsFree && costmap.cells.at<short>(t) != costmap.obsWall){
				if( infMat.at<float>(t) > 1.25 ){
					costmap.cells.at<short>(t) = costmap.infWall;
				}
				else if( infMat.at<float>(t) < 0.8){
					costmap.cells.at<short>(t) = costmap.infFree;
				}
			}
		}
	}
}

void Inference::displayVisInfMat(Costmap &costmap){
	Mat temp= Mat::zeros(costmap.cells.size(),CV_8UC1);
	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point a(i,j);
			uchar shade;
			shade = round((1-visInfMat.at<float>(a))*255);
			temp.at<uchar>(a) = shade;
		}
	}

	
	//namedWindow("Prob Inf Grid", WINDOW_NORMAL);
	//imshow("Prob Inf Grid", temp);
	//waitKey(1);
	
}

void Inference::visualInference( Costmap &costmap, Mat &mergeMat){

	Graph graph;
	graph.createThinGraph(costmap, 1, 1);

	Mat blankCostmap = Mat::ones(costmap.cells.size(), CV_8UC3);
	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point t(i,j);
			if( costmap.cells.at<short>(t) == costmap.obsFree){
				blankCostmap.at<Vec3b>(t) = costmap.cObsFree;
			}
			else if( costmap.cells.at<short>(t) == costmap.obsWall){
				blankCostmap.at<Vec3b>(t) = costmap.cObsWall;
			}
			//else if( costmap.cells.at<short>(t) == costmap.infWall){
			//	blankCostmap.at<Vec3b>(t) = costmap.cInfWall;
			//}
			//else if( costmap.cells.at<short>(t) == costmap.infFree){
			//	blankCostmap.at<Vec3b>(t) = costmap.cInfFree;
			//}
			
			else{
				blankCostmap.at<Vec3b>(t) = costmap.cUnknown;
			}
		}
	}
	
	//namedWindow("costmat", WINDOW_NORMAL);
	//imshow("costmat", blankCostmap);
	//waitKey(1);
	
	mergeMat = blankCostmap.clone();
	int nInfIters = 20;
	vector<Pose> candidateMatches;
	vector<Point2f> candidateLocs;
	for(int infIters = 0; infIters < nInfIters; infIters++){

		// only select points that are observed free as candidate library pose
		int ns;
		while(true){
			ns = int(rand() % graph.nodeLocations.size());  // select random pts
			if(costmap.cells.at<short>(graph.nodeLocations[ns]) == costmap.obsFree){
				break;
			}
		}

		Pose obsPose(graph.nodeLocations[ns], costmap); // make it into a pose
		if( obsPose.needInference){ // does it need to be inferred over?


			//blankCostmap.at<Vec3b>(obsPose.loc) = Vec3b(255,0,0);
			//namedWindow("costmat", WINDOW_NORMAL);
			//imshow("costmat", blankCostmap);
			//waitKey(1);

			//Mat obsMat = obsPose.makeMat();
			//namedWindow("obsMat", WINDOW_NORMAL);
			//imshow("obsMat", obsMat);
			//waitKey(0);
	
			float matchCost;
			float minCost = INFINITY;

			int libIndex = getVisualLibraryIndex( obsPose );

			Pose bestLibMatch = visualLibrary[libIndex][0];
			for(size_t vl=0; vl<visualLibrary[libIndex].size(); vl++){
				int orient = 0;
				matchCost = calcVisualFit( obsPose, visualLibrary[libIndex][vl], costmap, orient, minCost );
				visualLibrary[libIndex][vl].reward = matchCost;
				visualLibrary[libIndex][vl].orient = orient;

				//cout << "matchCost / rotate: " << matchCost << " / " << float(orient*360) / float(obsPose.nSamples) << endl;
				//Mat libMat = visualLibrary[libIndex][vl].makeMat();
				//namedWindow("libMat", WINDOW_NORMAL);
				//imshow("libMat", libMat);
				//waitKey(0);
				

				if( matchCost < minCost ){
					minCost = matchCost;
					bestLibMatch = visualLibrary[libIndex][vl];
				}
			}

			if( matchCost < 30){
				candidateMatches.push_back( bestLibMatch );
				candidateLocs.push_back( obsPose.loc );
			}
		}

		trimCandidatePoses( candidateMatches, candidateLocs );

		for(size_t i=0; i<candidateMatches.size(); i++){
			candidateMatches[i].rotateLimits();
			candidateMatches[i].insertPoseInCostmap(costmap, candidateLocs[i], mergeMat);
			mergeMat.at<Vec3b>(candidateLocs[i]) = Vec3b(0,0,255);
		}

		//namedWindow("mergeMat", WINDOW_NORMAL);
		//imshow("mergeMat", mergeMat);
		//waitKey(1);
		
	}
}

int Inference::getVisualLibraryIndex( Pose &pose ){

	double minDist = INFINITY;
	int minDex = -1;
	for(int i=0; i<libraryCenters.size(); i++){
		double dist = 0;
		for(int j=0; j<libraryCenters[i].obsLen.size(); j++){
			dist += pow(pose.obsLen[j] - libraryCenters[i].obsLen[j],2);
		}

		if( dist < minDist ){
			minDist = dist;
			minDex = i;
		}
	}
	return minDex;
}

void Inference::trimCandidatePoses( vector<Pose> &candidatePoses, vector<Point2f> &candidateLocs){
	// remove redundant inferences in the local area while keeping the best one
	int iterI = 0;
	while( iterI < candidatePoses.size()){
		int iterJ = iterI+1;
		bool keepI = true;
		while( iterJ < candidatePoses.size()){
			float d = euclidianDistance( candidateLocs[iterI], candidateLocs[iterJ] );

			if( d <= candidatePoses[0].radius/4){
				if( candidatePoses[iterI].reward < candidatePoses[iterJ].reward ){
					candidatePoses.erase(candidatePoses.begin() + iterJ);
					candidateLocs.erase(candidateLocs.begin() + iterJ);
					continue;
				}
				else{
					keepI = false;
					candidatePoses.erase(candidatePoses.begin() + iterI);
					candidateLocs.erase(candidateLocs.begin() + iterI);
					break;
				}
			}
			else{
				iterJ++;
			}
		}
		if( keepI ){
			iterI++;
		}
	}
}

float euclidianDistance( Point a, Point b){
	return sqrt( pow( a.x - b.x,2 ) + pow( a.y - b.y,2 ) );
}

float Inference::calcVisualFit(Pose &po, Pose &pl, Costmap &costmap, int &orient, float minCost){

	// for finding and identifying best match
	int offSetChecks = 4;

	for(size_t offset=0; offset<po.obsLen.size(); offset+=po.nSamples/offSetChecks){ // every length in po as a starting point
		float cost = 0;
		int libIter = 0;
		bool quitFlag = false;

		for(size_t obsIter=offset; obsIter<po.obsLen.size(); obsIter++){ // compare every length in po against every member in pl with starting point
			cost -= log2( visualReward( po, pl, obsIter, libIter, costmap) );
			libIter++; // if fit isnt good then track the number below a certain threshold and only include those with enough matches
			if(cost >= minCost){
				quitFlag = true;
				break;
			}
		}

		for(size_t obsIter=0; obsIter<offset; obsIter++){ // compare every length in po against every member in pl with starting point
			cost -= log2( visualReward( po, pl, obsIter, libIter, costmap) );
			libIter++;
			if(cost >= minCost){
				quitFlag = true;
				break;
			}
		}

		if(cost < minCost && !quitFlag){
			minCost = cost;
			orient = offset;
		}
	}

	// run the same thing but BACKWARDS for mirror image matching
	for(size_t offset=0; offset<po.obsLen.size(); offset+=po.nSamples/offSetChecks){ // every length in po as a starting point
		float cost = 0;
		int libIter = pl.obsLen.size()-1;
		bool quitFlag = false;

		for(size_t obsIter=offset; obsIter<po.obsLen.size(); obsIter++){  // compare every length in po against every member in pl with starting point
			cost -= log2( visualReward( po, pl, obsIter, libIter, costmap) );
			libIter++;
			if(cost >= minCost){
				quitFlag = true;
				break;
			}
		}

		for(size_t obsIter=0; obsIter<offset; obsIter++){ // compare every length in po against every member in pl with starting point
			cost -= log2( visualReward( po, pl, obsIter, libIter, costmap) );
			libIter++;
			if(cost >= minCost){
				quitFlag = true;
				break;
			}
		}

		if(cost < minCost && !quitFlag){
			minCost = cost;
			orient = offset;
		}
	}
	return minCost;
}

float Inference::visualReward(Pose &obs, Pose &lib, int obsI, int libI, Costmap &costmap){

	float l1 = obs.obsLen[obsI] + ( lib.obsLen[libI] - obs.obsLen[obsI]*lib.mean / obs.mean);
	float l2 = obs.obsLen[obsI] - ( lib.obsLen[libI] - obs.obsLen[obsI]*lib.mean / obs.mean);

	if(obs.obsVal[obsI] == costmap.obsWall || obs.obsVal[obsI] == costmap.obsFree){ // ends on a free space or wall space
		return 1 - abs( obs.getCDF( l1 ) - obs.getCDF( l2 ) );
	}
	else{ // costmap.cells.at<short>(obsPt) == costmap.infWall
		return 0.5 + 0.5*(1-abs( obs.getCDF( l1 ) - obs.getCDF( l2 ) ) );
	}
}

float absV( float num ){
	if( num >= 0 ){
		return num;
	}
	else{
		return num*-1;
	}
}

void printVector( vector<float> vec, string name ){
	cout << name << ": ";
	for(size_t i=0; i<vec.size(); i++){
		cout << vec[i];
		if( i+1 < vec.size() ){
			cout << ", ";
		}
	}
	cout << endl;
}

void Inference::drawHistogram(vector<float> histogram, char* title){
	Point base;
	Point top;

	int maxv = -1;
	for(size_t i =0; i<histogram.size(); i++){
		if(histogram[i] > maxv){
			maxv = histogram[i];
		}
	}

	base.y = 10;
	top.y = maxv + 20;

	Mat h = Mat::zeros(maxv + 20, histogram.size() + 20, CV_8UC1);

	base.x = 10;
	for(size_t i=0; i<histogram.size(); i++){
		base.x = 10 + i;
		top.x = base.x;
		top.y = 10 + round(histogram[i]);
		line(h, base, top, Scalar(255), 1, 8);
	}

	char buffer[150];
	sprintf(buffer, "drawHistogram::histogram::%s", title);


	namedWindow(buffer, WINDOW_NORMAL);
	imshow(buffer, h);
	waitKey(1);
}

void Inference::makeGeometricInference(Costmap &costmap){

	geometricInference( costmap );

	// inflate walls
	inflateWalls(costmap);

	// black out inaccessible areas
	removeInaccessibleContoursFromCostmap(costmap);

	// close dominated contours
	//closeDominatedContours( costmap, geoInferenceMat );


}

void Inference::geometricInference(Costmap &costmap){

	bool flag = false;

	if(geoInferenceMat.empty()){
		flag = true;
	}
	else{
		for(int i=0; i<costmap.cells.cols; i++){
			for(int j=0; j<costmap.cells.rows; j++){
				Point a(i,j);
				if(costmap.cells.at<short>(a) == costmap.obsFree && geoInferenceMat.at<short>(a) != costmap.obsFree){
					flag = true;
					break;
				}
				if(costmap.cells.at<short>(a) == costmap.obsWall && geoInferenceMat.at<short>(a) != costmap.obsWall){
					flag = true;
					break;
				}
			}
			if(flag){
				break;
			}
		}
	}
	if( flag ){
		// undoPrevious inference
		resetInference(costmap);
		// getOuterHull
		vector<Point> outerHullPoints;
		Mat outerHullMat = Mat::zeros(costmap.cells.size(), CV_8UC1);
		getOuterHull(costmap, outerHullMat, outerHullPoints);

		// get hull breaches
		costmap.hullBreaches = findHullBreaches(outerHullMat, costmap);

		// my observations are always right
		for(int i=0; i<costmap.cells.cols; i++){
			for(int j=0; j<costmap.cells.rows; j++){
				Point a(i,j);
				if(costmap.cells.at<short>(a) != costmap.obsFree && costmap.cells.at<short>(a) != costmap.obsWall){
					if(geoInferenceMat.at<uchar>(a) == costmap.infFree){
						costmap.cells.at<short>(a) = costmap.infFree;
					}
					else if(geoInferenceMat.at<uchar>(a) ==costmap.infWall){
						costmap.cells.at<short>(a) = costmap.infWall;
					}
				}
			}
		}
	}
}

void Inference::geometricInference(Costmap &costmap, Mat &geoInfMat){

	bool flag = false;

	if(geoInferenceMat.empty()){
		flag = true;
	}
	else{
		for(int i=0; i<costmap.cells.cols; i++){
			for(int j=0; j<costmap.cells.rows; j++){
				Point a(i,j);
				if(costmap.cells.at<short>(a) == costmap.obsFree && geoInferenceMat.at<short>(a) != costmap.obsFree){
					flag = true;
					break;
				}
				if(costmap.cells.at<short>(a) == costmap.obsWall && geoInferenceMat.at<short>(a) != costmap.obsWall){
					flag = true;
					break;
				}
			}
			if(flag){
				break;
			}
		}
	}
	if( flag ){
		// undoPrevious inference
		resetInference(costmap);
		// getOuterHull
		vector<Point> outerHullPoints;
		Mat outerHullMat = Mat::zeros(costmap.cells.size(), CV_8UC1);
		getOuterHull(costmap, outerHullMat, outerHullPoints);

		// get hull breaches
		costmap.hullBreaches = findHullBreaches(outerHullMat, costmap);

		// my observations are always right
		geoInfMat = Mat::ones(costmap.cells.size(), CV_16S)*costmap.unknown;
		for(int i=0; i<costmap.cells.cols; i++){
			for(int j=0; j<costmap.cells.rows; j++){
				Point a(i,j);
				if(costmap.cells.at<short>(a) == costmap.obsFree || costmap.cells.at<short>(a) == costmap.obsWall){
					if( costmap.cells.at<short>(a) == costmap.obsFree){
						geoInferenceMat.at<uchar>(a) = costmap.obsFree;
						geoInfMat.at<short>(a) = costmap.obsFree;
					}
					else{
						geoInferenceMat.at<uchar>(a) = costmap.obsWall;
						geoInfMat.at<short>(a) = costmap.obsWall;
					}
				}
				else{
					if(geoInferenceMat.at<uchar>(a) == costmap.infFree){
						costmap.cells.at<short>(a) = costmap.infFree;
						geoInfMat.at<short>(a) = costmap.infFree;
					}
					else if(geoInferenceMat.at<uchar>(a) ==costmap.infWall){
						costmap.cells.at<short>(a) = costmap.infWall;
						geoInfMat.at<short>(a) = costmap.infWall;
					}
				}
			}
		}
	}

}

void Inference::removeInaccessibleContoursFromCostmap(Costmap &costmap){
	Mat infFreeMat = Mat::zeros(costmap.cells.size(), CV_8UC1);
	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point a(i,j);
			if(costmap.cells.at<short>(a) == costmap.infFree){
				infFreeMat.at<uchar>(a) = 255;
			}
		}
	}

	vector<Vec4i> hierarchy;
	vector<vector<Point> > cont;
	findContours(infFreeMat,cont, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	for(size_t i=0; i<cont.size(); i++){
		Contour c(infFreeMat, cont[i]);
		c.getNbrsWithValue(costmap, costmap.obsFree);
		if(c.nbrs.size() == 0){
			c.fillContourInCostmap(costmap, costmap.infWall);
		}
	}
}

void Inference::closeDominatedContours( Costmap &costmap){

	Mat infFreeMat = Mat::zeros( costmap.cells.size(), CV_8UC1 );
	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point a(i,j);
			if( costmap.cells.at<short>(a) == costmap.infFree){
				infFreeMat.at<uchar>(a) = 255;
			}
		}
	}

	vector<Vec4i> hierarchy;
	vector<vector<Point> > cont;
	findContours(infFreeMat,cont, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	for(size_t i=0; i<cont.size(); i++){
		Contour a(infFreeMat, cont[i]);
		a.getDominatedStatus(costmap);
		if(a.dominated){
			a.fillContourInCostmap(costmap, costmap.domFree);
		}
	}
}

vector<Point> Inference::findHullBreaches(Mat &outerHullMat, Costmap &costmap){

	vector<Point> breaches;

	vector<Vec4i> hierarchy;
	vector<vector<Point> > cont;
	findContours(outerHullMat,cont, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	Contour c(outerHullMat, cont[0]);
	c.getVals(costmap);
	vector<Point> exits = c.getContourExitPoints(costmap.obsFree);

	/*
	for(size_t i=0; i<exits.size(); i++){
		circle(outerHullMat, exits[i], 1, Scalar(127), -1, 8);
	}

	namedWindow("graph::geoInfer::OuterHull2", WINDOW_NORMAL);
	imshow("graph::geoInfer::OuterHull2", outerHullMat);
	waitKey(1);
	*/
	return exits;
}

void Inference::inflateFree(Costmap &costmap, int inflationSteps){

	int inflatedFree = 10;

	for(int k=0; k<inflationSteps; k++){ // this is number of iterations
		for(int i=1; i<costmap.cells.cols-1; i++){ // check every cell
			for(int j=1; j<costmap.cells.rows-1; j++){ // for every cell

				if(costmap.cells.at<short>(j,i) ==  costmap.unknown){ //  inferred free cell

					bool wallFlag = false;
					if(costmap.cells.at<short>(j+1,i) == costmap.obsFree || costmap.cells.at<short>(j+1,i) == costmap.infFree){
						wallFlag = true;
					}
					else if(costmap.cells.at<short>(j-1,i) == costmap.obsFree || costmap.cells.at<short>(j-1,i) == costmap.infFree){
						wallFlag = true;
					}
					else if(costmap.cells.at<short>(j,i+1) == costmap.obsFree || costmap.cells.at<short>(j,i+1) == costmap.infFree){
						wallFlag = true;
					}
					else if(costmap.cells.at<short>(j,i-1) == costmap.obsFree || costmap.cells.at<short>(j,i-i) == costmap.infFree){
						wallFlag = true;
					}

					if( wallFlag){
						costmap.cells.at<short>(j,i) = inflatedFree;
					}
				}
			}
		}
		for(int i=1; i<costmap.cells.cols-1; i++){ // check every cell
			for(int j=1; j<costmap.cells.rows-1; j++){ // for every cell
				Point a(i,j);
				if(costmap.cells.at<short>(a) == inflatedFree){
					costmap.cells.at<short>(a) = costmap.infFree;
				}
			}
		}
	}

}

void Inference::inflateWalls(Costmap &costmap, int inflationSteps){

	for(int k=0; k<inflationSteps; k++){ // this is number of iterations
		for(int i=1; i<costmap.cells.cols-1; i++){ // check every cell
			for(int j=1; j<costmap.cells.rows-1; j++){ // for every cell

				if(costmap.cells.at<short>(j,i) ==  costmap.infFree){ //  inferred free cell

					if(costmap.cells.at<short>(j+1,i) == costmap.obsFree){
						continue;
					}
					else if(costmap.cells.at<short>(j-1,i) == costmap.obsFree){
						continue;
					}
					else if(costmap.cells.at<short>(j,i+1) == costmap.obsFree){
						continue;
					}
					else if(costmap.cells.at<short>(j,i-1) == costmap.obsFree){
						continue;
					}
					else{
						bool wallFlag = false;
						if(costmap.cells.at<short>(j+1,i) == costmap.obsWall || costmap.cells.at<short>(j+1,i) == costmap.infWall){
							wallFlag = true;
						}
						else if(costmap.cells.at<short>(j-1,i) == costmap.obsWall || costmap.cells.at<short>(j-1,i) == costmap.infWall){
							wallFlag = true;
						}
						else if(costmap.cells.at<short>(j,i+1) == costmap.obsWall || costmap.cells.at<short>(j,i+1) == costmap.infWall){
							wallFlag = true;
						}
						else if(costmap.cells.at<short>(j,i-1) == costmap.obsWall || costmap.cells.at<short>(j,i-i) == costmap.infWall){
							wallFlag = true;
						}
						if( wallFlag){
							costmap.cells.at<short>(j,i) = costmap.inflatedWall;
						}
					}
				}
			}
		}
		for(int i=1; i<costmap.cells.cols-1; i++){ // check every cell
			for(int j=1; j<costmap.cells.rows-1; j++){ // for every cell
				Point a(i,j);
				if(costmap.cells.at<short>(a) == costmap.inflatedWall){
					costmap.cells.at<short>(a) = costmap.infWall;
				}
			}
		}
	}
}

int Inference::getMatReward(Mat &in){
	// get Mat entropy
	float observed = 0;
	for(int i=0; i<in.rows; i++){
		for(int j=0; j<in.cols; j++){
			if(in.at<uchar>(i,j,0) > 0){
				observed++;
			}
		}
	}
	return observed;
}

void Inference::getMatWithValue(Costmap &costmap, Mat &mat, int value){
	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point a(i,j);
			if(costmap.cells.at<short>(a) == value){ // not free space or walls
				mat.at<uchar>(a) = 255;
			}
		}
	}
}

void Inference::getOuterHull(Costmap &costmap, Mat &outerHullDrawing, vector<Point> &outerHull){

	vector<Point> obsPoints;
	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point a(i,j);
			if(costmap.cells.at<short>(a) == costmap.obsFree || costmap.cells.at<short>(a) == costmap.obsWall){ // not free space or walls
				obsPoints.push_back(a);
			}
		}
	}

	/*
	costmap.buildCellsPlot();
	Mat tcostDisplay = costmap.displayPlot.clone();
	namedWindow("obsSpace", WINDOW_NORMAL);
	imshow("obsSpace", tcostDisplay);
	waitKey(0);
	*/

	// get outerHull of observed space
    vector<Point> hullPts;
   	convexHull( Mat(obsPoints), hullPts, true, CV_CHAIN_APPROX_NONE);

    Mat hullMat = Mat::zeros(costmap.cells.size(), CV_8UC1);
    for(size_t i=0; i<hullPts.size()-1; i++){
    	line(hullMat, hullPts[i], hullPts[i+1], Scalar(255), 1,8);
    }
    line(hullMat, hullPts[0], hullPts[hullPts.size()-1], Scalar(255), 1,8);

    /*
	namedWindow("hullMat", WINDOW_NORMAL);
	imshow("hullMat", hullMat);
	waitKey(0);
	*/

    // bitwise and between hull andcostmap.obsWalls to get outer walls, extend to find intersection points
    Mat obsWallMat = Mat::zeros( costmap.cells.size(), CV_8UC1);
    getMatWithValue(costmap, obsWallMat, costmap.obsWall);

    // To include obsFree on perim
    Mat obsFreeMat = Mat::zeros( costmap.cells.size(), CV_8UC1);
    getMatWithValue(costmap, obsFreeMat, costmap.obsFree);
    bitwise_or(obsWallMat,obsFreeMat,obsWallMat);

    Mat hullWalls = Mat::zeros( costmap.cells.size(), CV_8UC1 );
	bitwise_and(hullMat, obsWallMat, hullWalls);

	//namedWindow("graph::getOuterHull::OuterWalls", WINDOW_NORMAL);
	//imshow("graph::getOuterHull::OuterWalls", hullWalls);
	//waitKey(1);

	// URGENT I changed this to max gap of 2 from 20
	vector<Vec4i> lines;
	HoughLinesP(hullWalls, lines, 1, CV_PI/180, 2, 3, 2);

	Mat wallLines = Mat::zeros(costmap.cells.size(), CV_8UC1);
    for( size_t i = 0; i < lines.size(); i++){
        line( wallLines, Point(lines[i][0], lines[i][1]),Point(lines[i][2], lines[i][3]), Scalar(255), 1, 8 );
    }

    //namedWindow("graph::getOuterHull::wallLines", WINDOW_NORMAL);
	//imshow("graph::getOuterHull::wallLines", wallLines);
	//waitKey(1);

	// find intersection points of hull walls and add to hullMat
    Mat intImage = Mat::zeros(outerHullDrawing.size(), CV_8UC1);
	for( size_t i = 0; i < lines.size(); i++){
		for(size_t j=i; j<lines.size(); j++){
			Point t = findIntersection(lines[i],lines[j]);
			if(t.x > 0 && t.x < wallLines.cols && t.y > 0 && t.y < wallLines.rows){
				hullMat.at<uchar>(t) = 255;
				hullPts.push_back(t);
				//circle(tcostDisplay, t, 3, Scalar(0,255,0), -1, 8);
			}
		}
    }

	/*
	costmap.buildCostmapPlot();
	namedWindow("obsSpace with hull pts", WINDOW_NORMAL);
	imshow("obsSpace with hull pts", tcostDisplay);
	waitKey(0);
	*/

	// create new hull with intersection points
	// add intersection points
    // get new convex hull with projected boundaries
    outerHull.clear();
   	convexHull( Mat(hullPts), outerHull, true, CV_CHAIN_APPROX_NONE);

    /// Draw contours + hull results
    for(int i=0; i<(int)outerHull.size()-1; i++){
    	line(outerHullDrawing, outerHull[i], outerHull[i+1], Scalar(255), 1,8);
    }
    line(outerHullDrawing, outerHull[0], outerHull[outerHull.size()-1], Scalar(255), 1,8);

    // Convex Hull implementation
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Find contours
    threshold( outerHullDrawing, threshold_output, 200, 255, THRESH_BINARY );
    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    // Find the convex hull object for each contour
    vector<vector<Point> >hull( contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    {  convexHull( Mat(contours[i]), hull[i], false ); }

    // Draw contours + hull results
    outerHullDrawing = Mat::zeros( threshold_output.size(), CV_8UC1 );
    for( size_t i = 0; i< contours.size(); i++ ){
     drawContours( outerHullDrawing, contours, i, Scalar(255), -1, 8, vector<Vec4i>(), 0, Point() );
     drawContours( outerHullDrawing, hull, i, Scalar(255), -1, 8, vector<Vec4i>(), 0, Point() );
    }

    // add to
	// initialize the final mat
	geoInferenceMat = Mat::ones(costmap.cells.size(), CV_8UC1)*costmap.unknown; // set all cells as unknown
    for( size_t i = 0; i< contours.size(); i++ ){
    	drawContours( geoInferenceMat, contours, i, Scalar(costmap.infFree), -1, 8, vector<Vec4i>(), 0, Point() );
    	drawContours( geoInferenceMat, hull, i, Scalar(costmap.infFree), -1, 8, vector<Vec4i>(), 0, Point() );
    }
    for(int i=0; i<(int)outerHull.size()-1; i++){
    	line(geoInferenceMat, outerHull[i], outerHull[i+1], Scalar(costmap.infWall), 1,8);
    }
    line(geoInferenceMat, outerHull[0], outerHull[outerHull.size()-1], Scalar(costmap.infWall), 1,8);

    /*
    namedWindow("graph::getOuterHull::geoInf", WINDOW_NORMAL);
 	imshow("graph::getOuterHull::geoInf", geoInferenceMat);
 	waitKey(1);
 	*/

}

bool Inference::checkVisibility(Costmap &costmap, Point a, Point b){
	float dist = sqrt(pow(a.x-b.x,2) + pow(a.y-b.y,2));
	float unitVecX = (b.x - a.x) / dist; // get unit vector in right direction
	float unitVecY = (b.y - a.y) / dist;
	int steps = dist; // steps to check

	for(int m=1; m<steps; m++){ // check all intermediate points between two cells
		int aX = a.x + m*unitVecX;
		int aY = a.y + m*unitVecY;
		if(costmap.cells.at<uchar>(aX, aY) > 10){
			return false;
		}
	}
	return true;
}

Mat Inference::getObservedWallsOnContour( Costmap &costmap, BuildingTemplate &groundTruth){

	Mat obsExtHulls = Mat::zeros(costmap.cells.size(), CV_8UC1);

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			if(costmap.cells.at<uchar>(i,j) == costmap.obsWall && groundTruth.thinMap.at<uchar>(i,j) == 255){
				obsExtHulls.at<uchar>(i,j) = 255;
			}
		}
	}
	return obsExtHulls;
}

void Inference::placeBuildingInCostmap( Costmap &costmap, Mat &bestMap ){

	// add the inferred walls
	vector<vector<Point> > contour;
	vector<Vec4i> hier;
	findContours(bestMap, contour, hier, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
	Mat t = Mat::zeros(bestMap.size(), CV_8UC1);
	drawContours(t, contour, 0, Scalar(255), -1, 8);

	// fill in inferred free space
	for(int i=0; i<t.rows; i++){
		for(int j=0; j<t.cols; j++){
			if(t.at<uchar>(i,j) > 0){ // is the matched contour free here?
				if( costmap.cells.at<uchar>(i,j) == costmap.infWall || costmap.cells.at<uchar>(i,j) == costmap.inflatedWall || costmap.cells.at<uchar>(i,j) == costmap.unknown ){ // inferred wall
					costmap.cells.at<uchar>(i,j) = costmap.infFree;
				}
			}
		}
	}
	// add the inferred walls
	for(size_t j=0; j<contour.size(); j++){
		for(size_t i=0; i<contour[j].size(); i++){
			Point pc = contour[j][i];
			if( costmap.cells.at<uchar>(pc) == costmap.infFree || costmap.cells.at<uchar>(pc) == costmap.unknown ){
				costmap.cells.at<uchar>(pc) = costmap.infWall;
			}
		}
	}
}

void Inference::geoInferenceRecycling( Costmap &costmap, Mat &structMat ){
	// if struct infers unknown or wall and geo says free, make free and include geo walls
	for(int i=0; i<structMat.cols; i++){
		for(int j=0; j<structMat.rows; j++){
			if( structMat.at<uchar>(i,j) == costmap.infWall || structMat.at<uchar>(i,j) == costmap.unknown){
				if(this->geoInferenceMat.at<uchar>(i,j) == costmap.infFree){
					structMat.at<uchar>(i,j) = costmap.infFree;
				}
				else if(this->geoInferenceMat.at<uchar>(i,j) == costmap.infWall){
					structMat.at<uchar>(i,j) = costmap.infWall;
				}
				else if(this->geoInferenceMat.at<uchar>(i,j) == costmap.inflatedWall){
					structMat.at<uchar>(i,j) = costmap.inflatedWall;
				}
			}
		}
	}
}

Mat Inference::getObservedMap( Costmap &costmap ){
	Mat te = Mat::zeros(costmap.cells.size(), CV_8UC1);

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			if( costmap.cells.at<uchar>(i,j) == costmap.obsFree || costmap.cells.at<uchar>(i,j) == costmap.obsWall){
				te.at<uchar>(i,j) = 255;
			}
		}
	}
	return te;
}

void Inference::UCBstructuralBagOfWordsBuildingInference(Costmap &costmap){

	/*
	namedWindow("geo inf in", WINDOW_NORMAL);
	imshow("geo inf in", mapToMat( costmap.cells ) );
	waitKey(1);
	*/

	// get buildingAlignment of actual building and initialize
	BuildingTemplate groundTruth;
	groundTruth.getBuildingContour( costmap ); // get the contour, includes thinMat, filledMat
	groundTruth.contour.getInvariantHistogram(); // histogram
	groundTruth.contour.getSequenceHistogram(); // lengths in order normalized to 100
	groundTruth.wallMap = getObservedWallsOnContour( costmap, groundTruth);
	groundTruth.obsMap = getObservedMap( costmap );
	groundTruth.contour.center = findContourCenter( groundTruth.fillMap );
	//groundTruth.getObservedContour( costmap );
	groundTruth.thinCnt = getMatReward( groundTruth.thinMap );
	groundTruth.fillCnt = getMatReward( groundTruth.fillMap );
	groundTruth.wallCnt = getMatReward( groundTruth.wallMap );
	groundTruth.obsCnt = getMatReward( groundTruth.obsMap );

	/*
	namedWindow("Inf::buildingInf::building contour in", WINDOW_NORMAL);
	imshow("Inf::buildingInf::building contour in", groundTruth.thinMap);
	waitKey(1);

	namedWindow("Inf::buildingInf::building mat in", WINDOW_NORMAL);
	imshow("Inf::buildingInf::building mat in", groundTruth.fillMap);
	waitKey(1);

	namedWindow("Inf::buildingInf::hullWalls", WINDOW_NORMAL);
	imshow("Inf::buildingInf::hullWalls", groundTruth.wallMap);
	waitKey(0);
	*/

	/*
	char title1[100];
	sprintf(title1, "buildingContour contour to match");
	groundTruth.contour.drawHistogram(groundTruth.contour.histogram, title1);
	*/

	int numMatches = 10;
	vector<int> matches = findMatchingBuildingHistograms(groundTruth.contour.histogram, numMatches);

	//vector<int> matches;
	//vector<int> alignment;
	//findMatchesByAutoCorrelation( groundTruth.contour.sequence, matches, alignment, numMatches );

	int numRANSAC = 5; // initialize number of matches for UCB RANSAC

	if( buildingMatches.size() < numRANSAC){
		for(size_t i=buildingMatches.size(); i < numRANSAC; i++){
			BuildingTemplate a;
			a.reward = -INFINITY;
			buildingMatches.push_back(a);
		}
	}
	else{
		for( int i=0; i<numRANSAC; i++){

			/*
			cout << "reward: " << buildingMatches[i].bestReward << endl;
			cout << "rotate: " << buildingMatches[i].bestRotateAngle << endl;
			cout << "stretch: " << buildingMatches[i].bestStretch_x << ", " << buildingMatches[i].bestStretch_y << endl;
			cout << "shift: " << buildingMatches[i].bestShift_x << ", " << buildingMatches[i].bestShift_y << endl;
			*/
			buildingMatches[i].getTestMaps( costmap, groundTruth);
			buildingMatches[i].evaluateAlignment( groundTruth );

			/*
			cout << "reward: " << buildingMatches[i].bestReward << endl;
			namedWindow("buildingMatches.check", WINDOW_NORMAL);
			imshow("buildingMatches.check", buildingMatches[i].bestMap);
			waitKey(0);
			*/
		}
	}

	for(size_t j=0; j<matches.size(); j++){

		// pull buildingAlignment match from library
		BuildingTemplate a;
		a.contour.histogram = buildingHistogramList[ matches[j] ];
		a.contour.points = buildingPointList[ matches[j] ];
		a.contour.meanLength = buildingMeanLengthList[ matches[j] ];

		/*
		sprintf(title1, "contour from library");
		drawHistogram(buildingHistogramList[ matches[j] ], title1);
		waitKey(1);
		*/

		 // get size of mat to display on
		int maxS = 0;
		float mx = 0;
		float my = 0;
		for(size_t hi=0; hi<a.contour.points.size(); hi++){
			if(a.contour.points[hi].x > maxS){
				maxS = a.contour.points[hi].x;
			}
			if(a.contour.points[hi].y > maxS){
				maxS = a.contour.points[hi].y;
			}
			mx += a.contour.points[hi].x;
			my += a.contour.points[hi].y;
		}
		mx /= a.contour.points.size();
		my /= a.contour.points.size();

		Mat fMat = Mat::zeros(2*maxS, 2*maxS, CV_8UC1); // draw match
		Mat tMat = Mat::zeros(2*maxS, 2*maxS, CV_8UC1); // draw match

		// get offset to center the contour
		Point offset;
		offset.x = maxS - mx;
		offset.y = maxS - my;

		// draw the contour
		vector<vector<Point> > temp;
		temp.push_back(a.contour.points);
		drawContours(fMat, temp, 0, Scalar(255), -1, 8, noArray(), INT_MAX, offset);
		drawContours(tMat, temp, 0, Scalar(255), 1, 8, noArray(), INT_MAX, offset);

		// resize to match groundTruth
		double fxy = groundTruth.contour.meanLength / a.contour.meanLength;
		Size f0;
		resize(fMat,fMat,f0,fxy,fxy, INTER_AREA); // scale to match the partial segment
		threshold(fMat, fMat, 5, 255, THRESH_BINARY);

		resize(tMat,tMat,f0,fxy,fxy, INTER_AREA); // scale to match the partial segment
		threshold(tMat, tMat, 5, 255, THRESH_BINARY);

		// save to a
		a.fillMat = fMat;
		a.thinMat = tMat;
		a.contour.center = findContourCenter( a.fillMat );

		/*
		namedWindow("matched building contour resized", WINDOW_NORMAL);
		imshow("matched building contour resized", a.fillMat);
		waitKey(0);
		*/

		/*
		a.initialAlignmentCheck(groundTruth, alignment[j], costmap);
		for(size_t ie=0; ie<buildingMatches.size(); ie++){
			if(a.reward > buildingMatches[ie].reward){
				BuildingTemplate b = buildingMatches[ie];
				buildingMatches[ie] = a;
				a = b;
			}
		}
		*/



		// flipped building alignment match
		BuildingTemplate af;
		flip( a.fillMat, af.fillMat, 1 );
		flip( a.thinMat, af.thinMat, 1 );
		af.contour.center = findContourCenter( af.fillMat );


		//namedWindow("flipped matched building contour resized", WINDOW_NORMAL);
		//imshow("flipped matched building contour resized", af.fillMat);
		//waitKey(1);

		a.initialRotationCheckBisection( groundTruth, costmap);
		af.initialRotationCheckBisection( groundTruth, costmap);

		// find the best N rotation matches
		//a.initialRotationCheck( groundTruth, rotationChecks, costmap );
		//af.initialRotationCheck( groundTruth, rotationChecks, costmap);

		// check both the contour and flipped contour
		for(size_t ie=0; ie<buildingMatches.size(); ie++){
			if(a.reward > buildingMatches[ie].reward){
				BuildingTemplate b = buildingMatches[ie];
				buildingMatches[ie] = a;
				a = b;
			}
		}

		for(size_t ie=0; ie<buildingMatches.size(); ie++){
			if(af.reward > buildingMatches[ie].reward){
				BuildingTemplate b = buildingMatches[ie];
				buildingMatches[ie] = af;
				af = b;
			}
		}
	}

	// perform UCB RANSAC on rotated matches
	ucbBuildingRANSAC(costmap, groundTruth);

	// find the best match
	float maxReward = -INFINITY;
	int maxdex = -1;
	for(size_t i=0; i<buildingMatches.size(); i++){
		if(buildingMatches[i].bestReward > maxReward){
			maxdex = i;
			maxReward = buildingMatches[i].bestReward;
		}
	}

	/*
	namedWindow("buildingMatches[i].bestMat", WINDOW_NORMAL);
	imshow("buildingMatches[i].bestMap", buildingMatches[maxdex].bestMap);
	waitKey(0);
	*/

	// add the best match to the costmap
	resetInference( costmap );
	Mat bestM = buildingMatches[maxdex].bestMap.clone();
	placeBuildingInCostmap(costmap, bestM);
}

void Inference::ucbBuildingRANSAC(Costmap &costmap, BuildingTemplate &groundTruth){

	for(size_t i=0; i<buildingMatches.size(); i++){
		buildingMatches[i].resetUCB();
	}

	float temperature = 0.5;
	int nPulls = 500;
	for(float j=0; j<nPulls; j++){
		temperature *= 0.997;

		int c = 0;
		float best = 0;
		for(size_t i=0; i<buildingMatches.size(); i++){
			// ucb select which arm to pull
			float s;
			if(buildingMatches[i].pulls == 0){
				s = INFINITY;
			}
			else{
				s = buildingMatches[i].bestReward + sqrt( 2*log( j/buildingMatches[i].pulls ) );
			}
			if(s > best){
				best = s;
				c = i;
			}
		}
		buildingMatches[c].simulatedAnnealingIteration( costmap, groundTruth, temperature );
	}
}

void Inference::placeRoomInCostmap(Costmap& costmap, Mat &bestMap){

	for(int i=0; i<bestMap.rows; i++){
		for(int j=0; j<bestMap.cols; j++){
			if(bestMap.at<uchar>(j,i) == 255){ // structural inference says free
				if( costmap.cells.at<uchar>(i,j) == costmap.infWall || costmap.cells.at<uchar>(i,j) == costmap.inflatedWall ){ // if it is inferred free then update
					costmap.cells.at<uchar>(i,j) = costmap.infFree;
				}
			}
			else{ // to account for walls


			}
		}
	}
}

void Inference::UCBstructuralBagOfWordsRoomInference(Costmap &costmap, vector<Contour> &rooms){

	// get buildingAlignment of actual rooms and initialize
	if( roomMatches.size() > 0){
		 roomMatches.clear();
	}

	for(size_t i=0; i<rooms.size(); i++){
		if( rooms[i].points.size() > 0){
			RoomTemplate groundTruth;
			groundTruth.getGroundTruthRoomMaps( costmap, rooms[i] ); // get the contour, includes thinMat, filledMat
			groundTruth.contour.getInvariantHistogram(); // histogram
			groundTruth.contour.center = findContourCenter( groundTruth.fillMap );
			groundTruth.thinCnt = getMatReward( groundTruth.thinMap );
			groundTruth.fillCnt = getMatReward( groundTruth.fillMap );
			groundTruth.wallCnt = getMatReward( groundTruth.wallMap );
			groundTruth.obsCnt = getMatReward( groundTruth.obsMap );

			/*
			namedWindow("Inf::roomInf::groundTruth.wallMap", WINDOW_NORMAL);
			imshow("Inf::roomInf::groundTruth.wallMap", groundTruth.wallMap);
			waitKey(1);

			namedWindow("Inf::roomInf::groundTruth.thinMap", WINDOW_NORMAL);
			imshow("Inf::roomInf::groundTruth.thinMap", groundTruth.thinMap);
			waitKey(1);

			namedWindow("Inf::roomInf::groundTruth.fillMap", WINDOW_NORMAL);
			imshow("Inf::roomInf::groundTruth.fillMap", groundTruth.fillMap);
			waitKey(0);
			*/

			size_t numRANSAC = 5; // initialize number of matches for UCB RANSAC
			for(size_t j=0; j < numRANSAC; j++){
				RoomTemplate a;
				a.reward = -INFINITY;
				roomMatches.push_back(a);
			}

			int numMatches = 5;
			vector<int> matches = findMatchingRoomHistograms(groundTruth.contour.histogram, numMatches);

			for(size_t j=0; j<matches.size(); j++){

				// pull roomAlignment match from library
				RoomTemplate a;
				a.contour.points = roomPointList[ matches[j] ];
				a.contour.meanLength = roomMeanLengthList[ matches[j] ];
				a.wallList = this->roomWallsList[ matches[j] ];

				/*
				a.contour.histogram = roomHistogramList[ matches[j] ];
				sprintf(title1, "contour from library");
				drawHistogram(buildingHistogramList[ matches[j] ], title1);
				waitKey(1);
				*/

				 // get size of mat to display on
				int maxS = 0;
				float mx = 0;
				float my = 0;
				for(size_t hi=0; hi<a.contour.points.size(); hi++){
					if(a.contour.points[hi].x > maxS){
						maxS = a.contour.points[hi].x;
					}
					if(a.contour.points[hi].y > maxS){
						maxS = a.contour.points[hi].y;
					}
					mx += a.contour.points[hi].x;
					my += a.contour.points[hi].y;
				}

				mx /= a.contour.points.size();
				my /= a.contour.points.size();

				Mat fMat = Mat::zeros(2*maxS, 2*maxS, CV_8UC1); // draw match
				//Mat tMat = Mat::zeros(2*maxS, 2*maxS, CV_8UC1); // draw match

				// get offset to center the contour
				Point offset;
				offset.x = maxS - mx;
				offset.y = maxS - my;

				// draw the contour
				vector<vector<Point> > temp;
				temp.push_back(a.contour.points);
				drawContours(fMat, temp, 0, Scalar(255), -1, 8, noArray(), INT_MAX, offset);
				a.getWallMat(offset, maxS);

				// resize to match groundTruth
				double fxy = groundTruth.contour.meanLength / a.contour.meanLength;
				Size f0;
				resize(fMat,fMat,f0,fxy,fxy, INTER_AREA); // scale to match the partial segment
				threshold(fMat, fMat, 5, 255, THRESH_BINARY);

				resize(a.wallMat,a.wallMat,f0,fxy,fxy, INTER_AREA); // scale to match the partial segment
				threshold(a.wallMat, a.wallMat, 5, 255, THRESH_BINARY);

				// save to a
				a.fillMat = fMat;
				a.contour.center = findContourCenter( a.fillMat );

				namedWindow("ground truth room contour", WINDOW_NORMAL);
				imshow("ground truth room contour", groundTruth.fillMap);
				waitKey(1);

				namedWindow("matched room contour resized", WINDOW_NORMAL);
				imshow("matched room contour resized", a.fillMat);
				waitKey(1);


				namedWindow("matched room wallMat", WINDOW_NORMAL);
				imshow("matched room wallMat", a.wallMat);
				waitKey(1);

				// flipped building alignment match
				RoomTemplate af;
				flip( a.fillMat, af.fillMat, 1 );
				flip( a.wallMat, af.wallMat, 1 );
				af.contour.center = findContourCenter( af.fillMat );

				/*
				namedWindow("flipped matched building contour resized", WINDOW_NORMAL);
				imshow("flipped matched building contour resized", af.fillMat);
				waitKey(1);
				*/

				a.initialRotationCheckBisection( groundTruth, costmap);
				af.initialRotationCheckBisection( groundTruth, costmap);

				// find the best N rotation matches
				//a.initialRotationCheck( groundTruth,18, costmap );
				//af.initialRotationCheck( groundTruth, 18, costmap );

				// check both the contour and flipped contour
				for(size_t ie=0; ie<roomMatches.size(); ie++){
					if(a.reward > roomMatches[ie].reward){
						RoomTemplate b = roomMatches[ie];
						roomMatches[ie] = a;
						a = b;
					}
				}

				for(size_t ie=0; ie<roomMatches.size(); ie++){
					if(af.reward > roomMatches[ie].reward){
						RoomTemplate b = roomMatches[ie];
						roomMatches[ie] = af;
						af = b;
					}
				}
			}

			// perform UCB RANSAC on rotated matches
			ucbRoomRANSAC(costmap, groundTruth);

			// find the best match
			float maxReward = -INFINITY;
			int maxdex = -1;
			for(size_t i=0; i<roomMatches.size(); i++){
				if(roomMatches[i].bestReward > maxReward){
					maxdex = i;
					maxReward = roomMatches[i].bestReward;
				}
			}


			namedWindow("roomMatches[i].bestMap", WINDOW_NORMAL);
			imshow("roomMatches[i].bestMap", roomMatches[maxdex].bestMap);
			waitKey(1);


			// add the best match to the costmap
			Mat bestM = roomMatches[maxdex].bestMap.clone();
			placeRoomInCostmap(costmap, bestM);
		}
	}

}

void Inference::ucbRoomRANSAC(Costmap &costmap, RoomTemplate &groundTruth){

	for(size_t i=0; i<roomMatches.size(); i++){
		roomMatches[i].resetUCB();
	}

	float temperature = 0.5;
	int nPulls = 500;
	for(float j=0; j<nPulls; j++){
		temperature *= 0.997;

		int c = 0;
		float best = 0;
		for(size_t i=0; i<roomMatches.size(); i++){
			// ucb select which arm to pull
			float s;
			if( roomMatches[i].pulls == 0){
				s = INFINITY;
			}
			else{
				s = roomMatches[i].bestReward + sqrt( 2* log( j/roomMatches[i].pulls ) );
			}
			if(s > best){
				best = s;
				c = i;
			}
		}
		roomMatches[c].simulatedAnnealingIteration( costmap, groundTruth, temperature );
	}
}

void Inference::simulateObservation(Point pose, Mat &resultingView, Costmap &costmap){
	// make perimeter of viewing circle fit on image

	for(size_t i=0; i<viewPerim.size(); i++){
		Point v(viewPerim[i].x + pose.x, viewPerim[i].y + pose.y);

		//circle(ta, v, 1, Scalar(255), -1, 8);

		LineIterator it(costmap.cells, pose, v, 4, false);

		for(int i=0; i<it.count; i++, ++it){

			Point pp  = it.pos();
			//circle(ta, pp, 1, Scalar(255), -1, 8);

			if(!pointOnMat(pp, costmap.cells) || costmap.cells.at<uchar>(pp) > costmap.infFree){
				break;
			}
			else if(costmap.cells.at<uchar>(pp) == costmap.obsFree){
				resultingView.at<uchar>(pp) = 255;
			}
		}
	}
}

Point Inference::findContourCenter( Mat &mat ){

	float xSum = 0;
	float ySum = 0;
	float iter = 0;

	for(int i=0; i<mat.rows; i++){
		for(int j=0; j<mat.cols; j++){
			if( mat.at<uchar>(i,j) == 255){
				xSum += j;
				ySum += i;
				iter += 1;
			}
		}
	}

	int x = round( xSum / iter );
	int y = round( ySum / iter );

	Point c(x,y);
	return c;
}



vector<int> Inference::findMatchingBuildingHistograms(vector<float> &histogram, int nMatches){

	// compare to histogram library
	vector<float> cost(nMatches, INFINITY);
	vector<int> mindex(nMatches, -1);

	for(size_t hi=0; hi<buildingHistogramList.size(); hi++){
		int i = hi;
		float dist = compareHistogramByAbsDist( histogram, buildingHistogramList[hi] );
		//float dist = compareHistogramByInvCorrelation( contours[i].histogram, roomHistogramList[hi] );
		for(size_t j = 0; j<cost.size(); j++){
			if(dist < cost[j] && dist >= 0){

				float tC = cost[j];
				int tM = mindex[j];

				cost[j] = dist;
				mindex[j] = i;

				dist = tC;
				i = tM;
			}
		}
	}

	return mindex;
}

void Inference::findMatchesByAutoCorrelation(vector<float> &sequence, vector<int> &matches, vector<int> &alignment, int nMatches){

	// compare to histogram library
	vector<float> cost(nMatches, INFINITY);
	matches.clear();
	alignment.clear();
	for(int i=0; i<nMatches; i++){
		matches.push_back(-1);
		alignment.push_back(-1);
	}

	for(size_t hi=0; hi<buildingSequenceList.size(); hi++){
		int i = hi;
		vector<float> dist = compareSequenceByAutoCorrelation( sequence, buildingSequenceList[hi] );
		for(size_t j = 0; j<cost.size(); j++){
			if(dist[0] < cost[j] && dist[0] >= 0){

				float tC = cost[j];
				float tA = alignment[j];
				int tM = matches[j];

				cost[j] = dist[0];
				alignment[j] = dist[1];
				matches[j] = i;

				dist[0] = tC;
				dist[1] = tA;
				i = tM;
			}
		}
	}
}

vector<float> Inference::compareSequenceByAutoCorrelation( vector<float> &hist1, vector<float> &hist2){

	vector<float> bI(2, INFINITY);

	for(size_t i=0; i<hist1.size(); i++){
		// shift sequence by i
		vector<float> temp = shiftHistogram(hist1, i);
		float tDist = compareHistogramByAbsDist( temp, hist2);
		if( tDist < bI[0]){
			bI[0] = tDist;
			bI[1] = i;
		}
	}
	return bI;
}

vector<float> Inference::shiftHistogram( vector<float> hist, int skip){

	for(size_t i=0; i<hist.size()-skip; i++){
		float temp = hist[i];
		int ti = i+skip;
		if(ti >= int(hist.size() ) ){
			ti -= int(hist.size() );
		}
		else if(ti < 0){
			ti += int(hist.size() );
		}
		hist[i] = hist[ti];
		hist[ti] = temp;
	}
	return hist;
}


vector<int> Inference::findMatchingRoomHistograms(vector<float> &histogram, int nMatches){

	// compare to histogram library
	vector<float> cost(nMatches, INFINITY);
	vector<int> mindex(nMatches, -1);

	for(size_t hi=0; hi<roomHistogramList.size(); hi++){
		int i = hi;
		float dist = compareHistogramByAbsDist( histogram, roomHistogramList[hi] );
		//float dist = compareHistogramByInvCorrelation( contours[i].histogram, roomHistogramList[hi] );
		for(size_t j = 0; j<cost.size(); j++){
			if(dist < cost[j] && dist >= 0){

				float tC = cost[j];
				int tM = mindex[j];

				cost[j] = dist;
				mindex[j] = i;

				dist = tC;
				i = tM;
			}
		}
	}
	return mindex;
}

vector<int> Inference::getFrontierExits(vector<Point> &outerHull){
	vector<int> frontierExits;
	for(size_t i=0; i<frontiers.size(); i++){
		double t = pointPolygonTest(outerHull,frontiers[i].projection, false); // +1 means inside
		if(t <= 0){
			frontierExits.push_back(i);
		}
	}
	return frontierExits;
}


void Inference::displayInferenceMat(Costmap &costmap, Mat &outerHullDrawing, Mat &obstaclesAndHull, vector<Point> &outerHull, vector<int> frontierExits){
	Mat inferDisplay = Mat::zeros(outerHullDrawing.size(), CV_8UC1);
	Mat Frontiers = Mat::zeros(outerHullDrawing.size(), CV_8UC1);//getFrontiersImage();
	for(size_t i=0; i<frontiers.size(); i++){
		circle(Frontiers, frontiers[i].projection, 1, Scalar(100), -1);
		circle(Frontiers, frontiers[i].center,    1, Scalar(200), -1);
		line(Frontiers, frontiers[i].projection, frontiers[i].center, Scalar(100), 1, 8);
	}

	vector<vector<Point> > contours2;
	vector<Vec4i> hierarchy;
	findContours(outerHullDrawing, contours2, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
	drawContours(Frontiers, contours2, 0, Scalar(127), 1, 8);

	Mat inferredMiniMap = Mat::zeros(outerHullDrawing.size(), CV_8UC1);

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point pa, pb;
			pa.x = i;
			pa.y = j;
			if(costmap.cells.at<uchar>(i,j) < 10){ // free space with inflation
				inferDisplay.at<uchar>(i,j) = 255;
				inferredMiniMap.at<uchar>(i,j,0) = 0;
			}
			else if(costmap.cells.at<uchar>(i,j) == costmap.obsWall){ // obstacle
				Scalar color;
				color[0] = 127;
				color[1] = 127;
				color[2] = 127;
				inferDisplay.at<uchar>(i,j) = 127;
				obstaclesAndHull.at<uchar>(i,j) = 255;
				inferredMiniMap.at<uchar>(i,j,0) = 255;
			}
			else if(costmap.cells.at<uchar>(i,j) == costmap.obsFree){ // unknown space
				inferDisplay.at<uchar>(i,j) = 0;
				inferredMiniMap.at<uchar>(i,j,0) = 100;
			}
			else if(costmap.cells.at<uchar>(i,j) == costmap.unknown){ // unknown space
				Scalar color;
				color[0] = 0;
				color[1] = 0;
				color[2] = 127;
				rectangle(inferDisplay,pa,pb,color,-1);
				inferredMiniMap.at<uchar>(i,j,0) = 100;
			}
		}
	}

	for(size_t i=0; i<frontiers.size(); i++){
		Scalar color;
		color[0] = 0;
		color[1] = 0;
		color[2] = 200;

		circle(inferDisplay,frontiers[i].projection,5,color,-1);

		int radius = round(sqrt(frontiers[i].reward) / 10);
		if(radius < 1){
			radius = 1;
		}
		circle(inferDisplay,  frontiers[i].center, radius, color,-1,8);
		color[2] = 255;
		line(inferDisplay,frontiers[i].projection, frontiers[i].center,color,5,8);
	}

	for(size_t i=0; i<frontierExits.size(); i++){
		Scalar color;
		color[0] = 0;
		color[1] = 255;
		color[2] = 0;

		int radius = frontiers[i].reward;
		if(radius < 1){
			radius = 1;
		}
		circle(inferDisplay,frontiers[frontierExits[i]].projection,radius,color,-1);
	}


	Scalar color;
	color [0] = 0;
	color[1] = 255;
	color[2] = 0;
	for(size_t i=0; i<outerHull.size()-1; i++){
		line(inferDisplay, outerHull[i], outerHull[i+1], color, 2, 8);
		line(obstaclesAndHull, outerHull[i], outerHull[i+1], Scalar(255), 2, 8);
	}
	line(inferDisplay, outerHull[0], outerHull[outerHull.size()-1],color, 2, 8);
	line(obstaclesAndHull, outerHull[0], outerHull[outerHull.size()-1], Scalar(255), 2, 8);

    hullPts.clear();
    for(size_t i=0; i<outerHull.size(); i++){
    		hullPts.push_back(outerHull[i]);
    }

    // for each Frontier
    /*
    for(size_t i=0; i<frontiers.size(); i++){
    	// find all obstacle points within distance r of each Frontier member
    	for(size_t j=0; j<frontiers[i].members.size(); j++){

    		for(int k = -2; k<3; k++){
    			for(int l=-2; l<3; l++){

    				if(costmap[frontiers[i].members[j][0] + k][frontiers[i].members[j][1] + l] > 255){
    					Scalar color;
						color[0] = 0;
						color[1] = 255;
						color[2] = 0;
    					vector<int> t;
    					t.push_back(frontiers[i].members[j][0] + k);
    					t.push_back(frontiers[i].members[j][1] + l);

    					Point pa;
    					pa.x = t[1];
    					pa.y = t[0];

    					frontiers[i].obstacles.push_back(t);
    					circle(inferDisplay,pa,2,color,-1);
    				}
    			}
    		}
    	}
    }

    imshow( "infer miniMap", inferredMiniMap );
    imshow( "infer display", inferDisplay );
    */
}

void Inference::getImagePoints(Mat &image, vector<Point> &list, int val){
	for(int i=0; i<image.rows; i++){
		for(int j=0; j<image.cols; j++){
			Scalar intensity = image.at<uchar>(i,j);
			if(intensity[0] == val){
				Point t;
				t.x = j;
				t.y = i;
				list.push_back(t);
				break;
			}
		}
	}
}

float Inference::compareHistogramByAbsDist(vector<float> &hist1, vector<float> &hist2){

	float dist = 0;
	for(size_t i=0; i< hist1.size(); i++){
		dist += abs(hist1[i] - hist2[i]);
	}
	return dist;
}

float Inference::compareHistogramByChiSquaredDist(vector<float> &hist1,vector<float> &hist2){

	float dist = 0;
	for(size_t i=0; i<hist1.size(); i++){
		dist += pow(hist1[i] - hist2[i],2) / hist2[i];
	}
	return dist;
}

float Inference::compareHistogramByInvCorrelation(vector<float> &hist1,vector<float> &hist2){

	float meanMHist = 0;
	for(size_t i=0; i<hist2.size(); i++){
		meanMHist += hist2[i];
	}
	meanMHist /= (float)hist2.size();

	float meanCHist = 0;
	for(size_t i=0; i<hist1.size(); i++){
		meanCHist += hist1[i];
	}
	meanCHist /= (float)hist1.size();

	float num = 0;
	float dcHist = 0;
	float dmHist = 0;
	for(size_t i=0; i<hist1.size(); i++){
		num += ((float)hist2[i] - meanMHist)*((float)hist1[i] - meanCHist);
		dcHist += pow(meanCHist - hist1[i],2);
		dmHist += pow(meanMHist - hist2[i],2);
	}

	float den = sqrt(dcHist*dmHist);

	float dist = den / num;

	return dist;
}

void drawHistogram(vector<float> histogram, char* title){
	Point base;
	Point top;

	int maxv = -1;
	for(size_t i =0; i<histogram.size(); i++){
		if(histogram[i] > maxv){
			maxv = histogram[i];
		}
	}

	base.y = 10;
	top.y = maxv + 20;

	Mat h = Mat::zeros(maxv + 20, histogram.size() + 20, CV_8UC1);

	base.x = 10;
	for(size_t i=0; i<histogram.size(); i++){
		base.x = 10 + i;
		top.x = base.x;
		top.y = 10 + round(histogram[i]);
		line(h, base, top, Scalar(255), 1, 8);
	}

	char buffer[50];
	sprintf(buffer, "drawHistogram::histogram::%s", title);


	namedWindow(buffer, WINDOW_NORMAL);
	imshow(buffer, h);
	waitKey(1);
}

void Inference::simulateObservation(Point pose, Mat &resultingView, Costmap &costmap, vector<float> &length, vector<Point> &endPts){
	// make perimeter of viewing circle fit on image


	length.clear();
	endPts.clear();
	for(size_t i=0; i<viewPerim.size(); i++){
		length.push_back(0);
		Point t(-1,-1);
		endPts.push_back(t);
	}

	for(size_t i=0; i<viewPerim.size(); i++){
		Point v(viewPerim[i].x + pose.x, viewPerim[i].y + pose.y);

		//circle(ta, v, 1, Scalar(255), -1, 8);

		LineIterator it(costmap.cells, pose, v, 4, false);

		for(int i=0; i<it.count; i++, ++it){

			Point pp  = it.pos();
			//circle(ta, pp, 1, Scalar(255), -1, 8);

			if(!pointOnMat(pp, costmap.cells) || costmap.cells.at<uchar>(pp) > costmap.infFree){
				break;
			}
			else if(costmap.cells.at<uchar>(pp) == costmap.obsFree){
				resultingView.at<uchar>(pp) = 255;
				length[i]++;
				endPts[i] = pp;
			}
		}
	}
}

vector<Point> Inference::findFrontiers(Costmap &costmap){
	vector<Point> frontiersList;
	for(int i=1; i<costmap.cells.cols-1; i++){
		for(int j=1; j<costmap.cells.rows-1; j++){
			bool newFrnt = false;
			if(costmap.cells.at<uchar>(i,j) == costmap.infFree){ // i'm unobserved
				if(costmap.cells.at<uchar>(i+1,j) == costmap.obsFree){ //  but one of my nbrs is observed
					newFrnt = true;
				}
				else if(costmap.cells.at<uchar>(i-1,j) == costmap.obsFree){ //  but one of my nbrs is observed
					newFrnt = true;
				}
				else if(costmap.cells.at<uchar>(i,j+1) == costmap.obsFree){ //  but one of my nbrs is observed
					newFrnt = true;
				}
				else if(costmap.cells.at<uchar>(i,j-1) == costmap.obsFree){ //  but one of my nbrs is observed
					newFrnt = true;
				}
			}
			if(newFrnt){
				Point fT(i,j);
				frontiersList.push_back(fT);
			}
		}
	}
	return frontiersList;
}

void Inference::clusterFrontiers(vector<Point>  frntList, Costmap &costmap){
	// check to see if frnt.center is still a Frontier cell, if so keep, else delete
	for(size_t i=0; i<this->frontiers.size(); i++){
		this->frontiers[i].editFlag = true;
		bool flag = true;
		for(size_t j=0; j<frntList.size(); j++){
			if(this->frontiers[i].center == frntList[j]){
				flag = false;
				frntList.erase(frntList.begin()+j);
			}
		}
		if(flag){
			this->frontiers.erase(this->frontiers.begin()+i);
		}
		else{
			this->frontiers[i].editFlag = false;
		}
	}
	// breadth first search through known clusters
	for(size_t i=0; i<this->frontiers.size(); i++){ // keep checking for new Frontier clusters while there are unclaimed Frontiers
		vector<Point> q; // current cluster
		vector<Point> qP; // open set in cluster
		qP.push_back(this->frontiers[i].center);

		while((int)qP.size() > 0){ // find all nbrs of those in q
			Point seed = qP[0];
			q.push_back(qP[0]);
			qP.erase(qP.begin(),qP.begin()+1);
			for(int ni = seed.x-2; ni<seed.x+3; ni++){
				for(int nj = seed.y-2; nj<seed.y+3; nj++){
					for(size_t i=0; i<frntList.size(); i++){
						if(frntList[i].x == ni && frntList[i].y == nj){
							qP.push_back(frntList[i]); // in range, add to open set
							frntList.erase(frntList.begin() + i);
						}
					}
				}
			}
		}
		this->frontiers[i].members = q; // save to list of clusters
	}

	// breadth first search
	while(frntList.size() > 0){ // keep checking for new Frontier clusters while there are unclaimed Frontiers
		vector<Point> q; // current cluster
		vector<Point> qP; // open set in cluster
		qP.push_back(frntList[0]);
		frntList.erase(frntList.begin());

		while((int)qP.size() > 0){ // find all nbrs of those in q
			Point seed = qP[0];
			q.push_back(qP[0]);
			qP.erase(qP.begin(),qP.begin()+1);
			for(int ni = seed.x-1; ni<seed.x+2; ni++){
				for(int nj = seed.y-1; nj<seed.y+2; nj++){
					for(int i=0; i<(int)frntList.size(); i++){
						if(frntList[i].x == ni && frntList[i].y == nj){
							qP.push_back(frntList[i]); // in range, add to open set
							frntList.erase(frntList.begin() + i, frntList.begin()+i+1);
						}
					}
				}
			}
		}
		Frontier a(q);
		this->frontiers.push_back(a);
	}
	for(size_t i=0; i<this->frontiers.size(); i++){ // number of clusters
		if(this->frontiers[i].editFlag){
			//frontiers[i].getCentroid(costmap);
			frontiers[i].getCenter();
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////
//////////////// Retired Functions ///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
/*

void Inference::inflateWalls(Costmap &costmap, Mat &costMat){
	for(int freeInfStep=0; freeInfStep<freeInflationDistance+1; freeInfStep++){ // check for all ranges of free inflation
		for(int i=freeInfStep; i<costMat.rows-freeInfStep; i++){ // check every cell
			for(int j=freeInfStep; j<costMat.cols-freeInfStep; j++){ // for every cell
				if(costMat.at<uchar>(i,j) ==  costmap.infFree){ //  inferred free
					if(freeInfStep > wallInflationDistance){
						if( checkForInflation( costMat, i, j, wallInflationDistance,costmap.obsWall) || checkForInflation( costMat, i, j, wallInflationDistance, costmap.infWall) ){ // is there a wall nearby
							if( !checkForInflation( costMat, i, j, freeInfStep, costmap.obsFree) ){ // is there NOT an obsFree nearby
								costMat.at<uchar>(i,j,0) = costmap.inflatedWall;
							}
						}
					}
					else{
						if( checkForInflation( costMat, i, j, freeInfStep,costmap.obsWall) || checkForInflation( costMat, i, j, freeInfStep, costmap.infWall) ){ // is there a wall nearby
							if( !checkForInflation( costMat, i, j, freeInfStep, costmap.obsFree) ){ // is there NOT an obsFree nearby
								costMat.at<uchar>(i,j,0) = costmap.inflatedWall;
							}
						}
					}
				}
			}
		}
	}
}

float Inference::evalPointsBitwiseAnd(Mat &gt, Mat &test){


	namedWindow("gt", WINDOW_NORMAL);
	imshow("gt", gt);
	waitKey(1);

	namedWindow("test", WINDOW_NORMAL);
	imshow("test", test);
	waitKey(1);


	Mat t = Mat::zeros(gt.size(), CV_8UC1);
	bitwise_not(test, t);


	namedWindow("t", WINDOW_NORMAL);
	imshow("t", t);
	waitKey(1);


	bitwise_and(gt, t, t);


	namedWindow("t'", WINDOW_NORMAL);
	imshow("t'", t);
	waitKey(1);


	float observed = 0;
	for(int i=0; i<t.rows; i++){
		for(int j=0; j<t.cols; j++){
			if(t.at<uchar>(i,j,0) > 0){
				observed++;
			}
		}
	}

	Mat t2 = Mat::zeros(gt.size(), CV_8UC1);
	bitwise_not(gt, t2);


	namedWindow("t2", WINDOW_NORMAL);
	imshow("t2", t2);
	waitKey(1);


	bitwise_and(test, t2, t2);


	namedWindow("t2'", WINDOW_NORMAL);
	imshow("t2'", t2);
	waitKey(1);


	float observed2 = 0;

	for(int i=0; i<t2.rows; i++){
		for(int j=0; j<t2.cols; j++){
			if(t2.at<uchar>(i,j,0) > 0){
				observed2++;
			}
		}
	}

	return observed + observed2;
}

float Inference::evalPointsBitwiseAndContours(Mat &gt, Mat &test){


	namedWindow("gt", WINDOW_NORMAL);
	imshow("gt", gt);
	waitKey(1);

	namedWindow("test", WINDOW_NORMAL);
	imshow("test", test);
	waitKey(1);


	Mat t = Mat::zeros(gt.size(), CV_8UC1);
	bitwise_not(test, t);


	namedWindow("t", WINDOW_NORMAL);
	imshow("t", t);
	waitKey(1);


	bitwise_and(gt, t, t);

	namedWindow("t'", WINDOW_NORMAL);
	imshow("t'", t);
	waitKey(1);

	float observed = 0;
	for(int i=0; i<t.rows; i++){
		for(int j=0; j<t.cols; j++){
			if(t.at<uchar>(i,j,0) > 0){
				observed++;
			}
		}
	}
	return observed;
}

void Inference::mergeInferenceContours(Costmap &costmap){

	for(size_t i=0; i<contours.size(); i++){
		for(size_t j=i+1; j<contours.size(); j++){
			if(checkVisibility(costmap, contours[i].center, contours[j].center)){
				Mat temp = Mat::zeros(contours[i].filledMat.size(), CV_8UC1);
				bitwise_or(contours[i].filledMat, contours[j].filledMat, temp);

				namedWindow("merge", WINDOW_NORMAL);
				imshow("merge", temp);
				waitKey(1);
			}
		}
	}
}

void Inference::initMyRANSAC(Costmap &costmap, Mat &matchMat, Point matchCenter, Contour &contour, float &theta, float &minError){

	float rotateAngle = 0;

	minError = INFINITY;
	for(int j=0; j<72; j++){
		rotateAngle = 5*j;

		Mat ransacMat;
		Mat ransacMap = Mat::zeros(costmap.cells.size(), costmap.cells[0].size(), CV_8UC1);

		// warp the matched contour
		Mat rotMatrix = getRotationMatrix2D(matchCenter, rotateAngle, 1);
		warpAffine(matchMat, ransacMat, rotMatrix, matchMat.size());

		threshold(ransacMat, ransacMat, 5, 255, THRESH_BINARY);

		// pull the warped contour
		vector<vector<Point> > ransacCont;
		vector<Vec4i> tHier;
		findContours(ransacMat, ransacCont, tHier, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

		// shift and place in the map
		int px = contour.center.y - ransacMat.cols / 2;
		int py = contour.center.x - ransacMat.rows / 2;
		drawContours(ransacMap, ransacCont, 0, Scalar(255), -1, 8, noArray(), INT_MAX, Point(py ,px) );


		namedWindow("initMyRansac::Map", WINDOW_NORMAL);
		imshow("initMyRansac::Map", contour.filledMat);

		namedWindow("initMyRansac::ransacMap", WINDOW_NORMAL);
		imshow("initMyRansac::ransacMap", ransacMap);
		waitKey(1);


		float error = evalPointsBitwiseAnd(contour.filledMat, ransacMap);


		cout << "initMyRansac::error: " << error << endl;
		cout << "initMyRansac::minError: " << minError << endl;
		waitKey(0);


		if( error < minError ){
			minError = error;
			theta = rotateAngle;
		}
	}
}

float Inference::evalPointsPoly(vector<Point> &gtCont, vector<Point> &testCont, float shift_x, float shift_y, int nTests){
	float error = 0;

	for(int i=0; i<nTests; i++){
		// select a random point on the contour of
		int tstI = rand() % testCont.size();
		testCont[tstI].x += shift_x;
		testCont[tstI].y += shift_y;

		error += abs( pointPolygonTest(gtCont, testCont[tstI], true) );
	}
	return error / float(nTests);
}


float Inference::evalBreadthSearch(Mat &gt, Mat &tst, int nTests){
	float error = 0;

	// get points in contour
	vector<Point> gtPts;
	getImagePoints(gt, gtPts, 255);

	for(int i=0; i<nTests; i++){
		// select a random point on the contour of
		int gtI = rand() % gtPts.size();

		error += breadthFirstSearchDist(gtPts[gtI], tst);
	}
	return error / float(nTests);
}

float Inference::breadthFirstSearchDist(Point in, Mat &mat){

	vector<Point> oSet;
	vector<Point> cSet;

	Point t = in;

	int dx[4] = {0,0,-1,1};
	int dy[4] = {-1,1,0,0};

	while(mat.at<uchar>(t.x, t.y) != 255){
		cSet.push_back(t);

		for(int i=0; i<4; i++){
			bool flag = true;
			Point a(t.x+dx[i], t.y+dy[i]);
			for(size_t j=0; j<cSet.size(); j++){
				if(cSet[j] == a){
					flag = false;
					break;
				}
			}
			if(flag){
				for(size_t j=0; j<oSet.size(); j++){
					if(oSet[j] == a){
						flag = false;
						break;
					}
				}
			}
			if(flag){
				oSet.push_back(a);
			}
		}

		t = oSet[0];
		oSet.erase( oSet.begin() );
	}
	return sqrt( pow(t.x-in.x,2) + pow(t.y-in.y,2) );
}


void Inference::myBuildingRANSAC(Costmap &costmap, Mat &matchMat, Mat &buildingHullWalls, Contour &contour, float &theta, Mat &bestMap, float cost){
	// perform rotation based ransac over the images
	// in degrees, Mat getRotationMatrix2D(Point2f center, double angle, double scale);

	float shift_x = 0;
	float shift_y = 0;
	float rotateAngle = theta;
	float stretch_x = 1;
	float stretch_y = 1;

	float workingShift_x = 0;
	float workingShift_y = 0;
	float workingRotateAngle = theta;
	float workingStretch_x = 1;
	float workingStretch_y = 1;

	float workingError = INFINITY;
	float bestError = INFINITY;

	Point matchCenter = findContourCenter(matchMat);
	contour.center = findContourCenter( contour.filledMat );

	float thinCnt = getMatReward( contour.thinMat );
	float filledCnt = getMatReward( contour.filledMat );
	float wallCnt = getMatReward( buildingHullWalls );

	minMatchStrength = -1;
	float temp = 0.5;
	for(int j=0; j<1000; j++){
		temp *= 0.995;

		int c = rand() % 5;
		if(c == 0){
		rotateAngle = workingRotateAngle + (rand() % 5) - 2;
		}
		else if(c==1){
			shift_x = workingShift_x + (rand() % 5) - 2;
		}
		else if(c==2){
			shift_y = workingShift_y + (rand() % 5) - 2;
		}
		else if(c==3){
			stretch_x = workingStretch_x + float(rand() % 101)/400 -0.125;
		}
		else if(c==4){
			stretch_y = workingStretch_y + float(rand() % 101)/400 -0.125;
		}


		rotateAngle = workingRotateAngle + (rand() % 5) - 2;
		shift_x = workingShift_x + (rand() % 5) - 2;
		shift_y = workingShift_y + (rand() % 5) - 2;
		stretch_x = workingStretch_x + float(rand() % 101)/400 -0.125;
		stretch_y = workingStretch_y + float(rand() % 101)/400 -0.125;


		// constrain rotate angle
		if(rotateAngle >= 360){
			rotateAngle -= 360;
		}
		else if(rotateAngle < 0){
			rotateAngle += 360;
		}
		// constrain stretch
		if( stretch_x < 0.05){
			stretch_x = 0.05;
		}
		if( stretch_y < 0.05){
			stretch_y = 0.05;
		}


		cout << "j: " << j << endl;
		cout << "rotateAngle: " << rotateAngle << endl;
		cout << "shift_x/y: " << shift_x << " / " << shift_y << endl;
		cout << "stretch_x/y: " << stretch_x << " / " << stretch_y << endl;


		Mat ransacMat;
		Mat ransacMapThin = Mat::zeros(costmap.cells.size(), costmap.cells[0].size(), CV_8UC1);
		Mat ransacMapFilled = Mat::zeros(costmap.cells.size(), costmap.cells[0].size(), CV_8UC1);


		namedWindow("matchMat", WINDOW_NORMAL);
		imshow("matchMat", matchMat);
		waitKey(1);


		// warp the matched contour
		Mat rotMatrix = getRotationMatrix2D(matchCenter, rotateAngle, 1);
		warpAffine(matchMat, ransacMat, rotMatrix, matchMat.size());
		resize(ransacMat, ransacMat, Size(), stretch_x, stretch_y);
		threshold(ransacMat, ransacMat, 5, 255, THRESH_BINARY);


		namedWindow("ransacMat", WINDOW_NORMAL);
		imshow("ransacMat", ransacMat);
		waitKey(1);


		// pull the warped contour
		vector<vector<Point> > ransacCont;
		vector<Vec4i> tHier;
		findContours(ransacMat, ransacCont, tHier, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

		//cout << "contour.center: " << contour.center.x << " , " << contour.center.y << endl;
		//cout << "match.center: " << matchCenter.x << " , " << matchCenter.y << endl;
		//cout << "shift: " << shift_x << " , " << shift_y << endl;


		// shift and place in the map
		int px = contour.center.x - matchCenter.x + shift_x;
		int py = contour.center.y - matchCenter.y + shift_y;

		//cout << "p: " << px << " , " << py << endl;
		//cout << "ransacMat: " << ransacMat.rows << " , " << ransacMat.cols << endl;


		if( px < 0){
			px = 0;
		}
		else if( px > ransacMat.rows ){
			px = ransacMat.rows;
		}

		if( py < 0){
			py = 0;
		}
		else if( py > ransacMat.cols ){
			py = ransacMat.cols;
		}

		drawContours(ransacMapThin, ransacCont, 0, Scalar(255), 3, 8, noArray(), INT_MAX, Point(px ,py) );
		drawContours(ransacMapFilled, ransacCont, 0, Scalar(255), -1, 8, noArray(), INT_MAX, Point(px ,py) );

		namedWindow("building ransac filled match", WINDOW_NORMAL);
		imshow("building ransac filled match", ransacMapFilled);
		waitKey(1);

		namedWindow("building ransac thin match", WINDOW_NORMAL);
		imshow("building ransac thin match", ransacMapThin);
		waitKey(1);

		if(j > 0){
			namedWindow("building best match", WINDOW_NORMAL);
			imshow("building best match", bestMap);
			waitKey(1);
		}


		circle( contour.filledMat, Point(contour.center.x, contour.center.y), 3, Scalar(127), -1, 8);
		circle( contour.filledMat, contour.center, 3, Scalar(50), -1, 8);


		namedWindow("contour.thinMat", WINDOW_NORMAL);
		imshow("contour.thinMat", contour.thinMat);
		waitKey(1);

		namedWindow("contour.filledMat", WINDOW_NORMAL);
		imshow("contour.filledMat", contour.filledMat);
		waitKey(1);


		float errorThin = evalPointsBitwiseAndContours(contour.thinMat, ransacMapThin);
		float errorFill = evalPointsBitwiseAnd(contour.filledMat, ransacMapFilled);
		float errorObsWalls = evalPointsBitwiseAndContours( buildingHullWalls, ransacMapThin );
		//float error = evalPointsPoly( contours[i].points, ransacCont[0], shift_x, shift_y, 10); // point poly to test points along each contour
		//error = evalBreadthSearch(wallsInContour, ransacMap, 10); // breadth first search to nearest walls

		float error = errorThin / thinCnt + errorFill / filledCnt + errorObsWalls / wallCnt;

		cout << "errorObsWalls: " << errorObsWalls << " / " << wallCnt << endl;
		cout << "errorThin: " << errorThin << " / " << thinCnt << endl;
		cout << "errorFill: " << errorFill << " / " << filledCnt << endl;
		cout << "error: " << error << endl;
		cout << "workingError: " << workingError << endl;
		cout << "bestError: " << bestError << endl;
		cout << "p: " << exp( (workingError - error)/temp )  << endl;
		cout << "r: " << float(rand() % 1000)/1000 << endl;
		waitKey(0);

		if( exp( (workingError - error)/temp ) > float(rand() % 1000)/1000 ){
			workingError = error;

			workingRotateAngle = rotateAngle;

			workingShift_x = shift_x;
			workingShift_y = shift_y;

			workingStretch_x = stretch_x;
			workingStretch_y = stretch_y;
		}

		if(error < bestError){
			bestMap = ransacMapFilled;
			bestError = error;
		}
	}
}

void Inference::structuralBagOfWordsRoomInference(Costmap &costmap){

	for( size_t i = 0; i < contours.size(); i++ ){ // for room contours
		if(contours[i].area > 20){ // not too small of a contour

			// get inference histogram
			contours[i].getCompleteLengthHistogram();
			char title1[50];
			sprintf(title1, "contour to match");
			drawHistogram(contours[i].histogram, title1);


			namedWindow("contour to infer about", WINDOW_NORMAL);
			imshow("contour to infer about", contours[i].filledMat);
			waitKey(1);

			vector<int> matches = findMatchingRoomHistograms(contours[i].histogram, 10);

			Mat bestMatch;
			Mat bestWalls;
			float bestAngle;
			float minError = INFINITY;

			for(size_t j=0; j<matches.size(); j++){
				sprintf(title1, "contour from library");
				drawHistogram(roomHistogramList[ matches[j] ], title1);
				waitKey(1);

				// pull match from library
				vector<Point> matchContour = roomPointList[ matches[j] ];
				float matchMeanLength = roomMeanLengthList[ matches[j] ];
				Point matchCenter = roomCenterList[ matches[j] ];
				vector<Point> wallsList = roomWallsList[ matches[j] ];

				int maxW = 0; // get size of mat to display on
				float wx = 0;
				float wy = 0;
				for(size_t hi=0; hi<wallsList.size(); hi++){
					if(wallsList[hi].x > maxW){
						maxW = wallsList[hi].x;
					}
					if(wallsList[hi].y > maxW){
						maxW = wallsList[hi].y;
					}
					wx += wallsList[hi].x;
					wy += wallsList[hi].y;
				}
				wx /= wallsList.size();
				wy /= wallsList.size();

				Mat wallsMat = Mat::zeros(2*maxW, 2*maxW, CV_8UC1);

				for(size_t wi=0; wi<wallsList.size(); wi++){
					int wx = wallsList[wi].y;
					int wy = wallsList[wi].x;
					wallsMat.at<uchar>(wx, wy) = 255;
				}

				namedWindow("Inference::structuralBagOfWordsRoomsInference::wallsMat", WINDOW_NORMAL);
				imshow("Inference::structuralBagOfWordsRoomsInference::wallsMat", wallsMat);
				waitKey(1);


				int maxS = 0; // get size of mat to display on
				float mx = 0;
				float my = 0;
				for(size_t hi=0; hi<matchContour.size(); hi++){
					if(matchContour[hi].x > maxS){
						maxS = matchContour[hi].x;
					}
					if(matchContour[hi].y > maxS){
						maxS = matchContour[hi].y;
					}
					mx += matchContour[hi].x;
					my += matchContour[hi].y;
				}
				mx /= matchContour.size();
				my /= matchContour.size();

				Mat matchMat = Mat::zeros(2*maxS, 2*maxS, CV_8UC1); // draw match

				Point offset;
				offset.x = maxS - mx;
				offset.y = maxS - my;
				vector<vector<Point> > temp;
				temp.push_back(matchContour);
				drawContours(matchMat, temp, 0, Scalar(255), -1, 8, noArray(), INT_MAX, offset);

				double fxy = contours[i].meanLength / matchMeanLength;
				Size f0;
				resize(matchMat,matchMat,f0,fxy,fxy, INTER_AREA); // scale to match the partial segment
				threshold(matchMat, matchMat, 5, 255, THRESH_BINARY);

				Mat matchMatFlipped, wallsMatFlipped;
				flip( matchMat, matchMatFlipped, 1);
				flip( wallsMat, wallsMatFlipped, 1);

				namedWindow("matched contour resized", WINDOW_NORMAL);
				imshow("matched contour resized", matchMat);
				waitKey(0);

				float cost = 0;
				float costF = 0;
				float theta = 0;
				float thetaF = 0;

				initMyRANSAC( costmap, matchMat, matchCenter, contours[i], theta, cost);
				initMyRANSAC( costmap, matchMatFlipped, matchCenter, contours[i], thetaF, costF);

				if( cost < costF){
					if(cost < minError){
						bestMatch = matchMat;
						bestWalls = wallsMat;
						bestAngle = theta;
						minError = cost;
					}
				}
				else{
					if(costF < minError){
						bestMatch = matchMatFlipped;
						bestWalls = wallsMatFlipped;
						bestAngle = thetaF;
						minError = costF;
					}
				}
			}

			Mat bestMap;

			myRoomRANSAC(costmap, bestWalls, contours[i], bestAngle, bestMap, minError);
			namedWindow("best Room Map", WINDOW_NORMAL);
			imshow("best Room Map", bestMap);
			waitKey(1);

			placeRoomInCostmap(costmap, bestMap);

		}
	}
}


void Inference::structuralBagOfWordsBuildingInference(Costmap &costmap){

	// get contour of building
	Contour buildingContour = getBuildingContour(costmap);
	buildingContour.getCompleteLengthHistogram();
	Mat buildingHullWalls;// = getObservedWallsOnContour( costmap, buildingContour );

	namedWindow("Inf::buildingInf::hullWalls", WINDOW_NORMAL);
	imshow("Inf::buildingInf::hullWalls", buildingHullWalls);
	waitKey(0);

	namedWindow("Inf::buildingInf::building contour in", WINDOW_NORMAL);
	imshow("Inf::buildingInf::building contour in", buildingContour.thinMat);
	waitKey(1);

	char title1[50];
	sprintf(title1, "buildingContour contour to match");
	cerr << "into drawHistogram" << endl;
	drawHistogram(buildingContour.histogram, title1);
	cerr << "out of drawHistogram" << endl;

	vector<int> matches = findMatchingBuildingHistograms(buildingContour.histogram, 10);

	Mat bestMatch;
	float bestAngle;
	float minError = INFINITY;

	for(size_t j=0; j<matches.size(); j++){

		sprintf(title1, "contour from library");
		drawHistogram(buildingHistogramList[ matches[j] ], title1);
		waitKey(1);

		// pull match from library
		vector<Point> matchContour = buildingPointList[ matches[j] ];
		float matchMeanLength = buildingMeanLengthList[ matches[j] ];

		int maxS = 0; // get size of mat to display on
		float mx = 0;
		float my = 0;
		for(size_t hi=0; hi<matchContour.size(); hi++){
			if(matchContour[hi].x > maxS){
				maxS = matchContour[hi].x;
			}
			if(matchContour[hi].y > maxS){
				maxS = matchContour[hi].y;
			}
			mx += matchContour[hi].x;
			my += matchContour[hi].y;
		}
		mx /= matchContour.size();
		my /= matchContour.size();

		Mat matchMat = Mat::zeros(2*maxS, 2*maxS, CV_8UC1); // draw match

		Point offset;
		offset.x = maxS - mx;
		offset.y = maxS - my;
		vector<vector<Point> > temp;
		temp.push_back(matchContour);
		drawContours(matchMat, temp, 0, Scalar(255), -1, 8, noArray(), INT_MAX, offset);

		double fxy = buildingContour.meanLength / matchMeanLength;
		Size f0;
		resize(matchMat,matchMat,f0,fxy,fxy, INTER_AREA); // scale to match the partial segment
		threshold(matchMat, matchMat, 5, 255, THRESH_BINARY);
		Point matchCenter = findContourCenter(matchMat);

		// flipped match mat
		Mat matchMatFlipped;
		flip( matchMat, matchMatFlipped, 1);
		Point matchCenterF = findContourCenter(matchMatFlipped);

		namedWindow("matched building contour resized", WINDOW_NORMAL);
		imshow("matched building contour resized", matchMat);
		waitKey(1);


		float cost = 0;
		float costF = 0;
		float theta = 0;
		float thetaF = 0;

		initMyRANSAC( costmap, matchMat, matchCenter, buildingContour, theta, cost);
		initMyRANSAC( costmap, matchMatFlipped, matchCenterF, buildingContour, thetaF, costF);

		if( cost < costF){
			if(cost < minError){
				bestMatch = matchMat;
				bestAngle = theta;
				minError = cost;
			}
		}
		else{
			if(costF < minError){
				bestMatch = matchMatFlipped;
				bestAngle = thetaF;
				minError = costF;
			}
		}
	}

	Mat bestMap;
	myBuildingRANSAC(costmap, bestMatch, buildingHullWalls, buildingContour, bestAngle, bestMap, minError);

	namedWindow("myBuildingRansac::bestMap", WINDOW_NORMAL);
	imshow("myBuildingRansac::bestMap", bestMap);
	waitKey(1);

	placeBuildingInCostmap(costmap, bestMap);
}


Mat Inference::createMiniMapInferImg(){
	Mat temp = Mat::zeros(costmap.cells[0].size(), costmap.cells.size(),CV_8UC1);
	for(size_t i=0; i<hullPts.size(); i++){
		temp.at<uchar>(hullPts[i][1],hullPts[i][0],0) = 255;
	}

	Scalar color;
	color[0] = 255;

   for(size_t i=0; i<hullPts.size()-1; i++){
   	Point pa, pb;
   	pa.x = hullPts[i][0];
   	pa.y = hullPts[i][1];
   	pb.x = hullPts[i+1][0];
   	pb.y = hullPts[i+1][1];
   	line(temp, pa, pb, color, 2, 8);
   }

   for(int i=0; i<1; i++){
   	Point pa, pb;
   	pa.x = hullPts[0][0];
   	pa.y = hullPts[0][1];

   	pb.x = hullPts[hullPts.size()-1][0];
   	pb.y = hullPts[hullPts.size()-1][1];
   	line(temp, pa, pb, color, 2, 8);
   }

	return temp;
}

Mat Inference::getFrontiersImage(){

}


vector<float> Inference::getInferenceContourRewards(vector<int> frontierExits, vector<vector<Point> > contours){

	vector<float> contourAreas;

	for( size_t i = 0; i < contours.size(); i++ ){
		contourAreas.push_back(contourArea(contours[i]));
		cerr << "contourAreas: " << contourAreas[i] << endl;
	}

    // set value for all Frontiers in each internal contour
	vector<float> contourRewards;
    float maxReward = 0;
    for(size_t i=0; i<contourAreas.size(); i++){
    	Mat temp = Mat::zeros(costmap.cells.size(), costmap.cells[0].size()(),CV_8UC3);
    	Scalar color;
    	color[0] = rand() % 255;
    	color[1] = rand() % 255;
    	color[2] = rand() % 255;
		drawContours(temp,contours,i,color);

    	float members = 0;
    	for(size_t j=0; j<frontiers.size(); j++){
    		Point fp;
    		fp.x = frontiers[j].projection[1];
    		fp.y = frontiers[j].projection[0];
    		Scalar color2;
    		color2[0] = rand() % 255;
    		color2[1] = rand() % 255;
    		color2[2] = rand() % 255;

    		circle(temp,fp,1,color2);

    		if(pointPolygonTest(contours[i],fp, false) >= 0){
    			members++;
    		}
    		imshow("inferred contours", temp);
    		waitKey(1);
    	}

    	contourRewards.push_back(contourAreas[i] / members);
    	if(contourRewards[i] > maxReward && members > 0){
    		maxReward = contourRewards[i];
    	}
    	cerr << "contourRewards: " << contourRewards[i] << endl;
    }
    waitKey(1);

    // set value for all external Frontiers
    for(size_t i=0;i<frontierExits.size(); i++){
    	contourRewards[frontierExits[i]] = 10*maxReward;
    }

    return contourRewards;
}

void GraphCoordination::extractInferenceContour(){
	Mat temp;

	bitwise_or(obstacleMat, inferenceMat, temp);
	bitwise_or(temp, freeMat, temp);
	bitwise_not(temp,temp);

	namedWindow("Inference Contours", WINDOW_AUTOSIZE);
	imshow("Inference Contours", temp);
}


void GraphCoordination::cornerFinder(Mat &inputMat){
	corners.clear();
	for(size_t i=1; i<inputMat.rows-1; i++){
		for(size_t j=1; j<inputMat.cols-1; j++){
			int xp = inputMat.at<uchar>(i+1,j);
			int xm = inputMat.at<uchar>(i-1,j);

			if(xp != xm){
				int yp = inputMat.at<uchar>(i,j+1);
				int ym = inputMat.at<uchar>(i,j-1);

				if(yp != ym){
					vector<int> c;
					c.push_back(i);
					c.push_back(j);
					corners.push_back(c);
				}
			}
		}
	}
}

Mat GraphCoordination::breadthFirstSearchFindRoom(Mat &src, vector<int> pt){

	// find the clostest point to the identified point
	float minDist = INFINITY;
	vector<int> cPt;
	cPt.push_back(-1);
	cPt.push_back(-1);
	for(int i =0; i<src.cols; i++){
		for(int j=0; j<src.rows; j++){ // go through complete source image
			if(src.at<uchar>(i,j,0) > 0){ // do we care about this point?
				float d = pow(pt[0]-i,2) + pow(pt[1]-j,2);
				if(d < minDist){
					minDist = d;
					cPt[0] = i;
					cPt[1] = j;
				}
			}
		}
	}

	//find adjacent points to the closest point and add to the openSet
	vector<vector<int> > cSet;
	vector<vector<int> > oSet;
	for(int i=-1; i<2; i++){
		for(int j=-1; j<2; j++){
			if(i !=0 || j !=0){
				if(src.at<uchar>(i+cPt[0],j+cPt[1],0) > 0){
					vector<int> t;
					t.push_back(i+cPt[0]);
					t.push_back(j+cPt[1]);
					oSet.push_back(t);
				}
				cout << endl;
			}
		}
	}

	// add initial point to the closedSet
	cSet.push_back(cPt);

	// while there are still points in the openSet
	while(oSet.size() > 0){
		// add current point to closedSet and remove from openSet
		cPt = oSet[oSet.size()-1];
		cout << cPt[0] << " < " << cPt[1] << endl;
		cSet.push_back(cPt);
		oSet.pop_back();

		// find all adjacent points to cPt
		vector<vector<int> > temp;
		for(int i=-1; i<2; i++){
			for(int j=-1; j<2; j++){
				if(i !=0 || j !=0){
					if(src.at<uchar>(i+cPt[0],j+cPt[1],0) > 0){
						vector<int> t;
						t.push_back(i+cPt[0]);
						t.push_back(j+cPt[1]);

						bool flag = true;
						for(size_t k=0; k<cSet.size(); k++){
							if(t == cSet[k]){
								flag = false;
								break;
							}
						}
						if(flag){
							temp.push_back(t);
						}
					}
				}
			}
		}
		// if there is more than 1 adjacent point, add closest to oSet
		if(temp.size() > 1){
			float minDist = INFINITY;
			float mindex = -1;
			for(size_t i=0; i<temp.size(); i++){
				float d = pow(pt[0]-temp[i][0],2) + pow(pt[1]-temp[i][1],2);
				if(d < minDist){
					minDist = d;
					mindex = i;

				}
			}
			oSet.push_back(temp[mindex]);
		}
		else if(temp.size() > 0){
			oSet.push_back(temp[0]);
		}
	}

	Mat dst =Mat::zeros(src.cols, src.rows, CV_8UC1);

	for(size_t i=0; i<cSet.size(); i++){
		src.at<uchar>(cSet[i][0], cSet[i][1],0) = 255;
	}

	imshow("src", src);
	waitKey(1);

	return dst;
}

void GraphCoordination::growFrontiers(vector<Frontier> frnt){

	// find all Frontiers with traversable bath through free and observed space between them using line check

	// find all members of each Frontier and Frontier orientation^-1 and add all to oSet
	// each member of open set extend one cell in direction orient^-1 and add current cell to closed set and extended cell to oset
	// repeat

	// Frontier useful class members
	vector<float> orient; // unit vector descirbing orientation
	vector<int> center; // [x/y]
	vector<vector<int> > members; // [list][x/y]


	// get Mat of obstacles and inference combined
	Mat temp;
	bitwise_or(obstacleMat, inferenceMat, temp);

	threshold(temp,temp,10,255,CV_THRESH_BINARY);
	for(size_t i=0; i<frnt.size(); i++){
		temp.at<uchar>(frnt[i].center[0],frnt[i].center[1],0) = 255;
	}

	// find all Frontiers with traversable bath through free and observed space between them using line check
	vector<vector<Frontier> > mFrnts;
	for(size_t i=0; i<frnt.size()-1; i++){
		vector<Frontier> t;
		t.push_back(frnt[i]);
		for(size_t j=i+1; j<frnt.size(); j++){
			if(lineTraversabilityCheck(freeMat, frnt[i].center, frnt[j].center, 255)){
				t.push_back(frnt[j]);
				line(temp, Point{frnt[i].center[1], frnt[i].center[0]},Point{frnt[j].center[1], frnt[j].center[0]}, Scalar(127), 1, 8);
			}
		}
		mFrnts.push_back(t);
	}

	imshow("temp obs", temp);
	waitKey(1);

	imshow("explored", freeMat);
	waitKey(1);


	// all points in Frontier extend in direction of orient one unit as long as it is free space
	for(size_t i=0; i<frnt.members.size(); i++){
		src.at<uchar>(frnt.members[i][0], frnt.members[i][1],0) = 255;
	}

	imshow("src", src);
	waitKey(1);

	// check visibility to all points on circle
	float dx = frnt.orient[0];
	float dy = frnt.orient[1];

	if(dx != 0){
		if(dx > 0){
			float m = dy/dx;
			float b = cLoc[1]-m*cLoc[0];

			int y0 = cLoc[1];
			for(int x0 = cLoc[0]; x0 < px; x0++){
				y0 = m*x0+b;
				if(obstacleMat.at<uchar>(x0,y0,0) > 0){
					break;
				}
				else{
					viewMat.at<uchar>(x0,y0,0) = 255;
				}
			}
		}
		else{
			float m = dy/dx;
			float b = cLoc[1]-m*cLoc[0];

			int y0 = cLoc[1];
			for(int x0 = cLoc[0]; x0 > px; x0--){
				y0 = m*x0+b;
				if(obstacleMat.at<uchar>(x0,y0,0) > 0){
					break;
				}
				else{
					viewMat.at<uchar>(x0,y0,0) = 255;
				}
			}
		}
	}
	else{
		if(dy > 0){
			int x0 = cLoc[0];
			for(int y0 = cLoc[1]; y0 < py; y0++){
				if(obstacleMat.at<uchar>(x0,y0,0) > 0){
					break;
				}
				else{
					viewMat.at<uchar>(x0,y0,0) = 255;
				}
			}
		}
		else{
			int x0 = cLoc[0];
			for(int y0 = cLoc[1]; y0 > py; y0--){
				if(obstacleMat.at<uchar>(x0,y0,0) > 0){
					break;
				}
				else{
					viewMat.at<uchar>(x0,y0,0) = 255;
				}
			}

		}
	}

}

*/


