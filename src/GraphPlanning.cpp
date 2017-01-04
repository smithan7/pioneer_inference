/*
 * GraphPlanning.cpp
 *
 *  Created on: Jul 13, 2016
 *      Author: andy
 */

#include "GraphPlanning.h"


void tspRecursiveSolverP( vector<int> tour, float tourLength, vector<vector<float> > &distGraph, vector<int> oSet, vector<int> &minTour, float &minLength, int depth);
bool pointCompare(Point &a, Point &b); // in costmap.cpp

GraphPlanning::GraphPlanning() {

	nPullsEvolvePoseGraph = 10;
	minObsForPose = 1;
	vector<Point> hullBreaches;
}

GraphPlanning::~GraphPlanning() {}

void GraphPlanning::init(float obsRadius ){
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

void GraphPlanning::marketPoses( Costmap &costmap, Point cLoc, Point &gLoc ){

	cerr << "GraphPlanning::marketPoses::A" << endl;

	vector<float> poseRewards;
	vector<float> poseDistances;
	vector<bool> trueDist;
	vector<float> poseValue;

	float maxVal = -INFINITY;
	int maxPose = -1;

	Mat gView = Mat::zeros(costmap.cells.size(), CV_8UC1);
	//cout << "PoseGraph.nodeLocations: ";
	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		//cout << poseGraph.nodeLocations[i] << ", ";
		Mat lView = Mat::zeros( costmap.cells.size(), CV_8UC1);
		simulateObservation( poseGraph.nodeLocations[i], lView, costmap );
		bitwise_or(gView, lView, gView);
	}
	//cout << endl;
	//	float globalReward = observedReward( gView, costmap.reward );

	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		//poseRewards.push_back( getDiscountedRewards( costmap, poseGraph.nodeLocations, i, globalReward ) );
		poseRewards.push_back( observedReward(poseGraph.nodeObservations[i], costmap.reward) );
		if(poseRewards.back() < 1){
			poseRewards.back() = -INFINITY;
		}

		poseDistances.push_back( sqrt(pow(cLoc.x-poseGraph.nodeLocations[i].x,2) + pow(cLoc.y-poseGraph.nodeLocations[i].y,2) ));
		trueDist.push_back( false );

		poseValue.push_back( poseRewards.back() - 0.1*pow(poseDistances.back(),2) );

		//cout << "index, location, value, rewards, distance: " << i << ", " << poseGraph.nodeLocations[i] << ", " << poseValue.back() << ", " << poseRewards.back() << ", " << poseDistances.back() << endl;

		if(poseValue.back() > maxVal){
			maxVal = poseValue.back();
			maxPose = i;
		}
	}

	while( true ){ // find best pose

		if( trueDist[maxPose]){
			break;
		}

		poseDistances[maxPose] = costmap.aStarDist(cLoc, poseGraph.nodeLocations[maxPose]);
		poseValue[maxPose] = poseRewards[maxPose] - 0.1*pow(poseDistances[maxPose],2);
		//cout << "GraphPlanning::marketPoses::poseValue[maxPose]: " << poseValue[maxPose] << endl;
		trueDist[maxPose] = true;

		maxVal = -INFINITY;
		maxPose = -1;
		for(size_t i=0; i<poseValue.size(); i++){
			if(poseValue[i] > maxVal){
				maxPose = i;
				maxVal = poseValue[i];
			}
		}


		//cout << "maxPose: index, value, rewards, distance: " << maxPose << ", " << poseValue[maxPose] << ", " << poseRewards[maxPose] << ", " << poseDistances[maxPose] << endl;
	}

	cout << "GraphPlanning::marketPoses::goalPose / gValue: " << poseGraph.nodeLocations[maxPose] << " / " << poseValue[maxPose] << endl;

	gLoc = poseGraph.nodeLocations[maxPose];
}


float GraphPlanning::getDiscountedRewards( Costmap &costmap, vector<Point> &locs, int i, float globalReward ){

	Mat notiView = Mat::zeros(costmap.cells.size(), CV_8UC1);
	size_t i_size = (size_t) i;
	for(size_t j=0; j<locs.size(); j++){
		if(j != i_size ){
			Mat jView = Mat::zeros( costmap.cells.size(), CV_8UC1);
			simulateObservation( locs[j], jView, costmap );
			bitwise_or(notiView, jView, notiView);
		}
	}
	return globalReward - observedReward( notiView, costmap.reward );
}


bool GraphPlanning::gNodeValueCheck(Point cLoc, Point gLoc, Costmap &costmap){

	cout << "cLoc: " << cLoc.x << " , " << cLoc.y << endl;
	cout << "gLoc: " << gLoc.x << " , " << gLoc.y << endl;

	if(cLoc == gLoc){
		return true;
	}

	if(this->poseGraph.nodeLocations.size() <= 1){
		return false;
	}

	Mat resView = Mat::zeros(costmap.cells.size(), CV_8UC1);
	simulateObservation(gLoc, resView, costmap);
	float rew = matCount( resView );

	cout << "rew: " << rew << endl;

	return (rew < minObsForPose);

}

Point GraphPlanning::TSPPosePathPlanning(Costmap &costmap){

	findPoseGraphTransitions(costmap); // get A* dist
	cout << "GraphPlanning::found poseGraphTransitions" << endl;
	displayPoseGraph(costmap);

	tspPoseFullPathPlanner();
	cout << "GraphPlanning::out of tspPosePathPlanner" << endl;

	checkTSPPosePathNodes(costmap);

	displayPosePath(costmap);
	waitKey(1);

	cout << "TSPPosePathPlanning::returning tspPosePath[1]: " << poseGraph.nodeLocations[tspPosePath[1]].x << ", " << poseGraph.nodeLocations[tspPosePath[1]].y << endl;
	return poseGraph.nodeLocations[ tspPosePath[1] ];
}

float GraphPlanning::getGraphObservations(Graph &graph, Costmap &costmap, Mat &gView, vector<int> &workingSet){
	graph.nodeObservations.clear();
	for(size_t i=0; i<graph.nodeLocations.size(); i++){
		Mat t = Mat::zeros(gView.size(), CV_8UC1);
		graph.nodeObservations.push_back(t);
	}
	for(size_t i=0; i<workingSet.size(); i++){ // simulate observations from all nodes
		simulateObservation(graph.nodeLocations[workingSet[i] ], graph.nodeObservations[workingSet[i] ], costmap);
		bitwise_or(gView, graph.nodeObservations[workingSet[i] ], gView);
	}
	return observedReward(gView, costmap.reward);
}

void GraphPlanning::getViews(Costmap &costmap, vector<int> cPoses, Mat &cView, float &cReward, vector<Point> &poses, vector<Mat> &views, vector<float> poseRewards, float &gReward){

	Mat gView = Mat::zeros(costmap.cells.size(), CV_8UC1);

	cerr << "cPoses: ";
	for(size_t i=0; i<cPoses.size(); i++){
		Mat t = Mat::zeros(costmap.cells.size(), CV_8UC1);
		simulateObservation(poses[ cPoses[i] ], t, costmap);
		cerr << poses[ cPoses[i] ] << ", ";
		bitwise_or(cView, t, cView);
	}
	cerr << endl;
	cReward = observedReward(cView, costmap.reward);

	for(size_t i=0; i<poses.size(); i++){
		Mat t = Mat::zeros(costmap.cells.size(), CV_8UC1);
		simulateObservation(poses[i], t, costmap);
		views.push_back(t);
		poseRewards.push_back(observedReward( t, costmap.reward ) );
		bitwise_or(gView, t, gView);
	}
	gReward = observedReward(gView, costmap.reward);

	/*
	cerr << "gReward: " << gReward << endl;
	namedWindow("gView", WINDOW_NORMAL);
	imshow("gView", gView);
	waitKey(1);
	*/
}

void GraphPlanning::plotPoses( Costmap &costmap, vector<int> &cPoses, vector<int> &oPoses, vector<Mat> &views, vector<Point> &poses){

	Mat gView = Mat::zeros(views[0].size(), CV_8UC1);

	for(size_t i=0; i<cPoses.size(); i++){
		bitwise_or(gView, views[cPoses[i] ], gView);
	}

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point a(i,j);
			if( gView.at<uchar>(a) > 127 && costmap.reward.at<float>(a) == 0){
				gView.at<uchar>(a) = 0;
			}
		}
	}


	for(size_t i=0; i<oPoses.size(); i++){
		circle(gView, poses[ oPoses[i] ], 2, Scalar(100), -1, 8);
	}

	for(size_t i=0; i<cPoses.size(); i++){
		circle(gView, poses[ cPoses[i] ], 2, Scalar(200), -1, 8);
	}


	Point2f src_center(gView.cols/2.0F, gView.rows/2.0F);
	Mat rot_mat = getRotationMatrix2D(src_center, 270, 1.0);
	Mat dst;
	warpAffine(gView, dst, rot_mat, gView.size());

	namedWindow("Pose Set", WINDOW_NORMAL);
	imshow("Pose Set", dst);
	waitKey(1);
}


void GraphPlanning::getOset(vector<int> &oStates, vector<int> &cStates, vector<int> &workingSet){
	for(size_t i = 0; i<workingSet.size(); i++){
		bool flag = true;
		for(size_t j=0; j<cStates.size(); j++){
			if(workingSet[i] == cStates[j]){
				flag = false;
				break;
			}
		}
		if(flag){
			oStates.push_back(workingSet[i]);
		}
	}
}

vector<int> GraphPlanning::getCSatesFromWorkingSet( vector<int> workingSet ){

	vector<int> cStates;

	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		for(size_t j=0; j<workingSet.size(); j++){
			if(poseGraph.nodeLocations[i] == thinGraph.nodeLocations[workingSet[j]]){
				cStates.push_back(workingSet[j]);
				break;
			}
		}
	}

	return cStates;
}

void GraphPlanning::mergePoseGraphIntoThinGraph(){

	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		bool flag = true;
		for(size_t j=0; j<thinGraph.nodeLocations.size(); j++){
			if(poseGraph.nodeLocations[i] == thinGraph.nodeLocations[j]){
				flag = false; // don't need to add node to thinGraph
				break;
			}
		}
		if(flag){ // was not in thinGraph, so add
			thinGraph.nodeLocations.push_back(poseGraph.nodeLocations[i]);
			thinGraph.nodeObservations.push_back(poseGraph.nodeObservations[i]);
			thinGraph.nodeSubGraphIndices.push_back(poseGraph.nodeSubGraphIndices[i]);
		}
	}
}

float GraphPlanning::getCurrentPoseSetReward(Graph &graph, Costmap &costmap, Mat &cView, vector<int> &workingSet){
	// check if poseGraph nodes are still on graph, if yes import their view, else erase
	//cView = Mat::zeros(graph.nodeObservations[workingSet[0]].size(), CV_8UC1);
	for(size_t j=0; j<graph.nodeLocations.size(); j++){
		bool flag = true;
		for(size_t i = 0; i<workingSet.size(); i++){
			if(graph.nodeLocations[j] == graph.nodeLocations[workingSet[i] ]){
				graph.nodeSubGraphIndices[j] = workingSet[i];
				graph.nodeObservations[j] = graph.nodeObservations[workingSet[i] ];

				bitwise_or(cView, graph.nodeObservations[workingSet[i] ], cView);
				flag = false;
				break;
			}
		}
		if(flag){
			graph.nodeLocations.erase(graph.nodeLocations.begin()+j);
			graph.nodeObservations.erase(graph.nodeObservations.begin()+j);
			graph.nodeSubGraphIndices.erase(graph.nodeSubGraphIndices.begin()+j);
		}
	}
	return matCount(cView);
}

vector<int> GraphPlanning::getWorkingSet(Costmap &costmap){
	vector<int> workingSet;
	for(size_t i = 0; i<thinGraph.nodeLocations.size(); i++){
		int con = thinGraph.nodeSubGraphIndices[i];
		if(con > -1){
			float d = travelGraph.aStarDist(con, travelGraph.cNode, costmap);
			if( d != INFINITY && travelGraph.nodeLocations[con] != travelGraph.nodeLocations[travelGraph.cNode]){ // reachable and not current node
				workingSet.push_back(i);
			}
		}
	}
	return workingSet;
}


void GraphPlanning::findPosesEvolution(Costmap &costmap){

	cout << "GraphPlanning::findPosesEvolution::A" << endl;

	// get graph observations and global reward
	vector<int> oPoses, cPoses, bPoses;
	vector<Point> poseLocations;
	vector<Mat> poseViews;
	vector<float> poseRewards;

	// add old nodes and get view
	Mat cView = Mat::zeros(costmap.cells.size(), CV_8UC1);
	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		poseLocations.push_back( poseGraph.nodeLocations[i] );
		Mat tView = Mat::zeros(costmap.cells.size(), CV_8UC1);
		simulateObservation(poseGraph.nodeLocations[i], tView, costmap);
		poseViews.push_back(tView);
		float tReward = observedReward( tView, costmap.reward );
		poseRewards.push_back( tReward );
		if( tReward > 0 ){
			cPoses.push_back(i);
			bitwise_or(cView, tView, cView);
		}
	}
	float cReward = observedReward( cView, costmap.reward );
	Mat gView = Mat::ones(costmap.cells.size(), CV_8UC1)*255;
	float gReward = observedReward( gView, costmap.reward );

	// add potential new nodes
	for(size_t i=0; i<thinGraph.nodeLocations.size(); i++){
		oPoses.push_back( cPoses.size() + i );
		poseLocations.push_back( thinGraph.nodeLocations[i] );
		poseRewards.push_back(-1);
		Mat tView = Mat::zeros(costmap.cells.size(), CV_8UC1);
		poseViews.push_back(tView);
	}

	// any poses with 0 reward removed from cPoses
	for(size_t i=0; i<cPoses.size(); i++){
		int t = cPoses[i];
		if( poseRewards[ t ] <= 0){
			cPoses.erase(cPoses.begin() + i);
			oPoses.push_back( t );
		}
	}

	cout << "GraphPlanning::findPosesEvolution::C::cReward / gReward: " << cReward << " / " << gReward << endl;


	// add more nodes to pose graph if not fully observing world

	clock_t tStart1 = clock();
	int iter = 0;
	while(cReward < gReward && iter < 50){ // add states that increase the observed are until % is observed
		iter++;
		int c = rand() % oPoses.size();
		int index = oPoses[c];
		if(poseRewards[index] == -1){
			simulateObservation(poseLocations[index], poseViews[index], costmap);
			poseRewards[index] = observedReward( poseViews[index], costmap.reward );
		}

		Mat tView;
		bitwise_or(cView, poseViews[ index ], tView);
		float tReward = observedReward(tView, costmap.reward);

		if(tReward > cReward){
			cPoses.push_back( index );

			cView = tView;
			cReward = tReward;

			oPoses.erase(oPoses.begin() + c);
		}
	}
	printf("Time taken to create get poses: %.2fs\n", (double)(clock() - tStart1)/CLOCKS_PER_SEC);

	//cerr << "GraphPlanning::findPosesEvolution::D::cReward / gReward: " << cReward << " / " << gReward << endl;
	int cPost = cPoses.size();
	iter = 0;
	tStart1 = clock();
	while( iter < cPost ){
		int dP = cPoses[iter];

		Mat dMat = Mat::zeros(costmap.cells.size(), CV_8UC1);
		for(size_t i=0; i<cPoses.size(); i++){
			if( int(i) != iter ){
				bitwise_or(dMat, poseViews[i], dMat);
			}
		}

		float dReward = observedReward( dMat, costmap.reward );

		if( dReward == gReward ){
			cPoses.erase(cPoses.begin() + iter);
			oPoses.push_back( dP );
			cPost--;
			cerr << "erased dP" << endl;
		}
		iter++;
	}
	printf("Time taken to prune poses: %.2fs\n", (double)(clock() - tStart1)/CLOCKS_PER_SEC);

	plotPoses( costmap, cPoses, oPoses, poseViews , poseLocations);
	waitKey(1);

	poseGraph.nodeLocations.clear();
	poseGraph.nodeObservations.clear();
	for(size_t i=0; i<cPoses.size(); i++){
		poseGraph.nodeLocations.push_back( poseLocations[ cPoses[i] ] );
		poseGraph.nodeObservations.push_back( poseViews[ cPoses[i] ] );
	}

	cerr << "GraphPlanning::findPosesEvolution::E" << endl;
}

void GraphPlanning::simulateNULLObservation(Point pose, Mat &resultingView, Costmap &costmap){
	// make perimeter of viewing circle fit on image

	for(size_t i=0; i<viewPerim.size(); i++){
		Point v(viewPerim[i].x + pose.x, viewPerim[i].y + pose.y);
		LineIterator it(resultingView, pose, v, 8, false);
		for(int i=0; i<it.count; i++, ++it){

			Point pp  = it.pos();

			if(costmap.cells.at<short>(pp) > costmap.domFree){
				break;
			}
			else{
				resultingView.at<uchar>(pp) = 0;
			}
		}
	}

	/*
	Mat fu = resultingView.clone();
	circle(fu, pose, 2, Scalar(127), -1, 8);
	namedWindow("GraphPlanning::simulateView::view", WINDOW_NORMAL);
	imshow("GraphPlanning::simulateView::view", fu);
	waitKey(0);
	*/
}

void GraphPlanning::simulateObservation(Point pose, Mat &resultingView, Costmap &costmap){
	// make perimeter of viewing circle fit on image

	for(size_t i=0; i<viewPerim.size(); i++){
		Point v(viewPerim[i].x + pose.x, viewPerim[i].y + pose.y);
		LineIterator it(resultingView, pose, v, 8, false);
		for(int i=0; i<it.count; i++, ++it){

			Point pp  = it.pos();

			if(costmap.cells.at<short>(pp) > costmap.domFree){
				break;
			}
			else{
				resultingView.at<uchar>(pp) = 255;
			}
		}
	}

	/*
	Mat fu = resultingView.clone();
	circle(fu, pose, 2, Scalar(127), -1, 8);
	namedWindow("GraphPlanning::simulateView::view", WINDOW_NORMAL);
	imshow("GraphPlanning::simulateView::view", fu);
	waitKey(0);
	*/
}

int GraphPlanning::matCount(Mat &in){
	// get Mat entropy
	float observed = 0;
	for(int i=0; i<in.cols; i++){
		for(int j=0; j<in.rows; j++){
			Point a(i,j);
			if(in.at<uchar>(a) > 0){
				observed++;
			}
		}
	}
	return observed;
}

float GraphPlanning::getPoseReward( Point in, Costmap &costmap){

	Mat t = Mat::zeros(costmap.cells.size(), CV_8UC1);
	simulateObservation( in, t, costmap);

	return observedReward( t, costmap.reward );
}

float GraphPlanning::observedReward(Mat &observed, Mat &reward){
	// get Mat entropy
	float r = 0;
	for(int i=0; i<observed.cols; i++){
		for(int j=0; j<observed.rows; j++){
			Point a(i,j);
			if(observed.at<uchar>(a) > 0){
				r += reward.at<float>(a);
			}
		}
	}
	return r;
}

void GraphPlanning::findGraphTravelGraphConnectors(Graph &graph, Costmap &costmap){
	vector<int> a(graph.nodeLocations.size(), -1);
	for(size_t i=0; i<graph.nodeLocations.size(); i++){
		a[i] = travelGraph.findNearestNode(graph.nodeLocations[i], costmap);
	}
	graph.nodeSubGraphIndices = a;
}

void GraphPlanning::findPoseGraphTransitions(Costmap &costmap){

	poseGraph.nodeTransitions.clear();
	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		vector<float> t;
		for(size_t j=0; j<poseGraph.nodeLocations.size(); j++){
			t.push_back(INFINITY);
		}
		poseGraph.nodeTransitions.push_back(t);
	}

	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		for(size_t j=0; j<poseGraph.nodeLocations.size(); j++){
			if(i != j){
				float d = travelGraph.aStarDist(poseGraph.nodeSubGraphIndices[i],poseGraph.nodeSubGraphIndices[j], costmap);
				poseGraph.nodeTransitions[i][j] = d;
				poseGraph.nodeTransitions[j][i] = d;
			}
		}
	}

	/*
	cout << "GraphPlanning::findPoseGraphTransitions::nodeTransitions array:" << endl;
	for(size_t i=0; i<poseGraph.nodeTransitions.size(); i++){
		cout << "   ";
		for(size_t j=0; j<poseGraph.nodeTransitions.size(); j++){
			cout <<poseGraph.nodeTransitions[i][j] << ", ";
		}
		cout << endl;
	}
	*/
}

void GraphPlanning::checkTSPPosePathNodes(Costmap &costmap){
	size_t i = 1;
	while(i<tspPosePath.size() ){
		Mat temp = Mat::zeros(costmap.cells.size(), CV_8UC1);
		simulateObservation( tspPosePathLocations[i], temp, costmap);
		float t = matCount( temp );
		if( t < minObsForPose){
			tspPosePath.erase( tspPosePath.begin() + i);
			tspPosePathLocations.erase( tspPosePathLocations.begin() + i);
		}
		else{
			i++;
		}

	}
}


void GraphPlanning::tspPoseFullPathPlanner(){
	cout << "GraphPlanning::tspPoseFullPathPlanner::in" << endl;

	size_t origPathSize, clearedPathSize, appendedPathSize;
	origPathSize = tspPosePath.size();

 	cout << "GraphPlanning::tspPosePath::path: ";
	for(size_t i=0; i<tspPosePath.size(); i++){
		cout << tspPosePath[i] << ", ";
	}
	cout << endl;




	// initialize  / seed path
	if(tspPosePath.size() > 1){
		vector<int> tempPath;
		for(size_t i=1; i<tspPosePath.size(); i++){ // check if all nodes on tspPosePath are still pose nodes
			for(size_t j=0; j<poseGraph.nodeLocations.size(); j++){
				if( pointCompare(tspPosePathLocations[i], poseGraph.nodeLocations[j]) ){
					tempPath.push_back(j);
				}
			}
		}
		// found existing path nodes, build new path
		tspPosePath.clear();
		tspPosePathLocations.clear();
		tspPosePath.push_back(0); // add starting locations
		for(size_t i=0; i<tempPath.size(); i++){
			tspPosePath.push_back(tempPath[i]);
		}
	}
	else{
		tspPosePath.clear();
		tspPosePath.push_back(0);
		tspPosePathLocations.clear();
		tspBuildGreedyPath(tspPosePath);
	}

	clearedPathSize = tspPosePath.size();

	cout << "GraphPlanning::tspPosePath::path after clearing out old nodes: ";
	for(size_t i=0; i<tspPosePath.size(); i++){
		cout << tspPosePath[i] << ", ";
	}
	cout << endl;

	tspBuildGreedyPath( tspPosePath );

	appendedPathSize = tspPosePath.size();

	cout << "GraphPlanning::tspPosePath::path after appending with unclaimed nodes: ";
	for(size_t i=0; i<tspPosePath.size(); i++){
		cout << tspPosePath[i] << ", ";
	}
	cout << endl;

	vector<int> bestPath = tspPosePath;
	float bestLength = INFINITY;

	vector<int> tempPath;
	tempPath.push_back(0);
	tspBuildGreedyPath(tempPath);


	//tspRecursiveSolverP(tempPath, 0, poseGraph.nodeTransitions, oSet, bestPath, bestLength, 0);

	bestLength = tspEstimatePathLength(bestPath);

	vector<int> workingPath = bestPath;
	float workingLength = bestLength;

	cout << "nodes.size(): " << poseGraph.nodeLocations.size() << endl;
	cout << "bestPath.size():* " << bestPath.size() << endl;

	cout << origPathSize << " , " << clearedPathSize << " , " << appendedPathSize << endl;
	//	int newNodes = appendedPathSize - clearedPathSize; // equivalent to: (origPathSize - clearedPathSize) + (appendedPathSize - origPathSize);

	float temperature = 1000;
	nPullsTSP = 1500*poseGraph.nodeLocations.size();
	cerr << "nPullsTSP: " << nPullsTSP << endl;
	float coolingRate = 0.9995;

	if(bestPath.size() > 2){
		for(int i=0; i<nPullsTSP; i++){

			vector<int> tempPath = tspPoseEvolveFullPath(workingPath);
			float tempLength = tspEstimatePathLength( tempPath );

			/*
			cout << "GraphPlanning::tspFullPath::path: ";
			for(size_t i=0; i<tempPath.size(); i++){
				cout << tempPath[i] << ", ";
			}
			cout << endl;

			cout << "tempPathLength: " << tempLength << endl;
			cout << "bestPathLength: " << bestLength << endl;
			cout << "workingPathLength: " << workingLength << endl;
			cout << "temp: " << temperature << endl;
			cout << "Probability of acceptance: " << exp( (workingLength - tempLength)/temperature ) << endl;
			waitKey(0);
			*/

			if( exp( (workingLength - tempLength)/temperature ) > float(rand() % 1000)/1000 ){
				workingPath = tempPath;
				workingLength = tempLength;
			}
			if(tempLength < bestLength * 0.9){
				bestLength = tempLength;
				bestPath = tempPath;
			}
			temperature *= coolingRate;
		}
	}
	//waitKey(0);



	tspPosePath = bestPath;
	tspPosePathLocations.clear();
	for(size_t i=0; i<tspPosePath.size(); i++){
		tspPosePathLocations.push_back(poseGraph.nodeLocations[tspPosePath[i]]);
	}
}


void tspRecursiveSolverP( vector<int> tour, float tourLength, vector<vector<float> > &distGraph, vector<int> oSet, vector<int> &minTour, float &minLength, int depth){

	if(oSet.size() == 0){ // is the search complete
		if(tourLength < minLength){
			minLength = tourLength;
			minTour = tour;
		}
	}
	else{ // search incomplete

		for(size_t i=0; i<oSet.size(); i++){ // try all children of this node

			float di = distGraph[tour.back()][oSet[0]] + tourLength; // get distance

			if( di < minLength){ // do NOT prune, still viable

				tour.push_back(oSet[0]); // add to tour
				tourLength += di; // add to length
				oSet.erase( oSet.begin() ); // erase from oSet

				tspRecursiveSolverP(tour, tourLength, distGraph, oSet, minTour, minLength, depth + 1); // recursive call

				// undo all changes to path from previous branch
				tourLength -= di;
				oSet.push_back( tour.back() );
				tour.pop_back();
			}
			else{
			//	cout << "GraphCoordination::tspAddNode::branch not viable" << endl;
				int t = oSet[0];
				oSet.erase( oSet.begin() );
				oSet.push_back(t);
			}
		}
	}
}

vector<int> GraphPlanning::tspPoseEvolveFullPath(vector<int> path){

	/*
	cout << "GraphPlanning::tspBuildGreedyFullPath::path: ";
	for(size_t i=0; i<path.size(); i++){
		cout << path[i] << ", ";
	}
	cout << endl;
	*/

	if(path.size() > 2){
		int nSwaps = 1;
		if(rand() % 1000 > 500){ // swap 1
			nSwaps = 2;
		}

		for(int i=0; i<nSwaps; i++){
			int s1 = rand() % (path.size()-1) + 1;
			int s2 = s1;
			while(s1 == s2){
				s2 = rand() % (path.size()-1) + 1;
			}
			int t = path[s1];
			path[s1] = path[s2];
			path[s2] = t;
		}
	}
	/*
	cout << "GraphPlanning::tspBuildGreedyFullPath::path: ";
	for(size_t i=0; i<path.size(); i++){
		cout << path[i] << ", ";
	}
	cout << endl;
	*/
	return path;
}

void GraphPlanning::tspBuildGreedyPath(vector<int> &path){

	if(path.size() > 0){
		path[0] = 0;
	}
	else{
		path.push_back(0);
	}

	/*
	cout << "GraphPlanning::tspBuildGreedyFullPath::path: ";
	for(size_t i=0; i<path.size(); i++){
		cout << path[i] << ", ";
	}
	cout << endl;
	*/

	// remove duplicate entries in path
	for(size_t i =0; i<path.size(); i++){
		size_t j = 0;
		while(j < path.size()){
			if(i == j){ // don't erase self
				j++;
				continue;
			}
			else{
				if(path[i] == path[j]){ // if duplicate entry, erase
					path.erase(path.begin() + j);
				}
			}
			j++;
		}
	}

	vector<int> oSet;
	// populate oSet
	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		bool flag = true;
			for(size_t j=0; j<path.size(); j++){
			if((int)i == path[j]){
				flag = false;
				break;
			}
		}
		if(flag){
			oSet.push_back(i);
		}
	}

	// populate path with greedy selection
	while(oSet.size() > 0){
		float minDist = INFINITY;
		int mindex = -1;
		for(size_t i=0; i<oSet.size(); i++){
			if(poseGraph.nodeTransitions[path.back()][oSet[i]] <= minDist){
				minDist = poseGraph.nodeTransitions[path.back()][oSet[i]];
				mindex = i;
			}
		}
		path.push_back(oSet[mindex]);
		oSet.erase(oSet.begin()+mindex);
	}

	cout << "GraphPlanning::tspBuildGreedyFullPath::path: ";
	for(size_t i=0; i<path.size(); i++){
		cout << path[i] << ", ";
	}
	cout << endl;
}



void GraphPlanning::tspPosePathPlanner(float maxLength){
	cerr << "GraphPlanning::tspPosePathPlanner::in" << endl;

	// initialize path
	if(tspPosePath.size() > 1){
		tspPosePath[0] = poseGraph.cNode; // reseed cNode
		vector<int> tempPath;
		for(int i=tspPosePath.size()-1; i>-1; i--){ // check if all nodes on tspPosePath are still pose nodes
			for(size_t j=0; j<poseGraph.nodeLocations.size(); j++){
				if( pointCompare(tspPosePathLocations[i], poseGraph.nodeLocations[j]) ){
					tempPath.push_back(j);
				}
			}
		}
		tspPosePath.clear();
		for(int i=tempPath.size()-1; i>-1; i--){
			tspPosePath.push_back(tempPath[i]);
		}
	}

	vector<int> bestPath;
	float bestReward;
	//float bestLength;
	//float bestValue;

	vector<int> workingPath;
	float workingReward;
	//float workingLength;
	//float workingValue;

	if(tspPosePath.size() > 1){
		cerr << "seeding tspPosePath" << endl;
		bestPath = tspPosePath;
		tspTrimPathToLength(bestPath, maxLength);
		//bestLength = tspEstimatePathLength(bestPath);
		bestReward = tspEstimatePathReward(bestPath);
		//bestValue = bestReward - bestLength;

		workingPath = tspPosePath;
		workingReward = bestReward;
		//workingLength = bestLength;
		//workingValue  = bestValue;
	}
	else{
		cerr << "building new path" << endl;
		bestPath = tspBuildRandomPath(maxLength);
		tspTrimPathToLength(bestPath, maxLength);
		//bestLength = tspEstimatePathLength(bestPath);
		bestReward = tspEstimatePathReward(bestPath);
		//bestValue = bestReward - bestLength;

		workingPath = tspPosePath;
		workingReward = bestReward;
		//workingLength = bestLength;
		//workingValue = bestValue;
	}

	cout << "bestPath.size():* " << bestPath.size() << endl;
	for(size_t i=0; i<bestPath.size(); i++){
		cout << poseGraph.nodeLocations[ bestPath[i] ].x << ", " << poseGraph.nodeLocations[ bestPath[i] ].y << "; ";
	}
	cout << endl;
	waitKey(10);
	float temp = 100;
	for(int i=0; i<nPullsTSP; i++){
		vector<int> tempPath = tspEvolvePath(workingPath, maxLength);
		tspTrimPathToLength(bestPath, maxLength);
		float tempReward = tspEstimatePathReward(tempPath);
		//float tempLength = tspEstimatePathLength( tempPath );
		//		float tempValue = tempReward - tempLength;

		if( exp( (tempReward - workingReward)/temp ) > float(rand() % 1000)/1000 ){
			workingReward = tempReward;
			workingPath = tempPath;
		}
		if(tempReward > bestReward){
			bestReward = tempReward;
			bestPath = tempPath;
		}
		temp = temp * 0.9995;
	}
	tspPosePath = bestPath;
	tspPosePathLocations.clear();
	for(size_t i=0; i<tspPosePath.size(); i++){
		tspPosePathLocations.push_back(poseGraph.nodeLocations[tspPosePath[i]]);
	}
}

vector<int> GraphPlanning::tspBuildRandomPath(float maxLength){
	cout << "GraphPlanning::tspBuildRandomPath::in" << endl;
	vector<int> path;
	float pathLength = 0;
	path.push_back(poseGraph.cNode);
	int current = 0;
	vector<int> oSet;

	for(size_t i=1; i<poseGraph.nodeLocations.size(); i++){
		oSet.push_back(i);
	}

	while(pathLength < maxLength && oSet.size() != 0){
		int index = rand() % oSet.size();
		int nbr = oSet[index];
		oSet.erase(oSet.begin() + index);
		pathLength += poseGraph.nodeTransitions[current][nbr];
		current = nbr;
		path.push_back(current);
	}

	return path;
}

void GraphPlanning::tspTrimPathToLength(vector<int> &path, float maxLength){
	float l = 0;
	vector<int> pOut;
	pOut.push_back(path[0]);
	for(size_t i=1; i<path.size(); i++){
		if(l + poseGraph.nodeTransitions[path[i]][path[i-1]] < maxLength){
			pOut.push_back(path[i]);
		}
		else{
			break;
		}
	}
	path = pOut;
}

vector<int> GraphPlanning::tspEvolvePath(vector<int> path, float maxLength = INFINITY){ // URGENT account for max length and rebuild using nbrs
	//cout << "GraphPlanning::tspEvolvePath::in" << endl;

	vector<int> oSet;
	for(uint i=0; i<poseGraph.nodeLocations.size(); i++){
		bool flag = true;
		for(size_t j=0; j<path.size(); j++){
		  if(path[j] == (int)i){
				flag = false;
				break;
			}
		}
		if(flag){
			oSet.push_back(i);
		}
	}

	if(oSet.size() > 1 && path.size() > 1){
		int swaps = 1;
		if(poseGraph.nodeLocations.size() - path.size() > 2){
			int p = rand() % 10000;
			if(p > 5000){ // swap two
				swaps = 2;
			}
			else{
				swaps = 1;
			}
		}
		for(int i=0; i<swaps; i++){
			int c = 1+rand() % (path.size()-1); // random node from path that is not!!! the start point
			int c2 = rand() % oSet.size(); // random node from oSet

			oSet.push_back(path[c]);
			path[c] = oSet[c2];
			oSet.erase(oSet.begin() + c2);
		}
	}
	return path;
}

float GraphPlanning::tspEstimatePathReward(vector<int> path){
	Mat pathView = Mat::zeros(poseGraph.nodeObservations[0].size(), CV_8UC1);
	for(size_t i=0; i < path.size(); i++){
		bitwise_or(pathView, poseGraph.nodeObservations[path[i]], pathView);
	}
	return matCount( pathView );
}

float GraphPlanning::tspEstimatePathLength(vector<int> path){
	float pathLength = 0;
	for(size_t i=1; i<path.size(); i++){
		pathLength += poseGraph.nodeTransitions[ path[i] ][ path[i-1] ];
	}
	return pathLength;
}

void GraphPlanning::displayPoseGraph(Costmap &costmap){
	Mat view = Mat::zeros(costmap.cells.size(), CV_8UC1);
	for(size_t i=1; i<poseGraph.nodeLocations.size(); i++){
		bitwise_or(view, poseGraph.nodeObservations[i], view);
	}
	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		circle(view, poseGraph.nodeLocations[i], 2, Scalar(127), -1);
		char buff[5];
		sprintf(buff, "%i", (int)i);
		putText(view, buff, poseGraph.nodeLocations[i], 0, 0.35, Scalar(127));
	}

	circle(view, poseGraph.nodeLocations[0], 3, Scalar(255), -1);
	char buff[5];
	sprintf(buff, "C");
	putText(view, buff, poseGraph.nodeLocations[0], 0, 0.35, Scalar(127));

	namedWindow("GraphPlanning::poseGraph.nodes.observations", WINDOW_NORMAL);
	imshow("GraphPlanning::poseGraph.nodes.observations", view);
}

void GraphPlanning::displayPosePath(Costmap &costmap){
	// build plot to show off the path
	Mat tMat = Mat::zeros(costmap.cells.size(), CV_8UC1);;
	for(size_t i=1; i<poseGraph.nodeLocations.size(); i++){
		bitwise_or(tMat, poseGraph.nodeObservations[i], tMat);
	}

	cout << "tspPosePath[" << tspPosePath.size() << "]: " << poseGraph.nodeLocations[tspPosePath[0]].x << " / " << poseGraph.nodeLocations[tspPosePath[0]].y;
	for(size_t i=1; i<tspPosePath.size(); i++){
		cout << ", " << poseGraph.nodeLocations[tspPosePath[i]].x << " / " << poseGraph.nodeLocations[tspPosePath[i]].y;
		line(tMat, poseGraph.nodeLocations[tspPosePath[i]], poseGraph.nodeLocations[tspPosePath[i-1]], Scalar(127), 1);
	}
	cerr << endl;

	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		circle(tMat, poseGraph.nodeLocations[i], 2, Scalar(127), -1);
	}

	circle(tMat, poseGraph.nodeLocations[tspPosePath[1]], 2, Scalar(200), 3);




	namedWindow("ws view", WINDOW_NORMAL);
	imshow("ws view", tMat);
}

Point GraphPlanning::MCTSPosePathPlanning(float maxLength, Graph &graph, Costmap &costmap){

	findPosesEvolution(costmap); // find master states
	cout << "GraphPlanning::found graphPoses with " << poseGraph.nodeLocations.size() << " nodes" << endl;
	findPoseGraphTransitions(costmap); // get A* dist
	cout << "GraphPlanning::found poseGraphTransitions" << endl;
	displayPoseGraph(costmap);

	// MCTS posePath planning
	vector<int> path;
	int maxPulls = 1000;
	return poseGraph.nodeLocations[ mctsPathPlanner(path, maxLength, maxPulls) ];
}

int GraphPlanning::mctsPathPlanner(vector<int> &path, float maxLength, int maxPulls){

	TreeNode myTree(poseGraph.cNode, poseGraph.nodeTransitions, poseGraph.nodeObservations, path, 0, 0);
	while(myTree.nPulls < maxPulls){
	//	cerr << "mctsPathPlanner::into searchTree" << endl;
		myTree.searchTree(poseGraph.nodeTransitions, poseGraph.nodeObservations, maxLength, true);
	//	cerr << "out of searchTree" << endl;
	}
	vector<float> mctsPathValues;
	vector<float> mctsPathRewards;
	vector<float> mctsPathCosts;
	path.clear();
	myTree.exploitTree(path, mctsPathValues, mctsPathRewards, mctsPathCosts);
	//cerr << "out of loop" << endl;


	cout << "GraphPlanning::mctsPath: ";
	for(size_t i=0; i<path.size(); i++){
		cout << path[i] << ", ";
	}
	cout << endl;

	cout << "GraphPlanning::mctsPath.Locations: ";
	for(size_t i=0; i<path.size(); i++){
		cout << poseGraph.nodeLocations[ path[i] ].x << ", " << poseGraph.nodeLocations[ path[i] ].y << "; ";
	}
	cout << endl;
	cout << "GraphPlanning::mctsValues: ";
	for(size_t i=0; i<mctsPathValues.size(); i++){
		cout << mctsPathValues[i] << ", ";
	}
	cout << endl;

	cout << "GraphPlanning::mctsRewards: ";
	for(size_t i=0; i<mctsPathRewards.size(); i++){
		cout << mctsPathRewards[i] << ", ";
	}
	cout << endl;

	cout << "GraphPlanning::mctsCosts: ";
	for(size_t i=0; i<mctsPathCosts.size(); i++){
		cout << mctsPathCosts[i] << ", ";
	}
	cout << endl;

	return path[1];
}

///////////////////////////////////////////////////////////////////////////////////////
/* Retired Functions //////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////


void GraphPlanning::simulateObservation(vector<int> pose, Mat &resultingView, Costmap &costmap){
	// make perimeter of viewing circle fit on image
	for(size_t i=0; i<viewPerim.size(); i++){
		int px = pose[0] + viewPerim[i][0];
		int py = pose[1] + viewPerim[i][1];

		bool flag = true;
		while(flag){
			flag = false;
			if(px < 0){ // constrain points to be on the map
				float dx = pose[0] - px;
				float dy = pose[1] - py;

				float m = dy / dx;
				float b = pose[1]-m*pose[0];
				px = 0;
				py = b;
				flag = true;
			}
			else if(px >= costmap.nCols){
				float dx = pose[0] - px;
				float dy = pose[1] - py;

				float m = dy / dx;
				float b = pose[1]-m*pose[0];
				px = costmap.nCols-1;
				py = m*px + b;
				flag = true;
			}
			if(py < 0){
				float dx = pose[0] - px;
				float dy = pose[1] - py;

				float m = dy / dx;
				float b = pose[1]-m*pose[0];
				py = 0;
				px = (py-b)/m;
				flag = true;
			}
			else if(py >= costmap.nRows){
				float dx = pose[0] - px;
				float dy = pose[1] - py;

				float m = dy / dx;
				float b = pose[1]-m*pose[0];
				py = costmap.nRows-1;
				px = (py-b)/m;
				flag = true;
			}
		}

		// check visibility to all points on circle
		float dx = px - pose[0];
		float dy = py - pose[1];
		if(abs(dx) > abs(dy)){
			if(dx > 0){
				float m = dy/dx;
				float b = pose[1]-m*pose[0];

				int y0 = pose[1];
				for(int x0 = pose[0]; x0 < px; x0++){
					y0 = m*x0+b;
					if(costmap.cells[x0][y0] > costmap.infFree){
						break;
					}
					else if(costmap.cells[x0][y0] == costmap.infFree){
						resultingView.at<uchar>(x0,y0,0) = 255;
					}
				}
			}
			else{
				float m = dy/dx;
				float b = pose[1]-m*pose[0];

				int y0 = pose[1];
				for(int x0 = pose[0]; x0 > px; x0--){
					y0 = m*x0+b;
					if(costmap.cells[x0][y0] > costmap.infFree){
						break;
					}
					else if(costmap.cells[x0][y0] == costmap.infFree){
						resultingView.at<uchar>(x0,y0,0) = 255;
					}
				}
			}
		}
		else{
			if(dy > 0){
				float m = dx/dy;
				float b = pose[0]-m*pose[1];
				int x0 = pose[0];
				for(int y0 = pose[1]; y0 < py; y0++){
					x0 = m*y0+b;
					if(costmap.cells[x0][y0] > costmap.infFree){
						break;
					}
					else if(costmap.cells[x0][y0] == costmap.infFree){
						resultingView.at<uchar>(x0,y0,0) = 255;
					}
				}
			}
			else{
				float m = dx/dy;
				float b = pose[0]-m*pose[1];
				int x0 = pose[0];
				for(int y0 = pose[1]; y0 > py; y0--){
					x0 = m*y0+b;
					if(costmap.cells[x0][y0] > costmap.infFree){
						break;
					}
					else if(costmap.cells[x0][y0] == costmap.infFree){
							resultingView.at<uchar>(x0,y0,0) = 255;
					}
				}
			}
		}
	}

	//namedWindow("GraphPlanning::simulateView::view", WINDOW_NORMAL);
	//imshow("GraphPlanning::simulateView::view", resultingView);
	//waitKey(0);
}



vector<Node*> GraphPlanning::buildPath(Graph &graph, float maxDist){
	vector<Node*> path;
	path.push_back(graph.nodes[cState]);
	float remDist = maxDist;
	int cur = cState;
	while(remDist > 0){
		int nbr = getRandomNbr(cur, remDist);
		if(nbr >= 0){
			remDist = remDist - distGraph[cur][nbr];
			path.push_back(nbr);
			cur = nbr;
		}
		else{
			remDist = -1;
		}
	}
	return path;
}

void GraphPlanning::getNodeCosts(int cState){
	stateCost.erase(stateCost.begin(), stateCost.end());
	for(int i=0; i<(int)nmstates; i++){// for each state
		stateCost.push_back(distGraph[cState][i]);//.push_back(aStarDist(cState,i)); // get A* cost to each Frontier
	}
	cout << "stateCosts: ";
	for(int i=0;i<nmstates; i++){
		cout << "stateCost[" << i << "]: " << stateCost[i] << endl;
	}
}

void GraphPlanning::importFrontiers(vector<vector<int> > FrontierList){ // bring in Frontiers to GraphPlanning
	frontiers.clear();
	frontiers = FrontierList;
}

void GraphPlanning::importUAVLocations(vector<vector<int> > cLocList){ // bring in UAV locations to GraphPlanning
	cLocListMap.clear();
	cLocListMap = cLocList;
}

void GraphPlanning::getstateRewards(){
	stateReward.erase(stateReward.begin(), stateReward.end());
	stateFrontiers.erase(stateFrontiers.begin(), stateFrontiers.end());
	for(int i=0; i<nmstates; i++){// for each Frontier
		stateReward.push_back(0);
		vector<int> t;
		stateFrontiers.push_back(t);
	}
	for(int i=0; i<(int)frontiers.size(); i++){// for each Frontier
		vector<int> t;
		t.push_back(frontiers[i][1]);
		t.push_back(frontiers[i][0]);
		int a = findNeareststate(t); // find state closest to Frontier
		stateFrontiers[a].push_back(i);
		stateReward[a] += 50; // sub in froniter value
	}

	for(int i=0;i<nmstates; i++){
		cout << "stateFrontiers[" << i << "]: ";
		for(int j=0; j<stateFrontiers[i].size(); j++){
			cout << stateFrontiers[i][j] << ", ";
		}
		cout << endl;
	}
}




bool GraphPlanning::lineTraversabilityCheck(Mat &tSpace, vector<int> sPt, vector<int> fPt, int fValue){

	if(abs(fPt[0] - sPt[0]) == abs(fPt[1] - sPt[1])){ // larger change in x direction, count along x
		if(fPt[0] < sPt[0]){ // set order right
			vector<int> t = sPt;
			sPt = fPt;
			fPt = t;
			cout << "inv" << endl;
		}

		float m = float(fPt[1] - sPt[1]) / float(fPt[0] - sPt[0]);
		float b = float(fPt[1]) - m*float(fPt[0]);

		cout << "fPt: " << fPt[1] << " , " << fPt[0] << endl;
		cout << "sPt: " << sPt[1] << " , " << sPt[0] << endl;
		cout << "x: " << m << " , " << b << endl;

		Mat temp = tSpace;

		for(int x = sPt[0]+1; x<fPt[0]-1; x++){
			float tx = x;
			float ty = m*tx+b;
			int y = ty;
			temp.at<uchar>(y,x,0) = 127;
			imshow("zzz", temp);
			//waitKey(1);

			if(tSpace.at<uchar>(y,x,0) != fValue){
				cout << "false" << endl;
				return false;
			}
		}
		cout << "return true" << endl;
		return true;
	}
	else{
		if(fPt[1] < sPt[1]){ // set order right
			vector<int> t = sPt;
			sPt = fPt;
			fPt = t;
			cout << "inv" << endl;
		}
		float m = float(fPt[0] - sPt[0]) / float(fPt[1] - sPt[1]);
		float b = float(fPt[0]) - m*float(fPt[1]);

		cout << "y: " << m << " , " << b << endl;

		Mat temp = tSpace;

		for(int x = sPt[1]+1; x<fPt[1]-1; x++){
			int y = round(m*x+b);
			temp.at<uchar>(y,x,0) = 127;
			imshow("zzz", temp);
			//waitKey(1);

			if(tSpace.at<uchar>(x,y,0) != fValue){
				cout << "false" << endl;
				return false;
			}
		}
		cout << "return true" << endl;
		return true;
	}
}

bool GraphPlanning::bresenhamLineCheck(vector<int> cLoc, vector<int> cPt){
	float dx = cLoc[0] - cPt[0];
	float dy = cLoc[1] - cPt[1];

	float er = -1;
	float de = 1;
	if(dx != 0){
		de = abs(dy/dx);
	}
	int y = cLoc[1];
	for(int x = cLoc[0]; x<cPt[0]-1; x++){
		if(obstacleMat.at<uchar>(x,y,0)){
			return false;
		}
		er = er + de;
		if(er >= 0){
			y++;
			er--;
		}
	}
	return true;

}

bool GraphPlanning::bisectionCheck(vector<int> a, vector<int> b){
	if(cityBlockDist(a,b) > 2){ // do I bisect further?
		vector<int> c;
		c.push_back((a[0] + b[0])/2); // find midpoint
		c.push_back((a[1] + b[1])/2);
		if(miniImage.at<uchar>(c[0],c[1],0) == 0){ // is midpoint an obstacle?
			return false;
		}
		else{ // midpoint is not an obstacle
			if(bisectionCheck(a,c) && bisectionCheck(b,c)){
				return true;
			}
		}
	}
	else{ // end of bisection
		return true;
	}
}

void GraphPlanning::findPoseSet(Mat &costMat){

	cout << "Graph::getposeSet::poseSet.size(): " << poseSet.size() << endl;

	Mat cView = Mat::zeros(costMat.size(), CV_8UC1);
	int cReward = 0;

	int gReward = 0;
	for(int i=0; i<costMat.cols; i++){
		for(int j=0; j<costMat.rows; j++){
			if(costMat.at<uchar>(i,j) == 2){
				gReward++;
			}
		}
	}
	while(true){
		if(poseSet.size() == 0){ // initialize poseSet
			while(poseSet.size() == 0){
				vector<int> ps;
				ps.push_back( rand() % costMat.cols );
				ps.push_back( rand() % costMat.rows );
				if(costMat.at<uchar>(ps[0],ps[1]) < 10){
					Mat tView = Mat::zeros(cView.size(), CV_8UC1);
					simulateObservation(ps, tView, costMat);
					int tReward = matCount(tView);
					if(tReward > 0){
						poseSet.push_back(ps);
					}
				}
			}
		}
		else{
			vector<vector<int> > tSet = poseSet;
			poseSet.clear();
			for(size_t i=0; i<tSet.size(); i++){
				if(costMat.at<uchar>(tSet[i][0],tSet[i][1]) < 10){
					Mat tView = Mat::zeros(cView.size(), CV_8UC1);
					simulateObservation(tSet[i], tView, costMat);
					int tReward = matCount(tView);
					if(tReward > 0){
						poseSet.push_back(tSet[i]);
					}
				}
			}
		}
		if(poseSet.size() > 0){
			break;
		}
	}

	// create poseSetView
	for(size_t i=0; i<poseSet.size(); i++){
		Mat tView = Mat::zeros(cView.size(), CV_8UC1);
		simulateObservation(poseSet[i], tView, costMat);
		bitwise_or(cView, tView, cView);
	}
	cReward = matCount(cView);

	float entropy = float(cReward) / float(gReward);

	// erase poses that don't contribute
	int pi = rand() % poseSet.size();
	vector<vector<int> > tSet = poseSet;
	tSet.erase(tSet.begin()+pi);
	// create cView minus selected node
	Mat pView =  Mat::zeros(cView.size(), CV_8UC1);
	for(size_t i=0; i<tSet.size(); i++){
		Mat tView = Mat::zeros(cView.size(), CV_8UC1);
		simulateObservation(tSet[i], tView, costMat);
		bitwise_or(pView, tView, pView);
	}
	int pReward = matCount(pView);
	float pEntropy = float(pReward) / float(gReward);
	if(pEntropy >= 0.95*entropy){
		poseSet = tSet;
	}

	int cnt = 0;
	while(entropy < 1 && cnt < 1000){
		cnt++;
		// add new pose
		if(entropy < (rand() % 1000) / 500){
			//cout << "graph::poseSet::addPose" << endl;
			while(true){
				vector<int> ps;
				ps.push_back( rand() % costMat.cols );
				ps.push_back( rand() % costMat.rows );
				if(costMat.at<uchar>(ps[0],ps[1]) < 10){ // free or inferred free
					Mat tView = Mat::zeros(cView.size(), CV_8UC1);
					simulateObservation(ps, tView, costMat);
					bitwise_or(cView, tView, tView);
					int tReward = matCount(tView);
					if(tReward > cReward){
						poseSet.push_back(ps);
						break;
					}
				}
			}
		}
		else{
		// erase pose
			if(rand() % 1000 > 500){
				//cout << "graph::poseSet::gradientPose" << endl;
				int pi = rand() % poseSet.size();
				vector<vector<int> > tSet = poseSet;
				tSet.erase(tSet.begin()+pi);
				// create cView minus selected node
				Mat pView =  Mat::zeros(cView.size(), CV_8UC1);
				for(size_t i=0; i<tSet.size(); i++){
					Mat tView = Mat::zeros(cView.size(), CV_8UC1);
					simulateObservation(tSet[i], tView, costMat);
					bitwise_or(pView, tView, pView);
				}
				int pReward = matCount(pView);

				// get +/- x/y nbrs
				vector<vector<int> > tP;
				tP.push_back(poseSet[pi]);
				vector<int> ps = poseSet[pi];
				ps[1]++;
				tP.push_back(ps);
				ps = poseSet[pi];
				ps[1]--;
				tP.push_back(ps);
				ps = poseSet[pi];
				ps[0]++;
				tP.push_back(ps);
				ps = poseSet[pi];
				ps[0]--;
				tP.push_back(ps);
				int maxi;
				int maxv = -1;
				for(size_t i=0; i<tP.size(); i++){
					Mat tView = Mat::zeros(cView.size(), CV_8UC1);
					simulateObservation(tP[i], tView, costMat);
					bitwise_or(pView, tView, tView);
					int tReward = matCount(tView);
					if(tReward > maxv){
						maxv = tReward;
						maxi = i;
					}
				}
				tSet.push_back(tP[maxi]);
				poseSet = tSet;
			}
			else{ // move pose with +/- x/y gradient
				//cout << "graph::poseSet::erasePose" << endl;
				int pi = rand() % poseSet.size();
				vector<vector<int> > tSet = poseSet;
				tSet.erase(tSet.begin()+pi);

				// create cView minus selected node
				Mat pView =  Mat::zeros(cView.size(), CV_8UC1);
				for(size_t i=0; i<tSet.size(); i++){
					Mat tView = Mat::zeros(cView.size(), CV_8UC1);
					simulateObservation(tSet[i], tView, costMat);
					bitwise_or(pView, tView, pView);
				}
				int pReward = matCount(pView);
				float pEntropy = float(pReward) / float(gReward);
				if(pEntropy >= 0.95*entropy){
					poseSet = tSet;
				}
			}
		}
		// get cReward
		for(size_t i=0; i<poseSet.size(); i++){
			Mat tView = Mat::zeros(cView.size(), CV_8UC1);
			simulateObservation(poseSet[i], tView, costMat);
			bitwise_or(cView, tView, cView);
		}
		cReward = matCount(cView);
		entropy = float(cReward) / float(gReward);
	}

	cout << "Graph::getposeSet::entropy/poseSet.size()/cnt: " << entropy << " / " << poseSet.size() << " / " << cnt << endl;
	for(size_t i=0; i<poseSet.size(); i++){
		cout << poseSet[i][0] << " , " << poseSet[i][1] << endl;
		circle(cView,Point(poseSet[i][1],poseSet[i][0]),2,Scalar(127),-1);
	}
	namedWindow("Graph::getposeSet::cView", WINDOW_NORMAL);
	imshow("Graph::getposeSet::cView", cView);

	waitKey(0);

}

*/

