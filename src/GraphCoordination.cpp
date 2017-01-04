/*
 * GraphCoordination.cpp
 *
 *  Created on: Mar 14, 2016
 *      Author: andy
 */



#include "GraphCoordination.h"

using namespace cv;
using namespace std;

void buildGreedyTours(vector<vector<int> > &tours, vector<int> &oSet, vector<vector<float> > &distGraph);
void marketBids(vector<vector<int> > &tours, vector<int> &oSet, vector<int> &goalPoses, vector<float> &goalBids);
void getBids(vector< vector<int> > &tours, vector<int> &oSet, vector<vector<float> > distGraph, vector<int> &goalPoses, vector<float> &goalBids);
float getTourLength(vector<int> &tour, vector<vector<float> > &distGraph);
void swapPosesBetweenAgents(vector<vector<int> > &tours);
void transferPoses(vector<vector<int> > &tours);
void swapEntireTours(vector<vector<int> > &tours);
void evolveIndividualTours(vector<vector<int> > &tours, vector<vector<float> > &distGraph);
void evoluAlgTSP(vector<int> &tour, vector<vector<float> > &distGraph);
vector<vector<int> > tspEvolveTours(vector<vector<int> > &tours, vector<vector<float> > &distGraph);
void tspRecursiveSolver( vector<int> tour, float tourLength, vector<vector<float> > &distGraph, vector<int> oSet, vector<int> &minTour, float &minLength);
vector<int> getOpenPoses(vector<vector<int> > &tours, vector<vector<float> > &distGraph);

void prettyPrint2D(vector<vector<int> > &in);
void prettyPrint2D(vector<vector<float> > &in);

/*
void bruteForceTSP(vector<int> &tour, vector<vector<float> > &distGraph);
void bruteForce3(vector<int> tour, vector<vector<float> > distGraph);
void bruteForce4(vector<int> tour, vector<vector<float> > distGraph);
void bruteForce5(vector<int> tour, vector<vector<float> > distGraph);
void bruteForce6(vector<int> tour, vector<vector<float> > distGraph);
void bruteForce7(vector<int> tour, vector<vector<float> > distGraph);
*/

GraphCoordination::GraphCoordination(){
	nPullsEvolvePoseGraph = 1000;
	minObsForPose = 5;
}

void GraphCoordination::init( float obsRadius, float comRadius){
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

	temp =Mat::zeros(2*(comRadius + 1), 2*(comRadius + 1), CV_8UC1);
	cent.x = comRadius;
	cent.y = comRadius;
	circle(temp,cent,comRadius, Scalar(255));

	for(int i=0; i<temp.cols; i++){
		for(int j=0; j<temp.rows; j++){
			if(temp.at<uchar>(i,j,0) == 255){
				Point t(i-obsRadius, j-obsRadius);
				comPerim.push_back(t);
			}
		}
	}
}

void GraphCoordination::marketPoses( Costmap &costmap, Point cLoc, Point &gLoc, Market &market ){


	// update each agents bid assuming they took an optimal step
	for(size_t i=0; i<market.exploreCosts.size(); i++){
		market.exploreCosts[i] -= float(market.times[i]);
	}

	// am I closer to any poses in market than the current purchaser?
	for(size_t i=0; i<market.gLocs.size(); i++){
		//cerr << "GraphCoordination::marketPoses::A1" << endl;
		if( i != market.myIndex ){
			bool flag = false;
			//cerr << "GraphCoordination::marketPoses::A2" << endl;
			if( market.gLocs[i].x > 0 && market.gLocs[i].y > 0){
				//cerr << "GraphCoordination::marketPoses::A3" << endl;
				if(sqrt(pow(cLoc.x - market.gLocs[i].x,2) + pow(cLoc.y - market.gLocs[i].y,2)) <=  market.exploreCosts[i]){
					//cerr << "GraphCoordination::marketPoses::A4" << endl;
					if(costmap.aStarDist(market.gLocs[i], cLoc) - market.exploreCosts[i] > 0.1){
						//cerr << "out of aStarDist: true" << endl;
						flag = true; // I am not a* closer
					}
					else if(abs(costmap.aStarDist(market.gLocs[i], cLoc) - market.exploreCosts[i]) < 0.1 && market.myIndex < i){
						flag = true;
					}
				}
				else{ // I am not euclidian closer
					//cerr << "GraphCoordination::marketPoses::A5" << endl;
					flag = true;
				}
				//cerr << "GraphCoordination::marketPoses::A6" << endl;
			}
			//cerr << "GraphCoordination::marketPoses::A7" << endl;
			if(flag){ // they are closer, remove all reward from their goals
				Mat nView = Mat::zeros(costmap.cells.size(), CV_8UC1);
				simulateObservation( market.gLocs[i], nView, costmap);

				for(int j=0; j<costmap.reward.cols; j++){
					for(int k=0; k<costmap.reward.rows; k++){
						Point a(j,k);
						if( nView.at<uchar>(a) == 255){
							costmap.reward.at<float>(a) = 0;
						}
					}
				}
			}
		}
	}
	//

	//cerr << "GraphCoordination::marketPoses::B" << endl;


	vector<float> poseRewards;
	vector<float> poseDistances;
	vector<bool> trueDist;
	vector<float> poseValue;

	float maxVal = -INFINITY;
	int maxPose = -1;

	//cerr << "GraphCoordination::marketPoses::C" << endl;

	Mat gView = Mat::zeros(costmap.cells.size(), CV_8UC1);
	//cerr << "PoseGraph.nodeLocations: ";
	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		//cerr << poseGraph.nodeLocations[i] << ", ";
		Mat lView = Mat::zeros( costmap.cells.size(), CV_8UC1);
		simulateObservation( poseGraph.nodeLocations[i], lView, costmap );
		bitwise_or(gView, lView, gView);
	}
	//cerr << endl;
	float globalReward = observedReward( gView, costmap.reward );

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


	if(maxPose >= 0){
		//cerr << "GraphCoordination::marketPoses::D" << endl;

		while( true ){ // find best pose

			//cerr << "GraphCoordination::marketPoses::D1: maxPose = " << maxPose << endl;
			if( trueDist[maxPose]){
				break;
			}
			//cerr << "GraphCoordination::marketPoses::D2" << endl;

			poseDistances[maxPose] = costmap.aStarDist(cLoc, poseGraph.nodeLocations[maxPose]);
			//cerr << "GraphCoordination::marketPoses::D3" << endl;
			poseValue[maxPose] = poseRewards[maxPose] - 0.1*pow(poseDistances[maxPose],2);

			//cerr << "GraphCoordination::marketPoses::D4:poseValue[maxPose]: " << poseValue[maxPose] << endl;
			trueDist[maxPose] = true;

			maxVal = -INFINITY;
			maxPose = -1;
			for(size_t i=0; i<poseValue.size(); i++){
				if(poseValue[i] > maxVal){
					maxPose = i;
					maxVal = poseValue[i];
				}
			}
			//cerr << "GraphCoordination::marketPoses::D5" << endl;


			//cout << "maxPose: index, value, rewards, distance: " << maxPose << ", " << poseValue[maxPose] << ", " << poseRewards[maxPose] << ", " << poseDistances[maxPose] << endl;
		}

		//cerr << "GraphCoordination::marketPoses::gValue: " << poseValue[maxPose] << endl;
		//cerr << "GraphCoordination::marketPoses::goalPose: " << poseGraph.nodeLocations[maxPose] << endl;

		gLoc = poseGraph.nodeLocations[maxPose];

		market.gLocs[market.myIndex] = gLoc;
		market.exploreCosts[market.myIndex] = poseDistances[maxPose];

		//cerr << "GraphCoordination::marketPoses::Z" << endl;
	}
	else{
		gLoc = cLoc;
	}
}

void GraphCoordination::simulateNULLObservation(Point pose, Mat &resultingView, Costmap &costmap){
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
	namedWindow("GraphCoordination::simulateView::view", WINDOW_NORMAL);
	imshow("GraphCoordination::simulateView::view", fu);
	waitKey(0);
	*/
}

void GraphCoordination::getViews(Costmap &costmap, vector<int> cPoses, Mat &cView, float &cReward, vector<Point> &poses, vector<Mat> &views, vector<float> poseRewards, float &gReward){

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

void GraphCoordination::plotPoses( Costmap &costmap, vector<int> &cPoses, vector<int> &oPoses, vector<Mat> &views, vector<Point> &poses){

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


	namedWindow("Pose Set", WINDOW_NORMAL);
	imshow("Pose Set", gView);
	waitKey(1);
}

void GraphCoordination::findPosesEvolution(Costmap &costmap){

	//cout << "GraphCoordination::findPosesEvolution::A" << endl;

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

	//cout << "GraphCoordination::findPosesEvolution::C::cReward / gReward: " << cReward << " / " << gReward << endl;


	// add more nodes to pose graph if not fully observing world

	clock_t tStart1 = clock();
	int iter = 0;
	while(cReward < gReward && iter < 500){ // add states that increase the observed are until % is observed
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
	//printf("Time taken to create get poses: %.2fs\n", (double)(clock() - tStart1)/CLOCKS_PER_SEC);

	//cerr << "GraphCoordination::findPosesEvolution::D::cReward / gReward: " << cReward << " / " << gReward << endl;
	int cPost = cPoses.size();
	iter = 0;
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

	//plotPoses( costmap, cPoses, oPoses, poseViews , poseLocations);
	//waitKey(1);

	poseGraph.nodeLocations.clear();
	poseGraph.nodeObservations.clear();
	for(size_t i=0; i<cPoses.size(); i++){
		poseGraph.nodeLocations.push_back( poseLocations[ cPoses[i] ] );
		poseGraph.nodeObservations.push_back( poseViews[ cPoses[i] ] );
	}

	//cerr << "GraphCoordination::findPosesEvolution::E" << endl;
}

void GraphCoordination::getOset(vector<int> &oStates, vector<int> &cStates, vector<int> &workingSet){
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

float GraphCoordination::observedReward(Mat &observed, Mat &reward){
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

void GraphCoordination::simulateObservation(Point pose, Mat &resultingView, Costmap &costmap){
	// make perimeter of viewing circle fit on image

	for(size_t i=0; i<viewPerim.size(); i++){
		Point v(viewPerim[i].x + pose.x, viewPerim[i].y + pose.y);
		LineIterator it(resultingView, pose, v, 8, false);
		for(int i=0; i<it.count; i++, ++it){

			Point pp  = it.pos();

			if(costmap.cells.at<short>(pp) > costmap.unknown){
				break;
			}
			else{// if(costmap.cells.at<short>(x0, y0) == costmap.infFree){
				resultingView.at<uchar>(pp) = 255;
			}
		}
	}

	/*
	Mat fu = resultingView.clone();
	circle(fu, pose, 2, Scalar(127), -1, 8);
	namedWindow("GraphCoordination::simulateView::view", WINDOW_NORMAL);
	imshow("GraphCoordination::simulateView::view", fu);
	waitKey(0);
	*/
}

void GraphCoordination::simulateCommunication(Point pose, Mat &resultingView, Costmap &costmap){
	// make perimeter of viewing circle fit on image

	for(size_t i=0; i<comPerim.size(); i++){
		Point v(comPerim[i].x + pose.x, comPerim[i].y + pose.y);
		LineIterator it(resultingView, pose, v, 8, false);
		for(int i=0; i<it.count; i++, ++it){

			Point pp  = it.pos();

			if(costmap.cells.at<short>(pp) >= costmap.obsWall){
				break;
			}
			else{// if(costmap.cells.at<short>(x0, y0) == costmap.infFree){
				resultingView.at<uchar>(pp) = 255;
			}
		}
	}

	/*
	Mat fu = resultingView.clone();
	circle(fu, pose, 2, Scalar(127), -1, 8);
	namedWindow("GraphCoordination::simulateView::view", WINDOW_NORMAL);
	imshow("GraphCoordination::simulateView::view", fu);
	waitKey(0);
	*/
}

Point GraphCoordination::posePathPlanningTSP(Graph &graph, Costmap &costmap, vector<Point> &agentLocs, int &myIndex){
	Point gLoc = agentLocs[myIndex];

	findPoseGraphTransitions(graph, costmap); // get A* dist
	cout << "GraphCoordination::found poseGraphTransitions" << endl;

	displayPoseGraph(costmap);
	waitKey(10);

	tspPoseFullTourPlanner(agentLocs, myIndex);
	cerr << "GraphCoordination::out of tspPoseToursPlanner" << endl;

	displayPoseTours(costmap);
	waitKey(1);

	cout << "TSPPosePathPlanning::returning tspPoseTours[1]: " << poseGraph.nodeLocations[tspPoseTours[myIndex][1]].x << ", " << poseGraph.nodeLocations[tspPoseTours[myIndex][1]].y << endl;
	return poseGraph.nodeLocations[ tspPoseTours[myIndex][1] ];
}


void GraphCoordination::tspPoseFullTourPlanner(vector<Point> &agentLocs, int &myIndex){

	cout << "GraphCoordination::tspPoseFullTourPlanner::seeding tours" << endl;
	// initialize + seed paths
	if( tspPoseTours.size() > 1){
		for(size_t i=0; i<tspPoseToursLocations.size(); i++){
			if( tspPoseTours[i].size() > 1){
				vector<int> tempPath;
				tempPath.push_back(i); // reseed cNode
				cerr << "a" << endl;
				for(size_t k=1; k<tspPoseToursLocations[i].size(); k++){
					cerr << "b" << endl;
					for(size_t j=0; j<poseGraph.nodeLocations.size(); j++){
						cerr << "tspPoseToursLocations[i].size(): " << tspPoseToursLocations[i].size() << endl;
						cerr << "k: " << k << endl;
						cerr << "tspPoseToursLocations[i][k].x: " << tspPoseToursLocations[i][k].x << endl;
						cerr << "poseGraph.nodeLocations[j].x: " << poseGraph.nodeLocations[j].x << endl;
						if(tspPoseToursLocations[i][k] == poseGraph.nodeLocations[j]){
							cerr << "d" << endl;
							tempPath.push_back(j);
							break;
						}
					}
				}
				cerr << "e" << endl;
				tspPoseTours[i] = tempPath;
			}
		}
		cerr << "f" << endl;
	}
	else{ // tspPoseTours.size() == 0, so initialize
		for(size_t i=0; i<agentLocs.size(); i++){ // first time, initialize tspPoseTours
			vector<int> t(1,i); // seed in cNodes
			tspPoseTours.push_back(t);
		}
	}

	cout << "GraphCoordination::tspPoseFullTourPlanner::seeded tours" << endl;;
	prettyPrint2D(tspPoseTours);
	waitKey(0);

	// get openSet
	vector<int> openPoses = getOpenPoses(tspPoseTours, poseGraph.nodeTransitions);

	cout << "GraphCoordination::tspPoseFullTourPlanner::got open poses" << endl;
	cout << "   ";
	for(size_t i=0; i<openPoses.size(); i++){
		cout << openPoses[i] << ", ";
	}
	cout << endl;

	// greedily add openPoses to tours
	buildGreedyTours(tspPoseTours, openPoses, poseGraph.nodeTransitions);

	cout << "GraphCoordination::tspPoseFullTourPlanner::built greedy tours" << endl;
	prettyPrint2D(tspPoseTours);

	vector< vector<int> > bestTours = tspPoseTours;
	float bestMaxLength = -1;
	for(size_t i=0; i<bestTours.size(); i++){
		float l = getTourLength(bestTours[i], poseGraph.nodeTransitions);
		if(l > bestMaxLength){ // want the smallest max length
			bestMaxLength = l;
		}
	}

	cout << "bestMaxLength: " << bestMaxLength << endl;

	vector< vector<int> > workingTours = bestTours;
	float workingMaxLength = bestMaxLength;

	cout << "GraphCoordination::tspPoseFullTourPlanner::into sim annealing" << endl;
	float temperature = 50;
	nPullsTSP = 100;
	for(int i=0; i<nPullsTSP; i++){
		cout << "GraphCoordination::tspPoseFullTourPlanner::pulling tsp" << endl;
		cout << "workingTours: " << endl;
		prettyPrint2D(workingTours);

		cout << "GraphCoordination::tspPoseFullTourPlanner::into tspEvolveTours" << endl;
		vector<vector<int> > tempTours = tspEvolveTours(workingTours, poseGraph.nodeTransitions);
		cerr << "GraphCoordination::tspPoseFullTourPlanner::out of tspEvolveTours" << endl;

		float tempMaxLength = -1;
		for(size_t i=0; i<bestTours.size(); i++){ // get worst length tour of all agents
			float l = getTourLength(bestTours[i], poseGraph.nodeTransitions);
			if(l > tempMaxLength){
				tempMaxLength = l;
			}
		}

		/*
		cout << "GraphCoordination::tspPoseFullTourPlanner::tempMaxLength: " << tempMaxLength << endl;
		cout << "GraphCoordination::tspPoseFullTourPlanner::workingMaxLength: " << workingMaxLength << endl;
		cout << "GraphCoordination::tspPoseFullTourPlanner::bestMaxLength: " << bestMaxLength << endl;
		*/

		if( exp( (workingMaxLength - tempMaxLength)/temperature ) > (rand() % 1000)/1000 ){
			//cout << "GraphCoordination::tspPoseFullTourPlanner::new workingMaxLength: " << workingMaxLength << endl;
			workingTours = tempTours;
			workingMaxLength = tempMaxLength;
		}
		if(tempMaxLength < bestMaxLength){
			bestMaxLength = tempMaxLength;
			//cout << "GraphCoordination::tspPoseFullTourPlanner::new bestMaxLength: " << bestMaxLength << endl;
			bestTours = tempTours;
		}
		temperature = temperature * 0.995;
	}

	cout << "GraphCoordination::tspPoseFullTourPlanner::out of sim annealing" << endl;
	prettyPrint2D(bestTours);

	tspPoseTours = bestTours;
	tspPoseToursLocations.clear();

	for(size_t i=0; i<tspPoseTours.size(); i++){
		vector<Point> t;
		for(size_t j=0; j<tspPoseTours[i].size(); j++){
			t.push_back(poseGraph.nodeLocations[tspPoseTours[i][j]]);
		}
		tspPoseToursLocations.push_back(t);
	}
}

vector<int> getOpenPoses(vector<vector<int> > &tours, vector<vector<float> > &distGraph){
	vector<int> openPose;
	for(size_t d=0; d<distGraph.size(); d++){
		bool open = true;
		for(size_t t=0; t<tours.size(); t++){
			for(size_t at=0; at<tours[t].size(); at++){
				if(tours[t][at] == (int)d){
					open = false;
					break;
				}
			}
			if(!open){
				break;
			}
		}
		if(open){
			openPose.push_back(d);
		}
	}
	return openPose;
}


void buildGreedyTours(vector<vector<int> > &tours, vector<int> &oSet, vector<vector<float> > &distGraph){

	// track each agent's locations as the end of its tour

	// go through each agent's locations and select the pose it is closest to
		// check for conflict, can another agent get to that pose in less total tour length

	vector<int> goalPoses(tours.size(), -1); // holds current bids, 1 per agent
	vector<float> goalBids(tours.size(), -1); // holds current bids, 1 per agent

	while(oSet.size() > 0){ // while there are still open poses
		// add poses to agents tour that will get it searched the quickest with each agent bidding on each pose
		getBids(tours, oSet, distGraph, goalPoses, goalBids); // everyone bids on their closest pose
		marketBids(tours, oSet, goalPoses, goalBids); // bids are compared and added to tours / removed from oSet
		cout << "GraphCoordination::buildGreedyTours::oSet.size: " << oSet.size() << endl;
	}
}

void marketBids(vector<vector<int> > &tours, vector<int> &oSet, vector<int> &goalPoses, vector<float> &goalBids){
	/*
	cout << "GraphCoordination::marketBids::goalPoses at goalBids" << endl;
	for(size_t i=0; i<goalPoses.size(); i++){
		cout << goalPoses[i] << " at " << goalBids[i] << endl;
	}
	*/

	for(size_t i=0; i<goalPoses.size(); i++){  // one goal pose per agent, check to see if there is a conflict
		if(goalPoses[i] >= 0 && goalBids[i] != INFINITY){
			bool conflict = false;
			for(size_t j=0; j<goalPoses.size(); j++){ // check against each other agent for conflict
				if(goalPoses[i] == goalPoses[j] && i != j){ // is there a conflict
					conflict = true;
					if(goalBids[i] > goalBids[j]){ // did they outbid me?
						goalPoses[i] = -1; // i dont bid this round
						goalBids[i] = INFINITY; // i dont bid this round
						break;
					}
					else if(goalBids[i] == goalBids[j] && i < j){ // tied and they outrank me
						goalPoses[i] = -1; // I dont bid this round
						goalBids[i] = INFINITY; // I dont bid this round
						break;
					}
					else{ // doesn't matter there is a conflict
						goalPoses[j] = -1; // they don't bid this round
						goalBids[j] = INFINITY; // they don't bid this round
						conflict = false;
					}
				}
			}
			if( !conflict ){ // conflict and I lost
				//cout << "no conflict on " << goalPoses[i] << " with index " << i << endl;
				//cout << "GraphCoordination::marketBids::oSet.size(): " << oSet.size() << endl;
				for(size_t j=0; j<oSet.size(); j++){
					if(oSet[j] == goalPoses[i]){
						oSet.erase(oSet.begin()+j); // take off the open list
						break;
					}
				}
				tours[i].push_back(goalPoses[i]); // add to my tour
			}
		}
	}
}


void getBids(vector< vector<int> > &tours, vector<int> &oSet, vector<vector<float> > distGraph, vector<int> &goalPoses, vector<float> &goalBids){
	for(size_t i=0; i<tours.size(); i++){ // for all agents get closest open pose
		float minDist = INFINITY;
		int mindex = -1;
		for(size_t j=0; j<oSet.size(); j++){
			if(distGraph[ tours[i].back() ][ oSet[j] ] < minDist){ // from the end of my tour to all open poses
				minDist = distGraph[ tours[i].back() ][ oSet[j] ];
				mindex = oSet[j];
			}
		}
		goalPoses[i] = mindex;
		goalBids[i] = getTourLength(tours[i], distGraph) + minDist;
	}
}

float getTourLength(vector<int> &tour, vector<vector<float> > &distGraph){
	float length = 0;

	if(tour.size() < 2){
		return 0;
	}
	else{
		for(size_t i=1; i<tour.size(); i++){
			length += distGraph[tour[i]][tour[i-1]];
		}
		return length;
	}
}

vector<vector<int> > tspEvolveTours(vector<vector<int> > &tours, vector<vector<float> > &distGraph){

	vector<vector<int> > tempTours = tours;
	cout << "GraphCoordination::tspEvolveTours::tours.size(): " << tempTours.size() << endl;
	prettyPrint2D(tempTours);


	int maxPulls = 1;
	for(int pull=0; pull<maxPulls; pull++){

	int c = rand() % 1000;
	if(c < 400){
		swapPosesBetweenAgents(tempTours); // exchange, each give / take 1:n poses
		cout << "GraphCoordination::tspEvolveTours::tours.size() after swap poses: " << tempTours.size() << endl;
		prettyPrint2D(tempTours);
	}
	else if(c < 800){
		transferPoses(tempTours); // don't exchange, remove and give 1:n poses
		cout << "GraphCoordination::tspEvolveTours::tours.size() after transfer: " << tempTours.size() << endl;
		prettyPrint2D(tempTours);
	}
	else{
		swapEntireTours(tempTours);
		cout << "GraphCoordination::tspEvolveTours::tours.size(): after swap entire" << tempTours.size() << endl;
		prettyPrint2D(tempTours);
	}

	// evolveIndividualTours(tempTours, distGraph);

	// while evolving individual paths, maybe find nodes that add the largest cost and place
	//		them on the bench with added cost?
	// then other agents can pull from bench if their cost is less?

	}
	cerr << "A " << endl;

	return tempTours;
}

void swapPosesBetweenAgents(vector<vector<int> > &tours){

	int a1,a2;

	// should I check that a1 and a2 are viable for swap?
	a1 = rand() % tours.size();
	a2 = a1;
	while(a1 == a2){
		a2 = rand() % tours.size();
	}

	if(tours[a1].size() > 1 && tours[a2].size() > 1){ // both have at least 1

		int nSwaps = 1;

		if(rand() % 1000 > 500 && tours[a1].size() >= 2 && tours[a2].size() >= 2){
			nSwaps = 2;
		}

		for(int i=0; i<nSwaps; i++){
			int p1 = rand() % (tours[a1].size()-1) + 1;
			int p2 = rand() % (tours[a2].size()-1) + 1;
			int t = tours[a1][p1];
			tours[a1][p1] = tours[a2][p2];
			tours[a2][p2] = t;
		}
	}
}

void transferPoses(vector<vector<int> > &tours){

	int a1,a2;
	// should I check that a1 and a2 are viable for swap?
	a1 = rand() % tours.size();
	a2 = a1;
	while(a1 == a2){
		a2 = rand() % tours.size();
	}

	if(tours[a1].size() > 1 || tours[a2].size() > 1){ // one has at least 2
		float p = 1000 * (tours[a1].size()-1) / (tours[a1].size() + tours[a2].size() -2);

		int donor = a1;
		int recip = a2;
		if(rand() % 1000 > p){ // larger share you have the more likely you give
			donor = a2;
			recip = a1;
		}

		int t = rand() % (tours[donor].size()-1) + 1;

		tours[recip].push_back( tours[donor][t] );
		tours[donor].erase(tours[donor].begin() + t);
	}
}

void swapEntireTours(vector<vector<int> > &tours){

	int a1,a2;
	// should I check that a1 and a2 are viable for swap?
	a1 = rand() % tours.size();
	a2 = a1;
	while(a1 == a2){
		a2 = rand() % tours.size();
	}

	vector<int> t;
	for(size_t i=1; i<tours[a1].size(); i++){
		t.push_back( tours[a1][i] );
	}
	int t1 = tours[a1][0];
	tours[a1].clear();
	tours[a1].push_back(t1);

	for(size_t i=1; i<tours[a2].size(); i++){
		tours[a1].push_back(tours[a2][i]);
	}
	int t2 = tours[a2][0];

	tours[a2].clear();
	tours[a2].push_back(t2);
	for(size_t i=0; i<t.size(); i++){
		tours[a2].push_back(t[i]);
	}
}

void evolveIndividualTours(vector<vector<int> > &tours, vector<vector<float> > &distGraph){

	for(size_t i=0; i<tours.size(); i++){
		cout << "GraphCoordination::evolveIndividualTours::tours[" << i << "].size(): " << tours[i].size() << endl;
		if(tours[i].size() < 8){
			cout << "GraphCoordination::evolveIndividualTours::into brute force individual solver" << endl;
			cout << "GraphCoordination::evolveIndividualTours::tours[" << i << "].size(): " << tours[i].size() << endl;
			//bruteForceTSP( tours[i], distGraph );
			vector<int> temp;
			temp.push_back(tours[i][0]);
			float tempLength = 0;

			vector<int> oSet;
			for(size_t j=1; j<tours[i].size(); j++){
				oSet.push_back(tours[i][j]);
			}
			vector<int> minTour;
			float minLength = INFINITY;

			cout << "GraphCoordination::evolveIndividualTours::in tspAddNode" << endl;
			tspRecursiveSolver(temp, tempLength, distGraph, oSet, tours[i], minLength);
			cout << "GraphCoordination::evolveIndividualTours::out of tspAddNode with length: " << minLength << endl;
		}
		else{ // too many nodes for brute force, use simulated annealing

			float temp = 1000;
			int nPullsTSP = 5;
			vector<int> bestTour = tours[i];
			vector<int> workingTour = tours[i];
			float bestLength = getTourLength( workingTour, distGraph );
			float workingLength = bestLength;


			for(int iter=0; iter<nPullsTSP; iter++){
				vector<int> tempTour = workingTour;
				evoluAlgTSP(tempTour, distGraph);
				float tempLength = getTourLength( tempTour, distGraph );

				if( exp( (workingLength - tempLength)/temp ) > (rand() % 1000)/1000 ){
					workingTour = tempTour;
					workingLength = tempLength;
				}

				if(tempLength < bestLength){
					bestLength = tempLength;
					bestTour = tempTour;
				}

				temp = temp * 0.9999;
			}
		}
	}
}


void tspRecursiveSolver( vector<int> tour, float tourLength, vector<vector<float> > &distGraph, vector<int> oSet, vector<int> &minTour, float &minLength){

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

				tspRecursiveSolver(tour, tourLength, distGraph, oSet, minTour, minLength); // recursive call

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

void evoluAlgTSP(vector<int> &tour, vector<vector<float> > &distGraph){
	if(tour.size() > 2){
		int nSwaps = 1;
		if(rand() % 1000 > 500){ // swap 1
			nSwaps = 2;
		}

		for(int i=0; i<nSwaps; i++){
			int s1 = rand() % (tour.size()-1) + 1;
			int s2 = s1;
			while(s1 == s2){
				s2 = rand() % (tour.size()-1) + 1;
			}
			int t = tour[s1];
			tour[s1] = tour[s2];
			tour[s2] = t;
		}
	}
}

float GraphCoordination::getGraphObservations(Graph &graph, Costmap &costmap, Mat &gView, vector<int> &workingSet){
	graph.nodeObservations.clear();
	for(size_t i=0; i<graph.nodeLocations.size(); i++){
		Mat t = Mat::zeros(costmap.cells.size(), CV_8UC1);
		graph.nodeObservations.push_back(t);
	}
	for(size_t i=0; i<workingSet.size(); i++){ // simulate observations from all nodes
		simulateObservation(graph.nodeLocations[workingSet[i] ], graph.nodeObservations[workingSet[i] ], costmap);
		bitwise_or(gView, graph.nodeObservations[workingSet[i] ], gView);
	}
	return matReward(gView);
}

vector<int> GraphCoordination::getWorkingSet(Graph &graph, Costmap &costmap){
	vector<int> workingSet;
	for(size_t i = 0; i<graph.nodeLocations.size(); i++){
		float d = graph.aStarDist(i, graph.cNode, costmap);
		if( d < INFINITY && graph.nodeLocations[i] != graph.nodeLocations[graph.cNode]){ // reachable and not current node
			workingSet.push_back(i);
		}
	}
	return workingSet;
}

void GraphCoordination::findPosesEvolution(Graph &graph, Costmap &costmap, vector<Point> &agentLocs){

	vector<int> workingSet = getWorkingSet(graph, costmap);

	// get graph observations and global reward
	Mat gView = Mat::zeros(costmap.cells.size(), CV_8UC1);
	float gReward = getGraphObservations(graph, costmap, gView, workingSet);

	// get current reward, view
	Mat cView = Mat::zeros(costmap.cells.size(), CV_8UC1);
	float cReward = getCurrentPoseSetReward(graph, costmap, cView, workingSet);

	//  populate oStates and cStates
	vector<int> oStates;
	vector<int> cStates;
	getOset(graph, costmap, oStates, cStates, workingSet);

	//cout << "cReward: " << cReward << endl;
	//cout << "gReward: " << gReward << endl;


	if(cReward < 0.99*gReward){// does the old set cover the whole map?
		vector<int> bestStates;
		vector<int> workingstates;

		// add more nodes to pose graph if not fully observing world
		while(cReward < 0.999*gReward){ // add states that increase the observed are until 95% is observed
			int c = rand() % oStates.size();
			Mat tView;
			bitwise_or(cView, graph.nodeObservations[oStates[c]], tView);
			int tReward = matReward(tView);

			if(tReward > cReward){
				cStates.push_back(oStates[c]);
				cView = tView;
				cReward = tReward;
				oStates.erase(oStates.begin() + c);
			}
		}
		// initialize
		workingstates = cStates;
		bestStates = cStates;
		float bestReward = cReward;

		vector<int> tempList;
		for(int i=0; i<nPullsEvolvePoseGraph; i++){
			tempList = oStates;
			if(cStates.size() == 0){
				cout << "GraphCoordination::findPosesEvolution::Error: cStates.size() == 0" << endl;
				waitKey(0);
			}
			int oni = rand() % tempList.size();
			int cni = rand() % cStates.size();

			if(  rand() % 1000 > 500 ){ // swap a state with open set
				int swpIn = tempList[oni];
				int swpOut = cStates[cni];

				cStates.erase(cStates.begin() + cni);
				cStates.push_back(swpIn);
				tempList.erase(tempList.begin() + oni);
				tempList.push_back(swpOut);
			}
			else{ // erase a state
				int swap = cStates[cni];
				cStates.erase(cStates.begin()+cni);
				tempList.push_back(swap);
			}

			Mat cView = Mat::zeros(costmap.cells.size(), CV_8UC1);
			for(size_t i=1; i<cStates.size(); i++){
				bitwise_or(cView, graph.nodeObservations[cStates[i]], cView);
			}
			cReward = matReward(cView);
			bool flag = false;

			if(cReward > 0.999*gReward){
				if(cStates.size() < bestStates.size()){// && cReward >= bestReward){ // if less always take
					bestStates = cStates;
					bestReward = cReward;
				}
				if( cStates.size() < workingstates.size()){ // if less always take
					workingstates = cStates;
					flag = true;
				}
				else if( rand() % 1000 > 500 ){ // sometimes take anyways
					workingstates = cStates;
					flag = true;
				}
			}
			if(flag){ // update openStates
				oStates = tempList;
			}
			cStates = workingstates; // update cStates
		}
		tempGraph.nodeLocations.clear(); // ensure I start with a fresh list
		tempGraph.nodeObservations.clear();

		for(size_t i=0; i<agentLocs.size(); i++){ // seed in n agent locations as the first n poses in graph
			int cn = graph.findNearestNode(agentLocs[i], costmap);
			tempGraph.nodeLocations.push_back( graph.nodeLocations[cn] );
			tempGraph.nodeObservations.push_back( graph.nodeObservations[cn] );
		}
		Mat bView = Mat::zeros(costmap.cells.size(), CV_8UC1);
		for(size_t i=0; i<bestStates.size(); i++){
			tempGraph.nodeLocations.push_back( graph.nodeLocations[bestStates[i]] );
			tempGraph.nodeObservations.push_back( graph.nodeObservations[bestStates[i]] );
			bitwise_or(bView, graph.nodeObservations[bestStates[i]], bView);
		}
		cReward = matReward(bView);
	}

	poseGraph.nodeLocations.clear(); // ensure I start with a fresh list
	poseGraph.nodeObservations.clear();

	for(size_t i=0; i<agentLocs.size(); i++){
		poseGraph.nodeLocations.push_back( agentLocs[i] );
		int cn = graph.findNearestNode(agentLocs[i], costmap);
		poseGraph.nodeObservations.push_back( graph.nodeObservations[cn] );
	}

	for(size_t i=1; i<tempGraph.nodeLocations.size(); i++){
		float tReward = 0;
		if(poseGraph.nodeObservations.size() > 1){ // if poseGraph is populated compare against self
			Mat temp = Mat::zeros(poseGraph.nodeObservations[1].size(), CV_8UC1);
			for(size_t j=1; j<poseGraph.nodeObservations.size(); j++){ // get cReward without node
				if(i != j){
					bitwise_or(temp, poseGraph.nodeObservations[j], temp);
				}
			}

			bitwise_and(temp, tempGraph.nodeObservations[i], temp);
			bitwise_not(temp, temp);
			bitwise_and(temp, tempGraph.nodeObservations[i], temp);
			tReward = matReward(temp);

		}
		else{ // poseGraph is empty
			Mat temp = Mat::zeros(tempGraph.nodeObservations[1].size(), CV_8UC1);
			for(size_t j=1; j<tempGraph.nodeLocations.size(); j++){ // get cReward without node
				if(i != j){
					bitwise_or(temp, tempGraph.nodeObservations[j], temp);
				}
			}

			bitwise_and(temp, tempGraph.nodeObservations[i], temp);
			bitwise_not(temp, temp);
			bitwise_and(temp, tempGraph.nodeObservations[i], temp);
			tReward = matReward(temp);
		}

		if( tReward > this->minObsForPose ){ // has value
			//URGENT
			float d;// = costmap.aStarDist(tempGraph.nodeLocations, cLoc, costmap);
			if( d < INFINITY){ // reachable
				poseGraph.nodeLocations.push_back( tempGraph.nodeLocations[i] );
				poseGraph.nodeObservations.push_back( tempGraph.nodeObservations[i] );
			}
		}
	}


	cout << "GraphCoordination::findPosesEvolution::NodeLocations: ";
	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		cout << poseGraph.nodeLocations[i].x << ", " << poseGraph.nodeLocations[i].y << "; ";
	}
	cout << endl;
}

void GraphCoordination::getOset(Graph &graph, Costmap &costmap, vector<int> &oStates, vector<int> &cStates, vector<int> &workingSet){
	for(size_t i = 0; i<workingSet.size(); i++){
		bool flag = true;
		for(size_t j=0; j<tempGraph.nodeLocations.size(); j++){
			if(tempGraph.nodeLocations[j] == graph.nodeLocations[workingSet[i] ]){
				cStates.push_back(workingSet[i]);
				flag = false;
				break;
			}
		}
		if(flag){
			oStates.push_back(workingSet[i]);
		}
	}
}

float GraphCoordination::getCurrentPoseSetReward(Graph &graph, Costmap &costmap, Mat &cView, vector<int> &workingSet){
	// check if poseGraph nodes are still on graph, if yes import their view, else erase

	for(size_t j=0; j<tempGraph.nodeLocations.size(); j++){
		bool flag = true;
		for(size_t i = 0; i<workingSet.size(); i++){
			if(tempGraph.nodeLocations[j] == graph.nodeLocations[workingSet[i] ]){
				tempGraph.nodeObservations[j] = graph.nodeObservations[workingSet[i] ];

				bitwise_or(cView, graph.nodeObservations[workingSet[i] ], cView);
				flag = false;
				break;
			}
		}
		if(flag){
			tempGraph.nodeLocations.erase(tempGraph.nodeLocations.begin()+j);
			tempGraph.nodeObservations.erase(tempGraph.nodeObservations.begin()+j);
		}
	}
	return matReward(cView);
}

int GraphCoordination::matReward(Mat &in){
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

void GraphCoordination::displayPoseGraph(Costmap &costmap){
	Mat view = Mat::zeros(costmap.cells.size(), CV_8UC1);
	for(size_t i=1; i<poseGraph.nodeLocations.size(); i++){
		bitwise_or(view, poseGraph.nodeObservations[i], view);
	}
	for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
		circle(view, poseGraph.nodeLocations[i], 2, Scalar(127), -1);
		char buff[5];
		sprintf(buff, "%i", int(i) );
		putText(view, buff, poseGraph.nodeLocations[i], 0, 0.35, Scalar(127));
	}

	circle(view, poseGraph.nodeLocations[0], 3, Scalar(255), -1);
	char buff[5];
	sprintf(buff, "C");
	putText(view, buff, poseGraph.nodeLocations[0], 0, 0.35, Scalar(127));

	namedWindow("GraphCoordination::poseGraph.nodes.observations", WINDOW_NORMAL);
	imshow("GraphCoordination::poseGraph.nodes.observations", view);
}

void GraphCoordination::displayPoseTours(Costmap &costmap){
	// build plot to show off the path
	Mat tMat = Mat::zeros(costmap.cells.size(), CV_8UC1);;
	for(size_t i=1; i<poseGraph.nodeLocations.size(); i++){
		bitwise_or(tMat, poseGraph.nodeObservations[i], tMat);
	}

	for(size_t j=0; j<tspPoseTours.size(); j++){

		cout << "tspPoseTours[" << tspPoseTours[j].size() << "]: " << poseGraph.nodeLocations[tspPoseTours[j][0]].x << " / " << poseGraph.nodeLocations[tspPoseTours[j][0]].y;
		for(size_t i=1; i<tspPoseTours[j].size(); i++){
			cout << ", " << poseGraph.nodeLocations[tspPoseTours[j][i]].x << " / " << poseGraph.nodeLocations[tspPoseTours[j][i]].y;
			line(tMat, poseGraph.nodeLocations[tspPoseTours[j][i]],  poseGraph.nodeLocations[tspPoseTours[j][i-1]], Scalar(127), 1);
		}
		cout << endl;

		for(size_t i=0; i<poseGraph.nodeLocations.size(); i++){
			circle(tMat, poseGraph.nodeLocations[i], 2, Scalar(127), -1);
		}
		circle(tMat, poseGraph.nodeLocations[tspPoseTours[j][1]], 2, Scalar(200), 3);
	}
	namedWindow("ws view", WINDOW_NORMAL);
	imshow("ws view", tMat);
}

void GraphCoordination::findPoseGraphTransitions(Graph &graph, Costmap &costmap){
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
				float d = costmap.aStarDist(poseGraph.nodeLocations[i],poseGraph.nodeLocations[j]);
				poseGraph.nodeTransitions[i][j] = d;
				poseGraph.nodeTransitions[j][i] = d;
			}
		}
	}

	/*
	cout << "GraphCoordination::findPoseGraphTransitions::nodeTransitions array:" << endl;
	for(size_t i=0; i<poseGraph.nodeTransitions.size(); i++){
		cout << "   ";
		for(size_t j=0; j<poseGraph.nodeTransitions.size(); j++){
			cout <<poseGraph.nodeTransitions[i][j] << ", ";
		}
		cout << endl;
	}
	*/
}

void prettyPrint2D(vector<vector<int> > &in){
	for(size_t i=0; i<in.size(); i++){
		cout << "   ";
		for(size_t j=0; j<in[i].size(); j++){
			cout << in[i][j] << ", ";
		}
		cout << endl;
	}
}

void prettyPrint2D(vector<vector<float> > &in){
	for(size_t i=0; i<in.size(); i++){
		cout << "   ";
		for(size_t j=0; j<in[i].size(); j++){
			printf( "%.1f, ", in[i][j]);
		}
		cout << endl;
	}
}

GraphCoordination::~GraphCoordination() {

}

/*


void bruteForceTSP( vector<int> &tour, vector<vector<float> > &distGraph){

	if(tour.size() <= 2){
		// nothing to do...
	}
	else if(tour.size() == 3){
		bruteForce3(tour, distGraph);
	}
	else if(tour.size() == 4){
		bruteForce4(tour, distGraph);
	}
	else if(tour.size() == 5){
		bruteForce5(tour, distGraph);
	}
	else if(tour.size() == 6){
		bruteForce6(tour, distGraph);
	}
	else if(tour.size() == 7){
		bruteForce7(tour, distGraph);
	}
}

void bruteForce3(vector<int> tour, vector<vector<float> > distGraph){

	float minLength = INFINITY;
	vector<int> minTour;

	for(int i=1; i<3; i++){
		for(int j=1; j<3; j++){
			if(i != j){
				vector<int> t;
				t.push_back(tour[0]);
				t.push_back(tour[i]);
				t.push_back(tour[j]);
				float d = getTourLength( t, distGraph );
				if(d < minLength){
					minLength = d;
					minTour = t;
				}
			}
		}
	}

	tour = minTour;
}

void bruteForce4(vector<int> tour, vector<vector<float> > distGraph){

	float minLength = INFINITY;
	vector<int> minTour;

	vector<int> t(tour.size(), tour[0]);
	float tL = 0;

	for(int i=1; i<4; i++){
		t[1] = tour[i];
		tL = distGraph[t[0]][t[1]];
		for(int j=1; j<4; j++){
			if(i != j){
				t[2] = tour[j];
				tL += distGraph[t[1]][t[2]];
				if(tL < minLength){
					for(int k=1; k<4; k++){
						if(i != k && j != k){
							t[3] = tour[k];
							tL += distGraph[t[2]][t[3]];
							if(tL < minLength){
								minLength = tL;
								minTour = t;
							}
						}
					}
				}
			}
		}
	}

	tour = minTour;
}

void bruteForce5(vector<int> tour, vector<vector<float> > distGraph){

	float minLength = INFINITY;
	vector<int> minTour;

	vector<int> t(tour.size(), tour[0]);
	float tL = 0;

	for(int i=1; i<5; i++){
		t[1] = tour[i];
		tL = distGraph[t[0]][t[1]];
		for(int j=1; j<5; j++){
			if(i != j){
				t[2] = tour[j];
				tL += distGraph[t[1]][t[2]];
				if(tL < minLength){
					for(int k=1; k<5; k++){
						if(i != k && j != k){
							t[3] = tour[k];
							tL += distGraph[t[2]][t[3]];
							if(tL < minLength){
								for(int l=1; l<5; l++){
									if(i != l && j != l && k != l){
										t[4] = tour[l];
										tL += distGraph[t[3]][t[4]];
										if(tL < minLength){
											minLength = tL;
											minTour = t;
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}

	tour = minTour;
}

void bruteForce6(vector<int> tour, vector<vector<float> > distGraph){

	float minLength = INFINITY;
	vector<int> minTour;

	vector<int> t(tour.size(), tour[0]);
	float tL = 0;

	for(int i=1; i<6; i++){
		t[1] = tour[i];
		tL = distGraph[t[0]][t[1]];
		for(int j=1; j<6; j++){
			if(i != j){
				t[2] = tour[j];
				tL += distGraph[t[1]][t[2]];
				if(tL < minLength){
					for(int k=1; k<6; k++){
						if(i != k && j != k){
							t[3] = tour[k];
							tL += distGraph[t[2]][t[3]];
							if(tL < minLength){
								for(int l=1; l<6; l++){
									if(i != l && j != l && k != l){
										t[4] = tour[l];
										tL += distGraph[t[3]][t[4]];
										if(tL < minLength){
											for(int m=1; m<6; m++){
												if(i != m && j != m && k != m && l != m){
													t[5] = tour[m];
													tL += distGraph[t[4]][t[5]];
													if(tL < minLength){
														minLength = tL;
														minTour = t;
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}

	tour = minTour;
}

void bruteForce7(vector<int> tour, vector<vector<float> > distGraph){

	float minLength = INFINITY;
	vector<int> minTour;

	vector<int> t(tour.size(), tour[0]);
	float tL = 0;

	for(int i=1; i<7; i++){
		t[1] = tour[i];
		tL = distGraph[t[0]][t[1]];
		for(int j=1; j<7; j++){
			if(i != j){
				t[2] = tour[j];
				tL += distGraph[t[1]][t[2]];
				if(tL < minLength){
					for(int k=1; k<7; k++){
						if(i != k && j != k){
							t[3] = tour[k];
							tL += distGraph[t[2]][t[3]];
							if(tL < minLength){
								for(int l=1; l<7; l++){
									if(i != l && j != l && k != l){
										t[4] = tour[l];
										tL += distGraph[t[3]][t[4]];
										if(tL < minLength){
											for(int m=1; m<7; m++){
												if(i != m && j != m && k != m && l != m){
													t[5] = tour[m];
													tL += distGraph[t[4]][t[5]];
													if(tL < minLength){
														for(int n=1; n<7; n++){
															if(i != n && j != n && k != n && l != n && m != n){
																t[6] = tour[n];
																tL += distGraph[t[5]][t[6]];
																if(tL < minLength){
																	minLength = tL;
																	minTour = t;
																}
															}
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}

	tour = minTour;
}




float GraphCoordination::aStarDist(int strt, int gl){
	vector<int> cSet; // 1 means in closed set, 0 means not
	vector<int> oSet; // 1 means in open set, 0 means not
	vector<float> gScore; // known cost from initial state to n
	vector<float> fScore; // gScore + heuristic score (dist to goal + imposed cost)
	vector<int> cameFrom; // each square has a vector of the location it came from
	for(int i=0;i<this->nmstates; i++){
		cSet.push_back(0);
		oSet.push_back(0);
		cameFrom.push_back(0);
		gScore.push_back(INFINITY); // init scores to inf
		fScore.push_back(INFINITY); // init scores to inf
	}
	oSet[strt] = 1; // starting state has score 0
	gScore[strt] = 0; // starting state in open set
	fScore[strt] = gScore[strt] + this->distGraph[strt][gl];
	int foo = 1;
	int finishFlag = 0;
	while(foo>0 && finishFlag == 0){
		/////////////////// this finds state with lowest fScore and makes current
		float min = INFINITY;
		int iMin = 0;
		for(int i=0; i<this->nmstates; i++){
			if(oSet[i] > 0 && fScore[i] < min){
				min = fScore[i];
				iMin = i;
			}
		}
		int cLoc = iMin;
		/////////////////////// end finding current state
		if(cLoc == gl){ // if the current state equals goal, construct path
			finishFlag = 1;
			return gScore[gl];
		} ///////////////////////////////// end construct path
		oSet[cLoc] = 0;
		cSet[cLoc] = 1;
		for(int nbr=0; nbr<this->nmstates;nbr++){
			if(this->distGraph[cLoc][nbr] < 3){
				float tGScore;
				if(cSet[nbr] == 1){ // has it already been eval? in cSet
					continue;
				}
				tGScore = gScore[cLoc] + this->distGraph[nbr][gl]; // calc temporary gscore
				if(oSet[nbr] == 0){
					oSet[nbr] = 1;  // add nbr to open set
				}
				else if(tGScore >= gScore[nbr]){ // is temp gscore better than stored g score of nbr
					continue;
				}
				cameFrom[nbr] = cLoc;
				gScore[nbr] = tGScore;
				fScore[nbr] = gScore[nbr] + this->distGraph[gl][nbr];
			}
		}
		/////////////// end condition for while loop, check if oSet is empty
		foo = 0;
		for(int i=0; i<this->nmstates; i++){
			foo+= oSet[i];
		}
	}
	return INFINITY;
}
*/

/*
void GraphCoordination::buildTree(){

	int maxPulls = 2;

	Mat myVisibleMat = Mat::zeros(this->miniImage.rows, this->miniImage.cols,CV_8UC1);
	vector<int> t;
	t.push_back(this->graf[this->cState][0]);
	t.push_back(this->graf[this->cState][1]);

	this->observe(t,myVisibleMat);
	treestate myTree(true, this->distGraph, this->cState, myVisibleMat);
	myTree.value = this->matReward(myVisibleMat);
	while(myTree.nPulls < maxPulls){
		myTree.searchTree(myTree.value);
	}
	cout << "myState: " << myTree.myState << endl;
	cout << "children.size(): " << myTree.children.size() << endl;
	for(size_t i=0; i < myTree.children.size(); i++){
		cout << "   " << myTree.children[i].myState << endl;
	}
	waitKey(1);
}
*/