/*
 * BuildingTemplate.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: andy
 */

#include "BuildingTemplate.h"

using namespace std;

BuildingTemplate::BuildingTemplate(){

	shift_x = 0;
	shift_y = 0;
	rotateAngle = 0;
	stretch_x = 1;
	stretch_y = 1;

	workingShift_x = 0;
	workingShift_y = 0;
	workingRotateAngle = 0;
	workingStretch_x = 1;
	workingStretch_y = 1;

	reward = 0;
	workingReward = -INFINITY;
	bestReward = -INFINITY;

	pulls = 1;

}

BuildingTemplate::~BuildingTemplate() {}

void BuildingTemplate::initialAlignmentCheck( BuildingTemplate &gt, int align, Costmap &costmap){

	rotateAngle = float(align) * 7.2;
	getTestMaps( costmap, gt );
	evaluateAlignment( gt);

	if( reward > bestReward ){
		bestMap = fillMap.clone();
		bestReward = reward;
		bestRotateAngle = rotateAngle;
		bestShift_x = 0;
		bestShift_y = 0;
		bestStretch_x = 1;
		bestStretch_y = 1;
	}
	waitKey(0);
}


void BuildingTemplate::initialRotationCheckBisection( BuildingTemplate &gt, Costmap &costmap){

	/*
	namedWindow("initialRotationCheck::fillMat in", WINDOW_NORMAL);
	imshow("initialRotationCheck::fillMat in", fillMat);
	waitKey(1);

	namedWindow("initialRotationCheck::gt.fillMap in", WINDOW_NORMAL);
	imshow("initialRotationCheck::gt.fillMap in", gt.fillMap);
	waitKey(1);
	*/

	for(float theta=0; theta<360; theta += 90){

		rotateAngle = theta;
		getTestMaps( costmap, gt);
		evaluateAlignment( gt );

		/*
		namedWindow("initialRotationCheck::fillMap", WINDOW_NORMAL);
		imshow("initialRotationCheck::fillMap", fillMap);
		waitKey(0);
		*/

		/*
		cout << "initMyRansac::reward: " << reward << endl;
		cout << "initMyRansac::bestReward: " << bestReward << endl;
		waitKey(0);
		*/

		if( reward > bestReward ){
			bestMap = fillMap.clone();
			bestReward = reward;
			bestRotateAngle = rotateAngle;
			bestShift_x = 0;
			bestShift_y = 0;
			bestStretch_x = 1;
			bestStretch_y = 1;
		}
	}


	float theta = 90;
	while(theta > 5){
		//cout << "theta: " << theta << endl;
		theta /= 2;

		for(int i=-1; i<2; i+=2){

			rotateAngle = bestRotateAngle + theta * i;
			getTestMaps( costmap, gt);
			evaluateAlignment( gt );

			/*
			namedWindow("initialRotationCheck::fillMap", WINDOW_NORMAL);
			imshow("initialRotationCheck::fillMap", fillMap);
			waitKey(1);

			cout << "initMyRansac::reward: " << reward << endl;
			cout << "initMyRansac::bestReward: " << bestReward << endl;
			waitKey(0);
			*/

			if( reward > bestReward ){
				bestMap = fillMap.clone();
				bestReward = reward;
				bestRotateAngle = rotateAngle;
				bestShift_x = 0;
				bestShift_y = 0;
				bestStretch_x = 1;
				bestStretch_y = 1;
			}
		}
	}
}

void BuildingTemplate::initialRotationCheck( BuildingTemplate &gt, float steps, Costmap &costmap){

	/*
	namedWindow("initialRotationCheck::fillMat in", WINDOW_NORMAL);
	imshow("initialRotationCheck::fillMat in", fillMat);
	waitKey(1);

	namedWindow("initialRotationCheck::gt.fillMap in", WINDOW_NORMAL);
	imshow("initialRotationCheck::gt.fillMap in", gt.fillMap);
	waitKey(1);
	*/


	float dStep = 360 / steps;

	for(float theta=0; theta<360; theta += dStep){

		rotateAngle = theta;
		getTestMaps( costmap, gt);
		evaluateAlignment( gt );

		/*
		namedWindow("initialRotationCheck::fillMap", WINDOW_NORMAL);
		imshow("initialRotationCheck::fillMap", fillMap);
		waitKey(0);
		*/

		/*
		cout << "initMyRansac::error: " << error << endl;
		cout << "initMyRansac::minError: " << minError << endl;
		waitKey(0);
		*/

		if( reward > bestReward ){
			bestMap = fillMap.clone();
			bestReward = reward;
			bestRotateAngle = rotateAngle;
			bestShift_x = 0;
			bestShift_y = 0;
			bestStretch_x = 1;
			bestStretch_y = 1;
		}
	}
}

void BuildingTemplate::simulatedAnnealingIteration(Costmap &costmap, BuildingTemplate &groundTruth, float temperature){
	pulls++;
	modifyWorkingSet();
	getTestMaps( costmap, groundTruth );
	//lookForHullPivots( groundTruth );
	evaluateAlignment( groundTruth );
	simulatedAnnealingUpdate( temperature );
}


void BuildingTemplate::getBuildingContour( Costmap &costmap ){

	// get mat of building hull
	Mat buildMat = Mat::zeros(costmap.cells.size(), CV_8UC1 );

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			if(costmap.cells.at<uchar>(i,j) != costmap.unknown && costmap.cells.at<uchar>(i,j) != costmap.infWall){//costmap.obsFree || costmap.cells[i][j] == costmap.obsWall || costmap.cells[i][j] == costmap.infFree){
				buildMat.at<uchar>(i,j) = 255;
			}
		}
	}

	// get contour
	vector<vector<Point> > cont;
	vector<Vec4i> hier;

	Mat t = buildMat.clone();
	Mat t2 = Mat::zeros(t.size(), CV_8UC1);

	findContours( t, cont, hier, CV_RETR_TREE, CV_CHAIN_APPROX_NONE );
	drawContours( t2, cont, 0, Scalar(255), 1, 8);

	contour.points = cont[0];
	fillMap = buildMat;
	thinMap = t2;
}

void BuildingTemplate::getObservedContour( Costmap &costmap ){

	// get mat of building hull
	Mat buildMat = Mat::zeros(costmap.cells.size(), CV_8UC1 );

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			if(costmap.cells.at<uchar>(i,j) == costmap.obsFree || costmap.cells.at<uchar>(i,j) == costmap.obsWall){
				buildMat.at<uchar>(i,j) = 255;
			}
		}
	}

	// get contour
	vector<vector<Point> > cont;
	vector<Vec4i> hier;

	Mat t = buildMat.clone();
	Mat t2 = Mat::zeros(t.size(), CV_8UC1);

	findContours( t, cont, hier, CV_RETR_TREE, CV_CHAIN_APPROX_NONE );
	drawContours( t2, cont, 0, Scalar(255), 1, 8);

	contour.points = cont[0];
	fillMap = buildMat;
	thinMap = t2;
}


void BuildingTemplate::modifyWorkingSet(){

	/*
	rotateAngle = workingRotateAngle;
	shift_x = workingShift_x;
	shift_y = workingShift_y;
	stretch_x = workingStretch_x;
	stretch_y = workingStretch_y;

	int c = rand() % 5;
	if(c == 0){
		rotateAngle = workingRotateAngle + (rand() % 5) - 2;

		// constrain rotate angle
		if(rotateAngle >= 360){
			rotateAngle -= 360;
		}
		else if(rotateAngle < 0){
			rotateAngle += 360;
		}
	}
	else if(c==1){
		shift_x = workingShift_x + (rand() % 3) - 1;
	}
	else if(c==2){
		shift_y = workingShift_y + (rand() % 3) - 1;
	}
	else if(c==3){
		stretch_x = workingStretch_x + float(rand() % 101)/400 -0.125;
		// constrain stretch
		if( stretch_x < 0.05){
			stretch_x = 0.05;
		}
	}
	else if(c==4){
		stretch_y = workingStretch_y + float(rand() % 101)/400 -0.125;
		if( stretch_y < 0.05){
			stretch_y = 0.05;
		}
	}
	*/

	rotateAngle = workingRotateAngle + (rand() % 5) - 2;
	shift_x = workingShift_x + (rand() % 3) - 1;
	shift_y = workingShift_y + (rand() % 3) - 1;
	stretch_x = workingStretch_x + float(rand() % 101)/400 -0.125;
	stretch_y = workingStretch_y + float(rand() % 101)/400 -0.125;

	// constrain rotate angle
	if(rotateAngle >= 360){
		rotateAngle -= 360;
	}
	else if(rotateAngle < 0){
		rotateAngle += 360;
	}

	if( stretch_x < 0.05){
		stretch_x = 0.05;
	}
	if( stretch_y < 0.05){
		stretch_y = 0.05;
	}
	/*
	cout << "j: " << j << endl;
	cout << "rotateAngle: " << rotateAngle << endl;
	cout << "shift_x/y: " << shift_x << " / " << shift_y << endl;
	cout << "stretch_x/y: " << stretch_x << " / " << stretch_y << endl;
	*/

}

void BuildingTemplate::resetUCB(){
	pulls = 0;
	workingReward = bestReward;
	workingRotateAngle = bestRotateAngle;
	workingShift_x = bestShift_x;
	workingShift_y = bestShift_y;
	workingStretch_x = bestStretch_x;
	workingStretch_y = bestStretch_y;
}

void BuildingTemplate::getTestMaps(Costmap &costmap, BuildingTemplate &groundTruth){
	Mat ransacMat = fillMat.clone();

	/*
	namedWindow("BuildingTemplate::getWorkingMats::ransacMat in", WINDOW_NORMAL);
	imshow("BuildingTemplate::getWorkingMats::ransacMat in", ransacMat);
	waitKey(1);
	*/

	// reset thinMat and fillMat
	thinMap = Mat::zeros(costmap.cells.size(), CV_8UC1);
	fillMap = Mat::zeros(costmap.cells.size(), CV_8UC1);

	// warp the matched contour
	Mat rotMatrix = getRotationMatrix2D(contour.center, rotateAngle, 1);
	warpAffine(ransacMat, ransacMat, rotMatrix, wallMap.size());
	resize(ransacMat, ransacMat, Size(), stretch_x, stretch_y);
	threshold(ransacMat, ransacMat, 5, 255, THRESH_BINARY);

	/*
	namedWindow("BuildingTemplate::getWorkingMats::ransacMat warped", WINDOW_NORMAL);
	imshow("BuildingTemplate::getWorkingMats::ransacMat warped", ransacMat);
	waitKey(1);
	*/

	// pull the warped contour
	vector<vector<Point> > ransacCont;
	vector<Vec4i> tHier;
	findContours(ransacMat, ransacCont, tHier, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	//cout << "contour.center: " << contour.center.x << " , " << contour.center.y << endl;
	//cout << "groundTruth.contour.center: " << groundTruth.contour.center.x << " , " << groundTruth.contour.center.y << endl;
	//cout << "shift: " << shift_x << " , " << shift_y << endl;

	// shift and place in the map
	Point p(groundTruth.contour.center.x - contour.center.x + shift_x, groundTruth.contour.center.y - contour.center.y + shift_y);
	//cout << "p: " << px << " , " << py << endl;
	//cout << "ransacMat: " << ransacMat.rows << " , " << ransacMat.cols << endl;

	drawContours(thinMap, ransacCont, 0, Scalar(255), 1, 8, noArray(), INT_MAX, p );
	drawContours(fillMap, ransacCont, 0, Scalar(255), -1, 8, noArray(), INT_MAX, p );

	/*
	namedWindow("building fillMap", WINDOW_NORMAL);
	imshow("building fillMap", fillMap);
	waitKey(1);

	namedWindow("building ransac thin match", WINDOW_NORMAL);
	imshow("building ransac thin match", thinMap);
	waitKey(1);

	circle( fillMat, contour.center, 3, Scalar(50), -1, 8);

	namedWindow("contour.thinMat", WINDOW_NORMAL);
	imshow("contour.thinMat", thinMat);
	waitKey(1);

	namedWindow("contour.filledMat", WINDOW_NORMAL);
	imshow("contour.filledMat", fillMat);
	waitKey(1);
	*/
}

void BuildingTemplate::lookForHullPivots(BuildingTemplate &groundTruth){

	/*
	namedWindow("groundTruth.wallMap", WINDOW_NORMAL);
	imshow("groundTruth.wallMap", groundTruth.wallMap);
	waitKey(1);

	namedWindow("thinMap", WINDOW_NORMAL);
	imshow("thinMap", thinMap);
	waitKey(1);
	*/

	Mat t = Mat::zeros(thinMap.size(), CV_8UC1);
	bitwise_not(thinMap, t);

	/*
	namedWindow("not(thinMap)", WINDOW_NORMAL);
	imshow("not(thinMap)", t);
	waitKey(1);
	*/

	bitwise_and(groundTruth.wallMap, t, t);

	namedWindow("gt.wallMap & not(thinMap)", WINDOW_NORMAL);
	imshow("gt.wallMap & not(thinMap)", t);
	waitKey(1);

	Mat tLines = Mat::zeros(t.size(), CV_8UC1);
	vector<Vec4i> lines;
	HoughLinesP(t, lines, 1, CV_PI/180, 5, 5, 10);
	for( size_t i = 0; i < lines.size(); i++ ){
	    Vec4i l = lines[i];
	    line( tLines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 1, 8);
	}

	namedWindow("detected lines", WINDOW_NORMAL);
	imshow("detected lines", tLines);
	waitKey(1);

	// get line slope
	vector<Vec3f> lineSlope;
	for( size_t i=0; i <lines.size(); i++){
		Vec4i l = lines[i];
		Vec3f m;
		m[0] = float(l[0] - l[2]) / float(l[1] - l[3]);
		m[1] = float( l[1] ) / ( m[0]*float(l[0]) );
		m[2] = 1;
		lineSlope.push_back(m);
	}

	// find near parrallel lines
	for( size_t i=0; i<lines.size()-1; i++){
		for(size_t j=i; j<lines.size(); j++){
			if( abs( abs(lineSlope[i][0]) - abs(lineSlope[j][0]) ) < 0.25){
				lineSlope[i][2] = -1;
				lineSlope[j][2] = -1;
			}
		}
	}



}


void BuildingTemplate::evaluateAlignment(BuildingTemplate &groundTruth){

	float errorThin = 0;//evalPointsBitwiseAndThin( groundTruth.thinMap, thinMap );
	float errorFill = evalPointsBitwiseAndFill( groundTruth.fillMap, fillMap );
	float errorObsWalls = evalPointsBitwiseAndThin( groundTruth.wallMap, thinMap );
	float errorObsSpace = evalPointsBitwiseAndObservedFill( groundTruth.obsMap, fillMap );

	float rewardGeoInfer = 2 - (errorThin / groundTruth.thinCnt + errorFill / groundTruth.fillCnt);
	float rewardObserved = 2 - (errorObsWalls / groundTruth.wallCnt + errorObsSpace / groundTruth.obsCnt);

	reward = rewardObserved + rewardGeoInfer;

	/*
	namedWindow("fillMap", WINDOW_NORMAL);
	imshow("fillMap", fillMap);

	cout << "errorObsSpace: " << errorObsSpace << " / " << groundTruth.obsCnt << " = " << errorObsSpace / groundTruth.obsCnt << endl;
	cout << "errorObsWalls: " << errorObsWalls << " / " << groundTruth.wallCnt << " = " << errorObsWalls / groundTruth.wallCnt << endl;
	cout << "errorThin: " << errorThin << " / " << groundTruth.thinCnt << " = " << errorThin / groundTruth.thinCnt  << endl;
	cout << "errorFill: " << errorFill << " / " << groundTruth.fillCnt << " = " << errorFill / groundTruth.fillCnt  << endl;
	cout << "reward: " << reward << endl;
	//waitKey(0);
	*/
}

void BuildingTemplate::simulatedAnnealingUpdate(float temperature){
	/*
	cout << "reward: " << reward << endl;
	cout << "workingReward: " << workingReward << endl;
	cout << "bestReward: " << bestReward << endl;
	cout << "P: " << exp( (reward - workingReward)/temperature ) << endl;
	waitKey(0);
	*/

	if( exp( (reward - workingReward)/temperature ) > float(rand() % 1000)/1000 ){
		workingReward = reward;

		workingRotateAngle = rotateAngle;

		workingShift_x = shift_x;
		workingShift_y = shift_y;

		workingStretch_x = stretch_x;
		workingStretch_y = stretch_y;
	}

	if(reward > bestReward){
		bestMap = fillMap.clone();
		bestReward = reward;

		bestRotateAngle = rotateAngle;

		bestShift_x = shift_x;
		bestShift_y = shift_y;

		bestStretch_x = stretch_x;
		bestStretch_y = stretch_y;
	}
}


float BuildingTemplate::evalPointsBitwiseAndFill(Mat &gt, Mat &test){

	/*
	namedWindow("gt", WINDOW_NORMAL);
	imshow("gt", gt);
	waitKey(1);

	namedWindow("test", WINDOW_NORMAL);
	imshow("test", test);
	waitKey(1);
	*/

	Mat t = Mat::zeros(test.size(), CV_8UC1);
	bitwise_not(test, t);

	/*
	namedWindow("t", WINDOW_NORMAL);
	imshow("t", t);
	waitKey(1);
	*/

	bitwise_and(gt, t, t);

	/*
	namedWindow("t'", WINDOW_NORMAL);
	imshow("t'", t);
	waitKey(1);
	*/

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

	/*
	namedWindow("t2", WINDOW_NORMAL);
	imshow("t2", t2);
	waitKey(1);
	*/

	bitwise_and(test, t2, t2);

	/*
	namedWindow("t2'", WINDOW_NORMAL);
	imshow("t2'", t2);
	waitKey(1);
	*/

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

float BuildingTemplate::evalPointsBitwiseAndObservedFill(Mat &gt, Mat &test){

	/*
	namedWindow("gt", WINDOW_NORMAL);
	imshow("gt", gt);
	waitKey(1);

	namedWindow("test", WINDOW_NORMAL);
	imshow("test", test);
	waitKey(1);
	*/

	Mat t = Mat::zeros(test.size(), CV_8UC1);
	bitwise_not(test, t);

	/*
	namedWindow("t", WINDOW_NORMAL);
	imshow("t", t);
	waitKey(1);
	*/

	bitwise_and(gt, t, t);

	/*
	namedWindow("t'", WINDOW_NORMAL);
	imshow("t'", t);
	waitKey(1);
	*/

	float observed = 0;
	for(int i=0; i<t.rows; i++){
		for(int j=0; j<t.cols; j++){
			if(t.at<uchar>(i,j,0) > 0){
				observed++;
			}
		}
	}

	/*
	cout << "observed: " << observed << endl;
	waitKey(0);
	*/
	return observed;
}


float BuildingTemplate::evalPointsBitwiseAndThin(Mat &gt, Mat &test){

	/*
	namedWindow("gt", WINDOW_NORMAL);
	imshow("gt", gt);
	waitKey(1);

	namedWindow("test", WINDOW_NORMAL);
	imshow("test", test);
	waitKey(1);
	*/

	Mat t = Mat::zeros(test.size(), CV_8UC1);
	bitwise_not(test, t);

	/*
	namedWindow("t", WINDOW_NORMAL);
	imshow("t", t);
	waitKey(1);
	*/

	bitwise_and(gt, t, t);
	/*
	namedWindow("t'", WINDOW_NORMAL);
	imshow("t'", t);
	waitKey(1);
	*/
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
