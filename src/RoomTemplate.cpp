/*
 * RoomTemplate.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: andy
 */

#include "RoomTemplate.h"

RoomTemplate::RoomTemplate() {

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

RoomTemplate::~RoomTemplate() {}

void RoomTemplate::getWallMat(Point offset, int maxS){

	wallMat = Mat::zeros(2*maxS, 2*maxS, CV_8UC1);
	for(size_t i=0; i<wallList.size(); i++){
		circle(wallMat, Point(wallList[i].x*5 + offset.x, wallList[i].y*5 + offset.y), 1, 255, -1, 8);
	}
}


void RoomTemplate::getGroundTruthRoomMaps( Costmap &costmap, Contour &room){

	getGroundTruthWallMap( costmap, room);
	getGroundTruthObsMap( costmap, room );
	getGroundTruthFillMap( costmap, room );
	getGroundTruthThinMap( costmap, room );
	contour = room;

}

void RoomTemplate::getGroundTruthThinMap(Costmap &costmap, Contour &room){

	thinMap = Mat::zeros(costmap.cells.size(), CV_8UC1);
	vector<vector<Point> > temp;
	temp.push_back( room.points );

	drawContours(thinMap, temp, 0, Scalar(255), 1, 8);
}

void RoomTemplate::getGroundTruthFillMap(Costmap &costmap, Contour &room){

	fillMap = Mat::zeros(costmap.cells.size(), CV_8UC1);
	vector<vector<Point> > temp;
	temp.push_back( room.points );

	drawContours(fillMap, temp, 0, Scalar(255), -1, 8);
}

void RoomTemplate::getGroundTruthObsMap(Costmap &costmap, Contour &room){
	obsMap = Mat::zeros(costmap.cells.size(), CV_8UC1);
	Mat tempMap = Mat::zeros(costmap.cells.size(), CV_8UC1);
	vector<vector<Point> > temp;
	temp.push_back( room.points );

	drawContours(tempMap, temp, 0, Scalar(255), -1, 8);

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			if( costmap.cells.at<uchar>(i,j) == costmap.obsFree || costmap.cells.at<uchar>(i,j) == costmap.obsWall){
				if(tempMap.at<uchar>(i,j) == 255){
					obsMap.at<uchar>(i,j) = 255;
				}
			}
		}
	}

}

void RoomTemplate::getGroundTruthWallMap(Costmap &costmap, Contour &room){
	wallMap = Mat::zeros(costmap.cells.size(), CV_8UC1);
	Mat tempMap = Mat::zeros(costmap.cells.size(), CV_8UC1);
	vector<vector<Point> > temp;
	temp.push_back( room.points );

	drawContours(tempMap, temp, 0, Scalar(255), 3, 8);
	drawContours(tempMap, temp, 0, Scalar(255), -1, 8);

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			if(costmap.cells.at<uchar>(i,j) == costmap.obsWall || costmap.cells.at<uchar>(i,j) == costmap.infWall){
				//if(tempMap.at<uchar>(i,j) == 255){
					wallMap.at<uchar>(i,j) = 255;
				//}
			}
		}
	}

}


float RoomTemplate::initialRotationCheck( RoomTemplate &gt, float steps, Costmap &costmap){

	/*
	namedWindow("initialRotationCheck::fillMat in", WINDOW_NORMAL);
	imshow("initialRotationCheck::fillMat in", fillMat);
	waitKey(1);

	namedWindow("initialRotationCheck::gt.fillMap in", WINDOW_NORMAL);
	imshow("initialRotationCheck::gt.fillMap in", gt.fillMap);
	waitKey(1);
	*/


	float dStep = 360 / steps;

	for(float theta=0; theta < 360; theta += dStep){

		rotateAngle = theta;
		getTestMaps( costmap, gt );
		evaluateAlignment( gt );

		/*
		cout << "initMyRansac::reward: " << reward << endl;
		waitKey(0);
		*/

		if( reward > bestReward ){
			bestMap = fillMap.clone();
			bestReward = reward;
			bestRotateAngle = theta;
			bestShift_x = 0;
			bestShift_y = 0;
			bestStretch_x = 1;
			bestStretch_y = 1;
		}
	}
	return bestReward;
}

void RoomTemplate::initialRotationCheckBisection( RoomTemplate &gt, Costmap &costmap){


	namedWindow("initialRotationCheck::fillMat in", WINDOW_NORMAL);
	imshow("initialRotationCheck::fillMat in", fillMat);
	waitKey(1);

	namedWindow("initialRotationCheck::gt.fillMap in", WINDOW_NORMAL);
	imshow("initialRotationCheck::gt.fillMap in", gt.fillMap);
	waitKey(1);



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

void RoomTemplate::simulatedAnnealingIteration(Costmap &costmap, RoomTemplate &groundTruth, float temperature){
	cerr << "!" << endl;
	pulls++;
	cerr << "@" << endl;
	modifyWorkingSet();
	cerr << "#" << endl;
	getTestMaps( costmap, groundTruth );
	cerr << "$" << endl;
	evaluateAlignment( groundTruth );
	cerr << "%" << endl;
	simulatedAnnealingUpdate( temperature );
	cerr << "^" << endl;
}

void RoomTemplate::modifyWorkingSet(){

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
		shift_x = workingShift_x + (rand() % 5) - 2;
	}
	else if(c==2){
		shift_y = workingShift_y + (rand() % 5) - 2;
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
	cerr << "rotateAngle: " << rotateAngle << endl;
	cerr << "shift_x/y: " << shift_x << " / " << shift_y << endl;
	cerr << "stretch_x/y: " << stretch_x << " / " << stretch_y << endl;
	*/
}

void RoomTemplate::resetUCB(){
	pulls = 1;
	workingReward = bestReward;
	workingRotateAngle = bestRotateAngle;
	workingShift_x = bestShift_x;
	workingShift_y = bestShift_y;
	workingStretch_x = bestStretch_x;
	workingStretch_y = bestStretch_y;
}

void RoomTemplate::getTestMaps(Costmap &costmap, RoomTemplate &groundTruth){
	Mat ransacMat = fillMat.clone();
	Mat ransacWall = wallMat.clone();


	namedWindow("RoomTemplate::getWorkingMats::ransacMat in", WINDOW_NORMAL);
	imshow("RoomTemplate::getWorkingMats::ransacMat in", ransacMat);
	waitKey(1);

	namedWindow("RoomTemplate::getWorkingMats::ransacWall in", WINDOW_NORMAL);
	imshow("RoomTemplate::getWorkingMats::ransacWall in", ransacWall);
	waitKey(1);

	// reset thinMat and fillMat
	fillMap = Mat::zeros(costmap.cells.size(), CV_8UC1);
	wallMap = Mat::zeros(costmap.cells.size(), CV_8UC1);

	// warp the matched contour
	Mat rotMatrix = getRotationMatrix2D(contour.center, rotateAngle, 1);
	resize(ransacMat, ransacMat, Size(), stretch_x, stretch_y);
	warpAffine(ransacMat, ransacMat, rotMatrix, wallMap.size());
	threshold(ransacMat, ransacMat, 5, 255, THRESH_BINARY);

	resize(ransacWall, ransacWall, Size(), stretch_x, stretch_y);
	warpAffine(ransacWall, ransacWall, rotMatrix, wallMap.size());
	threshold(ransacWall, ransacWall, 5, 255, THRESH_BINARY);


	namedWindow("RoomTemplate::getWorkingMats::ransacMat warped", WINDOW_NORMAL);
	imshow("RoomTemplate::getWorkingMats::ransacMat warped", ransacMat);
	waitKey(1);

	namedWindow("RoomTemplate::getWorkingMats::ransacWall warped", WINDOW_NORMAL);
	imshow("RoomTemplate::getWorkingMats::ransacWall warped", ransacWall);
	waitKey(0 );

	// pull the warped contour
	vector<vector<Point> > ransacCont;
	vector<Vec4i> tHier;
	findContours(ransacMat, ransacCont, tHier, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	/*
	cout << "contour.center: " << contour.center.x << " , " << contour.center.y << endl;
	cout << "groundTruth.contour.center: " << groundTruth.contour.center.x << " , " << groundTruth.contour.center.y << endl;
	cout << "shift: " << shift_x << " , " << shift_y << endl;
	*/

	// shift and place in the map
	int px = groundTruth.contour.center.x - contour.center.x + shift_x;
	int py = groundTruth.contour.center.y - contour.center.y + shift_y;


	//cerr << "p: " << px << " , " << py << endl;

	drawContours(fillMap, ransacCont, 0, Scalar(255), -1, 8, noArray(), INT_MAX, Point(px ,py) );

	namedWindow("getTest maps::room fillMap", WINDOW_NORMAL);
	imshow("getTest maps::room fillMap", fillMap);
	waitKey(1);


	wallMap = Mat::zeros(fillMap.size(), CV_8UC1);
	for(int i=0; i<ransacWall.rows; i++){
		for(int j=0; j<ransacWall.cols; j++){
			if(ransacWall.at<uchar>(i,j) == 255){
				if(i+px < wallMap.rows && i+px >= 0 && j+py < wallMap.cols && j+py >= 0){
					wallMap.at<uchar>(i+px, j+py) = 255;
				}
			}
		}
	}

	namedWindow("getTest maps::room wallMap", WINDOW_NORMAL);
	imshow("getTest maps::room wallMap", wallMap);
	waitKey(1);

	/*
	circle( fillMat, contour.center, 3, Scalar(50), -1, 8);
	namedWindow("contour.filledMat", WINDOW_NORMAL);
	imshow("contour.filledMat", fillMat);
	waitKey(0);
	*/

}

void RoomTemplate::evaluateAlignment(RoomTemplate &groundTruth){

	cerr << "A" << endl;
	float errorFill = evalPointsBitwiseAndFill( groundTruth.fillMap, fillMap );
	cerr << "B" << endl;
	float errorWalls = evalPointsBitwiseAndWalls( groundTruth.wallMap, wallMap );
	cerr << "C" << endl;

	cerr << "groundTruth: " << groundTruth.wallCnt << " , " << groundTruth.fillCnt << endl;

	reward = 2 - (errorWalls / groundTruth.wallCnt + errorFill / groundTruth.fillCnt);

	/*
	cout << "errorWalls: " << errorWalls << " / " << groundTruth.wallCnt << " = " << errorWalls / groundTruth.wallCnt << endl;
	cout << "errorFill: " << errorFill << " / " << groundTruth.fillCnt << " = " << errorFill / groundTruth.fillCnt  << endl;
	cout << "reward: " << reward << endl;
	*/
}

void RoomTemplate::simulatedAnnealingUpdate(float temperature){


	cout << "reward: " << reward << endl;
	cout << "workingReward: " << workingReward << endl;
	cout << "bestReward: " << bestReward << endl;
	cout << "P: " << exp( (reward - workingReward)/temperature ) << endl;
	waitKey(0);

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


float RoomTemplate::evalPointsBitwiseAndFill(Mat &gt, Mat &test){

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

float RoomTemplate::evalPointsBitwiseAndWalls(Mat &gt, Mat &test){

	/*
	namedWindow("gt", WINDOW_NORMAL);
	imshow("gt", gt);
	waitKey(1);

	namedWindow("test", WINDOW_NORMAL);
	imshow("test", test);
	waitKey(0);
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


