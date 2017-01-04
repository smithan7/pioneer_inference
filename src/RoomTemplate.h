/*
 * RoomTemplate.h
 *
 *  Created on: Aug 29, 2016
 *      Author: andy
 */

#ifndef ROOMTEMPLATE_H_
#define ROOMTEMPLATE_H_

#include "Contour.h"
#include "Costmap.h"

class RoomTemplate {
public:
	RoomTemplate();
	virtual ~RoomTemplate();

	Contour contour;
	vector<Point> wallList;
	Mat fillMat, thinMat, wallMat; // holds the original version
	Mat fillMap, thinMap; // holds working maps
	Mat bestMap, wallMap, obsMap; // holds the best map and wall map
	float wallCnt, obsCnt, fillCnt, thinCnt;

	void simulatedAnnealingIteration(Costmap &costmap, RoomTemplate &groundTruth, float temperature);
	void modifyWorkingSet();
	void getTestMaps(Costmap &costmap, RoomTemplate &groundTruth);
	void evaluateAlignment(RoomTemplate &groundTruth);
	void simulatedAnnealingUpdate(float temperature);
	float initialRotationCheck( RoomTemplate &gt, float steps, Costmap &costmap);
	void initialRotationCheckBisection( RoomTemplate &gt, Costmap &costmap);
	void resetUCB();

	void getGroundTruthRoomMaps( Costmap &costmap, Contour &room);
	void getGroundTruthWallMap(Costmap &costmap, Contour &room);
	void getWallMat(Point offset, int maxS); // for each room
	void getGroundTruthObsMap(Costmap &costmap, Contour &room);
	void getGroundTruthFillMap(Costmap &costmap, Contour &room);
	void getGroundTruthThinMap(Costmap &costmap, Contour &room);

	float evalPointsBitwiseAndFill(Mat &gt, Mat &test); // compare filled areas accounting for not in both
	float evalPointsBitwiseAndWalls(Mat &gt, Mat &test); // compare outer contours only and only those covered by gt

	float pulls;

	float shift_x;
	float shift_y;
	float rotateAngle;
	float stretch_x;
	float stretch_y;

	float workingShift_x;
	float workingShift_y;
	float workingRotateAngle;
	float workingStretch_x;
	float workingStretch_y;

	float bestShift_x;
	float bestShift_y;
	float bestRotateAngle;
	float bestStretch_x;
	float bestStretch_y;

	float reward;
	float workingReward;
	float bestReward;

};

#endif /* ROOMTEMPLATE_H_ */
