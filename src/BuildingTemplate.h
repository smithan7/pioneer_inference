/*
 * BuildingTemplate.h
 *
 *  Created on: Aug 29, 2016
 *      Author: andy
 */

#ifndef BUILDINGTEMPLATE_H_
#define BUILDINGTEMPLATE_H_

#include "Contour.h"
#include "Costmap.h"

class BuildingTemplate {
public:
	BuildingTemplate();
	virtual ~BuildingTemplate();

	Contour contour;
	Mat fillMat, thinMat; // holds the original version
	Mat fillMap, thinMap; // holds working maps
	Mat bestMap, wallMap, obsMap; // holds the best map and wall map
	float wallCnt, obsCnt, fillCnt, thinCnt;


	void simulatedAnnealingIteration(Costmap &costmap, BuildingTemplate &groundTruth, float temperature);
	void modifyWorkingSet();
	void lookForHullPivots(BuildingTemplate &groundTruth);
	void getTestMaps(Costmap &costmap, BuildingTemplate &groundTruth);
	void evaluateAlignment(BuildingTemplate &groundTruth);
	void simulatedAnnealingUpdate(float temperature);

	void initialAlignmentCheck( BuildingTemplate &gt, int align, Costmap &costmap);
	void initialRotationCheck( BuildingTemplate &gt, float steps, Costmap &costmap);
	void initialRotationCheckBisection( BuildingTemplate &gt, Costmap &costmap);
	float evalPointsBitwiseAndFill(Mat &gt, Mat &test); // compare filled areas accounting for not in both
	float evalPointsBitwiseAndThin(Mat &gt, Mat &test); // compare outer contours only and only those covered by gt
	float evalPointsBitwiseAndObservedFill(Mat &gt, Mat &test);
	void getBuildingContour( Costmap &costmap );
	void getObservedContour( Costmap &costmap );
	void resetUCB();

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

#endif /* BUILDINGTEMPLATE_H_ */
