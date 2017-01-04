/*
 * Inference.h
 *
 *  Created on: Jul 12, 2016
 *      Author: andy
 */

#ifndef INFERENCE_H_
#define INFERENCE_H_

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <vector>

#include "Frontier.h"
#include "Graph.h"
#include "Pose.h"
#include "RoomTemplate.h"
#include "BuildingTemplate.h"
#include "World.h"
#include "Contour.h"


using namespace std;
using namespace cv;

class Inference {
public:
	Inference();
	virtual ~Inference();
	void init( float obsRadius );

	// usfeul stuff
	int minContourToInfer;
	vector<Point> hullBreaches;

	// main inference functions
	void makeInference(string method, Costmap &costmap);
	void makeGeometricInference(Costmap &costmap);
	void makeNaiveInference( Costmap &costmap);
	void makeVisualInference(Costmap &costmap);


	void geometricInference(Costmap &costmap);
	void geometricInference(Costmap &costmap, Mat &geoInf);

	
	// visual inference
	void visualInference( Costmap &costmap, Mat &visInfMat);
	Pose visualInferenceGetPose( Costmap &costmap, Point iLoc);
	float calcVisualFit(Pose &po, Pose &pl, Costmap &costmap, int &orient, float maxCost); // calc the matching of two histograms, full rotation
	float visualReward(Pose &obs, Pose &lib, int obsI, int libI, Costmap &costmap);
	void displayVisInfMat(Costmap &costmap);
	void mergeVisualInferenceProbability( Costmap &costmap, Mat &visualInferenceMat, Mat &geoInfMat );
	void trimCandidatePoses( vector<Pose> &candidatePoses, vector<Point2f> &candidateLocs);

	// calculate the reward of two aligned scans

	void addToVisualLibrary(Pose &pose);
	int getVisualLibraryIndex( Pose &pose );
	vector<vector<Pose> > visualLibrary;
	vector<Pose> libraryCenters;
	Mat visInfMat, geoInfMat;
	vector<Point> viewPerim;
	
	void drawHistogram(vector<float> histogram, char* title);

	// wall inflation
	void inflateWalls(Costmap &costmap, int = 3);
	void inflateFree(Costmap &costmap, int inflationSteps);
	bool checkForInflation(Mat &costMat, int i, int j, int dist, int val);

	void simulateObservation(Point pose, Mat &resultingView, Costmap &costmap);
	void simulateObservation(Point pose, Mat &resultingView, Costmap &costmap, vector<float> &length, vector<Point> &endPts);

	// frontier exits
	vector<int> getFrontierExits(vector<Point> &outerHull);

	// Inference tools
	void getImagePoints(Mat &image, vector<Point> &ptList, int value); // all points int the image with value
	void getMatWithValue(Costmap &costmap, Mat &mat, int value);  // mat with all points in costmap of value
	void resetInference(Costmap &costmap); // set all inferred spaces to unknown

	vector<Point> findHullBreaches(Mat &in, Costmap &costmap);
	Point findContourCenter( Mat &mat );
	void removeInaccessibleContours(Costmap &costmap);
	void removeInaccessibleContoursFromCostmap(Costmap &costmap);
	float evalPointsBitwiseAnd(Mat &gt, Mat &test);
	float evalPointsBitwiseAndContours(Mat &gt, Mat &test);
	Mat getObservedWallsOnContour( Costmap &costmap, BuildingTemplate &groundTruth);

	vector<BuildingTemplate> buildingMatches;
	void ucbBuildingRANSAC(Costmap &costmap, BuildingTemplate &groundTruth);
	void UCBstructuralBagOfWordsBuildingInference(Costmap &costmap);
	vector<int> findMatchingBuildingHistograms(vector<float> &histogram, int nMatches);
	void findMatchesByAutoCorrelation(vector<float> &histogram, vector<int> &matches, vector<int> &alignment, int nMatches);
	void placeBuildingInCostmap( Costmap &costmap, Mat &bestMap );
	Mat getObservedMap( Costmap &costmap );
	void geoInferenceRecycling( Costmap &costmap, Mat &structMat );
	void getBuildingTemplate2(Costmap &costmap, BuildingTemplate &groundTruth);

	vector< RoomTemplate > roomMatches;
	void closeDominatedContours( Costmap &costmap );
	void placeRoomInCostmap(Costmap& costmap, Mat &bestMap);
	//void UCBstructuralBagOfWordsRoomInference(Costmap &costmap, vector<Contour> &rooms);
	void ucbRoomRANSAC(Costmap &costmap, RoomTemplate &groundTruth);
	vector<int> findMatchingRoomHistograms(vector<float> &histogram, int nMatches);
	void doorFinderByTravelGraph(Costmap &costmap, vector<Vec4i> &doors);
	void doorFinderAlongFrontiers(Costmap &costmap, vector<Vec4i> &doors);
	Graph inferenceGraph; // for finding rooms

	vector<Frontier> frontiers;
	vector<Point> findFrontiers(Costmap &costmap);
	void clusterFrontiers(vector<Point>  frntList, Costmap &costmap);

	vector<Point> hullPts;
	void displayInferenceMat(Costmap &costmap, Mat &outerHullDrawing, Mat &obstacleHull, vector<Point> &outerHull, vector<int> frontierExits);

	// structural inference stuff
	void structuralBagOfWordsRoomInference(Costmap &costmap);
	BuildingTemplate getBuildingTemplate( Costmap &costmap );
	void UCBstructuralBagOfWordsRoomInference(Costmap &costmap, vector<Contour> &rooms);

	//vector<Contour> getRoomContours(Costmap &costmap, vector<Vec4i> doors);
	bool checkVisibility(Costmap &costmap, Point a, Point b);

	vector<float> shiftHistogram( vector<float> hist, int skip);
	vector<float> compareSequenceByAutoCorrelation( vector<float> &hist1, vector<float> &hist2);
	float compareHistogramByAbsDist(vector<float> &hist1,vector<float> &hist2);
	float compareHistogramByChiSquaredDist(vector<float> &hist1,vector<float> &hist2);
	float compareHistogramByInvCorrelation(vector<float> &hist1,vector<float> &hist2);


	void roomFinder(Graph &graph);
	Graph infGraph; // for keeping the node locations to see what has changed
	vector<vector<int> > doorList; // list of door locations, draw in to seal off rooms

	// geometric inference stuff
	void getOuterHull(Costmap &costmap, Mat &outerHullDrawing, vector<Point> &outerHull);

	void extractInferenceContour();
	int getMatReward(Mat &in);
	void getLengthHistogram(vector<float> length, float meanLength, vector<int> &histogram, vector<float> &sequence);
	vector<vector<float> > roomHistogramList;
	vector<vector<Point> > roomPointList;
	vector<String> roomNameList;
	vector<Point> roomCenterList;
	vector<float> roomMeanLengthList;
	vector<vector<Point> > roomWallsList;

	vector<vector<float> > buildingHistogramList;
	vector<vector<float> > buildingSequenceList;
	vector<vector<Point> > buildingPointList;
	vector<String> buildingNameList;
	vector<Point> buildingCenterList;
	vector<float> buildingMeanLengthList;

	Mat geoInferenceMat, structInferenceMat;
	vector<Point> geoInfContour;

	float minMatchStrength;

	void visualBagOfWordsInference(vector<vector<Point> > &contours, Mat &obstaclesAndHull);
	void makeInferenceAndSetFrontierRewards(); // create outer hull and divide into contours
	void visualInference(); // perform visual inference on each contour
	void valueFrontiers(); // take the contours from inference and get the value of each

	void clusteringObstacles();

	/*
	//////////////////// Begin Retired Functions
	void mergeInferenceContours(Costmap &costmap);
	float evalBreadthSearch(Mat &gt, Mat &tst, int nTests);
	float breadthFirstSearchDist(Point in, Mat &mat);
	float evalPointsPoly(vector<Point> &gtCont, vector<Point> &testCont, float shift_x, float shift_y, int nTests);
	void structuralBagOfWordsBuildingInference(Costmap &costmap);
	void initMyRANSAC(Costmap &costmap, Mat &matchMat, Point matchCenter, Contour &contour, float &theta, float &cost);
	void myBuildingRANSAC(Costmap &costmap, Mat &matchMat, Mat &buildingHullWalls, Contour &contour, float &theta, Mat &bestMap, float cost);
	vector<float> getInferenceContourRewards(vector<int> frontierExits, vector<vector<Point> > contours);
	void setFrontierRewards(vector<float> rewards, vector<vector<Point> > inferenceContours);
	Mat createMiniMapInferImg();
	*/

};

#endif /* INFERENCE_H_ */
