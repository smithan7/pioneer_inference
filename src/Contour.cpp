/*
 * Contour.cpp
 *
 *  Created on: Jul 10, 2016
 *      Author: andy
 */

#include "Contour.h"

int constrainToVec(int a, float l);
void drawHistogram(vector<float> histogram, char* title);
int getMatReward(Mat &in);
void getMatWithValue(Costmap &costmap, Mat &mat, int value);

using namespace cv;

Contour::Contour(Mat &matIn, vector<Point> pts) {
	points = pts;
	area = contourArea(pts);
	this->matRows = matIn.rows;
	this->matCols = matIn.cols;
	histogramLength = 100;
	dominated = false;
}

Contour::Contour(){};

void Contour::getCenter(Mat &cells){

	Mat a = Mat::zeros(cells.size(), CV_8UC1);

	vector<vector<Point> > pts;
	pts.push_back( points );
	drawContours(a, pts, 0, Scalar(255), -1, 8);

	Point m(0,0);
	float cntr = 0;

	for(int i=0; i<cells.cols; i++){
		for(int j=0; j<cells.rows; j++){
			Point p(i,j);
			if( a.at<uchar>(p) == 255){
				m.x += p.x;
				m.y += p.y;
				cntr++;
			}
		}
	}

	center.x = float(m.x)/cntr;
	center.y = float(m.y)/cntr;
}

void Contour::fillContourInCostmap( Costmap &costmap, int val ){

	Mat a = Mat::zeros(costmap.cells.size(), CV_8UC1);

	vector<vector<Point> > pts;
	pts.push_back( points );
	drawContours(a, pts, 0, Scalar(255), -1, 8);

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point p(i,j);
			if( a.at<uchar>(p) == 255){
				costmap.cells.at<short>(p) = val;
			}
		}
	}

}


void Contour::getInvariantHistogram(){
	histogramLength = 100;

	float xSum = 0;
	float ySum = 0;

	for(size_t j=0; j<points.size(); j++){
		xSum += points[j].x;
		ySum += points[j].y;
	}

	Point center;
	center.x = xSum / points.size();
	center.y = ySum / points.size();

	// getMean Length
	meanLength = 0;
	for(uint i=0; i<points.size(); i++){
		float l = sqrt( pow(center.y - points[i].y, 2) +  pow(center.x - points[i].x,2)); // get euclidian distance to point
		length.push_back( l ); // save it
		meanLength += l; // for calcing mean length
	}
	meanLength /= float(length.size() ); // calc mean length


	// set up the histogram bins
	for(float i = 0; i<histogramLength; i++){
		histogram.push_back(0);
	}

	float binWidth = 3.0 / histogramLength; // 100 bins over 3 unit
	for(size_t i = 0; i<length.size(); i++){
		float normLength = length[i] / meanLength;
		if(normLength > 3 - binWidth){
			histogram[histogram.size()-1]++;
		}
		else if (normLength < binWidth){
			histogram[0]++;
		}
		else{
			int bin = round(normLength / binWidth); // normallize length and put in a bin;
			histogram[bin]++;
		}
	}
	for(int i=0; i<100; i++){
		histogram[i] = histogramLength * histogram[i] / float(length.size());
	}
}

void Contour::getSequenceHistogram(){

	float ls = length.size();

	float w = ls  / histogramLength;
	int w2 = round( w/2 );

	for(int i=0; i<histogramLength; i++){
		float lwSum = 0;
		for(int c = i*w-w2; c < i*w+w2; c++){
			int cv = constrainToVec( c, ls );
			lwSum += length[cv] / meanLength;
		}
		sequence.push_back(lwSum);
	}
	/*
	for(int i=0; i<histogramLength; i++){
		sequence[i] = histogramLength * sequence[i] / float(length.size() );
	}
	*/

	/*
	char buffer[50];
	sprintf(buffer, "sequence");
	drawHistogram( sequence, buffer);
	*/
}


void Contour::getVals(Costmap &costmap){
	for(size_t i=0; i<points.size(); i++){
		vals.push_back( costmap.cells.at<short>(points[i]) );
	}
}

void Contour::getNbrsWithValue(Costmap &costmap, int value){
	int dx[4] = {-1,1,0,0};
	int dy[4] = {0,0,-1,1};

	vector<Point> tbrs;

	for(size_t i=0; i<points.size(); i++){
		for(int j=0; j<4; j++){
			Point a;
			a.x = points[i].x + dx[j];
			a.y = points[i].y + dy[j];
			if(costmap.cells.at<short>(a) == value){
				nbrs.push_back(a);
			}
		}
	}
}

int getMatReward(Mat &in){
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

void getMatWithValue(Costmap &costmap, Mat &mat, int value){
	for(int i=0; i<mat.rows; i++){
		for(int j=0; j<mat.cols; j++){
			if(costmap.cells.at<uchar>(i,j) == value){ // not free space or walls
				mat.at<uchar>(i,j) = 255;
			}
		}
	}
}

void Contour::getDominatedStatus(Costmap &costmap){

	Mat cMat = Mat::zeros( costmap.cells.size(), CV_8UC1 );

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point a(i,j);
			if( costmap.cells.at<short>(a) == costmap.infFree || costmap.cells.at<short>(a) == costmap.unknown){
				cMat.at<uchar>(i,j) = 255;
			}
		}
	}

	/*
	namedWindow("infFree and Unknown", WINDOW_NORMAL);
	imshow("infFree and Unknown", cMat);
	waitKey(1);
	*/

	Mat tMat = Mat::zeros(cMat.size(), CV_8UC1);
	vector<vector<Point> > pts;
	pts.push_back( points );
	drawContours(tMat, pts, 0, Scalar(255), 5, 8); // inflate wall of contour for set width
	drawContours(tMat, pts, 0, Scalar(0), -1, 8); // get rid of internal inflation
	drawContours(tMat, pts, 0, Scalar(0), 1, 8); // get rid of countour boundary

	/*
	namedWindow("inflated contour", WINDOW_NORMAL);
	imshow("inflated contour", tMat);
	waitKey(1);
	*/

	bitwise_and(tMat, cMat, tMat);

	/*
	namedWindow("bitwise and", WINDOW_NORMAL);
	imshow("bitwise and", tMat);
	waitKey(1);
	*/

	int reward = getMatReward( tMat );
	if( reward == 0){
		dominated = true;
	}
	else{
		dominated = false;
	}

	/*
	cout << "dominated: " << dominated << endl;
	waitKey(0);
	*/
}

vector<Point> Contour::getContourExitPoints(int val){
	vector<Point> exits;

	for(size_t i=0; i<vals.size(); i++){
		if(vals[i] == val){
			float cx = points[i].x;
			float cy = points[i].y;
			int cntr = 1;

			int nbr = i+1;

			while(nbr == val){
				nbr++;
				cntr++;
				cx += points[i].x;
				cy += points[i].y;
			}
			i = nbr; // skip over segment

			Point a;
			a.x = cx / cntr;
			a.y = cy / cntr;

			exits.push_back(a);

		}
	}


	return exits;
}

int constrainToVec(int a, float l){

	if( a >= l ){
		a = l-a;
	}
	else if( a < 0 ){
		a = l+a;
	}

	return a;
}

void Contour::drawHistogram(vector<float> histogram, char* title){
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


Contour::~Contour() {}
