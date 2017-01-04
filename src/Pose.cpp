/*
 * Pose.cpp
 *
 *  Created on: Nov 2, 2016
 *      Author: andy
 */

#include "Pose.h"

bool pointOnMat(Point &a, Mat &b);

Pose::Pose(Point ploc, Costmap &costmap) {
	loc = ploc;
	needInference = false;

	radius = 50;
	nSamples = 108;

	mean = -1;
	stanDev = -1;

	reward = -1;

	orient = -1;
	direction = -1;

	getPoseHistogram(costmap);
}

void Pose::rotateLimits(){

	float theta = float(orient)*6.283185307 / float(nSamples);

	rotLim.clear();
	for(size_t i=0; i<obsLim.size(); i++){
		float x = float(obsLim[i].x*cos(theta) + float(obsLim[i].y)*sin(theta));
		float y = float(-obsLim[i].x*sin(theta) + float(obsLim[i].y)*cos(theta));
		rotLim.push_back( Point( int(x), int(y) ) );
	}
}

void Pose::insertPoseInCostmap(Costmap &costmap, Point2f oLoc, Mat &mat){

	Point2f p;
	for(size_t i=0; i<rotLim.size(); i++){

		p = rotLim[i]+oLoc; // end point I am aiming at

		LineIterator it(mat, oLoc, p, 4, false);
		for(int j=0; j<it.count; j++, ++it){
			Point pp  = it.pos();

			int ppVal = costmap.cells.at<short>(pp);

			if( ppVal == costmap.obsFree){
				// known to be free, keep moving
				continue;
			}
			else if( ppVal == costmap.obsWall ){
				// known wall so stop
				break;
			}
			else if( ppVal == costmap.infWall ){
				// previously inferred its a wall, do I now think it's a wall or free?
				if(j+1 >= it.count){ // didn't hit a wall
					if(this->obsVal[i] == costmap.obsFree){
						mat.at<Vec3b>(pp) = costmap.cInfFree;
					}
					else{
						mat.at<Vec3b>(pp) = costmap.cInfWall;
					}
					break;
				}
				else{
					mat.at<Vec3b>(pp) = costmap.cInfFree;
				}

			}
			else if( ppVal == costmap.infFree ){
				// previously think it's free, do I now think it's a wall or free?
				if(j+1 >= it.count){ // didn't hit a wall
					if(this->obsVal[i] == costmap.obsFree){
						mat.at<Vec3b>(pp) = costmap.cInfFree;
					}
					else{
						mat.at<Vec3b>(pp) = costmap.cInfWall;
					}
					break;
				}
				else{
					mat.at<Vec3b>(pp) = costmap.cInfFree;
				}
			}
			else if( ppVal == costmap.unknown ){
				// no idea
				if(j+1 >= it.count){ // didn't hit a wall
					if(this->obsVal[i] == costmap.obsFree){
						mat.at<Vec3b>(pp) = costmap.cInfFree;
					}
					else{
						mat.at<Vec3b>(pp) = costmap.cInfWall;
					}
					break;
				}
				else{
					mat.at<Vec3b>(pp) = costmap.cInfFree;
				}
			}
			else{
				mat.at<Vec3b>(pp) = costmap.cInfFree;
			}
		}
	}

}

void Pose::getPoseHistogram(Costmap &costmap){

	Mat ta = Mat::zeros(costmap.cells.size(), CV_8UC1);

	Point2f p;
	for(int ns=0; ns<nSamples; ns++){
		p.x = float(loc.x) + radius*cos( 6.283185307*ns/nSamples); // end point I am aiming at
		p.y = float(loc.y) + radius*sin( 6.283185307*ns/nSamples);

		LineIterator it(ta, loc, p, 4, false);
		for(int i=0; i<it.count; i++, ++it){
			Point pp  = it.pos();

			int ppVal = costmap.cells.at<short>(pp);

			if( ppVal == costmap.infWall || ppVal == costmap.inflatedWall || ppVal == costmap.infFree || ppVal == costmap.domFree || ppVal == costmap.unknown){
				this->needInference = true;
			}

			if(ppVal > costmap.unknown){ // detect if hit a wall
				this->obsLim.push_back( Point(pp.x-loc.x, pp.y-loc.y) ); // record position of end pt relative to center
				this->obsLen.push_back( sqrt( pow(loc.x - pp.x,2) + pow(loc.y - pp.y,2)) ); // record lengths
				this->obsVal.push_back( ppVal );
				break;
			}

			if(i+1 >= it.count){ // didn't hit a wall, but about to end
				this->obsLim.push_back( Point(p.x-loc.x, p.y-loc.y) ); // record position of end pt relative to center
				this->obsLen.push_back( radius ); // record lengths
				this->obsVal.push_back( costmap.cells.at<short>(pp));
			}
		}
	}
	getMean();
	getStanDev();
}


void Pose::getObservedCells(Costmap &costmap){

	obsWalls.clear();
	obsFree.clear();

	vector<Point> allPts;

	Mat ta = Mat::ones(costmap.cells.size(), CV_8UC1)*127;

	Point p;
	for(int ns=0; ns<nSamples; ns++){
		p.x = loc.x + round( radius*cos( 6.283185307*ns/(nSamples)) ); // end point I am aiming at
		p.y = loc.y + round( radius*sin( 6.283185307*ns/(nSamples)) );

		LineIterator it(ta, loc, p, 4, false);
		for(int i=0; i<it.count; i++, ++it){
			Point pp  = it.pos();

			allPts.push_back(pp);

			int ppVal = costmap.cells.at<short>(pp);


			if(ppVal > costmap.infFree){ // detect if a cell is not free
				if(ta.at<uchar>(pp) == 127){
					Point a(pp.x - loc.x, pp.y - loc.y);
					this->obsWalls.push_back(a);
					ta.at<uchar>(pp) = 0;
				}
				break;
			}

			if(ta.at<uchar>(pp) == 127){
				Point a(pp.x - loc.x, pp.y - loc.y);
				this->obsFree.push_back(a);
				ta.at<uchar>(pp) = 255;
			}
		}
	}
}

Mat Pose::makeMat(){

	// find min and max x/y vals
	int minx = 100000;
	int miny = 100000;
	int maxx = -100000;
	int maxy = -100000;

	if(this->needInference){
		for(size_t i=0; i<obsLim.size(); i++){
			if(obsLim[i].x > maxx){
				maxx = obsLim[i].x;
			}
			else if(obsLim[i].x < minx){
				minx = obsLim[i].x;
			}

			if(obsLim[i].y > maxy){
				maxy = obsLim[i].y;
			}
			else if(obsLim[i].y < miny){
				miny = obsLim[i].y;
			}
		}

		Mat out = Mat::zeros(maxy - miny+10,maxx - minx+10,CV_8UC1);
		for(size_t i=0; i<obsLim.size(); i++){
			Point t(obsLim[i].x + 5 - minx, obsLim[i].y + 5 -  miny);
			out.at<uchar>(t) = 255;
		}
		out.at<uchar>(Point( 5 - minx, 5 - miny) ) = 127;
		return out;
	}
	else{
		for(size_t i=0; i<obsWalls.size(); i++){
			if(obsWalls[i].x > maxx){
				maxx = obsWalls[i].x;
			}
			else if(obsWalls[i].x < minx){
				minx = obsWalls[i].x;
			}

			if(obsWalls[i].y > maxy){
				maxy = obsWalls[i].y;
			}
			else if(obsWalls[i].y < miny){
				miny = obsWalls[i].y;
			}
		}

		for(size_t i=0; i<obsFree.size(); i++){
			if(obsFree[i].x > maxx){
				maxx = obsFree[i].x;
			}
			else if(obsFree[i].x < minx){
				minx = obsFree[i].x;
			}

			if(obsFree[i].y > maxy){
				maxy = obsFree[i].y;
			}
			else if(obsFree[i].y < miny){
				miny = obsFree[i].y;
			}
		}

		Mat out = Mat::ones(maxy - miny+10,maxx - minx+10,CV_8UC1)*127;
		for(size_t i=0; i<obsWalls.size(); i++){
			Point t(obsWalls[i].x + 5 - minx, obsWalls[i].y + 5 -  miny);
			out.at<uchar>(t) = 0;
		}

		for(size_t i=0; i<obsFree.size(); i++){
			Point t(obsFree[i].x + 5 - minx, obsFree[i].y + 5 -  miny);
			out.at<uchar>(t) = 255;
		}
		out.at<uchar>(Point( 5 - minx, 5 - miny) ) = 127;
		return out;
	}
}

void Pose::drawHistogram(char* title){
	Point base;
	Point top;

	int maxv = -1;
	for(size_t i =0; i<obsLen.size(); i++){
		if(obsLen[i] > maxv){
			maxv = obsLen[i];
		}
	}

	base.y = 10;
	top.y = maxv + 20;

	Mat h = Mat::zeros(maxv + 20, obsLen.size() + 20, CV_8UC1);

	base.x = 10;
	for(size_t i=0; i<obsLen.size(); i++){
		base.x = 10 + i;
		top.x = base.x;
		top.y = 10 + round(obsLen[i]);
		line(h, base, top, Scalar(255), 1, 8);
	}

	char buffer[150];
	sprintf(buffer, "Pose::drawHistogram::obsLen");


	namedWindow(buffer, WINDOW_NORMAL);
	imshow(buffer, h);
	waitKey(1);
}

void Pose::getMean(){

	float sum = 0;
	float N = obsLen.size();
	for(int i=0; i<N; i++){
		sum += obsLen[i];
	}

	this->mean = sum / N;
}

void Pose::getStanDev(){

	float sum = 0;
	float N = obsLen.size();
	for(int i=0; i<N; i++){
		sum += pow(this->mean - obsLen[i],2);
	}

	this->stanDev = sqrt(sum / N);
}

float Pose::getPDF( float x ){
    float pi = 3.14159265358979323846264338327950288419716939937510582;
    return 1 / sqrt(2*pow(stanDev,2)*pi)*exp(-pow(x-mean,2)/(2*pow(stanDev,2)));
}

float Pose::getCDF( float x ){
	// A Sigmoid Approximation of the Standard Normal Integral
	// Gary R. Waissi and Donald F. Rossin

    float z = (x- mean) / stanDev;
    float pi = 3.14159265358979323846264338327950288419716939937510582;
    return 1 / (1+exp(-sqrt(pi)*(-0.0004406*pow(z,5) + 0.0418198*pow(z,3) + 0.9*z) ) );
}

Pose::~Pose() {}

