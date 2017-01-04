/*
 * World.cpp
 *
 *  Created on: Mar 28, 2016
 *      Author: andy
 */

#include "World.h"

using namespace cv;
using namespace std;

World::World(string fName, int resolution, float obsThresh, float comThresh) {
	this->obsThresh = obsThresh;
	this->commThresh = comThresh;

	string fileName = fName + ".jpg";
	Mat image = imread(fileName,1);
	cvtColor(image,image,CV_BGR2GRAY);
	threshold(image,image,230,255,THRESH_BINARY);

	/*
	namedWindow("map in", WINDOW_NORMAL);
	imshow("map in", image);
	waitKey(10);
	*/

	initializeMaps(image, resolution);
	cout << "world::costmap.cells.size(): " << costmap.cells.cols << " x " << costmap.cells.rows << endl;
	cout << "World::Finished building " << fName << ".yml" << endl;
}

void World::initializeMaps(Mat &imgGray, int resolution){

	costmap.cells = Mat::zeros(imgGray.rows / resolution, imgGray.cols / resolution, CV_16S);
	Mat temp = Mat::zeros(costmap.cells.size(), CV_8UC1);

	resize(imgGray, temp, temp.size(), 0, 0, INTER_AREA);

	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point a(i,j);
			Scalar intensity = temp.at<uchar>(a);
			if(intensity[0] >= 225){
				costmap.cells.at<short>(a) = costmap.obsFree;
			}
			else{
				costmap.cells.at<short>(a) = costmap.obsWall;
			}
		}
	}
}


void World::observe(Point cLoc, Costmap &costmap){

	if(viewPerim.size() == 0){
		Mat temp =Mat::zeros(2*(obsThresh + 1), 2*(obsThresh + 1), CV_8UC1);
		Point cent(obsThresh,obsThresh);
		circle(temp,cent,obsThresh, Scalar(255));

		for(int i=0; i<temp.rows; i++){
			for(int j=0; j<temp.cols; j++){
				if(temp.at<uchar>(i,j,0) == 255){
					Point t(i-obsThresh, j-obsThresh);
					viewPerim.push_back(t);
				}
			}
		}
	}

	// if needed, initialize costmap
	if(costmap.cells.empty()){
		costmap.cells = Mat::ones(this->costmap.cells.size(), CV_16S) * costmap.unknown;
		costmap.occ = Mat::ones(this->costmap.cells.size(), CV_32F) * 0.5;
		costmap.euclidDist = this->costmap.euclidDist;
	}

	costmap.cellUpdates.clear();
	costmap.cellUpdates = getObservableCells(cLoc);

	// set obstacles in costmap
	float pOcc = 0.65;
	float pFree = 0.49;
	for(size_t i=0; i<costmap.cellUpdates.size(); i++){
		Point c = costmap.cellUpdates[i];
		costmap.occ.at<float>(c) = costmap.occ.at<float>(c)*pFree / (costmap.occ.at<float>(c)*pFree + (1-costmap.occ.at<float>(c))*(1-pFree) ); // this cell is always free and being updated
		if(costmap.cells.at<short>(c) != costmap.obsFree){
			costmap.cells.at<short>(c) = costmap.obsFree;
			for(int k=c.x-1; k<c.x+2; k++){
				for(int l=c.y-1; l<c.y+2; l++){
					Point a(k,l);
					if(this->costmap.cells.at<short>(a) == this->costmap.obsWall){ // are any of my nbrs visible?
						costmap.cells.at<short>(a) = costmap.obsWall; // set my cost
						costmap.occ.at<float>(a) = costmap.occ.at<float>(a)*pOcc / (costmap.occ.at<float>(a)*pOcc + (1-costmap.occ.at<float>(a))*(1-pOcc) ); // nbr cell that is a wall
					}
				}
			}
		}
	}
}

vector<Point> World::getObservableCells(Point p){
	Mat ta = Mat::zeros(costmap.cells.size(), CV_8UC1);
	vector<Point> obsCellList;

	/*
	circle(ta, p, 3, Scalar(127), -1, 8);
	namedWindow("ta", WINDOW_NORMAL);
	imshow("ta", ta);
	waitKey(1);
	*/

	for(size_t i=0; i<viewPerim.size(); i++){
		Point v(viewPerim[i].x + p.x, viewPerim[i].y + p.y);
		//circle(ta, v, 1, Scalar(255), -1, 8);
		LineIterator it(ta, p, v, 4, false);
		for(int i=0; i<it.count; i++, ++it){
			Point pp  = it.pos();
			//circle(ta, pp, 1, Scalar(255), -1, 8);
			if(pp.x >= costmap.cells.cols || pp.y >= costmap.cells.rows || costmap.cells.at<short>(pp) > costmap.infFree){
				break;
			}
			else if(costmap.cells.at<short>(pp) == costmap.obsFree){
				obsCellList.push_back(pp);
				//circle(ta, pp, 3, Scalar(127), -1, 8);
			}
		}
		//imshow("ta", ta);
		//waitKey(1);
	}
	//waitKey(0);
	return obsCellList;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Retired Functions /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

/*

void World::saveWorldToYML(string fName){
	string filename =  fName + ".yml";
	FileStorage fs(filename, FileStorage::WRITE);

	fs << "costMap" << "[" << this->costmap.cells << "]";

	fs.release();
}

void World::pullWorldFromYML(string fName){

	FileStorage fsN(fName + ".yml", FileStorage::READ);
	fsN["costMap"] >> this->costmap.cells;

	vector<vector<int> > tObsGraph;
	fsN["obsGraph"] >> tObsGraph;
	fsN.release();

	cout << "world::costmap.cells.size(): " << this->costmap.cells.cols << " x " << this->costmap.cells.rows << endl;

	for(int i=0; i<this->costmap.nCols; i++){
		vector<vector<vector<int> > > c;
		for(int j=0; j<this->costmap.nRows; j++){
			vector<vector<int> > a;
			for(size_t k=0; k<tObsGraph[this->costmap.nRows*i+j].size(); k = k+2){
				vector<int> b;
				b.push_back(tObsGraph[this->costmap.nRows*i+j][k]);
				b.push_back(tObsGraph[this->costmap.nRows*i+j][k+1]);
				a.push_back(b);
			}
			c.push_back(a);
		}
		this->obsGraph.push_back(c);
	}

	for(int i=0; i<this->obsGraph.size(); i++){
		for(int j=0; j<this->obsGraph[i].size(); j++){
			for(int k=0; k<this->obsGraph[i][j].size(); k++){
				cerr << this->obsGraph[i][j][k][0] << ", " << this->obsGraph[i][j][k][1] << "; ";
			}
			cerr << endl;
		}
	}
}

Mat World::createMiniMapImg(){
	Mat temp = Mat::zeros(this->costmap.cells.size(),CV_8UC1);
	for(int i=0; i<this->costmap.cells.cols; i++){
		for(int j=0; j<this->costmap.cells.rows; j++){
			if(this->costmap.cells.at<uchar>(i,j) > 100){
				temp.at<uchar>(i,j) = 101;
			}
			else{
				temp.at<uchar>(i,j) = costmap.cells.at<uchar>(i,j);
			}
		}
	}
	return temp;
}

void World::getCommGraph(){
	for(int i=0; i<this->costmap.nRows; i++){
		for(int j=0; j<this->costmap.nCols; j++){ // check each node
			if(this->costmap.cells[i][j] == 0){ // am I traversable?

				for(int k=i; k<this->costmap.nRows;k++){
					for(int l=j; l<this->costmap.nCols; l++){ // against all other nodes
						if(this->costmap.cells[k][l] == 0){ // are they traversable?

							if(this->pointMap[i][j].distGraph[k][l] < this->commThresh){ // is it close enough to observe
								float unitVecX = (this->pointMap[k][l].x - this->pointMap[i][j].x) / this->pointMap[i][j].distGraph[k][l]; // get unit vector in right direction
								float unitVecY = (this->pointMap[k][l].y - this->pointMap[i][j].y) / this->pointMap[i][j].distGraph[k][l];
								int steps = this->pointMap[i][j].distGraph[k][l]; // steps to check
								bool obsFlag = true;
								for(int m=1; m<steps; m++){
									int aX = this->pointMap[i][j].x + m*unitVecX;
									int aY = this->pointMap[i][j].y + m*unitVecY;
									Scalar intensity = imgGray.at<uchar>(aY,aX);
									if(intensity[0] != 255 ){
										obsFlag = false;
										break;
									}
								}
								if(obsFlag){ // are there no obstacles between me and them?
									this->pointMap[i][j].comGraph[k][l] = true;
									this->pointMap[k][l].comGraph[i][j] = true;
								}
							}
						}

					}
				}
			}
		}
	}
}

void World::getObsGraph(){
	for(int i=0; i<this->costmap.nCols; i++){
		vector<vector<vector<int> > > to; // [yLoc][list][x/y]
		for(int j=0; j<this->costmap.nRows; j++){
			vector<vector<int> > too; // [list][x/y]
			to.push_back(too);
		}
		this->obsGraph.push_back(to);
	}
	for(int i=0; i<this->costmap.nCols; i++){
		for(int j=0; j<this->costmap.nRows; j++){ // check each node
			if(this->costmap.cells[i][j] < 10){ // am I traversable?
				//cout << "cell: " << i << ", " << j << " is traversable" << endl;
				for(int k=0; k<this->costmap.nCols;k++){
					for(int l=0; l<this->costmap.nRows; l++){ // against all other nodes
						if(this->costmap.cells[k][l] == this->costmap.obsFree){ // are they traversable?
							//cout << "cell: " << i << ", " << j << " against " << k << ", " << l << ", both traversable*" << endl;
							vector<int> a;
							a.push_back(i);
							a.push_back(j);
							vector<int> b;
							b.push_back(k);
							b.push_back(l);
							float dist = this->costmap.getEuclidianDistance(a,b);
							//cout << "   dist: " << dist << endl;

							if(dist < this->obsThresh && dist > 0){ // is it close enough to observe
								//cout << "   dist crit met" << endl;
								float unitVecX = (k - i) / dist; // get unit vector in right direction
								float unitVecY = (l - j) / dist;
								//cout << "   uVec: " << unitVecX << ", " << unitVecY << endl;
								int steps = dist; // steps to check
								bool obsFlag = true;
								for(int m=1; m<steps; m++){ // check all intermediate points between two cells
									int aX = i + m*unitVecX;
									int aY = j + m*unitVecY;
									//cout << "   ax / ay: " << aX << " / " << aY << endl;
									if(this->costmap.cells[aX][aY] != this->costmap.obsFree ){
										obsFlag = false;
										break;
									}
								}
								if(obsFlag){ // are there no obstacles between me and them?
									vector<int> t;
									t.push_back(k);
									t.push_back(l);
									this->obsGraph[i][j].push_back(t);
								}
							}
							else if(dist == 0){
								vector<int> t;
								t.push_back(i);
								t.push_back(j);
								this->obsGraph[i][j].push_back(t);
							}
						}
					}
				}
			}
		}
	}
}

void World::addCommLine(vector<int> b,vector<int> c){
	vector<int> a;
	a.push_back(b[0]);
	a.push_back(b[1]);
	a.push_back(c[0]);
	a.push_back(c[1]);
	this->commLine.push_back(a);
}

void World::plotCommLines(){
	Scalar color;
	color[0] = 255;
	color[1] = 0;
	color[2] = 0;
	for(int i=0; i<(int)this->commLine.size(); i++){
		// TODO fix world pointmap
		//line(this->image,this->pointMap[this->commLine[i][0]][this->commLine[i][1]],this->pointMap[this->commLine[i][2]][this->commLine[i][3]],color,2,8);
	}
	this->commLine.erase(this->commLine.begin(),this->commLine.end());
}

void World::plotTravelGraph(){
	Scalar color;
	color[0] = 0;
	color[1] = 0;
	color[2] = 0;
	for(int i=0; i<this->costmap.nRows; i++){
		for(int j=0; j<this->costmap.nCols; j++){
			if(this->costmap.cells[i][j] == 0){ // are they traversable?

				for(int k=0; k<2; k++){
					if(!this->costmap.cells[i][j+k] == 0){
						// TODO fix world pointmap
						// line(this->image,this->pointMap[i][j],this->pointMap[i][j+k],color,1,8);
					}
				}
			}
		}
	}
}

void World::plotFrontierGraph(){
	Scalar color;
	color[0] = 0;
	color[1] = 0;
	color[2] = 255;
	//for(int i=0; i<(int)this->frntList.size(); i++){
		//circle(this->image,this->pointMap[this->frntList[i][0]][this->frntList[i][1]],1,color,-1);
	//}
}

Mat World::createExplImage(){
	Scalar color;
	color[0] = 255;
	color[1] = 255;
	color[2] = 255;
	Mat skel(this->image.rows,this->image.cols,CV_8UC1,Scalar(0));
	for(int i=0; i<this->costmap.nRows; i++){
		for(int j=0; j<this->costmap.nCols; j++){
			if(this->costmap.cells[i][j] == 0){ // are they traversable?

				// TODO fix world pointmap

				//Point a;
				//a.x = this->pointMap[i][j].x;
				//a.y = this->pointMap[i][j].y;
				//rectangle(skel,this->pointMap[i][j],a,color,-1);

			}
		}
	}
	return(skel);
}

void World::plotPath(vector<vector<int> > myPath, int myColor[3], int pathIndex){
	Scalar color;
	color[0] = myColor[0];
	color[1] = myColor[1];
	color[2] = myColor[2];
	//for(int i=pathIndex; i<(int)myPath.size(); i++){
	//	line(this->image,this->pointMap[myPath[i-1]],this->pointMap[myPath[i]],Scalar{myColor[0],myColor[1],myColor[2]},3,8);
	//}
	// TODO fix world pointmap
	// //circle(this->image, this->pointMap[myPath[pathIndex][0]][myPath[pathIndex][1]], 10, color, -1);
	//circle(this->image,this->pointMap[myPath[myPath.size()-1]],10,Scalar{myColor[0],myColor[1],myColor[2]},-1);
	//circle(this->image,this->pointMap[myPath[myPath.size()-1]],5,Scalar{0,0,0},-1);
}

void World::plotMap(){
	Mat temp;
	resize(this->image,temp,Size(),1,1,CV_INTER_AREA);
	imshow("Global Map", temp);
	waitKey(1);
}

*/

World::~World() {

}
