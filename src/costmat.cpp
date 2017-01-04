#include "costmat.h"

bool pointCompare(Point &a, Point &b);
bool pointOnMat(Point &a, Mat &b);

Costmat::Costmat(){
	ros_unknown = -1;
	ros_wall = 100;
	ros_free = 0;

	Vec3b a(255,255,255);
	cObsFree = a;

	Vec3b b(200,200,200);
	cInfFree = b;

	Vec3b c(0,0,0);
	cObsWall = c;

	Vec3b d(50,50,50);
	cInfWall = d;

	Vec3b e(127,127,127);
	cUnknown = e;

	Vec3b f(255,0,0);
	cError = f;


	obsFree = 1;
	infFree = 2;
	domFree = 3;

	unknown = 101;

	obsWall = 201;
	infWall = 202;
	inflatedWall = 203;

	init_flag = true;
}

Costmat::~Costmat(){}

void Costmat::updateCells( vector<int8_t> &occupancy_grid_array ){

	cout << cells.cols << " / " << cells.rows << endl;
	
	for(size_t i=0; i<occupancy_grid_array.size(); i++){
		 Point p = getCellIndex( i );

		 //ROS_INFO("p.x/y: %d/%d: %d", p.x, p.y,occupancy_grid_array[i]);
		 
		 if(occupancy_grid_array[i] == ros_unknown){
		 	cells.at<short>(p) = unknown;
		 }
		 else if(occupancy_grid_array[i] < ros_wall / 4){//ros_free){
		 	cells.at<short>(p) = obsFree;
		 }
		 else{//} if(occupancy_grid_array[i] == ros_wall){
		 	cells.at<short>(p) = obsWall;
		 }
	}
	displayCells();
}

void Costmat::buildCellsPlot(){
	this->displayPlot= Mat::zeros(cells.size(),CV_8UC3);
	for(int i=0; i<cells.cols; i++){
		for(int j=0; j<cells.rows; j++){
			Point a(i,j);
			if(this->cells.at<short>(a) == this->obsFree){
				this->displayPlot.at<Vec3b>(a) = this->cObsFree;
			}
			else if(this->cells.at<short>(a) == this->infFree){
				this->displayPlot.at<Vec3b>(a) = this->cInfFree;
			}
			else if(this->cells.at<short>(a) == this->obsWall){
				this->displayPlot.at<Vec3b>(a) = this->cObsWall;
			}
			else if(this->cells.at<short>(a) == this->infWall){
				this->displayPlot.at<Vec3b>(a) = this->cInfWall;
			}
			else if(this->cells.at<short>(a) == this->inflatedWall){
				this->displayPlot.at<Vec3b>(a) = this->cInfWall;
			}
			else if(this->cells.at<short>(a) == this->unknown){
				this->displayPlot.at<Vec3b>(a) = this->cUnknown;
			}
			else{ // anything else, should never happen
				this->displayPlot.at<Vec3b>(a) = this->cError;
			}
		}
	}
}

float Costmat::getEuclidianDistance(Point a, Point b){
	int dx = a.x - b.x;
	int dy = a.x - b.y;

	if(euclidianDistance.at<float>(dx,dy) == -1){
		euclidianDistance.at<float>(dx,dy) = sqrt(pow(dx,2) + pow(dy,2));
	}

	return(euclidianDistance.at<float>(dx,dy) si}

vector<Point> Costmat::aStarPath(Point sLoc, Point gLoc){

	if(sLoc == gLoc){
		vector<Point> totalPath;
		for(int i=0; i<4; i++){
			Point t = sLoc;
			totalPath.push_back(t);
		}
		return totalPath;
	}

	Mat cSet = Mat::zeros(cells.size(), CV_16S); // 1 means in closed set, 0 means not
	Mat oSet = Mat::zeros(cells.size(), CV_16S); // 1 means in open set, 0 means not
	Mat cameFromX = Mat::ones(cells.size(), CV_16S)*-1; // each square has a vector of the location it came from
	Mat cameFromY = Mat::ones(cells.size(), CV_16S)*-1; // each square has a vector of the location it came from

	Mat gScore = Mat::ones(cells.size(), CV_32F)*INFINITY; // known cost from initial node to n
	Mat fScore = Mat::ones(cells.size(), CV_32F)*INFINITY; // known cost from initial node to n

	vector<Point> oVec;
	oVec.push_back(sLoc);
	oSet.at<short>(sLoc) = 1; // starting node has score 0
	gScore.at<float>(sLoc)  = 0; // starting node in open set
	fScore.at<float>(sLoc) = sqrt(pow(sLoc.x-gLoc.x,2) + pow(sLoc.y-gLoc.y,2));
	fScore.at<float>(gLoc) = 1;

	bool foo = true;

	while(foo){
		/////////////////// this finds node with lowest fScore and makes current
		float min = INFINITY;
		int mindex = -1;

		for(size_t i=0; i<oVec.size(); i++){
			if(fScore.at<float>(oVec[i]) < min){
				min = fScore.at<float>(oVec[i]);
				mindex = i;
			}
		}

		Point cLoc = oVec[mindex];
		oVec.erase(oVec.begin() + mindex);
		oSet.at<short>(cLoc) = 0;
		cSet.at<short>(cLoc) = 1;

		//cout << "*** sLoc/cLoc/gLoc: " << sLoc << " / "<< cLoc << " / "<< gLoc << endl;

		/////////////////////// end finding current node
		if(pointCompare(cLoc, gLoc) ){ // if the current node equals goal, construct path
			vector<Point> totalPath;
			totalPath.push_back(gLoc);
			while( cLoc.x != sLoc.x || cLoc.y != sLoc.y ){ // work backwards to start
				Point tLoc(cameFromX.at<short>(cLoc), cameFromY.at<short>(cLoc));
				totalPath.push_back(tLoc); // append path
				cLoc.x = tLoc.x;
				cLoc.y = tLoc.y;
			}
			reverse(totalPath.begin(),totalPath.end());
			return totalPath;
		} ///////////////////////////////// end construct path

		// for nbrs
		int nx[8] = {-1,-1,-1,0,0,1,1,1};
		int ny[8] = {1,0,-1,1,-1,1,0,-1};

		for(int ni = 0; ni<8; ni++){
			Point nbr;
			nbr.x += cLoc.x + nx[ni];
			nbr.y += cLoc.y + ny[ni];
			if(pointOnMat(nbr, cells) ){
				//cout << "sLoc/cLoc/nbr/gLoc: " << sLoc << " / "<< cLoc << " / "<< nbr << " / "<< gLoc << endl;
				if(cSet.at<short>(nbr) == 1){ // has it already been eval? in cSet
					continue;
				}
				float ngScore = gScore.at<float>(cLoc) + sqrt(pow(cLoc.x-nbr.x,2) + pow(cLoc.y-nbr.y,2));//getEuclidianDistance(cLoc, nbr); // calc temporary gscore, estimate of total cost
				if(oSet.at<short>(nbr) == 0){
					oSet.at<short>(nbr) = 1;  // add nbr to open set
					oVec.push_back(nbr);
				}
				else if(ngScore >= gScore.at<float>(nbr) ){ // is temp gscore worse than stored g score of nbr
					continue;
				}
				cameFromX.at<short>(nbr) = cLoc.x;
				cameFromY.at<short>(nbr) = cLoc.y;

				gScore.at<float>(nbr) = ngScore;
				if(cells.at<short>(nbr) < 102){
					fScore.at<float>(nbr) = gScore.at<float>(nbr) + sqrt(pow(nbr.x-gLoc.x,2) + pow(nbr.y-gLoc.y,2));// +getEuclidianDistance(gLoc,nbr)
				}
				else{
					fScore.at<float>(nbr)= INFINITY;
				}
			}
		}
		/////////////// end condition for while loop, check if oSet is empty
		if(oVec.size() == 0){
			foo = false;
		}
	}
	vector<Point> totalPath;
	for(int i=0; i<4; i++){
		Point t = sLoc;
		totalPath.push_back(t);
	}
	return totalPath;
}

float Costmat::aStarDist(Point sLoc, Point gLoc){

	cerr << "into aStarDist" << endl;

	if(sLoc == gLoc){
		return 0;
	}

	Mat cSet = Mat::zeros(cells.size(), CV_16S); // 1 means in closed set, 0 means not
	Mat oSet = Mat::zeros(cells.size(), CV_16S); // 1 means in open set, 0 means not

	Mat gScore = Mat::ones(cells.size(), CV_32F)*INFINITY; // known cost from initial node to n
	Mat fScore = Mat::ones(cells.size(), CV_32F)*INFINITY; // known cost from initial node to n

	vector<Point> oVec;
	oVec.push_back(sLoc);
	oSet.at<short>(sLoc) = 1; // starting node has score 0
	gScore.at<float>(sLoc)  = 0; // starting node in open set
	fScore.at<float>(sLoc) = sqrt(pow(sLoc.x-gLoc.x,2) + pow(sLoc.y-gLoc.y,2));
	fScore.at<float>(gLoc) = 1;

	bool foo = true;

	while(foo){
		/////////////////// this finds node with lowest fScore and makes current
		float min = INFINITY;
		int mindex = -1;

		for(size_t i=0; i<oVec.size(); i++){
			if(fScore.at<float>(oVec[i]) < min){
				min = fScore.at<float>(oVec[i]);
				mindex = i;
			}
		}

		Point cLoc = oVec[mindex];
		oVec.erase(oVec.begin() + mindex);
		oSet.at<short>(cLoc) = 0;
		cSet.at<short>(cLoc) = 1;

		//cout << "*** sLoc/cLoc/gLoc: " << sLoc << " / "<< cLoc << " / "<< gLoc << endl;

		/////////////////////// end finding current node
		if(pointCompare(cLoc, gLoc) ){ // if the current node equals goal, construct path
			cerr << "out of aStarDist" << endl;
			return gScore.at<float>(cLoc);
		} ///////////////////////////////// end construct path

		// for nbrs
		int nx[8] = {-1,-1,-1,0,0,1,1,1};
		int ny[8] = {1,0,-1,1,-1,1,0,-1};

		for(int ni = 0; ni<8; ni++){
			Point nbr;
			nbr.x += cLoc.x + nx[ni];
			nbr.y += cLoc.y + ny[ni];
			if(pointOnMat(nbr, cells) ){
				if(cSet.at<short>(nbr) == 1){ // has it already been eval? in cSet
					continue;
				}
				float ngScore = gScore.at<float>(cLoc) + sqrt(pow(cLoc.x-nbr.x,2) + pow(cLoc.y-nbr.y,2));//getEuclidianDistance(cLoc, nbr); // calc temporary gscore, estimate of total cost
				if(oSet.at<short>(nbr) == 0){
					oSet.at<short>(nbr) = 1;  // add nbr to open set
					oVec.push_back(nbr);
				}
				else if(ngScore >= gScore.at<float>(nbr) ){ // is temp gscore worse than stored g score of nbr
					continue;
				}

				gScore.at<float>(nbr) = ngScore;
				if(cells.at<short>(nbr) < 102){
					fScore.at<float>(nbr) = gScore.at<float>(nbr) + sqrt(pow(nbr.x-gLoc.x,2) + pow(nbr.y-gLoc.y,2));// +getEuclidianDistance(gLoc,nbr)
				}
				else{
					fScore.at<float>(nbr)= INFINITY;
				}
			}
		}
		/////////////// end condition for while loop, check if oSet is empty
		if(oVec.size() == 0){
			foo = false;
		}
	}

	return INFINITY;
}

void Costmat::addAgentToPlot(Scalar color, vector<Point> myPath, Point cLoc){
	circle(displayPlot, cLoc, 2, color, -1);
	for(size_t i=1; i<myPath.size(); i++){
		Point a = myPath[i];
		Point b = myPath[i-1];
		line(displayPlot, a, b, color, 1);
	}
}

bool pointCompare(Point &a, Point &b){
	if(a.x == b.x && a.y == b.y){
		return true;
	}
	else{
		return false;
	}
}

bool pointOnMat(Point &a, Mat &b){
	if(a.x >= 0 && a.x < b.cols && a.y >= 0 && a.y < b.rows){
		return true;
	}
	else{
		return false;
	}
}

void Costmat::updateRewards(){

}

 Point Costmat::getCellIndex(int l){ //TODO check these
	Point p;
	p.y = floor( l / map_width );
	p.x = l % map_height;

	return p;
}

int Costmat::getArrayIndex( Point p){ //TODO chek these
	int l = p.y * map_width + p.x;
	return l;
}



/*


void Costmat::CostmatCallback(const nav_msgs::OccupancyGrid& cost_in)
{
	ROS_INFO("OSU::Map height/width: %d / %d", cost_in.info.height, cost_in.info.width);
	map_height = cost_in.info.height;
	map_width = cost_in.info.width;

	if( init_flag ){
		 Mat a =  Mat::zeros(map_height, map_width, CV_8UC1);
		cells = a.clone();

		 Mat b =  Mat::zeros(map_height, map_width, CV_32FC1);
		rewards = b.clone();
	}
	
	updateCells( &cost_in );
	updateRewards();
	

	double resolution = Costmat.info.resolution;
	double origin_x = Costmat.info.origin.position.x;
	double origin_y = Costmat.info.origin.position.y;

	// create a Costmat 2d object
	Costmat_2d::Costmat2D cost_obj(map_height, map_width, resolution, origin_x, origin_y);
	
	// ROS_INFO("Costmat created.");
	occupancy_grid_array = cost_in.data;
	frontierExtract(&cost_obj);
	pubFrontierMap(cost_in);
	getCentroids(&cost_obj);
	pubFrontierCentroid(cost_in);
	pubCentroid(cost_in);
}


void Costmat::frontierExtract(Costmat_2d::Costmat2D *Costmat_)
{
	// ROS_INFO("Begin Frontier Extraction...");
	uint array_length = map_height * map_width;
	for (int i = 0; i < array_length; i++) {
		if (occupancy_grid_array[i] == FREE_SPACE && isFrontier(i)) {
			// add to frontiers	
			frontiers.push_back(i);
		}
	}
	// ROS_INFO("Frontiers size %d", (int)frontiers.size());
	// ofstream frontiers_file ("frontiers_file.csv");
	// for (int i = 0; i < frontiers.size(); i++) {
	// // 	ROS_INFO("Frontiers: %d", frontiers[i]);
	// 	frontiers_file << frontiers[i] << ",";
	// 	ROS_INFO("writing...");
	// }
	// frontiers_file.close();
}


void Costmat::getCentroids(Costmat_2d::Costmat2D *Costmat_)
{	
	// ROS_INFO("Now getting centroids...");

	// store all nbors of a cell
 vector<int> region;	
	centroid_coord.clear();
	while (!frontiers.empty()) {
	 pair<uint, uint> coordinate;
		int cell = frontiers.front();
		frontiers.erase(frontiers.begin());

		getRegion(cell, region);

		// if region too small
		if (region.size() < minRegion) {
			region.clear();
			continue;
		}

		uint centroid_x = 0;
		uint centroid_y = 0;
		uint  mx, my;

		for (int index = 0; index < region.size(); index++) {
			Costmat_->indexToCells(region[index], mx, my);
			// mx = region[index] / map_width;
			// my = region[index] % map_width;
			centroid_x += mx;
			centroid_y += my;
		}

		centroid_x /= (uint)region.size();
		centroid_y /= (uint)region.size();
		centroid_cells.push_back(centroid_y * map_width + centroid_x);
		coordinate.first = centroid_x;
		coordinate.second = centroid_y;
		double wx, wy;
		Costmat_->mapToWorld(centroid_x, centroid_y, wx, wy);
		// ROS_INFO("Centroids coordinate: wx %f, wy %f.", wx, wy);
		centroid_coord.push_back((float)wx);
		centroid_coord.push_back((float)wy);
		region.clear();
		centroids.push_back(coordinate);
	}
	// for (int i = 0; i < centroids.size(); i++) {
	// 	ROS_INFO("Centroids x and y, %d, %d", centroids[i].first, centroids[i].second);
	// }
	// ROS_INFO("Centroids number %d", (int)centroids.size());
}


void Costmat::getRegion(int cell, vector<int> & region)
{
 queue<int> cell_queue;
	cell_queue.push(cell);
	int current;
	while(!cell_queue.empty()) {
	 priority_queue<int> del_cells;
		current = cell_queue.front();
		cell_queue.pop();
		region.push_back(current);
		for (int i = 0; i < frontiers.size(); i++) {
			if (isAdjacent(current, frontiers[i])) {
				cell_queue.push(frontiers[i]);
				del_cells.push(i);
			}
		}
		while (!del_cells.empty()) {
			frontiers.erase(frontiers.begin()+del_cells.top());
			del_cells.pop();
		}
	}
}


bool Costmat::isAdjacent(int cell1, int cell2)
{
	if (cell1 == cell2) {
		ROS_ERROR("Two cells are the same.");
	}
	int adjacent;
	for (int i = 0; i < 8; i++) {
		adjacent = getAdjacents(cell1 ,i);
		if (adjacent == cell2) {
			return true;
		}
	}
	return false;
}


bool Costmat::isFrontier(int point)
{
	int adjacent[8];
	for (int i = 0; i < 8; i++) {
		adjacent[i] = getAdjacents(point, i);
	}
	for (int i = 0; i < 8; i++) {
		if (adjacent[i] != -1 && occupancy_grid_array[adjacent[i]] == UNKNOWN) {
			if (isSufficient(adjacent[i])) {
				return true;
			}
		}
	}
	return false;
}


bool Costmat::isSufficient(int point) 
{
	int adjacent[8];
	int count = 0;
	for (int i = 0; i < 8; i++) {
		adjacent[i] = getAdjacents(point, i);
	}
	for (int i = 0; i < 8; i++) {
		if (adjacent[i] != -1 && occupancy_grid_array[adjacent[i]] == UNKNOWN) {
			count++;
		}
	}
	if (count > 2) {
		return true;
	} else {
		return false;
	}
}


int Costmat::getAdjacents(int point, int direction)
{
	if (direction == 0) { // left
		if (point % map_width != 0) {
			return point - 1;
		}
	} else if (direction == 1) { // upleft
		if (point % map_width != 0 && point >= map_width) {
			return point - 1 - map_width;
		}
	} else if (direction == 2) { // up
		if (point >= map_width) {
			return point - map_width;
		}
	} else if (direction == 3) { // upright
		if (point >= map_width && ((point + 1) % map_width != 0)) {
			return point - map_width + 1;
		}
	} else if (direction == 4) { // right
		if ((point + 1) % map_width != 0) {
			return point + 1;
		}
	} else if (direction == 5) { // downright
		if ((point + 1) % map_width != 0 && (point / map_width) < (map_height - 1)) {
			return point + map_width + 1;
		}
	} else if (direction == 6) { // down
		if ((point / map_width) < (map_height - 1)) {
			return point + map_width;
		}
	} else if (direction == 7) { // downleft
		if ((point / map_width) < (map_height - 1) && (point % map_width != 0)) {
			return point + map_width - 1;
		}
	}
	return -1;
}

void Costmat::pubFrontierMap(const nav_msgs::OccupancyGrid & Costmat)
{
	// ROS_INFO("Publish frontier cells...");
	frontierMap.header.seq = Costmat.header.seq;
    frontierMap.header.stamp = Costmat.header.stamp;
    frontierMap.header.frame_id = Costmat.header.frame_id;
    
    frontierMap.info.map_load_time = Costmat.info.map_load_time;
    frontierMap.info.resolution = Costmat.info.resolution;
    frontierMap.info.width = Costmat.info.width;
    frontierMap.info.height = Costmat.info.height;
    
    frontierMap.info.origin.position.x = Costmat.info.origin.position.x;
    frontierMap.info.origin.position.y = Costmat.info.origin.position.y;
    frontierMap.info.origin.position.z = Costmat.info.origin.position.z;
    frontierMap.info.origin.orientation.x = Costmat.info.origin.orientation.x;
    frontierMap.info.origin.orientation.y = Costmat.info.origin.orientation.y;
    frontierMap.info.origin.orientation.z = Costmat.info.origin.orientation.z;
    frontierMap.info.origin.orientation.w = Costmat.info.origin.orientation.w;

    frontierMap.data = Costmat.data;
    for (int i = 0; i < Costmat.data.size(); i++) {
    	frontierMap.data[i] = -1;
    }

    for (int i = 0; i < frontiers.size(); i++) {
    	frontierMap.data[frontiers[i]] = 0;
    }
    pub_frontierMap.publish(frontierMap);
}

void Costmat::pubFrontierCentroid(const nav_msgs::OccupancyGrid & Costmat)
{
	// ROS_INFO("Publishing frontier centroids...");
	frontierCentroid.header.seq = Costmat.header.seq;
    frontierCentroid.header.stamp = Costmat.header.stamp;
    frontierCentroid.header.frame_id = Costmat.header.frame_id;
    
    frontierCentroid.info.map_load_time = Costmat.info.map_load_time;
    frontierCentroid.info.resolution = Costmat.info.resolution;
    frontierCentroid.info.width = Costmat.info.width;
    frontierCentroid.info.height = Costmat.info.height;
    
    frontierCentroid.info.origin.position.x = Costmat.info.origin.position.x;
    frontierCentroid.info.origin.position.y = Costmat.info.origin.position.y;
    frontierCentroid.info.origin.position.z = Costmat.info.origin.position.z;
    frontierCentroid.info.origin.orientation.x = Costmat.info.origin.orientation.x;
    frontierCentroid.info.origin.orientation.y = Costmat.info.origin.orientation.y;
    frontierCentroid.info.origin.orientation.z = Costmat.info.origin.orientation.z;
    frontierCentroid.info.origin.orientation.w = Costmat.info.origin.orientation.w;

    frontierCentroid.data = Costmat.data;
    for (int i = 0; i < Costmat.data.size(); i++) {
    	frontierCentroid.data[i] = -1;
    }

    for (int i = 0; i < centroid_cells.size(); i++) {
    	frontierCentroid.data[centroid_cells[i]] = 0;
    }
    pub_frontierCentroidMap.publish(frontierCentroid);
}

void Costmat::pubCentroid(const nav_msgs::OccupancyGrid & Costmat)
{
	// ROS_INFO("Publishing centroids list...");
	// centroid_msg.layout.dim[0].label = "centroids";
	// centroid_msg.layout.dim[0].size = centroid_cells.size();
	// centroid_msg.layout.dim[0].stride = 2 * centroid_cells.size();
	// centroid_msg.layout.dim[1].label = "location";
	// centroid_msg.layout.dim[1].size = 2;
	// centroid_msg.layout.dim[1].stride = 2;

	centroid_msg.layout.data_offset = 0;
	centroid_msg.data.resize(centroid_coord.size());
	for (int i = 0; i < centroid_coord.size(); i++) {
		centroid_msg.data[i] = centroid_coord[i];
	}

	pub_centroid.publish(centroid_msg);
}

*/