/*
 * CostmapPlanning.cpp
 *
 *  Created on: Jul 13, 2016
 *      Author: andy
 */

#include "CostmapPlanning.h"

bool pointCompare(Point &a, Point &b); // in costmap.cpp
bool pointOnMat(Point &a, Mat &b); // in costmap.cpp

CostmapPlanning::CostmapPlanning() {}

CostmapPlanning::~CostmapPlanning() {}


Point CostmapPlanning::explorePlanner(Costmap &costmap, Point cLoc){
	vector<Contour> exploreContours = getExploreContours(costmap);

	if(exploreContours.size() + costmap.hullBreaches.size() == 0){
		Point a(-1,-1);
		return a;
	}

	float min = INFINITY;
	uint mindex = 0;

	vector<Point> exploreGoals;
	for(size_t i=0; i<exploreContours.size(); i++){
		exploreGoals.push_back( exploreContours[i].center );
	}
	for(size_t i=0; i<exploreContours.size(); i++){
		exploreGoals.push_back( costmap.hullBreaches[i] );
	}


	for(size_t i=0; i<exploreContours.size(); i++){
		float d = sqrt( pow(cLoc.x - exploreGoals[i].x,2) + pow(cLoc.y - exploreGoals[i].y,2) );
		if(d < min){
			min = d;
			mindex = i;
		}
	}

	if(mindex < exploreContours.size()){
		exploreContours[mindex].getNbrsWithValue(costmap, costmap.obsFree);

		int nbr = 0;
		min = INFINITY;
		for(size_t i=0; i<exploreContours[mindex].nbrs.size(); i++){
			float d = sqrt( pow(cLoc.x - exploreContours[mindex].nbrs[i].x,2) + pow(cLoc.y - exploreContours[mindex].nbrs[i].y,2) );
			if(d < min){
				min = d;
				nbr = i;
			}
		}

		return exploreContours[mindex].nbrs[nbr];
	}
	else{
		return costmap.hullBreaches[mindex];
	}

}

Point CostmapPlanning::mappingPlanner(Costmap &costmap, Point cLoc){

	vector<Contour> mappingContours = getMappingContours(costmap);

	if(mappingContours.size() == 0){
		Point a(-1,-1);
		return a;
	}

	float min = INFINITY;
	int mindex = 0;

	for(size_t i=0; i<mappingContours.size(); i++){
		float d = sqrt( pow(cLoc.x - mappingContours[i].center.x,2) + pow(cLoc.y - mappingContours[i].center.y,2) );
		if(d < min){
			min = d;
			mindex = i;
		}
	}

	mappingContours[mindex].getNbrsWithValue(costmap, costmap.obsFree);

	if(mappingContours[mindex].nbrs.size() == 0){
		return Point(-1,-1);
	}

	int nbr = 0;
	min = INFINITY;
	for(size_t i=0; i<mappingContours[mindex].nbrs.size(); i++){
		float d = sqrt( pow(cLoc.x - mappingContours[mindex].nbrs[i].x,2) + pow(cLoc.y - mappingContours[mindex].nbrs[i].y,2) );
		if(d < min){
			min = d;
			nbr = i;
		}
	}

	Point a;

	while(true){
		a.x = mappingContours[mindex].nbrs[nbr].x + rand() % 5 - 2;
		a.y = mappingContours[mindex].nbrs[nbr].y + rand() % 5 - 2;

		if(costmap.cells.at<short>(a) == costmap.obsFree){
			break;
		}
	}

	return a;
}

Point CostmapPlanning::searchPlanner(Costmap &costmap, Point cLoc){
	//costmap.spreadSearchArea(0.1);
	//costmap.displaySearchReward();

	vector<Contour> searchContours = getSearchContours(costmap);

	if(searchContours.size() == 0){
		Point a(-1,-1);
		return a;
	}

	float min = INFINITY;
	int mindex = 0;

	for(size_t i=0; i<searchContours.size(); i++){
		float d = sqrt( pow(cLoc.x - searchContours[i].center.x,2) + pow(cLoc.y - searchContours[i].center.y,2) );
		if(d < min){
			min = d;
			mindex = i;
		}
	}

	searchContours[mindex].getNbrsWithValue(costmap, costmap.obsFree);

	int nbr = 0;
	min = INFINITY;
	for(size_t i=0; i<searchContours[mindex].nbrs.size(); i++){
		float d = sqrt( pow(cLoc.x - searchContours[mindex].nbrs[i].x,2) + pow(cLoc.y - searchContours[mindex].nbrs[i].y,2) );
		if(d < min){
			min = d;
			nbr = i;
		}
	}

	return searchContours[mindex].nbrs[nbr];
}

vector<Contour> CostmapPlanning::getSearchContours(Costmap &costmap){

	Mat s = Mat::zeros(costmap.cells.size(), CV_8UC1);

	for(int i= 0; i<costmap.searchReward.cols; i++){
		for(int j=0; j<costmap.searchReward.rows; j++){
			Point a(i,j);
			s.at<uchar>(a) = round(255 * costmap.searchReward.at<float>(a));
		}
	}

	/*
	namedWindow("s",WINDOW_NORMAL);
	imshow("s",s);
	waitKey(1);
	*/

	threshold(s,s,5,255,THRESH_BINARY);

	vector<Contour> contours;
	vector<Vec4i> hierarchy;
	vector<vector<Point> > cont;
	findContours(s,cont, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	for(size_t i=0; i<cont.size(); i++){
		Contour c(s, cont[i]);
		c.getCenter(costmap.cells);
		contours.push_back(c);

		s.at<uchar>(c.center) = 0;
	}

	return contours;
}

vector<Contour> CostmapPlanning::getExploreContours(Costmap &costmap){
  //	float domPenalty = 0.5;

	Mat s = Mat::zeros(costmap.cells.size(), CV_8UC1);

	for(int i= 0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point a(i,j);
			if(costmap.cells.at<short>(a) == costmap.infFree || costmap.cells.at<short>(a) == costmap.domFree){
				s.at<uchar>(a) = 255;
			}
		}
	}

	threshold(s,s,5,255,THRESH_BINARY);

	vector<Contour> contours;
	vector<Vec4i> hierarchy;
	vector<vector<Point> > cont;
	findContours(s,cont, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	for(size_t i=0; i<cont.size(); i++){
		Contour c(s, cont[i]);
		c.getCenter(costmap.cells);
		contours.push_back(c);
	}
	return contours;
}

vector<Contour> CostmapPlanning::getMappingContours(Costmap &costmap){

	Mat m = Mat::zeros(costmap.cells.size(), CV_8UC1);

	for(int i= 0; i<costmap.occ.cols; i++){
		for(int j=0; j<costmap.occ.rows; j++){
			Point a(i,j);
			//float mv = 0;

			if( costmap.occ.at<float>(a) < 0.2 || costmap.occ.at<float>(a) > 0.51 ){
				m.at<uchar>(a) = 255;
			}

			/*
			float mi = costmap.occ.at<float>(a);
			float ma = 1-costmap.occ.at<float>(a);
			if(ma > mi){
				m.at<uchar>(a) = round(255 * mi);
			}
			else{
				m.at<uchar>(a) = round(255 * ma);
			}

			if(costmap.cells.at<short>(a) == costmap.infWall){
				m.at<uchar>(a) = 0;
			}
			*/
		}
	}

	/*
	namedWindow("m",WINDOW_NORMAL);
	imshow("m",m);
	waitKey(1);
	*/
	Mat m2 = m.clone();

	threshold(m,m2,100,255,THRESH_BINARY);

	vector<Contour> contours;
	vector<Vec4i> hierarchy;
	vector<vector<Point> > cont;
	findContours(m2,cont, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	for(size_t i=0; i<cont.size(); i++){
		Contour c(m2, cont[i]);
		//if(c.area > 20 && c.area < 300){
			c.getCenter(costmap.cells);
			contours.push_back(c);
			circle(m,c.center,3,Scalar(255), -1, 8);
		//}
	}

	/*
	namedWindow("m2",WINDOW_NORMAL);
	imshow("m2",m);
	waitKey(0);
	*/
	return contours;
}


Point CostmapPlanning::greedyFrontierPlanner(Costmap &costmap, Point pLoc){

	vector< Point > oSet;
	oSet.push_back(pLoc);
	vector<float> oCost;
	oCost.push_back(0);
	float cCost = 0;

	vector< Point > cSet;

	while(oSet.size() > 0){

		// find lowest cost in oSet and set as pLoc
		int mindex = 0;
		float mincost = INFINITY;
		for(size_t i=0; i<oSet.size(); i++){
			if(oCost[i] < mincost){
				mindex = i;
				mincost = oCost[i];
			}
		}

		pLoc = oSet[mindex];
		cCost = oCost[mindex];

		oSet.erase(oSet.begin() + mindex);
		oCost.erase(oCost.begin() + mindex);

		cSet.push_back(pLoc);

		// find nbrs of pLoc, if a frontier, return it; if not in cSet or oSet, add to oSet and calc cost
		int dx[4] = {1,-1,0,0};
		int dy[4] = {0,0,-1,1};
		for(int i=0; i<4; i++){
			Point tLoc(pLoc.x+dx[i], pLoc.y+dy[i]);
			float tCost = cCost + 1;

			if(costmap.cells.at<short>(tLoc) == costmap.infFree || costmap.cells.at<short>(tLoc) == costmap.domFree){ // frontier?
				return tLoc;
			}
			else if(costmap.cells.at<short>(tLoc) == costmap.obsFree){ // add to openset
				bool flag = true;
				for(size_t i=0; i<oSet.size(); i++){ // in oSet?
					if(pointCompare(oSet[i], tLoc) ){
						if(tCost < oCost[i]){
							oCost[i] = tCost;
						}
						flag = false;
						break;
					}
				}
				if(flag){
					for(size_t i=0; i<cSet.size(); i++){ // in cSet?
						if(cSet[i] == tLoc){
							flag = false;
							break;
						}
					}
				}
				if(flag){ // obsFree, not in oSet or cSet
					oSet.push_back(tLoc);
					oCost.push_back(tCost);
				}
			}
		}
	}
	return pLoc;
}





/*
vector< vector<int> > CostmapCoordination::kMeansClusteringEuclid(int numClusters, Costmap &costmap){
	vector<Frontier> tempFrnt = this->frontiers;
	vector<int> cluster[numClusters]; // array of vectors
	float bestClusterDist = INFINITY;
	vector< vector<int> > bestClusterSet;
	for(int i=0; i<numClusters; i++){
		vector<int> temp;
		bestClusterSet.push_back(temp);
	}
	bool convergeFlag = false;
	bool initFlag = true;
	while(initFlag){
		for(int i=0; i<numClusters; i++){ // generate one random cluster per UAV
			if(tempFrnt.size() == 0){
				initFlag = false;
			}
			else{
				int temp = rand() % tempFrnt.size();
				// TODO this was the node I think and I changed to index of 'frontiers'
				cluster[i].push_back(temp); // assign random cluster centroid initially
				//
				tempFrnt.erase(tempFrnt.begin()+temp); // don't allow center to be taken
			}
		}
	}

	tempFrnt = this->frontiers;
	while(convergeFlag == false){ // until the solution converges
		float tempDist = 0;
		for(int i=0; i<numClusters; i++){ // compute centroid Frontier of each cluster
			if(cluster[i].size() > 2){ // no point if a cluster has 1 or 2 members
				float minDist[2] = {INFINITY, -1};
				for(int j=0; j<int(cluster[i].size()); j++){ // find A* dist from each node in each cluster
					float distSum = 0;
					for(int k=0; k<int(cluster[i].size()); k++){ // to each other node in that cluster
						distSum += costmap.euclidDist[ cluster[i][j] ][ cluster[i][k] ];
					} // end to each other node
					if(distSum < minDist[0]){ // is it the most central
						minDist[0] = distSum;
						minDist[1] = cluster[i][j]; // assign as new centroid
					} // end is it most central
				} // end find A* dist from each node in each cluster
				cluster[i].erase(cluster[i].begin(), cluster[i].end()); // erase cluster
				cluster[i].push_back(minDist[1]); // set the new cluster centroid
				tempDist += minDist[0];
			} // end if has more than 2 members
			else if(cluster[i].size() == 2){ // if it has two members calc distance between them
				cluster[i].erase(cluster[i].begin()+1); // erase cluster
				tempDist += costmap.getEuclidDist(cluster[i][0][0],  ][ cluster[i][1] ];
			} // end if it has two members calc distance
			else{ // this cluster has one member
				tempDist += 0;
			} // end this cluster has one member
		} // end compute centroid Frontier of each cluster

		for(int i=0; i<int(openFrnt.size()); i++){ // find A* dist from each Frontier to each cluster and group Frontier in closest cluster
			float minDist[2] = {INFINITY, -1}; // [dist, index];
			bool isCenter = false;
			for(int j=0; j<numClusters; j++){ // find closest cluster center to current front
				if(openFrnt[i] == cluster[j][0]){
					isCenter = true;
				}
			}
			if(!isCenter){ // is it a centroid
				for(int j=0; j<numClusters; j++){ // find closest cluster center to current front
					float tempDist = costmap.distGraph[ openFrnt[i]] [cluster[j][0]]; // calc dist between
					if(tempDist < minDist[0]){ // new centroid is closer, so switch
						minDist[0] = tempDist;
						minDist[1] = j; // closest centroid
					} // end new centroid is closer
				} // end find closest cluster center to current front
				if(minDist[1] >= 0){
					cluster[int(minDist[1])].push_back(openFrnt[i]); // add to appropriate cluster
				}
			} // end is it a centroid
		} // end check each cluster centroid
		if(tempDist < bestClusterDist){ // found a better solution, has not converged yet
			bestClusterDist = tempDist;
			for(int i=0; i<numClusters; i++){
				bestClusterSet[i].erase(bestClusterSet[i].begin(),bestClusterSet[i].end());
				for(int j=0; j<int(cluster[i].size()); j++){
					bestClusterSet[i].push_back(cluster[i][j]);
				}
			}
			convergeFlag = false;
		}
		else{ // it has converged
			convergeFlag = true;
		}
	} // end until solution converges
	return(bestClusterSet);
}

vector< vector<int> > CostmapCoordination::kMeansClusteringTravel(int numClusters, Costmap &costmap){
	vector<int> tempFrnt = openFrnt;
	vector<int> cluster[numClusters]; // array of vectors
	float bestClusterDist = INFINITY;
	vector< vector<int> > bestClusterSet;
	for(int i=0; i<numClusters; i++){
		vector<int> temp;
		bestClusterSet.push_back(temp);
	}
	bool convergeFlag = false;
	bool initFlag = true;
	while(initFlag){
		for(int i=0; i<numClusters; i++){ // generate one random cluster per UAV
			if(tempFrnt.size() == 0){
				initFlag = false;
			}
			else{
				int temp = rand() % tempFrnt.size();
				cluster[i].push_back(tempFrnt[temp]); // assign random cluster centroid initially
				tempFrnt.erase(tempFrnt.begin()+temp); // don't allow center to be taken
			}
		}
	}

	tempFrnt = openFrnt;
	while(convergeFlag == false){ // until the solution converges
		float tempDist = 0;
		for(int i=0; i<numClusters; i++){ // compute centroid Frontier of each cluster
			if(cluster[i].size() > 2){ // no point if a cluster has 1 or 2 members
				float minDist[2] = {INFINITY, -1};
				for(int j=0; j<int(cluster[i].size()); j++){ // find A* dist from each node in each cluster
					float distSum = 0;
					for(int k=0; k<int(cluster[i].size()); k++){ // to each other node in that cluster
						distSum += costmap.aStarDist(cluster[i][j],cluster[i][k]);
					} // end to each other node
					if(distSum < minDist[0]){ // is it the most central
						minDist[0] = distSum;
						minDist[1] = cluster[i][j]; // assign as new centroid
					} // end is it most central
				} // end find A* dist from each node in each cluster
				cluster[i].erase(cluster[i].begin(), cluster[i].end()); // erase cluster
				cluster[i].push_back(minDist[1]); // set the new cluster centroid
				tempDist += minDist[0];
			} // end if has more than 2 members
			else if(cluster[i].size() == 2){ // if it has two members calc distance between them
				cluster[i].erase(cluster[i].begin()+1); // erase cluster
				tempDist += costmap.aStarDist(cluster[i][0],cluster[i][1]);
			} // end if it has two members calc distance
			else{ // this cluster has one member
				tempDist += 0;
			} // end this cluster has one member
		} // end compute centroid Frontier of each cluster

		for(int i=0; i<int(openFrnt.size()); i++){ // find A* dist from each Frontier to each cluster and group Frontier in closest cluster
			float minDist[2] = {INFINITY, -1}; // [dist, index];
			bool isCenter = false;
			for(int j=0; j<numClusters; j++){ // find closest cluster center to current front
				if(openFrnt[i] == cluster[j][0]){
					isCenter = true;
				}
			}
			if(!isCenter){ // is it a centroid
				for(int j=0; j<numClusters; j++){ // find closest cluster center to current front
					float tempDist = costmap.aStarDist(openFrnt[i], cluster[j][0]); // calc dist between
					if(tempDist < minDist[0]){ // new centroid is closer, so switch
						minDist[0] = tempDist;
						minDist[1] = j; // closest centroid
					} // end new centroid is closer
				} // end find closest cluster center to current front
				if(minDist[1] >= 0){
					cluster[int(minDist[1])].push_back(openFrnt[i]); // add to appropriate cluster
				}
			} // end is it a centroid
		} // end check each cluster centroid
		if(tempDist < bestClusterDist){ // found a better solution, has not converged yet
			bestClusterDist = tempDist;
			for(int i=0; i<numClusters; i++){
				bestClusterSet[i].erase(bestClusterSet[i].begin(),bestClusterSet[i].end());
				for(int j=0; j<int(cluster[i].size()); j++){
					bestClusterSet[i].push_back(cluster[i][j]);
				}
			}
			convergeFlag = false;
		}
		else{ // it has converged
			convergeFlag = true;
		}
	} // end until solution converges
	return(bestClusterSet);
}
*/

