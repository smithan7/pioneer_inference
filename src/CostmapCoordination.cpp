/*
 * CostmapCoordination.cpp
 *
 *  Created on: Jul 12, 2016
 *      Author: andy
 */

#include "CostmapCoordination.h"

CostmapCoordination::CostmapCoordination(){}

CostmapCoordination::~CostmapCoordination() {}

Point CostmapCoordination::marketFrontiers( Costmap &costmap, Point cLoc, Market &market){
	//cerr << "into market frontiers" << endl;

	market.printMarket();

	// update each agents bid assuming they took an optimal step
	for(size_t i=0; i<market.exploreCosts.size(); i++){
		market.exploreCosts[i] -= float(market.times[i]);
	}

	for(size_t i=0; i<frontiers.size(); i++){
		frontiers[i].reward = 1;
	}

	// am I closer to any poses in market than the current purchaser?
	for(size_t i=0; i<market.gLocs.size(); i++){
		//cerr << "GraphCoordination::marketPoses::A1" << endl;
		if( i != market.myIndex ){
			bool flag = false;
			//cerr << "GraphCoordination::marketPoses::A2" << endl;
			if( market.gLocs[i].x > 0 && market.gLocs[i].y > 0){ // published gLocs only

				bool stillAFrontier = false;
				int frontierIndex = -1;

				for(size_t j=0; j<frontiers.size(); j++){ // check if each gLoc is a frontier
					if(sqrt( pow(market.gLocs[i].x - frontiers[j].center.x, 2) + pow(market.gLocs[i].y - frontiers[j].center.y, 2) ) < 5 ){
	//					cerr << "GraphCoordination::marketPoses::" << market.gLocs[i] << " is frontier " << j << endl;
						stillAFrontier = true;
						frontierIndex = j;
						break;
					}
				}


				if( stillAFrontier ){
					//cerr << "GraphCoordination::marketPoses::A3" << endl;
					if(sqrt(pow(cLoc.x - market.gLocs[i].x,2) + pow(cLoc.y - market.gLocs[i].y,2)) <=  market.exploreCosts[i]){
						//cerr << "GraphCoordination::marketPoses::A4" << endl;
						if(costmap.aStarDist(market.gLocs[i], cLoc) - market.exploreCosts[i] > 0.1){
	//						cerr << "GraphCoordination::marketPoses::I am not A* closer" << endl;
							flag = true; // I am not a* closer
						}
						else if( abs(costmap.aStarDist(market.gLocs[i], cLoc) - market.exploreCosts[i]) > 0.1 && market.myIndex < int(i)){
	//						cerr << "GraphCoordination::marketPoses::I am A* equidistant but my index is lower" << endl;
							flag = true;
						}
						else{
	//						cerr << "GraphCoordination::marketPoses::I am A* closer" << endl;
						}
					}
					else{ // I am not euclidian closer
	//					cerr << "GraphCoordination::marketPoses::I am not euclid closer" << endl;
						flag = true;
					}
					if(flag){ // they are closer, remove all reward from their goals
	//					cerr << "GraphCoordination::marketPoses::I am not closer, lose reward for frontier: " << frontierIndex << endl;
						frontiers[frontierIndex].reward = -1;
					}
				}

			}
		}
	}

	//for(size_t i=0; i<frontiers.size(); i++){
	//	cout << market.myIndex << "f.reward[" << i << "]: " << frontiers[i].reward << endl;
	//}
	//cin.ignore();
	//

	//cerr << "GraphCoordination::marketPoses::B" << endl;

	if( frontiers.size() > 0){

		float minCost = INFINITY;
		int minDex = -1;

		vector<bool> trueCost;
		for(size_t i=0; i<frontiers.size(); i++){

			if( frontiers[i].reward > 0){
				frontiers[i].cost = sqrt(pow(cLoc.x-frontiers[i].center.x,2) + pow(cLoc.y-frontiers[i].center.y,2) );
			}
			else{
				frontiers[i].cost = INFINITY;
			}
			trueCost.push_back( false );

			//cout << "index, location, value, rewards, distance: " << i << ", " << poseGraph.nodeLocations[i] << ", " << poseValue.back() << ", " << poseRewards.back() << ", " << poseDistances.back() << endl;

			if( frontiers[i].cost < minCost){
				minCost = frontiers[i].cost;
				minDex = i;
			}
		}


		//for(size_t i=0; i<frontiers.size(); i++){
		//	cout <<  market.myIndex << "f.reward/cost/loc[" << i << "]: " << frontiers[i].reward << " / " << frontiers[i].cost << " / " << frontiers[i].center << endl;
		//}

		if(minDex >= 0){
			//cerr << "GraphCoordination::marketPoses::D" << endl;

			while( true ){ // find best pose

				//cerr << "GraphCoordination::marketPoses::D1: maxPose = " << maxPose << endl;
				if( trueCost[minDex]){
					break;
				}
				//cerr << "GraphCoordination::marketPoses::D2" << endl;

				frontiers[minDex].cost = costmap.aStarDist(cLoc, frontiers[minDex].center);

				//cerr << "GraphCoordination::marketPoses::D4:poseValue[maxPose]: " << poseValue[maxPose] << endl;
				trueCost[minDex] = true;

				minCost = frontiers[minDex].cost;
				for(size_t i=0; i<frontiers.size(); i++){
					if( frontiers[i].cost < minCost){
						minCost = frontiers[i].cost;
						minDex = i;
					}
				}
				//cerr << "GraphCoordination::marketPoses::D5" << endl;


				//cout << "maxPose: index, value, rewards, distance: " << maxPose << ", " << poseValue[maxPose] << ", " << poseRewards[maxPose] << ", " << poseDistances[maxPose] << endl;
			}

			//cerr << "GraphCoordination::marketPoses::gValue: " << poseValue[maxPose] << endl;
			//cerr << "GraphCoordination::marketPoses::goalPose: " << poseGraph.nodeLocations[maxPose] << endl;

			market.gLocs[market.myIndex] = frontiers[minDex].center;
			market.exploreCosts[market.myIndex] = frontiers[minDex].cost;
			cerr << "out of market frontiers with gLoc / cost: " << frontiers[minDex].center << " / " << frontiers[minDex].cost << endl;
			
			frontiers[minDex].getOrientation( costmap );
			frontiers[minDex].getProjection();

			return frontiers[minDex].projection;
		}
		else{
			return Point(-1,-1);
		}

		//cerr << "GraphCoordination::marketPoses::Z" << endl;
	}
	else{
		cerr << "out of market frontiers" << endl;
		return cLoc;
	}
}

vector<Point> CostmapCoordination::findFrontiers( Costmap &costmap){
	vector<Point> frntList;
	// essentially findFrontiers and mark as inferred free
	for(int i=1; i<costmap.cells.cols-1; i++){
		for(int j=1; j<costmap.cells.rows-1; j++){
			bool newFrnt = false;
			if(costmap.cells.at<short>(j,i) == costmap.obsFree){ // i'm observed
				if(costmap.cells.at<short>(j+1,i) == costmap.unknown || costmap.cells.at<short>(j+1,i) == costmap.infFree || costmap.cells.at<short>(j+1,i) == costmap.infWall || costmap.cells.at<short>(j+1,i) == costmap.domFree || costmap.cells.at<short>(j+1,i) == costmap.inflatedWall){ //  but one of my nbrs is observed
					newFrnt = true;
				}
				else if(costmap.cells.at<short>(j-1,i) == costmap.unknown || costmap.cells.at<short>(j-1,i) == costmap.infFree || costmap.cells.at<short>(j-1,i) == costmap.infWall || costmap.cells.at<short>(j-1,i) == costmap.domFree || costmap.cells.at<short>(j-1,i) == costmap.inflatedWall){ //  but one of my nbrs is observed
					newFrnt = true;
				}
				else if(costmap.cells.at<short>(j,i+1) == costmap.unknown || costmap.cells.at<short>(j,i+1) == costmap.infFree || costmap.cells.at<short>(j,i+1) == costmap.infWall || costmap.cells.at<short>(j,i+1) == costmap.domFree || costmap.cells.at<short>(j,i+1) == costmap.inflatedWall){ //  but one of my nbrs is observed
					newFrnt = true;
				}
				else if(costmap.cells.at<short>(j,i-1) == costmap.unknown || costmap.cells.at<short>(j,i-1) == costmap.infFree || costmap.cells.at<short>(j,i-1) == costmap.infWall || costmap.cells.at<short>(j,i-1) == costmap.domFree || costmap.cells.at<short>(j,i-1) == costmap.inflatedWall){ //  but one of my nbrs is observed
					newFrnt = true;
				}
			}
			if(newFrnt){
				Point a(i,j);
				frntList.push_back(a);
			}
		}
	}
	return frntList;
}

void CostmapCoordination::plotFrontiers(Costmap &costmap, vector<Point> &frontierCells){

	Mat displayPlot= Mat::zeros(costmap.cells.size(), CV_8UC3);
	for(int i=0; i<costmap.cells.cols; i++){
		for(int j=0; j<costmap.cells.rows; j++){
			Point a(i,j);
			if(costmap.cells.at<short>(a) == costmap.obsFree){
				displayPlot.at<Vec3b>(a) = costmap.cObsFree;
			}
			else if(costmap.cells.at<short>(a) == costmap.infFree){
				displayPlot.at<Vec3b>(a) = costmap.cInfFree;
			}
			else if(costmap.cells.at<short>(a) == costmap.obsWall){
				displayPlot.at<Vec3b>(a) = costmap.cObsWall;
			}
			else if(costmap.cells.at<short>(a) == costmap.infWall){
				displayPlot.at<Vec3b>(a) = costmap.cInfWall;
			}
			else if(costmap.cells.at<short>(a) == costmap.inflatedWall){
				displayPlot.at<Vec3b>(a) = costmap.cInfWall;
			}
			else if(costmap.cells.at<short>(a) == costmap.unknown){
				displayPlot.at<Vec3b>(a) = costmap.cUnknown;
			}
			else{ // anything else, should never happen
				displayPlot.at<Vec3b>(a) = costmap.cError;
			}
		}
	}

	Vec3b red;
	red[0] = 0;
	red[1] = 0;
	red[2] = 127;
	for(size_t i=0; i<frontierCells.size(); i++){
		displayPlot.at<Vec3b>(frontierCells[i]) = red;
	}

	for(size_t i=0; i<frontiers.size(); i++){

		circle(displayPlot, frontiers[i].center, 1, Scalar(0,0,255), -1, 8);
		if(frontiers[i].projection.x > 0 && frontiers[i].projection.y > 0){
			circle(displayPlot, frontiers[i].projection, 1, Scalar(0,255,0), -1, 8);
		}
	}

	namedWindow("frontiers", WINDOW_NORMAL);
	imshow("frontiers", displayPlot);
	waitKey(1);
}

void CostmapCoordination::findClosestFrontier(Costmap &costmap, Point cLoc, int &goalIndex, float &goalDist){
	float minDist = INFINITY;
	int mindex = -1;

	vector<float> dists;

	for(size_t i=0; i<frontiers.size(); i++){
		float dist = costmap.getEuclidianDistance(cLoc, frontiers[i].center);
		dists.push_back( dist );

		if(dist < minDist){
			minDist = dist;
			mindex = i;
		}
	}

	vector<bool> aDist(this->frontiers.size(), false);

	while(true){
		// get A* dist of closest
		if(!aDist[mindex]){
			dists[mindex] = costmap.aStarDist(cLoc, frontiers[mindex].center);
			aDist[mindex] = true;
		}
		else{
			break;
		}
		// is it still the closest with A* dist?
		for(size_t i=0; i<dists.size(); i++){
			if(dists[i] < minDist){
				minDist = dists[i];
				mindex = i;
			}
		}
	}

	goalIndex = mindex;
	goalDist = dists[mindex];

}

void CostmapCoordination::clusterFrontiers(vector<Point >  frntList, Costmap &costmap){
	// check to see if frnt.centroid is still a Frontier cell, if so keep, else delete
	for(size_t i=0; i<frontiers.size(); i++){
		frontiers[i].editFlag = true;
		bool flag = true;
		for(size_t j=0; j<frntList.size(); j++){
			if(frontiers[i].center == frntList[j]){
				flag = false;
				frntList.erase(frntList.begin()+j);
			}
		}
		if(flag){
			frontiers.erase(frontiers.begin()+i);
		}
		else{
			frontiers[i].editFlag = false;
		}
	}
	// breadth first search through known clusters
	for(size_t i=0; i<frontiers.size(); i++){ // keep checking for new Frontier clusters while there are unclaimed Frontiers
		vector<Point> q; // current cluster
		vector<Point> qP; // open set in cluster
		qP.push_back(frontiers[i].center);

		while((int)qP.size() > 0){ // find all nbrs of those in q
			Point seed = qP[0];
			q.push_back(qP[0]);
			qP.erase(qP.begin(),qP.begin()+1);
			for(int ni = seed.x-2; ni<seed.x+3; ni++){
				for(int nj = seed.y-2; nj<seed.y+3; nj++){
					for(size_t i=0; i<frntList.size(); i++){
						if(frntList[i].x == ni && frntList[i].y == nj){
							qP.push_back(frntList[i]); // in range, add to open set
							frntList.erase(frntList.begin() + i);
						}
					}
				}
			}
		}
		this->frontiers[i].members = q; // save to list of clusters
	}

	// breadth first search
	while(frntList.size() > 0){ // keep checking for new Frontier clusters while there are unclaimed Frontiers
		vector<Point> q; // current cluster
		vector<Point> qP; // open set in cluster
		qP.push_back(frntList[0]);
		frntList.erase(frntList.begin());

		while((int)qP.size() > 0){ // find all nbrs of those in q
			Point seed = qP[0];
			q.push_back(qP[0]);
			qP.erase(qP.begin(),qP.begin()+1);
			for(int ni = seed.x-1; ni<seed.x+2; ni++){
				for(int nj = seed.y-1; nj<seed.y+2; nj++){
					for(int i=0; i<(int)frntList.size(); i++){
						if(frntList[i].x == ni && frntList[i].y == nj){
							qP.push_back(frntList[i]); // in range, add to open set
							frntList.erase(frntList.begin() + i, frntList.begin()+i+1);
						}
					}
				}
			}
		}
		if(q.size() > 3){
			Frontier a(q);
			this->frontiers.push_back(a);
		}
	}
	for(size_t i=0; i<this->frontiers.size(); i++){ // number of clusters
		if(this->frontiers[i].editFlag){
			//frontiers[i].getCentroid(costmap);
			frontiers[i].getCenter();
		}
	}
}


/*

vector<vector<int> > CostmapCoordination::centralMarket(Costmap &costmap, vector<vector<int> > cLoc){

	cerr << "into get Frontier costs: " << endl;
	this->getFrontierCosts(cLoc, costmap);
	cerr << "out of frotier costs" << endl;

	cout << "Frontier costs and Rewards: " << endl;
	for(size_t i=0; i<this->frontiers.size(); i++){
		for(size_t j=0; j<cLoc.size(); j++){
			cout << "   cost: " << bots[j].fCost[i] << endl;;
			cout << "   reward: " << this->frontiers[i].reward << endl;
		}
	}
	waitKey(1);

	if(bots.size() <= this->frontiers.size()){ // more Frontiers than Agents
		cout << "more Frontiers than Agents" << endl;
		vector<vector<float> > fValueList; // list of Frontier values for all Agents, [Agent][Frontier]
		for(int i=0; i<bots.size(); i++){
			vector<float> cVal;
			for(int j=0; j<this->frontiers.size(); j++){
				cVal.push_back( this->frontiers[i].reward - bots[i].fCost[j] );
			}
			fValueList.push_back( cVal );
		}

		cout << "fValueList: " << endl;
		for(int i=0; i<bots.size(); i++){
			cout << "   ";
			for(int j=0; j<this->frontiers.size(); j++){
				cout << fValueList[i][j] << " , ";
			}
			cout << endl;
		}

		bool fin = false;
		vector<int> maxDex;
		vector<float> maxVal;

		while(!fin){
			maxDex.erase(maxDex.begin(), maxDex.end());
			maxVal.erase(maxVal.begin(), maxVal.end());
			fin = true;

			for(int i=0; i<bots.size(); i++){ // get each Agents best Frontier
				maxDex.push_back( -1 );
				maxVal.push_back( -INFINITY );

				for(int j=0; j<(int)fValueList[i].size(); j++){
					if(fValueList[i][j] > maxVal[i]){
						maxDex[i] = j; // Agent's max value is Frontier j
						maxVal[i] = fValueList[i][j];
					}
				}
			}

			// make sure no one shares the same Frontier
			for(int i=0; i<bots.size(); i++){
				for(int j=i+1; j<bots.size(); j++){
					if(i!=j && maxDex[i] == maxDex[j]){ // not me and has the same goal;
						fin = false;
						if(maxVal[i] >= maxVal[j]){
							fValueList[j][maxDex[j]] = -INFINITY;
						}
						else{
							fValueList[i][maxDex[i]] = -INFINITY;
						}
					}
				}
			}
		}
		for(int i=0; i<bots.size(); i++){
			bots[i].gLoc = this->frontiers[maxDex[i]].centroid;
			bots[i].gIndex = maxDex[i];
		}
	}
	else{ // more Agents than Frontiers
		cout << "more Agents than Frontiers" << endl;
		for(int i=0; i<this->frontiers.size(); i++){ // go through all Frontiers and find the best Agent
			float mV = INFINITY;
			int mI;
			for(int j=0; j<bots.size(); j++){
				if(bots[j].fCost[i] < mV){
					mV = bots[j].fCost[i];
					mI = j;
				}
			}
			bots[mI].gLoc = this->frontiers[i].centroid;
			bots[mI].gIndex = i;
			for(int j = 0; j<(int)this->frontiers.size(); j++){ // erase all of the value for the worst Agent
				bots[mI].fCost[j] = INFINITY;
			}
		}
	}
}



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
				tempDist += costmap.getEuclidDist(cluster[i][0][0] ][ cluster[i][1] ];
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