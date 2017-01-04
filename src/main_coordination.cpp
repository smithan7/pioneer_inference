//============================================================================
// Name        : discrete_coordination.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include<dirent.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "Agent.h"
#include "Observer.h"

using namespace cv;
using namespace std;

int findMapToRun(vector<string> &fName);
void loadMapNames(vector<string> &fName);
float getPercentObserved(Costmap &globalCostmap, Costmap &workingCostmap);
float getPercentDominated(Costmap &globalCostmap, Costmap &workingCostmap);
float getPrecisionInferredAndObserved(Costmap &workingCostmap, Costmap &globalCostmap);
float getPrecisionInferred(Costmap &workingCostmap, Costmap &globalCostmap);
float getPrecisionObserved(Costmap &workingCostmap, Costmap &globalCostmap);
float getRecallInferred(Costmap &workingCostmap, Costmap &globalCostmap);
float getRecallObserved(Costmap &workingCostmap, Costmap &globalCostmap);

int main(){

	destroyAllWindows();

	vector<int> treePath;
	srand( time(NULL) );
	bool videoFlag = false;

	int numAgents = 2;
	int numIterations = 1;

	int gSpace = 5;
	float obsThresh = 50;
	float comThresh = 100;
	int maxTime = 2000;

	vector<string> fName;
	fName.push_back("mineMap");
	//fName.push_back("test6");
	//loadMapNames(fName);

	int map_iter = findMapToRun(fName);
	cout << "fName.size(): " << fName.size() << " & mapIter: " << map_iter << endl;

	vector<string> exploreMethod;
	//exploreMethod.push_back("pose");
	//exploreMethod.push_back("select");
	exploreMethod.push_back("selectPose");


	//exploreMethod.push_back("greedy");

	vector<string> inferenceMethod;
	//inferenceMethod.push_back("naive");
	inferenceMethod.push_back("geometric");
	//inferenceMethod.push_back("structural");
	//inferenceMethod.push_back("global");

	string mapLog;
	vector<string> inferenceMethodLog;
	vector<string> exploreMethodLog;
	vector<int> timeLog;
	vector<float> percentObservedLog;
	vector<float> precisionInferredAndObservedLog;
	vector<float> precisionInferredLog;
	vector<float> precisionObservedLog;
	vector<float> recallInferredLog;
	vector<float> recallObservedLog;

	srand( time(NULL) );
	// create world
	World world(fName[map_iter], gSpace, obsThresh, comThresh);
	cout << "main::loaded world" << fName[ map_iter ] << endl;

	vector<Point> sLoc;
	for(int i=0; i<numIterations*numAgents; i++){
		while(true){
			Point tLoc(rand() % world.costmap.cells.cols, rand() % world.costmap.cells.rows);
			if(world.costmap.cells.at<short>(tLoc) == world.costmap.obsFree){ // its not an obstacle
				sLoc.push_back(tLoc);
				break;
			}
		}
	}
	cout << "main::Chose starting locations" << endl;

	for(size_t exploreMethod_iter = 0; exploreMethod_iter<exploreMethod.size(); exploreMethod_iter++){
		for(int iterations_iter = 0; iterations_iter<numIterations; iterations_iter++){

			Observer observer(sLoc[iterations_iter]);
			vector<Agent> agents;
			for(int i=0; i<numAgents; i++){
				agents.push_back(Agent(sLoc[iterations_iter*numAgents+i], i, world, obsThresh, comThresh, numAgents));
				cout << "Main::Agent[" << i << "]: created at : " << sLoc[iterations_iter*numAgents+i].x << " , " << sLoc[iterations_iter*numAgents+i].y << endl;
			}
			cout << "Main::All agents created" << endl;

			time_t start = clock();

			// video writers
			VideoWriter slamVideo;
			if(videoFlag){
				//Size frameSize( static_cast<int>(world.costmap.cells.cols, world.costmap.cells.rows) );
				//slamVideo.open("multiAgentInference.avi",CV_FOURCC('M','J','P','G'), 30, frameSize, true );
				cout << "Main::Videos started" << endl;
			}

			cout << "Main::Ready, press any key to begin." << endl;
			waitKey(1);
			cout << "Main::Here we go!" << endl;

			int timeSteps = -1;
			float percentObserved = 0;

			while(timeSteps < maxTime-1 && percentObserved < 0.99){
				cout << "Main::Starting while loop" << endl;
				timeSteps++;

				// all agents observe
				for(int i=0; i<numAgents; i++){
					world.observe(agents[i].cLoc, agents[i].costmap);
				}
				world.observe(observer.cLoc, observer.costmap);
				cout << "Main::made observations" << endl;

				// all agents communicate
				for(int i=0; i<numAgents-1; i++){
					for(int j=i; j<numAgents; j++){
						if(i!=j){
							agents[i].shareCostmap(agents[i].costmap, agents[j].costmap);
							agents[j].shareCostmap(agents[j].costmap, agents[i].costmap);
							agents[i].agentLocs[j] = agents[j].cLoc;
							agents[j].agentLocs[i] = agents[i].cLoc;

							agents[i].market.locations[j] = agents[j].market.locations[j];
							agents[j].market.locations[i] = agents[i].market.locations[i];

							agents[i].market.costs[j] = agents[j].market.costs[j];
							agents[j].market.costs[i] = agents[i].market.costs[i];


							//world.communicate(agents[i].cLoc, agents[i].costmap, agents[j].cLoc, agents[j].costmap);
						}
					}
				}
				cout << "Main::Out of communicate with other agents" << endl;

				cout << "Main::Into inference" << endl;
				for(size_t inferenceMethod_iter = 0; inferenceMethod_iter<inferenceMethod.size(); inferenceMethod_iter++){
					for(int i=0; i<numAgents; i++){ // trust inference of 1
						agents[i].inference.makeInference( inferenceMethod[inferenceMethod_iter], agents[i].costmap, world);
					}

					if( false){;
						float precisionInferredAndObserved = getPrecisionInferredAndObserved(agents[0].costmap, world.costmap);
						float precisionInferred = getPrecisionInferred(agents[0].costmap, world.costmap);
						float precisionObserved = getPrecisionObserved(agents[0].costmap, world.costmap);
						float recallInferred = getRecallInferred(agents[0].costmap, world.costmap);
						float recallObserved = getRecallObserved(agents[0].costmap, world.costmap);

						size_t slashPos = fName[map_iter].find_last_of("/") + 1;
						size_t dotPos = fName[map_iter].find_last_of(".");
						mapLog = fName[map_iter].substr(slashPos,dotPos);
						inferenceMethodLog.push_back(inferenceMethod[ inferenceMethod_iter]);
						exploreMethodLog.push_back(exploreMethod[ exploreMethod_iter ]);
						timeLog.push_back(timeSteps);

						precisionInferredAndObservedLog.push_back( precisionInferredAndObserved );
						precisionInferredLog.push_back( precisionInferred );
						precisionObservedLog.push_back( precisionObserved );

						recallInferredLog.push_back( recallInferred );
						recallObservedLog.push_back( recallObserved );
					}
				}

				// all agents communicate
				for(int i=0; i<numAgents; i++){
					agents[i].shareCostmap(agents[i].costmap, observer.costmap);
				}
				cout << "Main::Out of communicate with observer" << endl;

				observer.showCellsPlot(agents);
				waitKey(1);
				//observer.costmap.buildOccPlot();

				//waitKey(100);

				cout << "Main::Into plan for one timestep" << endl;
				// all agents plan for one timestep
				if(numAgents == 1){
					agents[0].soloPlan(exploreMethod[exploreMethod_iter], maxTime - timeSteps); // includes updating internal cLoc
				}
				else{
					//agents[0].coordinatedPlan(exploreMethod[exploreMethod_iter], maxTime - timeSteps, agents); // includes updating internal cLoc
					for(int i=0; i<numAgents; i++){
						agents[i].coordinatedPlan(exploreMethod[exploreMethod_iter], maxTime - timeSteps, agents); // includes updating internal cLoc
					}
				}

				// all agents act for one timestep
				for(int i=0; i<numAgents; i++){
					agents[i].act();
				}

				// display observer view
				//observer.showCostmapPlot(agents);

				if(videoFlag){
					for(int i =0; i<5; i++){ // slower frmae rate through repeated frames
						slamVideo << observer.costmap.displayPlot;
					}
				}

				// print out progress
				percentObserved = getPercentObserved(world.costmap, agents[0].costmap);
				if(exploreMethod[exploreMethod_iter].compare("pose") == 0){
					float percentDominated = getPercentDominated(world.costmap, agents[0].costmap);
					percentObserved += percentDominated;
				}
				cout << "------timeSteps & percent observed: " << timeSteps << " & " << percentObserved << " " << map_iter << ": " << fName[map_iter] << endl;

			} // end timeStep in simulation

			cerr << "made it to the end of the simulation!" << endl;
		} // end iterations
	} // end explore methods
	ofstream myfile;
	myfile.open ("/home/andy/git/coordination/Debug/InferenceSept14.csv", ofstream::out | ofstream::app);
	for(size_t i=0; i<timeLog.size(); i++){
		myfile << mapLog << ",";
		myfile << inferenceMethodLog[i] << ",";
		myfile << exploreMethodLog[i] << ",";
		myfile << timeLog[i] << ",";

		myfile << precisionInferredAndObservedLog[i] << ",";
		myfile << precisionInferredLog[i] << ",";
		myfile << precisionObservedLog[i] << ",";

		myfile << recallInferredLog[i] << ",";
		myfile << recallObservedLog[i] << "\n";
	}
	myfile.close();

	cerr << "wrote the file!" << endl;
	waitKey(0);
} // end maps

float getPercentObserved(Costmap &globalCostmap, Costmap &workingCostmap){

	float globalFree = 0;
	float workingFree = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
				globalFree++;
			}
			if(workingCostmap.cells.at<short>(a) == workingCostmap.obsFree){
				workingFree++;
			}
		}
	}

	return workingFree / globalFree;
}

float getPercentDominated(Costmap &globalCostmap, Costmap &workingCostmap){

	float globalFree = 0;
	float dominated = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
				globalFree++;
			}
			if(workingCostmap.cells.at<short>(a) == workingCostmap.domFree){
				dominated++;
			}
		}
	}

	return dominated / globalFree;
}


int findMapToRun(vector<string> &fName){

	/*
	ifstream file ( "/home/andy/git/coordination/Debug/InferenceSept10.csv", ifstream::in ); // declare file stream: http://www.cplusplus.com/reference/iostream/ifstream/
	if ( file.is_open() ){
		string value;
		vector<string> mapNames;
		string mapName;
		int c = 0;
		while ( file.good() ){
			c++;
			getline ( file, value, '\n' ); // read a string until next comma: http://www.cplusplus.com/reference/string/getline/

			int fnd = value.find_first_of(",");

			//if(!mapName.compare( value.substr(0, fnd)) == 0){
				//cout << value << endl; // display value removing the first and the last character from it
				mapName = value.substr(0, fnd);
				//cout << mapName << endl;
				mapNames.push_back( mapName );
			//}
		}
		file.close();

		vector<int> cnt;
		for(size_t i=0; i<fName.size(); i++){
			int fi = fName[i].find_last_of("/");
			string fn = fName[i].substr(fi+1);
			int cntr = 0;
			for(size_t j=0; j<mapNames.size(); j++){
				if( fn.compare( mapNames[j] ) == 0 ){
					cntr++;
				}
			}
			//cout << fn << " : " << cntr << endl;
			cnt.push_back(cntr);
		}

		double minV = INFINITY;
		vector<int> minI;

		for(size_t i=0; i<cnt.size(); i++){
			if(cnt[i] == minV){
				minV = cnt[i];
				minI.push_back(i);
			}
			if(cnt[i] < minV){
				minV = cnt[i];
				minI.clear();
				minI.push_back(i);
			}
		}


		int v = rand() % minI.size();

		return minI[v];
	}
	*/

	return rand() % fName.size();

}


void loadMapNames(vector<string> &fName){
	string dir = "/home/andy/git/fabmap2Test/InferenceLibrary/TestMaps/generated/";
	ifstream fin;
	string filepath;
	DIR *dp;
	struct dirent *dirp;
	struct filestat;

	dp = opendir( dir.c_str() ); // open provided directory
	if (dp == NULL){ // did it open
		cerr << "Error opening " << dir << endl;
		waitKey(0);
	}

	while ((dirp = readdir( dp ))){ // read all files in directory to get filenames
		// get file to open
		filepath = dir + dirp->d_name;
		string fileName = dirp->d_name;
		if(fileName.substr(fileName.find_last_of(".") + 1) != "jpg"){
		    continue;
		}
		fName.push_back( filepath.substr( 0, filepath.find_last_of(".") ) );
	}
}

float getRecallObserved(Costmap &workingCostmap, Costmap &globalCostmap){

	// recall = true positives / all positives

	int globalFree = 0;
	int workingFree = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
				globalFree++;
			}
			if(workingCostmap.cells.at<short>(a) == workingCostmap.obsFree){
				workingFree++;
			}
		}
	}
	return float(workingFree)/float(globalFree);
}

float getRecallInferred(Costmap &workingCostmap, Costmap &globalCostmap){

	// recall = true positives / all positives

	int globalFree = 0;
	int workingFree = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
				globalFree++;
			}
			if(workingCostmap.cells.at<short>(a) == workingCostmap.obsFree){
				workingFree++;
			}
			else if(workingCostmap.cells.at<short>(a) == workingCostmap.infFree || workingCostmap.cells.at<short>(a) == workingCostmap.domFree){
				if( globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
					workingFree++;
				}
			}
		}
	}
	return float(workingFree)/float(globalFree);
}

float getPrecisionObserved(Costmap &workingCostmap, Costmap &globalCostmap){

	// precision = correctly observed / observed free

	int correctlyObsFree = 0;
	int obsFree = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(workingCostmap.cells.at<short>(a) == workingCostmap.obsFree){
				obsFree++;
				if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
					correctlyObsFree++;
				}
			}
		}
	}

	return float(correctlyObsFree)/float(obsFree);
}

float getPrecisionInferred(Costmap &workingCostmap, Costmap &globalCostmap){

	// precision = (correctly inferred free) / (inferred free)

	int correctlyInferredFree = 0;
	int inferredFree = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(workingCostmap.cells.at<short>(a) == workingCostmap.infFree || workingCostmap.cells.at<short>(a) == workingCostmap.domFree){
				inferredFree++;
				if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
					correctlyInferredFree++;
				}
			}
		}
	}

	return float(correctlyInferredFree)/float(inferredFree);
}

float getPrecisionInferredAndObserved(Costmap &workingCostmap, Costmap &globalCostmap){

	// precision = (correctly inferred + observed free) / (inferred + observed free)

	int correctlyInferredFree = 0;
	int inferredFree = 0;

	for(int i=0; i<globalCostmap.cells.cols; i++){
		for(int j=0; j<globalCostmap.cells.rows; j++){
			Point a(i,j);
			if(workingCostmap.cells.at<short>(a) == workingCostmap.infFree || workingCostmap.cells.at<short>(a) == workingCostmap.domFree || workingCostmap.cells.at<short>(a) == workingCostmap.obsFree){
				inferredFree++;
				if(globalCostmap.cells.at<short>(a) == globalCostmap.obsFree){
					correctlyInferredFree++;
				}
			}
		}
	}

	return float(correctlyInferredFree)/float(inferredFree);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Retired Bits /////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

/*



Mat naiveCostMap = master.inference.makeNaiveMatForMiniMap(); // naive to remaining space
cout << "made naive" << endl;
Mat inferredGeometricCostMap = master.inference.makeGeometricInferenceMatForMiniMap(); // inferred remaining space
cout << "made geometric" << endl;
//Mat inferredStructuralCostMap = master.makeStructuralInferenceMatForMiniMap();
Mat inferredGlobalCostMap = master.inference.makeGlobalInferenceMat(world); // all remaining space
cout << "made global" << endl;

float observedEntropy = getObservedMapEntropy(inferredGlobalCostMap, naiveCostMap);
float inferredEntropy = getInferredMapEntropy(inferredGlobalCostMap, inferredGeometricCostMap);

namedWindow("main::naive Costmap", WINDOW_NORMAL);
imshow("main::naive Costmap", naiveCostMap);

namedWindow("main::inferred Geo Costmap", WINDOW_NORMAL);
imshow("main::inferred Geo Costmap", inferredGeometricCostMap);

namedWindow("main::inferred Global Costmap", WINDOW_NORMAL);
imshow("main::inferred Global Costmap", inferredGlobalCostMap);
waitKey(1);

// build miniMap
cout << "building miniMap" << endl;

miniMaster.importFrontiers(master.frontiers);
for(int i=0; i<numAgent; i++){ 		// import UAV locations
	miniMaster.cLocList.push_back(agents[i].cLoc);
}

cout << "main while loop1: " << master.frntsExist <<  endl;

miniMaster.createMiniGraph(inferredGlobalCostMap);
cout << "into display" << endl;
miniMaster.displayCoordMap();
cout << "out of display" << endl;
miniMaster.cState = miniMaster.findNearestNode(agents[0].cLoc);

cout << "main while loop2: " << master.frntsExist <<  endl;


// masterGraph planning /////////////////////////////////////////////////////////////////////////////////////
float maxPathLength = 100 - timeSteps;
int gNode = agents[0].gNode;
if(timeSteps % 1 == 0){
	gNode = miniMaster.masterGraphPathPlanning(maxPathLength, 10000, 10000);
	cout << "out of masterGraphPathPlanning" << endl;
	agents[0].gNode = gNode;
}
cout << "gNode: " << gNode << endl;
vector<int> gLoc;
gLoc.push_back(miniMaster.graf[gNode][0]);
gLoc.push_back(miniMaster.graf[gNode][1]);

cout << "main while loop3: " << master.frntsExist <<  endl;



// tsp path planner ///////////////////////////////////////////////////////////////////////////////////////////
cout << "into tsp" << endl;
vector<int> path = miniMaster.tspPathPlanner(2000,1);
for(size_t i=0; i<path.size(); i++){
	cout << path[i] << endl;
}
cout << "out of tsp" << endl;

vector<int> gLoc;
gLoc.push_back(miniMaster.graf[path[1]][0]);
gLoc.push_back(miniMaster.graf[path[1]][1]);
*/

/*
// mcts path planner //////////////////////////////////////////////////////////////////////////////////////////
vector<int> path;
miniMaster.mctsPathPlanner(path, 1000, 10000);

cout << "mctsPath: ";
for(size_t i=0; i<path.size(); i++){
	cout << path[i] << ", ";
}
cout << endl;

vector<int> gLoc;
gLoc.push_back(miniMaster.graf[path[1]][0]);
gLoc.push_back(miniMaster.graf[path[1]][1]);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


cout << "out of build miniMap" << endl;

*/
